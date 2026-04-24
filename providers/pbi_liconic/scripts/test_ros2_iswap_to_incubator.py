"""ROS 2 analogue of ``test_iswap_to_incubator.py``.

Drives the Hamilton STAR action server (``hamilton_star_action_server``)
through a HandoffTransfer action to move a 6-well plate from a plate
carrier at rails=20 site 0 → off-deck ``incubator_handoff``. The actual
motion (6-stage manual jog, middle-Y rotations, traverse-Z descents) is
executed inside the server by the same ``iswap_handoff.transfer``
routine the standalone script uses; this file is just an action client.

Prereqs:
  1. Action server running against real hardware:
       pixi run -e ros2 action-server
  2. ROS 2 env sourced in THIS shell (for ``rclpy`` + message types):
       cd ros2_ws && source install/setup.bash
  3. ``incubator_handoff`` calibration present on the server. The
     baked-in defaults match ``iswap_calibrations.json`` as of
     2026-04-22; override live via::
       ros2 param set /hamilton_star_action_server \\
           handoff.incubator_handoff.y 172.5

Run with:
    python test_ros2_iswap_to_incubator.py
    python test_ros2_iswap_to_incubator.py --yes
    python test_ros2_iswap_to_incubator.py --rails 15
    python test_ros2_iswap_to_incubator.py --pickup-distance-from-top 15
    python test_ros2_iswap_to_incubator.py --safe-y 350
    python test_ros2_iswap_to_incubator.py --skip-load-deck   # deck already set

On failure the plate may be stuck in the gripper. Recover with
    pixi run iswap-release
"""

from __future__ import annotations

import argparse
import json
import sys
import tempfile
from pathlib import Path

import rclpy
from geometry_msgs.msg import Vector3
from rclpy.action import ActionClient
from rclpy.node import Node

from hamilton_star_msgs.action import HandoffTransfer
from hamilton_star_msgs.srv import GetStatus, InitializeModule, LoadDeck, ResetError

from pylabrobot.resources import corning as _corning
from pylabrobot.resources.hamilton import STARDeck
from pylabrobot.resources.hamilton import plate_carriers as _carriers

from iswap_transfer import DEFAULT_PICKUP_DISTANCE_FROM_TOP, SAFE_Y, TRAVERSE_Z


NODE_NAME = "hamilton_star_action_server"
HANDOFF_ACTION = f"/{NODE_NAME}/handoff_transfer"
LOAD_DECK_SRV = f"/{NODE_NAME}/load_deck"
GET_STATUS_SRV = f"/{NODE_NAME}/get_status"
RESET_ERROR_SRV = f"/{NODE_NAME}/reset_error"
INIT_MODULE_SRV = f"/{NODE_NAME}/initialize_module"

# Names must match what the on-deck resource resolver looks up at goal
# time. ``six_well_01`` is reachable via Deck.get_resource's recursion.
CARRIER_NAME = "plate_carrier_01"
PLATE_NAME = "six_well_01"
CALIBRATION = "incubator_handoff"


def build_deck_json(rails: int) -> str:
    """Same deck shape as ``iswap_transfer.build_lh_with_carrier`` — a
    PLT_CAR_L5MD_A00 at ``rails`` with a Corning 6-well on site 0 —
    serialized for LoadDeck. Avoids instantiating a STARBackend (the
    server already owns that)."""
    deck = STARDeck()
    carrier = _carriers.PLT_CAR_L5MD_A00(CARRIER_NAME)
    carrier[0] = _corning.Cor_Cos_6_wellplate_16800ul_Fb(PLATE_NAME)
    deck.assign_child_resource(carrier, rails=rails)
    return json.dumps(deck.serialize(), default=str)


def make_confirm(skip: bool):
    def _confirm(prompt: str) -> None:
        if skip:
            print(f"\n{prompt} [auto-yes]")
            return
        ans = input(f"\n{prompt} [y/N] ").strip().lower()
        if ans not in ("y", "yes"):
            print("aborting.")
            sys.exit(1)
    return _confirm


def get_op_state(node: Node) -> str:
    """Read the FSM ``op_state`` via the GetStatus service. Returns an
    empty string if the service isn't reachable."""
    cli = node.create_client(GetStatus, GET_STATUS_SRV)
    if not cli.wait_for_service(timeout_sec=5.0):
        return ""
    fut = cli.call_async(GetStatus.Request())
    rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
    resp = fut.result()
    return resp.status.op_state if resp else ""


def clear_error_if_needed(node: Node, reason: str) -> bool:
    """If the FSM is in Error, call ResetError so the next goal is accepted.
    Returns True if a reset was issued."""
    state = get_op_state(node)
    if state != "Error":
        return False
    cli = node.create_client(ResetError, RESET_ERROR_SRV)
    if not cli.wait_for_service(timeout_sec=5.0):
        raise RuntimeError(f"service {RESET_ERROR_SRV} not available")
    fut = cli.call_async(ResetError.Request(acknowledgment=reason))
    rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
    resp = fut.result()
    if resp is None or not resp.success:
        msg = resp.message if resp else "no response"
        raise RuntimeError(f"ResetError failed: {msg}")
    print(f"  cleared prior Error state ({reason!r})")
    return True


def initialize_iswap(node: Node) -> None:
    """Re-initialize the STAR iSWAP module. Clears firmware drive faults
    that an FSM ResetError alone can't — e.g. an X-drive left in an
    ``er99/... X001/32`` state after a failed transfer. Physically
    re-homes the arm, so skip when state is already clean."""
    cli = node.create_client(InitializeModule, INIT_MODULE_SRV)
    if not cli.wait_for_service(timeout_sec=5.0):
        raise RuntimeError(f"service {INIT_MODULE_SRV} not available")
    fut = cli.call_async(InitializeModule.Request(module="iswap"))
    rclpy.spin_until_future_complete(node, fut, timeout_sec=60.0)
    resp = fut.result()
    if resp is None or not resp.success:
        msg = resp.message if resp else "no response"
        raise RuntimeError(f"InitializeModule(iswap) failed: {msg}")
    print("  iSWAP re-initialized")


def load_deck_on_server(node: Node, deck_json: str) -> str:
    path = Path(tempfile.gettempdir()) / "ros2_iswap_to_incubator_deck.json"
    path.write_text(deck_json)

    cli = node.create_client(LoadDeck, LOAD_DECK_SRV)
    if not cli.wait_for_service(timeout_sec=10.0):
        raise RuntimeError(f"service {LOAD_DECK_SRV} not available — is the action server running?")
    fut = cli.call_async(LoadDeck.Request(deck_file=str(path)))
    rclpy.spin_until_future_complete(node, fut, timeout_sec=30.0)
    resp = fut.result()
    if resp is None or not resp.success:
        msg = resp.message if resp else "no response"
        raise RuntimeError(f"LoadDeck failed: {msg}")
    return resp.deck_hash


def send_handoff(node: Node, args: argparse.Namespace) -> None:
    client = ActionClient(node, HandoffTransfer, HANDOFF_ACTION)
    if not client.wait_for_server(timeout_sec=10.0):
        raise RuntimeError(f"action {HANDOFF_ACTION} not available")

    goal = HandoffTransfer.Goal()
    goal.calibration_name = CALIBRATION
    goal.direction = "to_handoff"
    goal.on_deck_resource = PLATE_NAME
    goal.pickup_distance_from_top = float(args.pickup_distance_from_top)
    goal.safe_y = float(args.safe_y)
    goal.traverse_z = float(TRAVERSE_Z)
    goal.handoff_offset = Vector3(
        x=float(args.offset_x), y=float(args.offset_y), z=float(args.offset_z),
    )
    # 0 means "use the calibration's plate_width" on the server side.
    goal.plate_width = 0.0

    def on_feedback(fb_msg) -> None:
        fb = fb_msg.feedback
        print(f"  stage {fb.stage}/6: {fb.stage_name}")

    send_fut = client.send_goal_async(goal, feedback_callback=on_feedback)
    rclpy.spin_until_future_complete(node, send_fut)
    handle = send_fut.result()
    if handle is None or not handle.accepted:
        raise RuntimeError("goal rejected by server (check ~/status for FSM state)")

    result_fut = handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_fut)
    result = result_fut.result().result
    if not result.success:
        raise RuntimeError(f"handoff_transfer failed: {result.message}")
    print(f"\nok — {result.message}")


def main(args: argparse.Namespace) -> None:
    confirm = make_confirm(args.yes)

    print("== plan ==")
    print(f"  server action: {HANDOFF_ACTION}")
    print(f"  calibration:   {CALIBRATION}")
    print(f"  direction:     to_handoff")
    print(f"  pickup:        carrier rails={args.rails}, site 0 "
          f"(on_deck_resource={PLATE_NAME!r})")
    print(f"  traverse Z={TRAVERSE_Z:.1f}, safe Y={args.safe_y:.1f}, "
          f"pickup_distance_from_top={args.pickup_distance_from_top:.1f}")
    offset = (args.offset_x, args.offset_y, args.offset_z)
    if any(offset):
        print(f"  incubator offset (applied to release): "
              f"dx={offset[0]:+.2f} dy={offset[1]:+.2f} dz={offset[2]:+.2f}")
    if args.skip_load_deck:
        print("  deck load:     SKIPPED (using existing server deck)")
    else:
        print(f"  deck load:     PLT_CAR_L5MD_A00@rails={args.rails} + "
              f"Cor_Cos_6_wellplate_16800ul_Fb")

    confirm("Proceed with transfer via ROS 2 HandoffTransfer action?")

    rclpy.init()
    node = rclpy.create_node("iswap_to_incubator_test_client")
    try:
        # Pre-flight: an Error state from a prior failed run blocks every
        # goal at the FSM gate. Clear it up front so this run proceeds.
        was_errored = clear_error_if_needed(
            node, "pre-run auto-clear (to_incubator test)",
        )
        # If the FSM was in Error OR the user asked for it, also cycle the
        # iSWAP. ResetError alone doesn't clear firmware drive faults
        # (e.g. X-drive X001/32 persists across an FSM reset), so the
        # first motion on the next goal hits the same fault and aborts.
        # Re-init asks the STAR to re-home, which clears the drive fault.
        if args.init_iswap or was_errored:
            initialize_iswap(node)

        try:
            if not args.skip_load_deck:
                deck_hash = load_deck_on_server(node, build_deck_json(args.rails))
                print(f"  deck loaded on server — hash={deck_hash[:12]}…")
            send_handoff(node, args)
        except Exception:
            # The handoff (or deck load) failed. Leave the server ready
            # for the next invocation by (1) clearing the FSM Error and
            # (2) re-initializing the iSWAP, which clears any firmware
            # drive fault (e.g. X-drive X001/32) that an FSM reset alone
            # can't reach. Doing both here — instead of punting re-init
            # to the next run's pre-flight — matters because the clear
            # hides the Error, so the pre-flight can no longer tell.
            # NOTE: a stuck plate may still be in the gripper; that is
            # a physical problem, not an FSM one. Recover with
            # ``pixi run iswap-release`` (requires stopping the server).
            try:
                if clear_error_if_needed(node, "post-failure auto-clear"):
                    print("  NOTE: FSM reset. Check for a stuck plate "
                          "before the next run.")
                    try:
                        initialize_iswap(node)
                    except Exception as init_exc:  # noqa: BLE001
                        print(
                            f"  WARNING: iSWAP re-init failed: {init_exc}. "
                            "The drives may still be faulted; run with "
                            "--init-iswap next time or cycle the STAR."
                        )
            except Exception as reset_exc:  # noqa: BLE001
                print(f"  WARNING: failed to clear Error state: {reset_exc}")
            raise
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("--rails", type=int, default=20,
                   help="carrier rails position (default: 20)")
    p.add_argument("--pickup-distance-from-top", type=float,
                   default=DEFAULT_PICKUP_DISTANCE_FROM_TOP,
                   help="mm below plate top where gripper closes (default: 18)")
    p.add_argument("--safe-y", type=float, default=SAFE_Y,
                   help=f"middle Y at which shoulder rotations happen (default: "
                        f"{SAFE_Y:.0f}). Raise or lower if rotation hits an end "
                        "stop (front or back wall — R072 drive-locked error).")
    p.add_argument("--offset-x", type=float, default=0.0,
                   help="mm to add to the incubator release X at runtime "
                        "(without re-saving calibration). Positive = toward right.")
    p.add_argument("--offset-y", type=float, default=0.0,
                   help="mm to add to the incubator release Y. Positive = toward back.")
    p.add_argument("--offset-z", type=float, default=0.0,
                   help="mm to add to the incubator release Z. Positive = higher "
                        "(drop the plate from higher up).")
    p.add_argument("--skip-load-deck", action="store_true",
                   help="don't overwrite the server's deck — assume "
                        f"{CARRIER_NAME}/{PLATE_NAME} are already present.")
    p.add_argument("--init-iswap", action="store_true",
                   help="force an iSWAP re-initialize before the transfer "
                        "(auto-enabled if the FSM was in Error on entry, "
                        "which clears firmware drive faults that an FSM "
                        "reset alone can't).")
    p.add_argument("--yes", action="store_true",
                   help="skip the pre-goal confirmation prompt")
    main(p.parse_args())
