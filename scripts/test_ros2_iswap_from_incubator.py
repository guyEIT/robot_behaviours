"""ROS 2 analogue of ``test_iswap_from_incubator.py``.

Drives the Hamilton STAR action server through a HandoffTransfer action
to move a 6-well plate from the off-deck ``incubator_handoff`` position
→ plate carrier at rails=20 site 0. Reverse of
``test_ros2_iswap_to_incubator.py``; same motion recipe, same 6-stage
feedback, same server-side lock.

Assumes the plate is physically resting at the incubator handoff site
(e.g. because ``test_ros2_iswap_to_incubator`` delivered it earlier)
and that carrier site 0 at rails=20 is EMPTY.

Prereqs — see ``test_ros2_iswap_to_incubator.py``.

Run with:
    python test_ros2_iswap_from_incubator.py
    python test_ros2_iswap_from_incubator.py --yes
    python test_ros2_iswap_from_incubator.py --rails 15
    python test_ros2_iswap_from_incubator.py --skip-load-deck

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

CARRIER_NAME = "plate_carrier_01"
PLATE_NAME = "six_well_01"
CALIBRATION = "incubator_handoff"


def build_deck_json(rails: int) -> str:
    """Same deck shape as ``test_iswap_from_incubator.py``. A placeholder
    plate on site 0 is still included so the server's resource resolver
    can read geometry for the release grip-XYZ calculation — the plate
    object isn't a live payload, just a size lookup."""
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
    cli = node.create_client(GetStatus, GET_STATUS_SRV)
    if not cli.wait_for_service(timeout_sec=5.0):
        return ""
    fut = cli.call_async(GetStatus.Request())
    rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
    resp = fut.result()
    return resp.status.op_state if resp else ""


def clear_error_if_needed(node: Node, reason: str) -> bool:
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
    """Cycle the iSWAP to clear firmware drive faults. ResetError alone
    doesn't reach the drives — an ``er99/... X001/32`` X-drive fault
    survives an FSM reset and re-fires on the next motion."""
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
    path = Path(tempfile.gettempdir()) / "ros2_iswap_from_incubator_deck.json"
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
    goal.direction = "from_handoff"
    goal.on_deck_resource = PLATE_NAME
    goal.pickup_distance_from_top = float(args.pickup_distance_from_top)
    goal.safe_y = float(args.safe_y)
    goal.traverse_z = float(TRAVERSE_Z)
    # For from_handoff, the handoff offset applies to the pickup end.
    goal.handoff_offset = Vector3(
        x=float(args.offset_x), y=float(args.offset_y), z=float(args.offset_z),
    )
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
    print(f"  direction:     from_handoff")
    print(f"  release:       carrier rails={args.rails}, site 0 "
          f"(on_deck_resource={PLATE_NAME!r})")
    print(f"  traverse Z={TRAVERSE_Z:.1f}, safe Y={args.safe_y:.1f}, "
          f"pickup_distance_from_top={args.pickup_distance_from_top:.1f}")
    offset = (args.offset_x, args.offset_y, args.offset_z)
    if any(offset):
        print(f"  incubator offset (applied to pickup): "
              f"dx={offset[0]:+.2f} dy={offset[1]:+.2f} dz={offset[2]:+.2f}")
    if args.skip_load_deck:
        print("  deck load:     SKIPPED (using existing server deck)")
    else:
        print(f"  deck load:     PLT_CAR_L5MD_A00@rails={args.rails} + "
              f"Cor_Cos_6_wellplate_16800ul_Fb")

    confirm("Proceed with transfer via ROS 2 HandoffTransfer action?")

    rclpy.init()
    node = rclpy.create_node("iswap_from_incubator_test_client")
    try:
        was_errored = clear_error_if_needed(
            node, "pre-run auto-clear (from_incubator test)",
        )
        if args.init_iswap or was_errored:
            initialize_iswap(node)

        try:
            if not args.skip_load_deck:
                deck_hash = load_deck_on_server(node, build_deck_json(args.rails))
                print(f"  deck loaded on server — hash={deck_hash[:12]}…")
            send_handoff(node, args)
        except Exception:
            # See the matching block in test_ros2_iswap_to_incubator.py
            # for why we also re-init the iSWAP here, not just the FSM.
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
                        f"{SAFE_Y:.0f}).")
    p.add_argument("--offset-x", type=float, default=0.0,
                   help="mm to add to the incubator pickup X at runtime.")
    p.add_argument("--offset-y", type=float, default=0.0,
                   help="mm to add to the incubator pickup Y.")
    p.add_argument("--offset-z", type=float, default=0.0,
                   help="mm to add to the incubator pickup Z.")
    p.add_argument("--skip-load-deck", action="store_true",
                   help="don't overwrite the server's deck — assume "
                        f"{CARRIER_NAME}/{PLATE_NAME} are already present.")
    p.add_argument("--init-iswap", action="store_true",
                   help="force an iSWAP re-initialize before the transfer "
                        "(auto-enabled if the FSM was in Error on entry).")
    p.add_argument("--yes", action="store_true",
                   help="skip the pre-goal confirmation prompt")
    main(p.parse_args())
