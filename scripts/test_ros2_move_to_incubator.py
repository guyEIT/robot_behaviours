"""Move a 6-well plate from plate carrier rails=20, position 1 (site 0)
to the off-deck incubator loading dock using the generic
``MoveResource`` ROS 2 action + the runtime handoff registry.

The flow:

  1. Read the incubator coordinate from ``iswap_calibrations.json``.
  2. Register it with the action server via ``/define_handoff`` (or
     update it, via ``replace_existing=True``). Hotel depth / clearance
     are set here so they live with the *site*, not with every goal.
  3. Load a deck containing just ``plate_carrier_01`` at rails=20 with
     a 6-well on site 0. No off-deck ``ResourceHolder`` needed — the
     destination comes from the registry.
  4. Send a single ``MoveResource`` goal with ``to_handoff=<name>``.
     The server resolves the coordinate, enables ``use_unsafe_hotel``,
     and pulls ``hotel_depth`` + ``hotel_clearance_height`` from the
     registered record.

Firmware runs the whole transfer atomically via STAR ``PO``/``PI``.

Prereqs — see ``test_ros2_iswap_to_incubator.py``.

Run with:
    python test_ros2_move_to_incubator.py
    python test_ros2_move_to_incubator.py --yes
    python test_ros2_move_to_incubator.py --rails 15
    python test_ros2_move_to_incubator.py --hotel-depth 600
    python test_ros2_move_to_incubator.py --skip-load-deck

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

from hamilton_star_msgs.action import MoveResource
from hamilton_star_msgs.msg import Handoff as HandoffMsg
from hamilton_star_msgs.srv import (
    DefineHandoff, GetStatus, InitializeModule, LoadDeck, ResetError,
)

from pylabrobot.resources import corning as _corning
from pylabrobot.resources.hamilton import STARDeck
from pylabrobot.resources.hamilton import plate_carriers as _carriers

from iswap_transfer import (
    DEFAULT_PICKUP_DISTANCE_FROM_TOP,
    load_incubator_handoff,
)


NODE_NAME = "hamilton_star_action_server"
MOVE_RESOURCE_ACTION = f"/{NODE_NAME}/move_resource"
LOAD_DECK_SRV = f"/{NODE_NAME}/load_deck"
GET_STATUS_SRV = f"/{NODE_NAME}/get_status"
RESET_ERROR_SRV = f"/{NODE_NAME}/reset_error"
INIT_MODULE_SRV = f"/{NODE_NAME}/initialize_module"
DEFINE_HANDOFF_SRV = f"/{NODE_NAME}/define_handoff"

CARRIER_NAME = "plate_carrier_01"
PLATE_NAME = "six_well_01"
HANDOFF_NAME = "incubator_handoff"

# Hotel depth default: pylabrobot asserts ``0 <= hotel_depth <= 3000``
# in 0.1 mm units — hard max 300 mm. For the incubator at X≈-219 that
# puts the hotel boundary at X = -219 + 300 = +81 (on-deck); the iSWAP
# rotation completes at the boundary before a straight-in approach.
DEFAULT_HOTEL_DEPTH = 300.0
DEFAULT_HOTEL_CLEARANCE = 8.0


def build_deck_json(rails: int) -> str:
    """Deck has only what's physically on the Hamilton — no off-deck
    stub. The incubator destination is a registry lookup, not a deck
    resource."""
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
    if get_op_state(node) != "Error":
        return False
    cli = node.create_client(ResetError, RESET_ERROR_SRV)
    if not cli.wait_for_service(timeout_sec=5.0):
        raise RuntimeError(f"service {RESET_ERROR_SRV} not available")
    fut = cli.call_async(ResetError.Request(acknowledgment=reason))
    rclpy.spin_until_future_complete(node, fut, timeout_sec=30.0)
    resp = fut.result()
    if resp is None or not resp.success:
        msg = resp.message if resp else "no response"
        raise RuntimeError(f"ResetError failed: {msg}")
    print(f"  cleared prior Error state ({reason!r})")
    return True


def initialize_iswap(node: Node) -> None:
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
    path = Path(tempfile.gettempdir()) / "ros2_move_to_incubator_deck.json"
    path.write_text(deck_json)
    cli = node.create_client(LoadDeck, LOAD_DECK_SRV)
    if not cli.wait_for_service(timeout_sec=10.0):
        raise RuntimeError(f"service {LOAD_DECK_SRV} not available")
    fut = cli.call_async(LoadDeck.Request(deck_file=str(path)))
    rclpy.spin_until_future_complete(node, fut, timeout_sec=30.0)
    resp = fut.result()
    if resp is None or not resp.success:
        msg = resp.message if resp else "no response"
        raise RuntimeError(f"LoadDeck failed: {msg}")
    return resp.deck_hash


def register_handoff(
    node: Node,
    name: str,
    xyz: tuple[float, float, float],
    plate_width: float,
    hotel_depth: float,
    hotel_clearance: float,
    grip_direction: str = "LEFT",
) -> None:
    cli = node.create_client(DefineHandoff, DEFINE_HANDOFF_SRV)
    if not cli.wait_for_service(timeout_sec=10.0):
        raise RuntimeError(f"service {DEFINE_HANDOFF_SRV} not available")
    handoff = HandoffMsg()
    handoff.name = name
    handoff.x, handoff.y, handoff.z = (float(v) for v in xyz)
    handoff.plate_width = float(plate_width)
    handoff.rotation = "LEFT"
    handoff.wrist = "STRAIGHT"
    handoff.grip_direction = grip_direction
    handoff.hotel_depth = float(hotel_depth)
    handoff.hotel_clearance_height = float(hotel_clearance)
    req = DefineHandoff.Request(handoff=handoff, replace_existing=True)
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
    resp = fut.result()
    if resp is None or not resp.success:
        msg = resp.message if resp else "no response"
        raise RuntimeError(f"DefineHandoff failed: {msg}")
    print(f"  registered handoff {name!r} @ "
          f"({xyz[0]:.1f},{xyz[1]:.1f},{xyz[2]:.1f}) "
          f"hotel_depth={hotel_depth:.0f} grip_direction={grip_direction}")


def send_move_resource(node: Node, args: argparse.Namespace) -> None:
    client = ActionClient(node, MoveResource, MOVE_RESOURCE_ACTION)
    if not client.wait_for_server(timeout_sec=10.0):
        raise RuntimeError(f"action {MOVE_RESOURCE_ACTION} not available")

    goal = MoveResource.Goal()
    goal.resource = PLATE_NAME
    # Unified: `to` is just a deck-resource name. Since the handoff
    # was registered via /define_handoff, the server has already
    # injected a matching ResourceHolder onto the deck. The server
    # auto-detects this is a handoff and applies hotel mode.
    goal.to = HANDOFF_NAME
    goal.transport = "iswap"
    # Per-side overrides win over the handoff-registered grip_direction.
    goal.pickup_direction = args.pickup_grip_direction or ""
    goal.drop_direction = args.drop_grip_direction or ""
    goal.pickup_offset = Vector3(x=0.0, y=0.0, z=0.0)
    goal.destination_offset = Vector3(
        x=float(args.offset_x), y=float(args.offset_y), z=float(args.offset_z),
    )
    goal.pickup_distance_from_top = float(args.pickup_distance_from_top)
    # Leave these zero to use the values stored on the handoff record.
    # Set them non-zero only for on-the-fly tuning.
    goal.use_unsafe_hotel = False
    goal.hotel_depth = 0.0
    goal.hotel_clearance_height = 0.0

    def on_feedback(fb_msg) -> None:
        fb = fb_msg.feedback
        if fb.stage:
            print(f"  stage: {fb.stage} ({fb.progress:.0%})")

    send_fut = client.send_goal_async(goal, feedback_callback=on_feedback)
    rclpy.spin_until_future_complete(node, send_fut)
    handle = send_fut.result()
    if handle is None or not handle.accepted:
        raise RuntimeError("goal rejected by server (check ~/status for FSM state)")

    result_fut = handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_fut)
    result = result_fut.result().result
    if not result.success:
        raise RuntimeError(f"move_resource failed: {result.message}")
    if result.warnings:
        for w in result.warnings:
            print(f"  warning: {w}")
    print(f"\nok — {result.message or 'move_resource completed'}")


def main(args: argparse.Namespace) -> None:
    confirm = make_confirm(args.yes)
    incubator = load_incubator_handoff()
    incubator_xyz = (incubator["x"], incubator["y"], incubator["z"])

    print("== plan ==")
    print(f"  server action: {MOVE_RESOURCE_ACTION}")
    print(f"  resource:      {PLATE_NAME} (on carrier rails={args.rails}, "
          "site 0 / position 1)")
    print(f"  destination:   to={HANDOFF_NAME!r} "
          f"@ ({incubator_xyz[0]:.1f}, {incubator_xyz[1]:.1f}, "
          f"{incubator_xyz[2]:.1f}) "
          "(registered handoff → auto hotel mode)")
    print(f"  registry:      hotel_depth={args.hotel_depth:.1f}, "
          f"hotel_clearance={args.hotel_clearance_height:.1f}, "
          f"plate_width={incubator['plate_width']:.1f}")
    print(f"  pickup_distance_from_top={args.pickup_distance_from_top:.1f}")
    offset = (args.offset_x, args.offset_y, args.offset_z)
    if any(offset):
        print(f"  destination offset: dx={offset[0]:+.2f} dy={offset[1]:+.2f} "
              f"dz={offset[2]:+.2f}")
    if args.skip_load_deck:
        print("  deck load:     SKIPPED (using existing server deck)")
    else:
        print(f"  deck load:     PLT_CAR_L5MD_A00@rails={args.rails} "
              f"+ Cor_Cos_6_wellplate_16800ul_Fb (no off-deck stub)")
    if args.skip_register:
        print("  handoff:       SKIPPED (using existing registry entry)")
    else:
        print(f"  handoff:       /define_handoff {HANDOFF_NAME!r} "
              "(replace_existing=True)")

    confirm("Proceed with move_resource → incubator?")

    rclpy.init()
    node = rclpy.create_node("move_to_incubator_test_client")
    try:
        was_errored = clear_error_if_needed(
            node, "pre-run auto-clear (move_to_incubator test)",
        )
        if args.init_iswap or was_errored:
            initialize_iswap(node)

        try:
            if not args.skip_register:
                register_handoff(
                    node,
                    HANDOFF_NAME,
                    incubator_xyz,
                    plate_width=incubator["plate_width"],
                    hotel_depth=args.hotel_depth,
                    hotel_clearance=args.hotel_clearance_height,
                    grip_direction=args.grip_direction,
                )
            if not args.skip_load_deck:
                deck_hash = load_deck_on_server(
                    node, build_deck_json(args.rails),
                )
                print(f"  deck loaded on server — hash={deck_hash[:12]}…")
            send_move_resource(node, args)
        except Exception:
            try:
                if clear_error_if_needed(node, "post-failure auto-clear"):
                    print("  NOTE: FSM reset. Check for a stuck plate "
                          "before the next run.")
                    try:
                        initialize_iswap(node)
                    except Exception as init_exc:  # noqa: BLE001
                        print(
                            f"  WARNING: iSWAP re-init failed: {init_exc}. "
                            "Restart the action server or power-cycle the STAR."
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
                   help="carrier rails position (default: 20, = position 1)")
    p.add_argument("--pickup-distance-from-top", type=float,
                   default=DEFAULT_PICKUP_DISTANCE_FROM_TOP,
                   help="mm below plate top where gripper closes (default: 18)")
    p.add_argument("--hotel-depth", type=float, default=DEFAULT_HOTEL_DEPTH,
                   help=f"mm the firmware approaches into the hotel; boundary "
                        f"must land ON the Hamilton deck (default: "
                        f"{DEFAULT_HOTEL_DEPTH:.0f}). Stored on the handoff.")
    p.add_argument("--hotel-clearance-height", type=float,
                   default=DEFAULT_HOTEL_CLEARANCE,
                   help=f"mm vertical clearance before entering the hotel "
                        f"(default: {DEFAULT_HOTEL_CLEARANCE:.1f}). "
                        "Stored on the handoff.")
    p.add_argument("--grip-direction", type=str, default="RIGHT",
                   choices=("FRONT", "BACK", "LEFT", "RIGHT"),
                   help="Stored on the handoff record AND used for both "
                        "pickup and drop so the wrist stays in one "
                        "orientation across the transit (pylabrobot "
                        "hotel atomics don't rotate the wrist mid-move). "
                        "For the 6-well plate on a standard carrier, "
                        "RIGHT or LEFT grips the 85 mm axis; experiment "
                        "to find the one that gives the correct plate "
                        "orientation at both ends.")
    p.add_argument("--pickup-grip-direction", type=str, default="",
                   choices=("", "FRONT", "BACK", "LEFT", "RIGHT"),
                   help="Override the handoff-derived pickup grip. Only "
                        "useful when pickup and drop need DIFFERENT "
                        "orientations (e.g. plate on-deck in one "
                        "orientation, incubator dock rotated 90°). The "
                        "server currently doesn't insert a wrist "
                        "rotation between the atomic pickup and drop, "
                        "so expect R027 if pickup!=drop until we add one.")
    p.add_argument("--drop-grip-direction", type=str, default="",
                   choices=("", "FRONT", "BACK", "LEFT", "RIGHT"),
                   help="Override the handoff-derived drop grip. See "
                        "--pickup-grip-direction caveat.")
    p.add_argument("--offset-x", type=float, default=0.0,
                   help="mm added to the destination X at runtime.")
    p.add_argument("--offset-y", type=float, default=0.0,
                   help="mm added to the destination Y.")
    p.add_argument("--offset-z", type=float, default=0.0,
                   help="mm added to the destination Z.")
    p.add_argument("--skip-load-deck", action="store_true",
                   help="don't overwrite the server's deck — assume "
                        f"{CARRIER_NAME} and {PLATE_NAME} are already present.")
    p.add_argument("--skip-register", action="store_true",
                   help=f"don't re-register {HANDOFF_NAME!r} — assume it's "
                        "already in the server's handoff registry "
                        "(seeded from handoffs.yaml at startup).")
    p.add_argument("--init-iswap", action="store_true",
                   help="force iSWAP re-initialize before the transfer")
    p.add_argument("--yes", action="store_true",
                   help="skip the pre-goal confirmation prompt")
    main(p.parse_args())
