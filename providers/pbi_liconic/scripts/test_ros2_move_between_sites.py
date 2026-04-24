"""On-deck plate swap test: move a 6-well plate between two sites on
the same plate carrier, then back again. Exercises the ``MoveResource``
ROS 2 action on the all-on-deck path (no handoff involved, so the
server uses pylabrobot's atomic ``iswap_get_plate`` /
``iswap_put_plate`` with firmware collision-control + end-of-move
fold-up enabled).

Default: carrier at rails=21, plate starts on site 0 (position 1),
moves to site 1 (position 2), then back to site 0.

Prereqs: ``pixi run -e ros2 action-server`` running against real
hardware, with a 6-well plate physically on rails=21, site 0.

Run with:
    python test_ros2_move_between_sites.py --yes
    python test_ros2_move_between_sites.py --rails 15
    python test_ros2_move_between_sites.py --from-site 0 --to-site 2 --yes
    python test_ros2_move_between_sites.py --skip-load-deck --once
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
from hamilton_star_msgs.srv import GetStatus, InitializeModule, LoadDeck, ResetError

from pylabrobot.resources import corning as _corning
from pylabrobot.resources.hamilton import STARDeck
from pylabrobot.resources.hamilton import plate_carriers as _carriers

from iswap_transfer import DEFAULT_PICKUP_DISTANCE_FROM_TOP


NODE_NAME = "hamilton_star_action_server"
MOVE_RESOURCE_ACTION = f"/{NODE_NAME}/move_resource"
LOAD_DECK_SRV = f"/{NODE_NAME}/load_deck"
GET_STATUS_SRV = f"/{NODE_NAME}/get_status"
RESET_ERROR_SRV = f"/{NODE_NAME}/reset_error"
INIT_MODULE_SRV = f"/{NODE_NAME}/initialize_module"

CARRIER_NAME = "plate_carrier_swap"
PLATE_NAME = "six_well_swap"


def site_name(site_index: int) -> str:
    """pylabrobot names PLT_CAR_L5MD_A00 sites ``<carrier>-<index>``."""
    return f"{CARRIER_NAME}-{site_index}"


def build_deck_json(rails: int, starting_site: int) -> str:
    """Deck with the plate on ``starting_site`` of a carrier at ``rails``.
    Other sites stay empty."""
    deck = STARDeck()
    carrier = _carriers.PLT_CAR_L5MD_A00(CARRIER_NAME)
    carrier[starting_site] = _corning.Cor_Cos_6_wellplate_16800ul_Fb(PLATE_NAME)
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
    path = Path(tempfile.gettempdir()) / "ros2_move_between_sites_deck.json"
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


def send_move(
    node: Node,
    resource: str,
    to: str,
    args: argparse.Namespace,
) -> None:
    client = ActionClient(node, MoveResource, MOVE_RESOURCE_ACTION)
    if not client.wait_for_server(timeout_sec=10.0):
        raise RuntimeError(f"action {MOVE_RESOURCE_ACTION} not available")

    goal = MoveResource.Goal()
    goal.resource = resource
    goal.to = to
    goal.transport = "iswap"
    goal.pickup_direction = args.pickup_direction
    goal.drop_direction = args.drop_direction or args.pickup_direction
    goal.pickup_offset = Vector3(x=0.0, y=0.0, z=0.0)
    goal.destination_offset = Vector3(x=0.0, y=0.0, z=0.0)
    goal.pickup_distance_from_top = float(args.pickup_distance_from_top)
    goal.use_unsafe_hotel = False
    goal.hotel_depth = 0.0
    goal.hotel_clearance_height = 0.0

    def on_feedback(fb_msg) -> None:
        fb = fb_msg.feedback
        if fb.stage:
            print(f"  stage: {fb.stage} ({fb.progress:.0%})")

    print(f"  → move_resource {resource!r} → {to!r}")
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
    print(f"  ok — {result.message or 'complete'}")


def main(args: argparse.Namespace) -> None:
    confirm = make_confirm(args.yes)

    from_site = args.from_site
    to_site = args.to_site
    if from_site == to_site:
        print(f"error: --from-site and --to-site must differ (got {from_site}).")
        sys.exit(2)

    site_from = site_name(from_site)
    site_to = site_name(to_site)

    print("== plan ==")
    print(f"  server action: {MOVE_RESOURCE_ACTION}")
    print(f"  carrier:       {CARRIER_NAME} (PLT_CAR_L5MD_A00) @ rails={args.rails}")
    print(f"  plate:         {PLATE_NAME} (6-well), starts on site {from_site}")
    print(f"  move 1:        site {from_site} ({site_from}) → site {to_site} ({site_to})")
    if not args.once:
        print(f"  move 2:        site {to_site} ({site_to}) → site {from_site} ({site_from})")
    print(f"  pickup_direction={args.pickup_direction}, "
          f"drop_direction={args.drop_direction or args.pickup_direction} (default same)")
    print(f"  pickup_distance_from_top={args.pickup_distance_from_top:.1f}")

    confirm("Proceed with site swap?")

    rclpy.init()
    node = rclpy.create_node("move_between_sites_test_client")
    try:
        was_errored = clear_error_if_needed(
            node, "pre-run auto-clear (move_between_sites test)",
        )
        if args.init_iswap or was_errored:
            initialize_iswap(node)

        try:
            if not args.skip_load_deck:
                deck_hash = load_deck_on_server(
                    node, build_deck_json(args.rails, from_site),
                )
                print(f"  deck loaded on server — hash={deck_hash[:12]}…")

            # Forward move
            send_move(node, PLATE_NAME, site_to, args)

            if not args.once:
                # Reverse: after the first move the plate lives on
                # site_to. Both its name and the destination name are
                # the same as before.
                send_move(node, PLATE_NAME, site_from, args)

            print("\nok — site swap complete.")
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
    p.add_argument("--rails", type=int, default=21,
                   help="carrier rails position (default: 21)")
    p.add_argument("--from-site", type=int, default=0,
                   help="site index the plate starts on (default: 0, "
                        "= position 1 on the carrier)")
    p.add_argument("--to-site", type=int, default=1,
                   help="site index the plate ends up on after move 1 "
                        "(default: 1, = position 2)")
    p.add_argument("--once", action="store_true",
                   help="only do the forward move; skip the reverse")
    p.add_argument("--pickup-direction", type=str, default="FRONT",
                   choices=("FRONT", "BACK", "LEFT", "RIGHT"),
                   help="GripDirection for pickup. pylabrobot uses "
                        "size_x for FRONT/BACK, size_y for LEFT/RIGHT. "
                        "Default FRONT.")
    p.add_argument("--drop-direction", type=str, default="",
                   choices=("", "FRONT", "BACK", "LEFT", "RIGHT"),
                   help="GripDirection for drop. Empty = same as pickup "
                        "(keeps the wrist in one orientation across the "
                        "transit; pylabrobot's atomics don't rotate the "
                        "wrist mid-move).")
    p.add_argument("--pickup-distance-from-top", type=float,
                   default=DEFAULT_PICKUP_DISTANCE_FROM_TOP,
                   help="mm below plate top where gripper closes (default: 18)")
    p.add_argument("--skip-load-deck", action="store_true",
                   help="don't overwrite the server's deck")
    p.add_argument("--init-iswap", action="store_true",
                   help="force iSWAP re-initialize before the transfer")
    p.add_argument("--yes", action="store_true",
                   help="skip the pre-goal confirmation prompt")
    main(p.parse_args())
