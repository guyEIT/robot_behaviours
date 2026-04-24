"""Round-trip client test against the Liconic action server.

Takes a plate from the transfer tray into a cassette position, pauses,
then fetches it back out. Both motions go through the ROS 2 action
server (``liconic_action_server``); no direct pylabrobot calls. The
server's JSON plate registry is updated and persisted on each step.

Physical prereqs:
  - ``liconic_action_server`` is running (``pixi run -e ros2 liconic-server``).
  - The target cassette/position is empty.
  - A plate is physically sitting on the transfer tray at the start.
  - The plate_name you pick isn't already registered (the server will
    reject duplicates).

Flow:
  1. /get_status — print a summary (connected, climate, stored plates).
  2. /take_in goal — wait for result, print feedback stages.
  3. Operator pause (default: just press enter; ``--yes`` auto-continues).
  4. /fetch goal — wait for result.
  5. Operator retrieves the plate from the tray.

Run with:

    pixi run -e ros2 liconic-roundtrip
    pixi run -e ros2 liconic-roundtrip -- --yes
    pixi run -e ros2 liconic-roundtrip -- --cassette 2 --position 5
    pixi run -e ros2 liconic-roundtrip -- --plate-name my_plate
    pixi run -e ros2 liconic-roundtrip -- --barcode ABC123
    pixi run -e ros2 liconic-roundtrip -- --no-fetch        # take-in only
    pixi run -e ros2 liconic-roundtrip -- --no-take-in      # fetch only
"""

from __future__ import annotations

import argparse
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from liconic_msgs.action import Fetch, TakeIn
from liconic_msgs.srv import GetStatus


NODE_NAME = "liconic_action_server"
TAKE_IN_ACTION = f"/{NODE_NAME}/take_in"
FETCH_ACTION = f"/{NODE_NAME}/fetch"
GET_STATUS_SRV = f"/{NODE_NAME}/get_status"


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


def print_status(node: Node) -> None:
    cli = node.create_client(GetStatus, GET_STATUS_SRV)
    if not cli.wait_for_service(timeout_sec=5.0):
        raise RuntimeError(f"service {GET_STATUS_SRV} not available — is the server running?")
    fut = cli.call_async(GetStatus.Request())
    rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
    resp = fut.result()
    if resp is None:
        raise RuntimeError(f"no response from {GET_STATUS_SRV}")
    print("== server status ==")
    print(f"  connected          : {resp.connected}")
    print(f"  model              : {resp.model}")
    if resp.climate_supported:
        print(f"  target temperature : {resp.target_temperature_c:.1f} C")
        print(f"  actual temperature : {resp.actual_temperature_c:.1f} C")
        print(f"  target humidity    : {resp.target_humidity * 100:.1f} %")
        print(f"  actual humidity    : {resp.actual_humidity * 100:.1f} %")
    else:
        print("  climate control    : N/A (*_NC model)")
    tray = resp.loading_tray_plate
    if tray.plate_name:
        print(f"  loading tray       : {tray.plate_name!r}")
    else:
        print("  loading tray       : empty")
    if resp.plates:
        print("  stored plates      :")
        for p in resp.plates:
            print(
                f"      cassette {p.cassette} pos {p.position}: "
                f"{p.plate_name!r}"
                + (f" (barcode {p.barcode!r})" if p.barcode else "")
            )
    else:
        print("  stored plates      : none")
    if not resp.connected:
        raise RuntimeError("server reports not connected — abort before sending motion goals")


def send_take_in(
    node: Node, plate_name: str, cassette: int, position: int, barcode: str,
) -> None:
    client = ActionClient(node, TakeIn, TAKE_IN_ACTION)
    if not client.wait_for_server(timeout_sec=10.0):
        raise RuntimeError(f"action {TAKE_IN_ACTION} not available")

    goal = TakeIn.Goal()
    goal.plate_name = plate_name
    goal.barcode = barcode
    goal.cassette = cassette
    goal.position = position

    def on_feedback(fb_msg) -> None:
        fb = fb_msg.feedback
        if fb.stage:
            print(f"  take_in stage: {fb.stage}")

    print(f"\n-> take_in plate={plate_name!r} cassette={cassette} position={position}")
    send_fut = client.send_goal_async(goal, feedback_callback=on_feedback)
    rclpy.spin_until_future_complete(node, send_fut)
    handle = send_fut.result()
    if handle is None or not handle.accepted:
        raise RuntimeError("take_in goal rejected by server")

    result_fut = handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_fut)
    result = result_fut.result().result
    if not result.success:
        raise RuntimeError(f"take_in failed: {result.message}")
    print(f"  ok — {result.message}")


def send_fetch(
    node: Node, plate_name: str, cassette: int, position: int, by_name: bool,
) -> str:
    client = ActionClient(node, Fetch, FETCH_ACTION)
    if not client.wait_for_server(timeout_sec=10.0):
        raise RuntimeError(f"action {FETCH_ACTION} not available")

    goal = Fetch.Goal()
    if by_name:
        goal.plate_name = plate_name
        goal.cassette = 0
        goal.position = 0
    else:
        goal.plate_name = ""
        goal.cassette = cassette
        goal.position = position

    def on_feedback(fb_msg) -> None:
        fb = fb_msg.feedback
        if fb.stage:
            print(f"  fetch stage: {fb.stage}")

    lookup = (
        f"plate_name={plate_name!r}" if by_name
        else f"cassette={cassette} position={position}"
    )
    print(f"\n-> fetch {lookup}")
    send_fut = client.send_goal_async(goal, feedback_callback=on_feedback)
    rclpy.spin_until_future_complete(node, send_fut)
    handle = send_fut.result()
    if handle is None or not handle.accepted:
        raise RuntimeError("fetch goal rejected by server")

    result_fut = handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_fut)
    result = result_fut.result().result
    if not result.success:
        raise RuntimeError(f"fetch failed: {result.message}")
    print(f"  ok — {result.message}")
    return result.plate_name


def main(args: argparse.Namespace) -> None:
    confirm = make_confirm(args.yes)
    plate_name = args.plate_name or f"test_plate_{int(time.time())}"

    print("== plan ==")
    print(f"  server   : /{NODE_NAME}")
    print(f"  plate    : {plate_name!r}")
    print(f"  target   : cassette {args.cassette}, position {args.position}")
    if args.barcode:
        print(f"  barcode  : {args.barcode!r}")
    steps = []
    if not args.no_take_in:
        steps.append("take_in (tray → cassette)")
    if not args.no_fetch:
        steps.append("fetch (cassette → tray)")
    print(f"  sequence : {' → '.join(steps) if steps else '(nothing — both skipped?)'}")

    if not steps:
        print("both --no-take-in and --no-fetch set; nothing to do.")
        return

    rclpy.init()
    node = rclpy.create_node("liconic_roundtrip_test_client")
    try:
        print()
        print_status(node)

        if not args.no_take_in:
            confirm("Plate physically on transfer tray? Proceed with take_in?")
            send_take_in(
                node, plate_name=plate_name,
                cassette=args.cassette, position=args.position,
                barcode=args.barcode,
            )

        if not args.no_fetch and not args.no_take_in:
            confirm("Pause before fetch (eye-check the plate in the cassette)?")

        if not args.no_fetch:
            # Prefer name-based lookup when we know the plate name
            # (either we just placed it, or the user supplied one).
            # Otherwise fall back to (cassette, position) resolution.
            by_name = (not args.no_take_in) or bool(args.plate_name)
            fetched = send_fetch(
                node, plate_name=plate_name,
                cassette=args.cassette, position=args.position,
                by_name=by_name,
            )
            print(f"\n== final ==")
            print(f"  fetched plate  : {fetched!r}")
            print("  physical plate : now on the Liconic loading tray — retrieve it.")

        print("\nok — round-trip complete.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument(
        "--plate-name", type=str, default="",
        help="plate label for the registry (default: test_plate_<unix_ts>). "
             "Must be unique across all plates currently registered.",
    )
    p.add_argument(
        "--cassette", type=int, default=1,
        help="target cassette, 1-indexed (default: 1)",
    )
    p.add_argument(
        "--position", type=int, default=1,
        help="target position within cassette, 1-indexed (default: 1)",
    )
    p.add_argument(
        "--barcode", type=str, default="",
        help="barcode to store with the registry entry (default: '')",
    )
    p.add_argument(
        "--no-take-in", action="store_true",
        help="skip the take_in step; assumes the plate is already registered "
             "at --cassette/--position (or use --plate-name to look up).",
    )
    p.add_argument(
        "--no-fetch", action="store_true",
        help="skip the fetch step; leaves the plate in the cassette.",
    )
    p.add_argument(
        "--yes", action="store_true",
        help="skip all confirmation prompts",
    )
    main(p.parse_args())
