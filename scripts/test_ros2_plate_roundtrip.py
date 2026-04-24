"""Full workcell round-trip: Hamilton on-deck plate → Liconic cassette → back.

Chains four ROS 2 motions against the two action servers, in order:

  1. Hamilton ``move_resource``: on-deck carrier site 0 (rails=20 by
     default) → off-deck ``incubator_handoff`` (Liconic transfer dock).
  2. Liconic ``take_in``: transfer tray → (cassette, position) and
     registers the plate in the Liconic JSON registry.
  3. Liconic ``fetch``: (cassette, position) → transfer tray; registry
     moves the entry back onto the loading tray.
  4. Hamilton ``move_resource``: ``incubator_handoff`` → on-deck carrier
     site 0.

Both servers must be running in their own terminals:

    pixi run -e ros2 action-server          # Hamilton STAR
    pixi run -e ros2 liconic-server         # Liconic STX44

Physical prereqs:
  - A plate is on the Hamilton carrier at rails=20, site 0.
  - ``incubator_handoff`` is registered (or will be re-registered).
  - Target Liconic cassette/position is empty.

Run with:

    pixi run -e ros2 liconic-workcell-roundtrip
    pixi run -e ros2 liconic-workcell-roundtrip -- --yes
    pixi run -e ros2 liconic-workcell-roundtrip -- --rails 15 \\
        --cassette 2 --position 5
    pixi run -e ros2 liconic-workcell-roundtrip -- --skip-load-deck \\
        --skip-register
    pixi run -e ros2 liconic-workcell-roundtrip -- --only hamilton-to
"""

from __future__ import annotations

import argparse
import sys
import time

import rclpy
from geometry_msgs.msg import Vector3
from rclpy.action import ActionClient
from rclpy.node import Node

from hamilton_star_msgs.action import MoveResource
from liconic_msgs.action import Fetch, TakeIn
from liconic_msgs.srv import (
    ClearCassettePosition as LiconicClearCassettePosition,
    ClearLoadingTray as LiconicClearLoadingTray,
    GetStatus as LiconicGetStatus,
)

# Reuse the Hamilton-side helpers — no point duplicating deck build,
# handoff registration, error-recovery scaffolding. These are plain
# top-level defs in the sibling script; importing is safe because that
# file gates argparse behind __name__ == "__main__".
from test_ros2_move_to_incubator import (
    CARRIER_NAME,
    DEFAULT_HOTEL_CLEARANCE,
    DEFAULT_HOTEL_DEPTH,
    HANDOFF_NAME,
    PLATE_NAME,
    build_deck_json,
    clear_error_if_needed,
    initialize_iswap,
    load_deck_on_server,
    register_handoff,
)
from iswap_transfer import (
    DEFAULT_PICKUP_DISTANCE_FROM_TOP,
    load_incubator_handoff,
)


HAMILTON_NODE = "hamilton_star_action_server"
LICONIC_NODE = "liconic_action_server"
MOVE_RESOURCE_ACTION = f"/{HAMILTON_NODE}/move_resource"
LICONIC_TAKE_IN_ACTION = f"/{LICONIC_NODE}/take_in"
LICONIC_FETCH_ACTION = f"/{LICONIC_NODE}/fetch"
LICONIC_STATUS_SRV = f"/{LICONIC_NODE}/get_status"
LICONIC_CLEAR_TRAY_SRV = f"/{LICONIC_NODE}/clear_loading_tray"
LICONIC_CLEAR_SLOT_SRV = f"/{LICONIC_NODE}/clear_cassette_position"


STEPS = ("hamilton-to", "liconic-take-in", "liconic-fetch", "hamilton-from")


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


# ---- Liconic sanity check ----

def check_liconic_connected(node: Node) -> None:
    cli = node.create_client(LiconicGetStatus, LICONIC_STATUS_SRV)
    if not cli.wait_for_service(timeout_sec=5.0):
        raise RuntimeError(
            f"service {LICONIC_STATUS_SRV} not available — is "
            f"'pixi run -e ros2 liconic-server' running?"
        )
    fut = cli.call_async(LiconicGetStatus.Request())
    rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
    resp = fut.result()
    if resp is None:
        raise RuntimeError(f"no response from {LICONIC_STATUS_SRV}")
    if not resp.connected:
        raise RuntimeError(
            "Liconic server reports not connected to the PLC. "
            "Check the server log and the /dev/liconic_stx44 symlink."
        )
    tray_name = resp.loading_tray_plate.plate_name or "empty"
    print(f"  Liconic      : {resp.model} connected, "
          f"{len(resp.plates)} plate(s) stored, tray={tray_name!r}")


# ---- Hamilton MoveResource (two directions, one helper) ----

def send_move_resource(
    node: Node,
    *,
    source: str,
    destination: str,
    offset_pickup: tuple[float, float, float] = (0.0, 0.0, 0.0),
    offset_destination: tuple[float, float, float] = (0.0, 0.0, 0.0),
    pickup_distance_from_top: float = DEFAULT_PICKUP_DISTANCE_FROM_TOP,
    pickup_grip_direction: str = "",
    drop_grip_direction: str = "",
) -> None:
    client = ActionClient(node, MoveResource, MOVE_RESOURCE_ACTION)
    if not client.wait_for_server(timeout_sec=10.0):
        raise RuntimeError(
            f"action {MOVE_RESOURCE_ACTION} not available — is the Hamilton "
            "server running?"
        )

    goal = MoveResource.Goal()
    goal.resource = source
    goal.to = destination
    goal.transport = "iswap"
    goal.pickup_direction = pickup_grip_direction
    goal.drop_direction = drop_grip_direction
    goal.pickup_offset = Vector3(
        x=float(offset_pickup[0]),
        y=float(offset_pickup[1]),
        z=float(offset_pickup[2]),
    )
    goal.destination_offset = Vector3(
        x=float(offset_destination[0]),
        y=float(offset_destination[1]),
        z=float(offset_destination[2]),
    )
    goal.pickup_distance_from_top = float(pickup_distance_from_top)
    goal.use_unsafe_hotel = False
    goal.hotel_depth = 0.0
    goal.hotel_clearance_height = 0.0

    def on_feedback(fb_msg) -> None:
        fb = fb_msg.feedback
        if fb.stage:
            print(f"  stage: {fb.stage} ({fb.progress:.0%})")

    print(f"\n-> hamilton move_resource: {source!r} → {destination!r}")
    send_fut = client.send_goal_async(goal, feedback_callback=on_feedback)
    rclpy.spin_until_future_complete(node, send_fut)
    handle = send_fut.result()
    if handle is None or not handle.accepted:
        raise RuntimeError(
            "Hamilton goal rejected (check ~/status for FSM state)"
        )
    result_fut = handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_fut)
    result = result_fut.result().result
    if not result.success:
        raise RuntimeError(f"move_resource failed: {result.message}")
    for w in (result.warnings or []):
        print(f"  warning: {w}")
    print(f"  ok — {result.message or 'move_resource completed'}")


# ---- Liconic take_in / fetch ----

def send_liconic_take_in(
    node: Node, plate_name: str, cassette: int, position: int, barcode: str,
) -> None:
    client = ActionClient(node, TakeIn, LICONIC_TAKE_IN_ACTION)
    if not client.wait_for_server(timeout_sec=10.0):
        raise RuntimeError(f"action {LICONIC_TAKE_IN_ACTION} not available")

    goal = TakeIn.Goal()
    goal.plate_name = plate_name
    goal.barcode = barcode
    goal.cassette = cassette
    goal.position = position

    def on_feedback(fb_msg) -> None:
        if fb_msg.feedback.stage:
            print(f"  take_in stage: {fb_msg.feedback.stage}")

    print(f"\n-> liconic take_in plate={plate_name!r} "
          f"cassette={cassette} position={position}")
    send_fut = client.send_goal_async(goal, feedback_callback=on_feedback)
    rclpy.spin_until_future_complete(node, send_fut)
    handle = send_fut.result()
    if handle is None or not handle.accepted:
        raise RuntimeError("take_in goal rejected by Liconic server")
    result_fut = handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_fut)
    result = result_fut.result().result
    if not result.success:
        raise RuntimeError(f"take_in failed: {result.message}")
    print(f"  ok — {result.message}")


def clear_liconic_slot_if_stale(
    node: Node, cassette: int, position: int,
) -> None:
    """Drop any stale registry entry at (cassette, position).

    Called before take-in so an interrupted prior run (whose fetch never
    completed, leaving the cassette entry stranded) doesn't block the
    new workflow. There is no cassette-slot sensor, so this is purely a
    registry mutation — the caller is asserting the physical slot is
    empty.
    """
    cli = node.create_client(LiconicClearCassettePosition, LICONIC_CLEAR_SLOT_SRV)
    if not cli.wait_for_service(timeout_sec=5.0):
        print(f"\nWARNING: {LICONIC_CLEAR_SLOT_SRV} not available — "
              "Liconic registry not pre-cleared.")
        return
    req = LiconicClearCassettePosition.Request()
    req.cassette = cassette
    req.position = position
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
    resp = fut.result()
    if resp is None or not resp.success:
        msg = resp.message if resp else "no response"
        print(f"\nWARNING: clear_cassette_position failed: {msg}")
        return
    if resp.cleared_plate_name:
        print(f"  liconic registry: cleared stale entry "
              f"{resp.cleared_plate_name!r} from cassette {cassette} "
              f"position {position}")


def clear_liconic_tray(node: Node) -> None:
    """Tell the Liconic server the tray is physically empty.

    The Hamilton has just picked the plate up off the transfer tray via
    iSWAP — that event is invisible to the Liconic PLC, so we have to
    explicitly update its registry. Failure is logged but not raised:
    the physical move already succeeded and it's the registry that's
    out of sync, not the hardware.
    """
    cli = node.create_client(LiconicClearLoadingTray, LICONIC_CLEAR_TRAY_SRV)
    if not cli.wait_for_service(timeout_sec=5.0):
        print(f"\nWARNING: {LICONIC_CLEAR_TRAY_SRV} not available — "
              "Liconic registry tray entry not cleared.")
        return
    fut = cli.call_async(LiconicClearLoadingTray.Request())
    rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
    resp = fut.result()
    if resp is None or not resp.success:
        msg = resp.message if resp else "no response"
        print(f"\nWARNING: clear_loading_tray failed: {msg}")
        return
    if resp.cleared_plate_name:
        print(f"  liconic registry: cleared {resp.cleared_plate_name!r} "
              "from loading tray")
    else:
        print("  liconic registry: tray was already empty")


def send_liconic_fetch(node: Node, plate_name: str) -> str:
    client = ActionClient(node, Fetch, LICONIC_FETCH_ACTION)
    if not client.wait_for_server(timeout_sec=10.0):
        raise RuntimeError(f"action {LICONIC_FETCH_ACTION} not available")

    goal = Fetch.Goal()
    goal.plate_name = plate_name   # name-based lookup in the registry

    def on_feedback(fb_msg) -> None:
        if fb_msg.feedback.stage:
            print(f"  fetch stage: {fb_msg.feedback.stage}")

    print(f"\n-> liconic fetch plate={plate_name!r}")
    send_fut = client.send_goal_async(goal, feedback_callback=on_feedback)
    rclpy.spin_until_future_complete(node, send_fut)
    handle = send_fut.result()
    if handle is None or not handle.accepted:
        raise RuntimeError("fetch goal rejected by Liconic server")
    result_fut = handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_fut)
    result = result_fut.result().result
    if not result.success:
        raise RuntimeError(f"fetch failed: {result.message}")
    print(f"  ok — {result.message}")
    return result.plate_name


# ---- orchestration ----

def main(args: argparse.Namespace) -> None:
    confirm = make_confirm(args.yes)
    incubator = load_incubator_handoff()
    incubator_xyz = (incubator["x"], incubator["y"], incubator["z"])

    selected = set(args.only) if args.only else set(STEPS)
    invalid = selected - set(STEPS)
    if invalid:
        raise SystemExit(f"--only got unknown step(s): {sorted(invalid)}")

    plate_name = args.plate_name or f"{PLATE_NAME}_{int(time.time())}"

    print("== plan ==")
    print(f"  Hamilton node    : /{HAMILTON_NODE}")
    print(f"  Liconic  node    : /{LICONIC_NODE}")
    print(f"  plate            : {plate_name!r}  (on carrier, rails={args.rails})")
    print(f"  handoff          : {HANDOFF_NAME!r} @ "
          f"({incubator_xyz[0]:.1f}, {incubator_xyz[1]:.1f}, "
          f"{incubator_xyz[2]:.1f})")
    print(f"  Liconic target   : cassette {args.cassette}, position {args.position}")
    print(f"  sequence         : "
          f"{' → '.join(s for s in STEPS if s in selected)}")
    if args.skip_load_deck:
        print("  deck load        : SKIPPED")
    if args.skip_register:
        print("  handoff register : SKIPPED")

    confirm("Proceed with the full workcell round-trip?")

    hamilton_needed = bool(selected & {"hamilton-to", "hamilton-from"})
    liconic_needed = bool(selected & {"liconic-take-in", "liconic-fetch"})

    rclpy.init()
    node = rclpy.create_node("workcell_roundtrip_client")
    try:
        print("\n== sanity checks ==")
        if liconic_needed:
            check_liconic_connected(node)

        if hamilton_needed:
            was_errored = clear_error_if_needed(
                node, "pre-run auto-clear (workcell roundtrip)",
            )
            if args.init_iswap or was_errored:
                initialize_iswap(node)

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
                print(f"  deck loaded on Hamilton — hash={deck_hash[:12]}…")

        try:
            # --- 1. Hamilton: on-deck → incubator handoff -----------------
            if "hamilton-to" in selected:
                confirm(
                    "Step 1/4: Hamilton will deliver the plate to the "
                    "incubator handoff. Proceed?"
                )
                send_move_resource(
                    node,
                    source=PLATE_NAME, destination=HANDOFF_NAME,
                    offset_destination=(
                        args.to_offset_x, args.to_offset_y, args.to_offset_z,
                    ),
                    pickup_distance_from_top=args.pickup_distance_from_top,
                )

            # --- 2. Liconic take_in (tray → cassette) ---------------------
            if "liconic-take-in" in selected:
                confirm(
                    f"Step 2/4: Liconic take_in {plate_name!r} → "
                    f"cassette {args.cassette} position {args.position}. "
                    "(Plate on transfer tray?)"
                )
                # Pre-flight: drop any stale registry entry at the target.
                # Guards against the common "interrupted prior run left
                # cassette entry stranded" case. Does NOT guard against a
                # physically-occupied slot — the take-in motion will hit
                # that and error.
                clear_liconic_slot_if_stale(
                    node, cassette=args.cassette, position=args.position,
                )
                send_liconic_take_in(
                    node, plate_name=plate_name,
                    cassette=args.cassette, position=args.position,
                    barcode=args.barcode,
                )

            # --- 3. Liconic fetch (cassette → tray) -----------------------
            if "liconic-fetch" in selected:
                confirm(
                    f"Step 3/4: Liconic fetch {plate_name!r} back to "
                    "transfer tray. Proceed?"
                )
                fetched = send_liconic_fetch(node, plate_name=plate_name)
                if fetched != plate_name:
                    print(
                        f"  NOTE: server echoed plate {fetched!r}; "
                        f"expected {plate_name!r}"
                    )

            # --- 4. Hamilton: incubator handoff → on-deck -----------------
            if "hamilton-from" in selected:
                confirm(
                    "Step 4/4: Hamilton will retrieve the plate from the "
                    "incubator handoff. Proceed?"
                )
                # Always clear the Liconic tray registry entry — whether
                # the move succeeds OR fails mid-motion. Rationale: once
                # the iSWAP grips the plate (stage 3 of the 6-stage jog),
                # it has physically left the tray. A failure AFTER pickup
                # (most common: transit collision, drop error) leaves the
                # tray physically empty but, without a finally, would
                # leave the registry marked occupied — which is exactly
                # what was breaking subsequent runs.
                # Cost of false-clear: a failure BEFORE pickup (rare on
                # the from-handoff direction — stages 1-2 are non-gripping
                # park/rotate) would mark the registry empty while the
                # plate is still on the tray. The tray sensor pre-flight
                # catches that case on the next take-in.
                try:
                    send_move_resource(
                        node,
                        source=HANDOFF_NAME, destination=PLATE_NAME,
                        offset_pickup=(
                            args.from_offset_x, args.from_offset_y,
                            args.from_offset_z,
                        ),
                        pickup_distance_from_top=args.pickup_distance_from_top,
                    )
                finally:
                    clear_liconic_tray(node)

            print("\nok — workcell round-trip complete.")
        except Exception:
            # Only Hamilton has an FSM that benefits from reset_error +
            # iSWAP re-init. Liconic errors surface as action aborts and
            # don't leave the STAR in a bad state, so just re-raise.
            if hamilton_needed:
                try:
                    if clear_error_if_needed(node, "post-failure auto-clear"):
                        print("  NOTE: Hamilton FSM reset. Check for a stuck "
                              "plate in the iSWAP before the next run.")
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
    # Hamilton geometry
    p.add_argument("--rails", type=int, default=20,
                   help="Hamilton carrier rails position (default: 20)")
    p.add_argument("--pickup-distance-from-top", type=float,
                   default=DEFAULT_PICKUP_DISTANCE_FROM_TOP,
                   help="mm below plate top where iSWAP closes (default: 18)")
    p.add_argument("--hotel-depth", type=float, default=DEFAULT_HOTEL_DEPTH,
                   help=f"hotel approach depth in mm "
                        f"(default: {DEFAULT_HOTEL_DEPTH:.0f}). "
                        "Stored on the handoff record.")
    p.add_argument("--hotel-clearance-height", type=float,
                   default=DEFAULT_HOTEL_CLEARANCE,
                   help=f"hotel vertical clearance in mm "
                        f"(default: {DEFAULT_HOTEL_CLEARANCE:.1f})")
    p.add_argument("--grip-direction", type=str, default="RIGHT",
                   choices=("FRONT", "BACK", "LEFT", "RIGHT"),
                   help="iSWAP grip_direction stored on the handoff record")
    # Liconic target
    p.add_argument("--cassette", type=int, default=1,
                   help="Liconic cassette, 1-indexed (default: 1)")
    p.add_argument("--position", type=int, default=1,
                   help="Liconic position within the cassette, 1-indexed (default: 1)")
    p.add_argument("--plate-name", type=str, default="",
                   help="Plate label used for the Liconic registry. Default: "
                        f"{PLATE_NAME}_<unix_ts> (unique per run).")
    p.add_argument("--barcode", type=str, default="",
                   help="barcode stored with the Liconic registry entry")
    # Per-side fine-tuning offsets (plate may drift mm-scale across dock
    # swaps — same knobs as the single-direction scripts expose).
    p.add_argument("--to-offset-x", type=float, default=0.0,
                   help="mm added to incubator DROP X (hamilton-to step)")
    p.add_argument("--to-offset-y", type=float, default=0.0)
    p.add_argument("--to-offset-z", type=float, default=0.0)
    p.add_argument("--from-offset-x", type=float, default=0.0,
                   help="mm added to incubator PICKUP X (hamilton-from step)")
    p.add_argument("--from-offset-y", type=float, default=0.0)
    p.add_argument("--from-offset-z", type=float, default=0.0)
    # Setup skips
    p.add_argument("--skip-load-deck", action="store_true",
                   help=f"don't overwrite deck — assume {CARRIER_NAME} and "
                        f"{PLATE_NAME} are already present.")
    p.add_argument("--skip-register", action="store_true",
                   help=f"don't re-register {HANDOFF_NAME!r} — assume it's "
                        "already in the Hamilton handoff registry.")
    p.add_argument("--init-iswap", action="store_true",
                   help="force iSWAP re-initialize before step 1")
    p.add_argument("--only", nargs="+", choices=STEPS, default=None,
                   help=f"run a subset of steps in order. Available: "
                        f"{', '.join(STEPS)}.")
    p.add_argument("--yes", action="store_true",
                   help="skip all confirmation prompts")
    main(p.parse_args())
