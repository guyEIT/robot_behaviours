"""Move a 6-well plate from the off-deck ``incubator_handoff`` saved
position → plate carrier at rails=20 site 0 (position 1).

Reverse of ``test_iswap_to_incubator.py`` — same motion rules: shoulder
rotations and long X transits happen at ``safe_y`` (rear of deck), and
every descent starts from traverse Z.

Assumes the plate is physically resting at the incubator handoff site
(e.g. because ``iswap-to-incubator`` delivered it earlier) and that
carrier site 0 at rails 20 is EMPTY.

Run with:
    pixi run iswap-from-incubator
    pixi run iswap-from-incubator --yes
    pixi run iswap-from-incubator --rails 15
    pixi run iswap-from-incubator --pickup-distance-from-top 15
    pixi run iswap-from-incubator --safe-y 350

On failure the plate may be stuck in the gripper. Recover with
    pixi run iswap-release
"""

from __future__ import annotations

import argparse
import asyncio

from iswap_transfer import (
    DEFAULT_PICKUP_DISTANCE_FROM_TOP,
    SAFE_Y,
    TRAVERSE_Z,
    build_lh_with_carrier,
    compute_carrier_grip_xyz,
    ensure_iswap_initialized,
    free_iswap_y_range,
    load_incubator_handoff,
    make_confirm,
    transfer,
)


async def main(args: argparse.Namespace) -> None:
    confirm = make_confirm(args.yes)
    # Build with a placeholder plate on the carrier so compute_carrier_grip_xyz
    # has geometry to read — the plate object is only for size lookup, not
    # as a live resource.
    lh, carrier = build_lh_with_carrier(rails=args.rails, plate_on_site_0=True)
    incubator = load_incubator_handoff()

    pickup_xyz = (incubator["x"], incubator["y"], incubator["z"])
    release_xyz = compute_carrier_grip_xyz(carrier, args.pickup_distance_from_top)
    plate_width = incubator["plate_width"]

    print("== plan ==")
    print(f"  pickup: incubator_handoff — gripper xyz="
          f"({pickup_xyz[0]:.2f}, {pickup_xyz[1]:.2f}, {pickup_xyz[2]:.2f})")
    print(f"  release: carrier rails={args.rails}, site 0 — gripper xyz="
          f"({release_xyz[0]:.2f}, {release_xyz[1]:.2f}, {release_xyz[2]:.2f})")
    print(f"  grip=LEFT, plate_width={plate_width:.1f}, "
          f"traverse Z={TRAVERSE_Z:.1f}, safe Y={args.safe_y:.1f}, "
          f"pickup_distance_from_top={args.pickup_distance_from_top:.1f}")

    offset = (args.offset_x, args.offset_y, args.offset_z)
    if any(offset):
        print(f"  incubator offset (applied to pickup): "
              f"dx={offset[0]:+.2f} dy={offset[1]:+.2f} dz={offset[2]:+.2f}")

    confirm("Proceed with setup (connects to STAR, homes on cold start)?")
    await lh.setup()
    try:
        await ensure_iswap_initialized(lh.backend)
        await free_iswap_y_range(lh.backend)
        await transfer(
            lh.backend,
            pickup_xyz=pickup_xyz,
            release_xyz=release_xyz,
            plate_width=plate_width,
            traverse_z=TRAVERSE_Z,
            safe_y=args.safe_y,
            confirm=confirm,
            pickup_offset=offset,
        )
        print("\nok — plate returned to carrier.")
    finally:
        await lh.stop()


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
                   help="mm to add to the incubator pickup X at runtime "
                        "(without re-saving calibration). Positive = toward right.")
    p.add_argument("--offset-y", type=float, default=0.0,
                   help="mm to add to the incubator pickup Y. Positive = toward back.")
    p.add_argument("--offset-z", type=float, default=0.0,
                   help="mm to add to the incubator pickup Z. Positive = higher "
                        "(grip the plate higher up on its rim).")
    p.add_argument("--yes", action="store_true",
                   help="skip all confirmation prompts")
    asyncio.run(main(p.parse_args()))
