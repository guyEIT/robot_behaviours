"""Fetch a plate from a Liconic carousel site → the transfer tray.

Prereqs:
  1. Liconic is powered, USB-serial adapter has a /dev/liconic_stx44
     symlink (install-liconic-udev.sh).
  2. A plate is physically in the cassette+position you specify.
  3. The transfer tray is empty.

What the script does:
  - Opens the PLC, configures N racks, assigns a dummy Plate to the
    source site (so the backend can resolve DM0/DM23/DM25/DM5), then
    calls ``fetch_plate_to_loading_tray(plate)``. That issues ST 1905
    (plate to transfer station) + ST 1903 (terminate access).
  - A [y/N] prompt gates the actual motion unless ``--yes`` is passed.

Run with:

    pixi run test-liconic-fetch                           # cassette 1, pos 1
    pixi run test-liconic-fetch --cassette 2 --position 5
    pixi run test-liconic-fetch --rack-model liconic_rack_22mm_17
    pixi run test-liconic-fetch --yes
"""

from __future__ import annotations

import argparse
import asyncio

from liconic_common import (
    DEFAULT_MODEL,
    DEFAULT_PORT,
    DEFAULT_RACK_MODEL,
    DEFAULT_STATE_FILE,
    DEFAULT_TOTAL_CASSETTES,
    build_backend,
    build_racks,
    make_confirm,
    make_test_plate,
    pick_site,
    register_fetch,
)


async def main(args: argparse.Namespace) -> None:
    confirm = make_confirm(args.yes)
    backend = build_backend(args.port, args.model)
    racks = build_racks(args.rack_model, args.total_cassettes)
    site = pick_site(racks, args.cassette, args.position)

    print("== plan ==")
    print(f"  port            : {args.port}")
    print(f"  model           : {args.model}")
    print(f"  rack model      : {args.rack_model} x {args.total_cassettes}")
    print(f"  fetch source    : cassette {args.cassette}, position {args.position}")
    print(f"                    ({site.name})")
    if args.skip_registry:
        print(f"  registry        : SKIPPED (--skip-registry)")
    else:
        print(f"  registry        : {args.state_file}")

    print(f"\nopening {args.port} ...")
    await backend.setup()
    try:
        await backend.set_racks(racks)

        # fetch_plate_to_loading_tray derives the source from plate.parent,
        # so the Plate must be assigned to the source PlateHolder first.
        plate = make_test_plate()
        site.assign_child_resource(plate)

        confirm(
            f"Plate physically at cassette {args.cassette} "
            f"position {args.position}? Proceed with fetch?"
        )
        await backend.fetch_plate_to_loading_tray(plate)
        print("\nok — plate delivered to transfer tray.")
    finally:
        await backend.stop()

    if not args.skip_registry:
        register_fetch(
            state_file=args.state_file,
            cassette=args.cassette,
            position=args.position,
        )


if __name__ == "__main__":
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("--port", default=DEFAULT_PORT,
                   help=f"serial port (default: {DEFAULT_PORT})")
    p.add_argument("--model", default=DEFAULT_MODEL,
                   help=f"Liconic model (default: {DEFAULT_MODEL})")
    p.add_argument("--rack-model", default=DEFAULT_RACK_MODEL,
                   help=f"rack factory name (default: {DEFAULT_RACK_MODEL}); "
                        "determines the motor pitch")
    p.add_argument("--total-cassettes", type=int, default=DEFAULT_TOTAL_CASSETTES,
                   help=f"total cassettes on the unit (default: "
                        f"{DEFAULT_TOTAL_CASSETTES})")
    p.add_argument("--cassette", type=int, default=1,
                   help="source cassette, 1-indexed (default: 1)")
    p.add_argument("--position", type=int, default=1,
                   help="source position within cassette, 1-indexed (default: 1)")
    p.add_argument("--state-file", type=str, default=DEFAULT_STATE_FILE,
                   help=f"plate registry JSON path (default: {DEFAULT_STATE_FILE}; "
                        "same file the liconic_action_server reads)")
    p.add_argument("--skip-registry", action="store_true",
                   help="don't touch the JSON plate registry on success")
    p.add_argument("--yes", action="store_true",
                   help="skip the [y/N] confirm prompt")
    asyncio.run(main(p.parse_args()))
