"""Take a plate from the Liconic transfer tray → a carousel site.

Prereqs:
  1. Liconic is powered, USB-serial adapter has a /dev/liconic_stx44
     symlink (install-liconic-udev.sh).
  2. A plate is physically sitting on the transfer station.
  3. The target cassette+position is empty.

What the script does:
  - Opens the PLC, configures N racks (virtual — mirrors how the unit is
    physically populated), then calls ``take_in_plate(site)``. This issues
    WR DM0/DM23/DM25/DM5 + ST 1904 (plate from transfer station), waits
    for the ready flag, then ST 1903 (terminate access).
  - A [y/N] prompt gates the actual motion unless ``--yes`` is passed.

Run with:

    pixi run test-liconic-take-in                         # cassette 1, pos 1
    pixi run test-liconic-take-in --cassette 2 --position 5
    pixi run test-liconic-take-in --rack-model liconic_rack_22mm_17
    pixi run test-liconic-take-in --yes
"""

from __future__ import annotations

import argparse
import asyncio

import time

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
    register_take_in,
)


async def main(args: argparse.Namespace) -> None:
    confirm = make_confirm(args.yes)
    backend = build_backend(args.port, args.model)
    racks = build_racks(args.rack_model, args.total_cassettes)
    site = pick_site(racks, args.cassette, args.position)

    plate_name = args.plate_name or f"manual_takein_{int(time.time())}"

    print("== plan ==")
    print(f"  port            : {args.port}")
    print(f"  model           : {args.model}")
    print(f"  rack model      : {args.rack_model} x {args.total_cassettes}")
    print(f"  take-in target  : cassette {args.cassette}, position {args.position}")
    print(f"                    ({site.name})")
    print(f"  plate_name      : {plate_name!r}")
    if args.skip_registry:
        print(f"  registry        : SKIPPED (--skip-registry)")
    else:
        print(f"  registry        : {args.state_file}")

    print(f"\nopening {args.port} ...")
    await backend.setup()
    try:
        await backend.set_racks(racks)

        # The backend only uses the plate for barcode association on
        # take-in; the motion is specified entirely by `site`.
        plate = make_test_plate()

        confirm("Plate physically on transfer tray? Proceed with take-in?")
        await backend.take_in_plate(plate, site)
        print("\nok — plate taken in from transfer tray to carousel.")
    finally:
        await backend.stop()

    # After the backend stop so a registry-update warning doesn't block
    # the PLC close. The motion is already done at this point.
    if not args.skip_registry:
        register_take_in(
            state_file=args.state_file,
            plate_name=plate_name,
            cassette=args.cassette,
            position=args.position,
            barcode=args.barcode,
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
                   help="target cassette, 1-indexed (default: 1)")
    p.add_argument("--position", type=int, default=1,
                   help="target position within cassette, 1-indexed (default: 1)")
    p.add_argument("--plate-name", type=str, default="",
                   help="registry label for the plate (default: "
                        "manual_takein_<unix_ts>). Must be unique across the "
                        "registry.")
    p.add_argument("--barcode", type=str, default="",
                   help="barcode stored alongside the registry entry")
    p.add_argument("--state-file", type=str, default=DEFAULT_STATE_FILE,
                   help=f"plate registry JSON path (default: {DEFAULT_STATE_FILE}; "
                        "same file the liconic_action_server reads)")
    p.add_argument("--skip-registry", action="store_true",
                   help="don't touch the JSON plate registry on success")
    p.add_argument("--yes", action="store_true",
                   help="skip the [y/N] confirm prompt")
    asyncio.run(main(p.parse_args()))
