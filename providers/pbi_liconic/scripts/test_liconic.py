"""Direct pylabrobot test for a Liconic STX44 incubator over RS-232.

Connects to the Liconic PLC via a USB-serial adapter, handshakes, and
runs a progressively less-conservative set of operations. Stage 1 is
pure-query (zero mechanical motion). Later stages exercise the gate,
carousel home, and (optionally) set a new target temperature.

Stages:

  stage 1  query-only: handshake + read target/actual temperature,
           humidity (if climate-controlled), shovel/transfer sensors
  stage 2  initialize: re-home carousel + handler (ST 1900 + ST 1801)
  stage 3  door: open gate → pause → close gate
  stage 4  set-temperature: writes a new target setpoint (value via
           --set-temperature). No climate on *_NC models.

Each stage runs all earlier stages. A ``[y/N]`` prompt gates every
physical motion unless ``--yes`` is passed.

The STX44 PLC is RS-232 @ 9600 8E1 with hardware RTS/CTS flow control.
The default ``--port`` is ``/dev/liconic_stx44``, the symlink installed
by ``install-liconic-udev.sh``. Override with a different path if you
have not installed the udev rule yet.

Run with:

    ./install-liconic-udev.sh            # once, gives /dev/liconic_stx44
    pixi run test-liconic 1
    pixi run test-liconic 2 --yes
    pixi run test-liconic 3
    pixi run test-liconic 4 --set-temperature 37.0
    pixi run test-liconic 1 --port /dev/ttyUSB0
    pixi run test-liconic 1 --model STX44_NC
"""

from __future__ import annotations

import argparse
import asyncio
import sys

from pylabrobot.storage.liconic import ExperimentalLiconicBackend
from pylabrobot.storage.liconic.constants import LiconicType


_SKIP_CONFIRM = False


def _confirm(prompt: str) -> None:
    if _SKIP_CONFIRM:
        print(f"\n{prompt} [auto-yes]")
        return
    ans = input(f"\n{prompt} [y/N] ").strip().lower()
    if ans not in ("y", "yes"):
        print("aborting.")
        sys.exit(1)


def _is_climate_controlled(model: LiconicType) -> bool:
    return not model.value.endswith("_NC")


async def stage_query(backend: ExperimentalLiconicBackend) -> None:
    print("== stage 1: Liconic status queries ==")
    print(f"  model               : {backend.model.value}")
    print(f"  port                : {backend.io.port}")

    if _is_climate_controlled(backend.model):
        target_t = await backend.get_target_temperature()
        actual_t = await backend.get_temperature()
        print(f"  target temperature  : {target_t:.1f} C")
        print(f"  actual temperature  : {actual_t:.1f} C")

        target_h = await backend.get_target_humidity()
        actual_h = await backend.get_humidity()
        print(f"  target humidity     : {target_h * 100:.1f} %")
        print(f"  actual humidity     : {actual_h * 100:.1f} %")
    else:
        print("  (climate control not supported on *_NC models — skipping temp/humidity)")

    # Sensor reads are pure PLC register reads — no motion.
    try:
        transfer_plate = await backend.check_transfer_sensor()
        print(f"  plate at transfer   : {transfer_plate}")
    except RuntimeError as e:
        print(f"  plate at transfer   : <unreadable> ({e})")


async def stage_initialize(backend: ExperimentalLiconicBackend) -> None:
    print("\n== stage 2: initialize (carousel home + handler re-init) ==")
    _confirm("Initialize Liconic (ST 1900 — carousel and handler will home)?")
    await backend.initialize()
    print("  initialize ok")


async def stage_door(backend: ExperimentalLiconicBackend) -> None:
    print("\n== stage 3: door open/close ==")
    _confirm("Open door (ST 1901)?")
    await backend.open_door()
    print("  open_door ok")

    _confirm("Close door (ST 1902)?")
    await backend.close_door()
    print("  close_door ok")


async def stage_set_temperature(
    backend: ExperimentalLiconicBackend, target: float
) -> None:
    print("\n== stage 4: set target temperature ==")
    if not _is_climate_controlled(backend.model):
        print("  skipped — climate control not supported on *_NC models")
        return
    before = await backend.get_target_temperature()
    _confirm(f"Set target temperature from {before:.1f} C to {target:.1f} C?")
    await backend.set_temperature(target)
    after = await backend.get_target_temperature()
    print(f"  set_temperature ok — target now {after:.1f} C")


async def main(args: argparse.Namespace) -> None:
    try:
        model = LiconicType(args.model)
    except ValueError:
        raise SystemExit(
            f"Unknown Liconic model '{args.model}'. "
            f"Valid: {', '.join(m.value for m in LiconicType)}"
        )

    backend = ExperimentalLiconicBackend(model=model, port=args.port)
    print(f"opening {args.port} as {model.value} ...")
    await backend.setup()
    try:
        await stage_query(backend)
        if args.stage >= 2:
            await stage_initialize(backend)
        if args.stage >= 3:
            await stage_door(backend)
        if args.stage >= 4:
            await stage_set_temperature(backend, args.set_temperature)
        print("\nok.")
    finally:
        await backend.stop()


if __name__ == "__main__":
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument(
        "stage", type=int, choices=[1, 2, 3, 4],
        help="1=query, 2=+initialize, 3=+door, 4=+set-temperature",
    )
    p.add_argument(
        "--port", default="/dev/liconic_stx44",
        help="serial port of the Liconic PLC (default: /dev/liconic_stx44, "
             "installed by install-liconic-udev.sh)",
    )
    p.add_argument(
        "--model", default="STX44_IC",
        help="Liconic model — STX44_IC (incubator, default), STX44_HC, "
             "STX44_DC2, STX44_HR, STX44_DR2, STX44_AR, STX44_DF, "
             "STX44_NC (no climate), STX44_DH",
    )
    p.add_argument(
        "--set-temperature", type=float, default=37.0,
        help="target temperature in degrees C for stage 4 (default: 37.0)",
    )
    p.add_argument(
        "--yes", action="store_true", help="skip all confirmation prompts",
    )
    args = p.parse_args()
    _SKIP_CONFIRM = args.yes
    asyncio.run(main(args))
