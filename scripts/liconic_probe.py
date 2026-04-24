"""Raw Liconic PLC register probe — for diagnosing non-pylabrobot faults.

Sends arbitrary ``RD <register>`` commands to the PLC and prints the raw
responses. Useful for tracking down alarms that don't surface through
pylabrobot's wrapped methods (e.g. CO₂ low, door, compressor).

Requires direct USB — STOP the ros2 liconic action server first:

    pkill -f liconic_action_server

Then run one of:

    pixi run liconic-probe                                   # default bundle
    pixi run liconic-probe -- --preset alarms
    pixi run liconic-probe -- --preset sensors
    pixi run liconic-probe -- --reg "RD 1814" --reg "RD DM200" --reg "RD DM201"
    pixi run liconic-probe -- --sweep-dm 200 220             # RD DM200..DM220
    pixi run liconic-probe -- --sweep-bit 1800 1830          # RD 1800..RD 1830

Register reading is read-only and has no side-effects on plate state, so
this is safe to run while the 30 s alarm is beeping — it won't make
anything worse. The PLC ack'ing these reads while the alarm is pending
is fine.
"""

from __future__ import annotations

import argparse
import asyncio
import sys
from typing import Iterable, List

from liconic_common import DEFAULT_MODEL, DEFAULT_PORT, build_backend


# Curated diagnostic bundles. Values are raw command strings; the PLC
# accepts ``RD <register>`` with the operand in two forms: plain (for
# bit/coil flags like ``1814``, ``1813``) or ``DM<n>`` (for data memory
# words like ``DM200``, ``DM982``).
_PRESETS = {
    "default": [
        # plate-handling flags + error (what pylabrobot already queries)
        "RD 1914",  # plate-ready flag
        "RD 1915",  # ready flag
        "RD 1814",  # error flag (1 = latched)
        "RD DM200", # error code (HandlingError enum)
        # climate readbacks
        "RD DM890", "RD DM982",    # target / actual temperature
        "RD DM893", "RD DM983",    # target / actual humidity
        "RD DM894", "RD DM984",    # target / actual CO2
        "RD DM895", "RD DM985",    # target / actual N2
        # sensors
        "RD 1812",  # shovel sensor
        "RD 1813",  # transfer-tray sensor
        "RD 1807",  # 2nd transfer sensor
        # shaker config
        "RD DM39",
    ],
    "alarms": [
        # A grab-bag of alarm-ish registers to try when the handling
        # error register (RD 1814 / DM200) shows clean. The exact layout
        # varies by Liconic firmware build; these are what's known to
        # surface climate / gas / door faults on STX44 variants. Read
        # everything, then look for non-zero values.
        "RD 1814",                       # handling error flag (known)
        "RD DM200",                      # handling error code
        # Climate subsystem (may expose its own alarm bits on _IC / _HC):
        "RD DM210", "RD DM211", "RD DM212",
        "RD DM220", "RD DM221", "RD DM222",
        # Gas-mode flags — the PLC usually indicates absence of required
        # gas here when the unit is configured as CO2/N2.
        "RD DM230", "RD DM231", "RD DM232",
        # Door / cover sensor bit range (guess based on STX44 comms):
        "RD 1815", "RD 1816", "RD 1817", "RD 1818", "RD 1819", "RD 1820",
    ],
    "sensors": [
        "RD 1812",  # shovel
        "RD 1813",  # transfer tray
        "RD 1807",  # 2nd transfer
        "RD 1800", "RD 1801", "RD 1802", "RD 1803", "RD 1804", "RD 1805",
        "RD 1806", "RD 1808", "RD 1809", "RD 1810", "RD 1811",
        # Door / swap / misc flags
        "RD 1912", "RD 1911", "RD 1910",
    ],
}


def _sweep_dm(lo: int, hi: int) -> List[str]:
    return [f"RD DM{i}" for i in range(lo, hi + 1)]


def _sweep_bit(lo: int, hi: int) -> List[str]:
    return [f"RD {i}" for i in range(lo, hi + 1)]


async def probe(backend, commands: Iterable[str]) -> None:
    for cmd in commands:
        try:
            resp = await backend._send_command(cmd)
        except Exception as exc:  # noqa: BLE001
            print(f"  {cmd:<16} -> <error: {type(exc).__name__}: {exc}>")
            continue
        marker = "  *" if (resp not in ("", "0", "00000")) else "   "
        print(f"{marker} {cmd:<16} -> {resp!r}")


async def main(args: argparse.Namespace) -> None:
    commands: List[str] = []
    if args.preset:
        commands.extend(_PRESETS[args.preset])
    if args.reg:
        commands.extend(args.reg)
    if args.sweep_dm:
        commands.extend(_sweep_dm(*args.sweep_dm))
    if args.sweep_bit:
        commands.extend(_sweep_bit(*args.sweep_bit))
    if not commands:
        commands = _PRESETS["default"]

    backend = build_backend(args.port, args.model)
    print(f"opening {args.port} as {args.model} ...")
    await backend.setup()
    try:
        print(f"\nprobing {len(commands)} register(s) (* = non-zero):\n")
        await probe(backend, commands)
    finally:
        await backend.stop()


if __name__ == "__main__":
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("--port", default=DEFAULT_PORT)
    p.add_argument("--model", default=DEFAULT_MODEL)
    p.add_argument(
        "--preset", choices=sorted(_PRESETS), default=None,
        help="probe a curated register bundle "
             f"({'/'.join(sorted(_PRESETS))}); default: 'default' bundle if "
             "no other flags",
    )
    p.add_argument(
        "--reg", action="append", default=None,
        help=(
            "raw command to send (e.g. --reg 'RD 1814' --reg 'RD DM200'). "
            "Can be repeated. Combine with --preset / --sweep-* if you "
            "want both."
        ),
    )
    p.add_argument(
        "--sweep-dm", type=int, nargs=2, metavar=("LO", "HI"), default=None,
        help="probe RD DM<LO>..RD DM<HI> inclusive",
    )
    p.add_argument(
        "--sweep-bit", type=int, nargs=2, metavar=("LO", "HI"), default=None,
        help="probe RD <LO>..RD <HI> inclusive (bit/coil-style)",
    )
    asyncio.run(main(p.parse_args()))
