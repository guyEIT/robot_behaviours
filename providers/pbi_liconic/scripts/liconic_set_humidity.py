"""Read or set the Liconic STX44 humidity setpoint.

No arguments → prints the current target + actual humidity, no writes.
An argument → confirms and writes the new setpoint via WR DM893.

Accepts the target in either percent (``60``) or fraction (``0.6``).
Values of 1.0 or less are treated as fractions; anything above 1.0 as
percent. Climate-controlled models only — *_NC variants don't support
humidity.

Run with:

    pixi run liconic-humidity                     # query only
    pixi run liconic-humidity 60                  # set 60 %
    pixi run liconic-humidity 0.6                 # set 60 %
    pixi run liconic-humidity 70 --yes            # no prompt
    pixi run liconic-humidity --model STX44_HR 80
"""

from __future__ import annotations

import argparse
import asyncio

from liconic_common import (
    DEFAULT_MODEL,
    DEFAULT_PORT,
    build_backend,
    make_confirm,
)


def _parse_humidity(value: float) -> float:
    """Accept percent (>1) or fraction (≤1); return a fraction in [0,1]."""
    if value < 0:
        raise SystemExit(f"humidity must be non-negative, got {value}")
    fraction = value if value <= 1.0 else value / 100.0
    if fraction > 1.0:
        raise SystemExit(f"humidity {value} out of range (max 100%)")
    return fraction


async def main(args: argparse.Namespace) -> None:
    if args.model.endswith("_NC"):
        raise SystemExit(
            f"model {args.model} has no climate control — humidity unavailable"
        )

    backend = build_backend(args.port, args.model)
    print(f"opening {args.port} ...")
    await backend.setup()
    try:
        target_before = await backend.get_target_humidity()
        actual = await backend.get_humidity()
        print(f"  target humidity : {target_before * 100:.1f} %")
        print(f"  actual humidity : {actual * 100:.1f} %")

        if args.humidity is None:
            return

        new_fraction = _parse_humidity(args.humidity)
        confirm = make_confirm(args.yes)
        confirm(
            f"Set target humidity from {target_before * 100:.1f} % "
            f"to {new_fraction * 100:.1f} %?"
        )
        await backend.set_humidity(new_fraction)
        target_after = await backend.get_target_humidity()
        print(f"\nok — target humidity now {target_after * 100:.1f} %")
        print("  (actual humidity will drift toward the setpoint — takes minutes)")
    finally:
        await backend.stop()


if __name__ == "__main__":
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument(
        "humidity", type=float, nargs="?",
        help="new setpoint — percent (e.g. 60) or fraction (e.g. 0.6). "
             "Omit to just print current values.",
    )
    p.add_argument("--port", default=DEFAULT_PORT,
                   help=f"serial port (default: {DEFAULT_PORT})")
    p.add_argument("--model", default=DEFAULT_MODEL,
                   help=f"Liconic model (default: {DEFAULT_MODEL})")
    p.add_argument("--yes", action="store_true",
                   help="skip the [y/N] confirm prompt")
    asyncio.run(main(p.parse_args()))
