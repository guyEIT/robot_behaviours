"""Recovery script: release a plate stuck in the iSWAP gripper.

When the STAR errors out mid-move, the iSWAP can be left holding a
plate. The next ``lh.setup()`` then fails during its park_iswap call
with ``ElementStillHoldingError('Unexpected object found')`` because
firmware refuses to park while something is in the fingers.

This script bypasses that by calling ``lh.setup(skip_iswap=True)`` to
skip the park call during initialization, then manually opens the
gripper to clear the holding state, and finally parks the arm.

Run with:
    pixi run iswap-release

Safety:
- The plate WILL fall as soon as the gripper opens. Before confirming,
  either:
    (a) have a hand under the plate ready to catch it, or
    (b) ensure the plate is already resting on / above a carrier site
        so a short drop is harmless.
- If the arm is parked far from anywhere useful, jog it first via
  iswap_teach.py (connects with the same skip flag as a fallback).
"""

from __future__ import annotations

import argparse
import asyncio
import sys

from pylabrobot.liquid_handling import LiquidHandler
from pylabrobot.liquid_handling.backends import STARBackend
from pylabrobot.resources.hamilton import STARDeck


def _confirm(prompt: str, skip: bool) -> None:
    if skip:
        print(f"{prompt} [auto-yes]")
        return
    ans = input(f"{prompt} [y/N] ").strip().lower()
    if ans not in ("y", "yes"):
        print("aborting.")
        sys.exit(1)


async def main(args: argparse.Namespace) -> None:
    backend = STARBackend()
    lh = LiquidHandler(backend=backend, deck=STARDeck())

    # skip_iswap=True avoids the setup-time park_iswap that fails when
    # the gripper is still holding a plate. The USB connection and the
    # other modules still come up normally.
    print("connecting to STAR with skip_iswap=True…")
    await lh.setup(skip_iswap=True)
    try:
        parked_resp = await backend.request_iswap_in_parking_position()
        is_parked = bool(parked_resp.get("rg"))
        print(f"iSWAP parked: {is_parked}")

        pos = await backend.request_iswap_position()
        print(f"iSWAP position: x={pos.x:.1f} y={pos.y:.1f} z={pos.z:.1f} mm")

        _confirm(
            "\nOpen iSWAP gripper now? The plate WILL fall — make sure a "
            "hand is under it or it's resting on a safe surface.",
            skip=args.yes,
        )
        await backend.iswap_open_gripper()
        print("gripper opened — plate released")

        _confirm("\nPark iSWAP?", skip=args.yes)
        await backend.park_iswap()
        print("iSWAP parked")
    finally:
        await lh.stop()


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--yes", action="store_true", help="skip all confirmation prompts")
    args = p.parse_args()
    asyncio.run(main(args))
