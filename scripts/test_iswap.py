"""Direct pylabrobot test for the Hamilton iSWAP (SCARA plate-transport arm).

The iSWAP is the optional robotic arm mounted on the back rail of a
STAR — distinct from the CoRe II gripper (which uses the pipetting
channels). This script starts with pure queries (zero motion) and
only performs motion on opt-in stages.

Stages:

  stage 1  query-only: iSWAP version + init + parking status
  stage 2  initialize + park (homes the arm to known positions)
  stage 3  open gripper fingers + query arm position

``iswap_close_gripper`` is intentionally NOT exercised here: it's a
plate-grasping primitive (firmware requires ``plate_width > 76.5mm``
and assumes the fingers are already positioned around a plate), so
calling it in free air is meaningless and the firmware rejects it.
To actually grip a plate, use the full ``iswap_get_plate`` sequence
via the action server's ``MoveResource`` action.

Each stage also runs all earlier stages. A ``[y/N]`` prompt gates
each physical motion unless ``--yes`` is passed.

Run with:

    pixi run test-iswap 1               # query only
    pixi run test-iswap 2               # + initialize / park
    pixi run test-iswap 3 --yes         # full cycle, no prompts
"""

from __future__ import annotations

import argparse
import asyncio
import sys

from pylabrobot.liquid_handling import LiquidHandler
from pylabrobot.liquid_handling.backends import STARBackend
from pylabrobot.resources.hamilton import STARDeck


_SKIP_CONFIRM = False


def _confirm(prompt: str) -> None:
    if _SKIP_CONFIRM:
        print(f"\n{prompt} [auto-yes]")
        return
    ans = input(f"\n{prompt} [y/N] ").strip().lower()
    if ans not in ("y", "yes"):
        print("aborting.")
        sys.exit(1)


async def stage_query(backend: STARBackend) -> None:
    print("== stage 1: iSWAP status queries ==")
    if not backend.extended_conf.left_x_drive.iswap_installed:
        raise RuntimeError("iSWAP is not installed on this STAR")
    version = await backend.request_iswap_version()
    print(f"  iswap_version       : {version}")
    initialized = await backend.request_iswap_initialization_status()
    print(f"  iswap_initialized   : {initialized}")
    # Firmware returns {'rg': 0|1, 'id': ...}; rg=1 means parked.
    parked_resp = await backend.request_iswap_in_parking_position()
    print(f"  iswap_in_parking    : {bool(parked_resp.get('rg'))}")


async def stage_init_park(backend: STARBackend) -> None:
    print("\n== stage 2: initialize + park iSWAP ==")
    _confirm("Initialize iSWAP (homes the arm — large motion)?")
    await backend.initialize_iswap()
    print("  initialize_iswap ok")

    _confirm("Park iSWAP?")
    await backend.park_iswap()
    print("  park_iswap ok")

    parked_resp = await backend.request_iswap_in_parking_position()
    print(f"  iswap_in_parking    : {bool(parked_resp.get('rg'))}")


async def stage_gripper(backend: STARBackend) -> None:
    print("\n== stage 3: open gripper + query arm position ==")
    _confirm("Open gripper fingers?")
    await backend.iswap_open_gripper()
    print("  iswap_open_gripper ok")

    pos = await backend.request_iswap_position()
    print(f"  iswap_position (grip centre): x={pos.x:.1f} y={pos.y:.1f} z={pos.z:.1f}")


async def main(stage: int) -> None:
    backend = STARBackend()
    lh = LiquidHandler(backend=backend, deck=STARDeck())
    await lh.setup()
    try:
        await stage_query(backend)
        if stage >= 2:
            await stage_init_park(backend)
        if stage >= 3:
            await stage_gripper(backend)
        print("\nok.")
    finally:
        await lh.stop()


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument(
        "stage", type=int, choices=[1, 2, 3],
        help="1=query, 2=initialize+park, 3=+gripper open/close",
    )
    p.add_argument(
        "--yes", action="store_true", help="skip all confirmation prompts",
    )
    args = p.parse_args()
    _SKIP_CONFIRM = args.yes
    asyncio.run(main(args.stage))
