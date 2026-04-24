"""Pick up a 6-well plate from a plate carrier via the iSWAP.

Assumes you've physically placed a ``Cor_Cos_6_wellplate_16800ul_Fb``
(Corning Costar 6-well, 16.8 mL, flat-bottom) in site 0 ("position 1",
1-indexed) of a ``PLT_CAR_L5MD_A00`` plate carrier. Override with
``--carrier`` / ``--plate`` / ``--rails`` if your setup differs.

Staged so you can stop after any step:

  stage 1  build deck, initialize iSWAP, print layout — no plate motion
  stage 2  move the plate from site 0 → site 1 (iSWAP picks, carries,
           places one site back)
  stage 3  one cycle (0 → 1 → 0), or a soak loop with ``--cycles N``

Run with:

    pixi run test-iswap-grab 1
    pixi run test-iswap-grab 2
    pixi run test-iswap-grab 3 --yes            # one cycle, no prompts
    pixi run test-iswap-grab 3 --cycles 20      # 20 cycles of 0↔1
    pixi run test-iswap-grab 3 --cycles 0       # soak until Ctrl+C

If your carrier isn't L5MD or your rails aren't 21, pass explicit
overrides:

    pixi run test-iswap-grab 2 --carrier PLT_CAR_L5AC_A00 --rails 15
"""

from __future__ import annotations

import argparse
import asyncio
import sys
import time
from typing import Any, Optional

from pylabrobot.liquid_handling import LiquidHandler
from pylabrobot.liquid_handling.backends import STARBackend
from pylabrobot.resources import Coordinate
from pylabrobot.resources.hamilton import STARDeck
from pylabrobot.resources.hamilton import plate_carriers as _carriers
from pylabrobot.resources import corning as _corning
from pylabrobot.resources import nest as _nest


_SKIP_CONFIRM = False


def _confirm(prompt: str) -> None:
    if _SKIP_CONFIRM:
        print(f"\n{prompt} [auto-yes]")
        return
    ans = input(f"\n{prompt} [y/N] ").strip().lower()
    if ans not in ("y", "yes"):
        print("aborting.")
        sys.exit(1)


def _resolve_plate_class(name: str) -> Any:
    """Look up a plate definition by symbolic name in corning / nest."""
    for mod in (_corning, _nest):
        if hasattr(mod, name):
            return getattr(mod, name)
    raise ValueError(
        f"unknown plate '{name}'. Try Cor_Cos_6_wellplate_16800ul_Fb "
        f"or another symbol from pylabrobot.resources.corning / .nest."
    )


def _resolve_carrier_class(name: str) -> Any:
    if not hasattr(_carriers, name):
        raise ValueError(
            f"unknown carrier '{name}'. Examples: PLT_CAR_L5MD_A00, "
            f"PLT_CAR_L5AC_A00, PLT_CAR_L5PCR_A00."
        )
    return getattr(_carriers, name)


def _build_lh(args: argparse.Namespace) -> LiquidHandler:
    backend = STARBackend()
    deck = STARDeck()
    carrier_cls = _resolve_carrier_class(args.carrier)
    plate_cls = _resolve_plate_class(args.plate)

    carrier = carrier_cls("plate_carrier_01")
    plate = plate_cls("six_well_01")
    # site 0 == "position 1" (1-indexed) — the slot the user placed the plate in
    carrier[0] = plate
    deck.assign_child_resource(carrier, rails=args.rails)
    return LiquidHandler(backend=backend, deck=deck)


async def stage_init(lh: LiquidHandler) -> None:
    print("== stage 1: deck + iSWAP init ==")
    for res in lh.deck.get_all_resources():
        print(f"  {res.name:<24} ({res.category})")
    backend: STARBackend = lh.backend  # type: ignore[assignment]
    initialized = await backend.request_iswap_initialization_status()
    if not initialized:
        _confirm("iSWAP not initialized — run initialize_iswap (homes the arm)?")
        await backend.initialize_iswap()
        print("  initialize_iswap ok")
    parked = await backend.request_iswap_in_parking_position()
    if not parked.get("rg"):
        _confirm("iSWAP not parked — park it first?")
        await backend.park_iswap()
        print("  park_iswap ok")


async def _do_move(
    lh: LiquidHandler,
    src_site: int,
    dst_site: int,
    pickup_distance_from_top: float,
    open_gripper_position: Optional[float] = None,
    xy_offset: tuple[float, float] = (0.0, 0.0),
) -> None:
    carrier = lh.deck.get_resource("plate_carrier_01")
    src = carrier.sites[src_site].resource
    if src is None:
        raise RuntimeError(
            f"no resource at site {src_site} — is the plate actually there?"
        )
    dst = carrier.sites[dst_site]
    dx, dy = xy_offset
    kwargs: dict[str, Any] = {
        "pickup_distance_from_top": pickup_distance_from_top,
        "pickup_offset": Coordinate(x=dx, y=dy, z=0.0),
        "destination_offset": Coordinate(x=dx, y=dy, z=0.0),
    }
    if open_gripper_position is not None:
        kwargs["open_gripper_position"] = open_gripper_position
    await lh.move_resource(src, dst, **kwargs)


async def stage_move(
    lh: LiquidHandler,
    src_site: int,
    dst_site: int,
    pickup_distance_from_top: float,
    open_gripper_position: Optional[float] = None,
    xy_offset: tuple[float, float] = (0.0, 0.0),
) -> None:
    print(f"\n== stage: move plate site {src_site} → site {dst_site} ==")
    carrier = lh.deck.get_resource("plate_carrier_01")
    src = carrier.sites[src_site].resource
    if src is None:
        raise RuntimeError(
            f"no resource at site {src_site} — is the plate actually there?"
        )
    ogp_note = (
        f"; open to {open_gripper_position:.1f} mm"
        if open_gripper_position is not None else "; open=default(plate_width+5)"
    )
    dx, dy = xy_offset
    off_note = f"; xy offset ({dx:+.2f}, {dy:+.2f}) mm" if (dx or dy) else ""
    _confirm(
        f"Pick up '{src.name}' from site {src_site} and place it on site "
        f"{dst_site}? (grip {pickup_distance_from_top:.1f} mm below plate top"
        f"{ogp_note}{off_note}; iSWAP traverses {abs(dst_site - src_site)} sites)"
    )
    await _do_move(
        lh, src_site, dst_site, pickup_distance_from_top,
        open_gripper_position, xy_offset,
    )
    print(f"  move ok — plate now on site {dst_site}")


async def stage_soak(
    lh: LiquidHandler,
    cycles: int,
    pickup_distance_from_top: float,
    open_gripper_position: Optional[float] = None,
    xy_offset: tuple[float, float] = (0.0, 0.0),
) -> None:
    """Repeatedly move the plate 0 ↔ 1. cycles=0 means until Ctrl+C.

    One cycle = two moves (0→1 and 1→0), so the plate ends where it started.
    """
    infinite = cycles <= 0
    label = "until Ctrl+C" if infinite else f"{cycles} cycles"
    print(f"\n== soak: plate 0 ↔ 1, {label} ==")
    _confirm(
        f"Run soak test of 0↔1 moves ({label})? The plate must start at site 0."
    )

    cycle = 0
    worst: float = 0.0
    total: float = 0.0
    start_wall = time.monotonic()
    try:
        while infinite or cycle < cycles:
            cycle += 1
            t0 = time.monotonic()
            await _do_move(lh, 0, 1, pickup_distance_from_top, open_gripper_position, xy_offset)
            await _do_move(lh, 1, 0, pickup_distance_from_top, open_gripper_position, xy_offset)
            dt = time.monotonic() - t0
            total += dt
            worst = max(worst, dt)
            tag = f"{cycle}" if infinite else f"{cycle}/{cycles}"
            print(f"  cycle {tag} ok ({dt:.1f}s, worst {worst:.1f}s, avg {total/cycle:.1f}s)")
    except KeyboardInterrupt:
        elapsed = time.monotonic() - start_wall
        print(f"\n  interrupted after {cycle} cycles, {elapsed:.1f}s total")
        # Make sure the plate ends up at site 0 before we stop. If the
        # interrupt arrived between the two halves of a cycle, the plate
        # is on site 1 and needs to come back.
        carrier = lh.deck.get_resource("plate_carrier_01")
        if carrier.sites[1].resource is not None:
            print("  plate left on site 1 — returning to site 0…")
            try:
                await _do_move(
                    lh, 1, 0, pickup_distance_from_top,
                    open_gripper_position, xy_offset,
                )
                print("  returned to site 0")
            except Exception as exc:  # noqa: BLE001
                print(f"  !! recovery move failed: {exc}")
        raise
    print(f"  soak ok — {cycle} cycles, total {total:.1f}s, worst {worst:.1f}s")


async def main(args: argparse.Namespace) -> None:
    lh = _build_lh(args)
    await lh.setup()
    try:
        await stage_init(lh)
        xy = (args.pickup_offset_x, args.pickup_offset_y)
        if args.stage == 2:
            await stage_move(
                lh, src_site=0, dst_site=1,
                pickup_distance_from_top=args.pickup_distance_from_top,
                open_gripper_position=args.open_gripper_position,
                xy_offset=xy,
            )
        elif args.stage == 3:
            if args.cycles == 1:
                await stage_move(
                    lh, src_site=0, dst_site=1,
                    pickup_distance_from_top=args.pickup_distance_from_top,
                    open_gripper_position=args.open_gripper_position,
                    xy_offset=xy,
                )
                await stage_move(
                    lh, src_site=1, dst_site=0,
                    pickup_distance_from_top=args.pickup_distance_from_top,
                    open_gripper_position=args.open_gripper_position,
                    xy_offset=xy,
                )
            else:
                await stage_soak(
                    lh, cycles=args.cycles,
                    pickup_distance_from_top=args.pickup_distance_from_top,
                    open_gripper_position=args.open_gripper_position,
                    xy_offset=xy,
                )
        print("\nok.")
    finally:
        await lh.stop()


if __name__ == "__main__":
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument(
        "stage", type=int, choices=[1, 2, 3],
        help="1=init only, 2=pick site 0→1, 3=+ place back 1→0",
    )
    p.add_argument(
        "--carrier", default="PLT_CAR_L5MD_A00",
        help="plate carrier class name (default: PLT_CAR_L5MD_A00)",
    )
    p.add_argument(
        "--plate", default="Cor_Cos_6_wellplate_16800ul_Fb",
        help="plate class name (default: Cor_Cos_6_wellplate_16800ul_Fb)",
    )
    p.add_argument(
        "--rails", type=int, default=1,
        help="carrier rails position (default: 1)",
    )
    p.add_argument(
        "--pickup-distance-from-top", type=float, default=18.0,
        help=(
            "mm below the plate top where the gripper closes (default: 15.0). "
            "Raise this to skip a lid and grip the plate body's flange; 0 "
            "means grip at the top rim."
        ),
    )
    p.add_argument(
        "--cycles", type=int, default=1,
        help=(
            "stage 3 only: number of full 0↔1 cycles to run. Default 1 = "
            "one cycle with per-move confirmation. >1 = soak loop, single "
            "confirm at start. 0 = loop until Ctrl+C (recovery returns the "
            "plate to site 0 if interrupted mid-cycle)."
        ),
    )
    p.add_argument(
        "--open-gripper-position", type=float, default=None,
        help=(
            "mm to open the gripper fingers to for approach and release. "
            "Default None → pylabrobot computes plate_width+5 (≈132.8 mm "
            "for a 6-well). The iSWAP's physical max is just above 132 mm; "
            "values >132 are rejected by firmware. If the fingers catch, "
            "use --pickup-offset-x instead of trying to open wider."
        ),
    )
    p.add_argument(
        "--pickup-offset-x", type=float, default=0.0,
        help=(
            "mm to offset the gripper in X at both pickup and place "
            "(default: 0.0). Useful when one finger is catching on a "
            "flange — shift +0.5 to +1.0 mm to bias away from the catching "
            "side. Applied identically to source and destination so a "
            "symmetric carrier still lines up."
        ),
    )
    p.add_argument(
        "--pickup-offset-y", type=float, default=0.0,
        help="mm to offset the gripper in Y at pickup and place (default: 0.0).",
    )
    p.add_argument(
        "--yes", action="store_true", help="skip all confirmation prompts",
    )
    args = p.parse_args()
    _SKIP_CONFIRM = args.yes
    asyncio.run(main(args))
