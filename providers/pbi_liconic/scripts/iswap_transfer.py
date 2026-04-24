"""Shared iSWAP transfer helpers for moving a 6-well plate between a
plate carrier on the STAR deck and the off-deck ``incubator_handoff``
saved position.

Motion rules (learned from end-stop crashes):
  - Every shoulder rotation happens at the MIDDLE of the iSWAP Y range
    (``safe_y ≈ 450 mm``). Rotating at a forward Y (e.g. plate Y=114)
    drives the arm into the front end stop; rotating at a far-back Y
    (e.g. post-init Y=626) drives it into the back wall with
    ``R072 Rotation-drive movement error``. Middle Y clears both.
  - Before any big move (rotation or long X translation), the iSWAP is
    parked at ``safe_y`` — moving forward from a back Y, or backward
    from a forward Y, whichever is needed.
  - ``position_components_for_free_iswap_y_range`` (C0 FY) is called
    once at session start so the pipetting channels don't block iSWAP
    Y approaches to plate sites.
  - Descents from ``traverse_z`` approach each endpoint from above.

The same primitives the teach pendant jogs with: ``move_iswap_x/y/z``,
``rotate_iswap_rotation_drive``, ``rotate_iswap_wrist``,
``iswap_open_gripper``, ``iswap_close_gripper``. No firmware atomics —
those enforce the on-deck workspace and refuse X=-216.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Any

from pylabrobot.liquid_handling import LiquidHandler
from pylabrobot.liquid_handling.backends import STARBackend
from pylabrobot.resources import corning as _corning
from pylabrobot.resources.hamilton import STARDeck
from pylabrobot.resources.hamilton import plate_carriers as _carriers


CALIBRATIONS_FILE = Path(__file__).resolve().parent / "iswap_calibrations.json"

# 6-well grip height below the plate rim. Memory note:
# "6-well plate grip needs pickup_distance_from_top ≈ 15-18mm".
DEFAULT_PICKUP_DISTANCE_FROM_TOP = 18.0

# Open width for approach (wider than plate_width so fingers clear the
# plate's external footprint on descent).
APPROACH_OPEN_EXTRA = 10.0

# Firmware pre-close rule: open must be at plate_width + tolerance + 2.
PLATE_WIDTH_TOLERANCE = 2.0

# Open width for RELEASE. Must exceed the plate's actual outer footprint,
# which is larger than plate_width when plate_width is a squeezing grip
# (e.g. plate_width=80 on an ~85 mm plate). Using the pre-close formula
# (plate_width + 4) leaves the fingers still pinching the plate.
RELEASE_OPEN_EXTRA = 15.0

# Safe Z to traverse at between endpoints.
TRAVERSE_Z = 280.0

# "Safe middle" Y — the iSWAP Y at which shoulder rotations have clearance
# on BOTH sides. Too forward (e.g. Y=114 near plates) and the sweep hits
# the front end stop; too back (e.g. Y=626 post-init) and it hits the
# back wall with ``R072 Rotation-drive movement error: drive locked``.
# Picked empirically; adjust via --safe-y if your machine binds.
SAFE_Y = 450.0


def load_incubator_handoff() -> dict[str, Any]:
    if not CALIBRATIONS_FILE.exists():
        raise FileNotFoundError(
            f"no calibration file at {CALIBRATIONS_FILE}. Run `pixi run iswap-teach` first."
        )
    data = json.loads(CALIBRATIONS_FILE.read_text())
    if "incubator_handoff" not in data:
        raise KeyError(
            "'incubator_handoff' not in calibrations — re-save it from iswap_teach."
        )
    return data["incubator_handoff"]


def build_lh_with_carrier(rails: int, plate_on_site_0: bool = True):
    """Deck with a PLT_CAR_L5MD_A00 at ``rails``. If ``plate_on_site_0``
    the carrier has a 6-well on site 0 (for grip-coord computation)."""
    backend = STARBackend()
    deck = STARDeck()
    carrier = _carriers.PLT_CAR_L5MD_A00("plate_carrier_01")
    if plate_on_site_0:
        carrier[0] = _corning.Cor_Cos_6_wellplate_16800ul_Fb("six_well_01")
    deck.assign_child_resource(carrier, rails=rails)
    lh = LiquidHandler(backend=backend, deck=deck)
    return lh, carrier


def compute_carrier_grip_xyz(
    carrier: Any, pickup_distance_from_top: float
) -> tuple[float, float, float]:
    """iSWAP gripper XYZ to grip the plate on site 0 of ``carrier``."""
    site = carrier.sites[0]
    plate = site.resource
    if plate is None:
        plate = _corning.Cor_Cos_6_wellplate_16800ul_Fb("_tmp_geom")
        site.assign_child_resource(plate)
    center = plate.get_absolute_location(x="c", y="c", z="t")
    return center.x, center.y, center.z - pickup_distance_from_top


async def ensure_iswap_initialized(backend: STARBackend) -> None:
    initialized = await backend.request_iswap_initialization_status()
    parked_resp = await backend.request_iswap_in_parking_position()
    is_parked = bool(parked_resp.get("rg"))
    if not initialized:
        print("  iSWAP not initialized — running initialize_iswap…")
        await backend.initialize_iswap()
    elif is_parked:
        print("  iSWAP parked — running initialize_iswap to unpark…")
        await backend.initialize_iswap()
    else:
        print("  iSWAP initialized and unparked")


async def free_iswap_y_range(backend: STARBackend) -> None:
    """Move the pipetting channels out of the iSWAP's Y path. Without
    this, ``move_iswap_y`` to any Y below the backmost channel's Y is
    blocked by pylabrobot's collision check (channel 0 sits in front of
    the iSWAP and blocks the Y approach to on-deck plate sites)."""
    print("  positioning channels for free iSWAP Y range (C0 FY)…")
    await backend.position_components_for_free_iswap_y_range()


async def _lift_to_traverse(backend: STARBackend, traverse_z: float) -> None:
    pos = await backend.request_iswap_position()
    if pos.z < traverse_z - 0.5:
        await backend.move_iswap_z(traverse_z)


async def _park_at_safe_y(backend: STARBackend, safe_y: float) -> None:
    """Move the iSWAP Y to the safe middle ``safe_y`` — park there
    before every shoulder rotation and every long X translation. Moves
    in either direction (backwards from a forward Y like the plate site,
    or forwards from the post-init Y=626 back pose)."""
    pos = await backend.request_iswap_position()
    if abs(pos.y - safe_y) > 0.5:
        await backend.move_iswap_y(safe_y)


async def _orient(
    backend: STARBackend, rotation: str, wrist: str = "STRAIGHT"
) -> None:
    """Shoulder + wrist rotation. Caller must ensure Y is at ``safe_y``
    before this."""
    await backend.rotate_iswap_rotation_drive(
        backend.RotationDriveOrientation[rotation]
    )
    await backend.rotate_iswap_wrist(backend.WristDriveOrientation[wrist])


async def _log_iswap_pos(backend: STARBackend, tag: str) -> None:
    try:
        p = await backend.request_iswap_position()
        print(f"  [debug] iSWAP @ {tag}: "
              f"x={p.x:.2f} y={p.y:.2f} z={p.z:.2f}")
    except Exception as exc:  # noqa: BLE001
        print(f"  [debug] iSWAP @ {tag}: position read failed: {exc}")


async def transfer(
    backend: STARBackend,
    pickup_xyz: tuple[float, float, float],
    release_xyz: tuple[float, float, float],
    plate_width: float,
    traverse_z: float,
    safe_y: float,
    confirm,
    pickup_offset: tuple[float, float, float] = (0.0, 0.0, 0.0),
    release_offset: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> None:
    """One-way transfer: ``pickup_xyz → release_xyz`` with LEFT grip.

    Every shoulder rotation happens at ``safe_y``. Every long X move
    also happens at ``safe_y`` (and at ``traverse_z``) so the arm is
    retracted during transit.

    ``pickup_offset`` / ``release_offset`` are added to the endpoint XYZ
    for per-run tuning without re-teaching the saved calibration.
    """
    px = pickup_xyz[0] + pickup_offset[0]
    py = pickup_xyz[1] + pickup_offset[1]
    pz = pickup_xyz[2] + pickup_offset[2]
    rx = release_xyz[0] + release_offset[0]
    ry = release_xyz[1] + release_offset[1]
    rz = release_xyz[2] + release_offset[2]

    # Stage 1: retreat Y, then rotate to LEFT, then lift and open.
    await _log_iswap_pos(backend, "stage1.enter")
    confirm(
        f"Stage 1/6: park Y→{safe_y:.1f}, then rotate shoulder=LEFT, "
        f"wrist=STRAIGHT at safe Y?"
    )
    await _park_at_safe_y(backend, safe_y)
    await _orient(backend, rotation="LEFT", wrist="STRAIGHT")
    await _lift_to_traverse(backend, traverse_z)
    await backend.iswap_open_gripper(
        open_position=plate_width + APPROACH_OPEN_EXTRA
    )

    # Stage 2: travel X at safe_y, then advance Y to pickup.
    await _log_iswap_pos(backend, f"stage2.enter (about to move_iswap_x→{px:.2f})")
    confirm(
        f"Stage 2/6: translate X→{px:.2f} at Y={safe_y:.1f}, then advance "
        f"Y→{py:.2f} (all at Z={traverse_z:.1f})?"
    )
    await backend.move_iswap_x(px)
    await _log_iswap_pos(backend, f"stage2.post_x (about to move_iswap_y→{py:.2f})")
    await backend.move_iswap_y(py)

    # Stage 3: descend, close on plate, lift.
    confirm(
        f"Stage 3/6: descend to Z={pz:.2f}, close on plate_width="
        f"{plate_width:.1f}, lift to Z={traverse_z:.1f}?"
    )
    await backend.move_iswap_z(pz)
    await backend.iswap_close_gripper(
        plate_width=plate_width, plate_width_tolerance=PLATE_WIDTH_TOLERANCE,
    )
    await backend.move_iswap_z(traverse_z)

    # Stage 4: retreat Y, translate X across deck, advance Y to release.
    confirm(
        f"Stage 4/6: park Y→{safe_y:.1f}, translate X→{rx:.2f}, then "
        f"advance Y→{ry:.2f} (all at Z={traverse_z:.1f})?"
    )
    await _park_at_safe_y(backend, safe_y)
    await backend.move_iswap_x(rx)
    await backend.move_iswap_y(ry)

    # Stage 5: descend, open to release, lift, retreat Y.
    confirm(
        f"Stage 5/6: descend to Z={rz:.2f}, open gripper to release, "
        f"lift to Z={traverse_z:.1f}, park Y→{safe_y:.1f}?"
    )
    await backend.move_iswap_z(rz)
    await backend.iswap_open_gripper(
        open_position=plate_width + RELEASE_OPEN_EXTRA
    )
    await backend.move_iswap_z(traverse_z)
    await _park_at_safe_y(backend, safe_y)

    # Stage 6: rotate shoulder to FRONT at safe Y, park.
    confirm(
        f"Stage 6/6: rotate shoulder=FRONT at safe Y={safe_y:.1f}, "
        f"then park iSWAP?"
    )
    await _orient(backend, rotation="FRONT", wrist="STRAIGHT")
    await backend.park_iswap()
    print("  parked")


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
