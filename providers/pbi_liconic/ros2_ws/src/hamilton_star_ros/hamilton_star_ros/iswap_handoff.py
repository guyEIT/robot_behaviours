"""iSWAP handoff-transfer helpers — the manual-jog recipe that moves a
plate between an on-deck plate site and a calibrated off-deck handoff
position (e.g. an incubator loading dock at X ≈ -216 mm).

Ported from the standalone ``iswap_transfer.py`` at the repo root.
Machinery-only: no ROS, no argparse, no JSON — the calibration shape
comes from the caller (the action server, which resolves it from ROS
params at goal time).

Motion rules (learned from end-stop crashes):

- Every shoulder rotation happens at the MIDDLE of the iSWAP Y range
  (``safe_y ≈ 450 mm``). Rotating at a forward Y (e.g. plate Y=114)
  drives the arm into the front end stop; rotating at a far-back Y
  (e.g. post-init Y=626) crashes into the back wall with
  ``R072 Rotation-drive movement error``. Middle Y clears both.
- Before any big move (rotation or long X translation), the iSWAP is
  parked at ``safe_y`` — either moving forward from a back Y or
  backward from a forward Y.
- ``position_components_for_free_iswap_y_range`` (C0 FY) must be called
  before forward iSWAP Y moves to plate sites, or channel 0 blocks the
  approach with pylabrobot's pre-flight collision check. The action
  server calls this once per session (and again whenever a pipetting
  action invalidates the state).
- The firmware atomics ``iswap_get_plate`` / ``iswap_put_plate`` refuse
  off-deck X (~-216 mm) with ``PositionNotReachableError`` (R027). We
  use manual ``move_iswap_x/y/z`` jogs throughout — same primitives
  ``iswap_teach`` uses — which have no such limit.
- Descents from ``traverse_z`` approach each endpoint from above.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Any, Awaitable, Callable, Optional, Tuple

from pylabrobot.liquid_handling import LiquidHandler
from pylabrobot.liquid_handling.backends import STARBackend
from pylabrobot.resources import corning as _corning

_log = logging.getLogger(__name__)


# 6-well grip height below the plate rim (memory: 15-18 mm empirically).
DEFAULT_PICKUP_DISTANCE_FROM_TOP = 18.0

# Open width for APPROACH — wider than plate_width so fingers clear the
# plate's external footprint on descent.
APPROACH_OPEN_EXTRA = 10.0

# Firmware pre-close rule: open must be at plate_width + tolerance + 2.
PLATE_WIDTH_TOLERANCE = 2.0

# Open width for RELEASE. Must exceed the plate's outer footprint, which
# is larger than ``plate_width`` when plate_width is a squeezing grip
# (e.g. plate_width=80 on an ~85 mm plate). Using the pre-close formula
# (plate_width + 4) leaves the fingers still pinching the plate.
RELEASE_OPEN_EXTRA = 15.0

# Safe Z to traverse between endpoints (above any deck obstacle).
TRAVERSE_Z = 280.0

# Middle of the iSWAP Y range — where rotations have clearance both sides.
SAFE_Y = 450.0


# Stage labels published as action feedback. Keep in sync with the
# ``transfer`` state machine below.
STAGE_NAMES = (
    "park_y_rotate_open",     # 1
    "travel_to_pickup",       # 2
    "descend_close_lift",     # 3
    "travel_to_release",      # 4
    "descend_open_lift",      # 5
    "rotate_front_park",      # 6
)


@dataclass
class HandoffCalibration:
    """Shape the action server resolves a ``calibration_name`` to by
    reading ROS params ``handoff.<name>.{x,y,z,plate_width,rotation,wrist}``
    at goal time."""
    x: float
    y: float
    z: float
    plate_width: float
    rotation: str = "LEFT"
    wrist: str = "STRAIGHT"


def compute_grip_xyz_from_resource(
    lh: LiquidHandler,
    resource_name: str,
    pickup_distance_from_top: float,
) -> Tuple[float, float, float]:
    """iSWAP gripper XYZ to grip the plate named ``resource_name`` on the
    deck. Works for a plate resource directly OR a PlateHolder-ish site
    — we place a throwaway 6-well on an empty site to read its geometry
    and return the computed grip XYZ."""
    res = lh.deck.get_resource(resource_name)
    plate = _get_or_stage_plate(res)
    center = plate.get_absolute_location(x="c", y="c", z="t")
    return center.x, center.y, center.z - pickup_distance_from_top


def _get_or_stage_plate(res: Any):
    """Return the plate at ``res``. If ``res`` is itself a plate, return
    it. If it's a PlateHolder-style site with a child, return the child.
    If it's an empty site, stage a throwaway Corning 6-well so we can
    read geometry."""
    if hasattr(res, "get_size_x") and not hasattr(res, "sites"):
        # already a plate-like resource
        return res
    child = getattr(res, "resource", None)
    if child is not None:
        return child
    # empty holder — stage a placeholder 6-well for geometry
    placeholder = _corning.Cor_Cos_6_wellplate_16800ul_Fb("_tmp_geom")
    res.assign_child_resource(placeholder)
    return placeholder


async def ensure_iswap_initialized(backend: STARBackend) -> bool:
    """Initialize the iSWAP if not yet initialized or currently parked.
    Returns True if ``initialize_iswap`` was actually invoked (caller
    can use this to flip an FSM flag)."""
    initialized = await backend.request_iswap_initialization_status()
    parked_resp = await backend.request_iswap_in_parking_position()
    is_parked = bool(parked_resp.get("rg"))
    if not initialized or is_parked:
        await backend.initialize_iswap()
        return True
    return False


async def free_iswap_y_range(backend: STARBackend) -> None:
    """Position the pipetting channels out of the iSWAP's Y path
    (firmware ``C0 FY``). Without this, pylabrobot's pre-flight check
    blocks any iSWAP Y move that would cross channel 0's Y."""
    await backend.position_components_for_free_iswap_y_range()


async def _lift_to_traverse(backend: STARBackend, traverse_z: float) -> None:
    pos = await backend.request_iswap_position()
    if pos.z < traverse_z - 0.5:
        await backend.move_iswap_z(traverse_z)


async def _park_at_safe_y(backend: STARBackend, safe_y: float) -> None:
    """Move the iSWAP Y to ``safe_y`` — the clearance-both-sides middle.
    Moves forward or backward as needed."""
    pos = await backend.request_iswap_position()
    if abs(pos.y - safe_y) > 0.5:
        await backend.move_iswap_y(safe_y)


async def _orient(
    backend: STARBackend, rotation: str, wrist: str = "STRAIGHT"
) -> None:
    """Shoulder + wrist rotation. Caller must ensure Y is at ``safe_y``
    before invoking this."""
    await backend.rotate_iswap_rotation_drive(
        backend.RotationDriveOrientation[rotation]
    )
    await backend.rotate_iswap_wrist(backend.WristDriveOrientation[wrist])


StageCallback = Callable[[int, str], Optional[Awaitable[None]]]


async def _notify(on_stage: Optional[StageCallback], stage: int, name: str) -> None:
    if on_stage is None:
        return
    result = on_stage(stage, name)
    if result is not None:
        await result


async def transfer(
    backend: STARBackend,
    pickup_xyz: Tuple[float, float, float],
    release_xyz: Tuple[float, float, float],
    plate_width: float,
    traverse_z: float = TRAVERSE_Z,
    safe_y: float = SAFE_Y,
    rotation: str = "LEFT",
    wrist: str = "STRAIGHT",
    pickup_offset: Tuple[float, float, float] = (0.0, 0.0, 0.0),
    release_offset: Tuple[float, float, float] = (0.0, 0.0, 0.0),
    on_stage: Optional[StageCallback] = None,
) -> None:
    """One-way transfer: ``pickup_xyz → release_xyz``.

    All shoulder rotations happen at ``safe_y``. Long X translations
    happen at ``safe_y`` and ``traverse_z`` so the arm is retracted in
    transit. ``on_stage(stage, name)`` is called at the start of each
    of the six stages so the action server can publish feedback and
    toggle the "holding plate" FSM flag at stages 3 and 5.
    """
    px = pickup_xyz[0] + pickup_offset[0]
    py = pickup_xyz[1] + pickup_offset[1]
    pz = pickup_xyz[2] + pickup_offset[2]
    rx = release_xyz[0] + release_offset[0]
    ry = release_xyz[1] + release_offset[1]
    rz = release_xyz[2] + release_offset[2]

    async def _log_pos(tag: str) -> None:
        try:
            p = await backend.request_iswap_position()
            # INFO so it shows up in both the direct script's terminal and
            # the ROS 2 action server's ~/events pipeline without extra
            # configuration.
            _log.info(
                "iSWAP @ %s: x=%.2f y=%.2f z=%.2f", tag, p.x, p.y, p.z,
            )
            print(f"  [debug] iSWAP @ {tag}: "
                  f"x={p.x:.2f} y={p.y:.2f} z={p.z:.2f}")
        except Exception as exc:  # noqa: BLE001
            print(f"  [debug] iSWAP @ {tag}: position read failed: {exc}")

    # Stage 1: park Y at safe_y, rotate, lift Z, open for approach.
    await _notify(on_stage, 1, STAGE_NAMES[0])
    await _log_pos("stage1.enter")
    await _park_at_safe_y(backend, safe_y)
    await _orient(backend, rotation=rotation, wrist=wrist)
    await _lift_to_traverse(backend, traverse_z)
    await backend.iswap_open_gripper(
        open_position=plate_width + APPROACH_OPEN_EXTRA
    )

    # Stage 2: travel X at safe_y, then advance Y to pickup.
    await _notify(on_stage, 2, STAGE_NAMES[1])
    await _log_pos(f"stage2.enter (about to move_iswap_x→{px:.2f})")
    await backend.move_iswap_x(px)
    await _log_pos(f"stage2.post_x (about to move_iswap_y→{py:.2f})")
    await backend.move_iswap_y(py)

    # Stage 3: descend, close on plate, lift. Plate is now held.
    await _notify(on_stage, 3, STAGE_NAMES[2])
    await backend.move_iswap_z(pz)
    await backend.iswap_close_gripper(
        plate_width=plate_width, plate_width_tolerance=PLATE_WIDTH_TOLERANCE,
    )
    await backend.move_iswap_z(traverse_z)

    # Stage 4: park Y, translate X across deck, advance Y to release.
    await _notify(on_stage, 4, STAGE_NAMES[3])
    await _log_pos(f"stage4.enter (about to move_iswap_x→{rx:.2f})")
    await _park_at_safe_y(backend, safe_y)
    await backend.move_iswap_x(rx)
    await backend.move_iswap_y(ry)

    # Stage 5: descend, open to release, lift, park Y. Plate released.
    await _notify(on_stage, 5, STAGE_NAMES[4])
    await backend.move_iswap_z(rz)
    await backend.iswap_open_gripper(
        open_position=plate_width + RELEASE_OPEN_EXTRA
    )
    await backend.move_iswap_z(traverse_z)
    await _park_at_safe_y(backend, safe_y)

    # Stage 6: rotate to FRONT at safe_y, park.
    await _notify(on_stage, 6, STAGE_NAMES[5])
    await _orient(backend, rotation="FRONT", wrist="STRAIGHT")
    await backend.park_iswap()


def resolve_endpoints(
    lh: LiquidHandler,
    calibration: HandoffCalibration,
    on_deck_resource: str,
    direction: str,
    pickup_distance_from_top: float,
) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
    """Given the calibration and the on-deck plate site, return
    ``(pickup_xyz, release_xyz)`` oriented per ``direction``.

    ``direction="to_handoff"`` → pickup is the on-deck resource, release
    is the calibrated off-deck position.
    ``direction="from_handoff"`` → pickup is the calibrated off-deck
    position, release is the on-deck resource.
    """
    on_deck_xyz = compute_grip_xyz_from_resource(
        lh, on_deck_resource, pickup_distance_from_top,
    )
    handoff_xyz = (calibration.x, calibration.y, calibration.z)
    if direction == "to_handoff":
        return on_deck_xyz, handoff_xyz
    if direction == "from_handoff":
        return handoff_xyz, on_deck_xyz
    raise ValueError(
        f"unknown direction {direction!r}: expected 'to_handoff' or 'from_handoff'"
    )
