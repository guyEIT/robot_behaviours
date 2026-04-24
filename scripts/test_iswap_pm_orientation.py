"""Empirical check: does the Hamilton STAR's ``PM`` firmware command
(pylabrobot ``move_plate_to_position``) respect the current shoulder +
wrist orientation, or does it rotate to whatever ``grip_direction`` is
passed?

If PM is **declarative** (validates but doesn't rotate), we can use it
as a drop-in replacement for the chunked ``GX``/``GY`` calls in the
6-stage manual jog — one firmware command per long traverse, no 99.9 mm
stutter, orientation preserved. If PM is **imperative** (rotates to
match grip_direction), it's off the table for orientation-sensitive
handoffs like the left-side incubator dock.

This test:
  1. Connects to the STAR (USB — stop ros2 action server first).
  2. Parks the iSWAP at safe_y, lifts Z to traverse_z.
  3. Rotates shoulder = LEFT, wrist = STRAIGHT (same pre-orient the
     incubator handoff uses).
  4. Reads + prints shoulder / wrist / XYZ pose ("before" snapshot).
  5. Calls PM with a small ±50 mm X delta and grip_direction=4 (LEFT,
     MATCHING current orientation). Gripper is empty, stays at safe_y
     and traverse_z — safest possible pose.
  6. Reads + prints orientation again ("after" snapshot).
  7. Returns X to the starting value so the test is idempotent.

Prereqs:
  - Hamilton STAR connected and idle.
  - No plate in the iSWAP gripper.
  - No on-deck obstacle within ±50 mm of the iSWAP's current X at Y=safe
    and Z=traverse_z.
  - The ros2 hamilton_star_action_server is NOT running (``pkill -f
    hamilton_star_action_server``). This script opens its own USB
    connection.

Run with:
    pixi run test-iswap-pm
    pixi run test-iswap-pm --yes
    pixi run test-iswap-pm --delta 30
"""

from __future__ import annotations

import argparse
import asyncio
import sys

from pylabrobot.liquid_handling import LiquidHandler
from pylabrobot.liquid_handling.backends import STARBackend
from pylabrobot.resources.hamilton import STARDeck


SAFE_Y = 450.0      # middle iSWAP Y — rotation clearance both sides
TRAVERSE_Z = 280.0  # above any on-deck obstacle

# PM grip_direction firmware codes (move_plate_to_position):
#   1 = -Y  (FRONT), 2 = +X (RIGHT), 3 = +Y (BACK), 4 = -X (LEFT)
PM_GRIP_LEFT = 4


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


async def _read_orientation_safe(fn, raw_increments_fn):
    """Return the orientation enum if pylabrobot can classify it, else the
    raw encoder value. Some STARs return encoder counts slightly outside
    pylabrobot's hard-coded enum ranges (library bug), which raises
    ValueError — we fall back to the raw count so the test still runs."""
    try:
        return ("enum", await fn())
    except ValueError:
        try:
            return ("raw", await raw_increments_fn())
        except Exception as exc:  # noqa: BLE001
            return ("err", f"{type(exc).__name__}: {exc}")


def _fmt_orientation(tag_value) -> str:
    tag, val = tag_value
    if tag == "enum":
        return val.name
    if tag == "raw":
        return f"raw_enc={val}"
    return f"<{val}>"


async def _snapshot(backend: STARBackend, label: str):
    pos = await backend.request_iswap_position()
    rot = await _read_orientation_safe(
        backend.request_iswap_rotation_drive_orientation,
        backend.request_iswap_rotation_drive_position_increments,
    )
    wrist = await _read_orientation_safe(
        backend.request_iswap_wrist_drive_orientation,
        backend.request_iswap_wrist_drive_position_increments,
    )
    print(f"  [{label}] "
          f"x={pos.x:7.2f} y={pos.y:7.2f} z={pos.z:7.2f}  "
          f"shoulder={_fmt_orientation(rot):<14} wrist={_fmt_orientation(wrist)}")
    return pos, rot, wrist


async def _call_pm(
    backend: STARBackend,
    target_x: float,
    target_y: float,
    target_z: float,
    grip_direction: int,
) -> None:
    await backend.move_plate_to_position(
        x_position=round(abs(target_x) * 10),
        x_direction=0 if target_x >= 0 else 1,
        y_position=round(abs(target_y) * 10),
        y_direction=0 if target_y >= 0 else 1,
        z_position=round(target_z * 10),
        z_direction=0,
        grip_direction=grip_direction,
        minimum_traverse_height_at_beginning_of_a_command=round(TRAVERSE_Z * 10),
        collision_control_level=1,
    )


async def main(args: argparse.Namespace) -> None:
    confirm = make_confirm(args.yes)
    backend = STARBackend()
    lh = LiquidHandler(backend=backend, deck=STARDeck())

    print("connecting to STAR ...")
    await lh.setup()
    try:
        # iSWAP init if needed
        initialized = await backend.request_iswap_initialization_status()
        if not initialized:
            confirm("iSWAP is not initialized — initialize now (will home)?")
            await backend.initialize_iswap()

        # Free Y-path so pipetting channels don't block the iSWAP
        await backend.position_components_for_free_iswap_y_range()

        # Park at safe_y, lift to traverse_z, via the legacy GX/GY
        # primitives so this setup doesn't depend on PM behaviour.
        print("\nparking iSWAP at safe_y, traverse_z ...")
        pos = await backend.request_iswap_position()
        if abs(pos.y - SAFE_Y) > 0.5:
            confirm(f"Move iSWAP Y from {pos.y:.1f} to safe_y={SAFE_Y}?")
            await backend.move_iswap_y(SAFE_Y)
        if pos.z < TRAVERSE_Z - 0.5:
            await backend.move_iswap_z(TRAVERSE_Z)

        # Orient: shoulder LEFT + wrist STRAIGHT (the incubator-dock combo)
        confirm("Rotate shoulder=LEFT, wrist=STRAIGHT?")
        await backend.rotate_iswap_rotation_drive(
            backend.RotationDriveOrientation.LEFT
        )
        await backend.rotate_iswap_wrist(
            backend.WristDriveOrientation.STRAIGHT
        )

        # Snapshot BEFORE
        print("\n== before PM ==")
        pos0, rot0, wrist0 = await _snapshot(backend, "before")

        target_x = pos0.x - args.delta  # move X left by args.delta
        print(f"\n== PM call ==")
        print(f"  target         : x={target_x:.2f} y={SAFE_Y:.2f} z={TRAVERSE_Z:.2f}")
        print(f"  grip_direction : {PM_GRIP_LEFT} (LEFT — MATCHES current shoulder)")
        print(f"  delta_x        : {-args.delta:+.1f} mm")
        print("\n  The gripper is empty, at safe Y and traverse Z. The test")
        print("  is whether the shoulder/wrist rotate during the PM call.")
        print("  WATCH THE ARM.")
        confirm("Call PM?")
        await _call_pm(backend, target_x, SAFE_Y, TRAVERSE_Z, PM_GRIP_LEFT)

        # Snapshot AFTER
        print("\n== after PM ==")
        pos1, rot1, wrist1 = await _snapshot(backend, "after ")

        # Return X to starting pose so the bench is idempotent
        print(f"\nreturning X to {pos0.x:.2f} ...")
        await _call_pm(backend, pos0.x, SAFE_Y, TRAVERSE_Z, PM_GRIP_LEFT)
        print("\n== after return ==")
        await _snapshot(backend, "return")

        # Verdict — tolerate raw-encoder form if pylabrobot can't classify.
        # A real shoulder rotation shifts the encoder by tens of thousands
        # of counts; a translation leaves it within noise (~a few counts).
        ENC_TOLERANCE = 200

        def _same(a, b) -> bool:
            ta, va = a
            tb, vb = b
            if ta != tb:
                return False
            if ta == "enum":
                return va == vb
            if ta == "raw":
                return abs(va - vb) <= ENC_TOLERANCE
            return False

        print("\n" + "=" * 60)
        shoulder_unchanged = _same(rot0, rot1)
        wrist_unchanged = _same(wrist0, wrist1)
        if shoulder_unchanged and wrist_unchanged:
            print("✓ PM did NOT change shoulder or wrist orientation.")
            print()
            print("Conclusion: PM is declarative when grip_direction matches")
            print("the pre-set pose. Safe to use as a drop-in replacement")
            print("for chunked GX/GY in orientation-stable manual jogs, as")
            print("long as the caller keeps grip_direction in sync with the")
            print("physical shoulder rotation (FRONT=1, RIGHT=2, BACK=3,")
            print("LEFT=4, assuming wrist=STRAIGHT).")
        else:
            print("✗ PM CHANGED orientation during the move:")
            if not shoulder_unchanged:
                print(f"    shoulder: {_fmt_orientation(rot0)} → "
                      f"{_fmt_orientation(rot1)}")
            if not wrist_unchanged:
                print(f"    wrist:    {_fmt_orientation(wrist0)} → "
                      f"{_fmt_orientation(wrist1)}")
            print()
            print("Conclusion: PM is imperative. Do NOT use for the incubator")
            print("handoff — the current 99.9 mm chunked GX/GY approach is")
            print("the only orientation-stable option via pylabrobot.")
        print("=" * 60)

    finally:
        await lh.stop()


if __name__ == "__main__":
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("--delta", type=float, default=50.0,
                   help="X translation in mm for the PM call (default: 50)")
    p.add_argument("--yes", action="store_true",
                   help="skip all confirmation prompts")
    asyncio.run(main(p.parse_args()))
