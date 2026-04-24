"""Shared helpers for the direct-hardware Liconic test scripts.

Keep this deliberately small: backend construction, rack creation, a
yes/no confirmation prompt. Anything flow-specific (take-in vs fetch vs
humidity) lives in the per-script file.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import List, Optional

from pylabrobot.resources import Plate, PlateCarrier, PlateHolder
from pylabrobot.resources.corning import Cor_96_wellplate_360ul_Fb
from pylabrobot.storage.liconic import ExperimentalLiconicBackend
from pylabrobot.storage.liconic import racks as liconic_racks
from pylabrobot.storage.liconic.constants import LiconicType

# Share the ROS server's PlateRegistry implementation. The module is pure
# Python (stdlib only) and has no rclpy deps, so importing it from the
# default env via a sys.path insert is safe — and it means the JSON
# schema stays single-source-of-truth.
_LICONIC_ROS_PATH = Path(__file__).resolve().parent.parent / "ros2_ws/src/liconic_ros"
if str(_LICONIC_ROS_PATH) not in sys.path:
    sys.path.insert(0, str(_LICONIC_ROS_PATH))
from liconic_ros.plate_registry import PlateRegistry, PlateRegistryError  # noqa: E402


DEFAULT_PORT = "/dev/liconic_stx44"
DEFAULT_MODEL = "STX44_IC"
DEFAULT_RACK_MODEL = "liconic_rack_12mm_27"
DEFAULT_TOTAL_CASSETTES = 2   # STX44 stock is a two-cassette carousel
DEFAULT_STATE_FILE = "~/.local/state/liconic/plate_registry.json"


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


def build_backend(port: str, model: str) -> ExperimentalLiconicBackend:
    try:
        liconic_type = LiconicType(model)
    except ValueError as e:
        raise SystemExit(
            f"Unknown Liconic model '{model}'. "
            f"Valid: {', '.join(m.value for m in LiconicType)}"
        ) from e
    return ExperimentalLiconicBackend(model=liconic_type, port=port)


def build_racks(rack_model: str, total_cassettes: int) -> List[PlateCarrier]:
    """Build `total_cassettes` identical racks (1-indexed by the PLC as DM0=m)."""
    factory = getattr(liconic_racks, rack_model, None)
    if factory is None:
        # list the STX44 variants only — the STX500/1000 ones won't fit here
        stx44 = sorted(
            n for n in dir(liconic_racks)
            if n.startswith("liconic_rack_") and "mm_" in n
        )
        raise SystemExit(
            f"Unknown rack model '{rack_model}'. Known rack factories:\n  "
            + "\n  ".join(stx44)
        )
    return [factory(name=f"liconic_cassette_{i + 1}") for i in range(total_cassettes)]


def pick_site(
    racks: List[PlateCarrier], cassette: int, position: int
) -> PlateHolder:
    if not 1 <= cassette <= len(racks):
        raise SystemExit(
            f"--cassette {cassette} out of range; racks configured: 1..{len(racks)}"
        )
    rack = racks[cassette - 1]
    sites = list(rack.sites.values())
    if not 1 <= position <= len(sites):
        raise SystemExit(
            f"--position {position} out of range for rack "
            f"'{rack.model}'; valid: 1..{len(sites)}"
        )
    return sites[position - 1]


def make_test_plate(name: str = "test_plate") -> Plate:
    """SBS-footprint plate for source/destination bookkeeping.

    The Liconic backend doesn't care about wells or size_z — it only uses
    the plate's parent PlateHolder for fetch-source lookup. Uses a stock
    Corning 96-well plate because PlateHolder.assign_child_resource
    computes a sinking depth from the wells' z-coordinates, so a
    well-less Plate fails that check.
    """
    return Cor_96_wellplate_360ul_Fb(name=name)


def open_registry(state_file: str) -> PlateRegistry:
    """Open the same JSON plate registry the ROS server reads."""
    return PlateRegistry(Path(state_file).expanduser())


def register_take_in(
    state_file: str,
    plate_name: str,
    cassette: int,
    position: int,
    barcode: str = "",
) -> None:
    """Mirror the ROS server's post-motion registry update.

    Call this AFTER the physical take-in succeeds. If the update fails
    (duplicate name, slot occupied), prints a warning — the physical
    plate is already in the cassette, so we don't re-raise.
    """
    try:
        reg = open_registry(state_file)
        reg.place_in_cassette(
            plate_name=plate_name, cassette=cassette,
            position=position, barcode=barcode,
        )
        print(f"  registry updated: {plate_name!r} → cassette {cassette} "
              f"position {position}  [{reg.path}]")
    except PlateRegistryError as exc:
        print(
            f"\nWARNING: physical take-in ok but registry update failed: {exc}\n"
            f"         Plate is in the cassette; registry at {state_file} is "
            "now out of sync."
        )


def register_fetch(
    state_file: str,
    cassette: int,
    position: int,
) -> Optional[str]:
    """Mirror the ROS server's post-motion registry update on fetch.

    Returns the plate_name that was moved, or None if the slot wasn't
    registered (warning printed — registry stays with tray=None).
    """
    try:
        reg = open_registry(state_file)
        rec = reg.at(cassette, position)
        if rec is None:
            print(
                f"\nWARNING: physical fetch ok but no plate was registered at "
                f"cassette {cassette} position {position}. Registry at "
                f"{state_file} left unchanged — run take-in first, or edit "
                "the JSON if you want to reconcile a manually-placed plate."
            )
            return None
        moved = reg.move_cassette_to_tray(cassette=cassette, position=position)
        print(f"  registry updated: {moved.plate_name!r} → loading tray  "
              f"[{reg.path}]")
        return moved.plate_name
    except PlateRegistryError as exc:
        print(
            f"\nWARNING: physical fetch ok but registry update failed: {exc}\n"
            f"         Plate is on the tray; registry at {state_file} is "
            "now out of sync."
        )
        return None
