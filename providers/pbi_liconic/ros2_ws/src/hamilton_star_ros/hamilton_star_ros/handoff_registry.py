"""Runtime registry of off-deck handoff sites (incubators, cytomats,
centrifuge ports) the iSWAP can reach.

One record per site. The same record serves both transfer styles:

- ``HandoffTransfer`` (6-stage manual-jog recipe) consumes
  ``x, y, z, plate_width, rotation, wrist``.
- ``HotelTransfer`` and ``MoveResource(to_handoff=...)`` (firmware-
  atomic ``PO``/``PI`` path) consume
  ``x, y, z, plate_width, grip_direction, hotel_depth,
  hotel_clearance_height``.

Thread-safe. Seeded at node startup from the same defaults that the
legacy ``handoff.<name>.*`` ROS params used, then mutated at runtime
via ``~/define_handoff`` / ``~/delete_handoff`` services.
"""

from __future__ import annotations

import threading
from dataclasses import dataclass, replace
from typing import Any, Iterable, Optional


@dataclass(frozen=True)
class HandoffRecord:
    """One registered handoff site. All fields are by-value; updates
    create a new record via :func:`dataclasses.replace`."""
    name: str
    x: float
    y: float
    z: float
    plate_width: float
    rotation: str = "LEFT"
    wrist: str = "STRAIGHT"
    grip_direction: str = "FRONT"
    # pylabrobot defaults to 160 mm / 7.5 mm; off-deck sites like the
    # incubator at X≈-216 usually need hotel_depth bumped to ≥500 mm so
    # the iSWAP rotation happens on-deck (firmware plans the rotation
    # at the hotel boundary).
    hotel_depth: float = 160.0
    hotel_clearance_height: float = 7.5

    @classmethod
    def from_dict(cls, name: str, data: dict[str, Any]) -> "HandoffRecord":
        """Build from a ``handoffs.yaml``-shaped dict (legacy params
        only carry x/y/z/plate_width/rotation/wrist; new fields get
        their defaults from the dataclass)."""
        return cls(
            name=name,
            x=float(data["x"]),
            y=float(data["y"]),
            z=float(data["z"]),
            plate_width=float(data.get("plate_width", 80.0)),
            rotation=str(data.get("rotation", "LEFT")),
            wrist=str(data.get("wrist", "STRAIGHT")),
            grip_direction=str(data.get("grip_direction", "FRONT")),
            hotel_depth=float(data.get("hotel_depth", 160.0)),
            hotel_clearance_height=float(
                data.get("hotel_clearance_height", 7.5),
            ),
        )


class HandoffRegistry:
    """Thread-safe in-memory registry of :class:`HandoffRecord`.

    No persistence — the action server seeds this at startup and
    operators mutate it at runtime. Writing back to disk is a separate
    concern (out of scope for the registry itself)."""

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._items: dict[str, HandoffRecord] = {}

    def set(self, record: HandoffRecord, *, replace_existing: bool = True) -> bool:
        """Insert or replace ``record`` under ``record.name``.
        Returns True on success, False when ``replace_existing`` is
        False and the name is already taken."""
        with self._lock:
            if not replace_existing and record.name in self._items:
                return False
            self._items[record.name] = record
            return True

    def get(self, name: str) -> Optional[HandoffRecord]:
        with self._lock:
            return self._items.get(name)

    def delete(self, name: str) -> bool:
        with self._lock:
            return self._items.pop(name, None) is not None

    def all(self) -> list[HandoffRecord]:
        with self._lock:
            return list(self._items.values())

    def names(self) -> list[str]:
        with self._lock:
            return list(self._items.keys())

    def seed(self, records: Iterable[HandoffRecord]) -> None:
        """Bulk insert — only for startup seeding. Overwrites whatever
        is in the registry."""
        with self._lock:
            self._items = {r.name: r for r in records}

    def __len__(self) -> int:
        with self._lock:
            return len(self._items)

    def __contains__(self, name: str) -> bool:
        with self._lock:
            return name in self._items
