"""Persistent plate-location registry for the Liconic action server.

One JSON file is the authoritative record of what plate sits where. It
is loaded into memory at node startup and rewritten (atomically, via
temp-file + rename) after every mutation so the server's state survives
process restarts.

The registry does NOT drive motion — it is updated only after the
pylabrobot backend acknowledges a command. If a motion fails midway the
entry stays in its pre-call location; an operator can reconcile
manually or via a `reset` service in a future revision.

Schema (version 1):

    {
      "version": 1,
      "loading_tray": {"plate_name": str, "barcode": str, "placed_at": str} | null,
      "cassettes": {
        "<cassette_int>": {
          "<position_int>": {"plate_name": str, "barcode": str, "placed_at": str}
        },
        ...
      }
    }
"""

from __future__ import annotations

import json
import os
import threading
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, List, Optional


SCHEMA_VERSION = 1


@dataclass(frozen=True)
class PlateRecord:
    plate_name: str
    barcode: str
    cassette: int   # 0 = loading tray; >= 1 = carousel cassette
    position: int   # 0 = loading tray; >= 1 = carousel site within cassette
    placed_at: str


class PlateRegistryError(Exception):
    pass


class PlateRegistry:
    """Thread-safe plate-state store backed by a single JSON file."""

    def __init__(self, state_file: Path) -> None:
        self._path = Path(state_file).expanduser()
        self._lock = threading.Lock()
        self._loading_tray: Optional[PlateRecord] = None
        self._cassettes: Dict[int, Dict[int, PlateRecord]] = {}
        self._load()

    # ---- persistence ----

    def _load(self) -> None:
        if not self._path.exists():
            return
        try:
            data = json.loads(self._path.read_text())
        except json.JSONDecodeError as e:
            raise PlateRegistryError(
                f"plate registry at {self._path} is not valid JSON: {e}"
            ) from e
        if data.get("version") != SCHEMA_VERSION:
            raise PlateRegistryError(
                f"plate registry at {self._path} has version "
                f"{data.get('version')!r}, expected {SCHEMA_VERSION}"
            )
        tray = data.get("loading_tray")
        self._loading_tray = (
            PlateRecord(
                plate_name=tray["plate_name"],
                barcode=tray.get("barcode", ""),
                cassette=0, position=0,
                placed_at=tray.get("placed_at", ""),
            ) if tray else None
        )
        self._cassettes = {}
        for cassette_str, positions in (data.get("cassettes") or {}).items():
            cassette = int(cassette_str)
            self._cassettes[cassette] = {}
            for pos_str, entry in positions.items():
                position = int(pos_str)
                self._cassettes[cassette][position] = PlateRecord(
                    plate_name=entry["plate_name"],
                    barcode=entry.get("barcode", ""),
                    cassette=cassette,
                    position=position,
                    placed_at=entry.get("placed_at", ""),
                )

    def _persist(self) -> None:
        self._path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self._path.with_suffix(self._path.suffix + ".tmp")
        tray = self._loading_tray
        data = {
            "version": SCHEMA_VERSION,
            "loading_tray": (
                None if tray is None else {
                    "plate_name": tray.plate_name,
                    "barcode": tray.barcode,
                    "placed_at": tray.placed_at,
                }
            ),
            "cassettes": {
                str(c): {
                    str(p): {
                        "plate_name": rec.plate_name,
                        "barcode": rec.barcode,
                        "placed_at": rec.placed_at,
                    }
                    for p, rec in positions.items()
                }
                for c, positions in self._cassettes.items()
                if positions
            },
        }
        tmp.write_text(json.dumps(data, indent=2, sort_keys=True))
        os.replace(tmp, self._path)

    @property
    def path(self) -> Path:
        return self._path

    # ---- queries ----

    def loading_tray(self) -> Optional[PlateRecord]:
        with self._lock:
            return self._loading_tray

    def at(self, cassette: int, position: int) -> Optional[PlateRecord]:
        with self._lock:
            return self._cassettes.get(cassette, {}).get(position)

    def find_by_name(self, plate_name: str) -> Optional[PlateRecord]:
        with self._lock:
            tray = self._loading_tray
            if tray and tray.plate_name == plate_name:
                return tray
            for positions in self._cassettes.values():
                for rec in positions.values():
                    if rec.plate_name == plate_name:
                        return rec
        return None

    def all_cassette_plates(self) -> List[PlateRecord]:
        with self._lock:
            out: List[PlateRecord] = []
            for cassette in sorted(self._cassettes):
                for position in sorted(self._cassettes[cassette]):
                    out.append(self._cassettes[cassette][position])
            return out

    # ---- mutations ----

    def place_in_cassette(
        self,
        plate_name: str,
        cassette: int,
        position: int,
        barcode: str = "",
        placed_at: Optional[str] = None,
    ) -> PlateRecord:
        """Record that ``plate_name`` is now at (cassette, position).

        Also clears the loading tray entry if it named this plate.
        Persists to disk. Raises PlateRegistryError on collision.
        """
        if cassette < 1 or position < 1:
            raise PlateRegistryError(
                f"cassette and position must be >= 1; got ({cassette}, {position})"
            )
        now = placed_at or _now_iso()
        with self._lock:
            if cassette in self._cassettes and position in self._cassettes[cassette]:
                existing = self._cassettes[cassette][position]
                raise PlateRegistryError(
                    f"cassette {cassette} position {position} is already holding "
                    f"plate {existing.plate_name!r}"
                )
            # Reject duplicate plate_name across cassettes. The loading tray is
            # exempt: if it already holds this plate we treat take-in as a
            # move-from-tray and clear it after placement.
            tray = self._loading_tray
            for c, positions in self._cassettes.items():
                for p, rec in positions.items():
                    if rec.plate_name == plate_name:
                        raise PlateRegistryError(
                            f"plate_name {plate_name!r} is already registered at "
                            f"cassette {c} position {p}"
                        )

            record = PlateRecord(
                plate_name=plate_name, barcode=barcode,
                cassette=cassette, position=position, placed_at=now,
            )
            self._cassettes.setdefault(cassette, {})[position] = record
            if tray is not None and tray.plate_name == plate_name:
                self._loading_tray = None
            self._persist()
            return record

    def move_cassette_to_tray(
        self,
        cassette: int,
        position: int,
        placed_at: Optional[str] = None,
    ) -> PlateRecord:
        """Move the plate at (cassette, position) onto the loading tray.

        Raises PlateRegistryError if no plate is registered at the
        source site or if the tray is already occupied.
        """
        now = placed_at or _now_iso()
        with self._lock:
            if self._loading_tray is not None:
                raise PlateRegistryError(
                    f"loading tray is occupied by plate "
                    f"{self._loading_tray.plate_name!r}"
                )
            rec = self._cassettes.get(cassette, {}).get(position)
            if rec is None:
                raise PlateRegistryError(
                    f"no plate registered at cassette {cassette} position {position}"
                )
            del self._cassettes[cassette][position]
            if not self._cassettes[cassette]:
                del self._cassettes[cassette]
            self._loading_tray = PlateRecord(
                plate_name=rec.plate_name, barcode=rec.barcode,
                cassette=0, position=0, placed_at=now,
            )
            self._persist()
            return self._loading_tray

    def set_loading_tray(
        self,
        plate_name: str,
        barcode: str = "",
        placed_at: Optional[str] = None,
    ) -> PlateRecord:
        """Directly record that ``plate_name`` now occupies the tray.

        For bootstrap or reconciliation — normal operation goes through
        ``move_cassette_to_tray`` or ``place_in_cassette``.
        """
        now = placed_at or _now_iso()
        with self._lock:
            if self._loading_tray is not None:
                raise PlateRegistryError(
                    f"loading tray is occupied by plate "
                    f"{self._loading_tray.plate_name!r}"
                )
            if self.find_by_name(plate_name) is not None:
                raise PlateRegistryError(
                    f"plate_name {plate_name!r} is already registered"
                )
            self._loading_tray = PlateRecord(
                plate_name=plate_name, barcode=barcode,
                cassette=0, position=0, placed_at=now,
            )
            self._persist()
            return self._loading_tray

    def clear_cassette_position(
        self, cassette: int, position: int,
    ) -> Optional[PlateRecord]:
        """Drop the entry at (cassette, position) without touching the tray.

        Recovery primitive for when a physical plate has left the
        cassette outside the server's control (operator removed it, or a
        prior fetch motion succeeded but registry persistence failed).
        Returns the PlateRecord that was cleared, or None if the slot
        was already empty.
        """
        with self._lock:
            positions = self._cassettes.get(cassette)
            if positions is None or position not in positions:
                return None
            rec = positions.pop(position)
            if not positions:
                del self._cassettes[cassette]
            self._persist()
            return rec

    def clear_loading_tray(self) -> Optional[PlateRecord]:
        with self._lock:
            prev = self._loading_tray
            self._loading_tray = None
            self._persist()
            return prev


def _now_iso() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")
