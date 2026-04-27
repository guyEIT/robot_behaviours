"""In-memory imaging sim backend.

Writes placeholder image files for each requested site so the campaign tree
can run a full Liconic → Hamilton → Imager → Hamilton → Liconic loop without
real hardware. Output layout:

    {output_root}/{plate_name}/{cycle_dir}/site_{n}.png

where `cycle_dir` is a timestamped directory (epoch ms) so two back-to-back
cycles of the same plate don't overwrite each other. The actual file
contents are a tiny PNG header + protocol/site metadata as ancillary text
chunks — enough to round-trip through tools that expect a real PNG, but
small enough to write 30+ plates per day for weeks without disk pressure.

Pure-stdlib: no numpy, no PIL, no rclpy. Tests run with plain pytest.
"""

from __future__ import annotations

import itertools
import json
import os
import struct
import time
import zlib
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Optional


# Monotonic per-process counter to disambiguate two captures that land in
# the same millisecond. Real imager backends don't have this problem (they
# take seconds per capture); the sim is fast enough to clash without it.
_CYCLE_COUNTER = itertools.count()


_DEFAULT_OUTPUT_ROOT = Path.home() / ".local" / "state" / "imaging_station"
_DEFAULT_SITE_COUNT = 3
_DEFAULT_DELAY_PER_SITE_SEC = 0.5


def default_output_root() -> Path:
    base = os.environ.get("XDG_STATE_HOME", "")
    if base:
        return Path(base) / "imaging_station"
    return _DEFAULT_OUTPUT_ROOT


@dataclass
class SiteCapture:
    """One placeholder image-on-disk produced by the sim."""

    site_index: int
    path: Path

    @property
    def uri(self) -> str:
        return f"file://{self.path}"


@dataclass
class CaptureResult:
    plate_name: str
    protocol: str
    sites: list[SiteCapture]
    metadata: dict

    @property
    def uris(self) -> list[str]:
        return [s.uri for s in self.sites]

    @property
    def metadata_json(self) -> str:
        return json.dumps(self.metadata, sort_keys=True)


def _placeholder_png_bytes(width: int = 8, height: int = 8,
                           text_chunks: Optional[dict] = None) -> bytes:
    """Tiny but valid PNG. Solid mid-grey square so a viewer renders something
    rather than complaining; tEXt ancillary chunks carry plate metadata.

    Hand-rolled to avoid a Pillow dependency on every host (the sim has to
    work in `lab-sim-up` and `lite-up` envs that don't ship imaging libs).
    """

    def _chunk(tag: bytes, data: bytes) -> bytes:
        crc = zlib.crc32(tag + data) & 0xFFFFFFFF
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", crc)

    sig = b"\x89PNG\r\n\x1a\n"
    ihdr = struct.pack(">IIBBBBB", width, height, 8, 0, 0, 0, 0)  # 8-bit gray
    # One filter byte (0) + width grey pixels per row.
    raw = b"".join(b"\x00" + b"\x80" * width for _ in range(height))
    idat = zlib.compress(raw, level=1)
    chunks = [sig, _chunk(b"IHDR", ihdr)]
    for k, v in (text_chunks or {}).items():
        # tEXt: keyword (1-79 chars latin-1) + 0x00 + text
        chunks.append(_chunk(
            b"tEXt", k.encode("latin-1") + b"\x00" + v.encode("latin-1", "replace")
        ))
    chunks.append(_chunk(b"IDAT", idat))
    chunks.append(_chunk(b"IEND", b""))
    return b"".join(chunks)


class ImagingSim:
    """Configurable in-process imager.

    Parameters
    ----------
    output_root :
        Base dir for all captures. Default ``~/.local/state/imaging_station/``
        (or ``$XDG_STATE_HOME/imaging_station/`` when set).
    delay_per_site_sec :
        How long to "spend" on each site. Drives feedback cadence and lets
        tests run with delay=0. Real-imager-equivalent values are 5–60 s
        per site; the sim default is 0.5 so a 3-site protocol takes ~1.5 s.
    default_site_count :
        Used when an ImagePlate goal arrives with site_count=0.
    sleep :
        Injection point for tests to skip real waiting. Default uses
        ``time.sleep``; tests pass a no-op or a hook that records calls.
    halt_check :
        Optional callable that returns True when the action should abort
        early. Polled between sites.
    """

    def __init__(
        self,
        output_root: Optional[Path] = None,
        delay_per_site_sec: float = _DEFAULT_DELAY_PER_SITE_SEC,
        default_site_count: int = _DEFAULT_SITE_COUNT,
        sleep=time.sleep,
        halt_check=None,
    ):
        self.output_root = Path(output_root) if output_root else default_output_root()
        self.delay_per_site_sec = float(delay_per_site_sec)
        self.default_site_count = int(default_site_count)
        self._sleep = sleep
        self._halt_check = halt_check

    def capture(
        self,
        plate_name: str,
        protocol: str = "",
        site_count: int = 0,
        output_root: Optional[Path] = None,
        on_progress=None,
    ) -> CaptureResult:
        """Run a synthetic capture. Yields per-site progress via ``on_progress``
        (called with site_completed, site_total, stage_name) so an action
        server can republish as feedback.
        """
        plate_name = plate_name or "unnamed_plate"
        protocol = protocol or "default"
        n = site_count if site_count > 0 else self.default_site_count
        root = Path(output_root) if output_root else self.output_root
        cycle_dir = root / plate_name / (
            f"{int(time.time() * 1000)}_{next(_CYCLE_COUNTER):04d}"
        )
        cycle_dir.mkdir(parents=True, exist_ok=True)

        if on_progress:
            on_progress(0, n, "warming_up")
        self._sleep_or_halt(self.delay_per_site_sec)

        if on_progress:
            on_progress(0, n, "focusing")
        self._sleep_or_halt(self.delay_per_site_sec)

        sites: list[SiteCapture] = []
        for i in range(n):
            if self._halt_check and self._halt_check():
                # Mid-capture abort: return what we have so far. Caller maps
                # this to action-FAILED.
                break
            stage = f"site_{i + 1}"
            if on_progress:
                on_progress(i, n, stage)
            self._sleep_or_halt(self.delay_per_site_sec)
            path = cycle_dir / f"site_{i + 1}.png"
            png = _placeholder_png_bytes(text_chunks={
                "Plate": plate_name,
                "Protocol": protocol,
                "Site": str(i + 1),
                "CapturedAt": str(int(time.time())),
            })
            path.write_bytes(png)
            sites.append(SiteCapture(site_index=i + 1, path=path))

        if on_progress:
            on_progress(len(sites), n, "writing")

        metadata = {
            "plate_name": plate_name,
            "protocol": protocol,
            "site_count": len(sites),
            "site_count_requested": n,
            "captured_at": int(time.time()),
            "cycle_dir": str(cycle_dir),
            "engine": "imaging_station_sim",
        }
        # Drop a JSON metadata file alongside the images so a downstream
        # consumer can read the same info without parsing PNG tEXt chunks.
        (cycle_dir / "metadata.json").write_text(
            json.dumps(metadata, sort_keys=True, indent=2)
        )

        if on_progress:
            on_progress(len(sites), n, "done")

        return CaptureResult(
            plate_name=plate_name,
            protocol=protocol,
            sites=sites,
            metadata=metadata,
        )

    def _sleep_or_halt(self, seconds: float) -> None:
        if seconds <= 0:
            return
        # Coarse poll so a halt during the inter-site delay returns quickly.
        end = time.monotonic() + seconds
        while True:
            if self._halt_check and self._halt_check():
                return
            remaining = end - time.monotonic()
            if remaining <= 0:
                return
            self._sleep(min(remaining, 0.05))


def list_uris(captures: Iterable[SiteCapture]) -> list[str]:
    return [c.uri for c in captures]
