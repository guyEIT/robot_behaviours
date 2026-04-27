"""Unit tests for ImagingSim backend. Pure stdlib, no ROS deps.

Run via: colcon test --packages-select imaging_station
"""

import json
import os
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(
    0,
    os.path.abspath(os.path.join(os.path.dirname(__file__), "..")),
)

from imaging_station.sim_backend import (  # noqa: E402
    ImagingSim,
    _placeholder_png_bytes,
    default_output_root,
)


class TestPlaceholderPng(unittest.TestCase):
    def test_starts_with_png_signature(self):
        png = _placeholder_png_bytes()
        self.assertEqual(png[:8], b"\x89PNG\r\n\x1a\n")

    def test_contains_iend(self):
        png = _placeholder_png_bytes()
        self.assertIn(b"IEND", png)

    def test_metadata_chunk_present(self):
        png = _placeholder_png_bytes(text_chunks={"Plate": "P1", "Protocol": "x"})
        self.assertIn(b"Plate\x00P1", png)
        self.assertIn(b"Protocol\x00x", png)


class TestDefaultRoot(unittest.TestCase):
    def test_xdg_state_home(self):
        prev = os.environ.get("XDG_STATE_HOME")
        try:
            os.environ["XDG_STATE_HOME"] = "/tmp/test_xdg_imaging"
            self.assertEqual(
                str(default_output_root()),
                "/tmp/test_xdg_imaging/imaging_station",
            )
        finally:
            if prev is None:
                os.environ.pop("XDG_STATE_HOME", None)
            else:
                os.environ["XDG_STATE_HOME"] = prev


class TestCapture(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.tmp = Path(self._tmp.name)
        # Zero delay so tests run instantly.
        self.sim = ImagingSim(
            output_root=self.tmp,
            delay_per_site_sec=0,
            default_site_count=3,
            sleep=lambda _s: None,
        )

    def tearDown(self):
        self._tmp.cleanup()

    def test_captures_default_site_count(self):
        result = self.sim.capture(plate_name="P1", protocol="brightfield")
        self.assertEqual(len(result.sites), 3)
        for site in result.sites:
            self.assertTrue(site.path.is_file())
            self.assertEqual(site.path.suffix, ".png")
            self.assertTrue(site.path.read_bytes().startswith(b"\x89PNG"))

    def test_explicit_site_count_overrides_default(self):
        result = self.sim.capture(plate_name="P1", site_count=5)
        self.assertEqual(len(result.sites), 5)

    def test_writes_metadata_json(self):
        result = self.sim.capture(plate_name="P1", protocol="bf3")
        cycle_dir = Path(result.metadata["cycle_dir"])
        meta_path = cycle_dir / "metadata.json"
        self.assertTrue(meta_path.is_file())
        loaded = json.loads(meta_path.read_text())
        self.assertEqual(loaded["plate_name"], "P1")
        self.assertEqual(loaded["protocol"], "bf3")
        self.assertEqual(loaded["site_count"], 3)

    def test_uris_are_file_urls_to_real_paths(self):
        result = self.sim.capture(plate_name="P1")
        for uri, site in zip(result.uris, result.sites):
            self.assertTrue(uri.startswith("file://"))
            self.assertEqual(uri[len("file://"):], str(site.path))

    def test_progress_callback_receives_stages(self):
        seen = []
        self.sim.capture(plate_name="P1", on_progress=lambda d, t, s: seen.append(s))
        # Always start with warming_up + focusing, end with writing + done,
        # have one site_N callback per captured site.
        self.assertIn("warming_up", seen)
        self.assertIn("focusing", seen)
        self.assertIn("done", seen)
        site_stages = [s for s in seen if s.startswith("site_")]
        self.assertEqual(len(site_stages), 3)

    def test_separate_cycles_dont_collide(self):
        r1 = self.sim.capture(plate_name="P1")
        r2 = self.sim.capture(plate_name="P1")
        self.assertNotEqual(
            r1.metadata["cycle_dir"], r2.metadata["cycle_dir"]
        )

    def test_per_plate_dirs(self):
        self.sim.capture(plate_name="P1")
        self.sim.capture(plate_name="P2")
        self.assertTrue((self.tmp / "P1").is_dir())
        self.assertTrue((self.tmp / "P2").is_dir())


class TestHaltMidRun(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.tmp = Path(self._tmp.name)
        self._halted = False
        # Halt before starting site 2.
        self._calls = 0

        def _halt():
            self._calls += 1
            return self._halted

        self.sim = ImagingSim(
            output_root=self.tmp,
            delay_per_site_sec=0,
            default_site_count=5,
            sleep=lambda _s: None,
            halt_check=_halt,
        )
        self._halt = _halt

    def tearDown(self):
        self._tmp.cleanup()

    def test_halt_before_any_site_returns_zero_captures(self):
        self._halted = True
        result = self.sim.capture(plate_name="P1")
        self.assertEqual(len(result.sites), 0)

    def test_partial_capture_keeps_completed_sites(self):
        # Halt is checked at the TOP of each capture loop iteration, so
        # flipping the flag after site 2's progress callback means site 3
        # never starts and we exit with 2 sites on disk.
        def _on_progress(done, _total, stage):
            if stage == "site_2":
                # done=1 fires BEFORE site 2's image is written. Move the
                # flip to after site 2 has captured by checking on the
                # post-capture progress (next stage's start).
                pass
            if stage == "site_3":
                # site_3's progress fires AFTER halt-check at i=2, so by
                # the time we're here the third capture is committed.
                # Flip the flag here too — it'll only stop site_4+ from
                # starting, which is the case we care about (5-site default).
                self._halted = True

        result = self.sim.capture(plate_name="P1", on_progress=_on_progress)
        # 5-site protocol; halt set during site_3 progress → site_3 still
        # writes (halt-check already passed for i=2), site_4 onwards do not.
        self.assertEqual(len(result.sites), 3)


if __name__ == "__main__":
    unittest.main()
