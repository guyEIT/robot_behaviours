"""Unit tests for the pure-Python parts of seed_detector.

Cover the depth-sampling helper exhaustively; for the YOLO wrapper itself
just smoke-test that it picks up class names from a fake model and that
``detect()`` returns ``Detection`` instances when given a stub
ultralytics-style result. Real-model end-to-end testing happens in the
docker/pixi integration jobs once last.pt is present.
"""

from __future__ import annotations

import sys
import types
from pathlib import Path

import numpy as np
import pytest


# --------------------------------------------------------------------------
# sample_depth_patch
# --------------------------------------------------------------------------

def test_sample_depth_patch_handles_uniform_uint16_mm() -> None:
    from seed_detector.detector import sample_depth_patch

    depth = np.full((10, 10), 800, dtype=np.uint16)  # 800 mm everywhere
    out = sample_depth_patch(depth, cx=5, cy=5, patch=5, depth_scale_to_meters=1e-3)
    assert out == pytest.approx(0.8)


def test_sample_depth_patch_ignores_zero_holes() -> None:
    from seed_detector.detector import sample_depth_patch

    depth = np.full((10, 10), 0, dtype=np.uint16)
    depth[5, 5] = 600  # one valid pixel inside the patch
    out = sample_depth_patch(depth, cx=5, cy=5, patch=5, depth_scale_to_meters=1e-3)
    assert out == pytest.approx(0.6)


def test_sample_depth_patch_returns_none_when_all_invalid() -> None:
    from seed_detector.detector import sample_depth_patch

    depth = np.zeros((10, 10), dtype=np.uint16)
    assert sample_depth_patch(depth, cx=5, cy=5, patch=5) is None


def test_sample_depth_patch_handles_float32_meters() -> None:
    from seed_detector.detector import sample_depth_patch

    depth = np.full((10, 10), 0.42, dtype=np.float32)
    out = sample_depth_patch(depth, cx=5, cy=5, patch=3, depth_scale_to_meters=1.0)
    assert out == pytest.approx(0.42)


def test_sample_depth_patch_clips_window_at_image_edge() -> None:
    from seed_detector.detector import sample_depth_patch

    depth = np.full((10, 10), 1000, dtype=np.uint16)
    # Centre on (0, 0) — the 5×5 window is clipped to the top-left 3×3.
    out = sample_depth_patch(depth, cx=0, cy=0, patch=5, depth_scale_to_meters=1e-3)
    assert out == pytest.approx(1.0)


def test_sample_depth_patch_returns_none_outside_image() -> None:
    from seed_detector.detector import sample_depth_patch

    depth = np.full((10, 10), 1000, dtype=np.uint16)
    assert sample_depth_patch(depth, cx=100, cy=100, patch=3) is None


def test_sample_depth_patch_handles_nan() -> None:
    from seed_detector.detector import sample_depth_patch

    depth = np.full((10, 10), np.nan, dtype=np.float32)
    depth[5, 5] = 0.7
    out = sample_depth_patch(depth, cx=5, cy=5, patch=5, depth_scale_to_meters=1.0)
    assert out == pytest.approx(0.7)


def test_sample_depth_patch_foreground_picks_seed_over_tray() -> None:
    """A small seed (9 px @ 300 mm) sitting on a tray (16 px @ 400 mm) inside
    a 5×5 window: median pulls in the tray depth — the marker plants behind
    the seed. foreground mode (lower quartile) recovers the seed depth.
    """
    from seed_detector.detector import sample_depth_patch

    depth = np.full((5, 5), 400, dtype=np.uint16)
    depth[1:4, 1:4] = 300

    median = sample_depth_patch(
        depth, cx=2, cy=2, patch=5, depth_scale_to_meters=1e-3, mode="median",
    )
    fg25 = sample_depth_patch(
        depth, cx=2, cy=2, patch=5, depth_scale_to_meters=1e-3,
        mode="foreground", foreground_quantile=0.25,
    )
    fg0 = sample_depth_patch(
        depth, cx=2, cy=2, patch=5, depth_scale_to_meters=1e-3,
        mode="foreground", foreground_quantile=0.0,
    )
    assert median == pytest.approx(0.4)
    assert fg25 == pytest.approx(0.3)
    assert fg0 == pytest.approx(0.3)


def test_sample_depth_patch_default_is_foreground() -> None:
    from seed_detector.detector import sample_depth_patch

    depth = np.full((5, 5), 400, dtype=np.uint16)
    depth[1:4, 1:4] = 300
    out = sample_depth_patch(depth, cx=2, cy=2, patch=5, depth_scale_to_meters=1e-3)
    assert out == pytest.approx(0.3)


def test_sample_depth_patch_unknown_mode_raises() -> None:
    from seed_detector.detector import sample_depth_patch

    depth = np.full((5, 5), 500, dtype=np.uint16)
    with pytest.raises(ValueError):
        sample_depth_patch(depth, cx=2, cy=2, patch=3, mode="bogus")


# --------------------------------------------------------------------------
# SeedDetector — stub ultralytics so we don't need the real .pt or torch
# --------------------------------------------------------------------------

class _FakeBoxes:
    """Mimic ultralytics' Boxes return shape (xyxy, conf, cls all on .cpu().numpy())."""

    def __init__(self, xyxy, conf, cls):
        self._xyxy = np.asarray(xyxy, dtype=np.float32)
        self._conf = np.asarray(conf, dtype=np.float32)
        self._cls = np.asarray(cls, dtype=np.int64)

    def __len__(self):
        return len(self._conf)

    @property
    def xyxy(self):
        return _CpuArray(self._xyxy)

    @property
    def conf(self):
        return _CpuArray(self._conf)

    @property
    def cls(self):
        return _CpuArray(self._cls)


class _CpuArray:
    def __init__(self, arr):
        self._arr = arr

    def cpu(self):
        return self

    def numpy(self):
        return self._arr


class _FakeResult:
    def __init__(self, boxes):
        self.boxes = boxes


class _FakeYOLO:
    def __init__(self, weights_path):
        self.weights_path = weights_path
        # Two classes — verifies dict copy in SeedDetector.__init__.
        self.names = {0: "seed", 1: "shell"}
        self.last_call = None

    def predict(self, source, conf, iou, device, verbose):
        self.last_call = dict(source=source, conf=conf, iou=iou, device=device, verbose=verbose)
        return [
            _FakeResult(_FakeBoxes(
                xyxy=[[10, 20, 50, 80], [100, 110, 130, 150]],
                conf=[0.91, 0.62],
                cls=[0, 1],
            ))
        ]


@pytest.fixture
def stub_ultralytics(monkeypatch, tmp_path):
    """Inject a fake ``ultralytics`` module so ``import ultralytics`` returns YOLO=_FakeYOLO."""
    fake = types.ModuleType("ultralytics")
    fake.YOLO = _FakeYOLO  # type: ignore[attr-defined]
    monkeypatch.setitem(sys.modules, "ultralytics", fake)
    weights = tmp_path / "fake.pt"
    weights.write_bytes(b"")  # SeedDetector only checks the path is a file
    return weights


def test_seed_detector_loads_class_names(stub_ultralytics) -> None:
    from seed_detector.detector import SeedDetector

    det = SeedDetector(stub_ultralytics, confidence=0.4, iou=0.5, device="cpu")
    assert det.class_names == {0: "seed", 1: "shell"}
    assert det.confidence == 0.4
    assert det.iou == 0.5
    assert det.device == "cpu"


def test_seed_detector_returns_detections(stub_ultralytics) -> None:
    from seed_detector.detector import Detection, SeedDetector

    det = SeedDetector(stub_ultralytics)
    image = np.zeros((100, 200, 3), dtype=np.uint8)
    out = det.detect(image)

    assert len(out) == 2
    assert all(isinstance(d, Detection) for d in out)

    seed = out[0]
    assert seed.class_id == 0
    assert seed.class_name == "seed"
    assert seed.score == pytest.approx(0.91, rel=1e-3)
    assert seed.cx == pytest.approx(30.0)
    assert seed.cy == pytest.approx(50.0)
    assert seed.width == pytest.approx(40.0)
    assert seed.height == pytest.approx(60.0)


def test_seed_detector_passes_thresholds_to_predict(stub_ultralytics) -> None:
    from seed_detector.detector import SeedDetector

    det = SeedDetector(stub_ultralytics, confidence=0.7, iou=0.3, device="cuda:0")
    det.detect(np.zeros((10, 10, 3), dtype=np.uint8))
    call = det._model.last_call  # type: ignore[attr-defined]
    assert call["conf"] == 0.7
    assert call["iou"] == 0.3
    assert call["device"] == "cuda:0"
    assert call["verbose"] is False


def test_seed_detector_missing_weights_raises(tmp_path) -> None:
    from seed_detector.detector import SeedDetector

    with pytest.raises(FileNotFoundError):
        SeedDetector(tmp_path / "nope.pt")
