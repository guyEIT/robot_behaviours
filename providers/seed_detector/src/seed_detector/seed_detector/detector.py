"""Pure-Python YOLO wrapper. No rclpy here so it's unit-testable in isolation."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List, Optional

import numpy as np


@dataclass(frozen=True)
class Detection:
    """One YOLO detection in pixel coordinates."""

    class_id: int
    class_name: str
    score: float
    x1: float
    y1: float
    x2: float
    y2: float

    @property
    def cx(self) -> float:
        return 0.5 * (self.x1 + self.x2)

    @property
    def cy(self) -> float:
        return 0.5 * (self.y1 + self.y2)

    @property
    def width(self) -> float:
        return self.x2 - self.x1

    @property
    def height(self) -> float:
        return self.y2 - self.y1


class SeedDetector:
    """Thin wrapper around ultralytics.YOLO.

    Loads the model once at construction; ``detect`` runs inference on a
    single BGR numpy frame and returns a list of ``Detection`` objects.
    """

    def __init__(
        self,
        model_path: str | Path,
        confidence: float = 0.5,
        iou: float = 0.45,
        device: str = "",
    ) -> None:
        path = Path(model_path).expanduser()
        if not path.is_file():
            raise FileNotFoundError(f"YOLO weights not found at {path}")

        # Imported lazily so the unit tests can patch this attribute and
        # exercise SeedDetector without ultralytics installed.
        from ultralytics import YOLO  # type: ignore

        self.model_path = path
        self.confidence = float(confidence)
        self.iou = float(iou)
        self.device = device or None  # ultralytics treats None as "auto"
        self._model = YOLO(str(path))
        # ultralytics loads class names from the trained model.
        self._class_names: dict[int, str] = dict(self._model.names) if self._model.names else {}

    @property
    def class_names(self) -> dict[int, str]:
        return self._class_names

    def detect(
        self,
        bgr_image: np.ndarray,
        with_overlay: bool = False,
    ):
        """Run YOLO on a BGR image; return per-detection class/score/bbox.

        When ``with_overlay=True`` returns ``(detections, overlay_bgr)``
        where ``overlay_bgr`` is a copy of the input with bounding boxes,
        class labels, and scores drawn on (via ultralytics' built-in
        ``Results.plot()``). When ``with_overlay=False`` returns just the
        list of detections.
        """
        results = self._model.predict(
            source=bgr_image,
            conf=self.confidence,
            iou=self.iou,
            device=self.device,
            verbose=False,
        )
        detections = list(self._iter_detections(results))
        if not with_overlay:
            return detections
        if results:
            overlay = results[0].plot()  # BGR ndarray
        else:
            overlay = bgr_image.copy()
        return detections, overlay

    def _iter_detections(self, results: Iterable) -> Iterable[Detection]:
        for result in results:
            boxes = getattr(result, "boxes", None)
            if boxes is None or len(boxes) == 0:
                continue
            xyxy = boxes.xyxy.cpu().numpy()
            confs = boxes.conf.cpu().numpy()
            classes = boxes.cls.cpu().numpy().astype(int)
            for (x1, y1, x2, y2), score, cls_id in zip(xyxy, confs, classes):
                yield Detection(
                    class_id=int(cls_id),
                    class_name=self._class_names.get(int(cls_id), str(int(cls_id))),
                    score=float(score),
                    x1=float(x1), y1=float(y1),
                    x2=float(x2), y2=float(y2),
                )


def sample_depth_patch(
    depth_image: np.ndarray,
    cx: float,
    cy: float,
    patch: int = 5,
    depth_scale_to_meters: float = 1e-3,
    mode: str = "foreground",
    foreground_quantile: float = 0.25,
) -> Optional[float]:
    """Robust depth (in metres) at (cx, cy), sampled over a ``patch``×``patch``
    window. Zeros / NaNs are ignored. Returns ``None`` if the window has
    no valid samples (e.g. it lies on a depth shadow).

    ``mode`` controls how the window is reduced to a single depth:

    * ``"foreground"`` (default): take the ``foreground_quantile`` percentile
      (default 25%) of valid depths. Biased toward the closest object in the
      window — what you want when a small target (e.g. a seed) is in front
      of a tray and the patch straddles both. ``foreground_quantile=0.0``
      degenerates to ``min``.
    * ``"median"``: legacy behaviour. Robust to single-pixel noise but
      averages target-depth and background-depth pixels when they coexist
      in the window — the marker ends up planted behind the target.

    RealSense aligned depth ships as 16-bit unsigned millimetres — pass
    ``depth_scale_to_meters=1e-3``. For float32 depth already in metres,
    pass ``1.0``.
    """
    h, w = depth_image.shape[:2]
    if not (0 <= cx < w and 0 <= cy < h):
        return None

    half = patch // 2
    x0 = max(int(round(cx)) - half, 0)
    y0 = max(int(round(cy)) - half, 0)
    x1 = min(int(round(cx)) + half + 1, w)
    y1 = min(int(round(cy)) + half + 1, h)

    window = depth_image[y0:y1, x0:x1].astype(np.float32)
    valid = window[(window > 0) & np.isfinite(window)]
    if valid.size == 0:
        return None

    if mode == "median":
        depth = float(np.median(valid))
    elif mode == "foreground":
        q = float(np.clip(foreground_quantile, 0.0, 1.0))
        depth = float(np.quantile(valid, q))
    else:
        raise ValueError(f"unknown depth-sampling mode {mode!r}")
    return depth * depth_scale_to_meters
