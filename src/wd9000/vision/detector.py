"""Wasp-nest detectors behind one interface: cloud, local, or mock.

All detectors take a BGR image (as returned by cv2) and return a list of
Detection objects. Downstream code (overlay, camera alignment) only ever
sees Detection, never raw Roboflow JSON.
"""

import random
from dataclasses import dataclass
from typing import Protocol

import numpy as np


@dataclass(frozen=True)
class Detection:
    label: str
    confidence: float
    # Bounding-box center and size, in pixels.
    x: float
    y: float
    width: float
    height: float

    @property
    def corners(self) -> tuple[int, int, int, int]:
        """(x1, y1, x2, y2) integer pixel corners."""
        return (
            int(self.x - self.width / 2),
            int(self.y - self.height / 2),
            int(self.x + self.width / 2),
            int(self.y + self.height / 2),
        )


class Detector(Protocol):
    def detect(self, image: np.ndarray) -> list[Detection]: ...


def parse_predictions(result: dict, min_confidence: float = 0.0) -> list[Detection]:
    """Convert a Roboflow object-detection response into Detections."""
    detections = [
        Detection(
            label=p["class"],
            confidence=float(p["confidence"]),
            x=float(p["x"]),
            y=float(p["y"]),
            width=float(p["width"]),
            height=float(p["height"]),
        )
        for p in result.get("predictions", [])
    ]
    detections = [d for d in detections if d.confidence >= min_confidence]
    return sorted(detections, key=lambda d: d.confidence, reverse=True)


class RoboflowDetector:
    """Runs a Roboflow model through either the hosted API or a local server."""

    def __init__(self, api_url: str, api_key: str, model_id: str, min_confidence: float = 0.4):
        # Imported here so mock mode and unit tests don't need the SDK.
        from inference_sdk import InferenceConfiguration, InferenceHTTPClient

        self.model_id = model_id
        self.min_confidence = min_confidence
        self.last_result: dict = {}
        # Send the key in the Authorization header, never in the URL.
        self._client = InferenceHTTPClient(api_url=api_url, api_key=api_key).configure(
            InferenceConfiguration(api_key_transport="header")
        )

    def detect(self, image: np.ndarray) -> list[Detection]:
        self.last_result = self._client.infer(image, model_id=self.model_id)
        return parse_predictions(self.last_result, self.min_confidence)


class MockDetector:
    """Returns one plausible fake nest per call. No network or key needed."""

    def __init__(self, seed: int | None = None, min_confidence: float = 0.4):
        self._rng = random.Random(seed)
        self.min_confidence = min_confidence

    def detect(self, image: np.ndarray) -> list[Detection]:
        h, w = image.shape[:2]
        size = self._rng.uniform(0.15, 0.35) * min(w, h)
        detection = Detection(
            label="mock-nest",
            confidence=self._rng.uniform(0.5, 0.95),
            x=self._rng.uniform(size / 2, w - size / 2),
            y=self._rng.uniform(size / 2, h - size / 2),
            width=size,
            height=size,
        )
        return [detection] if detection.confidence >= self.min_confidence else []


def make_detector(settings: dict, api_key: str | None) -> Detector:
    """Build the detector selected by settings['inference']['backend']."""
    cfg = settings["inference"]
    backend = cfg["backend"]
    min_conf = cfg.get("confidence_threshold", 0.4)

    if backend == "mock":
        return MockDetector(min_confidence=min_conf)
    if backend not in ("cloud", "local"):
        raise ValueError(f"Unknown inference backend: {backend!r} (expected cloud, local, or mock)")
    if not api_key:
        raise RuntimeError("ROBOFLOW_API_KEY is not set. Copy .env.example to .env and add your key.")

    url = cfg["cloud_url"] if backend == "cloud" else cfg["local_url"]
    return RoboflowDetector(url, api_key, cfg["model_id"], min_conf)
