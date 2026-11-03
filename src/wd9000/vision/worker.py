"""Run a detector in a background thread so slow inference never blocks video."""

import logging
import threading
import time
from dataclasses import dataclass
from typing import Callable

from wd9000.camera.stream import Frame
from wd9000.utils.rate import RateMeter
from wd9000.vision.detector import Detection, Detector

log = logging.getLogger(__name__)


@dataclass(frozen=True)
class InferenceResult:
    detections: list[Detection]
    frame_index: int
    inference_s: float  # time spent in detector.detect()
    latency_s: float  # frame received -> result ready
    completed_at: float  # time.monotonic()


class InferenceWorker:
    """Repeatedly runs the detector on the newest frame, skipping stale ones."""

    def __init__(self, detector: Detector, get_frame: Callable[[], Frame | None], min_interval_s: float = 0.0):
        self.detector = detector
        self.get_frame = get_frame
        self.min_interval_s = min_interval_s
        self.enabled = True
        self.last_error: str | None = None
        self._rate = RateMeter(window_s=5.0)
        self._latest: InferenceResult | None = None
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    @property
    def fps(self) -> float:
        return self._rate.rate()

    def latest(self) -> InferenceResult | None:
        return self._latest

    def start(self) -> "InferenceWorker":
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="InferenceWorker", daemon=True)
        self._thread.start()
        return self

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=5)

    def _run(self) -> None:
        last_index = -1
        while not self._stop.is_set():
            frame = self.get_frame() if self.enabled else None
            if frame is None or frame.index == last_index:
                self._stop.wait(0.01)
                continue
            last_index = frame.index
            start = time.monotonic()
            try:
                detections = self.detector.detect(frame.image)
            except Exception as e:
                self.last_error = f"{type(e).__name__}: {e}"
                log.warning("Inference failed: %s", self.last_error)
                self._stop.wait(1.0)
                continue
            now = time.monotonic()
            self._latest = InferenceResult(detections, frame.index, now - start, now - frame.timestamp, now)
            self._rate.tick(now)
            self.last_error = None
            if self.min_interval_s:
                self._stop.wait(self.min_interval_s)
