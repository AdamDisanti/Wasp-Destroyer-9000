import time

import numpy as np

from wd9000.camera.stream import Frame
from wd9000.vision.detector import MockDetector
from wd9000.vision.worker import InferenceWorker


def _wait_for(predicate, timeout=3.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return False


def test_worker_produces_results_for_latest_frame():
    frame = Frame(np.zeros((480, 640, 3), dtype=np.uint8), index=7, timestamp=time.monotonic())
    worker = InferenceWorker(MockDetector(seed=1), lambda: frame).start()
    try:
        assert _wait_for(lambda: worker.latest() is not None)
        result = worker.latest()
        assert result.frame_index == 7
        assert result.latency_s >= result.inference_s >= 0
    finally:
        worker.stop()


def test_worker_skips_frames_already_processed():
    calls = []

    class CountingDetector:
        def detect(self, image):
            calls.append(1)
            return []

    frame = Frame(np.zeros((8, 8, 3), dtype=np.uint8), index=1, timestamp=time.monotonic())
    worker = InferenceWorker(CountingDetector(), lambda: frame).start()
    try:
        assert _wait_for(lambda: len(calls) == 1)
        time.sleep(0.1)
        assert len(calls) == 1  # same frame index -> not re-run
    finally:
        worker.stop()


def test_worker_survives_detector_errors():
    class BrokenDetector:
        def detect(self, image):
            raise ConnectionError("server down")

    frame = Frame(np.zeros((8, 8, 3), dtype=np.uint8), index=1, timestamp=time.monotonic())
    worker = InferenceWorker(BrokenDetector(), lambda: frame).start()
    try:
        assert _wait_for(lambda: worker.last_error is not None)
        assert "server down" in worker.last_error
        assert worker.latest() is None
    finally:
        worker.stop()


def test_worker_does_nothing_while_disabled():
    frame = Frame(np.zeros((8, 8, 3), dtype=np.uint8), index=1, timestamp=time.monotonic())
    worker = InferenceWorker(MockDetector(), lambda: frame)
    worker.enabled = False
    worker.start()
    try:
        time.sleep(0.1)
        assert worker.latest() is None
    finally:
        worker.stop()
