"""Thread-safe events-per-second meter."""

import threading
import time
from collections import deque


class RateMeter:
    """Measures events per second over a sliding time window.

    The rate decays toward zero when events stop, so a stalled stream
    shows up as a falling FPS rather than a frozen one.
    """

    def __init__(self, window_s: float = 2.0):
        self.window_s = window_s
        self._times: deque[float] = deque()
        self._lock = threading.Lock()

    def tick(self, now: float | None = None) -> None:
        now = time.monotonic() if now is None else now
        with self._lock:
            self._times.append(now)
            self._prune(now)

    def rate(self, now: float | None = None) -> float:
        now = time.monotonic() if now is None else now
        with self._lock:
            self._prune(now)
            if len(self._times) < 2:
                return 0.0
            return (len(self._times) - 1) / max(now - self._times[0], 1e-6)

    def _prune(self, now: float) -> None:
        while self._times and now - self._times[0] > self.window_s:
            self._times.popleft()
