"""Camera sources: ESP32 MJPEG over HTTP, video files, still images, or webcams.

FrameSource runs a background reader thread and keeps only the newest
frame, so a slow consumer (display, inference) never builds up lag.
"""

import logging
import threading
import time
import urllib.request
from dataclasses import dataclass
from pathlib import Path
from typing import Iterator

import cv2
import numpy as np

from wd9000.utils.rate import RateMeter

log = logging.getLogger(__name__)

SOI = b"\xff\xd8"  # JPEG start-of-image marker
EOI = b"\xff\xd9"  # JPEG end-of-image marker
IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp"}
STILL_IMAGE_FPS = 10.0


def iter_mjpeg_jpegs(chunks: Iterator[bytes], max_buffer: int = 2_000_000) -> Iterator[bytes]:
    """Yield complete JPEGs from a stream of raw MJPEG bytes.

    Scans for JPEG start/end markers instead of trusting multipart headers,
    because the ELEGOO firmware omits the boundary before the first frame.
    """
    buf = b""
    for chunk in chunks:
        buf += chunk
        while True:
            start = buf.find(SOI)
            if start < 0:
                buf = buf[-1:]  # keep a possibly split marker byte
                break
            end = buf.find(EOI, start + 2)
            if end < 0:
                buf = buf[start:]
                if len(buf) > max_buffer:
                    log.warning("Dropping oversized partial frame (%d bytes)", len(buf))
                    buf = b""
                break
            yield buf[start:end + 2]
            buf = buf[end + 2:]


def _http_chunks(url: str, timeout_s: float, chunk_size: int = 4096) -> Iterator[bytes]:
    with urllib.request.urlopen(url, timeout=timeout_s) as resp:
        while chunk := resp.read1(chunk_size):
            yield chunk


def _mjpeg_frames(url: str, timeout_s: float) -> Iterator[np.ndarray]:
    for jpg in iter_mjpeg_jpegs(_http_chunks(url, timeout_s)):
        frame = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
        if frame is not None:
            yield frame


def _image_frames(path: Path) -> Iterator[np.ndarray]:
    image = cv2.imread(str(path))
    if image is None:
        raise ConnectionError(f"Could not read image: {path}")
    while True:
        yield image
        time.sleep(1 / STILL_IMAGE_FPS)


def _capture_frames(source: str | int, realtime: bool) -> Iterator[np.ndarray]:
    cap = cv2.VideoCapture(source)
    if not cap.isOpened():
        raise ConnectionError(f"Could not open video source: {source}")
    interval = 1 / (cap.get(cv2.CAP_PROP_FPS) or 30) if realtime else 0
    try:
        next_time = time.monotonic()
        while True:
            ok, frame = cap.read()
            if not ok:
                return
            yield frame
            if interval:
                next_time += interval
                time.sleep(max(0.0, next_time - time.monotonic()))
    finally:
        cap.release()


def parse_source(source: str | int) -> str | int:
    """'0' -> webcam 0; everything else is a URL or file path."""
    if isinstance(source, str) and source.isdigit():
        return int(source)
    return source


@dataclass(frozen=True)
class Frame:
    image: np.ndarray
    index: int
    timestamp: float  # time.monotonic() when received


class FrameSource:
    """Reads frames in a background thread and keeps only the latest one.

    Network streams reconnect automatically with exponential backoff.
    Video files are played at their native frame rate and optionally looped.
    """

    def __init__(self, source: str | int, timeout_s: float = 5.0, reconnect_delay_s: float = 1.0,
                 max_reconnect_delay_s: float = 10.0, loop_files: bool = True):
        self.source = parse_source(source)
        self.timeout_s = timeout_s
        self.reconnect_delay_s = reconnect_delay_s
        self.max_reconnect_delay_s = max_reconnect_delay_s
        self.loop_files = loop_files

        self.is_url = isinstance(self.source, str) and self.source.startswith(("http://", "https://"))
        self.is_file = isinstance(self.source, str) and not self.is_url
        if self.is_file and not Path(self.source).is_file():
            raise FileNotFoundError(f"Video source not found: {self.source}")

        self.status = "stopped"
        self.last_error: str | None = None
        self._rate = RateMeter()
        self._latest: Frame | None = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    @property
    def fps(self) -> float:
        return self._rate.rate()

    def latest(self) -> Frame | None:
        with self._lock:
            return self._latest

    def start(self) -> "FrameSource":
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="FrameSource", daemon=True)
        self._thread.start()
        return self

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=self.timeout_s + 1)
        self.status = "stopped"

    def _open(self) -> Iterator[np.ndarray]:
        if self.is_url:
            return _mjpeg_frames(self.source, self.timeout_s)
        if self.is_file and Path(self.source).suffix.lower() in IMAGE_SUFFIXES:
            return _image_frames(Path(self.source))
        return _capture_frames(self.source, realtime=self.is_file)

    def _run(self) -> None:
        delay = self.reconnect_delay_s
        index = 0
        while not self._stop.is_set():
            self.status = "connecting"
            try:
                for image in self._open():
                    if self._stop.is_set():
                        return
                    index += 1
                    with self._lock:
                        self._latest = Frame(image, index, time.monotonic())
                    self._rate.tick()
                    self.status = "streaming"
                    self.last_error = None
                    delay = self.reconnect_delay_s
                if self.is_file:
                    if self.loop_files:
                        continue
                    self.status = "ended"
                    return
                raise ConnectionError("stream ended")
            except Exception as e:
                self.last_error = f"{type(e).__name__}: {e}"
                self.status = "reconnecting"
                log.warning("Camera source %s failed (%s); retrying in %.1fs", self.source, self.last_error, delay)
                self._stop.wait(delay)
                delay = min(delay * 2, self.max_reconnect_delay_s)
