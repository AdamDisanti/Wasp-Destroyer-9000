import time

import cv2
import numpy as np
import pytest

from wd9000.camera.stream import FrameSource, iter_mjpeg_jpegs, parse_source
from wd9000.utils.rate import RateMeter


def _jpeg(value: int) -> bytes:
    ok, buf = cv2.imencode(".jpg", np.full((16, 16, 3), value, dtype=np.uint8))
    assert ok
    return buf.tobytes()


def _elegoo_style_stream(jpegs: list[bytes]) -> bytes:
    # Mirrors the factory firmware: part headers, JPEG, then boundary (none before the first part).
    boundary = b"\r\n--123456789000000000000987654321\r\n"
    out = b""
    for j in jpegs:
        out += b"Content-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" % len(j) + j + boundary
    return out


@pytest.mark.parametrize("chunk_size", [1, 7, 4096])
def test_mjpeg_parser_handles_any_chunking(chunk_size):
    jpegs = [_jpeg(v) for v in (10, 120, 250)]
    data = _elegoo_style_stream(jpegs)
    chunks = (data[i:i + chunk_size] for i in range(0, len(data), chunk_size))
    assert list(iter_mjpeg_jpegs(chunks)) == jpegs


def test_mjpeg_parser_ignores_trailing_partial_frame():
    full = _jpeg(50)
    data = _elegoo_style_stream([full]) + full[: len(full) // 2]
    assert list(iter_mjpeg_jpegs([data])) == [full]


def test_mjpeg_parser_drops_oversized_garbage():
    data = b"\xff\xd8" + b"\x00" * 100 + _jpeg(5)
    # Second SOI is inside the oversized partial frame; it must not crash or hang.
    assert len(list(iter_mjpeg_jpegs([data[:50], data[50:]], max_buffer=40))) <= 1


def test_parse_source():
    assert parse_source("0") == 0
    assert parse_source(1) == 1
    assert parse_source("http://192.168.4.1:81/stream") == "http://192.168.4.1:81/stream"


def test_rate_meter_measures_and_decays():
    meter = RateMeter(window_s=2.0)
    for i in range(11):
        meter.tick(now=i * 0.1)  # 10 Hz for 1 s
    assert meter.rate(now=1.0) == pytest.approx(10, rel=0.01)
    assert meter.rate(now=1.5) < 10  # stalled -> decays
    assert meter.rate(now=10.0) == 0.0


def test_frame_source_missing_file():
    with pytest.raises(FileNotFoundError):
        FrameSource("does/not/exist.mp4")


def _wait_for(predicate, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.02)
    return False


def test_frame_source_plays_video_file(tmp_path):
    path = tmp_path / "clip.avi"
    writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"MJPG"), 30, (64, 48))
    for v in range(10):
        writer.write(np.full((48, 64, 3), v * 20, dtype=np.uint8))
    writer.release()

    source = FrameSource(str(path), loop_files=False).start()
    try:
        assert _wait_for(lambda: source.status == "ended")
        frame = source.latest()
        assert frame is not None and frame.index == 10
        assert frame.image.shape == (48, 64, 3)
    finally:
        source.stop()


def test_frame_source_reconnects_after_failure(monkeypatch):
    attempts = []

    def flaky_open(self):
        attempts.append(1)
        if len(attempts) == 1:
            raise ConnectionError("boom")
        return iter([np.zeros((4, 4, 3), dtype=np.uint8)])

    monkeypatch.setattr(FrameSource, "_open", flaky_open)
    source = FrameSource("http://example.invalid/stream", reconnect_delay_s=0.01).start()
    try:
        assert _wait_for(lambda: source.latest() is not None)
        assert len(attempts) >= 2
    finally:
        source.stop()
