import json
import socket
import threading
import time
import urllib.error
import urllib.request

import pytest

from wd9000.camera.stream import FrameSource
from wd9000.config import PROJECT_ROOT
from wd9000.robot.client import RobotClient
from wd9000.robot.sim import SimRobot
from wd9000.vision.detector import MockDetector
from wd9000.vision.worker import InferenceWorker
from wd9000.web.server import CameraService, SprayService, WebServer

SAMPLE = PROJECT_ROOT / "samples" / "paper_wasp_nest.jpg"


def _serve(robot_port: int):
    camera = FrameSource(str(SAMPLE)).start()
    worker = InferenceWorker(MockDetector(), camera.latest).start()
    spray = SprayService(RobotClient("127.0.0.1", robot_port, connect_timeout_s=1, reply_timeout_s=1))
    server = WebServer(("127.0.0.1", 0), CameraService(camera, worker, "mock", 2.0), spray)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return server


def _stop(server):
    server.shutdown()
    server.server_close()
    server.spray.close()
    server.camera.stop()


@pytest.fixture
def web():
    with SimRobot(spray_lease_s=0.3) as robot:
        server = _serve(robot.port)
        yield server, robot
        _stop(server)


def _call(server, path, body=None, headers=None):
    url = f"http://127.0.0.1:{server.server_address[1]}{path}"
    data = None if body is None else json.dumps(body).encode()
    req = urllib.request.Request(url, data=data, headers={"Content-Type": "application/json", **(headers or {})})
    try:
        with urllib.request.urlopen(req, timeout=5) as r:
            return r.status, json.loads(r.read())
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read())


def _wait_for(predicate, timeout=3.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.02)
    return False


def test_index_page(web):
    server, _ = web
    with urllib.request.urlopen(f"http://127.0.0.1:{server.server_address[1]}/", timeout=5) as r:
        assert b"Spray test" in r.read()


def test_spray_status_and_jog(web):
    server, _ = web
    status, data = _call(server, "/api/spray")
    assert data["connected"] and data["status"]["press"] == [0, 0]
    status, data = _call(server, "/api/spray/jog", {"servo": 3, "angle": 45})
    assert status == 200
    assert data["status"]["pos"] == [45, 45]
    assert data["status"]["raw"] == [45, 135]  # servo 2 mirrored


def test_hold_release_and_timed_press(web):
    server, robot = web
    assert _call(server, "/api/spray/hold", {})[1]["value"] == "ok"
    assert robot.state.spray.state == "PRESS"
    _call(server, "/api/spray/release", {})
    assert robot.state.spray.state == "REST"

    _call(server, "/api/spray/hold", {})
    assert _wait_for(lambda: robot.state.spray.state == "REST", timeout=1)  # lease ran out without resends

    _call(server, "/api/spray/press", {"ms": 100})
    assert robot.state.spray.state == "PRESS"
    assert _wait_for(lambda: _call(server, "/api/spray")[1]["status"]["st"] == "REST")


def test_save_and_goto(web):
    server, _ = web
    _call(server, "/api/spray/jog", {"servo": 1, "angle": 10})
    _call(server, "/api/spray/jog", {"servo": 2, "angle": 5})
    assert _call(server, "/api/spray/save", {"pose": "press"})[1]["status"]["press"] == [10, 5]
    _call(server, "/api/spray/goto", {"pose": "rest"})
    assert _call(server, "/api/spray/goto", {"pose": "press"})[1]["status"]["pos"] == [10, 5]
    data = _call(server, "/api/spray/settings", {"max_ms": 3000, "speed": 0})[1]
    assert (data["status"]["max"], data["status"]["spd"]) == (3000, 0)


@pytest.mark.parametrize("path, body, headers, code", [
    ("/api/spray/hold", {}, {"Content-Type": "text/plain"}, 415),
    ("/api/spray/hold", {}, {"Origin": "http://evil.example"}, 403),
    ("/api/spray/hold", {}, {"Host": "evil.example:8000"}, 403),
    ("/api/spray/jog", {"servo": 3, "angle": 181}, {}, 400),
    ("/api/spray/jog", {"servo": 3}, {}, 400),
    ("/api/spray/explode", {}, {}, 400),
])
def test_bad_requests_rejected(web, path, body, headers, code):
    server, robot = web
    assert _call(server, path, body, headers)[0] == code
    assert robot.state.spray.state == "REST"


def test_camera_stream_and_status(web):
    server, _ = web
    with urllib.request.urlopen(f"http://127.0.0.1:{server.server_address[1]}/video.mjpg", timeout=5) as r:
        assert r.headers.get_content_type() == "multipart/x-mixed-replace"
        chunk = r.read(4096)
    assert b"--frame" in chunk and b"\xff\xd8" in chunk
    assert _wait_for(lambda: _call(server, "/api/camera")[1]["detection"]["detections"])
    assert _call(server, "/api/camera/detect", {"enabled": False})[1]["detection"]["enabled"] is False


def test_robot_unreachable_reports_error():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        dead_port = s.getsockname()[1]
    server = _serve(dead_port)
    try:
        status, data = _call(server, "/api/spray")
        assert status == 200 and not data["connected"] and "can't reach" in data["error"]
        assert _call(server, "/api/spray/hold", {})[0] == 503
    finally:
        _stop(server)


def test_frame_polling(web):
    server, _ = web
    base = f"http://127.0.0.1:{server.server_address[1]}/frame.jpg"
    with urllib.request.urlopen(base, timeout=5) as r:
        key = r.headers["X-Frame-Key"]
        assert r.headers.get_content_type() == "image/jpeg" and r.read(2) == b"\xff\xd8"
    with urllib.request.urlopen(f"{base}?after={key}", timeout=5) as r:
        assert r.status in (200, 204)  # a newer frame, or none within the wait
        if r.status == 200:
            assert r.headers["X-Frame-Key"] != key
