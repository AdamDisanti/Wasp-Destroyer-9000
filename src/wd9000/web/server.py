"""Local web app: live camera with nest detection, and a spray actuator test panel.

Usage:
    python -m wd9000.web                       # then open http://localhost:8000
    python -m wd9000.web --source samples/paper_wasp_nest.jpg --backend mock --robot 127.0.0.1:10100

Listens on localhost only: anyone who can reach it can press the spray.
"""

import argparse
import json
import logging
import threading
import time
import urllib.parse
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import cv2

from wd9000.camera.stream import FrameSource
from wd9000.config import get_api_key, load_settings
from wd9000.robot import protocol
from wd9000.robot.client import RobotClient
from wd9000.robot.protocol import SprayPose, SprayServo
from wd9000.vision.detector import make_detector
from wd9000.vision.visualization import draw_detections
from wd9000.vision.worker import InferenceWorker

log = logging.getLogger("web")

SUPPLY_WARN_MV = 4500  # the ESP32 camera board shares this 5 V rail and resets on dips
BATTERY_LOW_MV = 7000  # the kit firmware's low-battery threshold (2-cell pack)

STATIC = Path(__file__).parent / "static"
JPEG_QUALITY = 80
STREAM_POLL_S = 0.02
FRAME_WAIT_S = 1.0  # /frame.jpg waits this long for a new frame before answering 204


class CameraService:
    """Camera frames with detection boxes drawn on, encoded once per change for all viewers."""

    def __init__(self, camera: FrameSource, worker: InferenceWorker | None, backend: str, stale_s: float):
        self.camera = camera
        self.worker = worker
        self.backend = backend
        self.stale_s = stale_s
        self._lock = threading.Lock()
        self._key = None
        self._jpeg: bytes | None = None

    def jpeg(self) -> tuple[str, bytes] | None:
        """(change key, JPEG bytes) for the latest frame, or None before the first frame."""
        frame = self.camera.latest()
        if frame is None:
            return None
        result = self._fresh_result()
        key = f"{frame.index}-{result.frame_index if result else 'none'}"
        with self._lock:
            if key != self._key:
                image = draw_detections(frame.image, result.detections) if result else frame.image
                ok, buf = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
                if ok:
                    self._key, self._jpeg = key, buf.tobytes()
            return self._key, self._jpeg

    def _fresh_result(self):
        if self.worker is None or not self.worker.enabled:
            return None
        result = self.worker.latest()
        if result is None or time.monotonic() - result.completed_at > self.stale_s:
            return None
        return result

    def status(self) -> dict:
        frame = self.camera.latest()
        status = {
            "source": str(self.camera.source),
            "status": self.camera.status,
            "fps": round(self.camera.fps, 1),
            "frame_age_s": None if frame is None else round(time.monotonic() - frame.timestamp, 1),
            "error": self.camera.last_error,
            "detection": None,
        }
        if self.worker is not None:
            result = self._fresh_result()
            status["detection"] = {
                "backend": self.backend,
                "enabled": self.worker.enabled,
                "fps": round(self.worker.fps, 1),
                "latency_ms": None if result is None else round(result.latency_s * 1000),
                "error": self.worker.last_error,
                "detections": [] if result is None else
                [{"label": d.label, "confidence": round(d.confidence, 3)} for d in result.detections],
            }
        return status

    def stop(self) -> None:
        if self.worker:
            self.worker.stop()
        self.camera.stop()


class RobotUnavailable(Exception):
    pass


class SprayService:
    """Owns the single command connection to the robot (the ESP32 allows one client)."""

    RETRY_S = 2.0

    def __init__(self, client: RobotClient):
        self.client = client
        self.last_error: str | None = None
        self._connect_lock = threading.Lock()
        self._next_attempt = 0.0
        self.last_boot: int | None = None  # UNO boot counter; a jump means it reset (usually a brownout)

    def _ensure_connected(self) -> None:
        if self.client.connected:
            return
        if not self._connect_lock.acquire(blocking=False):
            raise RobotUnavailable("connecting to the robot ...")
        try:
            if self.client.connected:
                return
            if time.monotonic() < self._next_attempt:
                raise RobotUnavailable(self.last_error or "robot not connected")
            try:
                self.client.connect()
                self.last_error = None
            except OSError as e:
                self.last_error = (f"can't reach the robot at {self.client.host}:{self.client.port} ({e}). "
                                   "On the robot's Wi-Fi? ELEGOO phone app closed?")
                self._next_attempt = time.monotonic() + self.RETRY_S
                raise RobotUnavailable(self.last_error) from None
        finally:
            self._connect_lock.release()

    def request(self, cmd: dict) -> str:
        self._ensure_connected()
        try:
            value = self.client.request(cmd)
        except TimeoutError:
            # Without heartbeats a silent link only shows up here; reconnect on the next request.
            self.client.close()
            self.last_error = ("the robot didn't answer. Is firmware/servo_spray_test on the UNO and the "
                               "shield switch on 'cam'? Reconnecting ...")
            raise RobotUnavailable(self.last_error) from None
        except ConnectionError as e:
            self.last_error = str(e)
            raise RobotUnavailable(self.last_error) from None
        if value in ("err", "unknown"):
            raise ValueError(f"the robot rejected {cmd} ({value}). Is firmware/servo_spray_test on the UNO?")
        return value

    def status(self) -> dict:
        value = self.request(protocol.spray_status())
        if not value.startswith("st="):
            raise ValueError(f"unexpected reply {value!r}. Is firmware/servo_spray_test on the UNO?")
        status = protocol.parse_spray_status(value)
        boot = status.get("boot")
        if isinstance(boot, int):
            if self.last_boot is not None and boot != self.last_boot:
                log.warning("UNO restarted (boot #%d -> #%d). If this follows a servo move, the servos are "
                            "browning out the board: power them from a separate supply.", self.last_boot, boot)
            self.last_boot = boot
        vmin, bmin = status.get("vmin"), status.get("bmin")
        if isinstance(vmin, int) and vmin < SUPPLY_WARN_MV:
            log.warning("5 V supply dipped to %.2f V (now %.2f V). The servos are pulling it down.",
                        vmin / 1000, status.get("vcc", 0) / 1000)
        if isinstance(bmin, int) and bmin < BATTERY_LOW_MV:
            log.warning("Battery dipped to %.2f V (now %.2f V). Below %.1f V it needs charging.",
                        bmin / 1000, status.get("bat", 0) / 1000, BATTERY_LOW_MV / 1000)
        return status

    def handle(self, action: str, body: dict) -> dict:
        """Run one API action; returns the reply value plus fresh status."""
        if action == "jog":
            value = self.request(protocol.spray_jog(SprayServo(int(body["servo"])), int(body["angle"])))
        elif action == "save":
            value = self.request(protocol.spray_save(SprayPose[str(body["pose"]).upper()]))
        elif action == "goto":
            pose = self.status()["rest" if body["pose"] == "rest" else "press"]
            self.request(protocol.spray_jog(SprayServo.LEFT, pose[0]))
            value = self.request(protocol.spray_jog(SprayServo.RIGHT, pose[1]))
        elif action == "press":
            value = self.request(protocol.spray_press_for(int(body["ms"])))
        elif action == "hold":
            value = self.request(protocol.spray(True))
            return {"value": value}  # sent several times a second; skip the status round trip
        elif action == "release":
            value = self.request(protocol.spray(False))
        elif action == "settings":
            if "max_ms" in body:
                self.request(protocol.spray_max_on(int(body["max_ms"])))
            if "speed" in body:
                self.request(protocol.spray_speed(int(body["speed"])))
            value = "ok"
        else:
            raise KeyError(action)
        return {"value": value, "status": self.status()}

    def close(self) -> None:
        self.client.close()


class Handler(BaseHTTPRequestHandler):
    server: "WebServer"

    def log_message(self, fmt, *args) -> None:
        log.debug("%s " + fmt, self.address_string(), *args)

    def _allowed_hosts(self) -> set[str]:
        port = self.server.server_address[1]
        return {f"localhost:{port}", f"127.0.0.1:{port}"}

    def _check_host(self) -> bool:
        # Blocks DNS-rebinding: only pages served from localhost may talk to this server.
        if self.headers.get("Host") not in self._allowed_hosts():
            self._send_json({"error": "forbidden host"}, HTTPStatus.FORBIDDEN)
            return False
        return True

    def _send_json(self, data: dict, status: HTTPStatus = HTTPStatus.OK) -> None:
        body = json.dumps(data).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self) -> None:
        if not self._check_host():
            return
        path = self.path.split("?", 1)[0]
        if path in ("/", "/index.html"):
            body = (STATIC / "index.html").read_bytes()
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif path == "/frame.jpg":
            after = urllib.parse.parse_qs(urllib.parse.urlsplit(self.path).query).get("after", [""])[0]
            self._send_frame(after)
        elif path == "/video.mjpg":
            self._stream_video()
        elif path == "/api/camera":
            self._send_json(self.server.camera.status())
        elif path == "/api/spray":
            spray = self.server.spray
            try:
                self._send_json({"connected": True, "status": spray.status()})
            except (RobotUnavailable, ValueError) as e:
                self._send_json({"connected": spray.client.connected, "error": str(e)})
        else:
            self._send_json({"error": "not found"}, HTTPStatus.NOT_FOUND)

    def do_POST(self) -> None:
        if not self._check_host():
            return
        # Cross-site pages can't send JSON here without a CORS preflight, which we never approve.
        origin = self.headers.get("Origin")
        if origin is not None and origin.split("://", 1)[-1] not in self._allowed_hosts():
            self._send_json({"error": "forbidden origin"}, HTTPStatus.FORBIDDEN)
            return
        if self.headers.get_content_type() != "application/json":
            self._send_json({"error": "expected application/json"}, HTTPStatus.UNSUPPORTED_MEDIA_TYPE)
            return
        try:
            length = int(self.headers.get("Content-Length", 0))
            body = json.loads(self.rfile.read(length) or b"{}") if length <= 10_000 else None
            if not isinstance(body, dict):
                raise ValueError("expected a JSON object")
        except ValueError as e:
            self._send_json({"error": f"bad request: {e}"}, HTTPStatus.BAD_REQUEST)
            return

        path = self.path.split("?", 1)[0]
        if path == "/api/camera/detect":
            worker = self.server.camera.worker
            if worker is None:
                self._send_json({"error": "detection is off (started with --no-detect)"}, HTTPStatus.CONFLICT)
                return
            worker.enabled = bool(body.get("enabled"))
            self._send_json(self.server.camera.status())
        elif path.startswith("/api/spray/"):
            try:
                self._send_json(self.server.spray.handle(path.rsplit("/", 1)[1], body))
            except KeyError as e:
                self._send_json({"error": f"unknown action or missing field {e}"}, HTTPStatus.BAD_REQUEST)
            except ValueError as e:
                self._send_json({"error": str(e)}, HTTPStatus.BAD_REQUEST)
            except RobotUnavailable as e:
                self._send_json({"error": str(e)}, HTTPStatus.SERVICE_UNAVAILABLE)
        else:
            self._send_json({"error": "not found"}, HTTPStatus.NOT_FOUND)

    def _send_frame(self, after: str) -> None:
        """Latest annotated frame, waiting briefly for one newer than `after`. The page polls
        this instead of holding an MJPEG stream open, so it recovers when the server restarts."""
        deadline = time.monotonic() + FRAME_WAIT_S
        while True:
            latest = self.server.camera.jpeg()
            if latest is not None and latest[0] != after:
                break
            if time.monotonic() >= deadline or self.server.stopping.is_set():
                self.send_response(HTTPStatus.NO_CONTENT)
                self.end_headers()
                return
            time.sleep(STREAM_POLL_S)
        key, jpeg = latest
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Content-Length", str(len(jpeg)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Frame-Key", key)
        self.end_headers()
        try:
            self.wfile.write(jpeg)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _stream_video(self) -> None:
        """MJPEG stream of the annotated video, for VLC or a plain <img>."""
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        last_key = None
        try:
            while not self.server.stopping.is_set():
                latest = self.server.camera.jpeg()
                if latest is None or latest[0] == last_key:
                    time.sleep(STREAM_POLL_S)
                    continue
                last_key, jpeg = latest
                self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                 + str(len(jpeg)).encode() + b"\r\n\r\n" + jpeg + b"\r\n")
        except (BrokenPipeError, ConnectionResetError):
            pass  # viewer closed the tab


class WebServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, address: tuple[str, int], camera: CameraService, spray: SprayService):
        super().__init__(address, Handler)
        self.camera = camera
        self.spray = spray
        self.stopping = threading.Event()

    def shutdown(self) -> None:
        self.stopping.set()
        super().shutdown()


def parse_host_port(text: str) -> tuple[str, int]:
    host, _, port = text.rpartition(":")
    return host, int(port)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", type=int, default=8000, help="web page port (default 8000)")
    parser.add_argument("--source", help="camera stream URL, video/image file, or webcam index")
    parser.add_argument("--backend", choices=["cloud", "local", "mock"], help="inference backend")
    parser.add_argument("--no-detect", action="store_true", help="video only, no inference")
    parser.add_argument("--robot", help="robot command server as host:port (default from settings.yaml)")
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")

    settings = load_settings()
    cam_cfg, view_cfg, robot_cfg = settings["camera"], settings.get("viewer", {}), settings["robot"]
    if args.backend:
        settings["inference"]["backend"] = args.backend
    backend = settings["inference"]["backend"]

    try:
        camera = FrameSource(args.source or cam_cfg["source"], timeout_s=cam_cfg.get("timeout_s", 5),
                             reconnect_delay_s=cam_cfg.get("reconnect_delay_s", 1),
                             loop_files=cam_cfg.get("loop_video_files", True)).start()
    except FileNotFoundError as e:
        log.error("%s", e)
        return 1
    worker = None
    if not args.no_detect:
        try:
            detector = make_detector(settings, get_api_key())
        except Exception as e:
            log.error("Could not start %s inference backend: %s", backend, e)
            camera.stop()
            return 1
        worker = InferenceWorker(detector, camera.latest, view_cfg.get("inference_interval_s", 0)).start()
    camera_service = CameraService(camera, worker, backend, view_cfg.get("detection_stale_s", 2.0))

    host, port = (parse_host_port(args.robot) if args.robot
                  else (robot_cfg["host"], robot_cfg["command_port"]))
    client = RobotClient(host, port, robot_cfg["connect_timeout_s"], robot_cfg["reply_timeout_s"],
                         robot_cfg["link_timeout_s"], motion_enabled=False)
    spray = SprayService(client)

    server = WebServer(("127.0.0.1", args.port), camera_service, spray)
    log.info("Open http://localhost:%d  (camera: %s, detection: %s, robot: %s:%d)  Ctrl+C to quit",
             args.port, camera.source, "off" if worker is None else backend, host, port)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.stopping.set()
        server.server_close()
        spray.close()  # sends stop, which releases the spray
        camera_service.stop()
    return 0
