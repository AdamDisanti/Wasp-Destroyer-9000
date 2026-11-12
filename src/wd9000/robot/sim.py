"""Simulated robot speaking the factory protocol, for tests and offline dev.

Mirrors the ESP32 firmware's TCP server: one client at a time, a {Heartbeat}
every interval, the client dropped after 3 missed heartbeats, and a stop sent
to the "UNO" whenever a client leaves. Replies mimic the UNO firmware.

Run standalone:  python -m wd9000.robot.sim --port 10100
"""

import argparse
import json
import logging
import socket
import threading
import time
from dataclasses import dataclass, field

from wd9000.robot import protocol

log = logging.getLogger(__name__)


@dataclass
class SimSpray:
    """Mirrors firmware/servo_spray_test. Angles are press angles."""
    state: str = "REST"
    pos: list[int] = field(default_factory=lambda: [90, 90])
    rest: list[int] = field(default_factory=lambda: [90, 90])
    press: list[int] = field(default_factory=lambda: [0, 0])
    invert: list[int] = field(default_factory=lambda: [0, 1])
    max_on_ms: int = 2000
    speed: int = 120
    capped: bool = False
    presses: int = 0
    on_since: float = 0.0
    lease_until: float | None = None
    pulse_s: float | None = None

    def start_press(self, now: float, lease_s: float | None, pulse_s: float | None) -> None:
        if self.state != "PRESS":
            self.on_since = now
            self.presses += 1
        self.state = "PRESS"
        self.lease_until = now + lease_s if lease_s else None
        self.pulse_s = pulse_s
        self.pos = list(self.press)

    def release(self) -> None:
        self.state = "REST"
        self.lease_until = self.pulse_s = None
        self.pos = list(self.rest)

    def update(self, now: float) -> None:
        if self.state != "PRESS":
            return
        held = now - self.on_since
        if held >= self.max_on_ms / 1000:
            if self.lease_until is not None:
                self.capped = True
            self.release()
        elif self.lease_until is not None and now >= self.lease_until:
            self.release()
        elif self.pulse_s is not None and held >= self.pulse_s:
            self.release()

    def status(self) -> str:
        raw = [180 - p if inv else p for p, inv in zip(self.pos, self.invert)]
        pairs = {"pos": self.pos, "raw": raw, "rest": self.rest, "press": self.press, "inv": self.invert}
        return ";".join([f"st={self.state}", *(f"{k}={v[0]},{v[1]}" for k, v in pairs.items()),
                         f"max={self.max_on_ms}", f"spd={self.speed}", f"cap={int(self.capped)}", "boot=1",
                         "vcc=5000", "vmin=5000", "bat=8000", "bmin=8000"])


@dataclass
class SimState:
    light: tuple[int, int, int] = (0, 0, 0)
    servo: dict[int, int] = field(default_factory=lambda: {1: 90, 2: 90})
    motion: dict | None = None  # last motion command, None when stopped
    stops: int = 0
    commands: list[dict] = field(default_factory=list)
    clients_served: int = 0
    spray: SimSpray = field(default_factory=SimSpray)


class SimRobot:
    def __init__(self, host: str = "127.0.0.1", port: int = 0, heartbeat_interval_s: float = 1.0,
                 missed_heartbeats_allowed: int = 3, distance_cm: int = 57,
                 spray_lease_s: float = protocol.SPRAY_LEASE_S, send_heartbeats: bool = True):
        self.heartbeat_interval_s = heartbeat_interval_s
        self.missed_heartbeats_allowed = missed_heartbeats_allowed
        self.distance_cm = distance_cm
        self.spray_lease_s = spray_lease_s
        self.send_heartbeats = send_heartbeats  # False mimics the ESP32 on our robot
        self.state = SimState()
        self._server = socket.create_server((host, port))
        self._server.settimeout(0.1)
        self.host, self.port = self._server.getsockname()[:2]
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> "SimRobot":
        self._thread = threading.Thread(target=self._serve, name="SimRobot", daemon=True)
        self._thread.start()
        return self

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2)
        self._server.close()

    def __enter__(self) -> "SimRobot":
        return self.start()

    def __exit__(self, *exc) -> None:
        self.stop()

    def _serve(self) -> None:
        while not self._stop.is_set():
            try:
                conn, addr = self._server.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            self.state.clients_served += 1
            log.info("[Client connected] %s", addr)
            try:
                self._handle_client(conn)
            finally:
                self._apply_stop()  # firmware always sends {"N":100} on disconnect
                conn.close()
                log.info("[Client disconnected]")

    def _handle_client(self, conn: socket.socket) -> None:
        conn.settimeout(0.02)
        parser = protocol.TokenParser()
        got_heartbeat = False
        missed = 0
        next_beat = time.monotonic()  # the ESP32's beat timer is global, so the first beat can come at once
        while not self._stop.is_set():
            self.state.spray.update(time.monotonic())
            try:
                data = conn.recv(1024)
                if not data:
                    return
            except socket.timeout:
                data = b""
            except OSError:
                return
            for token in parser.feed(data.replace(b" ", b"")):
                if token == protocol.HEARTBEAT:
                    got_heartbeat = True
                    continue
                reply = self._handle_command(token)
                if reply:
                    try:
                        conn.sendall(reply.encode())
                    except OSError:
                        return

            if self.send_heartbeats and time.monotonic() >= next_beat:
                next_beat += self.heartbeat_interval_s
                try:
                    conn.sendall(protocol.HEARTBEAT.encode())
                except OSError:
                    return
                if got_heartbeat:
                    got_heartbeat, missed = False, 0
                else:
                    missed += 1
                if missed > self.missed_heartbeats_allowed:
                    log.info("Client missed %d heartbeats; dropping", missed)
                    return

    def _apply_stop(self) -> None:
        self.state.motion = None
        self.state.stops += 1
        self.state.spray.capped = False
        self.state.spray.release()

    def _handle_spray(self, n: int, cmd: dict) -> str:
        sp, now = self.state.spray, time.monotonic()
        sp.update(now)
        d1, d2 = cmd.get("D1"), cmd.get("D2")
        servos = [i for i in (0, 1) if isinstance(d1, int) and d1 & (1 << i)]
        if n == 120 and d1 in (0, 1):
            if d1 == 0:
                sp.capped = False
                sp.release()
            elif sp.capped:
                return "capped"
            else:
                sp.start_press(now, self.spray_lease_s, None)
        elif n == 121 and isinstance(d1, int) and 50 <= d1 <= sp.max_on_ms:
            sp.start_press(now, None, d1 / 1000)
        elif n == 122 and servos and isinstance(d2, int) and 0 <= d2 <= 180:
            for i in servos:
                sp.pos[i] = d2
            sp.state, sp.lease_until, sp.pulse_s = "MANUAL", None, None
        elif n == 123 and d1 in (0, 1):
            (sp.press if d1 else sp.rest)[:] = sp.pos
        elif n == 124 and servos and d2 in (0, 1):
            for i in servos:
                sp.invert[i] = d2
        elif n == 125 and isinstance(d1, int) and 100 <= d1 <= 30_000:
            sp.max_on_ms = d1
        elif n == 126 and isinstance(d1, int) and 0 <= d1 <= 1000:
            sp.speed = d1
        elif n == 127:
            return sp.status()
        elif n == 128:
            self.state.spray = SimSpray(presses=sp.presses)
        else:
            return "err"
        return "ok"

    def _handle_command(self, token: str) -> str | None:
        try:
            cmd = json.loads(token)
        except json.JSONDecodeError:
            return None
        self.state.commands.append(cmd)
        n, h = cmd.get("N"), cmd.get("H", "")
        if n == 100:
            self._apply_stop()
            return "{ok}"
        if n == 110:
            self._apply_stop()
            return f"{{{h}_ok}}"
        if n in (7, 8):
            self.state.light = (cmd.get("D2", 0), cmd.get("D3", 0), cmd.get("D4", 0))
            return f"{{{h}_ok}}" if n == 8 else None
        if n == 5:
            for s in (1, 2) if cmd.get("D1") == 3 else (cmd.get("D1"),):
                self.state.servo[s] = max(0, min(180, cmd.get("D2", 90)))
            return f"{{{h}_ok}}"
        if n in protocol.MOTION_COMMANDS:
            self.state.motion = cmd
            return f"{{{h}_ok}}" if n in (1, 3, 4) else None
        if 120 <= n <= 128:
            return f"{{{h}_{self._handle_spray(n, cmd)}}}" if h else None
        if n == 21:
            if cmd.get("D1") == 2:
                return f"{{{h}_{self.distance_cm}}}"
            return f"{{{h}_{'true' if self.distance_cm < 20 else 'false'}}}"
        return None


def main() -> None:
    parser = argparse.ArgumentParser(description="Simulated ELEGOO robot command server")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=10100)
    parser.add_argument("--no-heartbeats", action="store_true", help="behave like our robot's ESP32 (sends none)")
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    robot = SimRobot(args.host, args.port, send_heartbeats=not args.no_heartbeats).start()
    log.info("Simulated robot listening on %s:%d (Ctrl+C to quit)", robot.host, robot.port)
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        robot.stop()


if __name__ == "__main__":
    main()
