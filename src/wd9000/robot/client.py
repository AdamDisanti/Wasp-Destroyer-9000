"""TCP client for the robot's factory command channel.

Answers the ESP32's {Heartbeat} messages when they come. The kit's ESP32 source
sends one a second and drops clients that stop answering, but the ESP32 on
our robot (different firmware, see docs/hardware.md) sends none. So the
heartbeat watchdog only switches on once a heartbeat has been seen; without
one, a dead link shows up as a send error or a reply timeout instead.
"""

import itertools
import logging
import socket
import threading
import time
from typing import Callable

from wd9000.robot import protocol

log = logging.getLogger(__name__)


class MotionDisabledError(RuntimeError):
    pass


class RobotClient:
    def __init__(self, host: str, port: int = 100, connect_timeout_s: float = 5.0, reply_timeout_s: float = 2.0,
                 link_timeout_s: float = 3.5, motion_enabled: bool = False):
        self.host = host
        self.port = port
        self.connect_timeout_s = connect_timeout_s
        self.reply_timeout_s = reply_timeout_s
        self.link_timeout_s = link_timeout_s
        self.motion_enabled = motion_enabled
        # Tests and the bench failsafe check turn this off to simulate a hung client.
        self.answer_heartbeats = True
        self.on_disconnect: Callable[[str], None] | None = None

        self.last_heartbeat_at: float | None = None
        self.disconnect_reason: str | None = None
        self._sock: socket.socket | None = None
        self._send_lock = threading.Lock()
        self._pending: dict[str, list] = {}  # id -> [Event, value]
        self._pending_lock = threading.Lock()
        self._ids = itertools.count(1)
        self._reader: threading.Thread | None = None
        self._connected = threading.Event()
        self._heartbeats_seen = False  # on this connection

    @property
    def connected(self) -> bool:
        return self._connected.is_set()

    @property
    def heartbeats_seen(self) -> bool:
        return self._heartbeats_seen

    @property
    def heartbeat_age_s(self) -> float | None:
        return None if self.last_heartbeat_at is None else time.monotonic() - self.last_heartbeat_at

    def connect(self) -> "RobotClient":
        sock = socket.create_connection((self.host, self.port), timeout=self.connect_timeout_s)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        sock.settimeout(0.1)
        self._sock = sock
        self.disconnect_reason = None
        self.last_heartbeat_at = None
        self._heartbeats_seen = False
        self._connected.set()
        self._reader = threading.Thread(target=self._read_loop, name="RobotClient", daemon=True)
        self._reader.start()
        log.info("Connected to robot at %s:%d", self.host, self.port)
        return self

    def close(self) -> None:
        """Send stop (best effort) and disconnect."""
        if self.connected:
            try:
                self.stop()
            except OSError:
                pass
        self._drop("closed by client")
        if self._reader and self._reader is not threading.current_thread():
            self._reader.join(timeout=1)

    def __enter__(self) -> "RobotClient":
        return self.connect()

    def __exit__(self, *exc) -> None:
        self.close()

    def stop(self) -> None:
        """Stop the motors. Never gated."""
        self._write(protocol.encode(protocol.stop()))

    def send(self, cmd: dict) -> None:
        """Send without waiting for a reply."""
        self._check_motion(cmd)
        self._write(protocol.encode(cmd))

    def request(self, cmd: dict, timeout_s: float | None = None) -> str:
        """Send a command tagged with an id and wait for its reply value.

        Only for commands the firmware answers with {id_value} (see protocol.py).
        """
        self._check_motion(cmd)
        command_id = f"c{next(self._ids)}"
        entry = [threading.Event(), None]
        with self._pending_lock:
            self._pending[command_id] = entry
        try:
            self._write(protocol.encode(cmd, command_id))
            entry[0].wait(timeout_s or self.reply_timeout_s)
            if entry[1] is not None:
                return entry[1]
            if not self.connected:  # _drop() wakes waiters without a value
                raise ConnectionError(f"robot disconnected: {self.disconnect_reason}")
            raise TimeoutError(f"no reply to {cmd}")
        finally:
            with self._pending_lock:
                self._pending.pop(command_id, None)

    def _check_motion(self, cmd: dict) -> None:
        if protocol.is_motion(cmd) and not self.motion_enabled:
            raise MotionDisabledError(f"motion command refused (robot.motion_enabled is false): {cmd}")

    def _write(self, data: bytes) -> None:
        if not self.connected or self._sock is None:
            raise ConnectionError(f"not connected to robot ({self.disconnect_reason or 'never connected'})")
        with self._send_lock:
            try:
                self._sock.sendall(data)
            except OSError as e:
                self._drop(f"send failed: {e}")
                raise ConnectionError(f"send failed: {e}") from e

    def _drop(self, reason: str) -> None:
        if not self._connected.is_set():
            return
        self._connected.clear()
        self.disconnect_reason = reason
        if self._sock:
            try:
                self._sock.close()
            except OSError:
                pass
        with self._pending_lock:
            for entry in self._pending.values():
                entry[0].set()
        log.warning("Robot link down: %s", reason)
        if self.on_disconnect:
            self.on_disconnect(reason)

    def _read_loop(self) -> None:
        parser = protocol.TokenParser()
        while self.connected:
            try:
                data = self._sock.recv(1024)
                if not data:
                    self._drop("connection closed by robot")
                    return
            except socket.timeout:
                data = b""
            except OSError as e:
                self._drop(f"receive failed: {e}")
                return

            for token in parser.feed(data):
                if token == protocol.HEARTBEAT:
                    self.last_heartbeat_at = time.monotonic()
                    if not self._heartbeats_seen:
                        self._heartbeats_seen = True
                        log.info("Robot sends heartbeats; link watchdog on (%.1fs)", self.link_timeout_s)
                    if self.answer_heartbeats:
                        try:
                            self._write(protocol.HEARTBEAT.encode())
                        except ConnectionError:
                            return
                    continue
                reply = protocol.parse_reply(token)
                with self._pending_lock:
                    entry = self._pending.get(reply.command_id) if reply.command_id else None
                    if entry:
                        entry[1] = reply.value
                        entry[0].set()
                if not entry:
                    log.info("Message from robot: %s", token)  # e.g. {spray_boot 12} when the UNO resets

            if self._heartbeats_seen and self.heartbeat_age_s > self.link_timeout_s:
                self._drop(f"no heartbeat for {self.heartbeat_age_s:.1f}s")
                return
