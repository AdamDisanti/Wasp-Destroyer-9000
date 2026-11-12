"""Holding the spray actuator pressed over the Wi-Fi link.

The firmware only holds a press while it keeps hearing {"N":120,"D1":1}
(a lease), so a crashed or disconnected client can't leave the can spraying.
SprayHold resends it in the background until stopped.
"""

import threading

from wd9000.robot import protocol
from wd9000.robot.client import RobotClient


class SprayHold:
    def __init__(self, client: RobotClient, interval_s: float = protocol.SPRAY_LEASE_S / 3):
        self.client = client
        self.interval_s = interval_s
        self.capped = False  # firmware hit max_on_ms and refused to keep pressing
        self.error: Exception | None = None
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    @property
    def active(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def start(self) -> "SprayHold":
        self._thread = threading.Thread(target=self._run, name="SprayHold", daemon=True)
        self._thread.start()
        return self

    def stop(self) -> None:
        """Stop resending and release."""
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2)
        if self.client.connected:
            self.client.request(protocol.spray(False))

    def __enter__(self) -> "SprayHold":
        return self.start()

    def __exit__(self, *exc) -> None:
        self.stop()

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                if self.client.request(protocol.spray(True)) == "capped":
                    self.capped = True
                    return
            except (ConnectionError, TimeoutError) as e:
                self.error = e
                return
            self._stop.wait(self.interval_s)
