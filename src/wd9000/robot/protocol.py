"""ELEGOO Conqueror factory command protocol (raw TCP, port 100 on the ESP32).

Commands are compact JSON objects that the ESP32 relays verbatim to the UNO.
Everything coming back is a brace-delimited token: {Heartbeat} from the ESP32
once a second, and UNO replies such as {ok}, {c3_ok} or {c4_57} where c3/c4
is the "H" id we attached. Sources and caveats: docs/hardware.md.
"""

import json
from dataclasses import dataclass
from enum import IntEnum

HEARTBEAT = "{Heartbeat}"

# Drivetrain commands. RobotClient refuses these unless motion is enabled.
MOTION_COMMANDS = frozenset({1, 2, 3, 4, 101, 102, 103})


class Direction(IntEnum):
    """Direction codes for N:2 / N:3."""
    LEFT = 1
    RIGHT = 2
    FORWARD = 3
    BACKWARD = 4


class Light(IntEnum):
    ALL = 0
    LEFT = 1
    UPPER = 2
    RIGHT = 3
    LOWER = 4
    MIDDLE = 5


class Servo(IntEnum):
    PAN = 1  # pin 10, "z" in the firmware
    TILT = 2  # pin 11, "y" in the firmware
    BOTH = 3


# --- Spray actuator (our firmware/servo_spray_test, not the factory firmware) ---

# A lease press releases unless resent within this window.
SPRAY_LEASE_S = 0.5
SPRAY_REST_ANGLE = 90
SPRAY_PRESS_ANGLE = 0


class SprayServo(IntEnum):
    LEFT = 1  # servo 1
    RIGHT = 2  # servo 2, mirror-mounted
    BOTH = 3


class SprayPose(IntEnum):
    REST = 0
    PRESS = 1


def _check_range(name: str, value: int, low: int, high: int) -> int:
    if not low <= value <= high:
        raise ValueError(f"{name} must be in {low}..{high}, got {value}")
    return int(value)


def stop() -> dict:
    """Stop motors and clear the current mode. Always allowed."""
    return {"N": 100}


def set_light(r: int, g: int, b: int, light: Light = Light.ALL) -> dict:
    """Set the RGB LED until changed. Replies {id_ok}."""
    return {"N": 8, "D1": int(light), "D2": _check_range("r", r, 0, 255),
            "D3": _check_range("g", g, 0, 255), "D4": _check_range("b", b, 0, 255)}


def flash_light(r: int, g: int, b: int, duration_ms: int, light: Light = Light.ALL) -> dict:
    """Set the RGB LED for a limited time. No reply."""
    return {**set_light(r, g, b, light), "N": 7, "T": _check_range("duration_ms", duration_ms, 0, 60_000)}


def servo_angle(servo: Servo, angle: int) -> dict:
    """Move a gimbal servo to an absolute angle. Replies {id_ok}.

    The firmware reads D1=servo, D2=angle (the PDF's D1/D2/D3 is wrong) and
    blocks for 500 ms after each move.
    """
    return {"N": 5, "D1": int(servo), "D2": _check_range("angle", angle, 0, 180)}


def ultrasonic_distance() -> dict:
    """Read the ultrasonic distance in cm. Replies {id_<cm>}."""
    return {"N": 21, "D1": 2}


def drive(direction: Direction, speed: int) -> dict:
    """Drive until another command arrives. Motion command."""
    return {"N": 3, "D1": int(direction), "D2": _check_range("speed", speed, 0, 255)}


def drive_for(direction: Direction, speed: int, duration_ms: int) -> dict:
    """Drive for a limited time. Motion command, no reply."""
    return {"N": 2, "D1": int(direction), "D2": _check_range("speed", speed, 0, 255),
            "T": _check_range("duration_ms", duration_ms, 0, 20_000)}


def track_speeds(left: int, right: int) -> dict:
    """Set left/right track speeds directly (forward only). Motion command."""
    return {"N": 4, "D1": _check_range("left", left, 0, 255), "D2": _check_range("right", right, 0, 255)}


def spray(on: bool) -> dict:
    """Press (held only while resent within SPRAY_LEASE_S) or release. Replies ok / capped."""
    return {"N": 120, "D1": 1 if on else 0}


def spray_press_for(duration_ms: int) -> dict:
    """Press for a fixed time; the firmware releases. Must not exceed max_on_ms."""
    return {"N": 121, "D1": _check_range("duration_ms", duration_ms, 50, 30_000)}


def spray_jog(servo: SprayServo, angle: int) -> dict:
    """Move servo(s) to a press angle (90 rest, 0 pressing by default) without pressing."""
    return {"N": 122, "D1": int(servo), "D2": _check_range("angle", angle, 0, 180)}


def spray_save(pose: SprayPose) -> dict:
    """Store the servos' current positions as the rest or press pose."""
    return {"N": 123, "D1": int(pose)}


def spray_invert(servo: SprayServo, inverted: bool) -> dict:
    return {"N": 124, "D1": int(servo), "D2": int(inverted)}


def spray_max_on(duration_ms: int) -> dict:
    """Hard cap on any press."""
    return {"N": 125, "D1": _check_range("duration_ms", duration_ms, 100, 30_000)}


def spray_speed(deg_per_s: int) -> dict:
    """Servo move speed; 0 = instant."""
    return {"N": 126, "D1": _check_range("deg_per_s", deg_per_s, 0, 1000)}


def spray_status() -> dict:
    """Replies {id_st=REST;pos=90,90;...}; parse with parse_spray_status."""
    return {"N": 127}


def spray_defaults() -> dict:
    return {"N": 128}


def parse_spray_status(value: str) -> dict:
    """'st=REST;pos=90,90;max=2000' -> {'st': 'REST', 'pos': (90, 90), 'max': 2000}."""
    status = {}
    for item in value.split(";"):
        key, _, raw = item.partition("=")
        parts = raw.split(",")
        try:
            nums = tuple(int(x) for x in parts)
        except ValueError:
            status[key] = raw
            continue
        status[key] = nums if len(nums) > 1 else nums[0]
    return status


def is_motion(cmd: dict) -> bool:
    return cmd.get("N") in MOTION_COMMANDS


def encode(cmd: dict, command_id: str | None = None) -> bytes:
    """Serialize a command, optionally tagged with an "H" reply id."""
    if command_id is not None:
        cmd = {**cmd, "H": command_id}
    # The UNO parses into a 200-byte StaticJsonDocument, so keep it compact.
    return json.dumps(cmd, separators=(",", ":")).encode("ascii")


class TokenParser:
    """Splits the byte stream from the robot into {...} tokens."""

    def __init__(self, max_len: int = 256):
        self.max_len = max_len
        self._buf = ""

    def feed(self, data: bytes) -> list[str]:
        self._buf += data.decode("ascii", errors="replace")
        tokens = []
        while True:
            start = self._buf.find("{")
            if start < 0:
                self._buf = ""
                break
            end = self._buf.find("}", start)
            if end < 0:
                self._buf = self._buf[start:]
                if len(self._buf) > self.max_len:
                    self._buf = ""
                break
            tokens.append(self._buf[start:end + 1])
            self._buf = self._buf[end + 1:]
        return tokens


@dataclass(frozen=True)
class Reply:
    token: str
    command_id: str | None
    value: str


def parse_reply(token: str) -> Reply:
    """'{c3_ok}' -> Reply(id='c3', value='ok'); '{ok}' -> Reply(id=None, value='ok')."""
    body = token[1:-1]
    if "_" in body:
        command_id, value = body.split("_", 1)
        return Reply(token, command_id or None, value)
    return Reply(token, None, body)
