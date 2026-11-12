"""Spray actuator test console, over the robot's Wi-Fi.

Needs firmware/servo_spray_test on the UNO and the shield switch on "cam".
Angles are press angles: 90 = rest (horns level), 0 = pressing, the same
numbers for both servos (the firmware mirrors servo 2). For a point-and-click
version, use the web app: python -m wd9000.web

Usage (on the robot's Wi-Fi, ELEGOO phone app closed):
    python scripts/spray_test.py
Against the simulator (in another terminal: python -m wd9000.robot.sim):
    python scripts/spray_test.py --host 127.0.0.1 --port 10100
"""

import argparse
import logging
import sys
import time

try:
    import readline  # noqa: F401  (arrow-key history on macOS/Linux)
except ImportError:
    pass

from wd9000.config import load_settings
from wd9000.robot import protocol
from wd9000.robot.client import RobotClient
from wd9000.robot.protocol import SprayPose, SprayServo
from wd9000.robot.spray import SprayHold

HELP = """\
Commands (servo 1 = left, servo 2 = right; angles 0-180, 90 = rest, 0 = pressing)
  both 135 | 1 135 | 2 135   move servo(s) to an angle (no press)
  both +5  | 1 -5  | 2 +5    nudge
  save rest | save press     store where the servos are now
  press 300                  press for 300 ms, then release
  on                         press and hold until you hit Enter
  off  (or just Enter)       release
  status                     show positions and settings
  max 2000                   hard cap on any press, ms (100-30000)
  speed 0                    move speed in deg/s (0 = instant, default 120)
  mirror 2 on|off            invert a servo's direction (servo 2 is on by default)
  defaults                   reset rest 90, press 0, mirror servo 2, max 2000, speed 120
  help | quit"""

SERVOS = {"1": SprayServo.LEFT, "left": SprayServo.LEFT, "2": SprayServo.RIGHT, "right": SprayServo.RIGHT,
          "both": SprayServo.BOTH, "12": SprayServo.BOTH}


class CommandError(Exception):
    pass


def describe(status: dict) -> str:
    mirrored = [f"servo {i + 1}" for i, inv in enumerate(status.get("inv", (0, 0))) if inv] or ["none"]
    pos, raw = status.get("pos", ("?", "?")), status.get("raw", ("?", "?"))
    rest, press = status.get("rest", ("?", "?")), status.get("press", ("?", "?"))
    speed = status.get("spd")
    lines = [
        f"  {status.get('st', '?'):<6} servo 1 = {pos[0]}   servo 2 = {pos[1]}   (raw {raw[0]} / {raw[1]})",
        f"  rest {rest[0]} / {rest[1]}   press {press[0]} / {press[1]}   mirrored: {', '.join(mirrored)}",
        f"  max {status.get('max')} ms   speed {'instant' if speed == 0 else f'{speed} deg/s'}"
        f"   boot #{status.get('boot')}",
    ]
    if status.get("cap"):
        lines.append("  hold hit the max time: send 'off' before pressing again")
    return "\n".join(lines)


class Console:
    def __init__(self, client: RobotClient):
        self.client = client

    def request(self, cmd: dict) -> str:
        value = self.client.request(cmd)
        if value in ("err", "unknown"):
            raise CommandError(f"robot rejected {cmd} ({value}). Is firmware/servo_spray_test on the UNO?")
        return value

    def status(self) -> dict:
        value = self.request(protocol.spray_status())
        if not value.startswith("st="):
            raise CommandError(f"unexpected status reply {value!r}. Is firmware/servo_spray_test on the UNO?")
        return protocol.parse_spray_status(value)

    def show(self) -> None:
        print(describe(self.status()))

    def run(self, line: str) -> bool:
        """Run one command line. Returns False to quit."""
        words = line.lower().split()
        if not words or words[0] == "off":
            self.request(protocol.spray(False))
        elif words[0] in ("q", "quit", "exit"):
            return False
        elif words[0] in ("h", "help", "?"):
            print(HELP)
            return True
        elif words[0] in SERVOS and len(words) == 2:
            self.jog(SERVOS[words[0]], words[1])
        elif words[0] == "save" and len(words) == 2 and words[1] in ("rest", "press"):
            self.request(protocol.spray_save(SprayPose.PRESS if words[1] == "press" else SprayPose.REST))
            print(f"  saved {words[1]} pose")
        elif words[0] in ("press", "p") and len(words) == 2:
            self.press_for(self.number(words[1]))
        elif words[0] == "on":
            self.hold()
        elif words[0] in ("s", "status"):
            pass
        elif words[0] == "max" and len(words) == 2:
            self.request(protocol.spray_max_on(self.number(words[1])))
        elif words[0] == "speed" and len(words) == 2:
            self.request(protocol.spray_speed(self.number(words[1])))
        elif words[0] == "mirror" and len(words) == 3 and words[1] in SERVOS and words[2] in ("on", "off"):
            self.request(protocol.spray_invert(SERVOS[words[1]], words[2] == "on"))
        elif words[0] == "defaults":
            self.request(protocol.spray_defaults())
        else:
            raise CommandError(f"unknown command {line.strip()!r} (type help)")
        self.show()
        return True

    @staticmethod
    def number(text: str) -> int:
        try:
            return int(text)
        except ValueError:
            raise CommandError(f"expected a number, got {text!r}") from None

    def jog(self, servo: SprayServo, arg: str) -> None:
        if arg[0] in "+-":
            pos = self.status()["pos"]
            delta = self.number(arg)
            for index in (0, 1):
                if servo & (1 << index):
                    angle = max(0, min(180, pos[index] + delta))
                    self.request(protocol.spray_jog(SprayServo(1 << index), angle))
        else:
            self.request(protocol.spray_jog(servo, self.number(arg)))

    def press_for(self, duration_ms: int) -> None:
        max_on = self.status()["max"]
        if duration_ms > max_on:
            raise CommandError(f"press is limited to max {max_on} ms (raise it with: max {duration_ms})")
        self.request(protocol.spray_press_for(duration_ms))
        print(f"  pressing for {duration_ms} ms")
        time.sleep(duration_ms / 1000 + 0.1)

    def hold(self) -> None:
        max_on = self.status()["max"]
        hold = SprayHold(self.client).start()
        try:
            input(f"  PRESSING. Hit Enter to release (auto-releases after {max_on} ms) ")
        finally:
            hold.stop()
        if hold.capped:
            print(f"  auto-released at {max_on} ms")
        if hold.error:
            print(f"  hold lost: {hold.error}")


def connect(client: RobotClient) -> bool:
    print(f"Connecting to {client.host}:{client.port} ...")
    try:
        client.connect()
    except OSError as e:
        print(f"  could not connect: {e}")
        print("  Check: on the robot's Wi-Fi? ELEGOO phone app closed (the ESP32 allows one client)?")
        return False
    return True


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host")
    parser.add_argument("--port", type=int)
    args = parser.parse_args()
    logging.basicConfig(level=logging.WARNING, format="%(levelname)s %(name)s: %(message)s")

    cfg = load_settings()["robot"]
    client = RobotClient(args.host or cfg["host"], args.port or cfg["command_port"], cfg["connect_timeout_s"],
                         cfg["reply_timeout_s"], cfg["link_timeout_s"], motion_enabled=False)
    if not connect(client):
        return 1
    console = Console(client)
    try:
        console.show()
    except (CommandError, TimeoutError, ConnectionError) as e:
        print(f"  {e}")
        print("  Check: firmware/servo_spray_test uploaded? Shield switch on 'cam'?")
    print("Type help for commands. Enter on an empty line releases. Ctrl+C releases and quits.")

    try:
        while True:
            line = input("spray> ")
            if not client.connected and not connect(client):
                continue
            try:
                if not console.run(line):
                    break
            except (CommandError, ValueError) as e:
                print(f"  {e}")
            except (TimeoutError, ConnectionError) as e:
                print(f"  link problem: {e}")
    except (KeyboardInterrupt, EOFError):
        print()
    finally:
        client.close()  # sends stop, which releases the spray
        print("Released and disconnected.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
