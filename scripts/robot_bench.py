"""Bench test for the robot command channel. Safe with no motors or servos attached.

Connects to the ESP32 command port, checks heartbeats, cycles the shield's RGB
LED with round-trip timing, reads the ultrasonic value, and sends stop.
Drive commands are never sent.

Usage (on the robot's Wi-Fi, ELEGOO phone app disconnected):
    python scripts/robot_bench.py
    python scripts/robot_bench.py --failsafe      # also verify the ESP32 heartbeat failsafe
Against the simulator (in another terminal: python -m wd9000.robot.sim):
    python scripts/robot_bench.py --host 127.0.0.1 --port 10100
"""

import argparse
import logging
import statistics
import sys
import time

from wd9000.config import load_settings
from wd9000.robot import protocol
from wd9000.robot.client import MotionDisabledError, RobotClient

COLORS = [("red", (255, 0, 0)), ("green", (0, 255, 0)), ("blue", (0, 0, 255)), ("off", (0, 0, 0))]


def step(name: str, ok: bool, detail: str = "") -> bool:
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}{'  ' + detail if detail else ''}")
    return ok


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host")
    parser.add_argument("--port", type=int)
    parser.add_argument("--failsafe", action="store_true", help="stop answering heartbeats and time the ESP32 drop")
    parser.add_argument("--hold", type=float, default=1.0, help="seconds to show each LED color")
    args = parser.parse_args()
    logging.basicConfig(level=logging.WARNING, format="%(levelname)s %(name)s: %(message)s")

    cfg = load_settings()["robot"]
    host, port = args.host or cfg["host"], args.port or cfg["command_port"]
    client = RobotClient(host, port, cfg["connect_timeout_s"], cfg["reply_timeout_s"], cfg["link_timeout_s"],
                         motion_enabled=False)

    print(f"Connecting to {host}:{port} ...")
    try:
        client.connect()
    except OSError as e:
        print(f"  [FAIL] connect: {e}")
        print("  Check: on the robot's Wi-Fi? ELEGOO phone app closed (the ESP32 allows one client)?")
        return 1
    results = [step("connect", True)]

    deadline = time.monotonic() + 3
    start_beat = client.last_heartbeat_at
    while client.last_heartbeat_at == start_beat and time.monotonic() < deadline:
        time.sleep(0.05)
    results.append(step("heartbeat received", client.last_heartbeat_at != start_beat,
                        "" if client.last_heartbeat_at != start_beat else "none within 3 s"))

    print("Cycling LED (watch the RGB LED on the shield):")
    rtts = []
    for name, (r, g, b) in COLORS:
        t0 = time.monotonic()
        try:
            value = client.request(protocol.set_light(r, g, b))
            rtts.append((time.monotonic() - t0) * 1000)
            results.append(step(f"LED {name}", value == "ok", f"reply={value!r} rtt={rtts[-1]:.0f} ms"))
        except (TimeoutError, ConnectionError) as e:
            results.append(step(f"LED {name}", False, str(e)))
        time.sleep(args.hold)
    if rtts:
        print(f"  round trip: median {statistics.median(rtts):.0f} ms, max {max(rtts):.0f} ms")

    try:
        value = client.request(protocol.ultrasonic_distance())
        results.append(step("ultrasonic read", True, f"value={value!r} (meaningless if no sensor attached)"))
    except (TimeoutError, ConnectionError) as e:
        results.append(step("ultrasonic read", False, str(e)))

    try:
        client.request(protocol.drive(protocol.Direction.FORWARD, 100))
        results.append(step("motion lock", False, "drive command was NOT refused"))
    except MotionDisabledError:
        results.append(step("motion lock", True, "drive refused locally, nothing sent"))
    except (TimeoutError, ConnectionError) as e:
        results.append(step("motion lock", False, f"drive was sent: {e}"))

    try:
        client.stop()
        results.append(step("stop sent", True))
    except ConnectionError as e:
        results.append(step("stop sent", False, str(e)))

    if args.failsafe:
        print("Failsafe: no longer answering heartbeats; the ESP32 should drop us in ~3-5 s ...")
        client.link_timeout_s = 30  # let the robot, not us, end the link
        client.answer_heartbeats = False
        t0 = time.monotonic()
        while client.connected and time.monotonic() - t0 < 15:
            time.sleep(0.05)
        elapsed = time.monotonic() - t0
        results.append(step("ESP32 failsafe drop", not client.connected,
                            f"after {elapsed:.1f} s ({client.disconnect_reason})"))

    client.close()
    passed = sum(results)
    print(f"\n{passed}/{len(results)} checks passed")
    return 0 if passed == len(results) else 1


if __name__ == "__main__":
    sys.exit(main())
