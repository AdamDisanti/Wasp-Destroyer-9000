import time

import pytest

from wd9000.robot import protocol
from wd9000.robot.client import MotionDisabledError, RobotClient
from wd9000.robot.sim import SimRobot


def _wait_for(predicate, timeout=3.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return False


# --- protocol -----------------------------------------------------------------

def test_encode_is_compact_and_tagged():
    assert protocol.encode(protocol.set_light(255, 0, 10), "c1") == b'{"N":8,"D1":0,"D2":255,"D3":0,"D4":10,"H":"c1"}'
    assert protocol.encode(protocol.stop()) == b'{"N":100}'


def test_servo_uses_firmware_field_layout():
    assert protocol.servo_angle(protocol.Servo.TILT, 45) == {"N": 5, "D1": 2, "D2": 45}


@pytest.mark.parametrize("build", [
    lambda: protocol.set_light(256, 0, 0),
    lambda: protocol.servo_angle(protocol.Servo.PAN, 181),
    lambda: protocol.drive(protocol.Direction.FORWARD, -1),
    lambda: protocol.drive_for(protocol.Direction.FORWARD, 100, 25_000),
])
def test_out_of_range_values_rejected(build):
    with pytest.raises(ValueError):
        build()


def test_motion_classification():
    assert protocol.is_motion(protocol.drive(protocol.Direction.LEFT, 50))
    assert protocol.is_motion(protocol.track_speeds(10, 10))
    assert not protocol.is_motion(protocol.stop())
    assert not protocol.is_motion(protocol.set_light(1, 2, 3))
    assert not protocol.is_motion(protocol.servo_angle(protocol.Servo.PAN, 90))


def test_token_parser_reassembles_split_tokens():
    parser = protocol.TokenParser()
    assert parser.feed(b"noise{Heart") == []
    assert parser.feed(b"beat}{c1_o") == ["{Heartbeat}"]
    assert parser.feed(b"k}{ok}") == ["{c1_ok}", "{ok}"]


def test_parse_reply():
    assert protocol.parse_reply("{c7_42}") == protocol.Reply("{c7_42}", "c7", "42")
    assert protocol.parse_reply("{ok}") == protocol.Reply("{ok}", None, "ok")
    assert protocol.parse_reply("{_ok}").command_id is None  # firmware reply when H is missing


# --- client against the simulator ---------------------------------------------

@pytest.fixture
def sim():
    with SimRobot(heartbeat_interval_s=0.05) as robot:
        yield robot


def _client(sim, **kwargs) -> RobotClient:
    kwargs.setdefault("link_timeout_s", 0.5)
    return RobotClient(sim.host, sim.port, **kwargs)


def test_request_round_trip_sets_light(sim):
    with _client(sim) as client:
        assert client.request(protocol.set_light(0, 255, 0)) == "ok"
    assert sim.state.light == (0, 255, 0)


def test_ultrasonic_value(sim):
    sim.distance_cm = 33
    with _client(sim) as client:
        assert client.request(protocol.ultrasonic_distance()) == "33"


def test_heartbeats_keep_link_alive(sim):
    with _client(sim) as client:
        time.sleep(0.6)  # > 3 missed heartbeats if we weren't answering
        assert client.connected
        assert client.heartbeat_age_s < 0.2


def test_motion_refused_by_default_and_nothing_sent(sim):
    with _client(sim) as client:
        with pytest.raises(MotionDisabledError):
            client.send(protocol.drive(protocol.Direction.FORWARD, 100))
        client.request(protocol.set_light(1, 1, 1))  # sync point
    assert not any(protocol.is_motion(c) for c in sim.state.commands)


def test_motion_allowed_when_enabled(sim):
    with _client(sim, motion_enabled=True) as client:
        assert client.request(protocol.drive(protocol.Direction.FORWARD, 80)) == "ok"
        assert sim.state.motion["N"] == 3


def test_close_sends_stop(sim):
    client = _client(sim, motion_enabled=True).connect()
    client.request(protocol.drive(protocol.Direction.FORWARD, 80))
    client.close()
    assert _wait_for(lambda: {"N": 100} in sim.state.commands)
    assert sim.state.motion is None


def test_robot_drops_hung_client_and_stops(sim):
    reasons = []
    client = _client(sim, link_timeout_s=30, motion_enabled=True)
    client.on_disconnect = reasons.append
    client.connect()
    client.request(protocol.drive(protocol.Direction.FORWARD, 80))
    client.answer_heartbeats = False
    assert _wait_for(lambda: not client.connected)
    assert reasons == ["connection closed by robot"]
    assert sim.state.motion is None and sim.state.stops >= 1


def test_client_detects_silent_robot(sim):
    client = _client(sim, link_timeout_s=0.3).connect()
    sim.heartbeat_interval_s = 10  # robot goes quiet
    assert _wait_for(lambda: not client.connected)
    assert "no heartbeat" in client.disconnect_reason
    with pytest.raises(ConnectionError):
        client.send(protocol.set_light(0, 0, 0))


def test_request_fails_fast_when_link_drops(sim):
    client = _client(sim, reply_timeout_s=5).connect()
    sim.stop()
    t0 = time.monotonic()
    with pytest.raises((ConnectionError, TimeoutError)):
        client.request(protocol.set_light(1, 2, 3))
    assert time.monotonic() - t0 < 2


# --- spray actuator -----------------------------------------------------------

from wd9000.robot.protocol import SprayPose, SprayServo  # noqa: E402
from wd9000.robot.spray import SprayHold  # noqa: E402


def test_spray_builders():
    assert protocol.spray(True) == {"N": 120, "D1": 1}
    assert protocol.spray_jog(SprayServo.BOTH, 180) == {"N": 122, "D1": 3, "D2": 180}
    with pytest.raises(ValueError):
        protocol.spray_jog(SprayServo.LEFT, 181)
    with pytest.raises(ValueError):
        protocol.spray_press_for(10)


def test_parse_spray_status():
    status = protocol.parse_spray_status("st=PRESS;pos=180,180;raw=180,0;inv=0,1;max=2000;cap=1")
    assert status == {"st": "PRESS", "pos": (180, 180), "raw": (180, 0), "inv": (0, 1), "max": 2000, "cap": 1}


@pytest.fixture
def spray_rig():
    with SimRobot(spray_lease_s=0.2) as robot:
        client = RobotClient(robot.host, robot.port, reply_timeout_s=1).connect()
        yield robot, client
        client.close()


def _status(client):
    return protocol.parse_spray_status(client.request(protocol.spray_status()))


def test_servo2_is_mirrored_by_default(spray_rig):
    _, client = spray_rig
    client.request(protocol.spray_jog(SprayServo.BOTH, 180))
    assert _status(client)["raw"] == (180, 0)
    client.request(protocol.spray_invert(SprayServo.RIGHT, False))
    assert _status(client)["raw"] == (180, 180)


def test_jog_and_save_poses(spray_rig):
    _, client = spray_rig
    client.request(protocol.spray_jog(SprayServo.BOTH, 180))
    client.request(protocol.spray_jog(SprayServo.LEFT, 170))
    client.request(protocol.spray_save(SprayPose.PRESS))
    client.request(protocol.spray_press_for(100))
    assert _status(client)["pos"] == (170, 180)
    assert _wait_for(lambda: _status(client)["st"] == "REST")


def test_lease_press_releases_without_resend(spray_rig):
    _, client = spray_rig
    assert client.request(protocol.spray(True)) == "ok"
    assert _status(client)["st"] == "PRESS"
    time.sleep(0.35)
    assert _status(client)["st"] == "REST"


def test_hold_keeps_pressing_then_caps(spray_rig):
    _, client = spray_rig
    client.request(protocol.spray_max_on(800))
    hold = SprayHold(client, interval_s=0.05).start()
    time.sleep(0.5)  # longer than the 0.2 s lease
    assert _status(client)["st"] == "PRESS"
    assert _wait_for(lambda: hold.capped, timeout=2)
    assert _status(client)["st"] == "REST"
    assert client.request(protocol.spray(True)) == "capped"
    hold.stop()  # releasing clears the latch
    assert client.request(protocol.spray(True)) == "ok"


def test_stop_and_disconnect_release_spray(spray_rig):
    robot, client = spray_rig
    client.request(protocol.spray_press_for(1000))
    client.stop()
    assert _wait_for(lambda: robot.state.spray.state == "REST")
    client.request(protocol.spray_press_for(1000))
    client.close()
    assert _wait_for(lambda: robot.state.spray.state == "REST")


def test_robot_without_heartbeats_stays_connected():
    # The ESP32 on our robot never sends {Heartbeat}; commands must still work.
    with SimRobot(send_heartbeats=False) as robot:
        client = RobotClient(robot.host, robot.port, link_timeout_s=0.3).connect()
        time.sleep(0.6)  # past link_timeout_s
        assert client.connected and not client.heartbeats_seen
        assert client.request(protocol.spray_status()).startswith("st=REST")
        client.close()


def test_heartbeat_watchdog_switches_on_after_first_heartbeat():
    with SimRobot(heartbeat_interval_s=0.1) as robot:
        client = RobotClient(robot.host, robot.port).connect()
        assert _wait_for(lambda: client.heartbeats_seen)
        client.close()
