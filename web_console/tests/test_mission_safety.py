import pytest

from web_console.mission import MAX_WAYPOINTS, MissionController, parse_waypoints
from web_console.safety import StallDetector


def test_parse_waypoints_fills_defaults_and_names():
    points = parse_waypoints([{"x": "1.5", "y": 2}, {"x": 0, "y": 0, "name": "B", "dwell_sec": -3}])
    assert points[0] == {"x": 1.5, "y": 2.0, "yaw": 0.0, "dwell_sec": 0.0, "name": "P1"}
    assert points[1]["name"] == "B" and points[1]["dwell_sec"] == 0.0


@pytest.mark.parametrize("raw", [None, [], [{"x": 1}], [{"x": "a", "y": 0}]])
def test_parse_waypoints_rejects_invalid(raw):
    with pytest.raises(ValueError):
        parse_waypoints(raw)


def test_parse_waypoints_limits_count():
    with pytest.raises(ValueError):
        parse_waypoints([{"x": 0, "y": 0}] * (MAX_WAYPOINTS + 1))


def test_controller_start_without_bindings():
    ctl = MissionController()
    assert ctl.start([{"x": 0, "y": 0}])["ok"] is False
    assert ctl.get_status() == {"status": None}


def test_controller_forwards_to_bindings():
    ctl = MissionController()
    seen = []
    ctl.bind(lambda pts, loop, temp: (seen.append((len(pts), loop, temp)) or (True, "ok", "id-7")),
             lambda name: (True, name))
    assert ctl.start([{"x": 1, "y": 1}], loop=True) == {"ok": True, "message": "ok", "mission_id": "id-7"}
    assert seen == [(1, True, False)]
    assert ctl.command("return_home") == {"ok": True, "message": "/waypoint_manager/return_home"}
    assert ctl.command("bogus")["ok"] is False


def test_stall_detector_flags_stopped_robot_during_mission():
    det = StallDetector(window_sec=30.0, stall_radius_m=0.2)
    det.update_mission_state("NAVIGATING", now=0.0)
    for t in range(0, 40):
        det.update_odom(1.0, 1.0, now=float(t))
    result = det.evaluate(now=40.0)
    assert result["alarm"] is True
    assert result["alarm_type"] == "STOPPED"


def test_stall_detector_idle_when_no_mission_or_motion():
    det = StallDetector()
    det.update_odom(0.0, 0.0, now=0.0)
    assert det.evaluate(now=100.0)["monitoring_active"] is False
