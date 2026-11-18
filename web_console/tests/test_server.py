import array
import json
import ssl
import threading
import urllib.error
import urllib.request

import pytest

from web_console.hub import API_VERSION, MonitorStateHub
from web_console.server import make_server


def bind_fakes(hub):
    calls = {"estop": [], "start": [], "chassis": [], "trigger": []}

    def start_mission(points, loop, temporary):
        calls["start"].append((points, loop, temporary))
        return True, "accepted", "m-1"

    hub.bind(
        call_trigger=lambda name: (calls["trigger"].append(name) or (True, "ok")),
        publish_cmd_vel=lambda vx, vy, vyaw: None,
        publish_neupan_stop=lambda: None,
        publish_estop=lambda active: calls["estop"].append(active),
        subscribe_image=lambda: None,
        unsubscribe_image=lambda: None,
        start_mission=start_mission,
        set_chassis=lambda enabled: (calls["chassis"].append(enabled) or (False, "no service")),
    )
    return calls


@pytest.fixture
def served():
    hub = MonitorStateHub()
    calls = bind_fakes(hub)
    server = make_server("127.0.0.1", 0, hub, enable_tls=False)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    yield hub, calls, "http://127.0.0.1:{}".format(server.server_address[1])
    server.shutdown()
    server.server_close()


def get(url, **kwargs):
    try:
        with urllib.request.urlopen(url, timeout=5, **kwargs) as res:
            return res.status, res.headers, res.read()
    except urllib.error.HTTPError as exc:
        return exc.code, exc.headers, exc.read()


def post(url, payload=None, raw=None):
    body = raw if raw is not None else json.dumps(payload or {}).encode("utf-8")
    req = urllib.request.Request(url, data=body, method="POST", headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=5) as res:
            return res.status, json.loads(res.read())
    except urllib.error.HTTPError as exc:
        return exc.code, json.loads(exc.read())


def test_index_page_served(served):
    _, _, base = served
    status, headers, body = get(base + "/")
    assert status == 200
    assert headers["Content-Type"].startswith("text/html")
    assert b"<canvas" in body


def test_state_endpoint_returns_pose_mission_and_safety(served):
    hub, _, base = served
    hub.console.set_pose(0.5, 0.25, 0.0)
    status, _, body = get(base + "/api/state")
    payload = json.loads(body)
    assert status == 200
    assert payload["pose"]["x"] == 0.5
    assert payload["map"] is None
    assert payload["mission"]["chassis_supported"] is False
    assert "alarm" in payload["safety"]


def test_map_png_404_until_map_received(served):
    hub, _, base = served
    status, _, _ = get(base + "/api/map.png")
    assert status == 404
    hub.console.set_map(1, 1, 0.1, 0.0, 0.0, array.array("b", [0]))
    status, headers, body = get(base + "/api/map.png")
    assert status == 200
    assert headers["Content-Type"] == "image/png"
    assert body[:8] == b"\x89PNG\r\n\x1a\n"
    assert headers["ETag"] == '"1"'


def test_map_png_conditional_request_returns_304(served):
    hub, _, base = served
    hub.console.set_map(1, 1, 0.1, 0.0, 0.0, array.array("b", [0]))
    req = urllib.request.Request(base + "/api/map.png", headers={"If-None-Match": '"1"'})
    with pytest.raises(urllib.error.HTTPError) as err:
        urllib.request.urlopen(req, timeout=5)
    assert err.value.code == 304


def test_camera_frame_204_without_frames(served):
    _, _, base = served
    status, _, _ = get(base + "/api/camera/frame.jpg")
    assert status == 204


def test_mission_start_forwards_waypoints(served):
    _, calls, base = served
    status, res = post(base + "/api/mission/start", {
        "waypoints": [{"x": 1, "y": 2, "yaw": 0.5, "dwell_sec": 3, "name": "A"}, {"x": 4, "y": 5}],
        "loop": True, "temporary": False})
    assert status == 200 and res["ok"] is True and res["mission_id"] == "m-1"
    points, loop, temporary = calls["start"][0]
    assert [p["name"] for p in points] == ["A", "P2"]
    assert (loop, temporary) == (True, False)


def test_mission_start_rejects_empty_waypoints(served):
    _, calls, base = served
    status, res = post(base + "/api/mission/start", {"waypoints": []})
    assert res["ok"] is False and calls["start"] == []


def test_estop_command_reaches_publisher(served):
    _, calls, base = served
    status, res = post(base + "/api/nav/command", {"action": "emergency_stop"})
    assert status == 200
    assert calls["estop"] == [True]


def test_chassis_control_reports_missing_service(served):
    _, calls, base = served
    status, res = post(base + "/api/chassis/control", {"enabled": True})
    assert res["ok"] is False and calls["chassis"] == [True]


def test_mission_command_return_home_uses_trigger(served):
    _, calls, base = served
    status, res = post(base + "/api/mission/command", {"action": "return_home"})
    assert res["ok"] is True and calls["trigger"] == ["/waypoint_manager/return_home"]


def test_bad_json_returns_400(served):
    _, _, base = served
    status, res = post(base + "/api/teleop/cmd_vel", raw=b"{not json")
    assert status == 400 and res["ok"] is False


def test_unknown_path_404(served):
    _, _, base = served
    status, _, _ = get(base + "/nope")
    assert status == 404


def test_single_port_serves_https_and_http():
    hub = MonitorStateHub()
    server = make_server("127.0.0.1", 0, hub, enable_tls=True)
    if server.ssl_context is None:
        pytest.skip("openssl unavailable")
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    port = server.server_address[1]
    try:
        ctx = ssl._create_unverified_context()
        status, _, _ = get("https://127.0.0.1:{}/api/state".format(port), context=ctx)
        assert status == 200
        status, _, _ = get("http://127.0.0.1:{}/api/state".format(port))
        assert status == 200
    finally:
        server.shutdown()
        server.server_close()


def test_state_reports_api_version(served):
    _, _, base = served
    status, _, body = get(base + "/api/state")
    assert json.loads(body)["api_version"] == API_VERSION


def test_speak_tts_returns_before_playback_finishes(tmp_path):
    import time
    from web_console.intercom import AudioIntercom

    def slow_tts(text, out_wav):
        time.sleep(1.0)
        with open(out_wav, "wb") as f:
            f.write(b"x" * 1024)

    hub = MonitorStateHub(intercom=AudioIntercom(cache_dir=str(tmp_path), custom_tts_fn=slow_tts,
                                                 custom_play_fn=lambda p, m: None))
    server = make_server("127.0.0.1", 0, hub, enable_tls=False)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        started = time.time()
        status, res = post("http://127.0.0.1:{}/api/audio/speak_tts".format(server.server_address[1]),
                           {"text": "请让一让"})
        assert status == 200 and res["queued"] is True
        assert time.time() - started < 0.8
    finally:
        server.shutdown()
        server.server_close()


def test_stack_state_lists_maps_and_mode_options(tmp_path):
    from web_console.stack import MODES, NavStack

    maps = tmp_path / "ros2_ws" / "src" / "map_planner" / "maps"
    maps.mkdir(parents=True)
    (maps / "floor2.yaml").write_text("image: x\n")
    (maps / "floor2.pgm").write_bytes(b"P5")
    hub = MonitorStateHub(intercom=None)
    hub.stack = NavStack(repo_root=str(tmp_path), session_active=lambda: False)
    server = make_server("127.0.0.1", 0, hub, enable_tls=False)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    try:
        base = "http://127.0.0.1:{}".format(server.server_address[1])
        status, _, body = get(base + "/api/stack/state")
        payload = json.loads(body)
        assert status == 200 and payload["state"] == "stopped"
        assert [m["name"] for m in payload["maps"]] == ["floor2"]
        assert payload["modes"] == list(MODES)
        status, res = post(base + "/api/stack/start", {"map": "nope", "navigation_mode": "escape"})
        assert status == 400 and res["ok"] is False
    finally:
        server.shutdown()
        server.server_close()


def test_record_start_and_topics_endpoints(tmp_path):
    from web_console.recorder import BagRecorder
    from web_console.tests.test_recorder import FakeTmux

    hub = MonitorStateHub(intercom=None)
    hub.recorder = BagRecorder(bag_root=str(tmp_path), run=FakeTmux(), has_tmux=lambda: True)
    hub._list_topics = lambda: {"/cmd_vel": ["geometry_msgs/msg/Twist"],
                                "/odin1/cloud_filtered": ["sensor_msgs/msg/PointCloud2"]}
    server = make_server("127.0.0.1", 0, hub, enable_tls=False)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    try:
        base = "http://127.0.0.1:{}".format(server.server_address[1])
        status, _, body = get(base + "/api/record/topics")
        topics = json.loads(body)["topics"]
        assert [t["name"] for t in topics] == ["/cmd_vel", "/odin1/cloud_filtered"]
        assert [t["heavy"] for t in topics] == [False, True]
        status, res = post(base + "/api/record/start", {"topics": "", "name": "x"})
        assert status == 400
        status, res = post(base + "/api/record/start", {"topics": ["/cmd_vel"], "name": "run"})
        assert status == 200 and res["ok"] is True and res["topics"] == ["/cmd_vel"]
        status, _, body = get(base + "/api/record/state")
        assert json.loads(body)["active"] is True
        status, res = post(base + "/api/record/stop", {})
        assert status == 200 and res["ok"] is True
    finally:
        server.shutdown()
        server.server_close()


def test_stack_state_lists_sru_policies_and_accepts_sru_start(tmp_path):
    from web_console.stack import NavStack

    repo = tmp_path / "repo"
    maps = repo / "ros2_ws" / "src" / "map_planner" / "maps"
    maps.mkdir(parents=True)
    (maps / "floor2.yaml").write_text("image: x\n")
    (maps / "floor2.pgm").write_bytes(b"P5")
    models = repo / "sru" / "models"
    models.mkdir(parents=True)
    (models / "nav_policy_sw01.onnx").write_bytes(b"x")
    hub = MonitorStateHub(intercom=None)
    hub.stack = NavStack(repo_root=str(repo), session_active=lambda: False, has_tmux=lambda: True,
                         start_script=str(tmp_path / "missing.sh"))
    server = make_server("127.0.0.1", 0, hub, enable_tls=False)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    try:
        base = "http://127.0.0.1:{}".format(server.server_address[1])
        status, _, body = get(base + "/api/stack/state")
        payload = json.loads(body)
        policies = {p["name"]: p["available"] for p in payload["sru_policies"]}
        assert policies == {"main": True, "conservative": False, "roll_v2": False}
        status, res = post(base + "/api/stack/start", {"map": "floor2", "navigation_mode": "escape",
                                                       "planner": "sru", "sru_policy": "conservative"})
        assert status == 400 and "缺失" in res["error"]
        status, res = post(base + "/api/stack/start", {"map": "floor2", "navigation_mode": "escape",
                                                       "planner": "fancy"})
        assert status == 400
    finally:
        server.shutdown()
        server.server_close()
