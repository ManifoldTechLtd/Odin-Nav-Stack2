#!/usr/bin/env python3
"""Unit and API tests for remote_monitor."""

import json
import os
import sys
import urllib.request

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from remote_monitor import (
    AudioIntercom,
    MonitorStateHub,
    NavigationAndTeleopController,
    OnDemandImageGate,
    StallDetector,
    create_monitor_http_server,
)


def test_stall_detector_no_alarm_when_idle_and_not_forced():
    det = StallDetector(window_sec=30.0, stall_radius_m=0.20)
    det.update_mission_state("IDLE", now=0.0)
    for t in range(0, 40):
        det.update_odom(1.0, 2.0, vx=0.0, vy=0.0, vyaw=0.0, now=float(t))
    res = det.evaluate(now=39.0)
    assert res["monitoring_active"] is False
    assert res["alarm"] is False


def test_stall_detector_triggers_on_complete_stop_over_30s():
    det = StallDetector(window_sec=30.0, stall_radius_m=0.20)
    det.update_mission_state("NAVIGATING", now=0.0)
    for t in range(0, 32):
        det.update_odom(5.0, 5.0, vx=0.0, vy=0.0, vyaw=0.0, now=float(t))
    res = det.evaluate(now=31.0)
    assert res["monitoring_active"] is True
    assert res["alarm"] is True
    assert res["alarm_type"] == "STOPPED"
    assert res["stall_duration_sec"] >= 30.0
    assert res["max_displacement_m"] < 0.01


def test_stall_detector_triggers_on_creeping_in_place_over_30s():
    det = StallDetector(window_sec=30.0, stall_radius_m=0.20)
    det.update_mission_state("NAVIGATING", now=0.0)
    # Robot oscillates back and forth within 0.08m for 32 seconds (creeping/twitching in place)
    for t in range(0, 33):
        offset = 0.08 if (t % 2 == 1) else -0.04
        det.update_odom(10.0 + offset, 3.0 + offset * 0.5, vx=0.12, vy=0.0, vyaw=0.25, now=float(t))
    res = det.evaluate(now=32.0)
    assert res["monitoring_active"] is True
    assert res["alarm"] is True
    assert res["alarm_type"] == "CREEPING"
    assert res["max_displacement_m"] < 0.20


def test_stall_detector_no_alarm_when_making_normal_progress():
    det = StallDetector(window_sec=30.0, stall_radius_m=0.20)
    det.update_mission_state("NAVIGATING", now=0.0)
    for t in range(0, 35):
        det.update_odom(t * 0.15, 0.0, vx=0.15, vy=0.0, vyaw=0.0, now=float(t))
    res = det.evaluate(now=34.0)
    assert res["monitoring_active"] is True
    assert res["alarm"] is False
    assert res["max_displacement_m"] > 1.0


def test_stall_detector_force_armed_triggers_even_when_idle():
    det = StallDetector(window_sec=30.0, stall_radius_m=0.20)
    det.set_force_monitor(True, now=10.0)
    for t in range(10, 42):
        det.update_odom(0.0, 0.0, vx=0.0, vy=0.0, vyaw=0.0, now=float(t))
    res = det.evaluate(now=41.0)
    assert res["monitoring_active"] is True
    assert res["alarm"] is True


def test_stall_detector_mute_suppresses_buzzer_until_expiry():
    det = StallDetector(window_sec=30.0, stall_radius_m=0.20)
    det.update_mission_state("NAVIGATING", now=0.0)
    for t in range(0, 32):
        det.update_odom(1.0, 1.0, vx=0.0, vy=0.0, vyaw=0.0, now=float(t))
    det.acknowledge_alarm(mute_sec=60.0, now=31.0)
    res_muted = det.evaluate(now=50.0)
    assert res_muted["alarm"] is True
    assert res_muted["buzzer_muted"] is True
    res_unmuted = det.evaluate(now=95.0)
    assert res_unmuted["alarm"] is True
    assert res_unmuted["buzzer_muted"] is False


def test_on_demand_image_gate_starts_disabled_and_toggles_subscription():
    sub_calls = []
    unsub_calls = []

    gate = OnDemandImageGate(
        subscribe_fn=lambda: sub_calls.append("sub"),
        unsubscribe_fn=lambda: unsub_calls.append("unsub"),
        default_fps=2.0,
    )
    assert gate.enabled is False
    assert len(sub_calls) == 0

    gate.set_enabled(True, fps=4.0)
    assert gate.enabled is True
    assert gate.fps == 3.0
    assert len(sub_calls) == 1

    # Feeding frames respects FPS rate limit (3 FPS -> 0.333s interval)
    accepted_1 = gate.on_compressed_frame(b"\xff\xd8\xff\xd9", now=1.0)
    accepted_2 = gate.on_compressed_frame(b"\xff\xd8\xff\x00\xd9", now=1.1)
    accepted_3 = gate.on_compressed_frame(b"\xff\xd8\xff\x01\xd9", now=1.4)
    assert accepted_1 is True
    assert accepted_2 is False
    assert accepted_3 is True
    frame_bytes, frame_seq, _ = gate.get_latest_frame()
    assert frame_bytes == b"\xff\xd8\xff\x01\xd9"
    assert frame_seq == 2

    gate.set_enabled(False)
    assert gate.enabled is False
    assert len(unsub_calls) == 1
    assert gate.get_latest_frame()[0] is None


def test_http_server_status_camera_toggle_and_tts_endpoints(tmp_path):
    sub_state = {"subscribed": False}
    spoken_items = []

    hub = MonitorStateHub(
        subscribe_image_fn=lambda: sub_state.__setitem__("subscribed", True),
        unsubscribe_image_fn=lambda: sub_state.__setitem__("subscribed", False),
        intercom=AudioIntercom(
            cache_dir=str(tmp_path),
            custom_play_fn=lambda wav_path, meta: spoken_items.append((wav_path, meta)),
            custom_tts_fn=lambda text, out_wav: open(out_wav, "wb").write(b"RIFF____WAVEfmt "),
        ),
    )

    server, thread, port = create_monitor_http_server("127.0.0.1", 0, hub, enable_tls=False)
    try:
        base_url = f"http://127.0.0.1:{port}"
        # 1. Check initial status (camera must be disabled by default)
        with urllib.request.urlopen(f"{base_url}/api/status", timeout=3) as resp:
            data = json.loads(resp.read().decode("utf-8"))
        assert data["camera"]["enabled"] is False
        assert sub_state["subscribed"] is False

        # 2. Toggle camera ON
        req = urllib.request.Request(
            f"{base_url}/api/camera/control",
            data=json.dumps({"enabled": True, "fps": 3.0}).encode("utf-8"),
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with urllib.request.urlopen(req, timeout=3) as resp:
            cam_res = json.loads(resp.read().decode("utf-8"))
        assert cam_res["ok"] is True
        assert cam_res["camera"]["enabled"] is True
        assert sub_state["subscribed"] is True

        # 3. Toggle camera OFF
        req_off = urllib.request.Request(
            f"{base_url}/api/camera/control",
            data=json.dumps({"enabled": False}).encode("utf-8"),
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with urllib.request.urlopen(req_off, timeout=3) as resp:
            cam_off_res = json.loads(resp.read().decode("utf-8"))
        assert cam_off_res["camera"]["enabled"] is False
        assert sub_state["subscribed"] is False

        # 4. Trigger TTS intercom
        req_tts = urllib.request.Request(
            f"{base_url}/api/audio/speak_tts",
            data=json.dumps({"text": "请让一让，机器狗正在作业", "volume": 120}).encode("utf-8"),
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with urllib.request.urlopen(req_tts, timeout=3) as resp:
            tts_res = json.loads(resp.read().decode("utf-8"))
        assert tts_res["ok"] is True
        assert len(spoken_items) == 1
        assert spoken_items[0][1]["text"] == "请让一让，机器狗正在作业"
    finally:
        server.shutdown()
        server.server_close()


def test_stall_detector_clears_alarm_once_robot_resumes_movement():
    det = StallDetector(window_sec=30.0, stall_radius_m=0.20)
    det.update_mission_state("NAVIGATING", now=0.0)
    for t in range(0, 32):
        det.update_odom(2.0, 2.0, vx=0.0, vy=0.0, vyaw=0.0, now=float(t))
    assert det.evaluate(now=31.0)["alarm"] is True

    # Robot moves forward by 0.5m at t=32
    det.update_odom(2.5, 2.0, vx=0.5, vy=0.0, vyaw=0.0, now=32.0)
    res_cleared = det.evaluate(now=32.0)
    assert res_cleared["alarm"] is False
    assert res_cleared["max_displacement_m"] >= 0.49


def test_single_port_dual_http_and_https_and_raw_audio(tmp_path):
    import ssl

    spoken_items = []
    hub = MonitorStateHub(
        intercom=AudioIntercom(
            cache_dir=str(tmp_path),
            custom_play_fn=lambda wav_path, meta: spoken_items.append((wav_path, meta)),
        )
    )
    server, _, port = create_monitor_http_server("127.0.0.1", 0, hub, enable_tls=True)
    try:
        # 1. Plain HTTP on port
        with urllib.request.urlopen(f"http://127.0.0.1:{port}/api/status", timeout=3) as resp:
            http_data = json.loads(resp.read().decode("utf-8"))
        assert http_data["ok"] is True

        # 2. HTTPS on the EXACT SAME port
        unverified_ctx = ssl._create_unverified_context()
        with urllib.request.urlopen(
            f"https://127.0.0.1:{port}/api/status", context=unverified_ctx, timeout=3
        ) as resp:
            https_data = json.loads(resp.read().decode("utf-8"))
        assert https_data["ok"] is True

        # 3. Generate a minimal valid WAV and POST to /api/audio/speak_raw over HTTPS
        import wave
        wav_file = tmp_path / "sample.wav"
        with wave.open(str(wav_file), "wb") as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)
            wf.setframerate(8000)
            wf.writeframes(b"\x00\x00" * 800)
        raw_wav = wav_file.read_bytes()

        req_raw = urllib.request.Request(
            f"https://127.0.0.1:{port}/api/audio/speak_raw?volume=120&mime=audio/wav",
            data=raw_wav,
            headers={"Content-Type": "audio/wav"},
            method="POST",
        )
        with urllib.request.urlopen(req_raw, context=unverified_ctx, timeout=5) as resp:
            raw_res = json.loads(resp.read().decode("utf-8"))
        assert raw_res["ok"] is True
        assert raw_res["mode"] == "mic"
        assert len(spoken_items) == 1
    finally:
        server.shutdown()
        server.server_close()


def test_nav_and_teleop_controller_emergency_stop_and_release():
    calls = []
    published_twists = []
    estop_signals = []

    ctrl = NavigationAndTeleopController(
        call_service_fn=lambda srv: (calls.append(srv) or (True, f"{srv} ok")),
        publish_cmd_vel_fn=lambda vx, vy, vyaw: published_twists.append((vx, vy, vyaw)),
        publish_neupan_stop_fn=lambda: estop_signals.append("neupan_stop"),
        publish_estop_fn=lambda flag: estop_signals.append(("estop", flag)),
        cmd_timeout_sec=0.6,
    )

    # Trigger Emergency Stop
    res = ctrl.execute_nav_command("emergency_stop")
    assert res["ok"] is True
    assert ctrl.get_status()["estop_locked"] is True
    assert ctrl.get_status()["teleop_active"] is False
    assert "/waypoint_manager/pause" in calls
    assert "neupan_stop" in estop_signals
    assert ("estop", True) in estop_signals

    # Tick control loop while E-Stop locked -> must publish (0.0, 0.0, 0.0)
    ctrl.tick(now=10.0)
    assert published_twists[-1] == (0.0, 0.0, 0.0)

    # Release Emergency Stop
    rel = ctrl.execute_nav_command("release_emergency_stop")
    assert rel["ok"] is True
    assert ctrl.get_status()["estop_locked"] is False
    assert ("estop", False) in estop_signals


def test_nav_and_teleop_controller_pause_resume_cancel_skip():
    calls = []
    ctrl = NavigationAndTeleopController(
        call_service_fn=lambda srv: (calls.append(srv) or (True, f"{srv} ok")),
        publish_cmd_vel_fn=lambda vx, vy, vyaw: None,
        publish_neupan_stop_fn=lambda: calls.append("neupan_stop"),
        publish_estop_fn=lambda flag: None,
    )

    assert ctrl.execute_nav_command("pause")["ok"] is True
    assert "/waypoint_manager/pause" in calls

    calls.clear()
    assert ctrl.execute_nav_command("resume")["ok"] is True
    assert "/waypoint_manager/resume" in calls

    calls.clear()
    assert ctrl.execute_nav_command("skip")["ok"] is True
    assert "/waypoint/goal/skip" in calls
    assert "/waypoint_manager/skip" in calls

    calls.clear()
    assert ctrl.execute_nav_command("cancel")["ok"] is True
    assert "/waypoint/goal/reset" in calls
    assert "/waypoint_manager/cancel" in calls


def test_teleop_takeover_deadman_watchdog_and_release_resume():
    calls = []
    published_twists = []

    ctrl = NavigationAndTeleopController(
        call_service_fn=lambda srv: (calls.append(srv) or (True, f"{srv} ok")),
        publish_cmd_vel_fn=lambda vx, vy, vyaw: published_twists.append((vx, vy, vyaw)),
        publish_neupan_stop_fn=lambda: calls.append("neupan_stop"),
        publish_estop_fn=lambda flag: None,
        cmd_timeout_sec=0.6,
    )

    # 1. Enter teleop takeover -> automatically pauses mission & stops neupan
    res_enter = ctrl.set_teleop_takeover(True, now=1.0)
    assert res_enter["ok"] is True
    assert res_enter["teleop_active"] is True
    assert "/waypoint_manager/pause" in calls
    assert "neupan_stop" in calls

    # 2. Send velocity command (clamped to max limits)
    ctrl.set_teleop_velocity(vx=0.3, vy=-0.1, vyaw=0.4, now=1.1)
    ctrl.tick(now=1.2)
    assert published_twists[-1] == (0.3, -0.1, 0.4)

    # 3. Deadman timeout after 0.6s without fresh command -> auto-zeros velocity!
    ctrl.tick(now=1.85)
    assert published_twists[-1] == (0.0, 0.0, 0.0)

    # 4. Release teleop takeover and resume navigation
    calls.clear()
    res_exit = ctrl.set_teleop_takeover(False, resume_navigation=True, now=2.0)
    assert res_exit["ok"] is True
    assert res_exit["teleop_active"] is False
    assert "/waypoint_manager/resume" in calls


def test_http_endpoints_for_nav_and_teleop():
    calls = []
    twists = []
    nav_ctrl = NavigationAndTeleopController(
        call_service_fn=lambda srv: (calls.append(srv) or (True, f"{srv} ok")),
        publish_cmd_vel_fn=lambda vx, vy, vyaw: twists.append((vx, vy, vyaw)),
        publish_neupan_stop_fn=lambda: calls.append("neupan_stop"),
        publish_estop_fn=lambda flag: calls.append(("estop", flag)),
    )
    hub = MonitorStateHub(nav_controller=nav_ctrl)
    server, _, port = create_monitor_http_server("127.0.0.1", 0, hub, enable_tls=False)
    try:
        base = f"http://127.0.0.1:{port}"

        def post_json(path, payload):
            req = urllib.request.Request(
                f"{base}{path}",
                data=json.dumps(payload).encode("utf-8"),
                headers={"Content-Type": "application/json"},
                method="POST",
            )
            with urllib.request.urlopen(req, timeout=3) as r:
                return json.loads(r.read().decode("utf-8"))

        # Skip waypoint via HTTP
        r_skip = post_json("/api/nav/command", {"action": "skip"})
        assert r_skip["ok"] is True
        assert "/waypoint_manager/skip" in calls

        # Takeover teleop & send velocity via HTTP
        r_take = post_json("/api/teleop/control", {"active": True})
        assert r_take["ok"] is True
        assert r_take["control"]["teleop_active"] is True

        r_vel = post_json("/api/teleop/cmd_vel", {"vx": 0.25, "vy": 0.0, "vyaw": -0.3})
        assert r_vel["ok"] is True
        nav_ctrl.tick()
        assert twists[-1] == (0.25, 0.0, -0.3)

        # Release teleop and resume navigation
        r_rel = post_json("/api/teleop/control", {"active": False, "resume_navigation": True})
        assert r_rel["ok"] is True
        assert r_rel["control"]["teleop_active"] is False
        assert "/waypoint_manager/resume" in calls
    finally:
        server.shutdown()
        server.server_close()


def test_image_gate_downscales_large_jpeg_for_4g_bandwidth_and_auto_idles():
    import io
    from PIL import Image

    # Create a noisy 1920x1080 RGB image (>250KB at quality 95)
    img = Image.effect_noise((1920, 1080), 80.0).convert("RGB")
    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=95)
    raw_large_jpeg = buf.getvalue()
    assert len(raw_large_jpeg) > 150 * 1024

    unsub_calls = []
    gate = OnDemandImageGate(
        subscribe_fn=lambda: None,
        unsubscribe_fn=lambda: unsub_calls.append("unsub"),
        default_fps=2.0,
        max_width=640,
        jpeg_quality=55,
        idle_timeout_sec=10.0,
    )
    gate.set_enabled(True, fps=2.0, now=100.0)
    assert gate.on_compressed_frame(raw_large_jpeg, now=101.0) is True

    compressed_frame, seq, _ = gate.get_latest_frame(now=101.2)
    assert seq == 1
    assert compressed_frame is not None
    # Compressed frame must be drastically smaller (< 60KB) so 4G + frp tunnel never chokes
    assert len(compressed_frame) < 60 * 1024

    # If no client fetches frame for > idle_timeout_sec (10s), incoming frame auto-disables subscription
    gate.on_compressed_frame(raw_large_jpeg, now=115.0)
    assert gate.enabled is False
    assert len(unsub_calls) == 1


def test_dual_protocol_server_not_blocked_by_idle_preconnect_socket():
    import socket
    import ssl
    import time

    hub = MonitorStateHub()
    server, _, port = create_monitor_http_server("127.0.0.1", 0, hub, enable_tls=True)
    idle_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        # Simulate Chrome/natfrp opening an idle preconnect socket without sending bytes
        idle_sock.connect(("127.0.0.1", port))
        time.sleep(0.05)

        # Now send a real HTTPS request while idle_sock is still open and silent;
        # must complete in < 0.5s (not blocked by 3.0s peek timeout in accept loop)
        t0 = time.monotonic()
        unverified_ctx = ssl._create_unverified_context()
        with urllib.request.urlopen(
            f"https://127.0.0.1:{port}/api/status", context=unverified_ctx, timeout=2.0
        ) as resp:
            data = json.loads(resp.read().decode("utf-8"))
        elapsed = time.monotonic() - t0
        assert data["ok"] is True
        assert elapsed < 0.5, f"Accept loop was blocked by idle socket! elapsed={elapsed:.2f}s"
    finally:
        idle_sock.close()
        server.shutdown()
        server.server_close()


def test_http11_keep_alive_reuses_single_tls_connection():
    import http.client
    import ssl

    hub = MonitorStateHub()
    server, _, port = create_monitor_http_server("127.0.0.1", 0, hub, enable_tls=True)
    try:
        ctx = ssl._create_unverified_context()
        conn = http.client.HTTPSConnection("127.0.0.1", port, context=ctx, timeout=3.0)
        first_fd = None
        for _ in range(5):
            conn.request("GET", "/api/status", headers={"Connection": "keep-alive"})
            resp = conn.getresponse()
            body = resp.read()
            assert resp.status == 200
            assert resp.version == 11  # HTTP/1.1
            assert json.loads(body.decode("utf-8"))["ok"] is True
            assert conn.sock is not None, "Server closed connection instead of keeping HTTP/1.1 alive!"
            if first_fd is None:
                first_fd = conn.sock.fileno()
            else:
                assert conn.sock.fileno() == first_fd
        conn.close()
    finally:
        server.shutdown()
        server.server_close()





