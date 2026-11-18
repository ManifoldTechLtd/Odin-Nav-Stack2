import json
import os
import subprocess

import pytest

from web_console.recorder import BagRecorder, bag_dir_name, heavy_topics, normalize_topics, PRESETS


class FakeTmux:
    """Stands in for tmux and pgrep: tracks sessions, environment, keys and the bag process."""

    def __init__(self):
        self.sessions = set()
        self.env = {}
        self.sent = []
        self.bag_alive = False

    def __call__(self, cmd, **_):
        if cmd[0] == "pgrep":
            return subprocess.CompletedProcess(cmd, 0 if self.bag_alive else 1, "", "")
        sub = cmd[1]
        if sub == "has-session":
            return subprocess.CompletedProcess(cmd, 0 if cmd[3] in self.sessions else 1, "", "")
        if sub == "new-session":
            self.sessions.add(cmd[cmd.index("-s") + 1])
            self.bag_alive = True
        elif sub == "set-environment":
            self.env[cmd[-2]] = cmd[-1]
        elif sub == "show-environment":
            key = cmd[-1]
            if key in self.env:
                return subprocess.CompletedProcess(cmd, 0, "{}={}\n".format(key, self.env[key]), "")
            return subprocess.CompletedProcess(cmd, 1, "", "")
        elif sub == "send-keys":
            if "-l" in cmd:
                self.sent.append(cmd[-1])
            elif "C-c" in cmd:
                self.bag_alive = False
        elif sub == "kill-session":
            self.sessions.discard(cmd[-1])
        return subprocess.CompletedProcess(cmd, 0, "", "")


def test_normalize_topics_accepts_separators_and_dedupes():
    assert normalize_topics("/a /b, /a\n/c") == ["/a", "/b", "/c"]
    assert normalize_topics(["/x", "/x"]) == ["/x"]


@pytest.mark.parametrize("raw", ["", "  ", "map", "/bad name", []])
def test_normalize_topics_rejects_invalid(raw):
    with pytest.raises(ValueError):
        normalize_topics(raw)


def test_heavy_topics_flags_images_and_clouds():
    types = {"/odin1/cloud_filtered": ["sensor_msgs/msg/PointCloud2"],
             "/odin1/image/compressed": ["sensor_msgs/msg/CompressedImage"],
             "/cmd_vel": ["geometry_msgs/msg/Twist"]}
    assert sorted(heavy_topics(types, ["/odin1/cloud_filtered", "/odin1/image/compressed", "/cmd_vel"])) == [
        "/odin1/cloud_filtered", "/odin1/image/compressed"]


def test_bag_dir_name_is_sanitized():
    assert bag_dir_name("nav test/1") == "nav_test_1"
    assert bag_dir_name("") == "bag"


def test_presets_cover_nav_core():
    assert "/cmd_vel" in PRESETS["nav"]["topics"] and "/map" in PRESETS["nav"]["topics"]


def test_start_creates_session_and_meta(tmp_path):
    tmux = FakeTmux()
    rec = BagRecorder(bag_root=str(tmp_path), run=tmux, has_tmux=lambda: True)
    res = rec.start("/cmd_vel /map", name="run one")
    assert res["ok"] is True
    status = rec.status()
    assert status["active"] is True
    assert status["topics"] == ["/cmd_vel", "/map"]
    assert os.path.dirname(status["dir"]) == str(tmp_path)
    assert os.path.basename(status["dir"]).startswith("run_one_")
    meta_files = [f for f in os.listdir(tmp_path) if f.endswith("_meta.json")]
    assert meta_files
    with open(os.path.join(str(tmp_path), meta_files[0]), encoding="utf-8") as f:
        assert json.load(f)["topics"] == ["/cmd_vel", "/map"]
    cmd = tmux.sent[0]
    assert "ros2 bag record -o " in cmd and "/cmd_vel" in cmd and "/map" in cmd


def test_start_refused_while_recording(tmp_path):
    tmux = FakeTmux()
    rec = BagRecorder(bag_root=str(tmp_path), run=tmux, has_tmux=lambda: True)
    rec.start("/map")
    with pytest.raises(ValueError):
        rec.start("/cmd_vel")


def test_stop_interrupts_bag_and_closes_session(tmp_path):
    tmux = FakeTmux()
    rec = BagRecorder(bag_root=str(tmp_path), run=tmux, has_tmux=lambda: True)
    rec.start("/map")
    out_dir = rec.status()["dir"]
    res = rec.stop()
    assert res["ok"] is True and res["dir"] == out_dir
    assert tmux.bag_alive is False
    assert rec.status()["active"] is False
    assert tmux.sessions == set()


def test_status_recovers_after_console_restart(tmp_path):
    tmux = FakeTmux()
    BagRecorder(bag_root=str(tmp_path), run=tmux, has_tmux=lambda: True).start("/tf /map")
    restarted = BagRecorder(bag_root=str(tmp_path), run=tmux, has_tmux=lambda: True)
    status = restarted.status()
    assert status["active"] is True
    assert status["topics"] == ["/tf", "/map"]


def test_stop_when_idle_is_rejected(tmp_path):
    rec = BagRecorder(bag_root=str(tmp_path), run=FakeTmux(), has_tmux=lambda: True)
    with pytest.raises(ValueError):
        rec.stop()


def test_status_reports_size_and_free_space(tmp_path):
    tmux = FakeTmux()
    rec = BagRecorder(bag_root=str(tmp_path), run=tmux, has_tmux=lambda: True)
    rec.start("/map")
    out_dir = rec.status()["dir"]
    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(out_dir, "data.db3"), "wb") as f:
        f.write(b"x" * 4096)
    status = rec.status()
    assert status["size_bytes"] >= 4096
    assert status["free_bytes"] > 0


def test_start_refused_without_tmux(tmp_path):
    rec = BagRecorder(bag_root=str(tmp_path), run=FakeTmux(), has_tmux=lambda: False)
    with pytest.raises(ValueError, match="tmux"):
        rec.start("/map")
    assert rec.status()["tmux"] is False


def test_record_command_uses_console_ros_domain(tmp_path):
    tmux = FakeTmux()
    rec = BagRecorder(bag_root=str(tmp_path), run=tmux, has_tmux=lambda: True, domain="55")
    rec.start("/map")
    assert "export ROS_DOMAIN_ID=55" in tmux.sent[0]
