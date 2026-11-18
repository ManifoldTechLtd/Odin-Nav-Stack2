import os
import time

import pytest

from web_console.stack import NavStack


def write_script(path, body):
    with open(path, "w", encoding="utf-8") as f:
        f.write("#!/usr/bin/env bash\n" + body)
    os.chmod(path, 0o755)
    return str(path)


@pytest.fixture
def repo(tmp_path):
    maps = tmp_path / "ros2_ws" / "src" / "map_planner" / "maps"
    maps.mkdir(parents=True)
    for name in ("map_opt", "floor2", "empty"):
        (maps / (name + ".yaml")).write_text("image: x.pgm\n")
        (maps / (name + ".pgm")).write_bytes(b"P5")
    reloc = tmp_path / "customer_file"
    reloc.mkdir()
    (reloc / "map_opt.bin").write_bytes(b"\x00")
    return tmp_path


def make_stack(repo, start_body=None, stop_body="echo '[ons2_sw01_stop] Done'\nexit 0\n", active=None, has_tmux=None):
    flag = repo / "session.flag"
    body = start_body or "echo '[ons2_sw01] w1 starting odin'\necho '[ons2_sw01]   [OK] Gate 1'\ntouch {}\nexit 0\n".format(flag)
    start = write_script(repo / "start.sh", body)
    stop = write_script(repo / "stop.sh", stop_body.replace("exit 0", "rm -f {}\nexit 0".format(flag)))
    return NavStack(repo_root=str(repo), maps_dir=str(repo / "ros2_ws/src/map_planner/maps"),
                    reloc_dir=str(repo / "customer_file"), start_script=start, stop_script=stop,
                    session_active=active or (lambda: flag.exists()), has_tmux=has_tmux or (lambda: True))


def wait_job(stack, timeout=5.0):
    end = time.time() + timeout
    while stack.status()["job"] and stack.status()["job"]["running"] and time.time() < end:
        time.sleep(0.02)


def test_lists_maps_without_empty_and_flags_reloc(repo):
    stack = make_stack(repo)
    maps = {m["name"]: m["reloc"] for m in stack.list_maps()}
    assert maps == {"map_opt": True, "floor2": False}


def test_start_rejects_unknown_map_and_bad_mode(repo):
    stack = make_stack(repo)
    with pytest.raises(ValueError):
        stack.start("nope", "replan_periodic")
    with pytest.raises(ValueError):
        stack.start("map_opt", "fly")


def test_start_runs_script_and_streams_progress(repo):
    stack = make_stack(repo)
    res = stack.start("map_opt", "escape")
    assert res["ok"] is True
    wait_job(stack)
    status = stack.status()
    assert status["state"] == "running"
    assert any("Gate 1" in line for line in status["log"])
    assert status["job"]["map"] == "map_opt" and status["job"]["mode"] == "escape"


def test_start_script_gets_map_and_mode_args(repo):
    stack = make_stack(repo, start_body='echo "ARGS $@"\ntouch {}\nexit 0\n'.format(repo / "session.flag"))
    stack.start("floor2", "neupan_only")
    wait_job(stack)
    assert "ARGS --map=floor2 --navigation-mode=neupan_only" in stack.status()["log"]


def test_failed_start_reports_failed_state(repo):
    stack = make_stack(repo, start_body="echo '[ons2_sw01][FAIL] Gate 2 timed out'\nexit 3\n")
    stack.start("map_opt", "replan_periodic")
    wait_job(stack)
    status = stack.status()
    assert status["state"] == "failed"
    assert status["job"]["returncode"] == 3
    assert any("FAIL" in line for line in status["log"])


def test_second_start_refused_while_session_exists(repo):
    stack = make_stack(repo, active=lambda: True)
    with pytest.raises(ValueError):
        stack.start("map_opt", "replan_periodic")


def test_second_job_refused_while_one_is_running(repo):
    stack = make_stack(repo, start_body="sleep 2\nexit 0\n")
    stack.start("map_opt", "replan_periodic")
    with pytest.raises(ValueError):
        stack.stop()


def test_stop_runs_stop_script(repo):
    stack = make_stack(repo)
    stack.stop()
    wait_job(stack)
    status = stack.status()
    assert status["state"] == "stopped"
    assert "[ons2_sw01_stop] Done" in status["log"]


def test_state_is_starting_while_script_runs(repo):
    stack = make_stack(repo, start_body="sleep 2\nexit 0\n")
    stack.start("map_opt", "replan_periodic")
    assert stack.status()["state"] == "starting"


def test_start_and_stop_refused_without_tmux(repo):
    stack = make_stack(repo, has_tmux=lambda: False)
    with pytest.raises(ValueError, match="tmux"):
        stack.start("map_opt", "escape")
    with pytest.raises(ValueError, match="tmux"):
        stack.stop()
    assert stack.status()["tmux"] is False


def make_sru_stack(repo, start_body=None):
    stack = make_stack(repo, start_body=start_body)
    models = repo / "sru" / "models"
    models.mkdir(parents=True, exist_ok=True)
    for f in ("nav_policy_sw01.onnx", "nav_policy_sw01_conservative.onnx"):
        (models / f).write_bytes(b"onnx")
    stack.sru_models_dir = str(models)
    return stack


def test_sru_start_passes_planner_and_policy_flags(repo):
    stack = make_sru_stack(repo, start_body='echo "ARGS $@"\ntouch {}\nexit 0\n'.format(repo / "session.flag"))
    stack.start("floor2", "replan_periodic", planner="sru", sru_policy="main")
    wait_job(stack)
    status = stack.status()
    assert "ARGS --map=floor2 --navigation-mode=replan_periodic --local-planner=sru --sru-policy=main" in status["log"]
    assert status["job"]["planner"] == "sru" and status["job"]["sru_policy"] == "main"


def test_neupan_start_keeps_original_arguments(repo):
    stack = make_sru_stack(repo, start_body='echo "ARGS $@"\ntouch {}\nexit 0\n'.format(repo / "session.flag"))
    stack.start("floor2", "escape")
    wait_job(stack)
    assert "ARGS --map=floor2 --navigation-mode=escape" in stack.status()["log"]


def test_sru_rejects_unknown_or_missing_policy(repo):
    stack = make_sru_stack(repo)
    with pytest.raises(ValueError):
        stack.start("floor2", "escape", planner="sru", sru_policy="nope")
    with pytest.raises(ValueError):
        stack.start("floor2", "escape", planner="sru", sru_policy="roll_v2")
    with pytest.raises(ValueError):
        stack.start("floor2", "escape", planner="fancy")


def test_lists_sru_policies_with_availability(repo):
    stack = make_sru_stack(repo)
    policies = {p["name"]: p["available"] for p in stack.list_sru_policies()}
    assert policies == {"main": True, "conservative": True, "roll_v2": False}
