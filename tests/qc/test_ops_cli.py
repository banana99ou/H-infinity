import subprocess
import sys

from tools.ops import limo_ops


def test_run_smoke_dry_run_never_requires_ros():
    p = subprocess.run(
        [sys.executable, "tools/ops/limo_ops.py", "run-smoke", "--dry-run"],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    assert p.returncode == 0
    assert "DRY RUN" in p.stdout
    assert "sequencer_smoke" in p.stdout


def test_start_motion_capable_process_is_gated_without_ros():
    p = subprocess.run(
        [sys.executable, "tools/ops/limo_ops.py", "start", "follower"],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    assert p.returncode == 2
    assert "motion-capable" in p.stderr


def test_run_smoke_armed_starts_smoke_sequencer(monkeypatch):
    calls = []
    monkeypatch.setattr(
        limo_ops,
        "topic_pub_once",
        lambda topic, typ, payload: calls.append((topic, typ, payload)) or 0,
    )
    args = type("Args", (), {"dry_run": False, "armed": True})()

    assert limo_ops.cmd_run_smoke(args) == 0
    assert calls == [
        ("/orchestrator/start", "std_msgs/msg/String", "data: sequencer_smoke")
    ]


def test_stop_all_kills_both_sequencers_and_latches_estop(monkeypatch):
    calls = []
    monkeypatch.setattr(
        limo_ops,
        "topic_pub_once",
        lambda topic, typ, payload: calls.append((topic, typ, payload)) or 0,
    )

    assert limo_ops.cmd_stop_all(None) == 0
    payloads = [c[2] for c in calls]
    assert "data: sequencer" in payloads
    assert "data: sequencer_smoke" in payloads
    assert "data: follower" in payloads
    assert "data: reposition" in payloads
    assert calls[-1] == ("/estop_trigger", "std_msgs/msg/Bool", "data: true")
