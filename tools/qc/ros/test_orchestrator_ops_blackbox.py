"""ros-sim tier: orchestrator and ops-node topic contracts."""

from __future__ import annotations

import json
import sys
import time

import pytest

pytest.importorskip("rclpy")

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from std_msgs.msg import Bool, String  # noqa: E402

from limo_path_follower import ops_node, orchestrator_node  # noqa: E402


class ControlProbe(Node):
    def __init__(self):
        super().__init__("qc_control_probe")
        self.orch_status = []
        self.ops_status = []
        self.starts = []
        self.kills = []
        self.exp_cmds = []
        self.estop = []

        self.pub_start = self.create_publisher(String, "/orchestrator/start", 10)
        self.pub_kill = self.create_publisher(String, "/orchestrator/kill", 10)
        self.pub_ops = self.create_publisher(String, "/ops/cmd", 10)
        self.pub_orch_status = self.create_publisher(String, "/orchestrator/status", 10)
        self.pub_exp_status = self.create_publisher(String, "/experiment/status", 10)
        self.pub_rtk = self.create_publisher(String, "/gps_rtk_f9p_helical/gps/rtk_status", 10)

        self.create_subscription(String, "/orchestrator/status", self._on_orch_status, 10)
        self.create_subscription(String, "/ops/status", self._on_ops_status, 10)
        self.create_subscription(String, "/orchestrator/start", lambda m: self.starts.append(m.data), 10)
        self.create_subscription(String, "/orchestrator/kill", lambda m: self.kills.append(m.data), 10)
        self.create_subscription(String, "/experiment/cmd", lambda m: self.exp_cmds.append(m.data), 10)
        self.create_subscription(Bool, "/estop_trigger", lambda m: self.estop.append(bool(m.data)), 10)

    def _on_orch_status(self, msg: String):
        try:
            self.orch_status.append(json.loads(msg.data))
        except ValueError:
            pass

    def _on_ops_status(self, msg: String):
        try:
            self.ops_status.append(json.loads(msg.data))
        except ValueError:
            pass

    def publish_start(self, name: str):
        self.pub_start.publish(String(data=name))

    def publish_kill(self, name: str):
        self.pub_kill.publish(String(data=name))

    def publish_ops(self, payload):
        data = payload if isinstance(payload, str) else json.dumps(payload)
        self.pub_ops.publish(String(data=data))

    def publish_orch_status(self, payload):
        self.pub_orch_status.publish(String(data=json.dumps(payload)))

    def publish_exp_status(self, payload):
        self.pub_exp_status.publish(String(data=json.dumps(payload)))


def _spin_until(nodes, predicate, *, timeout=4.0, what="condition"):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        for node in nodes:
            rclpy.spin_once(node, timeout_sec=0.02)
        if predicate():
            return
    raise AssertionError(f"timed out waiting for {what}")


def _sleep_cmd(seconds=30):
    return [sys.executable, "-c", f"import time; time.sleep({float(seconds)})"]


@pytest.fixture
def orchestrator_pair(ros_context, tmp_path, monkeypatch):
    monkeypatch.setattr(orchestrator_node, "LOG_DIR", str(tmp_path))
    monkeypatch.setattr(
        orchestrator_node,
        "PROCS",
        {
            "base_vanilla": _sleep_cmd(),
            "base_gnss": _sleep_cmd(),
            "estop": [sys.executable, "-c", "print('estop mock')"],
        },
    )
    monkeypatch.setattr(orchestrator_node, "EXCLUSIVE", {"base_vanilla", "base_gnss"})
    node = orchestrator_node.OrchestratorNode()
    probe = ControlProbe()
    try:
        yield node, probe
    finally:
        node.shutdown()
        probe.destroy_node()
        node.destroy_node()


def test_orchestrator_start_kill_status_and_exclusive_switch(orchestrator_pair):
    orch, probe = orchestrator_pair

    probe.publish_start("base_vanilla")
    _spin_until(
        [orch, probe],
        lambda: any(s.get("base_vanilla") is True for s in probe.orch_status),
        what="base_vanilla running status",
    )
    first = [s for s in probe.orch_status if s.get("base_vanilla") is True][-1]
    first_pid = first["_detail"]["base_vanilla"]["pid"]
    assert isinstance(first_pid, int)

    probe.publish_start("base_vanilla")
    time.sleep(0.15)
    _spin_until([orch, probe], lambda: True, timeout=0.1)
    latest = probe.orch_status[-1]
    assert latest["_detail"]["base_vanilla"]["pid"] == first_pid

    probe.publish_start("base_gnss")
    _spin_until(
        [orch, probe],
        lambda: any(s.get("base_gnss") is True and s.get("base_vanilla") is False
                    for s in probe.orch_status),
        what="exclusive switch status",
    )
    switched = probe.orch_status[-1]
    assert switched["_detail"]["base_vanilla"]["intentional_stop_reason"] == (
        "exclusive switch to base_gnss")

    probe.publish_kill("base_gnss")
    _spin_until(
        [orch, probe],
        lambda: any(s.get("base_gnss") is False and
                    s["_detail"]["base_gnss"]["intentional_stop_reason"] == "operator kill"
                    for s in probe.orch_status),
        what="operator kill status",
    )

    n_status = len(probe.orch_status)
    probe.publish_start("unknown")
    _spin_until([orch, probe], lambda: len(probe.orch_status) > n_status, timeout=2.0)
    assert "unknown" not in probe.orch_status[-1]


@pytest.fixture
def ops_pair(ros_context):
    node = ops_node.OpsNode()
    probe = ControlProbe()
    try:
        yield node, probe
    finally:
        probe.destroy_node()
        node.destroy_node()


def test_ops_node_validates_stack_and_stops_motion(ops_pair):
    ops, probe = ops_pair
    probe.publish_orch_status({
        "base_gnss": True,
        "estop": True,
        "odom_zero": True,
        "sequencer": True,
        "_detail": {},
    })
    _spin_until(
        [ops, probe],
        lambda: ops._orch.get("base_gnss") is True,
        what="ops received orchestrator status",
    )
    probe.publish_ops({"action": "validate_stack"})
    _spin_until(
        [ops, probe],
        lambda: any(s.get("last_result") == "validate_stack PASS" for s in probe.ops_status),
        what="validate stack PASS",
    )

    probe.publish_ops({"action": "stop_all_motion"})
    _spin_until(
        [ops, probe],
        lambda: "follower" in probe.kills and "reposition" in probe.kills and True in probe.estop,
        what="stop_all_motion commands",
    )
    assert probe.ops_status[-1]["last_result"].startswith("motion stopped")


def test_ops_node_smoke_e2e_requires_confirmation_then_arms_when_idle(ops_pair):
    ops, probe = ops_pair

    probe.publish_ops({"action": "run_smoke_e2e"})
    _spin_until(
        [ops, probe],
        lambda: any("refused run_smoke_e2e" in s.get("last_result", "")
                    for s in probe.ops_status),
        what="run_smoke_e2e refusal",
    )
    assert "sequencer_smoke" not in probe.starts

    probe.publish_ops({"action": "run_smoke_e2e", "confirmed_wheels_on_floor": True})
    _spin_until(
        [ops, probe],
        lambda: "sequencer" in probe.kills and "sequencer_smoke" in probe.starts,
        what="start smoke sequencer",
    )
    probe.publish_exp_status({"phase": "idle"})
    _spin_until(
        [ops, probe],
        lambda: any(json.loads(x).get("action") == "start" for x in probe.exp_cmds),
        what="arm smoke sequencer when idle",
    )
    assert probe.ops_status[-1]["last_result"] == "smoke sequencer armed"


def test_ops_node_recover_after_failure_is_safe_state(ops_pair):
    ops, probe = ops_pair
    probe.publish_ops({"action": "recover_after_failure"})
    _spin_until(
        [ops, probe],
        lambda: True in probe.estop and "follower" in probe.kills and
        "reposition" in probe.kills and any(json.loads(x).get("action") == "pause"
                                            for x in probe.exp_cmds),
        what="recover_after_failure safe-state commands",
    )
