"""ros-sim tier: estop_cli safety-filter contract.

Runs the real EstopCliNode in-process on the isolated ros-sim domain. The tests
publish only /cmd_vel_raw and /estop_trigger and observe /cmd_vel + /estop.
"""

from __future__ import annotations

import os
import sys
import time

import pytest

pytest.importorskip("rclpy")

import rclpy  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from rclpy.node import Node  # noqa: E402
from std_msgs.msg import Bool  # noqa: E402

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from estop_cli import EstopCliNode  # noqa: E402


class EstopProbe(Node):
    def __init__(self):
        super().__init__("qc_estop_probe")
        self.cmd_vel = []
        self.estop = []
        self.pub_raw = self.create_publisher(Twist, "/cmd_vel_raw", 10)
        self.pub_trigger = self.create_publisher(Bool, "/estop_trigger", 10)
        self.create_subscription(Twist, "/cmd_vel", self.cmd_vel.append, 10)
        self.create_subscription(Bool, "/estop", lambda m: self.estop.append(bool(m.data)), 10)

    def publish_raw(self, linear=0.0, angular=0.0):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.pub_raw.publish(msg)

    def publish_trigger(self, value: bool):
        msg = Bool()
        msg.data = bool(value)
        self.pub_trigger.publish(msg)


def _spin_until(nodes, predicate, *, timeout=3.0, what="condition"):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        for node in nodes:
            rclpy.spin_once(node, timeout_sec=0.02)
        if predicate():
            return
    raise AssertionError(f"timed out waiting for {what}")


@pytest.fixture
def estop_pair(ros_context):
    estop = EstopCliNode(ping_enabled=False)
    probe = EstopProbe()
    try:
        yield estop, probe
    finally:
        probe.destroy_node()
        estop.destroy_node()


def test_estop_filters_cmd_vel_raw_latch_clear(estop_pair):
    estop, probe = estop_pair

    probe.publish_raw(linear=0.42, angular=-0.3)
    _spin_until([estop, probe], lambda: probe.cmd_vel, what="/cmd_vel pass-through")
    assert probe.cmd_vel[-1].linear.x == pytest.approx(0.42)
    assert probe.cmd_vel[-1].angular.z == pytest.approx(-0.3)

    probe.publish_trigger(True)
    _spin_until(
        [estop, probe],
        lambda: bool(probe.estop) and probe.estop[-1] is True,
        what="latched estop state",
    )
    _spin_until(
        [estop, probe],
        lambda: bool(probe.cmd_vel) and probe.cmd_vel[-1].linear.x == pytest.approx(0.0),
        what="zero command on latch",
    )

    n = len(probe.cmd_vel)
    probe.publish_raw(linear=0.7, angular=0.2)
    _spin_until([estop, probe], lambda: len(probe.cmd_vel) > n, what="latched zero")
    assert probe.cmd_vel[-1].linear.x == pytest.approx(0.0)
    assert probe.cmd_vel[-1].angular.z == pytest.approx(0.0)

    probe.publish_trigger(False)
    _spin_until(
        [estop, probe],
        lambda: bool(probe.estop) and probe.estop[-1] is False,
        what="cleared estop state",
    )
    n = len(probe.cmd_vel)
    probe.publish_raw(linear=0.25, angular=0.1)
    _spin_until([estop, probe], lambda: len(probe.cmd_vel) > n, what="pass-through after clear")
    assert probe.cmd_vel[-1].linear.x == pytest.approx(0.25)
    assert probe.cmd_vel[-1].angular.z == pytest.approx(0.1)


def test_estop_ping_failure_threshold_latches(estop_pair):
    estop, probe = estop_pair
    estop.ping_enabled = True
    estop.ping_targets = ["missing-host"]
    estop.connectivity_status = {"missing-host": True}
    estop.failure_counts = {"missing-host": 0}
    estop.total_misses = {"missing-host": 0}
    estop.ping_host = lambda _host: False

    for _ in range(estop.ping_threshold):
        estop.check_connectivity()

    _spin_until(
        [estop, probe],
        lambda: bool(probe.estop) and probe.estop[-1] is True,
        what="ping-threshold estop latch",
    )
    assert estop.estop_active is True
    assert estop.failure_counts["missing-host"] == estop.ping_threshold
