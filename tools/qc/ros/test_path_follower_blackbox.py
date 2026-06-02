"""ros-sim tier: path_follower_node public-topic behavior."""

from __future__ import annotations

import math
import os
import signal
import subprocess
import sys
import time

import pytest

pytest.importorskip("rclpy")

import rclpy  # noqa: E402
from geometry_msgs.msg import PoseStamped, Twist  # noqa: E402
from nav_msgs.msg import Odometry, Path  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import (  # noqa: E402
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from std_msgs.msg import Bool, Float32MultiArray  # noqa: E402


REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
PKG_SRC = os.path.join(REPO_ROOT, "scalecar-vfg-h-infinite", "ros2_bridge")


def _latched(depth: int = 1) -> QoSProfile:
    return QoSProfile(
        depth=depth,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )


def _quat_from_yaw(yaw: float):
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


def _odom(x: float, y: float, yaw: float) -> Odometry:
    msg = Odometry()
    msg.header.frame_id = "odom"
    msg.child_frame_id = "base_link"
    msg.pose.pose.position.x = float(x)
    msg.pose.pose.position.y = float(y)
    qx, qy, qz, qw = _quat_from_yaw(yaw)
    msg.pose.pose.orientation.x = qx
    msg.pose.pose.orientation.y = qy
    msg.pose.pose.orientation.z = qz
    msg.pose.pose.orientation.w = qw
    msg.twist.twist.linear.x = 0.1
    return msg


def _straight_path(length=2.0) -> Path:
    msg = Path()
    msg.header.frame_id = "odom"
    for x in (0.0, length):
        ps = PoseStamped()
        ps.header.frame_id = "odom"
        ps.pose.position.x = float(x)
        ps.pose.orientation.w = 1.0
        msg.poses.append(ps)
    return msg


class NodeProcess:
    def __init__(self, module: str, *, args=None):
        env = dict(os.environ)
        env["ROS_DOMAIN_ID"] = "91"
        env["PYTHONPATH"] = os.pathsep.join(
            p for p in [PKG_SRC, REPO_ROOT, env.get("PYTHONPATH", "")] if p
        )
        self.proc = subprocess.Popen(
            [sys.executable, "-m", module, *(args or [])],
            cwd=REPO_ROOT,
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
        )

    def stop(self):
        if self.proc.poll() is None:
            os.killpg(self.proc.pid, signal.SIGINT)
            try:
                self.proc.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                os.killpg(self.proc.pid, signal.SIGTERM)
                self.proc.wait(timeout=2.0)
        try:
            self.proc.communicate(timeout=0.2)
        except subprocess.TimeoutExpired:
            pass


class FollowerProbe(Node):
    def __init__(self):
        super().__init__("qc_path_follower_probe")
        self.cmds = []
        self.status = []
        self.done = []
        self.pub_odom = self.create_publisher(Odometry, "/wheel/odom", 10)
        self.pub_path = self.create_publisher(Path, "/reference_path", _latched())
        self.pub_reset = self.create_publisher(Bool, "/path_follower/reset", 10)
        self.create_subscription(Twist, "/cmd_vel_raw", self.cmds.append, 10)
        self.create_subscription(Float32MultiArray, "/path_follower/status", self.status.append, 10)
        self.create_subscription(Bool, "/path_follower/done", lambda m: self.done.append(bool(m.data)), _latched())

    def publish_reset(self):
        msg = Bool()
        msg.data = True
        self.pub_reset.publish(msg)


def _spin_until(node: FollowerProbe, predicate, *, timeout=4.0, what="condition"):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return
    raise AssertionError(f"timed out waiting for {what}")


@pytest.fixture
def follower(ros_context):
    probe = FollowerProbe()
    proc = NodeProcess(
        "limo_path_follower.path_follower_node",
        args=["--ros-args", "-p", "v_const:=0.2", "-p", "dt_ctrl:=0.05"],
    )
    try:
        yield probe
    finally:
        proc.stop()
        probe.destroy_node()


def _has_path(status_msg) -> float:
    return float(status_msg.data[10])


def test_path_follower_silent_without_path(follower):
    _spin_until(follower, lambda: follower.status, what="status without path")
    time.sleep(0.25)
    rclpy.spin_once(follower, timeout_sec=0.1)
    assert _has_path(follower.status[-1]) == pytest.approx(0.0)
    assert follower.cmds == []
    assert follower.done and follower.done[-1] is False


def test_path_follower_drives_times_out_resets_and_latches_done(follower):
    _spin_until(follower, lambda: follower.status, what="follower startup status")
    path = _straight_path(2.0)
    deadline = time.monotonic() + 4.0
    while time.monotonic() < deadline and not (
        follower.status and _has_path(follower.status[-1]) == pytest.approx(1.0)
    ):
        follower.pub_path.publish(path)
        follower.pub_odom.publish(_odom(0.0, 0.2, 0.0))
        rclpy.spin_once(follower, timeout_sec=0.05)
    assert follower.status and _has_path(follower.status[-1]) == pytest.approx(1.0)

    for _ in range(5):
        follower.pub_odom.publish(_odom(0.0, 0.2, 0.0))
        rclpy.spin_once(follower, timeout_sec=0.05)
    _spin_until(
        follower,
        lambda: any(abs(c.linear.x) > 1e-6 for c in follower.cmds),
        what="nonzero cmd_vel_raw with path and odom",
    )
    active = follower.cmds[-1]
    assert active.linear.x == pytest.approx(0.2)
    assert abs(active.angular.z) <= 0.2 * math.tan(0.5) / 0.2 + 1e-6
    assert _has_path(follower.status[-1]) == pytest.approx(1.0)

    n_cmd = len(follower.cmds)
    time.sleep(0.65)
    _spin_until(
        follower,
        lambda: len(follower.cmds) > n_cmd and follower.cmds[-1].linear.x == pytest.approx(0.0),
        timeout=3.0,
        what="zero after odom timeout",
    )

    follower.pub_odom.publish(_odom(1.9, 0.0, 0.0))
    _spin_until(
        follower,
        lambda: bool(follower.done) and follower.done[-1] is True,
        timeout=4.0,
        what="done latch near path end",
    )
    assert follower.cmds[-1].linear.x == pytest.approx(0.0)

    follower.publish_reset()
    _spin_until(
        follower,
        lambda: bool(follower.done) and follower.done[-1] is False,
        timeout=3.0,
        what="done cleared on reset",
    )
    _spin_until(
        follower,
        lambda: bool(follower.status) and _has_path(follower.status[-1]) == pytest.approx(0.0),
        timeout=3.0,
        what="status has_path cleared",
    )
