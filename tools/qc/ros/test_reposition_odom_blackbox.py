"""ros-sim tier: black-box coverage for odom_zero_node and reposition_node.

The tests launch the real nodes on the isolated ros-sim domain and interact
only through their public topic contracts. No real robot graph is reachable
from ROS_DOMAIN_ID=91, and no test publishes to /cmd_vel.
"""

from __future__ import annotations

import json
import math
import os
import signal
import subprocess
import sys
import time

import pytest

pytest.importorskip("rclpy")  # laptop has no ROS -> clean SKIP at collection

import rclpy  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import (  # noqa: E402
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import NavSatFix  # noqa: E402
from std_msgs.msg import Bool, String  # noqa: E402

from tools.path_gen import path_overlay  # noqa: E402


REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
PKG_SRC = os.path.join(REPO_ROOT, "scalecar-vfg-h-infinite", "ros2_bridge")


def _latched(depth: int = 1) -> QoSProfile:
    return QoSProfile(
        depth=depth,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _quat_from_yaw(yaw: float):
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


def _yaw_from_quat(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _odom(x: float, y: float, yaw: float, *, vx: float = 0.0, vy: float = 0.0) -> Odometry:
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
    msg.twist.twist.linear.x = float(vx)
    msg.twist.twist.linear.y = float(vy)
    msg.twist.twist.angular.z = 0.4
    return msg


def _local_to_ll(x: float, y: float) -> tuple[float, float]:
    anchor = path_overlay.Anchor(path_overlay.LAT0, path_overlay.LON0, path_overlay.BEARING_DEG)
    ll = path_overlay.local_to_latlon([[x, y]], anchor)[0]
    return float(ll[0]), float(ll[1])


def _write_venue(path, *, exclusions=()):
    corners = [_local_to_ll(0.0, 0.0), _local_to_ll(8.0, 0.0),
               _local_to_ll(8.0, 5.0), _local_to_ll(0.0, 5.0)]
    data = {
        "name": "ros_sim_reposition",
        "safety_margin_m": 0.2,
        "corners_wgs84": [{"lat": lat, "lon": lon} for lat, lon in corners],
        "exclusions": [],
        "start_pins": [],
        "end_pins": [],
    }
    for x, y, r in exclusions:
        lat, lon = _local_to_ll(x, y)
        data["exclusions"].append(
            {"kind": "circle", "lat": lat, "lon": lon, "radius_m": r}
        )
    path.write_text(json.dumps(data), encoding="utf-8")


class RosProbe(Node):
    def __init__(self):
        super().__init__("qc_blackbox_probe")
        self.odom_zeroed = []
        self.odom_status = []
        self.repo_status = []
        self.cmds = []
        self.pub_odom = self.create_publisher(Odometry, "/wheel/odom", 10)
        self.pub_reset = self.create_publisher(Bool, "/odom_zero/reset", 10)
        self.pub_rtk = self.create_publisher(
            String, "/gps_rtk_f9p_helical/gps/rtk_status", 10)
        self.pub_fix = self.create_publisher(
            NavSatFix, "/gps_rtk_f9p_helical/gps/fix", 10)
        self.pub_goto = self.create_publisher(String, "/reposition/goto", 10)
        self.create_subscription(Odometry, "/wheel/odom_zeroed", self.odom_zeroed.append, 10)
        self.create_subscription(String, "/odom_zero/status", self._on_odom_status, _latched())
        self.create_subscription(String, "/reposition/status", self._on_repo_status, _latched())
        self.create_subscription(Twist, "/cmd_vel_raw", self.cmds.append, 10)

    def _on_odom_status(self, msg: String) -> None:
        try:
            self.odom_status.append(json.loads(msg.data))
        except ValueError:
            self.odom_status.append({"raw": msg.data})

    def _on_repo_status(self, msg: String) -> None:
        try:
            self.repo_status.append(json.loads(msg.data))
        except ValueError:
            self.repo_status.append({"raw": msg.data})

    def publish_reset(self, value: bool) -> None:
        msg = Bool()
        msg.data = bool(value)
        self.pub_reset.publish(msg)

    def publish_rtk(self, quality=4) -> None:
        msg = String()
        msg.data = f"FIX: RTK FIXED (quality={quality}, sats=20)"
        self.pub_rtk.publish(msg)

    def publish_fix_local(self, x: float, y: float) -> None:
        lat, lon = _local_to_ll(x, y)
        msg = NavSatFix()
        msg.latitude = lat
        msg.longitude = lon
        self.pub_fix.publish(msg)

    def publish_goto_local(self, x: float, y: float, heading_deg: float, **extra) -> None:
        lat, lon = _local_to_ll(x, y)
        payload = {"lat": lat, "lon": lon, "heading_deg": heading_deg}
        payload.update(extra)
        msg = String()
        msg.data = json.dumps(payload)
        self.pub_goto.publish(msg)


class NodeProcess:
    def __init__(self, module: str, *, args=None):
        env = dict(os.environ)
        env["ROS_DOMAIN_ID"] = "91"
        env["H_INFINITY_ROOT"] = REPO_ROOT
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

    def assert_running(self):
        if self.proc.poll() is not None:
            out = self.output()
            raise AssertionError(f"node exited with {self.proc.returncode}:\n{out}")

    def output(self) -> str:
        if self.proc.poll() is None:
            return ""
        try:
            out, _ = self.proc.communicate(timeout=0.2)
        except subprocess.TimeoutExpired:
            return ""
        return out or ""

    def stop(self):
        if self.proc.poll() is None:
            os.killpg(self.proc.pid, signal.SIGINT)
            try:
                self.proc.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                os.killpg(self.proc.pid, signal.SIGTERM)
                try:
                    self.proc.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    os.killpg(self.proc.pid, signal.SIGKILL)
                    self.proc.wait(timeout=2.0)
        self.output()


@pytest.fixture
def probe(ros_context):
    node = RosProbe()
    try:
        yield node
    finally:
        node.destroy_node()


def _spin_until(node: RosProbe, predicate, *, timeout=4.0, what="condition"):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return
    raise AssertionError(f"timed out waiting for {what}")


def _latest_status(statuses):
    assert statuses, "no status messages observed"
    return statuses[-1]


def _latest_cmd(cmds):
    assert cmds, "no cmd_vel_raw messages observed"
    return cmds[-1]


def _wait_for_new_cmd(probe: RosProbe, n_before: int, *, timeout=4.0) -> Twist:
    _spin_until(
        probe,
        lambda: len(probe.cmds) > n_before,
        timeout=timeout,
        what="new cmd_vel_raw sample",
    )
    return probe.cmds[-1]


def _nonzero(cmd: Twist) -> bool:
    return abs(cmd.linear.x) > 1e-6 or abs(cmd.angular.z) > 1e-6


def test_odom_zero_reanchors_pose_and_confirms_reset(probe):
    node = NodeProcess("limo_path_follower.odom_zero_node")
    try:
        _spin_until(probe, lambda: probe.odom_status, what="initial odom_zero status")
        assert _latest_status(probe.odom_status)["has_reset"] is False

        probe.publish_reset(True)
        time.sleep(0.2)
        rclpy.spin_once(probe, timeout_sec=0.1)
        assert all(s.get("has_reset") is False for s in probe.odom_status)

        probe.pub_odom.publish(_odom(2.0, 3.0, math.pi / 2.0, vx=0.7, vy=-0.2))
        _spin_until(probe, lambda: len(probe.odom_zeroed) >= 1, what="identity odom")
        ident = probe.odom_zeroed[-1]
        assert ident.pose.pose.position.x == pytest.approx(2.0, abs=1e-6)
        assert ident.pose.pose.position.y == pytest.approx(3.0, abs=1e-6)
        assert _yaw_from_quat(ident.pose.pose.orientation) == pytest.approx(math.pi / 2.0)

        probe.publish_reset(False)
        time.sleep(0.2)
        rclpy.spin_once(probe, timeout_sec=0.1)
        assert all(s.get("has_reset") is False for s in probe.odom_status)

        probe.publish_reset(True)
        _spin_until(
            probe,
            lambda: any(s.get("has_reset") is True for s in probe.odom_status),
            what="latched odom_zero reset",
        )
        latched = _latest_status(probe.odom_status)
        assert latched["origin"]["x"] == pytest.approx(2.0)
        assert latched["origin"]["y"] == pytest.approx(3.0)
        assert latched["origin"]["yaw"] == pytest.approx(math.pi / 2.0)

        n0 = len(probe.odom_zeroed)
        probe.pub_odom.publish(_odom(2.0, 4.0, math.pi, vx=0.7, vy=-0.2))
        _spin_until(probe, lambda: len(probe.odom_zeroed) > n0, what="reanchored odom")
        out = probe.odom_zeroed[-1]
        assert out.pose.pose.position.x == pytest.approx(1.0, abs=1e-6)
        assert out.pose.pose.position.y == pytest.approx(0.0, abs=1e-6)
        assert _wrap(_yaw_from_quat(out.pose.pose.orientation)) == pytest.approx(
            math.pi / 2.0, abs=1e-6)
        assert out.twist.twist.linear.x == pytest.approx(0.7)
        assert out.twist.twist.linear.y == pytest.approx(-0.2)
    finally:
        node.stop()


def _start_reposition(venue_path):
    return NodeProcess(
        "limo_path_follower.reposition_node",
        args=[
            "--ros-args",
            "-p", f"venue_file:={venue_path}",
            "-p", "control_rate_hz:=20.0",
            "-p", "rtk_timeout_s:=0.25",
            "-p", "cog_min_travel_m:=0.05",
            "-p", "three_point_turn_deg:=60.0",
        ],
    )


def _seed_fixed_pose(probe: RosProbe, x: float, y: float):
    probe.publish_rtk(4)
    for _ in range(4):
        probe.publish_fix_local(x, y)
        _spin_until(probe, lambda: True, timeout=0.05)


def test_reposition_rtk_gate_and_dry_run_planning_reasons(tmp_path, probe):
    venue = tmp_path / "venue.json"
    _write_venue(venue, exclusions=[(4.0, 2.5, 0.05)])
    node = _start_reposition(venue)
    try:
        _spin_until(probe, lambda: probe.repo_status, what="initial reposition status")

        probe.publish_rtk(1)
        probe.publish_fix_local(1.0, 1.0)
        probe.publish_goto_local(2.0, 1.0, 42.0, dry_run=True)
        _spin_until(
            probe,
            lambda: any("no RTK FIXED fix" in (s.get("reason") or "")
                        for s in probe.repo_status),
            what="RTK gate rejection",
        )
        gate = _latest_status(probe.repo_status)
        assert gate["state"] == "aborted"

        _seed_fixed_pose(probe, 1.0, 1.0)
        probe.publish_goto_local(2.0, 1.0, 42.0, dry_run=True)
        _spin_until(
            probe,
            lambda: any("dry-run OK" in (s.get("reason") or "")
                        for s in probe.repo_status),
            what="dry-run OK",
        )
        assert not any(_nonzero(c) for c in probe.cmds), "dry-run emitted motion"

        probe.publish_goto_local(9.0, 1.0, 42.0, dry_run=True)
        _spin_until(
            probe,
            lambda: any("target outside inset working area" in (s.get("reason") or "")
                        for s in probe.repo_status),
            what="target outside rejection",
        )

        probe.publish_goto_local(4.0, 2.5, 42.0, dry_run=True)
        _spin_until(
            probe,
            lambda: any("target inside exclusion" in (s.get("reason") or "")
                        for s in probe.repo_status),
            what="target exclusion rejection",
        )

        probe.publish_goto_local(4.3, 2.5, 42.0, dry_run=True)
        _spin_until(
            probe,
            lambda: any("segment enters exclusion" in (s.get("reason") or "")
                        for s in probe.repo_status),
            what="segment exclusion rejection",
        )
    finally:
        node.stop()


def test_reposition_control_branches_zero_creep_reverse_and_arrival(tmp_path, probe):
    venue = tmp_path / "venue.json"
    _write_venue(venue)
    node = _start_reposition(venue)
    try:
        _spin_until(probe, lambda: probe.repo_status, what="initial reposition status")
        _seed_fixed_pose(probe, 1.0, 1.0)

        n_cmd = len(probe.cmds)
        probe.publish_goto_local(3.0, 1.0, 42.0)
        _spin_until(
            probe,
            lambda: any("creeping to acquire COG heading" in (s.get("reason") or "")
                        for s in probe.repo_status),
            what="COG creep branch",
        )
        creep = _wait_for_new_cmd(probe, n_cmd)
        assert creep.linear.x > 0.0
        assert creep.angular.z == pytest.approx(0.0, abs=1e-6)

        n_status = len(probe.repo_status)
        time.sleep(0.35)
        _spin_until(
            probe,
            lambda: any("RTK fix stale" in (s.get("reason") or "")
                        for s in probe.repo_status[n_status:]),
            timeout=2.0,
            what="stale-fix zero branch",
        )
        _spin_until(
            probe,
            lambda: bool(probe.cmds) and not _nonzero(probe.cmds[-1]),
            timeout=2.0,
            what="zero cmd_vel_raw after stale fix",
        )
        stale_cmd = probe.cmds[-1]
        assert stale_cmd.linear.x == pytest.approx(0.0, abs=1e-6)
        assert stale_cmd.angular.z == pytest.approx(0.0, abs=1e-6)

        probe.publish_fix_local(1.1, 1.0)  # heading estimate: local +x.
        _spin_until(
            probe,
            lambda: any((s.get("state") in ("aligning", "approaching"))
                        for s in probe.repo_status),
            what="fresh fixed pose after stale",
        )
        n_cmd = len(probe.cmds)
        probe.publish_goto_local(3.0, 4.0, 42.0)  # entry lies mostly north of current.
        _spin_until(
            probe,
            lambda: any("3-point reverse" in (s.get("reason") or "")
                        for s in probe.repo_status),
            what="3-point reverse branch",
        )
        reverse = _wait_for_new_cmd(probe, n_cmd)
        assert reverse.linear.x < 0.0
        assert abs(reverse.angular.z) > 0.0

        _seed_fixed_pose(probe, 3.0, 1.0)
        probe.publish_fix_local(3.1, 1.0)  # heading estimate at target yaw.
        n_cmd = len(probe.cmds)
        probe.publish_goto_local(3.0, 1.0, 42.0)
        _spin_until(
            probe,
            lambda: any("already within tolerance" in (s.get("reason") or "")
                        for s in probe.repo_status),
            what="R4 no-op arrival",
        )
        arrived = _latest_status(probe.repo_status)
        assert arrived["state"] == "arrived"
        assert _wait_for_new_cmd(probe, n_cmd).linear.x == pytest.approx(0.0, abs=1e-6)
    finally:
        node.stop()


def test_reposition_runtime_area_guard_aborts_and_zeros(tmp_path, probe):
    venue = tmp_path / "venue.json"
    _write_venue(venue)
    node = _start_reposition(venue)
    try:
        _spin_until(probe, lambda: probe.repo_status, what="initial reposition status")
        _seed_fixed_pose(probe, 1.0, 1.0)
        probe.publish_goto_local(3.0, 1.0, 42.0)
        _spin_until(
            probe,
            lambda: any("creeping to acquire COG heading" in (s.get("reason") or "")
                        for s in probe.repo_status),
            what="active reposition",
        )

        n_cmd = len(probe.cmds)
        probe.publish_fix_local(20.0, 20.0)
        _spin_until(
            probe,
            lambda: any("live RTK pose left working area" in (s.get("reason") or "")
                        for s in probe.repo_status),
            what="runtime area abort",
        )
        status = _latest_status(probe.repo_status)
        assert status["state"] == "aborted"
        cmd = _wait_for_new_cmd(probe, n_cmd)
        assert cmd.linear.x == pytest.approx(0.0, abs=1e-6)
        assert cmd.angular.z == pytest.approx(0.0, abs=1e-6)
    finally:
        node.stop()
