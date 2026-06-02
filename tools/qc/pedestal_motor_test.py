#!/usr/bin/env python3
"""Pedestal MOTOR test with synthetic feedback — NOT a passive smoke test.

DANGER: this energizes the drive motors. It CLEARS the E-stop and SPOOFS RTK
FIXED (plus synthetic odom/battery) to push commands through the real safety
chain, so the wheels WILL turn. Run ONLY with the robot on a pedestal / wheels
off the ground. Intentionally outside the default/laptop/ros-sim tiers;
run_qc.py field-gated requires --confirm-wheels-on-floor + --confirm-pedestal +
--allow-motor-energize before invoking it.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, String

try:
    from limo_msgs.msg import LimoStatus  # type: ignore
except Exception:  # pragma: no cover - depends on NUC install
    LimoStatus = None

from rclpy.node import Node

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, REPO_ROOT)
from tools.path_gen import path_overlay as po  # noqa: E402


def _run(cmd, timeout=8.0):
    print("+ " + " ".join(cmd), flush=True)
    return subprocess.run(cmd, cwd=REPO_ROOT, text=True, timeout=timeout, check=False)


def _quat_from_yaw(yaw):
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


class SyntheticFeedback(Node):
    def __init__(self, venue_path: str):
        super().__init__("pedestal_synthetic_feedback")
        self.anchor = po.Anchor(po.LAT0, po.LON0, po.BEARING_DEG)
        with open(venue_path, "r", encoding="utf-8") as f:
            self.venue = json.load(f)
        pin = (self.venue.get("start_pins") or self.venue.get("end_pins") or [{}])[0]
        self.fix_lat = float(pin.get("lat", po.LAT0))
        self.fix_lon = float(pin.get("lon", po.LON0))
        self.raw_x = 0.0
        self.raw_y = 0.0
        self.raw_yaw = 0.0
        self.recipe_started_at = None
        self.goto_target = None
        self.exp_status = []
        self.cmd_raw = []
        self.cmd = []
        self.estop = []

        self.pub_rtk = self.create_publisher(
            String, "/gps_rtk_f9p_helical/gps/rtk_status", 10)
        self.pub_fix = self.create_publisher(
            NavSatFix, "/gps_rtk_f9p_helical/gps/fix", 10)
        self.pub_odom = self.create_publisher(Odometry, "/wheel/odom", 10)
        self.pub_batt = (
            self.create_publisher(LimoStatus, "/limo_status", 10)
            if LimoStatus is not None else None
        )
        self.pub_exp_cmd = self.create_publisher(String, "/experiment/cmd", 10)
        self.pub_estop_trigger = self.create_publisher(Bool, "/estop_trigger", 10)

        self.create_subscription(String, "/reposition/goto", self._on_goto, 10)
        self.create_subscription(String, "/reference_path_recipe", self._on_recipe, 10)
        self.create_subscription(String, "/experiment/status", self._on_exp, 10)
        self.create_subscription(Twist, "/cmd_vel_raw", self.cmd_raw.append, 10)
        self.create_subscription(Twist, "/cmd_vel", self.cmd.append, 10)
        self.create_subscription(Bool, "/estop", lambda m: self.estop.append(bool(m.data)), 10)
        self.create_timer(0.05, self._tick)

    def _on_goto(self, msg):
        try:
            d = json.loads(msg.data)
            lat = float(d["lat"])
            lon = float(d["lon"])
            heading = float(d.get("heading_deg", 0.0))
        except Exception:
            return
        xy = po.latlon_to_local([[lat, lon]], self.anchor)[0]
        local_yaw = math.radians(self.anchor.bearing_deg - heading)
        self.goto_target = {
            "started": time.monotonic(),
            "target_xy": (float(xy[0]), float(xy[1])),
            "local_yaw": local_yaw,
            "lat": lat,
            "lon": lon,
        }

    def _on_recipe(self, _msg):
        self.recipe_started_at = time.monotonic()

    def _on_exp(self, msg):
        try:
            self.exp_status.append(json.loads(msg.data))
        except ValueError:
            return
        d = self.exp_status[-1]
        if d.get("phase") == "idle":
            self.pub_exp_cmd.publish(String(data=json.dumps({"action": "start"})))

    def clear_estop(self):
        self.pub_estop_trigger.publish(Bool(data=False))

    def latch_estop(self):
        self.pub_estop_trigger.publish(Bool(data=True))

    def _tick(self):
        if self.goto_target is not None:
            age = time.monotonic() - self.goto_target["started"]
            tx, ty = self.goto_target["target_xy"]
            yaw = self.goto_target["local_yaw"]
            back = max(0.0, 0.25 * (1.0 - min(age / 0.8, 1.0)))
            xy = (tx - back * math.cos(yaw), ty - back * math.sin(yaw))
            ll = po.local_to_latlon([xy], self.anchor)[0]
            self.fix_lat = float(ll[0])
            self.fix_lon = float(ll[1])

        if self.recipe_started_at is not None:
            age = time.monotonic() - self.recipe_started_at
            self.raw_x = min(age * 0.45, 30.0)
            self.raw_y = 0.0
            self.raw_yaw = 0.0

        mr = String()
        mr.data = "FIX: RTK FIXED (quality=4, sats=22, HDOP=0.6) | RTCM: OK"
        self.pub_rtk.publish(mr)

        mf = NavSatFix()
        mf.latitude = self.fix_lat
        mf.longitude = self.fix_lon
        mf.altitude = 30.0
        self.pub_fix.publish(mf)

        mo = Odometry()
        mo.header.frame_id = "odom"
        mo.child_frame_id = "base_link"
        mo.pose.pose.position.x = self.raw_x
        mo.pose.pose.position.y = self.raw_y
        qx, qy, qz, qw = _quat_from_yaw(self.raw_yaw)
        mo.pose.pose.orientation.x = qx
        mo.pose.pose.orientation.y = qy
        mo.pose.pose.orientation.z = qz
        mo.pose.pose.orientation.w = qw
        mo.twist.twist.linear.x = 0.45 if self.recipe_started_at else 0.0
        self.pub_odom.publish(mo)

        if self.pub_batt is not None:
            mb = LimoStatus()
            mb.battery_voltage = 11.7
            self.pub_batt.publish(mb)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Run pedestal smoke with synthetic feedback.")
    ap.add_argument("--confirm-pedestal", action="store_true")
    ap.add_argument("--allow-motor-energize", action="store_true")
    ap.add_argument("--timeout", type=float, default=180.0)
    ap.add_argument("--venue", default=os.path.join(REPO_ROOT, "scenarios/venues/smoke_2026_05_29.json"))
    args = ap.parse_args(argv)

    if not args.confirm_pedestal or not args.allow_motor_energize:
        print("REFUSE: require --confirm-pedestal and --allow-motor-energize.", file=sys.stderr)
        return 2

    rclpy.init()
    node = SyntheticFeedback(args.venue)
    try:
        # Bring the minimal stack up through the CLI/orchestrator path.
        _run([sys.executable, "tools/ops/limo_ops.py", "start", "estop"], timeout=8)
        _run([sys.executable, "tools/ops/limo_ops.py", "start", "base_gnss", "--allow-motion-capable"], timeout=8)
        _run([sys.executable, "tools/ops/limo_ops.py", "start", "odom_zero"], timeout=8)
        _run([sys.executable, "tools/ops/limo_ops.py", "run-smoke", "--armed"], timeout=8)
        time.sleep(1.0)
        node.clear_estop()

        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if node.exp_status:
                s = node.exp_status[-1]
                print(
                    f"phase={s.get('phase')} pass={s.get('pass')} "
                    f"fail={s.get('fail')} message={s.get('message', '')}",
                    flush=True,
                )
                if s.get("phase") == "done":
                    node.latch_estop()
                    return 0 if int(s.get("fail", 0)) == 0 else 1
                if s.get("phase") == "paused" and int(s.get("fail", 0)) > 0:
                    node.latch_estop()
                    return 1
        print("pedestal smoke timed out", file=sys.stderr)
        node.latch_estop()
        return 1
    finally:
        node.latch_estop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
