#!/usr/bin/env python3
"""Ad-hoc reposition diagnostic logger. Subscribes to fused heading, RTK fix,
reposition status, and cmd_vel_raw; prints a 5 Hz CSV so we can compare the
fused heading against the *actual* course-over-ground (derived from the RTK
track) during a reposition drive. Read-only. Run with python3 directly (no
colcon needed) after sourcing ROS + the workspace.
"""
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy,
                       QoSHistoryPolicy)
from std_msgs.msg import Float64, String
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist


def _bearing(p0, p1):
    dE = (p1[1] - p0[1]) * 111320.0 * math.cos(math.radians(p0[0]))
    dN = (p1[0] - p0[0]) * 111320.0
    d = math.degrees(math.atan2(dE, dN))
    return (d + 360.0) % 360.0, math.hypot(dE, dN)


class Mon(Node):
    def __init__(self):
        super().__init__('repo_mon')
        be = QoSProfile(depth=50, reliability=QoSReliabilityPolicy.BEST_EFFORT,
                        history=QoSHistoryPolicy.KEEP_LAST)
        latched = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             history=QoSHistoryPolicy.KEEP_LAST)
        rel = QoSProfile(depth=50, reliability=QoSReliabilityPolicy.RELIABLE,
                         history=QoSHistoryPolicy.KEEP_LAST)
        self.fused = None
        self.fix = None
        self.prev_fix = None
        self.st = {}
        self.cmd = (None, None)
        self.create_subscription(Float64, '/heading/fused', self._f, latched)
        self.create_subscription(NavSatFix,
                                 '/gps_rtk_f9p_helical/gps/fix', self._g, be)
        self.create_subscription(String, '/reposition/status', self._s, latched)
        self.create_subscription(Twist, '/cmd_vel_raw', self._c, rel)
        self.t0 = time.monotonic()
        self.create_timer(0.2, self._tick)
        print("t,fused,cog,cog_travel_m,lat,lon,state,err_m,err_deg,lin,ang",
              flush=True)

    def _f(self, m):
        self.fused = m.data

    def _g(self, m):
        self.fix = (m.latitude, m.longitude)

    def _s(self, m):
        try:
            self.st = json.loads(m.data) or {}
        except Exception:
            self.st = {}

    def _c(self, m):
        self.cmd = (m.linear.x, m.angular.z)

    def _tick(self):
        t = time.monotonic() - self.t0
        cog = ''
        travel = ''
        if self.fix is not None and self.prev_fix is not None:
            b, d = _bearing(self.prev_fix, self.fix)
            if d > 0.03:                     # only report COG once we've moved
                cog = f"{b:.1f}"
                travel = f"{d:.3f}"
        if self.fix is not None:
            self.prev_fix = self.fix
        la, lo = (self.fix or (None, None))
        st = self.st or {}
        fused = '' if self.fused is None else f"{self.fused:.1f}"
        print(f"{t:.2f},{fused},{cog},{travel},{la},{lo},"
              f"{st.get('state')},{st.get('err_m')},{st.get('err_deg')},"
              f"{self.cmd[0]},{self.cmd[1]}", flush=True)


def main():
    rclpy.init()
    try:
        rclpy.spin(Mon())
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
