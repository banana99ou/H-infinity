#!/usr/bin/env python3
"""Bench-world simulator for hands-off matrix dry-runs (TEST HARNESS — NUC only).

WHAT THIS IS
------------
A full stand-in for the LIMO's *sensing + base* layer, so the whole
``experiment_sequencer`` matrix can be walked on a bench with NO real GPS and a
SCRIPTED battery fault. It runs INSTEAD OF the real base driver (kill
``base_vanilla`` first). It impersonates the node name ``limo_base_node`` so
``tools/preflight/preflight.sh`` (which checks the node graph + the safety chain)
passes unchanged.

ONE kinematic model drives everything, so the controller's commands are executed
exactly and the odom/RTK stay consistent:

  sub  /cmd_vel  (geometry_msgs/Twist, post-estop)  -> integrate a unicycle in the
       shared venue/local frame, seeded at the venue start pin.
  pub  /wheel/odom                         nav_msgs/Odometry      (the belief source:
         odom_zero re-zeros it -> /wheel/odom_zeroed -> the follower + the battle-
         station blue dot. The follower's commands move this pose, so s_star reaches
         the path end and legs COMPLETE — the real wheels-in-air odom barely moved,
         which is why v1 legs timed out.)
  pub  /gps_rtk_f9p_helical/gps/fix        sensor_msgs/NavSatFix  (same pose -> lat/lon;
         the RTK magenta dot now AGREES with the odom dot — v1 diverged because the
         two came from different sources.)
  pub  /gps_rtk_f9p_helical/gps/rtk_status std_msgs/String        ("quality=4" FIXED)
  pub  /limo_status                        limo_msgs/LimoStatus   (motion_mode=1 +
         battery; battery drops below the 10.5 V halt after N completed runs to
         exercise M2 + the operator notification.)

WHY synthetic odom (not the real wheels)
---------------------------------------
With the wheels off the ground the real base odom does not advance like real
driving, so the closed-loop follower can't reach the path end -> 180 s timeout ->
circuit breaker. Integrating the controller's own /cmd_vel through a clean
unicycle makes the world execute its intent exactly, so legs complete
deterministically. The motors do NOT spin (no real base driver) — that is
cosmetic; every mission-critical FEATURE is still exercised, and the safety chain
(cmd_vel_raw -> estop_cli -> /cmd_vel) is intact.

A "run" for the battery counter = one rising edge of /path_follower/done.

SAFETY: spoofs RTK FIXED and impersonates the base — strictly a tools/qc bench
harness; keep it OUT of the field stack.
"""

import math

import rclpy  # pyright: ignore[reportMissingImports]
from rclpy.node import Node  # pyright: ignore[reportMissingImports]
from rclpy.executors import ExternalShutdownException  # pyright: ignore[reportMissingImports]
from rclpy.qos import (  # pyright: ignore[reportMissingImports]
    QoSProfile,
    QoSDurabilityPolicy,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
)
from geometry_msgs.msg import Twist  # pyright: ignore[reportMissingImports]
from nav_msgs.msg import Odometry  # pyright: ignore[reportMissingImports]
from sensor_msgs.msg import NavSatFix  # pyright: ignore[reportMissingImports]
from std_msgs.msg import String, Bool  # pyright: ignore[reportMissingImports]

# limo_msgs is a vendor package present only on the NUC. Import defensively so
# the file at least parses on the laptop; the node only ever RUNS on the robot.
try:
    from limo_msgs.msg import LimoStatus  # pyright: ignore[reportMissingImports]
except Exception:  # pragma: no cover - laptop / missing vendor msg
    LimoStatus = None


# --- Shared local<->WGS84 anchor. Copied verbatim from the single source of ---
# truth, tools/path_gen/path_overlay.py (LAT0/LON0/BEARING_DEG/EARTH_R), inlined
# so this harness does NOT import path_overlay (which pulls numpy/requests/PIL at
# module load). The reposition node uses the SAME constants, so a fix we publish
# round-trips back to the (x, y) we integrated.
LAT0 = 37.61174497415274
LON0 = 126.99429176572984
BEARING_DEG = 42.0
EARTH_R = 6378137.0


def _local_to_latlon(x, y):
    b = math.radians(BEARING_DEG)
    east = x * math.sin(b) - y * math.cos(b)
    north = x * math.cos(b) + y * math.sin(b)
    dlat = (north / EARTH_R) * (180.0 / math.pi)
    dlon = (east / (EARTH_R * math.cos(math.radians(LAT0)))) * (180.0 / math.pi)
    return LAT0 + dlat, LON0 + dlon


def _latlon_to_local(lat, lon):
    b = math.radians(BEARING_DEG)
    north = (lat - LAT0) * (math.pi / 180.0) * EARTH_R
    east = (lon - LON0) * (math.pi / 180.0) * EARTH_R * math.cos(math.radians(LAT0))
    x = east * math.sin(b) + north * math.cos(b)
    y = -east * math.cos(b) + north * math.sin(b)
    return x, y


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def _bearing_deg_to_local_yaw(heading_deg):
    return _wrap(math.radians(BEARING_DEG - heading_deg))


class BenchWorld(Node):

    def __init__(self):
        # Impersonate the base node so preflight's node-graph + safety-chain
        # checks pass with the real base killed.
        super().__init__('limo_base_node')

        # -- Parameters --------------------------------------------------
        self.declare_parameter('venue_file',
                               '/home/agilex/H-infinity/scenarios/venues/rooftop.json')
        self.declare_parameter('pin_id', 'A')          # seed pose at this start pin
        self.declare_parameter('runs_before_low', 5)   # drop battery after N runs
        self.declare_parameter('low_voltage', 10.3)    # < halt (10.5) => M2 halt
        self.declare_parameter('healthy_voltage', 12.5)
        self.declare_parameter('integrate_hz', 50.0)
        self.declare_parameter('fix_hz', 15.0)
        self.declare_parameter('status_hz', 5.0)
        self.declare_parameter('cmd_timeout_s', 0.3)   # hold pose if cmd_vel silent
        self.declare_parameter('odom_topic', '/wheel/odom')
        self.declare_parameter('limo_status_topic', '/limo_status')

        self._runs_before_low = int(self.get_parameter('runs_before_low').value)
        self._low_v = float(self.get_parameter('low_voltage').value)
        self._healthy_v = float(self.get_parameter('healthy_voltage').value)
        self._cmd_timeout = float(self.get_parameter('cmd_timeout_s').value)
        odom_topic = str(self.get_parameter('odom_topic').value)
        status_topic = str(self.get_parameter('limo_status_topic').value)

        # -- Seed venue-frame pose at the start pin ----------------------
        self._x, self._y, self._yaw = self._seed_pose()

        # -- Command + run state -----------------------------------------
        self._v = 0.0
        self._w = 0.0
        self._last_cmd_t = None          # ros time of last /cmd_vel
        self._prev_done = None           # for /path_follower/done edge counting
        self._runs = 0
        self._battery_low = False

        # -- Publishers --------------------------------------------------
        self.pub_odom = self.create_publisher(Odometry, odom_topic, 10)
        self.pub_fix = self.create_publisher(
            NavSatFix, '/gps_rtk_f9p_helical/gps/fix', 10)
        self.pub_rtk = self.create_publisher(
            String, '/gps_rtk_f9p_helical/gps/rtk_status', 10)
        self.pub_status = (
            self.create_publisher(LimoStatus, status_topic, 10)
            if LimoStatus is not None else None)

        # -- Subscriptions -----------------------------------------------
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd, 10)
        # /path_follower/done is latched (transient_local) — match it so we
        # actually receive the edges.
        done_qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(Bool, '/path_follower/done', self._on_done, done_qos)

        # -- Timers ------------------------------------------------------
        self.create_timer(1.0 / float(self.get_parameter('integrate_hz').value),
                          self._integrate)
        self.create_timer(1.0 / float(self.get_parameter('fix_hz').value),
                          self._pub_fix)
        self.create_timer(1.0 / float(self.get_parameter('status_hz').value),
                          self._pub_rtk_status)
        self.create_timer(0.2, self._pub_limo_status)   # 5 Hz battery
        self.create_timer(2.0, self._log_state)

        lat0, lon0 = _local_to_latlon(self._x, self._y)
        self.get_logger().info(
            f"[BENCH SIM as limo_base_node] up. seeded at pin "
            f"'{self.get_parameter('pin_id').value}' local=({self._x:.2f},{self._y:.2f}) "
            f"yaw={math.degrees(self._yaw):.1f}deg => ({lat0:.7f},{lon0:.7f}). "
            f"synthetic odom -> {odom_topic}; RTK quality=4; limo_status -> "
            f"{status_topic}; battery <{self._low_v}V after {self._runs_before_low} runs."
            + ("" if LimoStatus is not None else
               "  WARNING: limo_msgs not importable — /limo_status DISABLED."))

    # ------------------------------------------------------------------
    def _seed_pose(self):
        import json
        path = str(self.get_parameter('venue_file').value)
        pin_id = str(self.get_parameter('pin_id').value)
        try:
            with open(path, 'r') as f:
                v = json.load(f)
            pins = (v.get('start_pins') or []) + (v.get('end_pins') or [])
            pin = next((p for p in pins if p.get('id') == pin_id), None)
            if pin is None and pins:
                pin = pins[0]
            if pin is None:
                raise ValueError('no pins in venue')
            x, y = _latlon_to_local(float(pin['lat']), float(pin['lon']))
            yaw = _bearing_deg_to_local_yaw(float(pin.get('heading_deg', 0.0)))
            return x, y, yaw
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                f'could not seed pose from {path} ({exc}); starting at origin.')
            return 0.0, 0.0, 0.0

    # ------------------------------------------------------------------
    def _on_cmd(self, msg: Twist):
        self._v = float(msg.linear.x)
        self._w = float(msg.angular.z)
        self._last_cmd_t = self.get_clock().now()

    def _on_done(self, msg: Bool):
        # Count rising edges (False -> True). The first message only sets the
        # baseline so a latched seed (possibly stale True) never miscounts.
        if self._prev_done is None:
            self._prev_done = bool(msg.data)
            return
        if bool(msg.data) and not self._prev_done:
            self._runs += 1
            self.get_logger().info(f'completed run #{self._runs} (/path_follower/done edge)')
            if self._runs >= self._runs_before_low and not self._battery_low:
                self._battery_low = True
                self.get_logger().warn(
                    f'>= {self._runs_before_low} runs done — battery now forced to '
                    f'{self._low_v}V (< halt). Expect M2 pause + notify at the next '
                    f'cell boundary (PREFLIGHT/NEXT).')
        self._prev_done = bool(msg.data)

    # ------------------------------------------------------------------
    def _integrate(self):
        dt = 1.0 / float(self.get_parameter('integrate_hz').value)
        v, w = self._v, self._w
        # Hold pose if no fresh command (idle between legs => no /cmd_vel
        # publisher; don't keep coasting on a stale command).
        if self._last_cmd_t is None:
            v = w = 0.0
        else:
            age = (self.get_clock().now() - self._last_cmd_t).nanoseconds * 1e-9
            if age > self._cmd_timeout:
                v = w = 0.0
        self._x += v * math.cos(self._yaw) * dt
        self._y += v * math.sin(self._yaw) * dt
        self._yaw = _wrap(self._yaw + w * dt)
        self._pub_odom(v, w)

    def _pub_odom(self, v, w):
        m = Odometry()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'odom'
        m.child_frame_id = 'base_link'
        m.pose.pose.position.x = self._x
        m.pose.pose.position.y = self._y
        m.pose.pose.orientation.z = math.sin(self._yaw / 2.0)
        m.pose.pose.orientation.w = math.cos(self._yaw / 2.0)
        m.twist.twist.linear.x = float(v)
        m.twist.twist.angular.z = float(w)
        self.pub_odom.publish(m)

    def _pub_fix(self):
        lat, lon = _local_to_latlon(self._x, self._y)
        m = NavSatFix()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'gps'
        m.status.status = 0   # STATUS_FIX
        m.status.service = 1  # SERVICE_GPS
        m.latitude = lat
        m.longitude = lon
        m.altitude = 0.0
        m.position_covariance_type = 0
        self.pub_fix.publish(m)

    def _pub_rtk_status(self):
        self.pub_rtk.publish(String(data='quality=4 (RTK FIXED) [bench-sim]'))

    def _pub_limo_status(self):
        if self.pub_status is None:
            return
        msg = LimoStatus()
        try:
            msg.battery_voltage = self._low_v if self._battery_low else self._healthy_v
            msg.motion_mode = 1   # Ackermann (preflight gate)
        except Exception:  # noqa: BLE001 - field-name drift across vendor versions
            pass
        self.pub_status.publish(msg)

    def _log_state(self):
        lat, lon = _local_to_latlon(self._x, self._y)
        batt = (f'{self._low_v}V FAULTED' if self._battery_low
                else f'{self._healthy_v}V')
        self.get_logger().info(
            f'pose local=({self._x:.2f},{self._y:.2f}) yaw={math.degrees(self._yaw):.0f}deg '
            f'=> ({lat:.7f},{lon:.7f}) | cmd v={self._v:.2f} w={self._w:.2f} | '
            f'runs={self._runs}/{self._runs_before_low} | batt={batt}')


def main(args=None):
    rclpy.init(args=args)
    node = BenchWorld()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
