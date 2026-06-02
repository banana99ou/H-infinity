#!/usr/bin/env python3
"""Bench-world simulator for hands-off matrix dry-runs (TEST HARNESS — NUC only).

WHAT THIS IS
------------
A stand-in for the *sensing* layer the autonomous test-runner gates on, so the
full ``experiment_sequencer`` matrix can be walked on a bench (the LIMO up on a
pedestal, **wheels OFF the ground**) with NO real GPS and a SCRIPTED battery
fault. It does NOT move the robot and it does NOT touch the safety chain.

It supplies the two things the real ``base_vanilla`` driver does not:

1. **Synthetic RTK** (there is no GPS FIXED indoors). It integrates the
   *commanded* velocity (``/cmd_vel`` — the post-estop signal that also drives
   the real wheels) through a unicycle model in the shared venue/local frame,
   seeded at the venue start pin, and publishes::

       /gps_rtk_f9p_helical/gps/fix         sensor_msgs/NavSatFix
       /gps_rtk_f9p_helical/gps/rtk_status  std_msgs/String   ("quality=4")

   Because the fix MOVES in response to commands, reposition's
   course-over-ground heading resolves and it converges on the pin. A *static*
   fix would make reposition creep forever and time out (it has no compass; it
   infers heading from RTK displacement — see reposition_node.py).

2. **A scripted battery fault.** The real base owns ``/limo_status`` (real volts
   + motion_mode); we must not collide with it. Instead we re-publish a copy on
   ``/limo_status_bench`` (the sequencer is remapped onto it with
   ``-r /limo_status:=/limo_status_bench``) and, after ``runs_before_low``
   completed follower runs, force ``battery_voltage`` below the M2 halt
   threshold. That lets us watch the sequencer detect low battery, pause, and
   fire the operator notification ("come pick it up").

SAFETY CONTRACT (read before running)
-------------------------------------
* Run alongside ``base_vanilla`` — NOT ``base_gnss``. base_gnss's real RTK/MAVROS
  stack publishes the same ``/gps_rtk_f9p_helical/*`` topics and would collide
  with the synthetic fix.
* **Wheels MUST be off the ground.** This node makes the system believe RTK is
  FIXED, so the movers WILL energize and spin the motors. No translation happens
  only because the wheels are in the air.
* This node never publishes ``cmd_vel`` / ``cmd_vel_raw`` / ``estop``. It only
  reads ``/cmd_vel`` and publishes *sensor* topics. The
  ``cmd_vel_raw -> estop_cli -> /cmd_vel`` chain is untouched.
* It spoofs RTK FIXED. Keep it OUT of the field stack — it is a tools/qc harness.

A "run" for the battery counter = one rising edge of ``/path_follower/done``
(one completed follower leg, which includes U-turn legs).
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
from sensor_msgs.msg import NavSatFix  # pyright: ignore[reportMissingImports]
from std_msgs.msg import String, Bool  # pyright: ignore[reportMissingImports]

# limo_msgs is a vendor package present only on the NUC. Import defensively so
# the file at least parses on the laptop; the node only ever RUNS on the robot.
try:
    from limo_msgs.msg import LimoStatus  # pyright: ignore[reportMissingImports]
except Exception:  # pragma: no cover - laptop / missing vendor msg
    LimoStatus = None


# --- Shared local<->WGS84 anchor. Copied verbatim from the single source of ---
# truth, tools/path_gen/path_overlay.py (LAT0/LON0/BEARING_DEG/EARTH_R +
# local_to_latlon / latlon_to_local), inlined here so this harness does NOT
# import path_overlay (which pulls in numpy/requests/PIL at module load). The
# reposition node uses the SAME constants, so a fix we publish round-trips back
# to the (x, y) we integrated. If the anchor ever changes in path_overlay,
# update it here too.
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
    # Mirror reposition_node._bearing_deg_to_local_yaw: a compass bearing maps to
    # local yaw via radians(anchor.bearing_deg - heading_deg).
    return _wrap(math.radians(BEARING_DEG - heading_deg))


class BenchWorld(Node):

    def __init__(self):
        super().__init__('bench_world')

        # -- Parameters --------------------------------------------------
        self.declare_parameter('venue_file',
                               '/home/agilex/H-infinity/scenarios/venues/rooftop.json')
        self.declare_parameter('pin_id', 'A')          # seed pose at this start pin
        self.declare_parameter('runs_before_low', 5)   # drop battery after N runs
        self.declare_parameter('low_voltage', 10.3)    # < halt (10.5) => M2 halt
        self.declare_parameter('healthy_voltage', 12.0)  # used until real status seen
        self.declare_parameter('integrate_hz', 50.0)
        self.declare_parameter('fix_hz', 15.0)
        self.declare_parameter('status_hz', 5.0)
        self.declare_parameter('cmd_timeout_s', 0.3)   # hold pose if cmd_vel silent
        self.declare_parameter('limo_status_in', '/limo_status')
        self.declare_parameter('limo_status_out', '/limo_status_bench')

        self._runs_before_low = int(self.get_parameter('runs_before_low').value)
        self._low_v = float(self.get_parameter('low_voltage').value)
        self._healthy_v = float(self.get_parameter('healthy_voltage').value)
        self._cmd_timeout = float(self.get_parameter('cmd_timeout_s').value)
        status_in = str(self.get_parameter('limo_status_in').value)
        status_out = str(self.get_parameter('limo_status_out').value)

        # -- Seed venue-frame pose at the start pin ----------------------
        self._x, self._y, self._yaw = self._seed_pose()

        # -- Command + run state -----------------------------------------
        self._v = 0.0
        self._w = 0.0
        self._last_cmd_t = None          # ros time of last /cmd_vel
        self._prev_done = None           # for /path_follower/done edge counting
        self._runs = 0
        self._battery_low = False
        self._real_status = None         # latest real LimoStatus (or None)
        self._motion_mode_default = 1    # Ackermann, for the synthesized fallback

        # -- Publishers --------------------------------------------------
        self.pub_fix = self.create_publisher(
            NavSatFix, '/gps_rtk_f9p_helical/gps/fix', 10)
        self.pub_rtk = self.create_publisher(
            String, '/gps_rtk_f9p_helical/gps/rtk_status', 10)
        self.pub_status = (
            self.create_publisher(LimoStatus, status_out, 10)
            if LimoStatus is not None else None)

        # -- Subscriptions -----------------------------------------------
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd, 10)
        if LimoStatus is not None:
            self.create_subscription(LimoStatus, status_in, self._on_status, 10)
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
        self.create_timer(0.2, self._pub_limo_status)   # 5 Hz battery republish
        self.create_timer(2.0, self._log_state)

        lat0, lon0 = _local_to_latlon(self._x, self._y)
        self.get_logger().info(
            f"bench_world up. seeded at pin '{self.get_parameter('pin_id').value}' "
            f"local=({self._x:.2f},{self._y:.2f}) yaw={math.degrees(self._yaw):.1f}deg "
            f"=> ({lat0:.7f},{lon0:.7f}). Battery drops <{self._low_v}V after "
            f"{self._runs_before_low} runs. RTK forced quality=4 (FIXED). "
            f"limo_status {status_in} -> {status_out}."
            + ("" if LimoStatus is not None else
               "  WARNING: limo_msgs not importable — battery overlay DISABLED."))

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

    def _on_status(self, msg):
        self._real_status = msg

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
        m.position_covariance_type = 0  # COVARIANCE_TYPE_UNKNOWN
        self.pub_fix.publish(m)

    def _pub_rtk_status(self):
        self.pub_rtk.publish(String(data='quality=4 (RTK FIXED) [bench-sim]'))

    def _pub_limo_status(self):
        if self.pub_status is None:
            return
        if self._real_status is not None:
            # Pass the real telemetry through, overriding only the voltage once
            # the fault is armed (preserves motion_mode + every other field).
            msg = self._real_status
            if self._battery_low:
                msg.battery_voltage = self._low_v
            self.pub_status.publish(msg)
        else:
            # No real /limo_status yet (base not up, or differential mode): emit
            # a healthy synthetic one so the sequencer's M2 gate has a value.
            msg = LimoStatus()
            try:
                msg.battery_voltage = self._low_v if self._battery_low else self._healthy_v
                msg.motion_mode = self._motion_mode_default
            except Exception:  # noqa: BLE001 - field name drift across vendor versions
                pass
            self.pub_status.publish(msg)

    def _log_state(self):
        lat, lon = _local_to_latlon(self._x, self._y)
        batt = (f'{self._low_v}V FAULTED' if self._battery_low
                else (f'{self._real_status.battery_voltage:.2f}V real'
                      if self._real_status is not None else f'{self._healthy_v}V synth'))
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
