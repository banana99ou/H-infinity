# -*- coding: utf-8 -*-
"""Always-on yaw-heading EKF (heading_node).

Fuses every real heading source on the LIMO into one smooth, standstill-capable
estimate, published as a compass bearing (deg E-of-N) on ``/heading/fused``.
Consumed by ``reposition_node`` (between-run) to converge arrival heading without
the course-over-ground (COG) limit-cycle. It NEVER feeds the control loop (ADR-01)
and NEVER publishes ``cmd_vel*`` — it only publishes ``/heading/*``.

Why a separate, always-on node
------------------------------
The sequencer kills/re-spawns ``reposition`` every leg, so a heading filter living
inside reposition would cold-start its gyro bias + heading each leg — exactly the
cold-start that drives the limit-cycle. Running continuously here keeps the filter
warm across reposition's respawns: each new leg gets an already-converged heading.

Sources (verified on the robot 2026-06-08)
------------------------------------------
  sub /imu                                sensor_msgs/Imu  100 Hz RELIABLE
                                          (LIMO base, PRIMARY gyro; z-up REP-103, rad/s)
  sub /pixhawk/imu/data_raw               sensor_msgs/Imu  BEST_EFFORT
                                          (gyro + accel for mag tilt-comp)
  sub /pixhawk/imu/mag                    sensor_msgs/MagneticField BEST_EFFORT
                                          (raw mag; needs set_message_interval(105))
  sub /pixhawk/global_position/compass_hdg std_msgs/Float64 BEST_EFFORT
                                          (GPS-fused EKF output; silent indoors)
  sub /gps_rtk_f9p_helical/gps/fix        sensor_msgs/NavSatFix RELIABLE (COG source)
  sub /gps_rtk_f9p_helical/gps/rtk_status std_msgs/String RELIABLE (quality gate)
  sub /wheel/odom                         nav_msgs/Odometry RELIABLE (fwd/rev for COG)
  pub /heading/fused                      std_msgs/Float64 (compass bearing deg E-of-N)
  pub /heading/fused_status               std_msgs/String  (JSON diagnostics)

EKF
---
State ``x = [psi, b_limo, b_pix]``: heading (compass bearing, rad, E-of-N, 0=N,
CW+) and the two gyro biases (rad/s, expressed in the compass-rate frame). Predict
at gyro rate off the primary (LIMO) gyro; scalar absolute updates from
``compass_hdg``, COG (geographic bearing between successive fixes), and the
tilt-compensated raw mag — each with its own measurement variance and gating. COG
variance scales ``1/travel`` so a sub-threshold (noisy) COG is *down-weighted*, not
hard sign-flipped — the clean cure for the limit-cycle. Innovations are wrapped.

Sign convention (the one real subtlety)
---------------------------------------
``psi`` is a compass bearing (CW-positive from North) while a REP-103 gyro is
CCW-positive, so a positive ``gyro_z`` must DECREASE the bearing. ``gyro_limo_sign``
folds that CCW->CW flip: default ``-1.0`` for a standard z-up/CCW+ gyro feeding the
CW+ compass state. Phase-A synthetic test is the arbiter (publish +gyro_z -> fused
must decrease; if it increases, flip the sign).

Compass offset (90 deg mount + declination + frame)
---------------------------------------------------
``true_bearing_deg = raw_hdg_deg - degrees(compass_offset_rad)``. heading_node OWNS
this now: it boots from the venue JSON value, auto-calibrates against forward COG,
and persists back (atomic). It is the SOLE writer of ``compass_offset_rad``.
"""

import json
import math
import os

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
)
from sensor_msgs.msg import Imu, MagneticField, NavSatFix
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, String
from rcl_interfaces.msg import SetParametersResult

try:
    from mavros_msgs.srv import MessageInterval
    _HAVE_MAVROS = True
except Exception:  # noqa: BLE001 — degrade gracefully on a LIMO-only build
    _HAVE_MAVROS = False

EARTH_R = 6378137.0          # m, WGS84 equatorial radius (equirectangular dist)
G = 9.80665                  # m/s^2
HIGHRES_IMU_MSG_ID = 105     # MAVLINK_MSG_ID_HIGHRES_IMU (carries the raw mag)


def _wrap(a):
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def _geo_bearing(lat1, lon1, lat2, lon2):
    """Initial great-circle bearing (rad, compass E-of-N: 0=N, CW+) of the
    displacement (lat1,lon1) -> (lat2,lon2). Native compass bearing — no local
    frame projection needed, which is exactly the heading_node output convention.
    """
    p1 = math.radians(lat1)
    p2 = math.radians(lat2)
    dl = math.radians(lon2 - lon1)
    y = math.sin(dl) * math.cos(p2)
    x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
    return math.atan2(y, x)


def _geo_dist(lat1, lon1, lat2, lon2):
    """Equirectangular ground distance (m) — good for the cm/m moves here."""
    p1 = math.radians(lat1)
    p2 = math.radians(lat2)
    dl = math.radians(lon2 - lon1)
    pm = 0.5 * (p1 + p2)
    return EARTH_R * math.hypot(dl * math.cos(pm), (p2 - p1))


class HeadingNode(Node):

    def __init__(self):
        super().__init__('heading_node')

        # -- Parameters: topics ------------------------------------------
        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter('pix_imu_topic', '/pixhawk/imu/data_raw')
        self.declare_parameter('mag_topic', '/pixhawk/imu/mag')
        self.declare_parameter('compass_topic',
                               '/pixhawk/global_position/compass_hdg')
        self.declare_parameter('fix_topic', '/gps_rtk_f9p_helical/gps/fix')
        self.declare_parameter('rtk_status_topic',
                               '/gps_rtk_f9p_helical/gps/rtk_status')
        self.declare_parameter('odom_topic', '/wheel/odom')
        self.declare_parameter('fused_topic', '/heading/fused')
        self.declare_parameter('status_topic', '/heading/fused_status')

        # -- Parameters: enable flags ------------------------------------
        self.declare_parameter('enable_gyro_limo', True)
        # Pixhawk gyro shares the 115200-baud MAVROS bottleneck (2026-06-08); off
        # until its stream rate is fixed and its sign verified.
        self.declare_parameter('enable_gyro_pix', False)
        self.declare_parameter('enable_compass', True)
        self.declare_parameter('enable_cog', True)
        self.declare_parameter('enable_mag', True)

        # -- Parameters: signs / frames ----------------------------------
        # -1.0 folds REP-103 (z-up, CCW+) gyro into the CW+ compass-bearing state.
        self.declare_parameter('gyro_limo_sign', -1.0)
        self.declare_parameter('gyro_pix_sign', -1.0)
        self.declare_parameter('mag_frame_offset_rad', 0.0)
        self.declare_parameter('mag_hard_iron_x', 0.0)
        self.declare_parameter('mag_hard_iron_y', 0.0)
        self.declare_parameter('mag_hard_iron_z', 0.0)

        # -- Parameters: compass offset ----------------------------------
        self.declare_parameter(
            'venue_file',
            '/home/agilex/H-infinity/scenarios/venues/rooftop.json')
        self.declare_parameter('compass_offset_rad', float('nan'))  # NaN -> venue
        self.declare_parameter('compass_autocal', True)
        self.declare_parameter('compass_cal_min_samples', 5)
        self.declare_parameter('compass_disagree_warn_deg', 30.0)

        # -- Parameters: variances / process noise -----------------------
        self.declare_parameter('R_compass_deg', 5.0)
        self.declare_parameter('R_cog_base', 0.05)     # rad*m (R = (base/travel)^2 + floor^2)
        self.declare_parameter('R_cog_floor_deg', 2.0)
        self.declare_parameter('R_mag_deg', 25.0)
        self.declare_parameter('R_gyro_pix_dps', 1.0)  # pix-gyro rate cross-check
        self.declare_parameter('q_psi', 1e-4)          # rad^2/s heading walk
        self.declare_parameter('q_bias', 1e-8)         # (rad/s)^2/s bias walk

        # -- Parameters: gates / timeouts --------------------------------
        self.declare_parameter('cog_min_travel_m', 0.10)
        self.declare_parameter('rtk_cog_qualities', [4, 5])
        self.declare_parameter('cog_disagree_skip_deg', 90.0)
        self.declare_parameter('reverse_vx_thresh', 0.02)
        self.declare_parameter('compass_timeout_s', 1.0)
        self.declare_parameter('mag_timeout_s', 1.0)
        self.declare_parameter('cog_timeout_s', 3.0)
        self.declare_parameter('gyro_timeout_s', 0.5)
        self.declare_parameter('accel_g_tol', 0.1)     # fraction of g

        # -- Parameters: output / mavros ---------------------------------
        self.declare_parameter('publish_rate_hz', 25.0)
        self.declare_parameter('init_heading_deg', float('nan'))
        self.declare_parameter('request_pixhawk_mag', True)
        self.declare_parameter('set_message_interval_srv',
                               '/pixhawk/set_message_interval')
        self.declare_parameter('pixhawk_msg_rate_hz', 20.0)

        self._read_params()

        # -- EKF state ---------------------------------------------------
        self.x = np.zeros(3, dtype=float)      # [psi, b_limo, b_pix]
        init_deg = self.get_parameter('init_heading_deg').value
        if init_deg == init_deg:               # not NaN
            self.x[0] = _wrap(math.radians(init_deg))
            p_psi0 = math.radians(20.0) ** 2
        else:
            p_psi0 = math.pi ** 2              # large -> first abs fix snaps psi
        self.P = np.diag([p_psi0,
                          (0.05) ** 2,          # b_limo init var (rad/s)
                          (0.05) ** 2]).astype(float)
        self._t_last_predict = None
        self._latest_limo_rate = None          # compass-frame rate (for pix x-check)
        self._n_updates = 0

        # -- Source bookkeeping ------------------------------------------
        self._t_gyro_limo = None
        self._t_compass = None
        self._t_mag = None
        self._t_cog = None
        self._t_accel = None
        self._last_accel = None                # (ax, ay, az) from pix data_raw
        self._raw_compass_deg = None
        self._rtk_quality = None
        self._fix_prev = None                  # (lat, lon)
        self._vx = 0.0
        self._cal_acc = []                     # forward (raw - cog) offset samples

        # -- Compass offset ----------------------------------------------
        self._compass_offset = 0.0
        self._compass_offset_known = False
        self._load_compass_offset()

        # -- ROS interfaces ----------------------------------------------
        be = QoSProfile(depth=10, history=QoSHistoryPolicy.KEEP_LAST,
                        reliability=QoSReliabilityPolicy.BEST_EFFORT)
        rel = QoSProfile(depth=10, history=QoSHistoryPolicy.KEEP_LAST,
                         reliability=QoSReliabilityPolicy.RELIABLE)
        latched = QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST,
                             reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        self.create_subscription(Imu, self._imu_topic, self._limo_imu_cb, rel)
        self.create_subscription(Imu, self._pix_imu_topic, self._pix_imu_cb, be)
        self.create_subscription(MagneticField, self._mag_topic, self._mag_cb, be)
        self.create_subscription(Float64, self._compass_topic,
                                 self._compass_cb, be)
        self.create_subscription(NavSatFix, self._fix_topic, self._fix_cb, rel)
        self.create_subscription(String, self._rtk_status_topic,
                                 self._rtk_cb, rel)
        self.create_subscription(Odometry, self._odom_topic, self._odom_cb, rel)

        self.pub_fused = self.create_publisher(Float64, self._fused_topic, latched)
        self.pub_status = self.create_publisher(String, self._status_topic, latched)

        self.add_on_set_parameters_callback(self._params_cb)
        self.create_timer(1.0 / max(self._pub_rate, 1.0), self._publish_cb)

        # Wake the raw Pixhawk mag (silent until requested; proven success live).
        self._mi_client = None
        self._mi_timer = None
        self._mi_tries = 0
        if self._request_mag and _HAVE_MAVROS:
            self._mi_client = self.create_client(MessageInterval, self._mi_srv)
            self._mi_timer = self.create_timer(2.0, self._try_request_mi)

        self.get_logger().info(
            'heading_node started (always-on EKF). Publishing compass bearing '
            f'deg E-of-N on {self._fused_topic}. gyro_limo_sign='
            f'{self._gyro_limo_sign:+.0f}, compass_offset='
            + (f'{math.degrees(self._compass_offset):.1f} deg'
               if self._compass_offset_known else 'auto-cal pending') + '.')

    # ------------------------------------------------------------------
    # Parameter handling
    # ------------------------------------------------------------------

    def _read_params(self):
        g = self.get_parameter
        self._imu_topic = str(g('imu_topic').value)
        self._pix_imu_topic = str(g('pix_imu_topic').value)
        self._mag_topic = str(g('mag_topic').value)
        self._compass_topic = str(g('compass_topic').value)
        self._fix_topic = str(g('fix_topic').value)
        self._rtk_status_topic = str(g('rtk_status_topic').value)
        self._odom_topic = str(g('odom_topic').value)
        self._fused_topic = str(g('fused_topic').value)
        self._status_topic = str(g('status_topic').value)

        self._en_gyro_limo = bool(g('enable_gyro_limo').value)
        self._en_gyro_pix = bool(g('enable_gyro_pix').value)
        self._en_compass = bool(g('enable_compass').value)
        self._en_cog = bool(g('enable_cog').value)
        self._en_mag = bool(g('enable_mag').value)

        self._gyro_limo_sign = float(g('gyro_limo_sign').value)
        self._gyro_pix_sign = float(g('gyro_pix_sign').value)
        self._mag_frame_offset = float(g('mag_frame_offset_rad').value)
        self._hi = (float(g('mag_hard_iron_x').value),
                    float(g('mag_hard_iron_y').value),
                    float(g('mag_hard_iron_z').value))

        self._venue_file = str(g('venue_file').value)
        self._autocal = bool(g('compass_autocal').value)
        self._cal_min = int(g('compass_cal_min_samples').value)
        self._disagree_warn = math.radians(
            float(g('compass_disagree_warn_deg').value))

        self._R_compass = math.radians(float(g('R_compass_deg').value)) ** 2
        self._R_cog_base = float(g('R_cog_base').value)
        self._R_cog_floor = math.radians(float(g('R_cog_floor_deg').value)) ** 2
        self._R_mag = math.radians(float(g('R_mag_deg').value)) ** 2
        self._R_gyro_pix = math.radians(float(g('R_gyro_pix_dps').value)) ** 2
        self._q_psi = float(g('q_psi').value)
        self._q_bias = float(g('q_bias').value)

        self._cog_min_travel = float(g('cog_min_travel_m').value)
        self._rtk_qualities = set(int(q) for q in g('rtk_cog_qualities').value)
        self._cog_disagree = math.radians(float(g('cog_disagree_skip_deg').value))
        self._rev_vx = float(g('reverse_vx_thresh').value)
        self._compass_timeout = float(g('compass_timeout_s').value)
        self._mag_timeout = float(g('mag_timeout_s').value)
        self._cog_timeout = float(g('cog_timeout_s').value)
        self._gyro_timeout = float(g('gyro_timeout_s').value)
        self._accel_g_tol = float(g('accel_g_tol').value)

        self._pub_rate = float(g('publish_rate_hz').value)
        self._request_mag = bool(g('request_pixhawk_mag').value)
        self._mi_srv = str(g('set_message_interval_srv').value)
        self._pix_msg_rate = float(g('pixhawk_msg_rate_hz').value)

    def _params_cb(self, params):
        """Live retune of the tuning-relevant params (signs, variances, enables,
        mag frame/hard-iron). Topic names + rates are set-once at construction."""
        for p in params:
            n = p.name
            try:
                if n == 'gyro_limo_sign':
                    self._gyro_limo_sign = float(p.value)
                elif n == 'gyro_pix_sign':
                    self._gyro_pix_sign = float(p.value)
                elif n == 'mag_frame_offset_rad':
                    self._mag_frame_offset = float(p.value)
                elif n == 'mag_hard_iron_x':
                    self._hi = (float(p.value), self._hi[1], self._hi[2])
                elif n == 'mag_hard_iron_y':
                    self._hi = (self._hi[0], float(p.value), self._hi[2])
                elif n == 'mag_hard_iron_z':
                    self._hi = (self._hi[0], self._hi[1], float(p.value))
                elif n == 'enable_gyro_limo':
                    self._en_gyro_limo = bool(p.value)
                elif n == 'enable_gyro_pix':
                    self._en_gyro_pix = bool(p.value)
                elif n == 'enable_compass':
                    self._en_compass = bool(p.value)
                elif n == 'enable_cog':
                    self._en_cog = bool(p.value)
                elif n == 'enable_mag':
                    self._en_mag = bool(p.value)
                elif n == 'R_compass_deg':
                    self._R_compass = math.radians(float(p.value)) ** 2
                elif n == 'R_cog_base':
                    self._R_cog_base = float(p.value)
                elif n == 'R_cog_floor_deg':
                    self._R_cog_floor = math.radians(float(p.value)) ** 2
                elif n == 'R_mag_deg':
                    self._R_mag = math.radians(float(p.value)) ** 2
                elif n == 'q_psi':
                    self._q_psi = float(p.value)
                elif n == 'q_bias':
                    self._q_bias = float(p.value)
                elif n == 'cog_min_travel_m':
                    self._cog_min_travel = float(p.value)
                elif n == 'compass_offset_rad':
                    v = float(p.value)
                    if v == v:
                        self._compass_offset = v
                        self._compass_offset_known = True
            except (ValueError, TypeError):
                return SetParametersResult(successful=False)
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------
    # Compass offset: venue load + atomic persist (sole writer)
    # ------------------------------------------------------------------

    def _load_compass_offset(self):
        p = self.get_parameter('compass_offset_rad').value
        if p == p:                              # explicit param (not NaN) wins
            self._compass_offset = float(p)
            self._compass_offset_known = True
            self.get_logger().info(
                f'compass offset from param: {math.degrees(p):.1f} deg.')
            return
        try:
            with open(self._venue_file, 'r') as f:
                v = json.load(f)
            voff = v.get('compass_offset_rad')
            if voff is not None:
                self._compass_offset = float(voff)
                self._compass_offset_known = True
                self.get_logger().info(
                    f'compass offset from venue: '
                    f'{math.degrees(self._compass_offset):.1f} deg.')
        except Exception as exc:               # noqa: BLE001
            self.get_logger().warn(
                f'venue compass offset load failed ({exc}); auto-cal will run '
                'on the first forward RTK drive.')

    def _persist_compass_offset(self, off):
        path = self._venue_file
        try:
            with open(path, 'r') as f:
                v = json.load(f)
            v['compass_offset_rad'] = float(off)
            tmp = path + '.tmp'
            with open(tmp, 'w') as f:
                json.dump(v, f, indent=2)
            os.replace(tmp, path)
            self.get_logger().info(
                f'persisted compass_offset_rad={off:.4f} to {path}.')
        except Exception as exc:               # noqa: BLE001
            self.get_logger().warn(
                f'could not persist compass offset: {exc} (in-memory cal holds).')

    # ------------------------------------------------------------------
    # EKF core
    # ------------------------------------------------------------------

    def _predict(self, compass_rate, bias_idx):
        """Propagate psi with one gyro's bias-corrected compass-frame rate."""
        now = self.get_clock().now()
        if self._t_last_predict is None:
            self._t_last_predict = now
            return
        dt = (now - self._t_last_predict).nanoseconds * 1e-9
        self._t_last_predict = now
        if dt <= 0.0:
            return
        dt = min(max(dt, 1e-4), 0.1)
        b = self.x[bias_idx]
        self.x[0] = _wrap(self.x[0] + (compass_rate - b) * dt)
        F = np.eye(3)
        F[0, bias_idx] = -dt
        Q = np.diag([self._q_psi * dt, self._q_bias * dt, self._q_bias * dt])
        self.P = F @ self.P @ F.T + Q

    def _update(self, z, R, h_idx=0, wrap_innov=True):
        """Scalar Kalman update with H = e_{h_idx}. h_idx=0 (heading) wraps the
        innovation onto SO(2); h_idx=2 (b_pix) does not."""
        y = z - self.x[h_idx]
        if wrap_innov:
            y = _wrap(y)
        S = self.P[h_idx, h_idx] + R
        if S <= 1e-12:
            return
        K = self.P[:, h_idx] / S
        self.x = self.x + K * y
        self.x[0] = _wrap(self.x[0])
        self.P = self.P - np.outer(K, self.P[h_idx, :])
        self.P = 0.5 * (self.P + self.P.T)      # keep symmetric
        self._n_updates += 1

    # ------------------------------------------------------------------
    # Sensor callbacks
    # ------------------------------------------------------------------

    def _limo_imu_cb(self, msg: Imu):
        if not self._en_gyro_limo:
            return
        self._t_gyro_limo = self._now()
        rate = self._gyro_limo_sign * msg.angular_velocity.z   # compass-frame rate
        self._predict(rate, bias_idx=1)
        self._latest_limo_rate = rate - self.x[1]              # bias-corrected

    def _pix_imu_cb(self, msg: Imu):
        a = msg.linear_acceleration
        self._last_accel = (a.x, a.y, a.z)
        self._t_accel = self._now()
        if not self._en_gyro_pix or self._latest_limo_rate is None:
            return
        # Rate pseudo-measurement of b_pix: pix compass-rate - best rate ~ b_pix.
        z = self._gyro_pix_sign * msg.angular_velocity.z - self._latest_limo_rate
        self._update(z, self._R_gyro_pix, h_idx=2, wrap_innov=False)

    def _mag_cb(self, msg: MagneticField):
        if not self._en_mag:
            return
        self._t_mag = self._now()
        if self._last_accel is None:
            return
        ax, ay, az = self._last_accel
        amag = math.sqrt(ax * ax + ay * ay + az * az)
        if amag < 1e-3 or abs(amag - G) > self._accel_g_tol * G:
            return                              # too dynamic/invalid for tilt-comp
        mx = msg.magnetic_field.x - self._hi[0]
        my = msg.magnetic_field.y - self._hi[1]
        mz = msg.magnetic_field.z - self._hi[2]
        roll = math.atan2(ay, az)
        pitch = math.atan2(-ax, math.hypot(ay, az))
        mxh = mx * math.cos(pitch) + mz * math.sin(pitch)
        myh = (mx * math.sin(roll) * math.sin(pitch) + my * math.cos(roll)
               - mz * math.sin(roll) * math.cos(pitch))
        heading = math.atan2(-myh, mxh) + self._mag_frame_offset
        self._update(_wrap(heading), self._R_mag, h_idx=0)

    def _compass_cb(self, msg: Float64):
        if not self._en_compass:
            return
        self._t_compass = self._now()
        self._raw_compass_deg = float(msg.data)
        if not self._compass_offset_known:
            return
        z = math.radians(self._raw_compass_deg) - self._compass_offset
        self._update(_wrap(z), self._R_compass, h_idx=0)

    def _rtk_cb(self, msg: String):
        q = None
        for tok in msg.data.replace('(', ' ').replace(',', ' ').split():
            if tok.startswith('quality='):
                try:
                    q = int(tok.split('=', 1)[1])
                except ValueError:
                    q = None
                break
        self._rtk_quality = q

    def _odom_cb(self, msg: Odometry):
        self._vx = msg.twist.twist.linear.x

    def _fix_cb(self, msg: NavSatFix):
        if not self._en_cog:
            return
        if self._rtk_quality not in self._rtk_qualities:
            self._fix_prev = None               # break COG continuity on bad fix
            return
        lat, lon = msg.latitude, msg.longitude
        if lat != lat or lon != lon or (lat == 0.0 and lon == 0.0):
            return
        if self._fix_prev is None:
            self._fix_prev = (lat, lon)
            return
        plat, plon = self._fix_prev
        travel = _geo_dist(plat, plon, lat, lon)
        if travel < self._cog_min_travel:
            return                              # accumulate; keep the same anchor
        cog = _geo_bearing(plat, plon, lat, lon)
        self._fix_prev = (lat, lon)
        reverse = self._vx < -self._rev_vx
        if reverse:
            cog = _wrap(cog + math.pi)
        # Reject COG that strongly disagrees with the gyro-propagated heading
        # (un-flagged reverse, multipath, etc.). The gyro carries heading through.
        if abs(_wrap(cog - self.x[0])) > self._cog_disagree:
            return
        R = (self._R_cog_base / max(travel, 1e-3)) ** 2 + self._R_cog_floor
        if self._rtk_quality == 5:              # FLOAT ~dm: inflate vs FIXED
            R *= 4.0
        self._update(cog, R, h_idx=0)
        self._t_cog = self._now()
        # Compass offset auto-cal: forward motion, good fix, not yet known.
        if (self._autocal and not reverse and not self._compass_offset_known
                and self._raw_compass_deg is not None):
            self._accumulate_cal(cog)
        elif (self._compass_offset_known and not reverse
                and self._raw_compass_deg is not None):
            self._crosscheck_compass(cog)

    def _accumulate_cal(self, cog):
        """offset s.t. true_bearing = raw - offset; with true ~ COG -> offset = raw - cog."""
        off = _wrap(math.radians(self._raw_compass_deg) - cog)
        self._cal_acc.append(off)
        if len(self._cal_acc) >= self._cal_min:
            sx = sum(math.sin(o) for o in self._cal_acc)
            sy = sum(math.cos(o) for o in self._cal_acc)
            self._compass_offset = math.atan2(sx, sy)
            self._compass_offset_known = True
            self._cal_acc = []
            self.get_logger().warn(
                'compass auto-calibrated: offset='
                f'{math.degrees(self._compass_offset):.1f} deg '
                f'(persist as compass_offset_rad:={self._compass_offset:.4f}).')
            self._persist_compass_offset(self._compass_offset)

    def _crosscheck_compass(self, cog):
        true_b = math.radians(self._raw_compass_deg) - self._compass_offset
        d = abs(_wrap(true_b - cog))
        if d > self._disagree_warn:
            self.get_logger().warn(
                f'compass vs COG disagree {math.degrees(d):.0f} deg '
                '(magnetic interference / bad offset?).')

    # ------------------------------------------------------------------
    # mavros: wake the raw mag stream
    # ------------------------------------------------------------------

    def _try_request_mi(self):
        self._mi_tries += 1
        if self._mi_client is None or not self._mi_client.service_is_ready():
            if self._mi_tries > 15:
                self.get_logger().warn(
                    f'{self._mi_srv} unavailable after {self._mi_tries} tries; '
                    'raw Pixhawk mag may stay silent (no MAVROS?).')
                self._mi_timer.cancel()
            return
        req = MessageInterval.Request()
        req.message_id = HIGHRES_IMU_MSG_ID
        req.message_rate = float(self._pix_msg_rate)
        self._mi_client.call_async(req).add_done_callback(self._mi_done)
        self._mi_timer.cancel()

    def _mi_done(self, fut):
        try:
            ok = fut.result().success
            self.get_logger().info(
                f'set_message_interval(HIGHRES_IMU @ {self._pix_msg_rate:.0f} Hz) '
                f'success={ok}.')
        except Exception as exc:               # noqa: BLE001
            self.get_logger().warn(f'set_message_interval failed: {exc}.')

    # ------------------------------------------------------------------
    # Publish
    # ------------------------------------------------------------------

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _fresh(self, t, timeout):
        return t is not None and (self._now() - t) <= timeout

    def _publish_cb(self):
        bearing = math.degrees(_wrap(self.x[0]))
        if bearing < 0.0:
            bearing += 360.0                    # report 0..360 compass bearing
        m = Float64()
        m.data = float(bearing)
        self.pub_fused.publish(m)

        now = self._now()

        def age(t):
            return None if t is None else round(now - t, 2)

        compass_fresh = self._fresh(self._t_compass, self._compass_timeout)
        cog_fresh = self._fresh(self._t_cog, self._cog_timeout)
        mag_fresh = self._fresh(self._t_mag, self._mag_timeout)
        gyro_fresh = self._fresh(self._t_gyro_limo, self._gyro_timeout)
        if compass_fresh or cog_fresh:
            mode = 'GNSS_AIDED'
        elif mag_fresh:
            mode = 'GYRO_MAG'
        elif gyro_fresh:
            mode = 'GYRO_ONLY'
        else:
            mode = 'STALE'
        active = []
        if gyro_fresh:
            active.append('gyro_limo')
        if compass_fresh:
            active.append('compass')
        if cog_fresh:
            active.append('cog')
        if mag_fresh:
            active.append('mag')

        payload = {
            'fused_deg': round(bearing, 2),
            'heading_std_deg': round(math.degrees(math.sqrt(max(self.P[0, 0],
                                                                0.0))), 2),
            'mode': mode,
            'active_sources': active,
            'compass_age_s': age(self._t_compass),
            'cog_age_s': age(self._t_cog),
            'mag_age_s': age(self._t_mag),
            'gyro_age_s': age(self._t_gyro_limo),
            'gyro_bias_limo_dps': round(math.degrees(self.x[1]), 3),
            'gyro_bias_pix_dps': round(math.degrees(self.x[2]), 3),
            'rtk_quality': self._rtk_quality,
            'n_updates': self._n_updates,
            'compass_offset_deg': (round(math.degrees(self._compass_offset), 2)
                                   if self._compass_offset_known else None),
        }
        s = String()
        s.data = json.dumps(payload)
        self.pub_status.publish(s)


def main(args=None):
    rclpy.init(args=args)
    node = HeadingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
