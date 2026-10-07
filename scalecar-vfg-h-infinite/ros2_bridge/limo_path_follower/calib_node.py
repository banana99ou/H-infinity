# -*- coding: utf-8 -*-
"""Open-loop figure-8 steering calibration mover (calib_node, orchestrator PROC
``calib``).

What this node IS
-----------------
A *dumb open-loop mover* that MEASURES the robot's tightest turn. On a request
it drives, at full steering lock, one 360 deg left circle then one 360 deg right
circle per requested speed (a figure-8 per speed), chaining the segments without
stopping, and reports per-segment turn radii measured two independent ways:

* IMU/odom: R_imu = median odom speed / |median gyro z| over the segment window;
* RTK: Kasa circle fit of the antenna fixes in the same window, corrected from
  the antenna to the rear-axle centre with the antenna lever arm.

It does not plan, does not choose speeds, and never closes a loop on position:
the command is fixed per segment (v, dir * v * tan(steer_cmd_rad) / L). The only
feedback it acts on is the set of ABORT rules below — they matter more than the
measurement, because this drives a real robot at up to 1.0 m/s at full lock.

Topology (interface spec 2026-10-08)
------------------------------------
  sub /imu                          sensor_msgs/Imu        angular_velocity.z (REP-103, CCW+)
  sub /wheel/odom                   nav_msgs/Odometry      speed = hypot(twist.linear.x, .y)
  sub /gps_rtk_f9p_helical/gps/fix  sensor_msgs/NavSatFix  valid = status.status >= 0, finite
  sub /estop                        std_msgs/Bool          latched state (estop_cli, 2 Hz)
  sub /calib/request                std_msgs/String JSON   re-sent at 1 Hz until status.seq == seq
  pub cmd_vel_raw                   geometry_msgs/Twist    relative name, gated by estop_cli (C3)
  pub /calib/status                 std_msgs/String JSON   LATCHED (depth 1, TRANSIENT_LOCAL)

Nothing else is ever published. Exactly one mover may be alive (C6): the
executor kills reposition/follower before starting this node. ``cmd_vel_raw`` is
silent until a request starts a run, and silent again ``stop_hold_s`` after the
run ends (zero commands are held for that long first).

Request / status / result JSON
------------------------------
Request: {seq, mode:"full"|"sanity", speeds, steer_cmd_rad, R_plan_m, turn_deg,
pin:{lat,lon,heading_deg}, venue:{corners_wgs84, exclusions}, min_clearance_m,
max_radius_m, max_duration_s}. Missing fields take the spec defaults; malformed
or out-of-limit fields REJECT the request (state aborted, no motion).

Status: {seq, state:"idle"|"running"|"done"|"aborted", segment:"v1.0 left",
reason, result (on done/aborted), stamp} plus informational turned_deg /
elapsed_s. While waiting for sensors after accepting a request the state is
"running" with segment "" (the seq echo is the acknowledgement); the final
zero-hold after the last segment is "running" with segment "stop", so "done"
means the robot has already been commanded zero for stop_hold_s.

Result: {ok, reason, mode, steer_cmd_rad, pedestal_test, segments:[{v_cmd, dir,
turned_deg, window_s, n_imu, w_imu_radps, v_odom_mps, R_imu_m, delta_imu_rad,
n_fix, R_rtk_ant_m, R_rtk_rear_m, fit_resid_m, v_rtk_mps, complete}], stamp_utc}
plus seq/speeds/R_plan_m/turn_deg/wheelbase_m/duration_s for traceability.
ok = every planned segment completed with a finite R_imu (never in pedestal).
Non-finite numbers are emitted as JSON null.

Abort rules (zero Twist on the abort tick, state aborted, reason)
-----------------------------------------------------------------
e-stop True; IMU stale > 0.3 s; odom stale > 0.5 s; require_rtk and no valid
fix for > 1.0 s; latest fresh RTK position with polygon clearance (or clearance
to any exclusion circle edge) < min_clearance_m, or farther than max_radius_m
from the pin; not turning (after 2.0 s in a segment the 0.5 s mean of
dir * gyro_z < 0.3 * v / R_plan_m — this signed form also catches turning the
WRONG way, i.e. a steering/IMU sign fault, which the spec's |w| form would let
run to the segment timeout); segment timeout 1.5 * 2 pi R_plan / v + 3 s; total
run time > max_duration_s; SIGTERM/SIGINT -> zero Twist x5, then exit. Before
the first motion the node also requires fresh IMU+odom(+RTK when required) and a
KNOWN e-stop state of False, else it aborts after start_timeout_s.

Pedestal mode (ROS param pedestal_test, wheels off the ground)
--------------------------------------------------------------
Segments are time-based (2 pi R_plan / v, capped at 8 s); no RTK, containment,
turning or segment-timeout checks; the result is ok=false, reason "pedestal
test". e-stop, IMU/odom staleness and the total-duration cap still apply.

Code layout
-----------
Everything except ``CalibNode``/``main`` is ROS-free and module-level (geometry,
Kasa fit, lever-arm correction, window statistics, request parsing and the
``CalibSequencer`` state machine), so tools/analysis/tests/test_calib_node.py
exercises it on a laptop with no ROS installed. rclpy and the message types are
imported under try/except and only required by the node class / main().
"""

import json
import math
import signal
import statistics
import time
from collections import deque
from datetime import datetime, timezone

try:  # ROS is only required to RUN the node; the pure logic imports anywhere.
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import (
        QoSProfile,
        QoSDurabilityPolicy,
        QoSReliabilityPolicy,
        QoSHistoryPolicy,
    )
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu, NavSatFix
    from std_msgs.msg import Bool, String
    _HAVE_ROS = True
except ImportError:  # laptop / unit tests
    rclpy = None
    Node = object
    _HAVE_ROS = False


# ----------------------------------------------------------------------------
# Constants (spec values). Thresholds that define the abort contract are NOT ROS
# params on purpose: the spec fixes them and the executor relies on them.
# ----------------------------------------------------------------------------

LEFT = +1
RIGHT = -1
ZERO_CMD = (0.0, 0.0)

DEFAULT_SPEEDS = {'full': [0.5, 1.0], 'sanity': [1.0]}
REQUEST_DEFAULTS = {
    'steer_cmd_rad': 0.40,
    'R_plan_m': 1.2,
    'turn_deg': 360.0,
    'min_clearance_m': 0.5,
    'max_radius_m': 2.9,
    'max_duration_s': 150.0,
}

# Internal sequencer states -> the status ``state`` the executor reads.
_ACTIVE = ('waiting', 'running', 'stopping')
_STATUS_STATE = {
    'waiting': 'running',
    'running': 'running',
    'stopping': 'running',
    'done': 'done',
    'aborted': 'aborted',
}

SHUTDOWN_ZERO_COUNT = 5
SHUTDOWN_ZERO_GAP_S = 0.02


class CalibConfig:
    """Node-level configuration (ROS params + fixed spec thresholds)."""

    def __init__(self, **kw):
        # ROS params (spec list)
        self.wheelbase_m = 0.2
        self.rate_hz = 20.0
        self.settle_deg = 90.0
        self.tail_deg = 20.0
        self.require_rtk = True
        self.pedestal_test = False
        self.antenna_fwd_m = 0.10
        self.antenna_right_m = 0.075
        self.stop_hold_s = 1.0
        # ROS params (safety extras, see module docstring)
        self.start_timeout_s = 5.0
        self.imu_yaw_sign = 1.0
        # Fixed spec thresholds
        self.imu_stale_s = 0.3
        self.odom_stale_s = 0.5
        self.rtk_stale_s = 1.0
        self.not_turning_after_s = 2.0
        self.not_turning_frac = 0.3
        self.turn_avg_s = 0.5
        self.seg_timeout_factor = 1.5
        self.seg_timeout_pad_s = 3.0
        self.pedestal_seg_cap_s = 8.0
        self.min_fixes = 8
        # Request validation limits
        self.max_speed_mps = 1.0
        self.max_steer_cmd_rad = 0.45
        self.max_speeds = 4
        for k, v in kw.items():
            if not hasattr(self, k):
                raise TypeError(f'unknown CalibConfig field {k!r}')
            setattr(self, k, v)


class CalibGeometry:
    """Containment geometry in a local East/North frame centred on the pin."""

    def __init__(self, lat0, lon0, poly_en, excl_en):
        self.lat0 = lat0
        self.lon0 = lon0
        self.poly_en = poly_en      # [(e, n), ...]
        self.excl_en = excl_en      # [(e, n, radius_m), ...]

    def to_en(self, lat, lon):
        return latlon_to_en(lat, lon, self.lat0, self.lon0)


class CalibRequest:
    def __init__(self, seq, mode, speeds, steer_cmd_rad, R_plan_m, turn_deg,
                 min_clearance_m, max_radius_m, max_duration_s, geom):
        self.seq = seq
        self.mode = mode
        self.speeds = speeds
        self.steer_cmd_rad = steer_cmd_rad
        self.R_plan_m = R_plan_m
        self.turn_deg = turn_deg
        self.min_clearance_m = min_clearance_m
        self.max_radius_m = max_radius_m
        self.max_duration_s = max_duration_s
        self.geom = geom            # CalibGeometry, or None in pedestal mode


# ----------------------------------------------------------------------------
# Pure helpers (no ROS)
# ----------------------------------------------------------------------------

def _finite(*xs):
    return all(isinstance(x, (int, float)) and math.isfinite(x) for x in xs)


def omega_cmd(v, steer_cmd_rad, wheelbase_m, direction):
    """Yaw-rate command for a full-lock segment: dir * v * tan(steer) / L.

    The patched limo_base driver in direct mode inverts this back to
    delta = atan(w L / v) = steer_cmd_rad (clamped at 0.408 by the driver)."""
    return direction * v * math.tan(steer_cmd_rad) / wheelbase_m


def segment_name(v, direction):
    return f"v{v:.1f} {'left' if direction > 0 else 'right'}"


def fix_is_valid(status, lat, lon):
    """NavSatFix validity per spec: status.status >= 0 and finite lat/lon."""
    try:
        return int(status) >= 0 and _finite(float(lat), float(lon))
    except (TypeError, ValueError):
        return False


def odom_speed(vx, vy):
    return math.hypot(vx, vy)


def imu_dt_from_stamps(prev_stamp, stamp, cap_s):
    """Integration step from sensor header stamps, or None when the stamps are
    unusable (first sample, non-monotonic, or a gap beyond cap_s) — the caller
    then falls back to receive-time spacing."""
    if prev_stamp is None or stamp is None:
        return None
    d = stamp - prev_stamp
    if 0.0 < d <= cap_s:
        return d
    return None


def latlon_to_en(lat, lon, lat0, lon0):
    """Equirectangular lat/lon -> local (East, North) metres about (lat0, lon0).
    Same small-area model as venue_geom.latlon_to_en."""
    mlat = 111320.0
    mlon = 111320.0 * math.cos(math.radians(lat0))
    return ((lon - lon0) * mlon, (lat - lat0) * mlat)


def en_to_latlon(e, n, lat0, lon0):
    """Inverse of latlon_to_en."""
    mlat = 111320.0
    mlon = 111320.0 * math.cos(math.radians(lat0))
    return (lat0 + n / mlat, lon0 + e / mlon)


def point_in_polygon(pt, poly):
    """Ray-cast point-in-polygon; poly is a list of (x, y)."""
    x, y = pt
    inside = False
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        if ((y1 > y) != (y2 > y)) and \
                (x < (x2 - x1) * (y - y1) / (y2 - y1) + x1):
            inside = not inside
    return inside


def dist_point_segment(p, a, b):
    px, py = p
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    if L2 <= 0.0:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def polygon_clearance(pt, poly):
    """Signed distance to the polygon boundary: + inside, - outside (m)."""
    d = min(dist_point_segment(pt, poly[i], poly[(i + 1) % len(poly)])
            for i in range(len(poly)))
    return d if point_in_polygon(pt, poly) else -d


def containment_margin(pt, geom):
    """Smallest clearance of pt to the venue edge or any exclusion circle edge.
    Returns (margin_m, what) with what = 'polygon' or 'exclusion <i>'.
    Negative margin = outside the polygon / inside an exclusion."""
    best = polygon_clearance(pt, geom.poly_en)
    what = 'polygon'
    for i, (ce, cn, r) in enumerate(geom.excl_en):
        m = math.hypot(pt[0] - ce, pt[1] - cn) - r
        if m < best:
            best, what = m, f'exclusion {i}'
    return best, what


def kasa_fit(points):
    """Algebraic (Kasa) circle fit. points: iterable of (x, y).

    Returns (cx, cy, R, rms_resid_m) or None (fewer than 3 points, or a
    degenerate/collinear set). Mean-centred normal equations for conditioning;
    rms_resid is the RMS of the GEOMETRIC residuals |p - c| - R."""
    pts = [(float(x), float(y)) for (x, y) in points]
    n = len(pts)
    if n < 3:
        return None
    mx = sum(p[0] for p in pts) / n
    my = sum(p[1] for p in pts) / n
    suu = suv = svv = suuu = svvv = suvv = svuu = 0.0
    for (x, y) in pts:
        u, v = x - mx, y - my
        uu, vv = u * u, v * v
        suu += uu
        svv += vv
        suv += u * v
        suuu += uu * u
        svvv += vv * v
        suvv += u * vv
        svuu += v * uu
    det = suu * svv - suv * suv
    if not (det > 1e-12 * max(suu * svv, 1e-300)):
        return None
    bu = 0.5 * (suuu + suvv)
    bv = 0.5 * (svvv + svuu)
    uc = (bu * svv - bv * suv) / det
    vc = (suu * bv - suv * bu) / det
    r2 = uc * uc + vc * vc + (suu + svv) / n
    if not (r2 > 0.0):
        return None
    R = math.sqrt(r2)
    cx, cy = uc + mx, vc + my
    rms = math.sqrt(sum((math.hypot(x - cx, y - cy) - R) ** 2
                        for (x, y) in pts) / n)
    return cx, cy, R, rms


def rear_axle_radius(R_ant, direction, antenna_fwd_m, antenna_right_m):
    """Antenna turn radius -> rear-axle-centre turn radius.

    The antenna sits a = antenna_fwd_m ahead of and b = antenna_right_m to the
    RIGHT of the rear-axle centre. For a no-slip turn the centre lies on the rear
    axle line, so R_ant^2 = a^2 + (R_rear + dir*b)^2 (dir +1 left, -1 right):
        left : R_rear = sqrt(R_ant^2 - a^2) - b
        right: R_rear = sqrt(R_ant^2 - a^2) + b
    Returns NaN when the geometry is impossible."""
    if not _finite(R_ant):
        return float('nan')
    s = R_ant * R_ant - antenna_fwd_m * antenna_fwd_m
    if s <= 0.0:
        return float('nan')
    r = math.sqrt(s) - direction * antenna_right_m
    return r if r > 0.0 else float('nan')


def median_or_nan(xs):
    xs = [x for x in xs if _finite(x)]
    return statistics.median(xs) if xs else float('nan')


def fix_speeds(fixes, max_dt_s=1.0):
    """Fix-to-fix speeds from [(t_meas, e, n), ...] in time order."""
    out = []
    for (t0, e0, n0), (t1, e1, n1) in zip(fixes, fixes[1:]):
        dt = t1 - t0
        if 0.0 < dt <= max_dt_s:
            out.append(math.hypot(e1 - e0, n1 - n0) / dt)
    return out


def json_safe(obj):
    """Recursively replace non-finite floats with None (strict JSON)."""
    if isinstance(obj, float):
        return obj if math.isfinite(obj) else None
    if isinstance(obj, dict):
        return {k: json_safe(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [json_safe(v) for v in obj]
    return obj


def request_action(cur_seq, cur_state, new_seq):
    """Decide what to do with an incoming request.

    'ack'    — same seq as the run we hold (the executor's 1 Hz re-send):
               re-publish status, change nothing;
    'busy'   — a different seq while a run is active: ignore it (the running
               figure-8 stays bounded by its own guards);
    'accept' — start (or reject on validation) a new run."""
    if cur_state is None:
        return 'accept'
    if new_seq is not None and new_seq == cur_seq:
        return 'ack'
    if cur_state in _ACTIVE:
        return 'busy'
    return 'accept'


def _num(d, key, default):
    v = d.get(key, default)
    if isinstance(v, bool):
        raise ValueError(f'{key} must be a number')
    v = float(v)
    if not math.isfinite(v):
        raise ValueError(f'{key} must be finite')
    return v


def build_geometry(pin, venue):
    """pin {lat,lon}, venue {corners_wgs84, exclusions} -> CalibGeometry.
    Fails CLOSED (ValueError) on anything it cannot verify."""
    if not isinstance(pin, dict):
        raise ValueError('pin {lat, lon} required')
    lat0, lon0 = float(pin['lat']), float(pin['lon'])
    if not _finite(lat0, lon0):
        raise ValueError('pin lat/lon not finite')
    if not isinstance(venue, dict):
        raise ValueError('venue {corners_wgs84, exclusions} required')
    corners = venue.get('corners_wgs84') or []
    if len(corners) < 3:
        raise ValueError('venue.corners_wgs84 needs >= 3 corners')
    poly = []
    for c in corners:
        la, lo = float(c['lat']), float(c['lon'])
        if not _finite(la, lo):
            raise ValueError('venue corner not finite')
        poly.append(latlon_to_en(la, lo, lat0, lon0))
    excl = []
    for i, ex in enumerate(venue.get('exclusions') or []):
        kind = str((ex or {}).get('kind', '')).lower()
        if kind != 'circle':
            raise ValueError(f"exclusion {i}: unsupported kind '{kind}'")
        la, lo, r = float(ex['lat']), float(ex['lon']), float(ex['radius_m'])
        if not _finite(la, lo, r) or r < 0.0:
            raise ValueError(f'exclusion {i}: malformed')
        e, n = latlon_to_en(la, lo, lat0, lon0)
        excl.append((e, n, r))
    return CalibGeometry(lat0, lon0, poly, excl)


def parse_request(d, cfg):
    """Validate a /calib/request dict. Returns (CalibRequest, None) or
    (None, error_string). Out-of-limit values are rejected, never clamped."""
    if not isinstance(d, dict):
        return None, 'request is not a JSON object'
    try:
        seq = d.get('seq', None)
        mode = str(d.get('mode', 'full')).lower()
        if mode not in DEFAULT_SPEEDS:
            raise ValueError(f"mode must be 'full' or 'sanity', got {mode!r}")
        speeds = d.get('speeds', DEFAULT_SPEEDS[mode])
        if not isinstance(speeds, list) or not speeds:
            raise ValueError('speeds must be a non-empty list')
        if len(speeds) > cfg.max_speeds:
            raise ValueError(f'at most {cfg.max_speeds} speeds')
        sp = []
        for v in speeds:
            if isinstance(v, bool):
                raise ValueError('speeds must be numbers')
            v = float(v)
            if not (math.isfinite(v) and 0.0 < v <= cfg.max_speed_mps):
                raise ValueError(
                    f'speed {v} outside (0, {cfg.max_speed_mps}] m/s')
            sp.append(v)
        steer = _num(d, 'steer_cmd_rad', REQUEST_DEFAULTS['steer_cmd_rad'])
        if not (0.0 < steer <= cfg.max_steer_cmd_rad):
            raise ValueError(
                f'steer_cmd_rad {steer} outside (0, {cfg.max_steer_cmd_rad}]')
        R_plan = _num(d, 'R_plan_m', REQUEST_DEFAULTS['R_plan_m'])
        if not (0.2 <= R_plan <= 5.0):
            raise ValueError(f'R_plan_m {R_plan} outside [0.2, 5.0]')
        turn = _num(d, 'turn_deg', REQUEST_DEFAULTS['turn_deg'])
        if not (cfg.settle_deg + cfg.tail_deg < turn <= 720.0):
            raise ValueError(
                f'turn_deg {turn} must be in (settle+tail='
                f'{cfg.settle_deg + cfg.tail_deg:.0f}, 720]')
        min_clear = _num(d, 'min_clearance_m', REQUEST_DEFAULTS['min_clearance_m'])
        if min_clear < 0.0:
            raise ValueError('min_clearance_m must be >= 0')
        max_r = _num(d, 'max_radius_m', REQUEST_DEFAULTS['max_radius_m'])
        if max_r <= 0.0:
            raise ValueError('max_radius_m must be > 0')
        max_dur = _num(d, 'max_duration_s', REQUEST_DEFAULTS['max_duration_s'])
        if not (0.0 < max_dur <= 600.0):
            raise ValueError('max_duration_s must be in (0, 600]')
        geom = None
        if not cfg.pedestal_test:
            geom = build_geometry(d.get('pin'), d.get('venue'))
    except (TypeError, ValueError, KeyError) as exc:
        return None, f'bad request: {exc}'
    return CalibRequest(seq, mode, sp, steer, R_plan, turn, min_clear, max_r,
                        max_dur, geom), None


# ----------------------------------------------------------------------------
# Segment bookkeeping + statistics
# ----------------------------------------------------------------------------

class SegmentLog:
    """One full-lock segment: yaw progress, window samples, statistics."""

    def __init__(self, v, direction, t0, duration_s=None):
        self.v = v
        self.dir = direction
        self.t0 = t0
        self.duration_s = duration_s   # pedestal (time-based) segments only
        self.t_end = None
        self.dpsi = 0.0                # IMU-integrated yaw change (rad, CCW+)
        self.win_wz = []
        self.win_v = []
        self.win_fix = []              # (t_meas, e, n)
        self.win_t0 = None
        self.win_t1 = None
        self.recent = deque()          # (t, wz) for the not-turning check
        self.complete = False

    @property
    def name(self):
        return segment_name(self.v, self.dir)

    def progress(self):
        """Signed yaw progress in the COMMANDED direction (rad)."""
        return self.dir * self.dpsi

    def in_window(self, t, lo_rad, hi_rad, turn_rad):
        if self.duration_s is not None:
            f = (t - self.t0) / self.duration_s
            return lo_rad / turn_rad <= f <= hi_rad / turn_rad
        return lo_rad <= self.progress() <= hi_rad

    def stats(self, cfg):
        w = median_or_nan(self.win_wz)
        v = median_or_nan(self.win_v)
        R_imu = v / abs(w) if (_finite(w, v) and w != 0.0) else float('nan')
        delta = (math.atan(cfg.wheelbase_m / R_imu)
                 if _finite(R_imu) and R_imu > 0.0 else float('nan'))
        n_fix = len(self.win_fix)
        R_ant = R_rear = resid = float('nan')
        if n_fix >= cfg.min_fixes:
            fit = kasa_fit([(e, n) for (_t, e, n) in self.win_fix])
            if fit is not None:
                R_ant, resid = fit[2], fit[3]
                R_rear = rear_axle_radius(R_ant, self.dir, cfg.antenna_fwd_m,
                                          cfg.antenna_right_m)
        v_rtk = median_or_nan(fix_speeds(self.win_fix))
        window_s = (self.win_t1 - self.win_t0
                    if self.win_t0 is not None else float('nan'))
        return {
            'v_cmd': self.v,
            'dir': 'left' if self.dir > 0 else 'right',
            'turned_deg': math.degrees(self.progress()),
            'window_s': window_s,
            'n_imu': len(self.win_wz),
            'w_imu_radps': w,
            'v_odom_mps': v,
            'R_imu_m': R_imu,
            'delta_imu_rad': delta,
            'n_fix': n_fix,
            'R_rtk_ant_m': R_ant,
            'R_rtk_rear_m': R_rear,
            'fit_resid_m': resid,
            'v_rtk_mps': v_rtk,
            'complete': self.complete,
        }


# ----------------------------------------------------------------------------
# Sequencer state machine (pure; the node feeds it samples and a 20 Hz tick)
# ----------------------------------------------------------------------------

class CalibSequencer:
    """Figure-8 sequencer.

    Feed sensor samples with ``ingest_imu/ingest_odom/ingest_fix`` (or pass them
    to ``step``) and call ``step(t, ...)`` at the control rate. ``t`` is any
    monotonic clock in seconds (the node uses time.monotonic()). ``step``
    returns ``(cmd, events)``: cmd is (v, w) to publish on cmd_vel_raw, or None
    to stay silent; events are log lines for this tick.

    Internal states: waiting -> running -> stopping -> done, or -> aborted from
    waiting/running. Once aborted/done the command is ZERO for stop_hold_s and
    then None (silent) — never non-zero again.
    """

    def __init__(self, req, cfg, t_accept):
        self.req = req
        self.cfg = cfg
        self.plan = [(float(v), d) for v in req.speeds for d in (LEFT, RIGHT)]
        self.state = 'waiting'
        self.reason = 'accepted; waiting for sensors'
        self.t_accept = t_accept
        self.t_start = None
        self.t_end = None              # stop/abort time (start of the zero hold)
        self.seg_i = -1
        self.seg = None
        self.segments = []             # completed SegmentLogs
        self.result = None
        self.events = []
        self._imu_t = None
        self._imu_wz = None
        self._odom_t = None
        self._fix_t = None
        self._fix_en = None
        self._estop = None             # None = state not yet known
        self._lo = math.radians(cfg.settle_deg)
        self._turn = math.radians(req.turn_deg)
        self._hi = self._turn - math.radians(cfg.tail_deg)

    # -- sample ingestion ---------------------------------------------------

    def _seg_in_window(self, t):
        return self.seg.in_window(t, self._lo, self._hi, self._turn)

    def ingest_imu(self, t, wz, dt=None):
        """Gyro z (rad/s, REP-103 CCW+ before imu_yaw_sign). dt = integration
        step from sensor stamps when known, else receive-time spacing."""
        try:
            wz = float(wz) * self.cfg.imu_yaw_sign
        except (TypeError, ValueError):
            return
        if not math.isfinite(wz):
            return
        prev_t, prev_wz = self._imu_t, self._imu_wz
        self._imu_t, self._imu_wz = t, wz
        seg = self.seg
        if self.state != 'running' or seg is None:
            return
        if prev_t is not None:
            if dt is None:
                dt = t - prev_t
            dt = min(max(dt, 0.0), self.cfg.imu_stale_s)
            seg.dpsi += 0.5 * (wz + prev_wz) * dt
        seg.recent.append((t, wz))
        while seg.recent and seg.recent[0][0] < t - self.cfg.turn_avg_s:
            seg.recent.popleft()
        if self._seg_in_window(t):
            seg.win_wz.append(wz)
            if seg.win_t0 is None:
                seg.win_t0 = t
            seg.win_t1 = t

    def ingest_odom(self, t, v):
        try:
            v = float(v)
        except (TypeError, ValueError):
            return
        if not math.isfinite(v):
            return
        self._odom_t = t
        if self.state == 'running' and self.seg is not None \
                and self._seg_in_window(t):
            self.seg.win_v.append(v)

    def ingest_fix(self, t, en, t_meas=None):
        """A VALID fix already projected to the pin-centred EN frame."""
        try:
            e, n = float(en[0]), float(en[1])
        except (TypeError, ValueError, IndexError):
            return
        if not math.isfinite(e) or not math.isfinite(n):
            return
        self._fix_t = t
        self._fix_en = (e, n)
        if self.state == 'running' and self.seg is not None \
                and self._seg_in_window(t):
            self.seg.win_fix.append((t if t_meas is None else t_meas, e, n))

    # -- tick ---------------------------------------------------------------

    def step(self, t, imu_wz=None, odom_v=None, fix_en=None, estop=None):
        if imu_wz is not None:
            self.ingest_imu(t, imu_wz)
        if odom_v is not None:
            self.ingest_odom(t, odom_v)
        if fix_en is not None:
            self.ingest_fix(t, fix_en)
        if estop is not None:
            self._estop = bool(estop)
        self.events = []

        if self.state == 'waiting':
            if self._estop:
                self._abort(t, 'e-stop active at start')
                return ZERO_CMD, self.events
            missing = self._not_ready(t)
            if missing:
                if t - self.t_accept > self.cfg.start_timeout_s:
                    self._abort(t, f'start timeout: {missing} not ready after '
                                   f'{self.cfg.start_timeout_s:.1f} s')
                    return ZERO_CMD, self.events
                self.reason = f'accepted; waiting for {missing}'
                return None, self.events
            self._start(t)

        if self.state == 'running':
            why = self._check_aborts(t)
            if why:
                self._abort(t, why)
                return ZERO_CMD, self.events
            if self._segment_complete(t):
                self._finish_segment(t)
                if self.state != 'running':
                    return ZERO_CMD, self.events
            return self._command(), self.events

        if self.state == 'stopping':
            if t - self.t_end >= self.cfg.stop_hold_s:
                self._finish(t)
            return ZERO_CMD, self.events

        # done / aborted: hold zero until stop_hold_s after the stop/abort
        # instant, then go silent (C6). ('done' already held zero while
        # 'stopping', so it is silent from the tick after it is reached.)
        if t - self.t_end < self.cfg.stop_hold_s:
            return ZERO_CMD, self.events
        return None, self.events

    def terminate(self, t, reason):
        """External stop (SIGTERM/SIGINT/exception). The caller publishes the
        zero burst; this only settles the state/result."""
        self.events = []
        if self.state in ('waiting', 'running'):
            self._abort(t, reason)
        elif self.state == 'stopping':
            self._finish(t)
        return self.events

    # -- internals ----------------------------------------------------------

    @staticmethod
    def _age(stamp, t):
        return float('inf') if stamp is None else t - stamp

    def _not_ready(self, t):
        c = self.cfg
        missing = []
        if self._age(self._imu_t, t) > c.imu_stale_s:
            missing.append('IMU')
        if self._age(self._odom_t, t) > c.odom_stale_s:
            missing.append('odom')
        if not c.pedestal_test and c.require_rtk and \
                self._age(self._fix_t, t) > c.rtk_stale_s:
            missing.append('RTK fix')
        if self._estop is None:
            missing.append('e-stop state')
        return ', '.join(missing)

    def _start(self, t):
        self.t_start = t
        self.state = 'running'
        self.reason = ''
        self.events.append(
            f'start: mode={self.req.mode} speeds={self.req.speeds} '
            f'steer_cmd={self.req.steer_cmd_rad:.3f} rad '
            f'pedestal={self.cfg.pedestal_test}')
        self._begin_segment(0, t)

    def _begin_segment(self, i, t):
        v, d = self.plan[i]
        dur = None
        if self.cfg.pedestal_test:
            dur = min(2.0 * math.pi * self.req.R_plan_m / v,
                      self.cfg.pedestal_seg_cap_s)
        self.seg_i = i
        self.seg = SegmentLog(v, d, t, dur)
        self.events.append(f'segment {self.seg.name} start '
                           f'(w_cmd={self._command()[1]:+.3f} rad/s)')

    def _segment_complete(self, t):
        seg = self.seg
        if seg.duration_s is not None:
            return t - seg.t0 >= seg.duration_s
        return seg.progress() >= self._turn

    def _finish_segment(self, t):
        seg = self.seg
        seg.complete = True
        seg.t_end = t
        self.segments.append(seg)
        s = seg.stats(self.cfg)
        self.events.append(
            f"segment {seg.name} done: turned {s['turned_deg']:.1f} deg in "
            f"{t - seg.t0:.1f} s, R_imu={s['R_imu_m']:.3f} m, "
            f"R_rtk_rear={s['R_rtk_rear_m']:.3f} m (n_fix={s['n_fix']})")
        if self.seg_i + 1 < len(self.plan):
            self._begin_segment(self.seg_i + 1, t)
        else:
            self.state = 'stopping'
            self.reason = 'all segments done; holding zero'
            self.t_end = t
            self.events.append('all segments done: zero hold')

    def _command(self):
        seg = self.seg
        return (seg.v, omega_cmd(seg.v, self.req.steer_cmd_rad,
                                 self.cfg.wheelbase_m, seg.dir))

    def _check_aborts(self, t):
        c, r, seg = self.cfg, self.req, self.seg
        if self._estop:
            return 'e-stop active'
        T = t - self.t_start
        if T > r.max_duration_s:
            return (f'max_duration: run time {T:.1f} s > max_duration_s '
                    f'{r.max_duration_s:.1f} s')
        a = self._age(self._imu_t, t)
        if a > c.imu_stale_s:
            return f'IMU stale ({a:.2f} s > {c.imu_stale_s:.2f} s)'
        a = self._age(self._odom_t, t)
        if a > c.odom_stale_s:
            return f'odom stale ({a:.2f} s > {c.odom_stale_s:.2f} s)'
        if c.pedestal_test:
            return None
        fa = self._age(self._fix_t, t)
        if c.require_rtk and fa > c.rtk_stale_s:
            return (f'RTK stale: no valid fix for {fa:.2f} s > '
                    f'{c.rtk_stale_s:.2f} s (require_rtk)')
        if fa <= c.rtk_stale_s:
            why = self._containment(self._fix_en)
            if why:
                return why
        ts = t - seg.t0
        if ts > c.not_turning_after_s and seg.recent:
            w = sum(x[1] for x in seg.recent) / len(seg.recent)
            thr = c.not_turning_frac * seg.v / r.R_plan_m
            sw = seg.dir * w
            if sw < -thr:
                return (f'turning opposite to command in {seg.name}: '
                        f'gyro z {w:+.3f} rad/s (check steering/IMU sign)')
            if sw < thr:
                return (f'not turning in {seg.name}: |gyro z| {abs(w):.3f} < '
                        f'{thr:.3f} rad/s after {ts:.1f} s')
        lim = (c.seg_timeout_factor * 2.0 * math.pi * r.R_plan_m / seg.v
               + c.seg_timeout_pad_s)
        if ts > lim:
            return (f'segment timeout in {seg.name}: {ts:.1f} s > {lim:.1f} s '
                    f'(turned {math.degrees(seg.progress()):.0f} deg)')
        return None

    def _containment(self, en):
        r = self.req
        d = math.hypot(en[0], en[1])
        if d > r.max_radius_m:
            return (f'max_radius: RTK position {d:.2f} m from pin > '
                    f'{r.max_radius_m:.2f} m')
        m, what = containment_margin(en, r.geom)
        if m < r.min_clearance_m:
            if what == 'polygon':
                if m < 0.0:
                    return (f'containment: RTK position outside the venue '
                            f'polygon ({-m:.2f} m out)')
                return (f'containment: venue edge clearance {m:.2f} m < '
                        f'min_clearance_m {r.min_clearance_m:.2f} m')
            if m < 0.0:
                return f'containment: RTK position inside {what}'
            return (f'containment: {what} clearance {m:.2f} m < '
                    f'min_clearance_m {r.min_clearance_m:.2f} m')
        return None

    def _abort(self, t, reason):
        partial = None
        if self.seg is not None and not self.seg.complete:
            partial = self.seg
        self.state = 'aborted'
        self.reason = reason
        self.t_end = t
        self.result = self._build_result(False, reason, t, partial)
        self.events.append(f'ABORT: {reason}')

    def _finish(self, t):
        c = self.cfg
        self.state = 'done'
        complete = len(self.segments) == len(self.plan)
        finite = all(_finite(s.stats(c)['R_imu_m']) for s in self.segments)
        if c.pedestal_test:
            ok, reason = False, 'pedestal test'
        elif not complete:
            ok, reason = False, (f'only {len(self.segments)}/{len(self.plan)} '
                                 'segments completed')
        elif not finite:
            bad = [s.name for s in self.segments
                   if not _finite(s.stats(c)['R_imu_m'])]
            ok, reason = False, f'no finite R_imu in: {", ".join(bad)}'
        else:
            ok, reason = True, 'ok'
        self.reason = reason
        self.result = self._build_result(ok, reason, t, None)
        self.events.append(f'done: ok={ok} ({reason})')

    def _build_result(self, ok, reason, t, partial):
        c, r = self.cfg, self.req
        segs = [s.stats(c) for s in self.segments]
        if partial is not None:
            segs.append(partial.stats(c))
        return {
            'ok': bool(ok),
            'reason': reason,
            'mode': r.mode,
            'steer_cmd_rad': r.steer_cmd_rad,
            'pedestal_test': bool(c.pedestal_test),
            'segments': segs,
            'stamp_utc': datetime.now(timezone.utc).isoformat(
                timespec='seconds'),
            'seq': r.seq,
            'speeds': list(r.speeds),
            'R_plan_m': r.R_plan_m,
            'turn_deg': r.turn_deg,
            'wheelbase_m': c.wheelbase_m,
            'duration_s': (t - self.t_start if self.t_start is not None
                           else 0.0),
        }

    # -- status -------------------------------------------------------------

    def status_payload(self, stamp, t=None):
        seg = self.seg
        if self.state == 'waiting':
            seg_name = ''
        elif self.state == 'stopping':
            seg_name = 'stop'
        else:
            seg_name = seg.name if seg is not None else ''
        p = {
            'seq': self.req.seq,
            'state': _STATUS_STATE[self.state],
            'segment': seg_name,
            'reason': self.reason,
            'stamp': stamp,
            'turned_deg': (math.degrees(seg.progress())
                           if seg is not None else None),
            'elapsed_s': (t - self.t_start
                          if (t is not None and self.t_start is not None)
                          else None),
        }
        if self.state in ('done', 'aborted'):
            p['result'] = self.result
        return p


def publish_zero_burst(publish_zero, n=SHUTDOWN_ZERO_COUNT,
                       gap_s=SHUTDOWN_ZERO_GAP_S, sleep=time.sleep):
    """Call publish_zero() n times, gap_s apart. Every call is attempted even
    if one raises (a half-torn-down publisher must not skip the rest).
    Returns the number of successful publishes."""
    ok = 0
    for i in range(n):
        try:
            publish_zero()
            ok += 1
        except Exception:  # noqa: BLE001 — keep trying the remaining zeros
            pass
        if i + 1 < n:
            sleep(gap_s)
    return ok


# ----------------------------------------------------------------------------
# ROS node
# ----------------------------------------------------------------------------

class CalibNode(Node):

    def __init__(self):
        super().__init__('calib_node')
        dflt = CalibConfig()
        for name in ('wheelbase_m', 'rate_hz', 'settle_deg', 'tail_deg',
                     'require_rtk', 'pedestal_test', 'antenna_fwd_m',
                     'antenna_right_m', 'stop_hold_s', 'start_timeout_s',
                     'imu_yaw_sign'):
            self.declare_parameter(name, getattr(dflt, name))
        g = self.get_parameter
        self._cfg = CalibConfig(
            wheelbase_m=float(g('wheelbase_m').value),
            rate_hz=float(g('rate_hz').value),
            settle_deg=float(g('settle_deg').value),
            tail_deg=float(g('tail_deg').value),
            require_rtk=bool(g('require_rtk').value),
            pedestal_test=bool(g('pedestal_test').value),
            antenna_fwd_m=float(g('antenna_fwd_m').value),
            antenna_right_m=float(g('antenna_right_m').value),
            stop_hold_s=float(g('stop_hold_s').value),
            start_timeout_s=float(g('start_timeout_s').value),
            imu_yaw_sign=float(g('imu_yaw_sign').value),
        )

        self._seqr = None              # CalibSequencer of the current/last run
        self._rejected = None          # (seq, reason) of a rejected request
        self._estop = None             # latest /estop (None = never received)
        self._imu_stamp = None         # last /imu header stamp (s)
        self._last_status_t = 0.0
        self._last_state = None

        latched = QoSProfile(
            depth=1, history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        rel = lambda depth: QoSProfile(  # noqa: E731
            depth=depth, history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE)

        # The ONLY two publishers this node owns.
        self.pub_cmd = self.create_publisher(Twist, 'cmd_vel_raw', 10)
        self.pub_status = self.create_publisher(String, '/calib/status', latched)

        # /imu is 100 Hz RELIABLE on the LIMO base (heading_node, verified
        # 2026-06-08); a deeper queue keeps the yaw integral gap-free.
        self.create_subscription(Imu, '/imu', self._imu_cb, rel(50))
        self.create_subscription(Odometry, '/wheel/odom', self._odom_cb, rel(10))
        self.create_subscription(NavSatFix, '/gps_rtk_f9p_helical/gps/fix',
                                 self._fix_cb, rel(10))
        # /estop is a latched STATE signal (estop_cli: TRANSIENT_LOCAL + 2 Hz
        # heartbeat) -> subscribe latched to read the current state on join.
        self.create_subscription(Bool, '/estop', self._estop_cb, latched)
        self.create_subscription(String, '/calib/request', self._request_cb,
                                 rel(10))

        self.create_timer(1.0 / max(1.0, self._cfg.rate_hz), self._tick)
        self._publish_status()
        self.get_logger().info(
            'calib_node up (idle). cmd_vel_raw silent until /calib/request. '
            f'pedestal_test={self._cfg.pedestal_test} '
            f'require_rtk={self._cfg.require_rtk} L={self._cfg.wheelbase_m} m')

    # -- subscriptions --------------------------------------------------------

    def _imu_cb(self, msg):
        t = time.monotonic()
        st = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        dt = imu_dt_from_stamps(self._imu_stamp, st, self._cfg.imu_stale_s)
        self._imu_stamp = st
        if self._seqr is not None:
            self._seqr.ingest_imu(t, msg.angular_velocity.z, dt)

    def _odom_cb(self, msg):
        if self._seqr is None:
            return
        tw = msg.twist.twist.linear
        self._seqr.ingest_odom(time.monotonic(), odom_speed(tw.x, tw.y))

    def _fix_cb(self, msg):
        s = self._seqr
        if s is None or s.req.geom is None:
            return
        if not fix_is_valid(msg.status.status, msg.latitude, msg.longitude):
            return
        en = s.req.geom.to_en(msg.latitude, msg.longitude)
        t_meas = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        s.ingest_fix(time.monotonic(), en, t_meas if t_meas > 0.0 else None)

    def _estop_cb(self, msg):
        self._estop = bool(msg.data)
        if self._estop and self._seqr is not None \
                and self._seqr.state in _ACTIVE:
            self._tick()   # zero NOW, not on the next 50 ms tick

    def _request_cb(self, msg):
        try:
            d = json.loads(msg.data)
        except (ValueError, TypeError) as exc:
            self.get_logger().warn(f'calib: unparseable /calib/request: {exc}')
            return
        new_seq = d.get('seq') if isinstance(d, dict) else None
        if self._seqr is not None:
            cur_seq, cur_state = self._seqr.req.seq, self._seqr.state
        elif self._rejected is not None:
            cur_seq, cur_state = self._rejected[0], 'aborted'
        else:
            cur_seq, cur_state = None, None
        act = request_action(cur_seq, cur_state, new_seq)
        if act == 'ack':
            self._publish_status()
            return
        if act == 'busy':
            self.get_logger().warn(
                f'calib: ignoring request seq={new_seq} while seq={cur_seq} '
                f'is {cur_state}', throttle_duration_sec=5.0)
            self._publish_status()
            return
        self.get_logger().info(f'calib: request <- {msg.data}')
        req, err = parse_request(d, self._cfg)
        if err:
            self._seqr = None
            self._rejected = (new_seq, err)
            self.get_logger().error(f'calib: REJECTED request seq={new_seq}: {err}')
            self._publish_status()
            return
        self._rejected = None
        self._seqr = CalibSequencer(req, self._cfg, time.monotonic())
        self._publish_status()

    # -- control tick -----------------------------------------------------------

    def _tick(self):
        s = self._seqr
        t = time.monotonic()
        if s is not None:
            cmd, events = s.step(t, estop=self._estop)
            if cmd is not None:
                self._publish_cmd(cmd)
            self._log_events(s, events)
        state = (s.state if s is not None
                 else ('rejected' if self._rejected else 'idle'))
        if state != self._last_state or t - self._last_status_t >= 0.5:
            self._publish_status()

    def _publish_cmd(self, cmd):
        m = Twist()
        m.linear.x = float(cmd[0])
        m.angular.z = float(cmd[1])
        self.pub_cmd.publish(m)

    def _publish_zero(self):
        self.pub_cmd.publish(Twist())

    def _log_events(self, s, events):
        for ev in events:
            if ev.startswith('ABORT'):
                self.get_logger().warn(f'calib: {ev}')
            else:
                self.get_logger().info(f'calib: {ev}')
        if events and s.state in ('done', 'aborted') and s.result is not None \
                and any(e.startswith(('ABORT', 'done')) for e in events):
            self.get_logger().info(
                'calib: result ' + json.dumps(json_safe(s.result),
                                              allow_nan=False))

    def _publish_status(self):
        t = time.monotonic()
        stamp = time.time()
        s = self._seqr
        if s is not None:
            p = s.status_payload(stamp, t)
            state = s.state
        elif self._rejected is not None:
            seq, reason = self._rejected
            p = {'seq': seq, 'state': 'aborted', 'segment': '',
                 'reason': reason, 'stamp': stamp,
                 'result': {'ok': False, 'reason': reason, 'segments': [],
                            'pedestal_test': bool(self._cfg.pedestal_test),
                            'stamp_utc': datetime.now(timezone.utc).isoformat(
                                timespec='seconds')}}
            state = 'rejected'
        else:
            p = {'seq': None, 'state': 'idle', 'segment': '', 'reason': '',
                 'stamp': stamp}
            state = 'idle'
        m = String()
        m.data = json.dumps(json_safe(p), allow_nan=False)
        self.pub_status.publish(m)
        self._last_status_t = t
        self._last_state = state

    # -- shutdown -------------------------------------------------------------

    def emergency_stop(self, reason):
        """Zero burst on cmd_vel_raw, then settle + publish the final status."""
        n = publish_zero_burst(self._publish_zero)
        s = self._seqr
        if s is not None:
            events = s.terminate(time.monotonic(), reason)
            self._log_events(s, events)
        try:
            self._publish_status()
        except Exception:  # noqa: BLE001
            pass
        self.get_logger().warn(
            f'calib: shutdown ({reason}): published {n} zero Twist(s)')
        time.sleep(0.05)   # let DDS flush the zeros before teardown


def main(args=None):
    if not _HAVE_ROS:
        raise SystemExit('calib_node needs ROS 2 (rclpy) to run')
    # rclpy's own SIGINT/SIGTERM handler shuts the context down BEFORE our
    # finally-block runs, so a zero publish there would fail. Disable it and
    # handle the signals ourselves: the handler only sets a flag; the spin loop
    # (50 ms timeout) notices it and runs the zero burst on a live context.
    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    except (ImportError, TypeError):
        rclpy.init(args=args)
    stop = {'sig': None}

    def _on_signal(signum, _frame):
        if stop['sig'] is None:
            stop['sig'] = signum

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    node = CalibNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    reason = 'node exit'
    try:
        while stop['sig'] is None and rclpy.ok():
            executor.spin_once(timeout_sec=0.05)
        if stop['sig'] is not None:
            reason = f'terminated by {signal.Signals(stop["sig"]).name}'
    except BaseException as exc:  # noqa: BLE001 — always reach the zero burst
        reason = f'node error: {type(exc).__name__}: {exc}'
        raise
    finally:
        try:
            node.emergency_stop(reason)
        finally:
            try:
                executor.remove_node(node)
                node.destroy_node()
            finally:
                if rclpy.ok():
                    rclpy.shutdown()


if __name__ == '__main__':
    main()
