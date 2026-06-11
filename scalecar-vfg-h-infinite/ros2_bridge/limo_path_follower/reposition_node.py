# -*- coding: utf-8 -*-
"""RTK reposition node — forward-only Ackermann path follower (ROC R1-R4).

What this node IS
-----------------
A *dumb executor*. The orchestrator (run_executor / experiment_sequencer) owns
the venue, the experiment curves, and the reposition curves between them; it
picks the reposition curve for the next leg and hands this node a pre-validated
**waypoint polyline** on ``/reposition/goto``. This node's only job is:

    follow that polyline to its last point and stop there, reasonably close in
    position and heading, then report ``arrived``.

It does NOT read curves from the venue file, does NOT know what an "experiment
curve" is, and does NOT choose which curve to run. It tracks the polyline it is
given. Curve *selection* lives upstream; this node only needs the venue file for
its own working-area safety guard (R3 backstop).

Topology (system_spec §3.4, §4 interface contract):

  sub  /heading/fused                        std_msgs/Float64  bearing deg E-of-N
  sub  /gps_rtk_f9p_helical/gps/fix          sensor_msgs/NavSatFix
  sub  /gps_rtk_f9p_helical/gps/rtk_status   std_msgs/String  (carries quality=N)
  sub  /reposition/goto                      std_msgs/String  JSON
  pub  /reposition/status                    std_msgs/String  JSON
                                             {state, err_m, err_deg, reason, seq}
                                             (seq echoes the goto's 'seq' id)
  pub  cmd_vel_raw                           geometry_msgs/Twist
                                             (ONLY while driving; routed through
                                              estop_cli.py -> /cmd_vel, C3)

state in {'idle','waiting','driving','arrived','aborted'}. The orchestrator only
branches on 'arrived'/'aborted'; the rest are informational. ``cmd_vel_raw`` is
silent unless state == 'driving' (C6: follower XOR reposition).

Feedback sources
----------------
* Position: RTK fix (``/gps_rtk_f9p_helical/gps/fix``) projected into the shared
  local metric frame (tools/path_gen/path_overlay.py rooftop anchor).
* Heading: ``/heading/fused`` from the always-on heading_node EKF (gyro +
  Pixhawk compass + RTK COG; standstill-capable). This node does NOT estimate
  heading; it consumes the already-calibrated fused bearing and converts it to
  the local frame via ``_bearing_deg_to_local_yaw``. If fused goes stale, HOLD.

RTK quality policy (R1 — DIVERGES FROM THE LOCKED SPEC, by operator decision)
-----------------------------------------------------------------------------
system_spec R1/§5 say "RTK FIXED only". Field reality (2026-06): the base often
cannot hold FIXED, so reposition is allowed to drive on **FIXED(4) OR FLOAT(5)**,
with a **quality-adaptive arrival tolerance** — tight on FIXED, widened on FLOAT
so we never chase sub-noise precision FLOAT can't deliver. This relaxation is
scoped to *between-run repositioning* only (L4); the recorded-run preflight gate
(M1) and RTK-loss fault policy (F2/F3) stay strictly FIXED, so ground truth is
never recorded on FLOAT. The locked spec text was NOT amended (operator chose
node-only changes 2026-06-09); this docstring is the record of the divergence —
flag to reconcile R1/§5 when convenient.

Kinematics — forward-only Ackermann
-----------------------------------
``motion_mode == Ackermann`` (system_spec §6): the chassis steers, it CANNOT spin
in place, and geometric R_min ≈ 0.37 m. So:

* The controller is **pure pursuit**: pick a look-ahead point on the polyline,
  compute the path curvature kappa to reach it, and command angular.z = v*kappa.
  kappa is **clamped to 1/R_min** — the real physical limit. (The old node
  clamped a fixed yaw rate independent of speed, which at creep implied a turn
  radius tighter than the chassis can steer.) angular.z -> 0 as v -> 0; we never
  command a turn at zero speed.
* It drives **forward only** — no reverse, no 3-point turn. The operator authors
  a reposition curve that is forward-drivable from the leg's start pose. On
  mission start the tracker JOINS the path at the closest segment whose target
  is in front and within infeasible_deg of the nose (a robot standing mid-path
  joins where it stands; the prefix behind it is skipped, not driven). If NO
  segment is forward-reachable, this node ABORTS asking for re-placement or a
  re-authored curve rather than loop.
* Heading at the pin (R2) is achieved by *geometry, not in-place rotation*: the
  operator draws the curve tail straight and roughly aligned with the experiment
  curve's start heading, the tracker follows it, and the arrival heading is the
  tail tangent it tracked onto. An Ackermann robot cannot correct heading once
  stopped at a point, so arrival is **gated on position**; the achieved heading
  error is reported in status (and folded into the reason vs heading_tol), not
  used as a blocking gate. Author a longer/straighter tail if you need it tighter.

Safety (R3 + backstops)
-----------------------
* Pre-flight every received polyline against the inset working area + inflated
  exclusions before committing (defense in depth on top of the orchestrator's
  own check — venue files can drift between layers).
* Live guard each tick: if the RTK pose leaves the inset area / enters an
  exclusion, ABORT. Corridor guard: once the robot has acquired the path, if it
  wanders > corridor_m off the polyline, ABORT (catches a wrong-way drive or a
  bad heading convention long before the area gate).
* On any abort: stop and hold (F1) — zero ``cmd_vel_raw`` and report. This node
  never tries to recover; the operator/orchestrator decides the next step.

Conventions to VERIFY on the robot (do not assume — see memory)
---------------------------------------------------------------
1. That the base interprets ``cmd_vel_raw.angular.z`` as a yaw rate (REP-103,
   +z = CCW) and derives the Ackermann steering angle itself. The ``omega =
   v*kappa`` output and the steering sign both depend on this.
2. That ``_bearing_deg_to_local_yaw`` has the correct sign for this anchor (the
   known heading-convention hazard). A wrong sign makes the robot steer the
   wrong way; the corridor guard will abort it quickly rather than run it away.
3. R_min / wheelbase and the FIXED/FLOAT tolerance bands (tune wheels-on-floor).
"""

import json
import math
import os
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
)
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float64

# Reuse the shared geo anchor + projections. The node lives in the colcon tree;
# tools/ is not on the ament path, so we import path_overlay by file location.
import importlib.util as _ilu

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
# Resolve the H-infinity repo root: env override, the known NUC repo root, then
# the in-source layout (laptop / rsync'd tree). First whose path_overlay exists.
_ROOT_CANDIDATES = [p for p in [
    os.environ.get('H_INFINITY_ROOT'),
    '/home/agilex/H-infinity',
    os.path.abspath(os.path.join(_THIS_DIR, '..', '..', '..')),
] if p]
_REPO_ROOT = next(
    (p for p in _ROOT_CANDIDATES
     if os.path.isfile(os.path.join(p, 'tools', 'path_gen', 'path_overlay.py'))),
    _ROOT_CANDIDATES[-1])
_PATH_OVERLAY = os.path.join(_REPO_ROOT, 'tools', 'path_gen', 'path_overlay.py')

# RTK fix qualities this node will drive on: FIXED(4) and FLOAT(5). See the
# "RTK quality policy" note in the module docstring (R1 divergence).
_ALLOWED_RTK = (4, 5)
_RTK_FIXED = 4


def _load_path_overlay():
    """Import tools/path_gen/path_overlay.py by file location (not a colcon
    package). Returns the module or raises ImportError. The node degrades
    gracefully: without the geo helpers it refuses to plan rather than fabricate
    coordinates."""
    if not os.path.isfile(_PATH_OVERLAY):
        raise ImportError(f'path_overlay.py not found at {_PATH_OVERLAY} '
                          f'(tried roots: {", ".join(_ROOT_CANDIDATES)})')
    spec = _ilu.spec_from_file_location('h_path_overlay', _PATH_OVERLAY)
    if spec is None or spec.loader is None:
        raise ImportError(f'cannot load path_overlay from {_PATH_OVERLAY}')
    mod = _ilu.module_from_spec(spec)
    # Register before exec: path_overlay defines an @dataclass (Anchor), which
    # resolves its owning module via sys.modules during class construction.
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def _wrap(angle):
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


# ----------------------------------------------------------------------------
# Pure-geometry helpers (no ROS) — module-level so they are unit-testable off
# the robot.
# ----------------------------------------------------------------------------

def point_in_polygon(pt, poly):
    """Ray-cast point-in-polygon test. poly is a list of (x, y) vertices."""
    x, y = pt
    inside = False
    n = len(poly)
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if ((yi > y) != (yj > y)) and \
                (x < (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi):
            inside = not inside
        j = i
    return inside


def inset_polygon(corners, margin):
    """Shrink a convex-ish polygon inward by `margin` toward its centroid.

    Exact for a rectangle (the rooftop case), conservative for mild
    quadrilaterals: every inset vertex moves strictly inward.
    """
    cx = sum(p[0] for p in corners) / len(corners)
    cy = sum(p[1] for p in corners) / len(corners)
    out = []
    for (x, y) in corners:
        dx = cx - x
        dy = cy - y
        d = math.hypot(dx, dy)
        if d < 1e-9:
            out.append((x, y))
            continue
        out.append((x + dx / d * margin, y + dy / d * margin))
    return out


def seg_clears_circles(p0, p1, circles, n_samples=24):
    """True if the segment p0->p1 stays outside every (cx, cy, r) circle.
    Sampled containment (cheap, conservative for the short reposition legs)."""
    for i in range(n_samples + 1):
        t = i / n_samples
        x = p0[0] + (p1[0] - p0[0]) * t
        y = p0[1] + (p1[1] - p0[1]) * t
        for (cx, cy, r) in circles:
            if math.hypot(x - cx, y - cy) <= r:
                return False
    return True


def seg_in_polygon(p0, p1, poly, n_samples=24):
    """True if every sampled point of segment p0->p1 lies inside poly."""
    for i in range(n_samples + 1):
        t = i / n_samples
        x = p0[0] + (p1[0] - p0[0]) * t
        y = p0[1] + (p1[1] - p0[1]) * t
        if not point_in_polygon((x, y), poly):
            return False
    return True


def point_seg_dist(p, a, b):
    """Shortest distance from point p to the segment a->b."""
    ax, ay = a
    bx, by = b
    px, py = p
    dx = bx - ax
    dy = by - ay
    L2 = dx * dx + dy * dy
    if L2 < 1e-12:
        return math.hypot(px - ax, py - ay)
    t = ((px - ax) * dx + (py - ay) * dy) / L2
    t = max(0.0, min(1.0, t))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def point_seg_nearest(p, a, b):
    """Nearest point to p on the segment a->b."""
    ax, ay = a
    bx, by = b
    px, py = p
    dx = bx - ax
    dy = by - ay
    L2 = dx * dx + dy * dy
    if L2 < 1e-12:
        return (ax, ay)
    t = ((px - ax) * dx + (py - ay) * dy) / L2
    t = max(0.0, min(1.0, t))
    return (ax + t * dx, ay + t * dy)


def polyline_dist(p, pts):
    """Cross-track distance from p to the nearest point on the polyline pts."""
    if len(pts) == 1:
        return math.hypot(p[0] - pts[0][0], p[1] - pts[0][1])
    return min(point_seg_dist(p, pts[i], pts[i + 1])
               for i in range(len(pts) - 1))


def seg_circle_far_t(p, R, a, b):
    """Largest t in [0,1] where segment a->b crosses the circle (centre p,
    radius R), or None if it does not. Used by pure pursuit to pick the
    look-ahead point furthest forward on the current segment."""
    ax, ay = a
    bx, by = b
    px, py = p
    dx = bx - ax
    dy = by - ay
    A = dx * dx + dy * dy
    if A < 1e-12:
        return None
    fx = ax - px
    fy = ay - py
    B = 2.0 * (fx * dx + fy * dy)
    C = fx * fx + fy * fy - R * R
    disc = B * B - 4.0 * A * C
    if disc < 0.0:
        return None
    sq = math.sqrt(disc)
    for cand in ((-B + sq) / (2.0 * A), (-B - sq) / (2.0 * A)):  # farthest first
        if 0.0 <= cand <= 1.0:
            return cand
    return None


# ============================================================================
# DEBUG I/O INSTRUMENTATION  (added 2026-06-09, post-rewrite, NOT yet runtime-
# verified). Every input callback and output publish carries a throttled log
# line tagged `[io-dbg]`  ->  grep io-dbg . Run the node, drive synthetic
# fix / rtk_status / heading / goto, and read the [io-dbg] trace to confirm what
# the node actually RECEIVED (inputs) and COMMANDED (cmd_vel_raw + status) before
# any wheels-on-floor.
# AGENTS / NEXT SESSION: the control law here is unverified on hardware. Use this
# trace to check what went wrong (inputs parsed? steering sign? clamp binding?),
# then gate behind a `debug_io` param or strip once confirmed on the robot.
# In:  _fix_cb / _rtk_cb / _fused_cb / _goto_cb      Out: _drive / _zero_cmd /
# _publish_status, plus a per-tick CTL vector in the driving branch of _control_cb.
# ============================================================================


class RepositionNode(Node):

    def __init__(self):
        super().__init__('reposition_node')

        # -- Parameters --------------------------------------------------
        # Arrival tolerance is quality-adaptive: the FIXED band is the goto's
        # pos_tol_m (or pos_tol_m param); on FLOAT it is widened to at least the
        # float floor so we never chase precision FLOAT can't deliver.
        self.declare_parameter('pos_tol_m', 0.15)            # FIXED arrival radius
        self.declare_parameter('pos_tol_float_m', 0.40)      # FLOAT arrival floor
        self.declare_parameter('heading_tol_deg', 5.0)       # FIXED heading band
        self.declare_parameter('heading_tol_float_deg', 10.0)  # FLOAT heading band
        # Speed: constant cruise, ramped down near the final point. Hard ceiling
        # well under the 1.0 m/s firmware cap; the goto's v_const overrides cruise.
        self.declare_parameter('cruise_speed', 0.30)         # m/s default
        self.declare_parameter('max_speed', 0.60)            # m/s clamp ceiling
        self.declare_parameter('min_speed', 0.08)            # m/s terminal floor
        self.declare_parameter('slowdown_m', 0.50)           # ramp start dist to pin
        # Pure pursuit look-ahead distance.
        self.declare_parameter('lookahead_m', 0.60)
        # Geometric minimum turn radius (system_spec §6). kappa_max = 1/R_min.
        self.declare_parameter('r_min_m', 0.37)
        # Synthesised straight-tail length for a bare single-pin+heading goto, so
        # the heading is reachable (the freelance entry-offset, only that case).
        self.declare_parameter('single_pin_tail_m', 0.60)
        # Off-path corridor guard + the radius within which the path counts as
        # acquired (the guard arms only after acquisition, so a start offset from
        # wp0 does not trip it).
        self.declare_parameter('corridor_m', 2.0)
        self.declare_parameter('acquire_radius_m', 1.0)
        # Heading error (deg) to the first look-ahead above which the start pose
        # is not forward-drivable (Ackermann, no reverse) -> abort.
        self.declare_parameter('infeasible_deg', 100.0)
        # RTK fix staleness timeout (s): no fresh fix -> hold + zero.
        self.declare_parameter('rtk_timeout_s', 1.0)
        # Fused-heading source + staleness.
        self.declare_parameter('fused_heading_topic', '/heading/fused')
        self.declare_parameter('fused_heading_timeout_s', 0.5)
        # Venue file: loaded once for the working-area + exclusion safety geometry.
        self.declare_parameter(
            'venue_file',
            os.path.join(_REPO_ROOT, 'scenarios', 'venues', 'rooftop.json'))
        self.declare_parameter('control_rate_hz', 20.0)

        g = self.get_parameter
        self._pos_tol_default = float(g('pos_tol_m').value)
        self._pos_tol_float = float(g('pos_tol_float_m').value)
        self._head_tol_fixed = math.radians(float(g('heading_tol_deg').value))
        self._head_tol_float = math.radians(float(g('heading_tol_float_deg').value))
        self._cruise = float(g('cruise_speed').value)
        self._max_speed = float(g('max_speed').value)
        self._min_speed = float(g('min_speed').value)
        self._slowdown = float(g('slowdown_m').value)
        self._lookahead = float(g('lookahead_m').value)
        r_min = max(1e-3, float(g('r_min_m').value))
        self._kappa_max = 1.0 / r_min
        self._tail_m = float(g('single_pin_tail_m').value)
        self._corridor = float(g('corridor_m').value)
        self._acquire_radius = float(g('acquire_radius_m').value)
        self._infeasible = math.radians(float(g('infeasible_deg').value))
        self._rtk_timeout = float(g('rtk_timeout_s').value)
        self._fused_topic = str(g('fused_heading_topic').value)
        self._fused_timeout = float(g('fused_heading_timeout_s').value)
        self._venue_file = str(g('venue_file').value)
        rate = float(g('control_rate_hz').value)

        # -- Geo helpers + venue geometry --------------------------------
        self._geo = None
        self._anchor = None
        self._inset = None        # working-area inset polygon, local (x, y)
        self._excl = []           # inflated exclusion circles (cx, cy, r) local
        try:
            self._geo = _load_path_overlay()
            self._anchor = self._geo.Anchor(
                self._geo.LAT0, self._geo.LON0, self._geo.BEARING_DEG)
            self._load_venue(self._venue_file)
        except Exception as exc:  # noqa: BLE001 — node must not crash on load
            self.get_logger().error(
                f'Geo/venue init failed ({exc}); reposition will refuse to plan '
                'until fixed. cmd_vel_raw stays silent.')

        # -- RTK + heading state -----------------------------------------
        self._rtk_quality = None     # latest parsed quality=N, or None
        self._last_logged_quality = None  # [io-dbg] last quality logged (on-change)
        self._fix_xy = None          # latest accepted fix in local (x, y)
        self._fix_stamp = None       # ros time of latest accepted fix
        self._heading_est = None     # fused heading in local frame (rad); None if stale
        self._fused_stamp = None     # ros time of latest /heading/fused

        # -- Mission state -----------------------------------------------
        self._state = 'idle'
        self._reason = ''
        # Orchestrator-supplied goto id, echoed verbatim in every status so the
        # caller can tell a fresh 'arrived' from the previous goto's (this node
        # keeps re-publishing 'arrived' after a mission while it stays alive
        # across consecutive glue curves). None until the first goto / for
        # manual gotos without a seq.
        self._goto_seq = None
        self._waypoints = None       # list of (x, y) local, the curve to follow
        self._seg_i = 0              # current pure-pursuit segment index (monotone)
        self._end_yaw = None         # arrival heading target in local frame (rad); None = position-only
        self._speed = self._cruise   # forward speed for this mission
        self._pos_tol = self._pos_tol_default  # FIXED arrival radius for this mission
        self._acquired = False       # has the robot reached the path corridor yet
        self._feasible_checked = False  # one-shot forward-drivable check done
        self._goto_start_dist = None    # distance to final at commit (info/logging)
        # Goto latched because it arrived before the first usable RTK fix
        # (spawn race); replanned from _fix_cb once a fix lands.
        self._pending_goto = None

        # -- ROS interfaces ----------------------------------------------
        self.sub_fix = self.create_subscription(
            NavSatFix, '/gps_rtk_f9p_helical/gps/fix', self._fix_cb, 10)
        self.sub_rtk = self.create_subscription(
            String, '/gps_rtk_f9p_helical/gps/rtk_status', self._rtk_cb, 10)
        self.sub_goto = self.create_subscription(
            String, '/reposition/goto', self._goto_cb, 10)
        # heading_node publishes /heading/fused LATCHED so a per-leg respawn of
        # this node gets the last fused heading immediately (warm start).
        fused_qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.sub_fused = self.create_subscription(
            Float64, self._fused_topic, self._fused_cb, fused_qos)

        self.pub_cmd = self.create_publisher(Twist, 'cmd_vel_raw', 10)
        status_qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_status = self.create_publisher(
            String, '/reposition/status', status_qos)

        self._timer = self.create_timer(1.0 / rate, self._control_cb)

        self._publish_status(err_m=float('nan'), err_deg=float('nan'))
        self.get_logger().info(
            'reposition_node started (idle, forward-only Ackermann pure-pursuit). '
            'Waiting for /reposition/goto. cmd_vel_raw silent until commanded; '
            f'RTK quality in {_ALLOWED_RTK} required.')

    # ------------------------------------------------------------------
    # Venue loading (working-area + exclusion safety geometry only)
    # ------------------------------------------------------------------

    def _load_venue(self, path):
        with open(path, 'r') as f:
            v = json.load(f)
        margin = float(v.get('safety_margin_m', 0.0))

        corners_xy = [
            tuple(self._geo.latlon_to_local((c['lat'], c['lon']), self._anchor)[0])
            for c in v['corners_wgs84']
        ]
        self._inset = inset_polygon(corners_xy, margin)

        self._excl = []
        for ex in v.get('exclusions', []):
            if ex.get('kind') != 'circle':
                self.get_logger().warn(
                    f"venue exclusion kind '{ex.get('kind')}' unsupported; skipping.")
                continue
            cxy = self._geo.latlon_to_local((ex['lat'], ex['lon']), self._anchor)[0]
            # Inflate by the safety margin: stay a margin clear of the island.
            r = float(ex['radius_m']) + margin
            self._excl.append((float(cxy[0]), float(cxy[1]), r))

        self.get_logger().info(
            f"venue '{v.get('name')}' loaded: inset polygon {len(self._inset)} "
            f'verts, {len(self._excl)} exclusion(s), margin {margin:.2f} m.')

    # ------------------------------------------------------------------
    # Geo conversion
    # ------------------------------------------------------------------

    def _bearing_deg_to_local_yaw(self, heading_deg):
        """Compass bearing (deg E of N) -> heading angle in the local frame.

        Local +x has compass bearing anchor.bearing_deg (E of N); local yaw is
        CCW from +x. A compass bearing increases clockwise from north, so::

            local_yaw = radians(anchor.bearing_deg - heading_deg)

        NOTE: this sign convention is the known heading-convention hazard — VERIFY
        on the robot. A wrong sign steers the robot the wrong way (the corridor
        guard then aborts it quickly).
        """
        return _wrap(math.radians(self._anchor.bearing_deg - heading_deg))

    # ------------------------------------------------------------------
    # Subscriptions
    # ------------------------------------------------------------------

    def _fused_cb(self, msg: Float64):
        """Fused heading from heading_node (already a calibrated true bearing).
        Sole writer of self._heading_est; staleness is enforced in _control_cb."""
        self._fused_stamp = self.get_clock().now()
        if self._anchor is not None:
            self._heading_est = self._bearing_deg_to_local_yaw(float(msg.data))
        # [io-dbg] INPUT heading (throttled): raw fused bearing -> local yaw.
        _he = ('n/a' if self._heading_est is None
               else f'{math.degrees(self._heading_est):.1f}')
        self.get_logger().info(
            f'[io-dbg] IN  fused={float(msg.data):.1f} deg(E-of-N) -> '
            f'heading_est={_he} deg(local)', throttle_duration_sec=2.0)

    def _fused_is_fresh(self):
        if self._fused_stamp is None:
            return False
        age = (self.get_clock().now() - self._fused_stamp).nanoseconds * 1e-9
        return age <= self._fused_timeout

    def _rtk_cb(self, msg: String):
        """Parse the `quality=N` token out of the rtk_status string (R1)."""
        q = None
        for tok in msg.data.replace('(', ' ').replace(',', ' ').split():
            if tok.startswith('quality='):
                try:
                    q = int(tok.split('=', 1)[1])
                except ValueError:
                    q = None
                break
        # [io-dbg] INPUT quality (on-change): the authority for the RTK gate.
        if q != self._last_logged_quality:
            self.get_logger().info(
                f'[io-dbg] IN  rtk_status quality={q} (drive-accept set {_ALLOWED_RTK}, '
                f'FIXED={_RTK_FIXED})')
            self._last_logged_quality = q
        self._rtk_quality = q

    def _fix_cb(self, msg: NavSatFix):
        """Record the latest fix in local (x, y), gated on FIXED/FLOAT quality so
        no-fix placeholder messages never enter the control geometry."""
        if self._geo is None or self._anchor is None:
            return
        if self._rtk_quality not in _ALLOWED_RTK:
            self.get_logger().info(
                f'[io-dbg] IN  fix DROPPED: quality={self._rtk_quality} not in '
                f'{_ALLOWED_RTK}', throttle_duration_sec=2.0)
            return
        lat = msg.latitude
        lon = msg.longitude
        if lat != lat or lon != lon:  # NaN guard (driver pre-fix placeholders)
            self.get_logger().info(
                '[io-dbg] IN  fix DROPPED: lat/lon NaN (driver pre-fix placeholder)',
                throttle_duration_sec=2.0)
            return
        xy = self._geo.latlon_to_local((lat, lon), self._anchor)[0]
        self._fix_xy = (float(xy[0]), float(xy[1]))
        self._fix_stamp = self.get_clock().now()
        # [io-dbg] INPUT position (throttled): accepted RTK fix -> local frame.
        self.get_logger().info(
            f'[io-dbg] IN  fix q={self._rtk_quality} ({lat:.7f}, {lon:.7f}) -> '
            f'local ({self._fix_xy[0]:.2f}, {self._fix_xy[1]:.2f})',
            throttle_duration_sec=2.0)
        # A goto that arrived before our first usable fix (spawn race) was
        # latched; now that we have one, plan it.
        if self._pending_goto is not None:
            m = self._pending_goto
            self._pending_goto = None
            self._goto_cb(m)

    # ------------------------------------------------------------------
    # Goto planning
    # ------------------------------------------------------------------

    def _goto_cb(self, msg: String):
        """Accept a /reposition/goto and commit to following its polyline.

        Payload (both forms accepted; normalised to a waypoint polyline)::

            {"waypoints": [{"lat":, "lon":}, ...],   # primary
             "end_heading_deg": <deg E-of-N>,        # optional arrival heading
             "v_const" | "speed": <m/s>,             # optional forward speed
             "pos_tol_m": <m>, "dry_run": <bool>}    # optional

            {"lat":, "lon":, "heading_deg": <deg>}   # legacy single pin

        The curve is pre-flighted against the working area + exclusions (R3) on
        top of the orchestrator's own check and the authoring-time check.
        """
        # [io-dbg] INPUT command (verbatim, low-rate): exactly what the
        # orchestrator sent on /reposition/goto.
        self.get_logger().info(f'[io-dbg] IN  goto <- {msg.data}')
        try:
            d = json.loads(msg.data)
        except (ValueError, TypeError) as exc:
            self._abort(f'bad /reposition/goto payload: {exc}')
            return
        if not isinstance(d, dict):
            self._abort('bad /reposition/goto payload: not a JSON object')
            return

        # Adopt the caller's goto id FIRST, so even an abort below (geo not
        # loaded, bad waypoints, R3 breach) is attributed to THIS goto in the
        # status stream, not silently to the previous one.
        self._goto_seq = d.get('seq', None)

        if self._geo is None or self._anchor is None or self._inset is None:
            self._abort('geo/venue not loaded; cannot plan')
            return

        # Spawn race: goto can arrive before our RTK subscription has delivered a
        # usable fix. Latch and replan from _fix_cb instead of hard-aborting (a
        # hard abort here trips the orchestrator's circuit breaker on leg 1).
        if self._fix_xy is None or self._rtk_quality not in _ALLOWED_RTK:
            self._pending_goto = msg
            self._state = 'waiting'
            self._reason = 'waiting for first RTK fix before planning (R1)'
            self.get_logger().warn(
                'goto arrived before first RTK fix; latching goal, will plan when '
                'a fix lands (not aborting).')
            self._publish_status(err_m=float('nan'), err_deg=float('nan'))
            return

        try:
            wps_ll, end_heading_deg, speed, pos_tol, dry_run = self._parse_goto(d)
        except (ValueError, TypeError, KeyError) as exc:
            self._abort(f'bad /reposition/goto payload: {exc}')
            return

        wps = [tuple(self._geo.latlon_to_local(ll, self._anchor)[0])
               for ll in wps_ll]
        wps = [(float(x), float(y)) for (x, y) in wps]

        # Arrival heading: explicit bearing, else the last-segment tangent, else
        # (single point, no heading) position-only.
        if end_heading_deg is not None:
            end_yaw = self._bearing_deg_to_local_yaw(end_heading_deg)
        elif len(wps) >= 2:
            a, b = wps[-2], wps[-1]
            end_yaw = _wrap(math.atan2(b[1] - a[1], b[0] - a[0]))
        else:
            end_yaw = None

        # A bare single pin WITH a heading has no tail to track; synthesise a
        # short straight final segment so the heading is reachable forward.
        if len(wps) == 1 and end_yaw is not None:
            pin = wps[0]
            entry = (pin[0] - self._tail_m * math.cos(end_yaw),
                     pin[1] - self._tail_m * math.sin(end_yaw))
            wps = [entry, pin]

        ok, reason = self._preflight(wps)
        if dry_run:
            self._state = 'idle'
            self._reason = (f'dry-run OK: {len(wps)}-pt plan fits' if ok
                            else f'dry-run FAIL: {reason} (R3)')
            self._zero_cmd()
            self._publish_status(err_m=float('nan'), err_deg=float('nan'))
            return
        if not ok:
            self._abort(f'path breaches working area: {reason} (R3)')
            return

        # R4: no-op if we already start within tolerance of the final pose.
        last = wps[-1]
        d_final = math.hypot(self._fix_xy[0] - last[0], self._fix_xy[1] - last[1])
        pos_tol_eff, head_tol_eff = self._active_tol(pos_tol)
        head_ok = (end_yaw is None or
                   (self._heading_est is not None and
                    abs(_wrap(self._heading_est - end_yaw)) <= head_tol_eff))
        if d_final <= pos_tol_eff and head_ok:
            # Adopt the mission fields even though we never drive: status err_m/
            # err_deg report against THIS goto's target (the orchestrator gates
            # the recipe on the arrival heading error; without this, a no-op
            # arrival reported err_deg=null and the gate could not act).
            self._waypoints = wps
            self._end_yaw = end_yaw
            self._state = 'arrived'
            self._reason = 'already within tolerance (R4 no-op)'
            self._zero_cmd()
            self.get_logger().info(
                f'goto: start pose already within tolerance (err {d_final:.3f} m); '
                'no-op (R4).')
            self._publish_status_current()
            return

        # Commit to following the curve.
        self._waypoints = wps
        self._seg_i = 0
        self._end_yaw = end_yaw
        self._speed = max(0.0, min(speed, self._max_speed))
        self._pos_tol = pos_tol
        self._acquired = False
        self._feasible_checked = False
        self._goto_start_dist = d_final
        self._state = 'driving'
        self._reason = ''
        _eh = ('position-only' if end_yaw is None
               else f'{math.degrees(end_yaw):.1f} deg (local)')
        self.get_logger().info(
            f'goto: {len(wps)} pts, end heading {_eh}, speed {self._speed:.2f} m/s, '
            f'start err {d_final:.2f} m -> driving.')

    def _parse_goto(self, d):
        """Normalise a goto dict to (waypoints_ll, end_heading_deg|None, speed,
        pos_tol_m, dry_run). Accepts the waypoint and legacy single-pin forms."""
        if 'waypoints' in d:
            wl = d['waypoints']
            if not isinstance(wl, list) or len(wl) < 1:
                raise ValueError('waypoints must be a non-empty list')
            wps_ll = [(float(w['lat']), float(w['lon'])) for w in wl]
            end_heading = d.get('end_heading_deg', None)
        else:
            wps_ll = [(float(d['lat']), float(d['lon']))]
            # legacy single pin: heading_deg is the arrival heading
            end_heading = d.get('end_heading_deg', d.get('heading_deg', None))
        end_heading = None if end_heading is None else float(end_heading)
        speed = float(d.get('v_const', d.get('speed', self._cruise)))
        pos_tol = float(d.get('pos_tol_m', self._pos_tol_default))
        dry_run = bool(d.get('dry_run', False))
        return wps_ll, end_heading, speed, pos_tol, dry_run

    def _preflight(self, wps):
        """Pre-flight a polyline against the inset area + inflated exclusions.

        Checks (local frame): every waypoint in-area and clear; and the
        current-pose->wp0 hop plus each wp[i]->wp[i+1] segment in-area and clear.
        Returns (ok, reason).
        """
        for i, p in enumerate(wps):
            if not point_in_polygon(p, self._inset):
                return False, f'waypoint {i} outside inset working area'
            for (cx, cy, r) in self._excl:
                if math.hypot(p[0] - cx, p[1] - cy) <= r:
                    return False, f'waypoint {i} inside exclusion'
        pts = [self._fix_xy] + list(wps)
        for i in range(len(pts) - 1):
            a, b = pts[i], pts[i + 1]
            if not seg_in_polygon(a, b, self._inset):
                return False, f'segment {i} leaves working area'
            if not seg_clears_circles(a, b, self._excl):
                return False, f'segment {i} enters exclusion'
        return True, ''

    # ------------------------------------------------------------------
    # Runtime safety
    # ------------------------------------------------------------------

    def _live_pose_safe(self):
        """Runtime guard (R3): the live RTK pose must stay in-area + clear."""
        if self._fix_xy is None:
            return True  # handled by the RTK/stale gate elsewhere
        if not point_in_polygon(self._fix_xy, self._inset):
            return False
        for (cx, cy, r) in self._excl:
            if math.hypot(self._fix_xy[0] - cx, self._fix_xy[1] - cy) <= r:
                return False
        return True

    def _active_tol(self, pos_tol_fixed=None):
        """Quality-adaptive (pos_tol, heading_tol). FIXED uses the mission/param
        position tol; FLOAT widens to at least the float floor + the float band."""
        base = self._pos_tol if pos_tol_fixed is None else pos_tol_fixed
        if self._rtk_quality == _RTK_FIXED:
            return base, self._head_tol_fixed
        return max(base, self._pos_tol_float), self._head_tol_float

    # ------------------------------------------------------------------
    # Pure-pursuit control loop
    # ------------------------------------------------------------------

    def _control_cb(self):
        if self._state != 'driving':
            # idle/waiting/arrived/aborted: silent on cmd_vel_raw (C6), keep
            # status fresh.
            self._publish_status_current()
            return

        # Heading staleness: drop a frozen value so steering HOLDS, not acts on it.
        if not self._fused_is_fresh():
            self._heading_est = None

        # RTK gate (R1): need a fresh accepted fix to keep moving.
        if self._rtk_quality not in _ALLOWED_RTK or self._fix_stamp is None or \
                self._fix_xy is None:
            self._zero_cmd()
            self._publish_status(err_m=float('nan'), err_deg=float('nan'),
                                 reason='HOLD: waiting for RTK fix')
            return
        age = (self.get_clock().now() - self._fix_stamp).nanoseconds * 1e-9
        if age > self._rtk_timeout:
            self._zero_cmd()
            self._publish_status(err_m=float('nan'), err_deg=float('nan'),
                                 reason=f'HOLD: RTK fix stale ({age:.1f}s)')
            return

        # Runtime area guard (R3).
        if not self._live_pose_safe():
            self._abort('live RTK pose left working area / entered exclusion (R3)')
            return

        last = self._waypoints[-1]
        d_final = math.hypot(self._fix_xy[0] - last[0], self._fix_xy[1] - last[1])
        pos_tol, head_tol = self._active_tol()
        head_err = (None if self._heading_est is None or self._end_yaw is None
                    else _wrap(self._heading_est - self._end_yaw))

        # Arrival: GATED ON POSITION (an Ackermann robot can't fix heading once
        # stopped at the point — heading is achieved by tracking the authored
        # tail and is reported, not blocked on). The reason notes heading vs band.
        if d_final <= pos_tol:
            self._state = 'arrived'
            self._zero_cmd()
            if self._end_yaw is None:
                self._reason = 'arrived (position only)'
            elif head_err is None:
                self._reason = 'arrived (position; heading unknown)'
            elif abs(head_err) <= head_tol:
                self._reason = 'arrived (position + heading)'
            else:
                self._reason = (
                    f'arrived (position; heading err {math.degrees(head_err):.1f} '
                    f'deg > {math.degrees(head_tol):.0f} — author a straighter/'
                    'longer curve tail)')
            self.get_logger().info(f'arrived: err {d_final:.3f} m. {self._reason}')
            self._publish_status(
                err_m=d_final,
                err_deg=(float('nan') if head_err is None
                         else math.degrees(head_err)))
            return

        # Need heading to steer; if stale/absent, HOLD rather than drive blind.
        if self._heading_est is None:
            self._zero_cmd()
            self._publish_status(err_m=d_final, err_deg=float('nan'),
                                 reason='HOLD: waiting for /heading/fused')
            return

        # Corridor guard: arm once the path is acquired, then abort if the robot
        # wanders off the authored curve (wrong-way drive / bad heading sign).
        xtrack = polyline_dist(self._fix_xy, self._waypoints)
        if xtrack <= self._acquire_radius:
            self._acquired = True
        if self._acquired and xtrack > self._corridor:
            self._abort(f'left path corridor: {xtrack:.1f} m off the authored '
                        f'curve (> {self._corridor:.1f} m)')
            return

        # Forward-drivability + path join (one-shot at the start of the mission):
        # pick the CLOSEST in-front feasible segment instead of always tracking
        # from segment 0 — a robot standing mid-path (operator placement, prior
        # partial drive) joins where it stands rather than aborting on a target
        # behind it (field regression 2026-06-10). Abort only if NO segment is
        # forward-reachable.
        if not self._feasible_checked:
            self._feasible_checked = True
            if not self._select_join_segment():
                self._abort(
                    'no forward-drivable join point on the path: every segment '
                    f'target is > {math.degrees(self._infeasible):.0f} deg off '
                    'the nose (Ackermann, no reverse) — re-place the robot '
                    'facing along the curve, or re-author it')
                return

        # Pure-pursuit steering target on the polyline.
        tx, ty = self._lookahead_target()
        bearing = math.atan2(ty - self._fix_xy[1], tx - self._fix_xy[0])
        alpha = _wrap(bearing - self._heading_est)

        # Forward-cone guard while driving: pure pursuit is singular at
        # |alpha| -> 180 deg (kappa = 2 sin(a)/L -> 0): a target behind the
        # nose commands STRAIGHT and the robot wanders off blind (field
        # failure 2026-06-11). Forward-only cannot recover from outside the
        # same cone the join logic enforces: stop and surface it.
        if abs(alpha) > self._infeasible:
            self._abort(f'look-ahead target {math.degrees(alpha):.0f} deg off '
                        'the nose while tracking (forward-only cannot reach '
                        'it) — overshot the path end or joined a backward '
                        'segment')
            return

        # Curvature to the look-ahead point, clamped to the physical R_min.
        kappa_raw = 2.0 * math.sin(alpha) / self._lookahead
        kappa = max(-self._kappa_max, min(self._kappa_max, kappa_raw))

        # Constant cruise, ramped down approaching the final point.
        v = self._speed
        if d_final < self._slowdown:
            v = max(self._min_speed, self._speed * d_final / self._slowdown)

        self._drive(v, v * kappa)
        # [io-dbg] CONTROL (throttled): the vector that produced this command —
        # seg/target/alpha/kappa explain WHY it steered where it did. *CLAMP means
        # R_min is binding (path demands a tighter turn than the chassis can make).
        self.get_logger().info(
            f'[io-dbg] CTL seg={self._seg_i} tgt=({tx:.2f},{ty:.2f}) '
            f'd_final={d_final:.2f} xtrack={xtrack:.2f} '
            f'alpha={math.degrees(alpha):.1f}deg '
            f'kappa={kappa:.2f}/{kappa_raw:.2f}'
            f'{"*CLAMP" if kappa != kappa_raw else ""} q={self._rtk_quality}',
            throttle_duration_sec=1.0)
        self._publish_status(
            err_m=d_final,
            err_deg=(float('nan') if head_err is None
                     else math.degrees(head_err)),
            reason='driving')

    def _select_join_segment(self, min_target_dist=0.3):
        """Boot-time path join (one-shot per mission). Choose ``_seg_i`` as the
        segment that is CLOSEST to the robot among those whose join target is
        in front and inside the feasibility cone (``infeasible_deg``). The join
        target for segment i is the first waypoint at least ``min_target_dist``
        ahead of the robot along the path from i — never a point at the robot's
        own position (bearing there is undefined noise).

        Returns True when a segment was selected (``_seg_i`` set); False when
        no segment is forward-reachable. ``_seg_i`` stays monotonic afterwards
        (``_lookahead_target`` only ever advances it).
        """
        wps = self._waypoints
        n = len(wps)
        rx, ry = self._fix_xy
        if n == 1:
            alpha = _wrap(math.atan2(wps[0][1] - ry, wps[0][0] - rx)
                          - self._heading_est)
            d0 = math.hypot(wps[0][0] - rx, wps[0][1] - ry)
            return d0 <= min_target_dist or abs(alpha) <= self._infeasible
        # ON the path (within acquire_radius): join where it stands — the
        # CLOSEST in-front feasible segment (field regression 2026-06-10;
        # driving back to the start would mean driving the path backward).
        # OFF the path: join the EARLIEST in-front feasible segment — the
        # authored curve delivers position AND heading (the tail is the
        # heading convergence), so a far robot drives TO the curve's start
        # and tracks all of it instead of cutting to whatever segment is
        # nearest (field failure 2026-06-11: parked before the start, the
        # node joined seg 30/30 5.5 m away and arrived heading-off).
        on_path = polyline_dist((rx, ry), wps) <= self._acquire_radius
        best = None                      # (cross-track dist, segment index)
        for i in range(n - 1):
            # Join target: first waypoint beyond i that is usefully ahead.
            tx, ty = None, None
            for j in range(i + 1, n):
                if math.hypot(wps[j][0] - rx, wps[j][1] - ry) >= min_target_dist:
                    tx, ty = wps[j]
                    break
            if tx is None:
                continue                 # only near-coincident points remain
            alpha = _wrap(math.atan2(ty - ry, tx - rx) - self._heading_est)
            if abs(alpha) > self._infeasible:
                continue
            d = point_seg_dist((rx, ry), wps[i], wps[i + 1])
            if on_path:
                if best is None or d < best[0]:
                    best = (d, i)
            else:
                best = (d, i)            # earliest feasible wins
                break
        if best is None:
            return False
        self._seg_i = best[1]
        if best[1] != 0:
            self.get_logger().info(
                f'joining path at segment {best[1]}/{n - 2} '
                f'({best[0]:.2f} m cross-track) — closest in-front feasible; '
                'the path prefix behind the robot is skipped, not driven.')
        return True

    def _lookahead_target(self):
        """Pure-pursuit look-ahead point on the polyline.

        Walk forward from the current segment; the first segment that the
        look-ahead circle crosses gives the target (farthest crossing). If none
        crosses (within a look-ahead of the end, or off-path), aim at the final
        point — driving straight in, or re-acquiring. self._seg_i only advances.
        """
        wps = self._waypoints
        n = len(wps)
        if n == 1:
            return wps[0]
        last = n - 1
        rx, ry = self._fix_xy
        # Endgame first: within a look-ahead of the final point, aim straight
        # at it. This must PRECEDE the crossing loop — near the end the far
        # crossing leaves the path and the loop degenerates to the crossing
        # BEHIND the robot, which only ever "worked" via the alpha~180
        # steering singularity the forward-cone guard now forbids.
        if math.hypot(wps[last][0] - rx, wps[last][1] - ry) <= self._lookahead:
            return wps[last]
        for i in range(self._seg_i, last):
            t = seg_circle_far_t(self._fix_xy, self._lookahead, wps[i], wps[i + 1])
            if t is not None:
                self._seg_i = i
                ax, ay = wps[i]
                bx, by = wps[i + 1]
                return (ax + (bx - ax) * t, ay + (by - ay) * t)
        # No crossing and not near the end: the robot is off-path. Re-acquire
        # by aiming at the nearest point on the CURRENT segment only — the
        # old fallback aimed at the curve END, so a robot 1.2 m off-path
        # beelined past the whole authored curve (field failure 2026-06-11).
        # Restricting to _seg_i (not the globally nearest remaining point)
        # preserves the join decision: a robot joined at seg 0 walks to the
        # curve START even when the tail happens to be nearer. Normal pursuit
        # resumes as soon as the look-ahead circle crosses the path again; a
        # target that ends up behind the nose trips the forward-cone guard.
        return point_seg_nearest((rx, ry), wps[self._seg_i],
                                 wps[min(self._seg_i + 1, last)])

    # ------------------------------------------------------------------
    # Actuation
    # ------------------------------------------------------------------

    def _drive(self, v, omega):
        """Publish a forward (v >= 0) Twist with yaw rate omega = v*kappa."""
        cmd = Twist()
        cmd.linear.x = max(0.0, float(v))
        cmd.angular.z = float(omega)
        self.pub_cmd.publish(cmd)
        # [io-dbg] OUTPUT actuation (throttled): command leaving on cmd_vel_raw.
        self.get_logger().info(
            f'[io-dbg] OUT cmd_vel_raw v={cmd.linear.x:.3f} w={cmd.angular.z:.3f}',
            throttle_duration_sec=1.0)

    def _zero_cmd(self):
        self.pub_cmd.publish(Twist())
        # [io-dbg] OUTPUT actuation (throttled): zero command (HOLD / idle / stop).
        self.get_logger().info(
            '[io-dbg] OUT cmd_vel_raw v=0.000 w=0.000 (zero)',
            throttle_duration_sec=2.0)

    def _abort(self, reason):
        # F1: stop in place and hold. Never self-recover.
        self._state = 'aborted'
        self._reason = reason
        self.get_logger().warn(f'reposition ABORT: {reason}')
        self._zero_cmd()
        self._publish_status(err_m=float('nan'), err_deg=float('nan'))

    # ------------------------------------------------------------------
    # Status
    # ------------------------------------------------------------------

    def _publish_status_current(self, reason=None):
        """Status with the current position/heading error to the final point."""
        if self._waypoints is not None and self._fix_xy is not None:
            last = self._waypoints[-1]
            err_m = math.hypot(self._fix_xy[0] - last[0], self._fix_xy[1] - last[1])
        else:
            err_m = float('nan')
        if self._end_yaw is not None and self._heading_est is not None:
            err_deg = math.degrees(_wrap(self._heading_est - self._end_yaw))
        else:
            err_deg = float('nan')
        self._publish_status(err_m=err_m, err_deg=err_deg, reason=reason)

    def _publish_status(self, err_m, err_deg, reason=None):
        payload = {
            'state': self._state,
            'err_m': None if err_m != err_m else round(float(err_m), 4),
            'err_deg': None if err_deg != err_deg else round(float(err_deg), 3),
            'reason': reason if reason is not None else self._reason,
            # Echo of the goto's 'seq' (None for manual/legacy gotos): lets the
            # orchestrator ignore status that narrates an older goto.
            'seq': self._goto_seq,
        }
        m = String()
        m.data = json.dumps(payload)
        self.pub_status.publish(m)
        # [io-dbg] OUTPUT status (throttled): the JSON the orchestrator reads.
        # State-transition edges (arrived/abort/commit) are logged unthrottled in
        # their own branches, so a throttled-out status here never hides an edge.
        self.get_logger().info(f'[io-dbg] OUT status -> {m.data}',
                               throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = RepositionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Always leave the actuation channel at zero on exit.
        try:
            node._zero_cmd()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
