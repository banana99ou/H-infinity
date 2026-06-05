# -*- coding: utf-8 -*-
"""RTK reposition node (ROC R1-R4, P3).

Autonomously drives the LIMO to a stored start/end pin using **GPS-RTK
feedback only**, arriving on a deterministic heading. This runs *between*
recorded runs to place the robot at a pin; it NEVER runs during a recorded
control run (ADR-01: RTK stays out of the follower control loop). The
sequencer (T6) enforces that the follower and this node never publish
``cmd_vel_raw`` at the same time (C6); this node simply stays silent on
``cmd_vel_raw`` unless it is actively repositioning.

Topology (see system_spec §3.4 + the T4 shared interface contract):

  sub  /gps_rtk_f9p_helical/gps/fix         sensor_msgs/NavSatFix
  sub  /gps_rtk_f9p_helical/gps/rtk_status  std_msgs/String  (carries quality=N)
  sub  /reposition/goto                     std_msgs/String  JSON
                                            {lat, lon, heading_deg}
  pub  /reposition/status                   std_msgs/String  JSON
                                            {state, err_m, err_deg, reason}
  pub  cmd_vel_raw                          geometry_msgs/Twist
                                            (ONLY while actively repositioning;
                                             routed through estop_cli.py -> /cmd_vel)

state in {'idle','aligning','approaching','arrived','aborted'}.

Why RTK FIXED only (R1)
-----------------------
quality=4 (RTK FIXED) is the only authoritative solution on this F9P (NMEA GGA
standard + the driver's fix_quality_to_desc(), GPS-RTK_ROS2_pub_node.py). FLOAT
(quality=5) is ~dm-level and not trusted for closed-loop placement here. If the
fix is not FIXED, we hold zero output and report the reason. The rtk_status
String is the authority for quality; NavSatFix.status cannot distinguish RTK on
this receiver.

Heading without a compass (course-over-ground, R1/R2)
-----------------------------------------------------
There is no IMU/compass in this loop. The robot's heading is inferred from
**course-over-ground**: the bearing of the displacement between successive RTK
fixes while creeping forward. Below a minimum displacement the COG is pure
noise, so we gate on a configurable travel threshold before trusting it. This
is exactly why every motion phase creeps *forward* (or *backward*, for the
3-point reverse leg): we must move to observe heading.

Go-to-pose geometry
--------------------
All work is done in the shared local frame defined by the rooftop anchor
(tools/path_gen/path_overlay.py): +x along the long axis (bearing 42 deg E of
N), +y 90 deg CCW. latlon_to_local() / local_to_latlon() convert pins and
live fixes into this metric frame; the venue working area + exclusions are
likewise projected once. Target heading_deg is a compass bearing (E of N); it
maps to a local-frame heading angle of (anchor.bearing_deg - heading_deg) in
radians... see _bearing_deg_to_local_yaw().

Phases:
  1. aligning     -- creep forward, estimate COG, steer to face the approach
                     entry point. If the required turn is too tight for the
                     working area, execute a 3-point (reverse) maneuver (P3).
  2. approaching  -- drive toward the approach-entry point (target offset back
                     along the target heading by approach_dist_m).
  3. (final)      -- past the entry point, drive the straight final segment
                     along the target heading into the pin (R2, deterministic
                     heading). Still reported as 'approaching' until arrival.
  4. arrived      -- within pos + heading tolerance (R4 also short-circuits to
                     here immediately if we already start within tolerance).
  aborted         -- area/exclusion breach (R3) or unrecoverable condition.

Area-aware abort (R3)
---------------------
Before committing to a plan we check that the target, the approach-entry point,
and the straight final segment all lie inside the inset working area
(venue corners shrunk inward by safety_margin_m) and clear of every exclusion
circle (also inflated by safety_margin_m). At runtime, if the live RTK pose
ever leaves the inset area or enters an exclusion, we abort and zero output.
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

# Reuse the shared geo anchor + projections (latlon_to_local is the inverse we
# added alongside local_to_latlon). The reposition node lives in the colcon
# tree; tools/ is not on the ament path, so we import by file location.
import importlib.util as _ilu

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
# Resolve the H-infinity repo root. path_overlay.py (tools/, battle station) and
# scenarios/ are NOT installed into the colcon tree, so the __file__-relative
# path only works in-source; once colcon copies this node into install/, it
# breaks. Try: env override, the known NUC repo root (the repo hardcodes
# /home/agilex/H-infinity elsewhere too, e.g. estop in orchestrator PROCS), then
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


def _load_path_overlay():
    """Import tools/path_gen/path_overlay.py by file location (not a colcon
    package, so not on the ament path / not installed). Returns the module or
    raises ImportError. The node degrades gracefully: if the geo helpers are
    unavailable it refuses to plan (reports an abort reason) rather than
    fabricate coordinates.
    """
    if not os.path.isfile(_PATH_OVERLAY):
        raise ImportError(f'path_overlay.py not found at {_PATH_OVERLAY} '
                          f'(tried roots: {", ".join(_ROOT_CANDIDATES)})')
    spec = _ilu.spec_from_file_location('h_path_overlay', _PATH_OVERLAY)
    if spec is None or spec.loader is None:
        raise ImportError(f'cannot load path_overlay from {_PATH_OVERLAY}')
    mod = _ilu.module_from_spec(spec)
    # Register before exec: path_overlay defines an @dataclass (Anchor), and
    # dataclasses resolves the owning module via sys.modules during class
    # construction. Without this, exec_module raises AttributeError.
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def _wrap(angle):
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


# ----------------------------------------------------------------------------
# Pure-geometry helpers (no ROS) — kept module-level so they are unit-testable
# off the robot.
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

    A centroid push-in is exact for a rectangle (the rooftop case) and a safe
    conservative approximation for mild quadrilaterals: every inset vertex
    moves strictly inward, so the inset region is contained in the original.
    corners: list of (x, y). Returns a new list of (x, y).
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

    circles already include any safety inflation. Sampled containment check
    (cheap, conservative for the short reposition legs here).
    """
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


class RepositionNode(Node):

    def __init__(self):
        super().__init__('reposition_node')

        # -- Parameters --------------------------------------------------
        # Tolerances. TODO(hw-tune): all three need a wheels-on-floor session
        # (Part C checklist item 4). Defaults are first-guess starting points.
        self.declare_parameter('pos_tol_m', 0.15)          # arrival radius
        self.declare_parameter('heading_tol_deg', 5.0)     # arrival heading band
        self.declare_parameter('creep_speed', 0.15)        # m/s forward creep
        # Straight final-approach length along the target heading (R2). Long
        # enough to settle heading via COG; short enough to fit the area.
        self.declare_parameter('approach_dist_m', 0.6)
        # Steering: yaw-rate command = k_yaw * heading_error, clipped.
        self.declare_parameter('k_yaw', 1.2)               # rad/s per rad
        self.declare_parameter('max_yaw_rate', 0.8)        # rad/s clip
        # COG noise gate: minimum travel between the two fixes used to estimate
        # course-over-ground. Below this the bearing is dominated by RTK noise.
        # TODO(hw-tune): set from observed RTK position jitter at creep speed.
        self.declare_parameter('cog_min_travel_m', 0.10)
        # Heading error (rad) above which a forward creep-turn would bow too far
        # out of the area; trigger the 3-point reverse maneuver instead (P3).
        self.declare_parameter('three_point_turn_deg', 100.0)
        # Reverse-leg duration cap for one 3-point backup (s).
        self.declare_parameter('reverse_time_s', 1.5)
        # RTK fix staleness timeout (s): no fresh FIXED fix -> hold + zero.
        self.declare_parameter('rtk_timeout_s', 1.0)
        # Venue file. Loaded once for the working-area + exclusion geometry.
        self.declare_parameter(
            'venue_file',
            os.path.join(_REPO_ROOT, 'scenarios', 'venues', 'rooftop.json'))
        self.declare_parameter('control_rate_hz', 20.0)
        # When True, reposition arrives on POSITION ALONE and ignores final
        # heading (steers straight at the pin, skips the R2 hold + P3 reverse).
        # This was a workaround for the COG-only heading limit-cycle; now that the
        # Pixhawk compass gives a standstill-capable heading (see below), the
        # default is heading-converging arrival (False, spec R2). Kept as an
        # explicit fallback. Per-goto override: {"pos_only": true|false} in /reposition/goto.
        self.declare_parameter('arrive_on_position_only', False)
        # Heading source: the Pixhawk magnetometer (compass_hdg, deg) is
        # standstill-capable, unlike COG (noise-dominated at slow near-pin moves
        # -> the heading limit-cycle). The Pixhawk is mounted ~90deg rotated (sign
        # unknown); the total offset (mount + declination + frame convention) is
        # auto-calibrated against forward-motion COG into compass_corr_rad. Set
        # use_compass_heading False for COG-only; a non-NaN compass_offset_rad
        # skips auto-cal and uses that correction directly.
        self.declare_parameter('use_compass_heading', True)
        self.declare_parameter('compass_topic', '/pixhawk/global_position/compass_hdg')
        self.declare_parameter('compass_offset_rad', float('nan'))

        self._pos_tol = float(self.get_parameter('pos_tol_m').value)
        self._heading_tol = math.radians(
            float(self.get_parameter('heading_tol_deg').value))
        self._creep = float(self.get_parameter('creep_speed').value)
        self._approach_dist = float(self.get_parameter('approach_dist_m').value)
        self._k_yaw = float(self.get_parameter('k_yaw').value)
        self._max_yaw = float(self.get_parameter('max_yaw_rate').value)
        self._cog_min_travel = float(
            self.get_parameter('cog_min_travel_m').value)
        self._three_point_turn = math.radians(
            float(self.get_parameter('three_point_turn_deg').value))
        self._reverse_time = float(self.get_parameter('reverse_time_s').value)
        self._rtk_timeout = float(self.get_parameter('rtk_timeout_s').value)
        self._pos_only_default = bool(
            self.get_parameter('arrive_on_position_only').value)
        self._use_compass = bool(self.get_parameter('use_compass_heading').value)
        self._compass_topic = str(self.get_parameter('compass_topic').value)
        self._compass_corr_rad = float(
            self.get_parameter('compass_offset_rad').value)  # NaN until calibrated
        self._venue_file = str(self.get_parameter('venue_file').value)
        rate = float(self.get_parameter('control_rate_hz').value)

        # -- Geo helpers + venue geometry --------------------------------
        self._geo = None
        self._anchor = None
        self._inset = None        # working-area inset polygon, local (x,y)
        self._excl = []           # inflated exclusion circles (cx, cy, r) local
        try:
            self._geo = _load_path_overlay()
            self._anchor = self._geo.Anchor(
                self._geo.LAT0, self._geo.LON0, self._geo.BEARING_DEG)
            self._load_venue(self._venue_file)
        except Exception as exc:  # noqa: BLE001 — node must not crash on load
            self.get_logger().error(
                f'Geo/venue init failed ({exc}); reposition will refuse to '
                'plan until fixed. cmd_vel_raw stays silent.')

        # -- RTK state ---------------------------------------------------
        self._rtk_quality = None     # latest parsed quality=N, or None
        self._fix_xy = None          # latest FIXED fix in local (x, y)
        self._fix_stamp = None       # ros time of latest FIXED fix
        # COG estimation anchor: the (x, y, ros_time) of the fix from which we
        # are currently accumulating travel to estimate course-over-ground.
        self._cog_ref_xy = None
        self._heading_est = None     # last trusted heading (rad, local): compass or COG
        self._compass_hdg_deg = None  # latest raw Pixhawk compass heading (deg)
        self._compass_cal_acc = []    # forward-motion (cog - compass) samples for auto-cal

        # -- Mission state ----------------------------------------------
        self._state = 'idle'
        self._reason = ''
        self._target_xy = None       # pin in local (x, y)
        self._target_yaw = None      # pin heading in local frame (rad)
        self._entry_xy = None        # approach-entry point in local (x, y)
        # 3-point turn sub-state: None | 'reverse'; with a deadline.
        self._turn_phase = None
        self._reverse_deadline = None
        # Per-mission position-only flag (set from the goto payload / param).
        self._pos_only = self._pos_only_default

        # -- ROS interfaces ----------------------------------------------
        self.sub_fix = self.create_subscription(
            NavSatFix, '/gps_rtk_f9p_helical/gps/fix', self._fix_cb, 10)
        self.sub_rtk = self.create_subscription(
            String, '/gps_rtk_f9p_helical/gps/rtk_status', self._rtk_cb, 10)
        self.sub_goto = self.create_subscription(
            String, '/reposition/goto', self._goto_cb, 10)
        self.sub_compass = self.create_subscription(
            Float64, self._compass_topic, self._compass_cb, 10)

        self.pub_cmd = self.create_publisher(Twist, 'cmd_vel_raw', 10)
        status_qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub_status = self.create_publisher(
            String, '/reposition/status', status_qos)

        self._timer = self.create_timer(1.0 / rate, self._control_cb)

        self._publish_status(err_m=float('nan'), err_deg=float('nan'))
        self.get_logger().info(
            'reposition_node started (idle). Waiting for /reposition/goto. '
            'cmd_vel_raw stays silent until commanded; RTK FIXED required.')

    # ------------------------------------------------------------------
    # Venue loading
    # ------------------------------------------------------------------

    def _load_venue(self, path):
        with open(path, 'r') as f:
            v = json.load(f)
        margin = float(v.get('safety_margin_m', 0.0))

        corners_ll = [(c['lat'], c['lon']) for c in v['corners_wgs84']]
        corners_xy = [
            tuple(self._geo.latlon_to_local(ll, self._anchor)[0])
            for ll in corners_ll
        ]
        self._inset = inset_polygon(corners_xy, margin)

        self._excl = []
        for ex in v.get('exclusions', []):
            if ex.get('kind') != 'circle':
                self.get_logger().warn(
                    f"venue exclusion kind '{ex.get('kind')}' unsupported; "
                    'skipping.')
                continue
            cxy = self._geo.latlon_to_local((ex['lat'], ex['lon']),
                                            self._anchor)[0]
            # Inflate the radius by the safety margin: keep the robot a margin
            # clear of the island, not just outside its painted edge.
            r = float(ex['radius_m']) + margin
            self._excl.append((float(cxy[0]), float(cxy[1]), r))

        self.get_logger().info(
            f"venue '{v.get('name')}' loaded: inset polygon {len(self._inset)} "
            f'verts, {len(self._excl)} exclusion(s), margin {margin:.2f} m.')

    # ------------------------------------------------------------------
    # Geo conversions
    # ------------------------------------------------------------------

    def _bearing_deg_to_local_yaw(self, heading_deg):
        """Compass bearing (deg E of N) -> heading angle in the local frame.

        Local +x has compass bearing anchor.bearing_deg (E of N) and the local
        yaw is measured CCW from +x. A compass bearing increases clockwise from
        north, while the local yaw increases counter-clockwise from +x, so::

            local_yaw = radians(anchor.bearing_deg - heading_deg)
        """
        return _wrap(math.radians(self._anchor.bearing_deg - heading_deg))

    # ------------------------------------------------------------------
    # Subscriptions
    # ------------------------------------------------------------------

    def _compass_cb(self, msg: Float64):
        """Pixhawk magnetometer heading (deg). When the compass is calibrated this
        drives self._heading_est continuously — including at standstill, which is
        what lets reposition converge heading instead of COG-limit-cycling."""
        self._compass_hdg_deg = float(msg.data)
        if self._anchor is None:
            return
        if self._use_compass and self._compass_corr_rad == self._compass_corr_rad:
            self._heading_est = _wrap(
                self._bearing_deg_to_local_yaw(self._compass_hdg_deg)
                + self._compass_corr_rad)

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
        self._rtk_quality = q

    def _fix_cb(self, msg: NavSatFix):
        """Record the latest fix in local (x, y) — only when RTK is FIXED.

        We gate the fix on quality=4 here so stale/float fixes never enter the
        control geometry. The lat/lon NaN guard skips the driver's pre-fix
        placeholder messages.
        """
        if self._geo is None or self._anchor is None:
            return
        if self._rtk_quality not in (4, 5):
            # Accept RTK FIXED(4) or FLOAT(5). TEMPORARY field relaxation
            # (2026-06-05): the base can't hold FIXED here; FLOAT is ~dm-level,
            # coarser than the cm-level FIXED this loop was designed for (R1).
            # Revert to {4} only once FIXED is reliable. See ToDo 2026-06-05.
            return
        lat = msg.latitude
        lon = msg.longitude
        if lat != lat or lon != lon:  # NaN check
            return
        xy = self._geo.latlon_to_local((lat, lon), self._anchor)[0]
        self._fix_xy = (float(xy[0]), float(xy[1]))
        self._fix_stamp = self.get_clock().now()
        self._update_cog()

    def _update_cog(self):
        """Estimate course-over-ground from accumulated FIXED-fix travel.

        We hold a reference fix and, once the robot has moved at least
        cog_min_travel_m from it, take the bearing of the displacement as the
        heading and re-anchor the reference. Direction of travel (forward vs
        reverse creep) is handled by the caller flipping the result by pi when
        we are commanding reverse.
        """
        if self._cog_ref_xy is None:
            self._cog_ref_xy = self._fix_xy
            return
        dx = self._fix_xy[0] - self._cog_ref_xy[0]
        dy = self._fix_xy[1] - self._cog_ref_xy[1]
        if math.hypot(dx, dy) >= self._cog_min_travel:
            cog_heading = math.atan2(dy, dx)
            self._cog_ref_xy = self._fix_xy
            # COG is the direction of travel — a valid heading only when moving
            # FORWARD (not during a 3-point reverse). Use it to auto-calibrate the
            # compass; once calibrated the compass owns heading_est (standstill-OK).
            fwd = (self._turn_phase != 'reverse')
            calibrated = (self._compass_corr_rad == self._compass_corr_rad)  # not NaN
            if (self._use_compass and fwd and self._compass_hdg_deg is not None
                    and not calibrated):
                corr = _wrap(cog_heading
                             - self._bearing_deg_to_local_yaw(self._compass_hdg_deg))
                self._compass_cal_acc.append(corr)
                if len(self._compass_cal_acc) >= 5:
                    sx = sum(math.sin(c) for c in self._compass_cal_acc)
                    sy = sum(math.cos(c) for c in self._compass_cal_acc)
                    self._compass_corr_rad = math.atan2(sx, sy)
                    calibrated = True
                    self.get_logger().warn(
                        "compass auto-calibrated: corr="
                        f"{math.degrees(self._compass_corr_rad):.1f} deg "
                        f"(persist with compass_offset_rad:={self._compass_corr_rad:.4f}).")
            if self._use_compass and calibrated:
                # Cross-check only; heading_est still trusts the compass.
                if fwd and self._heading_est is not None:
                    d = abs(_wrap(cog_heading - self._heading_est))
                    if d > math.radians(30):
                        self.get_logger().warn(
                            f"compass vs COG disagree {math.degrees(d):.0f} deg "
                            "(magnetic interference?).")
            else:
                # Fallback: COG drives heading_est until the compass is calibrated.
                self._heading_est = cog_heading

    def _goto_cb(self, msg: String):
        """Accept a new go-to-pose command and plan it (R1-R4)."""
        try:
            d = json.loads(msg.data)
            lat = float(d['lat'])
            lon = float(d['lon'])
            heading_deg = float(d['heading_deg'])
            dry_run = bool(d.get('dry_run', False))
            approach_dist = float(d.get('approach_dist_m', self._approach_dist))
            pos_only = bool(d.get('pos_only', self._pos_only_default))
        except (ValueError, TypeError, KeyError) as exc:
            self._abort(f'bad /reposition/goto payload: {exc}')
            return
        self._pos_only = pos_only

        if self._geo is None or self._anchor is None or self._inset is None:
            self._abort('geo/venue not loaded; cannot plan')
            return
        if self._fix_xy is None or self._rtk_quality not in (4, 5):
            self._abort('no usable RTK fix yet (need FIXED/FLOAT); refusing to start (R1)')
            return

        self._approach_dist = approach_dist
        xy = self._geo.latlon_to_local((lat, lon), self._anchor)[0]
        self._target_xy = (float(xy[0]), float(xy[1]))
        self._target_yaw = self._bearing_deg_to_local_yaw(heading_deg)
        # Approach-entry point: offset back along the target heading so the
        # final segment runs straight into the pin along that heading (R2).
        # In position-only mode heading is ignored, so collapse the entry onto
        # the target (no final-straight heading segment; we drive to the point).
        if self._pos_only:
            self._entry_xy = self._target_xy
        else:
            self._entry_xy = (
                self._target_xy[0] - approach_dist * math.cos(self._target_yaw),
                self._target_xy[1] - approach_dist * math.sin(self._target_yaw),
            )

        # R4: no-op if we already start within tolerance of the target pose.
        err_m = math.hypot(self._fix_xy[0] - self._target_xy[0],
                           self._fix_xy[1] - self._target_xy[1])
        # Heading error only meaningful if we have a COG estimate; if we don't
        # yet, position alone decides the no-op (a stationary robot at the pin
        # with unknown heading still needs a creep to confirm — so require a
        # heading estimate to claim arrived).
        _head_ok = (self._heading_est is not None and
                    abs(_wrap(self._heading_est - self._target_yaw))
                    <= self._heading_tol)
        if err_m <= self._pos_tol and (self._pos_only or _head_ok):
            self._state = 'arrived'
            self._reason = 'already within tolerance (R4 no-op)'
            self.get_logger().info(
                f'goto: start pose already within tolerance '
                f'(err {err_m:.3f} m); no-op (R4).')
            self._zero_cmd()
            _ed = (float('nan') if self._heading_est is None
                   else math.degrees(_wrap(self._heading_est - self._target_yaw)))
            self._publish_status(err_m=err_m, err_deg=_ed)
            return

        # R3: pre-flight the whole plan against the working area + exclusions.
        ok, reason = self._plan_clear()
        if not ok:
            if dry_run:
                self._state = 'idle'
                self._reason = f'dry-run FAIL: {reason} (R3)'
                self._zero_cmd()
                self._publish_status(err_m=err_m, err_deg=float('nan'))
                return
            self._abort(f'planned path breaches working area: {reason} (R3)')
            return

        if dry_run:
            self._state = 'idle'
            self._reason = (
                f'dry-run OK: plan fits, approach_dist_m={approach_dist:.2f}')
            self._zero_cmd()
            self._publish_status(err_m=err_m, err_deg=float('nan'))
            return

        self._state = 'aligning'
        self._reason = ''
        self._turn_phase = None
        self._reverse_deadline = None
        self._cog_ref_xy = self._fix_xy  # restart COG accumulation
        self.get_logger().info(
            f'goto: target local ({self._target_xy[0]:.2f}, '
            f'{self._target_xy[1]:.2f}) yaw {math.degrees(self._target_yaw):.1f} '
            f'deg; entry ({self._entry_xy[0]:.2f}, {self._entry_xy[1]:.2f}); '
            f'start err {err_m:.2f} m. -> aligning.')

    # ------------------------------------------------------------------
    # Planning / area checks (R3)
    # ------------------------------------------------------------------

    def _plan_clear(self):
        """Validate target, entry, and segments against area + exclusions.

        Returns (ok, reason). Checks (all in the local frame):
          - target and entry are inside the inset working area;
          - the current-pose -> entry leg and entry -> target final segment
            both stay inside the inset area and clear of every exclusion.
        """
        for label, p in (('target', self._target_xy), ('entry', self._entry_xy)):
            if not point_in_polygon(p, self._inset):
                return False, f'{label} outside inset working area'
            for (cx, cy, r) in self._excl:
                if math.hypot(p[0] - cx, p[1] - cy) <= r:
                    return False, f'{label} inside exclusion'

        legs = (
            ('approach', self._fix_xy, self._entry_xy),
            ('final', self._entry_xy, self._target_xy),
        )
        for label, a, b in legs:
            if not seg_in_polygon(a, b, self._inset):
                return False, f'{label} segment leaves working area'
            if not seg_clears_circles(a, b, self._excl):
                return False, f'{label} segment enters exclusion'
        return True, ''

    def _live_pose_safe(self):
        """Runtime guard (R3): the live RTK pose must stay in-area + clear."""
        if self._fix_xy is None:
            return True  # handled by the FIXED/stale gate elsewhere
        if not point_in_polygon(self._fix_xy, self._inset):
            return False
        for (cx, cy, r) in self._excl:
            if math.hypot(self._fix_xy[0] - cx, self._fix_xy[1] - cy) <= r:
                return False
        return True

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------

    def _control_cb(self):
        if self._state in ('idle', 'arrived', 'aborted'):
            # Silent on cmd_vel_raw when not actively repositioning so we never
            # contend with teleop/follower (C6). Keep status fresh.
            self._publish_status_current_err()
            return

        # RTK gate (R1): require a fresh FIXED fix to keep moving.
        if self._rtk_quality not in (4, 5) or self._fix_stamp is None or \
                self._fix_xy is None:
            self._zero_cmd()
            self._publish_status(err_m=float('nan'), err_deg=float('nan'),
                                 reason='waiting for RTK FIXED')
            return
        age = (self.get_clock().now() - self._fix_stamp).nanoseconds * 1e-9
        if age > self._rtk_timeout:
            self._zero_cmd()
            self._publish_status(err_m=float('nan'), err_deg=float('nan'),
                                 reason=f'RTK fix stale ({age:.1f}s)')
            return

        # Runtime area guard (R3).
        if not self._live_pose_safe():
            self._abort('live RTK pose left working area / entered exclusion (R3)')
            self._zero_cmd()
            return

        # --- 3-point reverse sub-phase (P3) -----------------------------
        if self._turn_phase == 'reverse':
            if self.get_clock().now() >= self._reverse_deadline:
                self._turn_phase = None
                self._cog_ref_xy = self._fix_xy  # re-estimate heading forward
            else:
                self._drive(reverse=True, yaw_rate=self._reverse_yaw)
                self._publish_status_current_err(reason='3-point reverse (P3)')
                return

        # Distance/heading errors to the relevant waypoint.
        # Approach the entry point until we are within approach_dist of target,
        # then steer straight into the pin along the target heading (final).
        d_to_target = math.hypot(self._fix_xy[0] - self._target_xy[0],
                                 self._fix_xy[1] - self._target_xy[1])
        in_final = d_to_target <= self._approach_dist

        # Arrival check (R4 condition, evaluated continuously). In position-only
        # mode we arrive on distance alone (heading is acquired by the path
        # lead-in); otherwise both position and COG heading must be in tolerance.
        head_err = (None if self._heading_est is None
                    else _wrap(self._heading_est - self._target_yaw))
        _arrived = d_to_target <= self._pos_tol and (
            self._pos_only or (head_err is not None
                               and abs(head_err) <= self._heading_tol))
        if _arrived:
            self._state = 'arrived'
            self._reason = ('arrived (position only)' if self._pos_only
                            else 'arrived within tolerance')
            self._zero_cmd()
            _hetxt = ('' if head_err is None
                      else f', heading err {math.degrees(head_err):.1f} deg')
            self.get_logger().info(f'arrived: err {d_to_target:.3f} m{_hetxt}.')
            self._publish_status(
                err_m=d_to_target,
                err_deg=(float('nan') if head_err is None
                         else math.degrees(head_err)))
            return

        # Pick the steering target: the entry point while approaching, the pin
        # itself during the final straight segment.
        steer_to = self._target_xy if in_final else self._entry_xy
        bearing_to = math.atan2(steer_to[1] - self._fix_xy[1],
                                steer_to[0] - self._fix_xy[0])

        # During the final segment we hold the *target heading* (R2) rather
        # than chase the point, so the approach is a clean straight line. In
        # position-only mode we always chase the point (no heading hold).
        desired_heading = (bearing_to if self._pos_only
                           else (self._target_yaw if in_final else bearing_to))

        # Need a heading estimate (COG) to steer. If we don't have one yet,
        # creep straight forward to generate one.
        if self._heading_est is None:
            self._state = 'aligning'
            self._drive(reverse=False, yaw_rate=0.0)
            self._publish_status(err_m=d_to_target, err_deg=float('nan'),
                                 reason='creeping to acquire COG heading')
            return

        yaw_err = _wrap(desired_heading - self._heading_est)

        # 3-point decision (P3): if the required turn is too sharp to creep
        # through within the area, back up first. Only triggers while aligning
        # (large initial mis-heading), not during the final straight segment,
        # and never in position-only mode (which must not reverse off the pin).
        if not in_final and not self._pos_only and \
                abs(yaw_err) > self._three_point_turn and \
                self._turn_phase is None:
            self._turn_phase = 'reverse'
            self._reverse_deadline = self.get_clock().now() + \
                rclpy.duration.Duration(seconds=self._reverse_time)
            # Reverse while steering the rear toward the inside of the turn:
            # steering sign flips in reverse, so command yaw toward the turn.
            self._reverse_yaw = self._clip_yaw(
                self._k_yaw * (-yaw_err))
            self.get_logger().info(
                f'3-point turn (P3): yaw err {math.degrees(yaw_err):.0f} deg '
                f'> {math.degrees(self._three_point_turn):.0f}; reversing.')
            self._drive(reverse=True, yaw_rate=self._reverse_yaw)
            self._publish_status_current_err(reason='3-point reverse (P3)')
            return

        self._state = 'aligning' if abs(yaw_err) > self._heading_tol and \
            not in_final else 'approaching'
        self._drive(reverse=False, yaw_rate=self._clip_yaw(self._k_yaw * yaw_err))
        self._publish_status(
            err_m=d_to_target, err_deg=math.degrees(yaw_err),
            reason='final straight segment (R2)' if in_final else None)

    # ------------------------------------------------------------------
    # Actuation
    # ------------------------------------------------------------------

    def _clip_yaw(self, w):
        return max(-self._max_yaw, min(self._max_yaw, w))

    def _drive(self, reverse, yaw_rate):
        cmd = Twist()
        cmd.linear.x = -self._creep if reverse else self._creep
        cmd.angular.z = float(yaw_rate)
        self.pub_cmd.publish(cmd)

    def _zero_cmd(self):
        self.pub_cmd.publish(Twist())

    def _abort(self, reason):
        self._state = 'aborted'
        self._reason = reason
        self.get_logger().warn(f'reposition ABORT: {reason}')
        self._zero_cmd()
        self._publish_status(err_m=float('nan'), err_deg=float('nan'))

    # ------------------------------------------------------------------
    # Status
    # ------------------------------------------------------------------

    def _publish_status_current_err(self, reason=None):
        if self._target_xy is not None and self._fix_xy is not None:
            err_m = math.hypot(self._fix_xy[0] - self._target_xy[0],
                               self._fix_xy[1] - self._target_xy[1])
        else:
            err_m = float('nan')
        if self._target_yaw is not None and self._heading_est is not None:
            err_deg = math.degrees(_wrap(self._heading_est - self._target_yaw))
        else:
            err_deg = float('nan')
        self._publish_status(err_m=err_m, err_deg=err_deg, reason=reason)

    def _publish_status(self, err_m, err_deg, reason=None):
        payload = {
            'state': self._state,
            'err_m': None if err_m != err_m else round(float(err_m), 4),
            'err_deg': None if err_deg != err_deg else round(float(err_deg), 3),
            'reason': reason if reason is not None else self._reason,
            'approach_dist_m': self._approach_dist,
        }
        m = String()
        m.data = json.dumps(payload)
        self.pub_status.publish(m)


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
