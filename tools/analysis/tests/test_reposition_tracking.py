#!/usr/bin/env python3
"""Unit tests for reposition_node pure-pursuit tracking (look-ahead target
selection + forward-cone guard).

Same AST-extraction harness as test_reposition_join.py (rclpy does not exist
off-robot).

Scenario under test is the 2026-06-11 field regression: a robot starting
1.2 m off the authored glue curve was aimed at the curve's FINAL waypoint
(_lookahead_target's no-crossing fallback), beelined past the whole curve,
then near the end picked up a backward target where pure pursuit is singular
(kappa = 2 sin(alpha)/L -> 0 at |alpha| ~ 180 deg) and drove straight off the
venue with near-zero steering until the R3 abort.

Run:  python3 tools/analysis/tests/test_reposition_tracking.py   (or pytest)
"""
import ast
import math
import os
import sys

SRC = os.path.join(
    os.path.dirname(__file__), '..', '..', '..',
    'scalecar-vfg-h-infinite', 'ros2_bridge', 'limo_path_follower',
    'reposition_node.py')

_FUNCS = {'_wrap', 'point_seg_dist', 'point_seg_nearest', 'polyline_dist',
          'seg_circle_far_t'}
_METHODS = {'_lookahead_target', '_control_cb', '_active_tol',
            '_select_join_segment', '_eff_lookahead'}
_CONSTS = {'_RTK_FIXED', '_ALLOWED_RTK'}


def _extract():
    with open(SRC) as f:
        tree = ast.parse(f.read())
    ns = {'math': math}
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name in _FUNCS:
            exec(compile(ast.Module([node], []), SRC, 'exec'), ns)
        elif isinstance(node, ast.Assign):
            for tgt in node.targets:
                if isinstance(tgt, ast.Name) and tgt.id in _CONSTS:
                    exec(compile(ast.Module([node], []), SRC, 'exec'), ns)
    for node in tree.body:
        if isinstance(node, ast.ClassDef) and node.name == 'RepositionNode':
            for sub in node.body:
                if isinstance(sub, ast.FunctionDef) and sub.name in _METHODS:
                    exec(compile(ast.Module([sub], []), SRC, 'exec'), ns)
    missing = (_FUNCS | _METHODS | _CONSTS) - set(ns)
    assert not missing, f'extraction missed: {missing}'
    return ns


NS = _extract()


class _Log:
    def info(self, *_a, **_k): pass
    def warn(self, *_a, **_k): pass
    def error(self, *_a, **_k): pass


class _Stamp:
    """now() - stamp -> .nanoseconds (always fresh)."""
    def __sub__(self, _other): return self
    nanoseconds = 0


class _Clock:
    def now(self): return _Stamp()


class Robot:
    """Stand-in providing everything _control_cb touches. State: mid-mission
    (joined, acquired), FIXED RTK, fresh heading — straight to the steering
    branch."""

    def __init__(self, waypoints, xy, heading_deg,
                 lookahead=0.60, infeasible_deg=100.0):
        self._waypoints = waypoints
        self._fix_xy = xy
        self._heading_est = math.radians(heading_deg)
        self._lookahead = lookahead
        self._la_taper = 0.60
        self._la_min = 0.25
        self._la_eff = lookahead
        self._la_arc_margin = 0.5 * lookahead
        self._la_branch = '-'
        self._la_tgt_xy = None
        self._infeasible = math.radians(infeasible_deg)
        self._seg_i = 0
        self._state = 'driving'
        self._rtk_quality = NS['_RTK_FIXED']
        self._fix_stamp = _Stamp()
        self._rtk_timeout = 1.0
        self._pos_tol = 0.15
        self._pos_tol_float = 0.40
        self._head_tol_fixed = math.radians(15.0)
        self._head_tol_float = math.radians(25.0)
        self._end_yaw = None
        self._corridor = 2.0
        self._acquire_radius = 1.0
        self._acquired = True
        self._feasible_checked = True
        self._kappa_max = 1.0 / 0.37
        self._speed = 0.2
        self._min_speed = 0.05
        self._slowdown = 0.5
        self._pos_tol_slack = 0.15
        self._min_dfinal = None
        self._reason = ''
        self.drives = []                 # (v, omega) actually commanded
        self.aborted = None              # abort reason, if any
        self.zeroed = 0

    # -- harness plumbing ------------------------------------------------
    def get_logger(self): return _Log()
    def get_clock(self): return _Clock()
    def _fused_is_fresh(self): return True
    def _live_pose_safe(self): return True
    def _publish_status(self, **_k): pass
    def _publish_status_current(self): pass
    def _zero_cmd(self): self.zeroed += 1
    def _drive(self, v, omega): self.drives.append((v, omega))

    def _abort(self, reason):
        self.aborted = reason
        self._state = 'aborted'

    def _active_tol(self, pos_tol_fixed=None):
        return NS['_active_tol'](self, pos_tol_fixed)

    def _lookahead_target(self):
        return NS['_lookahead_target'](self)

    def _eff_lookahead(self, d_end):
        return NS['_eff_lookahead'](self, d_end)

    def _select_join_segment(self):       # join already done in these tests
        return True

    def control(self):
        return NS['_control_cb'](self)


def arc_path(n=30, step=0.4, th0=-40.0, dth=100.0):
    """Gentle left-bending arc, same shape family as an authored glue curve."""
    pts, x, y = [], 0.0, 0.0
    for k in range(n):
        pts.append((x, y))
        th = math.radians(th0 + dth * k / (n - 1))
        x += step * math.cos(th)
        y += step * math.sin(th)
    return pts


# ---------------------------------------------------------------------------
# _lookahead_target
# ---------------------------------------------------------------------------

def test_offpath_targets_current_segment_not_end():
    """2026-06-11 regression core: 1.2 m off-path near the START must aim at
    the current (joined) segment, never the final waypoint."""
    pts = arc_path()
    # 1.2 m perpendicular-ish off the second waypoint.
    rx, ry = pts[1][0] - 0.85, pts[1][1] + 0.85
    r = Robot(pts, (rx, ry), 0.0)
    tx, ty = r._lookahead_target()
    assert (tx, ty) != pts[-1], 'aimed at the curve end from off-path (old bug)'
    want = NS['point_seg_nearest']((rx, ry), pts[0], pts[1])
    assert (abs(tx - want[0]) < 1e-9 and abs(ty - want[1]) < 1e-9), (
        f'target must be the nearest point on the CURRENT segment: got '
        f'({tx:.2f},{ty:.2f}), want ({want[0]:.2f},{want[1]:.2f})')
    assert r._seg_i == 0, f'_seg_i must not jump ahead, got {r._seg_i}'


def test_field_2026_06_11_join_plus_reacquire_targets_start():
    """Integration replay of the field failure: robot off-path, CLOSER to the
    curve end than to its start, start in-front feasible. Join must pick seg 0
    AND re-acquire must then walk it to the START area — with the old code the
    pair degenerated to a beeline at the last segment."""
    pts = arc_path()
    ex, ey = pts[-1]
    sx, sy = pts[0]
    rx, ry = ex + 0.5, ey - 2.0          # ~2 m from the end, ~6+ m from start
    th = math.degrees(math.atan2(sy - ry, sx - rx))   # facing the start
    r = Robot(pts, (rx, ry), th)
    assert math.hypot(rx - ex, ry - ey) < math.hypot(rx - sx, ry - sy), \
        'setup: must be closer to the end than to the start'
    assert NS['_select_join_segment'](r) is True
    assert r._seg_i == 0, f'join must pick seg 0, got {r._seg_i}'
    tx, ty = r._lookahead_target()
    assert math.hypot(tx - sx, ty - sy) < 0.5, (
        f'must walk to the curve START, not the tail: target ({tx:.2f},{ty:.2f})')
    assert r._seg_i == 0


def test_near_end_tracks_tail_in_then_latches_pin():
    """Near the end, ON the path: with the look-ahead taper (undershoot/heading
    fix 2026-06-25) the robot TRACKS THE TAIL straight in — the target is a point
    on the last segment AHEAD toward the pin — and latches the bare pin only once
    within lookahead_min_m. (Old behaviour aimed at the pin from 0.6 m out and
    arrived ~12-22 deg off-heading.)"""
    pts = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
    # d_end = 0.3 m (> lookahead_min 0.25): on-tail point, ahead, toward the pin.
    far = Robot(pts, (1.7, 0.0), 0.0)._lookahead_target()
    assert far != pts[-1] and far[0] > 1.7 and abs(far[1]) < 1e-9, (
        f'near-end must track the tail in (ahead, on the segment): {far}')
    # d_end = 0.15 m (< lookahead_min): latch the bare pin to finish.
    near = Robot(pts, (1.85, 0.0), 0.0)._lookahead_target()
    assert near == pts[-1], f'within lookahead_min the target is the pin: {near}'


def test_on_path_crossing_unchanged():
    """Normal pursuit: on the path, the target is the look-ahead circle
    crossing (~lookahead away), not the end."""
    pts = arc_path()
    r = Robot(pts, pts[0], -40.0)
    tx, ty = r._lookahead_target()
    d = math.hypot(tx - pts[0][0], ty - pts[0][1])
    assert abs(d - r._lookahead) < 0.05, f'crossing should be ~lookahead: {d:.2f}'
    assert (tx, ty) != pts[-1]


def test_seg_i_stays_monotone_in_reacquire():
    pts = arc_path()
    r = Robot(pts, (pts[10][0] - 0.9, pts[10][1] + 0.9), 0.0)
    r._seg_i = 8
    r._lookahead_target()
    assert r._seg_i >= 8, f'_seg_i went backward: {r._seg_i}'


# ---------------------------------------------------------------------------
# forward-cone guard in _control_cb
# ---------------------------------------------------------------------------

def test_behind_target_aborts_never_steers_straight():
    """2026-06-11 regression tail: target dead behind the nose (alpha ~ 180,
    the pure-pursuit singularity). Must ABORT, not command ~zero steering."""
    pts = [(0.0, 0.0), (1.0, 0.0)]
    r = Robot(pts, (2.5, 0.0), 0.0)      # past the end, facing away
    r.control()
    assert r.aborted is not None, 'kept driving with the target behind (old bug)'
    assert 'off the nose' in r.aborted
    assert not r.drives, f'commanded motion before abort: {r.drives}'


def test_overshoot_sideways_aborts():
    """Near-miss arrival: passed 0.4 m beside the endpoint, nose now past it.
    Old code orbited/wandered; must abort loudly."""
    pts = [(0.0, 0.0), (2.0, 0.0)]
    r = Robot(pts, (2.6, 0.4), 0.0)      # beyond the end, offset, facing on
    r.control()
    assert r.aborted is not None
    assert not r.drives


def test_front_target_still_drives():
    """Sanity: a reachable in-front target drives, no abort."""
    pts = arc_path()
    r = Robot(pts, pts[0], -40.0)
    r.control()
    assert r.aborted is None
    assert len(r.drives) == 1
    v, _w = r.drives[0]
    assert v > 0.0


def test_near_miss_arrives_undershoot_policy():
    """2026-06-11 field regression: passed 0.2 m beside the endpoint (tol
    0.15) — within tol+slack the closest pass must count as ARRIVAL with the
    true error reported, not pause the batch."""
    pts = [(0.0, 0.0), (2.0, 0.0)]
    r = Robot(pts, (2.1, 0.18), 0.0)     # just past, 0.21 m off the end
    r.control()
    assert r._state == 'arrived', f'expected arrival, got abort: {r.aborted}'
    assert 'closest pass' in r._reason
    assert not r.drives


def test_wide_miss_still_aborts():
    """Beyond tol+slack the pass-by is a genuine failure: abort, never accept."""
    pts = [(0.0, 0.0), (2.0, 0.0)]
    r = Robot(pts, (2.2, 0.35), 0.0)     # 0.40 m off the end (> 0.15+0.15)
    r.control()
    assert r.aborted is not None, 'accepted a 0.40 m miss'
    assert r._state == 'aborted'


def test_slack_only_applies_in_endgame():
    """A backward target far from the end must still abort even when some
    earlier closest approach was small (mid-path cone break is not a miss)."""
    pts = [(0.0, 0.0), (1.0, 0.0)]
    r = Robot(pts, (2.5, 0.0), 0.0)      # 1.5 m past the end, facing away
    r._min_dfinal = 0.05                  # pretend it once skimmed the end
    r.control()
    assert r.aborted is not None
    assert r._state == 'aborted'


def test_arrival_still_wins_over_guard():
    """Inside pos_tol the arrival branch fires before any steering/guard."""
    pts = [(0.0, 0.0), (2.0, 0.0)]
    r = Robot(pts, (2.1, 0.0), 0.0)      # 0.1 m past the end (< pos_tol 0.15)
    r.control()
    assert r._state == 'arrived'
    assert r.aborted is None


def main():
    fails = 0
    for name, fn in sorted(globals().items()):
        if name.startswith('test_') and callable(fn):
            try:
                fn()
                print(f'PASS {name}')
            except AssertionError as exc:
                print(f'FAIL {name}: {exc}')
                fails += 1
    sys.exit(1 if fails else 0)


def _pytest_collect():  # keep pytest happy about the helper name
    pass


if __name__ == '__main__':
    main()
