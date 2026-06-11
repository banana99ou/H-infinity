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
_METHODS = {'_lookahead_target', '_control_cb', '_active_tol'}
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

def test_offpath_targets_nearest_point_not_end():
    """2026-06-11 regression core: 1.2 m off-path near the START must aim at
    the nearby path point, never the final waypoint."""
    pts = arc_path()
    # 1.2 m perpendicular-ish off the second waypoint.
    rx, ry = pts[1][0] - 0.85, pts[1][1] + 0.85
    r = Robot(pts, (rx, ry), 0.0)
    tx, ty = r._lookahead_target()
    assert (tx, ty) != pts[-1], 'aimed at the curve end from off-path (old bug)'
    d_tgt = math.hypot(tx - rx, ty - ry)
    xtrack = NS['polyline_dist']((rx, ry), pts)
    assert abs(d_tgt - xtrack) < 1e-6, (
        f'target is not the nearest path point: {d_tgt:.2f} vs xtrack '
        f'{xtrack:.2f}')
    assert r._seg_i <= 3, f'nearest segment should be early, got {r._seg_i}'


def test_near_end_still_drives_straight_in():
    """Within a look-ahead of the end, ON the path: final point is the target
    (the legitimate original fallback)."""
    pts = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
    r = Robot(pts, (1.7, 0.0), 0.0)
    assert r._lookahead_target() == pts[-1]


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
