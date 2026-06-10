#!/usr/bin/env python3
"""Unit tests for reposition_node._select_join_segment (boot-time path join).

reposition_node imports rclpy, which does not exist off-robot, so the geometry
under test (_wrap, point_seg_dist, _select_join_segment) is extracted from the
source by AST and exec'd standalone — same approach as the venue_geom tests.

Scenario under test is the 2026-06-10 field regression: a J-hook glue curve
(starts pointing SE, arrives pointing NNE) and a robot standing mid-path facing
along the local tangent aborted with "start heading 157 deg off the path"
because the tracker always joined at segment 0.

Run:  python3 tools/analysis/tests/test_reposition_join.py   (or pytest)
"""
import ast
import math
import os
import sys

SRC = os.path.join(
    os.path.dirname(__file__), '..', '..', '..',
    'scalecar-vfg-h-infinite', 'ros2_bridge', 'limo_path_follower',
    'reposition_node.py')


def _extract(names):
    with open(SRC) as f:
        tree = ast.parse(f.read())
    ns = {'math': math}
    # module-level helpers
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name in names:
            exec(compile(ast.Module([node], []), SRC, 'exec'), ns)
    # the method, lifted off the class
    for node in tree.body:
        if isinstance(node, ast.ClassDef) and node.name == 'RepositionNode':
            for sub in node.body:
                if (isinstance(sub, ast.FunctionDef)
                        and sub.name == '_select_join_segment'):
                    exec(compile(ast.Module([sub], []), SRC, 'exec'), ns)
    return ns


NS = _extract({'_wrap', 'point_seg_dist'})


class _Log:
    def info(self, *_a, **_k): pass


class Robot:
    """Minimal stand-in for the node: only what _select_join_segment touches."""

    def __init__(self, waypoints, xy, heading_deg, infeasible_deg=100.0):
        self._waypoints = waypoints
        self._fix_xy = xy
        self._heading_est = math.radians(heading_deg)
        self._infeasible = math.radians(infeasible_deg)
        self._seg_i = 0

    def get_logger(self):
        return _Log()

    def select(self):
        return NS['_select_join_segment'](self)


def jhook():
    """J-hook in local XY (x east, y north, headings = math angle CCW from +x):
    starts heading -40 deg (SE-ish), bends left to +60 deg (NNE-ish) at the end.
    """
    pts, x, y = [], 0.0, 0.0
    th = math.radians(-40.0)
    for k in range(19):
        pts.append((x, y))
        th = math.radians(-40.0 + (100.0 * k / 18.0))
        x += 0.4 * math.cos(th)
        y += 0.4 * math.sin(th)
    return pts


def test_mid_path_join():
    """Robot ON the path at wp13 facing the local tangent: joins near 13, not 0."""
    pts = jhook()
    th13 = math.degrees(math.atan2(pts[14][1] - pts[13][1],
                                   pts[14][0] - pts[13][0]))
    r = Robot(pts, pts[13], th13)
    assert r.select() is True
    assert r._seg_i >= 11, f'joined too early: seg {r._seg_i}'


def test_at_start_facing_tangent():
    """Robot at wp0 facing the start tangent: joins at segment 0."""
    pts = jhook()
    r = Robot(pts, (pts[0][0] - 0.05, pts[0][1] + 0.05), -40.0)
    assert r.select() is True
    assert r._seg_i == 0, f'expected seg 0, got {r._seg_i}'


def test_prefix_behind_join():
    """2026-06-10 regression shape: the path's first half lies BEHIND the robot
    (segment-0 target >100 deg off the nose), its second half lies ahead. Old
    code aborted on segment 0; now a later segment must win."""
    pts = ([(-3.0 + 0.5 * k, 0.0) for k in range(6)]        # west arm (behind)
           + [(0.5 + 0.5 * k, 0.2) for k in range(6)])      # east arm (ahead)
    r = Robot(pts, (0.0, 0.1), 0.0)                          # facing east
    assert r.select() is True, 'must find a forward-feasible join'
    assert r._seg_i >= 5, f'expected an east-arm segment, got {r._seg_i}'


def test_truly_infeasible():
    """Robot past the end, facing away from the whole path: must refuse."""
    pts = jhook()
    ex, ey = pts[-1]
    r = Robot(pts, (ex + 1.0, ey + 1.5), 60.0)   # beyond the end, still heading out
    assert r.select() is False


def test_single_point_ahead_and_behind():
    pts = [(2.0, 0.0)]
    assert Robot(pts, (0.0, 0.0), 0.0).select() is True      # dead ahead
    assert Robot(pts, (4.0, 0.0), 0.0).select() is False     # behind the nose


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


if __name__ == '__main__':
    main()
