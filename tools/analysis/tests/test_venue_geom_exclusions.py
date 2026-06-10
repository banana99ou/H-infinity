# -*- coding: utf-8 -*-
"""Unit tests for venue_geom.check_legs_containment exclusion handling.

Pure python, no ROS, no vfg_pathfollowing (uses reposition-waypoint curves
only). Run either way:

    python3 tools/analysis/tests/test_venue_geom_exclusions.py
    python3 -m pytest tools/analysis/tests/test_venue_geom_exclusions.py
"""
import os
import sys

_REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
_PKG = os.path.join(_REPO, "scalecar-vfg-h-infinite", "ros2_bridge",
                    "limo_path_follower")
if _PKG not in sys.path:
    sys.path.insert(0, _PKG)

import venue_geom  # noqa: E402

# 40 m x 40 m venue, EN frame anchored at corner 0. margin 0.2 + footprint 0.3
# + track 0.3 => required clearance 0.8 m.
_M = 1.0 / 111320.0   # metres -> degrees (equator, lon scale == lat scale)
FOOT, TRACK = 0.3, 0.3
REQ = 0.2 + FOOT + TRACK


def _ll(e, n):
    return {"lat": n * _M, "lon": e * _M}


def _venue(exclusions=None):
    v = {
        "name": "test",
        "safety_margin_m": 0.2,
        "corners_wgs84": [_ll(0, 0), _ll(40, 0), _ll(40, 40), _ll(0, 40)],
    }
    if exclusions is not None:
        v["exclusions"] = exclusions
    return v


def _leg(*en_pts):
    return [{"id": "leg_1", "curves": [{
        "name": "glue", "kind": "reposition",
        "waypoints_wgs84": [_ll(e, n) for (e, n) in en_pts],
    }]}]


def _circle(e, n, r):
    c = _ll(e, n)
    return {"kind": "circle", "lat": c["lat"], "lon": c["lon"], "radius_m": r}


def test_clear_path_passes():
    ok, report = venue_geom.check_legs_containment(
        _leg((10, 10), (10, 30)), _venue([_circle(30, 20, 2.0)]), FOOT, TRACK)
    assert ok, report


def test_waypoint_inside_exclusion_fails():
    ok, report = venue_geom.check_legs_containment(
        _leg((10, 20), (20, 20)), _venue([_circle(20, 20, 2.0)]), FOOT, TRACK)
    assert not ok
    assert "glue" in report


def test_segment_through_exclusion_fails():
    # Both endpoints clear; the segment between them crosses the circle.
    ok, report = venue_geom.check_legs_containment(
        _leg((10, 20), (30, 20)), _venue([_circle(20, 20, 2.0)]), FOOT, TRACK)
    assert not ok
    assert "seg" in report


def test_near_miss_within_required_clearance_fails():
    # Path passes 2.5 m from a 2.0 m circle: 0.5 m clearance < 0.8 m required.
    ok, report = venue_geom.check_legs_containment(
        _leg((10, 22.5), (30, 22.5)), _venue([_circle(20, 20, 2.0)]),
        FOOT, TRACK)
    assert not ok


def test_no_exclusions_unchanged():
    ok, report = venue_geom.check_legs_containment(
        _leg((10, 20), (30, 20)), _venue(), FOOT, TRACK)
    assert ok, report


def test_malformed_exclusion_fails_closed():
    ok, report = venue_geom.check_legs_containment(
        _leg((10, 10), (10, 30)),
        _venue([{"kind": "circle", "lat": "not-a-number"}]), FOOT, TRACK)
    assert not ok
    assert "malformed" in report


def test_unsupported_exclusion_kind_fails_closed():
    ok, report = venue_geom.check_legs_containment(
        _leg((10, 10), (10, 30)),
        _venue([{"kind": "polygon"}]), FOOT, TRACK)
    assert not ok
    assert "unsupported kind" in report


if __name__ == "__main__":
    fails = 0
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            try:
                fn()
                print(f"  [PASS] {name}")
            except AssertionError as exc:
                print(f"  [FAIL] {name}: {exc}")
                fails += 1
    sys.exit(1 if fails else 0)
