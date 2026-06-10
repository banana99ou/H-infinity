# -*- coding: utf-8 -*-
"""Unit tests for venue_geom slalom recipe sampling + containment.

Slalom was previously NOT geometry-checkable (recipe_points_en returned []
and the containment gate flagged it UNVERIFIED) — which is why the WebUI
could not author half the experiment matrix. These tests pin the new
behaviour: slalom samples like step/uturn and is containment-gated the same
way. Needs vfg_pathfollowing (in-repo) + numpy. Run either way:

    python3 tools/analysis/tests/test_venue_geom_slalom.py
    python3 -m pytest tools/analysis/tests/test_venue_geom_slalom.py
"""
import math
import os
import sys

_REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
_PKG = os.path.join(_REPO, "scalecar-vfg-h-infinite", "ros2_bridge",
                    "limo_path_follower")
_VFG = os.path.join(_REPO, "scalecar-vfg-h-infinite")
for p in (_PKG, _VFG):
    if p not in sys.path:
        sys.path.insert(0, p)

import venue_geom  # noqa: E402

_M = 1.0 / 111320.0   # metres -> degrees (equator: lon scale == lat scale)
FOOT, TRACK = 0.3, 0.3
REQ = 0.2 + FOOT + TRACK


def _ll(e, n):
    return {"lat": n * _M, "lon": e * _M}


def _venue(side=40):
    return {
        "name": "test",
        "safety_margin_m": 0.2,
        "corners_wgs84": [_ll(0, 0), _ll(side, 0), _ll(side, side), _ll(0, side)],
    }


def _slalom(R=0.5, n_arcs=2, L_mid=0.5, L1=1.0, L_end=1.0,
            theta_deg=90.0):
    return {"type": "slalom",
            "params": {"R": R, "theta_arc": math.radians(theta_deg),
                       "L1": L1, "L_mid": L_mid, "n_arcs": n_arcs,
                       "L_end": L_end}}


def _leg(recipe, lat, lon, heading_deg):
    return {"id": "leg_1", "curves": [{
        "name": "exp1", "kind": "recipe", "scored": True,
        "start_pose": {"lat": lat, "lon": lon, "heading_deg": heading_deg},
        "recipe": recipe,
    }]}


def test_slalom_samples_nonempty_and_correct_extent():
    pts = venue_geom.recipe_points_en(
        _slalom(), {"lat": 0.0, "lon": 0.0, "heading_deg": 0.0}, 0.0, 0.0)
    assert len(pts) > 10
    # Heading 0 = due north; the slalom's first arc turns LEFT (west, -E).
    es = [p[0] for p in pts]
    ns = [p[1] for p in pts]
    assert max(es) <= 1e-9, "slalom should stay left (-E) of a north heading"
    # Lateral excursion for n_arcs=2 is 2R + L_mid; forward includes L1+L_end.
    assert abs(min(es) - (-(2 * 0.5 + 0.5))) < 0.05
    assert abs(max(ns) - 3.0) < 0.1
    # Arc-length monotonic + total = L1 + n_arcs*R*theta + L_mid + L_end.
    total = pts[-1][2]
    assert abs(total - (1.0 + 2 * 0.5 * math.pi / 2 + 0.5 + 1.0)) < 1e-6


def test_slalom_contained_passes():
    ok, report = venue_geom.check_legs_containment(
        [_leg(_slalom(), _ll(20, 20)["lat"], _ll(20, 20)["lon"], 90.0)],
        _venue(), FOOT, TRACK)
    assert ok, report
    assert "UNVERIFIED" not in report


def test_slalom_near_edge_fails():
    # Start 0.3 m from the west wall heading north: the left weave exits.
    ok, report = venue_geom.check_legs_containment(
        [_leg(_slalom(), _ll(0.3, 20)["lat"], _ll(0.3, 20)["lon"], 0.0)],
        _venue(), FOOT, TRACK)
    assert not ok
    assert "exp1" in report


def test_unknown_recipe_type_still_flagged_unverified():
    ok, report = venue_geom.check_legs_containment(
        [_leg({"type": "mystery", "params": {}},
              _ll(20, 20)["lat"], _ll(20, 20)["lon"], 0.0)],
        _venue(), FOOT, TRACK)
    assert not ok
    assert "UNVERIFIED" in report


def test_recipe_path_matches_follower_defaults():
    # No params: must mirror path_follower.build_path_from_recipe defaults
    # (R=0.5, theta=pi/2, L1=5, L_mid=2, n_arcs=6, L_end=25).
    p = venue_geom.recipe_path({"type": "slalom", "params": {}})
    assert p is not None
    expect = 5.0 + 6 * (0.5 * math.pi / 2) + 5 * 2.0 + 25.0
    assert abs(float(p.total_length) - expect) < 1e-6


if __name__ == "__main__":
    fails = 0
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            try:
                fn()
                print(f"PASS {name}")
            except AssertionError as exc:
                fails += 1
                print(f"FAIL {name}: {exc}")
    sys.exit(1 if fails else 0)
