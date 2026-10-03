# -*- coding: utf-8 -*-
"""Unit tests for the wiggle path families (wiggle_path.py, 2026-10-02).

Invariants that cannot hold if the curvature integration is wrong, plus the
two cross-checks that keep the three copies of the geometry honest: the
browser preview (interactive.html, JS) must equal the Python path the robot
drives, and the planner must seat every wiggle cell as a loader-clean pair.

    python3 tools/analysis/tests/test_wiggle_path.py
"""
import json
import math
import os
import re
import shutil
import subprocess
import sys
import tempfile

import numpy as np

_REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
_PKG = os.path.join(_REPO, "scalecar-vfg-h-infinite", "ros2_bridge",
                    "limo_path_follower")
_VFG = os.path.join(_REPO, "scalecar-vfg-h-infinite")
for p in (_PKG, _VFG, os.path.dirname(__file__)):
    if p not in sys.path:
        sys.path.insert(0, p)

import venue_geom                                  # noqa: E402
import experiment_planner as ep                    # noqa: E402
from wiggle_path import WigglePath, WIGGLE_TYPES   # noqa: E402

KW = dict(wavelength=3.0, wavelength_end=1.5, n_periods=2, L1=0.5, L_end=0.5)


def test_starts_at_origin_heading_plus_x():
    for kind in ("sine", "chirp", "square"):
        p = WigglePath(kind=kind, R=1.2, **KW)
        assert np.allclose(p.position(0.0), [0.0, 0.0])
        assert abs(p.heading(0.0)) < 1e-12


def test_peak_curvature_is_one_over_R():
    for kind in ("sine", "chirp", "square"):
        for R in (1.2, 1.5, 2.0):
            p = WigglePath(kind=kind, R=R, **KW)
            k = max(abs(p.curvature(s))
                    for s in np.linspace(0, p.total_length, 5001))
            assert abs(k * R - 1.0) < 1e-3, (kind, R, k * R)


def test_square_arcs_lie_on_circles_of_radius_R():
    R = 1.2
    p = WigglePath(kind="square", R=R, **KW)
    # First (half) arc, a quarter wavelength long, turns left about (L1, R).
    for s in np.linspace(0.5, 0.5 + 3.0 / 4.0, 40):
        assert abs(math.hypot(*(p.position(s) - [0.5, R])) - R) < 1e-4


def test_length_does_not_grow_with_R():
    # Field 2026-10-02: an R-scaled square reached 12.6 m and no longer fit a
    # 14.6 m venue as an A/B pair. Every kind is n wavelengths long.
    for kind in ("sine", "chirp", "square"):
        lens = [WigglePath(kind=kind, R=R, **KW).total_length
                for R in (1.2, 1.5, 2.0)]
        assert max(lens) - min(lens) < 0.02, (kind, lens)
        assert max(lens) < 7.5, (kind, lens)


def test_sine_and_square_keep_their_mean_direction():
    # Whole periods: heading returns to +x and the path ends on its axis.
    for kind in ("sine", "square"):
        p = WigglePath(kind=kind, R=1.2, **KW)
        assert abs(math.degrees(p.heading(p.total_length))) < 0.5, kind
        assert abs(p.position(p.total_length)[1]) < 0.05, kind


def test_curvature_agrees_with_the_geometry():
    # Turning of the integrated polyline (whole grid cells) == curvature().
    for kind in ("sine", "chirp", "square"):
        p = WigglePath(kind=kind, R=1.2, **KW)
        step = 10 * p.DS
        s = np.arange(0, p.total_length, step)
        P = np.array([p.position(x) for x in s])
        d = np.diff(P, axis=0)
        kg = np.diff(np.unwrap(np.arctan2(d[:, 1], d[:, 0]))) / step
        K = np.array([p.curvature(x) for x in s])
        near = np.zeros(len(kg), bool)
        for j in np.where(np.abs(np.diff(K)) > 0.05)[0]:
            near[max(0, j - 2): j + 2] = True
        assert np.max(np.abs(kg - K[1:-1])[~near]) < 5e-3, kind


def test_recipe_path_builds_every_wiggle_type():
    for t in WIGGLE_TYPES:
        path = venue_geom.recipe_path({"type": t, "params": {"R": 1.5}})
        assert path is not None and path.total_length > 5.0, t


def test_browser_preview_equals_the_driven_path():
    node = shutil.which("node")
    if node is None:
        print("  (skipped: node not installed)")
        return
    html = open(os.path.join(_REPO, "tools", "path_gen", "interactive.html"),
                encoding="utf-8").read()
    fn = re.search(r"function lbWiggleRecipeLocal\(recipe\) \{.*?\n\}\n",
                   html, re.S).group(0)
    js = fn + ("const o = {}; for (const t of %s) o[t] = lbWiggleRecipeLocal("
               "{type: t, params: %s}); console.log(JSON.stringify(o));"
               % (json.dumps(list(WIGGLE_TYPES)), json.dumps(KW)))
    with tempfile.NamedTemporaryFile("w", suffix=".js", delete=False) as f:
        f.write(js)
    try:
        res = json.loads(subprocess.check_output([node, f.name]))
    finally:
        os.unlink(f.name)
    for t, pts in res.items():
        p = WigglePath(kind=t.split("_", 1)[1], R=1.2, **KW)
        ss = [0.1 * j for j in range(len(pts) - 1)] + [p.total_length]
        err = max(math.hypot(*(p.position(s) - q)) for s, q in zip(ss, pts))
        assert err < 1e-6, f"{t}: browser preview differs by {err:.2e} m"


def test_planner_seats_wiggle_cells_as_clean_pairs():
    from test_experiment_planner import (_legs_from_stage, _rotated_venue,
                                         FOOT, TRACK)
    doc = {"matrix": {"radius_m": [1.0],
                      "radius_m_by_family": {t: [1.5] for t in WIGGLE_TYPES},
                      "controller": ["lpv-hinf"], "v_const": [1.0],
                      "path_family": list(WIGGLE_TYPES)},
           "repetitions": 2}
    venue = _rotated_venue(24.0, 10.0, 39.0)
    plan = ep.plan_stages(venue, doc, {}, FOOT, TRACK)
    assert plan["ok"], plan["notes"]
    assert sorted(g["family"] for st in plan["stages"]
                  for g in st["geometries"]) == sorted(WIGGLE_TYPES)
    for st in plan["stages"]:
        assert len(st["experiments"]) == 2, st["name"]
        ok, report = venue_geom.check_legs_containment(
            _legs_from_stage(st, venue), venue, FOOT, TRACK)
        assert ok, f"{st['name']}:\n{report}"


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
