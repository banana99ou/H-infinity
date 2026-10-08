# -*- coding: utf-8 -*-
"""Mirrored right turns (mirror_path.py, step_m / slalom_m), 2026-10-08.

A right turn is the left recipe reflected across its start heading. Pinned:
the reflection relations at every sample, the left-normal convention, the
analytic end point of a right step, no cusp (the vendor direction=-1 step
runs backward at the arc start), closest_point on the reflected curve, the
geometry gate building the same path, and — the strongest check — the REAL
vendor closed loop (both controllers) tracking step_m as the exact mirror
image of step. Each test says what result would make it fail.

Run:  python3 -m pytest -q tools/analysis/tests/test_mirror_path.py
"""
import math
import os
import sys

import numpy as np

_REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
_PKG = os.path.join(_REPO, "scalecar-vfg-h-infinite", "ros2_bridge",
                    "limo_path_follower")
_VFG = os.path.join(_REPO, "scalecar-vfg-h-infinite")
for p in (_PKG, _VFG):
    if p not in sys.path:
        sys.path.insert(0, p)

from vfg_pathfollowing.paths.step_curvature import StepCurvaturePath  # noqa: E402
from vfg_pathfollowing.paths.slalom import SlalomPath                  # noqa: E402
from mirror_path import MirroredPath, base_type, is_mirrored           # noqa: E402
import venue_geom                                                      # noqa: E402

STEP = dict(L1=2.0, R=1.0, theta_arc=math.pi / 2, L2=2.0, direction=1)
SLALOM = dict(R=1.0, theta_arc=math.pi / 2, L1=2.0, L_mid=0.5, n_arcs=2, L_end=2.0)


def _pairs():
    return [(StepCurvaturePath(**STEP), MirroredPath(StepCurvaturePath(**STEP))),
            (SlalomPath(**SLALOM), MirroredPath(SlalomPath(**SLALOM)))]


def test_reflection_relations_everywhere():
    # Fails if any method is not the reflection of the base, or the normal is
    # not the LEFT normal of the reflected tangent (the VFG sign convention).
    for base, m in _pairs():
        assert m.total_length == base.total_length
        for s in np.linspace(0, base.total_length, 97):
            p, q = base.position(s), m.position(s)
            assert abs(q[0] - p[0]) < 1e-12 and abs(q[1] + p[1]) < 1e-12
            t, tm = base.tangent(s), m.tangent(s)
            assert abs(tm[0] - t[0]) < 1e-12 and abs(tm[1] + t[1]) < 1e-12
            assert abs(m.curvature(s) + base.curvature(s)) < 1e-12
            assert abs(m.heading(s) + base.heading(s)) < 1e-12
            nm = m.normal(s)
            assert abs(nm[0] + tm[1]) < 1e-12 and abs(nm[1] - tm[0]) < 1e-12


def test_right_step_end_point_and_no_cusp():
    # Fails if the mirrored step does not end at (L1+R, -(R+L2)) heading -90
    # deg, or if it ever moves backward (the vendor direction=-1 cusp).
    m = MirroredPath(StepCurvaturePath(**STEP))
    e = m.position(m.total_length)
    assert abs(e[0] - (STEP["L1"] + STEP["R"])) < 1e-6
    assert abs(e[1] + (STEP["R"] + STEP["L2"])) < 1e-6
    assert abs(math.degrees(m.heading(m.total_length)) + 90.0) < 1e-6
    xs = [m.position(s)[0] for s in np.linspace(0, m.total_length, 400)]
    assert all(b >= a - 1e-9 for a, b in zip(xs, xs[1:]))
    # the arc curves RIGHT (negative curvature)
    assert m.curvature(STEP["L1"] + 0.5) < 0
    # the vendor's own right step does have the cusp this replaces
    v = StepCurvaturePath(**dict(STEP, direction=-1))
    vx = [v.position(s)[0] for s in np.linspace(0, v.total_length, 400)]
    assert any(b < a - 1e-6 for a, b in zip(vx, vx[1:])), "vendor cusp gone?"


def test_closest_point_on_reflected_curve():
    # Fails if PathBase.closest_point (Newton on the reflected methods) does
    # not return the reflection of the base's closest point.
    for base, m in _pairs():
        for q in ((2.5, 0.4), (3.0, 1.5), (1.0, 0.2), (4.0, 3.0)):
            sb, pb = base.closest_point(q)
            sm, pm = m.closest_point((q[0], -q[1]))
            assert abs(sb - sm) < 1e-6, (q, sb, sm)
            assert abs(pm[0] - pb[0]) < 1e-6 and abs(pm[1] + pb[1]) < 1e-6


def test_geometry_gate_builds_the_mirror():
    # Fails if venue_geom (loader / planner / executor containment) checks a
    # different curve than the follower drives.
    for t, cls, prm in (("step", StepCurvaturePath, STEP),
                        ("slalom", SlalomPath, SLALOM)):
        p = venue_geom.recipe_path({"type": t + "_m", "params": prm})
        assert isinstance(p, MirroredPath)
        ref = MirroredPath(cls(**prm))
        for s in np.linspace(0, ref.total_length, 31):
            a, b = p.position(s), ref.position(s)
            assert abs(a[0] - b[0]) < 1e-12 and abs(a[1] - b[1]) < 1e-12
    assert base_type("slalom_m") == "slalom" and is_mirrored("step_m")
    assert not is_mirrored("step")


def test_closed_loop_tracks_the_exact_mirror_image():
    # Fails if either REAL vendor controller (LPV-Hinf schedules on |kappa|,
    # PID-FF feeds forward atan(L kappa)) does not track the right step as the
    # mirror image of the left one: lateral position, heading, heading error
    # and steering command must all flip sign, the rest stay equal.
    from vfg_pathfollowing import Simulator
    for ctrl in ("lpv-hinf", "pid-ff"):
        for base, m in _pairs():
            T = base.total_length / 1.0 - 0.5          # stay on the path
            a = Simulator(base, controller=ctrl, speed=1.0, dt=0.05).run(T=T)
            b = Simulator(m, controller=ctrl, speed=1.0, dt=0.05).run(T=T)
            n = min(len(a.time), len(b.time))
            assert n > 50
            assert np.max(np.abs(a.X[:n] - b.X[:n])) < 1e-6, ctrl
            assert np.max(np.abs(a.Y[:n] + b.Y[:n])) < 1e-6, ctrl
            assert np.max(np.abs(a.e_d[:n] + b.e_d[:n])) < 1e-6, ctrl
            assert np.max(np.abs(a.psi[:n] + b.psi[:n])) < 1e-6, ctrl
            assert np.max(np.abs(a.e_psi[:n] + b.e_psi[:n])) < 1e-6, ctrl
            assert np.max(np.abs(a.delta_cmd[:n] + b.delta_cmd[:n])) < 1e-6, ctrl
            assert np.max(np.abs(a.rho[:n] - b.rho[:n])) < 1e-6, ctrl
            # and it actually turned (a no-motion sim would pass the above)
            assert np.max(np.abs(a.psi[:n])) > 1.0, ctrl


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print("PASS", name)
