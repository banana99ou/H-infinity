# -*- coding: utf-8 -*-
"""Steering calibration: lock math, auto matrix, planner stage, leg assembly.

2026-10-08. Every session starts with an open-loop figure-8 (calib_node); the
first one locks the matrix radii (calibration.py), the executor then plans the
matrix itself and builds the active.json legs in Python (stages_to_legs) — the
job the WebUI's lbBuiltLegs does on Send. These tests pin:

* the lock math (radii from R_min, bounds, worst side, refuse to overwrite);
* manifest.load_experiment resolving ``radius_m: auto`` from a lock;
* the planner's calibration stage on the LIVE rooftop capture (fits, approach
  loop joinable from any heading, re-seat at the driven pin is identical);
* stages_to_legs == the REAL WebUI JS assembly (node), so the executor's
  self-built batch is the batch the operator's Send would have produced.

Each test states what result would make it fail.

Run:  python3 -m pytest -q tools/analysis/tests/test_calibration_plan.py
"""
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile
from datetime import datetime, timedelta, timezone

_REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
_PKG = os.path.join(_REPO, "scalecar-vfg-h-infinite", "ros2_bridge",
                    "limo_path_follower")
_VFG = os.path.join(_REPO, "scalecar-vfg-h-infinite")
_TOOLS = os.path.join(_REPO, "tools", "analysis")
for p in (_PKG, _VFG, _TOOLS):
    if p not in sys.path:
        sys.path.insert(0, p)

import calibration as cal          # noqa: E402
import experiment_planner as ep    # noqa: E402
import manifest                    # noqa: E402
import venue_geom                  # noqa: E402

_FIX = os.path.join(_REPO, "tools", "analysis", "tests", "venue_fixtures")
_HTML = os.path.join(_REPO, "tools", "path_gen", "interactive.html")
_NODE_SCRIPT = os.path.join(os.path.dirname(__file__), "webui_assemble.js")
_YAML = os.path.join(_REPO, "scenarios", "experiment.yaml")


def _venue():
    """The live rooftop capture (2026-10-02 fixture: 23 x 9.8 m, the tighter
    of the two real captures)."""
    with open(os.path.join(_FIX, "active.json")) as f:
        v = json.load(f)
    return {k: x for k, x in v.items() if k not in ("plan_stages", "legs")}


def _result(r_left, r_right, speeds=(0.5, 1.0), ok=True):
    segs = []
    for v in speeds:
        for d, r in (("left", r_left), ("right", r_right)):
            segs.append({"v_cmd": v, "dir": d, "R_imu_m": r,
                         "R_rtk_rear_m": r * 1.02})
    return {"ok": ok, "reason": None if ok else "x", "segments": segs}


# ----------------------------------------------------------------------
# calibration.py
# ----------------------------------------------------------------------

def test_fig8_geometry():
    # Fails if the footprint does not start at the origin heading +x, does not
    # close, or a real circle of smaller radius could leave it.
    p = cal.CalibFig8Path(1.2)
    x0, y0 = p.position(0.0)
    x1, y1 = p.position(0.05)
    assert abs(x0) < 1e-9 and abs(y0) < 1e-9
    assert x1 > 0 and abs(y1) < 0.01            # leaves along +x, turning left
    xe, ye = p.position(p.total_length)
    assert math.hypot(xe, ye) < 1e-6            # closes at the start
    pts = [p.position(p.total_length * i / 400) for i in range(401)]
    assert max(y for _, y in pts) == max(y for _, y in pts) <= 2.4 + 1e-9
    assert min(y for _, y in pts) >= -2.4 - 1e-9
    assert max(abs(x) for x, _ in pts) <= 1.2 + 1e-9
    # a left circle of radius 0.6 tangent at the origin lies inside the
    # planned left circle (centre (0, 1.2), radius 1.2)
    for i in range(100):
        th = 2 * math.pi * i / 100
        x, y = 0.6 * math.sin(th), 0.6 - 0.6 * math.cos(th)
        assert math.hypot(x - 0.0, y - 1.2) <= 1.2 + 1e-9


def test_radii_from_rmin():
    # Fails if the radii are not descending, not rounded to 0.05, or the
    # tightest cell asks more than margin x the measured full-lock curvature.
    c = cal.config({})
    for rmin in (0.5, 0.55, 0.62, 0.8, 1.04):
        radii = cal.radii_from_rmin(rmin, c)
        assert radii == sorted(radii, reverse=True)
        assert all(abs(r / 0.05 - round(r / 0.05)) < 1e-6 for r in radii)
        # curvature of the tightest cell vs full lock: <= margin (+ rounding)
        assert (1.0 / radii[-1]) / (1.0 / rmin) <= c["margin"] + 0.06
        assert len(radii) == 4
    assert cal.radii_from_rmin(0.62, c) == [1.95, 1.35, 0.95, 0.75]


def test_compute_lock_worst_side_and_bounds():
    # Fails if the lock uses the tighter side (the looser side limits both
    # directions), accepts a stock-like or impossible R, or locks a bad run.
    c = cal.config({})
    lock, err = cal.compute_lock(_result(0.58, 0.64), c)
    assert err is None and lock["R_min_m"] == 0.64
    assert lock["radius_m"] == cal.radii_from_rmin(0.64, c)
    assert lock["epoch"].startswith("lock-")
    _l, err = cal.compute_lock(_result(1.25, 1.3), c)
    assert _l is None and "outside the sane range" in err
    _l, err = cal.compute_lock(_result(0.3, 0.3), c)
    assert _l is None
    _l, err = cal.compute_lock(_result(0.6, 0.6, ok=False), c)
    assert _l is None and "not ok" in err
    only_left = {"ok": True, "segments": [{"v_cmd": 1.0, "dir": "left", "R_imu_m": 0.6}]}
    _l, err = cal.compute_lock(only_left, c)
    assert _l is None and "left+right" in err
    lock, _e = cal.compute_lock(_result(0.5, 0.7), c)
    assert any("asymmetry" in w for w in lock["warnings"])


def test_sanity_check():
    # Fails if a stock-like R passes against a direct-mode lock, or a small
    # drift fails.
    c = cal.config({})
    lock, _e = cal.compute_lock(_result(0.6, 0.6), c)
    ok, why, r = cal.sanity_check(_result(0.62, 0.63, speeds=(1.0,)), lock, c)
    assert ok, why
    ok, why, r = cal.sanity_check(_result(1.05, 1.06, speeds=(1.0,)), lock, c)
    assert not ok and "steering changed" in why
    ok, why, r = cal.sanity_check(_result(0.6, 0.6, speeds=(1.0,), ok=False), lock, c)
    assert not ok


def test_lock_persistence_and_mode_due():
    # Fails if a second lock overwrites the first, or the due-mode logic runs
    # a sanity check right after a pass / skips it after recheck_after_h.
    c = cal.config({})
    tmp = tempfile.mkdtemp(prefix="hinf_cal_")
    try:
        path = cal.lock_path(tmp)
        assert cal.load_lock(path) is None
        assert cal.mode_due(c, None, None) == "full"
        lock, _e = cal.compute_lock(_result(0.6, 0.6), c)
        cal.write_lock(path, lock)
        assert cal.load_lock(path)["epoch"] == lock["epoch"]
        lock2, _e = cal.compute_lock(_result(0.7, 0.7), c)
        try:
            cal.write_lock(path, lock2)
            raise AssertionError("second lock overwrote the first")
        except FileExistsError:
            pass
        assert cal.load_lock(path)["R_min_m"] == 0.6
        now = datetime.now(timezone.utc)
        cp = cal.checks_path(tmp)
        assert cal.mode_due(c, lock, cal.last_pass_utc(cp, lock["epoch"])) == "sanity"
        cal.append_check(cp, {"ok": True, "epoch": lock["epoch"],
                              "stamp_utc": now.isoformat()})
        cal.append_check(cp, {"ok": True, "epoch": "other",
                              "stamp_utc": (now + timedelta(hours=9)).isoformat()})
        last = cal.last_pass_utc(cp, lock["epoch"])
        assert abs((last - now).total_seconds()) < 1
        assert cal.mode_due(c, lock, last, now) is None
        assert cal.mode_due(c, lock, last, now + timedelta(hours=5)) == "sanity"
        assert cal.mode_due(dict(c, enabled=False), None, None) is None
    finally:
        shutil.rmtree(tmp)


def test_manifest_resolves_auto_radius():
    # Fails if 'auto' is not resolved from the lock, if no-lock does not give
    # an empty matrix, or the epoch is not exposed for the credit filter.
    tmp = tempfile.mkdtemp(prefix="hinf_cal_")
    try:
        exp, reps, doc = manifest.load_experiment(_YAML, bag_root=tmp)
        assert doc["matrix"]["radius_m"] == [] and exp == set()
        assert doc["_radius_auto"] and doc["_matrix_epoch"] is None
        lock, _e = cal.compute_lock(_result(0.62, 0.6), cal.config(doc))
        cal.write_lock(cal.lock_path(tmp), lock)
        exp, reps, doc = manifest.load_experiment(_YAML, bag_root=tmp)
        assert doc["matrix"]["radius_m"] == [1.95, 1.35, 0.95, 0.75]
        assert doc["_matrix_epoch"] == lock["epoch"]
        assert len(exp) == 2 * 2 * 2 * 4 and reps == 10
        assert ("pid", 1.0, "step", 0.75) in exp
    finally:
        shutil.rmtree(tmp)


# ----------------------------------------------------------------------
# Planner
# ----------------------------------------------------------------------

def _doc(radii):
    import yaml
    with open(_YAML) as f:
        d = yaml.safe_load(f)
    d["matrix"]["radius_m"] = list(radii)
    d["_radius_auto"] = True
    return d


def test_calibration_only_plan_fits_and_is_joinable():
    # Fails if the first-session plan (no lock: no radii) has no calibration
    # stage, the figure-8 + approach does not clear the loader gate, or some
    # robot heading has no forward-drivable join point on the approach (the
    # reposition node aborts then: "no forward-drivable join point").
    venue = _venue()
    plan = ep.plan_stages(venue, _doc([]), {}, calibration={"mode": "full"})
    assert plan["ok"], plan["notes"]
    assert [s["name"] for s in plan["stages"]] == ["calibration"]
    st = plan["stages"][0]
    assert st["experiments"][0]["recipe"]["type"] == "calib_fig8"
    assert st["experiments"][0]["recipe"]["params"]["speeds"] == [0.5, 1.0]
    legs = ep.stages_to_legs(plan["stages"], venue)[0]["legs"]
    ok, rep = venue_geom.check_legs_containment(legs, venue, 0.30, 0.30)
    assert ok, rep
    assert legs[0]["curves"][1]["scored"] is False
    # Join feasibility from the approach: for every robot heading there is a
    # segment whose direction is within 100 deg of the nose.
    wps = legs[0]["curves"][0]["waypoints_wgs84"]
    lat0, lon0 = wps[0]["lat"], wps[0]["lon"]
    en = [venue_geom.latlon_to_en(w["lat"], w["lon"], lat0, lon0) for w in wps]
    seg_b = [math.degrees(math.atan2(b[0] - a[0], b[1] - a[1])) % 360
             for a, b in zip(en[:-1], en[1:])]
    for h in range(0, 360, 15):
        assert any(abs((b - h + 180) % 360 - 180) <= 100 for b in seg_b), h
    # the approach ends at the pin with the pin heading
    pin = legs[0]["curves"][1]["start_pose"]
    e_end = venue_geom.latlon_to_en(wps[-1]["lat"], wps[-1]["lon"], lat0, lon0)
    e_pin = venue_geom.latlon_to_en(pin["lat"], pin["lon"], lat0, lon0)
    assert math.hypot(e_end[0] - e_pin[0], e_end[1] - e_pin[1]) < 0.01
    assert abs((seg_b[-1] - pin["heading_deg"] + 180) % 360 - 180) < 1.0


def test_replan_reseats_calibration_and_glues_into_matrix():
    # Fails if the post-lock re-plan moves the figure-8 (the robot is standing
    # at the driven pin) or does not plan the transit from it into stage 2.
    venue = _venue()
    p0 = ep.plan_stages(venue, _doc([]), {}, calibration={"mode": "full"})
    pin = p0["stages"][0]["experiments"][0]["start"]
    plan = ep.plan_stages(venue, _doc([1.95, 0.75]), {},
                          calibration={"mode": "full", "pin": pin})
    names = [s["name"] for s in plan["stages"]]
    assert names[0] == "calibration" and len(names) == 1 + 2 * 2, names
    assert plan["stages"][0]["experiments"][0]["start"] == pin
    eg = plan["stages"][1].get("entry_glue")
    assert eg and eg["from_stage"] == "calibration"
    wg = plan["stages"][1].get("wrap_glue")
    assert wg and wg["from_stage"] == names[-1]      # first-pass wrap
    for st in ep.stages_to_legs(plan["stages"], venue):
        ok, rep = venue_geom.check_legs_containment(st["legs"], venue, 0.30, 0.30)
        assert ok, (st["name"], rep)


def _poly_dist(p, poly):
    best = float("inf")
    for a, b in zip(poly[:-1], poly[1:]):
        best = min(best, venue_geom.dist_to_seg(p, a, b))
    return best


def test_stages_to_legs_matches_real_webui_js():
    # Fails if the executor's self-built batch differs from what the WebUI
    # would Send for the same plan: leg ids, curve names/kinds, recipes, start
    # poses, scored flags and glue geometry (verbatim glues identical; Bezier
    # glues within 2 cm — the two sample the same curve at different counts).
    node = shutil.which("node")
    if node is None:
        print("SKIP: node not available")
        return
    venue = _venue()
    p0 = ep.plan_stages(venue, _doc([]), {}, calibration={"mode": "full"})
    pin = p0["stages"][0]["experiments"][0]["start"]
    plan = ep.plan_stages(venue, _doc([1.35, 0.75]), {},
                          calibration={"mode": "full", "pin": pin})
    py = ep.stages_to_legs(plan["stages"], venue)
    tmp = tempfile.mkdtemp(prefix="hinf_webui_")
    try:
        pp, lp = os.path.join(tmp, "plan.json"), os.path.join(tmp, "legs.json")
        with open(pp, "w") as f:
            json.dump({"venue": venue, "plan": plan}, f)
        r = subprocess.run([node, _NODE_SCRIPT, _HTML, pp, lp],
                           capture_output=True, text=True, timeout=60)
        assert r.returncode == 0, r.stderr
        with open(lp) as f:
            js = json.load(f)
    finally:
        shutil.rmtree(tmp)
    assert [s["name"] for s in js] == [s["name"] for s in py]
    lat0 = venue["corners_wgs84"][0]["lat"]
    lon0 = venue["corners_wgs84"][0]["lon"]
    n_bezier = 0
    for sj, sp in zip(js, py):
        assert len(sj["legs"]) == len(sp["legs"])
        for lj, lpy in zip(sj["legs"], sp["legs"]):
            assert lj["id"] == lpy["id"]
            for cj, cp in zip(lj["curves"], lpy["curves"]):
                assert cj["kind"] == cp["kind"] and cj["name"] == cp["name"]
                if cj["kind"] == "recipe":
                    assert cj["recipe"] == cp["recipe"]
                    assert cj["scored"] == cp["scored"]
                    for k in ("lat", "lon", "heading_deg"):
                        assert abs(cj["start_pose"][k] - cp["start_pose"][k]) < 1e-9
                    continue
                assert abs(cj["end_heading_deg"] - cp["end_heading_deg"]) < 1e-9
                assert cj["v_const"] == cp["v_const"]
                wj = [venue_geom.latlon_to_en(w["lat"], w["lon"], lat0, lon0)
                      for w in cj["waypoints_wgs84"]]
                wp = [venue_geom.latlon_to_en(w["lat"], w["lon"], lat0, lon0)
                      for w in cp["waypoints_wgs84"]]
                if len(wj) == len(wp) and all(
                        math.hypot(a[0] - b[0], a[1] - b[1]) < 1e-6
                        for a, b in zip(wj, wp)):
                    continue                      # verbatim glue: identical
                n_bezier += 1
                for a, b in ((wj[0], wp[0]), (wj[-1], wp[-1])):
                    assert math.hypot(a[0] - b[0], a[1] - b[1]) < 0.01
                assert max(_poly_dist(q, wp) for q in wj) < 0.02
                assert max(_poly_dist(q, wj) for q in wp) < 0.02
    print(f"compared; {n_bezier} Bezier glue(s) matched within 2 cm")


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print("PASS", name)
