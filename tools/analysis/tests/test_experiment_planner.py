# -*- coding: utf-8 -*-
"""Unit tests for experiment_planner (auto-layout of curves + glue).

The make-or-break property: a plan assembled into legs EXACTLY the way the
WebUI leg-batch editor assembles them must pass venue_geom's containment
gate (the same gate venue_loader and run_executor enforce). If this holds,
auto-plan -> Send -> Start can never be rejected for geometry.

Pure python + numpy (vfg path classes), no ROS. Run either way:

    python3 tools/analysis/tests/test_experiment_planner.py
    python3 -m pytest tools/analysis/tests/test_experiment_planner.py
"""
import json
import math
import os
import sys
import time

_REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
_PKG = os.path.join(_REPO, "scalecar-vfg-h-infinite", "ros2_bridge",
                    "limo_path_follower")
_VFG = os.path.join(_REPO, "scalecar-vfg-h-infinite")
_TOOLS = os.path.join(_REPO, "tools", "analysis")
for p in (_PKG, _VFG, _TOOLS):
    if p not in sys.path:
        sys.path.insert(0, p)

import experiment_planner as ep   # noqa: E402
import venue_geom                 # noqa: E402
import manifest                   # noqa: E402

FOOT, TRACK = 0.30, 0.30

_M = 1.0 / 111320.0


def _ll(e, n):
    return {"lat": n * _M, "lon": e * _M}


def _doc(**plan):
    d = {
        "matrix": {
            "radius_m": [1.0, 0.7, 0.5, 0.4],
            "controller": ["lpv-hinf", "pid"],
            "v_const": [1.0, 0.5],
            "path_family": ["step", "slalom"],
        },
        "repetitions": 10,
    }
    if plan:
        d["plan"] = plan
    return d


def _rooftop():
    path = os.path.join(_REPO, "scenarios", "venues", "rooftop.json")
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def _big_venue(exclusions=None):
    return {
        "name": "big", "safety_margin_m": 0.5,
        "corners_wgs84": [_ll(0, 0), _ll(30, 0), _ll(30, 30), _ll(0, 30)],
        "exclusions": exclusions or [],
    }


# ---- WebUI leg assembly mirror (lbBuiltLegs + lbRepoSamples) --------------

def _bezier_latlon(ctrl, n):
    pts = [(c["lat"], c["lon"]) for c in ctrl]
    out = []
    for i in range(n + 1):
        t = i / n
        cur = list(pts)
        while len(cur) > 1:
            cur = [(a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)
                   for a, b in zip(cur[:-1], cur[1:])]
        out.append(cur[0])
    return [{"lat": p[0], "lon": p[1]} for p in out]


def _exp_end_latlon(exp, venue):
    lat0 = venue["corners_wgs84"][0]["lat"]
    lon0 = venue["corners_wgs84"][0]["lon"]
    pts = venue_geom.recipe_points_en(exp["recipe"], exp["start"], lat0, lon0)
    e, n, _s = pts[-1]
    mlat = 111320.0
    mlon = 111320.0 * math.cos(math.radians(lat0))
    return {"lat": lat0 + n / mlat, "lon": lon0 + e / mlon}


def _legs_from_stage(stage, venue):
    """Mirror of the WebUI's lbBuiltLegs: leg i = [glue arriving at exp i,
    recipe exp i]; glue gap g connects exp[g].end -> exp[g+1 mod n].start."""
    exps = stage["experiments"]
    glues = stage["glues"]
    n = len(exps)
    legs = []
    for i, e in enumerate(exps):
        gap = (i - 1 + n) % n
        g = glues[gap]
        if g.get("waypoints"):
            wps = g["waypoints"]
        else:
            prev_end = _exp_end_latlon(exps[gap], venue)
            ctrl = ([prev_end] + g.get("mids", [])
                    + [{"lat": e["start"]["lat"], "lon": e["start"]["lon"]}])
            length = 0.0
            for a, b in zip(ctrl[:-1], ctrl[1:]):
                length += math.hypot((b["lat"] - a["lat"]) * 111320.0,
                                     (b["lon"] - a["lon"]) * 111320.0
                                     * math.cos(math.radians(a["lat"])))
            nseg = max(12, min(300, int(length / 0.3)))
            wps = _bezier_latlon(ctrl, nseg)
        legs.append({"id": f"leg_{i + 1}", "curves": [
            {"name": f"repo_to_{e['id']}", "kind": "reposition",
             "waypoints_wgs84": wps,
             "end_heading_deg": e["start"]["heading_deg"],
             "v_const": g["v_const"], "pos_tol_m": g["pos_tol_m"]},
            {"name": f"{e['id']}_x", "kind": "recipe", "scored": True,
             "start_pose": e["start"], "recipe": e["recipe"]},
        ]})
    return legs


# ---- tests ----------------------------------------------------------------

def test_full_matrix_plans_and_passes_loader_gate_on_rooftop():
    venue = _rooftop()
    t0 = time.time()
    plan = ep.plan_stages(venue, _doc(), {}, FOOT, TRACK)
    # Planning is a Send-time operation (once per field day), but the
    # executor's tick blocks while it runs — keep it bounded. A/B pair
    # placement (2-3 glue plans per stage + failures) raised the floor over
    # the old single-curve packing: ~60 s quiet, ~80 s under load
    # (measured 2026-06-11). This guards against runaway, not slowness.
    assert time.time() - t0 < 150.0, "planner runaway (was ~60-80 s)"
    assert plan["ok"], plan
    assert not plan["unfittable"], plan["unfittable"]
    fams = [(g["family"], g["R"]) for st in plan["stages"]
            for g in st["geometries"]]
    assert sorted(fams) == sorted(
        [(f, r) for f in ("step", "slalom") for r in (1.0, 0.7, 0.5, 0.4)])
    for st in plan["stages"]:
        legs = _legs_from_stage(st, venue)
        ok, report = venue_geom.check_legs_containment(
            legs, venue, FOOT, TRACK)
        assert ok, f"{st['name']} rejected by the loader gate:\n{report}"


def test_remaining_geometries_counts_and_order():
    doc = _doc()
    counts = {}
    # Fill step R1.0 completely, slalom R0.4 half-way.
    for c in ("lpv-hinf", "pid"):
        for v in (1.0, 0.5):
            counts[manifest.cell_key({"controller": c, "v_const": v,
                                      "path_family": "step",
                                      "radius_m": 1.0})] = 10
            counts[manifest.cell_key({"controller": c, "v_const": v,
                                      "path_family": "slalom",
                                      "radius_m": 0.4})] = 5
    rem = ep.remaining_geometries(doc, counts, key_fn=manifest.cell_key)
    d = {(f, r): n for (f, r, n) in rem}
    assert ("step", 1.0) not in d
    assert d[("slalom", 0.4)] == 20
    assert d[("step", 0.7)] == 40


def test_planner_keys_match_manifest_cell_key():
    # The planner's local fallback must agree with the real manifest.cell_key
    # (run_executor passes counts keyed by the real one).
    for fam in ("step", "slalom"):
        for R in (1.0, 0.7, 0.5, 0.4):
            for c in ("lpv-hinf", "pid"):
                for v in (1.0, 0.5):
                    assert ep._cell_key(fam, R, c, v) == manifest.cell_key(
                        {"controller": c, "v_const": v, "path_family": fam,
                         "radius_m": R})


def test_complete_matrix_yields_empty_plan():
    counts = {}
    for fam in ("step", "slalom"):
        for R in (1.0, 0.7, 0.5, 0.4):
            for c in ("lpv-hinf", "pid"):
                for v in (1.0, 0.5):
                    counts[ep._cell_key(fam, R, c, v)] = 10
    plan = ep.plan_stages(_rooftop(), _doc(), counts, FOOT, TRACK)
    assert plan["ok"] and not plan["stages"]
    assert any("complete" in n for n in plan["notes"])


def test_determinism():
    venue = _rooftop()
    p1 = ep.plan_stages(venue, _doc(), {}, FOOT, TRACK)
    p2 = ep.plan_stages(venue, _doc(), {}, FOOT, TRACK)
    assert json.dumps(p1, sort_keys=True) == json.dumps(p2, sort_keys=True)


def test_exclusion_is_respected():
    # A fat exclusion disc in the middle of a big venue: plans must avoid it
    # (every stage still passes the loader gate, which checks exclusions).
    venue = _big_venue(exclusions=[
        {"kind": "circle", "lat": _ll(15, 15)["lat"],
         "lon": _ll(15, 15)["lon"], "radius_m": 4.0}])
    plan = ep.plan_stages(venue, _doc(), {}, FOOT, TRACK)
    assert plan["ok"], plan["unfittable"] or plan["notes"]
    for st in plan["stages"]:
        legs = _legs_from_stage(st, venue)
        ok, report = venue_geom.check_legs_containment(
            legs, venue, FOOT, TRACK)
        assert ok, f"{st['name']} violates exclusion:\n{report}"


def test_unfittable_surfaced_not_silent():
    # A 6x6 m postage stamp cannot host a step R1.0 with 1.1 m clearance.
    # The planner must SURFACE that — as a needs_fix best-effort stage the
    # operator drags in, or in unfittable — never silently shrink R or drop
    # the geometry, and the plan is not "ok" until it is fixed.
    tiny = {"name": "tiny", "safety_margin_m": 0.5,
            "corners_wgs84": [_ll(0, 0), _ll(6, 0), _ll(6, 6), _ll(0, 6)]}
    plan = ep.plan_stages(tiny, _doc(), {}, FOOT, TRACK)
    fix_stages = [st for st in plan["stages"] if st.get("needs_fix")]
    assert fix_stages or plan["unfittable"], \
        "tiny venue should defeat at least one geometry"
    assert not plan["ok"], "best-effort/unfittable geometry => plan not ok"
    for st in fix_stages:
        assert st.get("fix_reason"), "best-effort stage must explain itself"
        # Still a real, editable A/B pair: 2 experiments, 2 glues, valid pins.
        assert len(st["glues"]) == len(st["experiments"]) >= 1
        for e in st["experiments"]:
            assert "lat" in e["start"] and "lon" in e["start"]
    for u in plan["unfittable"]:
        assert u["reason"]


def test_best_effort_keeps_every_geometry_in_the_editor():
    # No geometry may silently vanish on a cramped venue: every remaining
    # (family, R) is still reachable in the editor — a clean stage, a
    # needs_fix best-effort stage, or an unfittable note. (Drop = a 320-run
    # matrix quietly becomes fewer.)
    tiny = {"name": "tiny", "safety_margin_m": 0.5,
            "corners_wgs84": [_ll(0, 0), _ll(6, 0), _ll(6, 6), _ll(0, 6)]}
    plan = ep.plan_stages(tiny, _doc(), {}, FOOT, TRACK)
    staged = {(g["family"], g["R"]) for st in plan["stages"]
              for g in st["geometries"]}
    unfit = {(u["family"], u["R"]) for u in plan["unfittable"]}
    remaining = {(f, R) for (f, R, _n) in ep.remaining_geometries(_doc(), {})}
    assert staged | unfit == remaining, "a geometry vanished from the plan"
    assert any(st.get("needs_fix") for st in plan["stages"]), \
        "a 6x6 venue should force at least one best-effort stage"


def test_dubins_endpoint_verification():
    cases = [((0, 0, 0), (3, 0, 0)), ((0, 0, 0), (-3.4, 1.5, 0)),
             ((0, 0, 0), (0, 3, math.pi)), ((0, 0, 0), (0.5, 0.2, 0.3))]
    for (a, b) in cases:
        for R in (0.45, 0.7, 1.0):
            for (_l, pts) in ep._dubins_paths(a, b, R):
                assert math.hypot(pts[-1][0] - b[0], pts[-1][1] - b[1]) < 0.06
                assert len(pts) >= 2


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
