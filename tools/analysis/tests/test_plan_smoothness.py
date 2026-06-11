# -*- coding: utf-8 -*-
"""B+ smoothness gate (field 2026-06-11): planned glue must be TRACKABLE —
not merely chassis-legal — and must arrive on a straight, heading-aligned
tail (reposition converges heading through the tail). Also pins the C
inter-stage glue contract: every stage boundary either carries an
entry_glue whose waypoints meet the same gate and land on the next stage's
first start pin, or the plan says why not.

Run:  python3 tools/analysis/tests/test_plan_smoothness.py   (or pytest)
"""
import json
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

import experiment_planner as ep   # noqa: E402
import venue_geom                 # noqa: E402

G = ep.PLAN_DEFAULTS["glue"]


def _arc(R, sweep_deg, n=40, start_bearing=0.0):
    """Constant-radius arc turning right from a given start bearing, then a
    1.2 m straight tail along the final bearing. Returns (pts, final_bearing)."""
    pts = [(0.0, 0.0)]
    b = math.radians(start_bearing)
    step = math.radians(sweep_deg) / n
    e, n_ = 0.0, 0.0
    seg = abs(R * step)
    for _ in range(n):
        b += step
        e += seg * math.sin(b)
        n_ += seg * math.cos(b)
        pts.append((e, n_))
    fb = math.degrees(b) % 360.0
    fe, fn = math.sin(b), math.cos(b)
    for k in range(1, 7):
        pts.append((e + fe * 0.2 * k, n_ + fn * 0.2 * k))
    return pts, fb


def test_gate_rejects_tighter_than_trackable():
    pts, fb = _arc(0.45, 170.0)
    ok, why = ep.check_glue_tracking(pts, fb, G)
    assert not ok and "trackable" in why, why


def test_gate_accepts_gentle_arc_with_straight_tail():
    pts, fb = _arc(1.0, 170.0)
    ok, why = ep.check_glue_tracking(pts, fb, G)
    assert ok, why


def test_gate_rejects_bent_tail():
    """Gentle curvature everywhere, but the path arrives ~45 deg off the pin
    heading: position would be reached, heading would not."""
    pts, fb = _arc(1.5, 90.0)
    ok, why = ep.check_glue_tracking(pts, (fb + 45.0) % 360.0, G)
    assert not ok and "tail" in why, why


def _plan_rooftop(families=("step", "slalom"), radii=(0.5,)):
    venue = json.load(open(os.path.join(_REPO, "scenarios", "venues",
                                        "rooftop.json")))
    doc = {"matrix": {"radius_m": list(radii),
                      "controller": ["lpv-hinf"], "v_const": [0.5],
                      "path_family": list(families)},
           "repetitions": 1}
    return venue, ep.plan_stages(venue, doc, {})


def _glue_pts_en(wps, lat0, lon0):
    return [venue_geom.latlon_to_en(w["lat"], w["lon"], lat0, lon0)
            for w in wps]


def test_planned_glues_meet_trackable_floor_on_rooftop():
    venue, p = _plan_rooftop()
    assert p["ok"], p["unfittable"] or p["notes"]
    lat0 = venue["corners_wgs84"][0]["lat"]
    lon0 = venue["corners_wgs84"][0]["lon"]
    floor = max(float(G["track_radius_m"]), float(G["hard_radius_m"]))
    for st in p["stages"]:
        for gi, g in enumerate(st["glues"]):
            wps = g.get("waypoints")
            if not wps:        # Bezier mids: endpoints live on the exp curves
                continue
            kmax = ep._max_curvature(_glue_pts_en(wps, lat0, lon0))
            r = 1.0 / max(kmax, 1e-9)
            assert r >= floor - 0.02, \
                f"{st['name']} glue {gi}: min radius {r:.2f} < {floor:.2f}"
        eg = st.get("entry_glue")
        if eg:
            kmax = ep._max_curvature(
                _glue_pts_en(eg["waypoints_wgs84"], lat0, lon0))
            r = 1.0 / max(kmax, 1e-9)
            assert r >= floor - 0.02, \
                f"{st['name']} entry glue: min radius {r:.2f} < {floor:.2f}"


def test_entry_glue_lands_on_next_stage_start_pin():
    venue, p = _plan_rooftop()
    assert p["ok"]
    if len(p["stages"]) < 2:
        return                      # single stage: nothing to bridge
    lat0 = venue["corners_wgs84"][0]["lat"]
    lon0 = venue["corners_wgs84"][0]["lon"]
    for prev, st in zip(p["stages"], p["stages"][1:]):
        eg = st.get("entry_glue")
        if eg is None:
            # Allowed only when the plan SAYS the boundary has no glue.
            assert any("no inter-stage glue" in n and st["name"] in n
                       for n in p["notes"]), \
                f"{st['name']}: entry glue silently missing"
            continue
        assert eg["from_stage"] == prev["name"]
        end = venue_geom.latlon_to_en(eg["waypoints_wgs84"][-1]["lat"],
                                      eg["waypoints_wgs84"][-1]["lon"],
                                      lat0, lon0)
        pin = st["experiments"][0]["start"]
        pin_en = venue_geom.latlon_to_en(pin["lat"], pin["lon"], lat0, lon0)
        d = math.hypot(end[0] - pin_en[0], end[1] - pin_en[1])
        assert d < 0.05, (f"{st['name']}: entry glue ends {d:.2f}m from the "
                          "first start pin")
        assert abs(eg["end_heading_deg"] - pin["heading_deg"]) < 0.6


def test_stages_are_opposed_ab_pairs():
    """A: each stage = ONE geometry placed twice, headings ~180 deg apart
    (directional-bias removal), glued as a racetrack. Single-curve stages are
    allowed only with an explicit fallback note."""
    venue, p = _plan_rooftop()
    assert p["ok"]
    for st in p["stages"]:
        assert len(st["geometries"]) == 1, "stage mixes geometries"
        exps = st["experiments"]
        assert len(exps) <= 2
        if len(exps) == 1:
            geom = st["geometries"][0]
            assert any("single curve" in n and f"R{geom['R']}" in n
                       for n in p["notes"]), \
                f"{st['name']}: silent single-curve stage"
            continue
        assert exps[0]["recipe"] == exps[1]["recipe"], \
            "A/B legs must be the SAME treatment geometry"
        d = abs((exps[0]["start"]["heading_deg"]
                 - exps[1]["start"]["heading_deg"] + 180.0) % 360.0 - 180.0)
        assert d > 120.0, f"{st['name']}: pair only {d:.0f} deg apart"
        assert len(st["glues"]) == 2


def test_entry_glue_passes_loader_containment():
    venue, p = _plan_rooftop()
    assert p["ok"]
    checked = 0
    for st in p["stages"]:
        eg = st.get("entry_glue")
        if not eg:
            continue
        legs = [{"id": "entry", "curves": [{
            "kind": "reposition", "name": "entry_glue",
            "waypoints_wgs84": eg["waypoints_wgs84"]}]}]
        ok, report = venue_geom.check_legs_containment(legs, venue, 0.30, 0.30)
        assert ok, f"{st['name']} entry glue rejected by the gate:\n{report}"
        checked += 1
    if len(p["stages"]) >= 2:
        assert checked >= 1, "multi-stage plan produced no checkable entry glue"


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
