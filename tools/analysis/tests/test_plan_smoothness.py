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
        assert eg is not None, \
            f"{st['name']}: entry glue missing (B 2026-06-16: never skip)"
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


def test_entry_glue_carries_editable_mids_and_start():
    """C: a bezier-mids inter-stage glue carries its START pin + editable control
    mids, and rebuilding the Bezier from [start, *mids, dest_start] (exactly
    what the WebUI does on Send) reproduces the waypoints the loader checks and
    lands on the next stage's first pin. At higher track_radius_m floors, all
    inter-stage glues may be Dubins-only on committed_rooftop — the round-trip
    assertion is skipped if no mids glue is produced."""
    venue, p = _plan_rooftop(families=("slalom",), radii=(1.0, 0.7))
    assert p["ok"], p["unfittable"] or p["notes"]
    lat0 = venue["corners_wgs84"][0]["lat"]
    lon0 = venue["corners_wgs84"][0]["lon"]
    checked = 0
    for st in p["stages"]:
        eg = st.get("entry_glue")
        if eg is None or "mids" not in eg:        # Dubins fallback: waypoints only
            continue
        assert "start_wgs84" in eg, "editable glue missing its start pin"
        start_en = venue_geom.latlon_to_en(eg["start_wgs84"]["lat"],
                                            eg["start_wgs84"]["lon"], lat0, lon0)
        mids_en = _glue_pts_en(eg["mids"], lat0, lon0)
        pin = st["experiments"][0]["start"]
        dest_en = venue_geom.latlon_to_en(pin["lat"], pin["lon"], lat0, lon0)
        rebuilt = ep._bezier_samples([start_en] + mids_en + [dest_en])
        have = _glue_pts_en(eg["waypoints_wgs84"], lat0, lon0)
        assert len(rebuilt) == len(have), "rebuild changed the sample count"
        worst = max(math.hypot(a[0] - b[0], a[1] - b[1])
                    for a, b in zip(rebuilt, have))
        assert worst < 1e-3, f"{st['name']}: rebuild drifts {worst:.4f} m"
        d = math.hypot(rebuilt[-1][0] - dest_en[0], rebuilt[-1][1] - dest_en[1])
        assert d < 0.05, f"{st['name']}: glue ends {d:.2f} m off the start pin"
        checked += 1
    if checked == 0:
        return  # all inter-stage glues are Dubins at this floor — round-trip N/A


def test_entry_glue_always_present_on_every_boundary():
    """B: no stage boundary is ever left without an inter-stage glue, and each
    one lands on the next stage's first start pin. Best-effort ones (if any)
    are flagged needs_fix, which forces the plan not-ok."""
    venue, p = _plan_rooftop(families=("slalom",), radii=(1.0, 0.7, 0.5))
    assert len(p["stages"]) >= 2, "need a multi-stage plan to test boundaries"
    lat0 = venue["corners_wgs84"][0]["lat"]
    lon0 = venue["corners_wgs84"][0]["lon"]
    for prev, st in zip(p["stages"], p["stages"][1:]):
        eg = st.get("entry_glue")
        assert eg is not None, f"{st['name']}: entry glue missing (B: never skip)"
        assert eg["from_stage"] == prev["name"]
        end = venue_geom.latlon_to_en(eg["waypoints_wgs84"][-1]["lat"],
                                      eg["waypoints_wgs84"][-1]["lon"], lat0, lon0)
        pin = venue_geom.latlon_to_en(st["experiments"][0]["start"]["lat"],
                                      st["experiments"][0]["start"]["lon"],
                                      lat0, lon0)
        assert math.hypot(end[0] - pin[0], end[1] - pin[1]) < 0.05
        if eg.get("needs_fix"):
            assert not p["ok"], "best-effort entry glue must force plan not-ok"


def _bez(ctrl, nseg):
    out = []
    for i in range(nseg + 1):
        t = i / nseg
        cur = list(ctrl)
        while len(cur) > 1:
            cur = [(a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)
                   for a, b in zip(cur[:-1], cur[1:])]
        out.append(cur[0])
    return out


def _all_glue_pts(st, venue, lat0, lon0):
    """Every glue in a stage as EN polylines (waypoints used directly, bezier
    mids reconstructed exactly like the WebUI lbBuiltLegs)."""
    exps, glues, m = st["experiments"], st["glues"], len(st["experiments"])
    out = []
    for gi, g in enumerate(glues):
        wps = g.get("waypoints")
        if wps:
            out.append([venue_geom.latlon_to_en(w["lat"], w["lon"], lat0, lon0)
                        for w in wps])
            continue
        prev, nxt = exps[gi], exps[(gi + 1) % m]
        pen = venue_geom.recipe_points_en(prev["recipe"], prev["start"], lat0, lon0)
        ctrl = ([(pen[-1][0], pen[-1][1])]
                + [venue_geom.latlon_to_en(w["lat"], w["lon"], lat0, lon0)
                   for w in g.get("mids", [])]
                + [venue_geom.latlon_to_en(nxt["start"]["lat"],
                                           nxt["start"]["lon"], lat0, lon0)])
        L = sum(math.hypot(b[0] - a[0], b[1] - a[1])
                for a, b in zip(ctrl[:-1], ctrl[1:]))
        out.append(_bez(ctrl, max(12, min(300, int(L / 0.3)))))
    return out


def test_self_intersects_helper():
    # interior crossing (seg (2,0)-(2,2) vs seg (1,1)-(3,1) at (2,1)); NOT the
    # first/last pair, which the helper skips by design (glue ends meet pins)
    assert ep._self_intersects(
        [(0, 0), (2, 0), (2, 2), (1, 2), (1, 1), (3, 1)]) is True
    pts, _fb = _arc(1.0, 120.0)
    assert ep._self_intersects(pts) is False


def test_gate_rejects_self_intersecting_glue():
    # 2x-scaled teardrop: radii all ~2 m (clear of the 1.0 m floor) but the
    # path crosses itself — the self-intersection check is the sole gate here.
    loop = [(0, 0), (3.0, 0.4), (4.4, 2.8), (3.0, 4.8), (0.8, 4.0),
            (0.4, 1.8), (2.0, 0.4), (4.0, 0.0), (6.0, 0.0)]
    ok, why = ep.check_glue_tracking(loop, 90.0, G)
    assert not ok and "self-intersect" in why, why


def test_planned_glues_never_self_intersect_on_rooftop():
    """C guarantee (field 2026-06-12): no planned reposition glue may loop
    back on itself — pure pursuit latches the wrong branch at the crossing."""
    venue, p = _plan_rooftop(radii=(1.0, 0.5))
    assert p["ok"], p["unfittable"] or p["notes"]
    lat0 = venue["corners_wgs84"][0]["lat"]
    lon0 = venue["corners_wgs84"][0]["lon"]
    for st in p["stages"]:
        for gi, pts in enumerate(_all_glue_pts(st, venue, lat0, lon0)):
            assert not ep._self_intersects(pts), \
                f"{st['name']} glue {gi} self-intersects"


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
