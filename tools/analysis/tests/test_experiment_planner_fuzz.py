# -*- coding: utf-8 -*-
"""Fuzz the planner over randomized venues (fixed seed -> deterministic).

Field reality is a surveyed polygon of arbitrary shape with ad-hoc exclusions
and a half-finished manifest. The planner must NEVER raise, must terminate,
and must ACCOUNT for every remaining geometry: each one either appears in
exactly one returned stage or is listed in ``unfittable`` with a reason —
silent loss is how a 320-run matrix quietly becomes a 280-run paper.
Every returned stage must also pass the loader's containment gate.

Run:  python3 tools/analysis/tests/test_experiment_planner_fuzz.py   (or pytest)
"""
import math
import multiprocessing as mp
import os
import random
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

FOOT, TRACK = 0.30, 0.30
_M = 1.0 / 111320.0

# A/B-pair placement made plan_stages ~10-30x dearer than the single-curve
# packer, so cases run in a process pool. Inputs are pre-generated from the
# seeded rng in one sequential pass (identical draw order to the old serial
# loop), so the suite stays deterministic.
N_CASES = 60
SEED = 20260611


def _ll(e, n):
    return {"lat": n * _M, "lon": e * _M}


def _doc():
    return {
        "matrix": {
            "radius_m": [1.0, 0.7, 0.5, 0.4],
            "controller": ["lpv-hinf", "pid"],
            "v_const": [1.0, 0.5],
            "path_family": ["step", "slalom"],
        },
        "repetitions": 10,
    }


def _rand_venue(rng):
    """Convex-ish polygon: points on a randomly squashed/rotated ellipse."""
    n = rng.choice([4, 4, 5, 6])
    a = rng.uniform(5.0, 22.0)        # semi-axes (m)
    b = rng.uniform(3.0, 14.0)
    rot = rng.uniform(0, 2 * math.pi)
    cx, cy = rng.uniform(-5, 5), rng.uniform(-5, 5)
    corners = []
    for i in range(n):
        t = 2 * math.pi * i / n + rng.uniform(-0.25, 0.25)
        x = a * math.cos(t)
        y = b * math.sin(t)
        e = cx + x * math.cos(rot) - y * math.sin(rot)
        nn = cy + x * math.sin(rot) + y * math.cos(rot)
        corners.append(_ll(e, nn))
    excl = []
    for _ in range(rng.randint(0, 2)):
        excl.append({"kind": "circle",
                     **_ll(cx + rng.uniform(-a / 2, a / 2),
                           cy + rng.uniform(-b / 2, b / 2)),
                     "radius_m": rng.uniform(0.3, 2.0)})
    return {"name": "fuzz", "safety_margin_m": rng.choice([0.2, 0.5]),
            "corners_wgs84": corners, "exclusions": excl}


def _rand_counts(rng, doc):
    counts = {}
    m = doc["matrix"]
    for fam in m["path_family"]:
        for R in m["radius_m"]:
            for c in m["controller"]:
                for v in m["v_const"]:
                    if rng.random() < 0.5:
                        counts[ep._cell_key(fam, R, c, v)] = rng.randint(0, 10)
    return counts


def _pool():
    # fork: workers inherit the imported modules; spawn (macOS default)
    # would re-import this file as __main__ and re-run the suite.
    return mp.get_context("fork").Pool(max(1, (os.cpu_count() or 2) - 1))


def _check_accounting_case(args):
    case, venue, counts = args
    doc = _doc()
    failures = []
    try:
        plan = ep.plan_stages(venue, doc, counts, FOOT, TRACK)
    except Exception as exc:  # noqa: BLE001 - that IS the test
        return [f"case {case}: plan_stages raised {exc!r}"]
    remaining = {(f, R) for (f, R, _n)
                 in ep.remaining_geometries(doc, counts)}
    staged = [(g["family"], g["R"]) for st in plan["stages"]
              for g in st["geometries"]]
    unfit = {(u["family"], u["R"]) for u in plan["unfittable"]}
    if len(staged) != len(set(staged)):
        failures.append(f"case {case}: geometry planned twice: {staged}")
    accounted = set(staged) | unfit
    if accounted != remaining:
        failures.append(
            f"case {case}: accounting broken — remaining {sorted(remaining)}"
            f" vs staged {sorted(set(staged))} + unfit {sorted(unfit)}")
    for u in plan["unfittable"]:
        if not u.get("reason"):
            failures.append(f"case {case}: unfittable without reason")
    for st in plan["stages"]:
        if len(st["glues"]) != len(st["experiments"]):
            failures.append(
                f"case {case}: {st['name']} glue/exp count mismatch")
    return failures


def test_fuzz_never_raises_and_accounts_for_every_geometry():
    rng = random.Random(SEED)
    doc = _doc()
    cases = [(i, _rand_venue(rng), _rand_counts(rng, doc))
             for i in range(N_CASES)]
    with _pool() as pool:
        results = pool.map(_check_accounting_case, cases)
    hard_failures = [f for fs in results for f in fs]
    assert not hard_failures, "\n".join(hard_failures[:10])


def _check_containment_case(args):
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from test_experiment_planner import _legs_from_stage
    case, venue = args
    plan = ep.plan_stages(venue, _doc(), {}, FOOT, TRACK)
    failures, checked = [], 0
    for st in plan["stages"]:
        ok, report = venue_geom.check_legs_containment(
            _legs_from_stage(st, venue), venue, FOOT, TRACK)
        if not ok:
            failures.append(f"case {case} {st['name']} rejected:\n{report}")
        checked += 1
    return failures, checked


def test_fuzz_staged_output_passes_loader_gate():
    # Containment equivalence on a subset (it is the expensive half).
    rng = random.Random(SEED + 1)
    cases = [(i, _rand_venue(rng)) for i in range(20)]
    with _pool() as pool:
        results = pool.map(_check_containment_case, cases)
    failures = [f for fs, _c in results for f in fs]
    assert not failures, "\n".join(failures[:5])
    assert sum(c for _fs, c in results) > 0, \
        "fuzz never produced a stage — generator too hostile"


if __name__ == "__main__":
    sys.path.insert(0, os.path.dirname(__file__))
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
