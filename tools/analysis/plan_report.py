#!/usr/bin/env python3
"""plan_report — run the auto-planner over saved venues and audit every plan.

For each venue (default: every fixture in tools/analysis/tests/venue_fixtures)
plans the experiment.yaml matrix exactly as the run_executor would, then checks
what the robot will actually be handed:

  - ok / unfittable / needs_fix (stage or inter-stage transit glue)
  - every stage's legs through venue_geom.check_legs_containment — the SAME
    gate venue_loader and run_executor enforce on Send / Start
  - experiment headings vs the venue walls (deg off the nearest wall direction)
  - glue smoothness: tightest turn radius and the settled straight tail before
    each start pin (heading within 5 deg) — the arrival-heading drivers
  - planning time (the executor's tick blocks while it plans)

Exit status 1 if any venue fails the loader gate or is not ok, so it can gate
a push. Pure python, no ROS:

    python3 tools/analysis/plan_report.py [venue.json ...] [--experiment PATH]
"""
import argparse
import glob
import json
import math
import os
import sys
import time

_REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
for p in (os.path.join(_REPO, "scalecar-vfg-h-infinite", "ros2_bridge", "limo_path_follower"),
          os.path.join(_REPO, "scalecar-vfg-h-infinite"),
          os.path.join(_REPO, "tools", "analysis", "tests")):
    if p not in sys.path:
        sys.path.insert(0, p)

import yaml                                                   # noqa: E402
import experiment_planner as ep                               # noqa: E402
import venue_geom                                             # noqa: E402
from test_experiment_planner import _legs_from_stage          # noqa: E402

FOOT, TRACK = 0.30, 0.30


def _glue_stats(leg, venue):
    """(min radius, settled tail) of a leg's reposition curve."""
    rep = leg["curves"][0]
    wps = rep["waypoints_wgs84"]
    lat0, lon0 = venue["corners_wgs84"][0]["lat"], venue["corners_wgs84"][0]["lon"]
    pts = [venue_geom.latlon_to_en(w["lat"], w["lon"], lat0, lon0) for w in wps]
    k = ep._max_curvature(pts)
    return (1.0 / k if k > 1e-9 else math.inf,
            ep._tail_straight(pts, float(rep["end_heading_deg"]), 5.0))


def report(path, doc):
    venue = json.load(open(path))
    c = venue["corners_wgs84"]
    beta = ep._venue_rect(venue_geom.poly_en(c, c[0]["lat"], c[0]["lon"]))[0]
    t0 = time.time()
    plan = ep.plan_stages(venue, doc, {}, FOOT, TRACK)
    dt = time.time() - t0
    rejects, radii, tails, offs = [], [], [], []
    for st in plan["stages"]:
        legs = _legs_from_stage(st, venue)
        ok, _rep = venue_geom.check_legs_containment(legs, venue, FOOT, TRACK)
        if not ok:
            rejects.append(st["name"])
        for leg in legs:
            r, t = _glue_stats(leg, venue)
            radii.append(r)
            tails.append(t)
        for e in st["experiments"]:
            off = (e["start"]["heading_deg"] - beta) % 90.0
            offs.append(min(off, 90.0 - off))
    fix = [st["name"] for st in plan["stages"] if st.get("needs_fix")]
    tfix = [st["name"] for st in plan["stages"]
            if (st.get("entry_glue") or {}).get("needs_fix")]
    pairs = sum(len(st["experiments"]) == 2 for st in plan["stages"])
    good = plan["ok"] and not rejects
    print(f"{'PASS' if good else 'FAIL'} {os.path.basename(path):24s} {dt:5.1f}s "
          f"stages {len(plan['stages']):2d} pairs {pairs:2d}  gate_rejects {len(rejects)} "
          f"needs_fix {len(fix)} transit_fix {len(tfix)} unfittable {len(plan['unfittable'])}  "
          f"off-wall max {max(offs, default=0):4.1f} deg  "
          f"glue R min {min(radii, default=0):4.2f} med {sorted(radii)[len(radii) // 2] if radii else 0:4.2f} m  "
          f"tail min {min(tails, default=0):3.1f} med {sorted(tails)[len(tails) // 2] if tails else 0:3.1f} m")
    for name in rejects + fix + tfix:
        print(f"      problem stage: {name}")
    for u in plan["unfittable"]:
        print(f"      unfittable: {u}")
    return good


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("venues", nargs="*")
    ap.add_argument("--experiment", default=os.path.join(_REPO, "scenarios", "experiment.yaml"))
    ap.add_argument("--radii", default=None,
                    help="comma-separated radii overriding matrix.radius_m "
                         "(e.g. a preview of the calibration lock's radii when "
                         "radius_m is 'auto' and no lock is present)")
    a = ap.parse_args(argv)
    doc = yaml.safe_load(open(a.experiment))
    if a.radii:
        doc.setdefault("matrix", {})["radius_m"] = [float(x) for x in a.radii.split(",")]
    elif str((doc.get("matrix") or {}).get("radius_m", "")).strip().lower() == "auto":
        sys.path.insert(0, os.path.join(_REPO, "tools", "analysis"))
        import manifest  # noqa: E402
        _e, _r, doc = manifest.load_experiment(a.experiment)
        if not doc["matrix"]["radius_m"]:
            print("radius_m is 'auto' and no calibration lock is present — "
                  "pass --radii to preview a matrix")
    venues = a.venues or sorted(glob.glob(os.path.join(
        _REPO, "tools", "analysis", "tests", "venue_fixtures", "*.json")))
    results = [report(v, doc) for v in venues]
    print(f"\n{sum(results)}/{len(results)} venues plan clean")
    return 0 if all(results) else 1


if __name__ == "__main__":
    sys.exit(main())
