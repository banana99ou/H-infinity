# -*- coding: utf-8 -*-
"""The auto-planner on the venues actually captured in the field (fixtures
copied from the robot 2026-10-02). Slow (~25 s per venue): the full
experiment.yaml matrix, audited exactly like tools/analysis/plan_report.py.

The two smoke venues (usable width ~3 m) are too narrow for the full matrix by
geometry — a slalom pair alone is wider — so they are NOT expected to plan
clean; every roomy venue must.

    python3 tools/analysis/tests/test_saved_venues.py
"""
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, ".."))

import plan_report  # noqa: E402

ROOMY = ("active", "basketball_0921", "rooftop", "rooftop_0612_fri",
         "rooftop_shakedown")


def test_roomy_saved_venues_plan_clean_aligned_and_gentle():
    import yaml
    doc = yaml.safe_load(open(os.path.join(plan_report._REPO, "scenarios",
                                           "experiment.yaml")))
    for name in ROOMY:
        path = os.path.join(_HERE, "venue_fixtures", name + ".json")
        assert plan_report.report(path, doc), f"{name} did not plan clean"


if __name__ == "__main__":
    try:
        test_roomy_saved_venues_plan_clean_aligned_and_gentle()
        print("PASS test_roomy_saved_venues_plan_clean_aligned_and_gentle")
    except AssertionError as exc:
        print(f"FAIL: {exc}")
        sys.exit(1)
