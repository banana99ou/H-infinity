# -*- coding: utf-8 -*-
"""Cross-language equivalence: planner -> REAL WebUI JS -> loader gate.

The WebUI mirrors the NUC's geometry in JavaScript (lbStepRecipeLocal,
lbSlalomRecipeLocal, lbPlaceAtStartPose, lbBuiltLegs). Mirror drift is exactly
how the heading-convention bug class happens, so this test runs the planner in
Python, assembles the legs with the ACTUAL functions extracted from
interactive.html (executed by node), and feeds the result back into
venue_geom.check_legs_containment. Skips cleanly when node is unavailable.

Run:  python3 tools/analysis/tests/test_webui_assembly.py   (or pytest)
"""
import json
import os
import shutil
import subprocess
import sys
import tempfile

_REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
_PKG = os.path.join(_REPO, "scalecar-vfg-h-infinite", "ros2_bridge",
                    "limo_path_follower")
_VFG = os.path.join(_REPO, "scalecar-vfg-h-infinite")
for p in (_PKG, _VFG):
    if p not in sys.path:
        sys.path.insert(0, p)

import experiment_planner as ep   # noqa: E402
import venue_geom                 # noqa: E402

_HTML = os.path.join(_REPO, "tools", "path_gen", "interactive.html")
_NODE_SCRIPT = os.path.join(os.path.dirname(__file__), "webui_assemble.js")


def test_planner_through_real_webui_js_passes_loader_gate():
    node = shutil.which("node")
    if node is None:
        print("SKIP: node not available")
        return
    import yaml
    with open(os.path.join(_REPO, "scenarios", "venues", "rooftop.json")) as f:
        venue = json.load(f)
    with open(os.path.join(_REPO, "scenarios", "experiment.yaml")) as f:
        doc = yaml.safe_load(f)
    plan = ep.plan_stages(venue, doc, {})
    assert plan["ok"], plan

    tmp = tempfile.mkdtemp(prefix="hinf_webui_")
    plan_path = os.path.join(tmp, "plan.json")
    legs_path = os.path.join(tmp, "legs.json")
    with open(plan_path, "w") as f:
        json.dump({"venue": venue, "plan": plan}, f)
    r = subprocess.run(
        [node, _NODE_SCRIPT, _HTML, plan_path, legs_path],
        capture_output=True, text=True, timeout=60)
    assert r.returncode == 0, f"node assembly failed:\n{r.stderr}"
    with open(legs_path) as f:
        stages = json.load(f)
    assert len(stages) == len(plan["stages"])
    for st in stages:
        for lg in st["legs"]:
            kinds = [c["kind"] for c in lg["curves"]]
            assert kinds == ["reposition", "recipe"], kinds
            assert len(lg["curves"][0]["waypoints_wgs84"]) >= 2
        ok, report = venue_geom.check_legs_containment(
            st["legs"], venue, 0.30, 0.30)
        assert ok, f"{st['name']} (assembled by REAL webui JS) rejected:\n{report}"


if __name__ == "__main__":
    try:
        test_planner_through_real_webui_js_passes_loader_gate()
        print("PASS test_planner_through_real_webui_js_passes_loader_gate")
    except AssertionError as exc:
        print(f"FAIL: {exc}")
        sys.exit(1)
