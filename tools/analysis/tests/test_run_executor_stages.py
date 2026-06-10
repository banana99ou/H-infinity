# -*- coding: utf-8 -*-
"""Unit tests for run_executor_node multi-stage selection / advancement.

run_executor imports rclpy (robot-only), so the stage logic under test is
extracted from the source by AST and exec'd standalone — same approach as
test_reposition_join. Covers: Start selects the first stage with remaining
treatments (manifest-driven resume), stage auto-advance containment-gates
the next stage, a violating stage pauses instead of driving, and a fully
complete plan reports no work.

Run:  python3 tools/analysis/tests/test_run_executor_stages.py   (or pytest)
"""
import ast
import math
import os
import sys

_REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
_PKG = os.path.join(_REPO, "scalecar-vfg-h-infinite", "ros2_bridge",
                    "limo_path_follower")
_VFG = os.path.join(_REPO, "scalecar-vfg-h-infinite")
_TOOLS = os.path.join(_REPO, "tools", "analysis")
for p in (_PKG, _VFG, _TOOLS):
    if p not in sys.path:
        sys.path.insert(0, p)

import manifest      # noqa: E402
import venue_geom    # noqa: E402

SRC = os.path.join(_PKG, "run_executor_node.py")

_METHODS = {
    "_select_stage_with_work", "_advance_stage", "_all_geometries_done",
    "_all_leg_lists", "_stage_name", "_next_treatment_for", "_geometry_of",
    "_recipe_curve", "_cell_key_for", "_scored_geometries", "_runs_done",
    "_runs_target",
}


def _extract():
    with open(SRC, encoding="utf-8") as f:
        tree = ast.parse(f.read())
    ns = {"math": math, "manifest": manifest, "venue_geom": venue_geom,
          "_notify_discord": lambda *_a, **_k: False}
    for node in tree.body:
        if isinstance(node, ast.ClassDef) and node.name == "RunExecutor":
            for sub in node.body:
                if isinstance(sub, ast.FunctionDef) and sub.name in _METHODS:
                    exec(compile(ast.Module([sub], []), SRC, "exec"), ns)
    missing = _METHODS - set(ns)
    assert not missing, f"methods not found in source: {missing}"
    return ns


NS = _extract()

_M = 1.0 / 111320.0


def _ll(e, n):
    return {"lat": n * _M, "lon": e * _M}


def _venue():
    return {"name": "t", "safety_margin_m": 0.2,
            "corners_wgs84": [_ll(0, 0), _ll(30, 0), _ll(30, 30), _ll(0, 30)]}


def _leg(fam, R, e=15, n=15):
    p = _ll(e, n)
    params = {"R": R, "L1": 1.0, "L2": 1.0, "theta_arc": math.pi / 2,
              "direction": 1} if fam == "step" else \
             {"R": R, "L1": 1.0, "L_mid": 0.5, "n_arcs": 2, "L_end": 1.0,
              "theta_arc": math.pi / 2}
    return {"id": f"leg_{fam}_{R}", "curves": [{
        "name": f"{fam}_{R}", "kind": "recipe", "scored": True,
        "start_pose": {"lat": p["lat"], "lon": p["lon"], "heading_deg": 90.0},
        "recipe": {"type": fam, "params": params}}]}


class _Log:
    def info(self, *_a, **_k): pass
    def warn(self, *_a, **_k): pass
    def error(self, *_a, **_k): pass


class Exec:
    """Stand-in carrying exactly the state the extracted methods touch."""

    def __init__(self, stages, counts, n=2):
        self._stages = stages
        self._stage_idx = 0
        self._legs = stages[0]["legs"] if stages else []
        self._controllers = ["lpv-hinf", "pid"]
        self._speeds = [1.0]
        self._target_n = n
        self._max_retries = 2
        self._completed_counts = counts
        self._attempts = {}
        self._venue = _venue()
        self._footprint_r = 0.30
        self._track_margin = 0.30
        self._leg_idx = 0
        self._curve_idx = 0
        self.paused = None
        self.status_msgs = []

    def get_logger(self):
        return _Log()

    def _pause(self, reason):
        self.paused = reason

    def _publish_status(self, message=""):
        self.status_msgs.append(message)

    def __getattr__(self, name):
        if name in NS:
            return NS[name].__get__(self, Exec)
        raise AttributeError(name)


def _key(fam, R, c, v=1.0):
    return manifest.cell_key({"controller": c, "v_const": v,
                              "path_family": fam, "radius_m": R})


def _two_stages():
    return [{"name": "stage_1", "legs": [_leg("step", 1.0), _leg("step", 0.7)]},
            {"name": "stage_2", "legs": [_leg("slalom", 0.5)]}]


def test_start_selects_first_stage_with_work():
    ex = Exec(_two_stages(), counts={})
    assert ex._select_stage_with_work()
    assert ex._stage_idx == 0


def test_start_skips_completed_stage():
    counts = {}
    for c in ("lpv-hinf", "pid"):
        counts[_key("step", 1.0, c)] = 2
        counts[_key("step", 0.7, c)] = 2
    ex = Exec(_two_stages(), counts)
    assert ex._select_stage_with_work()
    assert ex._stage_idx == 1
    assert ex._legs[0]["id"] == "leg_slalom_0.5"


def test_all_stages_complete_reports_no_work():
    counts = {}
    for c in ("lpv-hinf", "pid"):
        for fam, R in (("step", 1.0), ("step", 0.7), ("slalom", 0.5)):
            counts[_key(fam, R, c)] = 2
    ex = Exec(_two_stages(), counts)
    assert not ex._select_stage_with_work()


def test_advance_stage_moves_on_and_gates_containment():
    counts = {}
    for c in ("lpv-hinf", "pid"):
        counts[_key("step", 1.0, c)] = 2
        counts[_key("step", 0.7, c)] = 2
    ex = Exec(_two_stages(), counts)
    ex._stage_idx = 0
    ex._legs = ex._stages[0]["legs"]
    assert ex._advance_stage() == "advanced"
    assert ex._stage_idx == 1 and ex._leg_idx == 0 and ex.paused is None


def test_advance_stage_pauses_on_violating_stage():
    counts = {}
    for c in ("lpv-hinf", "pid"):
        counts[_key("step", 1.0, c)] = 2
        counts[_key("step", 0.7, c)] = 2
    stages = _two_stages()
    # Put stage 2's curve 0.3 m from the wall (< 0.8 m required clearance):
    # containment must refuse it.
    stages[1]["legs"] = [_leg("slalom", 0.5, e=0.3, n=15)]
    ex = Exec(stages, counts)
    assert ex._advance_stage() == "paused"
    assert "containment" in (ex.paused or "")


def test_advance_stage_none_when_nothing_ahead():
    counts = {}
    for c in ("lpv-hinf", "pid"):
        counts[_key("slalom", 0.5, c)] = 2
    ex = Exec(_two_stages(), counts)
    ex._stage_idx = 0   # stage 1 has work, but pretend it just filled:
    for c in ("lpv-hinf", "pid"):
        counts[_key("step", 1.0, c)] = 2
        counts[_key("step", 0.7, c)] = 2
    assert ex._advance_stage() == "none"
    # restored to the current stage's legs
    assert ex._legs is ex._stages[0]["legs"]


def test_global_progress_spans_stages():
    counts = {_key("step", 1.0, "lpv-hinf"): 2}
    ex = Exec(_two_stages(), counts)
    # 3 geometries x 2 controllers x 1 speed x N=2 = 12; done = 2.
    assert ex._runs_target() == 12
    assert ex._runs_done() == 2


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
