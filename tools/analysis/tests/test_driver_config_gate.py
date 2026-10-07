# -*- coding: utf-8 -*-
"""Unit tests for the chassis driver-config gate (run_executor_node, 2026-10-07).

limo_base publishes its steering_mode / odom_model latched on /limo_base/config.
The executor must (a) refuse to start a batch unless the driver runs the
experiment's configuration, and (b) fail any scored leg whose driver config was
wrong at either end or changed in between (a respawn restarts odometry at the
origin and, before 2026-10-07, silently reverted to stock steering).

run_executor imports rclpy (robot-only), so the code under test is extracted
from the source by AST and exec'd standalone (as in test_sidecar_arrival). The
preflight and sidecar paths run through the REAL _tick_preflight and
_write_sidecar, so the call-site wiring is covered, not just the helpers.

Mutation check: every assertion group is re-run against a source-level mutant
that disables the behaviour under test; each mutant MUST fail an assertion.

Run:  python3 tools/analysis/tests/test_driver_config_gate.py   (or pytest)
"""
import ast
import json
import os
import shutil
import sys
import tempfile
import time
from datetime import datetime, timezone

_REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
_PKG = os.path.join(_REPO, "scalecar-vfg-h-infinite", "ros2_bridge",
                    "limo_path_follower")
if _REPO not in sys.path:
    sys.path.insert(0, _REPO)

import Data_Logger  # noqa: E402

SRC = os.path.join(_PKG, "run_executor_node.py")

_FUNCS = {"driver_config_problems", "leg_driver_config_verdict"}
_METHODS = {"_tick_preflight", "_write_sidecar"}
_CONSTS = {"RTK_OK", "RTK_FIXED"}

EXPECT = {"steering_mode": "direct", "odom_model": "hinf"}
GOOD = {"steering_mode": "direct", "max_steering_rad": 0.408,
        "odom_model": "hinf", "odom_point_x_m": 0.1, "node_start_unix": 100.0}


def _build_ns(mutants=()):
    """Exec the code under test from SOURCE. mutants: [(def_name, old, new)];
    a mutant that does not apply raises (it would 'survive' for no reason)."""
    with open(SRC, encoding="utf-8") as f:
        src = f.read()
    # manifest=None skips the bag quick gate, so `pass` isolates this gate.
    ns = {"json": json, "os": os, "time": time, "datetime": datetime,
          "timezone": timezone, "Data_Logger": Data_Logger, "manifest": None}
    defs = []
    for node in ast.parse(src).body:
        if isinstance(node, ast.Assign) and any(
                isinstance(t, ast.Name) and t.id in _CONSTS for t in node.targets):
            exec(compile(ast.Module([node], []), SRC, "exec"), ns)
        elif isinstance(node, ast.FunctionDef) and node.name in _FUNCS:
            defs.append(node)
        elif isinstance(node, ast.ClassDef) and node.name == "RunExecutor":
            defs += [s for s in node.body
                     if isinstance(s, ast.FunctionDef) and s.name in _METHODS]
    applied = set()
    for node in defs:
        seg = ast.get_source_segment(src, node)
        for (name, old, new) in mutants:
            if name == node.name:
                assert old in seg, f"mutant does not apply to {name}: {old!r}"
                seg = seg.replace(old, new)
                applied.add(name)
        exec(compile(seg, SRC, "exec"), ns)
    assert applied == {m[0] for m in mutants}, "a mutant target was not found"
    missing = (_FUNCS | _METHODS | _CONSTS) - set(ns)
    assert not missing, f"names not found in source: {missing}"
    return ns


class _Log:
    def __init__(self):
        self.lines = []

    def info(self, m):
        self.lines.append(m)

    warn = error = info


class _Exec:
    """Just the state _tick_preflight and _write_sidecar read."""

    def __init__(self, ns, bag=None):
        for name in _METHODS:
            setattr(self, name, ns[name].__get__(self))
        self.log = _Log()
        self.status = []
        self.began = False
        # preflight inputs, all healthy except the driver config under test
        self._estop = False
        self._last_odom_t = time.monotonic()
        self._rtk_quality = sorted(ns["RTK_OK"])[0]
        self._battery_v = 12.0
        self._batt_halt = 10.5
        self._preflight_timeout = 60.0
        self._driver_cfg = None
        self._driver_expect = dict(EXPECT)
        # sidecar inputs
        self._leg_bag_path = bag
        self._leg_driver_cfg_start = None
        self._cur_treatment = {"controller": "pid", "v_const": 1.0, "rep": 0}
        self._leg_rtk_total_samples = 10
        self._leg_rtk_fixed_samples = 10
        self._leg_estopped = False
        self._rtk_window_pct = 95.0
        self._run_end_reason = "done"
        self._leg_cell_id = "exp_1_pid_v1_n00"
        self._cur_recipe = {"type": "step", "params": {"R": 1.0}}
        self._run_id = "t"
        self._venue = {"name": "t"}
        self._leg_start_utc = None
        self._achieved_anchor = None
        self._arrival = None

    def get_logger(self):
        return self.log

    def _ensure_support_procs(self):
        return []

    def _in_phase_s(self):
        return 0.0

    def _publish_status(self, message=""):
        self.status.append(message)

    def _begin_current_curve(self):
        self.began = True

    def _pause(self, reason):
        self.status.append("PAUSE " + reason)

    def _leg_id(self, _idx):
        return "leg_1"

    _leg_idx = 0


# ---------------------------------------------------------------------------

def check_helpers(ns):
    probs, verdict = ns["driver_config_problems"], ns["leg_driver_config_verdict"]
    assert probs(None, EXPECT), "no config must be a problem"
    assert probs(dict(GOOD), EXPECT) == [], probs(dict(GOOD), EXPECT)
    stock_steer = dict(GOOD, steering_mode="agilex")
    stock_odom = dict(GOOD, odom_model="agilex")
    assert any("steering_mode" in p for p in probs(stock_steer, EXPECT))
    assert any("odom_model" in p for p in probs(stock_odom, EXPECT))
    # an empty expectation disables only that key
    assert probs(stock_steer, {"steering_mode": "", "odom_model": "hinf"}) == []
    assert probs(stock_odom, {"steering_mode": "", "odom_model": "hinf"})

    ok, why = verdict(dict(GOOD), dict(GOOD), EXPECT)
    assert ok and why == [], why
    respawn = dict(GOOD, node_start_unix=200.0)      # same mode, new process
    ok, why = verdict(dict(GOOD), respawn, EXPECT)
    assert not ok and any("changed" in r for r in why), why
    ok, why = verdict(None, dict(GOOD), EXPECT)        # no config at bag start
    assert not ok and any(r.startswith("start:") for r in why), why
    ok, why = verdict(dict(GOOD), stock_steer, EXPECT)  # param set to stock mid-leg
    assert not ok, why


def check_preflight_blocks(ns):
    ex = _Exec(ns)
    ex._tick_preflight()
    assert not ex.began, "preflight passed with no driver config"
    assert any("limo_base/config" in s for s in ex.status), ex.status

    ex = _Exec(ns)
    ex._driver_cfg = dict(GOOD, steering_mode="agilex")
    ex._tick_preflight()
    assert not ex.began, "preflight passed with stock steering"
    assert any("steering_mode" in s for s in ex.status), ex.status

    ex = _Exec(ns)
    ex._driver_cfg = dict(GOOD)
    ex._tick_preflight()
    assert ex.began, f"preflight blocked a correct driver: {ex.status}"


def _sidecar(ns, start, end):
    tmp = tempfile.mkdtemp(prefix="drvcfg_")
    try:
        bag = os.path.join(tmp, "leg_bag")
        os.makedirs(bag)
        ex = _Exec(ns, bag)
        ex._leg_driver_cfg_start, ex._driver_cfg = start, end
        passed = ex._write_sidecar({"name": "exp_1"}, {"duration_s": 1.0}, True)
        with open(os.path.join(bag, "leg_bag.sidecar.json"), encoding="utf-8") as f:
            return passed, json.load(f)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def check_sidecar(ns):
    passed, sc = _sidecar(ns, dict(GOOD), dict(GOOD))
    assert passed and sc["classification"]["pass"], sc["classification"]
    assert sc["classification"]["driver_config_ok"] is True
    assert sc["driver_config"]["at_start"]["steering_mode"] == "direct"
    assert sc["driver_config"]["expected"] == EXPECT

    passed, sc = _sidecar(ns, dict(GOOD), dict(GOOD, node_start_unix=200.0))
    c = sc["classification"]
    assert not passed and not c["pass"] and c["driver_config_ok"] is False, c
    assert any("changed" in r for r in c["driver_config_reasons"]), c
    assert sc["driver_config"]["at_end"]["node_start_unix"] == 200.0


CHECKS = [check_helpers, check_preflight_blocks, check_sidecar]

# Each mutant disables one behaviour; the named check must then fail.
MUTANTS = [
    (check_helpers, [("driver_config_problems", "!= want", "== want and False")]),
    (check_helpers, [("leg_driver_config_verdict", "and at_start != at_end", "and False")]),
    (check_preflight_blocks, [("_tick_preflight",
                               "missing += driver_config_problems(",
                               "_unused = driver_config_problems(")]),
    (check_sidecar, [("_write_sidecar", "if not cfg_ok:", "if False:")]),
]


def test_driver_config_gate():
    ns = _build_ns()
    for check in CHECKS:
        check(ns)


def test_mutants_are_caught():
    for check, mutants in MUTANTS:
        ns = _build_ns(mutants)
        try:
            check(ns)
        except AssertionError:
            continue
        raise AssertionError(f"mutant survived {check.__name__}: {mutants}")


if __name__ == "__main__":
    test_driver_config_gate()
    test_mutants_are_caught()
    print("ok: driver-config gate + %d mutants caught" % len(MUTANTS))
