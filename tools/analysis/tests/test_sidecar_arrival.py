# -*- coding: utf-8 -*-
"""Unit tests for the sidecar 'arrival' record (run_executor_node, 2026-10-04).

Every scored leg must record HOW the robot reached its start pin — the glue
as sent to reposition, its geometry metrics, and reposition's status — so the
arrival heading error (median 7 deg, p90 12 deg in the field) can be
attributed to glue turn radius / straight-tail length after the fact.

run_executor imports rclpy (robot-only), so the code under test is extracted
from the source by AST and exec'd standalone (same approach as
test_run_executor_stages). The goto -> arrived path runs through the REAL
_tick_reposition_goto, so the call-site wiring is covered, not just helpers.

Mutation check: the same assertions are re-run against source-level mutants
of the code under test (swapped projection axes, wrong radius, reversed tail
heading, ...). Each mutant MUST make an assertion fail — that is the evidence
these checks can fail at all.

Run:  python3 tools/analysis/tests/test_sidecar_arrival.py   (or pytest)
"""
import ast
import json
import math
import os
import shutil
import sys
import tempfile
import time
from datetime import datetime, timezone

_REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
_PKG = os.path.join(_REPO, "scalecar-vfg-h-infinite", "ros2_bridge",
                    "limo_path_follower")
_VFG = os.path.join(_REPO, "scalecar-vfg-h-infinite")
_TOOLS = os.path.join(_REPO, "tools", "analysis")
for p in (_PKG, _VFG, _TOOLS, _REPO):
    if p not in sys.path:
        sys.path.insert(0, p)

import Data_Logger         # noqa: E402
import experiment_planner  # noqa: E402
import manifest            # noqa: E402
import venue_geom          # noqa: E402

SRC = os.path.join(_PKG, "run_executor_node.py")

_FUNCS = {"glue_arrival_metrics"}
_METHODS = {"_tick_reposition_goto", "_snapshot_glue_arrival",
            "_capture_arrival", "_write_sidecar"}
_CONSTS = {"ARRIVAL_TAIL_ALIGN_DEG", "PROC_REPOSITION", "RTK_FIXED"}


class _String:
    def __init__(self, data=""):
        self.data = data


def _build_ns(mutants=()):
    """Exec the code under test from SOURCE. mutants: [(def_name, old, new)]
    text replacements; a mutant that does not apply raises (a no-op mutant
    would otherwise 'survive' for the wrong reason)."""
    with open(SRC, encoding="utf-8") as f:
        src = f.read()
    ns = {"json": json, "math": math, "os": os, "time": time,
          "datetime": datetime, "timezone": timezone,
          "experiment_planner": experiment_planner, "venue_geom": venue_geom,
          "manifest": manifest, "Data_Logger": Data_Logger, "String": _String}
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
    for node in defs:
        seg = ast.get_source_segment(src, node)
        for (name, old, new) in mutants:
            if name == node.name:
                assert old in seg, f"mutant does not apply to {name}: {old!r}"
                seg = seg.replace(old, new)
        exec(compile(seg, SRC, "exec"), ns)
    missing = (_FUNCS | _METHODS | _CONSTS) - set(ns)
    assert not missing, f"names not found in source: {missing}"
    return ns


NS = _build_ns()

# ---------------------------------------------------------------------------
# Synthetic glues (EN metres -> WGS84 at a real latitude, so cos(lat) != 1 and
# an E/N or lat/lon mix-up changes the geometry instead of hiding).
# ---------------------------------------------------------------------------

LAT0, LON0 = 37.5665, 126.9780


def _wgs(pts_en):
    out = []
    for (e, n) in pts_en:
        la, lo = venue_geom.en_to_latlon(e, n, LAT0, LON0)
        out.append({"lat": la, "lon": lo})
    return out


def _arc(cx, cy, R, t0_deg, t1_deg, step_deg):
    """Left (CCW) arc samples, excluding t0: tangent heading at t is t (CCW
    from East), so this starts heading East when t0 = 0."""
    out, t = [], t0_deg + step_deg
    while t <= t1_deg + 1e-9:
        r = math.radians(t)
        out.append((cx + R * math.sin(r), cy - R * math.cos(r)))
        t += step_deg
    return out


# Straight-tail glue: 1 m lead East, R=2 m left turn in 15 deg steps onto
# North, then a 1.5 m straight tail North (6 x 0.25 m) into the pin.
STRAIGHT_EN = ([(0.0, 0.0), (0.5, 0.0), (1.0, 0.0)]
               + _arc(1.0, 2.0, 2.0, 0.0, 90.0, 15.0)
               + [(3.0, 2.0 + 0.25 * i) for i in range(1, 7)])
STRAIGHT_R = 2.0
STRAIGHT_TAIL = 1.5
# Analytic chord length — independent of the code's polyline sum.
STRAIGHT_LEN = 1.0 + 6 * (2 * STRAIGHT_R * math.sin(math.radians(7.5))) + 1.5

# Tight-turn glue: 0.5 m East, then an R=0.5 m hairpin in 30 deg steps that
# ENDS mid-turn at the pin (heading West = 270). Its last chord is 15 deg off
# the pin heading, so there is NO settled tail.
TIGHT_EN = [(0.0, 0.0), (0.5, 0.0)] + _arc(0.5, 0.5, 0.5, 0.0, 180.0, 30.0)
TIGHT_R = 0.5


def _close(a, b, tol):
    return a is not None and abs(a - b) <= tol


def check_straight_tail(ns):
    m = ns["glue_arrival_metrics"](_wgs(STRAIGHT_EN), 0.0)
    assert m["n_waypoints"] == 15, m
    assert _close(m["min_turn_radius_m"], STRAIGHT_R, 2e-3), m
    assert _close(m["max_curvature_1pm"], 1.0 / STRAIGHT_R, 1e-3), m
    # The last arc chord is 7.5 deg off North (> 5): the tail is exactly the
    # 1.5 m straight, not one segment more or less.
    assert _close(m["tail_straight_m"], STRAIGHT_TAIL, 2e-3), m
    assert _close(m["length_m"], STRAIGHT_LEN, 2e-3), m
    assert m["tail_align_deg"] == 5.0, m
    json.dumps(m, allow_nan=False)


def check_tight_turn(ns):
    m = ns["glue_arrival_metrics"](_wgs(TIGHT_EN), 270.0)
    assert _close(m["min_turn_radius_m"], TIGHT_R, 1e-3), m
    assert _close(m["tail_straight_m"], 0.0, 1e-9), m
    arc_len = 6 * (2 * TIGHT_R * math.sin(math.radians(15.0)))
    assert _close(m["length_m"], 0.5 + arc_len, 2e-3), m


def test_straight_tail_metrics():
    check_straight_tail(NS)


def test_tight_turn_metrics():
    check_tight_turn(NS)


def test_tail_threshold_and_heading_wrap():
    """Pin heading 358 deg vs a North (0 deg) tail: 2 deg off, inside the 5 deg
    band across the 0/360 wrap. The next chord (7.5 deg) is 9.5 deg off, so it
    joins the tail only once the band exceeds 9.5 deg."""
    f, wps = NS["glue_arrival_metrics"], _wgs(STRAIGHT_EN)
    chord = 2 * STRAIGHT_R * math.sin(math.radians(7.5))
    assert _close(f(wps, 358.0)["tail_straight_m"], 1.5, 2e-3)
    assert _close(f(wps, 358.0, align_deg=1.0)["tail_straight_m"], 0.0, 1e-9)
    assert _close(f(wps, 358.0, align_deg=10.0)["tail_straight_m"],
                  1.5 + chord, 2e-3)


def test_straight_glue_has_no_finite_radius_and_no_heading_no_tail():
    m = NS["glue_arrival_metrics"](_wgs([(0, 0), (0, 1), (0, 2)]), None)
    assert m["max_curvature_1pm"] == 0.0 and m["min_turn_radius_m"] is None, m
    assert m["tail_straight_m"] is None, m
    json.dumps(m, allow_nan=False)      # no Infinity leaks into strict JSON


# ---------------------------------------------------------------------------
# The executor path: goto sent -> 'arrived' accepted -> odom reset capture.
# ---------------------------------------------------------------------------

class _Log:
    def __init__(self, sink):
        self.sink = sink

    def info(self, *_a, **_k):
        pass

    def warn(self, msg, *_a, **_k):
        self.sink.append(msg)

    def error(self, msg, *_a, **_k):
        self.sink.append(msg)


class _Pub:
    def __init__(self):
        self.sent = []

    def get_subscription_count(self):
        return 1

    def publish(self, msg):
        self.sent.append(json.loads(msg.data))


GLUE_CURVE = {"name": "repo_to_exp_1", "kind": "reposition",
              "waypoints_wgs84": _wgs(STRAIGHT_EN), "end_heading_deg": 0.0,
              "v_const": 0.4, "pos_tol_m": 0.15}
RECIPE_CURVE = {"name": "exp_1_step_R1p0", "kind": "recipe"}


class Exec:
    """Stand-in carrying exactly the state the extracted methods touch."""

    def __init__(self, ns):
        self._ns = ns
        self.warnings = []
        self.paused = None
        self.advanced = 0
        self._cur_curve = dict(GLUE_CURVE)
        self._repo_speed = 0.2            # cap: the authored 0.4 must be sent as 0.2
        self._settle_s = 3.0
        self._reposition_timeout = 120.0
        self._arrival_head_tol = 20.0
        self._repo_state = None
        self._repo_status = {}
        self._repo_status_t = None
        self._goto_sent = False
        self._goto_seq = 0
        self._goto_last_send_t = 0.0
        self._goto_payload = None
        self._arrival_glue = None
        self._arrival = None
        self.pub_goto = _Pub()

    def get_logger(self):
        return _Log(self.warnings)

    def _is_alive(self, _name):
        return True

    def _in_phase_s(self):
        return 10.0

    def _pause(self, reason):
        self.paused = reason

    def _advance_curve(self):
        self.advanced += 1
        self._cur_curve = dict(RECIPE_CURVE)

    def __getattr__(self, name):
        ns = self.__dict__.get("_ns") or {}
        if name in _METHODS and name in ns:
            return ns[name].__get__(self, Exec)
        raise AttributeError(name)


def _status(seq, err_deg, reason="arrived (position + heading)"):
    return {"state": "arrived", "err_m": 0.08, "err_deg": err_deg,
            "reason": reason, "seq": seq, "seg_i": 13, "n": 15, "la": "end"}


def _drive_glue_to_capture(ns, final_err=7.4, final_age_s=1.5):
    """Send the goto, accept 'arrived', sit through the dwell (reposition keeps
    streaming), then capture at odom reset — the executor's real order."""
    ex = Exec(ns)
    ex._tick_reposition_goto()                       # goto out (seq 1)
    assert len(ex.pub_goto.sent) == 1 and ex._goto_seq == 1
    ex._repo_status, ex._repo_state = _status(1, 6.2), "arrived"
    ex._repo_status_t = time.monotonic()
    ex._tick_reposition_goto()                       # arrival accepted
    assert ex.advanced == 1 and ex.paused is None, (ex.advanced, ex.paused)
    assert ex._cur_curve["kind"] == "recipe"         # the glue curve is gone now
    ex._repo_status = _status(1, final_err)          # last status before the kill
    ex._repo_status_t = time.monotonic() - final_age_s
    ex._capture_arrival()
    return ex


def check_capture_records_glue_as_sent(ns):
    ex = _drive_glue_to_capture(ns)
    a, sent = ex._arrival, ex.pub_goto.sent[0]
    assert a["captured"] is True and a["problems"] == [], a["problems"]
    g = a["glue"]
    assert g["curve_name"] == "repo_to_exp_1", g["curve_name"]
    assert g["seq"] == 1
    assert g["waypoints_wgs84"] == sent["waypoints"]
    assert len(g["waypoints_wgs84"]) == 15
    assert g["v_const"] == 0.2, g["v_const"]          # capped value actually sent
    assert g["pos_tol_m"] == 0.15 and g["end_heading_deg"] == 0.0
    m = a["glue_metrics"]
    assert _close(m["min_turn_radius_m"], STRAIGHT_R, 2e-3), m
    assert _close(m["tail_straight_m"], STRAIGHT_TAIL, 2e-3), m
    assert a["status_at_arrival"]["err_deg"] == 6.2
    assert a["final_status"]["err_deg"] == 7.4
    assert a["final_status"]["reason"].startswith("arrived")
    assert 1.4 <= a["final_status_age_s"] <= 5.0, a["final_status_age_s"]
    json.dumps(a, allow_nan=False)


def check_snapshot_consumed(ns):
    """A recipe NOT directly preceded by a glue must not inherit the previous
    leg's glue (a recipe-only leg would be mis-attributed)."""
    ex = _drive_glue_to_capture(ns)
    assert ex._arrival["captured"] is True
    ex._capture_arrival()                            # next recipe, no glue
    a = ex._arrival
    assert a["captured"] is False and a["glue"] is None, a
    assert any("no reposition glue" in p for p in a["problems"]), a["problems"]


def test_capture_records_glue_as_sent():
    check_capture_records_glue_as_sent(NS)


def test_snapshot_consumed_by_one_recipe():
    check_snapshot_consumed(NS)


def test_heading_pause_takes_no_snapshot():
    """Over-tolerance arrival pauses (no recipe runs): nothing to attribute."""
    ex = Exec(NS)
    ex._tick_reposition_goto()
    ex._repo_status, ex._repo_state = _status(1, 25.0), "arrived"
    ex._tick_reposition_goto()
    assert ex.paused and ex.advanced == 0 and ex._arrival_glue is None


def test_capture_never_raises_on_bad_glue():
    ex = Exec(NS)
    # Malformed waypoints: metrics fail, the rest is still recorded.
    ex._arrival_glue = {"glue": {"curve_name": "x", "seq": 3,
                                 "waypoints_wgs84": [{"lat": 1.0}]},
                        "status": {}, "stamp_utc": None, "problems": []}
    ex._repo_status = _status(3, 4.0)
    ex._capture_arrival()
    a = ex._arrival
    assert a["captured"] is True and a["glue_metrics"] is None, a
    assert any("glue metrics failed" in p for p in a["problems"]), a["problems"]
    json.dumps(a, allow_nan=False)
    # NaN input -> NaN length: the strict-JSON guard must catch it HERE (a NaN
    # reaching write_sidecar would fail the whole leg), never raise.
    ex._arrival_glue = {"glue": {"curve_name": "y", "seq": 4, "waypoints_wgs84":
                                 [{"lat": float("nan"), "lon": 0.0},
                                  {"lat": float("nan"), "lon": 1e-5}],
                                 "end_heading_deg": 0.0},
                        "status": {}, "stamp_utc": None, "problems": []}
    ex._capture_arrival()
    a = ex._arrival
    assert a["captured"] is False, a
    assert a["problems"][0].startswith("arrival capture failed"), a
    json.dumps(a, allow_nan=False)


def test_snapshot_failure_is_marked_not_silent():
    ex = Exec(NS)
    ex._goto_payload = {"seq": 1, "waypoints": 5}   # not a list -> list() raises
    ex._goto_seq = 1
    ex._snapshot_glue_arrival()
    assert ex._arrival_glue is not None and ex._arrival_glue["glue"] is None
    ex._capture_arrival()
    a = ex._arrival
    assert a["captured"] is False
    assert any("glue snapshot failed" in p for p in a["problems"]), a["problems"]


# ---------------------------------------------------------------------------
# End to end: the record reaches the sidecar file on disk.
# ---------------------------------------------------------------------------

class _SidecarExec(Exec):
    def __init__(self, ns, bag):
        super().__init__(ns)
        self._leg_bag_path = bag
        self._cur_treatment = {"controller": "pid", "v_const": 0.5, "rep": 0}
        self._leg_rtk_total_samples = 10
        self._leg_rtk_fixed_samples = 10
        self._leg_estopped = False
        self._estop = False
        self._rtk_window_pct = 95.0
        self._run_end_reason = "done"
        self._leg_cell_id = "exp_1_pid_v0p5_n00"
        self._cur_recipe = {"type": "step", "params": {"R": 1.0}}
        self._run_id = "t"
        self._venue = {"name": "t"}
        self._leg_start_utc = None
        self._achieved_anchor = None

    def _leg_id(self, _idx):
        return "leg_1"

    _leg_idx = 0


def check_sidecar_file_carries_arrival(ns):
    tmp = tempfile.mkdtemp(prefix="arrival_")
    try:
        bag = os.path.join(tmp, "leg_bag")
        os.makedirs(bag)
        src = _drive_glue_to_capture(ns)
        ex = _SidecarExec(ns, bag)
        ex._arrival = src._arrival
        curve = {"name": "exp_1", "start_pose": {"lat": 1.0, "lon": 2.0,
                                                 "heading_deg": 0.0}}
        ex._write_sidecar(curve, {"duration_s": 1.0}, True)
        path = os.path.join(bag, "leg_bag.sidecar.json")
        assert os.path.isfile(path), f"no sidecar written ({ex.warnings})"
        with open(path, encoding="utf-8") as f:
            sc = json.load(f)
        assert "arrival" in sc, sorted(sc)
        assert sc["arrival"] == src._arrival
        assert _close(sc["arrival"]["glue_metrics"]["min_turn_radius_m"],
                      STRAIGHT_R, 2e-3)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def test_sidecar_file_carries_arrival():
    check_sidecar_file_carries_arrival(NS)


# ---------------------------------------------------------------------------
# Mutation check: every mutant must be CAUGHT by the assertions above.
# ---------------------------------------------------------------------------

_GAM = "glue_arrival_metrics"
MUTANTS = [
    ("projection lat/lon swapped", [(_GAM,
        'latlon_to_en(float(w["lat"]), float(w["lon"]), lat0, lon0)',
        'latlon_to_en(float(w["lon"]), float(w["lat"]), lon0, lat0)')]),
    ("radius off by 2x", [(_GAM, "round(1.0 / k, 3)", "round(2.0 / k, 3)")]),
    ("curvature on raw degrees", [(_GAM, "_max_curvature(pts)",
        '_max_curvature([(float(w["lon"]), float(w["lat"])) for w in wps])')]),
    ("tail vs reversed heading", [(_GAM, "float(end_heading_deg) % 360.0",
                                   "(float(end_heading_deg) + 180.0) % 360.0")]),
    ("tail band ignored (x4)", [(_GAM, "float(align_deg)))",
                                 "float(align_deg) * 4.0))")]),
    ("length drops last segment", [(_GAM, "zip(pts[:-1], pts[1:]))",
                                    "zip(pts[:-2], pts[1:-1]))")]),
    ("authored v_const recorded, not the capped one sent",
     [("_snapshot_glue_arrival", '"v_const": p.get("v_const")',
       '"v_const": (self._cur_curve or {}).get("v_const")')]),
    ("snapshot not consumed", [("_capture_arrival",
        "snap, self._arrival_glue = self._arrival_glue, None",
        "snap = self._arrival_glue")]),
    ("snapshot never taken at arrival", [("_tick_reposition_goto",
        "self._snapshot_glue_arrival()\n", "pass\n")]),
    ("arrival not attached to sidecar", [("_write_sidecar",
        'sidecar["arrival"] = self._arrival', "pass")]),
]

CHECKS = [check_straight_tail, check_tight_turn,
          check_capture_records_glue_as_sent, check_snapshot_consumed,
          check_sidecar_file_carries_arrival]


def test_every_mutant_is_caught():
    # Only an AssertionError counts as "caught": any other exception is a
    # harness fault and propagates, so it can't masquerade as detection.
    survivors = []
    for desc, muts in MUTANTS:
        ns = _build_ns(muts)
        caught = False
        for chk in CHECKS:
            try:
                chk(ns)
            except AssertionError:
                caught = True
                break
        if not caught:
            survivors.append(desc)
    assert not survivors, f"mutants survived (checks too weak): {survivors}"


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
