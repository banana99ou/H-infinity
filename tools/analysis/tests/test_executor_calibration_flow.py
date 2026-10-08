# -*- coding: utf-8 -*-
"""End-to-end: the REAL run_executor_node driven through a first session.

The executor module is imported against a fake rclpy (fake_ros.py) and run
tick by tick against a simulated robot: an orchestrator that starts/kills
PROCs, reposition that arrives, odom_zero that confirms, a follower that
finishes, calib_node that returns a figure-8 result, and live sensors. Only
the ROS transport and the bag recorder are fake; every executor decision,
the planner, the manifest, the lock math and the files on disk are real.

Scenario A (first session, no lock): Auto-plan returns the calibration-only
plan -> Start -> approach + full figure-8 -> lock written -> matrix re-planned
and persisted by the executor -> transit into stage 2 -> scored legs carry the
lock epoch -> the first pass moves on after ONE rep of each cell. Run twice:
with the DEPLOYED experiment.yaml (the 2026-10-08 bridge set: the old fixed
radii gated by calibration.required, an old stock-driver leg planted in the
first cell) and with the shelved radius_m: auto campaign.
Scenario B (lock exists, check stale): a sanity figure-8 whose R moved pauses
the batch; Start retries; a good one continues into the matrix.

Invariant checked on EVERY tick: at most one cmd_vel_raw mover alive (C6).

Run:  python3 -m pytest -q tools/analysis/tests/test_executor_calibration_flow.py
"""
import json
import math
import os
import shutil
import sys
import tempfile
import time
from datetime import datetime, timedelta, timezone

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.abspath(os.path.join(_HERE, "..", "..", ".."))
_PKG = os.path.join(_REPO, "scalecar-vfg-h-infinite", "ros2_bridge",
                    "limo_path_follower")
_VFG = os.path.join(_REPO, "scalecar-vfg-h-infinite")
for p in (_HERE, _REPO, _PKG, _VFG, os.path.join(_REPO, "tools", "analysis")):
    if p not in sys.path:
        sys.path.insert(0, p)

import fake_ros  # noqa: E402

fake_ros.install()

import Data_Logger          # noqa: E402
import calibration as cal   # noqa: E402
import experiment_planner as ep   # noqa: E402
import run_executor_node as rex   # noqa: E402

MOVERS = ("reposition", "follower", "calib")
PROCS = MOVERS + ("odom_zero", "heading", "rtk_watchdog", "odom_watchdog",
                  "geofence")


class FakeRecorder:
    """Stands in for `ros2 bag record`: makes a bag dir the manifest finds."""

    def __init__(self, path, topics=None):
        self.path = path
        self.topics = list(topics or [])

    def start(self):
        os.makedirs(self.path, exist_ok=True)
        with open(os.path.join(self.path, "metadata.yaml"), "w") as f:
            f.write("rosbag2_bagfile_information: {}\n")

    def stop(self, timeout_s=10.0):
        return {"duration_s": 8.0,
                "end_utc": datetime.now(timezone.utc).isoformat()}


Data_Logger.BagRecorder = FakeRecorder
# The bag-level quick gate reads real rosbag2 metadata (topic counts); the fake
# recorder writes none, so the gate is stubbed to pass (it has its own tests —
# test_manifest_dropped). Everything else in classification stays real.
rex.manifest.quick_gate = lambda _d: {"pass": True, "reasons": [],
                                      "duration_s": 8.0}


def _calib_result(mode, r=0.60):
    speeds = [0.5, 1.0] if mode == "full" else [1.0]
    segs = [{"v_cmd": v, "dir": d, "R_imu_m": r + (0.01 if d == "right" else 0.0),
             "R_rtk_rear_m": r * 1.01, "delta_imu_rad": 0.32}
            for v in speeds for d in ("left", "right")]
    return {"ok": True, "reason": None, "mode": mode, "segments": segs}


class World:
    def __init__(self, pin, calib_r=0.60):
        self.alive = {p: False for p in PROCS}
        self.pin = pin
        self.calib_r = calib_r
        self.calib_requests = []
        self.gotos = []
        self.recipes = []
        self.pending = []           # (due_step, topic, msg)
        self.step_n = 0
        self.max_alive_movers = 0
        B = fake_ros.BUS
        B.subs["/orchestrator/start"].append(self._start)
        B.subs["/orchestrator/kill"].append(self._kill)
        B.subs["/reposition/goto"].append(self._goto)
        B.subs["/odom_zero/reset"].append(self._odom_reset)
        B.subs["/reference_path_recipe"].append(self._recipe)
        B.subs["/calib/request"].append(self._calib_req)

    def _later(self, steps, topic, msg):
        self.pending.append((self.step_n + steps, topic, msg))

    def _start(self, m):
        if m.data in self.alive:
            self.alive[m.data] = True

    def _kill(self, m):
        if m.data in self.alive:
            self.alive[m.data] = False

    def _goto(self, m):
        d = json.loads(m.data)
        self.gotos.append(d)
        if self.alive["reposition"]:
            self._later(2, "/reposition/status", fake_ros.String(data=json.dumps(
                {"state": "arrived", "seq": d["seq"], "err_m": 0.05,
                 "err_deg": 1.5, "reason": "arrived"})))

    def _odom_reset(self, m):
        self._later(1, "/odom_zero/status", fake_ros.String(data=json.dumps(
            {"has_reset": True, "stamp": time.time() + 0.5,
             "origin": {"x": 0, "y": 0, "yaw": 0}})))

    def _recipe(self, m):
        d = json.loads(m.data)
        if d.get("type") in (None, "none"):
            return
        self.recipes.append(d)
        self._later(3, "/path_follower/done", fake_ros.Bool(data=True))

    def _calib_req(self, m):
        d = json.loads(m.data)
        if not self.alive["calib"]:
            return
        if self.calib_requests and self.calib_requests[-1]["seq"] == d["seq"]:
            return
        self.calib_requests.append(d)
        self._later(1, "/calib/status", fake_ros.String(data=json.dumps(
            {"seq": d["seq"], "state": "running", "segment": "v0.5 left"})))
        self._later(3, "/calib/status", fake_ros.String(data=json.dumps(
            {"seq": d["seq"], "state": "done", "reason": None,
             "result": _calib_result(d["mode"], self.calib_r)})))

    def step(self):
        B = fake_ros.BUS
        self.step_n += 1
        due = [x for x in self.pending if x[0] <= self.step_n]
        self.pending = [x for x in self.pending if x[0] > self.step_n]
        for _s, topic, msg in due:
            B.publish(topic, msg)
        B.publish("/orchestrator/status", fake_ros.String(data=json.dumps(self.alive)))
        B.publish("/wheel/odom", fake_ros.Odometry())
        B.publish("/gps_rtk_f9p_helical/gps/rtk_status",
                  fake_ros.String(data="quality=4 (RTK FIXED)"))
        B.publish("/limo_base/config", fake_ros.String(data=json.dumps(
            {"steering_mode": "direct", "odom_model": "hinf",
             "max_steering_rad": 0.408, "node_start_unix": 1.0})))
        B.publish("/heading/fused_status", fake_ros.String(data=json.dumps(
            {"mode": "GNSS_AIDED", "fused_deg": 37.0, "heading_std_deg": 1.0,
             "active_sources": ["gyro", "cog"]})))
        B.publish("/gps_rtk_f9p_helical/gps/fix",
                  fake_ros.NavSatFix(latitude=self.pin["lat"],
                                     longitude=self.pin["lon"], status=2))
        B.publish("/estop", fake_ros.Bool(data=False))
        n = sum(1 for m in MOVERS if self.alive[m])
        self.max_alive_movers = max(self.max_alive_movers, n)
        assert n <= 1, f"C6 violated: {self.alive}"


_YAML = os.path.join(_REPO, "scenarios", "experiment.yaml")

# The shelved 320-run campaign (2026-10-08 night): auto radii, left + right
# step/slalom, 2 m straights, no calibration.required.
_AUTO_PATCH = {
    "matrix": {"radius_m": "auto",
               "path_family": ["step", "step_m", "slalom", "slalom_m"]},
    "calibration": {"required": False},
    "plan": {"step": {"L1": 2.0, "L2": 2.0, "theta_deg": 90.0}},
}


def _yaml_variant(tmp, patch):
    """The deployed experiment.yaml with top-level sections patched."""
    import yaml
    with open(_YAML) as f:
        d = yaml.safe_load(f)
    for k, v in patch.items():
        if isinstance(v, dict):
            d.setdefault(k, {}).update(v)
        else:
            d[k] = v
    path = os.path.join(tmp, "experiment_variant.yaml")
    with open(path, "w") as f:
        yaml.safe_dump(d, f)
    return path


def _plant_old_leg(bag_root, run_id, controller, v, R, family="step"):
    """A passing stock-driver leg as the June-October runs left it: same
    venue run_id and cell, no matrix_epoch."""
    bag = os.path.join(bag_root, f"26_1006_1200_{run_id}_old_{family}_R{R}_{controller}_v{v}")
    os.makedirs(bag)
    with open(os.path.join(bag, "metadata.yaml"), "w") as f:
        f.write("rosbag2_bagfile_information: {}\n")
    Data_Logger.write_sidecar(bag, {
        "run_id": run_id, "cell_id": "old", "leg": "leg_1",
        "cell_params": {"controller": controller, "v_const": v,
                        "radius_m": R, "path_family": family, "rep": 0},
        "classification": {"pass": True, "reached_end": True},
        "venue": {"venue_id": run_id}})
    return bag


def _setup(tmp, lock=None, last_check_age_h=None, calib_r=0.60,
           yaml_path=None, before_plan=None):
    fake_ros.BUS.__init__()
    venue_src = os.path.join(_REPO, "tools", "analysis", "tests",
                             "venue_fixtures", "active.json")
    with open(venue_src) as f:
        venue = {k: v for k, v in json.load(f).items()
                 if k not in ("plan_stages", "legs")}
    venues_dir = os.path.join(tmp, "venues")
    os.makedirs(venues_dir)
    active = os.path.join(venues_dir, "active.json")
    bag_root = os.path.join(tmp, "Experiment Data")
    os.makedirs(bag_root)
    if lock is not None:
        cal.write_lock(cal.lock_path(bag_root), lock)
        if last_check_age_h is not None:
            cal.append_check(cal.checks_path(bag_root), {
                "ok": True, "epoch": lock["epoch"],
                "stamp_utc": (datetime.now(timezone.utc)
                              - timedelta(hours=last_check_age_h)).isoformat()})
    if before_plan is not None:
        before_plan(bag_root, str(venue.get("name") or "run"))
    fake_ros.Node.OVERRIDES = {"run_executor_node": {
        "active_venue_file": active, "bag_root": bag_root,
        "experiment_yaml": yaml_path or _YAML,
        "inter_curve_dwell_s": 0.0, "orchestrator_settle_s": 0.0,
        "rtk_fix_wait_s": 2.0, "odom_settle_s": 5.0,
    }}
    node = rex.RunExecutor()
    # What the operator does: Auto-plan (through the executor's own handler)
    # -> the browser applies the plan -> Send persists it (venue_loader).
    got = []
    fake_ros.BUS.subs["/plan/result"].append(lambda m: got.append(json.loads(m.data)))
    node._on_plan_request(fake_ros.String(data=json.dumps({"venue": venue})))
    plan = got[-1]
    assert plan["stages"], plan
    payload = dict(venue)
    payload["plan_stages"] = ep.stages_to_legs(plan["stages"], venue)
    payload["legs"] = payload["plan_stages"][0]["legs"]
    with open(active, "w") as f:
        json.dump(payload, f)
    pin = plan["stages"][0]["experiments"][0]["start"]
    world = World(pin, calib_r)
    return node, world, plan, active, bag_root


def _run(node, world, until, max_steps=200000, max_s=600):
    """Step the world + executor until `until()`. Paced at ~2 ms/step: the
    executor's goto / calib re-send gates are wall-clock (1 s)."""
    t0 = time.time()
    for _ in range(max_steps):
        world.step()
        node._tick()
        if until():
            return True
        time.sleep(0.05 if node.phase.value == "replan" else 0.002)
        if time.time() - t0 > max_s:
            break
    return False


def _alerts():
    return [json.loads(m.data) for m in fake_ros.BUS.sent["/operator/alert"]]


def _first_session(yaml_path, n_matrix, radii_for, plant_old=False):
    # Fails if: the first plan is not calibration-only; Start does not drive
    # the approach and the FULL figure-8; no lock; the executor does not
    # persist calibration + n_matrix stages with the expected radii; it does
    # not transit into stage 2; scored legs lack the lock epoch; the first
    # pass does not move on after one rep of each of the stage's 4 cells
    # (a planted old leg that got credited would leave 3); or C6 is broken.
    tmp = tempfile.mkdtemp(prefix="hinf_flow_")
    try:
        planted = []
        hook = None
        if plant_old:
            def hook(bag_root, run_id):
                planted.append((run_id, _plant_old_leg(
                    bag_root, run_id, "lpv-hinf", 1.0, 1.0)))
        node, world, plan, active, bag_root = _setup(
            tmp, yaml_path=yaml_path, before_plan=hook)
        assert [s["name"] for s in plan["stages"]] == ["calibration"]
        node._on_go(fake_ros.String(data=""))
        assert node.phase.value == "preflight", node.phase
        ok = _run(node, world, lambda: cal.load_lock(cal.lock_path(bag_root)) is not None)
        assert ok, (node.phase, node._pause_reason, node.log[-15:])
        lock = cal.load_lock(cal.lock_path(bag_root))
        assert lock["R_min_m"] == 0.61                       # worse (right) side
        assert lock["radius_m"] == cal.radii_from_rmin(0.61, cal.config({}))
        req = world.calib_requests[0]
        assert req["mode"] == "full" and req["speeds"] == [0.5, 1.0]
        assert world.gotos and world.gotos[0]["v_const"] == 0.4   # glue speed
        # re-plan -> persisted batch -> transit -> first scored leg
        ok = _run(node, world, lambda: len(world.recipes) >= 1)
        assert ok, (node.phase, node._pause_reason, node.log[-15:])
        with open(active) as f:
            act = json.load(f)
        names = [s["name"] for s in act["plan_stages"]]
        assert names[0] == "calibration" and len(names) == 1 + n_matrix, names
        assert act["auto_replan"]["epoch"] == lock["epoch"]
        assert act["auto_replan"]["radius_m"] == radii_for(lock)
        assert node._matrix_doc["matrix"]["radius_m"] == radii_for(lock)
        assert any(g.get("waypoints") and len(g["waypoints"]) > 2
                   for g in world.gotos[1:2]), "no transit goto after the re-plan"
        assert node._stage_name() == "stage_2"
        # first pass: 4 cells (2 controllers x 2 speeds) in stage 2, one rep each
        ok = _run(node, world, lambda: node._stage_name() == "stage_3")
        assert ok, (node.phase, node._pause_reason, node.log[-15:])
        assert len(world.recipes) == 4, len(world.recipes)
        side = []
        for dp, _dn, fn in os.walk(bag_root):
            side += [os.path.join(dp, x) for x in fn if x.endswith(".sidecar.json")]
        scored = [json.load(open(x)) for x in side
                  if "calibration" not in x and "_old_" not in x]
        assert len(scored) == 4
        assert all(s["matrix_epoch"] == lock["epoch"] for s in scored)
        assert all(s["matrix_lock"]["radius_m"] == radii_for(lock)
                   and s["matrix_lock"]["delta_max_rad"] == lock["delta_max_rad"]
                   for s in scored)
        assert all(s["classification"]["pass"] for s in scored), \
            [s["classification"] for s in scored]
        cells = {(s["cell_params"]["controller"], s["cell_params"]["v_const"])
                 for s in scored}
        assert len(cells) == 4, cells
        cal_sc = [json.load(open(x)) for x in side if "calibration" in x]
        assert len(cal_sc) == 1 and cal_sc[0]["calibration"]["ok"]
        with open(cal.checks_path(bag_root)) as f:
            checks = [json.loads(l) for l in f]
        assert checks[-1]["ok"] and checks[-1]["epoch"] == lock["epoch"]
        alerts = _alerts()
        titles = [a["title"] for a in alerts]
        assert "R_min LOCKED" in titles and "MATRIX PLANNED" in titles, titles
        locked = [a for a in alerts if a["title"] == "R_min LOCKED"][-1]
        assert str(radii_for(lock)) in locked["detail"], locked["detail"]
        assert world.max_alive_movers == 1
        if planted:
            # The planted leg is valid credit for an UNGATED fixed matrix (so
            # the 4-recipe check above could fail) and none for the gated one.
            run_id, _bag = planted[0]
            key = node._cell_key_for("step", 1.0, "lpv-hinf", 1.0)
            gated_doc = node._matrix_doc
            assert node._counts_for(run_id).get(key) == 1      # this session's leg
            node._matrix_doc = dict(gated_doc, _calib_gated=False)
            assert node._counts_for(run_id).get(key) == 2      # + the old leg
            node._matrix_doc = gated_doc
        return world
    finally:
        shutil.rmtree(tmp)


def test_first_session_bridge_set_calibrates_then_runs_the_old_paths():
    # The DEPLOYED yaml: fixed old radii gated by calibration.required. Also
    # fails if the stage-2 recipes are not the old dataset's step exactly
    # (left, 90 deg, 1 m straights, R 1.0).
    world = _first_session(None, 4, lambda lock: [1.0, 0.7, 0.5, 0.4],
                           plant_old=True)
    for r in world.recipes:
        assert r["type"] == "step", r
        p = r["params"]
        assert (p["R"], p["L1"], p["L2"]) == (1.0, 1.0, 1.0), p
        assert abs(p["theta_arc"] - math.pi / 2) < 1e-9, p
        assert p["direction"] == 1, p                          # left turn


def test_first_session_auto_matrix_calibrates_locks_replans():
    # The shelved auto campaign: radii from the lock, left + right x 2 families.
    tmp = tempfile.mkdtemp(prefix="hinf_yaml_")
    try:
        _first_session(_yaml_variant(tmp, _AUTO_PATCH), 16,
                       lambda lock: lock["radius_m"])
    finally:
        shutil.rmtree(tmp)


def test_sanity_failure_pauses_and_retry_continues():
    # Fails if a sanity figure-8 whose R moved does not pause the batch with a
    # CALIBRATION FAILED card, if Start does not re-run it, or a good retry
    # does not continue into the matrix without a new lock.
    tmp = tempfile.mkdtemp(prefix="hinf_flow_")
    try:
        lock, _e = cal.compute_lock(_calib_result("full", 0.60), cal.config({}))
        node, world, plan, active, bag_root = _setup(
            tmp, lock=lock, last_check_age_h=20.0, calib_r=1.05)
        names = [s["name"] for s in plan["stages"]]
        assert names[0] == "calibration" and len(names) == 1 + 4, names  # bridge set
        assert plan["stages"][0]["experiments"][0]["recipe"]["params"]["mode"] == "sanity"
        node._on_go(fake_ros.String(data=""))
        ok = _run(node, world, lambda: node.phase.value == "paused")
        assert ok
        assert "steering changed" in (node._pause_reason or ""), node._pause_reason
        assert any(a["title"] == "CALIBRATION FAILED" for a in _alerts())
        assert world.calib_requests[-1]["mode"] == "sanity"
        assert world.calib_requests[-1]["speeds"] == [1.0]
        world.calib_r = 0.62                       # e.g. bad surface gone
        node._on_go(fake_ros.String(data=""))
        ok = _run(node, world, lambda: len(world.recipes) >= 1)
        assert ok, (node.phase, node._pause_reason, node.log[-15:])
        assert len(world.calib_requests) == 2
        assert cal.load_lock(cal.lock_path(bag_root))["epoch"] == lock["epoch"]
        assert node._stage_name() == "stage_2"
        assert world.max_alive_movers == 1
    finally:
        shutil.rmtree(tmp)


def test_repo_abort_pages_with_the_fallback_command_and_param_is_live():
    # Fails if two aborts in the window do not raise the card carrying the
    # exact command, or a live `ros2 param set` does not change the cap that
    # the next goto is sent with.
    tmp = tempfile.mkdtemp(prefix="hinf_flow_")
    try:
        node, world, plan, active, bag_root = _setup(tmp)
        node._note_repo_outcome(True)
        node._note_repo_outcome(False)
        assert not any(a["id"] == "repo_speed" for a in _alerts())
        node._note_repo_outcome(False)
        cards = [a for a in _alerts() if a["id"] == "repo_speed"]
        assert cards and "reposition_speed_mps 0.3" in cards[-1]["detail"]
        r = node.set_param_live("reposition_speed_mps", 0.3)
        assert r.successful and abs(node._repo_speed - 0.3) < 1e-9
        assert not node.set_param_live("reposition_speed_mps", 2.0).successful
        node._on_go(fake_ros.String(data=""))
        ok = _run(node, world, lambda: len(world.gotos) >= 1)
        assert ok and world.gotos[0]["v_const"] == 0.3
    finally:
        shutil.rmtree(tmp)


def test_start_refusals_are_loud():
    # Fails if a Start that is refused before any motion leaves the executor
    # silently IDLE (no pause reason, no /run/status, no card) — the review
    # finding of 2026-10-08: _pause() is a no-op from IDLE.
    tmp = tempfile.mkdtemp(prefix="hinf_flow_")
    try:
        node, world, plan, active, bag_root = _setup(tmp)
        # yesterday's browser plan: matrix legs, no figure-8, no lock
        with open(os.path.join(_REPO, "tools", "analysis", "tests",
                               "venue_fixtures", "active.json")) as f:
            old = json.load(f)
        with open(active, "w") as f:
            json.dump(old, f)
        node._on_go(fake_ros.String(data=""))
        assert node.phase.value == "paused", node.phase
        assert "no steering calibration yet" in (node._pause_reason or "")
        assert any(a["title"] == "START REFUSED" for a in _alerts())
        last = json.loads(fake_ros.BUS.sent["/run/status"][-1].data)
        assert last["phase"] == "paused" and last["pause_reason"]
    finally:
        shutil.rmtree(tmp)


def test_only_an_ungated_fixed_radius_list_skips_calibration():
    # Fails if a legacy fixed radius list (no 'auto', no calibration.required)
    # still schedules a figure-8 (and would write the one-time lock on its
    # first session), or if the gated fixed list does NOT ask for the full one.
    tmp = tempfile.mkdtemp(prefix="hinf_flow_")
    try:
        node, world, plan, active, bag_root = _setup(tmp)
        assert node._matrix_doc["_calib_gated"] and not node._matrix_doc["_radius_auto"]
        assert node._calibration_mode_due() == "full"
        node._matrix_doc = dict(node._matrix_doc, _calib_gated=False,
                                _matrix_lock=None)
        assert node._calibration_mode_due() is None
    finally:
        shutil.rmtree(tmp)


def test_first_pass_skips_retry_exhausted_cells():
    # Fails if ONE retry-exhausted cell keeps the whole batch at the first-
    # pass target forever (the batch would end DONE at ~1 rep per cell).
    tmp = tempfile.mkdtemp(prefix="hinf_flow_")
    try:
        node, world, plan, active, bag_root = _setup(tmp)
        lock, _e = cal.compute_lock(_calib_result("full", 0.60), cal.config({}))
        node._matrix_doc = cal.resolve_matrix(node._matrix_doc | {
            "matrix": dict(node._matrix_doc["matrix"], radius_m="auto")}, lock)
        node._matrix_doc["plan"] = {"first_pass_reps": 1}
        R = lock["radius_m"][0]
        node._stages = [{"name": "s", "legs": [
            {"id": "l", "curves": [{"kind": "recipe", "scored": True,
             "recipe": {"type": "step", "params": {"R": R}},
             "start_pose": {"lat": 0, "lon": 0, "heading_deg": 0}}]}]}]
        node._legs = node._stages[0]["legs"]
        node._target_n, node._max_retries = 10, 2
        node._controllers, node._speeds = ["lpv-hinf", "pid"], [1.0]
        k = lambda c: node._cell_key_for("step", R, c, 1.0)
        node._completed_counts = {k("lpv-hinf"): 1, k("pid"): 0}
        assert node._pass_target() == 1
        node._attempts = {("step", R, "pid", 1.0): 2}       # exhausted
        assert node._pass_target() == 10
    finally:
        shutil.rmtree(tmp)


def test_new_executor_never_reuses_a_calib_seq():
    # Fails if two executor processes start their calibration seq at the
    # same value (a restarted executor would take the still-alive node's
    # old 'done' status as its own result).
    tmp = tempfile.mkdtemp(prefix="hinf_flow_")
    try:
        node1, *_ = _setup(tmp)
        time.sleep(1.1)
        node2 = rex.RunExecutor()
        assert node2._cal_seq != node1._cal_seq
        node2._cal_status = {"seq": node2._cal_seq + 1, "state": "done"}
        node2._orch_status = {m: False for m in MOVERS}
        node2._orch_status["calib"] = True
        node2._tick_calib_start()
        assert node2._cal_status == {}, "stale calib status survived CALIB_START"
    finally:
        shutil.rmtree(tmp)


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print("PASS", name)
