# -*- coding: utf-8 -*-
"""run_executor_node — run an operator-authored leg batch with resume.

The operator authors a venue + an ordered list of LEGS in the WebUI (each leg =
a reposition glue curve + a scored experiment recipe), Sends it (venue_loader
persists active.json), then presses ONE Start button (``/run/go``). This node:

  - rescans the bag-root manifest to count which (controller, v_const, rep)
    cells already PASSED for each authored geometry (that is the resume
    mechanism — there is no leg checkpoint file),
  - re-checks containment of every curve BEFORE any motion,
  - runs a one-time preflight, then CYCLES the authored legs from leg 0,
    auto-advancing; each recipe traversal is assigned the least-done remaining
    treatment for its geometry (geometry-full legs are driven UNSCORED so the
    next reposition still lines up), until every geometry is filled or
    retry-exhausted OR the battery hits the halt threshold (then Start again
    rescans the manifest and continues filling the gaps).

Per curve:
  reposition -> reposition_node follows the DRAWN waypoints under RTK +
                /heading/fused to a deterministic end heading (FIXED or FLOAT ok);
  recipe     -> kill reposition (C6) -> ODOM_RESET (fresh (0,0,0) at the
                RTK-delivered start) -> require RTK FIXED(4) (post-hoc ground
                truth) -> bag -> follower set_params -> push analytic recipe ->
                wait /path_follower/done -> stop bag + sidecar.

The SINGLE MOST IMPORTANT INVARIANT (C6): exactly one cmd_vel_raw publisher is
live at any instant — the follower XOR reposition, never both. Enforced via
_start_exclusive_mover (kill the other and confirm it down via /orchestrator/
status before starting one). This node never publishes cmd_vel_raw itself.

No classify/retry — the operator is present: any leg failure -> PAUSED with a
reason + safe-state. RTK stays OUT of the control loop (ADR-01); it is the bag's
ground truth and a quality gate on scored runs only.

  sub  /run/go      std_msgs/String  (any message = Start / resume)
  sub  /run/cmd     std_msgs/String  JSON {action: pause|resume|abort}
  sub  /plan/request std_msgs/String JSON {venue}  (auto-planner; refused
                     while a batch is actively driving)
  pub  /plan/result  std_msgs/String JSON (latched) — experiment_planner
                     stages for the WebUI to render/accept
  pub  /run/status  std_msgs/String  JSON (latched)
  + the orchestrator / reposition / odom_zero / follower contracts (see imports).

Multi-stage batches (auto-planner): active.json may carry ``plan_stages`` =
[{name, legs}, ...] covering the whole remaining matrix. v.legs stays stage
1's legs for stage-unaware consumers. Start selects the FIRST stage with
remaining treatments (manifest-driven, restart-safe); when a stage fills,
the executor containment-gates the next stage and advances unattended.
"""
import json
import math
import os
import sys
import threading
import time
from collections import deque
from datetime import datetime, timezone
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import ParameterValue, ParameterType
from rcl_interfaces.msg import Parameter as ParameterMsg
from rclpy.qos import (
    QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy,
)
from std_msgs.msg import String, Bool
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix

# Recorder (T7) lives at the repo root, outside the colcon package. Import
# defensively (a path slip degrades to "cannot record" -> the scored leg pauses
# rather than crashing the node).
_REPO_ROOT = next(
    (p for p in [os.environ.get("H_INFINITY_ROOT"), "/home/agilex/H-infinity",
                 os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                              "..", "..", ".."))]
     if p and os.path.isdir(os.path.join(p, "scenarios", "venues"))),
    "/home/agilex/H-infinity")
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)
try:
    import Data_Logger  # type: ignore
except Exception:  # pragma: no cover
    Data_Logger = None

try:
    from limo_path_follower import venue_geom
except Exception:  # pragma: no cover
    import venue_geom

try:
    from limo_path_follower import experiment_planner
except Exception:  # pragma: no cover
    try:
        import experiment_planner  # type: ignore
    except Exception:
        experiment_planner = None

# Steering calibration (figure-8 -> one-time matrix lock), 2026-10-08.
try:
    from limo_path_follower import calibration
except Exception:  # pragma: no cover
    try:
        import calibration  # type: ignore
    except Exception:
        calibration = None

# manifest.py (tools/analysis) lives outside the colcon package. It is the source
# of truth for "which (controller, v_const, rep) cells already passed" — the
# system (not the operator) chooses the treatment for each authored geometry by
# reusing load_experiment/discover_legs/build_rows/cell_key here.
_TOOLS_ANALYSIS = os.path.join(_REPO_ROOT, "tools", "analysis")
if _TOOLS_ANALYSIS not in sys.path:
    sys.path.insert(0, _TOOLS_ANALYSIS)
try:
    import manifest  # type: ignore
except Exception:  # pragma: no cover
    manifest = None


PROC_REPOSITION = "reposition"
PROC_ODOM_ZERO = "odom_zero"
PROC_FOLLOWER = "follower"
# Open-loop figure-8 steering calibration (calib_node, 2026-10-08). A third
# cmd_vel_raw mover under the same C6 rule: exactly one of these is alive.
PROC_CALIB = "calib"
MOVERS = (PROC_REPOSITION, PROC_FOLLOWER, PROC_CALIB)

# The calibration bag records the D1 set plus the calib node's own status.
CALIB_EXTRA_TOPICS = ["/calib/status"]

RTK_FIXED = 4
RTK_OK = (4, 5)

# Support PROCs the executor brings up ITSELF at preflight, so pressing Start is
# sufficient and no bring-up step can be silently skipped (field 2026-09-16: the
# operator had to remember eight orchestrator names by hand; forgetting 'heading'
# makes reposition HOLD at zero output with no explanation, and forgetting a
# watchdog removes the only thing that would have reported the dead RTK link).
# Deliberately NOT here: base / gnss / estop (they open hardware and live in
# exclusive groups) and the movers (handled C6-safely by _start_exclusive_mover).
#   heading       -> reposition HOLDs without /heading/fused: a correctness dep.
#   rtk_watchdog  -> the fix-health page + auto-pause.
#   odom_watchdog -> base-serial dropout recovery.
#   odom_zero     -> /wheel/odom_zeroed, re-anchored by us every leg; the
#                    follower waits forever without it. Pure republisher.
SUPPORT_PROCS = ("heading", "rtk_watchdog", "odom_watchdog", "odom_zero")
SUPPORT_RETRY_S = 3.0

DEFAULT_ACTIVE = os.path.join(_REPO_ROOT, "scenarios", "venues", "active.json")
DEFAULT_BAG_ROOT = os.path.join(_REPO_ROOT, "Experiment Data")

# Neutral recipe published (latched) just before each follower spawn so a fresh
# follower replays a harmless clear instead of the PREVIOUS leg's curve (which
# would start it driving at default params before SET_PARAMS/PUSH_RECIPE).
NEUTRAL_RECIPE = json.dumps({"type": "none"})

# Operator paging (Discord webhook via tools/notify/ntfy.py, discord.env).
# Imported defensively like odom_watchdog: a missing module/webhook only
# disables paging, it can never interrupt a batch. Field rule 2026-06-10:
# every operator-actionable warning goes to BOTH channels — the browser card
# and Discord (this). Browser cards come from /run/status (pause / abort /
# done), /limo_status (battery) and, for events with no status field of their
# own (stage advance, anchor warning), /operator/alert via _operator_alert().
# Discord itself is switched off in ntfy.py (DISCORD_ENABLED, 2026-10-02).
try:
    sys.path.insert(0, os.path.join(_REPO_ROOT, "tools", "notify"))
    from ntfy import notify_discord as _notify_discord  # type: ignore
except Exception:  # noqa: BLE001
    def _notify_discord(*_a, **_k):
        return False

# Battery warn threshold (volts) for the once-per-crossing Discord page;
# matches preflight.sh / ntfy.py conventions (warn 10.8, halt 10.5).
BATT_WARN_V = 10.8

# Arrival attribution (sidecar "arrival", 2026-10-04). Field analysis found the
# robot reaches the start pin a median 7 deg (p90 12 deg, RTK) off its heading,
# but the sidecar did not say which glue it arrived on, so only 9 of 56 legs
# could be matched to glue geometry. Every scored leg now records the glue as
# sent to reposition plus these metrics. The tail threshold mirrors the
# planner's settle_align_deg default (experiment_planner._glue_quality) and
# tools/analysis/plan_report.py, so plan-time and run-time numbers compare.
ARRIVAL_TAIL_ALIGN_DEG = 5.0


def glue_arrival_metrics(waypoints_wgs84, end_heading_deg,
                         align_deg=ARRIVAL_TAIL_ALIGN_DEG):
    """Geometry of the glue polyline the robot arrived on (pure, no ROS).

    Projects the waypoints EXACTLY as sent to reposition into a local EN frame
    about waypoint 0 (venue_geom's equirectangular model; metrics are frame-
    invariant) and reuses the planner's own helpers, so these are the same
    numbers the planner optimized:
      max_curvature_1pm / min_turn_radius_m  Menger curvature over consecutive
          waypoint triples (_max_curvature). A straight glue has curvature 0
          and min_turn_radius_m None (infinite; strict JSON has no Infinity).
      tail_straight_m  the final stretch whose segment bearing stays within
          align_deg of end_heading_deg (_tail_straight): the straight the
          tracker settles its heading on before the pin. None when the glue
          has no explicit end heading.
      length_m  polyline length.
    Raises on malformed input or a missing planner — the caller guards it.
    """
    wps = list(waypoints_wgs84 or [])
    if not wps:
        raise ValueError("glue has no waypoints")
    lat0, lon0 = float(wps[0]["lat"]), float(wps[0]["lon"])
    pts = [venue_geom.latlon_to_en(float(w["lat"]), float(w["lon"]), lat0, lon0)
           for w in wps]
    length = sum(math.hypot(b[0] - a[0], b[1] - a[1])
                 for a, b in zip(pts[:-1], pts[1:]))
    k = float(experiment_planner._max_curvature(pts))
    tail = None
    if end_heading_deg is not None:
        tail = float(experiment_planner._tail_straight(
            pts, float(end_heading_deg) % 360.0, float(align_deg)))
    return {
        "n_waypoints": len(pts),
        "length_m": round(length, 3),
        "max_curvature_1pm": round(k, 4),
        "min_turn_radius_m": round(1.0 / k, 3) if k > 1e-9 else None,
        "tail_straight_m": round(tail, 3) if tail is not None else None,
        "tail_align_deg": float(align_deg),
    }


# Chassis driver configuration (2026-10-07). limo_base publishes it latched on
# /limo_base/config. The stock driver delivered ~0.4x steering and a crabbed,
# deadbanded odometry; a driver that comes back stock (old binary, other launch
# params, a respawn) is a different plant and must not produce scored legs.
def driver_config_problems(cfg, expect):
    """Reasons the parsed /limo_base/config `cfg` (dict, or None if never
    received) fails `expect` ({key: required value}; empty values are not
    checked). [] means OK."""
    if not isinstance(cfg, dict):
        return ["no /limo_base/config from the chassis driver "
                "(pre-2026-10-07 driver, or not running)"]
    out = []
    for key, want in expect.items():
        if want in (None, ""):
            continue
        if cfg.get(key) != want:
            out.append(f"driver {key}={cfg.get(key)!r}, expected {want!r}")
    return out


def leg_driver_config_verdict(at_start, at_end, expect):
    """(ok, reasons) for one scored leg. The config must satisfy `expect` at
    both ends and be identical across the leg: a changed node_start_unix is a
    driver respawn (odometry restarted at the origin mid-leg), any other change
    a runtime parameter set."""
    reasons = [f"start: {r}" for r in driver_config_problems(at_start, expect)]
    reasons += [f"end: {r}" for r in driver_config_problems(at_end, expect)]
    if (isinstance(at_start, dict) and isinstance(at_end, dict)
            and at_start != at_end):
        reasons.append("driver config changed during the leg "
                       "(respawn or runtime parameter set)")
    return (not reasons), reasons


class Phase(Enum):
    IDLE = "idle"
    PREFLIGHT = "preflight"
    REPOSITION_START = "reposition_start"
    REPOSITION_GOTO = "reposition_goto"
    KILL_REPOSITION = "kill_reposition"
    ODOM_RESET = "odom_reset"
    BAG_START = "bag_start"
    FOLLOWER_START = "follower_start"
    SET_PARAMS = "set_params"
    PUSH_RECIPE = "push_recipe"
    RUN = "run"
    CALIB_START = "calib_start"
    CALIB_RUN = "calib_run"
    REPLAN = "replan"
    STOP_LEG = "stop_leg"
    DONE = "done"
    PAUSED = "paused"
    ABORTED = "aborted"


class RunExecutor(Node):

    def __init__(self):
        super().__init__("run_executor_node")

        self.declare_parameter("active_venue_file", DEFAULT_ACTIVE)
        self.declare_parameter("bag_root", DEFAULT_BAG_ROOT)
        self.declare_parameter(
            "experiment_yaml",
            os.path.join(_REPO_ROOT, "scenarios", "experiment.yaml"))
        self.declare_parameter("preflight_timeout_s", 60.0)
        self.declare_parameter("reposition_timeout_s", 120.0)
        self.declare_parameter("run_timeout_s", 180.0)
        self.declare_parameter("orchestrator_settle_s", 3.0)
        # Confirm window for the odom_zero reset, timed from the FIRST reset
        # send (not phase entry). Generous: it must absorb a cold odom_zero
        # spawn whose subscription appears 1-2 s after the proc is "alive".
        self.declare_parameter("odom_settle_s", 10.0)
        self.declare_parameter("rtk_fix_wait_s", 30.0)       # scored-run FIXED gate
        self.declare_parameter("rtk_loss_wait_s", 5.0)       # FIXED loss during scored run
        self.declare_parameter("heartbeat_s", 30.0)
        self.declare_parameter("battery_volts_halt", 10.5)
        # Chassis driver config every scored leg requires (/limo_base/config);
        # "" disables that key's check.
        self.declare_parameter("expect_steering_mode", "direct")
        self.declare_parameter("expect_odom_model", "hinf")
        self.declare_parameter("rtk_run_window_pct", 95.0)
        self.declare_parameter("robot_footprint_radius_m", 0.30)
        self.declare_parameter("path_tracking_margin_m", 0.30)
        # Max acceptable |heading error| (deg) reported by reposition at arrival.
        # Larger => the analytic recipe would run ROTATED by that error in the
        # world (odom is zeroed at the achieved heading), leaving the corridor
        # the containment check verified -> pause for the operator instead.
        self.declare_parameter("arrival_heading_tol_deg", 20.0)
        # Operator breathing room between consecutive curves (exp<->rep): the
        # robot sits still, heading/RTK settle, and the operator can eyeball
        # alignment before the next maneuver starts (field request 2026-06-10).
        self.declare_parameter("inter_curve_dwell_s", 5.0)
        # Glue (reposition) cruise speed cap. 0.40 m/s oscillated on a tight
        # hook while the heading EKF was being dragged by a relapsing FCU
        # (field 2026-06-10); 0.2 gave pure pursuit and COG twice the time.
        # 2026-10-08 -> 0.4 (operator): with full steering (driver direct
        # mode) a replay of the 71 repositions of 2026-10-06 through the real
        # reposition code arrives identically at 0.2/0.4/0.5 in half the time.
        # LIVE-settable (ros2 param set /run_executor_node
        # reposition_speed_mps 0.3): takes effect at the next reposition.
        # Repeated reposition aborts raise an operator card suggesting 0.3.
        self.declare_parameter("reposition_speed_mps", 0.4)
        self.declare_parameter("repo_abort_window", 5)
        self.declare_parameter("repo_abort_alert_n", 2)
        # Calibration figure-8: hard cap on the whole open-loop run.
        self.declare_parameter("calib_timeout_s", 240.0)
        # Post-calibration re-plan (pure geometry, threaded): give up after.
        self.declare_parameter("replan_timeout_s", 420.0)

        self._active_file = str(self.get_parameter("active_venue_file").value)
        self._bag_root = str(self.get_parameter("bag_root").value)
        self._experiment_yaml = str(self.get_parameter("experiment_yaml").value)
        self._preflight_timeout = float(self.get_parameter("preflight_timeout_s").value)
        self._reposition_timeout = float(self.get_parameter("reposition_timeout_s").value)
        self._run_timeout = float(self.get_parameter("run_timeout_s").value)
        self._settle_s = float(self.get_parameter("orchestrator_settle_s").value)
        self._odom_settle_s = float(self.get_parameter("odom_settle_s").value)
        self._rtk_fix_wait = float(self.get_parameter("rtk_fix_wait_s").value)
        self._rtk_loss_wait = float(self.get_parameter("rtk_loss_wait_s").value)
        self._heartbeat_s = float(self.get_parameter("heartbeat_s").value)
        self._batt_halt = float(self.get_parameter("battery_volts_halt").value)
        self._driver_expect = {
            "steering_mode": str(self.get_parameter("expect_steering_mode").value),
            "odom_model": str(self.get_parameter("expect_odom_model").value),
        }
        self._rtk_window_pct = float(self.get_parameter("rtk_run_window_pct").value)
        self._footprint_r = float(self.get_parameter("robot_footprint_radius_m").value)
        self._track_margin = float(self.get_parameter("path_tracking_margin_m").value)
        self._arrival_head_tol = float(
            self.get_parameter("arrival_heading_tol_deg").value)
        self._dwell_s = float(self.get_parameter("inter_curve_dwell_s").value)
        self._repo_speed = float(self.get_parameter("reposition_speed_mps").value)
        self._repo_outcomes = deque(
            maxlen=max(1, int(self.get_parameter("repo_abort_window").value)))
        self._repo_alert_n = int(self.get_parameter("repo_abort_alert_n").value)
        self._repo_alerted = False
        self._calib_timeout = float(self.get_parameter("calib_timeout_s").value)
        self._replan_timeout = float(self.get_parameter("replan_timeout_s").value)
        self.add_on_set_parameters_callback(self._on_set_params)
        self._dwell_until = 0.0
        self._batt_warned = False

        # -- Batch state -----------------------------------------------
        self._venue = {}
        self._legs = []
        self._stages = []        # [{name, legs}] from active.json plan_stages
        self._stage_idx = 0      # index into _stages (when non-empty)
        self._run_id = "run"
        self._leg_idx = 0
        self._curve_idx = 0
        self._cur_curve = None
        self._last_completed_leg_idx = None  # park position for stage transit
        self._transit_curve = None           # one-shot synthetic repo curve
        self._cur_recipe = {}
        self._matrix_doc = {}    # raw experiment.yaml (planner input)
        # Steering calibration (2026-10-08). _matrix_doc carries the lock the
        # matrix is gated on (manifest.load_experiment resolves radius_m: auto
        # and fixed lists with calibration.required).
        self._cal_cfg = dict(calibration.DEFAULTS) if calibration else {}
        self._cal_done_session = False   # a figure-8 passed since this Start
        self._cur_calib = False          # the current recipe is the figure-8
        self._cal_mode = None            # 'full' | 'sanity' for the running one
        # Seeded from the clock: a restarted executor must never reuse a seq a
        # still-alive calib_node already finished (its latched status would be
        # taken as THIS run's result).
        self._cal_seq = int(time.time()) % 1000000000
        self._cal_req = None
        self._cal_req_sent = False
        self._cal_req_last_t = 0.0
        self._cal_status = {}
        self._cal_status_t = None
        self._cal_result = None
        self._cal_pin = None
        self._replan_thread = None
        self._replan_out = None
        self._replan_gen = 0
        self._replan_needed = False

        # -- Matrix / treatment sweep (system chooses controller x v x rep) ---
        # The operator authors only GEOMETRY (step shape at radius R); the system
        # fills controller x v_const x rep(N) per geometry from experiment.yaml +
        # the success manifest. counts/attempts are keyed off manifest cell_key.
        self._controllers = []          # matrix.controller, e.g. [lpv-hinf, pid]
        self._speeds = []               # matrix.v_const,    e.g. [1.0, 0.5]
        self._target_n = 0              # repetitions per cell
        self._max_retries = 2           # retry.max_retries (per-cell classification)
        self._breaker_k = 3             # retry.circuit_breaker_k (consecutive)
        self._completed_counts = {}     # cell_key -> #passing runs (this venue)
        self._progress = None           # whole-matrix X/320 dashboard summary
        self._attempts = {}             # (fam,R,c,v) -> consecutive failed attempts
        self._consec_fail = 0           # consecutive scored-run classification fails
        self._cur_treatment = None      # {controller, v_const, rep} or None (glue)
        self._cur_scored = False        # effective scored flag for the current curve
        self._leg_cell_id = None        # treatment-qualified cell id (bag dir + sidecar)

        self.phase = Phase.IDLE
        self._phase_entered = time.monotonic()
        self._pause_reason = None
        self._last_heartbeat = time.monotonic()

        # -- Per-run scratch -------------------------------------------
        self._recorder = None
        self._leg_bag_path = None
        self._leg_start_utc = None
        self._leg_estopped = False
        self._leg_rtk_fixed_samples = 0
        self._leg_rtk_total_samples = 0
        self._run_end_reason = None
        self._goto_sent = False
        self._goto_seq = 0            # monotonically increasing goto id; reposition
                                      # echoes it so stale 'arrived' can't be honored
        self._goto_last_send_t = 0.0  # monotonic time of last goto (re-)send
        self._params_sent = False
        self._params_future = None
        self._params_sent_t = 0.0       # when the live set_parameters call went out
        self._params_retries = 0        # re-sends after a dropped service response
        self._set_params_timeout = 5.0  # re-send a stalled set_parameters call
        self._set_params_max_retries = 3
        self._odom_reset_sent = False
        self._odom_reset_command_t = None       # ROS time of first send (confirm gate)
        self._odom_reset_first_sent_t = None    # monotonic time of first send (timeout)
        self._rtk_lost_since = None

        # -- Sensor snapshots ------------------------------------------
        self._done = False
        self._estop = False
        self._rtk_quality = None
        self._battery_v = None
        self._driver_cfg = None            # parsed /limo_base/config (latched)
        self._leg_driver_cfg_start = None  # snapshot when the leg's bag starts
        self._orch_status = {}
        self._support_start_t = {}    # PROC -> last /orchestrator/start we sent
        self._repo_state = None
        self._repo_status = {}
        self._odom_zero_status = None
        self._last_odom_t = None
        # Achieved-anchor inputs: fused heading health (heading_node JSON) and
        # last RTK fix, each with a monotonic receive time for staleness checks.
        self._heading_status = None
        self._heading_status_t = None
        self._last_fix = None           # (lat, lon)
        self._last_fix_t = None
        self._achieved_anchor = None    # snapshot taken at odom-zero confirm
        # Arrival attribution: the goto payload last sent, the glue snapshot
        # taken when reposition's 'arrived' is accepted (consumed by the next
        # recipe's odom reset), and the sidecar 'arrival' record built there.
        self._repo_status_t = None      # monotonic receive time of _repo_status
        self._goto_payload = None
        self._arrival_glue = None
        self._arrival = None

        # -- ROS interfaces --------------------------------------------
        latched = QoSProfile(
            depth=1, history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        self.pub_status = self.create_publisher(String, "/run/status", latched)
        self.pub_start = self.create_publisher(String, "/orchestrator/start", 10)
        self.pub_kill = self.create_publisher(String, "/orchestrator/kill", 10)
        self.pub_goto = self.create_publisher(String, "/reposition/goto", 10)
        self.pub_odom_reset = self.create_publisher(Bool, "/odom_zero/reset", 10)
        self.pub_recipe = self.create_publisher(
            String, "/reference_path_recipe", latched)
        self.pub_plan = self.create_publisher(String, "/plan/result", latched)
        self.pub_calib_req = self.create_publisher(String, "/calib/request", 10)
        # Operator cards in the battle-station browser (/operator/alert JSON).
        # Latched with a short history so a reconnecting browser replays the
        # recent events (a later "clear" for the same id cancels its card).
        self.pub_alert = self.create_publisher(
            String, "/operator/alert",
            QoSProfile(depth=10, history=QoSHistoryPolicy.KEEP_LAST,
                       reliability=QoSReliabilityPolicy.RELIABLE,
                       durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))

        self.create_subscription(String, "/run/go", self._on_go, 10)
        self.create_subscription(String, "/run/cmd", self._on_cmd, 10)
        self.create_subscription(String, "/plan/request", self._on_plan_request, 10)
        self.create_subscription(
            String, "/orchestrator/status", self._on_orch_status, 10)
        self.create_subscription(
            String, "/reposition/status", self._on_repo_status, 10)
        self.create_subscription(Bool, "/path_follower/done", self._on_done, latched)
        self.create_subscription(Bool, "/estop", self._on_estop, 10)
        self.create_subscription(
            String, "/gps_rtk_f9p_helical/gps/rtk_status", self._on_rtk, 10)
        self.create_subscription(Odometry, "/wheel/odom", self._on_odom, 10)
        self.create_subscription(
            String, "/limo_base/config", self._on_driver_config, latched)
        self.create_subscription(
            String, "/odom_zero/status", self._on_odom_zero_status, latched)
        self.create_subscription(
            String, "/heading/fused_status", self._on_heading_status, latched)
        self.create_subscription(
            NavSatFix, "/gps_rtk_f9p_helical/gps/fix", self._on_fix, 10)
        self.create_subscription(
            String, "/calib/status", self._on_calib_status, latched)
        try:
            from limo_msgs.msg import LimoStatus  # type: ignore
            self.create_subscription(LimoStatus, "/limo_status", self._on_limo, 10)
        except Exception:
            self.get_logger().warn(
                "limo_msgs not importable — battery gating disabled")

        self._param_cli = self.create_client(
            SetParameters, "/path_follower_node/set_parameters")

        self.create_timer(0.2, self._tick)

        # Load whatever is on disk so a fresh WebUI connect sees progress.
        self._reload_active()
        if self._load_matrix():
            self._rebuild_counts()
        self.get_logger().info(
            f"run_executor_node up. active={self._active_file}: "
            f"{len(self._legs)} legs, {self._runs_done()}/{self._runs_target()} "
            f"scored runs done. Waiting for /run/go (Start).")
        self._publish_status(message="loaded")

    # ==================================================================
    # Subscriptions
    # ==================================================================

    def _on_go(self, msg):
        if self.phase == Phase.IDLE:
            self._begin_run()
        elif self.phase == Phase.PAUSED:
            self._resume()
        elif self.phase in (Phase.DONE, Phase.ABORTED):
            # Not terminal: Start re-reads active.json + the manifest and runs
            # whatever remains (re-enters DONE cleanly if nothing does). This is
            # what lets a re-Send venue / post-abort session continue without
            # restarting this node.
            self.get_logger().info(
                f"Start from {self.phase.value}: reloading venue + manifest.")
            self._start_or_resume()
        else:
            self.get_logger().info(f"Start ignored — phase={self.phase.value}")

    def _on_cmd(self, msg):
        try:
            d = json.loads(msg.data)
        except (ValueError, TypeError):
            self.get_logger().warn(f"bad /run/cmd: {msg.data!r}")
            return
        action = str(d.get("action", "")).lower().strip()
        if action == "pause":
            # Optional reason so an automated guard (rtk_watchdog) pages with
            # WHY, instead of the batch reading "operator pause".
            self._pause(str(d.get("reason") or "operator pause"))
        elif action == "resume":
            # if_reason_prefix lets a guard resume ONLY the pause it caused: an
            # operator pause, or another guard's, is never overridden.
            want = d.get("if_reason_prefix")
            if want and not str(self._pause_reason or "").startswith(str(want)):
                self.get_logger().info(
                    f"resume ignored: pause reason {self._pause_reason!r} does "
                    f"not match {want!r}")
                return
            if self.phase == Phase.PAUSED:
                self._resume()
        elif action == "abort":
            self._abort("operator abort")
        else:
            self.get_logger().warn(f"unknown /run/cmd action: {action!r}")

    def _on_orch_status(self, msg):
        try:
            self._orch_status = json.loads(msg.data) or {}
        except (ValueError, TypeError):
            pass

    def _on_plan_request(self, msg):
        """Auto-plan request from the WebUI: {venue: {...}} in, the planner's
        stage list out on /plan/result (latched). Planning is synchronous
        (~18-20 s request->result for the full matrix, measured on the NUC
        2026-10-03) so it is REFUSED while a batch is actively driving — the
        tick state machine must not stall under a moving robot."""
        def _fail(why):
            self.get_logger().warn(f"/plan/request refused: {why}")
            self.pub_plan.publish(String(data=json.dumps(
                {"ok": False, "stages": [], "unfittable": [], "notes": [why]})))

        if self.phase not in (Phase.IDLE, Phase.PAUSED, Phase.DONE,
                              Phase.ABORTED):
            _fail(f"batch is active (phase={self.phase.value}) — "
                  "pause or finish before planning")
            return
        if experiment_planner is None or manifest is None:
            _fail("planner/manifest tooling unavailable on this host")
            return
        try:
            req = json.loads(msg.data) or {}
        except (ValueError, TypeError) as exc:
            _fail(f"bad /plan/request JSON: {exc}")
            return
        venue = req.get("venue") or {}
        if len(venue.get("corners_wgs84") or []) < 3:
            _fail("request venue has no polygon (corners_wgs84)")
            return
        if not self._load_matrix():
            _fail("experiment.yaml unreadable — cannot plan")
            return
        run_id = str(venue.get("name") or "run")
        counts = self._counts_for(run_id)
        # Steering calibration first (2026-10-08): 'full' until the matrix is
        # locked, a short 'sanity' figure-8 once a session; None skips it.
        mode = self._calibration_mode_due()
        try:
            plan = experiment_planner.plan_stages(
                venue, self._matrix_doc, counts,
                footprint_r=self._footprint_r,
                track_margin=self._track_margin,
                key_fn=manifest.cell_key,
                calibration=({"mode": mode} if mode else None))
        except Exception as exc:  # noqa: BLE001 - never die in a callback
            _fail(f"planner crashed: {exc!r}")
            return
        plan["venue_name"] = run_id
        plan["counts_runs_done"] = sum(counts.values())
        self.pub_plan.publish(String(data=json.dumps(plan)))
        self.get_logger().info(
            f"plan for '{run_id}': {len(plan.get('stages') or [])} stage(s), "
            f"{len(plan.get('unfittable') or [])} unfittable.")

    def _on_repo_status(self, msg):
        try:
            d = json.loads(msg.data)
            self._repo_status = d if isinstance(d, dict) else {}
            self._repo_status_t = time.monotonic()
            self._repo_state = str(d.get("state", "")).lower().strip()
        except (ValueError, TypeError):
            pass

    def _on_done(self, msg):
        self._done = bool(msg.data)

    def _on_estop(self, msg):
        was = self._estop
        self._estop = bool(msg.data)
        if self._estop and not was and self.phase == Phase.RUN:
            self._leg_estopped = True

    def _on_rtk(self, msg):
        q = None
        for tok in (msg.data or "").replace("(", " ").replace(",", " ").split():
            if tok.startswith("quality="):
                try:
                    q = int(tok.split("=", 1)[1])
                except ValueError:
                    q = None
                break
        self._rtk_quality = q
        if self.phase == Phase.RUN:
            self._leg_rtk_total_samples += 1
            if q == RTK_FIXED:
                self._leg_rtk_fixed_samples += 1

    def _on_odom(self, msg):
        self._last_odom_t = time.monotonic()

    def _on_calib_status(self, msg):
        try:
            d = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        if isinstance(d, dict):
            self._cal_status = d
            self._cal_status_t = time.monotonic()

    def _on_set_params(self, params):
        """Live parameter updates. reposition_speed_mps is the field knob
        (DOC/agent_field_runbook.md §7): applied at the next reposition."""
        from rcl_interfaces.msg import SetParametersResult
        for prm in params:
            if prm.name == "reposition_speed_mps":
                try:
                    v = float(prm.value)
                except (TypeError, ValueError):
                    return SetParametersResult(
                        successful=False, reason="reposition_speed_mps: not a number")
                if not (0.1 <= v <= 0.6):
                    return SetParametersResult(
                        successful=False,
                        reason="reposition_speed_mps must be within [0.1, 0.6] m/s")
        for prm in params:
            if prm.name == "reposition_speed_mps":
                old, self._repo_speed = self._repo_speed, float(prm.value)
                self._repo_outcomes.clear()
                self._repo_alerted = False
                self.get_logger().warn(
                    f"reposition_speed_mps {old:.2f} -> {self._repo_speed:.2f} m/s "
                    "(applies from the next reposition)")
                self._operator_alert(
                    "repo_speed", "info", "REPOSITION SPEED",
                    f"now {self._repo_speed:.2f} m/s (was {old:.2f}).")
        return SetParametersResult(successful=True)

    def _on_driver_config(self, msg):
        try:
            d = json.loads(msg.data)
        except (ValueError, TypeError):
            d = None
        self._driver_cfg = d if isinstance(d, dict) else None

    def _on_limo(self, msg):
        try:
            self._battery_v = float(msg.battery_voltage)
        except Exception:
            return
        # Once-per-crossing low-battery page (browser card comes from the
        # WebUI's own battery watch; this is the Discord half of the rule).
        if self._battery_v < BATT_WARN_V and not self._batt_warned:
            self._batt_warned = True
            _notify_discord(
                f"LIMO battery LOW: {self._battery_v:.2f} V "
                f"(warn {BATT_WARN_V}, halt {self._batt_halt}).",
                title="H-inf run_executor")
        elif self._battery_v > BATT_WARN_V + 0.2 and self._batt_warned:
            self._batt_warned = False

    def _on_odom_zero_status(self, msg):
        try:
            self._odom_zero_status = json.loads(msg.data) or {}
        except (ValueError, TypeError):
            pass

    def _on_heading_status(self, msg):
        try:
            self._heading_status = json.loads(msg.data) or {}
            self._heading_status_t = time.monotonic()
        except (ValueError, TypeError):
            pass

    def _on_fix(self, msg):
        # NavSatFix status -1 = no fix; lat/lon would be garbage.
        if msg.status.status < 0:
            return
        self._last_fix = (float(msg.latitude), float(msg.longitude))
        self._last_fix_t = time.monotonic()

    def _ros_now_s(self):
        return float(self.get_clock().now().nanoseconds) * 1e-9

    # ==================================================================
    # Orchestrator + C6 exclusivity (copied idiom from the sequencer)
    # ==================================================================

    def _orch_start(self, name):
        self.pub_start.publish(String(data=name))

    def _orch_kill(self, name):
        self.pub_kill.publish(String(data=name))

    def _is_alive(self, name):
        return bool(self._orch_status.get(name, False))

    def _other_movers(self, name):
        return [m for m in MOVERS if m != name]

    def _start_exclusive_mover(self, name):
        """Bring up one cmd_vel_raw mover, guaranteeing C6 (every OTHER mover
        is reported down by the orchestrator first). Returns
        'killing_other' | 'waiting_other_down' | 'started'."""
        others = self._other_movers(name)
        alive = [m for m in others if self._is_alive(m)]
        if alive:
            for m in alive:
                self._orch_kill(m)
            return "killing_other"
        # Wait until the orchestrator has reported every other mover down.
        # 'calib' is exempt: a PROC table without it (older orchestrator)
        # can never have it alive.
        unknown = [m for m in others
                   if m not in self._orch_status and m != PROC_CALIB]
        if unknown:
            return "waiting_other_down"
        if not self._is_alive(name):
            self._orch_start(name)
        return "started"

    def _set_follower_params(self, controller_type, v_const):
        if not self._param_cli.service_is_ready():
            return None
        req = SetParameters.Request()
        p_ctrl = ParameterMsg()
        p_ctrl.name = "controller_type"
        p_ctrl.value = ParameterValue(
            type=ParameterType.PARAMETER_STRING, string_value=str(controller_type))
        p_v = ParameterMsg()
        p_v.name = "v_const"
        p_v.value = ParameterValue(
            type=ParameterType.PARAMETER_DOUBLE, double_value=float(v_const))
        req.parameters = [p_ctrl, p_v]
        return self._param_cli.call_async(req)

    # ==================================================================
    # Batch loading / resume
    # ==================================================================

    def _reload_active(self):
        try:
            with open(self._active_file, "r", encoding="utf-8") as f:
                v = json.load(f)
        except (FileNotFoundError, ValueError):
            self._venue, self._legs, self._run_id = {}, [], "run"
            self._stages, self._stage_idx = [], 0
            return
        self._venue = v
        self._run_id = str(v.get("name") or "run")
        # Multi-stage plan (auto-planner): plan_stages = [{name, legs}, ...].
        # v.legs stays the FIRST stage's legs for backward compatibility, so
        # a stage-unaware consumer still sees a valid single-stage batch.
        self._stages = [
            {"name": str(st.get("name") or f"stage_{i + 1}"),
             "legs": st.get("legs") or [],
             # Planner-emitted inter-stage transit glue (C, 2026-06-11): the
             # curve from the PREVIOUS stage's exit pose into this stage's
             # first start pin. Optional — absent means path-join fallback.
             "entry_glue": st.get("entry_glue"),
             # First-pass wrap (2026-10-08): last stage -> this one.
             "wrap_glue": st.get("wrap_glue")}
            for i, st in enumerate(v.get("plan_stages") or [])
            if st.get("legs")]
        self._stage_idx = 0
        self._legs = (self._stages[0]["legs"] if self._stages
                      else (v.get("legs") or []))

    # -- Matrix + manifest (the treatment brain) -----------------------

    def _load_matrix(self):
        """Read controller/v_const/repetitions/retry from experiment.yaml.
        Returns True iff a usable matrix was loaded."""
        if manifest is None:
            self.get_logger().error("manifest tooling unavailable — cannot sweep")
            return False
        try:
            _expected, reps, doc = manifest.load_experiment(
                self._experiment_yaml, bag_root=self._bag_root)
        except TypeError:   # pre-2026-10-08 manifest (no bag_root): fixed radii
            _expected, reps, doc = manifest.load_experiment(self._experiment_yaml)
        except Exception as exc:
            self.get_logger().error(f"load_experiment failed: {exc}")
            return False
        self._matrix_doc = doc or {}
        if calibration is not None:
            self._cal_cfg = calibration.config(self._matrix_doc)
        matrix = doc.get("matrix") or {}
        self._controllers = [str(c) for c in (matrix.get("controller") or [])]
        self._speeds = [float(v) for v in (matrix.get("v_const") or [])]
        self._target_n = int(reps or 0)
        retry = doc.get("retry") or {}
        self._max_retries = int(retry.get("max_retries", 2))
        self._breaker_k = int(retry.get("circuit_breaker_k", 3))
        if not self._controllers or not self._speeds or self._target_n <= 0:
            self.get_logger().error(
                "experiment.yaml matrix missing controller/v_const/repetitions")
            return False
        return True

    def _counts_for(self, run_id):
        """Passing scored runs for a given run_id from the bag-root manifest,
        keyed by cell_key(controller, v_const, path_family, radius_m)."""
        counts = {}
        if manifest is None:
            return counts
        try:
            legs = manifest.discover_legs(self._bag_root)
            rows = manifest.build_rows(legs)
        except Exception as exc:
            self.get_logger().warn(f"manifest scan failed: {exc}")
            return counts
        gated, epoch = self._epoch_filter()
        for r in rows:
            if r.get("sidecar_pass") is not True:
                continue
            if str(r.get("run_id")) != str(run_id):
                continue          # other venue/batch — must not mark us done
            if gated and (epoch is None or r.get("matrix_epoch") != epoch):
                continue          # another lock's matrix (e.g. stock-driver legs)
            # Bag-level re-gate: pre-2026-06-12 sidecars say pass on
            # recorder-broken bags; those cells must be refilled, not skipped.
            if hasattr(manifest, "quick_gate") and not manifest.quick_gate(
                    r["bag_dir"])["pass"]:
                continue
            k = manifest.cell_key({
                "controller": r.get("controller"), "v_const": r.get("v_const"),
                "path_family": r.get("path_family"), "radius_m": r.get("radius_m")})
            if k is not None:
                counts[k] = counts.get(k, 0) + 1
        return counts

    def _rebuild_counts(self):
        """Count passing scored runs for THIS venue (the resume mechanism)."""
        self._completed_counts = self._counts_for(self._run_id)
        self._progress = self._global_progress()

    def _global_progress(self):
        """Whole-matrix dataset progress (the X/320 dashboard number).

        Cross-venue by design: the paper dataset accumulates over sessions, so
        usable legs are counted over the WHOLE bag-root manifest against the
        experiment.yaml expected-cell set, capped at N per cell. `sidecar_pass`
        embeds the bag-level quick gate for every leg recorded from 2026-06-12
        on, so this number only counts legs whose recording survived.
        """
        if manifest is None:
            return None
        try:
            try:
                expected, reps, _doc = manifest.load_experiment(
                    self._experiment_yaml, bag_root=self._bag_root)
            except TypeError:
                expected, reps, _doc = manifest.load_experiment(
                    self._experiment_yaml)
            rows = manifest.build_rows(manifest.discover_legs(self._bag_root))
        except Exception as exc:
            self.get_logger().warn(f"progress scan failed: {exc}")
            return None
        n = int(reps or self._target_n or 0)
        if not expected or n <= 0:
            return None
        per_cell = {k: 0 for k in expected}
        failed = 0
        gated, epoch = self._epoch_filter()
        for r in rows:
            if gated and (epoch is None or r.get("matrix_epoch") != epoch):
                continue
            k = manifest.cell_key({
                "controller": r.get("controller"), "v_const": r.get("v_const"),
                "path_family": r.get("path_family"),
                "radius_m": r.get("radius_m")})
            if k not in per_cell:
                continue        # glue/turnaround/off-matrix legs don't count
            # Re-run the bag quick gate even for legs whose sidecar predates
            # it (pre-2026-06-12 sidecars say pass on recorder-broken bags).
            if (r.get("sidecar_pass") is True
                    and manifest.quick_gate(r["bag_dir"])["pass"]):
                per_cell[k] += 1
            else:
                failed += 1
        usable = sum(min(c, n) for c in per_cell.values())
        target = n * len(per_cell)
        return {
            "usable": usable,
            "target": target,
            "failed_attempts": failed,
            "redo_pending": target - usable,
            "cells_short": sum(1 for c in per_cell.values() if c < n),
            "cells_total": len(per_cell),
        }

    def _leg_id(self, idx):
        if 0 <= idx < len(self._legs):
            return self._legs[idx].get("id", f"leg_{idx}")
        return None

    def _recipe_curve(self, leg):
        for c in (leg.get("curves") or []):
            if str(c.get("kind", "")).lower() == "recipe":
                return c
        return None

    def _geometry_of(self, leg):
        """(path_family, R) for the leg's scored recipe, or None for glue /
        unscored / non-recipe / calibration legs (never swept)."""
        rec = self._recipe_curve(leg)
        if not rec or not bool(rec.get("scored", True)):
            return None
        recipe = rec.get("recipe") or {}
        if self._is_calib_recipe(recipe):
            return None
        fam = recipe.get("type")
        R = (recipe.get("params") or {}).get("R")
        if fam is None or R is None:
            return None
        return (str(fam), R)

    def _cell_key_for(self, fam, R, c, v):
        return manifest.cell_key({
            "controller": c, "v_const": v, "path_family": fam, "radius_m": R})

    def _cell_id_for(self, curve):
        """Treatment-qualified id -> unique bag dir + sidecar cell_id per rep
        (build_leg_dirname stamps only to the minute, so two reps in one minute
        would otherwise collide)."""
        base = curve.get("name", "curve")
        t = self._cur_treatment
        if t is None:
            return base

        def _g(x):
            return ("%g" % float(x)).replace(".", "p")

        return f"{base}_{t['controller']}_v{_g(t['v_const'])}_n{int(t['rep']):02d}"

    def _next_treatment_for(self, leg):
        """The next (controller, v_const, rep) to fill for this leg's geometry —
        the least-done, non-exhausted cell (round-robin "fill gaps"). None when
        the geometry is fully filled / retry-exhausted / not a scored recipe."""
        geom = self._geometry_of(leg)
        if geom is None:
            return None
        fam, R = geom
        if not self._geometry_in_matrix(fam, R):
            return None      # an old plan's geometry, not in the locked matrix
        best = None   # (attempts, done, order, c, v)
        order = 0
        for c in self._controllers:
            for v in self._speeds:
                order += 1
                key = self._cell_key_for(fam, R, c, v)
                if key is None:
                    continue
                if self._completed_counts.get(key, 0) >= self._pass_target():
                    continue
                att = self._attempts.get((fam, R, c, v), 0)
                if att >= self._max_retries:
                    continue   # retry-exhausted — skip so the cycle can finish
                # Deferred redo (2026-06-13): attempts is the PRIMARY sort key,
                # so a cell that just failed sorts to the BACK — every other
                # still-incomplete cell of this geometry runs before we retry
                # it. A bad bag is usually a transient (RTK blip, recorder
                # hiccup); a cooldown of other runs beats an immediate
                # back-to-back retry under the same conditions. attempts resets
                # to 0 on a pass (see _record_treatment_result), so a cell only
                # carries the penalty while it is actively failing; once every
                # fresh cell is full it is retried (and parked at max_retries).
                cand = (att, self._completed_counts.get(key, 0), order, c, v)
                if best is None or cand[:3] < best[:3]:
                    best = cand
        if best is None:
            return None
        _att, done, _order, c, v = best
        return {"controller": c, "v_const": float(v), "rep": int(done)}

    def _all_geometries_done(self):
        """True when the CURRENT leg list has nothing left to drive: no
        geometry with a remaining treatment and no pending calibration."""
        if any(self._is_calib_leg(lg) for lg in self._legs):
            return not self._calibration_pending()
        return all(self._next_treatment_for(lg) is None for lg in self._legs)

    # -- Steering calibration (2026-10-08) --------------------------------

    @staticmethod
    def _is_calib_recipe(recipe):
        return str((recipe or {}).get("type", "")).lower() == "calib_fig8"

    def _is_calib_leg(self, leg):
        rec = self._recipe_curve(leg)
        return bool(rec) and self._is_calib_recipe(rec.get("recipe"))

    def _calibration_mode_due(self):
        """'full' (no matrix lock yet) | 'sanity' (lock exists, no passing
        figure-8 within recheck_after_h) | None."""
        if calibration is None or not (self._matrix_doc or {}).get("_calib_gated"):
            return None       # ungated fixed radius list (legacy): no figure-8
        lock = (self._matrix_doc or {}).get("_matrix_lock")
        last = None
        if lock is not None:
            last = calibration.last_pass_utc(
                calibration.checks_path(self._bag_root), lock.get("epoch"))
        return calibration.mode_due(self._cal_cfg, lock, last)

    def _calibration_pending(self):
        return (not self._cal_done_session
                and self._calibration_mode_due() is not None)

    def _pass_target(self):
        """Per-cell target for treatment selection. plan.first_pass_reps
        (experiment.yaml, 2026-10-08): until EVERY cell of the batch has that
        many passing runs, stages fill only to it, so a short field day
        still ends with a balanced first pass across the whole matrix."""
        n = int(self._target_n or 0)
        fp = int(((self._matrix_doc or {}).get("plan") or {}).get(
            "first_pass_reps", 0) or 0)
        if fp <= 0 or fp >= n:
            return n
        for fam, R in self._scored_geometries():
            if not self._geometry_in_matrix(fam, R):
                continue
            for c in self._controllers:
                for v in self._speeds:
                    if self._attempts.get((fam, R, c, v), 0) >= self._max_retries:
                        continue      # retry-exhausted: must not freeze the pass
                    key = self._cell_key_for(fam, R, c, v)
                    if key is not None and self._completed_counts.get(key, 0) < fp:
                        return fp
        return n

    def _geometry_in_matrix(self, fam, R):
        """A gated matrix (radius_m: auto, or calibration.required) sweeps
        only its own (family, R) cells; ungated fixed matrices keep the
        legacy behaviour (any authored R)."""
        doc = self._matrix_doc or {}
        if not doc.get("_calib_gated"):
            return True
        m = doc.get("matrix") or {}
        by_fam = m.get("radius_m_by_family") or {}
        try:
            r = round(float(R), 6)
        except (TypeError, ValueError):
            return False
        fams = [str(f) for f in (m.get("path_family") or [])]
        return str(fam) in fams and r in {
            round(float(x), 6) for x in by_fam.get(str(fam), m.get("radius_m") or [])}

    def _epoch_filter(self):
        """(gated, epoch): with a gated matrix (radius_m: auto, or a fixed
        list with calibration.required) only legs recorded under the current
        calibration lock count (stock-driver legs and any other lock's legs
        never credit this matrix)."""
        doc = self._matrix_doc or {}
        return bool(doc.get("_calib_gated")), doc.get("_matrix_epoch")

    def _all_leg_lists(self):
        """Every stage's legs (global progress scope); [current] when the
        batch is a legacy single-stage venue."""
        if self._stages:
            return [st["legs"] for st in self._stages]
        return [self._legs]

    def _stage_name(self):
        if self._stages and 0 <= self._stage_idx < len(self._stages):
            return self._stages[self._stage_idx]["name"]
        return None

    def _select_stage_with_work(self):
        """Point _legs at the first stage that still has remaining
        treatments (manifest-driven — survives restarts with no extra
        state). Returns False when every stage is full/exhausted."""
        if not self._stages:
            return not self._all_geometries_done()
        for i, st in enumerate(self._stages):
            self._legs = st["legs"]
            if not self._all_geometries_done():
                if i != self._stage_idx:
                    self.get_logger().info(
                        f"stage select: '{st['name']}' ({i + 1}/"
                        f"{len(self._stages)}) has remaining treatments.")
                self._stage_idx = i
                return True
        self._stage_idx = len(self._stages) - 1
        self._legs = self._stages[self._stage_idx]["legs"]
        return False

    def _build_transit(self, old_legs, last_leg_idx, entry_glue):
        """Concatenate the transit polyline for an unattended stage advance
        (C, field design 2026-06-11): from the robot's park position (the end
        of old_legs[last_leg_idx]'s experiment) FOLLOW THE OLD STAGE'S OWN
        ALREADY-VALIDATED LOOP — each remaining leg's glue then its recipe
        path, as plain unscored waypoints — to the stage exit pose (the last
        leg's experiment end), then the planner's inter-stage entry glue to
        the next stage's first start pin. Headings match at every junction by
        construction, so the whole thing is ONE reposition goto.

        Returns a synthetic reposition-curve dict, or None (caller falls back
        to the plain path-join)."""
        try:
            wps = []
            n = len(old_legs)
            if last_leg_idx is None:
                return None         # never completed a leg here (resume case)
            corners = (self._venue or {}).get("corners_wgs84") or []
            if not corners:
                return None
            lat0, lon0 = corners[0]["lat"], corners[0]["lon"]
            for idx in range(last_leg_idx + 1, n):
                for curve in old_legs[idx].get("curves") or []:
                    kind = str(curve.get("kind", "")).lower()
                    if kind == "reposition":
                        wps += [{"lat": float(w["lat"]), "lon": float(w["lon"])}
                                for w in curve.get("waypoints_wgs84") or []]
                    elif kind == "recipe":
                        pts = venue_geom.recipe_points_en(
                            curve.get("recipe") or {}, curve.get("start_pose"),
                            lat0, lon0, spacing_m=0.25)
                        if not pts:
                            return None   # unverifiable geometry — fall back
                        for (e, nn, _s) in pts:
                            la, lo = venue_geom.en_to_latlon(e, nn, lat0, lon0)
                            wps.append({"lat": la, "lon": lo})
            wps += [{"lat": float(w["lat"]), "lon": float(w["lon"])}
                    for w in entry_glue.get("waypoints_wgs84") or []]
            if len(wps) < 2:
                return None
            out = {"name": "stage_transit", "kind": "reposition",
                   "waypoints_wgs84": wps,
                   "v_const": float(entry_glue.get("v_const", 0.2)),
                   "pos_tol_m": float(entry_glue.get("pos_tol_m", 0.15))}
            if entry_glue.get("end_heading_deg") is not None:
                out["end_heading_deg"] = float(entry_glue["end_heading_deg"])
            return out
        except (KeyError, TypeError, ValueError) as exc:
            self.get_logger().warn(f"transit build failed ({exc!r}) — "
                                   "falling back to plain path-join")
            return None

    def _advance_stage(self):
        """After the current stage's geometries fill: move to the next stage
        with work, containment-gate it, and continue the batch unattended.
        Searches forward first, then wraps to the start (the first-pass
        sweep, plan.first_pass_reps, revisits every stage). Returns
        'advanced' | 'paused' | 'none'."""
        if not self._stages:
            return "none"
        old = self._stages[self._stage_idx]
        n_st = len(self._stages)
        order = (list(range(self._stage_idx + 1, n_st))
                 + list(range(0, self._stage_idx + 1)))
        for i in order:
            st = self._stages[i]
            self._legs = st["legs"]
            if self._all_geometries_done():
                continue
            ok, report = venue_geom.check_legs_containment(
                self._legs, self._venue, self._footprint_r, self._track_margin)
            if not ok:
                self.get_logger().error(
                    f"stage '{st['name']}' CONTAINMENT FAILED:\n" + report)
                self._pause(f"next stage '{st['name']}' violates containment "
                            "— re-plan, Send, then Start")
                return "paused"
            # Planned transit (C): only valid when advancing from exactly the
            # stage the glue was planned FROM — a skipped stage (resume
            # credit) leaves the robot somewhere the glue does not start.
            self._transit_curve = None
            eg = st.get("entry_glue")
            wg = st.get("wrap_glue")
            if not (eg and eg.get("from_stage") == old["name"]) and (
                    wg and wg.get("from_stage") == old["name"]):
                eg = wg      # first-pass wrap: last stage -> first matrix stage
            if eg and eg.get("from_stage") == old["name"]:
                self._transit_curve = self._build_transit(
                    old["legs"], self._last_completed_leg_idx, eg)
                if self._transit_curve is not None:
                    self.get_logger().info(
                        "stage transit planned: walk the old stage loop to "
                        "its exit + inter-stage glue "
                        f"({len(self._transit_curve['waypoints_wgs84'])} wps).")
            self._stage_idx = i
            self._leg_idx = 0
            self._curve_idx = 0
            self.get_logger().info(
                f"stage advance -> '{st['name']}' "
                f"({i + 1}/{len(self._stages)}).")
            self._operator_alert(
                "stage_advance", "info", "STAGE ADVANCE",
                f"'{st['name']}' ({i + 1}/{len(self._stages)}) "
                f"— sweep {self._runs_done()}/{self._runs_target()}.")
            self._publish_status(message=f"stage advance: {st['name']}")
            return "advanced"
        # Nothing ahead; restore the current stage's legs.
        self._legs = self._stages[self._stage_idx]["legs"]
        return "none"

    def _scored_geometries(self):
        seen, out = set(), []
        for legs in self._all_leg_lists():
            for lg in legs:
                geom = self._geometry_of(lg)
                if geom is not None and geom not in seen:
                    seen.add(geom)
                    out.append(geom)
        return out

    def _runs_target(self):
        per = len(self._controllers) * len(self._speeds) * max(self._target_n, 0)
        return len(self._scored_geometries()) * per

    def _runs_done(self):
        total = 0
        for fam, R in self._scored_geometries():
            for c in self._controllers:
                for v in self._speeds:
                    key = self._cell_key_for(fam, R, c, v)
                    if key is not None:
                        total += min(self._completed_counts.get(key, 0),
                                     self._target_n)
        return total

    def _record_treatment_result(self, passed):
        """Update per-cell counts/attempts + the consecutive-failure breaker
        after a scored run that REACHED the end (passed = classification.pass)."""
        t = self._cur_treatment
        if t is None:
            return
        geom = self._geometry_of(self._legs[self._leg_idx])
        if geom is None:
            return
        fam, R = geom
        k_cell = (fam, R, t["controller"], t["v_const"])
        key = self._cell_key_for(fam, R, t["controller"], t["v_const"])
        if passed:
            if key is not None:
                self._completed_counts[key] = self._completed_counts.get(key, 0) + 1
            self._attempts[k_cell] = 0
            self._consec_fail = 0
            self.get_logger().info(
                f"PASS {fam} R{R} {t['controller']} v{t['v_const']} rep{t['rep']} "
                f"({self._completed_counts.get(key, 0)}/{self._target_n}).")
        else:
            self._attempts[k_cell] = self._attempts.get(k_cell, 0) + 1
            self._consec_fail += 1
            self.get_logger().warn(
                f"FAIL-classification {fam} R{R} {t['controller']} v{t['v_const']} "
                f"attempt {self._attempts[k_cell]}/{self._max_retries}, "
                f"consec {self._consec_fail}/{self._breaker_k}.")
            if self._consec_fail >= self._breaker_k:
                self._pause(
                    f"circuit breaker: {self._consec_fail} consecutive scored-run "
                    "classification failures — check RTK/venue, then Start to resume")
        self._progress = self._global_progress()

    def _refuse(self, reason):
        """Start refused BEFORE anything moved: PAUSED with the reason on
        /run/status + an operator card. (_pause is a no-op from IDLE, which
        made these refusals silent.)"""
        self._pause_reason = reason
        self.get_logger().warn(f"START REFUSED: {reason}")
        self._operator_alert("start_refused", "warn", "START REFUSED", reason)
        self._safe_state()
        self._enter(Phase.PAUSED)
        self._publish_status(message=f"refused: {reason}")

    def _start_or_resume(self):
        """Shared Start/resume entry: reload venue + matrix + manifest counts,
        reset session retry state, containment-gate, then PREFLIGHT (or DONE)."""
        self.phase = Phase.IDLE   # so _pause/_enter below are not no-ops on resume
        # A new Start is a new session: the 4 h window in mode_due (not this
        # flag) is what prevents re-running a recent figure-8.
        self._cal_done_session = False
        self._reload_active()
        if not self._legs:
            self._publish_status(message="no legs loaded — Send a venue first")
            return
        if not self._load_matrix():
            self._refuse("experiment.yaml unreadable — cannot choose treatments")
            return
        self._rebuild_counts()
        self._attempts = {}
        self._consec_fail = 0
        self._pause_reason = None
        self._rtk_lost_since = None
        self._done_notified = False
        self._leg_idx = 0
        self._curve_idx = 0
        # Fresh Start: the robot may be parked anywhere (operator moved it,
        # resume after pause) — a transit planned for a previous advance no
        # longer starts where the robot stands. Path-join handles it instead.
        self._transit_curve = None
        self._last_completed_leg_idx = None
        # Same reason: a glue snapshot from before the pause/abort did not
        # bring the robot to wherever the next recipe starts.
        self._arrival_glue = None
        # Steering calibration bookkeeping (2026-10-08).
        self._replan_thread = None
        self._replan_gen += 1
        self._replan_out = None
        self._replan_needed = False
        has_calib = any(self._is_calib_leg(lg) for st in (self._stages or [])
                        for lg in st["legs"]) or any(
                            self._is_calib_leg(lg) for lg in (self._legs or []))
        gated, _epoch = self._epoch_filter()
        radii = ((self._matrix_doc or {}).get("matrix") or {}).get("radius_m") or []
        if gated and not radii and not has_calib:
            self._refuse("no steering calibration yet and this batch has no "
                         "figure-8 — press Auto-plan, Send, Start (the plan now "
                         "starts with the calibration)")
            return
        if gated and radii and not has_calib and self._calibration_mode_due():
            self._operator_alert(
                "calibration", "warn", "SANITY CHECK SKIPPED",
                "a steering sanity figure-8 is due but this batch has none — "
                "Auto-plan, Send, Start to include it (running without it).")
        matrix_stages = [st for st in (self._stages or [])
                         if not any(self._is_calib_leg(lg) for lg in st["legs"])]
        if (gated and radii and has_calib and not matrix_stages
                and not self._calibration_pending()):
            # The lock exists but the batch is still calibration-only (the
            # post-lock re-plan was interrupted): re-plan after preflight.
            cal_leg = next((lg for st in self._stages for lg in st["legs"]
                            if self._is_calib_leg(lg)), None)
            if cal_leg is None:
                self._refuse("calibration leg outside plan_stages — Auto-plan, "
                             "Send, Start")
                return
            self._cal_pin = dict(self._recipe_curve(cal_leg).get("start_pose") or {})
            self._replan_needed = True
            self.get_logger().info("calibration-only batch with a lock: "
                                   "will re-plan the matrix after preflight.")
            self._enter(Phase.PREFLIGHT)
            return
        if not self._select_stage_with_work():
            self._enter(Phase.DONE)
            return
        ok, report = venue_geom.check_legs_containment(
            self._legs, self._venue, self._footprint_r, self._track_margin)
        if not ok:
            self.get_logger().error(
                "VENUE CONTAINMENT FAILED — refusing to run (no motion):\n" + report)
            self._refuse("a planned curve leaves the venue (see log) — re-plan, "
                         "Send, Start")
            return
        self.get_logger().info("venue containment OK — " + report)
        stage = f" stage '{self._stage_name()}'" if self._stages else ""
        self.get_logger().info(
            f"sweep{stage}: {self._runs_done()}/{self._runs_target()} scored "
            f"runs done across {len(self._scored_geometries())} geometries; "
            f"controllers={self._controllers} speeds={self._speeds} N={self._target_n}.")
        self._enter(Phase.PREFLIGHT)

    def _begin_run(self):
        self._start_or_resume()

    def _resume(self):
        if self.phase != Phase.PAUSED:
            return
        self._start_or_resume()

    def _begin_current_curve(self):
        if self._leg_idx >= len(self._legs):
            self._enter(Phase.DONE)
            return
        curves = self._legs[self._leg_idx].get("curves") or []
        if self._curve_idx >= len(curves):
            self._complete_leg()
            return
        self._cur_curve = curves[self._curve_idx]
        kind = str(self._cur_curve.get("kind", "")).lower()
        # One-shot stage transit (C): the first reposition after an advance
        # drives the planned old-loop + inter-stage glue polyline instead of
        # the leg's own glue (the transit ENDS at that glue's endpoint — the
        # first start pin). An abort pauses; on resume the transit is gone
        # and the plain path-join takes over.
        if kind == "reposition" and self._transit_curve is not None:
            self._cur_curve = self._transit_curve
            self._transit_curve = None
            self.get_logger().info(
                "stage transit: driving the old stage's loop to its exit + "
                "inter-stage glue "
                f"({len(self._cur_curve['waypoints_wgs84'])} wps, unscored).")
        if kind == "reposition":
            self._cur_treatment = None   # glue move — no scored treatment
            self._cur_scored = False
            self._cur_calib = False
            self._enter(Phase.REPOSITION_START)
        elif kind == "recipe" and self._is_calib_recipe(self._cur_curve.get("recipe")):
            # Steering calibration figure-8 (open loop, calib_node). Driven only
            # while one is due this session; otherwise skipped like a full
            # geometry (the robot just continues to the next glue).
            self._cur_treatment = None
            self._cur_scored = False
            mode = self._calibration_mode_due()
            if self._cal_done_session or mode is None:
                self.get_logger().info("calibration not due — skipping the figure-8.")
                self._advance_curve()
                return
            self._cur_calib = True
            self._cal_mode = mode
            self.get_logger().info(
                f"leg '{self._leg_id(self._leg_idx)}': steering calibration "
                f"figure-8 ({mode}).")
            self._operator_alert(
                "calibration", "info", f"CALIBRATION ({mode.upper()})",
                "measuring full-lock turning (open-loop figure-8"
                + (" at 0.5 and 1.0 m/s" if mode == "full" else " at 1.0 m/s")
                + "). Keep clear; the robot turns tight circles.")
            self._enter(Phase.KILL_REPOSITION)
        elif kind == "recipe":
            # The SYSTEM picks the treatment (controller x v_const x rep) for this
            # geometry from the matrix + manifest. None => geometry already full /
            # retry-exhausted / unscored => drive it UNSCORED as glue (no bag /
            # sidecar / FIXED gate) so the next reposition still lines up.
            self._cur_calib = False
            self._cur_treatment = self._next_treatment_for(self._legs[self._leg_idx])
            self._cur_scored = (self._cur_treatment is not None
                                and bool(self._cur_curve.get("scored", True)))
            if self._cur_treatment is not None:
                t = self._cur_treatment
                self.get_logger().info(
                    f"leg '{self._leg_id(self._leg_idx)}' recipe "
                    f"'{self._cur_curve.get('name')}': treatment "
                    f"{t['controller']} v{t['v_const']} rep{t['rep']}.")
            else:
                self.get_logger().info(
                    f"leg '{self._leg_id(self._leg_idx)}' recipe "
                    f"'{self._cur_curve.get('name')}': geometry full — "
                    "unscored traversal.")
            self._enter(Phase.KILL_REPOSITION)
        else:
            self._pause(f"unknown curve kind '{kind}'")

    def _advance_curve(self):
        # Dwell before every curve->curve transition (the very first curve
        # after Start has no preceding curve and starts immediately).
        self._dwell_until = time.monotonic() + self._dwell_s
        self._curve_idx += 1
        curves = self._legs[self._leg_idx].get("curves") or []
        if self._curve_idx >= len(curves):
            self._complete_leg()
        else:
            self._begin_current_curve()

    def _complete_leg(self):
        lid = self._leg_id(self._leg_idx)
        self.get_logger().info(
            f"leg '{lid}' done — sweep {self._runs_done()}/{self._runs_target()}.")
        self._publish_status(message=f"leg '{lid}' done")
        # The robot is physically parked at THIS leg's experiment end — the
        # anchor for a stage-advance transit (C, 2026-06-11).
        self._last_completed_leg_idx = self._leg_idx
        # Cycle the authored legs ("fill gaps"); when no geometry in THIS
        # stage has a remaining treatment, auto-advance to the next planned
        # stage (the unattended multi-stage batch) or finish.
        self._leg_idx = (self._leg_idx + 1) % len(self._legs)
        self._curve_idx = 0
        if self._all_geometries_done():
            res = self._advance_stage()
            if res == "paused":
                return
            if res == "none":
                self._enter(Phase.DONE)
                return
        # M2 battery gate between legs: never start a new leg below halt.
        if self._battery_v is not None and self._battery_v < self._batt_halt:
            self._pause(
                f"battery {self._battery_v:.2f}V < halt {self._batt_halt}V — "
                "swap + press Start to resume")
            return
        self._begin_current_curve()

    # ==================================================================
    # State machine
    # ==================================================================

    def _enter(self, phase):
        if phase != self.phase:
            self.get_logger().info(f"phase {self.phase.value} -> {phase.value}")
            if phase == Phase.DONE:
                _notify_discord(
                    f"BATCH DONE: {self._runs_done()}/{self._runs_target()} "
                    "scored runs complete.", title="H-inf run_executor")
            if phase == Phase.ODOM_RESET:
                self._odom_reset_sent = False
                self._odom_reset_command_t = None
                self._odom_reset_first_sent_t = None
                self._achieved_anchor = None
                self._arrival = None
            if phase == Phase.FOLLOWER_START:
                # Neutralize the latched recipe BEFORE the follower spawns: a
                # fresh follower replays the last latched message, and the
                # previous leg's curve would start it driving at default params
                # while we are still in SET_PARAMS. The real recipe follows in
                # PUSH_RECIPE on this same latched publisher.
                self.pub_recipe.publish(String(data=NEUTRAL_RECIPE))
        self.phase = phase
        self._phase_entered = time.monotonic()
        self._publish_status()

    def _in_phase_s(self):
        return time.monotonic() - self._phase_entered

    def _tick(self):
        if time.monotonic() - self._last_heartbeat >= self._heartbeat_s:
            self._last_heartbeat = time.monotonic()
            self._publish_status(message="heartbeat")

        if self.phase in (Phase.IDLE, Phase.DONE, Phase.ABORTED, Phase.PAUSED):
            return

        # A fresh E-stop during any active maneuver pauses + safe-states (the
        # WebUI auto-clears estop at Start; PREFLIGHT waits for it to clear).
        if self._estop and self.phase != Phase.PREFLIGHT:
            self._pause("E-stop during run/maneuver")
            return

        # Inter-curve dwell: hold (robot motionless, no mover spawned yet) at
        # the entry phase of each new curve until the dwell window passes.
        if (self.phase in (Phase.REPOSITION_START, Phase.KILL_REPOSITION)
                and time.monotonic() < self._dwell_until):
            self._publish_status(message="dwell before next curve")
            return

        handler = getattr(self, f"_tick_{self.phase.value}", None)
        if handler is not None:
            handler()
        else:
            self.get_logger().error(f"no handler for phase {self.phase.value}")
            self._pause(f"no handler for phase {self.phase.value}")

    # -- Per-phase handlers --------------------------------------------

    def _ensure_support_procs(self):
        """Start the support stack ourselves and report what is not up yet.

        The orchestrator ignores a start for an already-running PROC, so the
        retry is idempotent; we rate-limit it only to keep the log readable.
        Returns the list of PROCs still not alive.
        """
        pending = []
        now = time.monotonic()
        for name in SUPPORT_PROCS:
            if self._is_alive(name):
                continue
            # Before the first /orchestrator/status arrives every PROC looks
            # dead; that is fine, the start is idempotent and status lands in
            # well under the preflight timeout.
            last = self._support_start_t.get(name, 0.0)
            if now - last >= SUPPORT_RETRY_S:
                self._support_start_t[name] = now
                self._orch_start(name)
                self.get_logger().info(f"preflight: starting support PROC '{name}'")
            pending.append(name)
        return pending

    def _tick_preflight(self):
        missing = []
        pending = self._ensure_support_procs()
        if pending:
            missing.append("support PROCs not up: " + ", ".join(pending))
        if self._estop:
            missing.append("estop engaged")
        if self._last_odom_t is None or (time.monotonic() - self._last_odom_t) > 1.0:
            missing.append("no fresh /wheel/odom")
        missing += driver_config_problems(self._driver_cfg, self._driver_expect)
        if (self._orch_status and PROC_CALIB not in self._orch_status
                and any(self._is_calib_leg(lg) for lg in self._legs)
                and self._calibration_pending()):
            missing.append("the orchestrator has no 'calib' PROC (started before "
                           "the 2026-10-08 deploy) — restart limo-battle")
        if self._rtk_quality not in RTK_OK:
            missing.append(f"RTK quality {self._rtk_quality} not in {RTK_OK}")
        if self._battery_v is not None and self._battery_v < self._batt_halt:
            missing.append(f"battery {self._battery_v:.2f}V < {self._batt_halt}V")
        if not missing:
            self.get_logger().info("preflight pass")
            if self._replan_needed:
                self._replan_needed = False
                self._enter(Phase.REPLAN)
                return
            self._begin_current_curve()
            return
        if self._in_phase_s() > self._preflight_timeout:
            self._pause("preflight timeout: " + "; ".join(missing))
            return
        self._publish_status(message="preflight waiting: " + "; ".join(missing))

    def _tick_reposition_start(self):
        res = self._start_exclusive_mover(PROC_REPOSITION)
        if res == "started":
            self._repo_state = None
            self._repo_status = {}
            self._goto_sent = False
            self._enter(Phase.REPOSITION_GOTO)
        elif self._in_phase_s() > self._reposition_timeout:
            self._pause("reposition start timeout")

    def _tick_reposition_goto(self):
        if not self._is_alive(PROC_REPOSITION):
            if self._in_phase_s() > self._settle_s:
                self._pause("reposition node not alive")
            return
        if self._in_phase_s() < self._settle_s and self._repo_state is None:
            return
        # Send the goto, then RE-SEND at ~1 Hz until /reposition/status echoes
        # our seq (the ack). A single volatile publish races the fresh
        # reposition's subscription DDS-matching after a respawn and is
        # silently dropped (hw-observed 2026-06-10: leg-2 goto never arrived).
        # Re-sending the SAME seq is idempotent: a duplicate inside the ~50 ms
        # ack window just re-commits the identical mission. The seq echo also
        # guarantees a stale 'arrived' (still streamed from the PREVIOUS goto
        # while reposition stays alive across consecutive glue curves) can
        # never be honored for THIS goto.
        if not self._goto_sent or self._repo_status.get("seq") != self._goto_seq:
            if (self.pub_goto.get_subscription_count() >= 1
                    and time.monotonic() - self._goto_last_send_t >= 1.0):
                curve = self._cur_curve
                try:
                    wps = curve.get("waypoints_wgs84") or []
                    payload = {
                        "waypoints": [{"lat": float(w["lat"]),
                                       "lon": float(w["lon"])} for w in wps],
                        # reposition_speed_mps is a CAP: an authored per-curve
                        # v_const may go slower, never faster.
                        "v_const": min(float(curve.get("v_const",
                                                       self._repo_speed)),
                                       self._repo_speed),
                        "pos_tol_m": float(curve.get("pos_tol_m", 0.15)),
                    }
                    if curve.get("end_heading_deg") is not None:
                        payload["end_heading_deg"] = float(curve["end_heading_deg"])
                except (KeyError, TypeError, ValueError) as exc:
                    # A malformed curve must pause, not kill this node from
                    # inside the timer callback (venue_loader validates, but
                    # active.json can be hand-edited).
                    self._pause(f"malformed reposition curve "
                                f"'{curve.get('name')}': {exc!r}")
                    return
                if not self._goto_sent:
                    self._goto_seq += 1   # one id per curve, kept across re-sends
                payload["seq"] = self._goto_seq
                self.pub_goto.publish(String(data=json.dumps(payload)))
                self._goto_payload = payload   # as sent (sidecar 'arrival')
                self._goto_sent = True
                self._goto_last_send_t = time.monotonic()
            if self._in_phase_s() > self._reposition_timeout:
                self._goto_sent = False
                self._pause("reposition goto timeout (goto never acked)")
            return
        if self._repo_state == "arrived":
            self._goto_sent = False
            err_deg = self._repo_status.get("err_deg")
            if (self._cur_curve.get("end_heading_deg") is not None
                    and isinstance(err_deg, (int, float))
                    and abs(float(err_deg)) > self._arrival_head_tol):
                # The recipe runs in the odom frame zeroed at the ACHIEVED
                # heading: a large arrival heading error rotates the whole
                # checked path in the world. Don't run it.
                self._pause(
                    f"arrived with heading error {float(err_deg):.1f} deg > "
                    f"{self._arrival_head_tol:.0f} deg — recipe would run rotated "
                    "outside the checked corridor; re-author the glue tail, "
                    "then Start")
                return
            if err_deg is None and self._cur_curve.get("end_heading_deg") is not None:
                self.get_logger().warn(
                    "arrived with UNKNOWN heading error (fused heading stale?) — "
                    "proceeding; recipe heading unverified.")
            # _advance_curve replaces _cur_curve with the recipe: snapshot the
            # glue we arrived on NOW, for the next odom reset to record.
            self._snapshot_glue_arrival()
            self._note_repo_outcome(True)
            self._advance_curve()
        elif self._repo_state == "aborted":
            self._goto_sent = False
            self._note_repo_outcome(False)
            self._pause("reposition aborted: " + self._repo_reason())
        elif self._in_phase_s() > self._reposition_timeout:
            self._goto_sent = False
            self._pause("reposition goto timeout")

    def _note_repo_outcome(self, arrived):
        """Field rule 2026-10-08 (DOC/agent_field_runbook.md §7): when
        repositions keep aborting at the faster glue speed, page the operator
        (and their agent) with the exact command to drop back to 0.3 m/s."""
        self._repo_outcomes.append(bool(arrived))
        n_abort = sum(1 for a in self._repo_outcomes if not a)
        if (n_abort >= self._repo_alert_n and self._repo_speed > 0.3 + 1e-6
                and not self._repo_alerted):
            self._repo_alerted = True
            msg = (f"{n_abort} of the last {len(self._repo_outcomes)} repositions "
                   f"aborted at {self._repo_speed:.2f} m/s. Drop the glue speed to "
                   "0.3 m/s: ros2 param set /run_executor_node "
                   "reposition_speed_mps 0.3   (then press Start to resume; "
                   "if 0.3 still fails, 0.2 is the old proven value — ask before "
                   "going lower or back up).")
            self.get_logger().warn("REPOSITION ABORTS: " + msg)
            self._operator_alert("repo_speed", "warn",
                                 "REPOSITIONS KEEP ABORTING", msg)

    def _tick_kill_reposition(self):
        if self._is_alive(PROC_REPOSITION):
            self._orch_kill(PROC_REPOSITION)
            return
        if PROC_REPOSITION not in self._orch_status:
            if self._in_phase_s() > self._settle_s:
                self._enter(Phase.ODOM_RESET)
            return
        self._enter(Phase.ODOM_RESET)

    def _tick_odom_reset(self):
        if not self._is_alive(PROC_ODOM_ZERO):
            self._orch_start(PROC_ODOM_ZERO)
            if self._in_phase_s() > self._reposition_timeout:
                self._pause("odom_zero failed to start")
            return
        # Orchestrator "alive" means the PROCESS spawned; a cold odom_zero's
        # /odom_zero/reset subscription appears 1-2 s later, and a volatile
        # publish with no subscriber is silently dropped. So: wait for the
        # subscription to exist, then RE-SEND every tick until the latched
        # status confirms (the robot is stationary here — re-latching the same
        # standstill pose is harmless; the confirm gate keys off the FIRST send
        # time, so any latch at/after it counts).
        if self.pub_odom_reset.get_subscription_count() < 1:
            if self._in_phase_s() > self._reposition_timeout:
                self._pause("odom_zero alive but /odom_zero/reset never subscribed")
            return
        if not self._odom_reset_sent:
            self._odom_reset_command_t = self._ros_now_s()
            self._odom_reset_first_sent_t = time.monotonic()
            self._odom_reset_sent = True
        self.pub_odom_reset.publish(Bool(data=True))
        s = self._odom_zero_status or {}
        if (s.get("has_reset") is True and s.get("stamp") is not None
                and float(s["stamp"]) >= float(self._odom_reset_command_t)):
            origin = s.get("origin") or {}
            self.get_logger().info(
                f"odom_zero latch confirmed at (x={origin.get('x')}, "
                f"y={origin.get('y')}, yaw={origin.get('yaw')}).")
            self._capture_achieved_anchor()
            self._capture_arrival()
            self._enter(Phase.BAG_START)
            return
        if time.monotonic() - self._odom_reset_first_sent_t > self._odom_settle_s:
            self._pause(
                f"odom_zero reset not confirmed within {self._odom_settle_s:.1f}s "
                "of first send")

    def _capture_achieved_anchor(self):
        """Snapshot the MEASURED world pose at the odom-zero instant.

        The ref path is generated in the just-zeroed odom frame, so its world
        placement is exactly the robot's true pose right now. The pin pose
        (path_frame_anchor) is only the COMMANDED target — reposition arrives
        up to pos_tol/heading_tol away from it — so run_eval must score
        RTK-truth metrics against this snapshot, not the pin.

        Never blocks the run: a stale/conflicted heading or missing fix is
        recorded as valid=false and paged to the operator (the leg stays
        analyzable via the post-hoc odom->RTK track fit).
        """
        now = time.monotonic()
        problems = []

        hs = self._heading_status or {}
        heading_fresh = (self._heading_status_t is not None
                         and now - self._heading_status_t <= 2.0)
        mode = hs.get("mode")
        if not heading_fresh:
            problems.append("fused heading stale/absent")
        elif mode not in ("GNSS_AIDED", "GYRO_MAG"):
            problems.append(f"no absolute heading reference (mode={mode})")
        if hs.get("src_conflict"):
            problems.append("heading sources in conflict")

        fix_fresh = (self._last_fix_t is not None
                     and now - self._last_fix_t <= 3.0)
        if not fix_fresh:
            problems.append("RTK fix stale/absent")

        err_deg = (self._repo_status or {}).get("err_deg")
        self._achieved_anchor = {
            "valid": not problems,
            "problems": problems,
            "lat": self._last_fix[0] if fix_fresh else None,
            "lon": self._last_fix[1] if fix_fresh else None,
            "fix_age_s": (round(now - self._last_fix_t, 2)
                          if self._last_fix_t is not None else None),
            "heading_deg": hs.get("fused_deg") if heading_fresh else None,
            "heading_convention": "compass_deg_east_of_north",
            "heading_std_deg": hs.get("heading_std_deg") if heading_fresh else None,
            "heading_mode": mode,
            "heading_sources": hs.get("active_sources") if heading_fresh else None,
            "reposition_err_deg": (float(err_deg)
                                   if isinstance(err_deg, (int, float)) else None),
            "stamp_utc": datetime.now(timezone.utc).isoformat(),
        }
        if problems:
            msg = ("achieved anchor INVALID at odom reset: "
                   + "; ".join(problems)
                   + " — RTK-truth scoring for this leg falls back to the "
                     "post-hoc odom->RTK track fit.")
            self.get_logger().error(msg)
            self._operator_alert("anchor_warning", "warn",
                                 f"ANCHOR WARNING ({self._leg_cell_id})", msg)
        else:
            self.get_logger().info(
                f"achieved anchor: lat={self._achieved_anchor['lat']:.7f} "
                f"lon={self._achieved_anchor['lon']:.7f} "
                f"hdg={self._achieved_anchor['heading_deg']:.1f} deg "
                f"(std {self._achieved_anchor['heading_std_deg']} deg, "
                f"mode {mode}, repo err {err_deg} deg).")

    def _snapshot_glue_arrival(self):
        """Freeze the glue reposition just reported 'arrived' on: the goto
        payload exactly as SENT (capped v_const, pos_tol, end heading, seq),
        the curve name (a leg glue, or 'stage_transit'), and the status the
        arrival gate accepted. Consumed by _capture_arrival at the next
        recipe's odom reset. Never raises — bookkeeping must not stop a batch.
        """
        try:
            p = self._goto_payload or {}
            problems = []
            if p.get("seq") != self._goto_seq:
                problems.append(f"last goto sent has seq {p.get('seq')} != "
                                f"accepted seq {self._goto_seq}")
            glue = {
                "curve_name": (self._cur_curve or {}).get("name"),
                "seq": p.get("seq"),
                "waypoints_wgs84": list(p.get("waypoints") or []),
                "end_heading_deg": p.get("end_heading_deg"),
                "pos_tol_m": p.get("pos_tol_m"),
                "v_const": p.get("v_const"),
            }
            self._arrival_glue = {
                "glue": glue,
                "status": dict(self._repo_status or {}),
                "stamp_utc": datetime.now(timezone.utc).isoformat(),
                "problems": problems,
            }
        except Exception as exc:  # noqa: BLE001
            # Keep a marker (not None): None would read as "no glue arrived".
            self._arrival_glue = {"glue": None, "status": None, "stamp_utc": None,
                                  "problems": [f"glue snapshot failed: {exc!r}"]}
            self.get_logger().warn(f"glue arrival snapshot failed: {exc!r}")

    def _capture_arrival(self):
        """Build the sidecar 'arrival' record: HOW the robot reached this
        recipe's start pin, so arrival heading error can be attributed to the
        glue geometry (turn radius, settled straight tail) after the fact.

          glue               the reposition curve as sent (_snapshot_glue_arrival)
          glue_metrics       glue_arrival_metrics() of it
          status_at_arrival  the /reposition/status the 'arrived' gate accepted
          final_status       the LAST /reposition/status before the kill — the
                             robot sat through the inter-curve dwell, so its
                             err_deg is the latest pre-zero heading error — and
                             final_status_age_s, its age at the odom reset
          problems           why any part is missing (never silently absent)

        Consumes the glue snapshot, so a recipe NOT directly preceded by a glue
        records captured=false instead of inheriting an earlier leg's glue.
        Never raises, and the record is checked to serialize as STRICT JSON
        here: a NaN/Infinity reaching write_sidecar would fail the whole leg.
        """
        snap, self._arrival_glue = self._arrival_glue, None
        try:
            problems = []
            rec = {"captured": False, "glue": None, "glue_metrics": None,
                   "status_at_arrival": None, "arrived_utc": None,
                   "final_status": None, "final_status_age_s": None,
                   "problems": problems}
            if snap is None:
                problems.append("no reposition glue arrived immediately before "
                                "this recipe")
            else:
                problems += list(snap.get("problems") or [])
                glue = snap.get("glue")
                rec["captured"] = glue is not None
                rec["glue"] = glue
                rec["status_at_arrival"] = snap.get("status")
                rec["arrived_utc"] = snap.get("stamp_utc")
                if glue is not None:
                    try:
                        rec["glue_metrics"] = glue_arrival_metrics(
                            glue.get("waypoints_wgs84"),
                            glue.get("end_heading_deg"))
                    except Exception as exc:  # noqa: BLE001
                        problems.append(f"glue metrics failed: {exc!r}")
                st = self._repo_status if isinstance(self._repo_status, dict) else {}
                if st:
                    rec["final_status"] = dict(st)
                    if self._repo_status_t is not None:
                        rec["final_status_age_s"] = round(
                            time.monotonic() - self._repo_status_t, 2)
                    if glue is not None and st.get("seq") != glue.get("seq"):
                        problems.append(f"final status seq {st.get('seq')} != "
                                        f"glue seq {glue.get('seq')}")
                else:
                    problems.append("no /reposition/status held at odom reset")
            json.dumps(rec, allow_nan=False)
        except Exception as exc:  # noqa: BLE001
            self._arrival = {"captured": False,
                             "problems": [f"arrival capture failed: {exc!r}"]}
            self.get_logger().warn(f"arrival capture failed: {exc!r}")
            return
        self._arrival = rec
        m = rec["glue_metrics"] or {}
        fin = rec["final_status"] or {}
        self.get_logger().info(
            f"arrival: glue '{(rec['glue'] or {}).get('curve_name')}' "
            f"R_min={m.get('min_turn_radius_m')} m "
            f"tail={m.get('tail_straight_m')} m, "
            f"final repo err {fin.get('err_deg')} deg"
            + (f"; problems: {'; '.join(problems)}" if problems else "") + ".")

    def _tick_bag_start(self):
        curve = self._cur_curve
        scored = self._cur_scored or self._cur_calib
        # RTK FIXED(4) gate for scored runs: the bag's RTK is the post-hoc ground
        # truth, so do not start recording until FIXED. The calibration needs it
        # too (its containment guard and the RTK radius cross-check).
        if scored and self._rtk_quality != RTK_FIXED:
            if self._in_phase_s() < self._rtk_fix_wait:
                self._publish_status(
                    message=f"waiting RTK FIXED(4) for scored run (q={self._rtk_quality})")
                return
            self._pause(
                f"RTK not FIXED(4) for scored run (q={self._rtk_quality})")
            return
        self._reset_run_scratch()
        self._leg_cell_id = (
            f"calibration_{self._cal_mode}_{datetime.now().strftime('%H%M%S')}"
            if self._cur_calib else self._cell_id_for(curve))
        if scored:
            if Data_Logger is None:
                self._pause("recorder unavailable (Data_Logger import failed)")
                return
            try:
                dirname = Data_Logger.build_leg_dirname(
                    self._run_id, self._leg_cell_id,
                    self._leg_id(self._leg_idx))
                root = (calibration.lock_dir(self._bag_root)
                        if self._cur_calib and calibration is not None
                        else self._bag_root)
                self._leg_bag_path = os.path.join(root, dirname)
                topics = list(Data_Logger.TOPICS)
                if self._cur_calib:
                    topics += [t for t in CALIB_EXTRA_TOPICS if t not in topics]
                self._recorder = Data_Logger.BagRecorder(
                    self._leg_bag_path, topics=topics)
                self._recorder.start()
                self._leg_start_utc = datetime.now(timezone.utc).isoformat()
                self._leg_driver_cfg_start = self._driver_cfg
            except Exception as exc:
                self._recorder = None
                self._pause(f"bag start failed: {exc}")
                return
        self._enter(Phase.CALIB_START if self._cur_calib else Phase.FOLLOWER_START)

    def _tick_calib_start(self):
        res = self._start_exclusive_mover(PROC_CALIB)
        if res == "started":
            self._cal_seq += 1
            self._cal_status = {}        # never judge a run by an older status
            self._cal_req = None
            self._cal_req_sent = False
            self._cal_req_last_t = 0.0
            self._cal_result = None
            self._run_end_reason = None
            self._enter(Phase.CALIB_RUN)
        elif self._in_phase_s() > self._reposition_timeout:
            self._pause("calibration node start timeout")

    def _calib_request(self):
        """The /calib/request payload for the current figure-8 (calib_node
        contract: scratchpad spec / calib_node.py docstring)."""
        cfg = self._cal_cfg
        recipe = (self._cur_curve or {}).get("recipe") or {}
        params = recipe.get("params") or {}
        speeds = (cfg["full_speeds"] if self._cal_mode == "full"
                  else cfg["sanity_speeds"])
        r_plan = float(params.get("R_plan_m", cfg["R_plan_m"]))
        sp = (self._cur_curve or {}).get("start_pose") or {}
        v = self._venue or {}
        return {
            "seq": self._cal_seq,
            "mode": self._cal_mode,
            "speeds": [float(x) for x in speeds],
            "steer_cmd_rad": float(params.get("steer_cmd_rad", cfg["steer_cmd_rad"])),
            "R_plan_m": r_plan,
            "turn_deg": float(params.get("turn_deg", cfg["turn_deg"])),
            "pin": {"lat": float(sp.get("lat")), "lon": float(sp.get("lon")),
                    "heading_deg": float(sp.get("heading_deg", 0.0))},
            "venue": {"corners_wgs84": v.get("corners_wgs84") or [],
                      "exclusions": v.get("exclusions") or []},
            "min_clearance_m": max(float(cfg["min_clearance_m"]),
                                   float(v.get("safety_margin_m", 0.0) or 0.0)),
            "max_radius_m": 2.0 * r_plan + float(cfg["radius_slack_m"]),
            "max_duration_s": float(cfg["max_duration_s"]),
        }

    def _tick_calib_run(self):
        if not self._is_alive(PROC_CALIB):
            if self._in_phase_s() > self._settle_s + 5.0:
                self._end_run("calibration node not alive")
            return
        st = self._cal_status if isinstance(self._cal_status, dict) else {}
        acked = st.get("seq") == self._cal_seq
        if not acked:
            # Re-send at ~1 Hz until the node echoes our seq (the goto idiom:
            # a single volatile publish can race the fresh node's DDS match).
            if (self.pub_calib_req.get_subscription_count() >= 1
                    and time.monotonic() - self._cal_req_last_t >= 1.0):
                try:
                    if self._cal_req is None:
                        self._cal_req = self._calib_request()
                except (TypeError, ValueError, KeyError) as exc:
                    self._end_run(f"calibration request malformed: {exc!r}")
                    return
                self.pub_calib_req.publish(String(data=json.dumps(self._cal_req)))
                self._cal_req_sent = True
                self._cal_req_last_t = time.monotonic()
            if self._in_phase_s() > 30.0:
                self._end_run("calibration request never acknowledged")
            return
        state = str(st.get("state", "")).lower()
        if state in ("done", "aborted"):
            self._cal_result = st.get("result") or {}
            self._run_end_reason = ("done" if state == "done"
                                    else "calibration aborted: "
                                    + str(st.get("reason") or "?"))
            self._enter(Phase.STOP_LEG)
            return
        if self._in_phase_s() > self._calib_timeout:
            self._end_run("calibration timeout")

    def _tick_follower_start(self):
        res = self._start_exclusive_mover(PROC_FOLLOWER)
        if res == "started":
            self._enter(Phase.SET_PARAMS)
        elif self._in_phase_s() > self._reposition_timeout:
            self._pause("follower start timeout")

    def _tick_set_params(self):
        curve = self._cur_curve
        if not self._is_alive(PROC_FOLLOWER):
            if self._in_phase_s() > self._settle_s:
                self._pause("follower not alive for set_params")
            return
        if not self._params_sent:
            # Controller + v_const come from the SYSTEM-chosen treatment, not the
            # authored curve. None (unscored traversal) -> any valid params drive.
            if self._cur_treatment is not None:
                ctrl = self._cur_treatment["controller"]
                vc = float(self._cur_treatment["v_const"])
            else:
                # Unscored traversal: params are "don't care" for science, so
                # pick the LEAST aggressive speed in the matrix, not the first.
                ctrl = self._controllers[0] if self._controllers else "lpv-hinf"
                vc = float(min(self._speeds)) if self._speeds else 0.5
            fut = self._set_follower_params(ctrl, vc)
            if fut is None:
                if self._in_phase_s() > self._reposition_timeout:
                    self._pause("follower set_parameters service never ready")
                return
            self._params_future = fut
            self._params_sent = True
            self._params_sent_t = self._ros_now_s()
            return
        if not self._params_future.done():
            # call_async has no timeout: if the follower's response is dropped
            # (rmw "failed to send response (timeout)", seen right after a
            # follower respawn), the future never resolves and the leg hangs
            # here forever. Re-send on a stall — setting params is idempotent.
            if self._ros_now_s() - self._params_sent_t > self._set_params_timeout:
                self._params_retries += 1
                if self._params_retries > self._set_params_max_retries:
                    self._params_sent = False
                    self._params_retries = 0
                    self._pause("follower set_parameters got no response after "
                                f"{self._set_params_max_retries} retries — "
                                "check the follower, then Start to retry")
                    return
                self.get_logger().warn(
                    f"set_parameters: no response in "
                    f"{self._set_params_timeout:.0f}s — re-sending "
                    f"(attempt {self._params_retries})")
                self._params_sent = False   # rebuild + re-send next tick
            return
        self._params_sent = False
        self._params_retries = 0
        # future.done() != success: a rejected parameter (e.g. bad
        # controller_type) returns successful=False per result.
        try:
            results = list(self._params_future.result().results)
        except Exception as exc:
            self._pause(f"follower set_parameters call failed: {exc}")
            return
        bad = [r.reason for r in results if not r.successful]
        if bad:
            self._pause("follower rejected params: " + "; ".join(bad))
            return
        self._enter(Phase.PUSH_RECIPE)

    def _tick_push_recipe(self):
        curve = self._cur_curve
        self._cur_recipe = curve.get("recipe", {}) or {}
        # Clear the done flag BEFORE publishing: an in-flight stale done=True
        # arriving after the clear-but-before-RUN would otherwise end the run
        # instantly. (The follower also re-publishes done=False on path load.)
        self._done = False
        self._rtk_lost_since = None
        self.pub_recipe.publish(String(data=json.dumps(self._cur_recipe)))
        self._enter(Phase.RUN)

    def _tick_run(self):
        scored = self._cur_scored
        if self._estop or self._leg_estopped:
            self._end_run("estop during run")
            return
        if self._done:
            self._end_run("done")
            return
        if scored:
            # Pause if FIXED is persistently lost during a scored run (ground
            # truth corrupted). A brief drop is tolerated up to rtk_loss_wait.
            if self._rtk_quality == RTK_FIXED:
                self._rtk_lost_since = None
            else:
                if self._rtk_lost_since is None:
                    self._rtk_lost_since = time.monotonic()
                elif time.monotonic() - self._rtk_lost_since > self._rtk_loss_wait:
                    self._end_run("RTK FIXED lost during scored run")
                    return
        if self._in_phase_s() > self._run_timeout:
            self._end_run("run timeout")
            return

    def _end_run(self, reason):
        self._run_end_reason = reason
        self._enter(Phase.STOP_LEG)

    def _tick_stop_leg(self):
        curve = self._cur_curve
        scored = self._cur_scored
        info = {}
        if self._cur_calib:
            self._orch_kill(PROC_CALIB)     # open-loop mover down before the bag
        if self._recorder is not None:
            try:
                info = self._recorder.stop(timeout_s=10.0)
            except Exception as exc:
                self.get_logger().warn(f"bag stop error: {exc}")
            self._recorder = None
        if self._cur_calib:
            self._orch_kill(PROC_CALIB)     # release cmd_vel_raw (C6)
            self._finish_calibration(curve, info)
            return
        # Release cmd_vel_raw before anything else moves (C6).
        self._orch_kill(PROC_FOLLOWER)
        reached = (self._run_end_reason == "done")
        if not reached:
            self._pause(f"curve '{curve.get('name')}' failed: {self._run_end_reason}")
            return
        if scored and self._leg_bag_path:
            passed = self._write_sidecar(curve, info, reached)
            self._record_treatment_result(passed)
            if self.phase == Phase.PAUSED:   # circuit breaker tripped
                return
            if not passed:
                # Notify-and-continue (operator decision 2026-06-12): a failed
                # leg is bookkeeping, not control flow. The cell's count stays
                # short, so the gap-filling planner re-runs it on a later
                # pass; the operator decides about exhausted cells at the end.
                self._publish_status(
                    message=f"leg FAILED validation — cell stays queued for "
                            f"redo (curve '{curve.get('name')}')")
        self.get_logger().info(
            f"curve '{curve.get('name')}' done (end_reason={self._run_end_reason}).")
        self._publish_status(message=f"curve '{curve.get('name')}' done")
        self._advance_curve()

    def _tick_done(self):
        if not getattr(self, "_done_notified", False):
            self._done_notified = True
            self._safe_state()
            done, target = self._runs_done(), self._runs_target()
            exhausted = [k for k, n in self._attempts.items()
                         if n >= self._max_retries]
            msg = f"batch COMPLETE: {done}/{target} scored runs."
            if exhausted:
                msg += (f" {len(exhausted)} cell(s) retry-exhausted / incomplete: "
                        f"{exhausted}")
            self.get_logger().info(msg)
            self._publish_status(message=msg)

    def _tick_aborted(self):
        pass

    # ==================================================================
    # Steering calibration: verdict, lock, re-plan (2026-10-08)
    # ==================================================================

    def _finish_calibration(self, curve, bag_info):
        """After the figure-8: record it, then
          full   -> lock the matrix radii (once) and re-plan the matrix here,
          sanity -> compare with the lock and continue,
        or pause with a card the operator / their agent can act on."""
        mode = self._cal_mode
        result = self._cal_result if isinstance(self._cal_result, dict) else {}
        now = datetime.now(timezone.utc)
        cfg = self._cal_cfg
        lock_doc = (self._matrix_doc or {}).get("_matrix_lock")
        record = {"stamp_utc": now.isoformat(), "mode": mode, "ok": False,
                  "epoch": (lock_doc or {}).get("epoch"), "reason": None,
                  "bag": self._leg_bag_path, "run_id": self._run_id,
                  "venue": (self._venue or {}).get("name"),
                  "pin": (curve or {}).get("start_pose"),
                  "end_reason": self._run_end_reason,
                  "driver_config": self._driver_cfg, "result": result}
        verdict, why, new_lock = False, None, None
        try:
            if self._run_end_reason != "done":
                why = str(self._run_end_reason)
            elif not result.get("ok"):
                why = f"figure-8 measurement not usable: {result.get('reason')}"
            elif mode == "full":
                lock, err = calibration.compute_lock(
                    result, cfg, now_utc=now,
                    source={"bag": self._leg_bag_path, "run_id": self._run_id,
                            "venue": (self._venue or {}).get("name"),
                            "driver_config": self._driver_cfg})
                if lock is None:
                    why = err
                else:
                    path = calibration.lock_path(self._bag_root)
                    try:
                        calibration.write_lock(path, lock)
                        verdict, new_lock = True, lock
                        record["epoch"] = lock["epoch"]
                        why = (f"locked R_min {lock['R_min_m']:.2f} m -> "
                               "radii "
                               f"{calibration.matrix_radii(self._matrix_doc, lock)}")
                    except FileExistsError:
                        # Someone locked meanwhile: judge this run against it.
                        existing = calibration.load_lock(path)
                        record["mode"] = mode = "sanity"
                        record["epoch"] = (existing or {}).get("epoch")
                        verdict, why, _r = calibration.sanity_check(
                            result, existing, cfg)
            else:
                if lock_doc is None:
                    why = "no matrix lock to check the sanity figure-8 against"
                else:
                    verdict, why, _r = calibration.sanity_check(result, lock_doc, cfg)
        except Exception as exc:  # noqa: BLE001 - a verdict must not crash the node
            verdict, why = False, f"calibration bookkeeping failed: {exc!r}"
        record["ok"] = bool(verdict)
        record["reason"] = why
        try:
            calibration.append_check(calibration.checks_path(self._bag_root), record)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"calibration check log write failed: {exc}")
        self._write_calib_sidecar(curve, bag_info, record)
        self.get_logger().info(f"calibration ({mode}) verdict ok={verdict}: {why}")
        if not verdict:
            msg = f"{mode} calibration FAILED: {why}"
            self._operator_alert(
                "calibration", "critical", "CALIBRATION FAILED",
                msg + " — press Start to retry the figure-8. If it fails again, "
                "stop and report this card.")
            self._pause(msg)
            return
        self._cal_done_session = True
        if new_lock is not None:
            warn = ("; WARN " + " | ".join(new_lock["warnings"])
                    if new_lock.get("warnings") else "")
            self._operator_alert(
                "calibration", "info", "R_min LOCKED",
                f"R_min {new_lock['R_min_m']:.2f} m at {new_lock['lock_speed']:g} "
                f"m/s -> matrix R = "
                f"{calibration.matrix_radii(self._matrix_doc, new_lock)} (epoch "
                f"{new_lock['epoch']}){warn}. Planning the matrix now "
                "(~1-2 min, the robot stays still).")
            self._cal_pin = dict((curve or {}).get("start_pose") or {})
            self._enter(Phase.REPLAN)
            return
        self._operator_alert("calibration", "info", "CALIBRATION OK", str(why))
        if not any(not any(self._is_calib_leg(lg) for lg in st["legs"])
                   for st in (self._stages or [])):
            # Calibration-only batch with a lock (the day-1 re-plan never
            # landed): plan the matrix now instead of finishing at 0/0.
            self._cal_pin = dict((curve or {}).get("start_pose") or {})
            self._enter(Phase.REPLAN)
            return
        self._advance_curve()

    def _write_calib_sidecar(self, curve, bag_info, record):
        """Sidecar next to the calibration bag. path_family calib_fig8 is not
        a matrix family, so the manifest never credits it to a cell."""
        if not self._leg_bag_path:
            return
        try:
            sc = {
                "schema_version": "calibration-1",
                "run_id": self._run_id,
                "cell_id": self._leg_cell_id,
                "leg": self._leg_id(self._leg_idx),
                "cell_params": {"controller": None, "v_const": None,
                                "radius_m": None, "path_family": "calib_fig8",
                                "rep": 0},
                "classification": {"pass": bool(record.get("ok")),
                                   "reached_end": self._run_end_reason == "done",
                                   "end_reason": self._run_end_reason},
                "wallclock": {"start_utc": self._leg_start_utc,
                              "end_utc": (bag_info or {}).get("end_utc"),
                              "duration_s": (bag_info or {}).get("duration_s")},
                "venue": {"venue_id": (self._venue or {}).get("name")},
                "bag_path": self._leg_bag_path,
                "calibration": record,
                "driver_config": {"at_start": self._leg_driver_cfg_start,
                                  "at_end": self._driver_cfg},
                "achieved_anchor": self._achieved_anchor,
            }
            json.dumps(sc, allow_nan=False)
            Data_Logger.write_sidecar(self._leg_bag_path, sc)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"calibration sidecar write failed: {exc!r}")

    def _replan_worker(self, gen, venue, doc, counts, pin):
        """Background thread: plan the locked matrix around the figure-8 just
        driven and build the active.json stages (pure geometry; no ROS).
        Publishes (gen, out); a stale generation (Start/pause in between) is
        ignored by _tick_replan."""
        out = None
        try:
            plan = experiment_planner.plan_stages(
                venue, doc, counts, footprint_r=self._footprint_r,
                track_margin=self._track_margin, key_fn=manifest.cell_key,
                calibration={"mode": "full", "pin": pin})
            stages = plan.get("stages") or []
            if not stages:
                raise ValueError("planner returned no stages: "
                                 + "; ".join(plan.get("notes") or []))
            dropped = []
            keep = []
            for st in stages:
                if st.get("needs_fix"):
                    dropped.append(f"{st.get('name')} "
                                   f"{[(g['family'], g['R']) for g in st.get('geometries') or []]}"
                                   " (no clean placement)")
                    continue
                keep.append(st)
            final = []
            for st in experiment_planner.stages_to_legs(keep, venue):
                ok, rep_ = venue_geom.check_legs_containment(
                    st["legs"], venue, self._footprint_r, self._track_margin)
                if not ok:
                    dropped.append(f"{st['name']} (containment: "
                                   f"{rep_.splitlines()[-1][:120]})")
                    continue
                for key in ("entry_glue", "wrap_glue"):
                    g = st.get(key)
                    if not g:
                        continue
                    wps = g.get("waypoints_wgs84") or []
                    gok = False
                    if not g.get("needs_fix") and len(wps) >= 2:
                        gok, _r = venue_geom.check_legs_containment(
                            [{"id": key, "curves": [{"kind": "reposition",
                                                      "name": key,
                                                      "waypoints_wgs84": wps}]}],
                            venue, self._footprint_r, self._track_margin)
                    if not gok:
                        st.pop(key)     # executor falls back to the path-join
                final.append(st)
            if not final or final[0]["name"] != "calibration":
                raise ValueError("the calibration stage is missing from the re-plan")
            out = {"plan_stages": final, "dropped": dropped,
                   "notes": plan.get("notes") or [], "plan": plan}
        except Exception as exc:  # noqa: BLE001
            out = {"error": repr(exc)}
        self._replan_out = (gen, out)

    def _tick_replan(self):
        if self._replan_thread is None:
            if experiment_planner is None or manifest is None:
                self._pause("re-plan impossible: planner/manifest unavailable")
                return
            if not self._load_matrix():
                self._pause("re-plan: experiment.yaml unreadable")
                return
            self._rebuild_counts()
            venue = {k: v for k, v in (self._venue or {}).items()
                     if k not in ("plan_stages", "legs")}
            self._replan_gen += 1
            self._replan_out = None
            self._replan_thread = threading.Thread(
                target=self._replan_worker,
                args=(self._replan_gen, venue, dict(self._matrix_doc),
                      dict(self._completed_counts), dict(self._cal_pin or {})),
                daemon=True)
            self._replan_thread.start()
            self.get_logger().info("re-plan started (matrix under the new lock).")
            self._publish_status(message="planning the matrix from the new lock")
            return
        if self._replan_thread is not None and self._replan_thread.is_alive():
            if self._in_phase_s() > self._replan_timeout:
                self._replan_thread = None
                self._replan_gen += 1          # orphan the stuck worker
                self._pause("re-plan timed out — Auto-plan, Send, Start "
                            "(the lock is kept; no new figure-8 needed)")
                return
            if int(self._in_phase_s()) % 5 == 0:
                self._publish_status(message="planning the matrix from the new lock")
            return
        got, self._replan_out, self._replan_thread = self._replan_out, None, None
        out = got[1] if (isinstance(got, tuple) and got[0] == self._replan_gen) else None
        if not out or out.get("error"):
            why = (out or {}).get("error", "no result")
            self._operator_alert("replan", "critical", "MATRIX RE-PLAN FAILED",
                                 f"{why} — Auto-plan, Send, Start (the lock is kept).")
            self._pause(f"re-plan failed: {why}")
            return
        self._apply_replan(out)

    def _apply_replan(self, out):
        """Persist the re-planned batch exactly as venue_loader does on Send
        (<name>.json + active.json, atomic), reload it, and continue from the
        figure-8's exit into the first matrix stage."""
        v = {k: val for k, val in (self._venue or {}).items()
             if k not in ("plan_stages", "legs")}
        v["plan_stages"] = out["plan_stages"]
        v["legs"] = out["plan_stages"][0]["legs"]
        v.setdefault("schema_version", 2)
        v["auto_replan"] = {
            "stamp_utc": datetime.now(timezone.utc).isoformat(),
            "epoch": (self._matrix_doc or {}).get("_matrix_epoch"),
            "radius_m": ((self._matrix_doc or {}).get("matrix") or {}).get("radius_m"),
            "dropped": out.get("dropped") or [],
        }
        name = str(v.get("name") or "venue")
        safe = "".join(c if (c.isalnum() or c in "-._") else "-" for c in name)
        try:
            for path in (os.path.join(os.path.dirname(self._active_file),
                                      safe + ".json"), self._active_file):
                tmp = path + ".tmp"
                with open(tmp, "w", encoding="utf-8") as f:
                    json.dump(v, f, indent=2)
                os.replace(tmp, path)
        except Exception as exc:  # noqa: BLE001
            self._pause(f"re-plan persist failed: {exc!r}")
            return
        self._reload_active()
        self._rebuild_counts()
        if not self._stages or self._stages[0]["name"] != "calibration":
            self._pause("re-plan reload lost the calibration stage")
            return
        self._stage_idx = 0
        self._legs = self._stages[0]["legs"]
        self._last_completed_leg_idx = 0
        self._leg_idx = 0
        self._curve_idx = 0
        n_matrix = len(self._stages) - 1
        dropped = out.get("dropped") or []
        msg = (f"matrix planned: {n_matrix} stage(s), "
               f"{self._runs_target()} scored runs"
               + (f"; NOT placed: {dropped}" if dropped else "") + ".")
        self.get_logger().info(msg)
        self._operator_alert("replan", "warn" if dropped else "info",
                             "MATRIX PLANNED", msg)
        try:
            plan = dict(out.get("plan") or {})
            plan["venue_name"] = self._run_id
            plan.setdefault("notes", []).insert(
                0, "planned and loaded by the robot after the calibration lock")
            self.pub_plan.publish(String(data=json.dumps(plan)))
        except Exception:  # noqa: BLE001
            pass
        res = self._advance_stage()
        if res == "paused":
            return
        if res == "none":
            self._enter(Phase.DONE)
            return
        self._dwell_until = time.monotonic() + self._dwell_s
        self._begin_current_curve()

    # ==================================================================
    # Sidecar
    # ==================================================================

    def _write_sidecar(self, curve, bag_info, reached):
        """Write the leg sidecar with the SYSTEM-chosen treatment. Returns the
        classification.pass verdict (False on any failure to write)."""
        if Data_Logger is None or not self._leg_bag_path:
            return False
        t = self._cur_treatment or {}
        ctrl = t.get("controller", "lpv-hinf")
        vc = float(t.get("v_const", 1.0))
        rep = int(t.get("rep", 0))
        try:
            if self._leg_rtk_total_samples > 0:
                pct = 100.0 * self._leg_rtk_fixed_samples / self._leg_rtk_total_samples
            else:
                pct = 0.0
            classification = {
                "pass": bool(reached) and not self._leg_estopped
                and pct >= self._rtk_window_pct,
                "reached_end": bool(reached),
                "estop": bool(self._leg_estopped or self._estop),
                "duration_s": float(bag_info.get("duration_s", 0.0) or 0.0),
                "rtk_fixed_pct": round(pct, 1),
                "rtk_window_pct_required": self._rtk_window_pct,
                "end_reason": self._run_end_reason,
            }
            # Bag-level quick gate: the bag is closed here, so its metadata is
            # final. A leg whose RECORDING is broken must not count toward the
            # cell's N even when the run itself was clean (the live checks
            # above can't see inside the bag) — folding the verdict into
            # `pass` makes the gap-filling treatment planner redo the cell on
            # a later pass, with no extra retry machinery.
            if manifest is not None and hasattr(manifest, "quick_gate"):
                qg = manifest.quick_gate(self._leg_bag_path)
                classification["quick_gate"] = qg
                if not qg["pass"]:
                    classification["pass"] = False
                    self.get_logger().warn(
                        "quick-gate FAIL on "
                        f"{os.path.basename(self._leg_bag_path)}: "
                        f"{';'.join(qg['reasons'])}")
            # Same plant for the whole leg: expected steering/odometry at both
            # ends and no driver respawn in between (leg_driver_config_verdict).
            cfg_ok, cfg_why = leg_driver_config_verdict(
                self._leg_driver_cfg_start, self._driver_cfg, self._driver_expect)
            classification["driver_config_ok"] = cfg_ok
            if not cfg_ok:
                classification["pass"] = False
                classification["driver_config_reasons"] = cfg_why
                self.get_logger().warn("driver-config FAIL: " + "; ".join(cfg_why))
            sp = curve.get("start_pose") or {}
            path_frame_anchor = None
            if sp:
                path_frame_anchor = {
                    "pin_id": self._leg_cell_id,
                    "lat": float(sp["lat"]),
                    "lon": float(sp["lon"]),
                    "heading_deg": float(sp.get("heading_deg", 0.0)),
                }
            recipe = self._cur_recipe or {}
            params = recipe.get("params", {}) or {}
            sidecar = Data_Logger.build_sidecar(
                run_id=self._run_id,
                cell_id=self._leg_cell_id or curve.get("name", "curve"),
                leg=self._leg_id(self._leg_idx),
                cell_params={
                    "controller": ctrl,
                    "v_const": vc,
                    "radius_m": params.get("R"),
                    "path_family": recipe.get("type"),
                    "rep": rep,
                },
                path_recipe=recipe,
                venue_id=self._venue.get("name"),
                start_pin_id=self._leg_cell_id,
                end_pin_id=None,
                rtk_summary={
                    "fixed_pct": classification["rtk_fixed_pct"],
                    "fixed_samples": self._leg_rtk_fixed_samples,
                    "total_samples": self._leg_rtk_total_samples,
                    "ok_qualities": [RTK_FIXED],
                },
                classification=classification,
                wallclock={
                    "start_utc": self._leg_start_utc,
                    "end_utc": bag_info.get("end_utc"),
                    "duration_s": bag_info.get("duration_s"),
                },
                controller_tuning={
                    "controller_type": ctrl,
                    "v_const": vc,
                },
                bag_path=self._leg_bag_path,
                topics=Data_Logger.TOPICS,
                path_frame_anchor=path_frame_anchor,
                achieved_anchor=self._achieved_anchor,
            )
            # How the robot reached the start pin (glue as sent + its metrics +
            # reposition's final status; see _capture_arrival). Additive
            # top-level key, set here rather than through a new build_sidecar
            # argument so an executor/Data_Logger version skew on the NUC can't
            # TypeError the sidecar (and so the leg). Always present on legs
            # from 2026-10-04 on; None only if the capture never ran.
            sidecar["arrival"] = self._arrival
            # Which chassis steering/odometry produced the leg (additive
            # top-level key, same skew reasoning as 'arrival').
            sidecar["driver_config"] = {
                "at_start": self._leg_driver_cfg_start,
                "at_end": self._driver_cfg,
                "expected": dict(self._driver_expect),
            }
            # Calibration lock the matrix is gated on (2026-10-08). Progress
            # credits only legs of the current epoch (manifest.build_rows).
            # radius_m = the radii this matrix sweeps (the lock's own for
            # auto, the authored list for calibration.required).
            lock = (self._matrix_doc or {}).get("_matrix_lock") or {}
            sidecar["matrix_epoch"] = (self._matrix_doc or {}).get("_matrix_epoch")
            sidecar["matrix_lock"] = ({"epoch": lock.get("epoch"),
                                       "R_min_m": lock.get("R_min_m"),
                                       "delta_max_rad": lock.get("delta_max_rad"),
                                       "radius_m": calibration.matrix_radii(
                                           self._matrix_doc, lock)
                                       if calibration else lock.get("radius_m")}
                                      if lock else None)
            Data_Logger.write_sidecar(self._leg_bag_path, sidecar)
            return bool(classification["pass"])
        except Exception as exc:
            self.get_logger().error(f"sidecar write failed: {exc}")
            return False

    # ==================================================================
    # Pause / abort / safe-state
    # ==================================================================

    def _repo_reason(self):
        st = self._repo_status if isinstance(self._repo_status, dict) else {}
        return str(st.get("reason") or "aborted")

    def _pause(self, reason):
        # IDLE: nothing is running, so there is nothing to pause. Guards like
        # rtk_watchdog send pause whenever the fix is unusable (correctly SEVERE
        # on a bench indoors); paging "RUN PAUSED" for a run that never started
        # would train the operator to ignore the page.
        if self.phase in (Phase.IDLE, Phase.PAUSED, Phase.ABORTED, Phase.DONE):
            return
        self._pause_reason = reason
        self.get_logger().warn(f"PAUSE: {reason}")
        _notify_discord(f"RUN PAUSED: {reason}", title="H-inf run_executor")
        self._safe_state()      # movers down FIRST; the recorder stop can take 5 s
        if self._recorder is not None:
            try:
                self._recorder.stop(timeout_s=5.0)
            except Exception:
                pass
            self._recorder = None
        self._safe_state()
        self._enter(Phase.PAUSED)
        self._publish_status(message=f"paused: {reason}")

    def _abort(self, reason):
        self.get_logger().warn(f"ABORT: {reason}")
        _notify_discord(f"RUN ABORTED: {reason}", title="H-inf run_executor")
        self._safe_state()      # movers down FIRST; the recorder stop can take 5 s
        if self._recorder is not None:
            try:
                self._recorder.stop(timeout_s=5.0)
            except Exception:
                pass
            self._recorder = None
        self._safe_state()
        self._enter(Phase.ABORTED)
        self._publish_status(message=f"aborted: {reason}")

    def _safe_state(self):
        for name in MOVERS:
            self._orch_kill(name)
        self._goto_sent = False
        self._params_sent = False
        self._params_retries = 0

    def _reset_run_scratch(self):
        self._leg_bag_path = None
        self._leg_cell_id = None
        self._leg_start_utc = None
        self._leg_driver_cfg_start = None
        self._leg_estopped = False
        self._leg_rtk_fixed_samples = 0
        self._leg_rtk_total_samples = 0
        self._done = False
        self._run_end_reason = None

    # ==================================================================
    # Status
    # ==================================================================

    def _operator_alert(self, alert_id, level, title, detail):
        """Page the operator: a browser card (/operator/alert) + Discord (off
        by default, see ntfy.DISCORD_ENABLED). level: 'info' | 'warn' |
        'critical' | 'clear'. Never raises — a page must not break a batch."""
        try:
            self.pub_alert.publish(String(data=json.dumps({
                "id": alert_id, "level": level, "title": title,
                "detail": detail, "stamp": time.time()})))
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"operator alert publish failed: {exc}")
        _notify_discord(f"{title}: {detail}", title="H-inf run_executor")

    def _publish_status(self, message=""):
        cmd_owner = None
        if self.phase in (Phase.REPOSITION_START, Phase.REPOSITION_GOTO):
            cmd_owner = "reposition"
        elif self.phase in (Phase.FOLLOWER_START, Phase.SET_PARAMS,
                            Phase.PUSH_RECIPE, Phase.RUN):
            cmd_owner = "follower"
        elif self.phase in (Phase.CALIB_START, Phase.CALIB_RUN):
            cmd_owner = "calib"
        curve = self._cur_curve or {}
        runs_done = self._runs_done()
        self.pub_status.publish(String(data=json.dumps({
            "run_id": self._run_id,
            "venue": self._venue.get("name"),
            "phase": self.phase.value,
            "stage": self._stage_name(),
            "stage_index": self._stage_idx if self._stages else None,
            "n_stages": len(self._stages) or None,
            "leg_index": self._leg_idx,
            "n_legs": len(self._legs),
            "n_complete": runs_done,
            "runs_done": runs_done,
            "runs_target": self._runs_target(),
            "treatment": self._cur_treatment,
            "leg_id": self._leg_id(self._leg_idx),
            "curve_name": curve.get("name"),
            "curve_kind": curve.get("kind"),
            "cmd_owner": cmd_owner,
            "battery_v": self._battery_v,
            "rtk_q": self._rtk_quality,
            "pause_reason": self._pause_reason,
            "progress": self._progress,
            "message": message,
        })))

    def shutdown(self):
        try:
            self._safe_state()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = RunExecutor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
