# -*- coding: utf-8 -*-
"""run_executor_node — run an operator-authored leg batch with resume.

The operator authors a venue + an ordered list of LEGS in the WebUI (each leg =
a reposition glue curve + a scored experiment recipe), Sends it (venue_loader
persists active.json), then presses ONE Start button (``/run/go``). This node:

  - finds the first INCOMPLETE leg (resume point — "which leg to run"),
  - re-checks containment of every remaining curve BEFORE any motion,
  - runs a one-time preflight, then runs that leg and every leg after it, in
    order, auto-advancing, until all legs finish OR the battery hits the halt
    threshold (then Start again resumes at the next incomplete leg).

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
  pub  /run/status  std_msgs/String  JSON (latched)
  + the orchestrator / reposition / odom_zero / follower contracts (see imports).
"""
import json
import os
import sys
import time
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


PROC_REPOSITION = "reposition"
PROC_ODOM_ZERO = "odom_zero"
PROC_FOLLOWER = "follower"
MOVERS = (PROC_REPOSITION, PROC_FOLLOWER)

RTK_FIXED = 4
RTK_OK = (4, 5)

DEFAULT_ACTIVE = os.path.join(_REPO_ROOT, "scenarios", "venues", "active.json")
DEFAULT_CHECKPOINT = os.path.join(_REPO_ROOT, "Experiment Data", "run_checkpoint.json")
DEFAULT_BAG_ROOT = os.path.join(_REPO_ROOT, "Experiment Data")


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
    STOP_LEG = "stop_leg"
    DONE = "done"
    PAUSED = "paused"
    ABORTED = "aborted"


class RunExecutor(Node):

    def __init__(self):
        super().__init__("run_executor_node")

        self.declare_parameter("active_venue_file", DEFAULT_ACTIVE)
        self.declare_parameter("checkpoint_path", DEFAULT_CHECKPOINT)
        self.declare_parameter("bag_root", DEFAULT_BAG_ROOT)
        self.declare_parameter("preflight_timeout_s", 60.0)
        self.declare_parameter("reposition_timeout_s", 120.0)
        self.declare_parameter("run_timeout_s", 180.0)
        self.declare_parameter("orchestrator_settle_s", 3.0)
        self.declare_parameter("odom_settle_s", 3.0)
        self.declare_parameter("rtk_fix_wait_s", 30.0)       # scored-run FIXED gate
        self.declare_parameter("rtk_loss_wait_s", 5.0)       # FIXED loss during scored run
        self.declare_parameter("heartbeat_s", 30.0)
        self.declare_parameter("battery_volts_halt", 10.5)
        self.declare_parameter("rtk_run_window_pct", 95.0)
        self.declare_parameter("robot_footprint_radius_m", 0.30)
        self.declare_parameter("path_tracking_margin_m", 0.30)

        self._active_file = str(self.get_parameter("active_venue_file").value)
        self._checkpoint_path = str(self.get_parameter("checkpoint_path").value)
        self._bag_root = str(self.get_parameter("bag_root").value)
        self._preflight_timeout = float(self.get_parameter("preflight_timeout_s").value)
        self._reposition_timeout = float(self.get_parameter("reposition_timeout_s").value)
        self._run_timeout = float(self.get_parameter("run_timeout_s").value)
        self._settle_s = float(self.get_parameter("orchestrator_settle_s").value)
        self._odom_settle_s = float(self.get_parameter("odom_settle_s").value)
        self._rtk_fix_wait = float(self.get_parameter("rtk_fix_wait_s").value)
        self._rtk_loss_wait = float(self.get_parameter("rtk_loss_wait_s").value)
        self._heartbeat_s = float(self.get_parameter("heartbeat_s").value)
        self._batt_halt = float(self.get_parameter("battery_volts_halt").value)
        self._rtk_window_pct = float(self.get_parameter("rtk_run_window_pct").value)
        self._footprint_r = float(self.get_parameter("robot_footprint_radius_m").value)
        self._track_margin = float(self.get_parameter("path_tracking_margin_m").value)

        # -- Batch state -----------------------------------------------
        self._venue = {}
        self._legs = []
        self._run_id = "run"
        self._completed = set()
        self._leg_idx = 0
        self._curve_idx = 0
        self._cur_curve = None
        self._cur_recipe = {}

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
        self._params_sent = False
        self._params_future = None
        self._odom_reset_sent = False
        self._odom_reset_command_t = None
        self._rtk_lost_since = None

        # -- Sensor snapshots ------------------------------------------
        self._done = False
        self._estop = False
        self._rtk_quality = None
        self._battery_v = None
        self._orch_status = {}
        self._repo_state = None
        self._repo_status = {}
        self._odom_zero_status = None
        self._last_odom_t = None

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

        self.create_subscription(String, "/run/go", self._on_go, 10)
        self.create_subscription(String, "/run/cmd", self._on_cmd, 10)
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
            String, "/odom_zero/status", self._on_odom_zero_status, latched)
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
        self._completed = self._load_completed()
        self.get_logger().info(
            f"run_executor_node up. active={self._active_file}: "
            f"{len(self._legs)} legs, {len(self._completed)} complete. "
            f"Waiting for /run/go (Start).")
        self._publish_status(message="loaded")

    # ==================================================================
    # Subscriptions
    # ==================================================================

    def _on_go(self, msg):
        if self.phase == Phase.IDLE:
            self._begin_run()
        elif self.phase == Phase.PAUSED:
            self._resume()
        elif self.phase == Phase.DONE:
            self.get_logger().info("Start ignored — batch DONE (re-Send to redo).")
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
            self._pause("operator pause")
        elif action == "resume":
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

    def _on_repo_status(self, msg):
        try:
            d = json.loads(msg.data)
            self._repo_status = d if isinstance(d, dict) else {}
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

    def _on_limo(self, msg):
        try:
            self._battery_v = float(msg.battery_voltage)
        except Exception:
            pass

    def _on_odom_zero_status(self, msg):
        try:
            self._odom_zero_status = json.loads(msg.data) or {}
        except (ValueError, TypeError):
            pass

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

    def _other_mover(self, name):
        return PROC_FOLLOWER if name == PROC_REPOSITION else PROC_REPOSITION

    def _start_exclusive_mover(self, name):
        """Bring up one cmd_vel_raw mover, guaranteeing C6. Returns
        'killing_other' | 'waiting_other_down' | 'started'."""
        other = self._other_mover(name)
        if self._is_alive(other):
            self._orch_kill(other)
            return "killing_other"
        if other not in self._orch_status:
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
            return
        self._venue = v
        self._legs = v.get("legs") or []
        self._run_id = str(v.get("name") or "run")

    def _load_completed(self):
        try:
            with open(self._checkpoint_path, "r", encoding="utf-8") as f:
                cp = json.load(f)
        except (FileNotFoundError, ValueError):
            return set()
        if str(cp.get("venue_name")) != str(self._run_id):
            return set()
        return set(cp.get("completed_leg_ids", []))

    def _save_completed(self):
        try:
            os.makedirs(os.path.dirname(self._checkpoint_path) or ".", exist_ok=True)
            tmp = self._checkpoint_path + ".tmp"
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump({
                    "venue_name": self._run_id,
                    "completed_leg_ids": sorted(self._completed),
                    "updated_utc": datetime.now(timezone.utc).isoformat(),
                }, f, indent=2)
                f.write("\n")
            os.replace(tmp, self._checkpoint_path)
        except Exception as exc:
            self.get_logger().error(f"checkpoint save failed: {exc}")

    def _leg_id(self, idx):
        if 0 <= idx < len(self._legs):
            return self._legs[idx].get("id", f"leg_{idx}")
        return None

    def _first_incomplete_leg(self):
        for i in range(len(self._legs)):
            if self._leg_id(i) not in self._completed:
                return i
        return len(self._legs)

    def _begin_run(self):
        self._reload_active()
        if not self._legs:
            self._publish_status(message="no legs loaded — Send a venue first")
            return
        self._completed = self._load_completed()
        self._leg_idx = self._first_incomplete_leg()
        self._curve_idx = 0
        if self._leg_idx >= len(self._legs):
            self._enter(Phase.DONE)
            return
        ok, report = venue_geom.check_legs_containment(
            self._legs[self._leg_idx:], self._venue,
            self._footprint_r, self._track_margin)
        if not ok:
            self.get_logger().error(
                "VENUE CONTAINMENT FAILED — refusing to run (no motion):\n" + report)
            self._pause("refused: a planned curve leaves the venue (see log)")
            return
        self.get_logger().info("venue containment OK — " + report)
        self.get_logger().info(
            f"starting at leg {self._leg_idx + 1}/{len(self._legs)} "
            f"('{self._leg_id(self._leg_idx)}').")
        self._enter(Phase.PREFLIGHT)

    def _resume(self):
        if self.phase != Phase.PAUSED:
            return
        self._completed = self._load_completed()
        self._leg_idx = self._first_incomplete_leg()
        self._curve_idx = 0
        self._pause_reason = None
        self._rtk_lost_since = None
        if self._leg_idx >= len(self._legs):
            self._enter(Phase.DONE)
            return
        self.get_logger().info(
            f"resume at leg {self._leg_idx + 1}/{len(self._legs)} "
            f"('{self._leg_id(self._leg_idx)}').")
        self._enter(Phase.PREFLIGHT)

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
        if kind == "reposition":
            self._enter(Phase.REPOSITION_START)
        elif kind == "recipe":
            self._enter(Phase.KILL_REPOSITION)
        else:
            self._pause(f"unknown curve kind '{kind}'")

    def _advance_curve(self):
        self._curve_idx += 1
        curves = self._legs[self._leg_idx].get("curves") or []
        if self._curve_idx >= len(curves):
            self._complete_leg()
        else:
            self._begin_current_curve()

    def _complete_leg(self):
        lid = self._leg_id(self._leg_idx)
        self._completed.add(lid)
        self._save_completed()
        self.get_logger().info(
            f"leg '{lid}' complete ({len(self._completed)}/{len(self._legs)}).")
        self._publish_status(message=f"leg '{lid}' complete")
        self._leg_idx += 1
        self._curve_idx = 0
        if self._leg_idx >= len(self._legs):
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
            if phase == Phase.ODOM_RESET:
                self._odom_reset_sent = False
                self._odom_reset_command_t = None
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

        handler = getattr(self, f"_tick_{self.phase.value}", None)
        if handler is not None:
            handler()
        else:
            self.get_logger().error(f"no handler for phase {self.phase.value}")
            self._pause(f"no handler for phase {self.phase.value}")

    # -- Per-phase handlers --------------------------------------------

    def _tick_preflight(self):
        missing = []
        if self._estop:
            missing.append("estop engaged")
        if self._last_odom_t is None or (time.monotonic() - self._last_odom_t) > 1.0:
            missing.append("no fresh /wheel/odom")
        if self._rtk_quality not in RTK_OK:
            missing.append(f"RTK quality {self._rtk_quality} not in {RTK_OK}")
        if self._battery_v is not None and self._battery_v < self._batt_halt:
            missing.append(f"battery {self._battery_v:.2f}V < {self._batt_halt}V")
        if not missing:
            self.get_logger().info("preflight pass")
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
        if not self._goto_sent:
            curve = self._cur_curve
            wps = curve.get("waypoints_wgs84") or []
            payload = {
                "waypoints": [{"lat": w["lat"], "lon": w["lon"]} for w in wps],
                "v_const": float(curve.get("v_const", 0.4)),
                "pos_tol_m": float(curve.get("pos_tol_m", 0.15)),
            }
            if curve.get("end_heading_deg") is not None:
                payload["end_heading_deg"] = float(curve["end_heading_deg"])
            self.pub_goto.publish(String(data=json.dumps(payload)))
            self._goto_sent = True
            return
        if self._repo_state == "arrived":
            self._goto_sent = False
            self._advance_curve()
        elif self._repo_state == "aborted":
            self._goto_sent = False
            self._pause("reposition aborted: " + self._repo_reason())
        elif self._in_phase_s() > self._reposition_timeout:
            self._goto_sent = False
            self._pause("reposition goto timeout")

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
        if not self._odom_reset_sent:
            self._odom_reset_command_t = self._ros_now_s()
            self.pub_odom_reset.publish(Bool(data=True))
            self._odom_reset_sent = True
            return
        s = self._odom_zero_status or {}
        if (s.get("has_reset") is True and s.get("stamp") is not None
                and float(s["stamp"]) >= float(self._odom_reset_command_t)):
            origin = s.get("origin") or {}
            self.get_logger().info(
                f"odom_zero latch confirmed at (x={origin.get('x')}, "
                f"y={origin.get('y')}, yaw={origin.get('yaw')}).")
            self._enter(Phase.BAG_START)
            return
        if self._in_phase_s() > self._odom_settle_s:
            self._pause(
                f"odom_zero reset not confirmed within {self._odom_settle_s:.1f}s")

    def _tick_bag_start(self):
        curve = self._cur_curve
        scored = bool(curve.get("scored", True))
        # RTK FIXED(4) gate for scored runs: the bag's RTK is the post-hoc ground
        # truth, so do not start recording until FIXED.
        if scored and self._rtk_quality != RTK_FIXED:
            if self._in_phase_s() < self._rtk_fix_wait:
                self._publish_status(
                    message=f"waiting RTK FIXED(4) for scored run (q={self._rtk_quality})")
                return
            self._pause(
                f"RTK not FIXED(4) for scored run (q={self._rtk_quality})")
            return
        self._reset_run_scratch()
        if scored:
            if Data_Logger is None:
                self._pause("recorder unavailable (Data_Logger import failed)")
                return
            try:
                dirname = Data_Logger.build_leg_dirname(
                    self._run_id, curve.get("name", "curve"),
                    self._leg_id(self._leg_idx))
                self._leg_bag_path = os.path.join(self._bag_root, dirname)
                self._recorder = Data_Logger.BagRecorder(
                    self._leg_bag_path, topics=Data_Logger.TOPICS)
                self._recorder.start()
                self._leg_start_utc = datetime.now(timezone.utc).isoformat()
            except Exception as exc:
                self._recorder = None
                self._pause(f"bag start failed: {exc}")
                return
        self._enter(Phase.FOLLOWER_START)

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
            fut = self._set_follower_params(
                curve.get("controller", "lpv-hinf"),
                float(curve.get("v_const", 1.0)))
            if fut is None:
                if self._in_phase_s() > self._reposition_timeout:
                    self._pause("follower set_parameters service never ready")
                return
            self._params_future = fut
            self._params_sent = True
            return
        if self._params_future.done():
            self._params_sent = False
            self._enter(Phase.PUSH_RECIPE)

    def _tick_push_recipe(self):
        curve = self._cur_curve
        self._cur_recipe = curve.get("recipe", {}) or {}
        self.pub_recipe.publish(String(data=json.dumps(self._cur_recipe)))
        self._done = False
        self._rtk_lost_since = None
        self._enter(Phase.RUN)

    def _tick_run(self):
        scored = bool(self._cur_curve.get("scored", True))
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
        scored = bool(curve.get("scored", True))
        info = {}
        if self._recorder is not None:
            try:
                info = self._recorder.stop(timeout_s=10.0)
            except Exception as exc:
                self.get_logger().warn(f"bag stop error: {exc}")
            self._recorder = None
        # Release cmd_vel_raw before anything else moves (C6).
        self._orch_kill(PROC_FOLLOWER)
        reached = (self._run_end_reason == "done")
        if scored and self._leg_bag_path:
            self._write_sidecar(curve, info, reached)
        if reached:
            self.get_logger().info(
                f"curve '{curve.get('name')}' done "
                f"(end_reason={self._run_end_reason}).")
            self._publish_status(message=f"curve '{curve.get('name')}' done")
            self._advance_curve()
        else:
            self._pause(f"curve '{curve.get('name')}' failed: {self._run_end_reason}")

    def _tick_done(self):
        if not getattr(self, "_done_notified", False):
            self._done_notified = True
            self._safe_state()
            self.get_logger().info(
                f"batch COMPLETE: {len(self._completed)}/{len(self._legs)} legs.")
            self._publish_status(message="batch complete")

    def _tick_aborted(self):
        pass

    # ==================================================================
    # Sidecar
    # ==================================================================

    def _write_sidecar(self, curve, bag_info, reached):
        if Data_Logger is None or not self._leg_bag_path:
            return
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
            sp = curve.get("start_pose") or {}
            path_frame_anchor = None
            if sp:
                path_frame_anchor = {
                    "pin_id": curve.get("name"),
                    "lat": float(sp["lat"]),
                    "lon": float(sp["lon"]),
                    "heading_deg": float(sp.get("heading_deg", 0.0)),
                }
            recipe = self._cur_recipe or {}
            params = recipe.get("params", {}) or {}
            sidecar = Data_Logger.build_sidecar(
                run_id=self._run_id,
                cell_id=curve.get("name", "curve"),
                leg=self._leg_id(self._leg_idx),
                cell_params={
                    "controller": curve.get("controller", "lpv-hinf"),
                    "v_const": curve.get("v_const", 1.0),
                    "radius_m": params.get("R"),
                    "path_family": recipe.get("type"),
                    "rep": 0,
                },
                path_recipe=recipe,
                venue_id=self._venue.get("name"),
                start_pin_id=curve.get("name"),
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
                    "controller_type": curve.get("controller", "lpv-hinf"),
                    "v_const": curve.get("v_const", 1.0),
                },
                bag_path=self._leg_bag_path,
                topics=Data_Logger.TOPICS,
                path_frame_anchor=path_frame_anchor,
            )
            Data_Logger.write_sidecar(self._leg_bag_path, sidecar)
        except Exception as exc:
            self.get_logger().error(f"sidecar write failed: {exc}")

    # ==================================================================
    # Pause / abort / safe-state
    # ==================================================================

    def _repo_reason(self):
        st = self._repo_status if isinstance(self._repo_status, dict) else {}
        return str(st.get("reason") or "aborted")

    def _pause(self, reason):
        if self.phase in (Phase.PAUSED, Phase.ABORTED, Phase.DONE):
            return
        self._pause_reason = reason
        self.get_logger().warn(f"PAUSE: {reason}")
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

    def _reset_run_scratch(self):
        self._leg_bag_path = None
        self._leg_start_utc = None
        self._leg_estopped = False
        self._leg_rtk_fixed_samples = 0
        self._leg_rtk_total_samples = 0
        self._done = False
        self._run_end_reason = None

    # ==================================================================
    # Status
    # ==================================================================

    def _publish_status(self, message=""):
        cmd_owner = None
        if self.phase in (Phase.REPOSITION_START, Phase.REPOSITION_GOTO):
            cmd_owner = "reposition"
        elif self.phase in (Phase.FOLLOWER_START, Phase.SET_PARAMS,
                            Phase.PUSH_RECIPE, Phase.RUN):
            cmd_owner = "follower"
        curve = self._cur_curve or {}
        self.pub_status.publish(String(data=json.dumps({
            "run_id": self._run_id,
            "venue": self._venue.get("name"),
            "phase": self.phase.value,
            "leg_index": self._leg_idx,
            "n_legs": len(self._legs),
            "n_complete": len(self._completed),
            "leg_id": self._leg_id(self._leg_idx),
            "curve_name": curve.get("name"),
            "curve_kind": curve.get("kind"),
            "cmd_owner": cmd_owner,
            "battery_v": self._battery_v,
            "rtk_q": self._rtk_quality,
            "pause_reason": self._pause_reason,
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
