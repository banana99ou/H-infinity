# -*- coding: utf-8 -*-
"""Experiment sequencer (T6) — the integrator that drives the full matrix unattended.

This node owns the per-cell state machine described in DOC/experiment.md
(operational model) and DOC/system_spec.md §3.6/§3.10. It coordinates the
already-built primitives via their documented topic contracts only — it does
NOT import other task agents' node code:

  - orchestrator supervisor : /orchestrator/{start,kill,status}
      managed names used here: 'reposition', 'odom_zero', 'follower'
  - reposition (T4)          : /reposition/goto (out), /reposition/status (in)
  - odom-zero overlay (T3)   : /odom_zero/reset (out)
  - follower (T1)            : /reference_path_recipe (out, latched),
                               /path_follower/done (in, latched),
                               /path_follower_node/set_parameters (service)
  - estop (C3/C4)            : /estop (in, latched) — a True means halt
  - RTK (L2/M1)              : /gps_rtk_f9p_helical/gps/rtk_status (in)
  - battery (M2)             : /limo_status.battery_voltage (in)
  - recorder (T7)            : Data_Logger.BagRecorder / build_sidecar / write_sidecar
  - preflight (T9)           : tools/preflight/preflight.sh (subprocess, exit 0 = pass)
  - notify (T8)              : tools/notify/ntfy.py (imported defensively)

Published / subscribed contract (T10 codes against this):
  pub  /experiment/status  std_msgs/String JSON
       {run_id, cell_id, cell_index, n_cells, leg, phase, pass, fail, eta_s, message}
  sub  /experiment/cmd     std_msgs/String JSON {action: pause|resume|abort|skip}

THE SINGLE MOST IMPORTANT INVARIANT (C6): exactly one cmd_vel_raw publisher is
live at any instant — the follower XOR the reposition node, never both. This is
enforced structurally: every transition that starts one first kills the other
and confirms (via /orchestrator/status) that it is down before bringing the
other up. See _start_exclusive_mover().
"""

import itertools
import json
import math
import os
import subprocess
import sys
import threading
import time
from datetime import datetime, timezone
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import ParameterValue, ParameterType
from rcl_interfaces.msg import Parameter as ParameterMsg
from std_msgs.msg import String, Bool
from nav_msgs.msg import Odometry

# --- Recorder (T7) lives at the repo root, outside the colcon package. -------
# CLAUDE.md / the shared contract say to import it by adding the repo root to
# sys.path. Keep the import defensive so a path slip degrades to "no recording"
# rather than crashing the whole batch (a recorded run with no bag will just be
# classified fail and retried).
_REPO_ROOT = "/home/agilex/H-infinity"
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)
try:
    import Data_Logger  # type: ignore
except Exception:  # pragma: no cover - exercised only on a broken deploy
    Data_Logger = None

# --- Notify (T8) is optional; import defensively so a missing module never ---
# crashes the batch (per the shared contract).
#
# Push channel. Populated from the config's `ntfy:` block by
# ExperimentSequencer.__init__ via set_notify_channel(). This wiring used to be
# MISSING: the old _notify wrapper dropped topic/server, and the node never read
# config.ntfy, so a non-empty ntfy.topic was silently ignored and NO operator
# alert (battery halt, circuit breaker, RTK loss) ever left the robot. See the
# "notify topic never wired" entry in ToDo.md.
_NTFY_TOPIC = ""
_NTFY_SERVER = "https://ntfy.sh"


def set_notify_channel(topic, server=None):
    """Wire the ntfy push channel from config (idempotent). Empty topic disables
    push (the safe default)."""
    global _NTFY_TOPIC, _NTFY_SERVER
    _NTFY_TOPIC = str(topic or "")
    if server:
        _NTFY_SERVER = str(server)


try:
    sys.path.insert(0, os.path.join(_REPO_ROOT, "tools", "notify"))
    from ntfy import notify as _ntfy_notify, notify_discord as _discord_notify  # type: ignore

    def _notify(message, title=None, priority=None, tags=None):
        # Fire-and-forget on a daemon thread: each channel does a blocking HTTP
        # POST (~5s) and this runs from the control _tick (incl. the 30s
        # heartbeat), so a synchronous call could stall the state machine on a
        # slow network. Two INDEPENDENT push channels: ntfy.sh (only if a topic is
        # wired) and a Discord webhook (if discord.env is present). The battle-
        # station browser alerts are a third, separate channel (rosbridge-side).
        def _send():
            if _NTFY_TOPIC:
                try:
                    _ntfy_notify(message, title=title, priority=priority,
                                 tags=tags, topic=_NTFY_TOPIC, server=_NTFY_SERVER)
                except Exception:
                    pass
            try:
                _discord_notify(message, title=title)
            except Exception:
                pass

        threading.Thread(target=_send, daemon=True).start()
        return True
except Exception:  # pragma: no cover
    def _notify(message, title=None, priority=None, tags=None):
        return False


# Orchestrator-managed process names (must match orchestrator_node.PROCS).
PROC_REPOSITION = "reposition"
PROC_ODOM_ZERO = "odom_zero"
PROC_FOLLOWER = "follower"

# The single cmd_vel_raw movers (C6 exclusivity set).
MOVERS = (PROC_REPOSITION, PROC_FOLLOWER)

DEFAULT_PREFLIGHT = os.path.join(_REPO_ROOT, "tools", "preflight", "preflight.sh")
DEFAULT_BAG_ROOT = os.path.join(_REPO_ROOT, "Experiment Data")
DEFAULT_CHECKPOINT = os.path.join(_REPO_ROOT, "Experiment Data", "checkpoint.json")
DEFAULT_EXPERIMENT_YAML = os.path.join(_REPO_ROOT, "scenarios", "experiment.yaml")

# Acceptable RTK qualities parsed from /gps_rtk_f9p_helical/gps/rtk_status. FIXED(4)
# is the spec target; FLOAT(5) is a TEMPORARY field acceptance (2026-06-05) because
# the base can't hold FIXED here — FLOAT is ~dm vs cm. Revert to (4,) once FIXED is
# reliable. Keep in sync with tools/preflight/preflight.sh.
RTK_OK_QUALITIES = (4, 5)


class Phase(Enum):
    """Explicit per-cell state machine. Transitions are in _tick()."""

    IDLE = "idle"                  # nothing loaded / batch not started
    PREFLIGHT = "preflight"        # M1 gate (subprocess) before touching actuators
    REPOSITION_START = "repo_start"      # bring up reposition (after killing follower)
    REPOSITION_GOTO = "repo_goto"        # send goto, wait for 'arrived'/'aborted'
    REPOSITION_KILL = "repo_kill"        # tear down reposition before the recorded leg
    ODOM_RESET = "odom_reset"            # latch a fresh (0,0,0) origin (L3)
    BAG_START = "bag_start"              # start the per-leg bag (T7)
    FOLLOWER_START = "follower_start"    # bring up follower (after confirming repo down)
    SET_PARAMS = "set_params"            # controller_type / v_const via set_parameters
    PUSH_RECIPE = "push_recipe"          # publish the analytic recipe (P1-P4)
    RUN = "run"                          # wait done / estop / timeout
    STOP_LEG = "stop_leg"                # stop bag, kill follower, sidecar, classify (D4)
    TURNAROUND = "turnaround"            # U-turn recipe via follower (operational glue)
    NEXT = "next"                        # advance leg / cell / repetition
    PAUSED = "paused"                    # F5 operator pause / F2 RTK-loss / F4 breaker
    DONE = "done"                        # whole batch finished
    ABORTED = "aborted"                  # F5 operator abort -> safe-state, stop


class Leg(Enum):
    A_TO_B = "AtoB"
    TURNAROUND_B = "turnaround_B"
    B_TO_A = "BtoA"
    TURNAROUND_A = "turnaround_A"


class ExperimentSequencer(Node):

    def __init__(self):
        super().__init__("experiment_sequencer_node")

        # -- Parameters -------------------------------------------------
        self.declare_parameter("experiment_yaml", DEFAULT_EXPERIMENT_YAML)
        self.declare_parameter("preflight_path", DEFAULT_PREFLIGHT)
        self.declare_parameter("bag_root", DEFAULT_BAG_ROOT)
        self.declare_parameter("checkpoint_path", DEFAULT_CHECKPOINT)
        # Auto-start the batch on launch. When False the node idles until an
        # /experiment/cmd {"action":"resume"} arrives (lets the battle station
        # arm it deliberately).
        self.declare_parameter("autostart", False)
        # Per-phase wall-time budgets [s]. Conservative; tune on the robot.
        self.declare_parameter("preflight_timeout_s", 90.0)
        self.declare_parameter("reposition_timeout_s", 120.0)
        self.declare_parameter("run_timeout_s", 180.0)
        self.declare_parameter("orchestrator_settle_s", 3.0)
        # RTK persistent-loss wait (F2) before pausing the batch.
        self.declare_parameter("rtk_loss_wait_s", 30.0)
        # Odom-loss wait before pausing: the chassis↔NUC USB (CP2102) drops under
        # vibration -> /wheel/odom goes silent -> the follower would drive on a
        # stale belief. SHORT (odom is ~50 Hz, so 1 s of silence is a real drop)
        # so the robot stops fast; odom_watchdog respawns the chassis driver and
        # we auto-resume. Mirrors the RTK F2 path.
        self.declare_parameter("odom_loss_wait_s", 1.0)
        # Heartbeat interval for status republish + ntfy wallclock (M3).
        self.declare_parameter("heartbeat_s", 30.0)

        self._yaml_path = self.get_parameter("experiment_yaml").value
        self._preflight_path = self.get_parameter("preflight_path").value
        self._bag_root = self.get_parameter("bag_root").value
        self._checkpoint_path = self.get_parameter("checkpoint_path").value
        self._autostart = bool(self.get_parameter("autostart").value)
        self._preflight_timeout = float(self.get_parameter("preflight_timeout_s").value)
        self._reposition_timeout = float(self.get_parameter("reposition_timeout_s").value)
        self._run_timeout = float(self.get_parameter("run_timeout_s").value)
        self._settle_s = float(self.get_parameter("orchestrator_settle_s").value)
        self._rtk_loss_wait = float(self.get_parameter("rtk_loss_wait_s").value)
        self._odom_loss_wait = float(self.get_parameter("odom_loss_wait_s").value)
        self._heartbeat_s = float(self.get_parameter("heartbeat_s").value)

        # -- Load config (experiment.yaml + venue json) ----------------
        self._cfg = self._load_yaml(self._yaml_path)
        self._venue = self._load_venue(self._cfg.get("venue"))
        self.run_id = str(self._cfg.get("run_id", "run"))
        self._gating = self._cfg.get("gating", {}) or {}
        self._retry = self._cfg.get("retry", {}) or {}
        # Fixed step-path geometry override (L1/R/theta_arc/L2/direction) for a
        # small/fixed venue, so the curve fits instead of the follower's 5 m
        # defaults. Empty -> use the cell radius + follower defaults. See smoke.yaml.
        self._path_override = self._cfg.get("path_override", {}) or {}
        self._max_retries = int(self._retry.get("max_retries", 1))
        self._breaker_k = int(self._retry.get("circuit_breaker_k", 3))
        self._batt_warn = float(self._gating.get("battery_volts_warn", 11.0))
        self._batt_halt = float(self._gating.get("battery_volts_halt", 10.5))
        self._rtk_window_pct = float(self._gating.get("rtk_run_window_pct", 95))

        # Wire the operator push channel (T8) from config. Without this the
        # _notify() calls below are no-ops (see set_notify_channel docstring).
        self._ntfy = self._cfg.get("ntfy", {}) or {}
        set_notify_channel(self._ntfy.get("topic", ""), self._ntfy.get("server"))

        # -- Artifact archiving: push each finished bag off the robot (Mac
        #    priority + NAS), detached + rate-limited so it never competes with
        #    the next run. See tools/sync/push_artifact.sh.
        self._art = self._cfg.get("artifact_sync", {}) or {}
        self._art_enabled = bool(self._art.get("enabled", False))
        self._push_artifact_sh = os.path.join(
            _REPO_ROOT, "tools", "sync", "push_artifact.sh")

        # -- Build the cell list (cartesian product x repetitions) ------
        self._cells = self._build_cells(self._cfg)
        self.n_cells = len(self._cells)

        # -- Pin lookups ------------------------------------------------
        self._start_pins = {p["id"]: p for p in (self._venue.get("start_pins") or [])}
        self._end_pins = {p["id"]: p for p in (self._venue.get("end_pins") or [])}
        # Default A/B if present, else first of each list.
        self._pin_A = self._first_pin(self._start_pins, "A")
        self._pin_B = self._first_pin(self._end_pins, "B")

        # -- Checkpoint (F6): set of passed "<cell_id>" we may skip -----
        self._passed = self._load_checkpoint()

        # -- Runtime state ----------------------------------------------
        self.cell_index = 0
        self.leg = Leg.A_TO_B
        self.phase = Phase.IDLE
        self._phase_entered = time.monotonic()
        self._n_pass = 0
        self._n_fail = 0
        self._consecutive_fail = 0
        self._retries_this_leg = 0
        self._pause_reason = None
        self._resume_to = None        # phase to resume into after PAUSED
        self._last_heartbeat = time.monotonic()
        self._batch_start_notified = False

        # Per-leg scratch (reset at BAG_START).
        self._recorder = None
        self._leg_bag_path = None
        self._leg_estopped = False
        self._leg_rtk_fixed_samples = 0
        self._leg_rtk_total_samples = 0
        self._leg_start_utc = None

        # Latest sensor snapshots.
        self._done = False
        self._estop = False
        self._rtk_fixed = False
        self._battery_v = None
        self._orch_status = {}        # name -> bool from /orchestrator/status

        # RTK-loss tracking (F2/F3).
        self._rtk_lost_since = None
        # Odom-loss tracking (base-serial dropout): wall-time of the last
        # /wheel/odom message; staleness = no message for > odom_loss_wait.
        self._last_odom_t = None

        # Odom-zero confirmation (closes the open-loop reset gap from the
        # 2026-05-27 B1 bring-up). _odom_reset_command_t is the ros-time when
        # we last published /odom_zero/reset True; we advance to BAG_START
        # only after /odom_zero/status reports has_reset with a stamp at or
        # after that time. Cleared on entry to ODOM_RESET.
        self._odom_zero_status = None
        self._odom_reset_sent = False
        self._odom_reset_command_t = None

        # -- ROS interfaces ---------------------------------------------
        from rclpy.qos import (
            QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy,
        )
        latched = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        # Latched so a late subscriber (operator `explain-last-failure`, a
        # reconnecting web UI) immediately sees the CURRENT status — including a
        # pause/fail reason. Previously volatile: once paused, status went
        # "dark" to any new subscriber until the 30 s heartbeat (T10 diag gap).
        self.pub_status = self.create_publisher(String, "/experiment/status", latched)
        self.pub_start = self.create_publisher(String, "/orchestrator/start", 10)
        self.pub_kill = self.create_publisher(String, "/orchestrator/kill", 10)
        self.pub_goto = self.create_publisher(String, "/reposition/goto", 10)
        self.pub_odom_reset = self.create_publisher(Bool, "/odom_zero/reset", 10)
        self.pub_recipe = self.create_publisher(String, "/reference_path_recipe", latched)

        self.create_subscription(String, "/experiment/cmd", self._on_cmd, 10)
        self.create_subscription(String, "/orchestrator/status", self._on_orch_status, 10)
        self.create_subscription(String, "/reposition/status", self._on_repo_status, 10)
        self.create_subscription(Bool, "/path_follower/done", self._on_done, latched)
        self.create_subscription(Bool, "/estop", self._on_estop, 10)
        self.create_subscription(
            String, "/gps_rtk_f9p_helical/gps/rtk_status", self._on_rtk, 10)
        # /wheel/odom health (base-serial dropout watchdog). limo_base publishes
        # it RELIABLE, so a default sub matches (unlike the mavros compass).
        self.create_subscription(Odometry, "/wheel/odom", self._on_odom, 10)
        # Odom-zero latch confirmation -- latched so we see the current state on
        # subscribe even if the overlay has been up since before us.
        self.create_subscription(
            String, "/odom_zero/status", self._on_odom_zero_status, latched)
        # /limo_status is limo_msgs/msg/LimoStatus; import defensively so a
        # missing message package degrades to "battery unknown" rather than a
        # construction crash.
        try:
            from limo_msgs.msg import LimoStatus  # type: ignore
            self.create_subscription(LimoStatus, "/limo_status", self._on_limo, 10)
        except Exception:
            self.get_logger().warn(
                "limo_msgs not importable — battery gating (M2) disabled")

        self._repo_state = None  # last /reposition/status 'state'
        self._repo_status = {}   # full last /reposition/status payload
        self._repo_status_raw = ""

        # set_parameters service client for the follower.
        self._param_cli = self.create_client(
            SetParameters, "/path_follower_node/set_parameters")

        # Main 5 Hz tick drives the state machine.
        self.create_timer(0.2, self._tick)

        self.get_logger().info(
            f"Sequencer loaded run_id={self.run_id}: {self.n_cells} cells "
            f"({len(self._passed)} already passed in checkpoint). "
            f"autostart={self._autostart}.")
        self._publish_status(message="loaded")

        if self._autostart:
            self._begin_batch()

    # ==================================================================
    # Config loading
    # ==================================================================

    def _load_yaml(self, path):
        try:
            import yaml
            with open(path, "r", encoding="utf-8") as f:
                return yaml.safe_load(f) or {}
        except Exception as exc:
            self.get_logger().error(f"failed to load experiment yaml {path}: {exc}")
            return {}

    def _load_venue(self, ref):
        if not ref:
            self.get_logger().warn("no venue reference in experiment.yaml")
            return {}
        # Allow a path relative to the repo root.
        path = ref if os.path.isabs(ref) else os.path.join(_REPO_ROOT, ref)
        try:
            with open(path, "r", encoding="utf-8") as f:
                return json.load(f)
        except Exception as exc:
            self.get_logger().error(f"failed to load venue {path}: {exc}")
            return {}

    def _build_cells(self, cfg):
        """Cartesian product of the matrix axes x repetitions.

        Each cell is a dict: {cell_id, controller, v_const, radius_m, rep,
        path_family}. cell_id is deterministic and stable across restarts so
        the checkpoint (F6) can match it.
        """
        matrix = cfg.get("matrix", {}) or {}
        radii = matrix.get("radius_m", [0.5])
        controllers = matrix.get("controller", ["lpv-hinf"])
        speeds = matrix.get("v_const", [1.0])
        families = matrix.get("path_family", ["step"])
        reps = int(cfg.get("repetitions", 1))

        cells = []
        for ctrl, v, R, fam in itertools.product(controllers, speeds, radii, families):
            for rep in range(reps):
                cid = self._cell_id(ctrl, v, R, fam, rep)
                cells.append({
                    "cell_id": cid,
                    "controller": str(ctrl),
                    "v_const": float(v),
                    "radius_m": float(R),
                    "path_family": str(fam),
                    "rep": int(rep),
                })
        return cells

    @staticmethod
    def _cell_id(ctrl, v, R, fam, rep):
        def _num(x):
            return ("%g" % float(x)).replace(".", "p")
        return f"{fam}_R{_num(R)}_v{_num(v)}_{ctrl}_n{int(rep):02d}"

    @staticmethod
    def _first_pin(pins, preferred):
        if preferred in pins:
            return pins[preferred]
        return next(iter(pins.values())) if pins else None

    # ==================================================================
    # Checkpoint (F6)
    # ==================================================================

    def _load_checkpoint(self):
        try:
            with open(self._checkpoint_path, "r", encoding="utf-8") as f:
                data = json.load(f)
            if str(data.get("run_id")) != str(self._cfg.get("run_id", "run")):
                self.get_logger().info(
                    "checkpoint run_id differs from config — ignoring (fresh batch)")
                return set()
            return set(data.get("passed_cells", []))
        except FileNotFoundError:
            return set()
        except Exception as exc:
            self.get_logger().warn(f"checkpoint load failed ({exc}); starting fresh")
            return set()

    def _save_checkpoint(self):
        """Persist passed cells atomically (F6).

        Schema: {"run_id", "updated_utc", "passed_cells":[cell_id,...]}.
        A cell is recorded only after BOTH its forward and return legs pass.
        """
        data = {
            "run_id": self.run_id,
            "updated_utc": datetime.now(timezone.utc).isoformat(),
            "passed_cells": sorted(self._passed),
        }
        try:
            os.makedirs(os.path.dirname(self._checkpoint_path) or ".", exist_ok=True)
            tmp = self._checkpoint_path + ".tmp"
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=2)
                f.write("\n")
            os.replace(tmp, self._checkpoint_path)
        except Exception as exc:
            self.get_logger().error(f"checkpoint save failed: {exc}")

    # ==================================================================
    # Subscriptions
    # ==================================================================

    def _on_cmd(self, msg):
        try:
            d = json.loads(msg.data)
        except (ValueError, TypeError):
            self.get_logger().warn(f"bad /experiment/cmd: {msg.data!r}")
            return
        action = str(d.get("action", "")).lower().strip()
        if action in ("start", "arm"):
            if self.phase == Phase.IDLE:
                self._publish_status(message=f"{action}: starting preflight")
                self._begin_batch()
            else:
                self.get_logger().info(f"{action} ignored — phase={self.phase.value}")
        elif action == "pause":
            self._request_pause("operator pause (F5)")
        elif action == "resume":
            self._resume()
        elif action == "abort":
            self._abort("operator abort (F5)")
        elif action == "skip":
            self._skip_current()
        else:
            self.get_logger().warn(f"unknown /experiment/cmd action: {action!r}")

    def _on_orch_status(self, msg):
        try:
            self._orch_status = json.loads(msg.data) or {}
        except (ValueError, TypeError):
            pass

    def _on_repo_status(self, msg):
        try:
            d = json.loads(msg.data)
            self._repo_status = d if isinstance(d, dict) else {}
            self._repo_status_raw = msg.data or ""
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
        self._rtk_fixed = q in RTK_OK_QUALITIES
        if self.phase == Phase.RUN:
            self._leg_rtk_total_samples += 1
            if self._rtk_fixed:
                self._leg_rtk_fixed_samples += 1
        # Track persistent loss for F2.
        if self._rtk_fixed:
            self._rtk_lost_since = None
        elif self._rtk_lost_since is None:
            self._rtk_lost_since = time.monotonic()

    def _on_odom(self, msg):
        # Base-serial dropout watchdog: just stamp arrival; staleness is judged
        # in _tick. (Presence, not content — a frozen-but-arriving stream is a
        # separate concern handled by the follower's own logic.)
        self._last_odom_t = time.monotonic()

    def _odom_stale(self):
        """True once we've seen odom and it has since gone silent past the wait."""
        return (self._last_odom_t is not None
                and (time.monotonic() - self._last_odom_t) > self._odom_loss_wait)

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
        """Current ROS time as float seconds (matches odom_zero_node's stamp)."""
        return float(self.get_clock().now().nanoseconds) * 1e-9

    # ==================================================================
    # Orchestrator helpers + C6 exclusivity
    # ==================================================================

    def _orch_start(self, name):
        m = String()
        m.data = name
        self.pub_start.publish(m)

    def _orch_kill(self, name):
        m = String()
        m.data = name
        self.pub_kill.publish(m)

    def _is_alive(self, name):
        """Best-effort liveness from the last /orchestrator/status.

        Conservative for the OTHER mover in the C6 check: an unknown name is
        treated as 'not confirmed down' only until status arrives. We gate the
        exclusivity transitions on positive confirmation that the other mover
        is False, so a stale/missing status simply delays the start.
        """
        return bool(self._orch_status.get(name, False))

    def _other_mover(self, name):
        return PROC_FOLLOWER if name == PROC_REPOSITION else PROC_REPOSITION

    def _start_exclusive_mover(self, name):
        """Bring up one cmd_vel_raw mover, guaranteeing C6.

        Returns one of: 'killing_other' (other still up; keep waiting),
        'waiting_other_down' (kill sent, awaiting status confirmation),
        'started' (other confirmed down, start sent).

        The caller re-invokes this each tick until it returns 'started'. We
        NEVER publish /orchestrator/start for `name` until /orchestrator/status
        reports the OTHER mover as down — this is the structural C6 guarantee.
        """
        other = self._other_mover(name)
        if self._is_alive(other):
            # Other mover is up: kill it and wait. Do NOT start `name` yet.
            self._orch_kill(other)
            return "killing_other"
        if other not in self._orch_status:
            # No status seen yet — cannot prove `other` is down. Wait.
            return "waiting_other_down"
        # Other confirmed down. Safe to start `name`.
        if not self._is_alive(name):
            self._orch_start(name)
        return "started"

    # ==================================================================
    # Follower parameterization (controller_type / v_const)
    # ==================================================================

    def _set_follower_params(self, controller_type, v_const):
        """Async set_parameters call; returns the future (or None if no service)."""
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
    # Recipe construction (per leg)
    # ==================================================================

    def _recipe_for_leg(self, cell, leg):
        """Build the JSON recipe string for the given leg.

        Forward/return legs use the cell's path family + radius; turnarounds
        use a U-turn at R_min (operational glue, DOC/experiment.md). The
        follower's build_path_from_recipe owns the exact analytic geometry; we
        only set the family + key params here.
        """
        fam = cell["path_family"]
        R = cell["radius_m"]
        if leg in (Leg.TURNAROUND_B, Leg.TURNAROUND_A):
            # 3-point turn (reverse) is owned by the reposition node and is the
            # fallback when a U-turn does not fit; the default turnaround is a
            # tracked U-turn semicircle via the follower (P3).
            return {"type": "uturn", "params": {}}
        if fam in ("slalom",):
            return {"type": "slalom", "params": {"R": R}}
        # default + "step". A config `path_override` lets a fixed/small venue pin
        # the exact step geometry (L1/R/theta_arc/L2/direction) instead of the
        # follower's 5 m defaults (which overrun a small venue). Single fixed path.
        params = {"R": R}
        params.update(self._path_override)
        return {"type": "step", "params": params}

    # ==================================================================
    # State machine
    # ==================================================================

    def _enter(self, phase):
        if phase != self.phase:
            self.get_logger().info(f"phase {self.phase.value} -> {phase.value}")
            # Per-phase one-shot flags reset on entry.
            if phase == Phase.ODOM_RESET:
                self._odom_reset_sent = False
                self._odom_reset_command_t = None
        self.phase = phase
        self._phase_entered = time.monotonic()

    def _in_phase_s(self):
        return time.monotonic() - self._phase_entered

    def _begin_batch(self):
        if not self._cells:
            self.get_logger().error("no cells to run — check experiment.yaml matrix")
            self._enter(Phase.DONE)
            return
        if not self._batch_start_notified:
            _notify(
                f"Batch '{self.run_id}' starting: {self.n_cells} cells.",
                title="H-inf experiment", tags="rocket")
            self._batch_start_notified = True
        # Skip already-passed cells from the checkpoint.
        self._advance_to_unpassed()
        if self.cell_index >= self.n_cells:
            self._enter(Phase.DONE)
            return
        self.leg = Leg.A_TO_B
        self._retries_this_leg = 0
        self._enter(Phase.PREFLIGHT)

    def _advance_to_unpassed(self):
        while (self.cell_index < self.n_cells
               and self._cells[self.cell_index]["cell_id"] in self._passed):
            self.get_logger().info(
                f"checkpoint: skipping passed cell "
                f"{self._cells[self.cell_index]['cell_id']}")
            self.cell_index += 1

    def _cur_cell(self):
        if 0 <= self.cell_index < self.n_cells:
            return self._cells[self.cell_index]
        return None

    def _tick(self):
        # Heartbeat status republish (M3 / M5). Status-only: it deliberately does
        # NOT push to ntfy/Discord. The heartbeat chimed every heartbeat_s (30s)
        # and desensitised the operator to the alert chime. Push channels now fire
        # only on action-needed events (pause: battery M2 / breaker F4 / RTK F2;
        # batch complete). Live progress is on /experiment/status (browser).
        if time.monotonic() - self._last_heartbeat >= self._heartbeat_s:
            self._last_heartbeat = time.monotonic()
            self._publish_status(message="heartbeat")

        # Global guards that can fire from any active phase.
        if self.phase in (Phase.IDLE, Phase.DONE, Phase.ABORTED, Phase.PAUSED):
            # PAUSED still needs the F2 auto-resume check below.
            if self.phase == Phase.PAUSED:
                self._tick_paused()
            return

        # F4 circuit breaker.
        if self._consecutive_fail >= self._breaker_k:
            self._request_pause(
                f"circuit breaker: {self._consecutive_fail} consecutive failures (F4)")
            _notify(
                f"[{self.run_id}] CIRCUIT BREAKER tripped after "
                f"{self._consecutive_fail} fails — batch paused.",
                title="H-inf HALT", priority="high", tags="warning")
            return

        # M2 battery halt — never start a NEW leg below the halt threshold.
        if (self._battery_v is not None and self._battery_v < self._batt_halt
                and self.phase in (Phase.PREFLIGHT, Phase.NEXT)):
            self._request_pause(
                f"battery {self._battery_v:.2f}V < halt {self._batt_halt}V (M2)")
            _notify(
                f"[{self.run_id}] battery {self._battery_v:.2f}V — halting new runs (M2).",
                title="H-inf battery halt", priority="high", tags="battery")
            return

        # F2 persistent RTK loss: pause if FIXED not held within the wait window
        # while we are in a phase that requires/expects RTK.
        if (self._rtk_lost_since is not None
                and self.phase in (Phase.PREFLIGHT, Phase.REPOSITION_GOTO)
                and (time.monotonic() - self._rtk_lost_since) > self._rtk_loss_wait):
            self._request_pause("persistent RTK-FIXED loss (F2)")
            _notify(
                f"[{self.run_id}] RTK FIXED lost > {self._rtk_loss_wait:.0f}s — "
                "paused; will auto-resume on reacquire.",
                title="H-inf RTK loss", priority="high", tags="satellite")
            return

        # Odom loss (base-serial dropout under vibration): pause FAST so the
        # follower never drives on a stale belief. Fires in any leg-execution
        # phase (PREFLIGHT excluded — preflight.sh gates odom there and a fail
        # retries cleanly). odom_watchdog respawns the chassis driver; we
        # auto-resume below when /wheel/odom returns. RTK/compass stay up (separate
        # USB), so the RTK F2 path does not also fire.
        if (self._odom_stale()
                and self.phase not in (Phase.IDLE, Phase.PREFLIGHT, Phase.NEXT,
                                       Phase.DONE, Phase.ABORTED, Phase.PAUSED)):
            self._request_pause("odom loss (base serial)")
            _notify(
                f"[{self.run_id}] /wheel/odom lost > {self._odom_loss_wait:.1f}s — "
                "paused; odom_watchdog recovering, will auto-resume.",
                title="H-inf odom loss", priority="high", tags="warning")
            return

        # Dispatch on phase.
        handler = getattr(self, f"_tick_{self.phase.value}", None)
        if handler is not None:
            handler()
        else:
            self.get_logger().error(f"no handler for phase {self.phase.value}")
            self._enter(Phase.PAUSED)

    # -- Per-phase handlers --------------------------------------------

    def _tick_preflight(self):
        cell = self._cur_cell()
        if cell is None:
            self._enter(Phase.DONE)
            return
        # Run the preflight script once (subprocess). Exit 0 = pass.
        if not hasattr(self, "_preflight_proc") or self._preflight_proc is None:
            try:
                self._preflight_proc = subprocess.Popen(
                    ["bash", self._preflight_path],
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                    text=True, errors="replace")
            except Exception as exc:
                self.get_logger().error(f"preflight launch failed: {exc}")
                self._preflight_proc = None
                self._fail_leg("preflight launch failed")
                return
        rc = self._preflight_proc.poll()
        if rc is None:
            if self._in_phase_s() > self._preflight_timeout:
                try:
                    self._preflight_proc.kill()
                    out, _ = self._preflight_proc.communicate(timeout=1.0)
                except Exception:
                    out = ""
                self._preflight_proc = None
                detail = self._summarize_preflight_output(out)
                self._fail_leg(f"preflight timeout{detail}")
            return
        # Finished.
        try:
            out, _ = self._preflight_proc.communicate(timeout=1.0)
        except Exception:
            out = ""
        self._preflight_proc = None
        if rc == 0:
            self._publish_status(message="preflight pass")
            self._enter(Phase.REPOSITION_START)
        else:
            detail = self._summarize_preflight_output(out)
            self._fail_leg(f"preflight FAIL (exit {rc}){detail}")

    def _tick_repo_start(self):
        # R4 no-op: if the pin we need to be at equals the prior end pin, skip
        # reposition entirely (handled by comparing leg target to current).
        if self._reposition_is_noop():
            self.get_logger().info("reposition no-op (start == prior end) — skipping (R4)")
            self._enter(Phase.ODOM_RESET)
            return
        # C6: bring up reposition only after follower is confirmed down.
        res = self._start_exclusive_mover(PROC_REPOSITION)
        if res == "started":
            self._repo_state = None
            self._repo_status = {}
            self._repo_status_raw = ""
            self._enter(Phase.REPOSITION_GOTO)
        elif self._in_phase_s() > self._reposition_timeout:
            self._fail_leg("reposition start timeout")

    def _tick_repo_goto(self):
        # Send the goto once reposition is confirmed up, then poll status.
        if not self._is_alive(PROC_REPOSITION):
            if self._in_phase_s() > self._settle_s:
                self._fail_leg("reposition node not alive")
            return
        if self._in_phase_s() < self._settle_s and self._repo_state is None:
            # Let the node settle then send the goal once.
            return
        if not getattr(self, "_goto_sent", False):
            pin = self._target_pin_for_leg()
            if pin is None:
                self._fail_leg("no target pin for leg")
                return
            m = String()
            m.data = json.dumps({
                "lat": pin["lat"], "lon": pin["lon"],
                "heading_deg": pin.get("heading_deg", 0.0),
            })
            self.pub_goto.publish(m)
            self._goto_sent = True
            return
        if self._repo_state == "arrived":
            self._goto_sent = False
            self._enter(Phase.REPOSITION_KILL)
        elif self._repo_state == "aborted":
            self._goto_sent = False
            self._fail_leg(self._reposition_abort_summary())
        elif self._in_phase_s() > self._reposition_timeout:
            self._goto_sent = False
            self._fail_leg("reposition goto timeout")

    def _tick_repo_kill(self):
        # Tear down reposition BEFORE any recorded leg (C6: follower comes next).
        if self._is_alive(PROC_REPOSITION):
            self._orch_kill(PROC_REPOSITION)
            return
        if PROC_REPOSITION not in self._orch_status:
            return  # await status confirmation
        self._enter(Phase.ODOM_RESET)

    def _tick_odom_reset(self):
        # L3: latch a fresh (0,0,0) origin. odom_zero must be up.
        if not self._is_alive(PROC_ODOM_ZERO):
            self._orch_start(PROC_ODOM_ZERO)
            if self._in_phase_s() > self._reposition_timeout:
                self._fail_leg("odom_zero failed to start")
            return

        # Send the reset True once on first tick of this phase entry. Closing
        # the loop on /odom_zero/status (below) means we no longer spam-publish
        # True every tick -- which previously could re-latch the origin on a
        # creeping robot.
        if not self._odom_reset_sent:
            self._odom_reset_command_t = self._ros_now_s()
            m = Bool()
            m.data = True
            self.pub_odom_reset.publish(m)
            self._odom_reset_sent = True
            return

        # Wait for /odom_zero/status to confirm a latch AT OR AFTER our command.
        # The stamp field is ros-time seconds matching _ros_now_s(); requiring
        # the stamp to be >= command time is what makes this closed-loop --
        # the latched topic's seed message (stamp=null, pre-reset) and any
        # stale prior-leg latch (stamp < command_t) both fail the check.
        s = self._odom_zero_status or {}
        if (s.get("has_reset") is True
                and s.get("stamp") is not None
                and float(s["stamp"]) >= float(self._odom_reset_command_t)):
            origin = s.get("origin") or {}
            self.get_logger().info(
                f"odom_zero latch confirmed at "
                f"(x={origin.get('x'):.3f}, y={origin.get('y'):.3f}, "
                f"yaw={origin.get('yaw'):.4f} rad).")
            self._enter(Phase.BAG_START)
            return

        # Previous behaviour: advance after settle_s regardless. That masked a
        # dropped-reset failure mode (overlay stays at identity offset, follower
        # lunges to origin). Now: fail the leg if no confirmation in settle_s,
        # let O4 auto-retry redo it from PREFLIGHT.
        if self._in_phase_s() > self._settle_s:
            self._fail_leg(
                f"odom_zero reset not confirmed within {self._settle_s:.1f}s "
                "(no /odom_zero/status latch with stamp >= command time)")

    def _tick_bag_start(self):
        cell = self._cur_cell()
        self._reset_leg_scratch()
        if Data_Logger is None:
            self.get_logger().error("Data_Logger unavailable — cannot record; failing leg")
            self._fail_leg("recorder unavailable")
            return
        leg_name = self.leg.value
        try:
            dirname = Data_Logger.build_leg_dirname(self.run_id, cell["cell_id"], leg_name)
            self._leg_bag_path = os.path.join(self._bag_root, dirname)
            self._recorder = Data_Logger.BagRecorder(
                self._leg_bag_path, topics=Data_Logger.TOPICS)
            self._recorder.start()
            self._leg_start_utc = datetime.now(timezone.utc).isoformat()
        except Exception as exc:
            self.get_logger().error(f"bag start failed: {exc}")
            self._recorder = None
            self._fail_leg("bag start failed")
            return
        self._enter(Phase.FOLLOWER_START)

    def _tick_follower_start(self):
        # C6: bring up follower only after reposition is confirmed down.
        res = self._start_exclusive_mover(PROC_FOLLOWER)
        if res == "started":
            self._enter(Phase.SET_PARAMS)
        elif self._in_phase_s() > self._reposition_timeout:
            self._fail_leg("follower start timeout")

    def _tick_set_params(self):
        cell = self._cur_cell()
        if not self._is_alive(PROC_FOLLOWER):
            if self._in_phase_s() > self._settle_s:
                self._fail_leg("follower not alive for set_params")
            return
        # Wait for the parameter service, then fire once.
        if not getattr(self, "_params_sent", False):
            fut = self._set_follower_params(cell["controller"], cell["v_const"])
            if fut is None:
                # Service not up yet; keep waiting within the budget.
                if self._in_phase_s() > self._reposition_timeout:
                    self._fail_leg("follower set_parameters service never ready")
                return
            self._params_future = fut
            self._params_sent = True
            return
        if self._params_future.done():
            self._params_sent = False
            self._enter(Phase.PUSH_RECIPE)

    def _tick_push_recipe(self):
        cell = self._cur_cell()
        recipe = self._recipe_for_leg(cell, self.leg)
        self._cur_recipe = recipe
        m = String()
        m.data = json.dumps(recipe)
        self.pub_recipe.publish(m)
        self._done = False
        self._enter(Phase.RUN)

    def _tick_run(self):
        # Terminate on: done edge, estop, or timeout.
        if self._estop or self._leg_estopped:
            self._end_run("estop during run")
            return
        if self._done:
            self._end_run("done")
            return
        if self._in_phase_s() > self._run_timeout:
            self._end_run("run timeout")
            return

    def _end_run(self, reason):
        self._run_end_reason = reason
        self._enter(Phase.STOP_LEG)

    def _tick_stop_leg(self):
        cell = self._cur_cell()
        # 1) stop bag.
        info = {}
        if self._recorder is not None:
            try:
                info = self._recorder.stop(timeout_s=10.0)
            except Exception as exc:
                self.get_logger().warn(f"bag stop error: {exc}")
            self._recorder = None
        # 2) kill follower (release cmd_vel_raw before anything else moves, C6).
        self._orch_kill(PROC_FOLLOWER)

        # 3) classify (D4).
        classification = self._classify(cell, info)
        # 4) sidecar.
        self._write_sidecar(cell, info, classification)

        passed = bool(classification.get("pass"))
        is_turn = self.leg in (Leg.TURNAROUND_B, Leg.TURNAROUND_A)
        if passed:
            if not is_turn:
                self._n_pass += 1
            self._consecutive_fail = 0
        else:
            if not is_turn:
                self._n_fail += 1
            self._consecutive_fail += 1

        self.get_logger().info(
            f"leg {self.leg.value} cell {cell['cell_id']}: "
            f"{'PASS' if passed else 'FAIL'} ({classification.get('reason')})")
        self._publish_status(message=f"{self.leg.value} {'pass' if passed else 'fail'}")

        if not passed:
            _notify(
                f"[{self.run_id}] FAIL {cell['cell_id']} {self.leg.value}: "
                f"{classification.get('reason')}",
                title="H-inf run failed", priority="high", tags="x")
            # Retry/skip handled in NEXT-style logic here.
            self._handle_leg_failure()
            return

        # Passed -> advance to the next leg in the cycle.
        self._advance_leg_after_pass()

    def _tick_turnaround(self):
        # Turnaround is a recorded+classified leg too; it is driven through the
        # same record/run pipeline. We re-enter at BAG_START with leg already
        # set to a TURNAROUND_* value (no reposition for a U-turn). odom is
        # re-zeroed first so the U-turn starts at (0,0,0).
        self._enter(Phase.ODOM_RESET)

    def _tick_next(self):
        # Advance repetition/cell and loop or finish.
        self._advance_to_unpassed()
        if self.cell_index >= self.n_cells:
            self._enter(Phase.DONE)
            return
        self.leg = Leg.A_TO_B
        self._retries_this_leg = 0
        self._enter(Phase.PREFLIGHT)

    def _tick_done(self):
        # Idempotent terminal state.
        if not getattr(self, "_done_notified", False):
            self._done_notified = True
            self._safe_state()
            _notify(
                f"[{self.run_id}] batch COMPLETE: pass={self._n_pass} fail={self._n_fail} "
                f"of {self.n_cells} cells.",
                title="H-inf done", tags="checkered_flag")
            self._publish_status(message="batch complete")

    def _tick_aborted(self):
        pass  # terminal; safe-state already commanded on entry.

    def _tick_paused(self):
        # F2 auto-resume: if paused for RTK loss and FIXED returns, resume.
        if self._pause_reason and "RTK" in self._pause_reason and self._rtk_fixed:
            self.get_logger().info("RTK FIXED reacquired — auto-resuming (F2)")
            _notify(f"[{self.run_id}] RTK reacquired — resuming.",
                    title="H-inf resume", tags="satellite")
            self._resume()
            return
        # Odom auto-resume: paused for a base-serial drop and /wheel/odom is back
        # (odom_watchdog respawned the chassis driver). Re-does the leg cleanly.
        if (self._pause_reason and "odom" in self._pause_reason
                and self._last_odom_t is not None and not self._odom_stale()):
            self.get_logger().info("odom recovered — auto-resuming")
            _notify(f"[{self.run_id}] /wheel/odom recovered — resuming.",
                    title="H-inf resume", tags="white_check_mark")
            self._resume()

    # ==================================================================
    # Leg outcome handling: classify, retry, advance
    # ==================================================================

    def _advance_leg_after_pass(self):
        """Cycle order: A->B, turnaround@B, B->A, turnaround@A, next rep/cell."""
        cell = self._cur_cell()
        self._retries_this_leg = 0
        if self.leg == Leg.A_TO_B:
            self.leg = Leg.TURNAROUND_B
            self._enter(Phase.TURNAROUND)
        elif self.leg == Leg.TURNAROUND_B:
            # Return leg starts from B; reposition to B-start may be a no-op.
            self.leg = Leg.B_TO_A
            self._enter(Phase.REPOSITION_START)
        elif self.leg == Leg.B_TO_A:
            # Both headline legs of this cell passed -> checkpoint it (F6).
            self._passed.add(cell["cell_id"])
            self._save_checkpoint()
            self.leg = Leg.TURNAROUND_A
            self._enter(Phase.TURNAROUND)
        elif self.leg == Leg.TURNAROUND_A:
            self.cell_index += 1
            self._enter(Phase.NEXT)

    def _handle_leg_failure(self):
        """O4 auto-retry; on exhaustion, skip the cell (turnarounds: skip leg)."""
        is_turn = self.leg in (Leg.TURNAROUND_B, Leg.TURNAROUND_A)
        if self._retries_this_leg < self._max_retries:
            self._retries_this_leg += 1
            self.get_logger().warn(
                f"retry {self._retries_this_leg}/{self._max_retries} of leg "
                f"{self.leg.value}")
            # Redo from preflight (partially-recorded leg already discarded by
            # being classified fail; F6 only checkpoints passing cells).
            self._enter(Phase.PREFLIGHT)
            return
        # Retries exhausted.
        if is_turn:
            # A failed turnaround pings + pauses (likely pose/area issue).
            _notify(
                f"[{self.run_id}] turnaround failed after retries — pausing.",
                title="H-inf turnaround fail", priority="high", tags="warning")
            self._request_pause("turnaround failed after retries")
            return
        # Headline leg: skip the whole cell and continue (O4).
        self.get_logger().warn(
            f"cell {self._cur_cell()['cell_id']} skipped after exhausting retries")
        self.cell_index += 1
        self._enter(Phase.NEXT)

    def _fail_leg(self, reason):
        """Failure BEFORE/at recording boundary (preflight, reposition, setup).

        No bag to classify; treat as a failed leg and route through retry/skip.
        Always release any mover first (safe-state, C6).
        """
        self.get_logger().warn(f"leg fail: {reason}")
        self._publish_status(message=f"leg fail: {reason}")
        # Stop any in-flight bag so we don't leak a partial recording.
        if self._recorder is not None:
            try:
                self._recorder.stop(timeout_s=5.0)
            except Exception:
                pass
            self._recorder = None
        self._safe_state()
        self._goto_sent = False
        self._params_sent = False
        if self.leg not in (Leg.TURNAROUND_B, Leg.TURNAROUND_A):
            self._n_fail += 1
        self._consecutive_fail += 1
        _notify(
            f"[{self.run_id}] leg fail ({reason}) cell "
            f"{self._cur_cell()['cell_id'] if self._cur_cell() else '?'} {self.leg.value}",
            title="H-inf leg fail", tags="x")
        self._handle_leg_failure()

    def _summarize_preflight_output(self, output, max_chars=600):
        """Return a compact failure detail from preflight stdout/stderr."""
        text = str(output or "")
        lines = [ln.strip() for ln in text.splitlines() if ln.strip()]
        if not lines:
            return ""
        interesting = [
            ln for ln in lines
            if any(tok in ln.lower() for tok in (
                "fail", "failed", "error", "warn", "missing", "stale", "not fixed",
                "rtk", "estop", "exclusive", "skipped",
            ))
        ]
        chosen = interesting[-6:] if interesting else lines[-6:]
        summary = " | ".join(chosen)
        if len(summary) > max_chars:
            summary = summary[-max_chars:]
        return f": {summary}"

    def _reposition_abort_summary(self):
        """Preserve the exact reposition-node abort reason in experiment status."""
        status = self._repo_status if isinstance(self._repo_status, dict) else {}
        reason = status.get("reason") or status.get("message") or "aborted"
        parts = [f"reposition aborted: {reason}"]
        for key in ("err_m", "err_deg"):
            value = status.get(key)
            if value is None:
                continue
            try:
                parts.append(f"{key}={float(value):.2f}")
            except (TypeError, ValueError):
                parts.append(f"{key}={value}")
        return " ".join(parts)

    def _classify(self, cell, bag_info):
        """D4 per-run pass/fail (system_spec §5 / experiment.md).

        Pass iff ALL of:
          - reached path end (the /path_follower/done edge fired)
          - no E-stop during the run
          - bag exists with a non-trivial duration
          - bag wallclock ~= arc_length / v within tolerance (best-effort;
            arc length not known here without the path object, so we only
            assert a positive duration + an upper sanity bound = run_timeout)
          - RTK FIXED for >= rtk_run_window_pct of the run window (F3)
        Else fail. Returns a dict embedded in the sidecar.
        """
        reasons = []
        passed = True

        reached = bool(self._done) and self._run_end_reason == "done"
        if not reached:
            passed = False
            reasons.append(f"did not reach end (end_reason={self._run_end_reason})")

        if self._leg_estopped or self._estop:
            passed = False
            reasons.append("estop during run")

        dur = float(bag_info.get("duration_s", 0.0) or 0.0)
        if dur <= 0.1:
            passed = False
            reasons.append("bag has no/short duration")

        # RTK window coverage (F3).
        if self._leg_rtk_total_samples > 0:
            pct = 100.0 * self._leg_rtk_fixed_samples / self._leg_rtk_total_samples
        else:
            pct = 0.0
        if pct < self._rtk_window_pct:
            passed = False
            reasons.append(
                f"RTK FIXED {pct:.1f}% < {self._rtk_window_pct:.0f}% window (F3)")

        return {
            "pass": passed,
            "reason": "ok" if passed else "; ".join(reasons),
            "reached_end": reached,
            "estop": bool(self._leg_estopped or self._estop),
            "duration_s": dur,
            "rtk_fixed_pct": round(pct, 1),
            "rtk_window_pct_required": self._rtk_window_pct,
            "end_reason": self._run_end_reason,
        }

    def _write_sidecar(self, cell, bag_info, classification):
        if Data_Logger is None or not self._leg_bag_path:
            return
        try:
            rtk_summary = {
                "fixed_pct": classification.get("rtk_fixed_pct"),
                "fixed_samples": self._leg_rtk_fixed_samples,
                "total_samples": self._leg_rtk_total_samples,
                # Accepted RTK qualities for this run (FIXED=4, FLOAT=5 TEMP).
                # Was RTK_FIXED_TOKEN, removed in the {4,5} rename -> NameError
                # that silently killed every sidecar write.
                "ok_qualities": list(RTK_OK_QUALITIES),
            }
            wallclock = {
                "start_utc": self._leg_start_utc,
                "end_utc": bag_info.get("end_utc"),
                "duration_s": bag_info.get("duration_s"),
            }
            # Per-leg path-frame anchor: the start pin where odom was zeroed.
            # A_TO_B starts at pin A; B_TO_A starts at pin B. Turnarounds are
            # operational glue (anchor undefined here) -> leave None and let
            # run_eval fall back to venue-local with a warning.
            anchor_pin = None
            if self.leg == Leg.A_TO_B:
                anchor_pin = self._pin_A
            elif self.leg == Leg.B_TO_A:
                anchor_pin = self._pin_B
            path_frame_anchor = None
            if anchor_pin is not None:
                path_frame_anchor = {
                    "pin_id": anchor_pin.get("id"),
                    "lat": float(anchor_pin["lat"]),
                    "lon": float(anchor_pin["lon"]),
                    "heading_deg": float(anchor_pin.get("heading_deg", 0.0)),
                }
            sidecar = Data_Logger.build_sidecar(
                run_id=self.run_id,
                cell_id=cell["cell_id"],
                leg=self.leg.value,
                cell_params={
                    "controller": cell["controller"],
                    "v_const": cell["v_const"],
                    "radius_m": cell["radius_m"],
                    "path_family": cell["path_family"],
                    "rep": cell["rep"],
                },
                path_recipe=getattr(self, "_cur_recipe", {}),
                venue_id=self._venue.get("name"),
                start_pin_id=(self._pin_A or {}).get("id"),
                end_pin_id=(self._pin_B or {}).get("id"),
                rtk_summary=rtk_summary,
                classification=classification,
                wallclock=wallclock,
                controller_tuning={
                    "controller_type": cell["controller"],
                    "v_const": cell["v_const"],
                },
                bag_path=self._leg_bag_path,
                topics=Data_Logger.TOPICS,
                path_frame_anchor=path_frame_anchor,
            )
            Data_Logger.write_sidecar(self._leg_bag_path, sidecar)
        except Exception as exc:
            self.get_logger().error(f"sidecar write failed: {exc}")
        # Bag + sidecar are finalized -> archive off the robot (non-blocking).
        self._archive_leg(self._leg_bag_path)

    def _archive_leg(self, bag_path):
        """Fire-and-forget push of one finished bag off the robot (Mac priority +
        NAS) via tools/sync/push_artifact.sh, fully detached + rate-limited so it
        does NOT compete with the next run. A copy failure never affects the
        batch. Granularity is per-run today (the tested, default path); the
        bwlimit/detach keep it non-interfering. Fall back to per-cell/batch in
        experiment.yaml only if a run ever shows contention."""
        if not (self._art_enabled and bag_path):
            return
        if not os.path.isfile(self._push_artifact_sh):
            self.get_logger().warn(
                f"artifact_sync enabled but missing {self._push_artifact_sh}; skip")
            return
        env = dict(os.environ)
        if self._art.get("mac_target"):
            env["ARTIFACT_MAC_TARGET"] = str(self._art["mac_target"])
        if self._art.get("nas_target"):
            env["ARTIFACT_NAS_TARGET"] = str(self._art["nas_target"])
        env["ARTIFACT_BWLIMIT_KBPS"] = str(self._art.get("bwlimit_kbps", 0))
        if self._art.get("local_repo"):
            env["ARTIFACT_LOCAL_REPO"] = str(self._art["local_repo"])
        try:
            subprocess.Popen(
                ["bash", self._push_artifact_sh, bag_path],
                env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                start_new_session=True,  # detach: own session, won't block the loop
            )
            self.get_logger().info(
                f"artifact_sync: dispatched push for {os.path.basename(bag_path)}")
        except Exception as exc:
            self.get_logger().warn(f"artifact_sync dispatch failed (non-fatal): {exc}")

    # ==================================================================
    # Reposition target / no-op (R4)
    # ==================================================================

    def _target_pin_for_leg(self):
        # Forward leg drives to start pin A; return leg drives to end pin B.
        if self.leg == Leg.A_TO_B:
            return self._pin_A
        if self.leg == Leg.B_TO_A:
            return self._pin_B
        return None

    def _reposition_is_noop(self):
        """R4: no-op when the next start pin equals the prior end pin.

        With a single A/B pin pair the forward leg always needs A and the
        return leg always needs B, so we only short-circuit when the same pin
        object would be re-targeted back-to-back. Conservative: only declares a
        no-op when A and B coincide (degenerate venue) — never skips a real move.
        """
        pin = self._target_pin_for_leg()
        if pin is None:
            return False
        a, b = self._pin_A, self._pin_B
        if a and b and a is not b:
            return False
        # A == B (or one missing) -> any reposition is a no-op.
        return True

    # ==================================================================
    # Pause / resume / abort / skip / safe-state
    # ==================================================================

    def _request_pause(self, reason):
        if self.phase in (Phase.PAUSED, Phase.ABORTED, Phase.DONE):
            return
        self._pause_reason = reason
        self._resume_to = Phase.PREFLIGHT  # always re-do the current leg cleanly
        self.get_logger().warn(f"PAUSE: {reason}")
        # Discard any partial recording (F6) and release movers (F1 safe-state).
        if self._recorder is not None:
            try:
                self._recorder.stop(timeout_s=5.0)
            except Exception:
                pass
            self._recorder = None
        self._safe_state()
        self._enter(Phase.PAUSED)
        self._publish_status(message=f"paused: {reason}")

    def _resume(self):
        if self.phase == Phase.IDLE:
            self._publish_status(message="resume from idle: starting preflight")
            self._begin_batch()
            return
        if self.phase != Phase.PAUSED:
            self.get_logger().info("resume ignored — not paused")
            return
        # Reset the breaker on a deliberate resume so the operator can clear it.
        self._consecutive_fail = 0
        self._rtk_lost_since = None
        self._pause_reason = None
        if self.cell_index >= self.n_cells:
            self._enter(Phase.DONE)
            return
        self._retries_this_leg = 0
        self._enter(self._resume_to or Phase.PREFLIGHT)
        self._publish_status(message="resumed")

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
        _notify(f"[{self.run_id}] ABORTED: {reason}", title="H-inf aborted",
                priority="high", tags="octagonal_sign")
        self._publish_status(message=f"aborted: {reason}")

    def _skip_current(self):
        """F8/operator skip: discard the current cell, advance to the next."""
        self.get_logger().warn("SKIP: current cell skipped by operator")
        if self._recorder is not None:
            try:
                self._recorder.stop(timeout_s=5.0)
            except Exception:
                pass
            self._recorder = None
        self._safe_state()
        self.cell_index += 1
        self._retries_this_leg = 0
        if self.phase == Phase.PAUSED:
            # Stay paused but advance the pointer; operator resumes when ready.
            self._resume_to = Phase.PREFLIGHT
        else:
            self._enter(Phase.NEXT)
        self._publish_status(message="skipped current cell")

    def _safe_state(self):
        """F1 stop-in-place: release BOTH cmd_vel_raw movers.

        Killing the follower/reposition drops the cmd_vel_raw source; estop_cli
        then lets /cmd_vel go silent (zero) and the base holds. We do NOT issue
        any new motion here.
        """
        for name in MOVERS:
            self._orch_kill(name)
        self._goto_sent = False
        self._params_sent = False

    # ==================================================================
    # Status + scratch
    # ==================================================================

    def _reset_leg_scratch(self):
        self._leg_bag_path = None
        self._leg_estopped = False
        self._leg_rtk_fixed_samples = 0
        self._leg_rtk_total_samples = 0
        self._leg_start_utc = None
        self._done = False
        self._run_end_reason = None

    def _eta_s(self):
        """Crude ETA: remaining cells x a nominal per-cell wall budget."""
        remaining = max(0, self.n_cells - self.cell_index)
        per_cell = 2 * (self._reposition_timeout * 0.4 + self._run_timeout * 0.6)
        return round(remaining * per_cell, 1)

    def _publish_status(self, message=""):
        cell = self._cur_cell()
        cmd_owner = None
        if self.phase in (Phase.REPOSITION_START, Phase.REPOSITION_GOTO, Phase.REPOSITION_KILL):
            cmd_owner = "reposition"
        elif self.phase in (Phase.FOLLOWER_START, Phase.RUN):
            cmd_owner = "follower"
        payload = {
            "run_id": self.run_id,
            "cell_id": cell["cell_id"] if cell else None,
            "cell_index": self.cell_index,
            "n_cells": self.n_cells,
            "leg": self.leg.value,
            "phase": self.phase.value,
            "pass": self._n_pass,
            "fail": self._n_fail,
            "eta_s": self._eta_s(),
            "cmd_owner": cmd_owner,
            "message": message,
        }
        m = String()
        m.data = json.dumps(payload)
        self.pub_status.publish(m)

    def shutdown(self):
        self.get_logger().info("sequencer shutting down — safe-state")
        try:
            self._safe_state()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = ExperimentSequencer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # ExternalShutdownException: rclpy already tore the context down from a
        # SIGINT/SIGTERM handler. Swallow it and skip the double-shutdown below
        # (that raised "rcl_shutdown already called" on every clean kill).
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
