#!/usr/bin/env python3
"""Fault-injection sim harness for experiment_sequencer_node (ros-sim tier).

Runs on the sourced NUC; the test module guards with ``pytest.importorskip`` so
the dev laptop SKIPs cleanly. This is the Tier-B harness from the unattended-
operation test plan: it exercises the sequencer's *recovery brain* (preflight
gate, C6 mover-exclusivity, reposition abort, RTK-loss pause/auto-resume,
circuit breaker, run timeout) without any hardware.

SAFETY — nothing here can move the robot:
  * The real movers (follower / reposition / base drivers / estop) are NEVER
    launched. A single ``MockWorld`` node impersonates them purely over the
    documented topic contracts.
  * The real ``experiment_sequencer_node`` runs as a subprocess on an ISOLATED
    ``ROS_DOMAIN_ID`` (forced by the test fixture). Its ``/orchestrator/start``
    commands therefore cannot reach a real orchestrator on the default domain.
  * Per-leg bags it records go to a pytest ``tmp_path`` and are torn down with
    the sequencer's own process group on stop().

Contract surface (mirrors experiment_sequencer_node.py):
  MockWorld PUBLISHES (sequencer subscribes):
    /orchestrator/status      String JSON {name: bool}
    /reposition/status        String JSON {state, reason, err_m, err_deg}
    /odom_zero/status         String JSON {has_reset, stamp, origin} (latched)
    /gps_rtk_f9p_helical/gps/rtk_status  String  ("quality=N ...")
    /estop                    Bool
    /path_follower/done       Bool (latched)
    /limo_status              limo_msgs/LimoStatus (battery_voltage)
    /experiment/cmd           String JSON {action}     (used to arm)
  MockWorld SUBSCRIBES (sequencer publishes):
    /orchestrator/start, /orchestrator/kill   String
    /reposition/goto          String
    /odom_zero/reset          Bool
    /reference_path_recipe    String
    /experiment/status        String JSON   (observed for assertions)

A second tiny node literally named ``path_follower_node`` exists only to host
the auto-created ``/path_follower_node/set_parameters`` service the sequencer's
SET_PARAMS phase calls.
"""

from __future__ import annotations

import json
import os
import signal
import subprocess
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from std_msgs.msg import Bool, String

try:
    from limo_msgs.msg import LimoStatus  # type: ignore

    HAVE_LIMO = True
except Exception:  # pragma: no cover - exercised only without limo_msgs
    LimoStatus = None
    HAVE_LIMO = False


# Orchestrator-managed names the sequencer drives (must match its PROCS subset).
MANAGED = ("reposition", "odom_zero", "follower")


def _latched(depth: int = 1) -> QoSProfile:
    return QoSProfile(
        depth=depth,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )


class MockWorld(Node):
    """Impersonates orchestrator + reposition + odom_zero + RTK + estop + battery.

    Scenario knobs are plain attributes; a test sets them before or during the
    run and the 10 Hz tick reflects them. All shared state is guarded by a lock
    because callbacks and the test thread both touch it.
    """

    def __init__(self) -> None:
        super().__init__("sim_world")
        self._lock = threading.Lock()
        self._t0 = time.monotonic()

        # --- injectable scenario knobs -------------------------------
        self.rtk_quality = 4            # 4 == FIXED; anything else == not fixed
        self.battery_v = 11.7
        self.repo_result = "arrived"    # "arrived" | "aborted"
        self.repo_reason = ""
        self.repo_err_m = 0.05
        self.repo_err_deg = 1.0
        self.repo_delay = 0.3
        self.run_outcome = "done"       # "done" | "timeout" | "estop"
        self.done_delay = 0.3

        # --- observed state ------------------------------------------
        self.alive = {n: False for n in MANAGED}
        self.estop_state = False
        self.status_log = []            # list[(t, dict)]
        self.start_events = []          # list[(t, name)]
        self.kill_events = []           # list[(t, name)]
        self.goto_events = []           # list[(t, raw)]
        self.reset_events = []          # list[t]
        self.recipe_events = []         # list[(t, raw)]
        self.alive_timeline = []        # list[(t, follower_alive, repo_alive)]
        self._due = []                  # list[(due_monotonic, fn)]
        self._limo_div = 0

        # --- publishers ----------------------------------------------
        self.pub_orch = self.create_publisher(String, "/orchestrator/status", 10)
        self.pub_repo = self.create_publisher(String, "/reposition/status", 10)
        self.pub_oz = self.create_publisher(String, "/odom_zero/status", _latched())
        self.pub_rtk = self.create_publisher(
            String, "/gps_rtk_f9p_helical/gps/rtk_status", 10)
        self.pub_estop = self.create_publisher(Bool, "/estop", 10)
        self.pub_done = self.create_publisher(Bool, "/path_follower/done", _latched())
        self.pub_cmd = self.create_publisher(String, "/experiment/cmd", 10)
        self.pub_limo = (
            self.create_publisher(LimoStatus, "/limo_status", 10) if HAVE_LIMO else None
        )

        # --- subscriptions -------------------------------------------
        self.create_subscription(String, "/orchestrator/start", self._on_start, 10)
        self.create_subscription(String, "/orchestrator/kill", self._on_kill, 10)
        self.create_subscription(String, "/reposition/goto", self._on_goto, 10)
        self.create_subscription(Bool, "/odom_zero/reset", self._on_reset, 10)
        self.create_subscription(String, "/reference_path_recipe", self._on_recipe, 10)
        self.create_subscription(String, "/experiment/status", self._on_status, 10)

        self.create_timer(0.1, self._tick)

    # -- time helpers --------------------------------------------------
    def _now(self) -> float:
        return time.monotonic() - self._t0

    def _ros_now_s(self) -> float:
        return float(self.get_clock().now().nanoseconds) * 1e-9

    def _schedule(self, delay: float, fn) -> None:
        with self._lock:
            self._due.append((time.monotonic() + delay, fn))

    # -- subscription callbacks ---------------------------------------
    def _on_start(self, msg: String) -> None:
        name = msg.data.strip()
        with self._lock:
            self.start_events.append((self._now(), name))
            if name in self.alive:
                self.alive[name] = True

    def _on_kill(self, msg: String) -> None:
        name = msg.data.strip()
        with self._lock:
            self.kill_events.append((self._now(), name))
            if name in self.alive:
                self.alive[name] = False

    def _on_goto(self, msg: String) -> None:
        with self._lock:
            self.goto_events.append((self._now(), msg.data))
            args = (self.repo_result, self.repo_reason, self.repo_err_m,
                    self.repo_err_deg, self.repo_delay)
        result, reason, em, ed, delay = args
        self._schedule(delay, lambda: self._emit_repo(result, reason, em, ed))

    def _emit_repo(self, result, reason, em, ed) -> None:
        m = String()
        m.data = json.dumps(
            {"state": result, "reason": reason, "err_m": em, "err_deg": ed})
        self.pub_repo.publish(m)

    def _on_reset(self, msg: Bool) -> None:
        if not msg.data:
            return
        with self._lock:
            self.reset_events.append(self._now())
        m = String()
        m.data = json.dumps({
            "has_reset": True,
            "stamp": self._ros_now_s(),
            "origin": {"x": 0.0, "y": 0.0, "yaw": 0.0},
        })
        self.pub_oz.publish(m)

    def _on_recipe(self, msg: String) -> None:
        with self._lock:
            self.recipe_events.append((self._now(), msg.data))
            outcome, delay = self.run_outcome, self.done_delay
        if outcome == "done":
            self._schedule(delay, self._emit_done)
        elif outcome == "estop":
            self._schedule(delay, self._emit_estop)
        # "timeout": deliberately emit nothing -> sequencer hits run_timeout.

    def _emit_done(self) -> None:
        m = Bool()
        m.data = True
        self.pub_done.publish(m)

    def _emit_estop(self) -> None:
        with self._lock:
            self.estop_state = True

    def _on_status(self, msg: String) -> None:
        try:
            d = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        with self._lock:
            self.status_log.append((self._now(), d))

    # -- periodic publish + due-action processing ----------------------
    def _tick(self) -> None:
        now = time.monotonic()
        with self._lock:
            alive = dict(self.alive)
            rtkq = self.rtk_quality
            batt = self.battery_v
            estop = self.estop_state
            due = [fn for (t, fn) in self._due if t <= now]
            self._due = [(t, fn) for (t, fn) in self._due if t > now]
            self.alive_timeline.append(
                (self._now(), alive.get("follower", False), alive.get("reposition", False)))

        mo = String()
        mo.data = json.dumps(alive)
        self.pub_orch.publish(mo)

        mr = String()
        if rtkq == 4:
            mr.data = "FIX: RTK FIXED (quality=4, sats=20, HDOP=0.8) | RTCM: ok"
        else:
            mr.data = f"FIX: RTK (quality={rtkq}) | RTCM: STALE"
        self.pub_rtk.publish(mr)

        me = Bool()
        me.data = bool(estop)
        self.pub_estop.publish(me)

        if self.pub_limo is not None:
            self._limo_div = (self._limo_div + 1) % 5  # ~2 Hz
            if self._limo_div == 0:
                ms = LimoStatus()
                ms.battery_voltage = float(batt)
                self.pub_limo.publish(ms)

        for fn in due:
            try:
                fn()
            except Exception:  # pragma: no cover - defensive
                pass

    # -- thread-safe getters for the test thread ----------------------
    def get_statuses(self):
        with self._lock:
            return [d for _, d in self.status_log]

    def get_start_names(self):
        with self._lock:
            return [n for _, n in self.start_events]

    def get_timeline(self):
        with self._lock:
            return list(self.alive_timeline)


class FollowerParamStub(Node):
    """Hosts /path_follower_node/set_parameters for the SET_PARAMS phase.

    rclpy auto-creates the parameter services for any node; naming this node
    ``path_follower_node`` puts the service at the path the sequencer's client
    targets. The two params it sets are declared so SetParameters succeeds.
    """

    def __init__(self) -> None:
        super().__init__("path_follower_node")
        self.declare_parameter("controller_type", "lpv-hinf")
        self.declare_parameter("v_const", 1.0)


_SMOKE_VENUE = {
    "name": "sim_venue",
    "frame": "rtk_enu",
    "corners_wgs84": [
        {"lat": 37.61244560055328, "lon": 126.99425116181375},
        {"lat": 37.61247986183099, "lon": 126.99428234249355},
        {"lat": 37.612522887599276, "lon": 126.99420019984245},
        {"lat": 37.61249579730363, "lon": 126.99417136609556},
    ],
    "safety_margin_m": 0.1,
    "exclusions": [],
    "start_pins": [
        {"id": "S1", "lat": 37.61247242527594, "lon": 126.99426356703046,
         "heading_deg": 135},
    ],
    "end_pins": [
        {"id": "E1", "lat": 37.612504561812116, "lon": 126.99419651180507,
         "heading_deg": 313},
    ],
}


class SimHarness:
    """Owns the mock world + the sequencer subprocess for one scenario."""

    def __init__(self, *, tmp_path, cells=1, circuit_breaker_k=1, max_retries=0,
                 rtk_window_pct=50, preflight="pass", preflight_sleep=0.0,
                 run_timeout_s=8.0, reposition_timeout_s=8.0, preflight_timeout_s=20.0,
                 rtk_loss_wait_s=2.0, settle_s=0.4):
        self.tmp = str(tmp_path)
        self.run_timeout_s = run_timeout_s
        self.reposition_timeout_s = reposition_timeout_s
        self.preflight_timeout_s = preflight_timeout_s
        self.rtk_loss_wait_s = rtk_loss_wait_s
        self.settle_s = settle_s

        self._write_fixtures(cells, circuit_breaker_k, max_retries, rtk_window_pct,
                             preflight, preflight_sleep)

        self.world = None
        self.pf = None
        self.exec = None
        self.thread = None
        self.proc = None
        self._logf = None

    # -- fixtures ------------------------------------------------------
    def _write_fixtures(self, cells, cbk, max_retries, rtk_window_pct,
                        preflight, preflight_sleep) -> None:
        self.venue_path = os.path.join(self.tmp, "venue.json")
        with open(self.venue_path, "w", encoding="utf-8") as f:
            json.dump(_SMOKE_VENUE, f)

        # repetitions == cells: same matrix point, distinct cell_ids (n00, n01..).
        self.yaml_path = os.path.join(self.tmp, "experiment.yaml")
        with open(self.yaml_path, "w", encoding="utf-8") as f:
            f.write(
                "run_id: sim_run\n"
                f"venue: {self.venue_path}\n"
                "matrix:\n"
                "  radius_m: [0.7]\n"
                "  controller: [lpv-hinf]\n"
                "  v_const: [0.2]\n"
                "  path_family: [step]\n"
                f"repetitions: {int(cells)}\n"
                "gating:\n"
                "  battery_volts_warn: 10.8\n"
                "  battery_volts_halt: 10.5\n"
                f"  rtk_run_window_pct: {rtk_window_pct}\n"
                "retry:\n"
                f"  max_retries: {int(max_retries)}\n"
                f"  circuit_breaker_k: {int(cbk)}\n"
                "ntfy: { server: https://ntfy.sh, topic: \"\" }\n"
                "artifact_sync: { enabled: false }\n"
            )

        self.preflight_path = os.path.join(self.tmp, "preflight.sh")
        if preflight == "pass":
            body = (
                "#!/usr/bin/env bash\n"
                f"sleep {float(preflight_sleep)}\n"
                'echo "  N/N checks passed"\n'
                'echo "  READY for next stage"\n'
                "exit 0\n"
            )
        else:  # "fail": emit FAILED_CHECKS-style lines the summarizer scrapes.
            body = (
                "#!/usr/bin/env bash\n"
                'echo "== rtk =="\n'
                'echo "  [FAIL] RTK status — not FIXED — got [quality=1]"\n'
                'echo "== summary =="\n'
                'echo "  FAILED:"\n'
                'echo "    - RTK status"\n'
                "exit 1\n"
            )
        with open(self.preflight_path, "w", encoding="utf-8") as f:
            f.write(body)
        os.chmod(self.preflight_path, 0o755)

        self.bag_root = os.path.join(self.tmp, "bags")
        self.ckpt = os.path.join(self.tmp, "checkpoint.json")
        self.logfile = os.path.join(self.tmp, "seq.log")

    # -- lifecycle -----------------------------------------------------
    def start(self) -> None:
        self.world = MockWorld()
        self.pf = FollowerParamStub()
        self.exec = MultiThreadedExecutor()
        self.exec.add_node(self.world)
        self.exec.add_node(self.pf)
        self.thread = threading.Thread(target=self._spin, daemon=True)
        self.thread.start()
        time.sleep(0.6)  # let the mock's pub/sub come up before the sequencer

        self._logf = open(self.logfile, "w", encoding="utf-8")
        cmd = [
            "ros2", "run", "limo_path_follower", "experiment_sequencer_node",
            "--ros-args",
            "-p", f"experiment_yaml:={self.yaml_path}",
            "-p", f"preflight_path:={self.preflight_path}",
            "-p", f"bag_root:={self.bag_root}",
            "-p", f"checkpoint_path:={self.ckpt}",
            "-p", "autostart:=false",
            "-p", f"orchestrator_settle_s:={self.settle_s}",
            "-p", f"reposition_timeout_s:={self.reposition_timeout_s}",
            "-p", f"run_timeout_s:={self.run_timeout_s}",
            "-p", f"preflight_timeout_s:={self.preflight_timeout_s}",
            "-p", f"rtk_loss_wait_s:={self.rtk_loss_wait_s}",
            "-p", "heartbeat_s:=3.0",
        ]
        self.proc = subprocess.Popen(
            cmd, env=dict(os.environ), stdout=self._logf, stderr=subprocess.STDOUT,
            text=True, start_new_session=True)

    def _spin(self) -> None:
        try:
            self.exec.spin()
        except Exception:  # pragma: no cover - shutdown races
            pass

    def stop(self) -> None:
        if self.proc is not None and self.proc.poll() is None:
            try:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGINT)
                self.proc.wait(timeout=6)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                try:
                    os.killpg(os.getpgid(self.proc.pid), signal.SIGKILL)
                except ProcessLookupError:
                    pass
        if self.exec is not None:
            try:
                self.exec.shutdown()
            except Exception:
                pass
        if self.thread is not None:
            self.thread.join(timeout=3)
        for n in (self.world, self.pf):
            if n is not None:
                try:
                    n.destroy_node()
                except Exception:
                    pass
        if self._logf is not None:
            try:
                self._logf.close()
            except Exception:
                pass
        time.sleep(0.3)  # let the domain settle before the next test

    # -- arming + observation -----------------------------------------
    def _pub_cmd(self, action: str) -> None:
        m = String()
        m.data = json.dumps({"action": action})
        self.world.pub_cmd.publish(m)

    def _armed(self) -> bool:
        for d in self.world.get_statuses():
            if "starting preflight" in (d.get("message") or ""):
                return True
            if d.get("phase") not in (None, "idle"):
                return True
        return bool(self.world.get_start_names())

    def arm(self, timeout: float = 20.0) -> None:
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self._pub_cmd("start")
            if self._armed():
                return
            if self.proc.poll() is not None:
                raise AssertionError(
                    f"sequencer exited early (rc={self.proc.returncode}); "
                    f"log tail:\n{self.log_tail()}")
            time.sleep(0.4)
        raise AssertionError(
            f"sequencer never acknowledged arm in {timeout}s; log tail:\n{self.log_tail()}")

    def last_phase(self):
        ss = self.world.get_statuses()
        return ss[-1].get("phase") if ss else None

    def messages(self) -> str:
        return " | ".join(d.get("message", "") or "" for d in self.world.get_statuses())

    def log_tail(self, n: int = 25) -> str:
        try:
            with open(self.logfile, "r", encoding="utf-8", errors="replace") as f:
                return "".join(f.readlines()[-n:])
        except OSError:
            return "(no log)"

    def wait_until(self, pred, timeout: float, what: str) -> None:
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if pred():
                return
            if self.proc.poll() is not None and not pred():
                raise AssertionError(
                    f"{what}: sequencer exited (rc={self.proc.returncode}); "
                    f"phase={self.last_phase()}; log tail:\n{self.log_tail()}")
            time.sleep(0.05)
        raise AssertionError(
            f"timeout waiting for {what}: last phase={self.last_phase()}; "
            f"messages=[{self.messages()}]; log tail:\n{self.log_tail()}")

    def wait_message_contains(self, substr: str, timeout: float) -> None:
        self.wait_until(lambda: substr in self.messages(), timeout,
                        f"status message containing {substr!r}")

    def wait_phase(self, phase: str, timeout: float) -> None:
        self.wait_until(lambda: self.last_phase() == phase, timeout,
                        f"phase == {phase}")
