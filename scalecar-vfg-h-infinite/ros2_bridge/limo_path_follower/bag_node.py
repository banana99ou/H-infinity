# -*- coding: utf-8 -*-
"""Topic-controlled bag recorder for the *manual* battle-station flow.

This is the manual-mode counterpart to the recording the experiment sequencer
does for itself (T6/T7). The sequencer owns a `Data_Logger.BagRecorder` and
drives start/stop in-process; an operator running a leg *by hand* from the
battle-station has no such driver. This node exposes the same recorder over two
topics so the browser (via rosbridge) can record a leg without SSH:

  sub  /bag/cmd     std_msgs/String  JSON  -- {"action":"start"|"stop", ...meta}
  pub  /bag/status  std_msgs/String  JSON  -- {recording, bag_path, duration_s,
                                               last_result, stamp}  (latched)

It reuses the exact recording + sidecar primitives the sequencer uses
(`Data_Logger.BagRecorder`, `build_leg_dirname`, `build_sidecar`,
`write_sidecar`) so a manually-recorded leg lands in the same `Experiment Data/`
layout with a paired `*.sidecar.json` (D2/D3). The sidecar is marked
`classification.manual = true` and `leg = "manual"` (unless overridden) so the
analysis pipeline never mistakes an operator bag for an auto-classified cell.

Safety: this node only *records*. It never publishes to `/cmd_vel*` or any mover
topic, so it sits entirely outside the `cmd_vel_raw -> estop -> cmd_vel` chain.

Mutual exclusion with the sequencer: starting a manual bag while the autonomous
sequencer is mid-run would put two `ros2 bag record` subprocesses on the same
topic set. This node soft-refuses a start whenever `/experiment/status` reports
an active phase (anything other than idle/done/aborted). Manual mode is for when
the sequencer is idle.

`/bag/cmd` start payload (all metadata fields optional; they only label the
sidecar):
  {"action": "start",
   "run_id": "...", "cell_id": "...", "leg": "...",
   "controller_type": "lpv-hinf"|"pid-ff", "v_const": 0.2,
   "path_family": "step"|"slalom"|"uturn", "radius_m": 0.7,
   "venue_id": "rooftop", "start_pin_id": "S1",
   "path_recipe": {"type": "step", "params": {...}}}
"""

import json
import os
import subprocess
import sys
import time
from datetime import datetime, timezone

import rclpy  # pyright: ignore[reportMissingImports]
from rclpy.node import Node  # pyright: ignore[reportMissingImports]
from rclpy.qos import (  # pyright: ignore[reportMissingImports]
    QoSProfile,
    QoSDurabilityPolicy,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
)
from std_msgs.msg import String  # pyright: ignore[reportMissingImports]

# Data_Logger lives at the repo root (outside the colcon tree), the same place
# the sequencer imports it from. Keep the import defensive so a path slip
# degrades to "cannot record" with a clear status, not a crash.
_REPO_ROOT = "/home/agilex/H-infinity"
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)
try:
    import Data_Logger  # type: ignore  # pyright: ignore[reportMissingImports]
except Exception:  # pragma: no cover - exercised only on a misconfigured NUC
    Data_Logger = None

# Same FIXED token the sequencer counts against (NMEA GGA quality=4), so the
# manual sidecar's rtk_summary is directly comparable to the auto one.
RTK_FIXED_TOKEN = "quality=4"

# Phases of /experiment/status during which a manual bag must NOT start (the
# sequencer is actively recording). Everything else (no sequencer, or idle/
# terminal) is safe.
_SEQ_IDLE_PHASES = {"", "idle", "done", "aborted", "paused"}


class BagNode(Node):

    def __init__(self):
        super().__init__("bag_node")

        # Bag output root. Mirrors the sequencer default
        # (_REPO_ROOT/"Experiment Data"); overridable for tests.
        self.declare_parameter("bag_root", os.path.join(_REPO_ROOT, "Experiment Data"))
        # Optional off-robot archive after stop, reusing the sequencer's
        # tools/sync/push_artifact.sh + experiment.yaml artifact_sync block.
        # Default off: manual bags are normally pulled with `sync.sh pull`.
        self.declare_parameter("archive", False)
        self.declare_parameter(
            "experiment_yaml", os.path.join(_REPO_ROOT, "scenarios", "experiment.yaml"))

        self._bag_root = str(self.get_parameter("bag_root").value)
        self._archive = bool(self.get_parameter("archive").value)
        self._experiment_yaml = str(self.get_parameter("experiment_yaml").value)
        self._push_artifact_sh = os.path.join(
            _REPO_ROOT, "tools", "sync", "push_artifact.sh")

        # Recording state.
        self._recorder = None
        self._bag_path = ""
        self._start_utc = None
        self._start_monotonic = None
        self._last_duration_s = 0.0
        self._last_result = "loaded" if Data_Logger is not None else (
            "Data_Logger unavailable — cannot record")
        self._meta = {}  # metadata from the start command, for the sidecar

        # RTK FIXED accounting over the current recording window.
        self._rtk_fixed_samples = 0
        self._rtk_total_samples = 0

        # Latest sequencer phase (for soft mutual-exclusion).
        self._seq_phase = ""

        latched = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub_status = self.create_publisher(String, "/bag/status", latched)
        self.create_subscription(String, "/bag/cmd", self._on_cmd, 10)
        self.create_subscription(
            String, "/experiment/status", self._on_exp, 10)
        self.create_subscription(
            String, "/gps_rtk_f9p_helical/gps/rtk_status", self._on_rtk, 10)

        # 1 Hz heartbeat so the browser sees a live duration while recording and
        # a fresh latched snapshot on (re)connect.
        self.create_timer(1.0, self._publish_status)
        self._publish_status()
        self.get_logger().info(
            "bag_node started. Waiting for /bag/cmd "
            "(start/stop). Records the D1 topic set to "
            f"{self._bag_root} with a paired sidecar.")

    # ------------------------------------------------------------------
    # Inputs
    # ------------------------------------------------------------------

    def _on_exp(self, msg):
        try:
            self._seq_phase = str((json.loads(msg.data) or {}).get("phase", ""))
        except (ValueError, TypeError):
            pass

    def _on_rtk(self, msg):
        if self._recorder is None:
            return
        self._rtk_total_samples += 1
        if RTK_FIXED_TOKEN in (msg.data or ""):
            self._rtk_fixed_samples += 1

    def _on_cmd(self, msg):
        try:
            payload = json.loads(msg.data)
        except (ValueError, TypeError):
            payload = {"action": str(msg.data or "").strip()}
        action = str(payload.get("action", "")).lower().strip()

        if action == "start":
            self._start(payload)
        elif action == "stop":
            self._stop()
        elif action in ("status", ""):
            self._last_result = "status published"
        else:
            self._last_result = f"unknown action: {action}"
        self._publish_status()

    # ------------------------------------------------------------------
    # Start / stop
    # ------------------------------------------------------------------

    def _start(self, payload):
        if Data_Logger is None:
            self._last_result = "Data_Logger unavailable — cannot record"
            return
        if self._recorder is not None:
            self._last_result = "already recording; stop first"
            return
        # Soft mutual-exclusion: never run a second recorder on the same topics
        # while the sequencer is mid-run.
        if self._seq_phase and self._seq_phase not in _SEQ_IDLE_PHASES:
            self._last_result = (
                f"refused: sequencer active (phase={self._seq_phase}); "
                "manual recording is for when the sequencer is idle")
            return

        run_id = str(payload.get("run_id") or "manual")
        cell_id = str(payload.get("cell_id") or "manual")
        leg = str(payload.get("leg") or "manual")
        try:
            dirname = Data_Logger.build_leg_dirname(run_id, cell_id, leg)
            self._bag_path = os.path.join(self._bag_root, dirname)
            self._recorder = Data_Logger.BagRecorder(
                self._bag_path, topics=Data_Logger.TOPICS)
            self._rtk_fixed_samples = 0
            self._rtk_total_samples = 0
            self._recorder.start()
            self._start_utc = datetime.now(timezone.utc).isoformat()
            self._start_monotonic = time.monotonic()
            self._meta = dict(payload)
            self._last_result = f"recording -> {os.path.basename(self._bag_path)}"
            self.get_logger().info(self._last_result)
        except Exception as exc:
            self.get_logger().error(f"bag start failed: {exc}")
            self._recorder = None
            self._bag_path = ""
            self._last_result = f"bag start failed: {exc}"

    def _stop(self):
        if self._recorder is None:
            self._last_result = "not recording"
            return
        try:
            info = self._recorder.stop(timeout_s=10.0)
        except Exception as exc:
            self.get_logger().error(f"bag stop failed: {exc}")
            info = {"bag_path": self._bag_path, "duration_s": 0.0, "end_utc": None}
        self._last_duration_s = float(info.get("duration_s") or 0.0)
        bag_path = self._bag_path

        self._write_sidecar(info)

        self._recorder = None
        self._bag_path = ""
        self._start_monotonic = None
        self._last_result = (
            f"stopped {os.path.basename(bag_path)} ({self._last_duration_s:.1f}s)")
        self.get_logger().info(self._last_result)

        if self._archive:
            self._archive_bag(bag_path)

    def _write_sidecar(self, info):
        if Data_Logger is None or not self._bag_path:
            return
        m = self._meta
        fixed_pct = None
        if self._rtk_total_samples > 0:
            fixed_pct = round(
                100.0 * self._rtk_fixed_samples / self._rtk_total_samples, 1)
        try:
            sidecar = Data_Logger.build_sidecar(
                run_id=str(m.get("run_id") or "manual"),
                cell_id=str(m.get("cell_id") or "manual"),
                leg=str(m.get("leg") or "manual"),
                cell_params={
                    "controller": m.get("controller_type"),
                    "v_const": m.get("v_const"),
                    "radius_m": m.get("radius_m"),
                    "path_family": m.get("path_family"),
                    "rep": m.get("rep"),
                },
                path_recipe=m.get("path_recipe") or {},
                venue_id=m.get("venue_id"),
                start_pin_id=m.get("start_pin_id"),
                end_pin_id=m.get("end_pin_id"),
                rtk_summary={
                    "fixed_pct": fixed_pct,
                    "fixed_samples": self._rtk_fixed_samples,
                    "total_samples": self._rtk_total_samples,
                    "token": RTK_FIXED_TOKEN,
                },
                classification={
                    "manual": True,
                    "pass": None,
                    "reason": "manual operator bag (not auto-classified)",
                    "duration_s": info.get("duration_s"),
                    "reached_end": None,
                    "estop": None,
                },
                wallclock={
                    "start_utc": self._start_utc,
                    "end_utc": info.get("end_utc"),
                    "duration_s": info.get("duration_s"),
                },
                controller_tuning={
                    "controller_type": m.get("controller_type"),
                    "v_const": m.get("v_const"),
                },
                bag_path=self._bag_path,
                topics=Data_Logger.TOPICS,
                # Operator bags have no guaranteed start-pin pose; leave the
                # path-frame anchor None (run_eval falls back to venue-local).
                path_frame_anchor=None,
            )
            Data_Logger.write_sidecar(self._bag_path, sidecar)
        except Exception as exc:
            self.get_logger().error(f"sidecar write failed: {exc}")

    def _archive_bag(self, bag_path):
        """Optional fire-and-forget off-robot push, mirroring the sequencer's
        _archive_leg. Reads the artifact_sync block from experiment.yaml; any
        failure is non-fatal."""
        if not bag_path or not os.path.isfile(self._push_artifact_sh):
            return
        art = self._load_artifact_sync()
        if not art.get("enabled"):
            return
        env = dict(os.environ)
        if art.get("mac_target"):
            env["ARTIFACT_MAC_TARGET"] = str(art["mac_target"])
        if art.get("nas_target"):
            env["ARTIFACT_NAS_TARGET"] = str(art["nas_target"])
        env["ARTIFACT_BWLIMIT_KBPS"] = str(art.get("bwlimit_kbps", 0))
        if art.get("local_repo"):
            env["ARTIFACT_LOCAL_REPO"] = str(art["local_repo"])
        try:
            subprocess.Popen(
                ["bash", self._push_artifact_sh, bag_path],
                env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                start_new_session=True,
            )
            self.get_logger().info(
                f"artifact_sync: dispatched push for {os.path.basename(bag_path)}")
        except Exception as exc:
            self.get_logger().warn(f"artifact_sync dispatch failed (non-fatal): {exc}")

    def _load_artifact_sync(self):
        try:
            import yaml  # pyright: ignore[reportMissingImports]
            with open(self._experiment_yaml, "r", encoding="utf-8") as f:
                cfg = yaml.safe_load(f) or {}
            return cfg.get("artifact_sync") or {}
        except Exception:
            return {}

    # ------------------------------------------------------------------
    # Status
    # ------------------------------------------------------------------

    def _publish_status(self):
        recording = self._recorder is not None and self._recorder.is_recording
        if recording and self._start_monotonic is not None:
            duration_s = round(time.monotonic() - self._start_monotonic, 1)
        else:
            duration_s = round(self._last_duration_s, 1)
        payload = {
            "recording": bool(recording),
            "bag_path": self._bag_path,
            "duration_s": duration_s,
            "rtk_fixed_samples": self._rtk_fixed_samples,
            "rtk_total_samples": self._rtk_total_samples,
            "last_result": self._last_result,
            "stamp": float(self.get_clock().now().nanoseconds) * 1e-9,
        }
        m = String()
        m.data = json.dumps(payload)
        self.pub_status.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = BagNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Best-effort: finalize a bag if the node is killed mid-recording so we
        # don't leak a running `ros2 bag record` subprocess.
        try:
            if node._recorder is not None:
                node._recorder.stop(timeout_s=10.0)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
