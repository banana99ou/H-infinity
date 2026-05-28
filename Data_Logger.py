#!/usr/bin/env python3

"""
ROS 2 bag recorder wrapper with a health/recording indicator.

Behavior:
- Starts `ros2 bag record` for a fixed topic list.
- Publishes a recording indicator + heartbeat so other tools can confirm recording is active.

Notes:
- This script is intended to be started/stopped by an orchestrator (e.g. run_scenarios_from_files.py).
- Stopping is done via SIGINT (Ctrl+C semantics) so we can rename the bag folder with the actual duration.
"""

import argparse
import json
import os
import signal
import subprocess
import sys
import time
from datetime import datetime, timezone
from typing import Any, Dict, Optional, Sequence

# ROS imports are optional at module load: the pure helpers (build_sidecar,
# write_sidecar, BagRecorder, build_leg_dirname) are reused off-robot (laptop
# analysis, tools/analysis/make_sidecar.py) where rclpy is not installed. The
# health node + CLI main() still require ROS and only run on the NUC.
try:
    import rclpy  # pyright: ignore[reportMissingImports]
    from rclpy.node import Node  # pyright: ignore[reportMissingImports]
    from rclpy._rclpy_pybind11 import RCLError  # pyright: ignore[reportMissingImports]
    from std_msgs.msg import Bool, String  # pyright: ignore[reportMissingImports]
    _HAVE_RCLPY = True
except Exception:  # pragma: no cover - laptop has no ROS
    _HAVE_RCLPY = False
    Node = object          # lets DataLoggerHealthNode be defined (used only on NUC)
    RCLError = Exception
    Bool = String = None   # resolve method annotations (msg: Bool) at class-def time


TOPICS = [
    # --- Ground truth: RTK (L2/L4) ---
    "/gps_rtk_f9p_helical/gps/fix",
    "/gps_rtk_f9p_helical/gps/nmea",
    "/gps_rtk_f9p_helical/gps/rtk_status",
    # --- Regular GPS (L5): the prof's separate dataset, no control role ---
    "/pixhawk/global_position/raw/satellites",
    "/pixhawk/global_position/raw/fix",
    "/pixhawk/gpsstatus/gps1/raw",
    # --- Actuation chain (C3) + control feedback (L1) ---
    "/cmd_vel",
    "/cmd_vel_raw",
    "/wheel/odom",
    # The follower is launched with -r /wheel/odom:=/wheel/odom_zeroed
    # (orchestrator PROCS), so this re-anchored stream is what the controller
    # actually tracked. Bag BOTH: raw for the GNSS/odom comparison, zeroed for
    # the odom-belief metric (run_eval prefers this when present).
    "/wheel/odom_zeroed",
    "/imu",
    "/estop",
    # --- Path-follower interface (D1; produced by T1) ---
    # NOTE: these four are authored by task T1 (parallel). We record against the
    # documented contract (system_spec.md §4); no dependency on T1's files here.
    "/reference_path",          # nav_msgs/Path, latched (TRANSIENT_LOCAL)
    "/path_follower/status",    # std_msgs/Float32MultiArray (telemetry + timing)
    "/path_follower/done",      # std_msgs/Bool, latched (completion edge)
    "/path_follower/timing",    # std_msgs/Float32 (per-cycle controller ms, A3)
]


def build_bag_name(scenario: str, duration_label: str) -> str:
    """
    Build bag name: YY_MMDD_HHMM_<scenario>_<duration>.bag
    Duration will be replaced at the end with the measured runtime.
    """
    stamp = datetime.now().strftime("%y_%m%d_%H%M")
    return f"{stamp}_{scenario}_{duration_label}.bag"

def _rewrite_bag_metadata_and_files(bag_dir: str, old_base: str, new_base: str) -> None:
    """
    After renaming the bag directory, also rename the internal sqlite files and
    update metadata.yaml so users don't see DURATION_PLACEHOLDER lingering.

    ros2 bag record typically writes:
      <old_base>_0.db3 (+ optional -wal/-shm) and metadata.yaml referencing it.
    """
    # 1) Rename DB3 and sidecar files (if present)
    try:
        for fname in os.listdir(bag_dir):
            if not fname.startswith(old_base):
                continue
            # Preserve suffix like "_0.db3", "_0.db3-wal", etc.
            suffix = fname[len(old_base):]
            new_name = new_base + suffix
            src = os.path.join(bag_dir, fname)
            dst = os.path.join(bag_dir, new_name)
            if src != dst and os.path.exists(src):
                os.rename(src, dst)
    except FileNotFoundError:
        return

    # 2) Rewrite metadata.yaml references (best-effort string replace)
    meta_path = os.path.join(bag_dir, "metadata.yaml")
    try:
        with open(meta_path, "r", encoding="utf-8") as f:
            txt = f.read()
        if old_base in txt:
            txt = txt.replace(old_base, new_base)
            with open(meta_path, "w", encoding="utf-8") as f:
                f.write(txt)
    except FileNotFoundError:
        return
    except OSError:
        # Best-effort; metadata rewrite isn't strictly required to use the bag.
        return


# ---------------------------------------------------------------------------
# Programmatic recording API (T7) — driven by the experiment sequencer (T6).
#
# The CLI flow below (main()) keeps the interactive Ctrl+C-to-stop behavior for
# manual use. The sequencer instead wants to start/stop bags in-process without
# signals, and to pair each bag with a sidecar JSON. That is what this section
# provides: a BagRecorder that owns a `ros2 bag record` subprocess, plus a
# write_sidecar() that emits the D3 metadata next to the bag.
# ---------------------------------------------------------------------------

# Default output root. The sequencer may override per-batch. Kept consistent
# with the CLI flow's "Experiment Data" convention (D2).
DEFAULT_BAG_ROOT = os.path.join(os.getcwd(), "Experiment Data")

# Schema version for the sidecar, so the analysis pipeline (T11) can branch if
# the field set ever changes.
SIDECAR_SCHEMA_VERSION = 1


def _git_commit(repo_dir: Optional[str] = None) -> Optional[str]:
    """Best-effort short git SHA of the repo (D3 'git commit' field).

    Returns None if git is unavailable or this is not a checkout. The sequencer
    runs on the NUC where the repo lives at /home/agilex/H-infinity.
    """
    cwd = repo_dir or os.path.dirname(os.path.abspath(__file__))
    try:
        out = subprocess.check_output(
            ["git", "-C", cwd, "rev-parse", "--short", "HEAD"],
            stderr=subprocess.DEVNULL,
        )
        return out.decode("utf-8", "replace").strip() or None
    except (OSError, subprocess.CalledProcessError):
        return None


def build_leg_dirname(run_id: str, cell_id: str, leg: str) -> str:
    """Deterministic per-leg directory name (D2).

    Layout: <YY_MMDD_HHMM>_<run_id>_<cell_id>_<leg>
    e.g. 26_0526_1432_run07_R0p5-LPV_AtoB

    `leg` is the segment label (e.g. "AtoB", "turnaround", "BtoA"). All fields
    are sanitized to keep the path filesystem-safe and grep-pairable with
    external exports.
    """
    stamp = datetime.now().strftime("%y_%m%d_%H%M")

    def _safe(s: str) -> str:
        return "".join(c if (c.isalnum() or c in "-._") else "-" for c in str(s))

    return f"{stamp}_{_safe(run_id)}_{_safe(cell_id)}_{_safe(leg)}"


class BagRecorder:
    """Owns a single `ros2 bag record` subprocess (one bag per leg, D1).

    Programmatic, signal-free start/stop intended for the sequencer (T6). Unlike
    the CLI flow, stopping does not rely on Ctrl+C reaching this process: stop()
    sends SIGINT *to the bag subprocess only*, so the caller's own process is
    untouched. The recorded topic set is TOPICS (the full D1 set) unless
    overridden.

    Typical use by the sequencer:

        rec = BagRecorder(out_dir, topics=TOPICS)
        rec.start()
        ... run the leg, wait for /path_follower/done ...
        info = rec.stop()                 # returns {duration_s, bag_path, ...}
        write_sidecar(rec.bag_path, {...})  # D3
    """

    def __init__(
        self,
        bag_path: str,
        topics: Optional[Sequence[str]] = None,
        ros2_bin: str = "ros2",
    ) -> None:
        # `bag_path` is the directory ros2 bag will create (-o target).
        self.bag_path = bag_path
        self.topics = list(topics) if topics is not None else list(TOPICS)
        self._ros2_bin = ros2_bin
        self._proc: Optional[subprocess.Popen] = None
        self._start_monotonic: Optional[float] = None
        self._start_wallclock_utc: Optional[str] = None
        self._end_monotonic: Optional[float] = None

    @property
    def pid(self) -> int:
        return self._proc.pid if (self._proc and self._proc.pid) else -1

    @property
    def is_recording(self) -> bool:
        return self._proc is not None and self._proc.poll() is None

    def start(self) -> int:
        """Spawn `ros2 bag record`. Returns the subprocess PID.

        Raises RuntimeError if already started, FileNotFoundError if `ros2`
        is not on PATH (environment not sourced).
        """
        if self._proc is not None:
            raise RuntimeError("BagRecorder.start() called twice")

        os.makedirs(os.path.dirname(self.bag_path) or ".", exist_ok=True)
        cmd = [self._ros2_bin, "bag", "record", "-o", self.bag_path, *self.topics]

        self._start_monotonic = time.monotonic()
        self._start_wallclock_utc = datetime.now(timezone.utc).isoformat()
        self._proc = subprocess.Popen(cmd)
        return self.pid

    def stop(self, timeout_s: float = 10.0) -> Dict[str, Any]:
        """Stop the bag cleanly (SIGINT, so ros2 finalizes metadata.yaml).

        Idempotent-ish: returns the summary dict even if already stopped.
        Returns: {bag_path, pid, duration_s, returncode, start_utc, end_utc}.
        """
        if self._proc is None:
            return {
                "bag_path": self.bag_path,
                "pid": -1,
                "duration_s": 0.0,
                "returncode": None,
                "start_utc": self._start_wallclock_utc,
                "end_utc": None,
            }

        if self._proc.poll() is None:
            try:
                self._proc.send_signal(signal.SIGINT)
            except OSError:
                pass
            try:
                self._proc.wait(timeout=timeout_s)
            except subprocess.TimeoutExpired:
                self._proc.kill()
                self._proc.wait()

        self._end_monotonic = time.monotonic()
        duration_s = 0.0
        if self._start_monotonic is not None:
            duration_s = max(0.0, self._end_monotonic - self._start_monotonic)

        return {
            "bag_path": self.bag_path,
            "pid": self.pid,
            "duration_s": round(duration_s, 3),
            "returncode": self._proc.returncode,
            "start_utc": self._start_wallclock_utc,
            "end_utc": datetime.now(timezone.utc).isoformat(),
        }


def build_sidecar(
    *,
    run_id: str,
    cell_id: str,
    leg: str,
    cell_params: Dict[str, Any],
    path_recipe: Dict[str, Any],
    venue_id: Optional[str],
    start_pin_id: Optional[str],
    end_pin_id: Optional[str],
    rtk_summary: Dict[str, Any],
    classification: Dict[str, Any],
    wallclock: Dict[str, Any],
    controller_tuning: Dict[str, Any],
    bag_path: Optional[str] = None,
    topics: Optional[Sequence[str]] = None,
    git_commit: Optional[str] = None,
    extra: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    """Assemble the D3 sidecar dict. Pure (no I/O) so it is easy to unit-test.

    Field set is the D3 list in system_spec.md §3.7:
      cell params, path recipe, venue + pin IDs, RTK fix summary,
      classification, wallclock, git commit, controller tuning.
    """
    sidecar: Dict[str, Any] = {
        "schema_version": SIDECAR_SCHEMA_VERSION,
        "run_id": run_id,
        "cell_id": cell_id,
        "leg": leg,
        "bag_path": bag_path,
        "topics": list(topics) if topics is not None else list(TOPICS),
        "cell_params": cell_params,
        "path_recipe": path_recipe,
        "venue": {
            "venue_id": venue_id,
            "start_pin_id": start_pin_id,
            "end_pin_id": end_pin_id,
        },
        "rtk_summary": rtk_summary,
        "classification": classification,
        "wallclock": wallclock,
        "controller_tuning": controller_tuning,
        "git_commit": git_commit if git_commit is not None else _git_commit(),
    }
    if extra:
        sidecar["extra"] = extra
    return sidecar


def write_sidecar(bag_path: str, sidecar: Dict[str, Any]) -> str:
    """Write the sidecar JSON next to the bag and return its path (D3).

    Naming: <bag_dir>/<bag_basename>.sidecar.json so the bag and its metadata
    stay paired and grep-discoverable (D2). Written atomically via a temp file.
    """
    base = os.path.basename(bag_path.rstrip("/"))
    out_dir = bag_path if os.path.isdir(bag_path) else os.path.dirname(bag_path)
    os.makedirs(out_dir or ".", exist_ok=True)
    out_path = os.path.join(out_dir, f"{base}.sidecar.json")

    tmp_path = out_path + ".tmp"
    with open(tmp_path, "w", encoding="utf-8") as f:
        json.dump(sidecar, f, indent=2, sort_keys=False)
        f.write("\n")
    os.replace(tmp_path, out_path)
    return out_path


class DataLoggerHealthNode(Node):
    def __init__(self, scenario: str):
        super().__init__("data_logger_health")
        self._scenario = scenario

        self._pub_recording = self.create_publisher(Bool, "/data_logger/recording", 10)
        self._pub_health = self.create_publisher(String, "/data_logger/health", 10)

        # Publish at 2 Hz while alive
        self._timer = self.create_timer(0.5, self._tick)

        self.recording_active = False
        self.bag_path = ""
        self.ros2_bag_pid = -1
        self.soft_stop_received = False

        self.create_subscription(Bool, "/scenario_runner/soft_stop", self._on_soft_stop, 10)

    def _on_soft_stop(self, msg: Bool):
        if msg.data:
            self.soft_stop_received = True
            self.get_logger().info("Soft stop received; bag will be marked with _SOFTSTOP")

    def _tick(self):
        # Publish recording flag
        rec = Bool()
        rec.data = bool(self.recording_active)
        self._pub_recording.publish(rec)

        # Publish health string (human readable)
        msg = String()
        msg.data = (
            f"recording={1 if self.recording_active else 0} "
            f"scenario={self._scenario} "
            f"pid={self.ros2_bag_pid} "
            f"bag_path={self.bag_path}"
        )
        self._pub_health.publish(msg)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Record a ROS 2 bag for a given scenario.\n"
            "File is initially created with a placeholder duration and renamed "
            "on Ctrl+C based on the actual recording time (in seconds)."
        )
    )
    parser.add_argument(
        "scenario",
        help="Scenario name to embed in the bag filename (e.g. 'slalom', 'rally_1').",
    )
    args = parser.parse_args(argv)

    scenario = args.scenario
    placeholder_duration = "DURATION_PLACEHOLDER"
    bag_name = build_bag_name(scenario, placeholder_duration)

    # Ensure output base directory exists: "./Experiment Data/"
    base_dir = os.path.join(os.getcwd(), "Experiment Data")
    os.makedirs(base_dir, exist_ok=True)
    bag_path = os.path.join(base_dir, bag_name)

    # ros2 bag record (ROS 2) actually creates a directory named <bag_name>
    # (even if it ends with .bag), not a single file. We still follow your
    # requested naming scheme and later rename that directory.
    cmd = [
        "ros2",
        "bag",
        "record",
        "-o",
        bag_path,
        *TOPICS,
    ]

    print(f"Recording ROS 2 bag to: {bag_path}")
    print("Press Ctrl+C to stop; the bag will then be renamed with the actual duration.")

    start_monotonic = time.monotonic()

    # Start ROS node for health publishing
    rclpy.init(args=None)
    node = DataLoggerHealthNode(scenario=scenario)
    node.recording_active = False
    node.bag_path = bag_path

    try:
        proc = subprocess.Popen(cmd)
    except FileNotFoundError:
        print("Error: 'ros2' command not found. Make sure your ROS 2 environment is sourced.", file=sys.stderr)
        node.destroy_node()
        rclpy.shutdown()
        return 1
    except Exception as e:
        print(f"Error: failed to start ros2 bag record: {e}", file=sys.stderr)
        node.destroy_node()
        rclpy.shutdown()
        return 1

    node.ros2_bag_pid = proc.pid or -1
    node.recording_active = True
    print(f"DATA_LOGGER_STARTED ros2_bag_pid={node.ros2_bag_pid}")

    end_monotonic = None
    try:
        # Spin the health node while ros2 bag record runs.
        while rclpy.ok() and proc.poll() is None:
            rclpy.spin_once(node, timeout_sec=0.2)
        end_monotonic = time.monotonic()
    except KeyboardInterrupt:
        # Stop request (Ctrl+C or orchestrator SIGINT)
        print("\nStopping ros2 bag recording...")
        try:
            proc.send_signal(signal.SIGINT)
        except Exception:
            pass
        try:
            proc.wait(timeout=10.0)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()
        end_monotonic = time.monotonic()
    finally:
        node.recording_active = False
        try:
            # Final publish
            node._tick()
        except Exception:
            pass
        node.destroy_node()
        # rclpy installs its own SIGINT handler; on Ctrl+C/SIGINT it may already
        # have shut down the context. We want to still run our bag rename step.
        try:
            rclpy.shutdown()
        except RCLError:
            pass

    if end_monotonic is None:
        end_monotonic = time.monotonic()

    duration_s = max(0, int(end_monotonic - start_monotonic))
    duration_label = f"{duration_s}s"
    if node.soft_stop_received:
        duration_label += "_SOFTSTOP"
    final_bag_name = build_bag_name(scenario, duration_label)
    final_bag_path = os.path.join(base_dir, final_bag_name)

    # Rename the output directory (or file, depending on future ros2 behaviors)
    try:
        if os.path.exists(bag_path):
            os.rename(bag_path, final_bag_path)
            print(f"Renamed bag from '{bag_path}' to '{final_bag_path}'")
            # Also rename internal files + rewrite metadata so the placeholder doesn't linger.
            _rewrite_bag_metadata_and_files(
                bag_dir=final_bag_path,
                old_base=os.path.basename(bag_name),
                new_base=os.path.basename(final_bag_name),
            )
        else:
            print(
                f"Warning: expected output '{bag_path}' does not exist; "
                f"cannot rename to '{final_bag_path}'.",
                file=sys.stderr,
            )
    except OSError as e:
        print(f"Error renaming bag: {e}", file=sys.stderr)
        return 1

    print("Done.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())