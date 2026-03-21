#!/usr/bin/env python3
"""
Reads human-readable scenario description files (hardcoded paths) and invokes
`limo_scenario_motion.py` accordingly.

This script is a ROS 2 node (rclpy) that orchestrates:
- (optional) Data_Logger.py (ros2 bag record wrapper)
- limo_scenario_motion.py (motion per level)

It publishes simple status/event strings for monitoring and subscribes to /estop
to refuse/abort motion when E-stop is active.

1. See all available levels:
    python3 run_scenarios_from_files.py --list-levels
2. Run a specific level (e.g., level_2 or open_sky_1):
    python3 run_scenarios_from_files.py --level level_2
3. Test the command without moving the robot (Dry Run):
    python3 run_scenarios_from_files.py --level level_2 --dry-run
"""

from __future__ import annotations

import argparse
import configparser
import subprocess
import sys
import time
import signal
import shutil
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Tuple, Optional

import rclpy  # pyright: ignore[reportMissingImports]
from rclpy.node import Node  # pyright: ignore[reportMissingImports]
from std_msgs.msg import String, Bool  # pyright: ignore[reportMissingImports]


ROOT = Path(__file__).resolve().parent
MOTION_SCRIPT = ROOT / "limo_scenario_motion.py"
DATA_LOGGER_SCRIPT = ROOT / "Data_Logger.py"

# Hardcoded scenario description files (as requested)
CONST_VEL_FILE = ROOT / "scenarios" / "const_vel_scenarios.ini"
CONST_ACC_FILE = ROOT / "scenarios" / "const_acc_scenarios.ini"
STATIC_FILE    = ROOT / "scenarios" / "static_scenarios.ini"
ANGULAR_RATE_FILE = ROOT / "scenarios" / "angular_rate_scenarios.ini"

def _try_get_data_logger_topics() -> List[str]:
    """
    Best-effort import of Data_Logger.TOPICS so we don't duplicate the canonical list.
    Falls back to a local copy if import fails (e.g., when running in a minimal env).
    """
    try:
        # Local import so this file still imports even if ROS 2 deps for Data_Logger
        # aren't available at static-analysis time.
        from Data_Logger import TOPICS as DL_TOPICS  # type: ignore
        return list(DL_TOPICS)
    except Exception:
        return [
            "/gps_rtk_f9p_helical/gps/fix",
            "/gps_rtk_f9p_helical/gps/nmea",
            "/gps_rtk_f9p_helical/gps/rtk_status",
            "/pixhawk/global_position/raw/satellites",
            "/pixhawk/global_position/raw/fix",
            "/pixhawk/gpsstatus/gps1/raw",
            "/cmd_vel",
            "/cmd_vel_raw",
            "/wheel/odom",
            "/imu",
            "/estop",
        ]


@dataclass(frozen=True)
class ScenarioCall:
    name: str
    argv: List[str]

EXPECTED_SECTION_KEYS = [
    "scenario",
    "heading_deg",
    "distance_m",
    "radius_m",
    "speed_mps",
    "acc_mps2",
    "max_speed_mps",
    "vel_tolerance",
    "acc_tolerance",
    "rate_hz",
    "max_duration_s",
    "planned_time_s",
    "final_speed_mps",
    "notes",
]


def _get_float(section: configparser.SectionProxy, key: str) -> float:
    try:
        return section.getfloat(key)
    except Exception as e:
        raise ValueError(f"Invalid or missing float '{key}' in section [{section.name}]: {e}") from e


def _get_optional_float(section: configparser.SectionProxy, key: str) -> Optional[float]:
    v = section.get(key, fallback=None)
    if v is None:
        return None
    s = str(v).strip()
    if not s:
        return None
    try:
        return float(s)
    except Exception as e:
        raise ValueError(f"Invalid float '{key}' in section [{section.name}]: {e}") from e


def _get_str(section: configparser.SectionProxy, key: str) -> str:
    v = section.get(key, fallback=None)
    if v is None or not str(v).strip():
        raise ValueError(f"Missing '{key}' in section [{section.name}]")
    return str(v).strip()


def load_scenario_file(path: Path, max_dist: Optional[float] = None) -> Tuple[Dict[str, str], List[ScenarioCall], float]:
    if not path.exists():
        raise FileNotFoundError(f"Scenario file not found: {path}")

    cfg = configparser.ConfigParser(interpolation=None)
    cfg.optionxform = str  # preserve key case (we use snake_case anyway)
    cfg.read(path, encoding="utf-8")

    if "meta" not in cfg:
        raise ValueError(f"Missing required [meta] section in {path}")

    meta = cfg["meta"]
    scenario_type = _get_str(meta, "type")
    inter_run_pause_s = meta.getfloat("inter_run_pause_s", fallback=0.0)

    calls: List[ScenarioCall] = []
    for section_name in cfg.sections():
        if section_name == "meta":
            continue

        s = cfg[section_name]

        # Enforce unified template: all sections must include the same keys (values may be blank).
        missing = [k for k in EXPECTED_SECTION_KEYS if k not in s]
        if missing:
            raise ValueError(
                f"Section [{section_name}] in {path} is missing required keys: {', '.join(missing)}"
            )

        # The actual motion type for this section
        this_scenario = _get_str(s, "scenario")

        # defaults
        rate_hz = meta.getfloat("default_rate_hz", fallback=20.0)
        max_duration_s = meta.getfloat("default_max_duration_s", fallback=60.0)

        # per-scenario overrides
        rate_hz_opt = _get_optional_float(s, "rate_hz")
        if rate_hz_opt is not None:
            rate_hz = rate_hz_opt
        max_dur_opt = _get_optional_float(s, "max_duration_s")
        if max_dur_opt is not None:
            max_duration_s = max_dur_opt

        distance_m = _get_float(s, "distance_m")

        # Option 1: Apply global max distance cap
        if max_dist is not None and distance_m > max_dist:
            distance_m = max_dist

        argv = [
            sys.executable,
            str(MOTION_SCRIPT),
            "--scenario",
            this_scenario,
            "--distance",
            f"{distance_m}",
            "--rate-hz",
            f"{rate_hz}",
            "--max-duration",
            f"{max_duration_s}",
        ]

        if this_scenario == "static":
            heading_deg = _get_float(s, "heading_deg")
            argv += ["--heading-deg", f"{heading_deg}"]

        elif this_scenario == "const_vel":
            speed_mps = _get_float(s, "speed_mps")
            vel_tol = meta.getfloat("default_vel_tolerance", fallback=0.05)
            vel_tol_opt = _get_optional_float(s, "vel_tolerance")
            if vel_tol_opt is not None:
                vel_tol = vel_tol_opt
            argv += ["--speed", f"{speed_mps}", "--vel-tolerance", f"{vel_tol}"]

        elif this_scenario == "const_acc":
            acc_mps2 = _get_float(s, "acc_mps2")
            acc_tol = meta.getfloat("default_acc_tolerance", fallback=0.1)
            acc_tol_opt = _get_optional_float(s, "acc_tolerance")
            if acc_tol_opt is not None:
                acc_tol = acc_tol_opt

            max_speed_mps = _get_optional_float(s, "max_speed_mps")
            if max_speed_mps is None:
                max_speed_mps = _get_optional_float(meta, "default_max_speed_mps")  # type: ignore[arg-type]
            if max_speed_mps is None:
                raise ValueError(
                    f"Missing 'max_speed_mps' in section [{section_name}] for const_acc scenarios"
                )

            argv += [
                "--acc",
                f"{acc_mps2}",
                "--acc-tolerance",
                f"{acc_tol}",
                "--max-speed",
                f"{max_speed_mps}",
            ]
        elif this_scenario == "circular":
            speed_mps = _get_float(s, "speed_mps")
            radius_m = _get_float(s, "radius_m")
            argv += ["--speed", f"{speed_mps}", "--radius", f"{radius_m}"]

        elif this_scenario == "s_curve":
            # s_curve currently has no special params other than distance/max_duration
            pass

        else:
            raise ValueError(f"Unsupported scenario type '{this_scenario}' in section [{section_name}] of {path}")

        calls.append(ScenarioCall(name=section_name, argv=argv))

    meta_dict = {k: v for k, v in meta.items()}
    return meta_dict, calls, float(inter_run_pause_s)


def start_data_logger(scenario_label: str) -> subprocess.Popen:
    """
    Start Data_Logger.py (ros2 bag record wrapper) in a child process.
    The logger is stopped later via SIGINT (or terminate/kill fallback) so it can finalize and rename the bag.
    """
    if not DATA_LOGGER_SCRIPT.exists():
        raise FileNotFoundError(f"Data_Logger.py not found: {DATA_LOGGER_SCRIPT}")

    # Use the same interpreter as this script.
    cmd = [sys.executable, str(DATA_LOGGER_SCRIPT), scenario_label]
    # Force a stable CWD so Data_Logger writes under this workspace's "Experiment Data/".
    return subprocess.Popen(cmd, cwd=str(ROOT))


def wait_process_alive(proc: subprocess.Popen, seconds: float) -> None:
    """
    Precaution-only: confirm the subprocess does not immediately die.
    """
    deadline = time.time() + float(seconds)
    while time.time() < deadline:
        if proc.poll() is not None:
            raise RuntimeError(f"Process exited early (code={proc.returncode})")
        time.sleep(0.05)


def stop_data_logger(proc: subprocess.Popen, timeout_s: float = 15.0) -> None:
    """
    Stop Data_Logger.py gracefully.
    On POSIX, SIGINT is the intended stop signal (similar to Ctrl+C) so Data_Logger can rename the bag.
    On platforms where SIGINT isn't delivered as expected, we fall back to terminate/kill.
    """
    if proc.poll() is not None:
        return

    try:
        proc.send_signal(signal.SIGINT)
    except Exception:
        try:
            proc.terminate()
        except Exception:
            pass

    try:
        proc.wait(timeout=timeout_s)
    except Exception:
        try:
            proc.kill()
        except Exception:
            pass
        try:
            proc.wait(timeout=5.0)
        except Exception:
            pass


class ScenarioOrchestrator(Node):
    def __init__(self):
        super().__init__("scenario_orchestrator")
        self._pub_status = self.create_publisher(String, "/scenario_runner/status", 10)
        self._pub_event = self.create_publisher(String, "/scenario_runner/event", 10)

        # Helical RTK status gating (String published by GPS-RTK_ROS2_pub_node.py)
        self._helical_rtk_status_text: Optional[str] = None
        self._helical_rtk_fix_desc: Optional[str] = None
        self._helical_rtk_quality: Optional[int] = None
        self._helical_rtk_seen = False
        self._helical_rtk_seen_walltime = 0.0
        self.create_subscription(String, "/gps_rtk_f9p_helical/gps/rtk_status", self._on_helical_rtk_status, 10)

        self.estop_active = False
        self._estop_seen = False
        self.create_subscription(Bool, "/estop", self._on_estop, 10)

        self.soft_stop_active = False
        self.create_subscription(Bool, "/scenario_runner/soft_stop", self._on_soft_stop, 10)

    _HELICAL_RTK_RE = re.compile(r"^FIX:\s*(?P<desc>.*?)\s*\(quality=(?P<q>\d+),")

    def _on_helical_rtk_status(self, msg: String):
        s = (msg.data or "").strip()
        self._helical_rtk_status_text = s
        self._helical_rtk_seen = True
        self._helical_rtk_seen_walltime = time.time()
        m = self._HELICAL_RTK_RE.match(s)
        if m:
            self._helical_rtk_fix_desc = m.group("desc").strip()
            try:
                self._helical_rtk_quality = int(m.group("q"))
            except Exception:
                self._helical_rtk_quality = None

    def _on_estop(self, msg: Bool):
        self.estop_active = bool(msg.data)
        self._estop_seen = True

    def _on_soft_stop(self, msg: Bool):
        if msg.data:
            self.soft_stop_active = True
            self.get_logger().info("Soft stop requested via topic")

    def status(self, text: str):
        m = String()
        m.data = text
        self._pub_status.publish(m)

    def event(self, text: str):
        m = String()
        m.data = text
        self._pub_event.publish(m)

    def spin_sleep(self, seconds: float):
        end = time.time() + float(seconds)
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def _wait_for_ros2_cli(self, timeout_s: float = 5.0) -> None:
        """
        Ensure `ros2` CLI is available before we rely on it for topic health checks.
        """
        if shutil.which("ros2"):
            return
        # Give a short grace period in case PATH updates are in-flight (rare, but cheap).
        end = time.time() + float(timeout_s)
        while time.time() < end:
            if shutil.which("ros2"):
                return
            time.sleep(0.1)
        raise RuntimeError(
            "Preflight failed: `ros2` command not found in PATH. "
            "Source your ROS 2 environment (e.g., `source /opt/ros/<distro>/setup.bash`) "
            "and try again, or run with --no-preflight."
        )

    def _topic_exists(self, topic: str) -> bool:
        try:
            names_and_types = self.get_topic_names_and_types()
        except Exception:
            return False
        return any(name == topic for name, _types in names_and_types)

    def _wait_for_topics(self, topics: List[str], timeout_s: float) -> List[str]:
        """
        Wait until all topics appear in the ROS graph. Returns missing topics.
        """
        pending = set(t.strip() for t in topics if t and t.strip())
        deadline = time.time() + float(timeout_s)
        while rclpy.ok() and pending and time.time() < deadline:
            # Graph updates happen while spinning.
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                present = {name for name, _types in self.get_topic_names_and_types()}
            except Exception:
                present = set()
            pending = {t for t in pending if t not in present}
        return sorted(pending)

    def _echo_topic_once(self, topic: str, timeout_s: float) -> Tuple[bool, str]:
        """
        Use `ros2 topic echo --once <topic>` to confirm messages are flowing.
        Returns (ok, debug_text).
        """
        cmd = ["ros2", "topic", "echo", "--once", topic]
        try:
            p = subprocess.run(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=float(timeout_s),
            )
        except subprocess.TimeoutExpired as e:
            out = (getattr(e, "stdout", None) or "").strip()
            err = (getattr(e, "stderr", None) or "").strip()
            tail = ""
            if err:
                tail = f" stderr={err[:300]}"
            elif out:
                tail = f" stdout={out[:300]}"
            return False, f"timeout after {timeout_s:.1f}s running: {' '.join(cmd)}{tail}"
        except Exception as e:
            return False, f"failed to run: {' '.join(cmd)} err={e}"

        out = (p.stdout or "").strip()
        err = (p.stderr or "").strip()
        if p.returncode == 0 and (out or err):
            return True, "ok"
        if p.returncode != 0:
            extra = ""
            if err:
                extra = f" stderr={err[:300]}"
            elif out:
                extra = f" stdout={out[:300]}"
            return False, f"exit={p.returncode} running: {' '.join(cmd)}{extra}"
        return False, f"no output running: {' '.join(cmd)}"

    def preflight_or_raise(
        self,
        must_exist_topics: List[str],
        must_have_publisher_topics: List[str],
        must_receive_message_topics: List[str],
        must_have_subscribers: Dict[str, int],
        topic_appear_timeout_s: float = 20.0,
        echo_timeout_s: float = 2.5,
        estop_wait_s: float = 3.0,
        require_rtk_fixed: bool = True,
        rtk_timeout_s: float = 60.0,
    ) -> None:
        """
        Preflight checks before experiment run:
        - ROS graph has required topics
        - required topics have publishers
        - messages can be received (via ros2 CLI) for key streams
        - critical command routing exists (subscribers on cmd topics)
        - E-stop is not active (and ideally has been observed at least once)
        """
        self.status("phase=preflight starting=1")
        self.get_logger().info("Preflight: validating ROS topics and E-stop state...")

        self._wait_for_ros2_cli()

        # Give subscriptions a moment so /estop state is known if it is publishing.
        if estop_wait_s > 0:
            end = time.time() + float(estop_wait_s)
            while rclpy.ok() and not self._estop_seen and time.time() < end:
                rclpy.spin_once(self, timeout_sec=0.1)

        if self.estop_active:
            raise RuntimeError("Preflight failed: E-STOP is ACTIVE (/estop == true). Refusing to start.")

        # 1) Topic presence in the graph
        missing = self._wait_for_topics(must_exist_topics, timeout_s=float(topic_appear_timeout_s))
        if missing:
            raise RuntimeError(
                "Preflight failed: required topic(s) not present in ROS graph: "
                + ", ".join(missing)
            )

        # 2) Publisher counts (when expected to exist pre-run)
        no_pub: List[str] = []
        for t in must_have_publisher_topics:
            try:
                if self.count_publishers(t) <= 0:
                    no_pub.append(t)
            except Exception:
                no_pub.append(t)
        if no_pub:
            raise RuntimeError(
                "Preflight failed: required topic(s) have no publishers: " + ", ".join(sorted(no_pub))
            )

        # 3) RTK gate: do not start experiments unless we have RTK FIXED.
        # This is stronger than "GPS topics exist" and matches the experiment requirement.
        if bool(require_rtk_fixed):
            deadline = time.time() + float(rtk_timeout_s)
            while rclpy.ok() and time.time() < deadline:
                # Keep subscriptions/graph fresh.
                rclpy.spin_once(self, timeout_sec=0.1)
                q = self._helical_rtk_quality
                if q == 4:  # NMEA GGA fix quality 4 == RTK FIXED
                    break
            else:
                last = self._helical_rtk_status_text or "(no /gps_rtk_f9p_helical/gps/rtk_status received)"
                raise RuntimeError(
                    "Preflight failed: RTK FIX not available (need RTK FIXED before starting). "
                    f"Waited {float(rtk_timeout_s):.1f}s; last status: {last}"
                )

        # 3) Subscriber checks for command routing (e.g., safety node listening to cmd_vel_raw)
        bad_sub: List[str] = []
        for t, min_n in must_have_subscribers.items():
            try:
                n = int(self.count_subscribers(t))
            except Exception:
                n = 0
            if n < int(min_n):
                bad_sub.append(f"{t} (subscribers={n} < {min_n})")
        if bad_sub:
            raise RuntimeError(
                "Preflight failed: missing required subscribers for command routing: "
                + ", ".join(bad_sub)
            )

        # 5) Message flow checks (best-effort but strict by default)
        no_msg: List[str] = []
        for t in must_receive_message_topics:
            ok, dbg = self._echo_topic_once(t, timeout_s=float(echo_timeout_s))
            if not ok:
                no_msg.append(f"{t} ({dbg})")
        if no_msg:
            for item in no_msg:
                self.get_logger().error(f"Preflight echo check failed: {item}")
            raise RuntimeError(
                "Preflight failed: topic(s) not producing messages (ros2 topic echo --once failed):\n"
                + "\n".join(f"- {item}" for item in no_msg)
            )

        self.get_logger().info("Preflight: OK (topics present, publishers/subscribers OK, messages flowing).")
        self.status("phase=preflight ok=1")

    def run_motion_subprocess(self, argv: List[str]) -> int:
        """
        Run a motion level and keep spinning so /estop or soft stop can be observed.
        If estop becomes active, attempt to stop the motion process.
        """
        self.soft_stop_active = False  # Reset for each run
        proc = subprocess.Popen(argv)
        try:
            while rclpy.ok() and proc.poll() is None:
                rclpy.spin_once(self, timeout_sec=0.1)
                if self.estop_active:
                    self.get_logger().error("E-STOP active; aborting motion subprocess")
                    try:
                        proc.send_signal(signal.SIGINT)
                    except Exception:
                        try:
                            proc.terminate()
                        except Exception:
                            pass
                    break

                if self.soft_stop_active:
                    self.get_logger().info("Soft stop active; terminating motion subprocess")
                    try:
                        proc.send_signal(signal.SIGINT)
                    except Exception:
                        pass
                    break

            try:
                return proc.wait(timeout=10.0)
            except Exception:
                try:
                    proc.kill()
                except Exception:
                    pass
                return proc.wait()
        finally:
            # Best-effort ensure it's not left running
            if proc.poll() is None:
                try:
                    proc.kill()
                except Exception:
                    pass

    def run_level(self, scenario_type: str, call: ScenarioCall, no_record: bool, record_startup_wait_s: float,) -> int:
        label = f"{scenario_type}_{call.name}" if scenario_type else call.name

        self.event(f"LEVEL_START {label}")
        self.status(f"phase=starting level={label}")

        if self.estop_active:
            raise RuntimeError("E-STOP active; refusing to start level")

        logger_proc: Optional[subprocess.Popen] = None
        try:
            if not no_record:
                logger_proc = start_data_logger(label)
                if record_startup_wait_s > 0:
                    wait_process_alive(logger_proc, seconds=float(record_startup_wait_s))

            if self.estop_active:
                raise RuntimeError("E-STOP active; refusing to start motion")

            self.status(f"phase=running level={label}")
            rc = self.run_motion_subprocess(call.argv)

            if self.soft_stop_active:
                self.event(f"LEVEL_SOFT_STOP {label}")
            return rc
        finally:
            self.status(f"phase=stopping level={label}")
            if logger_proc is not None:
                stop_data_logger(logger_proc)
            self.event(f"LEVEL_END {label}")


def main(argv: List[str] | None = None) -> int:
    p = argparse.ArgumentParser( description="Run limo_scenario_motion.py scenarios from INI description files (hardcoded).")
    p.add_argument(
        "-s", "--scenario-type",
        choices=["static", "const_vel", "const_acc", "angular_rate"],
        default="all",
        help="Which scenario file(s) to search in.",
    )
    p.add_argument(
        "--dry-run",
        action="store_true",
        help="Print the commands that would run, but do not execute them.",
    )
    p.add_argument(
        "-l", "--level",
        type=str,
        default=None,
        help="The level section name to run (e.g. 'level_2'). Required unless using --list-levels.",
    )
    p.add_argument(
        "-ll", "--list-levels",
        action="store_true",
        help="List available level section names and exit (filtered by --scenario-type).",
    )
    p.add_argument(
        "-sn", "--show-notes",
        action="store_true",
        help="Show notes for scenario levels (use with --list-levels or --level).",
    )
    p.add_argument(
        "--no-record",
        action="store_true",
        help="Do not start/stop Data_Logger.py (motion only).",
    )
    p.add_argument(
        "--record-startup-wait-s",
        type=float,
        default=1.0,
        help="Precaution-only: wait this many seconds and confirm Data_Logger.py process stays alive before motion starts.",
    )
    p.add_argument(
        "-md", "--max-dist",
        type=float,
        default=20,
        help="Hard cap on distance (m) for any scenario. Overrides .ini file distance if smaller.",
    )
    p.add_argument(
        "-np", "--no-preflight",
        action="store_true",
        help="Skip preflight checks (topics/publishers/message flow/E-stop).",
    )
    p.add_argument(
        "--preflight-topic-timeout-s",
        type=float,
        default=20.0,
        help="Seconds to wait for required topics to appear in the ROS graph.",
    )
    p.add_argument(
        "--preflight-echo-timeout-s",
        type=float,
        default=2.5,
        help="Seconds to wait for `ros2 topic echo --once` per topic during preflight.",
    )
    p.add_argument(
        "--preflight-estop-wait-s",
        type=float,
        default=3.0,
        help="Seconds to wait to observe at least one /estop message before starting (still fails if E-stop is active).",
    )
    p.add_argument(
        "--no-rtk-gate",
        "--no-rtk-gating",
        dest="no_rtk_gate",
        action="store_true",
        help="Disable RTK FIXED gating during preflight (not recommended for GNSS fix experiments).",
    )
    p.add_argument(
        "--preflight-rtk-timeout-s",
        type=float,
        default=60.0,
        help="Seconds to wait for RTK FIXED (via /gps_rtk_f9p_helical/gps/rtk_status) before starting.",
    )
    p.add_argument(
        "--preflight-extra-topic",
        action="append",
        default=[],
        help="Additional topic to require/check during preflight. Can be provided multiple times.",
    )
    p.add_argument(
        "--preflight-skip-topic",
        action="append",
        default=[],
        help="Topic to exclude from preflight requirements. Can be provided multiple times.",
    )
    args = p.parse_args(argv)

    if not args.list_levels and not args.level:
        print("Error: --level <name> is required unless using --list-levels.")
        return 1

    rclpy.init(args=None)
    node = ScenarioOrchestrator()

    # Load scenario files
    files: List[Path] = []
    if args.scenario_type in ("static"):
        files.append(STATIC_FILE)
    if args.scenario_type in ("const_vel"):
        files.append(CONST_VEL_FILE)
    if args.scenario_type in ("const_acc"):
        files.append(CONST_ACC_FILE)
    if args.scenario_type in ("angular_rate"):
        files.append(ANGULAR_RATE_FILE)

    # Error out if the motion script is not found
    if not MOTION_SCRIPT.exists():
        node.get_logger().error(f"Error: motion script not found: {MOTION_SCRIPT}")
        node.destroy_node()
        rclpy.shutdown()
        return 2

    # Run the scenario
    try:
        node.event("RUN_START")

        # 1. If listing, gather names and exit.
        if args.list_levels:
            for path in files:
                try:
                    meta, calls, _ = load_scenario_file(path, max_dist=args.max_dist)
                    scenario_type = meta.get("type", "").strip()
                    print(f"{path.name} (type={scenario_type})")
                    # Show header for notes if requested
                    if args.show_notes:
                        print("    Notes:")
                    for c in calls:
                        line = f"  - {c.name}"
                        if args.show_notes:
                            # Print the notes field if present and nonempty
                            try:
                                cfg = configparser.ConfigParser(interpolation=None)
                                cfg.optionxform = str
                                cfg.read(path, encoding="utf-8")
                                if c.name in cfg:
                                    notes = cfg[c.name].get("notes", "").strip()
                                    if notes:
                                        line += f"  [notes: {notes}]"
                            except Exception as e:
                                line += f"  [error reading notes: {e}]"
                        print(line)
                except Exception as e:
                    print(f"Error reading {path.name}: {e}")
            return 0

        # 2. Find the specific requested level (across all allowed files).
        target_call: Optional[Tuple[Path, str, ScenarioCall]] = None
        selected_level_notes = None
        for path in files:
            try:
                meta, calls, _ = load_scenario_file(path, max_dist=args.max_dist)
                scenario_type = meta.get("type", "").strip()
                for c in calls:
                    if c.name == args.level:
                        target_call = (path, scenario_type, c)
                        # If --show-notes, find notes for this level
                        if args.show_notes:
                            try:
                                cfg = configparser.ConfigParser(interpolation=None)
                                cfg.optionxform = str
                                cfg.read(path, encoding="utf-8")
                                if c.name in cfg:
                                    notes = cfg[c.name].get("notes", "").strip()
                                    if notes:
                                        selected_level_notes = notes
                            except Exception as e:
                                selected_level_notes = f"(Error reading notes: {e})"
                        break
                if target_call:
                    break
            except Exception:
                continue

        if not target_call:
            node.get_logger().error(
                f"Level '{args.level}' not found in the selected file(s) ({args.scenario_type}). "
                "Use --list-levels to see available names."
            )
            return 5

        path, scenario_type, call = target_call
        node.get_logger().info(f"Scenario file: {path.name} type={scenario_type}")
        node.get_logger().info(f"Level: {call.name}")
        node.get_logger().info("Command: " + " ".join(call.argv))
        node.status(f"phase=ready file={path.name} level={call.name}")

        # If --show-notes and notes exist, print them for this level
        if args.show_notes and selected_level_notes:
            print(f"Notes for {call.name}: {selected_level_notes}")

        if args.dry_run:
            node.get_logger().info("Dry run: not executing.")
            return 0

        if not bool(args.no_preflight):
            # Build preflight requirements from the Data_Logger topic list.
            # We treat command output topics differently:
            # `/cmd_vel_raw`, `/cmd_vel`: our motion node publishes this during the run; pre-run we require that
            # someone is subscribed (typically a safety/E-stop filter node).
            dl_topics = _try_get_data_logger_topics() # get Data_Logger topic list
            skip = set(t.strip() for t in (args.preflight_skip_topic or []) if t and t.strip()) # compile skip topic list
            extras = [t.strip() for t in (args.preflight_extra_topic or []) if t and t.strip()]      # compile extra topic list to check

            must_exist = sorted({t for t in (dl_topics + extras) if t and t not in skip})

            # Topics that should exist / have pubs pre-run but may be silent unless an event occurs.
            # `/estop` is often only published on state change, so "echo --once" can legitimately time out.
            must_have_pub = [t for t in must_exist if t not in {"/cmd_vel_raw", "/cmd_vel"}]
            must_recv_msg = [t for t in must_exist if t not in {"/cmd_vel_raw", "/cmd_vel", "/estop"}]

            must_have_subs = {
                "/cmd_vel_raw": 1,  # safety node should be listening
                "/cmd_vel": 1,      # base driver should be listening
            }
            # Respect user skip list for subscriber checks too
            for t in list(must_have_subs.keys()):
                if t in skip:
                    must_have_subs.pop(t, None)

            node.preflight_or_raise(
                must_exist_topics=must_exist,
                must_have_publisher_topics=must_have_pub,
                must_receive_message_topics=must_recv_msg,
                must_have_subscribers=must_have_subs,
                topic_appear_timeout_s=float(args.preflight_topic_timeout_s),
                echo_timeout_s=float(args.preflight_echo_timeout_s),
                estop_wait_s=float(args.preflight_estop_wait_s),
                require_rtk_fixed=not bool(args.no_rtk_gate),
                rtk_timeout_s=float(args.preflight_rtk_timeout_s),
            )

        # 3. Execute the single level
        try:
            rc = node.run_level(
                scenario_type=scenario_type,
                call=call,
                no_record=bool(args.no_record),
                record_startup_wait_s=float(args.record_startup_wait_s),
            )
        except Exception as e:
            node.get_logger().error(f"Aborted level '{call.name}': {e}")
            return 4

        if rc != 0:
            node.get_logger().error(f"Error: scenario '{call.name}' failed with exit code {rc}")
            return rc

        node.event("RUN_END")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())


