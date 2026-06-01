#!/usr/bin/env python3
"""CLI facade for AI/operator control of the LIMO orchestration system.

This is intentionally thin: it wraps the existing ROS topic API so callers do
not have to remember message shapes or scrape raw logs. Commands that can move
the robot are gated and default to dry-run behavior.
"""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
import time


ORCH_NAMES = {
    "base_vanilla",
    "base_gnss",
    "estop",
    "odom_zero",
    "follower",
    "reposition",
    "sequencer",
    # In-graph operator/LLM command facade (orchestrator_node.PROCS['ops'] ->
    # ros2 run limo_path_follower ops_node). Not a mover itself: it only
    # republishes onto /orchestrator/*, /experiment/cmd, /estop_trigger, so it
    # is deliberately NOT in the motion-capable refuse-set in cmd_start().
    "ops",
}


def _require_ros2() -> None:
    if shutil.which("ros2") is None:
        raise SystemExit("ros2 not found. Source ROS on the NUC before using limo_ops.py.")


def _ros2(args: list[str], timeout_s: float | None = None) -> subprocess.CompletedProcess:
    _require_ros2()
    try:
        return subprocess.run(
            ["ros2", *args],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=timeout_s,
            check=False,
        )
    except subprocess.TimeoutExpired as exc:
        return subprocess.CompletedProcess(
            ["ros2", *args],
            returncode=124,
            stdout=(exc.stdout or "") if isinstance(exc.stdout, str) else "",
            stderr=f"timeout after {timeout_s}s",
        )


def topic_pub_once(topic: str, type_name: str, payload: str) -> int:
    p = _ros2(["topic", "pub", "--once", topic, type_name, payload], timeout_s=5)
    if p.returncode != 0:
        print(p.stderr.strip() or p.stdout.strip(), file=sys.stderr)
    return p.returncode


def echo_once(topic: str, timeout_s: float) -> tuple[int, str]:
    p = _ros2(["topic", "echo", "--once", topic], timeout_s=timeout_s)
    return p.returncode, p.stdout.strip()


def cmd_status(args) -> int:
    p = _ros2(["node", "list"], timeout_s=5)
    print("nodes:")
    print(p.stdout.strip() or "(none)")
    rc, out = echo_once("/orchestrator/status", timeout_s=args.timeout)
    print("\norchestrator_status:")
    print(out if rc == 0 and out else "(no sample; status topic may be volatile or idle)")
    rc, out = echo_once("/experiment/status", timeout_s=1.0)
    print("\nexperiment_status:")
    print(out if rc == 0 and out else "(no sample)")
    return 0


def cmd_start(args) -> int:
    if args.name not in ORCH_NAMES:
        print(f"unknown orchestrator process '{args.name}'. Known: {sorted(ORCH_NAMES)}", file=sys.stderr)
        return 2
    if args.name in {"base_vanilla", "base_gnss", "follower", "reposition", "sequencer"} and not args.allow_motion_capable:
        print(
            f"REFUSE: '{args.name}' is motion-capable or can lead to motion. "
            "Pass --allow-motion-capable after confirming the robot state.",
            file=sys.stderr,
        )
        return 2
    return topic_pub_once("/orchestrator/start", "std_msgs/msg/String", f"data: {args.name}")


def cmd_kill(args) -> int:
    if args.name not in ORCH_NAMES:
        print(f"unknown orchestrator process '{args.name}'. Known: {sorted(ORCH_NAMES)}", file=sys.stderr)
        return 2
    return topic_pub_once("/orchestrator/kill", "std_msgs/msg/String", f"data: {args.name}")


def cmd_stop_all(_args) -> int:
    rc = 0
    for name in ("sequencer", "follower", "reposition"):
        rc = topic_pub_once("/orchestrator/kill", "std_msgs/msg/String", f"data: {name}") or rc
    rc = topic_pub_once("/estop_trigger", "std_msgs/msg/Bool", "data: true") or rc
    return rc


def cmd_preflight(_args) -> int:
    _require_ros2()
    return subprocess.call(["bash", "tools/preflight/preflight.sh"])


def _parse_rtk_quality(line: str) -> int | None:
    import re
    m = re.search(r"quality=(-?\d+)", line or "")
    return int(m.group(1)) if m else None


def cmd_wait_rtk_fixed(args) -> int:
    deadline = time.monotonic() + args.timeout
    last = ""
    while time.monotonic() < deadline:
        rc, out = echo_once("/gps_rtk_f9p_helical/gps/rtk_status", timeout_s=2.0)
        if rc == 0 and out:
            last = out
            if _parse_rtk_quality(out) == 4:
                print(out)
                return 0
        time.sleep(1.0)
    print("RTK FIXED not observed before timeout.", file=sys.stderr)
    if last:
        print("last_status:", last, file=sys.stderr)
    return 1


def cmd_explain_last_failure(_args) -> int:
    # First pass: collect the public status streams that should contain failure
    # reasons. This gives AI callers one command instead of multiple ros2 calls.
    for topic in ("/experiment/status", "/reposition/status", "/orchestrator/status"):
        rc, out = echo_once(topic, timeout_s=1.5)
        print(f"{topic}:")
        print(out if rc == 0 and out else "(no sample)")
    return 0


def cmd_run_smoke(args) -> int:
    if args.dry_run:
        print("DRY RUN: would start experiment sequencer with smoke config.")
        print("Use --armed only after field-gated QC confirmation.")
        return 0
    if not args.armed:
        print("REFUSE: run-smoke requires --dry-run or --armed.", file=sys.stderr)
        return 2
    return topic_pub_once("/orchestrator/start", "std_msgs/msg/String", "data: sequencer")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="AI/operator CLI for LIMO orchestration.")
    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("status")
    p.add_argument("--timeout", type=float, default=2.0)
    p.set_defaults(func=cmd_status)

    p = sub.add_parser("start")
    p.add_argument("name")
    p.add_argument("--allow-motion-capable", action="store_true")
    p.set_defaults(func=cmd_start)

    p = sub.add_parser("kill")
    p.add_argument("name")
    p.set_defaults(func=cmd_kill)

    sub.add_parser("stop-all").set_defaults(func=cmd_stop_all)
    sub.add_parser("preflight").set_defaults(func=cmd_preflight)

    p = sub.add_parser("wait-rtk-fixed")
    p.add_argument("--timeout", type=float, default=300.0)
    p.set_defaults(func=cmd_wait_rtk_fixed)

    sub.add_parser("explain-last-failure").set_defaults(func=cmd_explain_last_failure)

    p = sub.add_parser("run-smoke")
    g = p.add_mutually_exclusive_group(required=True)
    g.add_argument("--dry-run", action="store_true")
    g.add_argument("--armed", action="store_true")
    p.set_defaults(func=cmd_run_smoke)

    args = ap.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
