#!/usr/bin/env python3
"""Tiered QC runner for the H-infinity data-gathering system.

Default tiers are safe on a laptop. NUC/field tiers are present but gated so
they cannot accidentally start motion-capable checks.
"""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys


REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))


def run(cmd: list[str], *, cwd: str = REPO_ROOT, env: dict[str, str] | None = None) -> int:
    print("+ " + " ".join(cmd), flush=True)
    return subprocess.call(cmd, cwd=cwd, env=env)


def laptop_unit() -> int:
    env = dict(os.environ)
    env["PYTHONPATH"] = os.pathsep.join(
        p for p in [
            REPO_ROOT,
            os.path.join(REPO_ROOT, "scalecar-vfg-h-infinite"),
            env.get("PYTHONPATH", ""),
        ] if p
    )
    return run([sys.executable, "-m", "pytest", "-q", "tests/qc"], env=env)


def vfg_unit() -> int:
    env = dict(os.environ)
    env["PYTHONPATH"] = os.pathsep.join(
        p for p in [os.path.join(REPO_ROOT, "scalecar-vfg-h-infinite"), env.get("PYTHONPATH", "")] if p
    )
    return run(
        [sys.executable, "-m", "pytest", "-q"],
        cwd=os.path.join(REPO_ROOT, "scalecar-vfg-h-infinite"),
        env=env,
    )


def analysis_smoke() -> int:
    return run(["bash", "tools/analysis/tests/smoke.sh"])


def ros_sim() -> int:
    if shutil.which("ros2") is None:
        print("SKIP: ros2 is not on PATH. Run this tier on the sourced NUC.", file=sys.stderr)
        return 0
    tests = os.path.join(REPO_ROOT, "tools", "qc", "ros")
    if not os.path.isdir(tests):
        print("SKIP: no ROS QC tests installed yet.")
        return 0
    return run([sys.executable, "-m", "pytest", "-q", tests])


def field_gated(args: argparse.Namespace) -> int:
    if not args.confirm_wheels_on_floor:
        print(
            "REFUSE: field-gated tests require --confirm-wheels-on-floor. "
            "Default QC never moves the robot.",
            file=sys.stderr,
        )
        return 2
    if not args.confirm_pedestal:
        print(
            "REFUSE: field-gated pedestal motor test requires --confirm-pedestal.",
            file=sys.stderr,
        )
        return 2
    if not args.allow_motor_energize:
        print(
            "REFUSE: field-gated pedestal motor test requires --allow-motor-energize.",
            file=sys.stderr,
        )
        return 2
    return run([
        sys.executable,
        "tools/qc/pedestal_motor_test.py",
        "--confirm-pedestal",
        "--allow-motor-energize",
    ])


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Run tiered QC checks.")
    ap.add_argument(
        "tier",
        choices=["unit", "vfg", "analysis", "laptop", "ros-sim", "field-gated"],
        nargs="?",
        default="laptop",
    )
    ap.add_argument("--confirm-wheels-on-floor", action="store_true")
    ap.add_argument("--confirm-pedestal", action="store_true")
    ap.add_argument("--allow-motor-energize", action="store_true")
    args = ap.parse_args(argv)

    if args.tier == "unit":
        return laptop_unit()
    if args.tier == "vfg":
        return vfg_unit()
    if args.tier == "analysis":
        return analysis_smoke()
    if args.tier == "ros-sim":
        return ros_sim()
    if args.tier == "field-gated":
        return field_gated(args)

    # Laptop default: repo QC + vendored algorithm tests + synthetic analysis.
    for fn in (laptop_unit, vfg_unit, analysis_smoke):
        rc = fn()
        if rc != 0:
            return rc
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
