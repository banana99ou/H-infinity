import subprocess
import sys


def test_run_smoke_dry_run_never_requires_ros():
    p = subprocess.run(
        [sys.executable, "tools/ops/limo_ops.py", "run-smoke", "--dry-run"],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    assert p.returncode == 0
    assert "DRY RUN" in p.stdout


def test_start_motion_capable_process_is_gated_without_ros():
    p = subprocess.run(
        [sys.executable, "tools/ops/limo_ops.py", "start", "follower"],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    assert p.returncode == 2
    assert "motion-capable" in p.stderr
