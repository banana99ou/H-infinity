# QC Harness

Repo-owned quality-control entrypoint for the paper data-gathering system.

Default laptop-safe run:

```bash
python3 tools/qc/run_qc.py laptop
```

Tiers:

- `unit` — pure Python contract tests in `tests/qc`.
- `vfg` — vendored `vfg_pathfollowing` pytest suite with `PYTHONPATH` set.
- `analysis` — synthetic bag -> manifest/QC/eval/dataset smoke.
- `laptop` — `unit` + `vfg` + `analysis`.
- `ros-sim` — NUC-only ROS tests, no robot motion. Skips if `ros2` is absent.
- `field-gated` — requires `--confirm-wheels-on-floor`; this pass only gates
  the harness and refuses by default.

Default QC must never move the robot. Motion-capable work belongs behind an
explicit operator confirmation flag.
