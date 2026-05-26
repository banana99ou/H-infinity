# CLAUDE.md

Context for Claude Code when working in this repo.

## What this repo is

Hardware bring-up of the professor's `scalecar-vfg-h-infinite` path-following
controller (VFG + LPV-Hinf) on the AgileX LIMO platform. **The robot is the
source of truth** — this laptop has no ROS2 installed. Nothing runs locally;
everything runs on the NUC.

Authoritative docs:
- `DOC/system_spec.md` — locked requirements (ROC), canonical interface
  contract (§4), acceptance criteria. The "what it must be".
- `DOC/experiment.md` — claim pivot, experimental matrix, operational model,
  orchestrator target state. The "why + how".
- `DOC/deployment.md` — NUC caveats, support-script inventory, runbook.
- `ToDo.md` — working checklist, current status, next-session list.
- `README.md` — repo map + index of which doc owns what.

## Dev cycle

Every change follows the same loop:

1. **Edit on laptop** (`/Users/hyeon-yongjeong/code/H-infinity/`). Use Read/Edit/Write.
   Do not try to run ROS2 commands here — they do not exist on macOS.
2. **Rsync to NUC** (see `## Rsync pattern`). Laptop and NUC drift constantly.
   Assume they are out of sync until you have just rsync'd.
3. **SSH in and build/run** (see `## SSH pattern`, `## Build & run`).
4. **Verify on the robot graph** (see `## Verify pattern`).
5. **Report results with evidence** — log excerpts, topic output. Do not
   claim "done" from static checks alone (grep, AST parse). See
   `memory/feedback_verify_runtime.md`.

If a change needs multiple iterations, repeat 1–4. Do not batch edits without
verifying in between.

## Robot

- Host: `agilex-nuc12wski7`, user `agilex`.
- SSH/rsync use `expect` (`sshpass` is not installed on the laptop). The
  `agilex` user/sudo password is **documented in this repo** at
  `DOC/network_topology.md` (not held only per-session, as previously stated
  here). Note: SSH may also authenticate via **Tailscale SSH**, in which case
  the `expect`-sent password is a no-op. To honor a no-secrets-in-repo policy
  instead, scrub it from `network_topology.md` and share it per-session.
- NUC repo path: `/home/agilex/H-infinity/` (outside the colcon tree).
- NUC workspace: `/home/agilex/agilex_ws/` (ROS2 Humble).
- The package is visible to colcon via this symlink — recreate if it is ever
  missing:
  ```
  ln -sfn ~/H-infinity/scalecar-vfg-h-infinite/ros2_bridge \
          ~/agilex_ws/src/limo_path_follower
  ```

## SSH / rsync / sync

SSH and rsync to the NUC are **passwordless via Tailscale SSH** — plain
`ssh agilex@agilex-nuc12wski7 '<cmd>'` and `rsync` work with no password and no
`expect`. Add `-o BatchMode=yes` to fail fast rather than hang if Tailscale auth
is ever unavailable.

To sync the repo, prefer the wrapper (`tools/sync/sync.sh`):

```bash
tools/sync/sync.sh push   # laptop code/docs -> NUC  (laptop is canonical)
tools/sync/sync.sh pull   # NUC run artifacts ("Experiment Data/") -> laptop
tools/sync/sync.sh push-dry | pull-dry    # rsync --dry-run preview first
```

Direction of truth: code/docs only ever go **laptop → NUC**; run artifacts
(rosbags + sidecars under `Experiment Data/`, gitignored) only ever come
**NUC → laptop**. Push is additive (no `--delete`) so NUC-unique files survive.
This is the interim mechanism; the long-term plan is a shared git remote both
machines push/pull for code, plus this tool for the large artifacts.

**Fallback** (only if Tailscale SSH is down): `sshpass` is not installed and the
agilex password lives in `DOC/network_topology.md`; pass it via `expect`:

```bash
expect -c '
set timeout 30
spawn ssh agilex@agilex-nuc12wski7 {bash -lc "COMMAND_HERE"}
expect { -re "(P|p)assword:" { send "PASSWORD\r"; exp_continue } eof }
'
```

For anything non-trivial over the fallback, write a script to `/tmp/foo.sh`,
rsync it, then `bash /tmp/foo.sh` over ssh (avoids quoting hell). Do **not** use
`set -u` in such NUC scripts — `/opt/ros/humble/setup.bash` references unset
variables and will abort.

## Build & run

Always source in this order on the NUC:

```bash
source /opt/ros/humble/setup.bash
source /home/agilex/agilex_ws/install/setup.bash
```

Build (our package only):

```bash
cd /home/agilex/agilex_ws
colcon build --packages-select limo_path_follower
```

Do not use `--symlink-install` for this package: it has caused entry-point
resolution issues. A plain rebuild is fine; edits reach the NUC via rsync,
not via symlinked source.

Run:

```bash
ros2 run limo_path_follower path_follower_node
```

## Verify pattern

Topology:

```bash
ros2 node list
ros2 node info /path_follower_node
ros2 topic info /wheel/odom --verbose
ros2 topic info /cmd_vel_raw --verbose
```

Synthetic input when the real feedback stack is not running:

```bash
ros2 topic pub --rate 30 /wheel/odom nav_msgs/msg/Odometry \
  '{pose: {pose: {position: {x: 1.0, y: 1.0, z: 0.0}, \
   orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}, \
   twist: {twist: {linear: {x: 0.5, y: 0.0, z: 0.0}}}}'
```

Watch the output:

```bash
ros2 topic echo /cmd_vel_raw
```

Under an offset pose with `yaw=0`, expect non-zero `angular.z`. Under no odom
or silence > 0.5 s, expect all zeros (odom-timeout branch).

## Safety contract

- The only permitted output path from the controller is
  `cmd_vel_raw → estop_cli.py → /cmd_vel`. Never publish to `/cmd_vel`
  directly from a controller node. This is verified by `git diff` rejection
  of any `/cmd_vel` string in controller code.
- Before any test with the wheels on the floor: confirm with the user.
  Synthetic odom tests with wheels off the ground are fine without asking.
- Never push to `origin` without explicit user request.

## Deployment caveats

Four non-obvious prerequisites on a fresh NUC — missing `setup.cfg` fix,
`vfg_pathfollowing` pip install, `setuptools==68.2.2` pin, colcon symlink.
Canonical list lives in `DOC/deployment.md`. When setting up a new robot,
read that first.

## Commit conventions for this repo

- No `Co-Authored-By: Claude` trailer. No AI attribution in commit messages.
- Prefer a short subject + body that explains the *why*. Keep the vendor
  drop of `scalecar-vfg-h-infinite/` isolated from our patches so future
  upstream drops diff cleanly (see commits `f7a34a2` and `b577541` for the
  pattern).
- `DOC/system_spec.md` is the locked spec: if reality and the sheet disagree,
  fix the sheet first (per its own header). Don't silently diverge from it.
