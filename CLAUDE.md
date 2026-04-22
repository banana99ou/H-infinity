# CLAUDE.md

Context for Claude Code when working in this repo.

## What this repo is

Hardware bring-up of the professor's `scalecar-vfg-h-infinite` path-following
controller (VFG + LPV-Hinf) on the AgileX LIMO platform. **The robot is the
source of truth** — this laptop has no ROS2 installed. Nothing runs locally;
everything runs on the NUC.

Authoritative docs:
- `DOC/project_spec.md` — runtime interface contract, safety requirements.
- `ToDo.md` — working checklist, current status, deployment caveats.

## Dev cycle

Every change follows the same loop:

1. **Edit on laptop** (`/Volumes/Sandisk/code/H-infinity/`). Use Read/Edit/Write.
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
- SSH password is handled via `expect` (sshpass is not installed on the laptop).
  The literal password is not stored in this repo; it is shared per-session.
- NUC repo path: `/home/agilex/H-infinity/` (outside the colcon tree).
- NUC workspace: `/home/agilex/agilex_ws/` (ROS2 Humble).
- The package is visible to colcon via this symlink — recreate if it is ever
  missing:
  ```
  ln -sfn ~/H-infinity/scalecar-vfg-h-infinite/ros2_bridge \
          ~/agilex_ws/src/limo_path_follower
  ```

## SSH pattern

`ssh` will not accept a password on argv; `sshpass` is not installed. Use
`expect` (already permission-granted for this repo):

```bash
expect -c '
set timeout 30
spawn ssh agilex@agilex-nuc12wski7 {bash -lc "COMMAND_HERE"}
expect { -re "(P|p)assword:" { send "PASSWORD\r"; exp_continue } eof }
'
```

For anything non-trivial, write a script to `/tmp/foo.sh` on the laptop,
rsync it, then `bash /tmp/foo.sh` over ssh. Avoids quoting hell. Do **not**
use `set -u` in such scripts — `/opt/ros/humble/setup.bash` references unset
variables and will abort.

## Rsync pattern

Laptop → NUC, working-tree mirror (no `--delete` unless you are certain the
NUC has nothing unique):

```bash
expect -c '
set timeout 120
spawn rsync -avz \
  --exclude=.git --exclude=.DS_Store --exclude=__pycache__ --exclude=*.pyc \
  --exclude=.specstory --exclude=.vscode \
  --exclude=build --exclude=install --exclude=log --exclude=.pytest_cache \
  /Volumes/Sandisk/code/H-infinity/ \
  agilex@agilex-nuc12wski7:/home/agilex/H-infinity/
expect { -re "(P|p)assword:" { send "PASSWORD\r"; exp_continue } eof }
'
```

Prefer `--dry-run` first to confirm the change set before pushing.

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
Canonical list lives in `ToDo.md` under "NUC deployment caveats". When
setting up a new robot, read that section first.

## Commit conventions for this repo

- No `Co-Authored-By: Claude` trailer. No AI attribution in commit messages.
- Prefer a short subject + body that explains the *why*. Keep the vendor
  drop of `scalecar-vfg-h-infinite/` isolated from our patches so future
  upstream drops diff cleanly (see commits `f7a34a2` and `b577541` for the
  pattern).
- Do not touch `DOC/project_spec.md` unless the user asks — it has
  pre-existing unstaged edits that belong to the user's own flow.
