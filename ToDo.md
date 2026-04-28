# ToDo

Working checklist for getting the professor-provided H-infinity stack from
`scalecar-vfg-h-infinite` running on the real AgileX LIMO.

## Status

First hardware bring-up is green. `ros2 run limo_path_follower path_follower_node`
launches on the NUC, subscribes to `/wheel/odom`, publishes `/cmd_vel_raw`, and
responds correctly to synthetic odometry. Safety fallbacks (null-odom,
odom-timeout, shutdown) verified in isolation. No actual wheel motion has been
tested yet.

## Key contract to preserve

- controller consumes live robot feedback and a runtime reference
- controller publishes `cmd_vel_raw`
- `estop_cli.py` remains between the controller and final `cmd_vel`

## Next Thing To Implement

- [x] Reconcile `~/agiles_ws` (typo) vs `~/agilex_ws` across the repo.
  - NUC confirmed source of truth: `~/agilex_ws`.
  - `env_sanitizer.sh` and `start_ROS.sh` already use `~/agilex_ws` — no
    edits required. Remaining `agile_ws` / `agiles_ws` mentions live only
    in `DOC/project_spec.md`, which is owned by the user.

The first substantive task is done:

- [x] Replace the hardcoded `StepCurvaturePath` demo path in
  `scalecar-vfg-h-infinite/ros2_bridge/limo_path_follower/path_follower_node.py`
  with a runtime reference input.
  - Implemented: `nav_msgs/msg/Path` subscription on `/reference_path`
    (parameter `reference_path_topic`), latched QoS (transient_local +
    reliable + keep_last 1) so a one-shot publisher works.
  - Waypoints converted to `BezierPath`; guidance is rebuilt on each new
    message. `_delta_prev` is reset on swap.
  - Frame-id of incoming path is checked against the `odom_frame`
    parameter (default `odom`); mismatch logs a warn (no transform
    applied — see frame-consistency task below).
  - Demo path retained behind `use_demo_path:=true` for smoke tests.
  - Verified on NUC: zero cmd while no path, zero cmd while path-but-no-odom
    (odom-timeout branch), non-zero cmd with both (correct sign for an
    offset-left pose vs. straight x-axis path).

## Critical Blockers

- [ ] **Set the LIMO to Ackermann steering mode before any wheels-on-floor test.**
  - Why it matters: the controller does the bicycle-model conversion
    `omega = v * tan(delta) / L` on the assumption the LIMO is in Ackermann
    mode. In differential / 4WD mode, `cmd_vel.angular.z` is interpreted
    as a wheel-speed-difference command and the kinematics no longer match
    what the controller is solving for.
  - Where in code the assumption lives: `path_follower_node.py:179`
    (`omega = v * math.tan(delta_cmd) / self.wheelbase`).
  - **How to set it (confirmed):** mode is set on the LIMO chassis itself
    (physical wheel configuration + mode switch on the robot). The
    `limo_base` driver auto-detects whatever mode the chassis reports
    over CAN — there is no ROS service or parameter that changes it.
    Source: `limo_ros2/limo_base/src/limo_driver.cpp:271` reads
    `motion_mode_ = frame.data[6]` from the CAN status frame; mode
    constants in `limo_protocol.h` (`MODE_ACKERMANN = 0x01`).
  - **How to verify at runtime** (after `start_ROS.sh` is up):
    ```
    ros2 topic echo /limo_status --once
    ```
    Look for `motion_mode: 1`. Anything else (4 = MCNAMU/Mecanum, etc.)
    means the chassis is not in Ackermann — fix it on the robot before
    enabling autonomous control.
  - Add this check to the wheels-on-floor preflight.


- [x] Confirm the professor package is fully present and installable.
  - Done. Package vendored at `scalecar-vfg-h-infinite/`. Installable on the
    NUC with the prerequisites captured under "NUC deployment caveats" below.
- [x] Standardize the robot workspace path and environment sourcing.
  - Answer is `~/agilex_ws`. `env_sanitizer.sh` and `start_ROS.sh` are
    already correct. Remaining stale references are doc-only in
    `DOC/project_spec.md` (user-owned).
- [ ] Freeze the runtime interface before larger edits.
  - Decide: odometry topic (answered: `/wheel/odom`), reference input type
    (still open), diagnostic topics (still open), whether steering telemetry
    exists (still open).
  - Source of truth: `DOC/project_spec.md`.

## Before First Motion

- [x] Rewire the wrapper node to the current LIMO stack.
  - `/odom` -> `/wheel/odom`
  - `/cmd_vel` -> `cmd_vel_raw`
  - Robot remains the plant; no sim physics ported into ROS.
  - 2026-04-28: verified live against the real robot. `path_follower_node`
    subscribes to `/wheel/odom` from `limo_base_node` (49.9 Hz),
    publishes to `/cmd_vel_raw`. Zero command without path; non-zero
    command with path + real odom. `/limo_status.motion_mode = 1`
    (Ackermann). `estop_cli.py` was intentionally absent so no command
    reached the wheels.
  - 2026-04-28: full H-inf pipeline run end-to-end with wheels OFF the
    ground. estop_cli bypassed via runtime remap
    (`--ros-args --remap cmd_vel_raw:=/cmd_vel`); no code change.
    Real odom -> LPV-Hinf -> /cmd_vel -> limo_base_node. Commands
    saturated at `omega = v*tan(delta_max)/L = 2.73 rad/s` because the
    hardcoded test path (0,0)->(4,0) didn't match the robot's actual
    odom pose (x=0.27, y=-2.08, yaw=-0.72). Numbers are physically
    correct for that geometry; controller is responding to real
    odometry, math checks out.
- [x] Keep the safety path in the loop for every autonomous test.
  - Topic wiring verified: `cmd_vel_raw` -> `estop_cli.py` -> `/cmd_vel`.
  - Full chain not yet exercised with wheels on the floor.

- [x] Decide the minimum reference strategy for first bring-up.
  - Decided: runtime `nav_msgs/msg/Path` on `/reference_path`, latched QoS.
    Demo `StepCurvaturePath` retained behind `use_demo_path:=true`.

- [ ] Confirm frame consistency between odometry and reference.
  - Why: path following will look broken if frames are mismatched even when
    the controller is fine.
  - Start in: wrapper node plus whichever reference publisher is chosen.

- [ ] Set conservative first-motion parameters.
  - Low `v_const`, realistic wheelbase, tight steering limit, conservative
    `dt_ctrl`.
  - Start in: `scalecar-vfg-h-infinite/ros2_bridge/limo_path_follower/path_follower_node.py`,
    `scalecar-vfg-h-infinite/vfg_pathfollowing/controllers/lpv_hinf.py`.

- [ ] Document whether steering feedback is real or approximated.
  - Current bridge uses the previous command as `delta_meas`.
  - Acceptable for first tests, but it is not the same as measured steering.

- [ ] Run Stage 0 safety validation before any path-following test.
  - No odom -> zero command
  - Missing reference -> safe behavior
  - Shutdown -> zero command
  - E-stop override works every time
  - Source of truth: `DOC/project_spec.md`.

- [ ] Run one low-speed straight-path test before any curved-path test.
  - Goal: prove the wrapper, safety chain, and command mapping are not
    obviously wrong.

## Before Repeatable Experiments

- [x] Replace the hardcoded demo path with a runtime reference input.
  (See "first substantive task" above.)
- [ ] Extend bag recording with controller-specific topics (controller
  status, tracking error, active reference, optional debug command). Start
  in: `Data_Logger.py`.
- [ ] **Log GNSS data alongside controller runs** (professor's request,
  for his other project). `GPS-RTK_ROS2_pub_node.py` exists in the repo;
  confirm what topic it publishes and add it to `Data_Logger.py`'s bag
  list. Decide whether GNSS is required (block run) or best-effort
  (warn but continue) for controller experiments.
- [ ] Decide RTK gating policy for controller experiments. Start in:
  `run_scenarios_from_files.py`.
- [ ] Extend preflight topic checks for controller runs. Start in:
  `run_scenarios_from_files.py`.
- [ ] Decide whether the current INI scenario system should be extended or
  wrapped. Start in: `run_scenarios_from_files.py`, `scenarios/`.
- [ ] Define a run-ID or timestamp rule for associating ROS bags with the
  external Ohcoach-cell dataset. Source of truth: `DOC/project_spec.md`.

## Validation And Tuning

- [ ] Validate actual LIMO vehicle assumptions against sim (wheelbase,
  steering saturation, safe speed envelope, effective control period,
  whether `cmd_vel.angular.z` behaves consistently with the bicycle-model
  conversion).
- [ ] Tune only after the interface is correct. Suggested order:
  `dt_ctrl` -> `delta_max` -> `output_gain` -> `K_ff` -> `rho_scale`.
  Start in: `scalecar-vfg-h-infinite/vfg_pathfollowing/controllers/lpv_hinf.py`.
- [ ] Compare robot logs against simulation expectations after each stage.
  If behavior is structurally wrong at conservative speeds, consider model
  mismatch or controller re-synthesis.

## NUC deployment caveats

These four things are non-obvious and will re-bite anyone setting up a fresh
NUC. Keep them in mind before declaring a runtime environment "ready".

1. **Professor package shipped without `setup.cfg`.** Without the
   `[install] install_scripts=$base/lib/limo_path_follower` redirect, modern
   setuptools installs `console_scripts` into `install/<pkg>/bin/` instead of
   `install/<pkg>/lib/<pkg>/`, and `ros2 run limo_path_follower
   path_follower_node` returns "No executable found". Our fix added
   `scalecar-vfg-h-infinite/ros2_bridge/setup.cfg`. Every other ament_python
   package in `~/agilex_ws/src/` has the same pattern.

2. **`vfg_pathfollowing` is a hard prerequisite, not declared anywhere.**
   The ROS package's `package.xml` only lists `rclpy`, `nav_msgs`,
   `geometry_msgs`. The node imports `vfg_pathfollowing` at the top, so on
   a fresh NUC you must:
   ```
   pip3 install --user ~/H-infinity/scalecar-vfg-h-infinite/
   ```
   before `ros2 run` will succeed. If you skip this the node crashes with
   `ModuleNotFoundError: No module named 'vfg_pathfollowing'`.

3. **`setuptools==68.2.2` is pinned in `~/.local` on the NUC.** Versions
   >= 70 place `console_scripts` in `bin/` regardless of `setup.cfg` and
   silently break `ros2 run` for **all** ament_python packages on the
   workspace. If pip or a system update moves it, reinstall:
   ```
   pip3 install --user --force-reinstall setuptools==68.2.2
   ```

4. **Repo lives outside the colcon tree.** The clone is at `~/H-infinity/`;
   the package is made visible to colcon via a symlink:
   ```
   ln -sfn ~/H-infinity/scalecar-vfg-h-infinite/ros2_bridge \
           ~/agilex_ws/src/limo_path_follower
   ```
   Recreate the symlink if the repo moves.

## Known open noise

- [ ] Three `/path_follower_node` instances were visible on the DDS graph
  during testing. Cause not confirmed: possibly orphaned test processes, or
  the duplicate `path_follower_pkg` directories already present in
  `~/agilex_ws/src/`. Investigate before running alongside `limo_bringup`.

## Optional / Later

- [ ] Fold the controller node into a unified launch flow after manual
  bring-up is stable.
- [ ] Add stronger diagnostics if debugging is slow (examples: `psi_des`,
  `kappa`, `rho`, pre-saturation steering command, path progress).
- [ ] Add offline analysis scripts for repeated metric extraction from bags.

## Open Questions / Assumptions

- [x] Is `scalecar-vfg-h-infinite` fully checked out on the build machine?
  Yes - vendored into this repo at commit A; rsync'd onto the NUC at
  `~/H-infinity/scalecar-vfg-h-infinite/`.
- [ ] Does the real LIMO expose steering-angle telemetry, or must
  `delta_meas` remain approximated?
- [ ] Is `/wheel/odom` sufficient for the first hardware phase, or is a
  fused pose needed for evaluation quality?
- [ ] For the very first smoke test, will a temporary hardcoded path be
  accepted, or should runtime path input be implemented immediately?
