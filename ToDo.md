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

- [ ] Reconcile `~/agiles_ws` (typo) vs `~/agilex_ws` across the repo.
  - NUC confirmed source of truth: `~/agilex_ws`.
  - Files to edit: `env_sanitizer.sh`, `start_ROS.sh`.
  - Trivially unblocks the "standardize workspace path" Critical Blocker below.

After that, the first substantive task is:

- [ ] Replace the hardcoded `StepCurvaturePath` demo path in
  `scalecar-vfg-h-infinite/ros2_bridge/limo_path_follower/path_follower_node.py`
  with a runtime reference input. Candidates: `nav_msgs/msg/Path` subscription,
  a waypoint file loader, or a scenario-to-path adapter.

## Critical Blockers

- [x] Confirm the professor package is fully present and installable.
  - Done. Package vendored at `scalecar-vfg-h-infinite/`. Installable on the
    NUC with the prerequisites captured under "NUC deployment caveats" below.
- [ ] Standardize the robot workspace path and environment sourcing.
  - Answer is `~/agilex_ws`. Still need to fix `env_sanitizer.sh`,
    `start_ROS.sh`, and any other callers.
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
- [x] Keep the safety path in the loop for every autonomous test.
  - Topic wiring verified: `cmd_vel_raw` -> `estop_cli.py` -> `/cmd_vel`.
  - Full chain not yet exercised with wheels on the floor.

- [ ] Decide the minimum reference strategy for first bring-up.
  - Bare minimum smoke test: a temporary simple path is acceptable.
  - For anything repeatable: replace the hardcoded demo path with a real
    runtime reference.
  - Start in: `scalecar-vfg-h-infinite/ros2_bridge/limo_path_follower/path_follower_node.py`.

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

- [ ] Replace the hardcoded demo path with a runtime reference input.
- [ ] Extend bag recording with controller-specific topics (controller
  status, tracking error, active reference, optional debug command). Start
  in: `Data_Logger.py`.
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
