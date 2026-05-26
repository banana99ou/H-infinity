# ToDo

Working checklist for getting the professor-provided H-infinity stack from
`scalecar-vfg-h-infinite` running on the real AgileX LIMO.

## Status

Battle station online and field-tested. Major workstreams since last update
(2026-04-30):

- **Browser battle station** (`tools/path_gen/interactive.html`):
  Leaflet map + parametric path designer + rosbridge live link + run controls
  (Push&Run/Pause/Resume/Stop/Recenter), speed slider via `set_parameters`,
  inline keyboard teleop with WASD + reverse-turn fix, last-curve persistence
  via localStorage.
- **Process orchestrator** (`scalecar-vfg-h-infinite/ros2_bridge/limo_path_follower/orchestrator_node.py`):
  manages `base_vanilla` / `base_gnss` / `estop` / `follower` subprocesses
  via `/orchestrator/{start,kill,status}` topics. Stack panel in the HTML
  shows live state and start/kill buttons.
- **systemd service** (`tools/orchestrator/limo-battle.service` +
  `install_service.sh`): boots rosbridge + orchestrator on NUC startup. Enabled
  and verified surviving reboot.
- **Telemetry**: `/path_follower/status` (Float32MultiArray) added to
  `path_follower_node`; `v_const` runtime-tunable via parameter callback;
  `/path_follower/reset` Bool subscriber clears the loaded path.
- **Indoor test preset**: `indoor_s` family in the path designer
  (3 m × 0.4 m S-curve, R_min ≥ 0.57 m).
- **`estop_cli.py`** subscribes to `/estop_trigger` (browser-driven E-stop)
  and is now TTY-tolerant (works under systemd without a terminal).
- **Field network topology** documented in `DOC/network_topology.md`
  (phone-on-robot + USB-tether + LIMO-AP). Used during outdoor GNSS
  recording session.
- **RTK basestation broadcaster ported** (`tools/rtk_base/`, 2026-05-26):
  Linux-ready `rtcm_server.py` + systemd unit + udev rule + installer +
  README. Replaces the MacBook-bound `agile_ws/rtcm_server.py` so the
  operator no longer has to stay at the rooftop for unattended runs. **Not
  yet deployed on a real Pi** — first deployment + bench soak is now the
  blocking next step before any rooftop run can produce RTK-FIXED data.

First wheels-on-floor controller test still hasn't happened. The infrastructure
is ready; next session is the first real run.

## Architectural decisions (committed)

- **GPS stays out of the control loop.** See `DOC/decisions/01_gps_no_fusion.md`.
  Wheel odom feeds the controller; RTK GPS is logged as semi-ground-truth
  for post-hoc evaluation. No `robot_localization` EKF, no fused state.

## Key contract to preserve

- controller consumes live robot feedback and a runtime reference
- controller publishes `cmd_vel_raw`
- `estop_cli.py` remains between the controller and final `cmd_vel`
- GPS is **not** in the control loop (see ADR-01)

## Next session — start here

In rough priority order:

1. **First wheels-on-floor smoke test (indoor).** Indoor S-curve preset is
   ready. Bring up `base_vanilla` + `estop` + `follower` from the orchestrator,
   push the `indoor_s` path, set `v_const = 0.2 m/s`, clear the E-stop, and
   verify the robot tracks the curve. Confirm with the user before letting
   wheels touch the ground.

2. **Verify `base_gnss` actually publishes.** Start `base_gnss` from the
   orchestrator and run `ros2 topic list` / `ros2 topic echo --once` to
   confirm `/gps_rtk_f9p_helical/gps/fix` and `.../rtk_status` flow. The
   `LIMO+MAVROS+RTK_Node_Launcher.launch.py` launch file exists in
   `limo_base/launch/` but its end-to-end behavior on this hardware has
   not been verified post-systemd-install.

3. **Deploy + soak-test the RTK basestation** (`tools/rtk_base/`). Blocking
   precondition for anything producing RTK-FIXED in the field.
   **Day-of runbook:** [`DOC/rtk_base_deploy.md`](DOC/rtk_base_deploy.md) —
   linear checklist, go/no-go per phase, troubleshooting table. Read that
   first; the bullets below are the summary view.
   - Image a Pi (Raspberry Pi OS Bookworm), `git clone H-infinity`,
     `tools/rtk_base/README.md` has the full recipe.
   - Add LIMO_AP to the Pi's NetworkManager profiles with a **static IPv4
     of `10.42.0.170/24`** (matches the rover-side hardcode at
     `GPS-RTK_ROS2_pub_node.py:52`).
   - Run `tools/rtk_base/install.sh`. Plug in the base F9P, confirm
     `/dev/f9p_base` symlink, `sudo systemctl start rtk-base`,
     `journalctl -u rtk-base -f` should show `[reader] starting` and
     `[status] alive=True`.
   - Configure the base F9P once in u-center: **TMODE3 Survey-In** (60 s
     min, 5 m accuracy), RTCM3 1005/1077/1087/1097/1127/1230 on USB at
     1 Hz, save to flash. (Switch to Fixed-mode TMODE3 later when V1
     reproducibility matters.)
   - **Bench soak (4+ h):** Pi from the power bank, F9P with antenna at
     window, NUC running `GPS-RTK_ROS2_pub_node.py`. Watch journal +
     `ros2 topic echo /gps_rtk_f9p_helical/gps/rtk_status`. Provoke:
     yank F9P USB → replug; yank Pi WiFi → rejoin; reboot Pi. All three
     must recover unattended.
   - **Rooftop soak (2 h):** real survey-in, drive the LIMO manually
     through the corners of the intended working area, confirm `quality=4`
     sustained. Note any geometric dropouts (WiFi range marginality).
   - Follow-ups after deployment (not blocking the first soak):
     - Make `TCP_HOST` a CLI flag in `GPS-RTK_ROS2_pub_node.py` (drop the
       hardcoded `10.42.0.170`), and/or add mDNS resolution for
       `rtk-base.local` on the rover side.
     - Add a `/rtk_base/health` topic or ntfy push on `alive` transitions
       (M1/F2 in `DOC/system_spec.md`). Right now the journal is the only
       signal that the base is alive.

4. **Battle-station GPS view (per ADR-01 §"What we DO use GPS for").**
   Add to `interactive.html`:
   - Subscribe to `/gps_rtk_f9p_helical/gps/fix` (NavSatFix) and
     `.../rtk_status`
   - Big colored fix-quality bar: RTK FIX (green) / RTK FLOAT (yellow) /
     3D (orange) / NO FIX (red)
   - Magenta circle marker on the Leaflet map at the GPS lat/lon (distinct
     from the blue odom-projected marker, so drift is visible)
   - NTRIP correction-age display
   - Refuse Push&Run when fix < FLOAT (configurable threshold)

5. **Rosbag recording from the battle station.** Add a "Record run" button
   that starts/stops `ros2 bag record` for the minimum dataset
   (`DOC/system_spec.md` §4 required bag topic set):
   `/wheel/odom`, `/cmd_vel_raw`, `/cmd_vel`, `/estop`, `/reference_path`,
   `/path_follower/status`, `/gps_rtk_f9p_helical/gps/fix`,
   `/gps_rtk_f9p_helical/gps/rtk_status`. Bag filename = run-ID +
   wallclock timestamp; embed the same in a sidecar JSON for pairing
   with the FitTogether SD-card export.

6. **Offline analysis script** (`tools/analysis/run_eval.py`): given a
   rosbag, compute the headline table from ADR-01:
   `RMS e_d`, `max e_d`, terminal pose error — once from `/wheel/odom`,
   once from `/gps_rtk_f9p_helical/gps/fix`. Print the gap. Plot both
   trajectories overlaid on the reference.

7. **Captive `cmd_vel_raw` watchdog in `estop_cli.py`** (defense in depth):
   trip the latched E-stop after 1 s of `/cmd_vel_raw` silence. Catches
   any source going silent (teleop browser dies, follower crashes).

8. **Field-readiness checklist on the NUC**: small script that verifies
   pre-rooftop-trip everything is healthy (limo-battle.service active,
   internet reachable, both base launch files importable, etc.).
   - 2026-05-07: quick mode landed at `tools/preflight/preflight.sh`.
     Covers service state, node graph, safety chain pub/sub sets,
     Ackermann mode, battery, `/wheel/odom` liveness. 11/11 PASS on NUC.
   - Still TODO: `--full` dynamic test (synthetic odom + path push +
     E-stop reflex assertion). Stub WARNs in place.
   - Still TODO: outdoor-only checks (internet reachable, both base
     launch files importable).

Open questions for the user before next session:

- Sudo password for the agilex user is `agx` (separate from SSH password
  `nvidia`). Confirm this stays valid.
- LIMO AP config — is it persistent in NetworkManager? Do we need to
  script its setup as part of the orchestrator?
- For the GPS-quality threshold gate: hard-fail at FLOAT, or only at
  SINGLE / NO FIX?

## Next Thing To Implement

- [x] Reconcile `~/agiles_ws` (typo) vs `~/agilex_ws` across the repo.
  - NUC confirmed source of truth: `~/agilex_ws`.
  - `env_sanitizer.sh` and `start_ROS.sh` already use `~/agilex_ws` — no
    edits required. Remaining stale `agile_ws` / `agiles_ws` mentions lived
    only in the old root project spec, now deleted.

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

- [x] **Set the LIMO to Ackermann steering mode before any wheels-on-floor test.**
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
    already correct. The old root project spec's stale references are gone
    with it (file deleted).
- [ ] Freeze the runtime interface before larger edits.
  - Decide: odometry topic (answered: `/wheel/odom`), reference input type
    (still open), diagnostic topics (still open), whether steering telemetry
    exists (still open).
  - Source of truth: `DOC/system_spec.md` §4.

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
  - Source of truth: `DOC/system_spec.md` §5–6.

- [ ] Run one low-speed straight-path test before any curved-path test.
  - Goal: prove the wrapper, safety chain, and command mapping are not
    obviously wrong.

## Before Repeatable Experiments

- [x] Replace the hardcoded demo path with a runtime reference input.
  (See "first substantive task" above.)
- [ ] Extend bag recording with controller-specific topics. Start in:
  `Data_Logger.py` (TOPICS list at line 30). Currently records
  `/cmd_vel`, `/cmd_vel_raw`, `/wheel/odom`, `/imu`, `/estop`. Still
  missing: `/reference_path` (trivial -- just add the string), and
  controller-internal diagnostics (`psi_des`, `e_d`, `kappa`,
  `s_star`, pre-saturation `delta_cmd`) which `path_follower_node` does
  not currently publish. Decide whether to add a single
  `/path_follower/status` topic or one topic per quantity.
- [x] **Log GNSS data alongside controller runs** (professor's request).
  Already wired in `Data_Logger.py:30-35`: records
  `/gps_rtk_f9p_helical/gps/{fix,nmea,rtk_status}` plus the Pixhawk
  GPS topics. To verify: run `Data_Logger.py` during a controller test
  and confirm all six GPS topics appear in the resulting bag with
  non-empty messages. Open question for professor: format/ordering
  preferences, RTK-fix gating policy.
- [ ] Decide RTK gating policy for controller experiments. Start in:
  `run_scenarios_from_files.py`.
- [ ] Extend preflight topic checks for controller runs. Start in:
  `run_scenarios_from_files.py`.
- [ ] Decide whether the current INI scenario system should be extended or
  wrapped. Start in: `run_scenarios_from_files.py`, `scenarios/`.
- [ ] Define a run-ID or timestamp rule for associating ROS bags with the
  external Ohcoach-cell dataset. Source of truth: `DOC/system_spec.md` §4 (D2–D3).

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

Moved to `DOC/deployment.md` (the four fresh-NUC prerequisites: `setup.cfg`
fix, `vfg_pathfollowing` pip install, `setuptools==68.2.2` pin, colcon
symlink). Read that before setting up a new robot.

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
