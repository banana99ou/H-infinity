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
     - **Rover RTK node stale-socket / no-reconnect bug** (observed
       2026-05-29 field session). When `GPS-RTK_ROS2_pub_node.py` is started
       before the Pi is reachable at `10.42.0.170:2101` (Pi still on
       KMU_WiFi, or LIMO_AP briefly down due to phone-hotspot heat-drop),
       the node enters a stale-TCP state: gets a brief burst of bytes, the
       socket dies, and it never reconnects. Rover reports
       `RTCM: STALE (fwd_age=...)` indefinitely; `quality` stays at 1
       even after the Pi reaches the right IP. Temp workaround that worked
       in the field: `kill <pid>` the running node, then restart with
       `python3 /home/agilex/agilex_ws/GPS-RTK_ROS2_pub_node.py
       --ros-args -r __ns:=/gps_rtk_f9p_helical`. Long-term fix in
       `GPS-RTK_ROS2_pub_node.py` around the TCP socket setup: enable
       `SO_KEEPALIVE` with short intervals; add a watchdog that closes +
       reopens the socket if no bytes arrive for >N seconds; reconnect loop
       with exponential backoff on initial failure too (so starting before
       Pi is ready is recoverable). Pairs naturally with the existing
       follow-up to drop the `10.42.0.170` hardcode.
     - **`LIMO+MAVROS+RTK_Node_Launcher.launch.py` missing
       `respawn=True`** for the GPS-RTK node. When the rover RTK process
       dies (whether from the workaround above or anything else), the
       launcher does not restart it — `ros2 node list` simply loses the
       node and manual `python3 ...` is required. Add
       `respawn=True, respawn_delay=2.0` to the `Node()` entry for
       `GPS-RTK_ROS2_pub_node.py` in the launch description.
     - **Field-survey workflow gap (slow TTFF after tripod move)**: the
       base F9P resumes a stored TMODE3 position from BBR/flash on
       power-up. If the tripod has been physically moved since the stored
       survey (e.g. yesterday's bench location → today's field location),
       the base broadcasts a stale reference position in RTCM 1005 and
       carrier-phase ambiguity resolution takes much longer than it
       should — the rover can plateau at `quality=5` (FLOAT) for minutes
       before climbing to 4 (FIXED), or get stuck. Add either (a) a
       u-center SOP to force a fresh survey-in at each new venue, (b)
       switch base to TMODE3=2 Fixed-mode with venue-known coordinates,
       or **(c, user-preferred 2026-05-29) a startup-time UBX-CFG-TMODE3
       sweep in `rtcm_server.py` that re-arms survey on each (re)boot**
       — `--re-survey-on-boot` flag, ~30 lines of UBX-CFG-VALSET to
       disable then re-enable Survey-In with min_dur=60s acc_limit=5m.
       Observed at the 2026-05-29 field test: 1 → 2 → 5 in ~4 seconds,
       stuck at 5 for ~9 minutes, then climbed to 4. ~9 min TTFF after
       a tripod move is unacceptable for the matrix-run workflow.

4. **Battle-station GPS view (per ADR-01 §"What we DO use GPS for").**
   Add to `interactive.html`:
   - Subscribe to `/gps_rtk_f9p_helical/gps/fix` (NavSatFix) and
     `.../rtk_status`
   - Subscribe to `/pixhawk/global_position/raw/fix` (regular GPS, L5).
     User explicitly asked 2026-05-29 to see **both RTK and regular GPS
     positions simultaneously** on the map so divergence is visible.
   - Big colored fix-quality bar: RTK FIX (green) / RTK FLOAT (yellow) /
     3D (orange) / NO FIX (red)
   - Magenta circle marker on the Leaflet map at the RTK lat/lon (distinct
     from the blue odom-projected marker, so drift is visible).
   - Distinct marker (different color, e.g. orange) at the regular-GPS
     lat/lon for the dual-view above.
   - NTRIP correction-age display
   - Refuse Push&Run when fix < FLOAT (configurable threshold)
   - **Open bug (2026-05-29 field session):** the existing **blue
     odom-projected marker** appears on the wrong side of a campus
     building and may not be updating. User report: "GPS location on
     webui seems to be wrong … blue dot on the other side of the
     building. and it could be not moving." Provisional hypothesis: same
     family as the [[project_odom_belief_frame_bug]] we fixed 2026-05-27 —
     the projection probably consumes raw `/wheel/odom` against a
     stale session anchor in the JS, so today's robot is plotted on
     yesterday's frame, shifted by the inter-session origin delta.
     Confirm by (a) checking which topic the marker subscribes to in
     `interactive.html` (raw vs `/wheel/odom_zeroed`); (b) checking the
     map anchor logic (hardcoded LL or pulled from somewhere); (c) at
     the next field session, drive the LIMO a few meters and see whether
     the dot moves at all.

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

- [ ] **`Data_Logger.py` ExternalShutdownException leaks an orphan `ros2 bag
  record`** (observed 2026-05-29 field session). When the parent
  `Data_Logger.py` Python process receives SIGINT, `rclpy` raises
  `ExternalShutdownException` from inside `rclpy.spin_once()`. The `main()`
  function only catches `KeyboardInterrupt` (line ~462), so the cleanup
  path — sending SIGINT to the `ros2 bag record` subprocess (`proc`) and
  the post-shutdown rename / metadata-rewrite — is never executed. Result:
  the bag's `db3` keeps growing under an orphan recorder (PPID=1), there is
  no `metadata.yaml` until that orphan is hand-killed, and the bag directory
  remains stuck with the `_DURATION_PLACEHOLDER` name even after the
  operator "stops" the logger. Fix: catch `ExternalShutdownException` in
  the same except branch as `KeyboardInterrupt` (or use a `try/finally`
  that always SIGINTs `proc` and runs the rename block), and propagate the
  signal to the subprocess explicitly. Repro: `kill -SIGINT <Data_Logger
  PID>` while it's recording; observe `ps -ef | grep 'ros2 bag record'`
  still alive with PPID=1.

## Test coverage backlog (audit 2026-06-01)

Full-suite coverage audit + first remediation pass. **Trusted baseline:** the
vendored vfg math layer (`scalecar-vfg-h-infinite/tests/` — controllers / paths /
simulation) is well-tested (tight numerical tolerances, closed-loop perf specs);
leave it alone. Everything below concerns the ROS nodes and the laptop analysis
pipeline.

**Done this pass** — laptop-verified, `python3 tools/qc/run_qc.py laptop` green
(76 unit tests + smoke 22/22):

- [x] `tests/qc/conftest.py` — discovery no longer depends on the run_qc
  PYTHONPATH wrapper (bare `python3 -m pytest tests/qc` works).
- [x] Tier-1 pipeline characterization tests: `test_qc_gating.py`,
  `test_run_eval_metrics.py`, `test_aggregate_stats.py`, `test_manifest_cells.py`
  — pin RTK%/estop/odom-gap/window, the e_d/e_psi metric core, fixed-mask,
  Wilcoxon pairing, CTI walk, `cell_key` rounding, completeness counts.
- [x] Tier-2: rewrote the `/cmd_vel` safety contract to an AST positive check
  (controllers publish `cmd_vel_raw`, never `cmd_vel`); extended geometry +
  RTK-status edge cases.
- [x] ros-sim tier established (`tools/qc/ros/`, skips off-NUC).

**Done 2026-06-02 — Tier-B sequencer fault-injection harness (NUC `ros-sim`):**

- [x] `tools/qc/ros/sequencer_harness.py` + `test_sequencer_sim.py` — 8 tests
  drive the REAL `experiment_sequencer_node` (subprocess on an isolated
  `ROS_DOMAIN_ID`) against one mock node impersonating orchestrator /
  reposition / odom_zero / RTK / estop / battery over the topic contracts. No
  hardware, no possible motion (real movers never launched). `run_qc.py ros-sim`
  → 16 passed (8 new + 8 odom_zero); laptop SKIPs. Covers: happy-path → DONE,
  F2 RTK pause + auto-resume, reposition + preflight reason propagation
  (P0 #4/#5), C6 exclusivity, autostart-off arming (P0 #10), F4 breaker,
  run-timeout leg-fail.
- [!] **Finding (deploy):** the installed colcon package was STALE — it predated
  the `start`/`arm` action and `autostart=False`. `ros2 run` uses the install,
  not the rsync'd source, so those fixes were never running on the robot.
  Rebuilt (`colcon build --packages-select limo_path_follower`). Do a rebuild +
  re-verify sweep before the next field trip; other written-but-unbuilt node
  fixes may exist.
- [!] **Finding (unattended):** only the F2 RTK-loss pause auto-resumes
  (`experiment_sequencer_node.py:990`). Battery-halt (M2) and circuit-breaker
  (F4, `:649`) pauses wait for an operator `resume`. With smoke's
  `circuit_breaker_k: 1` (`scenarios/smoke.yaml:26`) one failed leg halts the
  whole batch — fine for a 1-cell smoke; raise `k` for a stage-2 walk-away
  matrix.

**2026-06-02 — full-matrix bench dry-run harness (NUC, wheels-off):**

- [x] `tools/qc/ros/bench_world_node.py` + `scenarios/experiment_bench.yaml` —
  walks the FULL 320-cell matrix on a pedestal with no GPS. The node supplies
  synthetic RTK (integrates `/cmd_vel` through a unicycle in the venue frame,
  seeded at pin A, publishes `/gps_rtk_f9p_helical/gps/{fix,rtk_status}`
  quality=4) + a scripted battery fault (republishes `/limo_status` ->
  `/limo_status_bench`, drops volts < halt after N completed runs). Exercises the
  autonomy brain + real movers + safety chain end-to-end and verifies the M2
  low-battery halt + operator notification. **Wheels-off only** — it spoofs RTK
  FIXED, so the movers energize; no translation only because the wheels are up.
- [!] **BUG FOUND + FIXED — operator notify (T8) was dead.**
  `experiment_sequencer_node._notify()` never passed `topic`/`server` to
  `tools/notify/ntfy.py`, and the node never read the config's `ntfy:` block, so
  a non-empty `ntfy.topic` was silently ignored and **NO operator push (battery
  halt M2, circuit breaker F4, RTK loss F2) ever left the robot.** In the field
  the unattended operator would never be told the battery died — exactly the gap
  the battery dry-run targets. Fixed: `set_notify_channel()` wires topic/server
  from config; `_notify` forwards them and fires on a daemon thread (the blocking
  ~5 s HTTP POST also sat in the control `_tick`). Empty topic still disables push
  (safe default). **TODO: verify the push end-to-end on the bench run, then set a
  real field topic in `experiment.yaml`.**

**2026-06-04 — desk field-readiness sweep + fresh bench dry-run (rebuilt code):**

- [x] Rebuilt `limo_path_follower` after rsync (clears the 2026-06-02 stale-install
  finding); `run_qc.py ros-sim` 28/28, laptop smoke 22/22. RTK fixes confirmed
  present in code: base re-survey-on-boot (`rtcm_server.force_survey_in_on_boot`,
  default on, `--no-force-survey-in` opt-out), rover launcher `respawn=True`, rover
  socket `SO_KEEPALIVE` + stale watchdog + reconnect. Discord operator-alert channel
  verified end-to-end (live test post returned 2xx).
- [x] **M2 low-battery halt + operator notify validated end-to-end (rebuilt code).**
  `scenarios/experiment_bench_m2.yaml` (F4 disarmed: `circuit_breaker_k=50`) +
  `bench_world --ros-args -p runs_before_low:=2`: autonomy ran AtoB + turnaround_B
  (2 legs, both PASS), fake battery dropped 12.5 → 10.3 V (< 10.5 halt), sequencer
  caught it at the next PREFLIGHT — `PAUSE: battery 10.30V < halt 10.5V (M2)` — and
  fired the high-priority operator alert (`_notify` → Discord; discord.env present).
  Confirms M2 detection + the "come collect the robot" push. The F4-disarm is
  required because `_tick` checks F4 *before* M2, so a stray reposition abort would
  otherwise pre-empt the battery halt.
- [!] **Bench full-matrix dry-run does NOT complete on the placeholder venue.**
  Fresh `bench_world` + bench sequencer (`experiment_bench.yaml`, rebuilt code):
  cell 1 AtoB + turnaround_B PASS, then `reposition aborted: target outside inset
  working area (R3) err_m=35.76`, retry exhausted → cell skip → next preflight →
  F4 circuit-breaker pause. Root cause = `rooftop.json` is PLACEHOLDER (42 m × 10 m
  projected rectangle, pins A/B ~42 m apart); the short step-curvature legs do NOT
  net back to pin A, so reposition plans a real out-of-bounds drive — exactly the
  case the yaml comment anticipated ("a finding about this venue, not the
  controller"). Safety logic behaved correctly (retry/skip/F4). Not a controller or
  sequencer bug; not a field blocker (the field venue is re-pinned from live RTK).
  BUT the bench cannot currently demo a clean full walk + M2 battery-halt on the
  placeholder venue. **Same R3 area-abort surface that killed the May-29 field run**
  → venue-pin correctness is the #1 on-site dependency.
- [!] **deployment.md claim vs reality (flag).** "Validated by the bench path: …
  reposition geometry" / "walks the FULL 320-cell matrix" overstates today's
  behaviour — the bench aborts at cell 1 on the placeholder venue. Narrow the wording
  to "exercises the state machine through the first reposition + the M2/F4 paths,
  given a self-consistent venue." Proposed, not yet edited.
- [ ] **Preflight `/wheel/odom` liveness false-negative (observed once).** A
  cell-boundary preflight reported "/wheel/odom — no messages in 3 s" even though
  `bench_world._integrate` publishes odom unconditionally at 50 Hz and the graph was
  small (no mavros). Cause unconfirmed (transient vs too-tight 3 s echo window under
  DDS churn). If it recurs in the field it spuriously fails preflight → cell failures
  → circuit-breaker pause (operator intervention). Widen the odom-liveness window /
  add a retry in `preflight.sh`. Sev: Med (field).
- [ ] **Field-start hazard:** `bench_world_node.py` impersonates `limo_base_node` and
  is currently RUNNING (paused bench). Before any field trip it MUST be killed and
  the REAL base brought up — a lingering impersonator silently replaces the base
  driver (see `DOC/deployment.md` "never let anything impersonate limo_base_node").
- [!] **FIELD BUG (2026-06-04) — `base_gnss` gives no `/wheel/odom`: `/dev/limo_base`
  resolves to the WRONG serial adapter.** The NUC has TWO CP2102 USB-serial adapters
  that both report serial `0001` (factory default). `99-limo-base.rules` keys the
  `/dev/limo_base` symlink on `serial=="0001"`, so it matches BOTH and lands on
  whichever enumerated first — **ttyUSB0 = the non-chassis adapter (USB port 3-3)**.
  The real chassis is **ttyUSB1 (USB port 3-7.4)**. `base_gnss` opens
  `port_name=limo_base` → `/dev/limo_base` → ttyUSB0 → no chassis → no `/wheel/odom`,
  no `/limo_status`. `base_vanilla` hardcodes `port_name=ttyUSB1`, so it worked this
  boot — but is equally fragile (ttyUSB0/1 can swap on reboot).
  - **QUICK FIX APPLIED (per-session):** `sudo ln -sfn ttyUSB1 /dev/limo_base`,
    restart `base_gnss`. Verified `/wheel/odom` 49.9 Hz, `motion_mode=1`, batt 12.2 V.
  - **IDEAL SOLVE (persistent — do later):** rewrite `/etc/udev/rules.d/99-limo-base.rules`
    to match the chassis by PHYSICAL USB PORT instead of the non-unique serial:
    `SUBSYSTEM=="tty", KERNELS=="3-7.4:1.0", SYMLINK+="limo_base"`, then
    `sudo udevadm control --reload && sudo udevadm trigger`. Survives reboots/port-swaps
    as long as the chassis cable stays in NUC USB port 3-7.4. Also point base_vanilla's
    launch default at the `limo_base` symlink so both launches share the robust path.
    Commit a copy of the rule under `tools/` so a fresh NUC inherits it. (Most-robust
    alternative: flash unique serials onto the two CP2102s via `cp210x-program`, then
    key udev on serial.) See [[project_limo_base_dual_cp2102]].
- [!] **FIELD BUG (2026-06-04) — battle-station vs robot HEADING-CONVENTION mismatch
  (suspected; needs on-robot confirm).** In `tools/path_gen/interactive.html` the
  start/end pin arrow is drawn with `localAng = (heading_deg - bearing)` then placed
  via `localToLatLon`, so the on-screen arrow points at compass `2*bearing - heading_deg`
  (a reflection about the venue bearing axis); on drag it stores the inverse. But
  `reposition_node._bearing_deg_to_local_yaw` treats `heading_deg` as a TRUE compass
  bearing (E of N) and steers the robot to `heading_deg`. Net: a venue whose arrows
  correctly point AT each other on the map (operator's export: S1=133.6, E1=312.6 with
  bearing~42 -> arrows ~310/131, i.e. facing each other) encodes headings that would
  drive the robot to the MIRROR (~133/312, facing away). I wrongly "corrected" the
  headings (mirrored them) by reading the raw numbers as compass; **reverted** — the
  venue file now holds the operator's verbatim export.
  - **DO NOT trust either side until verified on the robot.** Test: place robot at S1,
    reposition, drive a few cm, watch whether it heads toward E1 (UI correct) or away
    (UI mirrored vs robot). That decides which side to fix.
  - **Candidate fix:** make the UI render+export use true compass `heading_deg` (drop
    the double bearing-rotation in the tip math) so the drawn arrow == the bearing the
    robot drives; then re-export. Confirm against `reposition_node`.

**Fixed — real bug the suite caught:**

- [x] `tools/ops/limo_ops.py` `ORCH_NAMES` was missing `"ops"` — drift vs
  `orchestrator_node.PROCS`, which manages the real `ops_node` entry point
  (`setup.py:27`). `test_orchestrator_process_names_match_cli_facade` was RED;
  synced (`ops` is not motion-capable, so not added to the refuse-set) → green.

**Suspected analysis-pipeline bugs — behavior pinned with tests:**

- [x] **#1 `run_eval.fixed_mask_for` had no time tolerance**
  (`tools/analysis/run_eval.py:336`). Added opt-in `max_dt` (default `None` =
  legacy behaviour). Sparse `rtk_status` could label a fix FIXED from a
  temporally-distant sample → corrupts RTK-truth ground truth. **Still TODO:**
  wire a conservative `max_dt` into `evaluate()` (both `fixed_mask_for` call
  sites) once a threshold is chosen against real bags. Sev: Med.
- [x] **#2 `qc._window` degenerate fallback** (`tools/analysis/qc.py:47`). Added a
  `no_run_window` QC reason so a bag with <2 odom/status samples fails loudly
  instead of slipping through on whole-bag RTK%/estop with the length check
  silently skipped. Sev: Low–Med. (smoke.sh still green.)
- [ ] **#3 silent frame-provenance fallbacks** — `run_eval.odom_belief_source`
  (`run_eval.py:322`) silently uses raw `/wheel/odom` when `odom_zeroed` is
  absent; `transform_to_path_frame` is a pass-through when `path_frame_anchor is
  None`; hardcoded rooftop anchor when `venue.anchor` absent. Behaviour **pinned
  by tests; fix not applied** — proposal: surface the chosen odom source + anchor
  provenance into `qc.csv`/manifest and add a QC reason when an RTK bag lacks the
  per-leg `path_frame_anchor`, so a leg scored in the wrong frame is visible.
  Sev: Med.

**Remaining gaps — ROS node safety logic (NUC-gated, `ros-sim` tier):** every
safety gate is still untested. Needs the sourced NUC (`run_qc.py ros-sim`), or
extraction of the pure kernels into rclpy-free modules **inside the package**
(note: `tools/qc/common.py` is laptop-side and not importable by the installed
node, so a kernel must live under `limo_path_follower/`, e.g. a new `se2.py` /
`experiment_matrix.py`, with the node rewired to import it — a production change
that requires a colcon rebuild + on-robot verify, per the dev cycle).

- [ ] `odom_zero` SE(2) re-anchor `P'=R(-yaw0)(P-O)` (inline in `_odom_cb`) —
  extract + unit test (identity at reset, known rotation/translation). The
  building-block helpers are already covered in
  `tools/qc/ros/test_odom_zero_helpers.py`.
- [~] `experiment_sequencer_node`: COVERED at integration level by the Tier-B
  sim harness (see "Done 2026-06-02" above) — F4 breaker, F2 RTK-loss,
  C6 mover-exclusivity, L3 odom-zero confirmation, reposition + preflight
  failure-reason propagation, autostart-off arming, run-timeout leg-fail.
  **Still open:** `_build_cells`/`_cell_id` determinism + `_classify` (D4
  multi-condition pass) as Tier-A rclpy-free unit kernels; M2 battery-halt has
  no dedicated low-battery test yet.
- [ ] `path_follower_node`: odom-timeout halt (0.5 s → zero), steering clip
  `[-0.5, 0.5]`, kinematics `omega = v·tan(δ)/L`.
- [ ] `reposition_node`: `_plan_clear` composition (area + exclusion), RTK
  quality gate, 3-point-turn decision.
- [ ] `estop_cli.py`: ping-failure threshold (10 misses → latch) + the
  `cmd_vel_raw → cmd_vel` filter (the last motion gate).

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
