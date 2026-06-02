# Bug Fix Backlog — Field-Test Handoff

Context for the next agent: this repo is the H-infinity LIMO paper data-gathering
system. The latest field session was the 2026-05-29 full-system E2E attempt
documented in `DOC/May_29th_Full_system_e2e_testing_log.md`. The autonomous
headline smoke test did **not** complete. The system reached RTK/preflight and
reposition attempts, but multiple infrastructure bugs blocked a clean
`preflight -> reposition -> odom reset -> AtoB -> turnaround -> BtoA -> bag`
run.

Use this file as the first fix queue. Preserve the safety contract:
controller/reposition publish only `cmd_vel_raw`; `estop_cli.py` is the only
path to `/cmd_vel`. Wheels-on-floor tests require explicit operator
confirmation.

## P0 — Fix Before Next Field Run

### 1. Rover RTK stale-socket / no-reconnect bug

**Symptom:** `GPS-RTK_ROS2_pub_node.py` can get stuck reporting
`RTCM: STALE` and never reconnect after the Pi/base station, LIMO_AP, phone
hotspot, or route flaps. In the field, killing and restarting the rover GPS-RTK
process restored RTCM flow.

**Why it matters:** RTK FIXED is a preflight gate and reposition depends on
live RTK. This bug repeatedly blocked the field test.

**Likely files:**
- `GPS-RTK_ROS2_pub_node.py`
- `src/limo_ros2/limo_base/launch/LIMO+MAVROS+RTK_Node_Launcher.launch.py`

**Fix direction:**
- Add reconnect loop around the TCP RTCM socket.
- Add watchdog: if no RTCM bytes arrive for N seconds, close and recreate the
  socket.
- Enable `SO_KEEPALIVE` with short intervals where platform-supported.
- Use exponential backoff on initial connection failure too.
- Make start-before-Pi-ready recover without manual process kill.

**Verification:**
- Start rover node before Pi/base is reachable.
- Bring Pi/base online later.
- Confirm `/gps_rtk_f9p_helical/gps/rtk_status` transitions from stale/no
  correction to active RTCM without killing the process.
- Provoke network loss and recovery; confirm reconnect.

### 2. Base station stale survey position after tripod move

**Symptom:** The base F9P can reuse a stored TMODE3/BBR/flash position after
the tripod moves. This produced a false RTK FIXED solution offset by roughly
8-10 m, visible as the web UI RTK marker on the wrong side of the building.

**Why it matters:** False FIXED is worse than no FIXED. It poisons venue pins,
reposition, and ground truth.

**Likely files:**
- `tools/rtk_base/rtcm_server.py`
- `tools/rtk_base/README.md`
- `DOC/rtk_base_deploy.md`

**Fix direction:**
- Add a startup option such as `--re-survey-on-boot`.
- On service startup, send UBX-CFG-TMODE3 / CFG-VALSET commands that disable
  the stale stored survey/fixed position and re-enable Survey-In.
- Use the intended field parameters: about 60 s min duration and 5 m accuracy
  limit, unless docs specify otherwise.
- Log survey state clearly so field operator can see fresh survey progress.

**Verification:**
- Move base antenna/tripod between two positions.
- Reboot Pi/base service.
- Confirm base does not immediately broadcast the stale old RTCM 1005 position.
- Confirm rover FIXED position agrees with standalone rough GPS / physical
  location before venue pins are captured.

### 3. GPS-RTK launch file missing respawn

**Symptom:** If the rover GPS-RTK process dies, the launch file does not restart
it. During the field workaround, manual standalone restart was required.

**Likely file:**
- `src/limo_ros2/limo_base/launch/LIMO+MAVROS+RTK_Node_Launcher.launch.py`

**Fix direction:**
- Add `respawn=True` and `respawn_delay=2.0` to the `Node()` action that starts
  `GPS-RTK_ROS2_pub_node.py`.

**Verification:**
- Launch `base_gnss`.
- Kill only the GPS-RTK node process.
- Confirm launch respawns it and topics return automatically.

### 4. Sequencer hides exact reposition abort reason

**Symptom:** `reposition_node` publishes detailed JSON on `/reposition/status`
with a `reason`, but `experiment_sequencer_node.py` collapses it into:
`reposition aborted (R3 area/exclusion or unreachable)`.

**Why it matters:** The field session lost time because the LLM/operator had to
dig through logs to know whether the entry point, target, approach segment, or
final segment failed validation.

**Likely files:**
- `scalecar-vfg-h-infinite/ros2_bridge/limo_path_follower/experiment_sequencer_node.py`
- `scalecar-vfg-h-infinite/ros2_bridge/limo_path_follower/reposition_node.py`

**Fix direction:**
- Store the full last `/reposition/status` payload in the sequencer, not just
  `state`.
- When `state == "aborted"`, propagate `reason`, `err_m`, and `err_deg` into:
  - sequencer log line
  - `/experiment/status.message`
  - failure notification
  - sidecar/classification if a leg artifact exists

**Verification:**
- Create a venue where the approach entry point is outside the polygon.
- Command a reposition through the sequencer.
- Confirm `/experiment/status` says the exact failing condition, e.g.
  `entry outside inset working area`.

### 5. Preflight failure is opaque through sequencer

**Symptom:** Sequencer reports only `preflight FAIL (exit 1)`. The operator has
to rerun or inspect logs to know which check failed.

**Likely files:**
- `tools/preflight/preflight.sh`
- `scalecar-vfg-h-infinite/ros2_bridge/limo_path_follower/experiment_sequencer_node.py`

**Fix direction:**
- Capture preflight stdout/stderr instead of discarding both to `/dev/null`.
- Either parse `FAILED_CHECKS` output or add a machine-readable mode to
  `preflight.sh`, e.g. `--json`.
- Publish failed check names and short reason in `/experiment/status.message`.

**Verification:**
- Force RTK not FIXED or E-stop/preflight failure.
- Launch sequencer.
- Confirm `/experiment/status` includes exact failed checks and suggested next
  action.

## P1 — Fix Soon

### 6. `Data_Logger.py` SIGINT cleanup leaks orphan `ros2 bag record`

**Symptom:** Manual logger stop can raise `ExternalShutdownException`, skip
cleanup, leave an orphan `ros2 bag record` process, and fail to write
`metadata.yaml` / rename the `_DURATION_PLACEHOLDER` bag directory.

**Likely file:**
- `Data_Logger.py`

**Fix direction:**
- Catch `ExternalShutdownException` together with `KeyboardInterrupt`, or use a
  `try/finally`.
- Always send SIGINT to the bag subprocess, then kill if it does not exit.
- Always run the rename / metadata rewrite path when possible.

**Verification:**
- Start manual `Data_Logger.py`.
- Send SIGINT to parent process.
- Confirm no orphan `ros2 bag record` remains, metadata exists, and bag dir name
  no longer contains `_DURATION_PLACEHOLDER`.

### 7. Web UI blue odom-projected marker may use stale anchor/frame

**Symptom:** Direct RTK and regular GPS markers were added because the existing
blue odom-projected marker appeared on the wrong side of the building and may
not have moved.

**Likely file:**
- `tools/path_gen/interactive.html`

**Fix direction:**
- Confirm whether blue marker subscribes to raw `/wheel/odom` or
  `/wheel/odom_zeroed`.
- Confirm which lat/lon anchor and bearing it uses.
- Make the displayed odom marker explicitly tied to the current path-frame
  anchor or label it as raw/session-projected odom.
- Avoid carrying yesterday's anchor into today's field session.

**Verification:**
- With synthetic `/wheel/odom_zeroed`, marker should move predictably from a
  known anchor.
- In field, move robot a few meters; direct RTK marker and odom-projected marker
  should both move in consistent directions, while still showing odom drift.

### 8. Venue polygon click-order fragility

**Symptom:** A valid smoke venue became self-intersecting because corners were
clicked in non-CCW order. Reposition then rejected points that were inside the
intended convex area.

**Likely files:**
- `tools/path_gen/interactive.html`
- `scalecar-vfg-h-infinite/ros2_bridge/limo_path_follower/reposition_node.py`

**Fix direction:**
- Normalize venue corners to convex hull / CCW order before export and before
  validation.
- Show a warning in the UI if polygon is self-intersecting.
- Consider storing both click order and normalized order for provenance.

**Verification:**
- Click four corners in crossed order.
- Export venue.
- Confirm stored polygon is simple and reposition validation accepts points
  inside the intended area.

### 9. Reposition approach-entry validation brittle for small smoke areas

**Symptom:** Reposition validates target, approach entry point, approach
segment, and final segment. In a tight 5 m field smoke area, this can abort
before movement. The abort reason was hard to inspect before P0 item 4.

**Likely file:**
- `scalecar-vfg-h-infinite/ros2_bridge/limo_path_follower/reposition_node.py`

**Fix direction:**
- First fix exact reason propagation.
- Then tune `approach_dist_m` for smoke tests or make it configurable from the
  smoke scenario.
- Add a dry-run validator that reports whether current S1/E1/reposition plan
  fits before launch.

**Verification:**
- Use the smoke venue JSON with a narrow working area.
- Validate target/entry/final segment without moving robot.
- Confirm UI/sequencer can show whether the plan fits before start.

### 10. Sequencer autostart is unsafe for operator/LLM workflow

**Symptom:** Launching `experiment_sequencer_node` starts the batch by default.
In field use, loading config should be separate from arming and starting.

**Likely file:**
- `scalecar-vfg-h-infinite/ros2_bridge/limo_path_follower/experiment_sequencer_node.py`

**Fix direction:**
- Change default `autostart` to `False`.
- Add explicit `/experiment/cmd` actions such as `validate`, `arm`, and `start`,
  or make `resume` from idle clearly documented and safe.
- Ensure validation never commands motion.

**Verification:**
- Launch sequencer.
- Confirm it remains idle and publishes loaded/ready status.
- Send explicit start command and confirm it begins preflight only then.

## P2 — Operability / LLM Interface

### 11. No high-level "run shortest E2E smoke" API

**Symptom:** The LLM had to manipulate YAML, venue JSON, sync files, launch the
sequencer, and poll logs. This caused a mismatch with the user's intent: the
user wanted the shortest existing full-system test, not ad-hoc manual logging.

**Fix direction:**
- Add an LLM/operator-facing layer, probably a thin `ops_node`, above the
  existing orchestrator and sequencer.
- Candidate commands:
  - `status`
  - `validate_stack`
  - `validate_venue`
  - `run_smoke_e2e`
  - `restart_rtk_receiver`
  - `force_base_resurvey`
  - `recover_after_failure`
  - `stop_all_motion`

**Verification:**
- From a clean boot, a single command should run the shortest non-destructive
  validation.
- A second explicit command, after wheels-on-floor confirmation, should run the
  shortest motion E2E.

### 12. Orchestrator status is only boolean process liveness

**Symptom:** `/orchestrator/status` reports `{name: bool}` only. It cannot
explain PID, last exit code, intentional stop reason, or recent logs.

**Likely file:**
- `scalecar-vfg-h-infinite/ros2_bridge/limo_path_follower/orchestrator_node.py`

**Fix direction:**
- Extend status payload with:
  - `running`
  - `pid`
  - `last_exit_code`
  - `last_started_at`
  - `last_stopped_at`
  - `intentional_stop_reason`
  - short `log_tail`

**Verification:**
- Start and kill `follower`.
- Confirm status shows intentional stop and PID history.
- Crash a process and confirm nonzero exit is visible.

### 13. C6 exclusive mover behavior confuses UI/operator

**Symptom:** During reposition, the sequencer intentionally kills
`path_follower_node` so only one `cmd_vel_raw` publisher exists. The web UI then
shows follower down, which looks like a fault.

**Likely files:**
- `tools/path_gen/interactive.html`
- `scalecar-vfg-h-infinite/ros2_bridge/limo_path_follower/experiment_sequencer_node.py`
- `scalecar-vfg-h-infinite/ros2_bridge/limo_path_follower/orchestrator_node.py`

**Fix direction:**
- Publish a mode/reason such as `follower stopped intentionally: reposition owns
  cmd_vel_raw`.
- In the UI, distinguish "expected stopped" from "unexpected down".

**Verification:**
- Start a sequencer reposition phase.
- UI should show follower as intentionally inactive, not failed.

## Recommended Fix Order

1. Rover RTK reconnect watchdog.
2. Base fresh survey on boot.
3. GPS-RTK launch respawn.
4. Reposition abort reason propagation.
5. Preflight machine-readable failure reporting.
6. Data logger cleanup.
7. Venue polygon normalization.
8. Sequencer autostart/arm/start split.
9. Web UI odom marker frame fix.
10. Higher-level ops API.

After items 1-5, rerun the shortest field E2E smoke with:
- one cell
- one repetition
- low speed, e.g. `0.2 m/s`
- relaxed RTK run-window threshold for smoke only
- explicit wheels-on-floor confirmation before motion
