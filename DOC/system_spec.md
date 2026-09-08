# Final System Spec Sheet — H∞ Hardware Data-Gathering System

**Status: locked** (2026-05-24). This is the target-state specification — *what the system
must be*, not how to build it. The gap analysis and revision plan are derived against this
sheet; if reality and this sheet disagree, fix the sheet first.

Read alongside:
- `DOC/experiment.md` — experiment scope, the curvature-sweep pivot, matrix, operational model.
- `DOC/decisions/01_gps_no_fusion.md` — ADR-01, why RTK stays out of the control loop.
- `DOC/deployment.md` — NUC caveats, support-script inventory, runbook.
- `DOC/paper_ijat.pdf` — the sim paper this work follows up on.

## 1. Purpose

Produce a **paper-grade dataset** comparing the LPV-H∞ and PID-FF path-following controllers
on the AgileX LIMO across a **curvature sweep** at fixed speed (ρ = κv preserved), with RTK
as post-hoc ground truth, gathered **unattended**. Supports the follow-up paper to
`DOC/paper_ijat.pdf` (claim pivot in `DOC/experiment.md`).

## 2. Concept of operations

Operator pins the venue once (drive corners + map), defines the matrix in one config file,
presses start in the battle station, and walks away. The system repositions the robot to
each start point, records each run, executes turnarounds, alerts on battery/wallclock/faults,
and retries failures. A single analysis command turns the bag directory into the paper's
tables and plots.

## 3. Required Operational Capabilities (ROC)

The locked capability list. Each is testable; acceptance is in §5.

### 3.1 Reference path generation
- **P1** Generate **exact analytic step-curvature** references from {R, θ_arc, L1, L2, direction} — instant curvature step, no smoothing.
- **P2** Generate **exact analytic slalom** references from {R, θ_arc, L1, L_mid, n_arcs, L_end}.
- **P3** Generate **turnaround** references: U-turn (semicircle @ R_min) and 3-point turn (incl. reverse).
- **P4** Deliver the reference to the controller **without geometry-distorting transforms** — the controller tracks the exact analytic curve, matching the sim.
- **P5** Retain **ad-hoc hand-drawn** (waypoint/Bezier) paths for non-matrix testing.

### 3.2 Control & safety
- **C1** Select **LPV-H∞ or PID-FF** per run; expose controller tuning params.
- **C2** Constant-speed runs at configured v (**≤ 1.0 m/s** firmware cap).
- **C3** All actuation flows **controller → `cmd_vel_raw` → `estop_cli` → `/cmd_vel`**; no node writes `/cmd_vel` directly (ADR-01; contract in §4).
- **C4** E-stop: **latched**, remotely triggerable, zeros output; conservative on exit.
- **C5** **Odom-timeout failsafe**: zero velocity on > 0.5 s odom silence.
- **C6** **Exactly one** `cmd_vel_raw` publisher active at any instant (follower XOR reposition).

### 3.3 Localization & ground truth

GNSS topology (3 receivers; this system uses **only the RTK one**): **GPS-RTK** =
helical F9P, `/gps_rtk_f9p_helical/gps/*`, cm-level — the sole ground truth + reposition
feedback for this system. **Regular GPS** = Pixhawk via MAVROS, `/pixhawk/.../fix` —
recorded in the bags but **for a separate purpose** (extending the professor's earlier
LIMO-based dataset); no role in this system's control or analysis. **NTRIP GNSS** =
standalone blackbox (RTK correction source); ignored.

- **L1** Wheel odom is the **only** control feedback during a recorded run (ADR-01).
- **L2** **GPS-RTK** (`/gps_rtk_f9p_helical`) lat/lon + fix quality **recorded in every run bag** as the sole ground truth.
- **L3** **Reset odom to (0,0,0)** at a known physical point before each recorded run, **with closed-loop confirmation** on `/odom_zero/status` before driving, **without disturbing the RTK stack**. The analytic reference path is anchored at this reset pose with +x along the heading at reset (the "path frame"); the same frame `/wheel/odom_zeroed` exposes to the follower.
- **L4** RTK usable for **between-run** repositioning only — never inside a recorded run.
- **L5** **Regular GPS** (`/pixhawk/.../fix`, `…/satellites`, `…/gpsstatus/gps1/raw`) recorded alongside every run for the professor's separate dataset extension; not used for control, gating, or this paper's metrics.

### 3.4 Repositioning
- **R1** Drive autonomously to a stored start pin (lat/lon) using **GPS-RTK feedback** (closed-loop go-to-pose); proceed only with an **RTK FIXED** solution.
- **R2** Arrive with **deterministic heading** via a straight final-approach segment.
- **R3** **Abort + alert** if the reposition path would exit the working area or enter an exclusion zone.
- **R4** Reposition is a **no-op** when the next start pin equals the prior end pin.

### 3.5 Venue & working area

The rooftop test surface is a **rectangle with a central roundabout island** (rectangle with
a hole). Usable space is the periphery around the island; curves and turnarounds must be
*placed* in that space, not just checked for fit.

- **V1** Persist per-venue geometry — 4 corners, safety margin, **central circular exclusion (the roundabout island)** + any other exclusions, start/end pins (lat/lon + heading) — as **committed WGS84 config** surviving reboots and weeks.
- **V2** Capture corners by **manual drive + live-RTK pin** on the battle-station map; capture the island as a circular exclusion the same way.
- **V3** **Exclusion-aware placement:** lay out start/end pins and each curve + turnaround in the usable region around the central island (e.g. along the long sides), not overlapping the island or violating the margin.
- **V4** **Validate** every matrix curve, turnaround, and reposition path against the inset polygon AND all exclusions before any run; refuse to start a cell whose geometry doesn't fit.

### 3.6 Orchestration / autonomy
- **O1** Sequencer runs **on the NUC**, survives laptop disconnect/sleep.
- **O2** Iterate the **full matrix** (controller × speed × path family × radius × N) unattended.
- **O3** Per-cell cycle: preflight → reposition → odom reset → record → run → detect completion → stop → classify → turnaround → return leg.
- **O4** **Auto-retry** failed legs up to a limit, then skip the cell and continue.
- **O5** Start/stop from the battle station; processes managed by the orchestrator supervisor.

### 3.7 Recording & data management
- **D1** Record a **bag per leg** with the full required topic set (§4).
- **D2** **Deterministic** output directory layout + run-ID, pairable with external exports.
- **D3** **Per-leg sidecar JSON**: cell params, path recipe, venue + pin IDs, **`venue.path_frame_anchor`** (start-pin pose `{lat, lon, heading_deg, pin_id}`; null for turnarounds / manual bags), RTK fix summary, classification, wallclock, git commit, controller tuning.
- **D4** Per-run **pass/fail classification** per §5 criteria.

### 3.8 Monitoring & alerting
- **M1** **Preflight gate** each cell: **RTK FIXED** (quality 4) sustained for K s, topic health, `motion_mode == Ackermann`, battery OK. On loss of FIXED: alert + wait, do not start.
- **M2** Battery **alert at 10.3 V**, **halt new runs at 10.0 V** (telemetry is volts —
  `/limo_status.battery_voltage`; nominal 11.1 V LiPo). *Code/UI are not yet synced to
  this pair — see the `ToDo.md` follow-ups.*
- **M3** **Wallclock heartbeat** ping at a configured interval.
- **M4** **Failure / batch-complete** alerts. Channel: **ntfy.sh**.
- **M5** **Live progress** (current cell, ETA, pass/fail tally) visible in the battle station.

### 3.9 Analysis
- **A1** One-command **per-bag metrics**: RMS/max cross-track error, RMS/max heading error, terminal pose error, steering effort — computed **twice** (odom-belief and RTK-truth).
- **A2** **Per-cell aggregation** over N: Wilcoxon signed-rank LPV vs PID; headline plot (max heading error vs R, both controllers, error bars); Curvature Tolerance Index.
- **A3** **Compute-cost figure** (per-cycle controller time on the NUC) — second paper claim.

### 3.10 Fault handling, safe-state & resumability
- **F1** **Safe-state on any halt/abort/fault = stop in place and hold:** robot commands zero velocity through the estop chain and holds where it is — no repositioning or other motion during a fault. Operator decides the next step remotely.
- **F2** **Persistent RTK-FIXED loss:** if FIXED is not (re)acquired within a configured wait, pause the batch in safe-state and alert; resume automatically when FIXED returns (bounded), else hold for the operator.
- **F3** **Recorded-run RTK dropout:** loss of FIXED *during* a recorded run fails that run (incomplete ground truth) → auto-retry per O4.
- **F4** **Systemic-fault circuit breaker:** after K consecutive leg/cell failures, halt the whole batch in safe-state and alert (don't grind through a broken session).
- **F5** **Operator remote abort/pause/resume:** operator can halt the batch into safe-state at any time (weather, bystanders, anything) from a remote control surface, and resume later. Requires a reachable two-way control path even while otherwise unattended.
- **F6** **Checkpoint & resume:** completed *passing* cells are persisted; after any pause/abort/battery-swap/reboot the batch resumes without repeating them; partially-recorded legs are discarded and redone.
- **F7** **Connectivity-loss policy:** unattended estop runs without the ping-trip so a brief WiFi blip doesn't abort a run, but the system stays remotely abortable (F5). This trade-off is explicit and tunable.
- **F8** **Environmental hazards (rain, surface) are not auto-sensed:** handled via F5 operator abort + F6 resume; optional max-session-duration cap as a backstop.

## 4. Interface contract (canonical, target state)

| Interface | Type | Direction | Purpose |
|---|---|---|---|
| `/reference_path_recipe` | `std_msgs/String` (JSON, latched) | sequencer → follower | analytic curve type + params (P1–P4) |
| `/reference_path` | `nav_msgs/Path` (latched) | follower → viz / ad-hoc → follower | sampled analytic curve for viz + bag; ad-hoc Bezier input (P5) |
| `/path_follower/status` | `std_msgs/Float32MultiArray` | follower → * | telemetry incl. per-cycle timing (A3) |
| `/path_follower/done` | `std_msgs/Bool` (latched) | follower → sequencer | crisp completion edge (O3) |
| `cmd_vel_raw` → `/cmd_vel` | `geometry_msgs/Twist` | controller/reposition → estop → base | sole actuation path (C3) |
| `/estop`, `/estop_trigger` | `std_msgs/Bool` | estop ↔ * | latched safety (C4) |
| `/wheel/odom` | `nav_msgs/Odometry` | base → overlay / bag | raw wheel odom (recorded as L1 reference; not the control input) |
| `/wheel/odom_zeroed` | `nav_msgs/Odometry` | overlay → follower / bag | re-anchored control feedback in the per-leg path frame (L1,L3) |
| `/odom_zero/reset` | `std_msgs/Bool` | sequencer → overlay | command odom zero (L3) |
| `/odom_zero/status` | `std_msgs/String` (JSON, latched, transient-local) | overlay → sequencer | reset-latch confirmation `{has_reset, origin{x,y,yaw}, stamp}` (L3 closed-loop) |
| `/gps_rtk_f9p_helical/gps/{fix,nmea,rtk_status}` | `NavSatFix` / `String` | GNSS → * | reposition + sole ground truth + RTK-FIXED gate (L2,L4,R1,M1) |
| `/pixhawk/global_position/raw/{fix,satellites}`, `/pixhawk/gpsstatus/gps1/raw` | MAVROS GPS | GNSS → bag | regular GPS, recorded for the prof's separate dataset (L5); no role here |
| `/reposition/{goto,status}` | JSON / status | sequencer ↔ reposition | go-to-pose (R1,R2) |
| `/orchestrator/{start,kill,status}` | `std_msgs/String` | sequencer ↔ supervisor | process control (O5) |
| battery / `motion_mode` source | TBD (`/limo_status`?) | base → preflight/sequencer | gating (M1,M2) — **verify on robot** |

> **"sequencer" in this table = the experiment-driver role**, not a specific node.
> That role is now `run_executor_node` (the webui leg-batch + auto-planner pathway).
> The original `experiment_sequencer_node` is **DEPRECATED** (operator decision
> 2026-06-12), kept only for the `sequencer_smoke` smoke-e2e until repointed. The
> topic contract is unchanged either way.

**Required bag topic set (D1):** `/wheel/odom` (raw L1 reference),
`/wheel/odom_zeroed` (the stream the follower actually tracks; required for the
odom-belief metric — without it the odom-belief column has a frame mismatch
against the analytic reference path), `/cmd_vel`, `/cmd_vel_raw`, `/estop`,
`/reference_path`, `/path_follower/status`, `/path_follower/done`,
`/gps_rtk_f9p_helical/gps/{fix,nmea,rtk_status}` (ground truth),
`/pixhawk/global_position/raw/{fix,satellites}`, `/pixhawk/gpsstatus/gps1/raw` (regular GPS,
for the prof's separate dataset — L5), `/imu`, `/path_follower/timing` (if separate).

## 5. Acceptance criteria

**Per recorded run passes iff:** reached path end within tolerance; no E-stop; no node
crash/exception; all required topics present for the full window; bag wallclock ≈
(arc-length / v) within ±X%; **GPS-RTK in FIXED** for ≥ Y% of the window. Else fail →
auto-retry → skip.

**Session-level (paper-grade):** zero unintended E-stops, zero missing topics, zero crashes
across a full session. Any such event is an orchestration/recording bug to fix before the
dataset counts.

**Repositioning:** starts only on RTK FIXED; arrives within position tolerance AND heading
tolerance (values TBD from first sessions); never exits working area / exclusions.

**Analysis:** `aggregate` reproduces the headline plot, the Wilcoxon table, and the
compute-cost figure from a bag directory in one command.

## 6. Constraints & invariants (non-negotiable)

- Firmware **1.0 m/s** total wheel-speed cap; geometric **R_min ≈ 0.37 m**.
- **Ackermann** mode (`motion_mode == 1`) required and checked before wheels-on-floor.
- **ADR-01**: RTK never fused into the control loop.
- Safety chain (C3) and E-stop semantics (C4) are inviolable.
- Wheels-on-floor tests require explicit operator confirmation; synthetic-odom (wheels up) does not.
- Never push to `origin` without explicit request; no AI-attribution in commits.

## 7. Out of scope (this phase)

- Indoor / OptiTrack venues (deferred until longer USB cables) — schema reserves space, no flow exercised.
- LIMO firmware modification; replatforming to a faster vehicle.
- MPC baseline (no in-package implementation).
