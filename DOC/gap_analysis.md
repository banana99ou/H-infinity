# Gap analysis & task backlog — H∞ data-gathering system

Derived from `DOC/system_spec.md` (the locked ROC) vs the current codebase. Part A is the
ROC→code gap matrix. Part B is a dependency-ordered task backlog where **each task is a
self-contained brief a fresh agent can execute cold**. Part C is shared context every task
agent needs (read it once before starting any task).

Legend: ✅ met · ⚠️ partial · ❌ missing.

---

## Session update — 2026-05-26 (hardware-verified on the live robot)

First on-robot pass. Brought up `base_gnss`, ran the missing-info sweep, deployed
the T1/T3/T5/T7/T9 work, and verified with wheels-up synthetic odom.

**Verified ✅ (synthetic, wheels up):**
- **T1** — recipe → exact analytic curve: reported `total_length` == analytic
  `4.7854 m` *exactly* (not a Bezier re-spline); curvature a sharp `0→1/R` step;
  `/path_follower/done` latches; `/path_follower/timing` mean **0.43 ms/cycle**.
  → P1, P4, O3(done), A3(timing) closed.
- **T3** — odom-zero overlay: mirror before reset → `(0,0,0)` at reset → SE(2)-
  relative after, raw `/wheel/odom` untouched. → L3 closed.
- **T7** — `BagRecorder` records a real bag, all 4 new topics captured, sidecar
  parses with all D3 keys. → D1, D3 closed.
- **T9** — RTK-FIXED gate FAILs on NO-FIX with exit code 1 (gate-able);
  `cmd_vel_raw` publisher check present; battery/motion_mode/odom checks pass.

**Corrected:** T5 now reads RTK-FIXED from the `rtk_status` string (`quality=4`),
not `NavSatFix.status` (which can't distinguish RTK on this F9P). Static-checked
only (browser UI unverified).

**Still blocked on a physical condition:** open-sky **RTK FIXED** (was NO-FIX
indoors) to confirm the `quality=4` literal; **R_min** measurement (U-turn);
wheels-on-floor for T4/T6.

Robot-verification checklist items 1–4 are now resolved (see Part C).

---

## Part A — Gap matrix (ROC → current state)

### 3.1 Reference path generation
| ROC | State | Evidence |
|---|---|---|
| P1 analytic step-curvature | ✅ | Delivered analytically via `/reference_path_recipe` (T1, `path_follower_node.py` `build_path_from_recipe`). **hw-verified**: curvature a sharp `0→1/R` step. |
| P2 analytic slalom | ⚠️ | `SlalomPath` exists (`vfg_pathfollowing/paths/slalom.py`), same delivery gap. |
| P3 turnaround (U-turn, 3-point) | ❌ | U-turn expressible as `StepCurvaturePath(theta_arc=π)` but not wired; 3-point needs reverse, which the follower can't do (`path_follower_node.py:309` sets `linear.x=v_const≥0`). |
| P4 deliver without distortion | ✅ | Recipe builds the analytic path in-node (T1); **hw-verified** reported `total_length` == analytic exactly. Bezier path kept only for ad-hoc P5. |
| P5 ad-hoc Bezier | ✅ | `_path_cb` (`path_follower_node.py:193-218`) + battle-station path designer. |

### 3.2 Control & safety
| ROC | State | Evidence |
|---|---|---|
| C1 LPV/PID selectable | ✅ | `controller_type` param (`path_follower_node.py:59,94-104`). |
| C2 constant v ≤1.0 | ✅ | `v_const` param (`:72`); firmware enforces the 1.0 cap (node clamps to 3.0). |
| C3 cmd_vel_raw→estop→/cmd_vel | ✅ | follower pubs `cmd_vel_raw` (`:142`); `estop_cli.py` gates to `/cmd_vel`; preflight asserts pub/sub sets. |
| C4 estop latched/remote/zero | ✅ | `estop_cli.py`, `/estop_trigger`. |
| C5 odom-timeout failsafe | ✅ | `path_follower_node.py:260`. |
| C6 single cmd_vel_raw publisher | ⚠️ | Today only follower (+teleop) publishes. Adding reposition means exclusivity must be enforced by the sequencer; preflight checks `cmd_vel_raw` *subscribers* only (`preflight.sh:94`), not publishers. |

### 3.3 Localization & ground truth
| ROC | State | Evidence |
|---|---|---|
| L1 odom-only feedback in run | ✅ | follower subscribes `/wheel/odom` only (`path_follower_node.py:121`). |
| L2 RTK recorded as GT | ⚠️ | RTK published (`GPS-RTK_ROS2_pub_node.py`) and in `Data_Logger.py` TOPICS, but not yet per-run sequenced. |
| L3 odom reset w/o disturbing RTK | ✅ | `odom_zero_node` (T3) re-anchors via SE(2) offset; **hw-verified** + confirmed no native `limo_base` reset service exists. Raw `/wheel/odom` untouched. |
| L4 RTK between-run only | ✅ | Invariant holds by design (ADR-01); no reposition yet to violate it. |
| L5 regular GPS recorded (sep. dataset) | ✅ | `/pixhawk/...` topics in `Data_Logger.py:34-36`. |

### 3.4 Repositioning — **all ❌** (no reposition node exists)
R1 RTK go-to-pose / R2 straight-approach heading / R3 abort on area breach / R4 no-op when start==end. Reuse: `local_to_latlon()` (`tools/path_gen/path_overlay.py:182-206`) — needs an inverse.

### 3.5 Venue & working area — **all ❌**
V1 persist venue / V2 RTK corner pins / V3 exclusion-aware placement / V4 validation. Today the rooftop anchor is hardcoded (`interactive.html` ~219-226, `path_overlay.py:33-38`); no `scenarios/venues/`. INI scenarios (`scenarios/*.ini`) are unrelated (legacy open-loop).

### 3.6 Orchestration / autonomy
| ROC | State | Evidence |
|---|---|---|
| O1–O4 sequencer/matrix/cycle/retry | ❌ | No sequencer node. |
| O5 start/stop via battle station | ⚠️ | Supervisor exists (`orchestrator_node.py`, `PROCS`) + battle-station buttons; the sequencer just needs adding to `PROCS`. |

### 3.7 Recording & data management
| ROC | State | Evidence |
|---|---|---|
| D1 bag per leg, full topic set | ✅ | `Data_Logger.py` TOPICS now include the 4 follower topics; `BagRecorder` (T7) records the full set per leg. **hw-verified** all 4 present. Per-leg sequencing comes with T6. |
| D2 deterministic layout + run-ID | ⚠️ | Naming scheme exists (`Data_Logger.py:45-51`) but no run-ID/cell pairing. |
| D3 sidecar JSON | ✅ | `build_sidecar`/`write_sidecar` (T7), all D3 fields; **hw-verified** parses next to the bag. |
| D4 pass/fail classification | ❌ | none. |

### 3.8 Monitoring & alerting
| ROC | State | Evidence |
|---|---|---|
| M1 preflight gate | ⚠️ | `preflight.sh` now adds an RTK-FIXED gate + `cmd_vel_raw` publisher check (T9, **hw-verified** FAIL→exit 1). Still standalone (T6 must invoke it per-cell) and run-window RTK-% (Y) unset. |
| M2 battery alert/halt | ⚠️ | `/limo_status.battery_voltage` available; preflight warns <10.8 V / fails <10.5 V. **Spec says 30%/20% but telemetry is VOLTS** — thresholds must be restated in volts (open item). No runtime alert/halt. |
| M3 wallclock heartbeat | ❌ | none. |
| M4 ntfy alerts | ❌ | no notification code in repo. |
| M5 live batch progress | ❌ | battle station shows follower telemetry only. |

### 3.9 Analysis
| ROC | State | Evidence |
|---|---|---|
| A1 per-bag metrics ×2 | ⚠️ | `compute_metrics()` (`vfg_pathfollowing/simulation/metrics.py`) does RMS/max e_psi/e_d on a `SimResult`; **no bag→SimResult converter**, no odom-belief vs RTK-truth split, terminal-pose-error + steering-effort need adding. |
| A2 aggregation/Wilcoxon/plot | ❌ | none (scipy available). |
| A3 compute-cost | ⚠️ | Per-cycle timing now published on `/path_follower/timing` (T1, **hw-verified** 0.43 ms mean); the aggregation/figure (T11) is still missing. |

### 3.10 Fault handling
| ROC | State | Evidence |
|---|---|---|
| F1 safe-state stop-in-place | ⚠️ | estop zeros output; follower zeros on timeout/shutdown — primitives exist, no batch-level halt. |
| F2 persistent RTK loss → pause+alert | ❌ | none. |
| F3 run RTK dropout → fail | ❌ | none (needs classification + sequencer). |
| F4 circuit breaker | ❌ | none. |
| F5 remote abort/pause/resume | ⚠️ | remote abort exists (`/estop_trigger` over rosbridge:9090); batch pause/resume missing. |
| F6 checkpoint & resume | ❌ | none. |
| F7 connectivity-loss policy | ⚠️ | estop runs `--no-ping` in `PROCS` (`orchestrator_node.py:39`), remotely abortable — behavior exists, not formalized/tunable. |
| F8 env-hazard handling | ❌ | depends on F5 pause + F6 resume. |

**Summary:** the control + safety + sensing *primitives* are largely ✅; everything that turns
them into an **unattended, repeatable, analyzable batch** (paths-as-recipes, odom reset,
reposition, venue, sequencer, recording integration, alerting, analysis, fault handling) is
⚠️/❌. That's the build.

---

## Part B — Task backlog (agent-ready briefs)

Dependency order: **T1, T3, T5, T7, T9, T11-prep can start in parallel.** T4 needs T5's venue
schema (can stub). **T6 (sequencer) is the integrator** — needs T1, T3, T4, T7. T8, T10 hook
into T6. T11 needs T1 (timing) + real bags from T6/T7.

```
T1 path-recipe ─┐
T3 odom-zero  ─┼─► T6 sequencer ─► T8 notify
T5 venue ─► T4 reposition ─┘        └─► T10 battle-progress
T7 recording ─┘
T9 preflight (standalone)        T11 analysis (needs T1 + bags)
```

Each brief: read **Part C first**. Paths are absolute-from-repo-root.

---

### T1 — Analytic path delivery (recipe → exact curve) + done + timing
**ROC:** P1, P2, P3(U-turn), P4, O3(done), A3(timing). **Status:** ⚠️/❌ → ✅.
**Objective:** make the follower track the *exact* analytic curve from a recipe, emit a crisp
completion signal, and time each control cycle.
**Modify:** `scalecar-vfg-h-infinite/ros2_bridge/limo_path_follower/path_follower_node.py`.
- Add latched sub `/reference_path_recipe` (`std_msgs/String`, JSON) → `build_path_from_recipe(d)` mapping `{type:"step"|"slalom"|"uturn", params:{...}}` to `StepCurvaturePath`/`SlalomPath` (U-turn = `StepCurvaturePath(theta_arc=π, R=R_min)`). Set `self.path`/`self.guidance` via the same atomic swap as `_path_cb` (`:217-218`).
- Keep `_path_cb` (Bezier) for P5.
- Add latched pub `/path_follower/done` (`std_msgs/Bool`), set True where the end check fires (`:277`).
- Wrap guidance+controller block (`:268-303`) with `time.perf_counter_ns()`; publish to `/path_follower/timing` (`std_msgs/Float32` ms) or append to status.
- Optionally publish the analytic path sampled as `nav_msgs/Path` on `/reference_path` for viz+bag.
**Reuse:** path constructors confirmed — `StepCurvaturePath(L1,R,theta_arc,L2,direction)`, `SlalomPath(R,theta_arc,L1,L_mid,n_arcs,L_end)`; both already start at origin heading +x.
**Deps:** none. **Verify:** synthetic odom (wheels-up, no OK needed) — push a step recipe, confirm `/path_follower/status.kappa` shows an instant 0→1/R step (not a ramp), `/path_follower/done` latches at end, timing publishes.

### T3 — Odom-zeroing overlay
**ROC:** L3. **Status:** ❌ → ✅.
**Objective:** reset odom origin on command without restarting the GNSS stack.
**Create:** `scalecar-vfg-h-infinite/ros2_bridge/limo_path_follower/odom_zero_node.py` — sub `/wheel/odom`, republish `/wheel/odom_zeroed` with a latched SE(2) offset; sub `/odom_zero/reset` (`std_msgs/Bool`) latches current pose as new origin. Register in `ros2_bridge/setup.py` console_scripts.
**Wire:** the follower must consume zeroed odom — add a launch **remap** `/wheel/odom→/wheel/odom_zeroed` (the sub topic is hardcoded at `path_follower_node.py:121`; remap at the orchestrator `PROCS` entry, do not edit the hardcode). Bag still records raw `/wheel/odom`.
**Robot-prereq:** check whether `limo_base` exposes a native odom-reset service first (Part C checklist) — if so, prefer it and skip the overlay.
**Deps:** none. **Verify:** synthetic odom — drive to nonzero pose, call reset, confirm `/wheel/odom_zeroed` jumps to 0 while raw is unchanged.

### T4 — RTK reposition node (go-to-pose + straight approach + area-aware + 3-point)
**ROC:** R1, R2, R3, P3(3-point). **Status:** ❌ → ✅.
**Objective:** autonomously drive to a start pin on RTK feedback, arriving on a deterministic heading; own reverse maneuvers (3-point turn).
**Create:** `scalecar-vfg-h-infinite/ros2_bridge/limo_path_follower/reposition_node.py` — sub `/gps_rtk_f9p_helical/gps/{fix,rtk_status}`; pub `cmd_vel_raw` (through estop); cmd via `/reposition/goto` (JSON lat/lon/heading) + `/reposition/status`. Proceed only on RTK FIXED. Closed-loop go-to-pose: heading-align (course-over-ground from successive fixes while moving) → approach → **straight final segment along target heading**. Abort+alert if path would exit working area / enter exclusion. Register in setup.py.
**Reuse:** `local_to_latlon()` (`tools/path_gen/path_overlay.py:182-206`) — **add `latlon_to_local()`** there (shared anchor). RTK fix-quality mapping in `GPS-RTK_ROS2_pub_node.py`.
**Safety:** **must never run concurrently with the follower** (C6) — sequencer enforces; until then test alone.
**Deps:** T5 venue schema (can stub a single target). **Verify:** wheels-on-floor on rooftop — **requires explicit operator OK** (Part C). Confirm arrival position+heading tolerance; confirm abort on simulated area breach.

### T5 — Venue config + working-area editor (battle station)
**ROC:** V1, V2, V3, V4. **Status:** ❌ → ✅.
**Objective:** persist drift-immune venue geometry and place curves around the central island.
**Create:** `scenarios/venues/rooftop.json` (schema in `system_spec.md` §3.5 + `experiment.md`): `corners_wgs84[4]`, `safety_margin_m`, `exclusions[]` (circles incl. the central island), `start_pins[]`/`end_pins[]` (lat/lon+heading).
**Modify:** `tools/path_gen/interactive.html` — subscribe `/gps_rtk_f9p_helical/gps/fix`; "pin corner" captures live RTK at click; draw inset rectangle (corners−margin) + circular exclusions + draggable start/end pins w/ heading (reuse existing heading-tip marker); validate curves+turnarounds fit inset polygon & clear exclusions (reuse the JS path generators already in the file); export venue JSON.
**Reuse:** rooftop anchor + `local_to_latlon` already in the file/`path_overlay.py`; existing rosbridge client + marker code.
**Deps:** none. **Verify:** produce a real `rooftop.json` from corner pins; render all matrix curves on the map and confirm containment + island clearance.

### T7 — Recording integration + sidecar
**ROC:** D1, D2, D3. **Status:** ⚠️/❌ → ✅.
**Objective:** record one bag per leg with the full topic set and a paired sidecar.
**Modify:** `Data_Logger.py` TOPICS (`:30-42`) — add `/reference_path`, `/path_follower/status`, `/path_follower/done`, `/path_follower/timing`; keep RTK + `/pixhawk/...` (L5) + `/cmd_vel{,_raw}` + `/wheel/odom` + `/estop` + `/imu`.
**Create:** a programmatic start/stop wrapper (the sequencer will call it) + per-leg sidecar JSON writer (fields in `system_spec.md` D3). Prefer driving `ros2 bag record` as a subprocess from the sequencer over the interactive Ctrl+C flow.
**Reuse:** `Data_Logger.py` naming + dir convention (`:45-51`), `/data_logger/{recording,health}` indicators.
**Deps:** T1 (the new topics must exist). **Verify:** record a synthetic run; confirm every required topic present for the full window; sidecar parses.

### T6 — Experiment sequencer node (the integrator)
**ROC:** O1–O5, C6, R4, D4, F1–F4, F6, F8. **Status:** ❌ → ✅.
**Objective:** drive the full matrix unattended.
**Create:** `scalecar-vfg-h-infinite/ros2_bridge/limo_path_follower/experiment_sequencer_node.py`; register in setup.py; add `'sequencer'` to `PROCS` in `orchestrator_node.py:31`.
**Create:** `scenarios/experiment.yaml` (venue ref, matrix axes, N, gating thresholds, retry limit, ntfy config).
**Behavior:** load experiment.yaml + venue json; per-cell cycle driving the orchestrator via `/orchestrator/start|kill` (watch `/orchestrator/status`): preflight gate → start `reposition`, `/reposition/goto` A, wait, kill → `/odom_zero/reset` → start bag (T7) → start `follower` (set `controller_type`/`v_const` via `/path_follower_node/set_parameters`), publish recipe to `/reference_path_recipe` → wait `/path_follower/done`/estop/timeout → stop bag, kill follower, sidecar, classify (D4) → turnaround (U-turn recipe via follower; 3-point via reposition) → return leg → next. **Enforce exactly one cmd_vel_raw publisher (C6).** Auto-retry (O4); circuit-breaker after K fails (F4); RTK-FIXED gating + dropout→fail (F2/F3); checkpoint passed cells to disk + resume (F6); safe-state = stop-in-place (F1).
**Reuse:** orchestrator `PROCS`/start/kill semantics; `/path_follower/done` (T1).
**Deps:** T1, T3, T4, T7 (+ T5 venue, T9 preflight ideally). **Verify:** one full cell end-to-end on rooftop (**operator OK**): A→B, turnaround, B→A; two bags+sidecars; correct classification; kill power mid-batch and confirm resume skips the passed leg.

### T8 — Notifications + battery/wallclock/RTK monitors
**ROC:** M2, M3, M4, F2/F4 alerts. **Status:** ❌/⚠️ → ✅.
**Create:** `tools/notify/ntfy.py` (single POST to an ntfy topic from `experiment.yaml`).
**Modify:** sequencer (T6) hooks — batch start/complete, per-failure, **battery: alert/halt in VOLTS** (resolve the %→V open item; align with preflight's 10.8/10.5 V or a user-confirmed curve), wallclock heartbeat (M3).
**Reuse:** `/limo_status.battery_voltage` (see `preflight.sh:109`).
**Deps:** T6. **Verify:** trigger a test push; simulate low voltage → confirm halt.

### T9 — Preflight hardening + RTK gate
**ROC:** M1, C6(publisher check), F7. **Status:** ⚠️ → ✅.
**Modify:** `tools/preflight/preflight.sh` — add an **RTK-FIXED** check on `/gps_rtk_f9p_helical/gps/rtk_status`; add a `cmd_vel_raw` **publisher-set** assertion (single publisher, C6); implement the `--full` dynamic test placeholder (`:136-142`).
**Reuse:** existing check idioms (`:75-98` pub/sub parsing, `:102-119` `/limo_status`).
**Deps:** none (standalone); consumed by T6. **Verify:** run on the robot; FIXED check passes only with RTK fixed.

### T10 — Battle-station live progress + pause/resume
**ROC:** M5, F5. **Status:** ❌/⚠️ → ✅.
**Modify:** `tools/path_gen/interactive.html` — subscribe to a sequencer status topic (define in T6, e.g. `/experiment/status` JSON: current cell, ETA, pass/fail tally); add pause/resume/abort controls (publish to a sequencer command topic; abort still also has the `/estop_trigger` path).
**Deps:** T6. **Verify:** drive a batch; confirm progress + pause/resume from the browser.

### T11 — Analysis pipeline
**ROC:** A1, A2, A3(plot). **Status:** ⚠️/❌ → ✅.
**Create:** `tools/analysis/run_eval.py` — read a bag, reconstruct trajectory from `/wheel/odom` and RTK, rebuild the analytic reference from the sidecar recipe, compute e_d/e_psi via `PathBase.signed_distance()` + `VectorFieldGuidance`, assemble a `SimResult`-shaped object, call `compute_metrics()`; emit metrics **twice** (odom-belief + RTK-truth); add terminal-pose-error + steering-effort.
**Create:** `tools/analysis/aggregate.py` — per-cell stats over N, `scipy.stats.wilcoxon` LPV vs PID, headline plot (max heading err vs R), Curvature Tolerance Index, compute-cost figure from `/path_follower/timing`.
**Reuse:** `compute_metrics()` (`vfg_pathfollowing/simulation/metrics.py`), `SimResult` (`.../simulation/result.py`), path classes for the analytic reference.
**Deps:** T1 (timing) + bags/sidecars from T6/T7. **Verify:** run on T6 bags; reproduce a headline plot + a Wilcoxon p-value.

---

## Part C — Shared context for every task agent

**What this repo is:** hardware bring-up of Prof. Suwon Lee's `scalecar-vfg-h-infinite`
(VFG + LPV-H∞) path follower on an AgileX LIMO. The robot is the source of truth; the laptop
has **no ROS2**. Read `CLAUDE.md`, `DOC/system_spec.md`, `DOC/experiment.md`, and
`DOC/decisions/01_gps_no_fusion.md` before starting.

**Dev cycle (never skip):** edit on laptop → `rsync` to NUC → SSH in → `colcon build
--packages-select limo_path_follower` (no `--symlink-install`) → verify on the robot graph.
Do not claim done from static checks; show runtime evidence (log excerpts, `ros2 topic echo`).
SSH/rsync use `expect` with a per-session password (not in the repo). See `CLAUDE.md` for the
exact patterns and the four NUC deployment caveats.

**Hard constraints (inviolable):**
- Only actuation path: `controller → cmd_vel_raw → estop_cli.py → /cmd_vel`. **Never** publish `/cmd_vel` from a controller node.
- **Exactly one** `cmd_vel_raw` publisher live at a time (follower XOR reposition).
- Wheels-on-floor tests require **explicit operator confirmation**; synthetic-odom (wheels up) does not.
- ADR-01: RTK never enters the control loop (reposition uses it only *between* recorded runs).
- No `Co-Authored-By: Claude` / AI attribution in commits. Never push to `origin` without explicit request.
- Don't touch `DOC/project_spec.md` (user-owned).

**Key file map:**
- Follower: `scalecar-vfg-h-infinite/ros2_bridge/limo_path_follower/path_follower_node.py`
- Supervisor: `.../limo_path_follower/orchestrator_node.py` (`PROCS` dict = where managed nodes register)
- Entry points: `scalecar-vfg-h-infinite/ros2_bridge/setup.py` (`console_scripts`)
- Estop: `estop_cli.py` (repo root, runs on NUC at `/home/agilex/H-infinity/estop_cli.py`)
- Bag recorder: `Data_Logger.py` (TOPICS list `:30-42`)
- RTK driver: `GPS-RTK_ROS2_pub_node.py` (namespaced `/gps_rtk_f9p_helical`)
- Launch (GNSS stack): `src/limo_ros2/limo_base/launch/LIMO+MAVROS+RTK_Node_Launcher.launch.py`
- Battle station: `tools/path_gen/interactive.html` (rosbridge `ws://…:9090`); geo helpers `tools/path_gen/path_overlay.py`
- Preflight: `tools/preflight/preflight.sh`
- Path lib: `scalecar-vfg-h-infinite/vfg_pathfollowing/paths/` ; metrics `.../simulation/metrics.py`

**Confirmed interface facts:**
- `/limo_status` is `limo_msgs/msg/LimoStatus`: `motion_mode` (1 = Ackermann), `control_mode`, `battery_voltage` (float64 VOLTS — 12.0 live, preflight warns <10.8, fails <10.5), `vehicle_state`, `error_code`. (hw-confirmed 2026-05-26.)
- `/path_follower/status` is a `Float32MultiArray`: `[x,y,yaw,v,s_star,total_length,kappa,rho,e_psi,delta_cmd,has_path]`.
- `/reference_path` is `nav_msgs/Path`, latched (TRANSIENT_LOCAL), frame must equal `odom_frame`.
- Recipe in on `/reference_path_recipe` (`std_msgs/String` JSON, TRANSIENT_LOCAL); done on `/path_follower/done` (`std_msgs/Bool`, latched); timing on `/path_follower/timing` (`std_msgs/Float32` ms). (T1, hw-verified.)
- `/wheel/odom` is `nav_msgs/Odometry` @ ~50 Hz from `limo_base_node`; `/cmd_vel` sink is `limo_base_node` (1 sub).
- RTK: `/gps_rtk_f9p_helical/gps/fix` is `sensor_msgs/NavSatFix` (lat/lon for pinning), but `status.status` does **not** distinguish RTK FIXED — use the `rtk_status` `quality=4` token instead.
- Path classes start at origin heading +x ⇒ a freshly-zeroed odom aligns the analytic curve.

**Robot-verification checklist — RESOLVED 2026-05-26 (live graph):**
1. ✅ `limo_base` exposes **no** odom-reset service (only MAVROS `*/reset|clear`). → T3 overlay is the mechanism.
2. ✅ `/limo_status` is `limo_msgs/msg/LimoStatus`; `battery_voltage` is **float64 VOLTS** (live 12.0–12.1), `motion_mode=1` (Ackermann), `control_mode=1`.
3. ✅ `rtk_status` is `std_msgs/String`, a rich line carrying a `quality=N` token (NO-FIX gave `quality=0`). FIXED → `quality=4` (driver `fix_quality_to_desc`); **literal still pending an open-sky FIXED**. Reacquisition time not measured (no fix indoors).
4. ⏳ Reposition tolerances + battery volt thresholds (M2) still open — need a wheels-on-floor session and a user decision on the %→V mapping (observed 12.0 V healthy).

**Open spec items to resolve with the user (not blocking most tasks):** battery %→volts
mapping (M2), exact RTK-FIXED dwell K and run-window RTK % Y (M1/§5), retry limit + circuit-
breaker K (O4/F4), bag-length tolerance X (§5).
