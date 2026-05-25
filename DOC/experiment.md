# Hardware verification experiment

The point of this hardware bring-up: produce a dataset sufficient for a
follow-up paper to the sim-based H∞ paper (`DOC/paper_ijat.pdf`). This doc
is the single source of truth for the experiment — what we are proving,
why the claim shape changed from the sim paper, the venue setup, the
experimental matrix, and the orchestrator target state needed to run the
matrix unattended.

Related:
- `DOC/decisions/01_gps_no_fusion.md` — why RTK stays out of the control loop.
- `DOC/system_spec.md` — locked requirements + canonical interface contract (§4).
- `DOC/paper_ijat.pdf` — the sim paper this work follows up on.

## Headline claim

The follow-up paper does **not** attempt to replicate the sim paper's
*speed-invariant performance* claim on hardware. Instead it repositions the
LPV H∞ controller as **specialized for the small-radius / high-curvature
regime**, with the speed axis collapsed to v ≈ 1.0 m/s and the experimental
sweep moved onto the curvature axis (R, equivalently κ = 1/R).

The bandwidth-mismatch story (ρ = κv) survives the pivot unchanged: holding
v fixed and sweeping κ traverses the same scheduling parameter space ρ that
the LPV controller was synthesized over, just along a different axis through
it.

## Why this pivot

1. **The sim claim is not reachable on this hardware.** The AgileX LIMO
   enforces a firmware-level total wheel-speed cap at **1.0 m/s** (chassis
   motor controller; not configurable from ROS, not bypassable from
   userland). The sim paper sweeps 0.5–3.0 m/s. The upper half of that range
   is physically unreachable. A "speed-invariant" claim across 0.5–1.0 m/s
   alone is too narrow to be the headline.

2. **The H∞ advantage is concentrated where LIMO can operate.** In sim at
   v = 1.0 m/s:

   | R [m] | PID max heading err [deg] | LPV max heading err [deg] | winner |
   |---|---|---|---|
   | 1.0  | 1.92  | 7.48  | PID |
   | 0.5  | 7.26  | 12.92 | PID |
   | 0.3  | 35.19 | 18.35 | **LPV** |
   | 0.2  | 61.39 | 23.34 | **LPV** |
   | 0.1  | 84.46 | 35.15 | **LPV** |

   Crossover at R ≈ 0.3–0.4 m. LIMO's geometric R_min ≈ 0.37 m, so the
   platform can be driven into the regime where LPV wins by design. PID
   degrades faster than LPV as R shrinks — this *is* the new headline.

3. **ρ = κv is the real scheduling axis, not v.** The paper's theoretical
   contribution is bandwidth-matched performance weighting through ρ.
   Sweeping κ at fixed v moves through the same ρ values as sweeping v at
   fixed κ. The LPV synthesis (six vertices, ρ ∈ {0..5}) is exercised the
   same way. The "speed-invariance" framing was one of multiple equivalent
   slices through ρ; pivoting to a curvature slice loses none of the
   underlying claim.

4. **Aligns with professor's framing.** Confirmed by Suwon Lee on Slack:
   "the paper's core point is that guidance-bandwidth is faster than
   control-bandwidth, captured by H∞ structure. Since we can't increase
   speed, increase curvature instead" → after seeing the sim crossover:
   "position it as a controller specialized for small radii."

### Alternatives considered and rejected

| Approach | Why rejected |
|---|---|
| Sweep speed 0.3–1.0 m/s and claim partial speed-invariance | Range too narrow; sim paper already covers it; not a publishable new claim. |
| Modify LIMO firmware to remove the cap | Out of scope; risks bricking the platform; voids any baseline claim of "off-the-shelf scale car." |
| Switch to a different scale platform (ScaleCar v3, F1Tenth, etc.) | Months of bring-up redo; loses every NUC deployment fix already in place. |
| Drop the paper, do operational deployment only | The whole point of this hardware bring-up is to support the paper. |

### When to reopen the pivot

- Firmware cap lifted (unlikely without AgileX firmware access).
- Replatform to a vehicle that can reach 3 m/s — then the original
  speed-invariance claim becomes reachable and the pivot is undone.

## Hardware constraints that shape the experiment

| Constraint | Source | Consequence |
|---|---|---|
| Total wheel speed capped at **1.0 m/s** (strict, firmware-level, not configurable from ROS) | LIMO chassis motor controller | Speed sweep impossible above 1.0 m/s. Turning at v_des = 1.0 reduces forward speed. |
| Geometric **R_min ≈ 0.37 m** | LIMO Ackermann linkage | Practical curvature floor for non-saturated runs ≈ R = 0.4 m. U-turn at R_min is the tightest planned maneuver. |
| **Ackermann mode required** (`motion_mode: 1` on `/limo_status`) | Controller assumes bicycle model | Confirm before every wheels-on-floor run. |
| **No `/cmd_vel` direct publishing from controller** | ADR-01, system_spec C3 | All commands go controller → `cmd_vel_raw` → `estop_cli.py` → `/cmd_vel`. |
| **RTK only, no fusion (for the controller)** | ADR-01 | Wheel odom feeds the controller during a recorded run; RTK is post-hoc ground truth. RTK *may* drive the robot **between** recorded runs (reposition). |
| **No in-place 180° on Ackermann** | Kinematics | Turnarounds are tracked U-turn or 3-point paths — see operational model. |

## Venue setup

The orchestrator stores per-venue configuration that survives across days
and reboots. Outdoor venues use RTK lat/lon as the persistent frame.
Indoor venues are deferred until longer OptiTrack USB cables arrive — the
config schema reserves space for them but no indoor flow is exercised yet.

### Frame strategy

| Frame | Outdoor (rooftop, B1) | Indoor (lab) |
|---|---|---|
| **RTK lat/lon (ENU)** | ✓ primary | ✗ no sky |
| **OptiTrack room frame** | ✗ | ⏳ deferred (awaiting cables) |

Working area, start positions, and end positions are stored as **WGS84
lat/lon (+ heading)** in a per-venue JSON committed to the repo (e.g.
`scenarios/venues/rooftop.json`).

### Working area: rectangle minus exclusions

A working area is a **rectangle inside the testing track**, with an
**inset safety margin** that all curves and turnaround paths must fit
inside, **minus circular exclusion zones** (e.g., the island in the middle
of the rooftop track).

Calibration procedure (one-time per venue, redo after any layout change):

1. Operator manually drives the LIMO (teleop) to **each of 4 corners** of
   the desired rectangle and clicks "pin corner" in the battle station map.
   The orchestrator records the live RTK lat/lon at click time as each
   corner's coordinate.
2. Operator sets a **safety margin** (m, default 0.5 m) inset from the
   rectangle edges. All paths must fit inside the inset polygon.
3. Operator marks **exclusion zones** (circular: center lat/lon + radius)
   for known obstacles like the rooftop island. Paths must not intersect
   these.
4. Operator marks **start/end pins** within the inset polygon (and outside
   exclusions), each with a heading. These are what the orchestrator drives
   to before each cell.
5. Config is committed as `scenarios/venues/<venue>.json`.

### Per-venue JSON (sketch)

```
{
  "name": "rooftop",
  "frame": "rtk_enu",
  "corners_wgs84": [{lat, lon}, x4],
  "safety_margin_m": 0.5,
  "exclusions": [{kind: "circle", lat, lon, radius_m}, ...],
  "start_pins": [{id, lat, lon, heading_deg}, ...],
  "end_pins":   [{id, lat, lon, heading_deg}, ...]
}
```

## Experimental matrix

| axis | values |
|---|---|
| **controller** | `lpv-hinf`, `pid-ff` |
| **v_const** | **1.0 m/s** (primary); **0.5 m/s** (sim-comparable sanity point) |
| **path family** | **step-curvature** (paper primary), **slalom** (paper §5) |
| **R** | {1.0, 0.7, 0.5, 0.4} m — spans PID-favored → crossover → LPV-favored regime |
| **N** | **10** repetitions per cell |

Total: 2 × 2 × 2 × 4 × 10 = **320 recorded headline runs** (turnarounds
are bagged separately and not counted in the matrix). At ≈ 90–120 s wall
time per recorded run (motion + reposition + turnaround + bag archive),
~8–10 hours pure run time, split across 5–6 battery-limited sessions.

**Headline plot for the paper:** max heading error vs R at v = 1.0 m/s,
both controllers overlaid, error bars across N = 10. The crossover point
on hardware vs sim is the headline number.

### What stays from the sim paper

- Path families: **step-curvature** (primary), **slalom**. Closest
  LIMO-feasible approximation, direct numerical comparability with the sim
  results table.
- Baseline: **PID-FF** (already in `vfg_pathfollowing/controllers/pid_ff.py`).
  MPC drops out — no in-package implementation, synthesizing one for
  hardware is its own paper.
- Metrics: RMS e_d, max e_d, terminal pose error, RMS heading error,
  steering effort. Reported twice (wheel-odom belief / RTK truth) per ADR-01.
- Statistical test: Wilcoxon, N = 10 per cell.

## Operational model

Persistent start/end positions live in RTK lat/lon. ADR-01 still holds:
RTK drives the robot **between** recorded runs, never **inside** one. The
controller under test only ever sees `/wheel/odom` during a bagged run.

### Per-cell cycle (forward leg + return leg)

For each cell `(controller, v, path, R)`, one cycle:

1. **Pre-flight gate**: RTK fix quality ≥ threshold sustained for K
   seconds; topics healthy; battery ≥ low-battery threshold.
2. **Reposition to start A** (RTK-direct, see below). Reset `/wheel/odom`
   on arrival.
3. **Bag start** for the forward leg.
4. **Push reference path A→B**, generated from cell parameters starting at
   the robot's current odom pose — which is `(0, 0, 0)` after the reset.
   Controller sees `/wheel/odom` only.
5. **Run**. Detect completion via `/path_follower/status` + endpoint
   tolerance, E-stop, or timeout.
6. **Bag stop**, write sidecar JSON (run-ID, cell parameters, RTK fix
   history, classification).
7. **Turnaround at B**: push a **tracked turnaround path** (U-turn at R_min
   by default; 3-point turn fallback when U-turn doesn't fit at the current
   pose / inside the inset polygon / outside exclusions). Bagged separately,
   classified separately from headline data.
8. **Bag start** for the return leg. Reset odom first.
9. **Push reference path B→A** (mirror of step 4). Controller sees
   `/wheel/odom` only.
10. **Bag stop**, sidecar JSON.
11. **Turnaround at A** (same as step 7, opposite direction) if more
    repetitions remain for this cell.
12. Loop: next repetition or next cell. If next cell's start pin equals the
    current cell's end pin, reposition is a no-op.

### Reposition between cells (RTK-direct)

Reposition is the only place GPS touches the motion stack. ADR-01 still
holds because reposition is not inside any recorded experiment run.

1. Read current RTK lat/lon and heading.
2. Read target lat/lon and heading from venue config.
3. Closed-loop drive on RTK feedback (simple go-to-pose: bearing → heading
   error → angular cmd; range → linear cmd, low v). No odom needed, no
   calibration transform to maintain.
4. Terminate when RTK position within tolerance of target **and** RTK
   heading within tolerance.
5. **Reset `/wheel/odom`** so the recorded run starts at `(0, 0, 0)` in
   odom at the known physical location. (Mechanism still TBD — see open
   questions.)
6. On failure (would exit working area, target unreachable, RTK quality
   degraded): ping operator, pause batch.

### Turnaround as a tracked path (per Ackermann)

180° in place is not available on Ackermann. The turnaround is a planned
maneuver expressed as a reference path the controller tracks like any
other run:

- **Default**: tight U-turn semicircle at R = R_min (~0.4 m), centered so
  the swept area stays inside the inset polygon and outside any exclusion
  zone.
- **Fallback** when the U-turn footprint doesn't fit at the current pose:
  3-point turn (forward arc → reverse arc → forward arc) with each segment
  a separate tracked path in sequence.

Turnaround paths are **bagged and pass/failed** like headline runs but are
**not part of the matrix** — they are operational glue. A failed turnaround
pings the operator and pauses the batch (likely indicates pose drift or a
misconfigured working area).

### Odom drift expectations

| Scope | Drift magnitude | Treatment |
|---|---|---|
| Within a single recorded run (≤ 60 s, ≤ 30 m) | cm to ~m | **Accepted, not corrected.** This is the disturbance ADR-01 says the controller absorbs. The odom-belief vs RTK-truth gap is the headline metric. |
| Between runs | Reset to zero on every reposition arrival | Each recorded run starts with odom = 0 at a known lat/lon. Drift cannot accumulate across runs. |
| Across `limo_base` restarts | Reset by definition | Every run already starts from a clean odom, so a restart between sessions is indistinguishable from a normal between-run reset. |

## Per-run pass criteria

A recorded run **passes** iff all of:

- robot reached path end pose (within tolerance, configurable)
- no E-stop trigger during the run
- no controller exception / node crash in the log
- all required topics present in the bag for the entire run window:
  - `/wheel/odom`
  - `/cmd_vel_raw`
  - `/cmd_vel`
  - `/estop`
  - `/reference_path`
  - `/path_follower/status`
  - `/gps_rtk_f9p_helical/gps/fix`
  - `/gps_rtk_f9p_helical/gps/rtk_status`
  - (OptiTrack pose topic — only once OptiTrack is online)
- bag wallclock length ≈ (path arc-length / v_const) within ±X% margin
- RTK fix quality ≥ threshold for ≥ Y% of the run window

Anything else = **fail**, auto-retry up to a configured limit, then skip
the cell and continue.

## Per-cell statistical outputs (the paper's tables/plots)

For each (controller, v, path, R) cell, computed from the N = 10 bags:

- **RMS e_d**, **max e_d**, **terminal pose error**, **RMS heading error**,
  **steering effort ∫|δ̇|** — computed twice: wheel-odom belief and
  RTK-truth (ADR-01 headline table).
- **Wilcoxon signed-rank** lpv-hinf vs pid-ff at each (path, v, R) cell.
- **Curvature Tolerance Index**: how flat the error profile stays as R
  shrinks; LPV is expected to stay flatter than PID.

## Orchestrator target state

What "baked into the orchestrator" means concretely, with current status:

| Capability | Today | Need |
|---|---|---|
| start/kill `base_vanilla` / `base_gnss` / `estop` / `follower` | ✓ done | — |
| push path, set `v_const`, reset | ✓ done | — |
| automatic network mode switcher | ✓ done | — |
| **working-area editor on Leaflet map**: 4-corner manual pin (live RTK at click), safety margin, circular exclusions, start/end pins; persist to `scenarios/venues/<venue>.json` | ✗ | **P0** |
| **curve-template library**: step-curvature, slalom, U-turn, 3-point turn; takes start lat/lon + heading + parameters → reference path in odom; validates against inset polygon + exclusions | ✗ | **P0** |
| **RTK-direct reposition** (closed-loop go-to-pose on RTK; terminate at tolerance; working-area clipping) | ✗ | **P0** |
| **odom reset on demand** (mechanism TBD — see open questions) | ✗ | **P0** |
| **experiment-definition YAML** (venue ref, matrix, N, gating, ping config) | ✗ | **P0** |
| **per-cell state machine** (preflight → reposition → reset → bag → push path → run → completion → bag stop → archive → classify → turnaround → next leg → auto-retry) | partial | **P0** |
| **batch loop** iterating matrix cells | ✗ | **P0** |
| **post-run analyzer** (`tools/analysis/run_eval.py`) producing per-bag metrics | ✗ | **P0** |
| dataset aggregator (per-cell stats, Wilcoxon, headline plot) | ✗ | P1 |
| **battery low ping** via `/limo_status` battery field | ✗ | P1 |
| **wallclock interval ping** | ✗ | P1 |
| live progress in battle station (current cell, ETA, pass/fail tally) | ✗ | P2 |
| indoor venue support (OptiTrack frame, room calibration) | ✗ | deferred — awaiting longer USB cables |

## What "final state" means

The system is **done** when all of these are true:

1. Operator places the LIMO somewhere on the rooftop, hits one button in
   the battle station, and walks away.
2. The orchestrator runs the full 320-cell matrix unattended, with
   RTK-direct reposition between cells, tracked-path turnarounds, and
   auto-retry on fail.
3. Operator receives push notifications on low battery and on configured
   wallclock intervals.
4. Every passing run yields a bag + sidecar JSON in a deterministic
   directory layout, paired by run-ID with any external SD-card export.
5. A single post-hoc command on the bag directory produces the headline
   table and plot for the paper.
6. **Zero unintended E-stops, zero missing topics, zero crashes across a
   full session.** If any of those happen in a session, they are bugs in
   the orchestrator/recording layer, not in the controller — and they must
   be fixed before the dataset is considered paper-grade.

## Open questions

1. **Odom reset mechanism.** Restart `limo_base_node` (~2 s downtime,
   guaranteed), driver reset service (need to check the AgileX driver on
   the NUC), or software overlay node (most flexible, adds a node in the
   safety path).

2. **Reposition controller implementation.** Reuse `path_follower_node`
   in a "reposition mode" parameter (cheaper to build, single code path),
   or a separate simple go-to-pose node (cleaner separation, more code)?

3. **Turnaround default.** Tight U-turn (semicircle at R_min) versus
   3-point turn as the default; or auto-pick based on pose-vs-area
   geometry?

4. **Ping channel.** Slack? ntfy.sh? Discord? Telegram? Email? Default
   recommendation: ntfy.sh (single curl, no auth, lands as phone push)
   unless you already use Slack regularly.

5. **Low-battery thresholds.** % at which to ping; % at which to
   auto-stop the batch (no new run started).

6. **R = 0.4 m saturation policy.** Keep R = 0.4 in the matrix (expected
   ugly at v = 1.0, ugliness is part of the data), or cap at R = 0.5 for
   first batches and add R = 0.4 only once everything else is clean?

7. **Single-claim vs two-claim paper.** Add a compute-cost figure
   (per-step `path_follower_node` time on the NUC) as a second
   contribution? Costs almost nothing extra; gives reviewers a second
   figure.
