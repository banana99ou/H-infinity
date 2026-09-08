# Agent field runbook — autonomous test & validation (rooftop)

**Audience: you, an AI agent (Claude Code) driving the LIMO over ssh to the NUC.**
You have no ROS2 on the laptop — the robot is the source of truth. This runbook
takes you from a (possibly just-booted) robot to a clean, supervised autonomous
run, and tells you exactly what to watch so a rooftop hiccup gets caught.

This doc is intentionally **self-contained** — you can act from it alone. The
start procedure mirrors `DOC/deployment.md` §"Runbook — autonomous smoke e2e
start"; if the two ever diverge, **`deployment.md` is canonical** — reconcile and
fix this copy. Live status lives in `ToDo.md`.

> **Integration status (2026-06-08):** the always-on `heading_node` EKF, the
> `reposition`→`/heading/fused` cutover, and the `heading` orchestrator PROC are
> **landed** — synced + built + bench-verified on the NUC (both nodes construct;
> `/heading/fused` links heading_node→reposition with matched QoS). Start the
> `heading` PROC with the sensor stack (§3); reposition consumes `/heading/fused`
> and HOLDs without it. **Still unverified in the field:** the outdoor
> no-limit-cycle convergence gate. Confirm against `ToDo.md`.

---

## 0 · Mission — what you are proving

Hardware bring-up of the professor's **VFG + LPV-H∞** path-following controller on
the AgileX LIMO. The end goal is a **paper-grade dataset** comparing LPV-H∞ vs
PID-FF across a curvature sweep at fixed speed, gathered **unattended** on a
rooftop track, with **RTK as post-hoc ground truth**.

The system is **autonomous**: an experiment **executor** drives the whole loop —
it repositions the robot to a start pin via RTK, zeroes odom, starts recording,
runs the controller down a reference path, classifies the leg, turns around, and
returns. Your job is to **start it correctly and supervise it**, not to drive it.

> **Which executor:** the real data-gathering matrix runs through
> **`run_executor_node`** + the webui leg-batch workflow (operator decision
> 2026-06-12). The original `experiment_sequencer_node` is **DEPRECATED**, kept only
> for the `run_smoke_e2e` smoke path this runbook arms (§5). The loop behaviour an
> agent supervises is the same either way.

**Definition of done (PASS):** `/experiment/status` reaches `phase: done` with
`fail: 0`, and a **bag + sidecar** land under `Experiment Data/`. Anything else is
a hiccup to diagnose, not a pass.

**Safety contract (non-negotiable):** the only path to the motors is
`cmd_vel_raw → estop_cli → /cmd_vel`. Synthetic-odom tests with **wheels off the
ground** are fine without asking. **Any wheels-on-floor motion requires explicit
human confirmation** (see §1).

---

## 1 · Your operating contract — read before touching anything

**You supervise an autonomous loop. You do not micromanage it.**

**Ground truth is the wheels, not the screen.** The `/experiment/status` phase
label is *not* proof the robot is stationary — on the last outdoor run the robot
drove ~11 m, nearly to the end pin, **while the WebUI still showed `preflight`** (a
silent compass-QoS failure that threw no error and never incremented `fail`; see
§8). So: in **any** no-motion phase (preflight, reposition-settling, between legs /
turnaround), **any sustained `/cmd_vel` or wheel motion = immediate E-stop**,
regardless of what the phase says. Do not wait for an alarm to fire — watch the
robot itself.

The system **pauses itself on purpose** and **auto-resumes** — RTK loss (F2), odom
loss (base-serial dropout), low battery (M2). A self-pause is the system working,
**not** a failure. Do **not** "fix" a healthy self-pause by restarting things —
you will corrupt the run. Watch, and let it resume (§6, §7).

**Guardrails — never do these:**
- ❌ Publish to `/cmd_vel` or `/cmd_vel_raw` directly. Ever.
- ❌ Hand-start `sequencer_smoke`, `follower`, or `reposition`. The **sequencer**
  brings those up itself in C6-safe order (exactly one `cmd_vel_raw` publisher).
  Hand-starting a second mover breaks mover-exclusivity → fighting cmd_vel.
- ❌ Hand-edit `scenarios/venues/rooftop.json` (pin-heading convention only
  round-trips through the battle-station editor — `memory/project_heading_convention_bug`).
- ❌ Disable estop or geofence; arm without `confirmed_wheels_on_floor:true`.
- ❌ Restart `limo-battle.service` while PROCs run — it **orphans** them
  (untracked, then duplicates). Tear down procs first, or reboot.
- ❌ `git push` to origin. Commit only when the human asks.

**What you CANNOT verify over ssh — require the human operator:**
- Robot placed at start pin **S1, nose toward E1**, on the track.
- Chassis **Ackermann** switch ON (`motion_mode == 1`).
- **Wheels on the floor**, area clear, operator hand on E-stop.
- The **magnetic environment** (rebar/metal under the start pin).

`confirmed_wheels_on_floor:true` in the arm command **is** that human handshake —
never fabricate it. Before arming a wheels-on-floor run, get an explicit human
"go".

---

## 2 · Connect to the robot

```bash
# Tailscale name when the NUC has an uplink; LAN IP otherwise.
# 2026-06-08: Tailscale was DOWN (NUC offline); the working path was the LAN IP.
NUC=agilex@192.168.0.54          # or agilex@agilex-nuc12wski7
SRC='source /opt/ros/humble/setup.bash; source /home/agilex/agilex_ws/install/setup.bash;'
rr() { ssh -o BatchMode=yes -o ConnectTimeout=8 "$NUC" "bash -lc '$*'"; }

rr "echo OK \$(hostname)"        # -> OK agilex-NUC12WSKi7
```

If this hangs: the NUC likely lost its uplink (Tailscale drops) — ask the operator
to plug the phone-hotspot USB, or join the LIMO AP and use its gateway IP. After a
NUC reboot the ros2 CLI daemon can bind only to a foreign LAN (QCar2) and miss
local nodes — if `ros2 node list` looks wrong: `rr "$SRC ros2 daemon stop && ros2 daemon start"`.

---

## 3 · Cold-boot bring-up (robot just powered on)

### 3a. Prerequisites (once per session)
- **Code synced + built on the NUC** (not just rsync'd — a rebuild is what reaches
  the graph):
  ```bash
  tools/sync/sync.sh push            # from the laptop (LIMO_HOST=$NUC to override host)
  rr "$SRC cd ~/agilex_ws && colcon build --packages-select limo_path_follower"
  ```
- **Ackermann mode ON** (human, physical switch).
- **Venue pinned + saved from the battle-station map under live RTK** — never
  hand-edit the JSON. Confirm `smoke.yaml`'s `venue:` points at it.

### 3b. Bring up the stack
**Fast path (recommended) — one script does bring-up + gates + arm + watch:**
```bash
rr "bash ~/H-infinity/tools/ops/field_smoke.sh --gates-only"   # non-motion dry run first
```
`--gates-only` brings everything up and runs the gates **without** motion (it
correctly aborts at the RTK gate if `quality=0`). Use it to validate §3–§4 hands-off.

**Manual path (when you need control):**
```bash
rr "bash ~/H-infinity/tools/orchestrator/start_battle.sh"      # rosbridge :9090 + orchestrator
for p in base gnss estop odom_zero ops geofence odom_watchdog heading; do
  rr "$SRC ros2 topic pub --once /orchestrator/start std_msgs/msg/String '{data: $p}'"
done
```
- `base` (chassis) and `gnss` (mavros+RTK) are **split** so `odom_watchdog` can
  respawn the chassis driver on a serial dropout without dropping RTK. Do **not**
  also run the legacy combined `base_gnss`.
- `heading` = the always-on heading EKF; reposition consumes its `/heading/fused`
  and HOLDs without it, so it must be up before any reposition leg.
- **Do not** start `sequencer_smoke`/`follower`/`reposition` here — the arm
  command (§5) does, via the sequencer, C6-safely.

### 3c. Verify the graph (verified baselines, 2026-06-08)
Use **≥8 s** `hz` windows — short windows give false SILENT reads. QoS matters: a
default-RELIABLE sub on a BEST_EFFORT topic silently gets **zero** messages.

| topic | type | QoS | healthy rate |
|---|---|---|---|
| `/imu` (LIMO, primary gyro) | sensor_msgs/Imu | RELIABLE | 100 Hz |
| `/wheel/odom` | nav_msgs/Odometry | RELIABLE | 50 Hz |
| `/pixhawk/imu/mag` | sensor_msgs/MagneticField | BEST_EFFORT | low (115200 baud cap) |
| `/pixhawk/global_position/compass_hdg` | std_msgs/Float64 | BEST_EFFORT | **GPS-gated** (silent indoors) |
| `/gps_rtk_f9p_helical/gps/{fix,rtk_status}` | NavSatFix / String | RELIABLE | 1 Hz |
| `/heading/fused` | std_msgs/Float64 | latched RELIABLE | 25 Hz |

```bash
rr "$SRC ros2 node list"
rr "$SRC timeout 8 ros2 topic hz /wheel/odom"      # ~50 Hz
rr "$SRC timeout 8 ros2 topic hz /imu"             # ~100 Hz
```

---

## 4 · Pre-flight gate (no wheels move)

```bash
rr "bash ~/H-infinity/tools/preflight/preflight.sh"   # MUST exit 0
```
Checks: node graph, the `cmd_vel_raw → estop → /cmd_vel` chain with a **single**
`cmd_vel_raw` publisher (C6), Ackermann, battery (preflight code still gates ≥ 10.5 V;
**canonical M2 halt is 10.0 V** — code sync pending, see `ToDo.md`), `/wheel/odom` > 30 Hz,
and **RTK FIXED (quality=4)**.

Then confirm the items preflight can't:
- **RTK quality** — `rr "$SRC ros2 topic echo /gps_rtk_f9p_helical/gps/rtk_status --once"`.
  `quality=4` (FIXED, cm) is the target. `quality=5` (FLOAT, dm) only **WARNs** in
  preflight, but the sequencer's M1/F2 gate is **FIXED-only** and will pause the
  batch on FLOAT — decide before arming. RTCM must not be `STALE`.
- **Heading live** — `rr "$SRC ros2 topic echo /heading/fused_status --once --qos-durability transient_local --qos-reliability reliable"`.
  Outdoors under RTK expect `mode: GNSS_AIDED`, single-digit `heading_std_deg`, no
  `compass vs COG disagree` warnings. Indoors/no-GPS expect `GYRO_MAG` (mag-bounded).
  **The cutover has landed:** reposition is now a pure consumer of `/heading/fused`
  (no own compass/COG sub), so a green, *fresh* `/heading/fused` IS reposition's
  heading — but if it goes stale (heading_node down) reposition HOLDs rather than
  drive blind, so keep the `heading` PROC up. Confirm `/reposition/status.err_deg`
  actually *updates and settles* once a goto starts, and that `compass_hdg` is live
  (BEST_EFFORT; GPS-gated — silent without a fix). See §8.
- **Gyro sign** (first time only) — hand-rotate the chassis ~90° CCW; `/heading/fused`
  must **decrease** ~90°. If it increases, set `gyro_limo_sign:=1.0`.
- **Battery topped** (≥ ~12.6 V; 2026-06-05 died mid-session at 12.4 V).
- **Geofence armed** (the `geofence` PROC; arms after RTK is live).

Get the **human "go"** (placement, Ackermann, wheels-on-floor, area clear) before §5.

---

## 5 · Arm the autonomous run (one command, then hands off)

```bash
rr "$SRC ros2 topic pub --once /ops/cmd std_msgs/String \
  '{data: \"{\\\"action\\\":\\\"run_smoke_e2e\\\",\\\"confirmed_wheels_on_floor\\\":true}\"}'"
```
`ops_node` kills the matrix driver, starts `sequencer_smoke` pinned to
`smoke.yaml`, and auto-sends `{action:start}` once it idles. (This smoke path uses
the **DEPRECATED** `sequencer_smoke`; the full data-gathering matrix runs through
`run_executor` + the webui leg-batch workflow, **not** `experiment.yaml` + a
sequencer PROC.) `confirmed_wheels_on_floor:true` is
**mandatory** — `ops_node` refuses the arm without it.

**The sequencer now runs itself** through its phase order — **do not touch it:**

```
preflight → reposition → odom-reset → bag → follower → set-params
   → push-recipe → run → stop/classify → turnaround → return-leg → done
```

From here you are a **supervisor** (§6, §7). The only legitimate agent actions
during a run are: **observe**, and **stop** (pause / abort / e-stop) on a real
failure.

---

## 6 · Live supervision — the monitor matrix

Poll these on an interval (or use `tools/ops/watch_experiment.py`). For each:
*healthy looks like* → *hiccup looks like* → *do*.

| Signal | Healthy | Hiccup | Action |
|---|---|---|---|
| `/experiment/status` | `phase` advances; `fail` stays 0; ETA sane | phase stuck >> expected; `fail` climbs | identify the stuck phase below; if a real failure → §7 stop |
| **robot motion vs claimed phase** | stationary in every no-motion phase (preflight, reposition-settling, turnaround) | wheels turning / `cmd_vel_raw` non-zero while the phase is pre-RUN | **STOP immediately — trust the wheels, not the label** (§1, §7). This is how the last runaway presented (~11 m during displayed `preflight`) |
| `/reposition/status` | `err_m` ↓ toward `pos_tol`; `err_deg` settles into `heading_tol` (5°) and **stays** | (a) `err_deg` swings ±150° and won't converge = **LIMIT-CYCLE**; (b) heading source dead/silent → `err_deg` never updates and the robot creeps or drives **straight** off-target = **DEAD-HEADING RUNAWAY** (the last-run signature) | STOP (pause); neither self-recovers — see §8 |
| `/heading/fused_status` | `mode: GNSS_AIDED`, `heading_std_deg` single-digit, bias stable | `compass vs COG disagree` warns; `heading_std` blows up; `mode` stuck `GYRO_ONLY` while RTK is up | magnetic interference / lost compass — note it; reposition may still ride COG+gyro; stop if heading is clearly wrong |
| `/gps_rtk_f9p_helical/gps/rtk_status` | `quality` holds 4 (or 5 if accepted); RTCM not stale | drops to 0/STALE | **expected self-pause (F2)** — let it auto-resume; only worry if it never recovers |
| `/wheel/odom` rate | ~50 Hz throughout | goes silent | base-serial dropout — **watchdog respawns `base`** (~6 s); sequencer pauses & re-does the leg. Let it. Stop only if it never returns |
| `cmd_vel_raw` publishers | exactly **1** at any instant | 2+ | mover-exclusivity broken — **STOP immediately** (you/someone hand-started a mover) |
| `/limo_status` battery | ≥ halt threshold | below M2 halt | **expected halt (M2)** + operator notify — swap/charge; resumes |
| geofence / `/estop_trigger` | quiet | fires | robot left the polygon or estop latched — STOP, investigate before re-arming |

Quick reads:
```bash
rr "$SRC ros2 topic echo /experiment/status --once"
rr "$SRC ros2 topic echo /reposition/status --once --qos-durability transient_local --qos-reliability reliable"
rr "$SRC ros2 topic echo /heading/fused_status --once --qos-durability transient_local --qos-reliability reliable"
rr "$SRC ros2 topic info /cmd_vel_raw"     # Publisher count must be <= 1
```

---

## 7 · Normal vs. intervene — the decision

**LET IT RIDE (the system's own safe-state pauses; they auto-resume):**
- **RTK loss → F2 pause/resume.** Robot stops; resumes when FIXED returns.
- **Odom loss (base serial) → pause + watchdog respawn `base` → re-do the leg.**
  Data integrity preserved.
- **Low battery → M2 halt + operator notify.** Resumes after battery.
- **One leg fails → retry (`max_retries`) → skip → circuit-breaker.** The batch
  self-manages; a single skipped cell is not your cue to intervene.

**STOP (real failures — these do not self-heal):**
- **Motion during a no-motion phase** — robot driving while `/experiment/status`
  shows a pre-RUN phase (preflight, reposition-settling, turnaround). E-stop now; do
  not trust the label (§1). This is how the last runaway presented.
- **Reposition heading limit-cycle** (`err_deg` swinging, won't converge) **or
  dead-heading runaway** (heading source silent → creep or **straight** drive
  off-target — the last-run signature).
- **Runaway-guard abort** (reposition driving the wrong way; it aborts itself —
  confirm it *actually* stopped).
- **Geofence breach / estop latched.**
- **Drive-blind**: `/wheel/odom` silent **and** the robot still commanding motion.
- **cmd_vel contention**: >1 `cmd_vel_raw` publisher.
- **Circuit-breaker tripped** repeatedly (systemic, not a one-off).

> **The automated nets are unproven in the field.** The runaway-guard and the
> geofence/E-stop that would now catch a dead-heading runaway are *post-incident*
> additions — never exercised outdoors. The last runaway threw **no error and never
> incremented `fail`**; the human eyeball was the only catch. Watch **actual motion**
> as ground truth; do not wait for an alarm to fire.

**Stop controls (escalating):**
```bash
rr "$SRC ros2 topic pub --once /experiment/cmd std_msgs/String '{data: \"{\\\"action\\\":\\\"pause\\\"}\"}'"   # safe-state hold
rr "$SRC ros2 topic pub --once /experiment/cmd std_msgs/String '{data: \"{\\\"action\\\":\\\"abort\\\"}\"}'"   # stop batch + latch E-stop
# E-STOP is the hard stop (latches /cmd_vel to zero) — use the battle-station E-STOP or estop_cli.
```
After any stop, **confirm `/cmd_vel` actually went to zero** before doing anything
else, and tell the human what tripped.

---

## 8 · Rooftop-specific watchpoints (known landmines)

- ⚠️ **Heading limit-cycle** (the bug `heading_node` fixes; `ToDo.md:682-705`,
  `ToDo.md:707`). Pre-fix workaround if it recurs: a venue with **A == B** start/end
  pins makes the sequencer skip reposition (`_reposition_is_noop`), operator hand-
  places, and the run proceeds. Watch `/reposition/status.err_deg`.
- ⚠️ **Dead-heading runaway (the last-run failure — distinct from the limit-cycle).**
  If reposition's heading source goes silent — a QoS mismatch, or no GPS/EKF fix to
  feed it — reposition gets **no** standstill heading and creeps / drives **straight**
  off-target instead of limit-cycling. On the last outdoor run this drove the robot
  ~11 m during displayed `preflight`, **silently** (no error, `fail` stayed 0). The
  compass-QoS cause is fixed (the sub is BEST_EFFORT now) and a runaway-guard was
  added — but **that guard is unproven in the field**. Watch the wheels, not the
  status (§1, §7).
- ⚠️ **Reposition now consumes `/heading/fused` (cutover landed).** It no longer has
  its own compass/COG subscription, so a *fresh* `/heading/fused` IS its heading — and
  if that topic goes stale (heading_node down) reposition HOLDs (zero output) rather
  than drive blind, so keep the `heading` PROC up. Confirm `/reposition/status.err_deg`
  actually *updates and settles*, and that `compass_hdg` is live (BEST_EFFORT;
  GPS-gated — silent without a fix).
- ⚠️ **RTK FIXED won't hold** → only FLOAT (q=5). Decide up front whether FLOAT is
  acceptable; the sequencer M1/F2 is FIXED-only and will pause on FLOAT.
- ⚠️ **Base-serial dropout under vibration** (chassis CP2102). Mitigated by
  `odom_watchdog` (respawns `base`) + sequencer odom-pause; the latter is
  code-verified but **not yet run-tested with wheels turning** — watch it on the
  first moving leg.
- ⚠️ **Magnetic interference** corrupts the compass auto-cal (top heading risk on
  metal-laden ground). Watch for `compass vs COG disagree` warnings and an offset
  that won't repeat across runs; COG + gyro carry heading if the compass is bad.
- ⚠️ **Path footprint vs venue.** The step recipe's follower defaults can drive an
  ~5.7 m path inside an ~8 × 2 m inset → leaves the box. Confirm the venue-fit
  `path_override` is in effect, keep geofence/E-stop ready.
- ⬜ **Heading-convention nudge test** (still pending, `ToDo.md:679`): at S1, a few
  cm of reposition must turn the nose **toward E1**. If it heads away, pin headings
  are mirrored — do not run scored cells.

---

## 9 · PASS, artifacts, wrap-up

- **PASS** = `/experiment/status` `phase: done`, `fail: 0`, **bag + sidecar** under
  `Experiment Data/` (both — a missing sidecar is a fail; see the A1 fix,
  `ToDo.md:66`).
- **Pull artifacts** back to the laptop (NUC → laptop only):
  ```bash
  tools/sync/sync.sh pull          # LIMO_HOST=$NUC to override host
  ```
- **Leave the robot safe:** ensure `/cmd_vel` is zero / E-stop latched if anything
  is uncertain. Don't tear down PROCs by restarting `limo-battle.service` (orphans).
- **Report** to the human: outcome, which phases ran, any self-pauses observed,
  artifacts pulled. If a real failure stopped the run, quote the signal that tripped.

---

## Appendix · command cheat-sheet

```bash
# state of the world
rr "$SRC ros2 node list"
rr "$SRC ros2 topic echo /orchestrator/status --once"      # which PROCs are up
rr "$SRC ros2 topic echo /experiment/status --once"        # run phase/cell/pass-fail/ETA
rr "$SRC ros2 topic echo /ops/status --once"               # mirror

# start a PROC / stop a PROC
rr "$SRC ros2 topic pub --once /orchestrator/start std_msgs/msg/String '{data: <name>}'"
rr "$SRC ros2 topic pub --once /orchestrator/kill  std_msgs/msg/String '{data: <name>}'"

# heading EKF: it's the 'heading' PROC (start it like any other, above); watch it
rr "$SRC ros2 topic echo /heading/fused_status --once --qos-durability transient_local --qos-reliability reliable"
```

PROC names: `base`, `gnss`, `estop`, `odom_zero`, `ops`, `geofence`,
`odom_watchdog`, `heading`, `bag` | sequencer-managed (don't hand-start):
`sequencer`, `sequencer_smoke`, `follower`, `reposition`.

_Created 2026-06-08 (rewritten from the human pre-flight checklist into an agent
runbook). Keep the start procedure reconciled with `DOC/deployment.md` §97._
