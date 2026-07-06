# SPEC.md — Follow-up paper specification

**Status:** DRAFT / pre-experiment. This is the specification for the hardware
follow-up paper. It fixes the controllers under test, the exact code that
defines them, the pre-experiment measurements that must be completed (and the
code that performs them) **before** any matrix run, and the configuration the
experiment will actually use.

Scope boundary: the experimental *matrix* (cells, N, operational cycle) lives in
`DOC/experiment.md`; the locked requirements live in `DOC/system_spec.md`. This
file owns (1) the controller topologies, (2) the measurement protocol that
grounds the sim model in hardware reality, and (3) the exact controller
configuration. Where this file and those disagree, this file is canonical for
items (1)–(3) only.

---

## 0. Working claim (corrected — pending §5 measurements)

The original doc framing ("LPV-H∞ specialized for the small-radius regime;
headline = a curvature crossover") **does not survive verification** and is
withdrawn as the headline. Evidence (reproduced in-repo, `tools/analysis/` once
committed):

- The sim "crossover at R≈0.39" is an artifact of running the sim **noise-free**
  and reading **max** heading error. The professor's paper headline is a Monte
  Carlo **with sensor noise** (RMS metric); H∞'s purpose is disturbance
  rejection.
- Re-run **with noise on, at the deployed dt=0.05, both controllers clipped to
  ±0.5 rad** (the hardware reality), 30-seed Monte Carlo: LPV-H∞ has **~2× lower
  RMS heading error and ~2× lower run-to-run variance than PID-FF at every
  reachable radius** (R = 1.0 → 0.37 m), and the advantage is roughly
  radius-independent. The earlier worry that "the LPV-win region is a thin
  sliver at the steering floor" is therefore withdrawn along with the crossover.
- Noise-on e_d sweep (verified 2026-07-06, 30 paired seeds): LPV wins RMS
  cross-track at every R (100% paired win-rate at R ≥ 0.5) — but the margin is
  **~3–4 mm RMS**, below RTK resolution (~1 cm). e_d discriminates in sim but is
  not measurable as the hardware headline metric (see §7.2).
- Noise-on slalom sweep (verified 2026-07-06): LPV wins RMS heading at all
  tested configs but only by **~15–20%** (vs ~2× on step-curvature); PID slightly
  wins slalom e_d. **Step-curvature is the headline path family; slalom is a
  secondary "advantage survives curvature sign-reversals" axis.** The
  "slalom loads guidance bandwidth where H∞ wins big" hypothesis is dead.

**Working headline:** *At fixed speed, LPV-H∞ rejects measurement/odometry
disturbance better than PID-FF — lower and more repeatable tracking error —
across the full reachable curvature range of the platform.* The R sweep becomes
a "the advantage is general across curvature" axis, not the headline.

**UPDATE 2026-07-06 (bag-mined — see §5 "Bag-mined measurements"):** two facts
from the existing rooftop bags revise the above. (1) The robot's *achieved*
steering limit is ~0.17 rad, not 0.5 — feasible radii are R ≳ 1.2 m and the
planned matrix is unexecutable until the steering path is fixed. (2) The
measured yaw-noise floor (0.005–0.010 rad) is 3–5× below the sim's assumption;
at that level the **mean-error advantage disappears** (tie at σ=0.01, PID wins
at 0.005) while LPV keeps a **3–8× lower run-to-run variance**. Working
headline candidate accordingly shifts from "lower error" to **"more repeatable
/ lower-variance tracking under real disturbance"**, pending the steering-limit
root cause (§7.8) and a noise-structure-faithful regeneration (§6).

**Known fragility of the working headline (must be resolved by §5-M4):** the sim
noise is **additive white Gaussian** on the heading measurement
(`noise_gen.py:29`, σ_ψ = 0.03 rad at the control rate). PID's D-term is an
**unfiltered backward difference** (`pid_ff.py:67`) — a white-noise amplifier at
20 Hz — while the H∞ design rolls off (ω_u = 25 rad/s). Real wheel-odom yaw is
an integrated, *smooth* signal whose dominant error is low-frequency drift, not
white jitter. If the measured white-noise floor is small, the 2× advantage may
shrink substantially. Related fairness decision (PID D-term filtering) in §3.

**Speed axis — mostly settled (2026-07-06):** the official AgileX LIMO user
manual (github.com/agilexrobotics/limo-doc) specifies **no-load max speed
1 m/s**, backing the 1.0 m/s cap premise. The deployed code's "LIMO max ~3 m/s"
comment (`path_follower_node.py:86`) is contradicted by the manufacturer and is
presumed wrong. Recovery of the sim paper's speed-invariance claim is therefore
**unlikely**; the disturbance-rejection reframe stands as the paper. §5-M2
remains as a cheap empirical confirm. The professor must still sign off on the
reframe of his own headline.

---

## 1. Signal chain (both controllers share this)

```
/wheel/odom ─▶ VFG guidance (k_e=3.0) ─▶ e_psi, kappa ─▶ controller.compute()
              path_projector + vfg.py        │                    │ delta_cmd [rad]
                                              ▼                    ▼
                              SAFETY CLIP  np.clip(delta_cmd, -0.5, +0.5)   (path_follower_node.py:534)
                                              │
                                              ▼
                     omega = v * tan(delta_cmd) / L        (bicycle, path_follower_node.py:539)
                                              │
                                              ▼
                  /cmd_vel_raw ─▶ estop_cli.py ─▶ /cmd_vel ─▶ limo_base (vendor)
```

- **Sign convention:** `e_psi = psi_des - psi` (standard) at the controller API.
  The LPV controller inverts internally to the MATLAB convention
  (`lpv_hinf.py:97-98`).
- **Controller input on hardware:** the controller sees the guidance-derived
  `e_psi` and path `kappa`. It **never** sees GPS (ADR-01). Vehicle speed `v` is
  the commanded constant `v_const`.
- **The ±0.5 rad clip (`path_follower_node.py:534`) applies to BOTH
  controllers** and is upstream of the vendor driver. This is the binding
  steering limit in software; the *physical* limit is measured in §5-M1.

---

## 2. Controller topologies (exact)

### 2.1 PID-FF (baseline) — `controllers/pid_ff.py`

Single-input (e_psi), single-output (delta), memoryless except the D/I terms.

```
delta_cmd = K_P · e_psi + K_I · ∫e_psi dt + K_D · (de_psi/dt) + arctan(L · kappa)
delta_cmd = clip(delta_cmd, ±delta_max)
```

- Derivative: backward difference `(e_psi - e_prev)/dt`, zero on first call
  (`pid_ff.py:65-70`).
- Integral: accumulated `e_psi·dt` (`:73`); **inactive because K_I = 0**.
- Feedforward `arctan(L·kappa)`: **always on**, fixed unity coefficient (`:79`).
- Saturation: `clip(±delta_max)` (`:82`), then the external ±0.5 safety clip.
- **Fixed gains, not scheduled** — this is the intended contrast with the LPV.

### 2.2 LPV-H∞ (controller under test) — `controllers/lpv_hinf.py`

Polytopic LPV: 6 vertex controllers interpolated on ρ = |kappa|·v.

- **Each vertex** = a continuous-time LTI controller, **6 states, 2 inputs, 1
  output**. Input vector `y = [-e_psi, delta_meas]` (`lpv_hinf.py:98`); output =
  steering. Dims confirmed from `data/controllers/lpv_hinf_v3.json`:
  A(6×6), B(6×2), C(1×6), D(1×2).
- **Discretization:** each vertex Tustin/bilinear-discretized **at the
  controller's dt** (`:55-57`). dt therefore changes the controller — predictions
  must use the deployed dt=0.05 (§3).
- **Scheduling:** ρ_eff = clip(ρ·rho_scale, 0, 5); linear interpolation between
  the two bracketing vertices (`:101-126`).
- **Output:** `delta_cmd = u[0]·output_gain` (+ `K_ff·arctan(L·kappa)` only if
  K_ff≠0; **K_ff defaults 0 → no feedforward**), then `clip(±delta_max)`
  (`:129-137`).

**Vertex synthesis metadata** (`lpv_hinf_v3.json`, `method =
rho_scheduling_v3_stratABD`, `schedule_type = sqrt`, `omega_u = 25` rad/s):

| vertex ρ | ω_B [rad/s] | γ (H∞ norm) |
|---:|---:|---:|
| 0 | 5.000 | 0.6787 |
| 1 | 6.968 | 0.7669 |
| 2 | 7.783 | 0.7992 |
| 3 | 8.408 | 0.8258 |
| 4 | 8.935 | 0.8496 |
| 5 | 9.400 | 0.8663 |

ω_B is the target closed-loop bandwidth that grows with ρ (the "guidance
bandwidth" knob, in **rad/s** — a temporal-frequency quantity, see §6 caveat).

**Implementation notes (carry to hardware as-is or flag):**
1. **Only the two bracketing vertices' states are advanced each step**
   (`:117-122`); the other four are frozen. Crossing a vertex boundary resumes a
   stale state. Acceptable as-is (deployed uses the same class) but record it.
2. **`delta_meas` on hardware is the previous *clipped command*, not a measured
   steering angle** (`path_follower_node.py:528, 535`). There is no steering
   encoder in the loop. §5-M3 determines whether a real steering feedback even
   exists; if not, the H∞ second input is synthetic and that is a documented
   modeling gap.

### 2.3 Sim ↔ deployed fidelity (must match before predictions are trusted)

| Item | Sim default | Deployed | Action |
|---|---|---|---|
| Control dt | 0.01 (`api.py:63`) | **0.05** (`params.yaml:6`, `path_follower_node.py:66`) | Predict at **0.05** |
| Steering clip | LPV `delta_max=∞`; PID 0.5 | **both ±0.5** (`:534`) | Predict with **both ±0.5** |
| `delta_meas` source | plant lagged state (`engine.py`) | previous **command** (`:528/535`) | Decide + match |
| Noise | off by default | real sensors | Predict **noise on** at measured σ (§5-M4) |
| Noise structure | AWGN on ψ, δ *measurements only*; **no position noise**; guidance sees the **true** pose (`engine.py:144-163`) | odom yaw drifts (low-freq), odom **position** drifts too, guidance runs on the belief | M4 measures structure; §6 injects it |
| Command interface | controller's δ applied to the plant directly | node converts δ→Twist (`ω = v·tanδ/L`, `:539`); **vendor firmware re-derives steering from (v, ω)** and may scale v in turns (wheel-speed cap) | M2-B measures the realized (v, κ) in a max-steer turn |
| End-of-path | runs fixed n_steps | stops within 0.3 m of end (`:505`) | Terminate predictions at path end |

---

## 3. Controller configuration the experiment will use

**[DECISION — provisional; pending §5-M-stability check]**

Identical configuration in sim-prediction and on hardware, so the hardware run
*tests* the sim prediction rather than a different controller:

| Param | Value | Source / rationale |
|---|---|---|
| dt_ctrl | **0.05 s** (20 Hz) | deployed rate (`params.yaml:6`) |
| Steering clip | **±0.5 rad** both controllers | `path_follower_node.py:534`; revise to measured δ_max (§5-M1) if it differs |
| LPV vertices | `lpv_hinf_v3.json` (6, ρ∈[0,5]) | unchanged; **do not re-synthesize** |
| LPV K_ff / rho_scale / output_gain | 0.0 / 1.0 / 1.0 | defaults |
| LPV delta_max | **set to measured δ_max** (not ∞) | §5-M1; this choice is consequential (it is what removes the noise-free "LPV advantage" artifact) |
| PID K_P | **2.0** | §4 |
| PID K_I | **0.0** | §4 |
| PID K_D | **0.3** | §4 |
| PID feedforward | `arctan(L·kappa)`, unity | `pid_ff.py:79` |
| Wheelbase L | **measured** (§5-M1), sim/deployed default 0.2 | `params.yaml:7` |
| v_const | 1.0 m/s (primary) | pending §5-M2 / speed-axis decision |

**Tuning policy:** do **not** hand-tune PID on hardware for performance — it
breaks sim/hardware comparability and invites the "baseline was tuned to
win/lose" reviewer attack. If the sim gains prove unstable on hardware (real
actuator lag > sim's τ_δ=0.07 s), retune via a documented procedure **and mirror
the new gains back into the sim, regenerating all predictions.** Tuning is
allowed only if mirrored.

**[OPEN — PID D-term filter fairness]** The baseline's derivative is an
unfiltered backward difference (`pid_ff.py:67`); under white heading noise this
dominates PID's error budget (see §0 fragility note), and "any practical PID
low-pass filters its D-term" is a foreseeable reviewer attack. Decide before
matrix lock: (a) keep the textbook PID exactly as in the sim paper (maximum
comparability, weakest fairness defense), or (b) add a documented first-order
D-filter to *both* sim and hardware PID and regenerate predictions (stronger
baseline, breaks direct comparability with the sim paper's table). Either way,
state the choice explicitly in the paper.

---

## 4. PID gains — exact code provenance

**Sim default (the numbers that produced every sim prediction):**
`scalecar-vfg-h-infinite/vfg_pathfollowing/controllers/pid_ff.py:31`
```python
def __init__(self, K_P=2.0, K_I=0.0, K_D=0.3, L=0.2, delta_max=0.5):
```

**Deployed (NUC):** `scalecar-vfg-h-infinite/ros2_bridge/config/params.yaml:8-9`
```yaml
K_P: 2.0
K_D: 0.3
```
declared in `path_follower_node.py:68-69`, read at `:90-91`. **K_I is not
declared → falls through to the `pid_ff.py:31` default 0.0.** So deployed PID =
{K_P 2.0, K_I 0.0, K_D 0.3, FF arctan(L·κ), δ_max 0.5}.

**Gains to be used in the experiment:** **K_P = 2.0, K_I = 0.0, K_D = 0.3**,
unchanged from sim, pending the stability check in §5. Any change is recorded
here with the date and mirrored to the sim.

---

## 5. Pre-experiment measurement protocol

These calibrate the platform envelope and the sim noise model. **No matrix run
starts until M1–M4 are complete and logged here.** Each measurement is a small
script under `tools/measure/` (to be written); the spec below is the
implementation contract. All driving tests are **wheels-on-floor → require user
confirmation** (safety contract) and an RTK FIX for ground truth.

**Safety path (all measurement scripts):** motion commands are published to
`cmd_vel_raw` and pass through `estop_cli.py → /cmd_vel` — never publish
`/cmd_vel` directly (safety contract, `CLAUDE.md` / `system_spec.md` C3). The
operator holds the e-stop during every driving measurement.

### Hardware envelope — current best knowledge (2026-07-06)

| Quantity | Manufacturer (official) | Repo/sim assumption | Measured |
|---|---|---|---|
| Wheelbase L | **0.200 m** (user manual) | 0.2 m ✓ | M1 (tape) |
| Min turn radius (Ackermann) | **0.4 m** (user manual) | 0.37 m | M1 |
| Max speed (no-load) | **1 m/s** (user manual) | 1.0 (docs) / "~3" (code comment — presumed wrong) | M2 |
| Max steering angle | not specified | 0.5 rad | M1; manufacturer geometry implies arctan(0.2/0.4) ≈ **0.46 rad** |

Source: AgileX `limo-doc` user manual (EN), github.com/agilexrobotics/limo-doc.
Two consequences: (a) the sim/clip assumption δ_max = 0.5 rad slightly
*exceeds* the manufacturer-implied 0.46 rad — if M1 confirms ≈0.46, the ±0.5
software clip is never the binding limit and predictions should re-run at the
measured value; (b) **the matrix's tightest cell R = 0.4 m sits exactly at the
manufacturer minimum radius — zero steering margin, 100% saturation by
design** (see §7.7).

### Bag-mined measurements (2026-07-06 — from existing rooftop bags, no robot)

Mined on the laptop (`rosbags`, pure Python) from the 06-08→06-16 rooftop bags.
Methods: RTK circle fit on the NMEA GGA track (7 Hz, quality 4, fit residual
0.4–0.6 cm) cross-checked by the antenna-offset-free gyro method
(R = v_VTG / gyro_z). The two agree within ~5% on every bag.

1. **Effective curvature ceiling — BLOCKING FINDING (full 71-bag sweep).**
   Commanded steering saturates at the ±0.5 rad clip (52–55% duty on R=0.5
   runs) while the vehicle delivers far less, with a mild speed dependence:
   sustained achieved κ ≈ **0.82–0.95 at v=1.0 (R ≈ 1.05–1.22 m)**,
   **0.88–0.99 at v=0.5 (R ≈ 1.01–1.13 m)**, **1.02–1.17 at v ≈ 0.2
   (R ≈ 0.85–0.98 m)** — the last including a **manual-teleop** bag whose
   commands exceeded the software clip (implied δ = 0.79 rad), proving the
   ceiling sits **below our code** (vendor driver or chassis firmware). Across
   every recorded command in every bag — both controllers, three speeds,
   teleop — **the platform has never delivered better than ~0.85 m radius**,
   ~2× short of the manufacturer's 0.4 m. The v→0 extrapolation implies a true
   steering angle ≈ 0.24–0.27 rad (~half the datasheet-implied 0.46; a driver
   scale/calibration factor ≈ 0.5 is a plausible, checkable cause); the
   speed trend on top of it looks like understeer/tire slip. The commanded
   R=0.5 runs actually drove ~1.1 m arcs; even R=1.0 (needs δ=0.197) is
   beyond the ceiling at v=1.0, which **root-causes the June-11 review's
   undiagnosed "non-recovering arc error"** — the plant couldn't follow the
   arc, for either controller. **Until fixed, the entire matrix R ∈ {1.0, 0.7,
   0.5, 0.4} is unexecutable; feasible tracked radii at v=1.0 are R ≳ 1.2 m
   (ρ ≤ ~0.9 — only LPV vertices 0–1 exercisable).** Discriminating test for
   the robot session (§7.8): full-lock circle via RC transmitter (no ROS) vs
   via ROS at the same crawl speed — tighter on RC ⇒ software, fixable;
   identical ⇒ machine.
2. **Speed:** commanded 1.0 → achieved 1.02–1.13 m/s (RTK VTG truth); **no
   forward-speed cut while turning** (the June-11 review's candidate
   explanation is refuted); wheel-odom under-reads speed by ~10% vs RTK
   (odom 0.91–0.93 vs VTG 1.02–1.05) — calibration bias, also skews the
   belief-frame turn radius (belief 0.95–1.04 m vs true 1.1–1.2 m).
3. **Yaw noise (the §0 fragility, now measured):** odom-yaw white floor
   during motion ≈ **0.006–0.010 rad @50 Hz at v=1.0**, 0.003–0.004 at v=0.5,
   0.0002 at v=0.2 — speed-scaling (vibration-driven), and **3–5× smaller
   than the sim's σ_ψ=0.03**. Slow drift dominates the long term (June-11:
   9–11 cm RMS position drift per leg).
4. **Controller rate:** `/path_follower/timing` inter-arrival median 50 ms =
   **20 Hz confirmed** (matches deployed dt_ctrl).
5. **Headline sensitivity at measured reality** (dt=0.05, δ_max=0.175 both,
   feasible R, 20 paired seeds): at σ=0.03 LPV wins clearly; at measured
   σ=0.01 it's a tie; at measured σ=0.005 **PID wins on mean error**. But
   PID's run-to-run SD is **3–8× larger** than LPV's in every condition.
   **The mean-error headline does not survive the measured noise floor;
   the repeatability/variance advantage does.** See §0 update and §7.8.

Reuse existing entry points where they exist: `tools/diagnostics/sweep_steering.py`
already drives the steering to ±0.5 rad at v=0.2 (constants `V_LIN=0.2`,
`W_AMP=0.55`) and is the basis for M1/M3.

Also: **read the vendor steering limit on the NUC** (not in this repo — `limo_base`
lives in `~/agilex_ws/`). Run on the robot and record here:
```
grep -rEn 'max_steering|steering.*limit|<limit' ~/agilex_ws/src/limo_* 2>/dev/null
```
The ±0.5 rad software clip (`:534`) is binding only if the vendor servo can
reach ≥0.5 rad; M1 is the physical truth.

### M1 — Maximum steering angle δ_max (and wheelbase L, R_min)

- **Goal:** the real physical steering limit and turn radius, vs the 0.5 rad /
  R_min 0.37 m assumptions.
- **Pre:** tape-measure (static, no motion): wheelbase L (front-axle to
  rear-axle), track width, and the **RTK antenna's planar offset from the
  rear-axle midpoint** — longitudinal `a` (forward +) and lateral `b`. Record
  all three.
- **Procedure:** command constant low speed `v=0.2 m/s` with steadily
  increasing `angular.z` (via `cmd_vel_raw`, see safety path above) until the
  turn radius stops shrinking — that plateau is the servo/linkage limit, which
  may sit above or below the follower node's ±0.5 rad software clip (the clip at
  `path_follower_node.py:534` is NOT in this command path; the vendor chassis
  maps twist→steering internally). Hold the plateau command and drive **≥1.25
  full circles**. Log `/gps_rtk_f9p_helical/gps/fix` (ENU). Repeat **both
  directions**.
- **Reduction:** least-squares circle fit (Kåsa / algebraic) to the ENU (x,y) →
  `R_ant` (the **antenna's** circle) + fit residual (RMS, m). **Correct for the
  antenna lever arm** — in a steady turn every body point circles the same ICR,
  which lies on the rear-axle line (bicycle model), so
  `R_axle = sqrt(R_ant² − a²) ∓ b` (sign of b per turn direction). At R≈0.4 m an
  uncorrected 0.1–0.15 m offset biases R by 25–40% — the correction is not
  optional. Then **δ_max = arctan(L / R_axle)**; R_min = R_axle.
- **Outputs (record here):** L, a, b, R_ant(L/R), R_axle(L/R), δ_max(L/R),
  residuals, L-vs-R asymmetry. Cross-check: manufacturer R_min = 0.4 m implies
  δ_max = arctan(0.2/0.4) ≈ 0.46 rad; the repo's 0.37 m implied 0.495 rad.
  Expect R_axle ≈ 0.4 m if the manufacturer figure holds.
- **Acceptance:** residual < 0.03 m and left/right δ_max within 10%.
- **Feeds:** §3 LPV `delta_max`, the ±0.5 clip validity, R_min in the matrix.

### M2 — Speed cap and speed-in-turn

- **Goal:** settle the "1.0 vs 3.0 m/s" contradiction (§0) and quantify how much
  forward speed a turn costs (real ρ).
- **Procedure A (straight, runway ≥ 8 m):** step commanded `linear.x` through
  {0.3, 0.5, 0.7, 0.9, 1.0, 1.2, 1.5} m/s, holding each ~3–5 s. Measure achieved
  ground speed from RTK (`|d(pos)/dt|`, the truth) **and** `/wheel/odom`
  twist.x (the belief).
- **Procedure B (turn):** command `linear.x = 1.0` + steering at δ_max (from M1);
  in steady turn measure achieved forward speed (RTK). Compute
  **ρ_max,real = κ_max · v_achieved_in_turn**.
- **Outputs:** commanded-vs-achieved speed curve + saturation knee (the real
  cap); odom-vs-RTK speed bias; v_achieved_in_turn; ρ_max,real.
- **Decision impact:** if the knee is > 1.0 m/s, escalate the §0 speed-axis
  decision to the professor before locking the matrix.

### M3 — Steering feedback noise (and existence)

- **Goal:** does a real steering encoder/feedback exist, and what is its noise?
  This determines whether the LPV `delta_meas` input (§2.2 note 2) is real.
- **Procedure:** on the NUC, `ros2 topic list` + inspect `/limo_status` (and any
  steering/servo feedback topic) for a reported steering angle. If present: log
  it (a) stationary at commanded δ=0 and (b) at a fixed commanded δ, ~30 s each.
- **Reduction:** mean (bias vs command), SD (σ_δ), and a PSD if a clear jitter
  band exists.
- **Outputs:** whether steering feedback exists; σ_δ, bias. If it does **not**
  exist, record that `delta_meas` = previous command is the only available signal
  and flag the modeling gap in §2.2.

### M4 — Heading noise: magnitude AND structure, both sources

- **Goal:** the **real σ_ψ and its frequency structure**. The sim injects
  **white** Gaussian heading noise (σ_ψ = 0.03 rad, `noise_gen.py:29`); the
  corrected headline (§0) stands or falls on whether real odom yaw noise has a
  comparable **white** component — because PID's unfiltered D-term amplifies
  white noise specifically (§0 fragility note). A matched SD with the wrong
  structure (drift instead of jitter) predicts the wrong outcome.
- **Sources to log simultaneously:** (1) wheel-odom yaw from `/wheel/odom`
  (the controller's belief); (2) RTK course-over-ground from the GPS fix
  velocity (truth, **valid only above ~0.3 m/s**); (3) MAVROS `compass_hdg` if
  streaming (note the known ~90° mount + offset, see project memory; needs a
  GPS fix to stream at all — outdoor only).
- **Procedure:** (a) stationary ~60 s; (b) straight constant 1.0 m/s drive
  (≥ 30 s); (c) the M1 circle drive doubles as a turning sample.
- **Reduction:** per-source: SD; **PSD (or Allan deviation) splitting the
  white-noise floor σ_w from low-frequency drift / random-walk**; drift rate
  (rad/s) over the straight drive vs RTK-COG as reference. Pairwise offsets
  (odom-yaw vs RTK-COG vs compass); confirm RTK-COG unusable < 0.3 m/s.
- **Outputs:** per source — σ_total, σ_white, drift rate, offsets. **Feeds §6:**
  regenerate predictions with the **measured structure** (white component at
  σ_w + a bias/random-walk term), not just a matched SD. If σ_w ≪ 0.03 rad, the
  §0 headline margin must be re-derived and the PID D-filter decision (§3)
  revisited. Also note: because RTK gives course-over-ground (≠ heading under
  sideslip), heading error is reportable as odom-belief only; see §7.2 for the
  quantified metric dilemma.

---

## 6. Sim-prediction regeneration (required before matrix lock)

Every sim number that informs the matrix must be regenerated under the
hardware-faithful configuration, **not** the defaults that produced the
withdrawn crossover:

1. **noise ON** with the **measured magnitude AND structure** from §5-M4
   (white component at measured σ_w plus measured drift/random-walk — not just
   an SD-matched AWGN), Monte Carlo ≥ 30 seeds; note the stock
   `NoiseGenerator` is white-only and needs a small extension for the drift
   term;
2. **dt = 0.05**;
3. **both controllers clipped to the measured δ_max** (§5-M1);
4. **terminate at path completion** (mirror `path_follower_node.py:505`), never
   run past the path end (the post-terminus projector blow-up is a sim artifact,
   not a controller failure);
5. report **RMS** error (and variance) as primary; max error only alongside.

**Caveat to carry into the paper (do not over-claim):** ρ = |κ|·v sets the same
*static* scheduling value whether reached by κ or by v, but ω_B/ω_u live in
rad/s — the **bandwidth** argument is temporal-frequency, which the **speed**
axis controls and the curvature axis does not. The curvature sweep raises
steady-state ρ and disturbance amplitude; it does **not** reproduce the
speed/bandwidth result. At v ≤ 1.0, ρ_max ≈ 2.7 → only vertices 0–2 are
exercised; vertices 3–5 stay dormant. Report the achieved ρ(t) range honestly.

---

## 7. Open decisions

1. **Speed axis (§0):** pending M2. Recover the original speed-invariance claim
   if the platform exceeds 1.0 m/s? Needs professor sign-off.
2. **Headline metric — now quantified (2026-07-06 noise-on sweeps):** neither
   metric is clean, and the trade is now numeric. **e_d:** LPV wins RMS at every
   R (100% paired win-rate at R ≥ 0.5) but the margin is **~3–4 mm RMS — below
   RTK resolution (~1 cm)**: hard truth, unmeasurable effect. **e_ψ:** ~2.5°
   margin — measurable, but RTK-COG truth is soft under sideslip (M4
   quantifies). Leading option: heading e_ψ as the headline **reported in the
   odom-belief frame** (which both controllers share), with RTK e_d as the
   bounded-truth secondary; decide after M4.
3. **`delta_meas`:** feed a real steering encoder (if M3 finds one) or keep the
   previous-command surrogate — match sim to whichever is chosen.
4. **Statistics [bug verified 2026-07-06]:** the headline test must operate at
   the **rep** level (per-cell LPV reps vs PID reps, Mann-Whitney U + effect
   size + CIs), not on R-averaged means — the existing
   `tools/analysis/aggregate.py` collapses the 10 reps via `np.mean` per R and
   runs `wilcoxon()` over the ~4 shared radii; with n=4 pairs the minimum
   two-sided p is 0.125, so **significance is impossible by construction**
   (verified in code). Add a power analysis (RMS is over-powered at N=10; max
   metrics may be under-powered).
5. **Run ordering:** controller order must be **randomized/interleaved at the
   rep level within each cell** (not "all LPV then all PID") so battery, tire,
   temperature, and RTK-quality drift cannot alias onto the controller factor.
   This requirement currently exists nowhere in the sequencer or
   `DOC/experiment.md` (whose per-cell cycle runs reps consecutively) — it must
   land in the run executor / experiment YAML before the matrix runs.
6. **PID D-term filter** (§3 open decision) — resolve together with M4's noise
   structure result.
7. **R = 0.4 m cell viability:** the manufacturer's minimum turning radius is
   0.4 m (envelope table, §5) — that cell runs at full steering saturation with
   zero margin. Decide: keep it as the "at the physical limit" data point
   (ugliness is data), or replace with R = 0.45 as the tightest *controlled*
   cell. Interacts with `DOC/experiment.md` open question 6. **[Superseded in
   practice by §7.8 — currently NO cell in the matrix is trackable.]**
8. **Steering-limit root cause — TOP BLOCKER (2026-07-06).** The achieved
   steering limit is ~0.17 rad vs 0.5 commanded and vs the manufacturer's
   implied 0.46 (§5 bag-mined finding 1). Find where the authority is lost:
   (a) read the vendor `limo_base` driver source/params on the NUC (steering
   clamp, angular.z→steering mapping, steering calibration offset); (b) bench
   test: at v=0.2, sweep `angular.z` and measure achieved κ (gyro/VTG) — find
   the ceiling; (c) compare against RC/teleop steering (does the servo reach
   further outside ROS?). Outcome branches: **fixable** (driver clamp/config)
   → original matrix restored, re-run predictions at the true δ_max;
   **not fixable** (firmware/hardware) → redesign matrix around R ≥ 1.2 m,
   accept ρ ≤ 0.9, and re-frame per §0 update. Every other §7 decision is
   downstream of this one.
