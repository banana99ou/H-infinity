# ADR-01: GPS stays out of the control loop

**Status**: accepted (2026-04-30)
**Scope**: paper experiments + everything that runs on the LIMO for this study
**Supersedes**: nothing
**Related**: `DOC/system_spec.md` §3.3 (L1–L5), `DOC/experiment.md` (operational model)

## Decision

For all evaluation runs of the VFG + LPV-Hinf controller on the LIMO:

- **Control feedback**: `/wheel/odom` only.
- **RTK GPS**: recorded alongside the run, used post-hoc as semi-ground-truth
  for path-tracking error metrics. Never fed into the controller.
- **Ohcoach-cell Y3 (FitTogether)**: standalone blackbox. No ROS interaction.
  We owe it only an SSID/PW (handled at hardware setup) and a wallclock
  timestamp / run-ID on every rosbag so the SD-card dump can be paired
  offline.
- **No `robot_localization` EKF**, no `navsat_transform_node`, no
  GPS-corrected odom topic that the controller subscribes to.

A future paper or operational deployment may revisit this; see "When to
revisit" below.

## Why (paper-evaluation lens — the deciding lens)

1. **Putting GPS in the loop destroys ground truth.** RTK is the only
   independent measurement of where the robot really is. The moment the
   controller consumes GPS, the realized trajectory is correlated with the
   GPS measurement noise, and "RTK-derived path-tracking error" becomes
   circular — it measures partly the controller and partly the EKF's noise
   suppression. Reviewers reject this on sight.

2. **It undermines the H-infinity claim.** The paper's contribution is
   robust control under model + sensor uncertainty. Wheel-odom drift is one
   of the disturbances the controller is designed to absorb (the Monte Carlo
   in `report/sections/results.tex` injects σ_ψ = 0.03 rad noise for
   exactly this reason). Replacing wheel odom with fused GPS removes the
   disturbance the controller is supposed to fight; you'd be evaluating it
   on an easier problem than the one it was designed for.

3. **The Monte Carlo simulations become non-comparable.** Sim used
   wheel-odom-quality noise; real runs would use GPS-quality state. The 10×
   gap between sim and real has no honest explanation.

4. **LPV scheduling assumption breaks.** The vertex controllers
   (ρ ∈ {0,1,2,3,4,5}) were synthesized under particular noise/uncertainty
   assumptions baked into each H-inf design. Fused state lives in an
   off-design regime; the polytopic interpolation isn't valid there.

5. **You lose the most publishable single number this dataset enables**:
   the gap between *what the controller believed about its position* (wheel
   odom) and *where the robot actually was* (RTK truth). Fusing collapses
   both columns into one. Keeping them separate gives you the headline
   table:

   | metric | wheel-odom belief | RTK truth | gap |
   |---|---|---|---|
   | RMS e_d | … | … | … |
   | max e_d | … | … | … |
   | terminal pose error after lap | … | … | … |

6. **Comparability with other LIMO/ScaleCar papers.** Most published work
   on these platforms uses wheel odom only. Apples-to-apples comparison
   requires the same.

7. **The spec already encoded this.** The original project spec called RTK
   "semi-ground truth for controller **evaluation**", put "direct code-level
   interaction with the Ohcoach-cell" out of scope, and confirmed the external
   dataset is collected independently of the ROS2 stack. That intent now lives
   in `system_spec.md` §3.3 (L1–L5). This ADR ratifies it — but in language
   that protects against accidentally building an EKF "for convenience" later.

## Why (operational lens — where it cuts the other way)

A working robot for real-world deployment usually fuses GPS. We are
deliberately leaving operational performance on the table to preserve
evaluation integrity:

- Wheel-odom drift will visibly accumulate on multi-lap runs. We accept
  this; the controller is still producing correct *belief-frame* commands.
- Terminal pose after lap N will not match start. Operators must reset the
  robot to a known starting pose between scenarios. Acceptable for
  experiment runs that are scripted scenario-by-scenario.
- Wheel slip / hop / debris will corrupt the controller's belief locally.
  This is the disturbance the H-inf design is meant to absorb; it's part of
  the experiment, not a bug.

## Hybrids considered and rejected

| Approach | Why rejected |
|---|---|
| GPS resets odom drift periodically; controller still on wheel odom | Reset events create state jumps that are themselves the controller's input — same circularity, intermittent. |
| Fuse only yaw from GPS velocity vector | Heading from velocity is unreliable below ~0.3 m/s; useless at start/stop and noisy at rooftop speeds. |
| Two-loop (GPS waypoint outer + wheel-odom tracking inner) | Turns a single-controller paper into a navigation-system paper. Different scope, different evaluation. |
| Run sims with GPS-quality noise to match real | Doesn't solve the ground-truth circularity; only papers over the sim-vs-real gap. |

## What we DO use GPS for

1. **Live battle-station display**: lat/lon marker (distinct from
   odom-projected marker), RTK fix-quality bar, NTRIP correction age. So
   the operator can see RTK quality in real time and abort runs where it's
   degraded.
2. **RTK gating** (per spec §4.7 line 96): refuse to start a recorded run
   when fix quality is below threshold (configurable; default
   `RTK_FIXED` for papers, `RTK_FLOAT` for shakedown runs).
3. **Rosbag recording** of `/gps_rtk_f9p_helical/gps/{fix,rtk_status}` (and
   the corresponding pixhawk topics if relevant) per spec §11 line 714.
4. **Post-hoc analysis**: compute path-tracking error in two flavors —
   wheel-odom-belief and RTK-truth — and report the gap.
5. **Optional ablation paragraph** in the paper: post-hoc EKF on the
   rosbag to compute "what controller performance would have been with
   fusion." Pure offline analysis, doesn't touch real-time control.

## When to revisit

Open this ADR for renegotiation if any of the following becomes the new
research question:

- Long-duration outdoor autonomy where wheel drift dominates and the paper
  is no longer about the controller in isolation.
- A v2 paper specifically about the controller + state-estimator as a
  combined system, with an independent ground-truth source (total station,
  motion capture, photogrammetry — i.e., something that is *not* the same
  GPS in the loop).
- A switch from single-antenna RTK to dual-antenna RTK that gives true
  heading at zero speed; some of the operational cons go away.
- A different platform whose wheel encoders are unreliable enough that
  pure odom-based control is infeasible regardless of paper concerns.
