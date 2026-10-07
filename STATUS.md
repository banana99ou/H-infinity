# Status

*Last updated: 2026-10-07.* This is a public snapshot. The live working checklist and
the detailed dated log are in [`ToDo.md`](ToDo.md), which is canonical for current
status.

| | |
|---|---|
| **Phase** | Rooftop data collection |
| **Progress** | **107 / 680** usable headline runs (as of 2026-10-03). The remaining 573 come to about 11.5 robot-hours at the measured ~72 s per leg. |
| **Next field step** | A short floor pilot on the patched driver, with RTK (below) |

## Known issues

1. **Steering reached the wheels at ~0.4× the command, and the odometry was
   skewed.** The stock AgileX ROS 2 `limo_base` driver converts the commanded
   bicycle steering to an inner-wheel angle, clamps it at 28° and sends the result
   divided by 2.47, so full lock was 0.198 rad, about R ≈ 1.0 m. Its odometry
   also dropped every heading step under 0.1° and integrated position with the
   steering angle it *believed*, so after one 90° turn it read "on the path"
   while RTK put the robot 0.2–0.7 m outside. **Both are fixed in a patched
   driver installed on 2026-10-07** (`steering_mode=direct`, `odom_model=hinf`,
   now the defaults). Measured through ROS: δ_max ≈ 0.35 rad, R_min ≈ 0.55 m.
   **Every scored run so far, including the 107 above, was driven by the stock
   driver.** Whether those runs count toward the paper is an open decision for
   the advisor. Detail: [`SPEC.md` §7.8](SPEC.md).
2. **The 1.0 m/s speed cap looks wrong.** Commanding 1.5 m/s drove ~1.2 m/s in a
   2026-06-16 recording (odom and RTK agree). It is not in the ROS protocol
   (speed travels as int16 ×1000). A wheels-off stand test is still pending
   ([`SPEC.md` §5-M2](SPEC.md)).
3. **The headline claim is under revision.** The simulation's "curvature
   crossover" framing did not survive verification and has been withdrawn. The
   working hypothesis is now more repeatable (lower-variance) tracking under real
   disturbance, pending the measurements above and the advisor's sign-off
   ([`SPEC.md` §0](SPEC.md)).

## Next steps

1. **Floor pilot on the patched driver**, with RTK: confirm the new odometry
   through real turns and that reposition behaves with full steering authority.
   Nothing on the floor has run on the patched driver yet.
2. **Safety fixes from the 2026-10-07 audit** (see `ToDo.md`): pause the run when
   odometry stops mid-leg, make the geofence fail closed, send a zero command
   before stopping a mover.
3. **Matrix decision:** R 0.5 and R 0.4 need more steering than the chassis has
   (0.38 / 0.46 rad vs ~0.35).
4. **Analysis fixes from the audit:** per-cell statistics, scoring from the
   actual start pose, real code provenance per run.
5. **Advisor decision** on the runs already recorded.
