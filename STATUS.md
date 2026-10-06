# Status

*Last updated: 2026-10-06.* This is a public snapshot. The live working checklist and
the detailed dated log are in [`ToDo.md`](ToDo.md), which is canonical for current
status.

| | |
|---|---|
| **Phase** | Rooftop data collection |
| **Progress** | **107 / 680** usable headline runs (as of 2026-10-03). The remaining 573 come to about 11.5 robot-hours at the measured ~72 s per leg. |
| **Next field step** | A steering sweep through a patched chassis driver (below) |

## Known issues

1. **Steering reached the wheels at ~0.4× the command.** The stock AgileX ROS 2
   `limo_base` driver converts the commanded bicycle steering to an inner-wheel
   angle, clamps it at 28° and sends the result divided by 2.47. Full lock was
   therefore 0.198 rad, about R ≈ 1.0 m. The cause is software and fixable: a
   patched driver with a `steering_mode` parameter (default = stock behaviour) is
   in progress. The physical steering limit has not been measured yet.
   **Every scored run so far, including the 107 above, was driven this way.**
   Whether those runs count toward the paper is an open decision for the advisor.
   Detail: [`SPEC.md` §7.8](SPEC.md).
2. **The 1.0 m/s speed cap is unlocated.** It is not in the ROS protocol, since
   speed travels as int16 ×1000, so it must sit in the chassis firmware or the
   motor limits. A wheels-off-the-ground stand test is pending
   ([`SPEC.md` §5-M2](SPEC.md)).
3. **The headline claim is under revision.** The simulation's "curvature
   crossover" framing did not survive verification and has been withdrawn. The
   working hypothesis is now more repeatable (lower-variance) tracking under real
   disturbance, pending the measurements above and the advisor's sign-off
   ([`SPEC.md` §0](SPEC.md)).

## Next steps

1. **Steering sweep** through the patched driver: raw chassis steering from
   0.20 to 0.45 rad at crawl speed, with an RTK circle fit, to find the real
   δ_max and R_min ([`SPEC.md` §5-M1](SPEC.md)).
2. **Speed stand test**, wheels off the ground, to locate the 1.0 m/s cap.
3. **Pilot the wiggle stages** (wiggle_sine 2.0, wiggle_sine 1.2, wiggle_square
   1.2, wiggle_chirp 1.5).
4. **Advisor decision** on the runs already recorded.
5. **Revisit the choices that were sized to the stock-driver ceiling** once the
   real limit is known: the glue track radius, the wiggle radii and the step/slalom
   R axis.
