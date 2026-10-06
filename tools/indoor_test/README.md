# Indoor sample curve

A 5×5 m workspace with 0.5 m wall margin (usable 4×4 m). Robot starts at
(0.5, 2.5) in the `odom` frame facing +x and follows a sinusoid to (4.5, 2.5):

- `y(x) = 2.5 + 0.7·sin(2π·(x − 0.5)/4)`, x ∈ [0.5, 4.5] — one full period over 4 m.
- y stays in [1.8, 3.2] (1.3 m clear of every wall); arc length 5.02 m.
- |κ|_max = 1.727 → R_min = 0.579 m (above the LIMO ~0.37 m steering limit, with margin).
  *2026-10-04: not reachable through the stock `limo_base` driver, whose
  steering scale caps the turn at R ≈ 1.0 m — see `SPEC.md` §7.8.*
- ρ_max = |κ|·v ≤ 1.73 even at v = 1.0 m/s — well inside the LPV envelope (ρ_max = 5).

Files: `tools/indoor_test/publish_indoor_path.py` (latched publisher on
`/reference_path`, 81 waypoints, frame `odom`), `run_indoor.sh` (launches the
follower + publisher), `out/indoor_path.png` (overlay + curvature plot).

Run on the NUC (after rsync), robot placed at ≈(0.5, 2.5) facing +x:
```
cd /home/agilex/H-infinity/tools/indoor_test
./run_indoor.sh 0.3        # speed in m/s, default 0.3
```
Ctrl+C stops both the controller and the path publisher.
