# H-infinity LIMO

Hardware bring-up of the professor's `scalecar-vfg-h-infinite` path-following
controller (VFG + LPV-H∞) on the AgileX LIMO. The goal is a **paper-grade
dataset** comparing LPV-H∞ vs PID-FF across a curvature sweep at fixed speed,
gathered unattended on a rooftop track with RTK as post-hoc ground truth.

**The robot is the source of truth.** This laptop has no ROS2 — everything
runs on the NUC. See `CLAUDE.md` for the edit → rsync → build → run → verify
loop.

## Canonical docs — and what each owns

| Doc | Owns |
|---|---|
| `DOC/system_spec.md` | **What the system must be** — locked requirements (ROC), the canonical interface contract (§4), acceptance criteria. Start here. |
| `DOC/experiment.md` | **Why + how** — the curvature-sweep pivot, the experimental matrix, the per-cell operational model, orchestrator target state. |
| `DOC/deployment.md` | NUC deployment caveats, support-script inventory, data-flow diagram, runbook. |
| `DOC/network_topology.md` | Field network (phone-on-robot + USB tether + LIMO AP) for outdoor RTK; RTK base subtopology (Pi + base F9P) for the helical F9P's RTCM source. |
| `DOC/decisions/` | Architecture Decision Records. ADR-01: GPS stays out of the control loop. |
| `DOC/paper_ijat.pdf` | The sim paper this work follows up on. |
| `ToDo.md` | Live working checklist + current status + next-session list. |
| `CLAUDE.md` | Agent operating manual: dev cycle, ssh/rsync, build/run, safety contract, conventions. |

Rule of thumb: if a fact lives in two docs, the table above says which copy is
canonical — fix that one.

## Repo map

| Path | What |
|---|---|
| `scalecar-vfg-h-infinite/` | Professor's controller + guidance library (vendored). `vfg_pathfollowing/` is the algorithm; `ros2_bridge/` is our `path_follower_node`. Keep upstream drops isolated from our patches. |
| `tools/path_gen/` | Browser battle station: Leaflet map + parametric path designer + rosbridge live link + run controls + inline teleop (`interactive.html`). |
| `tools/orchestrator/` | `start_battle.sh`, the `limo-battle` systemd service + installer. |
| `tools/network/` | NetworkManager AP-on-tether dispatcher (auto-switch client WiFi ↔ LIMO AP). |
| `tools/preflight/` | `preflight.sh` — pre-rooftop field-readiness checks. |
| `tools/rtk_base/` | RTK basestation broadcaster (Pi-side): `rtcm_server.py` + systemd unit + udev rule + installer + README. Replaces the MacBook-bound `agile_ws/rtcm_server.py` so unattended runs are possible. Pairs with the rover at `GPS-RTK_ROS2_pub_node.py`. |
| `tools/indoor_test/` | Short indoor sample curves for shaking out the system before the rooftop. |
| `tools/diagnostics/` | Ad-hoc on-robot diagnostics. |
| `scenarios/` | Scenario definitions (and per-venue WGS84 config under `venues/`). |
| `src/limo_ros2/` | LIMO base driver + launch files (`limo_base`, MAVROS, RTK launcher). |
| `legacy/` | Superseded scripts kept for reference (INI scenario runner, scripted motion baseline). |

## Indoor sample curve

A 5×5 m workspace with 0.5 m wall margin (usable 4×4 m). Robot starts at
(0.5, 2.5) in the `odom` frame facing +x and follows a sinusoid to (4.5, 2.5):

- `y(x) = 2.5 + 0.7·sin(2π·(x − 0.5)/4)`, x ∈ [0.5, 4.5] — one full period over 4 m.
- y stays in [1.8, 3.2] (1.3 m clear of every wall); arc length 5.02 m.
- |κ|_max = 1.727 → R_min = 0.579 m (above the LIMO ~0.37 m steering limit, with margin).
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
