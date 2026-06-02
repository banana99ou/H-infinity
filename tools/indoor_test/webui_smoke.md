# Webui smoke runbook — Tier 0

Indoor, synthetic, no driving needed. Validates every webui indicator under
known inputs. Uses `synth_publishers.sh` in this directory.

## Setup (once)

1. NUC up, `rosbridge_websocket` running on `:9090`.
2. Sync the publisher script to NUC: `tools/sync/sync.sh push` from the laptop.
   On the NUC it lives at `~/H-infinity/tools/indoor_test/synth_publishers.sh`.
3. Laptop browser open at `tools/path_gen/interactive.html`, ws URL
   `ws://agilex-nuc12wski7:9090`, **Connect** clicked. Conn dot should be green.
4. Keep a NUC terminal handy. ROS is auto-sourced by the script.

### Important: stop competing real publishers first

If `limo_base`, `GPS-RTK_ROS2_pub_node`, MAVROS, `path_follower_node`, etc. are
still running on the NUC, they will share `/limo_status`,
`/gps_rtk_f9p_helical/*`, `/path_follower/status`, etc. with the synthetic
publishers. The webui will see whichever publisher's message arrives first each
cycle — usually the real one, drowning out the synthetic. For a clean Tier-0
run, kill the real stack first:

```
ros2 topic pub --once /orchestrator/kill std_msgs/msg/String 'data: base_vanilla'
ros2 topic pub --once /orchestrator/kill std_msgs/msg/String 'data: follower'
ros2 topic pub --once /orchestrator/kill std_msgs/msg/String 'data: estop'
# or, if orchestrator isn't running, blunt-force:
pkill -f 'limo_base|path_follower_node|odom_zero_node|reposition_node|GPS-RTK_ROS2_pub_node|estop_cli|MAVROS'
```

Confirm with `ros2 node list` — only `rosbridge_websocket` (and whatever else
you intend) should remain. Then proceed with the synthetic publishers.

For each row below, run the command in a NUC terminal (Ctrl-C when done) and
eyeball the webui. Tick the box if the observed behavior matches expected.

## Connection block

- [ ] `synth_publishers.sh battery 11.7` → `● battery: 11.7 V` in green
- [ ] `synth_publishers.sh battery 10.6` → 10.6 V in **orange/warn**
- [ ] `synth_publishers.sh battery 10.3` → 10.3 V in **red/err**
- [ ] Click **Disconnect** in webui → battery row resets to `--`, dot clears
- [ ] Reconnect → battery row stays `--` until publisher restarted (no stale value)

## E-stop banner + button

- [ ] `synth_publishers.sh estop on` → big red banner appears: "WHEELS DISABLED — E-STOP LATCHED". Button text flips to **CLEAR E-STOP** (green).
- [ ] `synth_publishers.sh estop off` → banner hides. Button back to red **E-STOP**.
- [ ] Click webui **E-STOP** button while `estop off` is the last published state → button publishes to `/estop_trigger`. Confirm by `ros2 topic echo /estop_trigger` in a side terminal.
- [ ] Click webui **E-STOP** with no rosbridge connection → no crash, just an alert.

## RTK & GPS section

- [ ] `synth_publishers.sh rtk_status fixed 0.5` → `--, -- — FIXED RTCM OK (0.5s)` (green pill), dot ok (paired with rtk_fix below)
- [ ] In parallel: `synth_publishers.sh rtk_fix 37.61247 126.99426` → magenta dot appears on map at that location. Line reads e.g. `37.61247000, 126.99426000 — FIXED RTCM OK (0.5s)`.
- [ ] Stop the `rtk_status fixed` publisher and run `rtk_status float 0.5` → fix label becomes `not FIXED (quality=5)`, magenta dot turns **amber**.
- [ ] Stop and run `rtk_status stale 42` → RTCM pill flips to red `RTCM STALE (42.0s)`; quality string still shows.
- [ ] Stop and run `rtk_status nofix` → all-zero lat/lon, quality=0; nothing magenta drawn.
- [ ] `synth_publishers.sh pixhawk_fix 37.61250 126.99428` → orange dot appears, offset from magenta.

## Map markers (visual)

With multiple publishers running, confirm dots line up with the legend:

- [ ] Open the **Map legend** `<details>` under Map. Compare hex swatches to dots on the map.
- [ ] `synth_publishers.sh odom_zeroed 1.0 0.5 0.0` → blue dot appears at projected (1.0, 0.5) under current lat/lon/bearing inputs. (Open Ad-hoc details once if you need to set the anchor lat/lon/bearing so the projection is sensible.)
- [ ] Vary `odom_zeroed 0 0 1.57` → heading line on the blue dot rotates to roughly north.

## Telemetry block

- [ ] `synth_publishers.sh follower_status 0.5` → telemetry text fills with pose, v, kappa, rho, e_psi, delta. Progress bar advances to 50% (`s_star=5 / total=10`).
- [ ] Vary `follower_status 1.5` → v reads 1.50 m/s.
- [ ] Stop publisher → after ~1.5 s the follower-dot in Connection goes red and follower rate drops to 0.

## Experiment panel

- [ ] `synth_publishers.sh exp_status driving` → cell `1 / 3`, phase pill `driving`, ETA 2m 0s, pass=1 fail=0.
- [ ] Replace with `exp_status preflight`, `exp_status paused`, `exp_status done` → phase pill updates.
- [ ] Click **Pause** → confirm with `ros2 topic echo /experiment/cmd` in a side terminal: `data: '{"action": "pause"}'`. Same for **Resume**, **Skip cell**, **ABORT** (Abort additionally publishes `/estop_trigger true`).

## Stack panel (collapsed under Stack debug)

- [ ] Open the **Stack debug** `<details>`.
- [ ] `synth_publishers.sh orch_status base_vanilla,estop` → green dots on those two rows, red on `base_gnss`, `follower`.
- [ ] Vary to `orch_status base_vanilla,estop,follower` → follower row green.
- [ ] Click **start** / **kill** buttons in any row → confirm `/orchestrator/start` / `/orchestrator/kill` publish via side `ros2 topic echo`.

## Venue editor (no ROS needed)

- [ ] Click **Add corner (click map)** → cursor crosshair, hint shows. Click 4 spots on the map. Yellow polygon + green dashed inset draw.
- [ ] **Add start pin** → click anywhere. Green dot with cyan heading arrow. Drag pin to move; drag tip to rotate.
- [ ] **Add end pin** → same with orange.
- [ ] **Add exclusion** → red disc.
- [ ] **Validate current curve** → with ad-hoc closed (no path), expect FAIL "no current curve to validate". Open Ad-hoc, leave default path → expect PASS / FAIL depending on whether the default curve fits.
- [ ] **Export venue JSON** → file downloads. **Load rooftop.json** → file dialog opens.

## Manual control

- [ ] Click **Stop scenario** → confirm via echo: `/estop_trigger true`, `/path_follower/reset true`, `/reference_path` published with empty `poses`.
- [ ] Click **Recenter map on robot** → with `odom_zeroed` publisher running, view jumps to blue dot.
- [ ] Click **Toggle keyboard teleop** → floating panel opens. Press W → t-lv goes positive; A → t-av positive. Side terminal: `ros2 topic echo /cmd_vel_raw` shows non-zero twists at 20 Hz while held. Release → zero twist. Press SPACE → zero twist + status warns. Close panel → publisher unadvertises (`ros2 topic info /cmd_vel_raw --verbose` shows fewer publishers).

## Map options + Ad-hoc

- [ ] Switch basemap to Google Satellite, Hybrid, Esri, OSM → tiles change.
- [ ] Expand **Ad-hoc curve (legacy)** `<details>` → yellow dashed rectangle + cyan path-anchor + red curve appear on map. Stats populate.
- [ ] Change **Path family** → curve changes, stats update.
- [ ] Drag cyan position dot → translates curve. Drag cyan tip → rotates.
- [ ] Click **Snap to robot pose** → moves anchor to current `odom_zeroed`.
- [ ] **Reset (0,0,0)** → anchor returns to origin.
- [ ] **Push & Run** → publishes `/reference_path` (with current anchor applied) and clears `/estop_trigger`.
- [ ] Collapse Ad-hoc `<details>` → yellow rectangle, cyan anchor, red curve, start/end markers all disappear from map.

## Cleanup

- [ ] Ctrl-C every running publisher.
- [ ] Click **Disconnect** in webui.
- [ ] All dots → null/grey, all readouts → `--`.

---

If every box ticks, the webui is verified end-to-end without touching the
robot's wheels, without sky view, without a basestation. Failures here narrow
the search for field-test issues from "system" to "this one indicator vs. this
one topic".
