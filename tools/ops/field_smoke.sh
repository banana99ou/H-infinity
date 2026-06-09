#!/bin/bash
# One-shot autonomous smoke run for the LIMO — bring up, gate, arm, watch.
# Runs the canonical autonomous path (DOC/deployment.md §Runbook): it only
# *tells the system* and watches; it never drives a component by hand. The only
# motion is the sequencer's, gated by RTK + the geofence + this operator's go.
#
# OPERATOR PRECONDITIONS (wheels-on-floor, outdoors):
#   - Chassis in Ackermann mode (physical switch).
#   - Robot placed at/near the START pin (S1) facing ~toward E1. The first
#     reposition self-calibrates the Pixhawk compass on its first move (>=~0.5 m
#     of travel; if it sits exactly on the pin it bootstraps via a short creep)
#     and PERSISTS the offset into the venue file, so every later leg boots
#     calibrated. If reposition limit-cycles instead of converging, place the
#     robot ~1-2 m back from S1 (clear space ahead) so it gets a calibrating drive.
#   - RTK base up; rover acquiring.
#
# USAGE (on the NUC, e.g.  ssh agilex@nuc 'bash ~/H-infinity/tools/ops/field_smoke.sh'):
#   field_smoke.sh                 full run: gates -> arm -> watch
#   field_smoke.sh --gates-only    bring-up + RTK + compass + preflight + geofence,
#                                  then STOP (NO motion). Indoor dry-run of the gates.
#   field_smoke.sh --no-arm        gates + geofence, leave stack up, do not arm.
#   field_smoke.sh --rtk-timeout N --run-timeout N
#
# NOT set -u: ROS setup.bash references unset vars.
set -o pipefail
source /opt/ros/humble/setup.bash
source /home/agilex/agilex_ws/install/setup.bash

REPO=/home/agilex/H-infinity
GATES_ONLY=0; NO_ARM=0; RTK_TIMEOUT=120; RUN_TIMEOUT=900
while [ $# -gt 0 ]; do case "$1" in
  --gates-only) GATES_ONLY=1;;
  --no-arm) NO_ARM=1;;
  --rtk-timeout) RTK_TIMEOUT="$2"; shift;;
  --run-timeout) RUN_TIMEOUT="$2"; shift;;
  *) echo "unknown arg: $1"; exit 2;;
esac; shift; done

say(){ echo "[field $(date +%H:%M:%S)] $*"; }
start(){ ros2 topic pub --once /orchestrator/start std_msgs/msg/String "{data: $1}" >/dev/null 2>&1; }
kill_(){ ros2 topic pub --once /orchestrator/kill  std_msgs/msg/String "{data: $1}" >/dev/null 2>&1; }
estop(){ ros2 topic pub --once /estop_trigger std_msgs/msg/Bool "data: true" >/dev/null 2>&1; }
safe_stop(){ say "SAFE-STOP: kill movers + latch estop"; kill_ sequencer_smoke; kill_ follower; kill_ reposition; estop; }
abort(){ say "ABORT: $*"; safe_stop; exit 1; }

# 1 ----- bring up sensors + safety chain (NOT geofence yet: it would stale-trip
#         on a not-yet-FIXED fix and latch an estop before we even start) --------
say "bring up: base (chassis), gnss (mavros+RTK), heading (EKF), estop, odom_zero, ops, odom_watchdog"
# Split chassis (base) from mavros+RTK (gnss) so the odom_watchdog can respawn
# the chassis driver on a base-serial dropout WITHOUT dropping RTK/compass.
# 'heading' is the always-on heading EKF (heading_node): reposition consumes its
# /heading/fused and HOLDs without it, so bring it up here with the sensor stack.
start base; start gnss; start heading; start estop; start odom_zero; start ops
# odom_watchdog: auto-recovers the chassis serial if /wheel/odom goes silent
# (only acts when 'base' is alive, so safe to arm now during bring-up).
start odom_watchdog
sleep 8

# 2 ----- RTK gate: quality in {4 FIXED, 5 FLOAT} (TEMP field acceptance) --------
say "waiting up to ${RTK_TIMEOUT}s for RTK quality in {4,5}..."
q=""
for i in $(seq 1 "$RTK_TIMEOUT"); do
  s=$(timeout 2 ros2 topic echo --once /gps_rtk_f9p_helical/gps/rtk_status 2>/dev/null)
  q=$(printf '%s' "$s" | grep -o 'quality=[0-9]*' | head -1 | cut -d= -f2)
  [ "$q" = "4" ] || [ "$q" = "5" ] && { say "RTK quality=$q"; break; }
  sleep 1
done
[ "$q" = "4" ] || [ "$q" = "5" ] || abort "RTK not FIXED/FLOAT within ${RTK_TIMEOUT}s (last quality=${q:-none})"

# 3 ----- compass gate: the heading linchpin MUST be live + sane under RTK -------
say "verifying /pixhawk/global_position/compass_hdg is live..."
cv=""
for i in $(seq 1 25); do
  cv=$(timeout 2 ros2 topic echo --once /pixhawk/global_position/compass_hdg 2>/dev/null \
        | grep -o 'data: [-0-9.eE]*' | awk '{print $2}')
  [ -n "$cv" ] && break
  sleep 1
done
[ -n "$cv" ] || abort "compass_hdg SILENT under RTK -> compass approach blocked (see ToDo / project_pixhawk_compass_needs_gps); fall back to COG or fix autopilot stream"
ok=$(python3 -c "v=float('$cv'); print(1 if (v==v and 0.0<=v<=360.0) else 0)" 2>/dev/null)
[ "$ok" = "1" ] || abort "compass_hdg present but not sane ($cv)"
say "compass_hdg = $cv deg (live, sane)"

# 4 ----- preflight gate (no wheels move) ---------------------------------------
say "preflight..."
bash "$REPO/tools/preflight/preflight.sh" || abort "preflight failed (see output above)"

# 5 ----- geofence: arm only now that RTK is live (no stale-trip) ----------------
say "arming geofence (venue polygon RTK watchdog)"
start geofence; sleep 3

if [ "$GATES_ONLY" = "1" ]; then
  say "GATES-ONLY: all gates passed, NO motion commanded. Stack left up. Done."
  exit 0
fi
if [ "$NO_ARM" = "1" ]; then
  say "--no-arm: gates passed + geofence armed; not arming the run. Done."
  exit 0
fi

# 6 ----- ARM the autonomous run (sequencer owns all motion from here) ----------
say "ARM run_smoke_e2e (confirmed_wheels_on_floor=true) — sequencer drives the legs"
ros2 topic pub --once /ops/cmd std_msgs/msg/String \
  '{data: "{\"action\":\"run_smoke_e2e\",\"confirmed_wheels_on_floor\":true}"}' >/dev/null 2>&1
sleep 2

# 7 ----- watch until terminal; safe-stop on anything but a clean done ----------
say "watching the run (run-timeout ${RUN_TIMEOUT}s)..."
python3 "$REPO/tools/ops/watch_experiment.py" --timeout "$RUN_TIMEOUT"
rc=$?
if [ "$rc" = "0" ]; then
  say "RUN PASSED (phase=done, fail=0). Bag + sidecar under Experiment Data/."
else
  say "RUN did NOT cleanly pass (watcher rc=$rc)."
  safe_stop
fi
exit "$rc"
