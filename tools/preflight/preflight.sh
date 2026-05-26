#!/usr/bin/env bash
# Pre-floor-test verification for the H-infinity LIMO stack.
# Run on the NUC. Exit 0 only if every check passes.
#
# Usage:
#   preflight.sh           # quick: static + topology checks (~15 s)
#   preflight.sh --full    # quick + synthetic-odom dynamic test (~60 s)
#
# Quick mode does NOT touch any actuator. Full mode brings up the
# path_follower under synthetic odometry and verifies the cmd_vel_raw
# stream and the E-stop reflex without ever clearing the latched E-stop.

set -o pipefail

MODE="${1:-quick}"

PASS=0
FAIL=0
FAILED_CHECKS=()

# -- Tunables that still need on-robot confirmation ----------------------
# The RTK driver (GPS-RTK_ROS2_pub_node.py) publishes a verbose human-readable
# String on the rtk_status topic of the form:
#   "FIX: RTK FIXED (quality=4, sats=.., HDOP=.., rate=..Hz) | Lat=.., Lon=.. | RTCM: .."
# The most stable FIXED token is the NMEA fix-quality field (quality=4), so we
# match on that rather than the prose — a wording change in the driver should
# not silently flip the gate. quality=4 == FIXED, quality=5 == FLOAT (source:
# fix_quality_to_desc() in GPS-RTK_ROS2_pub_node.py).
#
# hw-confirmed 2026-05-26: rtk_status is std_msgs/String and carries a
# "quality=N" token, e.g. NO FIX gave
#   "FIX: NO FIX (quality=0, sats=0, HDOP=99.99, rate=10.0Hz) | ..."
# so the "quality=N" substring match below is the right shape and the gate
# correctly FAILs when not FIXED. Still pending: an open-sky FIXED to confirm
# the literal quality=4 end-to-end (was NO FIX indoors during verification).
RTK_STATUS_TOPIC="/gps_rtk_f9p_helical/gps/rtk_status"
RTK_FIXED_MATCH="quality=4"   # TODO(hw-verify): confirm literal at open-sky FIXED
RTK_FLOAT_MATCH="quality=5"   # TODO(hw-verify): FLOAT -> WARN (not yet converged)

# Run-window RTK-FIXED coverage requirement Y (spec §5: "FIXED for >= Y% of the
# window"). This is a per-run acceptance gate the sequencer (T6) will enforce
# against the bag; preflight only checks instantaneous FIXED. Surfaced here so
# the number lives in one place once known.
# TODO(hw-verify): set run-window RTK-FIXED coverage Y (percent) — UNKNOWN.
RTK_RUN_WINDOW_PCT_Y=""   # TODO(hw-verify): e.g. 95 — confirm with operator

green()  { printf '\033[32m%s\033[0m' "$1"; }
red()    { printf '\033[31m%s\033[0m' "$1"; }
yellow() { printf '\033[33m%s\033[0m' "$1"; }

ok()   { printf '  [%s] %s\n'        "$(green PASS)" "$1"; PASS=$((PASS+1)); }
bad()  { printf '  [%s] %s — %s\n'   "$(red  FAIL)" "$1" "$2"; FAIL=$((FAIL+1)); FAILED_CHECKS+=("$1"); }
warn() { printf '  [%s] %s — %s\n'   "$(yellow WARN)" "$1" "$2"; }
section() { printf '\n%s\n' "== $1 =="; }

# -- Source ROS env if not already sourced -------------------------------
if ! command -v ros2 >/dev/null 2>&1; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  # shellcheck disable=SC1091
  source /home/agilex/agilex_ws/install/setup.bash 2>/dev/null || true
fi

# -- 1. Service state ----------------------------------------------------
section "service"
if systemctl is-active --quiet limo-battle.service; then
  ok "limo-battle.service active"
else
  bad "limo-battle.service" "not active (run: sudo systemctl start limo-battle.service)"
fi

# -- 2. ros2 daemon + node list ------------------------------------------
section "node graph"
NODES="$(ros2 node list 2>/dev/null | sort | uniq)"
if [[ -z "$NODES" ]]; then
  bad "ros2 node list" "empty — daemon down or no nodes running"
  printf '\n%s\n' "$(red "Aborting: cannot run further checks without a node graph")"
  exit 1
fi

needed=(limo_base_node limo_estop_cli limo_orchestrator rosbridge_websocket)
for n in "${needed[@]}"; do
  if grep -qx "/$n" <<<"$NODES"; then
    ok "node /$n present"
  else
    bad "node /$n" "missing from graph"
  fi
done

pf_count=$(grep -c "^/path_follower_node" <<<"$NODES" || true)
case "$pf_count" in
  0) warn "path_follower_node" "not running (orchestrator-managed; OK if untriggered)" ;;
  1) ok   "path_follower_node single instance" ;;
  *) bad  "path_follower_node" "$pf_count instances — duplicate processes on the graph" ;;
esac

# -- 3. Safety chain pub/sub sets ----------------------------------------
section "safety chain (cmd_vel_raw -> estop -> cmd_vel)"

# /cmd_vel: publisher set must be {limo_estop_cli}, subscriber set must contain limo_base_node
cmd_vel_info="$(ros2 topic info /cmd_vel --verbose 2>/dev/null)"
cmd_vel_pubs="$(awk '/Publisher count/{p=1;next} /Subscription count/{p=0} p && /Node name:/{print $3}' <<<"$cmd_vel_info" | sort -u)"
cmd_vel_subs="$(awk '/Subscription count/{s=1;next} s && /Node name:/{print $3}'                          <<<"$cmd_vel_info" | sort -u)"

if [[ "$cmd_vel_pubs" == "limo_estop_cli" ]]; then
  ok "/cmd_vel publishers = {limo_estop_cli}"
else
  bad "/cmd_vel publishers" "got '$cmd_vel_pubs' — only limo_estop_cli is allowed"
fi

if grep -qx "limo_base_node" <<<"$cmd_vel_subs"; then
  ok "/cmd_vel subscribers include limo_base_node"
else
  bad "/cmd_vel subscribers" "limo_base_node not subscribing — wheels will not move"
fi

# /cmd_vel_raw: subscriber set must be exactly {limo_estop_cli}
raw_info="$(ros2 topic info /cmd_vel_raw --verbose 2>/dev/null)"
raw_subs="$(awk '/Subscription count/{s=1;next} s && /Node name:/{print $3}' <<<"$raw_info" | sort -u)"
if [[ "$raw_subs" == "limo_estop_cli" ]]; then
  ok "/cmd_vel_raw subscribers = {limo_estop_cli}"
else
  bad "/cmd_vel_raw subscribers" "got '$raw_subs' — only estop should subscribe"
fi

# /cmd_vel_raw: publisher set must contain EXACTLY ONE node (C6). Only one of
# {follower, reposition} may drive the wheels at a time; two live publishers
# means the sequencer's exclusivity invariant is broken and commands would mix.
# An empty set is allowed here (nothing triggered yet) — quick mode does not
# require a controller to be running, same as the path_follower_node check.
raw_pubs="$(awk '/Publisher count/{p=1;next} /Subscription count/{p=0} p && /Node name:/{print $3}' <<<"$raw_info" | sort -u)"
raw_pub_count="$(grep -c . <<<"$raw_pubs")"
[[ -z "$raw_pubs" ]] && raw_pub_count=0
case "$raw_pub_count" in
  0) warn "/cmd_vel_raw publishers" "none live (OK if no controller triggered yet)" ;;
  1) ok   "/cmd_vel_raw single publisher = {$raw_pubs}" ;;
  *) bad  "/cmd_vel_raw publishers" "$raw_pub_count live ($(tr '\n' ' ' <<<"$raw_pubs"))— exactly one allowed (C6)" ;;
esac

# -- 4. Ackermann mode ---------------------------------------------------
section "chassis"
mm="$(timeout 3 ros2 topic echo --once /limo_status 2>/dev/null | awk '/^motion_mode:/{print $2; exit}')"
case "$mm" in
  1) ok   "motion_mode = 1 (Ackermann)" ;;
  "") bad "motion_mode" "no /limo_status message in 3 s — base driver silent" ;;
  *)  bad "motion_mode" "= $mm — chassis NOT in Ackermann; set physical mode switch" ;;
esac

batt="$(timeout 3 ros2 topic echo --once /limo_status 2>/dev/null | awk '/^battery_voltage:/{print $2; exit}')"
if [[ -n "$batt" ]]; then
  # 11.1 V nominal LiPo; warn under 10.8, fail under 10.5.
  if awk -v v="$batt" 'BEGIN{exit !(v+0 < 10.5)}'; then
    bad "battery_voltage" "$batt V — too low for a run, charge first"
  elif awk -v v="$batt" 'BEGIN{exit !(v+0 < 10.8)}'; then
    warn "battery_voltage" "$batt V — getting low"
  else
    ok "battery_voltage = $batt V"
  fi
fi

# -- 5. RTK-FIXED gate (M1) ----------------------------------------------
# Ground truth (and reposition) require a FIXED RTK solution. Spec §5 gates each
# cell on sustained FIXED. Preflight checks the INSTANTANEOUS state only; the
# K-second dwell and the run-window >= Y% coverage are the sequencer's job (T6).
section "rtk"
rtk_status="$(timeout 3 ros2 topic echo --once "$RTK_STATUS_TOPIC" 2>/dev/null \
              | awk -F'data: ' '/^data:/{print $2; exit}')"
# Strip surrounding quotes ros2 adds around String payloads.
rtk_status="${rtk_status#\"}"; rtk_status="${rtk_status%\"}"
if [[ -z "$rtk_status" ]]; then
  bad "$RTK_STATUS_TOPIC" "no message in 3 s — RTK driver silent (start the GNSS stack)"
elif grep -qF "$RTK_FIXED_MATCH" <<<"$rtk_status"; then
  ok "RTK FIXED ($RTK_FIXED_MATCH)"
elif grep -qF "$RTK_FLOAT_MATCH" <<<"$rtk_status"; then
  warn "RTK status" "FLOAT, not FIXED — wait for convergence before a recorded run [$rtk_status]"
else
  bad "RTK status" "not FIXED — got [$rtk_status]"
fi

# -- 6. Odom liveness ----------------------------------------------------
section "odometry"
hz_line="$(timeout 3 ros2 topic hz /wheel/odom 2>&1 | awk '/average rate/{print; exit}')"
if [[ -n "$hz_line" ]]; then
  hz=$(awk '{print $3}' <<<"$hz_line")
  if awk -v h="$hz" 'BEGIN{exit !(h+0 > 30)}'; then
    ok "/wheel/odom rate = ${hz} Hz"
  else
    bad "/wheel/odom rate" "${hz} Hz — expected > 30 Hz from limo_base_node"
  fi
else
  bad "/wheel/odom" "no messages in 3 s — base driver not publishing odom"
fi

# -- 7. Optional dynamic test (synthetic odom) ---------------------------
# Wheels-up synthetic-odom is allowed without operator OK (CLAUDE.md safety
# contract); ANY step that could actually move the wheels is gated behind an
# explicit prompt below. The controller's only output path is
# cmd_vel_raw -> estop -> /cmd_vel, so while the E-stop is LATCHED, /cmd_vel
# stays zero even though cmd_vel_raw carries a live command — that is what lets
# us watch the controller stream safely.
if [[ "$MODE" == "--full" ]]; then
  section "dynamic test (synthetic odom)"

  # Abort the dynamic phase if the quick phase already failed: do not feed
  # synthetic input into a graph that is not in a known-good state.
  if (( FAIL > 0 )); then
    warn "dynamic" "skipped — quick checks failed; fix those first"
  else
    # 7a. E-stop must be LATCHED before we publish any controller input, so
    #     the cmd_vel_raw stream cannot reach the wheels. /estop true == latched
    #     (estop_cli zeros /cmd_vel while latched).
    estop_state="$(timeout 3 ros2 topic echo --once /estop 2>/dev/null \
                   | awk '/^data:/{print $2; exit}')"
    if [[ "$estop_state" != "true" ]]; then
      bad "dynamic precondition" "/estop is '$estop_state', expected latched (true) — refusing to inject synthetic odom while the actuation path is open"
    else
      ok "E-stop latched — cmd_vel_raw cannot reach wheels"

      # 7b. Publish synthetic /wheel/odom (offset pose, yaw=0) in the background.
      #     Wheels-up safe: nothing actuates while estop is latched.
      ros2 topic pub --rate 30 /wheel/odom nav_msgs/msg/Odometry \
        '{pose: {pose: {position: {x: 1.0, y: 1.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}, twist: {twist: {linear: {x: 0.5, y: 0.0, z: 0.0}}}}' \
        >/dev/null 2>&1 &
      odom_pid=$!
      # NOTE(hw-verify): this assumes the follower is already running (orchestrator-
      # managed) and has a path/recipe loaded. If path_follower_node is absent the
      # cmd_vel_raw check below will WARN, not FAIL — wire the orchestrator
      # /orchestrator/start + a /reference_path_recipe publish here once T1/T6 land.

      # 7c. Verify the controller produces a non-zero cmd_vel_raw stream.
      raw_lin="$(timeout 4 ros2 topic echo --once /cmd_vel_raw 2>/dev/null \
                 | awk '/^  x:/{print $2; exit}')"
      if [[ -z "$raw_lin" ]]; then
        warn "cmd_vel_raw stream" "no message in 4 s — is the follower running with a path?"
      elif awk -v v="$raw_lin" 'BEGIN{exit !(v+0 > 0)}'; then
        ok "cmd_vel_raw linear.x = $raw_lin (> 0, controller live)"
      else
        warn "cmd_vel_raw stream" "linear.x = $raw_lin (expected > 0 under offset odom)"
      fi

      # 7d. E-stop reflex: while latched, /cmd_vel must stay zero despite the
      #     live cmd_vel_raw above. This proves the gate, without clearing it.
      cmd_lin="$(timeout 3 ros2 topic echo --once /cmd_vel 2>/dev/null \
                 | awk '/^  x:/{print $2; exit}')"
      if [[ -z "$cmd_lin" ]]; then
        warn "estop reflex" "no /cmd_vel message in 3 s (estop may not be republishing zeros)"
      elif awk -v v="$cmd_lin" 'BEGIN{exit !(v+0 == 0)}'; then
        ok "estop reflex — /cmd_vel held at 0 while latched"
      else
        bad "estop reflex" "/cmd_vel linear.x = $cmd_lin while estop latched — gate LEAKING"
      fi

      # Tear down the synthetic odom publisher.
      kill "$odom_pid" 2>/dev/null || true
      wait "$odom_pid" 2>/dev/null || true

      # 7e. Wheels-on-floor end-to-end (clear estop, let a real command drive the
      #     wheels) is intentionally NOT automated: it requires explicit operator
      #     confirmation per the safety contract. Prompt and only proceed on an
      #     explicit "yes"; default and non-interactive runs decline.
      printf '\n  %s\n' "Run the WHEELS-ON-FLOOR leg (clears E-stop, robot will MOVE)? [y/N] "
      read -r -t 30 floor_ans || floor_ans=""
      if [[ "$floor_ans" == "y" || "$floor_ans" == "Y" ]]; then
        # TODO(hw-verify): implement the moving leg only after a live operator OK:
        #   clear /estop_trigger, confirm /cmd_vel tracks cmd_vel_raw, drive a short
        #   bounded segment, re-latch. Left unimplemented until the hardware session.
        warn "wheels-on-floor" "operator confirmed, but moving leg not implemented yet (hardware session)"
      else
        ok "wheels-on-floor leg declined — E-stop left latched"
      fi
    fi
  fi
fi

# -- Summary -------------------------------------------------------------
section "summary"
total=$((PASS+FAIL))
printf '  %d/%d checks passed\n' "$PASS" "$total"
if (( FAIL == 0 )); then
  printf '  %s\n' "$(green 'READY for next stage')"
  exit 0
else
  printf '  %s:\n' "$(red 'FAILED')"
  for c in "${FAILED_CHECKS[@]}"; do printf '    - %s\n' "$c"; done
  exit 1
fi
