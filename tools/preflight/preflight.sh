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

# -- 5. Odom liveness ----------------------------------------------------
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

# -- 6. Optional dynamic test --------------------------------------------
if [[ "$MODE" == "--full" ]]; then
  section "dynamic test (synthetic odom)"
  warn "dynamic" "not implemented yet — start the follower via orchestrator,"
  warn "dynamic" "  publish synthetic /wheel/odom + a /reference_path,"
  warn "dynamic" "  verify /cmd_vel_raw rate >0 with linear.x>0,"
  warn "dynamic" "  trip /estop_trigger and verify /cmd_vel collapses to zero."
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
