#!/usr/bin/env bash
# tools/diagnostics/odom_zero_check.sh
#
# Sanity-check the odom-zeroing chain before driving:
#
#   1) verify exactly ONE publisher on /wheel/odom_zeroed
#      (a duplicate -- e.g. an orphaned `ros2 run` from a previous validation
#      session -- gives the follower a flickering pose belief; on the
#      2026-05-27 B1 bring-up this caused an unexpected post-estop creep).
#   2) read raw /wheel/odom and /wheel/odom_zeroed and report the SE(2) gap
#      (which IS the latched origin -- the distance the robot would "lunge"
#      if the follower were ever started on un-zeroed odom).
#   3) optionally trigger /odom_zero/reset and confirm the zeroed pose
#      collapses to ~(0,0,0).
#
# Why this script, and not the /tmp one from 2026-05-27:
#   That script did `nohup ros2 run ... &; OZ_PID=$!; kill $OZ_PID`. `$!` is
#   the `ros2 run` WRAPPER pid; the actual node is a Python child with its
#   own pid, so the kill only tore down the wrapper and orphaned the node
#   to init -- a second publisher on /wheel/odom_zeroed survived past the
#   test. This script never spawns its own odom_zero_node (use the
#   orchestrator: `/orchestrator/start odom_zero`); it only inspects.
#
# Usage:
#   tools/diagnostics/odom_zero_check.sh             # inspect only
#   tools/diagnostics/odom_zero_check.sh --reset     # also publish reset True
#
# Run on the NUC after sourcing ROS:
#   source /opt/ros/humble/setup.bash
#   source /home/agilex/agilex_ws/install/setup.bash

set -eo pipefail

DO_RESET=0
for arg in "$@"; do
    case "$arg" in
        --reset) DO_RESET=1 ;;
        -h|--help)
            sed -n '2,30p' "$0"
            exit 0
            ;;
        *)
            echo "unknown arg: $arg" >&2
            exit 2
            ;;
    esac
done

# Tolerate the user not having sourced; emit a clear error.
if ! command -v ros2 >/dev/null 2>&1; then
    echo "error: ros2 not on PATH. Source /opt/ros/humble/setup.bash and the workspace first." >&2
    exit 2
fi

# Strip ANSI colour from ros2 output so grep/awk are robust.
strip_ansi() { sed -E 's/\x1B\[[0-9;]*[mGKHF]//g'; }

echo "== 1) publisher count on /wheel/odom_zeroed =="
PUB_LINE="$(ros2 topic info /wheel/odom_zeroed 2>&1 | strip_ansi | grep -E '^Publisher count:' || true)"
echo "    ${PUB_LINE:-<no info>}"
PUB_COUNT="$(echo "$PUB_LINE" | awk '{print $3}')"
if [ -z "$PUB_COUNT" ]; then
    echo "FAIL: no /wheel/odom_zeroed topic / could not read publisher count."
    echo "      Is odom_zero_node up? (orchestrator: /orchestrator/start odom_zero)"
    exit 1
fi
if [ "$PUB_COUNT" != "1" ]; then
    echo "FAIL: expected exactly 1 publisher on /wheel/odom_zeroed, got $PUB_COUNT."
    echo "      Common cause: an orphaned 'ros2 run odom_zero_node' from a"
    echo "      previous validation. Find + kill the real PIDs:"
    echo "          pgrep -af odom_zero_node"
    echo "          pkill -f odom_zero_node    # then start ONE via orchestrator"
    exit 1
fi
echo "    OK: single publisher."
echo

echo "== 2) raw vs zeroed pose (latched origin = the gap) =="
RAW="$(timeout 4 ros2 topic echo --once /wheel/odom --field pose.pose 2>/dev/null || true)"
ZER="$(timeout 4 ros2 topic echo --once /wheel/odom_zeroed --field pose.pose 2>/dev/null || true)"
if [ -z "$RAW" ]; then
    echo "FAIL: no /wheel/odom message in 4 s. Is the LIMO base running?"
    exit 1
fi
if [ -z "$ZER" ]; then
    echo "FAIL: no /wheel/odom_zeroed message in 4 s. odom_zero_node not republishing."
    exit 1
fi
echo "--- /wheel/odom (raw) ---"
echo "$RAW"
echo "--- /wheel/odom_zeroed ---"
echo "$ZER"
echo

if [ "$DO_RESET" = "1" ]; then
    echo "== 3) publishing /odom_zero/reset True =="
    ros2 topic pub --once /odom_zero/reset std_msgs/msg/Bool '{data: true}' >/dev/null
    sleep 1
    ZER2="$(timeout 4 ros2 topic echo --once /wheel/odom_zeroed --field pose.pose 2>/dev/null || true)"
    echo "--- /wheel/odom_zeroed after reset (expect ~(0,0,0)) ---"
    echo "$ZER2"
    echo
fi

echo "OK."
