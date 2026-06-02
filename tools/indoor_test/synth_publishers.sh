#!/bin/bash
# Tier-0 synthetic publishers for indoor webui testing.
#
# Each subcommand publishes one topic the webui consumes. Run in the foreground,
# Ctrl-C to stop. Open as many terminals as you want to drive multiple topics in
# parallel (the freshness dots in the webui need >=1 message every few seconds
# to stay green).
#
# Run on the NUC after sourcing ROS (this script also auto-sources for
# convenience):
#
#   ./synth_publishers.sh battery 11.7
#   ./synth_publishers.sh battery 10.4
#   ./synth_publishers.sh estop on
#   ./synth_publishers.sh rtk_status fixed 0.5
#   ./synth_publishers.sh rtk_status stale 42
#
# Companion runbook: webui_smoke.md in this directory.

# Don't `set -u`: ROS setup.bash references unset vars.

source /opt/ros/humble/setup.bash 2>/dev/null
source /home/agilex/agilex_ws/install/setup.bash 2>/dev/null

CMD="$1"; shift || true

usage() {
  cat <<EOF
Usage: $0 <subcommand> [args]

  battery <volts>                  /limo_status with battery_voltage at 1 Hz
                                   (warn <= 10.8, halt <= 10.5)
  estop on|off                     /estop one-shot (latched on consumer side)
  estop_trigger on|off             /estop_trigger one-shot (the cmd side)
  odom_zeroed <x> <y> <yaw_rad>    /wheel/odom_zeroed at 10 Hz
  odom <x> <y> <yaw_rad>           raw /wheel/odom at 10 Hz (feeds odom_zero_node)
  reset_odom_zero                  /odom_zero/reset true (one-shot)
  rtk_status fixed [age_s]         RTK FIXED (quality=4) + RTCM OK at <age_s> (default 0.5)
  rtk_status float [age_s]         RTK FLOAT (quality=5) + RTCM OK
  rtk_status stale <age_s>         RTCM STALE at <age_s> (e.g. 42 for stale-socket repro)
  rtk_status nofix                 quality=0 (no fix at all)
  rtk_fix <lat> <lon>              /gps_rtk_f9p_helical/gps/fix at 1 Hz
  pixhawk_fix <lat> <lon>          /pixhawk/global_position/raw/fix at 1 Hz
  follower_status [v]              /path_follower/status (11 floats) at 10 Hz, v=0.5 default
  exp_status [phase]               /experiment/status with one cell at 0.5 Hz
                                   phase examples: preflight, repo_goto, follower, paused, done
  orch_status <names>              /orchestrator/status; names = comma list of alive nodes
                                   from {base_vanilla, base_gnss, estop, follower}
                                   e.g. orch_status base_vanilla,estop
  reference_path_demo              /reference_path with a short 5-pt straight line (latched)
  cmd_vel_raw_watch                Subscribe /cmd_vel_raw + /cmd_vel side by side
EOF
}

if [ -z "$CMD" ] || [ "$CMD" = "-h" ] || [ "$CMD" = "--help" ]; then
  usage; exit 0
fi

case "$CMD" in

  battery)
    V="${1:-11.7}"
    exec ros2 topic pub --rate 1 /limo_status limo_msgs/msg/LimoStatus \
      "{vehicle_state: 0, control_mode: 1, battery_voltage: ${V}, error_code: 0, motion_mode: 1}"
    ;;

  estop)
    case "$1" in
      on)  exec ros2 topic pub --once /estop std_msgs/msg/Bool '{data: true}'  ;;
      off) exec ros2 topic pub --once /estop std_msgs/msg/Bool '{data: false}' ;;
      *) echo "estop on|off"; exit 2 ;;
    esac
    ;;

  estop_trigger)
    case "$1" in
      on)  exec ros2 topic pub --once /estop_trigger std_msgs/msg/Bool '{data: true}'  ;;
      off) exec ros2 topic pub --once /estop_trigger std_msgs/msg/Bool '{data: false}' ;;
      *) echo "estop_trigger on|off"; exit 2 ;;
    esac
    ;;

  odom_zeroed|odom)
    X="${1:-0.0}"; Y="${2:-0.0}"; YAW="${3:-0.0}"
    TOPIC="/wheel/odom_zeroed"
    [ "$CMD" = "odom" ] && TOPIC="/wheel/odom"
    # quaternion from yaw (z-axis): w=cos(yaw/2), z=sin(yaw/2)
    QZ=$(python3 -c "import math; print(math.sin($YAW/2))")
    QW=$(python3 -c "import math; print(math.cos($YAW/2))")
    exec ros2 topic pub --rate 10 "$TOPIC" nav_msgs/msg/Odometry \
      "{pose: {pose: {position: {x: ${X}, y: ${Y}, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: ${QZ}, w: ${QW}}}}, twist: {twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}}"
    ;;

  reset_odom_zero)
    exec ros2 topic pub --once /odom_zero/reset std_msgs/msg/Bool '{data: true}'
    ;;

  rtk_status)
    SUB="$1"
    case "$SUB" in
      fixed) Q=4; STATE="OK"; AGE="${2:-0.5}"; LAT="37.61247000"; LON="126.99426183"; SATS=22; HDOP=0.6 ;;
      float) Q=5; STATE="OK"; AGE="${2:-0.5}"; LAT="37.61247000"; LON="126.99426183"; SATS=18; HDOP=0.8 ;;
      stale) Q=4; STATE="STALE"; AGE="${2:-42.0}"; LAT="37.61247000"; LON="126.99426183"; SATS=22; HDOP=0.6 ;;
      nofix) Q=0; STATE="STALE"; AGE="${2:-99.9}"; LAT="0.00000000"; LON="0.00000000"; SATS=0; HDOP=99.99 ;;
      *) echo "rtk_status fixed|float|stale|nofix [age_s]"; exit 2 ;;
    esac
    FIXDESC="NO FIX"
    [ "$Q" = "4" ] && FIXDESC="RTK FIXED"
    [ "$Q" = "5" ] && FIXDESC="RTK FLOAT"
    MSG="FIX: ${FIXDESC} (quality=${Q}, sats=${SATS}, HDOP=${HDOP}, rate=10.0Hz) | Lat=${LAT}, Lon=${LON} | RTCM: ${STATE} (bytes=12345, fwd_age=${AGE}s, net_age=${AGE}s)"
    exec ros2 topic pub --rate 5 /gps_rtk_f9p_helical/gps/rtk_status std_msgs/msg/String \
      "{data: '${MSG}'}"
    ;;

  rtk_fix)
    LAT="${1:-37.61247}"; LON="${2:-126.99426}"
    exec ros2 topic pub --rate 1 /gps_rtk_f9p_helical/gps/fix sensor_msgs/msg/NavSatFix \
      "{latitude: ${LAT}, longitude: ${LON}, altitude: 30.0}"
    ;;

  pixhawk_fix)
    LAT="${1:-37.61247}"; LON="${2:-126.99426}"
    exec ros2 topic pub --rate 1 /pixhawk/global_position/raw/fix sensor_msgs/msg/NavSatFix \
      "{latitude: ${LAT}, longitude: ${LON}, altitude: 30.0}"
    ;;

  follower_status)
    V="${1:-0.5}"
    # 11 floats: x, y, yaw, v, s_star, total_length, kappa, rho, e_psi, delta_cmd, has_path
    exec ros2 topic pub --rate 10 /path_follower/status std_msgs/msg/Float32MultiArray \
      "{data: [1.0, 0.5, 0.1, ${V}, 5.0, 10.0, 0.1, 0.5, 0.05, 0.05, 1.0]}"
    ;;

  exp_status)
    PHASE="${1:-driving}"
    JSON="{\"run_id\":\"smoke_indoor\",\"cell_index\":1,\"n_cells\":3,\"cell_id\":\"R0.7_step\",\"leg\":\"AtoB\",\"phase\":\"${PHASE}\",\"pass\":1,\"fail\":0,\"eta_s\":120,\"message\":\"synthetic ${PHASE}\"}"
    exec ros2 topic pub --rate 2 /experiment/status std_msgs/msg/String \
      "{data: '${JSON}'}"
    ;;

  orch_status)
    LIVE="${1:-base_vanilla,estop}"
    bv=false; bg=false; es=false; fl=false
    IFS=',' read -ra NAMES <<< "$LIVE"
    for n in "${NAMES[@]}"; do
      case "$n" in
        base_vanilla) bv=true ;;
        base_gnss)    bg=true ;;
        estop)        es=true ;;
        follower)     fl=true ;;
      esac
    done
    JSON="{\"base_vanilla\":${bv},\"base_gnss\":${bg},\"estop\":${es},\"follower\":${fl}}"
    exec ros2 topic pub --rate 2 /orchestrator/status std_msgs/msg/String \
      "{data: '${JSON}'}"
    ;;

  reference_path_demo)
    # 5-point straight line ahead of origin. Latched so the path persists.
    exec ros2 topic pub --qos-durability transient_local --qos-reliability reliable --once \
      /reference_path nav_msgs/msg/Path \
      "{header: {frame_id: 'odom'}, poses: [
         {header: {frame_id: 'odom'}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}},
         {header: {frame_id: 'odom'}, pose: {position: {x: 0.5, y: 0.0, z: 0.0}, orientation: {w: 1.0}}},
         {header: {frame_id: 'odom'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}},
         {header: {frame_id: 'odom'}, pose: {position: {x: 1.5, y: 0.0, z: 0.0}, orientation: {w: 1.0}}},
         {header: {frame_id: 'odom'}, pose: {position: {x: 2.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}
       ]}"
    ;;

  cmd_vel_raw_watch)
    # Two-pane watch: raw controller output vs. post-estop. Helpful for the
    # safety-gate test in Tier 1 (estop_cli should zero /cmd_vel when latched
    # while /cmd_vel_raw still carries the controller's wish).
    exec bash -c "ros2 topic echo --no-arr /cmd_vel_raw & ros2 topic echo --no-arr /cmd_vel & wait"
    ;;

  *)
    echo "unknown subcommand: $CMD"; usage; exit 2
    ;;
esac
