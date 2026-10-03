#!/usr/bin/env bash
# Robot-side half of the relay deployer (tools/sync/relay_deploy.sh). The relay
# pipes its STAGED copy of this file over ssh instead of running the robot's own
# copy, so the very first deploy does not depend on what the robot already has:
#
#   ssh agilex@agilex-nuc12wski7 'bash -s' -- <cmd> [args] < robot_side.sh
#
#   busy          print "idle" or "busy: <why>" (exit 0 either way)
#   stamp         print the deployed stamp ("" if never deployed)
#   build VENDOR  colcon build limo_path_follower; VENDOR=1 first reinstalls
#                 vfg_pathfollowing (pip --user, NOT editable on the NUC —
#                 DOC/deployment.md gotcha 2 — so vendor edits need a reinstall)
#   restart       restart the limo-battle stack (rosbridge + orchestrator + procs)
#   set-stamp S   record S as the deployed stamp
#
# Do NOT 'set -u' — /opt/ros/humble/setup.bash references unset variables.

STAMP_FILE="$HOME/.hinf_deploy_stamp"
REPO="$HOME/H-infinity"

ros_env() {
  source /opt/ros/humble/setup.bash
  [ -f "$HOME/agilex_ws/install/setup.bash" ] && source "$HOME/agilex_ws/install/setup.bash"
}

case "$1" in
  busy)
    # Anything that drives the robot or records a leg means a run is live.
    for pat in 'ros2 bag record' 'path_follower_node' 'reposition_node'; do
      if pgrep -f "$pat" >/dev/null; then echo "busy: '$pat' running"; exit 0; fi
    done
    # Between legs none of the above may be alive while the batch still runs,
    # so ask run_executor (its /run/status is latched). An executor that is up
    # but whose status cannot be read counts as busy: never guess "idle".
    if pgrep -f run_executor_node >/dev/null; then
      ros_env
      st=$(timeout 20 ros2 topic echo --once --qos-durability transient_local \
             --qos-reliability reliable /run/status std_msgs/msg/String 2>/dev/null)
      phase=$(printf '%s' "$st" | grep -oE '"phase": *"[a-z_]+"' | head -1 | grep -oE '[a-z_]+"$' | tr -d '"')
      case "$phase" in
        idle|done|aborted) ;;
        "") echo "busy: run_executor is up but /run/status unreadable"; exit 0 ;;
        *)  echo "busy: run_executor phase=$phase"; exit 0 ;;
      esac
    fi
    echo idle
    ;;
  stamp)
    cat "$STAMP_FILE" 2>/dev/null
    ;;
  build)
    ros_env
    if [ "$2" = 1 ]; then
      # --no-build-isolation: build with the pinned setuptools 68.2.2 in ~/.local
      # (deployment.md gotcha 3) and without needing internet in the field.
      pip3 install --user --no-deps --force-reinstall --no-build-isolation \
        "$REPO/scalecar-vfg-h-infinite/" 2>&1 | tail -3
      [ "${PIPESTATUS[0]}" = 0 ] || { echo "vfg_pathfollowing reinstall FAILED"; exit 1; }
    fi
    cd "$HOME/agilex_ws" || exit 1
    colcon build --packages-select limo_path_follower 2>&1 | tail -15
    exit "${PIPESTATUS[0]}"
    ;;
  restart)
    pid=$(systemctl show -p MainPID --value limo-battle 2>/dev/null)
    if [ -z "$pid" ] || [ "$pid" = 0 ]; then
      echo "limo-battle not running — nothing to restart"; exit 0
    fi
    # No passwordless sudo on the NUC, so no `systemctl restart`. The unit runs
    # as agilex with Restart=on-failure (RestartSec=5): SIGKILL its main process
    # and systemd tears down the rest of the cgroup (KillMode=mixed) and starts
    # the whole stack again — the same state as after a boot.
    kill -KILL "$pid" || { echo "could not signal MainPID $pid"; exit 1; }
    for _ in $(seq 1 40); do
      sleep 1
      new=$(systemctl show -p MainPID --value limo-battle 2>/dev/null)
      if [ -n "$new" ] && [ "$new" != 0 ] && [ "$new" != "$pid" ]; then
        echo "limo-battle restarted (MainPID $pid -> $new)"; exit 0
      fi
    done
    echo "limo-battle did not come back within 40 s"; exit 1
    ;;
  set-stamp)
    printf '%s\n' "$2" > "$STAMP_FILE"
    ;;
  *)
    echo "usage: robot_side.sh {busy|stamp|build VENDOR|restart|set-stamp S}" >&2; exit 2
    ;;
esac
