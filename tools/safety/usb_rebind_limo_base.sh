#!/bin/bash
# Re-enumerate the LIMO chassis CP2102 WITHOUT a reboot: unbind + rebind its USB
# port via sysfs. Escalation tier for odom_watchdog.py when a plain driver
# respawn doesn't restore /wheel/odom (rare "stuck controller" case).
#
# Needs root to write /sys/bus/usb/drivers/usb/{un,}bind. For the unattended
# watchdog, grant NOPASSWD sudo for THIS script (visudo):
#   agilex ALL=(root) NOPASSWD: /home/agilex/H-infinity/tools/safety/usb_rebind_limo_base.sh
# The watchdog calls it directly; this script self-elevates via sudo -n.
#
# NOTE: the sysfs-path -> USB-port-id derivation below is best-effort and MUST be
# verified on the NUC (the exact devpath was .../usb1/1-7/1-7.4/1-7.4:1.0/ttyUSBx).
set -e

# Re-exec under sudo if not root (non-interactive; relies on NOPASSWD).
if [ "$(id -u)" -ne 0 ]; then
  exec sudo -n "$0" "$@"
fi

TTY="$(readlink -f /dev/limo_base 2>/dev/null)" || { echo "no /dev/limo_base"; exit 1; }
SYS="$(udevadm info -q path -n "$TTY" 2>/dev/null)" || { echo "no sysfs path for $TTY"; exit 1; }
# The USB *device* dir is the one holding the ":<config>.<intf>" interface; its
# id is the port (e.g. 1-7.4). Pull the last "<bus>-<ports>:<cfg>.<intf>" token.
IFACE="$(printf '%s\n' "$SYS" | grep -oE '[0-9]+-[0-9.]+:[0-9]+\.[0-9]+' | tail -1)"
PORT="${IFACE%%:*}"
[ -n "$PORT" ] || { echo "could not derive USB port from: $SYS"; exit 1; }

echo "rebinding USB port '$PORT' (chassis CP2102 on $TTY)"
echo "$PORT" > /sys/bus/usb/drivers/usb/unbind || { echo "unbind failed"; exit 1; }
sleep 1
echo "$PORT" > /sys/bus/usb/drivers/usb/bind || { echo "bind failed"; exit 1; }
sleep 1
echo "rebind done for $PORT"
