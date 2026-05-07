#!/bin/bash
# Install the NetworkManager dispatcher that switches LIMO between WiFi
# client and AP mode based on USB tether presence.
#
# Run on the NUC. Will prompt for sudo (password: agx).

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
SRC="$SCRIPT_DIR/90-limo-ap-on-tether"
DST=/etc/NetworkManager/dispatcher.d/90-limo-ap-on-tether
LOG=/var/log/limo-net-switch.log

if [ ! -f "$SRC" ]; then
  echo "ERROR: $SRC not found" >&2
  exit 1
fi

echo "Installing $SRC -> $DST"
sudo install -m 0755 -o root -g root "$SRC" "$DST"

echo "Ensuring log file exists at $LOG"
sudo touch "$LOG"
sudo chmod 0644 "$LOG"

echo
echo "Installed. Test by:"
echo "  1. Plug iPhone (Personal Hotspot ON) into NUC USB."
echo "     -> wlo1 should switch from FMCL-5G to LIMO_AP."
echo "  2. Unplug. -> NM should autoconnect back to a saved WiFi."
echo
echo "Watch the log:"
echo "  sudo tail -f $LOG"
echo
echo "Watch live nmcli state:"
echo "  watch -n1 nmcli device status"
