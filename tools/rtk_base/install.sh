#!/usr/bin/env bash
# Install the rtk-base broadcaster on a Raspberry Pi (or any Debian-family SBC).
#
# Idempotent. Safe to re-run after pulling an update to this directory.
#
# Assumes:
#   - The repo is cloned at $HOME/H-infinity on the Pi (matches the service unit).
#   - You're running this as the `pi` user (or whichever user owns the repo).
#   - You have sudo. No password is captured here — sudo will prompt.

set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SERVICE_NAME="rtk-base.service"
UDEV_RULE_NAME="99-f9p-base.rules"

echo "[install] from: $HERE"

# 1. Python deps. python3-serial from apt is fine on Raspberry Pi OS; avoids
#    needing pip + breaking PEP 668 externally-managed-environment errors.
if ! python3 -c "import serial" >/dev/null 2>&1; then
    echo "[install] installing python3-serial via apt"
    sudo apt-get update -qq
    sudo apt-get install -y python3-serial
else
    echo "[install] python3-serial already present"
fi

# 2. dialout group — required to open /dev/ttyACM* and the udev-created symlink.
WHO="$(id -un)"
if id -nG "$WHO" | tr ' ' '\n' | grep -qx dialout; then
    echo "[install] $WHO already in dialout group"
else
    echo "[install] adding $WHO to dialout group (you'll need to log out + back in for this shell to see it)"
    sudo usermod -aG dialout "$WHO"
fi

# 3. udev rule for /dev/f9p_base.
echo "[install] installing udev rule"
sudo cp "$HERE/$UDEV_RULE_NAME" /etc/udev/rules.d/
sudo udevadm control --reload
sudo udevadm trigger --subsystem-match=tty

# 4. systemd unit.
echo "[install] installing $SERVICE_NAME"
sudo cp "$HERE/$SERVICE_NAME" /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable "$SERVICE_NAME"

# 5. Sanity report.
echo
echo "[install] done."
echo
echo "  Plug in the base F9P, then verify the symlink:"
echo "    ls -l /dev/f9p_base"
echo
echo "  Start the service:"
echo "    sudo systemctl start $SERVICE_NAME"
echo
echo "  Follow the journal:"
echo "    journalctl -u $SERVICE_NAME -f"
echo
echo "  Test client (from another machine on the same network):"
echo "    nc <pi-ip> 2101 | xxd | head"
echo
echo "  If you just want to smoke-test without the F9P plugged in:"
echo "    sudo systemctl stop $SERVICE_NAME"
echo "    python3 $HERE/rtcm_server.py --demo"
