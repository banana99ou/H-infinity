#!/bin/bash
# Installs limo-battle.service to /etc/systemd/system/, enables it on boot,
# and starts it now. Requires sudo.
#
# Usage on the NUC:
#   bash ~/H-infinity/tools/orchestrator/install_service.sh
#
# To uninstall:
#   sudo systemctl disable --now limo-battle
#   sudo rm /etc/systemd/system/limo-battle.service
#   sudo systemctl daemon-reload

set -e

SRC="$(dirname "$(readlink -f "$0")")/limo-battle.service"
DST=/etc/systemd/system/limo-battle.service

if [ ! -f "$SRC" ]; then
  echo "ERR: unit file not found: $SRC" >&2
  exit 1
fi

echo "[install] copying $SRC -> $DST"
sudo cp "$SRC" "$DST"

echo "[install] reloading systemd"
sudo systemctl daemon-reload

echo "[install] enabling on boot"
sudo systemctl enable limo-battle.service

echo "[install] (re)starting now"
sudo systemctl restart limo-battle.service

sleep 2
echo
echo "[install] status:"
sudo systemctl --no-pager --full status limo-battle.service | head -30
echo
echo "[install] live logs:  journalctl -u limo-battle -f"
echo "[install] stop now:   sudo systemctl stop limo-battle"
echo "[install] disable:    sudo systemctl disable --now limo-battle"
