# `tools/rtk_base/` — RTK basestation broadcaster

The piece of the RTK pipeline that previously lived on the operator's MacBook
and pinned them to the rooftop. It moves here, onto a small SBC at the base
tripod, so unattended operation actually works.

## Topology this is part of

```
[ base F9P + survey antenna + tripod ]      ← stationary, at one rooftop corner
            │ USB
            ▼
[ Raspberry Pi running rtcm_server.py ]      ← joins LIMO_AP as a WiFi client
            │ TCP :2101 over LIMO_AP
            ▼
[ NUC on the LIMO running GPS-RTK_ROS2_pub_node.py ]
            │ USB
            ▼
[ helical F9P rover ] → /gps_rtk_f9p_helical/* → ROS graph
```

Sibling docs:
- `DOC/network_topology.md` — operator-laptop + LIMO_AP + phone-tether layout.
- `DOC/system_spec.md` §3.3 (L1–L5) — what RTK is for in this project.
- `DOC/decisions/01_gps_no_fusion.md` — why RTK never enters the control loop.

## Why this exists (the "obvious problem")

`agile_ws/rtcm_server.py` defaulted to `/dev/tty.usbmodem101` — a macOS serial
path. The base F9P was plugged into the MacBook, the broadcaster ran on the
MacBook, and the rover NUC TCP-connected to the MacBook over LIMO_AP. Result:
**the experiment couldn't run unattended.** The moment the operator left the
roof, the MacBook left with them and the rover dropped to `quality=1`.

This directory is the fix. The broadcaster lives on a Pi that stays at the
base; the operator can take the MacBook inside.

## Hardware

Per side:

| At the base (rooftop, stationary) | At the rover (on the LIMO) |
|---|---|
| ZED-F9P module | helical ZED-F9P (already wired) |
| Survey/geodetic GNSS antenna | helical antenna (already mounted) |
| Tripod | — |
| Raspberry Pi (3B / 4 / 5 / Zero 2 W — anything that runs Raspberry Pi OS) | NUC (already there) |
| USB power bank — 10 000 mAh is enough for ~10 h of Pi 4 + F9P, 20 000 mAh for full peace of mind | LIMO battery |
| USB-A to whatever the F9P uses cable (USB-C / micro-USB) | — |
| Small enclosure for the Pi if outdoor weather is a concern | — |

A USB WiFi dongle with an external antenna is a useful contingency if the Pi
to LIMO_AP link is marginal at the far edge of the working area — but don't
buy one pre-emptively, test first.

> **First-time deploy?** Use the linear checklist runbook at
> [`DOC/rtk_base_deploy.md`](../../DOC/rtk_base_deploy.md). This README is
> the reference; the runbook is the day-of script.

## One-time setup, on the Pi

Assumes a fresh Raspberry Pi OS Bookworm install, `pi` user, the repo cloned
at `~/H-infinity/`.

```bash
# 1. Add LIMO_AP to the Pi's known WiFi networks (do this BEFORE going to the
#    rooftop — the Pi needs to be able to find it).
sudo nmcli connection add type wifi con-name LIMO_AP ifname wlan0 \
    ssid '<the-LIMO-AP-SSID>' \
    wifi-sec.key-mgmt wpa-psk \
    wifi-sec.psk '<the-LIMO-AP-password>' \
    connection.autoconnect yes \
    connection.autoconnect-priority 100

# 2. Install the broadcaster.
cd ~/H-infinity/tools/rtk_base
./install.sh

# 3. Log out and back in so dialout group membership takes effect for your
#    shell. (sudo doesn't need it; this is for manual testing only.)
exit
```

Plug the base F9P into the Pi, then verify:

```bash
ls -l /dev/f9p_base                  # symlink should exist
sudo systemctl start rtk-base
journalctl -u rtk-base -f            # should show [reader] ... and [status] alive=True
```

From the NUC (or any other machine on LIMO_AP):

```bash
nc <pi-ip> 2101 | xxd | head         # should see 0xD3 RTCM3 frames
```

## Configure the base F9P (one-time, with u-center)

The F9P needs to be told "you are a base, broadcast RTCM3." Do this once with
u-center on a laptop before you mount the F9P on the tripod. Save to flash.

Suggested config:

- **TMODE3 = Survey-In** (1). Min duration 60 s, position accuracy 5.0 m.
  - Each power-on, the F9P self-surveys until both bounds are met, then locks
    that position and starts broadcasting. Within-session RTK is FIXED-capable
    even though the absolute frame jumps a few cm to ~1 m each session.
  - To switch to a fixed surveyed position later (V1 in `DOC/system_spec.md`),
    re-program TMODE3 to "Fixed mode" with the surveyed lat/lon.
- **RTCM3 output on the USB port**, types 1005, 1077, 1087, 1097, 1127, 1230
  at 1 Hz. (Driver-side `GPS-RTK_ROS2_pub_node.py` is type-agnostic; this set
  is what the rover wants for full multi-constellation correction.)
- **NMEA output: optional**, at low rate. The broadcaster will log fix quality
  if NMEA is present — useful for confirming the base sees satellites — but
  the rover only consumes RTCM3.
- **Save to flash (CFG-CFG)**.

Reference: https://docs.holybro.com/gps-and-rtk-system/zed-f9p-h-rtk-series/portable-rtk-base-station-setup

## Field bring-up

```
1. Power the Pi (plug in the USB power bank).
2. Pi auto-joins LIMO_AP (provided the LIMO is up first — the Pi can't find
   the AP otherwise). systemd starts rtk-base.service automatically.
3. Wait for survey-in. Logs will show RTCM messages starting to flow once
   the F9P locks its base position. Expect ~1 min in good open sky.
4. From the battle station, start `base_gnss`. The NUC's
   GPS-RTK_ROS2_pub_node.py will connect to <pi-ip>:2101 and start forwarding
   RTCM3 into the rover. /gps_rtk_f9p_helical/gps/rtk_status should show
   quality=5 (FLOAT) within seconds, then quality=4 (FIXED) within a minute.
5. Press start in the battle station, walk away.
```

## Daily teardown

```
1. Stop the service to flush logs cleanly (optional):
     ssh pi@<pi-ip> sudo systemctl stop rtk-base
2. Unplug the Pi power.
3. Pack the tripod + F9P + Pi. Same kit every time.
```

## Known gaps + follow-ups

- **Rover-side `TCP_HOST` is still hardcoded to `10.42.0.170`** in
  `GPS-RTK_ROS2_pub_node.py:52`. Until that's a config flag (or the Pi gets a
  guaranteed static IP via DHCP reservation on LIMO_AP / mDNS), the Pi must
  end up at that address. Follow-up.
- **No connectivity alert.** If the Pi loses LIMO_AP mid-session, the rover
  will silently degrade. The follower keeps publishing (control runs on
  `/wheel/odom`, ADR-01), but the recorded run loses ground-truth quality.
  `M1`/`F2`/`F3` in the spec want this gated. Next step in the broadcaster:
  expose a `/rtk_base/health` topic (via something — a ROS bridge, an HTTP
  endpoint, an ntfy ping). For now, the journal is the source of truth.
- **Survey-in vs fixed-mode** is a future decision. Survey-in is the right
  default while the venue config is still in flux (per
  `feedback_rtk_and_venue` — corners are arbitrary map clicks anyway). When
  the venue stabilizes and we want WGS84 corners reproducible across
  sessions, re-program TMODE3 with a fixed surveyed position.
- **No auto-time-sync** between Pi and NUC for journal correlation. NTP on
  both + GNSS-disciplined Pi clock once it has a fix should be sub-100 ms.
  Same caveat as the FitTogether (see `DOC/network_topology.md` § Known gaps).
- **No physical weatherproofing yet.** A small ABS project box + cable
  glands; not in scope here.
