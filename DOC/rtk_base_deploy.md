# RTK basestation — first-deployment runbook

Day-of checklist for standing up the Pi-based RTK basestation for the first
time. Written **2026-05-26** for the next-session deploy; reusable as a
generic first-deploy SOP for future Pis.

Pairs with:
- [tools/rtk_base/README.md](../tools/rtk_base/README.md) — the reference
  (topology, hardware, "why"). This document is the linear checklist; the
  README is what you look at when something is unclear.
- [ToDo.md](../ToDo.md) item 3 — the backlog entry this runbook executes.

**End-of-day target:** `quality=4` sustained on the bench for 30+ min, then a
4 h+ soak running unattended overnight. **Stretch:** repeat on the rooftop.

## Pre-trip (before leaving for the lab)

Pack:
- [ ] Pi (any model that runs Raspberry Pi OS Bookworm) + microSD (8 GB+) +
      Pi power supply
- [ ] **USB power bank** (10 000+ mAh) for the soak test
- [ ] Base F9P + USB-to-Pi cable
- [ ] Base antenna + SMA cable
- [ ] Tripod
- [ ] Ethernet cable for initial Pi setup (or preconfigure WiFi in the imager
      and skip the cable)

Confirm:
- [ ] LIMO_AP SSID + password (look up on the NUC:
      `nmcli connection show LIMO_AP | grep -E "ssid|psk"` — `psk` only shows
      if you run as the right user; otherwise read `/etc/NetworkManager/system-connections/LIMO_AP.nmconnection`)
- [ ] u-center installed on the MacBook for the F9P config step
- [ ] Raspberry Pi Imager installed on the MacBook (if the microSD isn't
      flashed yet)

## Phase 1 — Pi boots (10–15 min)

If flashing now, in Raspberry Pi Imager pick **Raspberry Pi OS Lite (64-bit)**
and use the imager's "Edit settings" panel:
- hostname: `rtk-base`
- user: `pi` + password
- enable SSH
- optionally preconfigure your lab WiFi (saves the ethernet cable in step 1.1)

1.1. Insert microSD, ethernet (or trust preconfigured WiFi), power on the Pi.

1.2. SSH in:
```
ssh pi@rtk-base.local       # mDNS, usually works on macOS
# fallback: nmap -p 22 192.168.1.0/24  to find the IP
```

1.3. Confirm internet: `ping -c 2 8.8.8.8`.

**Go/no-go:** SSH'd in, internet works.

## Phase 2 — Pi joins LIMO_AP with static IP `.170` (10 min)

The rover hardcodes `TCP_HOST = 10.42.0.170` at
[GPS-RTK_ROS2_pub_node.py:52](../GPS-RTK_ROS2_pub_node.py). The Pi must
land at that address.

2.1. On the Pi (substitute the real SSID and password):
```
sudo nmcli connection add type wifi con-name LIMO_AP ifname wlan0 \
    ssid '<LIMO_AP_SSID>' \
    wifi-sec.key-mgmt wpa-psk \
    wifi-sec.psk '<LIMO_AP_PASSWORD>' \
    ipv4.method manual \
    ipv4.addresses 10.42.0.170/24 \
    ipv4.gateway 10.42.0.1 \
    ipv4.dns 10.42.0.1 \
    connection.autoconnect yes \
    connection.autoconnect-priority 100
```

2.2. Power the LIMO on so LIMO_AP is actually broadcasting. The NUC
dispatcher only brings LIMO_AP up when the phone tether is plugged in
(see [DOC/network_topology.md](network_topology.md) "AP-on-tether") — so
plug the phone tether into the NUC first, or set `LIMO_AP` to autoconnect
on the NUC for this test.

2.3. The Pi should auto-join within ~10 s:
```
nmcli connection show --active
ip addr show wlan0 | grep "10.42.0.170"
ping -c 2 10.42.0.1
```

**Go/no-go:** Pi shows `10.42.0.170` on `wlan0`, pings the NUC. If not:
SSID/PW wrong, or someone else on LIMO_AP grabbed `.170` (rare — DHCP
shouldn't hand out a static-claimed address, but check `arp -a` on the
NUC if it happens).

## Phase 3 — Install rtk-base service (5 min)

3.1. Clone the repo on the Pi:
```
git clone <your-repo-url> ~/H-infinity
cd ~/H-infinity/tools/rtk_base
```

3.2. Run the installer:
```
./install.sh
```

It installs `python3-serial` via apt, adds `pi` to `dialout`, installs the
udev rule, installs and enables the systemd unit. Idempotent — safe to
re-run after a `git pull`.

3.3. Log out + back in so the new `dialout` group membership takes effect
for your shell (sudo doesn't need it; this is for manual testing only):
```
exit
ssh pi@10.42.0.170
groups | grep dialout       # confirm
```

**Go/no-go:** `systemctl status rtk-base.service` shows
`loaded; enabled; vendor preset: enabled`. It will **not** be `active` yet
because the F9P isn't plugged in — that's correct.

## Phase 4 — Configure base F9P in u-center (one-time, 15 min)

Do this on the **MacBook** for the first-time config. The Pi doesn't need
u-center; once the F9P's settings are saved to flash, they survive every
future power-on.

4.1. Plug the F9P into the MacBook. u-center → Receiver → Connection →
pick the USB device, baud 38400. You should see NMEA/UBX traffic in the
packet console.

4.2. **TMODE3 → Survey-In** (View → Configuration View → TMODE3):
- Mode: **Survey-in**
- Minimum observation time: **60 s**
- Required position accuracy: **5.000 m**
- Send.

(Coarse Survey-In is fine — venue corners are arbitrary map clicks
anyway per [feedback_rtk_and_venue](../../.claude/projects/-Users-hyeon-yongjeong-code-H-infinity/memory/feedback_rtk_and_venue.md).
Tighten to Fixed-mode later when V1 reproducibility matters.)

4.3. **Enable RTCM3 output on the USB port** (Configuration View → MSG, one
at a time; for each, click Send after setting USB rate = 1):
- `F5-05` (RTCM3 1005 — stationary RTK reference station ARP)
- `F5-4D` (1077 — GPS MSM7)
- `F5-57` (1087 — GLONASS MSM7)
- `F5-61` (1097 — Galileo MSM7)
- `F5-7F` (1127 — BeiDou MSM7)
- `F5-E6` (1230 — GLONASS code-phase biases)

4.4. **Save to flash** (Configuration View → CFG):
- Action: **Save current configuration**
- Devices: tick **BBR** and **Flash** (and **I2C-EEPROM** if available)
- Send. Then **power-cycle the F9P** (unplug + replug USB) and verify
  TMODE3 still reads Survey-In after the power cycle.

**Go/no-go:** TMODE3 reads Survey-In after a power cycle; RTCM3 messages
visible in u-center's packet console while connected outdoors / near a
window with sky.

## Phase 5 — End-to-end bench test (20 min)

5.1. Plug the base F9P into the **Pi** now (move it from the MacBook).
Connect the antenna; put the antenna near a window with the best sky view
you can get indoors.

5.2. Verify udev:
```
ls -l /dev/f9p_base       # symlink → ttyACM0 (or similar)
```

If missing: `udevadm trigger --subsystem-match=tty` and check
`udevadm info /dev/ttyACM0 | grep f9p_base`.

5.3. Start the service:
```
sudo systemctl start rtk-base
journalctl -u rtk-base -f
```

Expected within 10 s:
```
[serial] opened.
[reader] starting
[status] alive=True clients=0 sats(view/used)=... fix=... rtcm=no
```

5.4. Wait for survey-in to finish. Outdoors this is 1–5 min; near a window
it might take 10+ min or never complete (the F9P needs ≥4 satellites with
decent SNR). Once it does, the status log line switches to
`rtcm=yes types=[1005, 1077, ...]`.

5.5. On the NUC (separate SSH session — `ssh agilex@agilex-nuc12wski7`):
```
source /opt/ros/humble/setup.bash
source /home/agilex/agilex_ws/install/setup.bash
ros2 launch limo_base LIMO+MAVROS+RTK_Node_Launcher.launch.py
```

Watch in another NUC shell:
```
ros2 topic echo /gps_rtk_f9p_helical/gps/rtk_status
```

Expected progression: `quality=1` (no fix on rover yet) → `quality=5` (RTK
FLOAT, within ~10 s of corrections flowing) → `quality=4` (RTK FIXED,
within 1–3 min in decent sky).

**Go/no-go:** `quality=4` sustained for 5 minutes on the bench. If stuck
below 4: not enough sky for either base or rover, or corrections aren't
reaching the rover (check the `GPS-RTK_ROS2_pub_node` log for "RTCM:
ACTIVE" and a growing `bytes=` counter).

## Phase 6 — Provoke faults (10 min)

Validate resilience before trusting the unattended soak. Each fault should
recover **without manual intervention**.

6.1. **Yank the F9P USB, wait 10 s, replug.**
- Pi log: `[serial] read error ... reopening` then `[serial] opened`.
- NUC: `quality` drops below 4 briefly, returns to 4 within ~30 s.

6.2. **Yank the Pi off LIMO_AP, wait 10 s, rejoin.**
```
sudo nmcli connection down LIMO_AP && sleep 10 && sudo nmcli connection up LIMO_AP
```
- Rover-side log: `Connection error: ... Retrying in 5s` then `Connected.`
- NUC: `quality` drops, returns to 4 within ~30 s.

6.3. **Reboot the Pi:** `sudo reboot`.
- systemd brings `rtk-base.service` back unattended after the Pi finishes
  booting (~30 s on a Pi 4, longer on a Zero).
- NUC: `quality` returns to 4 without you touching the NUC.

**Go/no-go:** all three recover unattended within ~60 s.

## Phase 7 — Bench soak (set and forget, overnight ideal)

7.1. Move the Pi to the USB power bank (unplug the wall PSU). This also
validates the power bank itself. LIMO stays on.

7.2. Start journaling and tee to a log file you can review in the morning:
```
ssh pi@10.42.0.170 'journalctl -u rtk-base -f' | tee ~/soak-$(date +%Y%m%d-%H%M).log
```

7.3. Walk away.

**Go/no-go (morning check):**
- No unexplained restart events in the journal across 4+ hours
- `quality=4` sustained on the NUC for the same window (run a separate
  `ros2 topic echo /gps_rtk_f9p_helical/gps/rtk_status > rover-soak.log`
  on the NUC to capture)
- Power bank still ≥ 30% (sanity-check the 10 h estimate in the README)
- Pi `top` snapshot vs t=0 snapshot — RAM/CPU not creeping

## Stretch: rooftop soak (if everything above went green)

If the bench soak is clean, take the kit to the rooftop the same day for
the 2 h rooftop soak from [ToDo.md](../ToDo.md) item 3:
- Mount the antenna on the tripod, somewhere with full sky.
- Repeat phase 5 outdoors. Survey-in completes much faster (~1 min).
- Drive the LIMO manually through the corners of the intended working
  area; watch `quality=4` stay sustained as the robot moves to the far
  edges. Note any geometric dropouts (WiFi-range marginality).

If `quality=4` drops at the far edge of the working area: that's the
WiFi-range case the README flags. The fallback is a USB WiFi dongle with
an external antenna on the Pi — buy when needed, not pre-emptively.

## If you get stuck

| Symptom | First place to look |
|---|---|
| Pi can't see LIMO_AP | SSID/PW wrong; or LIMO_AP not actually up (NUC dispatcher needs a USB tether before it brings LIMO_AP up) |
| Pi gets a different IP than `.170` | Another device claimed it. `arp -a` on the NUC. Power-cycle the offender or reset its DHCP lease. |
| `/dev/f9p_base` missing | udev rule didn't fire. `udevadm test /sys/class/tty/ttyACM0` and look for the SYMLINK line. Confirm vendor:product = `1546:01a9` (`lsusb`). |
| `rtcm=no` in the Pi journal | Base survey-in not complete (poor sky), or the F9P config got lost (re-do Phase 4 and verify save-to-flash). |
| `quality=1` on the rover forever | Pi journal shows `rtcm=yes` but rover isn't reaching the Pi: check rover-side `RTCM: STALE` (`netstat -an \| grep 2101` on the Pi to see if the NUC even connected). |
| Power bank dies mid-soak | Spec-check the bank (10 000 mAh should give 10 h on Pi 4 + F9P; if it dies in 3 h, the bank is the problem, not the broadcaster). |

## What this runbook does *not* cover

- **Rover-side `TCP_HOST` becoming a CLI flag.** Still hardcoded. Workaround
  here is the Pi's static `.170`. Follow-up engineering, separate session.
- **`/rtk_base/health` topic or ntfy push.** Out of scope for first deploy;
  needed before truly-unattended matrix runs (M1/F2 in
  [DOC/system_spec.md](system_spec.md)).
- **Fixed-mode TMODE3 with a one-time survey.** Optional upgrade once the
  venue stabilizes — see the README for the rationale.
- **Weatherproof enclosure for the Pi.** Bring it inside if there's rain;
  cardboard + a plastic bag is acceptable for a first deploy.
