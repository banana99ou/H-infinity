# RTK basestation — Pi broadcaster deployment runbook

Day-of checklist for **putting the existing basestation (Pi + already-
configured base F9P) onto LIMO_AP and into the rover's RTCM pipeline** —
i.e., replacing the MacBook role for unattended operation. Written
**2026-05-26** for the next-session deploy.

What this runbook **assumes is already true**:
- The base F9P has RTCM3 output saved to its flash from prior basestation use.
  The Pi broadcaster forces TMODE3 Survey-In on every service start, so boot is
  a new survey and stale fixed/survey coordinates are not trusted.
- The Pi already has Raspberry Pi OS Bookworm. Phase 1 is just "power on
  and SSH" — no imager step.

Pairs with:
- [tools/rtk_base/README.md](../tools/rtk_base/README.md) — the reference
  (topology, hardware, "why"). This document is the linear checklist; the
  README is what you look at when something is unclear or you need to
  re-configure the F9P from scratch.
- [ToDo.md](../ToDo.md) item 3 — the backlog entry this runbook executes.

**End-of-day target:** `quality=4` sustained on the bench for 30+ min, then a
4 h+ soak running unattended overnight. **Stretch:** repeat on the rooftop.

## Phase 0 — Tonight at home (preload, 20 min)

The Pi already has Pi OS Bookworm onboard and the base F9P is already
configured as a basestation. Tonight is about preloading the two things you
*won't* have at school: a known WiFi to bring the Pi online, and Tailscale
so you can SSH from your MacBook without sharing a network.

0.1. **Power the Pi at home**, SSH in via whatever local method works
(ethernet, mDNS, or the home WiFi if it's already in the Pi's profiles).

0.2. **Preload the school WiFi profile** with autoconnect:
```
sudo nmcli connection add type wifi con-name school ifname wlan0 \
    ssid '<SCHOOL_SSID>' \
    wifi-sec.key-mgmt wpa-psk \
    wifi-sec.psk '<SCHOOL_PASSWORD>' \
    connection.autoconnect yes \
    connection.autoconnect-priority 50
```
(Lower priority than LIMO_AP — see Phase 2 — so the Pi prefers LIMO_AP when
both are available at the lab.)

0.3. **Install Tailscale on the Pi:**
```
curl -fsSL https://tailscale.com/install.sh | sh
sudo tailscale up                     # prints a URL; auth with your tailnet
tailscale ip -4                       # note the Tailscale IP
```

0.4. **Confirm from the MacBook** (while still at home):
```
ssh pi@<pi-tailscale-hostname>        # or @<tailscale-ip>
```

If that works, the Pi is reachable from anywhere as long as it has any
working internet — school WiFi, LIMO_AP-with-NUC-tether, anywhere.

0.5. **Pack:**
- [ ] Pi (already has PIOS) + Pi power supply
- [ ] **USB power bank** (10 000+ mAh) for the soak test
- [ ] Base F9P (already configured) + USB-to-Pi cable
- [ ] Base antenna + SMA cable
- [ ] Tripod
- [ ] Ethernet cable as a last-resort fallback if both school WiFi and
      Tailscale somehow fail (rare; school WiFi usually just works)

0.6. **Confirm — write down or screenshot now**, you'll want them tomorrow:
- [ ] LIMO_AP SSID + password (look up on the NUC:
      `sudo cat /etc/NetworkManager/system-connections/LIMO_AP.nmconnection`)
- [ ] The Pi's Tailscale hostname / IP

**Go/no-go for tonight:** MacBook SSHs into the Pi over Tailscale from any
network. Power-cycle the Pi and confirm school WiFi profile + Tailscale
both auto-come-up.

## Phase 1 — Pi online at the lab (5 min)

1.1. Power on the Pi. It autoconnects to school WiFi (Phase 0.2) and
Tailscale comes up automatically.

1.2. SSH from the MacBook over Tailscale:
```
ssh pi@<pi-tailscale-hostname>
```

1.3. Confirm: `ping -c 2 8.8.8.8`.

**Go/no-go:** SSH'd in. (You haven't touched LIMO_AP yet — that's Phase 2.)

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

## Phase 4 — Sanity-check the base F9P (5 min)

The F9P is already configured as a basestation from prior use. This phase
is a quick "did anything get reset" check — not a re-config.

4.1. Plug the F9P into the Pi over USB. Verify the udev symlink came up
(`ls -l /dev/f9p_base` — see Phase 5 for the full check).

4.2. Quickest sanity check **without u-center**: start the broadcaster
and look at the journal:
```
sudo systemctl start rtk-base
journalctl -u rtk-base -f
```

If the stored config is intact, within ~10 s of survey-in completing
you'll see lines like:
```
[status] alive=True clients=0 ... rtcm=yes types=[1005, 1077, 1087, 1097, 1127, 1230]
```

The presence of `rtcm=yes` with that type set is proof TMODE3 + RTCM3
output are still configured. No u-center needed.

4.3. **If `rtcm=no` persists past survey-in time** (~10 min), the F9P's
stored config has been lost (rare — maybe long power outage, maybe someone
else used it). Plug it into the MacBook and re-run the u-center steps
documented in [tools/rtk_base/README.md](../tools/rtk_base/README.md)
under "Configure the base F9P".

**Go/no-go:** `rtcm=yes` with all six message types in the Pi journal.

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
| `rtcm=no` in the Pi journal | Base survey-in not complete (poor sky), or the F9P stored config has been wiped — re-config via u-center using the steps in [tools/rtk_base/README.md](../tools/rtk_base/README.md) "Configure the base F9P". |
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
