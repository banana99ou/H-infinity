# Field Network Topology

The setup that worked outdoors during GNSS recording, and the constraint
that forced it.

## The constraint

The **FitTogether OHCOACH Cell Y3** RTK GPS receiver requires a **specific
SSID + password** to be configured into its firmware. It connects to that
WiFi to fetch NTRIP corrections from the internet. We can't change the
device's stored SSID per-session and we can't put the receiver on the lab
WiFi (the lab doesn't reach the rooftop / outdoor track).

So the network has to come *to* the GPS, not the other way around.

## Topology that worked

```
       ┌──────────────── physically mounted on the LIMO ────────────────┐
       │                                                                │
       │   [ phone, hotspot ON ]                                        │
       │           │                                                    │
       │           │ WiFi (specific SSID/PW the FitTogether expects)    │
       │           ↓                                                    │
       │   [ FitTogether OHCOACH Cell Y3 ] ──── internal SD card        │
       │       (NTRIP RTK GPS, standalone blackbox)                     │
       │                                                                │
       │           │ USB tether (RNDIS / iPhone Personal Hotspot)       │
       │           ↓                                                    │
       │   [ LIMO NUC ]                                                 │
       │       │   - receives internet via USB tether                   │
       │       │   - serves its own AP (hostapd / NM hotspot) for the   │
       │       │     operator laptop                                    │
       │       │                                                        │
       └───────│────────────────────────────────────────────────────────┘
               │ WiFi (LIMO AP — short range, follows the robot)
               ↓
       [ MacBook (operator) ]
           - connects to LIMO AP
           - Tailscale + ws://agilex-nuc12wski7:9090 still works
             (tailscale tunnels over the AP / through the LIMO)
```

## Why each piece is the way it is

- **Phone is on the robot, not in the operator's pocket.** The FitTogether
  needs continuous proximity to its configured WiFi SSID (the phone
  hotspot). If the phone walks away with the operator, the GPS loses
  corrections and degrades from `RTK FIXED` → `FLOAT` → `SINGLE` within
  ~30 s.
- **USB tether (not WiFi) for LIMO ↔ phone.** Same phone is broadcasting
  the hotspot and serving as the LIMO's uplink — USB tether is a separate
  channel from the hotspot WiFi, so they don't fight. iPhone "Personal
  Hotspot" over Lightning works as RNDIS by default.
- **LIMO is the AP for the operator, not the phone.** Phone hotspot range
  is short and inconsistent. The LIMO's onboard AP gives the operator a
  reliable local link that follows the robot wherever it goes. Range is
  ~30-50 m, fine for outdoor tracks.
- **Tailscale still works over this stack.** The LIMO has internet via the
  phone, which means it can reach the Tailscale coordination server. The
  operator's laptop, while connected to the LIMO AP, can reach the LIMO
  directly by hostname (`agilex-nuc12wski7`) since they're on the same L2
  segment via the AP. Tailscale name resolution still works either way.

## Roles + ownership

| Device | Brings | Configured by |
|---|---|---|
| Phone | cellular uplink + specific SSID for the GPS | session operator |
| FitTogether OHCOACH Cell Y3 | NTRIP RTK fix, logs to its own SD | turn on, walk away |
| LIMO NUC | bridge (USB-tether → WiFi AP) + ROS stack | systemd / `start_battle.sh` |
| MacBook | battle station (browser) + run control | operator |

## Implications for the battle station

- The browser connects to `ws://agilex-nuc12wski7:9090` whether you're on
  Tailscale (lab) or LIMO AP (field). Same URL works in both contexts —
  no UI change between indoor and outdoor.
- WiFi loss of operator-LIMO link only happens if the operator wanders out
  of the LIMO's AP range (~30-50 m). For a rooftop track with the operator
  near it, this is rarely a problem.
- WiFi loss of LIMO-phone link only happens if the phone falls off, the
  USB cable disconnects, or cell signal dies. The first two are mechanical
  problems; the third degrades RTK quality but does not stop control
  (control is on `/wheel/odom`, see ADR-01).
- E-stop button in the battle station works as long as the operator has
  the AP. If the AP drops, the LIMO firmware watchdog stops the robot
  within ~0.5 s of `/cmd_vel` silence (LIMO base default).

## Operational checklist (outdoor)

Before driving to the test site:

1. Charge phone, LIMO, MacBook.
2. Tape phone to LIMO chassis. Plug USB cable from phone to NUC.
3. Power on FitTogether (waits for SSID/PW it knows).
4. Power on LIMO. Confirm via SSH (Tailscale): NUC has internet
   (`ping 8.8.8.8`), LIMO AP is up (`nmcli connection show --active`),
   `limo-battle.service` is running (`systemctl status limo-battle`).
5. Operator laptop: switch WiFi from lab to **LIMO AP**. Verify
   `ws://agilex-nuc12wski7:9090` reachable.

At the test site:

6. Confirm RTK fix quality from FitTogether's own indicators (check the
   device documentation; typically a colored LED).
7. From the battle station, start `base_gnss` (instead of `base_vanilla`).
   Once the NavSatFix and RTK-status topics flow, the battle station's
   GPS-quality bar (TODO — see ADR-01 §"What we DO use GPS for") shows
   live status.
8. Start a run. Note the wallclock time; this becomes part of the rosbag
   filename + run-ID, used to pair with the FitTogether SD-card export
   later.

## Automatic AP-on-tether (NetworkManager dispatcher)

The NUC switches between client WiFi and AP automatically based on whether
a USB tether is plugged in. This means the operator does not have to
remember to flip modes when going outdoors:

- **Plug iPhone (Personal Hotspot ON) into NUC USB** → dispatcher fires
  `up` for the new `enx<MAC>` interface → `nmcli connection up LIMO_AP`
  (NM auto-deactivates the WiFi-client connection on `wlo1` to free the
  radio).
- **Unplug** → dispatcher fires `down` → `nmcli connection down LIMO_AP`
  → NM autoconnects `wlo1` to the highest-priority saved WiFi in range
  (FMCL-5G in the lab, or nothing outdoors).

Files:

- Source: `tools/network/90-limo-ap-on-tether` (in this repo).
- Installer: `tools/network/install_tether_dispatcher.sh` (run on NUC; sudo
  password `agx`).
- Installed at: `/etc/NetworkManager/dispatcher.d/90-limo-ap-on-tether`.
- Log: `/var/log/limo-net-switch.log`.

Caveats:

- Triggers on **any** USB ethernet (`enx*` or `usb[0-9]*`). Onboard wired
  NIC `enp114s0` does not match. A USB-Ethernet dongle plugged in for
  unrelated reasons would also flip the NUC into AP — be aware.
- If the AP comes up but the NUC has no working internet uplink, Tailscale
  loses connectivity. This is the normal field state (uplink comes via the
  tether). Don't activate `LIMO_AP` manually unless you intend it.

## Known gaps

- The **LIMO AP configuration is not in this repo**. It lives in the
  NUC's NetworkManager state. If the NUC is reimaged, the AP must be
  reconstructed by hand (or scripted; not done yet).
- We have **no automated check** that the FitTogether SD-card recording
  actually started. The device is opaque; we trust its LED. If the SD
  card is full or unmounted, the run still proceeds but no NTRIP data
  is captured. Open question for next iteration: add a pre-flight prompt
  in the battle station that the operator must confirm.
- **No automated time-sync** between FitTogether and the NUC. The
  FitTogether records its own timestamps; the rosbag records NUC's. They
  must be aligned offline. NTP on the NUC + FitTogether's GNSS-disciplined
  clock should give sub-100ms agreement, but this hasn't been validated.
