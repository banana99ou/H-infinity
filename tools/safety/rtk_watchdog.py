#!/usr/bin/env python3
"""RTK-health watchdog — page the operator (and pause the batch) when the fix
stops being trustworthy, instead of letting the stack drive on garbage.

Why this node exists (field 2026-09-16)
---------------------------------------
A whole field day produced 0 arrivals / 27 reposition aborts and nobody was
told anything was wrong. The RTK link had degraded from >=95% FIXED (every
session that ever produced data) to ~70%, with one 16-minute stretch at 0%:

    06-16 17:19  100.0% fixed   -> 20 arrivals /  3 aborts
    06-25 17:37   99.3% fixed   -> 19 arrivals /  9 aborts
    09-16 19:47   69.4% fixed   ->  0 arrivals / 27 aborts

Nothing warned, because every consumer degrades QUIETLY: heading_node drops the
COG anchor on a bad fix (so the heading EKF silently falls back to raw gyro and
drifts), and reposition then aborts with a geometry message that blames the
path. The operator sees "look-ahead target 108 deg off the nose" and goes
hunting for a path/compass bug that does not exist.

What it watches
---------------
1. FIX QUALITY   fraction of the window with quality in {4, 5}. Every session
                 that produced usable data ran >= 95%.
2. CORRECTIONS   RTCM fwd_age from the driver's status string. 'inf' means the
                 forwarder is delivering nothing at all (base/link down).
3. SKY           sats + HDOP. On 2026-09-16 the bad stretches read sats=6.3,
                 HDOP=36.7 against sats=12.0, HDOP=0.65 when healthy.
4. JITTER        the invariant that caught it: sum the fix-to-fix hops and
                 compare with the net displacement AND with wheel odometry.
                 A stationary robot logged a 10.54 m fix-to-fix path with a
                 0.09 m net displacement -- three independent measurements that
                 cannot disagree if the fix is real. 117x apart, silently.

What it does
------------
  OK      -> publish status, nothing else.
  WARN    -> survivable degradation: page ONCE per crossing, keep driving.
  SEVERE  -> unusable: page, and pause the run_executor batch via /run/cmd.
             When health comes back and holds for --resume-good-s, resume
             automatically -- but ONLY a pause this node caused (the resume
             carries if_reason_prefix so an operator pause is never overridden).

Publishes /rtk_watchdog/status (JSON) for the WebUI. Never publishes cmd_vel*.

  python3 rtk_watchdog.py [--window 30] [--warn-fixed-frac 0.95]
                          [--severe-fixed-frac 0.50] [--no-pause]
"""
import argparse
import json
import math
import os
import re
import sys
import time
from collections import deque

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

# Operator paging (Discord webhook from discord.env / ntfy), imported defensively
# so a missing module just disables paging (logs only). Same idiom as
# odom_watchdog.py: notify_discord resolves discord.env itself and never raises.
try:
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "notify"))
    from ntfy import notify_discord as _notify_discord  # type: ignore
except Exception:
    def _notify_discord(*_a, **_k):
        return False

# The driver publishes one human-readable line on gps/rtk_status, e.g.
#   FIX: RTK FIXED (quality=4, sats=12, HDOP=0.64, rate=8.0Hz) | Lat=..., Lon=...
#   | RTCM: ACTIVE (bytes=105524, fwd_age=0.8s, net_age=0.8s)
# Parse defensively: any field may be 'N/A' on a cold driver.
_RE_QUALITY = re.compile(r"quality=(\d+)")
_RE_SATS = re.compile(r"sats=(\d+)")
_RE_HDOP = re.compile(r"HDOP=([\d.]+)")
_RE_FWD_AGE = re.compile(r"fwd_age=([\d.]+|inf)s")

_GOOD_QUALITIES = (4, 5)          # RTK FIXED / FLOAT: the only usable ones
_EARTH_R = 6378137.0


def _geo_dist(lat1, lon1, lat2, lon2):
    """Flat-earth metres between two WGS84 points (fine over a rooftop)."""
    dlat = math.radians(lat2 - lat1) * _EARTH_R
    dlon = math.radians(lon2 - lon1) * _EARTH_R * math.cos(math.radians(lat1))
    return math.hypot(dlat, dlon)


class RtkWatchdog(Node):
    def __init__(self, args):
        super().__init__("rtk_watchdog")
        self._win = args.window
        self._warn_frac = args.warn_fixed_frac
        self._severe_frac = args.severe_fixed_frac
        self._warn_hdop = args.warn_hdop
        self._warn_sats = args.warn_sats
        self._warn_rtcm_s = args.warn_rtcm_age
        self._severe_rtcm_s = args.severe_rtcm_age
        self._severe_nofix_s = args.severe_nofix_s
        self._jitter_ratio = args.jitter_ratio
        self._jitter_floor = args.jitter_floor
        self._resume_good_s = args.resume_good_s
        self._may_pause = args.pause
        self._min_samples = args.min_samples

        # Rolling windows, all (t, value). Trimmed to --window on every tick.
        self._status = deque()        # (t, quality, sats, hdop, fwd_age)
        self._fixes = deque()         # (t, lat, lon)
        self._odom = deque()          # (t, vx)
        self._last_good_fix_t = None  # last time quality was in _GOOD_QUALITIES

        self._state = "OK"            # OK | WARN | SEVERE
        self._paged_state = "OK"      # worst state already paged this episode
        self._paused_by_us = False
        self._ok_since = None         # continuous-OK anchor for auto-resume

        self.pub_status = self.create_publisher(String, "/rtk_watchdog/status", 10)
        self.pub_cmd = self.create_publisher(String, "/run/cmd", 10)
        self.create_subscription(String, args.status_topic, self._on_status, 10)
        self.create_subscription(NavSatFix, args.fix_topic, self._on_fix, 10)
        self.create_subscription(Odometry, args.odom_topic, self._on_odom, 10)
        self.create_timer(1.0, self._tick)
        self.get_logger().warn(
            f"RTK WATCHDOG armed (window {self._win:.0f}s; warn below "
            f"{100 * self._warn_frac:.0f}% fixed, SEVERE below "
            f"{100 * self._severe_frac:.0f}%; "
            f"auto-pause {'ON' if self._may_pause else 'OFF'}).")

    # -- inputs --------------------------------------------------------
    def _on_status(self, msg):
        s = msg.data or ""
        m = _RE_QUALITY.search(s)
        if not m:
            return
        q = int(m.group(1))
        sats = int(_RE_SATS.search(s).group(1)) if _RE_SATS.search(s) else None
        hdop = float(_RE_HDOP.search(s).group(1)) if _RE_HDOP.search(s) else None
        age_m = _RE_FWD_AGE.search(s)
        age = None
        if age_m:
            age = float("inf") if age_m.group(1) == "inf" else float(age_m.group(1))
        now = time.time()
        self._status.append((now, q, sats, hdop, age))
        if q in _GOOD_QUALITIES:
            self._last_good_fix_t = now

    def _on_fix(self, msg):
        if msg.status.status < 0:
            return
        lat, lon = float(msg.latitude), float(msg.longitude)
        if lat != lat or lon != lon or (lat == 0.0 and lon == 0.0):
            return
        self._fixes.append((time.time(), lat, lon))

    def _on_odom(self, msg):
        self._odom.append((time.time(), float(msg.twist.twist.linear.x)))

    # -- metrics -------------------------------------------------------
    def _trim(self, now):
        for dq in (self._status, self._fixes, self._odom):
            while dq and now - dq[0][0] > self._win:
                dq.popleft()

    def _metrics(self, now):
        """Everything the severity rules and the WebUI need, in one dict."""
        st = list(self._status)
        n = len(st)
        good = sum(1 for r in st if r[1] in _GOOD_QUALITIES)
        sats = [r[2] for r in st if r[2] is not None]
        hdops = [r[3] for r in st if r[3] is not None]
        ages = [r[4] for r in st if r[4] is not None]

        # The jitter invariant: fix-to-fix path vs net displacement vs odometry.
        fx = list(self._fixes)
        path = sum(_geo_dist(fx[i][1], fx[i][2], fx[i + 1][1], fx[i + 1][2])
                   for i in range(len(fx) - 1)) if len(fx) > 1 else 0.0
        net = (_geo_dist(fx[0][1], fx[0][2], fx[-1][1], fx[-1][2])
               if len(fx) > 1 else 0.0)
        od = list(self._odom)
        odom_dist = 0.0
        for i in range(len(od) - 1):
            dt = od[i + 1][0] - od[i][0]
            if 0.0 < dt < 1.0:
                odom_dist += abs(od[i][1]) * dt
        # Compare the GPS path against the largest honest estimate of real
        # motion we have. The floor keeps normal cm-scale noise from tripping it
        # while the robot is legitimately parked.
        moved = max(net, odom_dist, self._jitter_floor)
        return {
            "samples": n,
            "fixed_frac": (good / n) if n else None,
            "sats": (sum(sats) / len(sats)) if sats else None,
            "hdop": (sum(hdops) / len(hdops)) if hdops else None,
            "rtcm_fwd_age_s": ages[-1] if ages else None,
            "quality_last": st[-1][1] if st else None,
            "nofix_s": (now - self._last_good_fix_t) if self._last_good_fix_t
                       else None,
            "gps_path_m": round(path, 2),
            "gps_net_m": round(net, 2),
            "odom_dist_m": round(odom_dist, 2),
            "jitter_ratio": round(path / moved, 1) if moved > 0 else None,
        }

    def _classify(self, m):
        """-> (state, [reasons]). SEVERE wins; WARN is survivable degradation."""
        severe, warn = [], []
        n = m["samples"]
        frac = m["fixed_frac"]
        age = m["rtcm_fwd_age_s"]
        nofix = m["nofix_s"]

        if n >= self._min_samples and frac is not None:
            if frac < self._severe_frac:
                severe.append(f"only {100 * frac:.0f}% of the last "
                              f"{self._win:.0f}s had an RTK fix "
                              f"(severe below {100 * self._severe_frac:.0f}%)")
            elif frac < self._warn_frac:
                warn.append(f"{100 * frac:.0f}% fixed over {self._win:.0f}s "
                            f"(want >= {100 * self._warn_frac:.0f}%)")
        if nofix is not None and nofix > self._severe_nofix_s:
            severe.append(f"no usable fix for {nofix:.0f}s")
        if age is not None:
            if age == float("inf") or age > self._severe_rtcm_s:
                severe.append("RTCM corrections are not arriving "
                              f"(fwd_age={age})")
            elif age > self._warn_rtcm_s:
                warn.append(f"RTCM corrections aging ({age:.0f}s)")
        if m["hdop"] is not None and m["hdop"] > self._warn_hdop:
            warn.append(f"HDOP {m['hdop']:.1f}")
        if m["sats"] is not None and m["sats"] < self._warn_sats:
            warn.append(f"only {m['sats']:.1f} satellites")
        # Jitter: the fix is moving and the robot is not. Needs a real amount of
        # apparent motion so cm-scale noise on a parked robot stays quiet.
        if (m["jitter_ratio"] is not None and m["gps_path_m"] > 2.0
                and m["jitter_ratio"] > self._jitter_ratio):
            severe.append(
                f"fix is jittering: {m['gps_path_m']:.1f} m of fix-to-fix "
                f"movement for {m['gps_net_m']:.2f} m net and "
                f"{m['odom_dist_m']:.2f} m of wheel odometry "
                f"({m['jitter_ratio']:.0f}x) — position is not real")
        if severe:
            return "SEVERE", severe
        if warn:
            return "WARN", warn
        return "OK", []

    # -- actions -------------------------------------------------------
    def _page(self, state, reasons, m):
        detail = "; ".join(reasons)
        bits = []
        if m["fixed_frac"] is not None:
            bits.append(f"fixed {100 * m['fixed_frac']:.0f}%")
        if m["sats"] is not None:
            bits.append(f"sats {m['sats']:.1f}")
        if m["hdop"] is not None:
            bits.append(f"HDOP {m['hdop']:.1f}")
        body = f"RTK {state}: {detail}. [{', '.join(bits)}]"
        try:
            ok = _notify_discord(body, title="H-inf RTK watchdog")
            self.get_logger().error(f"PAGED operator (discord ok={ok}): {body}")
        except Exception as exc:
            self.get_logger().warn(f"page failed: {exc}")

    def _clear_page(self):
        try:
            _notify_discord("RTK health recovered — watchdog all-clear.",
                            title="H-inf RTK watchdog")
        except Exception:
            pass
        self.get_logger().warn("RTK health recovered — operator notified.")

    def _pause_batch(self, reasons):
        if not self._may_pause or self._paused_by_us:
            return
        self._paused_by_us = True
        self.pub_cmd.publish(String(data=json.dumps({
            "action": "pause",
            "reason": "rtk_watchdog: " + "; ".join(reasons),
        })))
        self.get_logger().error("batch PAUSED on severe RTK degradation.")

    def _resume_batch(self):
        if not self._paused_by_us:
            return
        self._paused_by_us = False
        # if_reason_prefix: resume ONLY a pause this node caused. An operator
        # pause, or a pause from another guard, is never overridden.
        self.pub_cmd.publish(String(data=json.dumps({
            "action": "resume",
            "if_reason_prefix": "rtk_watchdog:",
        })))
        self.get_logger().warn("RTK healthy again — batch resume requested.")

    # -- main loop -----------------------------------------------------
    def _tick(self):
        now = time.time()
        self._trim(now)
        m = self._metrics(now)
        state, reasons = self._classify(m)
        self._state = state

        if state == "OK":
            if self._ok_since is None:
                self._ok_since = now
            if self._paged_state != "OK":
                self._clear_page()
                self._paged_state = "OK"
            if self._paused_by_us and (now - self._ok_since) >= self._resume_good_s:
                self._resume_batch()
        else:
            self._ok_since = None
            # Page once per escalation: OK->WARN pages, WARN->SEVERE pages again.
            rank = {"OK": 0, "WARN": 1, "SEVERE": 2}
            if rank[state] > rank[self._paged_state]:
                self._page(state, reasons, m)
                self._paged_state = state
            if state == "SEVERE":
                self._pause_batch(reasons)

        payload = dict(m)
        payload.update({
            "state": state,
            "reasons": reasons,
            "paused_by_watchdog": self._paused_by_us,
            "window_s": self._win,
        })
        self.pub_status.publish(String(data=json.dumps(payload)))
        if state != "OK":
            self.get_logger().warn(f"RTK {state}: " + "; ".join(reasons),
                                   throttle_duration_sec=10.0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--status-topic", default="/gps_rtk_f9p_helical/gps/rtk_status")
    ap.add_argument("--fix-topic", default="/gps_rtk_f9p_helical/gps/fix")
    ap.add_argument("--odom-topic", default="/wheel/odom")
    ap.add_argument("--window", type=float, default=30.0)
    ap.add_argument("--min-samples", type=int, default=10)
    # Every session that produced usable data ran >= 95% fixed; the 2026-09-16
    # washout ran 69-73%. Warn at the first, call it severe well below it.
    ap.add_argument("--warn-fixed-frac", type=float, default=0.95)
    ap.add_argument("--severe-fixed-frac", type=float, default=0.50)
    ap.add_argument("--warn-hdop", type=float, default=5.0)
    ap.add_argument("--warn-sats", type=float, default=8.0)
    ap.add_argument("--warn-rtcm-age", type=float, default=5.0)
    ap.add_argument("--severe-rtcm-age", type=float, default=30.0)
    ap.add_argument("--severe-nofix-s", type=float, default=15.0)
    ap.add_argument("--jitter-ratio", type=float, default=4.0)
    ap.add_argument("--jitter-floor", type=float, default=0.20)
    ap.add_argument("--resume-good-s", type=float, default=30.0)
    ap.add_argument("--no-pause", dest="pause", action="store_false")
    ap.set_defaults(pause=True)
    args, _ = ap.parse_known_args()
    rclpy.init()
    node = RtkWatchdog(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
