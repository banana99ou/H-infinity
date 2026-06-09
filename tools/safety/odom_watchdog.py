#!/usr/bin/env python3
"""Odom-loss watchdog — auto-recover the LIMO chassis serial without a reboot.

The chassis<->NUC USB (a CP2102 at /dev/limo_base) drops intermittently under
vibration, so /wheel/odom goes silent and the follower would drive on a stale
belief. The sequencer pauses the run on that (safety); THIS node performs the
RECOVERY so an unattended batch keeps going, escalating only as far as needed:

  1. grace        wait ~3 s for the LIMO driver to self-reopen the kernel-
                  re-enumerated device (observed to happen on its own).
  2. respawn      /orchestrator/kill base ; /orchestrator/start base — a fresh
                  open of /dev/limo_base. Fixes the common (re-contacted) case.
  3. usb rebind   if respawn didn't restore odom, unbind/rebind the CP2102 USB
                  port (re-enumerate just that device, no reboot) then respawn.
  4. give up      log + publish /odom_watchdog/status; operator must reboot.

It acts ONLY while the 'base' PROC is alive (per /orchestrator/status) so it
never fires when the chassis driver is intentionally down. RTK + compass live on
separate USB (ttyACM*), so this never disturbs them — no geofence interaction.

  python3 odom_watchdog.py [--base-proc base] [--grace 3] [--respawn-wait 6]
                           [--cooldown 20] [--rebind <script>]
"""
import argparse
import json
import os
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String

_DEFAULT_REBIND = "/home/agilex/H-infinity/tools/safety/usb_rebind_limo_base.sh"

# Operator paging (Discord webhook from discord.env / ntfy), imported defensively
# so a missing module just disables paging (logs only). notify_discord resolves
# discord.env on its own and never raises.
try:
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "notify"))
    from ntfy import notify_discord as _notify_discord  # type: ignore
except Exception:
    def _notify_discord(*_a, **_k):
        return False


class OdomWatchdog(Node):
    def __init__(self, base_proc, grace, respawn_wait, cooldown, rebind):
        super().__init__("odom_watchdog")
        self._base = base_proc
        self._grace = grace
        self._respawn_wait = respawn_wait
        self._cooldown = cooldown
        self._rebind = rebind

        self._last_odom = None        # wall-time of last /wheel/odom
        self._base_alive = False
        self._base_up_since = None

        # Recovery FSM: 'idle' -> 'kill' -> 'start' -> 'check' (-> 'rebind' ->
        # 'kill' once) -> 'giveup'. _stage_t = when the current stage started.
        self._stage = "idle"
        self._stage_t = 0.0
        self._rebound = False         # USB rebind already tried this cycle
        self._last_recovery_t = None  # cooldown anchor
        self._paged = False           # operator already paged this outage

        self.pub_kill = self.create_publisher(String, "/orchestrator/kill", 10)
        self.pub_start = self.create_publisher(String, "/orchestrator/start", 10)
        self.pub_status = self.create_publisher(String, "/odom_watchdog/status", 10)
        self.create_subscription(Odometry, "/wheel/odom", self._on_odom, 10)
        self.create_subscription(String, "/orchestrator/status", self._on_orch, 10)
        self.create_timer(0.5, self._tick)
        self.get_logger().warn(
            f"ODOM WATCHDOG armed (base PROC='{base_proc}', grace {grace:.1f}s, "
            f"respawn_wait {respawn_wait:.1f}s, cooldown {cooldown:.0f}s).")

    # -- inputs --------------------------------------------------------
    def _on_odom(self, _msg):
        self._last_odom = time.time()

    def _on_orch(self, msg):
        try:
            d = json.loads(msg.data) or {}
        except (ValueError, TypeError):
            return
        alive = bool(d.get(self._base, False))
        if alive and not self._base_alive:
            self._base_up_since = time.time()   # give a fresh driver time
        if not alive:
            self._base_up_since = None
        self._base_alive = alive

    # -- helpers -------------------------------------------------------
    def _odom_fresh(self):
        return (self._last_odom is not None
                and (time.time() - self._last_odom) <= self._grace)

    def _odom_silent(self):
        """Base is alive but odom has been silent past the grace window."""
        if not self._base_alive:
            return False
        now = time.time()
        if self._last_odom is not None and (now - self._last_odom) <= self._grace:
            return False
        # Never seen odom: only judge silent after the base has had grace to start.
        if self._last_odom is None:
            return (self._base_up_since is not None
                    and now - self._base_up_since > self._grace + 5.0)
        return True

    def _enter(self, stage):
        self._stage = stage
        self._stage_t = time.time()

    def _emit(self, state, detail=""):
        self.pub_status.publish(String(data=json.dumps({"state": state, "detail": detail})))

    def _page(self, message):
        """Page the operator ONCE per outage (Discord webhook / ntfy). Never raises."""
        if self._paged:
            return
        self._paged = True
        try:
            ok = _notify_discord(message, title="H-inf odom watchdog")
            self.get_logger().error(f"PAGED operator (discord ok={ok}): {message}")
        except Exception as exc:
            self.get_logger().warn(f"page failed: {exc}")

    def _clear_page(self):
        """All-clear after an outage we paged about (only fires if we paged)."""
        if not self._paged:
            return
        self._paged = False
        try:
            _notify_discord("LIMO /wheel/odom recovered — watchdog all-clear.",
                            title="H-inf odom watchdog")
        except Exception:
            pass
        self.get_logger().warn("odom outage cleared — operator notified.")

    def _kill(self):
        self.pub_kill.publish(String(data=self._base))

    def _start(self):
        self.pub_start.publish(String(data=self._base))

    # -- recovery FSM (driven by _tick) --------------------------------
    def _tick(self):
        now = time.time()

        # Healthy / base intentionally down: stay idle, reset cycle.
        if self._stage == "idle":
            if not self._odom_silent():
                self._rebound = False
                self._clear_page()  # all-clear if we'd paged an outage
                return
            # Cooldown after a recent recovery cycle (avoid respawn storms).
            if (self._last_recovery_t is not None
                    and now - self._last_recovery_t < self._cooldown):
                return
            self.get_logger().error(
                f"/wheel/odom silent while '{self._base}' alive — recovering.")
            self._emit("recovering", "respawn base")
            self._kill()
            self._enter("kill")
            return

        # If odom came back mid-recovery, we're done.
        if self._odom_fresh():
            self.get_logger().warn("odom recovered.")
            self._emit("recovered")
            self._last_recovery_t = now
            self._rebound = False
            self._clear_page()
            self._enter("idle")
            return

        if self._stage == "kill":
            if now - self._stage_t >= 2.0:     # let the kill complete
                self._start()
                self._enter("start")
            return

        if self._stage == "start":
            if now - self._stage_t >= self._respawn_wait:
                self._enter("check")
            return

        if self._stage == "check":
            # odom still not fresh (checked above). Escalate.
            if not self._rebound:
                self.get_logger().error("respawn did not restore odom — USB rebind.")
                self._emit("recovering", "usb rebind")
                self._rebound = True
                self._enter("rebind")
            else:
                self.get_logger().error(
                    "odom UNRECOVERABLE after respawn+rebind — operator reboot needed.")
                self._emit("unrecoverable", "respawn+rebind failed; reboot")
                self._page("LIMO /wheel/odom UNRECOVERABLE — chassis serial down; "
                           "auto-respawn + USB rebind failed. Manual reboot needed.")
                self._last_recovery_t = now    # cooldown before retrying the cycle
                self._rebound = False
                self._enter("idle")
            return

        if self._stage == "rebind":
            try:
                subprocess.run([self._rebind], timeout=15, check=False)
            except Exception as exc:
                self.get_logger().warn(f"usb rebind helper failed: {exc}")
            self._kill()                       # respawn after the rebind
            self._enter("kill")
            return


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--base-proc", default="base")
    ap.add_argument("--grace", type=float, default=3.0)
    ap.add_argument("--respawn-wait", type=float, default=10.0)
    ap.add_argument("--cooldown", type=float, default=20.0)
    ap.add_argument("--rebind", default=_DEFAULT_REBIND)
    args, _ = ap.parse_known_args()
    rclpy.init()
    node = OdomWatchdog(args.base_proc, args.grace, args.respawn_wait,
                        args.cooldown, args.rebind)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
