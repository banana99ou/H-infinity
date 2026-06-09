#!/usr/bin/env python3
"""Read-only monitor for the autonomous experiment sequencer.

Subscribes /experiment/status (transient_local), prints every phase/leg/cell/
message transition with a timestamp, and exits on a terminal phase:
  done    -> exit 0 if fail==0 else 1
  aborted -> exit 1
  timeout -> exit 2
A `paused` state is surfaced loudly but not treated as terminal (RTK-loss
auto-resumes; an operator may resume). Used by field_smoke.sh to watch a run,
but safe to run standalone alongside any sequencer run (it never publishes).

  python3 watch_experiment.py [--timeout SEC] [--quiet-heartbeat]
"""
import argparse
import json
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String


def _ts():
    return time.strftime("%H:%M:%S")


class Watcher(Node):
    def __init__(self, timeout_s, heartbeat):
        super().__init__("experiment_watcher")
        self._deadline = time.monotonic() + timeout_s
        self._heartbeat = heartbeat
        self._last_key = None
        self._last_beat = 0.0
        self.result = None  # set to exit code when terminal
        qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "/experiment/status", self._cb, qos)
        self.create_timer(1.0, self._tick)
        print(f"[{_ts()}] watching /experiment/status (timeout {timeout_s:.0f}s)...")

    def _cb(self, msg):
        try:
            d = json.loads(msg.data)
        except Exception:
            return
        key = (d.get("phase"), d.get("leg"), d.get("cell_id"), d.get("message"))
        if key != self._last_key:
            self._last_key = key
            print(f"[{_ts()}] phase={d.get('phase')} leg={d.get('leg')} "
                  f"cell={d.get('cell_id')} pass={d.get('pass')} fail={d.get('fail')}"
                  f" eta={d.get('eta_s')}s"
                  + (f"  :: {d.get('message')}" if d.get('message') else ""))
        phase = d.get("phase")
        if phase == "done":
            fail = d.get("fail") or 0
            print(f"[{_ts()}] DONE — pass={d.get('pass')} fail={fail}")
            self.result = 0 if fail == 0 else 1
        elif phase == "aborted":
            print(f"[{_ts()}] ABORTED :: {d.get('message')}")
            self.result = 1
        elif phase == "paused":
            print(f"[{_ts()}] *** PAUSED *** :: {d.get('message')} "
                  "(RTK-loss auto-resumes; else operator action / fault)")

    def _tick(self):
        now = time.monotonic()
        if self.result is not None:
            raise KeyboardInterrupt
        if now > self._deadline:
            print(f"[{_ts()}] TIMEOUT — no terminal phase reached")
            self.result = 2
            raise KeyboardInterrupt
        if self._heartbeat and now - self._last_beat > 20:
            self._last_beat = now
            print(f"[{_ts()}] ...watching (last: {self._last_key})")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--timeout", type=float, default=900.0)
    ap.add_argument("--quiet-heartbeat", action="store_true")
    a = ap.parse_args()
    rclpy.init()
    n = Watcher(a.timeout, not a.quiet_heartbeat)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        rc = n.result if n.result is not None else 2
        n.destroy_node()
        rclpy.shutdown()
        sys.exit(rc)


if __name__ == "__main__":
    main()
