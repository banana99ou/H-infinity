#!/usr/bin/env python3
"""Sweep the LIMO steering left↔right at constant v.

Wheels-off-floor diagnostic. Publishes /cmd_vel_raw (so estop_cli stays in
the loop). At v = 0.2 m/s and L = 0.2 m, ω = ±tan(0.5)/1 ≈ ±0.55 rad/s
drives the steering to ±δ_max = ±0.5 rad.

On Ctrl+C or normal exit, publishes zeros and stops.
"""
import math
import signal
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


V_LIN = 0.2          # m/s
W_AMP = 0.55         # rad/s — gives full ±0.5 rad steering at v=0.2
SWEEP_HZ = 0.5       # one full left-right-left cycle every 2 s
PUB_HZ = 30
DURATION_S = 10.0    # total sweep time


class Sweep(Node):
    def __init__(self):
        super().__init__('sweep_steering')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_raw', 10)
        self.estop_pub = self.create_publisher(Bool, '/estop_trigger', 1)
        self.t0 = time.monotonic()
        self.timer = self.create_timer(1.0 / PUB_HZ, self._tick)
        # Clear E-stop once at start. If user wants to keep it engaged, they
        # can re-trigger via the dashboard.
        clear = Bool(); clear.data = False
        self.estop_pub.publish(clear)
        self.get_logger().info(
            f'sweep starting: v={V_LIN} m/s, ω=±{W_AMP} rad/s @ {SWEEP_HZ} Hz, '
            f'pub {PUB_HZ} Hz, duration {DURATION_S} s'
        )

    def _tick(self):
        t = time.monotonic() - self.t0
        if t > DURATION_S:
            self.stop_and_exit()
            return
        msg = Twist()
        msg.linear.x = V_LIN
        msg.angular.z = W_AMP * math.sin(2.0 * math.pi * SWEEP_HZ * t)
        self.cmd_pub.publish(msg)

    def stop_and_exit(self):
        zero = Twist()
        for _ in range(5):
            self.cmd_pub.publish(zero)
            time.sleep(0.02)
        self.get_logger().info('sweep complete; published zero twist')
        self.timer.cancel()
        rclpy.shutdown()


def main():
    rclpy.init()
    node = Sweep()

    def on_sigint(*_):
        node.get_logger().info('interrupted; stopping')
        node.stop_and_exit()
        sys.exit(0)
    signal.signal(signal.SIGINT, on_sigint)

    try:
        rclpy.spin(node)
    except Exception:
        node.stop_and_exit()
        raise


if __name__ == '__main__':
    main()
