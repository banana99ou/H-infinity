#!/usr/bin/env python3

"""
Terminal-based E-stop and safety filter for LIMO.

Use this when you are SSH'ed into the robot and cannot open a GUI window.

Behavior:
- Checks connectivity to the MacBook control station (10.42.0.118) or internet (google.com) and trips the E-stop if it fails.
- Subscribes to raw velocity commands on `cmd_vel_raw`.
- Publishes safe velocity commands on `cmd_vel`.
- Publishes the E-stop state on `/estop` (std_msgs/Bool).

Keys (in the terminal where this script runs):
  s or SPACE : ACTIVATE E-stop (latch, send zero cmd_vel)
  c          : CLEAR E-stop (allow motion again)
  q or Ctrl+C: Quit (leaves E-stop ACTIVE as a conservative default)
"""

import os
import sys
import select
import termios
import tty
import subprocess
import platform
import argparse
import time
import threading
from datetime import datetime

import rclpy  # pyright: ignore[reportMissingImports]
from rclpy.node import Node # pyright: ignore[reportMissingImports]

from geometry_msgs.msg import Twist # pyright: ignore[reportMissingImports]
from std_msgs.msg import Bool # pyright: ignore[reportMissingImports]


class EstopCliNode(Node):
    def __init__(self, debug=False, ping_hosts=None, ping_enabled=True):
        super().__init__("limo_estop_cli")

        self.debug = debug
        self.estop_active = False
        self.ping_enabled = ping_enabled
        self.ping_targets = list(ping_hosts) if ping_hosts else ["macbook-air-m1"]
        # Track last known connectivity per host so we only log on state changes
        self.connectivity_status = {host: True for host in self.ping_targets}
        self.failure_counts = {host: 0 for host in self.ping_targets}
        self.total_misses = {host: 0 for host in self.ping_targets}

        # SENSITIVITY SETTINGS
        self.ping_interval  = 0.5  # seconds
        self.ping_timeout   = 3.0   # seconds
        self.ping_threshold = 10    # consecutive failures before tripping

        # Setup debug recording if enabled
        self.debug_file = None
        if self.debug:
            filename = f"estop_debug_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
            try:
                self.debug_file = open(filename, "w")
                self.debug_file.write("timestamp,host,status,consecutive_misses,total_misses\n")
                self.debug_file.flush()
                self.get_logger().info(f"Debug recording enabled: {filename}")
            except Exception as e:
                self.get_logger().error(f"Failed to open debug file: {e}")

        self.estop_pub = self.create_publisher(Bool, "/estop", 10)
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.cmd_vel_raw_sub = self.create_subscription(
            Twist, "cmd_vel_raw", self.cmd_vel_raw_callback, 10
        )
        # Remote E-stop trigger: True -> activate (latched), False -> clear.
        # Used by the battle-station HTML over rosbridge.
        self.estop_trigger_sub = self.create_subscription(
            Bool, "/estop_trigger", self.estop_trigger_callback, 10
        )

        # Create background thread for periodic ping checks
        if self.ping_enabled:
            self.ping_thread = threading.Thread(target=self._ping_loop, daemon=True)
            self.ping_thread.start()
        else:
            self.ping_thread = None

        self.get_logger().info(
            "LIMO E-stop CLI initialized. "
            "Keys: [s/SPACE]=STOP, [c]=CLEAR, [q/Ctrl+C]=quit."
        )
        if self.ping_enabled:
            self.get_logger().info(
                f"Ping monitoring: {', '.join(self.ping_targets)} "
                f"(interval: {self.ping_interval}s)"
            )
        else:
            self.get_logger().info("Ping monitoring DISABLED (--no-ping)")

    # ---- ROS helpers ----

    def publish_estop_state(self):
        msg = Bool()
        msg.data = self.estop_active
        self.estop_pub.publish(msg)

    def publish_zero_cmd(self):
        self.cmd_vel_pub.publish(Twist())

    def estop_trigger_callback(self, msg: Bool):
        if msg.data:
            print("\r")
            self.get_logger().warn("E-STOP requested via /estop_trigger (remote)")
            self.activate_estop()
        else:
            print("\r")
            self.get_logger().info("E-STOP clear requested via /estop_trigger (remote)")
            self.clear_estop()

    def cmd_vel_raw_callback(self, msg: Twist):
        """Filter raw velocity commands based on current E-stop state."""
        if self.estop_active:
            # Force zero command while stopped
            self.publish_zero_cmd()
        else:
            # Pass through original command
            self.cmd_vel_pub.publish(msg)

    # ---- E-stop control ----

    def activate_estop(self):
        if not self.estop_active:
            self.estop_active = True
            self.publish_estop_state()
            self.publish_zero_cmd()
            # In raw/cbreak mode, we need \r\n for the logger to not staircase
            print("\r") 
            self.get_logger().warn("E-STOP ACTIVATED (CLI)")

    def clear_estop(self):
        if not self.estop_active:
            return
        self.estop_active = False
        self.publish_estop_state()
        print("\r")
        self.get_logger().info("E-STOP CLEARED (CLI)")

    # ---- Connectivity monitoring ----

    def ping_host(self, host):
        """Ping a host and return True if successful, False otherwise."""
        try:
            # Linux/Unix: ping -c 1 -W timeout_sec host
            result = subprocess.run(
                ["ping", "-c", "1", "-W", str(int(self.ping_timeout)), host],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=self.ping_timeout + 0.5,
            )
            return result.returncode == 0
        except (subprocess.TimeoutExpired, FileNotFoundError, Exception) as e:
            self.get_logger().debug(f"Ping error for {host}: {e}")
            return False
    def check_connectivity(self):
        """Check connectivity to all ping targets and trip E-stop on threshold."""
        for target in self.ping_targets:
            ok = self.ping_host(target)
            
            if not ok:
                self.failure_counts[target] += 1
                self.total_misses[target] += 1
                
                # Record to debug file if enabled
                if self.debug and self.debug_file:
                    timestamp = datetime.now().isoformat()
                    self.debug_file.write(f"{timestamp},{target},FAIL,{self.failure_counts[target]},{self.total_misses[target]}\n")
                    self.debug_file.flush()

                # If we hit the threshold and we haven't already marked it as failed
                if self.failure_counts[target] >= self.ping_threshold and self.connectivity_status[target]:
                    print("\r")
                    self.get_logger().warn(
                        f"Connectivity FAILED for {target} ({self.failure_counts[target]} misses) - ACTIVATING E-STOP"
                    )
                    self.connectivity_status[target] = False
                    self.activate_estop()
            else:
                # If it was previously failed, log restoration
                if not self.connectivity_status[target]:
                    print("\r")
                    self.get_logger().info(f"Connectivity restored for {target}")
                
                # Record restoration to debug file
                if self.debug and self.debug_file and not ok: # This condition is slightly wrong, 'ok' is True here
                    pass # Handled below
                
                if self.debug and self.debug_file and self.failure_counts[target] > 0:
                    timestamp = datetime.now().isoformat()
                    self.debug_file.write(f"{timestamp},{target},OK,0,{self.total_misses[target]}\n")
                    self.debug_file.flush()

                self.connectivity_status[target] = True
                self.failure_counts[target] = 0

    def _ping_loop(self):
        """Background loop to run pings without blocking ROS callbacks."""
        while rclpy.ok():
            self.check_connectivity()
            time.sleep(self.ping_interval)


def save_terminal_settings():
    return termios.tcgetattr(sys.stdin)


def restore_terminal_settings(settings):
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


def print_help(debug_enabled=False):
    """Print the help header."""
    print("\r\n" + "="*50)
    print("      LIMO E-STOP COMMAND LINE INTERFACE")
    if debug_enabled:
        print("                (DEBUG MODE)")
    print("="*50)
    print("  [s / SPACE] : ACTIVATE E-stop (LATCHED)")
    print("  [c]         : CLEAR E-stop")
    print("  [q / Ctrl+C]: Quit (Leaves E-stop ACTIVE)")
    print("-" * 50 + "\r\n")


def print_status_line(node):
    """Print a one-line live status bar at the bottom."""
    # ANSI escape: \033[K clears the line from cursor to end
    # \r moves to start of line
    state_str = "\033[1;31m!!! E-STOP ACTIVE !!!\033[0m" if node.estop_active else "\033[1;32m[ OK: Motion Allowed ]\033[0m"

    if not node.ping_enabled:
        status = f"\r[STATUS] {state_str} | Pings: \033[33mDISABLED\033[0m \033[K"
        sys.stdout.write(status)
        sys.stdout.flush()
        return

    ping_parts = []
    for host in node.ping_targets:
        ok = node.connectivity_status[host]
        count = node.failure_counts[host]
        total = node.total_misses[host]
        color = "\033[32m" if ok else "\033[31m"
        status_text = "OK" if ok else "FAIL"

        # Show misses if any (e.g. "FAIL (2/3 misses)")
        miss_text = f" ({count}/{node.ping_threshold} misses)" if count > 0 else ""

        # In debug mode, show total misses as well
        if node.debug:
            miss_text += f" [Total: {total}]"

        ping_parts.append(f"{host}: {color}{status_text}{miss_text}\033[0m")

    status = f"\r[STATUS] {state_str} | Pings: {' | '.join(ping_parts)} \033[K"
    sys.stdout.write(status)
    sys.stdout.flush()


def main(args=None):
    # Parse custom arguments before ROS init
    parser = argparse.ArgumentParser(description="LIMO E-stop CLI")
    parser.add_argument("--debug", action="store_true", help="Enable debug mode (logging and extra status)")
    parser.add_argument(
        "--no-ping",
        action="store_true",
        help="Disable connectivity monitor entirely (no auto-trip on link loss).",
    )
    parser.add_argument(
        "--ping-host",
        action="append",
        default=None,
        metavar="HOST",
        help="Override ping target. Repeatable. Defaults to macbook-air-m1 (Tailscale).",
    )
    parsed_args, ros_args = parser.parse_known_args(args)

    rclpy.init(args=ros_args)
    node = EstopCliNode(
        debug=parsed_args.debug,
        ping_hosts=parsed_args.ping_host,
        ping_enabled=not parsed_args.no_ping,
    )

    # Spin ROS2 in a background thread so terminal UI/input doesn't throttle message frequency
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    have_tty = sys.stdin.isatty()

    settings = None
    if have_tty:
        settings = save_terminal_settings()
        # setcbreak is better for CLIs than setraw as it handles some output mapping
        tty.setcbreak(sys.stdin.fileno())
        print_help(debug_enabled=parsed_args.debug)
    else:
        node.get_logger().info(
            "stdin is not a TTY (running under systemd or non-interactive shell). "
            "Keyboard input disabled; control via /estop_trigger only."
        )

    try:
        while rclpy.ok():
            if have_tty:
                # Update status line frequently
                print_status_line(node)

                # Check if a key is available
                rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
                if rlist:
                    ch = sys.stdin.read(1)

                    if ch in ("s", "S", " "):
                        node.activate_estop()
                    elif ch in ("c", "C"):
                        node.clear_estop()
                    elif ch in ("q", "Q", "\x03"):  # q or Ctrl+C
                        break
            else:
                # No keyboard — just sleep so the spin thread does the work.
                time.sleep(0.1)

    except KeyboardInterrupt:
        pass
    finally:
        # On exit, enforce a safe state
        node.estop_active = True
        node.publish_estop_state()
        node.publish_zero_cmd()

        if have_tty:
            print("\r\n\n[Exiting E-STOP CLI] - E-stop remains ACTIVE for safety.\r\n")

        if node.debug_file:
            node.debug_file.close()

        if settings is not None:
            restore_terminal_settings(settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
