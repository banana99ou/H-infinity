#!/usr/bin/env python3
"""Publish a latched sinusoidal indoor reference path on /reference_path.

Workspace: 5 x 5 m square. Margin: 0.5 m on every edge -> usable [0.5, 4.5]^2.
Path: starts at (0.5, 2.5) heading +x, ends at (4.5, 2.5).
      y(x) = 2.5 + 0.7 * sin(2*pi*(x - 0.5)/4)
      One full sine period over 4 m of forward travel.
      R_min ~= 0.58 m (above LIMO mechanical limit ~0.37 m, safe margin).
      |kappa|_max ~= 1.73, so rho_max = |kappa|*v <= 1.73 even at v=1.0
      (well within LPV scheduling envelope rho_max=5).

Frame: 'odom' to match the path_follower_node default odom_frame.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
)
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


X0, X1 = 0.5, 4.5
Y_CENTER = 2.5
AMPLITUDE = 0.7
WAVELENGTH = 4.0
N_POINTS = 81  # 5 cm step along x


def build_path(frame_id: str, stamp) -> Path:
    msg = Path()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp

    k = 2.0 * math.pi / WAVELENGTH
    for i in range(N_POINTS):
        t = i / (N_POINTS - 1)
        x = X0 + t * (X1 - X0)
        phase = k * (x - X0)
        y = Y_CENTER + AMPLITUDE * math.sin(phase)
        dydx = AMPLITUDE * k * math.cos(phase)
        yaw = math.atan2(dydx, 1.0)

        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = stamp
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        msg.poses.append(pose)

    return msg


class IndoorPathPublisher(Node):
    def __init__(self):
        super().__init__('indoor_path_publisher')
        self.declare_parameter('frame_id', 'odom')
        frame_id = self.get_parameter('frame_id').value

        latched = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub = self.create_publisher(Path, '/reference_path', latched)

        msg = build_path(frame_id, self.get_clock().now().to_msg())
        self.pub.publish(msg)
        self.get_logger().info(
            f'published {len(msg.poses)} waypoints on /reference_path '
            f'(frame={frame_id}, span x=[{X0},{X1}], A={AMPLITUDE}, '
            f'lambda={WAVELENGTH})'
        )


def main():
    rclpy.init()
    node = IndoorPathPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
