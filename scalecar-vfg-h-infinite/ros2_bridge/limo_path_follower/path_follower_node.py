# -*- coding: utf-8 -*-
"""ROS2 path follower node for LIMO robot.

Author: Suwon Lee, Kookmin University
Created: 2026-03-01
Description: Subscribes to /wheel/odom, computes VFG + LPV-Hinf (or PID-FF)
    steering, and publishes cmd_vel_raw (routed through estop_cli.py to
    /cmd_vel).  Converts front-wheel steering angle to yaw-rate:
    omega = v * tan(delta) / L.

    A hardcoded StepCurvaturePath is used for demonstration.
    Replace it with your own path source for real experiments.
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from vfg_pathfollowing import (
    StepCurvaturePath,
    VectorFieldGuidance,
    LPVHinfController,
    PIDFeedforward,
)


def _yaw_from_quaternion(q):
    """Extract yaw from geometry_msgs Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class PathFollowerNode(Node):
    """VFG path follower node.

    Subscribes to /wheel/odom for vehicle state, runs guidance + controller,
    and publishes cmd_vel_raw at a fixed rate (default 20 Hz). The E-stop
    node (estop_cli.py) filters cmd_vel_raw and re-publishes /cmd_vel.
    """

    def __init__(self):
        super().__init__('path_follower_node')

        # -- Declare parameters -----------------------------------------
        self.declare_parameter('controller_type', 'lpv-hinf')
        self.declare_parameter('v_const', 1.0)
        self.declare_parameter('k_e', 3.0)
        self.declare_parameter('dt_ctrl', 0.05)
        self.declare_parameter('wheelbase', 0.2)
        self.declare_parameter('K_P', 2.0)
        self.declare_parameter('K_D', 0.3)

        # -- Read parameters --------------------------------------------
        ctrl_type = self.get_parameter('controller_type').value
        self.v_const = min(self.get_parameter('v_const').value, 3.0)  # LIMO max ~3 m/s
        k_e = self.get_parameter('k_e').value
        self.dt_ctrl = self.get_parameter('dt_ctrl').value
        self.wheelbase = self.get_parameter('wheelbase').value
        K_P = self.get_parameter('K_P').value
        K_D = self.get_parameter('K_D').value

        # -- Path (hardcoded demo -- replace for real experiments) ------
        self.path = StepCurvaturePath(L1=5.0, R=0.5, theta_arc=np.pi / 2, L2=25.0)

        # -- Guidance ---------------------------------------------------
        self.guidance = VectorFieldGuidance(self.path, k_e=k_e)

        # -- Controller -------------------------------------------------
        ctrl_key = ctrl_type.lower().strip()
        if ctrl_key in ('lpv-hinf', 'lpv_hinf', 'lpv', 'hinf'):
            self.controller = LPVHinfController.default(dt=self.dt_ctrl)
            self._ctrl_type = 'lpv-hinf'
        elif ctrl_key in ('pid-ff', 'pid_ff', 'pid'):
            self.controller = PIDFeedforward(K_P=K_P, K_D=K_D, L=self.wheelbase)
            self._ctrl_type = 'pid-ff'
        else:
            self.get_logger().error(f'Unknown controller_type: {ctrl_type}')
            raise ValueError(f"Unknown controller_type '{ctrl_type}'. "
                             f"Choose from: 'lpv-hinf', 'pid-ff'")

        self.get_logger().info(
            f'Controller: {self._ctrl_type}, v={self.v_const:.2f} m/s, '
            f'k_e={k_e:.1f}, dt={self.dt_ctrl:.3f} s, L={self.wheelbase:.3f} m'
        )

        # -- State from odometry ----------------------------------------
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._v = 0.0
        self._odom_stamp = None  # last odometry timestamp
        self._delta_prev = 0.0  # previous steering command

        # -- ROS2 interfaces --------------------------------------------
        self.sub_odom = self.create_subscription(
            Odometry, '/wheel/odom', self._odom_cb, 10)

        self.pub_cmd = self.create_publisher(Twist, 'cmd_vel_raw', 10)

        timer_period = self.dt_ctrl  # seconds
        self.timer = self.create_timer(timer_period, self._control_cb)

        self.get_logger().info('Path follower node started.')

    # -----------------------------------------------------------------
    # Callbacks
    # -----------------------------------------------------------------

    def _odom_cb(self, msg: Odometry):
        """Extract pose and velocity from Odometry message."""
        self._x = msg.pose.pose.position.x
        self._y = msg.pose.pose.position.y
        self._yaw = _yaw_from_quaternion(msg.pose.pose.orientation)
        self._v = math.hypot(
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
        )
        self._odom_stamp = self.get_clock().now()

    def _control_cb(self):
        """Timer callback: compute and publish control command."""
        cmd = Twist()

        # Safety: check odom timeout (0.5 s)
        if self._odom_stamp is None:
            self.pub_cmd.publish(cmd)  # zero velocity
            return

        dt_since_odom = (self.get_clock().now() - self._odom_stamp).nanoseconds * 1e-9
        if dt_since_odom > 0.5:
            self.get_logger().warn(
                f'Odometry timeout ({dt_since_odom:.2f} s). Sending zero velocity.')
            self._delta_prev = 0.0
            self.pub_cmd.publish(cmd)
            return

        # -- Guidance --------------------------------------------------
        q = np.array([self._x, self._y])
        gresult = self.guidance.compute(q, self._yaw)

        psi_des = gresult['psi_des']
        kappa = gresult['kappa']
        s_star = gresult['s_star']

        # Check path end: stop if within 0.3 m of the end
        if s_star >= self.path.total_length - 0.3:
            self.get_logger().info('Reached path end. Stopping.')
            self._delta_prev = 0.0
            self.pub_cmd.publish(cmd)
            return

        # Heading error (psi_des - psi)
        e_psi = psi_des - self._yaw
        # Wrap to [-pi, pi]
        e_psi = math.atan2(math.sin(e_psi), math.cos(e_psi))

        # -- Controller ------------------------------------------------
        v = self.v_const
        rho = abs(kappa) * v

        if self._ctrl_type == 'lpv-hinf':
            delta_cmd = self.controller.compute(
                e_psi, delta_meas=self._delta_prev, rho=rho, dt=self.dt_ctrl)
        else:
            delta_cmd = self.controller.compute(
                e_psi, kappa=kappa, dt=self.dt_ctrl)

        # Safety clip
        delta_cmd = float(np.clip(delta_cmd, -0.5, 0.5))
        self._delta_prev = delta_cmd

        # -- Convert to Twist ------------------------------------------
        # Bicycle kinematic: omega = v * tan(delta) / L
        omega = v * math.tan(delta_cmd) / self.wheelbase

        cmd.linear.x = v
        cmd.angular.z = omega
        self.pub_cmd.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PathFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Send zero velocity on shutdown
        stop_cmd = Twist()
        node.pub_cmd.publish(stop_cmd)
        node.get_logger().info('Shutting down. Sent zero velocity.')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
