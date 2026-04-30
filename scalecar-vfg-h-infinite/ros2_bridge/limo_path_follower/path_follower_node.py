# -*- coding: utf-8 -*-
"""ROS2 path follower node for LIMO robot.

Author: Suwon Lee, Kookmin University
Created: 2026-03-01
Description: Subscribes to /wheel/odom, computes VFG + LPV-Hinf (or PID-FF)
    steering, and publishes cmd_vel_raw (routed through estop_cli.py to
    /cmd_vel).  Converts front-wheel steering angle to yaw-rate:
    omega = v * tan(delta) / L.

    The reference path is taken at runtime from a nav_msgs/msg/Path
    subscription (default topic ``/reference_path``). The topic uses a
    transient-local QoS so a one-shot publisher is sufficient: the most
    recent message is replayed to late subscribers.

    For smoke tests the original hardcoded ``StepCurvaturePath`` demo can
    be re-enabled by setting the ``use_demo_path`` parameter to True.
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, Bool
from rcl_interfaces.msg import SetParametersResult

from vfg_pathfollowing import (
    BezierPath,
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
        self.declare_parameter('reference_path_topic', '/reference_path')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('use_demo_path', False)

        # -- Read parameters --------------------------------------------
        ctrl_type = self.get_parameter('controller_type').value
        self.v_const = min(self.get_parameter('v_const').value, 3.0)  # LIMO max ~3 m/s
        self._k_e = self.get_parameter('k_e').value
        self.dt_ctrl = self.get_parameter('dt_ctrl').value
        self.wheelbase = self.get_parameter('wheelbase').value
        K_P = self.get_parameter('K_P').value
        K_D = self.get_parameter('K_D').value
        ref_topic = self.get_parameter('reference_path_topic').value
        self._odom_frame = self.get_parameter('odom_frame').value
        use_demo = bool(self.get_parameter('use_demo_path').value)

        # -- Path (runtime; demo only when explicitly requested) --------
        if use_demo:
            self.path = StepCurvaturePath(L1=5.0, R=0.5, theta_arc=np.pi / 2, L2=25.0)
            self.guidance = VectorFieldGuidance(self.path, k_e=self._k_e)
            self.get_logger().warn(
                'use_demo_path=true: using hardcoded StepCurvaturePath. '
                'Publish to %s to override.' % ref_topic)
        else:
            self.path = None
            self.guidance = None

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
            f'k_e={self._k_e:.1f}, dt={self.dt_ctrl:.3f} s, L={self.wheelbase:.3f} m'
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

        latched_qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.sub_path = self.create_subscription(
            Path, ref_topic, self._path_cb, latched_qos)

        # Reset sub: True clears the loaded path, returning the node to idle.
        # Used by the battle-station "Stop scenario" action.
        self.sub_reset = self.create_subscription(
            Bool, '/path_follower/reset', self._reset_cb, 10)

        # Allow runtime tuning of v_const via /path_follower_node/set_parameters.
        # Without this callback, ros2 param set updates the parameter store but
        # self.v_const stays frozen at the constructor-time value.
        self.add_on_set_parameters_callback(self._params_cb)

        self.pub_cmd = self.create_publisher(Twist, 'cmd_vel_raw', 10)
        # Telemetry for the battle-station UI. Layout (Float32MultiArray):
        #   [x, y, yaw, v, s_star, total_length, kappa, rho, e_psi, delta_cmd, has_path]
        # has_path is 1.0 once a reference is loaded, else 0.0.
        self.pub_status = self.create_publisher(
            Float32MultiArray, '/path_follower/status', 10)
        self._status_labels = [
            'x', 'y', 'yaw', 'v', 's_star', 'total_length',
            'kappa', 'rho', 'e_psi', 'delta_cmd', 'has_path',
        ]

        timer_period = self.dt_ctrl  # seconds
        self.timer = self.create_timer(timer_period, self._control_cb)

        if self.path is None:
            self.get_logger().info(
                f'Path follower node started. Waiting for reference path on {ref_topic}.')
        else:
            self.get_logger().info('Path follower node started.')

    # -----------------------------------------------------------------
    # Callbacks
    # -----------------------------------------------------------------

    def _params_cb(self, params):
        for p in params:
            if p.name == 'v_const':
                v = float(p.value)
                v = max(0.0, min(3.0, v))
                self.v_const = v
                self.get_logger().info(f'v_const updated to {v:.2f} m/s (runtime)')
        return SetParametersResult(successful=True)

    def _reset_cb(self, msg: Bool):
        if msg.data and self.path is not None:
            self.path = None
            self.guidance = None
            self._delta_prev = 0.0
            self.get_logger().info('Path cleared via /path_follower/reset')

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

    def _path_cb(self, msg: Path):
        """Build a BezierPath from a runtime nav_msgs/Path reference."""
        # Frame consistency check: VFG works in odom (or whatever frame the
        # /wheel/odom poses live in). A mismatch will not be auto-corrected.
        if msg.header.frame_id and msg.header.frame_id != self._odom_frame:
            self.get_logger().warn(
                f"Reference path frame_id='{msg.header.frame_id}' does not match "
                f"odom_frame='{self._odom_frame}'. No transform is applied; "
                "tracking will be wrong unless they match.")

        waypoints = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if len(waypoints) < 2:
            self.get_logger().warn(
                f'Reference path has {len(waypoints)} waypoint(s); need >= 2. Ignoring.')
            return

        try:
            new_path = BezierPath(waypoints)
        except Exception as exc:
            self.get_logger().error(f'Failed to build BezierPath from reference: {exc}')
            return

        # Atomic swap: rclpy default executor is single-threaded so _control_cb
        # cannot interleave between these two assignments.
        self.path = new_path
        self.guidance = VectorFieldGuidance(new_path, k_e=self._k_e)
        self._delta_prev = 0.0

        self.get_logger().info(
            f'Loaded reference path: {len(waypoints)} waypoints, '
            f'total length {new_path.total_length:.2f} m.')

    def _publish_status(self, s_star=0.0, total_length=0.0, kappa=0.0,
                        rho=0.0, e_psi=0.0, delta_cmd=0.0, has_path=0.0):
        msg = Float32MultiArray()
        dim = MultiArrayDimension()
        dim.label = ','.join(self._status_labels)
        dim.size = len(self._status_labels)
        dim.stride = len(self._status_labels)
        msg.layout.dim.append(dim)
        msg.data = [
            float(self._x), float(self._y), float(self._yaw), float(self._v),
            float(s_star), float(total_length), float(kappa), float(rho),
            float(e_psi), float(delta_cmd), float(has_path),
        ]
        self.pub_status.publish(msg)

    def _control_cb(self):
        """Timer callback: compute and publish control command."""
        cmd = Twist()

        # Safety: no reference path yet -> zero velocity
        if self.path is None or self.guidance is None:
            self.pub_cmd.publish(cmd)
            self._publish_status(has_path=0.0)
            return

        # Safety: check odom timeout (0.5 s)
        if self._odom_stamp is None:
            self.pub_cmd.publish(cmd)  # zero velocity
            self._publish_status(total_length=self.path.total_length, has_path=1.0)
            return

        dt_since_odom = (self.get_clock().now() - self._odom_stamp).nanoseconds * 1e-9
        if dt_since_odom > 0.5:
            self.get_logger().warn(
                f'Odometry timeout ({dt_since_odom:.2f} s). Sending zero velocity.')
            self._delta_prev = 0.0
            self.pub_cmd.publish(cmd)
            self._publish_status(total_length=self.path.total_length, has_path=1.0)
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
            self._publish_status(s_star=s_star, total_length=self.path.total_length,
                                 kappa=kappa, has_path=1.0)
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
        self._publish_status(s_star=s_star, total_length=self.path.total_length,
                             kappa=kappa, rho=rho, e_psi=e_psi,
                             delta_cmd=delta_cmd, has_path=1.0)


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
