# -*- coding: utf-8 -*-
# hw-confirmed 2026-05-26 (base_gnss live): limo_base_node exposes NO odom-reset
# service — `ros2 service list` shows only MAVROS reset/clear services, none on
# limo_base. So this overlay is the chosen L3 mechanism, not a fallback. It was
# verified on the robot: zeroed mirrors raw before reset, reads (0,0,0) at the
# reset pose, and is SE(2)-relative after, with raw /wheel/odom left untouched.
"""Odom-zeroing overlay (ROC L3).

Republishes raw wheel odometry through a latched SE(2) offset so the
downstream follower sees a pose that reads (0, 0, 0) at a commanded physical
point — without restarting the GNSS/RTK stack (which a hard odom reset at the
driver level might require).

Topology (see system_spec §4 interface contract):

  sub  /wheel/odom         nav_msgs/Odometry   raw wheel odom from base
  pub  /wheel/odom_zeroed  nav_msgs/Odometry   re-anchored odom for follower
  sub  /odom_zero/reset    std_msgs/Bool       True -> latch current raw pose
                                                as the new (0,0,0) origin

The raw /wheel/odom is left untouched, so the bag still records it (L2/L5
recording is unaffected). The follower consumes the zeroed stream via a launch
remap applied at the orchestrator PROCS entry (it does not edit the follower's
hardcoded '/wheel/odom' subscription) — see orchestrator_node.py.

SE(2) offset math
-----------------
At a reset, we latch the current raw pose as the origin O = (x0, y0, yaw0).
For every subsequent raw pose P = (x, y, yaw) we express P in the frame of O:

  dx = x - x0
  dy = y - y0
  x'   =  cos(yaw0) * dx + sin(yaw0) * dy      # rotate by -yaw0
  y'   = -sin(yaw0) * dx + cos(yaw0) * dy
  yaw' = wrap(yaw - yaw0)

This is the rigid-body transform  P' = R(-yaw0) * (P - O), i.e. the inverse of
the latched origin pose composed with the current pose. At the reset instant
P == O so (x', y', yaw') == (0, 0, 0) exactly.

The twist is reported in the child (body) frame by REP-145 / the LIMO base, and
the body frame is unchanged by re-anchoring the fixed parent frame, so the
twist is copied through verbatim. (If a future source reports twist in the
parent frame, it would need the same R(-yaw0) rotation applied to its linear
component — guarded by TODO below.)
"""

import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


def _yaw_from_quaternion(q):
    """Extract yaw (rad) from a geometry_msgs/Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _quaternion_from_yaw(yaw):
    """Build a (x, y, z, w) quaternion for a planar yaw rotation."""
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


def _wrap(angle):
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class OdomZeroNode(Node):

    def __init__(self):
        super().__init__('odom_zero_node')

        # Latched SE(2) origin. Until the first reset the offset is identity,
        # so /wheel/odom_zeroed mirrors /wheel/odom 1:1.
        self._x0 = 0.0
        self._y0 = 0.0
        self._yaw0 = 0.0
        self._has_reset = False  # informational only
        self._last_raw = None  # (x, y, yaw) of the most recent raw odom

        # Whether the upstream twist is expressed in the body frame. The LIMO
        # base / REP-145 convention is body frame -> no rotation needed.
        # TODO(hw-verify): confirm /wheel/odom twist frame on the robot. If the
        # base ever reports twist in the parent/odom frame, set this False so
        # the linear twist is rotated by R(-yaw0) like the position.
        self._twist_in_body_frame = True

        self.sub_odom = self.create_subscription(
            Odometry, '/wheel/odom', self._odom_cb, 10)
        self.sub_reset = self.create_subscription(
            Bool, '/odom_zero/reset', self._reset_cb, 10)
        self.pub_odom = self.create_publisher(
            Odometry, '/wheel/odom_zeroed', 10)

        self.get_logger().info(
            'odom_zero_node started. Republishing /wheel/odom -> '
            '/wheel/odom_zeroed (identity offset until first '
            '/odom_zero/reset). NOTE: prefer a native limo_base odom-reset '
            'service if one exists (see header TODO).')

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _reset_cb(self, msg: Bool):
        """Latch the most recent raw pose as the new (0,0,0) origin.

        Acts only on a True message; a False is ignored so the topic can be
        used as a one-shot trigger without unlatching.
        """
        if not msg.data:
            self.get_logger().info('/odom_zero/reset received False; ignoring.')
            return
        if self._last_raw is None:
            self.get_logger().warn(
                '/odom_zero/reset received but no /wheel/odom seen yet; '
                'cannot latch an origin. Ignoring.')
            return
        self._x0, self._y0, self._yaw0 = self._last_raw
        self._has_reset = True
        self.get_logger().info(
            f'Odom origin latched at raw pose '
            f'(x={self._x0:.3f}, y={self._y0:.3f}, yaw={self._yaw0:.4f} rad). '
            f'/wheel/odom_zeroed will read (0, 0, 0) here.')

    def _odom_cb(self, msg: Odometry):
        """Re-anchor the raw pose by the latched SE(2) offset and republish."""
        raw_x = msg.pose.pose.position.x
        raw_y = msg.pose.pose.position.y
        raw_yaw = _yaw_from_quaternion(msg.pose.pose.orientation)
        # Remember the latest raw pose so a reset can latch it.
        self._last_raw = (raw_x, raw_y, raw_yaw)

        # P' = R(-yaw0) * (P - O)
        dx = raw_x - self._x0
        dy = raw_y - self._y0
        c = math.cos(self._yaw0)
        s = math.sin(self._yaw0)
        x_z = c * dx + s * dy
        y_z = -s * dx + c * dy
        yaw_z = _wrap(raw_yaw - self._yaw0)

        out = Odometry()
        # Preserve stamp and frame_ids; only the values shift. Keeping the
        # parent frame_id ('odom') is correct: the offset re-defines the
        # numeric origin within that same frame, not a new TF frame.
        out.header = msg.header
        out.child_frame_id = msg.child_frame_id

        out.pose.pose.position.x = x_z
        out.pose.pose.position.y = y_z
        out.pose.pose.position.z = msg.pose.pose.position.z
        qx, qy, qz, qw = _quaternion_from_yaw(yaw_z)
        out.pose.pose.orientation.x = qx
        out.pose.pose.orientation.y = qy
        out.pose.pose.orientation.z = qz
        out.pose.pose.orientation.w = qw
        # Covariance is unchanged by a rigid SE(2) re-anchoring of the origin
        # (a pure translation + planar rotation of the mean). Pass through.
        out.pose.covariance = msg.pose.covariance

        if self._twist_in_body_frame:
            out.twist = msg.twist
        else:
            # Parent-frame twist: rotate the linear part by R(-yaw0); angular
            # (yaw rate) is invariant. (Not the LIMO default — see header.)
            lx = msg.twist.twist.linear.x
            ly = msg.twist.twist.linear.y
            out.twist.twist.linear.x = c * lx + s * ly
            out.twist.twist.linear.y = -s * lx + c * ly
            out.twist.twist.linear.z = msg.twist.twist.linear.z
            out.twist.twist.angular = msg.twist.twist.angular
            out.twist.covariance = msg.twist.covariance

        self.pub_odom.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = OdomZeroNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
