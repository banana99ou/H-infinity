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

import json
import math
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, Bool, Float32, String
from rcl_interfaces.msg import SetParametersResult

from vfg_pathfollowing import (
    BezierPath,
    StepCurvaturePath,
    SlalomPath,
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
        # Minimum geometric turn radius [m]. Used as the default arc radius for
        # the "uturn" recipe. Spec §5 gives R_min ~= 0.37 m (firmware 1.0 m/s
        # cap + Ackermann geometry).
        # TODO(hw-verify): confirm the achievable R_min on the LIMO at v_const
        #   (Part C checklist item 4 — reposition/turn tolerances). 0.37 is the
        #   spec geometric figure, not yet measured on hardware.
        self.declare_parameter('R_min', 0.37)
        # How densely to sample analytic recipe paths when republishing them as
        # nav_msgs/Path on /reference_path (for viz + bag). Points per metre.
        self.declare_parameter('path_sample_density', 10.0)

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
        self._R_min = float(self.get_parameter('R_min').value)
        self._path_sample_density = float(
            self.get_parameter('path_sample_density').value)

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

        # -- Controller (live-switchable via the controller_type param) --
        self._K_P, self._K_D = K_P, K_D
        self.controller, self._ctrl_type = self._build_controller(ctrl_type)

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
        self._done_latched = False  # whether /path_follower/done is currently True
        # Source of the currently-loaded path: None | 'recipe' | 'bezier'.
        # Guards _path_cb against re-ingesting the analytic curve we publish on
        # /reference_path for viz (which would re-spline it to Bezier, undoing
        # the exact-curvature delivery of P4).
        self._path_source = None

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

        # Analytic-recipe sub (P1-P4): a JSON String describing an exact curve
        # (step / slalom / uturn). Latched so a one-shot publisher (sequencer)
        # is sufficient; the most recent recipe is replayed to late joiners.
        self.sub_recipe = self.create_subscription(
            String, '/reference_path_recipe', self._recipe_cb, latched_qos)

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

        # Crisp completion edge (O3). Latched so the sequencer reads the final
        # state even if it subscribes after the edge fires.
        self.pub_done = self.create_publisher(
            Bool, '/path_follower/done', latched_qos)
        # Per-control-cycle wall time of the guidance+controller block, in
        # milliseconds (A3 compute-cost). Best-effort, high-rate telemetry.
        self.pub_timing = self.create_publisher(
            Float32, '/path_follower/timing', 10)
        # Republish the loaded analytic curve, sampled, for viz + bag (latched
        # so it survives for late subscribers / bag start).
        self.pub_refpath = self.create_publisher(
            Path, ref_topic, latched_qos)

        # Seed the latched done topic with an explicit False so a fresh
        # subscriber sees a definite "not done". Bypasses the edge-guard in
        # _publish_done (which would no-op since _done_latched is already
        # False) by publishing directly once at startup.
        _seed = Bool()
        _seed.data = False
        self.pub_done.publish(_seed)

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

    def _build_controller(self, ctrl_type):
        """Build the steering controller for a controller_type string and return
        (controller, canonical_name). Used at startup and for live switching via
        the controller_type param (the sequencer flips lpv-hinf<->pid per cell)."""
        key = (ctrl_type or '').lower().strip()
        if key in ('lpv-hinf', 'lpv_hinf', 'lpv', 'hinf'):
            return LPVHinfController.default(dt=self.dt_ctrl), 'lpv-hinf'
        elif key in ('pid-ff', 'pid_ff', 'pid'):
            return PIDFeedforward(K_P=self._K_P, K_D=self._K_D, L=self.wheelbase), 'pid-ff'
        raise ValueError(f"Unknown controller_type '{ctrl_type}'. "
                         f"Choose from: 'lpv-hinf', 'pid-ff'")

    def _params_cb(self, params):
        for p in params:
            if p.name == 'v_const':
                v = float(p.value)
                v = max(0.0, min(3.0, v))
                self.v_const = v
                self.get_logger().info(f'v_const updated to {v:.2f} m/s (runtime)')
            elif p.name == 'controller_type':
                try:
                    new_ctrl, name = self._build_controller(p.value)
                except ValueError as exc:
                    self.get_logger().error(str(exc))
                    return SetParametersResult(successful=False, reason=str(exc))
                self.controller = new_ctrl
                self._ctrl_type = name
                self._delta_prev = 0.0  # drop stale smoothing state across a switch
                self.get_logger().info(f"controller_type switched to '{name}' (runtime)")
        return SetParametersResult(successful=True)

    def _reset_cb(self, msg: Bool):
        if msg.data and self.path is not None:
            self.path = None
            self.guidance = None
            self._path_source = None
            self._delta_prev = 0.0
            self._publish_done(False)
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
        """Build a BezierPath from a runtime nav_msgs/Path reference (P5)."""
        # Ignore our own analytic-curve republish: when a recipe path is active
        # we sample it onto /reference_path for viz, and that message arrives
        # right back here on the latched sub. Re-splining it to a Bezier would
        # round off the exact curvature step (defeats P4). A new Bezier input
        # is only honoured once the analytic path is cleared (reset/new recipe).
        if self._path_source == 'recipe':
            self.get_logger().debug(
                'Ignoring /reference_path while an analytic recipe is loaded '
                '(this is our own viz republish).')
            return

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
        self._path_source = 'bezier'
        self._delta_prev = 0.0
        self._publish_done(False)

        self.get_logger().info(
            f'Loaded reference path: {len(waypoints)} waypoints, '
            f'total length {new_path.total_length:.2f} m.')

    # -----------------------------------------------------------------
    # Analytic recipe (P1-P4)
    # -----------------------------------------------------------------

    def build_path_from_recipe(self, d):
        """Map a recipe dict to an analytic PathBase subclass.

        Schema::

            {"type": "step"|"slalom"|"uturn", "params": { ... }}

        - type "step"   -> StepCurvaturePath(L1, R, theta_arc, L2, direction)
        - type "slalom" -> SlalomPath(R, theta_arc, L1, L_mid, n_arcs, L_end)
        - type "uturn"  -> StepCurvaturePath(theta_arc=pi, R=R_min, ...):
              a semicircle (P3 U-turn). R defaults to the node's R_min param.

        All params are optional; the path classes supply defaults. Unknown
        params are ignored by the constructors' explicit signatures, so we pass
        only the keys each class accepts. All three curves start at the origin
        heading +x, so a freshly-zeroed odom aligns the curve to the robot.

        Returns a PathBase instance. Raises ValueError on unknown type.
        """
        if not isinstance(d, dict):
            raise ValueError(f'recipe must be a JSON object, got {type(d).__name__}')

        ptype = str(d.get('type', '')).lower().strip()
        params = d.get('params', {}) or {}
        if not isinstance(params, dict):
            raise ValueError("recipe 'params' must be a JSON object")

        def _f(key, default):
            return float(params.get(key, default))

        def _i(key, default):
            return int(params.get(key, default))

        if ptype == 'step':
            return StepCurvaturePath(
                L1=_f('L1', 5.0),
                R=_f('R', 0.5),
                theta_arc=_f('theta_arc', math.pi / 2),
                L2=_f('L2', 5.0),
                direction=_i('direction', 1),
            )
        elif ptype == 'slalom':
            return SlalomPath(
                R=_f('R', 0.5),
                theta_arc=_f('theta_arc', math.pi / 2),
                L1=_f('L1', 5.0),
                L_mid=_f('L_mid', 2.0),
                n_arcs=_i('n_arcs', 6),
                L_end=_f('L_end', 25.0),
            )
        elif ptype == 'uturn':
            # Semicircle U-turn (P3): theta_arc fixed at pi; R defaults to the
            # node's R_min param but may be overridden in params.
            return StepCurvaturePath(
                L1=_f('L1', 1.0),
                R=_f('R', self._R_min),
                theta_arc=math.pi,
                L2=_f('L2', 1.0),
                direction=_i('direction', 1),
            )
        else:
            raise ValueError(
                f"unknown recipe type '{ptype}'; "
                "expected 'step', 'slalom', or 'uturn'")

    def _recipe_cb(self, msg: String):
        """Load an exact analytic curve from a JSON recipe (P1-P4)."""
        try:
            d = json.loads(msg.data)
        except (ValueError, TypeError) as exc:
            self.get_logger().error(f'Failed to parse recipe JSON: {exc}')
            return

        # Explicit clear: run_executor latches {"type": "none"} before each
        # follower (re)spawn so a fresh subscriber replays a harmless no-op
        # instead of the PREVIOUS leg's curve (which would start this node
        # driving at default params before set_parameters/the real recipe).
        if isinstance(d, dict) and str(d.get('type', '')).lower().strip() == 'none':
            if self.path is not None:
                self.get_logger().info('Recipe "none": clearing loaded path.')
            self.path = None
            self.guidance = None
            self._path_source = None
            self._delta_prev = 0.0
            self._publish_done(False)
            return

        try:
            new_path = self.build_path_from_recipe(d)
        except Exception as exc:
            self.get_logger().error(f'Failed to build path from recipe: {exc}')
            return

        # Atomic swap: rclpy default executor is single-threaded, so _control_cb
        # cannot interleave between these assignments. Same pattern as _path_cb.
        self.path = new_path
        self.guidance = VectorFieldGuidance(new_path, k_e=self._k_e)
        self._path_source = 'recipe'
        self._delta_prev = 0.0
        self._publish_done(False)

        self.get_logger().info(
            f"Loaded analytic recipe '{d.get('type')}': "
            f'total length {new_path.total_length:.2f} m.')

        # Sample the analytic curve and republish on /reference_path for viz +
        # bag. Our own _path_cb ignores this while _path_source == 'recipe'.
        self._publish_sampled_path(new_path)

    def _publish_sampled_path(self, path):
        """Sample an analytic path onto a nav_msgs/Path and publish it."""
        msg = Path()
        msg.header.frame_id = self._odom_frame
        msg.header.stamp = self.get_clock().now().to_msg()

        total = float(path.total_length)
        n = max(2, int(math.ceil(total * self._path_sample_density)) + 1)
        for i in range(n):
            s = total * i / (n - 1)
            xy = path.position(s)
            ps = PoseStamped()
            ps.header.frame_id = self._odom_frame
            ps.header.stamp = msg.header.stamp
            ps.pose.position.x = float(xy[0])
            ps.pose.position.y = float(xy[1])
            hdg = float(path.heading(s))
            ps.pose.orientation.z = math.sin(hdg / 2.0)
            ps.pose.orientation.w = math.cos(hdg / 2.0)
            msg.poses.append(ps)
        self.pub_refpath.publish(msg)

    def _publish_done(self, value):
        """Publish /path_follower/done only on a state change (edge)."""
        value = bool(value)
        if value == self._done_latched:
            return
        self._done_latched = value
        m = Bool()
        m.data = value
        self.pub_done.publish(m)

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

        # No reference path yet -> stay silent on /cmd_vel_raw so a teleop
        # source (e.g. the battle-station keyboard) can drive the robot
        # without racing our 20 Hz zero-publish at this same topic. The
        # estop_cli safety chain still gates /cmd_vel; if no one publishes
        # to /cmd_vel_raw, /cmd_vel naturally goes silent.
        if self.path is None or self.guidance is None:
            self._publish_status(has_path=0.0)
            return

        # Safety: check odom timeout (0.5 s) -> zero velocity
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

        # -- Timed guidance + controller block (A3 compute-cost) -------
        # Wrap the per-cycle compute (guidance -> end-check -> controller) in
        # a monotonic timer; publish elapsed ms on /path_follower/timing.
        t0 = time.perf_counter_ns()

        # -- Guidance --------------------------------------------------
        q = np.array([self._x, self._y])
        gresult = self.guidance.compute(q, self._yaw)

        psi_des = gresult['psi_des']
        kappa = gresult['kappa']
        s_star = gresult['s_star']

        # Check path end: stop if within 0.3 m of the end
        if s_star >= self.path.total_length - 0.3:
            self._publish_timing(t0)
            if not self._done_latched:
                self.get_logger().info('Reached path end. Stopping.')
            self._delta_prev = 0.0
            self.pub_cmd.publish(cmd)
            # Crisp completion edge (O3) for the sequencer.
            self._publish_done(True)
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

        self._publish_timing(t0)

        cmd.linear.x = v
        cmd.angular.z = omega
        self.pub_cmd.publish(cmd)
        self._publish_status(s_star=s_star, total_length=self.path.total_length,
                             kappa=kappa, rho=rho, e_psi=e_psi,
                             delta_cmd=delta_cmd, has_path=1.0)

    def _publish_timing(self, t0_ns):
        """Publish elapsed wall time since t0_ns on /path_follower/timing [ms]."""
        elapsed_ms = (time.perf_counter_ns() - t0_ns) * 1e-6
        m = Float32()
        m.data = float(elapsed_ms)
        self.pub_timing.publish(m)


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
