#!/usr/bin/env python3
"""High-level operator/LLM command facade for the LIMO field stack.

The node intentionally stays thin: it translates explicit operator commands
into the existing orchestrator, sequencer, and E-stop topics, and republishes a
single JSON status snapshot. It does not bypass the safety contract.
"""

import json
import time

import rclpy  # pyright: ignore[reportMissingImports]
from rclpy.node import Node  # pyright: ignore[reportMissingImports]
from std_msgs.msg import Bool, String  # pyright: ignore[reportMissingImports]


class OpsNode(Node):

    def __init__(self):
        super().__init__('limo_ops_node')
        self._orch = {}
        self._exp = {}
        self._rtk = ''
        self._last_cmd = None
        self._last_result = 'loaded'

        self.pub_status = self.create_publisher(String, '/ops/status', 10)
        self.pub_orch_start = self.create_publisher(String, '/orchestrator/start', 10)
        self.pub_orch_kill = self.create_publisher(String, '/orchestrator/kill', 10)
        self.pub_exp_cmd = self.create_publisher(String, '/experiment/cmd', 10)
        self.pub_estop = self.create_publisher(Bool, '/estop_trigger', 10)

        self.create_subscription(String, '/ops/cmd', self._on_cmd, 10)
        self.create_subscription(String, '/orchestrator/status', self._on_orch, 10)
        self.create_subscription(String, '/experiment/status', self._on_exp, 10)
        self.create_subscription(
            String, '/gps_rtk_f9p_helical/gps/rtk_status', self._on_rtk, 10)
        self.create_timer(1.0, self._publish_status)
        self._publish_status()

    def _on_orch(self, msg):
        try:
            self._orch = json.loads(msg.data) or {}
        except (ValueError, TypeError):
            pass

    def _on_exp(self, msg):
        try:
            self._exp = json.loads(msg.data) or {}
        except (ValueError, TypeError):
            pass

    def _on_rtk(self, msg):
        self._rtk = msg.data or ''

    def _on_cmd(self, msg):
        try:
            payload = json.loads(msg.data)
        except (ValueError, TypeError):
            payload = {'action': str(msg.data or '').strip()}
        action = str(payload.get('action', '')).lower().strip()
        self._last_cmd = action

        if action == 'status':
            self._last_result = 'status published'
        elif action == 'validate_stack':
            self._last_result = self._validate_stack()
        elif action == 'run_smoke_e2e':
            if not bool(payload.get('confirmed_wheels_on_floor', False)):
                self._last_result = (
                    'refused run_smoke_e2e: confirmed_wheels_on_floor required')
            else:
                self._exp_cmd('start')
                self._last_result = 'sent experiment start'
        elif action == 'restart_rtk_receiver':
            self._orch_kill('base_gnss')
            self._orch_start('base_gnss')
            self._last_result = 'restarted base_gnss stack'
        elif action == 'force_base_resurvey':
            self._last_result = (
                'base resurvey is forced by restarting rtk-base.service on the Pi')
        elif action == 'recover_after_failure':
            self._estop(True)
            self._orch_kill('follower')
            self._orch_kill('reposition')
            self._exp_cmd('pause')
            self._last_result = 'safe recovery: estop latched, movers killed, sequencer paused'
        elif action == 'stop_all_motion':
            self._estop(True)
            self._orch_kill('follower')
            self._orch_kill('reposition')
            self._last_result = 'motion stopped: estop latched, follower/reposition killed'
        else:
            self._last_result = f'unknown action: {action}'
        self._publish_status()

    def _validate_stack(self):
        detail = self._orch.get('_detail') or {}
        required = ('base_gnss', 'estop', 'odom_zero', 'sequencer')
        missing = [name for name in required if not bool(self._orch.get(name, False))]
        crashed = [
            name for name, d in detail.items()
            if isinstance(d, dict)
            and d.get('last_exit_code') not in (None, 0)
            and not d.get('running')
        ]
        if missing or crashed:
            return f'validate_stack FAIL missing={missing} crashed={crashed}'
        return 'validate_stack PASS'

    def _orch_start(self, name):
        self.pub_orch_start.publish(String(data=name))

    def _orch_kill(self, name):
        self.pub_orch_kill.publish(String(data=name))

    def _exp_cmd(self, action):
        self.pub_exp_cmd.publish(String(data=json.dumps({'action': action})))

    def _estop(self, value):
        self.pub_estop.publish(Bool(data=bool(value)))

    def _publish_status(self):
        msg = String()
        msg.data = json.dumps({
            'stamp_monotonic': round(time.monotonic(), 3),
            'last_cmd': self._last_cmd,
            'last_result': self._last_result,
            'rtk_status': self._rtk,
            'orchestrator': self._orch,
            'experiment': self._exp,
        })
        self.pub_status.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OpsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
