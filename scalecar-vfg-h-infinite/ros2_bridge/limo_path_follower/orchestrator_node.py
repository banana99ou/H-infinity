# -*- coding: utf-8 -*-
"""ROS2 process supervisor for the LIMO battle-station.

Manages a fixed dictionary of named subprocesses (the LIMO base driver,
estop_cli, path_follower, etc.). Exposed as topics so rosbridge clients
(the battle-station HTML) can start/kill components without SSH access:

  /orchestrator/start  std_msgs/String   -- name of process to start
  /orchestrator/kill   std_msgs/String   -- name of process to kill
  /orchestrator/status std_msgs/String   -- JSON {name: bool_running, ...}

The supervisor itself + rosbridge_websocket are the only two things that
must be started by hand (or via tools/orchestrator/start_battle.sh).
"""

import json
import os
import shlex
import signal
import subprocess

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# Each entry is the bare command + args (excluding env sourcing). The
# orchestrator wraps the call in `bash -lc 'source ...; <cmd>'` so the
# child sees the ROS environment. base_vanilla and base_gnss are
# mutually exclusive — starting one auto-kills the other.
PROCS = {
    'base_vanilla': [
        'ros2', 'launch', 'limo_base', 'limo_base.launch.py',
    ],
    'base_gnss': [
        'ros2', 'launch', 'limo_base', 'LIMO+MAVROS+RTK_Node_Launcher.launch.py',
    ],
    'estop': [
        'python3', '/home/agilex/H-infinity/estop_cli.py', '--no-ping',
    ],
    'follower': [
        'ros2', 'run', 'limo_path_follower', 'path_follower_node',
    ],
}

EXCLUSIVE = {'base_vanilla', 'base_gnss'}

LOG_DIR = '/tmp/limo_orchestrator'

ROS_SOURCE = (
    'source /opt/ros/humble/setup.bash; '
    'source /home/agilex/agilex_ws/install/setup.bash; '
)


class OrchestratorNode(Node):

    def __init__(self):
        super().__init__('limo_orchestrator')
        os.makedirs(LOG_DIR, exist_ok=True)

        self.children = {}  # name -> Popen (alive or dead)
        self.log_files = {}  # name -> open file handle

        self.create_subscription(String, '/orchestrator/start', self._on_start, 10)
        self.create_subscription(String, '/orchestrator/kill', self._on_kill, 10)
        self.pub_status = self.create_publisher(String, '/orchestrator/status', 10)
        self.create_timer(1.0, self._publish_status)

        self.get_logger().info(
            f'Orchestrator started. Managed: {sorted(PROCS.keys())}. '
            f'Logs in {LOG_DIR}/.')
        self._publish_status()

    # ------------------------------------------------------------------
    # Public actions
    # ------------------------------------------------------------------

    def _on_start(self, msg: String):
        name = (msg.data or '').strip()
        if name not in PROCS:
            self.get_logger().warn(f"start: unknown name '{name}'")
            return
        if self._alive(name):
            self.get_logger().info(f"start: '{name}' already running, ignoring")
            return

        if name in EXCLUSIVE:
            for ex in EXCLUSIVE:
                if ex != name and self._alive(ex):
                    self.get_logger().info(
                        f"start: stopping exclusive '{ex}' before starting '{name}'")
                    self._kill_one(ex)

        cmd = PROCS[name]
        cmd_str = ' '.join(shlex.quote(a) for a in cmd)
        full = ROS_SOURCE + 'exec ' + cmd_str

        log_path = os.path.join(LOG_DIR, f'{name}.log')
        log = open(log_path, 'a')
        log.write(f'\n=== orchestrator spawning {name} ===\n')
        log.flush()

        try:
            p = subprocess.Popen(
                ['bash', '-lc', full],
                stdout=log, stderr=subprocess.STDOUT,
                preexec_fn=os.setsid,  # process group so we can kill the whole tree
            )
        except Exception as exc:
            self.get_logger().error(f"start '{name}' failed: {exc}")
            log.close()
            return

        self.children[name] = p
        self.log_files[name] = log
        self.get_logger().info(f"start: spawned '{name}' (pid {p.pid}); log {log_path}")
        self._publish_status()

    def _on_kill(self, msg: String):
        name = (msg.data or '').strip()
        if name not in PROCS:
            self.get_logger().warn(f"kill: unknown name '{name}'")
            return
        self._kill_one(name)
        self._publish_status()

    # ------------------------------------------------------------------
    # Internals
    # ------------------------------------------------------------------

    def _alive(self, name):
        p = self.children.get(name)
        return p is not None and p.poll() is None

    def _kill_one(self, name):
        p = self.children.get(name)
        if p is None or p.poll() is not None:
            self._close_log(name)
            return
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGTERM)
        except (ProcessLookupError, PermissionError) as exc:
            self.get_logger().warn(f"kill '{name}': SIGTERM failed: {exc}")
        try:
            p.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            self.get_logger().warn(f"kill '{name}': SIGTERM timed out, escalating to SIGKILL")
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGKILL)
            except (ProcessLookupError, PermissionError):
                pass
            try:
                p.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                pass
        self._close_log(name)
        self.get_logger().info(f"kill: '{name}' stopped")

    def _close_log(self, name):
        f = self.log_files.pop(name, None)
        if f is not None:
            try:
                f.close()
            except Exception:
                pass

    def _publish_status(self):
        state = {name: self._alive(name) for name in PROCS}
        msg = String()
        msg.data = json.dumps(state)
        self.pub_status.publish(msg)

    def shutdown(self):
        self.get_logger().info('Orchestrator shutting down — killing all children')
        for name in list(self.children):
            self._kill_one(name)


def main(args=None):
    rclpy.init(args=args)
    node = OrchestratorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
