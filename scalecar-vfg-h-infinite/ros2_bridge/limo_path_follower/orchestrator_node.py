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
from datetime import datetime, timezone

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
    # T3 (L3): the odom-zeroing overlay. Subscribes raw /wheel/odom,
    # republishes /wheel/odom_zeroed re-anchored to the latched origin; the
    # sequencer latches a fresh origin by publishing True to /odom_zero/reset
    # just before each recorded leg. Raw /wheel/odom is left intact so the bag
    # still records true wheel odom.
    'odom_zero': [
        'ros2', 'run', 'limo_path_follower', 'odom_zero_node',
    ],
    # The follower's odom subscription is hardcoded to '/wheel/odom'
    # (path_follower_node.py:121). We do NOT edit that hardcode; instead we
    # remap it onto the zeroed stream at launch so the controller tracks the
    # analytic curve from a freshly-zeroed (0,0,0) origin (analytic paths start
    # at origin heading +x). The '--ros-args -r' remap rewrites only this
    # process's subscription; the raw topic and the bag recording of it are
    # unaffected. Run 'odom_zero' before 'follower'.
    'follower': [
        'ros2', 'run', 'limo_path_follower', 'path_follower_node',
        '--ros-args', '-r', '/wheel/odom:=/wheel/odom_zeroed',
    ],
    # T4 (R1-R4, P3): RTK go-to-pose between recorded runs. Publishes cmd_vel_raw
    # only while repositioning. MUST NOT run concurrently with 'follower' (C6) —
    # the sequencer enforces exactly one cmd_vel_raw publisher.
    'reposition': [
        'ros2', 'run', 'limo_path_follower', 'reposition_node',
    ],
    # T6: the experiment sequencer (the integrator). Drives the matrix unattended
    # by start/kill-ing the movers above through this orchestrator.
    'sequencer': [
        'ros2', 'run', 'limo_path_follower', 'experiment_sequencer_node',
    ],
    # T6 smoke variant: the SAME sequencer pinned to the 1-cell field smoke
    # config (scenarios/smoke.yaml) instead of the full matrix (experiment.yaml,
    # the autostart-false default). This is the deterministic "shortest full
    # e2e" a fresh operator/LLM session arms via ops_node run_smoke_e2e.
    # autostart stays false (idles until armed). Mutually exclusive with
    # 'sequencer' — ops_node kills one before starting the other.
    'sequencer_smoke': [
        'ros2', 'run', 'limo_path_follower', 'experiment_sequencer_node',
        '--ros-args', '-p',
        'experiment_yaml:=/home/agilex/H-infinity/scenarios/smoke.yaml',
    ],
    'ops': [
        'ros2', 'run', 'limo_path_follower', 'ops_node',
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
        self.meta = {
            name: {
                'pid': None,
                'last_exit_code': None,
                'last_started_at': None,
                'last_stopped_at': None,
                'intentional_stop_reason': None,
            }
            for name in PROCS
        }

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
                    self._kill_one(ex, reason=f"exclusive switch to {name}")

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
        self.meta[name].update({
            'pid': p.pid,
            'last_exit_code': None,
            'last_started_at': _utc_now(),
            'last_stopped_at': None,
            'intentional_stop_reason': None,
        })
        self.get_logger().info(f"start: spawned '{name}' (pid {p.pid}); log {log_path}")
        self._publish_status()

    def _on_kill(self, msg: String):
        name = (msg.data or '').strip()
        if name not in PROCS:
            self.get_logger().warn(f"kill: unknown name '{name}'")
            return
        self._kill_one(name, reason='operator kill')
        self._publish_status()

    # ------------------------------------------------------------------
    # Internals
    # ------------------------------------------------------------------

    def _alive(self, name):
        p = self.children.get(name)
        return p is not None and p.poll() is None

    def _kill_one(self, name, reason='orchestrator stop'):
        p = self.children.get(name)
        if p is None or p.poll() is not None:
            self._record_exit(name, reason=reason)
            self._close_log(name)
            return
        self.meta[name]['intentional_stop_reason'] = reason
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
        self._record_exit(name, reason=reason)
        self.get_logger().info(f"kill: '{name}' stopped")

    def _close_log(self, name):
        f = self.log_files.pop(name, None)
        if f is not None:
            try:
                f.close()
            except Exception:
                pass

    def _publish_status(self):
        detail = {}
        state = {}
        for name in PROCS:
            running = self._alive(name)
            if not running:
                self._record_exit(name)
            state[name] = running
            meta = dict(self.meta.get(name, {}))
            meta.update({
                'running': running,
                'pid': meta.get('pid'),
                'log_tail': self._log_tail(name),
            })
            detail[name] = meta
        state['_detail'] = detail
        msg = String()
        msg.data = json.dumps(state)
        self.pub_status.publish(msg)

    def shutdown(self):
        self.get_logger().info('Orchestrator shutting down — killing all children')
        for name in list(self.children):
            self._kill_one(name, reason='orchestrator shutdown')

    def _record_exit(self, name, reason=None):
        p = self.children.get(name)
        if p is None:
            return
        rc = p.poll()
        if rc is None:
            return
        meta = self.meta[name]
        meta['pid'] = p.pid
        meta['last_exit_code'] = rc
        if meta.get('last_stopped_at') is None:
            meta['last_stopped_at'] = _utc_now()
        if reason and meta.get('intentional_stop_reason') is None:
            meta['intentional_stop_reason'] = reason

    def _log_tail(self, name, max_lines=8, max_chars=1200):
        path = os.path.join(LOG_DIR, f'{name}.log')
        f = self.log_files.get(name)
        if f is not None:
            try:
                f.flush()
            except Exception:
                pass
        try:
            with open(path, 'r', encoding='utf-8', errors='replace') as fh:
                lines = fh.readlines()[-max_lines:]
        except OSError:
            return ''
        return ''.join(lines)[-max_chars:]


def _utc_now():
    return datetime.now(timezone.utc).isoformat()


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
