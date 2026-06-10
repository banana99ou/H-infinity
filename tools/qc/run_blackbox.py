#!/usr/bin/env python3
"""Read-only black-box recorder for a rooftop validation run.

This is a LOGGING script for post-mortem analysis (its output is meant to be
read by an LLM, not watched live by a human). It NEVER publishes anything — it
sits entirely outside the safety chain (cmd_vel_raw -> estop_cli -> /cmd_vel)
and outside the control loop. It only subscribes, snapshots, and writes JSONL.

It folds in what tools/qc/repo_mon.py does (the fused-heading-vs-COG reposition
trace) and adds the run-level + follower-tracking + safety + proc-health story
that the rosbag does not capture. The rosbag remains the authoritative full-rate
source for the paper metrics; this is the supervisory flight recorder.

Run on the NUC (it needs ROS), after sourcing ROS + the workspace. No colcon
build needed — plain python3, like repo_mon.py:

    source /opt/ros/humble/setup.bash
    source /home/agilex/agilex_ws/install/setup.bash
    python3 ~/H-infinity/tools/qc/run_blackbox.py

By default it writes two files under "Experiment Data/monitor/":
  <base>.events.jsonl   — the narrative spine (meta, transitions, leg verdicts).
                          A few hundred lines; read this first, in full.
  <base>.samples.jsonl  — 5 Hz consolidated snapshots + computed flags. The
                          forensic detail; zoom in here on whatever the spine
                          flags.
where <base> = run_blackbox_<YYMMDD_HHMMSS>.

JSONL record types (every line has t=monotonic_s, wall=ISO8601, type=...):
  meta          once at startup: git commit, host, argv, thresholds, topics.
  topic_health  on a topic's present<->silent change, plus a snapshot every 30 s.
  event         immediate, on any tracked transition (phase, pause/abort, estop,
                rtk quality, proc exit, odom dropout, path done, reposition
                state, heading mode, a flag setting/clearing).
  sample        5 Hz consolidated latest-values snapshot with nested
                run/follow/heading/repo/rtk/cmd/flags blocks.
  leg_verdict   on leg completion: mirrors run_executor's classifier so the
                post-mortem gets the pass/fail call without opening the sidecar.

Each flag in a sample keeps the raw value beside it so a post-mortem can
re-threshold without re-running anything. Thresholds below are CLI-overridable.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import socket
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy,
                       QoSHistoryPolicy)
from std_msgs.msg import String, Bool, Float32, Float64, Float32MultiArray
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))

# --- thresholds (raw values are logged too, so these only gate boolean flags) ---
SAMPLE_HZ = 5.0
HEALTH_SNAPSHOT_S = 30.0
ODOM_SILENT_S = 0.5          # /wheel/odom gap > this => base-serial dropout (C5/F)
TOPIC_SILENT_S = 2.0         # generic per-topic silence for health
RTK_SCORED_GATE_Q = 4        # scored leg PASS needs FIXED(4); FLOAT(5) is below gate
HDG_STD_MAX_DEG = 15.0       # heading unstable above this while driving
HDG_MODE_OK = "GNSS_AIDED"
DELTA_SAT_RAD = 0.49         # steering clip is +/-0.5; near it = saturated
DELTA_SAT_S = 2.0            # sustained saturation duration to flag
STALL_V = 0.05               # m/s; "moving" threshold for the stall check
STALL_S = 3.0                # s_star not advancing while moving for this long
FUSED_COG_DISAGREE_DEG = 30.0
COG_MIN_TRAVEL_M = 0.03      # only trust COG after this much RTK travel
RTK_WINDOW_PCT = 95.0        # the scored-run FIXED% gate (mirror of run_executor)


def _ang_norm(d):
    """Wrap degrees to (-180, 180]."""
    return (d + 180.0) % 360.0 - 180.0


def _bearing(p0, p1):
    """Compass bearing p0->p1 (deg E-of-N) and ground distance (m)."""
    dE = (p1[1] - p0[1]) * 111320.0 * math.cos(math.radians(p0[0]))
    dN = (p1[0] - p0[0]) * 111320.0
    return (math.degrees(math.atan2(dE, dN)) + 360.0) % 360.0, math.hypot(dE, dN)


def _git_commit():
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=REPO_ROOT,
            stderr=subprocess.DEVNULL).decode().strip()
    except Exception:
        return None


# QoS profiles matching how the producer nodes publish.
LATCHED = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                     history=QoSHistoryPolicy.KEEP_LAST)
RELIABLE = QoSProfile(depth=20, reliability=QoSReliabilityPolicy.RELIABLE,
                      history=QoSHistoryPolicy.KEEP_LAST)
BEST_EFFORT = QoSProfile(depth=20, reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST)

# /path_follower/status Float32MultiArray index map (path_follower_node.py:168).
PF_LABELS = ['x', 'y', 'yaw', 'v', 's_star', 'total_length',
             'kappa', 'rho', 'e_psi', 'delta_cmd', 'has_path']


class BlackBox(Node):
    def __init__(self, args):
        super().__init__('run_blackbox')
        self.args = args
        self._t0 = time.monotonic()

        # output files
        ts = time.strftime('%y%m%d_%H%M%S')
        outdir = args.outdir
        os.makedirs(outdir, exist_ok=True)
        base = os.path.join(outdir, f'run_blackbox_{ts}')
        self._ev = open(base + '.events.jsonl', 'a', buffering=1)
        self._sm = open(base + '.samples.jsonl', 'a', buffering=1)
        self._base = base

        # latest-value state, snapshotted by the sample timer
        self.run = {}            # /run/status or /experiment/status (JSON)
        self.orch = {}           # /orchestrator/status (name -> running)
        self.orch_detail = {}    # _detail block
        self.follow = {k: None for k in PF_LABELS}
        self.timing_ms = None
        self.done = None
        self.heading = {}        # /heading/fused_status (JSON)
        self.repo = {}           # /reposition/status (JSON)
        self.rtk_q = None        # parsed quality int
        self.fix = None          # (lat, lon)
        self.prev_fix = None
        self.cog = None
        self.cog_travel = None
        self.cmd_raw = (None, None)
        self.cmd_final = (None, None)
        self.estop = None
        self.odom_reset = {}     # /odom_zero/status (JSON)

        # liveness bookkeeping: topic -> [count, last_monotonic]
        self._seen = {}
        # transition memory for events
        self._last = {}
        # flag timers
        self._delta_sat_since = None
        self._sstar_last = None
        self._sstar_rise_t = None
        # per-leg accumulators (reset on leg start)
        self._leg = self._new_leg_acc()
        self._last_phase = None
        self._last_health_snapshot = 0.0

        self._subscribe()
        self._emit('meta', {
            'git_commit': _git_commit(),
            'host': socket.gethostname(),
            'argv': sys.argv,
            'ros_domain_id': os.environ.get('ROS_DOMAIN_ID'),
            'thresholds': {
                'sample_hz': SAMPLE_HZ, 'odom_silent_s': ODOM_SILENT_S,
                'rtk_scored_gate_q': RTK_SCORED_GATE_Q,
                'hdg_std_max_deg': HDG_STD_MAX_DEG, 'hdg_mode_ok': HDG_MODE_OK,
                'delta_sat_rad': DELTA_SAT_RAD, 'delta_sat_s': DELTA_SAT_S,
                'stall_v': STALL_V, 'stall_s': STALL_S,
                'fused_cog_disagree_deg': FUSED_COG_DISAGREE_DEG,
                'rtk_window_pct': RTK_WINDOW_PCT,
            },
            'outfiles': [self._ev.name, self._sm.name],
        }, to_events=True)

        self.create_timer(1.0 / SAMPLE_HZ, self._tick)
        # minimal stdout heartbeat so the operator can confirm it's alive.
        print(f"[run_blackbox] recording -> {base}.{{events,samples}}.jsonl",
              flush=True)

    # ----- output helpers -----
    def _now(self):
        return time.monotonic() - self._t0

    def _emit(self, rtype, payload, to_events=False):
        rec = {'t': round(self._now(), 3),
               'wall': time.strftime('%Y-%m-%dT%H:%M:%S'),
               'type': rtype}
        rec.update(payload)
        line = json.dumps(rec, default=str)
        f = self._ev if to_events else self._sm
        f.write(line + '\n')

    def _event(self, evt, **kw):
        self._emit('event', {'evt': evt, **kw}, to_events=True)

    def _transition(self, key, value, evt, **extra):
        """Emit an event only when `value` changes vs the last seen for `key`."""
        prev = self._last.get(key, '__unset__')
        if prev != value:
            self._last[key] = value
            if prev != '__unset__':   # don't fire on the very first observation
                self._event(evt, **{'from': prev, 'to': value, **extra})
            else:
                self._event(evt, value=value, first=True, **extra)

    # ----- subscriptions -----
    def _subscribe(self):
        def mark(topic):
            c = self._seen.get(topic, [0, None])
            c[0] += 1
            c[1] = time.monotonic()
            self._seen[topic] = c

        def js(m):
            try:
                return json.loads(m.data) or {}
            except Exception:
                return {}

        # run spine: subscribe to both the new and legacy status topics.
        def on_run(m):
            mark('/run/status')
            self.run = js(m)
            self._on_run_status()
        self.create_subscription(String, '/run/status', on_run, LATCHED)

        def on_exp(m):
            mark('/experiment/status')
            # only adopt the legacy topic if the new one is silent
            if not self.run or '/run/status' not in self._seen:
                self.run = js(m)
                self._on_run_status()
        self.create_subscription(String, '/experiment/status', on_exp, LATCHED)

        def on_orch(m):
            mark('/orchestrator/status')
            d = js(m)
            self.orch_detail = d.pop('_detail', {}) if isinstance(d, dict) else {}
            self.orch = d
            self._on_orch_status()
        self.create_subscription(String, '/orchestrator/status', on_orch, RELIABLE)

        def on_pf(m):
            mark('/path_follower/status')
            data = list(m.data)
            for i, lbl in enumerate(PF_LABELS):
                if i < len(data):
                    self.follow[lbl] = float(data[i])
            self._accumulate_follow()
        self.create_subscription(Float32MultiArray, '/path_follower/status',
                                 on_pf, RELIABLE)

        def on_timing(m):
            mark('/path_follower/timing')
            self.timing_ms = float(m.data)
        self.create_subscription(Float32, '/path_follower/timing', on_timing,
                                 RELIABLE)

        def on_done(m):
            mark('/path_follower/done')
            self.done = bool(m.data)
            self._transition('pf_done', self.done, 'path_done')
        self.create_subscription(Bool, '/path_follower/done', on_done, LATCHED)

        def on_hdg(m):
            mark('/heading/fused_status')
            self.heading = js(m)
            self._transition('hdg_mode', self.heading.get('mode'),
                             'heading_mode')
        self.create_subscription(String, '/heading/fused_status', on_hdg,
                                 LATCHED)

        def on_repo(m):
            mark('/reposition/status')
            self.repo = js(m)
            self._transition('repo_state', self.repo.get('state'),
                             'reposition', reason=self.repo.get('reason'),
                             err_m=self.repo.get('err_m'),
                             err_deg=self.repo.get('err_deg'))
        self.create_subscription(String, '/reposition/status', on_repo, LATCHED)

        def on_rtk(m):
            mark('/gps_rtk_f9p_helical/gps/rtk_status')
            q = self._parse_q(m.data)
            self.rtk_q = q
            self._transition('rtk_q', q, 'rtk_quality')
        self.create_subscription(String, '/gps_rtk_f9p_helical/gps/rtk_status',
                                 on_rtk, RELIABLE)

        def on_fix(m):
            mark('/gps_rtk_f9p_helical/gps/fix')
            self.fix = (m.latitude, m.longitude)
            if self.prev_fix is not None:
                b, d = _bearing(self.prev_fix, self.fix)
                if d > COG_MIN_TRAVEL_M:
                    self.cog, self.cog_travel = b, d
            self.prev_fix = self.fix
        self.create_subscription(NavSatFix, '/gps_rtk_f9p_helical/gps/fix',
                                 on_fix, BEST_EFFORT)

        def on_cmd_raw(m):
            mark('/cmd_vel_raw')
            self.cmd_raw = (m.linear.x, m.angular.z)
        self.create_subscription(Twist, '/cmd_vel_raw', on_cmd_raw, RELIABLE)

        def on_cmd_final(m):
            mark('/cmd_vel')
            self.cmd_final = (m.linear.x, m.angular.z)
        self.create_subscription(Twist, '/cmd_vel', on_cmd_final, RELIABLE)

        def on_estop(m):
            mark('/estop')
            self.estop = bool(m.data)
            self._transition('estop', self.estop, 'estop')
        self.create_subscription(Bool, '/estop', on_estop, LATCHED)

        def on_odom(m):
            mark('/wheel/odom')      # count only; rate is the signal
        self.create_subscription(Odometry, '/wheel/odom', on_odom, RELIABLE)

        def on_odom_z(m):
            mark('/wheel/odom_zeroed')
        self.create_subscription(Odometry, '/wheel/odom_zeroed', on_odom_z,
                                 RELIABLE)

        def on_oz_status(m):
            mark('/odom_zero/status')
            self.odom_reset = js(m)
            self._transition('odom_reset', bool(self.odom_reset.get('has_reset')),
                             'odom_reset', origin=self.odom_reset.get('origin'),
                             stamp=self.odom_reset.get('stamp'))
        self.create_subscription(String, '/odom_zero/status', on_oz_status,
                                 LATCHED)

    @staticmethod
    def _parse_q(s):
        # rtk_status payload contains "quality=N" somewhere
        try:
            for tok in str(s).replace(',', ' ').split():
                if tok.startswith('quality='):
                    return int(float(tok.split('=', 1)[1]))
        except Exception:
            pass
        return None

    # ----- transition handlers -----
    def _on_run_status(self):
        phase = self.run.get('phase')
        self._transition('phase', phase, 'phase_change',
                         leg=self.run.get('leg_index'),
                         curve=self.run.get('curve_name'),
                         treatment=self.run.get('treatment'))
        pr = self.run.get('pause_reason')
        self._transition('pause_reason', pr, 'pause_reason')
        # leg boundary detection: a fresh run/scored phase resets the accumulator
        if phase in ('bag_start', 'follower_start') and self._last_phase not in (
                'bag_start', 'follower_start'):
            self._leg = self._new_leg_acc()
            self._leg['leg_id'] = self.run.get('leg_id')
            self._leg['curve_name'] = self.run.get('curve_name')
            self._leg['treatment'] = self.run.get('treatment')
        # leg completion: entering stop_leg/done finalizes the verdict
        if phase in ('stop_leg', 'done') and self._last_phase == 'run':
            self._finalize_leg()
        self._last_phase = phase

    def _on_orch_status(self):
        for name, det in (self.orch_detail or {}).items():
            running = det.get('running')
            self._transition(f'proc:{name}', running, 'proc_state',
                             name=name, exit_code=det.get('last_exit_code'),
                             intentional=det.get('intentional_stop_reason'))

    # ----- per-leg verdict -----
    def _new_leg_acc(self):
        return {'leg_id': None, 'curve_name': None, 'treatment': None,
                'rtk_fixed': 0, 'rtk_total': 0, 'max_abs_e_psi': 0.0,
                'estop_seen': False, 'final_s_star': None,
                'total_length': None, 't_start': self._now()}

    def _accumulate_follow(self):
        # called on each /path_follower/status; only meaningful during a run
        if self.run.get('phase') != 'run':
            return
        e = self.follow.get('e_psi')
        if e is not None:
            self._leg['max_abs_e_psi'] = max(self._leg['max_abs_e_psi'],
                                             abs(math.degrees(e)))
        self._leg['final_s_star'] = self.follow.get('s_star')
        self._leg['total_length'] = self.follow.get('total_length')
        if self.rtk_q is not None:
            self._leg['rtk_total'] += 1
            if self.rtk_q == RTK_SCORED_GATE_Q:
                self._leg['rtk_fixed'] += 1
        if self.estop:
            self._leg['estop_seen'] = True

    def _finalize_leg(self):
        L = self._leg
        pct = (100.0 * L['rtk_fixed'] / L['rtk_total']) if L['rtk_total'] else 0.0
        reached = bool(self.done)
        verdict = {
            'leg_id': L['leg_id'], 'curve_name': L['curve_name'],
            'treatment': L['treatment'],
            'duration_s': round(self._now() - L['t_start'], 1),
            'reached_end': reached, 'estop_seen': L['estop_seen'],
            'rtk_fixed_pct': round(pct, 1),
            'rtk_window_pct_required': RTK_WINDOW_PCT,
            'max_abs_e_psi_deg': round(L['max_abs_e_psi'], 1),
            'final_s_star': L['final_s_star'],
            'total_length': L['total_length'],
            'predict_pass': bool(reached and not L['estop_seen']
                                 and pct >= RTK_WINDOW_PCT),
        }
        self._emit('leg_verdict', verdict, to_events=True)

    # ----- flags -----
    def _compute_flags(self):
        now = time.monotonic()
        phase = self.run.get('phase')
        driving = (self.repo.get('state') == 'driving')

        def age(topic):
            c = self._seen.get(topic)
            return None if not c or c[1] is None else now - c[1]

        # odom dropout (base-serial)
        odom_age = age('/wheel/odom')
        odom_silent = odom_age is not None and odom_age > ODOM_SILENT_S
        self._transition('odom_silent', bool(odom_silent),
                         'odom_dropout' if odom_silent else 'odom_recovered')

        # rtk below the scored gate during a recorded run (F3)
        rtk_below_gate = (phase == 'run' and self.rtk_q != RTK_SCORED_GATE_Q)

        # heading instability while repositioning
        std = self.heading.get('heading_std_deg')
        heading_unstable = bool(driving and (
            (std is not None and std > HDG_STD_MAX_DEG)
            or self.heading.get('mode') != HDG_MODE_OK))

        # fused-vs-COG disagreement (the heading-convention / sign cross-check)
        fused = self.heading.get('fused_deg')
        disagree = None
        if fused is not None and self.cog is not None:
            disagree = abs(_ang_norm(fused - self.cog))

        # steering saturation sustained
        dc = self.follow.get('delta_cmd')
        if dc is not None and abs(dc) >= DELTA_SAT_RAD:
            self._delta_sat_since = self._delta_sat_since or now
        else:
            self._delta_sat_since = None
        delta_saturated = bool(self._delta_sat_since
                               and now - self._delta_sat_since > DELTA_SAT_S)

        # s_star stalled while moving
        s = self.follow.get('s_star')
        v = self.follow.get('v')
        stalled = False
        if s is not None:
            if self._sstar_last is None or s > self._sstar_last + 0.02:
                self._sstar_last = s
                self._sstar_rise_t = now
            elif (v is not None and v > STALL_V and self._sstar_rise_t
                  and now - self._sstar_rise_t > STALL_S):
                stalled = True

        # C6: cmd_vel_raw nonzero while no declared owner, or two movers alive
        owner = self.run.get('cmd_owner')
        raw_active = any(abs(x) > 1e-3 for x in self.cmd_raw if x is not None)
        c6_owner_violation = bool(raw_active and owner in (None, 'null', ''))
        c6_two_movers = bool(self.orch.get('follower')
                             and self.orch.get('reposition'))

        # estop chain engaged: raw commands but final is zero
        final_active = any(abs(x) > 1e-3 for x in self.cmd_final
                           if x is not None)
        estop_gating = bool(raw_active and not final_active)

        flags = {
            'odom_silent': odom_silent,
            'rtk_below_gate': bool(rtk_below_gate),
            'heading_unstable': heading_unstable,
            'fused_cog_disagree_deg': None if disagree is None else round(disagree, 1),
            'fused_cog_disagree': bool(disagree is not None
                                       and disagree > FUSED_COG_DISAGREE_DEG),
            'delta_saturated': delta_saturated,
            's_star_stalled': stalled,
            'c6_owner_violation': c6_owner_violation,
            'c6_two_movers': c6_two_movers,
            'estop_active': bool(self.estop),
            'estop_gating': estop_gating,
        }
        # mirror notable flag flips into the events spine
        for fk in ('rtk_below_gate', 'heading_unstable', 'fused_cog_disagree',
                   'delta_saturated', 's_star_stalled', 'c6_owner_violation',
                   'c6_two_movers'):
            self._transition(f'flag:{fk}', bool(flags[fk]), 'flag',
                             flag=fk, set=bool(flags[fk]))
        return flags

    # ----- periodic -----
    def _topic_health(self, snapshot=False):
        now = time.monotonic()
        health = {}
        for topic, (count, last) in self._seen.items():
            age = None if last is None else round(now - last, 2)
            present = age is not None and age <= TOPIC_SILENT_S
            health[topic] = {'count': count, 'age_s': age, 'present': present}
            self._transition(f'health:{topic}', present, 'topic_health_change',
                             topic=topic, age_s=age)
        # also flag topics never seen at all
        if snapshot:
            self._emit('topic_health', {'topics': health,
                                        'never_seen': sorted(
                                            t for t in EXPECTED_TOPICS
                                            if t not in self._seen)},
                       to_events=True)

    def _tick(self):
        flags = self._compute_flags()
        now = time.monotonic()
        if now - self._last_health_snapshot >= HEALTH_SNAPSHOT_S:
            self._last_health_snapshot = now
            self._topic_health(snapshot=True)
        else:
            self._topic_health(snapshot=False)

        sample = {
            'run': {k: self.run.get(k) for k in (
                'phase', 'leg_index', 'n_legs', 'runs_done', 'runs_target',
                'cmd_owner', 'pause_reason', 'treatment', 'curve_name',
                'curve_kind', 'battery_v', 'rtk_q')},
            'follow': {**self.follow, 'timing_ms': self.timing_ms,
                       'done': self.done},
            'heading': {k: self.heading.get(k) for k in (
                'fused_deg', 'heading_std_deg', 'mode', 'active_sources',
                'compass_offset_deg', 'gyro_bias_limo_dps', 'rtk_quality',
                'compass_age_s', 'cog_age_s')},
            'repo': {k: self.repo.get(k) for k in (
                'state', 'err_m', 'err_deg', 'reason')},
            'rtk': {'quality': self.rtk_q, 'fix': self.fix,
                    'cog_deg': self.cog, 'cog_travel_m': self.cog_travel},
            'cmd': {'raw': self.cmd_raw, 'final': self.cmd_final,
                    'estop': self.estop},
            'odom_reset': {'has_reset': self.odom_reset.get('has_reset'),
                           'stamp': self.odom_reset.get('stamp')},
            'flags': flags,
        }
        self._emit('sample', sample)

    def close(self):
        try:
            self._topic_health(snapshot=True)
            self._emit('meta', {'shutdown': True}, to_events=True)
        finally:
            self._ev.close()
            self._sm.close()


EXPECTED_TOPICS = [
    '/run/status', '/orchestrator/status', '/path_follower/status',
    '/path_follower/done', '/path_follower/timing', '/heading/fused_status',
    '/reposition/status', '/gps_rtk_f9p_helical/gps/rtk_status',
    '/gps_rtk_f9p_helical/gps/fix', '/cmd_vel_raw', '/cmd_vel', '/estop',
    '/wheel/odom', '/wheel/odom_zeroed', '/odom_zero/status',
]


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--outdir',
                    default=os.path.join(REPO_ROOT, 'Experiment Data', 'monitor'),
                    help='directory for the .events/.samples jsonl files')
    args = ap.parse_args()

    rclpy.init()
    node = BlackBox(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
