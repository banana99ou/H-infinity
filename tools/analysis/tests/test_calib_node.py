# -*- coding: utf-8 -*-
"""Unit tests for calib_node's ROS-free logic (Kasa fit, lever-arm correction,
containment geometry, request parsing, and the figure-8 sequencer driven by a
simulated bicycle robot).

No ROS needed: calib_node guards its rclpy/message imports. Run either way:

    python3 -m pytest tools/analysis/tests/test_calib_node.py
    python3 tools/analysis/tests/test_calib_node.py

Every test states in a comment what result would make it FAIL (a check that
cannot fail is not evidence).

Simulator: kinematic bicycle at 1 kHz, wheelbase 0.2 m, chassis full lock
0.35 rad (the driver clamp 0.408 does not bind), first-order steering (80 ms)
and speed (150 ms) lag, optional yaw "slip" factor (w = slip * v tan(delta)/L),
gyro bias 0.004 rad/s + 0.02 rad/s noise at 100 Hz, odom 50 Hz, RTK antenna
(0.10 m fwd, 0.075 m right of the rear axle) at 10 Hz with 1 cm noise, control
tick 20 Hz. A silent tick (cmd None) keeps the LAST command, like a base that is
not sent anything new — so an abort only passes if the zero hold really stops it.
"""
import ast
import json
import math
import os
import random
import sys

_REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
_PKG = os.path.join(_REPO, 'scalecar-vfg-h-infinite', 'ros2_bridge',
                    'limo_path_follower')
if _PKG not in sys.path:
    sys.path.insert(0, _PKG)

import calib_node as cn  # noqa: E402

SRC = os.path.join(_PKG, 'calib_node.py')

LAT0, LON0 = 37.6100, 126.9900
L = 0.2
DELTA_MAX = 0.35
DRIVER_CLAMP = 0.408
A_FWD, B_RIGHT = 0.10, 0.075
DT = 0.001                       # physics step
IMU_EVERY, ODOM_EVERY, FIX_EVERY, CTRL_EVERY = 10, 20, 100, 50


def _ll(e, n):
    lat, lon = cn.en_to_latlon(e, n, LAT0, LON0)
    return {'lat': lat, 'lon': lon}


def _square(half):
    return [_ll(-half, -half), _ll(half, -half), _ll(half, half), _ll(-half, half)]


def make_request(**over):
    d = {
        'seq': 7, 'mode': 'full', 'speeds': [0.5, 1.0], 'steer_cmd_rad': 0.40,
        'R_plan_m': 1.2, 'turn_deg': 360,
        'pin': {'lat': LAT0, 'lon': LON0, 'heading_deg': 90.0},
        'venue': {'corners_wgs84': _square(15.0), 'exclusions': []},
        'min_clearance_m': 0.5, 'max_radius_m': 2.9, 'max_duration_s': 150,
    }
    d.update(over)
    return d


class Bicycle:
    """Rear-axle kinematic bicycle in the pin-centred EN frame, heading +E."""

    def __init__(self, slip=1.0, steer_gain=1.0, rotate=True,
                 tau_steer=0.08, tau_v=0.15):
        self.slip, self.steer_gain, self.rotate = slip, steer_gain, rotate
        self.tau_steer, self.tau_v = tau_steer, tau_v
        self.x = self.y = self.psi = 0.0
        self.v = self.v_cmd = 0.0
        self.delta = self.d_cmd = 0.0
        self.w = 0.0

    def command(self, cmd):
        if cmd is None:              # silent: the base keeps the last command
            return
        v, w = cmd
        self.v_cmd = v
        d = math.atan(w * L / v) if v > 1e-6 else 0.0
        d = max(-DRIVER_CLAMP, min(DRIVER_CLAMP, d)) * self.steer_gain
        self.d_cmd = max(-DELTA_MAX, min(DELTA_MAX, d))

    def advance(self, dt):
        self.delta += (self.d_cmd - self.delta) * min(1.0, dt / self.tau_steer)
        self.v += (self.v_cmd - self.v) * min(1.0, dt / self.tau_v)
        self.w = (self.slip * self.v * math.tan(self.delta) / L
                  if self.rotate else 0.0)
        pm = self.psi + 0.5 * self.w * dt
        self.x += self.v * math.cos(pm) * dt
        self.y += self.v * math.sin(pm) * dt
        self.psi += self.w * dt

    def antenna(self):
        c, s = math.cos(self.psi), math.sin(self.psi)
        return (self.x + A_FWD * c + B_RIGHT * s, self.y + A_FWD * s - B_RIGHT * c)


def r_true(slip=1.0):
    return L / (slip * math.tan(DELTA_MAX))


def simulate(seqr, req, sim, t_end=120.0, faults=None, estop_at=None,
             estop_value=False, feed_fix=True, feed_sensors=True, seed=1,
             fix_status=0):
    """Drive seqr with the simulated robot. faults: {'imu'|'odom'|'fix': t}
    stops that stream from t on. Returns [(t, cmd, internal_state), ...] per
    control tick; also records the fix positions seen."""
    rng = random.Random(seed)
    faults = faults or {}
    log = []
    sim.fix_trace = []
    n = int(round(t_end / DT))
    for k in range(n + 1):
        t = k * DT
        if feed_sensors:
            if k % IMU_EVERY == 0 and t < faults.get('imu', 1e9):
                seqr.ingest_imu(t, sim.w + rng.gauss(0.004, 0.02),
                                IMU_EVERY * DT)
            if k % ODOM_EVERY == 0 and t < faults.get('odom', 1e9):
                seqr.ingest_odom(t, sim.v)
            if feed_fix and k % FIX_EVERY == 0 and t < faults.get('fix', 1e9):
                e, nn = sim.antenna()
                e += rng.gauss(0.0, 0.01)
                nn += rng.gauss(0.0, 0.01)
                lat, lon = cn.en_to_latlon(e, nn, LAT0, LON0)
                if cn.fix_is_valid(fix_status, lat, lon) and req.geom is not None:
                    en = req.geom.to_en(lat, lon)
                    seqr.ingest_fix(t, en, t_meas=t)
                    sim.fix_trace.append((t, en))
        if k % CTRL_EVERY == 0:
            if estop_at is not None and t >= estop_at:
                estop = True
            else:
                estop = estop_value
            cmd, _ev = seqr.step(t, estop=estop)
            sim.command(cmd)
            log.append((t, cmd, seqr.state))
            if seqr.state in ('done', 'aborted') and cmd is None:
                break
        sim.advance(DT)
    return log


def run_case(req_over=None, cfg_over=None, sim_kw=None, **sim_args):
    cfg = cn.CalibConfig(**(cfg_over or {}))
    req, err = cn.parse_request(make_request(**(req_over or {})), cfg)
    assert err is None, err
    seqr = cn.CalibSequencer(req, cfg, 0.0)
    sim = Bicycle(**(sim_kw or {}))
    log = simulate(seqr, req, sim, **sim_args)
    return seqr, sim, log, cfg


def assert_zero_from_abort(log, cfg, moving_before=True):
    """Common abort contract. Returns the abort tick time.

    Fails if: no tick ever reports 'aborted'; the abort tick's command is not
    exactly (0, 0); any later tick commands non-zero; the zero hold is shorter
    than stop_hold_s; or (moving_before) nothing non-zero was commanded before,
    i.e. the abort was never exercised from motion."""
    idx = next((i for i, (_t, _c, s) in enumerate(log) if s == 'aborted'), None)
    assert idx is not None, 'never aborted'
    t_abort, cmd_abort, _ = log[idx]
    assert cmd_abort == (0.0, 0.0), f'abort tick command {cmd_abort}'
    for (t, c, s) in log[idx:]:
        assert s == 'aborted'
        assert c is None or c == (0.0, 0.0), f'non-zero after abort at {t}: {c}'
    zeros = [t for (t, c, _s) in log[idx:] if c == (0.0, 0.0)]
    assert zeros[-1] - zeros[0] >= cfg.stop_hold_s - 0.051
    if moving_before:
        assert any(c not in (None, (0.0, 0.0)) for (_t, c, _s) in log[:idx])
    return t_abort


# ---------------------------------------------------------------------------
# Pure helpers
# ---------------------------------------------------------------------------

def test_kasa_fit_recovers_known_circle():
    # Fails if: the fit's centre/radius formula is wrong (e.g. sign of the
    # centre terms, missing the mean-centring add-back) -> R/centre off by far
    # more than the 1 cm noise; or the residual is not the geometric RMS.
    cx, cy, R = 3.2, -1.7, 0.62
    rng = random.Random(3)
    exact, noisy = [], []
    for i in range(40):
        th = math.radians(90 + 250 * i / 39)   # the 90..340 deg window arc
        p = (cx + R * math.cos(th), cy + R * math.sin(th))
        exact.append(p)
        noisy.append((p[0] + rng.gauss(0, 0.01), p[1] + rng.gauss(0, 0.01)))
    fx, fy, fR, rms = cn.kasa_fit(exact)
    assert abs(fx - cx) < 1e-9 and abs(fy - cy) < 1e-9 and abs(fR - R) < 1e-9
    assert rms < 1e-9
    fx, fy, fR, rms = cn.kasa_fit(noisy)
    assert abs(fR - R) < 0.01, fR
    assert math.hypot(fx - cx, fy - cy) < 0.015
    assert 0.004 < rms < 0.02, rms


def test_kasa_fit_degenerate_inputs():
    # Fails if: a collinear set or < 3 points returns a "circle" instead of None
    # (a straight drive would then report a finite bogus R_rtk).
    assert cn.kasa_fit([(0, 0), (1, 1)]) is None
    assert cn.kasa_fit([(i * 0.1, 2 * i * 0.1) for i in range(10)]) is None


def test_rear_axle_correction_left_right_signs():
    # Independent geometry: robot at the origin heading +x, rear axle at the
    # origin, antenna at (a, -b) (fwd a, right b). Left turn centre (0, +R),
    # right turn centre (0, -R). Fails if: the left/right b-signs are swapped
    # (that would return R +/- 2b = 0.70 / 0.40 instead of 0.55).
    R = 0.55
    R_ant_left = math.hypot(A_FWD - 0.0, -B_RIGHT - R)
    R_ant_right = math.hypot(A_FWD - 0.0, -B_RIGHT + R)
    assert abs(cn.rear_axle_radius(R_ant_left, cn.LEFT, A_FWD, B_RIGHT) - R) < 1e-12
    assert abs(cn.rear_axle_radius(R_ant_right, cn.RIGHT, A_FWD, B_RIGHT) - R) < 1e-12
    # The wrong direction is off by exactly 2b -> the check above can fail.
    assert abs(cn.rear_axle_radius(R_ant_left, cn.RIGHT, A_FWD, B_RIGHT)
               - (R + 2 * B_RIGHT)) < 1e-12
    assert math.isnan(cn.rear_axle_radius(0.05, cn.LEFT, A_FWD, B_RIGHT))


def test_command_sign_and_magnitude():
    # Fails if: angular.z is not dir * v * tan(steer)/L (sign flipped for a
    # direction, tan vs. sin/raw angle, or L missing).
    for v in (0.5, 1.0):
        for d in (cn.LEFT, cn.RIGHT):
            w = cn.omega_cmd(v, 0.40, 0.2, d)
            assert abs(w - d * v * math.tan(0.40) / 0.2) < 1e-12
    assert cn.omega_cmd(1.0, 0.40, 0.2, cn.LEFT) > 0      # REP-103: left = CCW+
    assert cn.omega_cmd(1.0, 0.40, 0.2, cn.RIGHT) < 0


def test_polygon_clearance_and_containment():
    # Fails if: the clearance sign convention is wrong (outside must be
    # negative), point-in-polygon is wrong, or exclusions are not counted.
    poly = [(-2, -2), (2, -2), (2, 2), (-2, 2)]
    assert abs(cn.polygon_clearance((0, 0), poly) - 2.0) < 1e-12
    assert abs(cn.polygon_clearance((1.5, 0), poly) - 0.5) < 1e-12
    assert abs(cn.polygon_clearance((3, 0), poly) + 1.0) < 1e-12
    g = cn.CalibGeometry(LAT0, LON0, poly, [(0.0, 1.0, 0.3)])
    m, what = cn.containment_margin((0.0, 0.5), g)
    assert what == 'exclusion 0' and abs(m - 0.2) < 1e-12
    m, what = cn.containment_margin((0.0, 1.0), g)
    assert what == 'exclusion 0' and m < 0


def test_fix_validity_and_imu_dt():
    # Fails if: a no-fix (status -1) or NaN fix is accepted, or a valid one is
    # refused; or a non-monotonic / too-large stamp gap is used as the step.
    assert cn.fix_is_valid(0, LAT0, LON0)
    assert cn.fix_is_valid(2, LAT0, LON0)
    assert not cn.fix_is_valid(-1, LAT0, LON0)
    assert not cn.fix_is_valid(0, float('nan'), LON0)
    assert cn.imu_dt_from_stamps(1.0, 1.01, 0.3) == 1.01 - 1.0
    assert cn.imu_dt_from_stamps(1.0, 0.99, 0.3) is None
    assert cn.imu_dt_from_stamps(1.0, 1.5, 0.3) is None
    assert cn.imu_dt_from_stamps(None, 1.0, 0.3) is None


# ---------------------------------------------------------------------------
# Request parsing / dedupe
# ---------------------------------------------------------------------------

def test_parse_request_defaults_and_rejections():
    # Fails if: a mode's default speeds are wrong, or any out-of-limit /
    # unverifiable request is ACCEPTED (it would then drive).
    cfg = cn.CalibConfig()
    d = make_request()
    del d['speeds']
    req, err = cn.parse_request(d, cfg)
    assert err is None and req.speeds == [0.5, 1.0]
    d = make_request(mode='sanity')
    del d['speeds']
    req, err = cn.parse_request(d, cfg)
    assert err is None and req.speeds == [1.0]
    bad = [
        dict(speeds=[1.2]), dict(speeds=[0.0]), dict(speeds=[]),
        dict(speeds=['fast']), dict(steer_cmd_rad=0.6), dict(steer_cmd_rad=-0.4),
        dict(mode='spin'), dict(turn_deg=100), dict(max_duration_s=0),
        dict(venue={'corners_wgs84': _square(15)[:2]}), dict(pin=None),
        dict(venue={'corners_wgs84': _square(15),
                    'exclusions': [{'kind': 'polygon'}]}),
        dict(venue={'corners_wgs84': _square(15),
                    'exclusions': [{'kind': 'circle', 'lat': LAT0}]}),
    ]
    for over in bad:
        req, err = cn.parse_request(make_request(**over), cfg)
        assert req is None and err, f'accepted bad request {over}'
    assert cn.parse_request([1, 2], cfg)[0] is None
    # Pedestal needs no pin/venue (wheels off the ground, no RTK checks).
    d = make_request()
    del d['pin'], d['venue']
    assert cn.parse_request(d, cfg)[0] is None
    req, err = cn.parse_request(d, cn.CalibConfig(pedestal_test=True))
    assert err is None and req.geom is None


def test_request_action_dedupe():
    # Fails if: the executor's 1 Hz re-send of the SAME seq restarts a run
    # ('accept' instead of 'ack'), or a different seq hijacks an active run.
    assert cn.request_action(None, None, 1) == 'accept'
    for st in ('waiting', 'running', 'stopping', 'done', 'aborted'):
        assert cn.request_action(5, st, 5) == 'ack'
    for st in ('waiting', 'running', 'stopping'):
        assert cn.request_action(5, st, 6) == 'busy'
    for st in ('done', 'aborted'):
        assert cn.request_action(5, st, 6) == 'accept'


# ---------------------------------------------------------------------------
# Full figure-8 on the simulated robot
# ---------------------------------------------------------------------------

def _check_full_run(slip):
    seqr, sim, log, cfg = run_case(sim_kw=dict(slip=slip), t_end=80.0)
    res = seqr.result
    # Fails if: the run did not finish cleanly.
    assert seqr.state == 'done', (seqr.state, seqr.reason)
    assert res['ok'] is True, res['reason']
    segs = res['segments']
    # Fails if: segments are missing, extra, or out of order.
    assert [(s['v_cmd'], s['dir']) for s in segs] == [
        (0.5, 'left'), (0.5, 'right'), (1.0, 'left'), (1.0, 'right')]
    R = r_true(slip)
    for s in segs:
        # Fails if: the segment ended early/late (yaw integration or the
        # completion test is wrong): 360 deg + at most one 20 Hz tick of turn.
        assert 360.0 <= s['turned_deg'] < 370.0, s
        # Fails if: R_imu (median odom / |median gyro|) is biased by > 3 %.
        assert abs(s['R_imu_m'] - R) / R < 0.03, (s, R)
        assert abs(s['delta_imu_rad'] - math.atan(L / R)) < 0.02
        # Fails if: the Kasa fit or the lever-arm correction (sign per dir)
        # is wrong — a swapped b-sign shifts R_rtk_rear by 0.15 m (~25 %).
        assert s['n_fix'] >= 8
        assert abs(s['R_rtk_rear_m'] - R) / R < 0.03, (s, R)
        assert s['fit_resid_m'] < 0.03
        assert abs(s['v_odom_mps'] - s['v_cmd']) / s['v_cmd'] < 0.02
        # Antenna speed = v * R_ant / R_rear (the antenna is farther out on a
        # left turn, nearer on a right turn).
        R_ant = math.hypot(A_FWD, R + (B_RIGHT if s['dir'] == 'left' else -B_RIGHT))
        assert abs(s['v_rtk_mps'] - s['v_cmd'] * R_ant / R) / (s['v_cmd'] * R_ant / R) < 0.08
        # Window = yaw progress in [settle 90, turn - tail 340] deg: 250 deg of
        # steady turning at w = v/R. Fails if: the window bounds are wrong
        # (e.g. no settle -> 340 deg; no tail -> 270 deg; both are > 8 % off).
        expect_ws = math.radians(360 - 20 - 90) * R / s['v_cmd']
        assert abs(s['window_s'] - expect_ws) / expect_ws < 0.05, (s, expect_ws)
        assert abs(s['n_imu'] - 100 * expect_ws) <= 0.05 * 100 * expect_ws + 2
    # Command contract during the run. Fails if: any running command is not
    # (v, dir * v * tan(0.40)/L), or the 4 commands are not in plan order.
    w = lambda v, d: d * v * math.tan(0.40) / L  # noqa: E731
    seen = []
    for (_t, c, st) in log:
        if c is not None and c != (0.0, 0.0):
            assert st == 'running'
            if not seen or seen[-1] != c:
                seen.append(c)
    expect = [(0.5, w(0.5, 1)), (0.5, w(0.5, -1)), (1.0, w(1.0, 1)), (1.0, w(1.0, -1))]
    assert len(seen) == 4
    for got, exp in zip(seen, expect):
        assert got[0] == exp[0] and abs(got[1] - exp[1]) < 1e-12
    # Ending. Fails if: 'done' is reported before a full stop_hold_s of zeros,
    # or anything non-zero is commanded after the last segment.
    i_last = max(i for i, (_t, c, _s) in enumerate(log)
                 if c not in (None, (0.0, 0.0)))
    tail = log[i_last + 1:]
    assert all(c in (None, (0.0, 0.0)) for (_t, c, _s) in tail)
    t_zero0 = tail[0][0]
    t_done = next(t for (t, _c, s) in tail if s == 'done')
    assert t_done - t_zero0 >= cfg.stop_hold_s - 1e-9
    assert log[-1][1] is None and seqr.state == 'done'
    # Robot really stopped (zero commands reached the simulated base).
    assert abs(sim.v) < 1e-3
    # Strict JSON must be producible (NaN -> null).
    json.dumps(cn.json_safe(seqr.status_payload(0.0, log[-1][0])), allow_nan=False)
    return seqr


def test_full_mode_figure8_no_slip():
    _check_full_run(1.0)


def test_full_mode_figure8_with_slip():
    # Fails if: the measurement silently assumes the geometric radius instead
    # of measuring it (0.9 slip -> true R 11 % larger than L/tan(delta)).
    seqr = _check_full_run(0.9)
    assert seqr.result['segments'][0]['R_imu_m'] > 1.08 * r_true(1.0)


def test_sanity_mode_single_speed():
    # Fails if: sanity mode does not run exactly one figure-8 at 1.0 m/s.
    seqr, _sim, _log, _cfg = run_case(req_over=dict(mode='sanity', speeds=[1.0]))
    assert seqr.state == 'done' and seqr.result['ok']
    assert [(s['v_cmd'], s['dir']) for s in seqr.result['segments']] == [
        (1.0, 'left'), (1.0, 'right')]


# ---------------------------------------------------------------------------
# Abort rules — each must fire, and the command must be zero from that tick on
# ---------------------------------------------------------------------------

def test_abort_estop():
    # Fails if: an e-stop at t=3.0 does not abort on THAT tick.
    seqr, _s, log, cfg = run_case(estop_at=3.0)
    t = assert_zero_from_abort(log, cfg)
    assert abs(t - 3.0) < 1e-9 and 'e-stop' in seqr.reason


def test_abort_estop_active_at_start_never_moves():
    # Fails if: the robot is commanded to move while the e-stop is already on.
    seqr, _s, log, cfg = run_case(estop_value=True)
    assert_zero_from_abort(log, cfg, moving_before=False)
    assert all(c in (None, (0.0, 0.0)) for (_t, c, _s) in log)
    assert 'e-stop active at start' in seqr.reason


def test_abort_imu_stale():
    # IMU stops at 3.0 (last sample 2.99) -> stale > 0.3 s first true at 3.30.
    # Fails if: the stale threshold is wrong or IMU staleness is not checked.
    seqr, _s, log, cfg = run_case(faults={'imu': 3.0})
    t = assert_zero_from_abort(log, cfg)
    assert 3.29 < t <= 3.36 and 'IMU stale' in seqr.reason, (t, seqr.reason)


def test_abort_odom_stale():
    # Odom stops at 3.0 (last 2.98) -> stale > 0.5 s first true at 3.50.
    seqr, _s, log, cfg = run_case(faults={'odom': 3.0})
    t = assert_zero_from_abort(log, cfg)
    assert 3.48 < t <= 3.56 and 'odom stale' in seqr.reason, (t, seqr.reason)


def test_abort_rtk_stale_only_when_required():
    # Fixes stop at 3.0 (last 2.9) -> no valid fix > 1.0 s at ~3.9-3.95.
    # Fails if: RTK loss does not abort with require_rtk, or DOES abort without
    # it (then the rule is not conditional on require_rtk).
    seqr, _s, log, cfg = run_case(faults={'fix': 3.0})
    t = assert_zero_from_abort(log, cfg)
    assert 3.89 < t <= 3.96 and 'RTK stale' in seqr.reason, (t, seqr.reason)
    seqr2, _s2, _log2, _ = run_case(faults={'fix': 3.0},
                                     cfg_over=dict(require_rtk=False))
    assert seqr2.state == 'done' and seqr2.result['ok'], seqr2.reason


def test_abort_invalid_fixes_count_as_missing():
    # status -1 fixes are filtered out -> no valid fix -> the run never starts.
    # Fails if: invalid fixes are treated as valid positions.
    seqr, _s, log, cfg = run_case(fix_status=-1, t_end=10.0)
    assert_zero_from_abort(log, cfg, moving_before=False)
    assert 'start timeout' in seqr.reason and 'RTK fix' in seqr.reason


def test_abort_leaving_polygon_clearance():
    # North venue edge 1.5 m from the pin; min_clearance 0.5 -> abort once the
    # antenna is north of 1.0 m (the left circle tops out ~1.2 m).
    # Fails if: the clearance check is missing or uses the wrong sign/frame.
    venue = {'corners_wgs84': [_ll(-15, -15), _ll(15, -15), _ll(15, 1.5),
                               _ll(-15, 1.5)], 'exclusions': []}
    seqr, _s, log, cfg = run_case(req_over=dict(venue=venue))
    assert_zero_from_abort(log, cfg)
    assert 'venue edge clearance' in seqr.reason, seqr.reason
    assert seqr._fix_en[1] > 1.0                 # consistent with the rule
    assert seqr.result['segments'][-1]['dir'] == 'left'


def test_abort_start_outside_polygon_never_moves():
    # Pin 2 m outside the venue -> abort on the first tick, before any motion.
    # Fails if: the containment check only runs after the robot moved.
    venue = {'corners_wgs84': [_ll(2, 2), _ll(10, 2), _ll(10, 10), _ll(2, 10)],
             'exclusions': []}
    seqr, _s, log, cfg = run_case(req_over=dict(venue=venue))
    assert_zero_from_abort(log, cfg, moving_before=False)
    assert all(c in (None, (0.0, 0.0)) for (_t, c, _s) in log)
    assert 'outside the venue polygon' in seqr.reason, seqr.reason


def test_abort_exclusion_clearance():
    # Exclusion circle centred 1.7 m north, r 0.3: edge at 1.4 m; the left
    # circle passes within 0.5 m of it. Fails if: exclusions are ignored.
    c = _ll(0.0, 1.7)
    venue = {'corners_wgs84': _square(15.0),
             'exclusions': [{'kind': 'circle', 'lat': c['lat'], 'lon': c['lon'],
                             'radius_m': 0.3}]}
    seqr, _s, log, cfg = run_case(req_over=dict(venue=venue))
    assert_zero_from_abort(log, cfg)
    assert 'exclusion 0' in seqr.reason, seqr.reason


def test_abort_max_radius():
    # max_radius 0.9 m: the left circle's far side is ~1.2 m from the pin.
    # Fails if: the distance-from-pin guard is missing.
    seqr, _s, log, cfg = run_case(req_over=dict(max_radius_m=0.9))
    assert_zero_from_abort(log, cfg)
    assert 'max_radius' in seqr.reason, seqr.reason
    assert math.hypot(*seqr._fix_en) > 0.9


def test_abort_not_turning():
    # Steering does nothing -> gyro ~ bias only. Abort on the first tick after
    # 2.0 s in the segment. Fails if: the not-turning rule is missing or its
    # 2.0 s arming delay is wrong.
    seqr, _s, log, cfg = run_case(sim_kw=dict(steer_gain=0.0))
    t = assert_zero_from_abort(log, cfg)
    assert 2.0 < t <= 2.06 and 'not turning' in seqr.reason, (t, seqr.reason)
    # The partial segment's R_imu is NaN (empty window) -> strict JSON must
    # still serialise (null). Fails if json_safe misses a nested NaN.
    part = seqr.result['segments'][-1]
    assert part['complete'] is False and math.isnan(part['R_imu_m'])
    try:
        json.dumps(seqr.result, allow_nan=False)
        raised = False
    except ValueError:
        raised = True
    assert raised                               # the raw result DOES hold NaN
    json.dumps(cn.json_safe(seqr.result), allow_nan=False)


def test_abort_turning_opposite_direction():
    # Steering sign inverted -> robot turns right on a left command. Fails if:
    # a wrong-way turn is accepted (it would be labelled 'left' in the result).
    seqr, _s, log, cfg = run_case(sim_kw=dict(steer_gain=-1.0))
    t = assert_zero_from_abort(log, cfg)
    assert 2.0 < t <= 2.06 and 'opposite' in seqr.reason, (t, seqr.reason)


def test_abort_segment_timeout():
    # slip 0.2 -> R 2.74 m: turns fast enough to pass not-turning (0.18 > 0.125
    # rad/s) but 360 deg needs ~35 s > timeout 1.5*2pi*1.2/0.5 + 3 = 25.62 s.
    # Wide venue + max_radius so only the timeout can fire.
    # Fails if: the segment timeout is missing or uses the wrong formula.
    seqr, _s, log, cfg = run_case(
        sim_kw=dict(slip=0.2), t_end=40.0,
        req_over=dict(max_radius_m=20.0,
                      venue={'corners_wgs84': _square(40.0), 'exclusions': []}))
    t = assert_zero_from_abort(log, cfg)
    lim = 1.5 * 2 * math.pi * 1.2 / 0.5 + 3.0
    assert lim < t <= lim + 0.051 and 'segment timeout' in seqr.reason, (t, seqr.reason)


def test_abort_total_duration():
    # max_duration 5 s; segment 1 alone needs ~7 s. Fails if: the total cap is
    # not enforced (or counted from the wrong instant).
    seqr, _s, log, cfg = run_case(req_over=dict(max_duration_s=5.0))
    t = assert_zero_from_abort(log, cfg)
    assert 5.0 < t <= 5.051 and 'max_duration' in seqr.reason, (t, seqr.reason)


def test_abort_start_timeout_without_sensors():
    # No sensors and no e-stop state: never moves, aborts after 5 s.
    # Fails if: the node starts driving blind, or waits forever.
    seqr, _s, log, cfg = run_case(feed_sensors=False, estop_value=None, t_end=10.0)
    t = assert_zero_from_abort(log, cfg, moving_before=False)
    assert 5.0 < t <= 5.051 and 'start timeout' in seqr.reason
    for word in ('IMU', 'odom', 'RTK fix', 'e-stop state'):
        assert word in seqr.reason
    assert all(c in (None, (0.0, 0.0)) for (_t, c, _s) in log)


def test_terminate_mid_run_and_zero_burst():
    # SIGTERM path (pure part). Fails if: terminate() leaves the run 'running'
    # or any later tick is non-zero; or the burst publishes != 5 zeros.
    cfg = cn.CalibConfig()
    req, _ = cn.parse_request(make_request(), cfg)
    seqr = cn.CalibSequencer(req, cfg, 0.0)
    sim = Bicycle()
    simulate(seqr, req, sim, t_end=3.0)
    assert seqr.state == 'running'
    seqr.terminate(3.01, 'terminated by SIGTERM')
    assert seqr.state == 'aborted' and seqr.result['ok'] is False
    assert seqr.reason == 'terminated by SIGTERM'
    for k in range(60):
        cmd, _ = seqr.step(3.05 + 0.05 * k, estop=False)
        assert cmd is None or cmd == (0.0, 0.0)
    calls, sleeps = [], []
    n = cn.publish_zero_burst(lambda: calls.append(1), sleep=sleeps.append)
    assert n == 5 and len(calls) == 5 and len(sleeps) == 4
    # A failing publisher must not stop the remaining attempts.
    state = {'k': 0}

    def flaky():
        state['k'] += 1
        if state['k'] == 2:
            raise RuntimeError('publisher gone')
    assert cn.publish_zero_burst(flaky, sleep=lambda _s: None) == 4
    assert state['k'] == 5


# ---------------------------------------------------------------------------
# Pedestal mode
# ---------------------------------------------------------------------------

def test_pedestal_mode_time_based():
    # Wheels off the ground: the body never rotates, no fixes at all.
    # Fails if: pedestal still needs rotation/RTK (it would abort), segments are
    # not time-based (8 s cap / 2*pi*R_plan/v), or the result claims ok.
    seqr, _sim, log, cfg = run_case(cfg_over=dict(pedestal_test=True),
                                    sim_kw=dict(rotate=False), feed_fix=False,
                                    t_end=60.0)
    assert seqr.state == 'done', seqr.reason
    res = seqr.result
    assert res['ok'] is False and 'pedestal' in res['reason']
    assert res['pedestal_test'] is True
    assert [(s['v_cmd'], s['dir']) for s in res['segments']] == [
        (0.5, 'left'), (0.5, 'right'), (1.0, 'left'), (1.0, 'right')]
    expect = [8.0, 8.0, 2 * math.pi * 1.2 / 1.0, 2 * math.pi * 1.2 / 1.0]
    for seg, dur in zip(seqr.segments, expect):
        assert dur - 1e-6 <= seg.t_end - seg.t0 < dur + 0.051, (seg.t_end - seg.t0, dur)
    for s in res['segments']:
        assert abs(s['turned_deg']) < 10.0
        assert abs(s['v_odom_mps'] - s['v_cmd']) < 0.02
        assert s['n_fix'] == 0
    # Same command law as on the ground.
    nz = [c for (_t, c, _s) in log if c not in (None, (0.0, 0.0))]
    assert abs(nz[0][1] - 0.5 * math.tan(0.40) / L) < 1e-12
    # Pedestal still honours the e-stop.
    seqr2, _s2, log2, cfg2 = run_case(cfg_over=dict(pedestal_test=True),
                                      sim_kw=dict(rotate=False), feed_fix=False,
                                      estop_at=4.0)
    assert_zero_from_abort(log2, cfg2)


# ---------------------------------------------------------------------------
# Status payload + static output-channel guard
# ---------------------------------------------------------------------------

def test_status_payload_shape():
    # Fails if: a waiting run is not reported as 'running', the seq is not
    # echoed, or a terminal status lacks the result.
    cfg = cn.CalibConfig()
    req, _ = cn.parse_request(make_request(seq=42), cfg)
    seqr = cn.CalibSequencer(req, cfg, 0.0)
    p = seqr.status_payload(123.0, 0.0)
    assert p['seq'] == 42 and p['state'] == 'running' and p['segment'] == ''
    assert 'result' not in p
    seqr.step(0.0, imu_wz=0.0, odom_v=0.0, fix_en=(0.0, 0.0), estop=False)
    p = seqr.status_payload(123.0, 0.0)
    assert p['state'] == 'running' and p['segment'] == 'v0.5 left'
    seqr.terminate(0.1, 'test')
    p = seqr.status_payload(123.0, 0.1)
    assert p['state'] == 'aborted' and p['result']['ok'] is False
    for k in ('seq', 'state', 'segment', 'reason', 'result', 'stamp'):
        assert k in p


def test_only_cmd_vel_raw_and_status_are_published():
    # Static guard on the source. Fails if: any create_publisher targets a
    # topic other than cmd_vel_raw / /calib/status, or the bare base command
    # topic appears as a string literal anywhere in the node.
    with open(SRC) as f:
        tree = ast.parse(f.read())
    topics = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and getattr(node.func, 'attr', '') == 'create_publisher':
            arg = node.args[1]
            assert isinstance(arg, ast.Constant), 'publisher topic must be a literal'
            topics.append(arg.value)
        if isinstance(node, ast.Constant) and isinstance(node.value, str):
            assert node.value not in ('cmd_vel', '/cmd_vel'), node.value
    assert sorted(topics) == ['/calib/status', 'cmd_vel_raw'], topics


if __name__ == '__main__':
    import inspect
    fns = [(n, f) for n, f in sorted(globals().items())
           if n.startswith('test_') and inspect.isfunction(f)]
    for name, fn in fns:
        fn()
        print(f'ok  {name}')
    print(f'{len(fns)} tests passed')
