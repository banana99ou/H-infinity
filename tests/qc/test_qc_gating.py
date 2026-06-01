"""Direct unit tests for the QC gating primitives (tools/analysis/qc.py).

These feed synthetic in-memory ``bag`` dicts (the ``{topic: {"stamp":...}}``
shape ``run_eval.read_bag`` produces) straight into the pure gating helpers, so
they pin the *computed values* — not just the end-to-end verdict that
``test_analysis_failure_fixtures`` covers through synthetic rosbags. No ROS, no
rosbags, numpy only.
"""

import numpy as np
import pytest

from tools.analysis import qc, run_eval


def _rtk(stamps, quals):
    return {"stamp": np.asarray(stamps, float), "quality": np.asarray(quals, float)}


def _estop(stamps, active):
    return {"stamp": np.asarray(stamps, float), "active": np.asarray(active, float)}


def _odom(stamps):
    return {"stamp": np.asarray(stamps, float)}


# --- rtk_fixed_pct ----------------------------------------------------------

def test_rtk_fixed_pct_counts_quality4_in_window():
    bag = {run_eval.TOPIC_RTK_STATUS: _rtk([0, 1, 2, 3], [4, 4, 1, 4])}
    pct, n = qc.rtk_fixed_pct(bag, None, None)
    assert n == 4
    assert pct == pytest.approx(75.0)  # 3 of 4 FIXED


def test_rtk_fixed_pct_respects_window():
    # The non-FIXED sample at t=2 is outside [0, 1.5] -> 100% inside the window.
    bag = {run_eval.TOPIC_RTK_STATUS: _rtk([0, 1, 2, 3], [4, 4, 1, 1])}
    pct, n = qc.rtk_fixed_pct(bag, 0.0, 1.5)
    assert n == 2
    assert pct == pytest.approx(100.0)


def test_rtk_fixed_pct_none_when_no_status():
    pct, n = qc.rtk_fixed_pct({}, None, None)
    assert pct is None and n == 0


# --- estop_fired ------------------------------------------------------------

def test_estop_fired_true_when_active_in_window():
    bag = {run_eval.TOPIC_ESTOP: _estop([0, 1, 2], [0, 1, 0])}
    assert qc.estop_fired(bag, None, None) is True


def test_estop_fired_ignores_events_outside_window():
    bag = {run_eval.TOPIC_ESTOP: _estop([0, 1, 5], [0, 0, 1])}
    assert qc.estop_fired(bag, 0.0, 2.0) is False


def test_estop_fired_false_without_topic():
    assert qc.estop_fired({}, None, None) is False


# --- max_odom_gap -----------------------------------------------------------

def test_max_odom_gap_returns_largest_interval():
    bag = {run_eval.TOPIC_ODOM: _odom([0.0, 0.1, 0.2, 1.5, 1.6])}
    assert qc.max_odom_gap(bag) == pytest.approx(1.3)


def test_max_odom_gap_none_when_too_few_samples():
    assert qc.max_odom_gap({run_eval.TOPIC_ODOM: _odom([0.0])}) is None
    assert qc.max_odom_gap({}) is None


# --- _window (incl. the degenerate fallback, Bug #2) ------------------------

def test_window_from_odom_stamps():
    bag = {run_eval.TOPIC_ODOM: _odom([10.0, 11.0, 12.0])}
    assert qc._window(bag) == (10.0, 12.0)


def test_window_falls_back_to_status_when_odom_thin():
    bag = {run_eval.TOPIC_ODOM: _odom([10.0]),  # only 1 odom sample
           run_eval.TOPIC_STATUS: _odom([20.0, 21.0])}
    assert qc._window(bag) == (20.0, 21.0)


def test_window_degenerate_returns_none_none():
    # Neither odom nor status has >=2 samples -> window cannot be inferred.
    # Characterizes Bug #2: callers must treat (None, None) as "no run window".
    bag = {run_eval.TOPIC_ODOM: _odom([10.0])}
    assert qc._window(bag) == (None, None)


# --- qc_leg: reason aggregation under multiple simultaneous failures --------

def _leg_with_bag(monkeypatch, bag, sidecar=None):
    sidecar = sidecar if sidecar is not None else {
        "run_id": "r", "cell_id": "c", "leg": "A_TO_B", "cell_params": {}}
    monkeypatch.setattr(qc.run_eval, "read_bag", lambda bag_dir: bag)
    return {"bag_dir": "/synthetic", "sidecar": sidecar}


def test_qc_leg_accumulates_all_failure_reasons(monkeypatch):
    bag = {
        run_eval.TOPIC_RTK_STATUS: _rtk([0, 1, 2, 3], [1, 1, 1, 1]),   # 0% FIXED
        run_eval.TOPIC_ESTOP: _estop([0, 1, 2, 3], [0, 1, 0, 0]),       # fired
        run_eval.TOPIC_ODOM: _odom([0.0, 0.1, 2.0, 2.1]),               # 1.9s gap
    }
    leg = _leg_with_bag(monkeypatch, bag)
    row = qc.qc_leg(leg, target_len_v=None, rtk_pct_min=95,
                    length_tol=0.30, gap_tol=0.5)
    assert row["usable"] is False
    reasons = row["reasons"]
    assert "estop_fired" in reasons
    assert "odom_gap" in reasons
    assert "rtk_fixed_0pct<95" in reasons


def test_qc_leg_clean_bag_is_usable(monkeypatch):
    bag = {
        run_eval.TOPIC_RTK_STATUS: _rtk([0, 1, 2, 3], [4, 4, 4, 4]),
        run_eval.TOPIC_ESTOP: _estop([0, 1, 2, 3], [0, 0, 0, 0]),
        run_eval.TOPIC_ODOM: _odom([0.0, 0.1, 0.2, 0.3]),
    }
    leg = _leg_with_bag(monkeypatch, bag)
    row = qc.qc_leg(leg, target_len_v=None, rtk_pct_min=95,
                    length_tol=0.30, gap_tol=0.5)
    assert row["usable"] is True
    assert row["reasons"] == ""


def test_qc_leg_degenerate_window_fails_loudly(monkeypatch):
    # Bug #2 fix: a bag whose window can't be inferred (only 1 odom sample, no
    # status) must be rejected with "no_run_window", not pass on partial checks.
    bag = {
        run_eval.TOPIC_RTK_STATUS: _rtk([0, 1], [4, 4]),   # would read 100% FIXED
        run_eval.TOPIC_ESTOP: _estop([0, 1], [0, 0]),      # not fired
        run_eval.TOPIC_ODOM: _odom([0.0]),                 # single sample -> no window
    }
    leg = _leg_with_bag(monkeypatch, bag)
    row = qc.qc_leg(leg, target_len_v=None, rtk_pct_min=95,
                    length_tol=0.30, gap_tol=0.5)
    assert row["usable"] is False
    assert "no_run_window" in row["reasons"]


def test_qc_leg_no_rtk_gate_skips_rtk_reason(monkeypatch):
    # Indoor/GPS-denied mode: a bad-RTK bag is still usable when the gate is off.
    bag = {
        run_eval.TOPIC_RTK_STATUS: _rtk([0, 1, 2, 3], [1, 1, 1, 1]),
        run_eval.TOPIC_ESTOP: _estop([0, 1, 2, 3], [0, 0, 0, 0]),
        run_eval.TOPIC_ODOM: _odom([0.0, 0.1, 0.2, 0.3]),
    }
    leg = _leg_with_bag(monkeypatch, bag)
    row = qc.qc_leg(leg, target_len_v=None, rtk_pct_min=95,
                    length_tol=0.30, gap_tol=0.5, no_rtk_gate=True)
    assert row["usable"] is True
    assert "rtk" not in row["reasons"]
