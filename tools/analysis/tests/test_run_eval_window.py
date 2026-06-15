#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Unit tests for the Group-D metric-correctness primitives in run_eval.

These exercise the new pure functions directly on hand-built inputs (no rosbag
I/O), so the expected values are simple arithmetic:

  * run_window         -- has_path ∩ motion intersection, and empty-overlap
                          sentinel (D2)
  * _slice_window      -- time-window slicing of an arrays-dict (D2)
  * _zero_to           -- shared t-origin re-zeroing (D5)
  * rtk_heading_reference -- /heading/fused -> venue-local yaw, COG fallback
                          (D-HEAD)
  * _masked_max_metrics   -- steady-state-masked max heading error (D3)
  * fixed_mask_for(max_dt) -- the RTK-status time guard (D1)

Run: ``pytest tools/analysis/tests/test_run_eval_window.py`` or as a script.
"""

from __future__ import annotations

import math
import os
import sys
from types import SimpleNamespace

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_ANALYSIS = os.path.dirname(_HERE)
if _ANALYSIS not in sys.path:
    sys.path.insert(0, _ANALYSIS)

import run_eval  # noqa: E402  (inserts vfg + path_gen on import)
import path_overlay as po  # noqa: E402


# --------------------------------------------------------------------------
# D2: run_window
# --------------------------------------------------------------------------

def _bag(status_stamp=None, has_path=None, odom_stamp=None, odom_v=None):
    bag = {t: None for t in (
        run_eval.TOPIC_ODOM, run_eval.TOPIC_ODOM_ZEROED, run_eval.TOPIC_STATUS)}
    if status_stamp is not None:
        bag[run_eval.TOPIC_STATUS] = {
            "stamp": np.asarray(status_stamp, float),
            "has_path": np.asarray(has_path, float)}
    if odom_stamp is not None:
        bag[run_eval.TOPIC_ODOM_ZEROED] = {
            "stamp": np.asarray(odom_stamp, float),
            "v": np.asarray(odom_v, float)}
    return bag


def test_run_window_intersection():
    # has_path active over [101,103]; motion over [102,103]. Intersection picks
    # the later start and the earlier end: t0=max(101,102)=102, t1=min(103,103).
    bag = _bag(status_stamp=[100, 101, 102, 103], has_path=[0, 1, 1, 1],
               odom_stamp=[100, 101, 102, 103], odom_v=[0, 0, 0.5, 0.5])
    t0, t1 = run_eval.run_window(bag)
    assert t0 == 102.0 and t1 == 103.0, (t0, t1)


def test_run_window_empty_overlap():
    # has_path active early [100,101], motion late [102,103]: no overlap -> None.
    bag = _bag(status_stamp=[100, 101, 102, 103], has_path=[1, 1, 0, 0],
               odom_stamp=[100, 101, 102, 103], odom_v=[0, 0, 0.5, 0.5])
    assert run_eval.run_window(bag) == (None, None)


def test_run_window_missing_streams():
    assert run_eval.run_window(_bag()) == (None, None)
    # has_path present but never active -> no lo/hi from status; motion present.
    bag = _bag(status_stamp=[100, 101], has_path=[0, 0],
               odom_stamp=[100, 101], odom_v=[0.5, 0.5])
    # only motion contributes; max([100])=100, min([101])=101.
    assert run_eval.run_window(bag) == (100.0, 101.0)


# --------------------------------------------------------------------------
# D2: _slice_window  /  D5: _zero_to
# --------------------------------------------------------------------------

def test_slice_window():
    arr = {"stamp": np.arange(100.0, 106.0), "x": np.arange(6.0)}
    out = run_eval._slice_window(arr, 101.0, 103.0)
    assert list(out["stamp"]) == [101.0, 102.0, 103.0]
    assert list(out["x"]) == [1.0, 2.0, 3.0]
    # None / no-window pass through unchanged.
    assert run_eval._slice_window(None, 1.0, 2.0) is None
    assert run_eval._slice_window(arr, None, None) is arr


def test_zero_to():
    s = np.array([105.0, 106.0, 107.0])
    assert list(run_eval._zero_to(s, 100.0)) == [5.0, 6.0, 7.0]
    assert list(run_eval._zero_to(s, None)) == [0.0, 1.0, 2.0]  # falls back to s[0]
    assert len(run_eval._zero_to(np.array([]), 100.0)) == 0


# --------------------------------------------------------------------------
# D-HEAD: rtk_heading_reference
# --------------------------------------------------------------------------

def test_heading_reference_fused():
    anchor = po.Anchor(po.LAT0, po.LON0, 42.0)
    # fused = bearing -> venue-local yaw 0; fused = bearing-90 -> yaw +pi/2.
    fused_deg = np.array([42.0, 42.0 - 90.0])
    bag = {run_eval.TOPIC_HEADING_FUSED: {
        "stamp": np.array([100.0, 101.0]), "deg": fused_deg}}
    fix_stamps = np.array([100.0, 101.0])
    yaw_cog = np.array([9.9, 9.9])  # sentinel; must NOT be returned
    yaw, src, warn = run_eval.rtk_heading_reference(
        bag, fix_stamps, yaw_cog, anchor, 100.0, 101.0)
    assert src == "heading_fused" and warn is None
    assert abs(yaw[0] - 0.0) < 1e-9
    assert abs(yaw[1] - math.pi / 2) < 1e-9


def test_heading_reference_cog_fallback():
    anchor = po.Anchor(po.LAT0, po.LON0, 42.0)
    yaw_cog = np.array([0.1, 0.2, 0.3])
    # No /heading/fused topic at all -> course-over-ground fallback + warning.
    yaw, src, warn = run_eval.rtk_heading_reference(
        {run_eval.TOPIC_HEADING_FUSED: None},
        np.array([1.0, 2.0, 3.0]), yaw_cog, anchor, 1.0, 3.0)
    assert src == "course_over_ground" and warn is not None
    assert list(yaw) == [0.1, 0.2, 0.3]
    # Fewer than 2 in-window fused samples also falls back.
    bag = {run_eval.TOPIC_HEADING_FUSED: {
        "stamp": np.array([1.0]), "deg": np.array([42.0])}}
    _y, src2, warn2 = run_eval.rtk_heading_reference(
        bag, np.array([1.0, 2.0]), yaw_cog[:2], anchor, 1.0, 2.0)
    assert src2 == "course_over_ground" and warn2 is not None


# --------------------------------------------------------------------------
# D3: _masked_max_metrics
# --------------------------------------------------------------------------

def test_masked_max_excludes_transient_spike():
    # A 90 deg spike at t=0 (transient) and a 5 deg steady-state max.
    sr = SimpleNamespace(
        time=np.array([0.0, 1.0, 2.0, 3.0, 4.0]),
        e_psi=np.radians(np.array([90.0, 80.0, 5.0, 4.0, 3.0])))
    m = run_eval._masked_max_metrics(sr, t_transient=1.5)
    assert abs(m["max_e_psi_deg"] - 5.0) < 1e-6          # spike masked out
    assert abs(m["max_e_psi_deg_unmasked"] - 90.0) < 1e-6  # spike still recorded
    # Mask that excludes everything falls back to the full range.
    m2 = run_eval._masked_max_metrics(sr, t_transient=99.0)
    assert abs(m2["max_e_psi_deg"] - 90.0) < 1e-6


# --------------------------------------------------------------------------
# D1: fixed_mask_for time guard
# --------------------------------------------------------------------------

def test_fixed_mask_max_dt_guard():
    fix = np.array([100.0, 102.0])
    status = {"stamp": np.array([100.0]), "quality": np.array([4.0])}
    # No guard: the distant fix still inherits the single FIXED status.
    assert list(run_eval.fixed_mask_for(fix, status)) == [True, True]
    # 1.0 s guard: the fix 2 s from the nearest status is NOT FIXED.
    assert list(run_eval.fixed_mask_for(fix, status, max_dt=1.0)) == [True, False]
    # No status topic -> None (caller uses all fixes).
    assert run_eval.fixed_mask_for(fix, None) is None


def _run_all():
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for fn in fns:
        fn()
        print(f"{fn.__name__}: OK")
    print(f"--- {len(fns)} run_eval window/heading tests passed ---")


if __name__ == "__main__":
    _run_all()
