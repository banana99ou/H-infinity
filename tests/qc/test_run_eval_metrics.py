"""Unit tests for the core analysis metrics (tools/analysis/run_eval.py).

These pin the *values* of the functions that produce the numbers in the paper:
the e_d/e_psi reconstruction, the analytic-path rebuild, and the RTK-FIXED
masking. The end-to-end smoke test only checks the on-path round-trip; here we
add off-path cases and edge cases. Pure numpy + the vendored vfg library.
"""

import math

import numpy as np
import pytest

from tools.analysis import run_eval
from tools.path_gen import path_overlay


# --- build_path_from_recipe -------------------------------------------------

def test_build_step_path_length_matches_formula():
    p = run_eval.build_path_from_recipe(
        {"type": "step", "params": {"L1": 5.0, "R": 0.5,
                                     "theta_arc": math.pi / 2, "L2": 5.0}})
    expected = 5.0 + 0.5 * (math.pi / 2) + 5.0
    assert p.total_length == pytest.approx(expected, rel=1e-6)


def test_build_step_path_uses_defaults_when_params_missing():
    # A recipe with no params must fall back to the documented defaults
    # (L1=5, R=0.5, theta=pi/2, L2=5) — same curve the follower would build.
    p = run_eval.build_path_from_recipe({"type": "step"})
    expected = 5.0 + 0.5 * (math.pi / 2) + 5.0
    assert p.total_length == pytest.approx(expected, rel=1e-6)


def test_build_uturn_uses_r_min_default():
    p = run_eval.build_path_from_recipe({"type": "uturn"}, r_min_default=0.6)
    # uturn => StepCurvaturePath(L1=1, R=r_min_default, theta=pi, L2=1)
    expected = 1.0 + 0.6 * math.pi + 1.0
    assert p.total_length == pytest.approx(expected, rel=1e-6)


def test_build_slalom_path_is_positive_length():
    p = run_eval.build_path_from_recipe({"type": "slalom"})
    assert p.total_length > 0.0


@pytest.mark.parametrize("bad", [
    {"type": "nope"},                  # unknown type
    [],                                 # not a dict
    {"type": "step", "params": [1, 2]},  # truthy non-dict params
])
def test_build_path_rejects_bad_recipes(bad):
    with pytest.raises(ValueError):
        run_eval.build_path_from_recipe(bad)


def test_build_path_falsy_params_fall_back_to_defaults():
    # `recipe.get("params", {}) or {}` coerces any *falsy* params to {} (use
    # defaults). Documents that []/None params are tolerated, not rejected.
    p = run_eval.build_path_from_recipe({"type": "step", "params": []})
    expected = 5.0 + 0.5 * (math.pi / 2) + 5.0
    assert p.total_length == pytest.approx(expected, rel=1e-6)


# --- errors_along_path: THE metric core -------------------------------------

def _step_path():
    return run_eval.build_path_from_recipe(
        {"type": "step", "params": {"L1": 5.0, "R": 0.5,
                                    "theta_arc": math.pi / 2, "L2": 5.0}})


def test_on_path_straight_segment_has_zero_error():
    path = _step_path()
    xs = np.array([1.5, 2.0, 2.5, 3.0])
    ys = np.zeros_like(xs)
    yaw = np.zeros_like(xs)
    e_d, e_psi, *_ = run_eval.errors_along_path(path, xs, ys, yaw)
    assert np.allclose(e_d, 0.0, atol=1e-6)
    assert np.allclose(e_psi, 0.0, atol=1e-6)


def test_constant_lateral_offset_gives_constant_signed_e_d():
    path = _step_path()
    xs = np.array([1.5, 2.0, 2.5, 3.0])
    yaw = np.zeros_like(xs)
    d = 0.2
    e_plus, _, *_ = run_eval.errors_along_path(path, xs, np.full_like(xs, d), yaw)
    e_minus, _, *_ = run_eval.errors_along_path(path, xs, np.full_like(xs, -d), yaw)
    # magnitude equals the offset, constant along the straight segment...
    assert np.allclose(np.abs(e_plus), d, atol=1e-6)
    assert np.allclose(e_plus, e_plus[0], atol=1e-9)
    # ...and the sign flips with the side of the path (antisymmetry).
    assert np.allclose(e_plus, -e_minus, atol=1e-6)


# --- fixed_mask_for: nearest-neighbour + the max_dt guard (Bug #1) ----------

def test_fixed_mask_aligned_status():
    rtk = {"stamp": [0, 1, 2, 3], "quality": [4, 4, 1, 4]}
    mask = run_eval.fixed_mask_for([0, 1, 2, 3], rtk)
    assert list(mask) == [True, True, False, True]


def test_fixed_mask_none_without_status():
    assert run_eval.fixed_mask_for([0, 1, 2], None) is None
    assert run_eval.fixed_mask_for([0, 1, 2], {"stamp": [], "quality": []}) is None


def test_fixed_mask_no_tolerance_labels_stale_sample_fixed():
    # Characterizes the legacy behaviour (Bug #1): with only one status sample
    # at t=0, a fix 100 s later is still labelled FIXED from that stale sample.
    rtk = {"stamp": [0.0], "quality": [4]}
    mask = run_eval.fixed_mask_for([0.0, 100.0], rtk)
    assert list(mask) == [True, True]


def test_fixed_mask_max_dt_rejects_stale_sample():
    # With the opt-in time guard, the temporally-distant fix is no longer FIXED.
    rtk = {"stamp": [0.0], "quality": [4]}
    mask = run_eval.fixed_mask_for([0.0, 100.0], rtk, max_dt=1.0)
    assert list(mask) == [True, False]


# --- odom_belief_source: the silent frame-provenance fallback (Bug #3) -------

def test_odom_belief_prefers_zeroed_stream():
    bag = {run_eval.TOPIC_ODOM_ZEROED: {"stamp": [0.0]},
           run_eval.TOPIC_ODOM: {"stamp": [0.0]}}
    arr, label = run_eval.odom_belief_source(bag)
    assert label == run_eval.TOPIC_ODOM_ZEROED
    assert arr is bag[run_eval.TOPIC_ODOM_ZEROED]


def test_odom_belief_falls_back_to_raw_odom_silently():
    # Pins Bug #3: with no zeroed stream the scorer silently uses raw /wheel/odom
    # (a different, un-anchored frame). The label is the only provenance signal —
    # backlog item is to surface it into qc.csv/manifest so this isn't silent.
    bag = {run_eval.TOPIC_ODOM: {"stamp": [0.0]}}
    arr, label = run_eval.odom_belief_source(bag)
    assert label == run_eval.TOPIC_ODOM
    assert arr is bag[run_eval.TOPIC_ODOM]


def test_odom_belief_none_when_no_odom_at_all():
    arr, label = run_eval.odom_belief_source({})
    assert arr is None
    assert label == run_eval.TOPIC_ODOM


def test_path_frame_transform_passes_through_without_anchor():
    x = np.array([1.0, 2.0])
    y = np.array([3.0, 4.0])
    yaw = np.array([0.1, 0.2])
    venue_anchor = type("Anchor", (), {"lat0": 1.0, "lon0": 2.0, "bearing_deg": 42.0})()

    xo, yo, yawo = run_eval.transform_to_path_frame(
        x, y, yaw, venue_anchor, path_frame_anchor=None)

    assert xo is x
    assert yo is y
    assert yawo is yaw


def test_project_rtk_to_local_uses_hardcoded_anchor_when_sidecar_missing():
    x, y, yaw, anchor = run_eval.project_rtk_to_local(
        [path_overlay.LAT0], [path_overlay.LON0], anchor_spec=None)

    assert x[0] == pytest.approx(0.0, abs=1e-6)
    assert y[0] == pytest.approx(0.0, abs=1e-6)
    # With one RTK sample there is no track direction to infer, so yaw is the
    # helper's zero default even though the anchor bearing is hardcoded.
    assert yaw[0] == pytest.approx(0.0)
    assert anchor.lat0 == pytest.approx(path_overlay.LAT0)
    assert anchor.lon0 == pytest.approx(path_overlay.LON0)
    assert anchor.bearing_deg == pytest.approx(path_overlay.BEARING_DEG)
