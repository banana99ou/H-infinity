"""Unit tests for the cross-leg statistics (tools/analysis/aggregate.py).

The smoke test only checks that aggregate runs and writes files; here we assert
the actual statistical outputs (Wilcoxon pairing, CTI walk, controller-label
normalisation) so a sign-flip or a pairing bug can't pass silently.
"""

import pytest

from tools.analysis import aggregate


# --- normalise_controller ---------------------------------------------------

@pytest.mark.parametrize("raw,expected", [
    ("lpv-hinf", "LPV"),
    ("LPV", "LPV"),
    ("h-inf", "LPV"),
    ("pid-ff", "PID"),
    ("PID", "PID"),
    (None, None),
    ("something_else", "SOMETHING_ELSE"),  # unknown -> upper-cased, not dropped
])
def test_normalise_controller(raw, expected):
    assert aggregate.normalise_controller(raw) == expected


# --- wilcoxon_lpv_vs_pid ----------------------------------------------------

def _rec(controller, R, val, metric="max_e_psi_deg"):
    return {"controller": controller, "R": R, "metrics": {metric: val}}


def test_wilcoxon_pairs_on_shared_radius_and_reports_winner():
    # LPV strictly better (lower max_e_psi) at all three shared radii.
    records = []
    for R, (lpv, pid) in {1.0: (5.0, 9.0), 0.7: (7.0, 12.0),
                          0.5: (10.0, 18.0)}.items():
        records.append(_rec("LPV", R, lpv))
        records.append(_rec("PID", R, pid))
    out = aggregate.wilcoxon_lpv_vs_pid(records)
    assert out["ok"] is True
    assert out["n_pairs"] == 3
    assert out["shared_R"] == [0.5, 0.7, 1.0]
    assert out["lpv_better_count"] == 3
    assert "p_value" in out and "statistic" in out


def test_wilcoxon_needs_both_controllers():
    records = [_rec("LPV", 1.0, 5.0), _rec("LPV", 0.5, 7.0)]
    out = aggregate.wilcoxon_lpv_vs_pid(records)
    assert out["ok"] is False
    assert "both" in out["reason"].lower()


def test_wilcoxon_needs_two_shared_radii():
    records = [_rec("LPV", 1.0, 5.0), _rec("PID", 1.0, 9.0)]
    out = aggregate.wilcoxon_lpv_vs_pid(records)
    assert out["ok"] is False
    assert out["shared_R"] == [1.0]


# --- curvature_tolerance_index ----------------------------------------------

def _cell(controller, R, max_e_psi):
    return {"controller": controller, "R": R, "max_e_psi_deg_mean": max_e_psi}


def test_cti_stops_at_first_failure_when_monotone():
    # Walk large R -> small R; tol=10deg. Passes R=1.0 and R=0.5, fails R=0.33.
    cells = [_cell("LPV", 1.0, 5.0), _cell("LPV", 0.5, 8.0),
             _cell("LPV", 0.33, 15.0)]
    out = aggregate.curvature_tolerance_index(cells, tol_deg=10.0)
    assert out["LPV"]["cti_kappa"] == pytest.approx(1.0 / 0.5)
    assert len(out["LPV"]["passing_cells"]) == 2


def test_cti_underestimates_on_non_monotone_data():
    # Documents the monotone-walk limitation: R=0.5 fails so the walk STOPS,
    # never crediting the sharper R=0.33 that would have passed. CTI = 1/1.0.
    cells = [_cell("LPV", 1.0, 5.0), _cell("LPV", 0.5, 15.0),
             _cell("LPV", 0.33, 8.0)]
    out = aggregate.curvature_tolerance_index(cells, tol_deg=10.0)
    assert out["LPV"]["cti_kappa"] == pytest.approx(1.0 / 1.0)
    passing_R = [c["R"] for c in out["LPV"]["passing_cells"]]
    assert 0.33 not in passing_R  # the recoverable cell is missed
