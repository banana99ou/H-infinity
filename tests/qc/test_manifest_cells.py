"""Unit tests for cell grouping / inventory (tools/analysis/manifest.py).

Cell identity drives both the dataset pooling and the rerun queue, so a rounding
slip in ``_num`` would silently split or merge cells. These pin that behaviour
and the completeness inventory counts.
"""

from tools.analysis import manifest as mf


# --- _num / cell_key determinism --------------------------------------------

def test_num_rounds_to_six_decimals():
    assert mf._num(0.5) == 0.5
    assert mf._num(0.5000001) == 0.5          # below 1e-6 -> same bucket
    assert mf._num("0.30") == 0.3              # string coerced
    assert mf._num(None) is None
    assert mf._num("not-a-number") is None


def test_cell_key_groups_float_noise_identically():
    a = mf.cell_key({"controller": "lpv-hinf", "v_const": 0.5,
                     "path_family": "step", "radius_m": 0.7})
    b = mf.cell_key({"controller": "lpv-hinf", "v_const": 0.5000001,
                     "path_family": "step", "radius_m": 0.7})
    assert a == b
    assert a == ("lpv-hinf", 0.5, "step", 0.7)


def test_cell_key_separates_beyond_tolerance():
    a = mf.cell_key({"controller": "lpv-hinf", "v_const": 0.5,
                     "path_family": "step", "radius_m": 0.7})
    c = mf.cell_key({"controller": "lpv-hinf", "v_const": 0.5001,
                     "path_family": "step", "radius_m": 0.7})
    assert a != c


def test_cell_key_none_on_empty():
    assert mf.cell_key(None) is None
    assert mf.cell_key({}) is None


# --- completeness inventory -------------------------------------------------

def _row(controller, v, fam, R):
    return {"controller": controller, "v_const": v,
            "path_family": fam, "radius_m": R}


def test_completeness_counts_present_missing_and_under_target():
    expected = {
        mf.cell_key(_row("lpv-hinf", 0.3, "step", 0.5)),
        mf.cell_key(_row("pid", 0.3, "step", 0.5)),
    }
    rows = [
        _row("lpv-hinf", 0.3, "step", 0.5),
        _row("lpv-hinf", 0.3, "step", 0.5),  # lpv cell has 2 legs
        # pid cell has 0 legs
    ]
    comp = mf.completeness(rows, expected, target_n=2)
    assert comp["n_expected_cells"] == 2
    assert comp["n_cells_with_data"] == 1
    assert comp["n_missing_cells"] == 1          # pid cell entirely absent
    assert comp["n_under_target_cells"] == 1     # pid 0/2; lpv 2/2 not under
    assert comp["unexpected_cells"] == []


def test_completeness_flags_unexpected_cells():
    expected = {mf.cell_key(_row("lpv-hinf", 0.3, "step", 0.5))}
    rows = [_row("pid", 0.3, "step", 0.5)]  # a cell not in the matrix
    comp = mf.completeness(rows, expected, target_n=1)
    assert comp["n_missing_cells"] == 1
    assert len(comp["unexpected_cells"]) == 1
