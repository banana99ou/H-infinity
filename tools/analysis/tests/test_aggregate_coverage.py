#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Unit tests for the Group-E aggregate-honesty primitives.

  * slice_coverage          -- the coverage gate that decides whether a headline
                               figure is drawn (E1)
  * curvature_tolerance_index -- the de-broken CTI walk: sharpest passing radius
                               anywhere, not "first failure" (E1)

Run: ``pytest tools/analysis/tests/test_aggregate_coverage.py`` or as a script.
"""

from __future__ import annotations

import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_ANALYSIS = os.path.dirname(_HERE)
if _ANALYSIS not in sys.path:
    sys.path.insert(0, _ANALYSIS)

import aggregate as agg  # noqa: E402


def _cell(controller, R, mean, n):
    return {"controller": controller, "R": R,
            "max_e_psi_deg_mean": mean, "n": n}


def test_coverage_sufficient():
    # Both controllers at 2 shared radii, all cells at target N -> draw figure.
    cells = [_cell("LPV", 1.0, 5.0, 10), _cell("LPV", 0.5, 8.0, 10),
             _cell("PID", 1.0, 7.0, 10), _cell("PID", 0.5, 12.0, 10)]
    cov = agg.slice_coverage(cells, target_n=10)
    assert cov["sufficient"] is True
    assert cov["shared_R_across_both_controllers"] == [0.5, 1.0]
    assert cov["under_N_cells"] == []


def test_coverage_one_shared_radius():
    # Both controllers, but only R=1.0 in common -> <2 shared R -> skip figure.
    cells = [_cell("LPV", 1.0, 5.0, 10), _cell("LPV", 0.5, 8.0, 10),
             _cell("PID", 1.0, 7.0, 10)]
    cov = agg.slice_coverage(cells, target_n=10)
    assert cov["sufficient"] is False
    assert cov["shared_R_across_both_controllers"] == [1.0]


def test_coverage_under_target_n():
    # Two shared radii but a cell is below N -> skip figure, name the cell.
    cells = [_cell("LPV", 1.0, 5.0, 10), _cell("LPV", 0.5, 8.0, 3),
             _cell("PID", 1.0, 7.0, 10), _cell("PID", 0.5, 12.0, 10)]
    cov = agg.slice_coverage(cells, target_n=10)
    assert cov["sufficient"] is False
    assert any(c["n"] == 3 for c in cov["under_N_cells"])


def test_cti_does_not_break_at_first_failure():
    # Non-monotonic: passes at R=1.0, FAILS at R=0.7, passes again at R=0.5.
    # The old break-at-first-failure stopped at 0.7 (CTI=1.0). De-broken, the
    # sharper passing R=0.5 is reached -> CTI = 1/0.5 = 2.0.
    cells = [_cell("LPV", 1.0, 5.0, 10), _cell("LPV", 0.7, 25.0, 10),
             _cell("LPV", 0.5, 9.0, 10)]
    cti = agg.curvature_tolerance_index(cells, tol_deg=10.0)
    assert abs(cti["LPV"]["cti_kappa"] - 2.0) < 1e-9
    passing_R = {p["R"] for p in cti["LPV"]["passing_cells"]}
    assert passing_R == {1.0, 0.5}  # 0.7 excluded (over tolerance)


def _run_all():
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for fn in fns:
        fn()
        print(f"{fn.__name__}: OK")
    print(f"--- {len(fns)} aggregate-coverage tests passed ---")


if __name__ == "__main__":
    _run_all()
