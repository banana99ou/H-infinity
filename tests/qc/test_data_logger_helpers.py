import json
import os

import Data_Logger


def test_leg_dirname_sanitizes_fields():
    name = Data_Logger.build_leg_dirname("run 1", "cell/R=0.7", "A to B")
    assert " " not in name
    assert "/" not in name
    assert "run-1" in name
    assert "cell-R-0.7" in name


def test_sidecar_contains_d3_fields(tmp_path):
    sidecar = Data_Logger.build_sidecar(
        run_id="run",
        cell_id="cell",
        leg="AtoB",
        cell_params={"controller": "lpv-hinf", "v_const": 0.2},
        path_recipe={"type": "step", "params": {"R": 0.7}},
        venue_id="smoke",
        start_pin_id="S1",
        end_pin_id="E1",
        rtk_summary={"fixed_pct": 100.0},
        classification={"pass": True, "reason": "ok"},
        wallclock={"duration_s": 1.0},
        controller_tuning={"controller_type": "lpv-hinf"},
        bag_path=str(tmp_path / "bag"),
        git_commit="test",
        path_frame_anchor={"pin_id": "S1", "lat": 1.0, "lon": 2.0, "heading_deg": 90.0},
    )
    for key in (
        "schema_version",
        "run_id",
        "cell_id",
        "leg",
        "cell_params",
        "path_recipe",
        "venue",
        "rtk_summary",
        "classification",
        "wallclock",
        "controller_tuning",
        "git_commit",
    ):
        assert key in sidecar
    assert sidecar["venue"]["path_frame_anchor"]["pin_id"] == "S1"


def test_write_sidecar_is_paired_with_bag_dir(tmp_path):
    bag = tmp_path / "leg_bag"
    bag.mkdir()
    out = Data_Logger.write_sidecar(str(bag), {"ok": True})
    assert out == os.path.join(str(bag), "leg_bag.sidecar.json")
    with open(out, "r", encoding="utf-8") as f:
        assert json.load(f) == {"ok": True}
