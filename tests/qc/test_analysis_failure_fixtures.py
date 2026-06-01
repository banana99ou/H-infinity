import pytest

from tools.analysis import manifest, qc

pytest.importorskip("rosbags")
from tools.analysis.tests import make_fixture


def _leg(root):
    legs = manifest.discover_legs(str(root))
    assert len(legs) == 1
    return legs[0]


def test_estop_fixture_is_rejected_by_qc(tmp_path):
    bag = tmp_path / "estop_leg"
    make_fixture.make(str(bag), estop_fired=True)
    row = qc.qc_leg(_leg(tmp_path), None, 95, 0.30, 0.5)
    assert not row["usable"]
    assert "estop_fired" in row["reasons"]


def test_odom_gap_fixture_is_rejected_by_qc(tmp_path):
    bag = tmp_path / "gap_leg"
    make_fixture.make(str(bag), odom_gap=True)
    row = qc.qc_leg(_leg(tmp_path), None, 95, 0.30, 0.5)
    assert not row["usable"]
    assert "odom_gap" in row["reasons"]


def test_missing_required_topic_is_visible_in_manifest(tmp_path):
    bag = tmp_path / "missing_done"
    make_fixture.make(str(bag), missing_topic="/path_follower/done")
    rows = manifest.build_rows(manifest.discover_legs(str(tmp_path)))
    assert len(rows) == 1
    assert not rows[0]["required_topics_ok"]
    assert "/path_follower/done" in rows[0]["missing_topics"]


def test_sidecar_pass_mismatch_is_reported(tmp_path):
    bag = tmp_path / "mismatch"
    make_fixture.make(str(bag), rtk_bad=True, sidecar_pass_mismatch=True)
    row = qc.qc_leg(_leg(tmp_path), None, 95, 0.30, 0.5)
    assert not row["usable"]
    assert row["sidecar_pass"] is True
    assert row["verdict_mismatch"] is True
