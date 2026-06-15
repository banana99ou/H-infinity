# -*- coding: utf-8 -*-
"""A1 regression: manifest must not silently drop inputs.

Two failure classes used to vanish with no trace:
  * a recording dir with a ``*.db3`` but no ``metadata.yaml`` (crashed recorder)
    — never even discovered;
  * a discovered leg with no sidecar — present in manifest.csv but dropped from
    every cell, so it disappears from completeness with no warning.

Both must now appear in ``discover_dropped``; a stray ``.git`` holding a ``.db3``
must NOT (the walk must prune it).

Run:  python3 tools/analysis/tests/test_manifest_dropped.py   (or pytest)
"""
import json
import os
import sys
import tempfile

_HERE = os.path.dirname(os.path.abspath(__file__))
_ANALYSIS = os.path.abspath(os.path.join(_HERE, ".."))
if _ANALYSIS not in sys.path:
    sys.path.insert(0, _ANALYSIS)

import manifest as mf  # noqa: E402


def _touch(path, content=""):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        f.write(content)


def _make_tree(root):
    # A good leg: metadata + sidecar -> NOT dropped.
    good = os.path.join(root, "leg_good")
    _touch(os.path.join(good, "metadata.yaml"), "rosbag2_bagfile_information: {}\n")
    _touch(os.path.join(good, "leg_good_0.db3"))
    _touch(os.path.join(good, "leg_good.sidecar.json"), "{}")
    # A leg with metadata but NO sidecar -> dropped (no_sidecar).
    nosc = os.path.join(root, "leg_no_sidecar")
    _touch(os.path.join(nosc, "metadata.yaml"), "rosbag2_bagfile_information: {}\n")
    _touch(os.path.join(nosc, "leg_no_sidecar_0.db3"))
    # A db3 with NO metadata (crashed recorder) -> dropped (recording_without_metadata).
    crashed = os.path.join(root, "gnss_sessions", "crashed_session")
    _touch(os.path.join(crashed, "crashed_session_0.db3"))
    # A stray .git holding a db3 -> must be pruned, NOT reported.
    _touch(os.path.join(root, ".git", "objects", "stray_0.db3"))


def test_discover_dropped():
    with tempfile.TemporaryDirectory() as root:
        _make_tree(root)
        legs = mf.discover_legs(root)
        dropped = mf.discover_dropped(root, legs)
        paths = {d["path"]: d["reason"] for d in dropped}

        # The good leg is never flagged.
        assert "leg_good" not in paths, paths

        # No-sidecar leg surfaces with the right reason.
        assert "leg_no_sidecar" in paths, paths
        assert "no sidecar" in paths["leg_no_sidecar"]

        # Crashed recorder (db3, no metadata) surfaces — this is the class that
        # vanished entirely before.
        crashed = os.path.join("gnss_sessions", "crashed_session")
        assert crashed in paths, paths
        assert "without metadata" in paths[crashed]

        # The .git tree is pruned: nothing under it is reported, and it is not
        # discovered as a leg.
        assert not any(".git" in p for p in paths), paths
        assert not any(".git" in leg["bag_dir"] for leg in legs), legs

    print("test_discover_dropped: OK")


def test_leg_provenance():
    # Permissive default: anything not smoke/manual/shakedown/test is paper.
    assert mf.leg_provenance("rooftop_0612_fri")[0] is True
    assert mf.leg_provenance("rooftop")[0] is True
    assert mf.leg_provenance("smoke_2026_06_05_field")[0] is False
    assert mf.leg_provenance("manual_drive_2026_05_29")[0] is False
    assert mf.leg_provenance(None)[0] is False
    # Allowlist (strict) mode: only listed run_ids count.
    allow = ["rooftop_0612_fri"]
    assert mf.leg_provenance("rooftop_0612_fri", allow)[0] is True
    assert mf.leg_provenance("rooftop", allow)[0] is False  # pedestrian-road venue
    print("test_leg_provenance: OK")


def test_cross_session_pool():
    # Same nominal cell, two run_ids -> flagged; single run_id -> not.
    rows = [
        {"is_paper": True, "run_id": "A", "controller": "lpv", "v_const": 1.0,
         "path_family": "step", "radius_m": 0.5},
        {"is_paper": True, "run_id": "B", "controller": "lpv", "v_const": 1.0,
         "path_family": "step", "radius_m": 0.5},
        {"is_paper": True, "run_id": "A", "controller": "pid", "v_const": 1.0,
         "path_family": "step", "radius_m": 0.5},
        # non-paper must not contribute to pooling
        {"is_paper": False, "run_id": "smoke", "controller": "lpv", "v_const": 1.0,
         "path_family": "step", "radius_m": 0.5},
    ]
    comp = mf.completeness(rows, set(), target_n=10)
    xs = comp["cross_session_cells"]
    assert len(xs) == 1, xs
    assert sorted(xs[0]["run_ids"]) == ["A", "B"], xs
    print("test_cross_session_pool: OK")


if __name__ == "__main__":
    test_discover_dropped()
    test_leg_provenance()
    test_cross_session_pool()
