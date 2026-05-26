#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Ingest + inventory of a recorded bag-root (T11 / dataset Stage 1).

Walks a directory tree of per-leg rosbag2 dirs, pairs each with its
``<bag>.sidecar.json`` (the T7/D3 contract), and emits:

  * ``manifest.csv`` — one row per leg: run_id, cell_id, leg, controller,
    v_const, radius_m, path_family, rep, venue, sidecar classification +
    rtk_summary, duration, required-topics-present, bag/sidecar paths.
  * ``completeness.json`` — present vs expected cells from the experiment
    matrix (``scenarios/experiment.yaml``): which cells are missing and which
    are under the target N legs.

Pairing + a cheap topic-presence check only (no message decode). Shared
helpers (``discover_legs``, ``load_experiment``, ``cell_key``) are imported by
qc.py / build_dataset.py.

Usage::

    python3 tools/analysis/manifest.py <bag_root> \
        [--experiment scenarios/experiment.yaml] [--target-n 10] \
        [--out-dir <dataset_dir>/derived]
"""

from __future__ import annotations

import argparse
import csv
import glob
import json
import os
import sys
from collections import defaultdict

import yaml

_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))

# The D1 topic set a complete leg should contain (system_spec §4). Used for the
# required-topics-present check; GNSS-RTK + the 4 follower topics are the ones
# whose absence invalidates a leg for analysis.
REQUIRED_TOPICS = [
    "/wheel/odom",
    "/cmd_vel", "/cmd_vel_raw", "/estop",
    "/gps_rtk_f9p_helical/gps/fix",
    "/gps_rtk_f9p_helical/gps/rtk_status",
    "/path_follower/status",
    "/path_follower/done",
]


def find_sidecar(bag_dir):
    """Resolve the paired sidecar for a bag dir (T7 naming), or None."""
    bag_dir = bag_dir.rstrip("/")
    base = os.path.basename(bag_dir)
    for cand in (f"{bag_dir}.sidecar.json",
                 os.path.join(bag_dir, f"{base}.sidecar.json")):
        if os.path.isfile(cand):
            return cand
    return None


def _is_bag_dir(d):
    return os.path.isfile(os.path.join(d, "metadata.yaml"))


def discover_legs(bag_root):
    """Find every per-leg bag dir under bag_root, paired with its sidecar.

    Returns a list of dicts: {bag_dir, sidecar_path, sidecar(dict or None)}.
    A rosbag2 dir is identified by a metadata.yaml inside it.
    """
    bag_root = os.path.abspath(bag_root)
    legs = []
    for dirpath, dirnames, filenames in os.walk(bag_root):
        if "metadata.yaml" in filenames:
            dirnames[:] = []  # do not descend into a bag dir
            sc_path = find_sidecar(dirpath)
            sc = None
            if sc_path:
                try:
                    with open(sc_path, "r", encoding="utf-8") as f:
                        sc = json.load(f)
                except Exception:
                    sc = None
            legs.append({"bag_dir": dirpath, "sidecar_path": sc_path,
                         "sidecar": sc})
    legs.sort(key=lambda r: r["bag_dir"])
    return legs


def topics_in_bag(bag_dir):
    """Return the set of topic names recorded in the bag (from metadata.yaml).

    Parses metadata.yaml directly (no message decode). Falls back to an empty
    set on any parse error.
    """
    meta = os.path.join(bag_dir, "metadata.yaml")
    try:
        with open(meta, "r", encoding="utf-8") as f:
            m = yaml.safe_load(f)
        info = m.get("rosbag2_bagfile_information", {})
        topics = set()
        for t in info.get("topics_with_message_count", []):
            name = (t.get("topic_metadata", {}) or {}).get("name")
            if name:
                topics.add(name)
        return topics
    except Exception:
        return set()


def cell_key(cell_params):
    """Canonical per-cell grouping key (controller, v_const, path_family, R)."""
    if not cell_params:
        return None
    return (
        str(cell_params.get("controller")),
        _num(cell_params.get("v_const")),
        str(cell_params.get("path_family")),
        _num(cell_params.get("radius_m")),
    )


def _num(x):
    try:
        return round(float(x), 6)
    except (TypeError, ValueError):
        return None


def load_experiment(path):
    """Load experiment.yaml → (expected_cells set, repetitions, raw dict).

    expected_cells is a set of cell_key tuples from the cartesian product of the
    matrix axes (controller × v_const × path_family × radius_m).
    """
    with open(path, "r", encoding="utf-8") as f:
        doc = yaml.safe_load(f)
    matrix = doc.get("matrix", {}) or {}
    controllers = matrix.get("controller", [])
    speeds = matrix.get("v_const", [])
    families = matrix.get("path_family", [])
    radii = matrix.get("radius_m", [])
    expected = set()
    for c in controllers:
        for v in speeds:
            for fam in families:
                for R in radii:
                    expected.add((str(c), _num(v), str(fam), _num(R)))
    reps = int(doc.get("repetitions", 0) or 0)
    return expected, reps, doc


def build_rows(legs):
    rows = []
    for leg in legs:
        sc = leg["sidecar"] or {}
        cp = sc.get("cell_params") or {}
        present = topics_in_bag(leg["bag_dir"])
        missing = [t for t in REQUIRED_TOPICS if t not in present]
        cls = sc.get("classification") or {}
        rtk = sc.get("rtk_summary") or {}
        wc = sc.get("wallclock") or {}
        rows.append({
            "run_id": sc.get("run_id"),
            "cell_id": sc.get("cell_id"),
            "leg": sc.get("leg"),
            "controller": cp.get("controller"),
            "v_const": cp.get("v_const"),
            "radius_m": cp.get("radius_m"),
            "path_family": cp.get("path_family"),
            "rep": cp.get("rep"),
            "venue": (sc.get("venue") or {}).get("venue_id"),
            "sidecar_pass": cls.get("pass"),
            "rtk_fixed_pct": rtk.get("fixed_pct"),
            "duration_s": wc.get("duration_s"),
            "required_topics_ok": (len(missing) == 0 and bool(present)),
            "missing_topics": ";".join(missing),
            "has_sidecar": leg["sidecar"] is not None,
            "bag_dir": leg["bag_dir"],
            "sidecar_path": leg["sidecar_path"],
        })
    return rows


def completeness(rows, expected_cells, target_n):
    """Present-vs-expected cell inventory. Counts legs (pooled over direction)."""
    present = defaultdict(int)
    for r in rows:
        key = cell_key({
            "controller": r["controller"], "v_const": r["v_const"],
            "path_family": r["path_family"], "radius_m": r["radius_m"]})
        if key is not None:
            present[key] += 1

    cells = []
    for key in sorted(expected_cells, key=lambda k: tuple(str(x) for x in k)):
        n = present.get(key, 0)
        cells.append({
            "controller": key[0], "v_const": key[1],
            "path_family": key[2], "radius_m": key[3],
            "legs_present": n, "target_n": target_n,
            "under_target": n < target_n,
            "missing": n == 0,
        })
    # cells present in data but not in the expected matrix
    unexpected = [list(k) for k in present if k not in expected_cells]
    return {
        "target_n_legs_per_cell": target_n,
        "n_expected_cells": len(expected_cells),
        "n_cells_with_data": sum(1 for c in cells if c["legs_present"] > 0),
        "n_missing_cells": sum(1 for c in cells if c["missing"]),
        "n_under_target_cells": sum(1 for c in cells if c["under_target"]),
        "cells": cells,
        "unexpected_cells": unexpected,
    }


def write_csv(rows, path):
    cols = ["run_id", "cell_id", "leg", "controller", "v_const", "radius_m",
            "path_family", "rep", "venue", "sidecar_pass", "rtk_fixed_pct",
            "duration_s", "required_topics_ok", "missing_topics",
            "has_sidecar", "bag_dir", "sidecar_path"]
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        for r in rows:
            w.writerow(r)


def main(argv=None):
    ap = argparse.ArgumentParser(description="Ingest + inventory a bag-root.")
    ap.add_argument("bag_root", help="directory tree of per-leg bag dirs")
    ap.add_argument("--experiment",
                    default=os.path.join(_REPO_ROOT, "scenarios", "experiment.yaml"))
    ap.add_argument("--target-n", type=int, default=None,
                    help="target usable legs per cell (default: yaml repetitions)")
    ap.add_argument("--out-dir", default=None,
                    help="output dir (default: <bag_root>/_manifest)")
    args = ap.parse_args(argv)

    out_dir = args.out_dir or os.path.join(args.bag_root, "_manifest")
    os.makedirs(out_dir, exist_ok=True)

    legs = discover_legs(args.bag_root)
    rows = build_rows(legs)

    expected, reps, _doc = (set(), 0, {})
    if os.path.isfile(args.experiment):
        expected, reps, _doc = load_experiment(args.experiment)
    target_n = args.target_n if args.target_n is not None else (reps or 1)

    comp = completeness(rows, expected, target_n)

    manifest_csv = os.path.join(out_dir, "manifest.csv")
    completeness_json = os.path.join(out_dir, "completeness.json")
    write_csv(rows, manifest_csv)
    with open(completeness_json, "w", encoding="utf-8") as f:
        json.dump(comp, f, indent=2)
        f.write("\n")

    n_no_sidecar = sum(1 for r in rows if not r["has_sidecar"])
    n_bad_topics = sum(1 for r in rows if not r["required_topics_ok"])
    print(f"[manifest] {len(rows)} legs under {args.bag_root}")
    print(f"[manifest] wrote {manifest_csv}")
    print(f"[manifest] wrote {completeness_json}")
    if n_no_sidecar:
        print(f"[manifest] WARNING: {n_no_sidecar} legs have no sidecar")
    if n_bad_topics:
        print(f"[manifest] WARNING: {n_bad_topics} legs missing required topics")
    print(f"[manifest] cells: {comp['n_cells_with_data']}/{comp['n_expected_cells']} "
          f"with data, {comp['n_missing_cells']} missing, "
          f"{comp['n_under_target_cells']} under target N={target_n}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
