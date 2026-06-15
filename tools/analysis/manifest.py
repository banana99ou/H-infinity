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
    "/reference_path",
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
        # Never descend into VCS / hidden dirs (e.g. the stray .git the artifact
        # sync creates inside the bag-root) — wasted walk, and not recordings.
        dirnames[:] = [d for d in dirnames if d != ".git" and not d.startswith(".")]
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


def discover_dropped(bag_root, legs):
    """Inputs that look like recordings but are NOT ingested as usable legs.

    Without this, two failure classes vanish silently:
      * a dir holding a ``*.db3``/``*.mcap``/``*.bag`` but no ``metadata.yaml``
        (recorder crashed / never finalized) — never even discovered;
      * a discovered leg with no paired sidecar — present in manifest.csv but
        dropped from every cell (no cell_params), so it disappears from
        completeness/usable counts with no trace.

    Returns a list of {path (relative to bag_root), reason}. ``path`` is
    relative so the list is stable across machines.
    """
    bag_root = os.path.abspath(bag_root)
    leg_dirs = {leg["bag_dir"] for leg in legs}
    dropped = []
    for dirpath, dirnames, filenames in os.walk(bag_root):
        dirnames[:] = [d for d in dirnames if d != ".git" and not d.startswith(".")]
        if "metadata.yaml" in filenames:
            dirnames[:] = []  # a discovered leg dir — handled below via `legs`
            continue
        has_recording = any(
            fn.endswith((".db3", ".mcap", ".bag")) for fn in filenames)
        if has_recording:
            dropped.append({
                "path": os.path.relpath(dirpath, bag_root),
                "reason": "recording without metadata.yaml "
                          "(unfinalized / crashed recorder)",
            })
    for leg in legs:
        if leg["sidecar"] is None:
            dropped.append({
                "path": os.path.relpath(leg["bag_dir"], bag_root),
                "reason": "no sidecar (cannot derive cell; excluded from cells)",
            })
    dropped.sort(key=lambda d: d["path"])
    return dropped


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


def topic_counts_in_bag(bag_dir):
    """{topic: message_count} from metadata.yaml (no message decode).

    Distinct from topics_in_bag(): a QoS-poisoned recorder subscription leaves
    the topic PRESENT in metadata with count 0, so presence checks pass while
    the data is gone. Counts are what the quick gate needs.
    """
    meta = os.path.join(bag_dir, "metadata.yaml")
    try:
        with open(meta, "r", encoding="utf-8") as f:
            m = yaml.safe_load(f)
        info = m.get("rosbag2_bagfile_information", {})
        counts = {}
        for t in info.get("topics_with_message_count", []):
            name = (t.get("topic_metadata", {}) or {}).get("name")
            if name:
                counts[name] = counts.get(name, 0) + int(t.get("message_count", 0))
        return counts
    except Exception:
        return {}


def quick_gate(bag_dir):
    """Bag-level quick gate — cheap enough to run ON-ROBOT right after each
    bag closes (stdlib + yaml only; no rosbags/numpy, no message decode).

    Catches the recorder-side failure class the live classifier cannot see
    from its own topic subscriptions (e.g. a QoS-poisoned /cmd_vel_raw
    subscription recording 0 msgs while the run itself looked healthy,
    rooftop 2026-06-11). The full laptop gate (qc.py) stays authoritative;
    this is the subset of it derivable from metadata.yaml alone.

    Returns {"pass": bool, "reasons": [str], "duration_s": float|None}.
    """
    meta = os.path.join(bag_dir, "metadata.yaml")
    reasons = []
    try:
        with open(meta, "r", encoding="utf-8") as f:
            m = yaml.safe_load(f)
        info = m.get("rosbag2_bagfile_information", {})
        duration_s = float((info.get("duration", {}) or {}).get("nanoseconds", 0)) * 1e-9
    except Exception:
        return {"pass": False, "reasons": ["metadata_unreadable"], "duration_s": None}
    counts = topic_counts_in_bag(bag_dir)
    absent = [t for t in REQUIRED_TOPICS if counts.get(t, 0) == 0]
    if absent:
        reasons.append("missing_topics:" + "+".join(absent))
    # estop_cli relays cmd_vel_raw -> cmd_vel 1:1; a large count gap means the
    # recorder captured only a fraction of the raw stream.
    n_raw = counts.get("/cmd_vel_raw", 0)
    n_cmd = counts.get("/cmd_vel", 0)
    if n_cmd > 0 and not absent and n_raw < 0.5 * n_cmd:
        reasons.append(f"cmd_vel_raw_undercount_{n_raw}/{n_cmd}")
    if duration_s < 0.5:
        reasons.append(f"bag_too_short_{duration_s:.2f}s")
    return {"pass": not reasons, "reasons": reasons,
            "duration_s": round(duration_s, 3)}


# run_id substrings that mark a NON-paper run (shakedown / smoke / manual
# teleop / ad-hoc test). Matched case-insensitively against the sidecar run_id.
# Provenance matters because cells key only on (controller,v,family,R): a smoke
# leg at the same nominal cell would otherwise pool into and corrupt paper stats
# (the 2026-06-08 smoke leg passes QC today).
NON_PAPER_RUN_PATTERNS = ("smoke", "manual", "shakedown", "test")


def leg_provenance(run_id, allowlist=None,
                   exclude_patterns=NON_PAPER_RUN_PATTERNS):
    """Classify a leg's run_id. Returns (is_paper: bool, reason: str).

    If ``allowlist`` is non-empty, a run_id MUST be in it to count as paper
    (the strict mode for a finalized dataset). Otherwise any run_id that does
    not match an exclude pattern counts (the permissive default mid-campaign,
    where paper run_ids vary by session — e.g. ``rooftop_0612_fri``).
    """
    if run_id is None:
        return False, "no_run_id"
    rid = str(run_id)
    low = rid.lower()
    for pat in exclude_patterns:
        if pat in low:
            return False, f"non_paper_run_id(*{pat}*)"
    if allowlist:
        return (True, "allowlisted") if rid in allowlist else (False, "not_in_allowlist")
    return True, "ok"


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


def build_rows(legs, allowlist=None):
    rows = []
    for leg in legs:
        sc = leg["sidecar"] or {}
        cp = sc.get("cell_params") or {}
        present = topics_in_bag(leg["bag_dir"])
        missing = [t for t in REQUIRED_TOPICS if t not in present]
        cls = sc.get("classification") or {}
        rtk = sc.get("rtk_summary") or {}
        wc = sc.get("wallclock") or {}
        is_paper, prov_reason = leg_provenance(sc.get("run_id"), allowlist)
        rows.append({
            "is_paper": is_paper,
            "provenance": prov_reason,
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
    """Present-vs-expected cell inventory. Counts legs (pooled over direction).

    Only paper-provenance legs (``is_paper``) count toward a cell. A cell whose
    paper legs span >1 run_id is flagged ``cross_session`` — different sessions
    can be different venues/anchors (e.g. pedestrian-road ``rooftop`` vs actual
    ``rooftop_0612_fri``, pins 2.82 m apart), so pooling them mixes frames; the
    inventory warns rather than silently averaging.
    """
    present = defaultdict(int)
    run_ids_by_cell = defaultdict(set)
    for r in rows:
        if not r.get("is_paper", True):
            continue
        key = cell_key({
            "controller": r["controller"], "v_const": r["v_const"],
            "path_family": r["path_family"], "radius_m": r["radius_m"]})
        if key is not None:
            present[key] += 1
            run_ids_by_cell[key].add(r.get("run_id"))

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
    # cells pooling legs from >1 run_id (cross-session frame-mixing hazard)
    cross_session = [
        {"cell": list(k), "run_ids": sorted(str(r) for r in v)}
        for k, v in run_ids_by_cell.items() if len(v) > 1
    ]
    return {
        "target_n_legs_per_cell": target_n,
        "n_expected_cells": len(expected_cells),
        "n_cells_with_data": sum(1 for c in cells if c["legs_present"] > 0),
        "n_missing_cells": sum(1 for c in cells if c["missing"]),
        "n_under_target_cells": sum(1 for c in cells if c["under_target"]),
        "cells": cells,
        "unexpected_cells": unexpected,
        "cross_session_cells": cross_session,
    }


def write_csv(rows, path):
    cols = ["is_paper", "provenance", "run_id", "cell_id", "leg", "controller",
            "v_const", "radius_m", "path_family", "rep", "venue", "sidecar_pass",
            "rtk_fixed_pct", "duration_s", "required_topics_ok", "missing_topics",
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
    ap.add_argument("--paper-run-ids", default=None,
                    help="comma-separated run_id allowlist; only these count as "
                         "paper data (default: permissive — exclude *smoke*/"
                         "*manual*/*shakedown*/*test* only)")
    args = ap.parse_args(argv)

    out_dir = args.out_dir or os.path.join(args.bag_root, "_manifest")
    os.makedirs(out_dir, exist_ok=True)

    allowlist = ([s.strip() for s in args.paper_run_ids.split(",") if s.strip()]
                 if args.paper_run_ids else None)

    legs = discover_legs(args.bag_root)
    rows = build_rows(legs, allowlist=allowlist)

    expected, reps, _doc = (set(), 0, {})
    if os.path.isfile(args.experiment):
        expected, reps, _doc = load_experiment(args.experiment)
    target_n = args.target_n if args.target_n is not None else (reps or 1)

    comp = completeness(rows, expected, target_n)
    comp["dropped_inputs"] = discover_dropped(args.bag_root, legs)

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
    dropped = comp["dropped_inputs"]
    if dropped:
        print(f"[manifest] WARNING: {len(dropped)} dropped input(s) "
              "(would otherwise vanish silently):", file=sys.stderr)
        for d in dropped:
            print(f"[manifest]   - {d['path']}: {d['reason']}", file=sys.stderr)
    n_non_paper = sum(1 for r in rows if not r["is_paper"])
    if n_non_paper:
        print(f"[manifest] {n_non_paper} leg(s) excluded as non-paper provenance "
              "(smoke/manual/etc.) — not counted toward cells")
    if comp["cross_session_cells"]:
        print(f"[manifest] WARNING: {len(comp['cross_session_cells'])} cell(s) "
              "pool legs from >1 run_id (cross-session frame-mixing hazard):",
              file=sys.stderr)
        for c in comp["cross_session_cells"]:
            print(f"[manifest]   - {c['cell']}: {c['run_ids']}", file=sys.stderr)
    print(f"[manifest] cells: {comp['n_cells_with_data']}/{comp['n_expected_cells']} "
          f"with data, {comp['n_missing_cells']} missing, "
          f"{comp['n_under_target_cells']} under target N={target_n}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
