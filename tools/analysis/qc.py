#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Quality gate — re-derive per-leg pass/fail from the bag (T11 / Stage 2).

The sequencer classifies each leg live (sidecar ``classification``); this gate
re-derives the verdict from the *recorded bag*, which is authoritative, and is
what decides inclusion in the dataset. Checks (system_spec §5):

  * **RTK FIXED %** — fraction of the run window with ``rtk_status`` quality==4
    (the authoritative RTK flag; NavSatFix.status can't distinguish RTK on this
    F9P). Pass if >= ``gating.rtk_run_window_pct`` (experiment.yaml, default 95).
  * **No E-stop** — ``/estop`` never went active during the window.
  * **Length** — bag duration ~= analytic arc-length / v_const within +/- X%.
  * **Topic continuity** — required topics present; no odom gap beyond a tol.

Emits ``qc.csv`` (per-leg verdict + reasons, reconciled against the sidecar
verdict), the **usable set**, and ``rerun_queue.json`` — cells below the target
N usable legs. The bag-root is the cumulative store: scanning it each session
makes the queue naturally cumulative.

Usage::

    python3 tools/analysis/qc.py <bag_root> \
        [--experiment scenarios/experiment.yaml] [--target-n 10] \
        [--length-tol 0.30] [--gap-tol 0.5] [--out-dir <dataset>/derived]
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import sys
from collections import defaultdict

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import run_eval  # noqa: E402
import manifest as mf  # noqa: E402


def _window(bag):
    """Run window [t0, t1] from odom stamps (fallback: status, then any)."""
    for topic in (run_eval.TOPIC_ODOM, run_eval.TOPIC_STATUS):
        d = bag.get(topic)
        if d is not None and len(d["stamp"]) >= 2:
            s = d["stamp"]
            return float(s[0]), float(s[-1])
    return None, None


def rtk_fixed_pct(bag, t0, t1):
    """% of rtk_status samples in [t0,t1] reporting quality==4."""
    rs = bag.get(run_eval.TOPIC_RTK_STATUS)
    if rs is None or len(rs["stamp"]) == 0:
        return None, 0
    s = rs["stamp"]
    q = rs["quality"]
    if t0 is not None:
        m = (s >= t0) & (s <= t1)
        q = q[m]
    if len(q) == 0:
        return None, 0
    return 100.0 * float(np.mean(q == run_eval.RTK_FIXED_QUALITY)), int(len(q))


def estop_fired(bag, t0, t1):
    es = bag.get(run_eval.TOPIC_ESTOP)
    if es is None or len(es["stamp"]) == 0:
        return False  # no estop topic recorded -> treat as not fired (warn elsewhere)
    s, a = es["stamp"], es["active"]
    if t0 is not None:
        m = (s >= t0) & (s <= t1)
        a = a[m]
    return bool(np.any(a > 0.5))


def max_odom_gap(bag):
    d = bag.get(run_eval.TOPIC_ODOM)
    if d is None or len(d["stamp"]) < 2:
        return None
    return float(np.max(np.diff(d["stamp"])))


def qc_leg(leg, target_len_v, rtk_pct_min, length_tol, gap_tol,
           no_rtk_gate=False):
    """Evaluate one leg dict (from manifest.discover_legs). Returns a row dict.

    ``no_rtk_gate`` skips the RTK-FIXED checks entirely — for GPS-denied venues
    (e.g. the basement track) where there is no sky and thus no RTK FIXED. In
    that mode RTK-truth metrics are meaningless; aggregate with
    ``--split odom_belief``.
    """
    bag_dir = leg["bag_dir"]
    sc = leg["sidecar"] or {}
    reasons = []
    row = {
        "run_id": sc.get("run_id"), "cell_id": sc.get("cell_id"),
        "leg": sc.get("leg"), "bag_dir": bag_dir,
    }
    cp = sc.get("cell_params") or {}
    row.update({"controller": cp.get("controller"), "v_const": cp.get("v_const"),
                "radius_m": cp.get("radius_m"), "path_family": cp.get("path_family")})

    if leg["sidecar"] is None:
        return {**row, "usable": False, "reasons": "no_sidecar",
                "rtk_fixed_pct": None, "duration_s": None,
                "sidecar_pass": None, "verdict_mismatch": False}

    try:
        bag = run_eval.read_bag(bag_dir)
    except SystemExit as exc:
        return {**row, "usable": False, "reasons": f"bag_read_error:{exc}",
                "rtk_fixed_pct": None, "duration_s": None,
                "sidecar_pass": (sc.get("classification") or {}).get("pass"),
                "verdict_mismatch": False}

    t0, t1 = _window(bag)
    duration = (t1 - t0) if t0 is not None else None

    # 1) RTK FIXED % (skipped at GPS-denied venues via --no-rtk-gate)
    pct, n_rtk = rtk_fixed_pct(bag, t0, t1)
    if not no_rtk_gate:
        if pct is None:
            reasons.append("no_rtk_status")
        elif pct < rtk_pct_min:
            reasons.append(f"rtk_fixed_{pct:.0f}pct<{rtk_pct_min:.0f}")

    # 2) E-stop
    if bag.get(run_eval.TOPIC_ESTOP) is None:
        reasons.append("no_estop_topic")
    elif estop_fired(bag, t0, t1):
        reasons.append("estop_fired")

    # 3) Length vs analytic
    if target_len_v is not None and duration is not None and target_len_v > 0:
        ratio = duration / target_len_v
        if not (1.0 - length_tol <= ratio <= 1.0 + length_tol):
            reasons.append(f"length_ratio_{ratio:.2f}")

    # 4) Continuity
    if bag.get(run_eval.TOPIC_ODOM) is None:
        reasons.append("no_odom")
    else:
        gap = max_odom_gap(bag)
        if gap is not None and gap > gap_tol:
            reasons.append(f"odom_gap_{gap:.2f}s")

    usable = len(reasons) == 0
    sidecar_pass = (sc.get("classification") or {}).get("pass")
    mismatch = (sidecar_pass is not None) and (bool(sidecar_pass) != usable)
    return {**row, "usable": usable, "reasons": ";".join(reasons),
            "rtk_fixed_pct": (round(pct, 1) if pct is not None else None),
            "duration_s": (round(duration, 2) if duration is not None else None),
            "sidecar_pass": sidecar_pass, "verdict_mismatch": mismatch}


def rerun_queue(rows, expected_cells, target_n):
    """Cells (in the matrix) with < target_n usable legs -> how many more."""
    usable_per_cell = defaultdict(int)
    for r in rows:
        if r["usable"]:
            key = mf.cell_key({
                "controller": r["controller"], "v_const": r["v_const"],
                "path_family": r["path_family"], "radius_m": r["radius_m"]})
            if key is not None:
                usable_per_cell[key] += 1
    queue = []
    for key in sorted(expected_cells, key=lambda k: tuple(str(x) for x in k)):
        have = usable_per_cell.get(key, 0)
        if have < target_n:
            queue.append({
                "controller": key[0], "v_const": key[1],
                "path_family": key[2], "radius_m": key[3],
                "usable": have, "target_n": target_n, "needed": target_n - have})
    return queue, usable_per_cell


def main(argv=None):
    ap = argparse.ArgumentParser(description="Quality gate over a bag-root.")
    ap.add_argument("bag_root")
    ap.add_argument("--experiment",
                    default=os.path.join(mf._REPO_ROOT, "scenarios", "experiment.yaml"))
    ap.add_argument("--target-n", type=int, default=None)
    ap.add_argument("--length-tol", type=float, default=0.30,
                    help="allowed fractional deviation of duration vs arc-len/v")
    ap.add_argument("--gap-tol", type=float, default=0.5,
                    help="max allowed odom inter-sample gap [s]")
    ap.add_argument("--rtk-pct-min", type=float, default=None,
                    help="min RTK-FIXED %% of window (default: yaml gating)")
    ap.add_argument("--no-rtk-gate", action="store_true",
                    help="skip RTK checks (GPS-denied venues, e.g. basement)")
    ap.add_argument("--out-dir", default=None)
    args = ap.parse_args(argv)

    out_dir = args.out_dir or os.path.join(args.bag_root, "_manifest")
    os.makedirs(out_dir, exist_ok=True)

    expected, reps, doc = (set(), 0, {})
    if os.path.isfile(args.experiment):
        expected, reps, doc = mf.load_experiment(args.experiment)
    target_n = args.target_n if args.target_n is not None else (reps or 1)
    rtk_pct_min = (args.rtk_pct_min if args.rtk_pct_min is not None
                   else float((doc.get("gating", {}) or {}).get("rtk_run_window_pct", 95)))

    legs = mf.discover_legs(args.bag_root)
    rows = []
    for leg in legs:
        sc = leg["sidecar"] or {}
        # analytic arc-length / v for the length check
        target_len_v = None
        recipe = sc.get("path_recipe")
        cp = sc.get("cell_params") or {}
        v = cp.get("v_const")
        if recipe and v:
            try:
                ct = sc.get("controller_tuning", {}) or {}
                rmin = float(ct.get("R_min", ct.get("r_min", 0.5)))
                p = run_eval.build_path_from_recipe(recipe, r_min_default=rmin)
                target_len_v = p.total_length / float(v)
            except Exception:
                target_len_v = None
        rows.append(qc_leg(leg, target_len_v, rtk_pct_min,
                           args.length_tol, args.gap_tol,
                           no_rtk_gate=args.no_rtk_gate))

    queue, usable_per_cell = rerun_queue(rows, expected, target_n)

    qc_csv = os.path.join(out_dir, "qc.csv")
    cols = ["run_id", "cell_id", "leg", "controller", "v_const", "radius_m",
            "path_family", "usable", "reasons", "rtk_fixed_pct", "duration_s",
            "sidecar_pass", "verdict_mismatch", "bag_dir"]
    with open(qc_csv, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=cols, extrasaction="ignore")
        w.writeheader()
        for r in rows:
            w.writerow(r)

    usable_list = [r["bag_dir"] for r in rows if r["usable"]]
    with open(os.path.join(out_dir, "usable_legs.json"), "w", encoding="utf-8") as f:
        json.dump(usable_list, f, indent=2)
        f.write("\n")
    rerun_path = os.path.join(out_dir, "rerun_queue.json")
    with open(rerun_path, "w", encoding="utf-8") as f:
        json.dump({"target_n": target_n, "rtk_pct_min": rtk_pct_min,
                   "cells": queue}, f, indent=2)
        f.write("\n")

    n_usable = sum(1 for r in rows if r["usable"])
    n_mismatch = sum(1 for r in rows if r["verdict_mismatch"])
    print(f"[qc] {n_usable}/{len(rows)} legs usable; wrote {qc_csv}")
    print(f"[qc] rerun queue: {len(queue)} cells under N={target_n} "
          f"(wrote {rerun_path})")
    if n_mismatch:
        print(f"[qc] WARNING: {n_mismatch} legs disagree with the live sidecar verdict")
    return 0


if __name__ == "__main__":
    sys.exit(main())
