#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""One-command bag-root -> dataset driver (T11 / Stage 7 orchestration).

Runs the whole pipeline and lays the output out in three layers:

    <out>/raw/        SOURCE.txt — pointer to the retained, untouched bag-root
                      (bags are never copied/modified; this records where they live)
    <out>/derived/    manifest.csv, completeness.json, qc.csv, usable_legs.json,
                      rerun_queue.json, metrics/<leg>.metrics.json,
                      cell_summary.csv, stats.json, *.png figures
    <out>/extracted/  per_sample/ + gnss/ tidy tables + data_dictionary.md
                      + dataset_manifest.json

Stages: manifest -> qc -> run_eval (usable legs only) -> aggregate -> export.
Idempotent: re-run as new sessions land in the bag-root (the quality gate's
rerun queue is cumulative because it always rescans the whole bag-root).

Usage::

    python3 tools/analysis/build_dataset.py <bag_root> --out <dataset_dir> \
        [--experiment scenarios/experiment.yaml] [--split rtk_truth] \
        [--target-n 10]
"""

from __future__ import annotations

import argparse
import json
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import manifest as mf  # noqa: E402
import qc as qcmod  # noqa: E402
import run_eval  # noqa: E402
import aggregate as agg  # noqa: E402
import export as exp  # noqa: E402


def main(argv=None):
    ap = argparse.ArgumentParser(description="Build the dataset from a bag-root.")
    ap.add_argument("bag_root")
    ap.add_argument("--out", required=True, help="dataset output dir")
    ap.add_argument("--experiment",
                    default=os.path.join(mf._REPO_ROOT, "scenarios", "experiment.yaml"))
    ap.add_argument("--split", default="rtk_truth",
                    choices=["rtk_truth", "odom_belief"])
    ap.add_argument("--target-n", type=int, default=None)
    ap.add_argument("--no-rtk-gate", action="store_true",
                    help="GPS-denied venue (e.g. basement): skip qc RTK gate "
                         "and aggregate the odom_belief split")
    args = ap.parse_args(argv)

    # GPS-denied venues have no RTK-truth; fall back to the odom-belief split.
    if args.no_rtk_gate and args.split == "rtk_truth":
        args.split = "odom_belief"
        print("[build] --no-rtk-gate: aggregating odom_belief split "
              "(no RTK ground truth without sky)")

    bag_root = os.path.abspath(args.bag_root)
    out = os.path.abspath(args.out)
    raw_dir = os.path.join(out, "raw")
    derived = os.path.join(out, "derived")
    metrics_dir = os.path.join(derived, "metrics")
    for d in (raw_dir, derived, metrics_dir):
        os.makedirs(d, exist_ok=True)

    # ---- raw layer pointer (never copy the bags) ------------------------
    with open(os.path.join(raw_dir, "SOURCE.txt"), "w", encoding="utf-8") as f:
        f.write(f"bag_root: {bag_root}\n"
                "Bags are retained in place, untouched. This dataset's derived/\n"
                "and extracted/ layers are regenerable from them via\n"
                "tools/analysis/build_dataset.py.\n")

    common = ["--experiment", args.experiment]
    if args.target_n is not None:
        common += ["--target-n", str(args.target_n)]

    # ---- Stage 1: manifest ----------------------------------------------
    print("== manifest ==")
    mf.main([bag_root, "--out-dir", derived, *common])

    # ---- Stage 2: quality gate ------------------------------------------
    print("== qc ==")
    qc_args = [bag_root, "--out-dir", derived, *common]
    if args.no_rtk_gate:
        qc_args.append("--no-rtk-gate")
    qcmod.main(qc_args)

    # ---- Stage 4: run_eval over usable legs -----------------------------
    print("== run_eval (usable legs) ==")
    with open(os.path.join(derived, "usable_legs.json"), encoding="utf-8") as f:
        usable = json.load(f)
    n_ok = 0
    for bag_dir in usable:
        tag = os.path.basename(bag_dir.rstrip("/"))
        out_metrics = os.path.join(metrics_dir, f"{tag}.metrics.json")
        try:
            run_eval.main([bag_dir, "--out", out_metrics])
            n_ok += 1
        except SystemExit as exc:
            print(f"[build] run_eval failed on {bag_dir}: {exc}")
    print(f"[build] evaluated {n_ok}/{len(usable)} usable legs")

    # ---- Stage 5/6: aggregate (cells + stats + figures) -----------------
    print("== aggregate ==")
    if n_ok:
        agg.main([metrics_dir, "--glob", "*.metrics.json",
                  "--split", args.split, "--out-dir", derived])
    else:
        print("[build] no metrics to aggregate (no usable legs yet)")

    # ---- Stage 7: export per-sample + GNSS product ----------------------
    print("== export ==")
    exp.main([bag_root, "--out", out, "--usable-only", "--manifest-dir", derived])

    print(f"\n[build] dataset ready at {out}")
    print(f"[build]   raw/        -> pointer to {bag_root}")
    print(f"[build]   derived/    -> manifest, qc, metrics, cell_summary, stats, figures")
    print(f"[build]   extracted/  -> per-sample tables + GNSS lat/lon product")
    return 0


if __name__ == "__main__":
    sys.exit(main())
