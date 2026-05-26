#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Cross-leg aggregation + figures (T11 / A2, A3-plot).

Consumes many per-leg metrics JSONs produced by ``run_eval.py``, groups them by
experiment cell, computes per-cell summary statistics over the N repetitions,
runs a Wilcoxon signed-rank test (LPV vs PID) and emits the paper figures:

  * **headline plot** — max heading error vs turn radius R, LPV vs PID, error
    bars over reps (A2 headline).
  * **Curvature Tolerance Index (CTI)** — per controller, the largest curvature
    (1/R) at which a tolerance is still met (default: max heading error <= 10
    deg). Reported as a table + a bar chart.
  * **compute-cost figure** — per-cycle controller time on the NUC from
    ``/path_follower/timing`` (A3), as a box/violin-style summary per controller.

Which split is aggregated (odom-belief vs RTK-truth) is selectable with
``--split`` (default rtk_truth — the ground-truth claim).

Usage::

    python3 tools/analysis/aggregate.py /path/to/metrics_dir \\
        [--glob '*.metrics.json'] [--split rtk_truth|odom_belief] \\
        [--cti-tol-deg 10.0] [--out-dir tools/analysis/out]

Real-bag validation is DEFERRED to the trip; this runs on whatever metrics
JSONs exist and skips figures it lacks data for, with explicit warnings.
"""

from __future__ import annotations

import argparse
import glob
import json
import os
import sys
from collections import defaultdict

import numpy as np

# Headless rendering (no display on the NUC / CI).
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

try:
    from scipy.stats import wilcoxon
except Exception as exc:  # pragma: no cover
    raise SystemExit(f"[aggregate] scipy is required: {exc}")


# Controller-label normalisation. Sidecars carry controller_tuning; the
# controller type usually lives there (controller_type) or in cell params.
LPV_ALIASES = {"lpv", "lpv-hinf", "lpv_hinf", "hinf", "h-infinity", "h-inf"}
PID_ALIASES = {"pid", "pid-ff", "pid_ff", "pid+ff"}


def normalise_controller(name):
    if name is None:
        return None
    n = str(name).lower().strip()
    if n in LPV_ALIASES:
        return "LPV"
    if n in PID_ALIASES:
        return "PID"
    return n.upper()


# =============================================================================
# Load + flatten per-leg records
# =============================================================================

def load_records(metrics_dir, pattern, split):
    """Load every per-leg metrics JSON and flatten to a list of dicts.

    Each output record carries: controller, R, cell_id, run_id, and the scored
    metrics for the selected split plus shared compute-cost / steering-effort.
    """
    paths = sorted(glob.glob(os.path.join(metrics_dir, "**", pattern),
                              recursive=True))
    paths += sorted(glob.glob(os.path.join(metrics_dir, pattern)))
    paths = sorted(set(paths))
    if not paths:
        raise SystemExit(
            f"[aggregate] no files matching {pattern!r} under {metrics_dir!r}.\n"
            "Run run_eval.py on each leg first.")

    records = []
    for p in paths:
        try:
            with open(p, "r", encoding="utf-8") as f:
                d = json.load(f)
        except Exception as exc:
            print(f"[aggregate] WARNING: skipping unreadable {p}: {exc}")
            continue

        tuning = d.get("controller_tuning", {}) or {}
        controller = normalise_controller(
            tuning.get("controller_type")
            or tuning.get("controller")
            or d.get("controller_type"))
        recipe = d.get("path_recipe", {}) or {}
        params = recipe.get("params", {}) or {}
        R = params.get("R")

        split_metrics = (d.get("metrics", {}) or {}).get(split)
        records.append({
            "file": p,
            "controller": controller,
            "R": float(R) if R is not None else None,
            "cell_id": d.get("cell_id"),
            "run_id": d.get("run_id"),
            "recipe_type": recipe.get("type"),
            "metrics": split_metrics,
            "compute_cost": d.get("compute_cost"),
            "steering_effort": d.get("steering_effort"),
        })
    return records, paths


# =============================================================================
# Per-cell aggregation
# =============================================================================

_METRIC_KEYS = [
    "rms_e_d", "max_e_d", "rms_e_psi_deg", "max_e_psi_deg",
    "settling_time", "terminal_pos_err_m", "terminal_heading_err_deg",
]


def aggregate_cells(records):
    """Group by (controller, R) and summarise each metric over N reps."""
    groups = defaultdict(list)
    for r in records:
        if r["metrics"] is None:
            continue
        groups[(r["controller"], r["R"])].append(r["metrics"])

    cells = []
    for (controller, R), ms in sorted(
            groups.items(), key=lambda kv: (str(kv[0][0]), kv[0][1] or 0)):
        summary = {"controller": controller, "R": R, "n": len(ms)}
        for key in _METRIC_KEYS:
            vals = np.array([m[key] for m in ms
                             if m.get(key) is not None], dtype=float)
            if len(vals):
                summary[f"{key}_mean"] = float(np.mean(vals))
                summary[f"{key}_std"] = float(np.std(vals, ddof=1) if len(vals) > 1 else 0.0)
                summary[f"{key}_n"] = int(len(vals))
            else:
                summary[f"{key}_mean"] = None
                summary[f"{key}_std"] = None
                summary[f"{key}_n"] = 0
        cells.append(summary)
    return cells


# =============================================================================
# Wilcoxon LPV vs PID
# =============================================================================

def wilcoxon_lpv_vs_pid(records, metric="max_e_psi_deg"):
    """Paired Wilcoxon signed-rank, LPV vs PID, paired on R.

    For each R present in BOTH controllers, take the mean of that controller's
    reps at that R; pair across controllers and test. Pairing on R keeps it a
    legitimate paired test (same operating condition, different controller).
    """
    by_ctrl_R = defaultdict(lambda: defaultdict(list))
    for r in records:
        if r["metrics"] is None or r["controller"] is None or r["R"] is None:
            continue
        val = r["metrics"].get(metric)
        if val is not None:
            by_ctrl_R[r["controller"]][r["R"]].append(float(val))

    if "LPV" not in by_ctrl_R or "PID" not in by_ctrl_R:
        return {"metric": metric, "ok": False,
                "reason": "need both LPV and PID legs to pair"}

    radii = sorted(set(by_ctrl_R["LPV"]) & set(by_ctrl_R["PID"]))
    if len(radii) < 2:
        return {"metric": metric, "ok": False,
                "reason": f"need >=2 shared R values, have {len(radii)}",
                "shared_R": radii}

    lpv = np.array([np.mean(by_ctrl_R["LPV"][R]) for R in radii])
    pid = np.array([np.mean(by_ctrl_R["PID"][R]) for R in radii])
    diff = lpv - pid
    if np.allclose(diff, 0.0):
        return {"metric": metric, "ok": False,
                "reason": "all paired differences are zero",
                "shared_R": radii}
    try:
        stat, pval = wilcoxon(lpv, pid)
    except ValueError as exc:
        return {"metric": metric, "ok": False, "reason": str(exc),
                "shared_R": radii}
    return {
        "metric": metric, "ok": True, "n_pairs": len(radii),
        "shared_R": radii,
        "lpv_mean_per_R": lpv.tolist(),
        "pid_mean_per_R": pid.tolist(),
        "statistic": float(stat), "p_value": float(pval),
        "lpv_better_count": int(np.sum(lpv < pid)),
    }


# =============================================================================
# Curvature Tolerance Index
# =============================================================================

def curvature_tolerance_index(cells, tol_deg=10.0):
    """Largest curvature 1/R at which max heading error stays <= tol_deg.

    For each controller, walk its cells from gentle (large R) to sharp (small
    R); CTI is the highest 1/R whose mean max_e_psi is within tolerance. If even
    the gentlest cell fails, CTI is 0; if all pass, CTI is the sharpest 1/R.
    """
    by_ctrl = defaultdict(list)
    for c in cells:
        if c["R"] and c["R"] > 0 and c.get("max_e_psi_deg_mean") is not None:
            by_ctrl[c["controller"]].append(
                (c["R"], c["max_e_psi_deg_mean"]))

    out = {}
    for ctrl, items in by_ctrl.items():
        items.sort(key=lambda t: t[0], reverse=True)  # large R -> small R
        cti = 0.0
        passing = []
        for R, e in items:
            if e <= tol_deg:
                cti = max(cti, 1.0 / R)
                passing.append({"R": R, "kappa": 1.0 / R, "max_e_psi_deg": e})
            else:
                break  # first failure at increasing curvature stops the run
        out[ctrl] = {"cti_kappa": cti, "tol_deg": tol_deg,
                     "passing_cells": passing}
    return out


# =============================================================================
# Figures
# =============================================================================

def plot_headline(cells, out_path, split):
    """Max heading error vs R, LPV vs PID, error bars over reps."""
    by_ctrl = defaultdict(list)
    for c in cells:
        if c["R"] is not None and c.get("max_e_psi_deg_mean") is not None:
            by_ctrl[c["controller"]].append(
                (c["R"], c["max_e_psi_deg_mean"], c.get("max_e_psi_deg_std") or 0.0))

    if not by_ctrl:
        print("[aggregate] WARNING: no (R, max_e_psi) data; skipping headline plot")
        return None

    fig, ax = plt.subplots(figsize=(7, 5))
    markers = {"LPV": "o", "PID": "s"}
    for ctrl, pts in sorted(by_ctrl.items()):
        pts.sort(key=lambda t: t[0])
        R = [p[0] for p in pts]
        mean = [p[1] for p in pts]
        std = [p[2] for p in pts]
        ax.errorbar(R, mean, yerr=std, marker=markers.get(ctrl, "^"),
                    capsize=4, linewidth=1.5, label=ctrl)
    ax.set_xlabel("Turn radius R [m]")
    ax.set_ylabel("Max heading error [deg]")
    ax.set_title(f"Max heading error vs R  ({split})")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f"[aggregate] wrote {out_path}")
    return out_path


def plot_cti(cti, out_path):
    """Bar chart of the Curvature Tolerance Index per controller."""
    items = [(c, v["cti_kappa"]) for c, v in cti.items()]
    if not items:
        print("[aggregate] WARNING: no CTI data; skipping CTI plot")
        return None
    items.sort()
    fig, ax = plt.subplots(figsize=(5, 4))
    labels = [i[0] for i in items]
    vals = [i[1] for i in items]
    ax.bar(labels, vals, color=["tab:blue", "tab:orange", "tab:green"][:len(vals)])
    ax.set_ylabel("CTI = max tolerable curvature 1/R [1/m]")
    tol = next(iter(cti.values()))["tol_deg"]
    ax.set_title(f"Curvature Tolerance Index (max e_psi <= {tol:g} deg)")
    ax.grid(True, axis="y", alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f"[aggregate] wrote {out_path}")
    return out_path


def plot_compute_cost(records, out_path):
    """Per-controller per-cycle compute cost (A3). Box plot of mean_ms per leg.

    Each leg contributes its mean_ms (and we annotate max). With many legs this
    becomes a per-controller distribution; with one leg it is a single point.
    """
    by_ctrl = defaultdict(list)
    maxes = defaultdict(list)
    for r in records:
        cc = r.get("compute_cost") or {}
        if cc.get("mean_ms") is not None:
            by_ctrl[r["controller"] or "?"].append(cc["mean_ms"])
        if cc.get("max_ms") is not None:
            maxes[r["controller"] or "?"].append(cc["max_ms"])

    if not by_ctrl:
        print("[aggregate] WARNING: no /path_follower/timing data; "
              "skipping compute-cost figure")
        return None

    labels = sorted(by_ctrl)
    data = [by_ctrl[k] for k in labels]
    fig, ax = plt.subplots(figsize=(6, 4))
    ax.boxplot(data, labels=labels, showmeans=True)
    for i, k in enumerate(labels, start=1):
        if maxes[k]:
            ax.scatter([i] * len(maxes[k]), maxes[k], marker="x",
                       color="red", label="per-leg max" if i == 1 else None)
    ax.set_ylabel("Per-cycle controller time [ms]")
    ax.set_title("Compute cost on NUC (/path_follower/timing)")
    ax.grid(True, axis="y", alpha=0.3)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f"[aggregate] wrote {out_path}")
    return out_path


# =============================================================================
# Main
# =============================================================================

def main(argv=None):
    ap = argparse.ArgumentParser(
        description="Aggregate per-leg metrics JSONs into cell stats + figures.")
    ap.add_argument("metrics_dir", help="directory holding per-leg metrics JSONs")
    ap.add_argument("--glob", default="*.metrics.json",
                    help="filename pattern (default: *.metrics.json)")
    ap.add_argument("--split", default="rtk_truth",
                    choices=["rtk_truth", "odom_belief"],
                    help="which metrics split to aggregate (default rtk_truth)")
    ap.add_argument("--cti-tol-deg", type=float, default=10.0,
                    help="max-heading-error tolerance for the CTI [deg]")
    ap.add_argument("--wilcoxon-metric", default="max_e_psi_deg",
                    help="metric for the LPV-vs-PID Wilcoxon test")
    ap.add_argument("--out-dir", default=None,
                    help="output dir for figures + summary "
                         "(default: <metrics_dir>/aggregate_out)")
    args = ap.parse_args(argv)

    out_dir = args.out_dir or os.path.join(args.metrics_dir, "aggregate_out")
    os.makedirs(out_dir, exist_ok=True)

    records, paths = load_records(args.metrics_dir, args.glob, args.split)
    print(f"[aggregate] loaded {len(records)} legs from {len(paths)} files "
          f"(split={args.split})")

    cells = aggregate_cells(records)
    wilcox = wilcoxon_lpv_vs_pid(records, metric=args.wilcoxon_metric)
    cti = curvature_tolerance_index(cells, tol_deg=args.cti_tol_deg)

    headline = plot_headline(cells, os.path.join(out_dir, "headline_max_epsi_vs_R.png"),
                             args.split)
    cti_png = plot_cti(cti, os.path.join(out_dir, "curvature_tolerance_index.png"))
    cost_png = plot_compute_cost(records, os.path.join(out_dir, "compute_cost.png"))

    summary = {
        "split": args.split,
        "n_legs": len(records),
        "n_cells": len(cells),
        "cells": cells,
        "wilcoxon_lpv_vs_pid": wilcox,
        "curvature_tolerance_index": cti,
        "figures": {
            "headline": headline,
            "cti": cti_png,
            "compute_cost": cost_png,
        },
    }
    summary_path = os.path.join(out_dir, "aggregate_summary.json")
    with open(summary_path, "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2)
        f.write("\n")
    print(f"[aggregate] wrote {summary_path}")

    if wilcox.get("ok"):
        print(f"[aggregate] Wilcoxon ({wilcox['metric']}): "
              f"p={wilcox['p_value']:.4g}, n_pairs={wilcox['n_pairs']}, "
              f"LPV better in {wilcox['lpv_better_count']}/{wilcox['n_pairs']}")
    else:
        print(f"[aggregate] Wilcoxon not run: {wilcox.get('reason')}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
