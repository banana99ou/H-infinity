#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Lossless per-sample export + separate GNSS-lat/lon product (T11 / Stage 7).

Two deliverables, both built from the kept-forever bags so future requests need
no robot re-runs:

  1. **Per-sample tidy tables** (the keep-everything layer): for each leg, the
     full aligned time series in the venue-local frame — odom-rate and RTK-rate
     tables with e_d/e_psi/kappa/rho reconstructed against the analytic
     reference (reuses ``run_eval.build_per_sample_frame``). This is what serves
     "the prof wants other data later".
  2. **GNSS lat/lon product (separate)**: the RTK fix and the regular Pixhawk
     GPS in **native lat/lon, untouched** (never projected, never quality-
     filtered), keyed by run_id + timestamp so it merges with the cell data for
     the professor's GNSS-sensor-performance paper.

Plus a **data dictionary** and a **dataset manifest** (provenance: per-leg git
commit, recipe, controller tuning, venue, tool version).

Output (Parquet if pyarrow present, else CSV) under ``<out>/extracted``::

    extracted/per_sample/<leg>__odom.parquet
    extracted/per_sample/<leg>__rtk.parquet
    extracted/gnss/<leg>__gnss_rtk.parquet     # RAW lat/lon, RTK
    extracted/gnss/<leg>__gnss_pix.parquet     # RAW lat/lon, regular GPS (L5)
    extracted/data_dictionary.md
    extracted/dataset_manifest.json

Usage::

    python3 tools/analysis/export.py <bag_root> --out <dataset_dir> \
        [--usable-only]   # restrict to qc's usable_legs.json
"""

from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import run_eval  # noqa: E402
import manifest as mf  # noqa: E402

TOOL_VERSION = "export.py/1"

try:
    import pyarrow as pa  # noqa: F401
    import pyarrow.parquet as pq
    _HAVE_PARQUET = True
except Exception:
    _HAVE_PARQUET = False


def _safe(s):
    return "".join(c if (str(c).isalnum() or c in "-._") else "-" for c in str(s))


def _leg_tag(sidecar, bag_dir):
    run_id = (sidecar or {}).get("run_id") or "run"
    cell_id = (sidecar or {}).get("cell_id") or os.path.basename(bag_dir)
    leg = (sidecar or {}).get("leg") or "leg"
    return f"{_safe(run_id)}__{_safe(cell_id)}__{_safe(leg)}"


def write_table(columns, out_base):
    """Write a dict[str->1d array] as Parquet (preferred) or CSV. Returns path."""
    if not columns:
        return None
    n = max(len(v) for v in columns.values())
    cols = {}
    for k, v in columns.items():
        arr = np.asarray(v)
        if len(arr) != n:  # broadcast scalars / pad
            arr = np.resize(arr, n)
        cols[k] = arr
    if _HAVE_PARQUET:
        path = out_base + ".parquet"
        table = pa.table({k: pa.array(v) for k, v in cols.items()})
        pq.write_table(table, path)
        return path
    # CSV fallback
    path = out_base + ".csv"
    keys = list(cols.keys())
    with open(path, "w", encoding="utf-8") as f:
        f.write(",".join(keys) + "\n")
        for i in range(n):
            f.write(",".join(repr(float(cols[k][i])) if isinstance(cols[k][i], (int, float, np.floating, np.integer)) else str(cols[k][i]) for k in keys) + "\n")
    return path


def _keyed(columns, run_id, cell_id, leg):
    """Prepend identity columns so a table merges with the cell data later."""
    if not columns:
        return columns
    n = max(len(v) for v in columns.values())
    out = {"run_id": np.array([run_id] * n),
           "cell_id": np.array([cell_id] * n),
           "leg": np.array([leg] * n)}
    out.update(columns)
    return out


def export_leg(leg, ps_dir, gnss_dir):
    """Export one leg's per-sample + GNSS tables. Returns a provenance record."""
    bag_dir = leg["bag_dir"]
    sc = leg["sidecar"] or {}
    tag = _leg_tag(sc, bag_dir)
    run_id = sc.get("run_id")
    cell_id = sc.get("cell_id")
    legname = sc.get("leg")

    written = {}
    try:
        bag = run_eval.read_bag(bag_dir)
        frame = run_eval.build_per_sample_frame(bag_dir, sc, bag=bag)
    except Exception as exc:
        return {"bag_dir": bag_dir, "tag": tag, "error": str(exc),
                "written": written}

    # 1) per-sample local-frame tables (odom + rtk)
    if frame.get("odom"):
        written["odom"] = write_table(
            _keyed(frame["odom"], run_id, cell_id, legname),
            os.path.join(ps_dir, f"{tag}__odom"))
    if frame.get("rtk"):
        written["rtk"] = write_table(
            _keyed(frame["rtk"], run_id, cell_id, legname),
            os.path.join(ps_dir, f"{tag}__rtk"))

    # 2) GNSS lat/lon product (RAW, separate) — RTK + regular GPS
    if frame.get("gnss_rtk"):
        written["gnss_rtk"] = write_table(
            _keyed(frame["gnss_rtk"], run_id, cell_id, legname),
            os.path.join(gnss_dir, f"{tag}__gnss_rtk"))
    if frame.get("gnss_pix"):
        written["gnss_pix"] = write_table(
            _keyed(frame["gnss_pix"], run_id, cell_id, legname),
            os.path.join(gnss_dir, f"{tag}__gnss_pix"))

    return {
        "bag_dir": bag_dir, "tag": tag, "run_id": run_id, "cell_id": cell_id,
        "leg": legname,
        "cell_params": sc.get("cell_params"),
        "path_recipe": sc.get("path_recipe"),
        "controller_tuning": sc.get("controller_tuning"),
        "venue": sc.get("venue"),
        "git_commit": sc.get("git_commit"),
        "written": written,
    }


DATA_DICTIONARY = """# Dataset data dictionary

Generated by tools/analysis/export.py. One row per recorded sample.

## extracted/per_sample/<leg>__odom.(parquet|csv)
Vehicle trajectory as the controller saw it (wheel odom), scored against the
analytic reference rebuilt from the leg's recipe.

| column | unit | meaning |
|---|---|---|
| run_id, cell_id, leg | - | identity (join keys) |
| t | s | seconds since leg start |
| stamp | s | absolute bag timestamp (epoch) |
| x, y | m | odom position (venue-local frame) |
| yaw | rad | odom heading |
| v | m/s | longitudinal speed |
| e_d | m | signed cross-track error vs analytic path |
| e_psi | rad | heading error (psi_des - yaw), wrapped |
| kappa | 1/m | path curvature at the closest point |
| rho | 1/m | |kappa| (path-intrinsic schedule) |
| s_star | m | arc-length of the closest point |
| psi_des | rad | VFG desired heading |

## extracted/per_sample/<leg>__rtk.(parquet|csv)
Same columns plus lat/lon, but the trajectory is RTK ground truth (FIXED-only),
projected to the venue-local frame. This is the "truth" half of the ADR-01
belief-vs-truth comparison.

## extracted/gnss/<leg>__gnss_rtk.(parquet|csv)  — RAW, for the GNSS paper
RTK fix in **native lat/lon, untouched** (never projected/filtered). `is_fixed`
is 1 where rtk_status quality==4, 0 otherwise, -1 if rtk_status was absent.

| column | unit | meaning |
|---|---|---|
| run_id, cell_id, leg | - | identity (join keys for merging with cell data) |
| stamp | s | absolute bag timestamp |
| lat, lon | deg (WGS84) | RTK position |
| alt | m | altitude |
| is_fixed | {1,0,-1} | RTK FIXED flag |

## extracted/gnss/<leg>__gnss_pix.(parquet|csv)  — regular GPS (L5)
Pixhawk/MAVROS GPS in native lat/lon (run-along data for the professor's
separate dataset; no role in the path-following paper).
"""


def main(argv=None):
    ap = argparse.ArgumentParser(description="Export per-sample + GNSS tables.")
    ap.add_argument("bag_root")
    ap.add_argument("--out", required=True, help="dataset dir (writes <out>/extracted)")
    ap.add_argument("--usable-only", action="store_true",
                    help="restrict to qc's usable_legs.json under the bag-root")
    ap.add_argument("--manifest-dir", default=None,
                    help="where qc/manifest outputs live (default <bag_root>/_manifest)")
    args = ap.parse_args(argv)

    extracted = os.path.join(args.out, "extracted")
    ps_dir = os.path.join(extracted, "per_sample")
    gnss_dir = os.path.join(extracted, "gnss")
    os.makedirs(ps_dir, exist_ok=True)
    os.makedirs(gnss_dir, exist_ok=True)

    legs = mf.discover_legs(args.bag_root)

    if args.usable_only:
        man_dir = args.manifest_dir or os.path.join(args.bag_root, "_manifest")
        usable_path = os.path.join(man_dir, "usable_legs.json")
        if os.path.isfile(usable_path):
            with open(usable_path, encoding="utf-8") as f:
                usable = set(json.load(f))
            legs = [l for l in legs if l["bag_dir"] in usable]
        else:
            print(f"[export] WARNING: --usable-only but {usable_path} missing; "
                  "exporting all legs")

    provenance = []
    for leg in legs:
        provenance.append(export_leg(leg, ps_dir, gnss_dir))

    with open(os.path.join(extracted, "data_dictionary.md"), "w",
              encoding="utf-8") as f:
        f.write(DATA_DICTIONARY)

    manifest = {
        "tool_version": TOOL_VERSION,
        "parquet": _HAVE_PARQUET,
        "n_legs": len(provenance),
        "git_commits": sorted({p.get("git_commit") for p in provenance
                               if p.get("git_commit")}),
        "legs": provenance,
    }
    with open(os.path.join(extracted, "dataset_manifest.json"), "w",
              encoding="utf-8") as f:
        json.dump(manifest, f, indent=2)
        f.write("\n")

    n_err = sum(1 for p in provenance if p.get("error"))
    fmt = "parquet" if _HAVE_PARQUET else "csv (pyarrow absent)"
    print(f"[export] exported {len(provenance)} legs as {fmt} -> {extracted}")
    if n_err:
        print(f"[export] WARNING: {n_err} legs failed to export (see manifest)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
