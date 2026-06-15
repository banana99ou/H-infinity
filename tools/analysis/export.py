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
    # The bag-dir basename (e.g. 26_0612_1746_..._leg_1) is the SAME tag
    # build_dataset uses for <tag>.metrics.json, so metrics <-> per-sample tables
    # join on it directly. It embeds the recording timestamp, so two scored
    # attempts of the same cell/rep land in distinct files (no silent overwrite);
    # identity columns inside each table still carry run_id/cell_id/leg for
    # joining to the cell-level data.
    return _safe(os.path.basename(bag_dir.rstrip("/")))


def write_table(columns, out_base):
    """Write a dict[str->1d array] as Parquet (preferred) or CSV. Returns path.

    Every column is one value per recorded sample, so all columns MUST be the
    same length. A mismatch is an upstream bug (a column built at the wrong
    rate); raise rather than pad with ``np.resize``, which fabricates samples by
    cyclic repetition. Callers isolate the failure per table.
    """
    if not columns:
        return None
    lengths = {k: len(np.asarray(v)) for k, v in columns.items()}
    n = max(lengths.values())
    ragged = {k: L for k, L in lengths.items() if L != n}
    if ragged:
        raise ValueError(
            f"ragged per-sample table (expected {n} rows): {ragged}; refusing "
            "to pad (np.resize would fabricate samples by cyclic repetition)")
    cols = {k: np.asarray(v) for k, v in columns.items()}
    if _HAVE_PARQUET:
        path = out_base + ".parquet"
        table = pa.table({k: pa.array(v) for k, v in cols.items()})
        pq.write_table(table, path)
        return path
    # CSV fallback — use the csv module for correct quoting/escaping.
    import csv as _csv
    path = out_base + ".csv"
    keys = list(cols.keys())
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = _csv.writer(f)
        w.writerow(keys)
        for i in range(n):
            w.writerow([cols[k][i] for k in keys])
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

    def _w(name, sub, out_dir):
        # Isolate each table: a ragged column (write_table raises) records the
        # error for this table and continues, rather than aborting the leg/build.
        if not frame.get(sub):
            return
        try:
            written[name] = write_table(
                _keyed(frame[sub], run_id, cell_id, legname),
                os.path.join(out_dir, f"{tag}__{name}"))
        except Exception as exc:
            written[name] = {"error": str(exc)}
            print(f"[export] {tag} {name}: {exc}")

    # 1) per-sample local-frame tables (odom + rtk)
    _w("odom", "odom", ps_dir)
    _w("rtk", "rtk", ps_dir)
    # 2) GNSS lat/lon product (RAW, separate) — RTK + regular GPS
    _w("gnss_rtk", "gnss_rtk", gnss_dir)
    _w("gnss_pix", "gnss_pix", gnss_dir)

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
projected to the per-leg path frame (origin = operator start pin, +x = pin
heading). This is the "truth" half of the ADR-01 belief-vs-truth comparison.

- `yaw` is the **body heading from `/heading/fused`** (heading_node EKF),
  resampled onto the RTK fix times — not the differenced course-over-ground
  (which is used only as a fallback when fused is absent). See the heading caveat
  below.
- `t` is **seconds since the run-window start** (the same origin as the
  `__odom` table, so the two streams share one clock); it is negative over the
  pre-path idle head, which is retained here (this layer is lossless — the
  windowed clip applies only to the scored metrics).

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

## Methods caveat — heading independence (read once; lives ONLY here)

Each error metric is reported on two channels (ADR-01):

- **odom-belief** — the trajectory as the controller saw it (wheel odom). This is
  the **headline** channel: max heading error vs turn radius R is computed from
  the controller's own tracked state, exactly as the simulation computes it from
  the simulated pose. Both PID and LPV-Hinf run on the same odom/gyro, so it is a
  fair, sim-comparable PID-vs-Hinf comparison. It needs **no** RTK frame (odom is
  already anchored at the path origin) — only the run-window clip.
- **RTK-truth** — an independent reality check.
  - **Position** (`e_d`, cross-track) is a genuine independent truth: the RTK fix
    is ~1-2 cm, unrelated to wheel odom, so it confirms odom did not drift. The
    trajectory is anchored on the operator's **start pin**; the residual `e_d`
    (a ~0.1 m floor on the R=1.0 legs) is **real** — driven-vs-commanded radius
    mismatch, not a frame artifact — and is reported as such.
  - **Heading** uses `/heading/fused` (heading_node EKF) as the body-heading
    reference. It is dense and standstill-stable (far better than differenced
    course-over-ground), **but it fuses the same LIMO chassis gyro that wheel
    odom uses** — so it is a good-enough heading *reference*, NOT a fully
    independent witness of odom heading. The RTK *heading* cross-check is
    therefore **secondary/caveated** and degrades under sideslip at small R; the
    RTK *position* cross-check remains fully independent. A fully independent
    body-heading truth would need OptiTrack / a total station (deferred
    hardware). Working assumption: fused heading is accepted as good-enough once
    this caveat is surfaced; if rejected, fall back to the course-over-ground
    cross-check.

This caveat is recorded **once, here** — it is deliberately NOT duplicated into
the per-leg metrics JSONs, the cell CSVs, or the figures.
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
