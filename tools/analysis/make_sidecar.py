#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Author a sidecar for a MANUALLY recorded bag (no sequencer).

The experiment sequencer (T6) writes the D3 sidecar automatically. For ad-hoc /
indoor (basement) runs recorded by hand (Data_Logger.py / BagRecorder), there is
no sidecar — but the analysis pipeline needs one (it carries the path_recipe used
to rebuild the analytic reference). This writes a minimal, correct sidecar next
to a bag so ``run_eval`` / ``build_dataset`` can consume it.

The recipe MUST match the curve you actually pushed to /reference_path_recipe.

Usage::

    python3 tools/analysis/make_sidecar.py <bag_dir> \
        --type step --R 0.5 --v 1.0 --controller lpv-hinf \
        [--family step] [--leg AtoB] [--run-id b1_test] [--rep 0] \
        [--L1 2.0 --L2 2.0 --theta-deg 90 --direction 1] \
        [--venue basement] [--k-e 3.0]
"""

from __future__ import annotations

import argparse
import math
import os
import sys

_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

import Data_Logger as dl  # build_sidecar / write_sidecar (T7)


def build_recipe(args):
    if args.type == "slalom":
        return {"type": "slalom",
                "params": {"R": args.R, "theta_arc": math.radians(args.theta_deg),
                           "L1": args.L1, "L_mid": args.L_mid,
                           "n_arcs": args.n_arcs, "L_end": args.L_end}}
    if args.type == "uturn":
        return {"type": "uturn",
                "params": {"R": args.R, "L1": args.L1, "L2": args.L2,
                           "direction": args.direction}}
    return {"type": "step",
            "params": {"L1": args.L1, "R": args.R,
                       "theta_arc": math.radians(args.theta_deg),
                       "L2": args.L2, "direction": args.direction}}


def main(argv=None):
    ap = argparse.ArgumentParser(description="Write a sidecar for a manual bag.")
    ap.add_argument("bag_dir")
    ap.add_argument("--type", choices=["step", "slalom", "uturn"], default="step")
    ap.add_argument("--R", type=float, required=True)
    ap.add_argument("--v", type=float, required=True, dest="v")
    ap.add_argument("--controller", default="lpv-hinf")
    ap.add_argument("--family", default=None, help="defaults to --type")
    ap.add_argument("--leg", default="AtoB")
    ap.add_argument("--rep", type=int, default=0)
    ap.add_argument("--run-id", default="manual")
    ap.add_argument("--venue", default="basement")
    ap.add_argument("--k-e", type=float, default=3.0)
    ap.add_argument("--L1", type=float, default=2.0)
    ap.add_argument("--L2", type=float, default=2.0)
    ap.add_argument("--L_mid", type=float, default=1.0)
    ap.add_argument("--L_end", type=float, default=2.0)
    ap.add_argument("--n_arcs", type=int, default=2)
    ap.add_argument("--theta-deg", type=float, default=90.0)
    ap.add_argument("--direction", type=int, default=1)
    args = ap.parse_args(argv)

    if not os.path.isdir(args.bag_dir):
        raise SystemExit(f"[make_sidecar] not a directory: {args.bag_dir}")

    family = args.family or args.type
    recipe = build_recipe(args)
    rtag = f"{args.R:g}".replace(".", "p")
    vtag = f"{args.v:g}".replace(".", "p")
    cell_id = f"R{rtag}-v{vtag}-{family}-{args.controller}"

    sidecar = dl.build_sidecar(
        run_id=args.run_id,
        cell_id=cell_id,
        leg=args.leg,
        cell_params={"controller": args.controller, "v_const": args.v,
                     "radius_m": args.R, "path_family": family, "rep": args.rep},
        path_recipe=recipe,
        venue_id=args.venue,
        start_pin_id=None,
        end_pin_id=None,
        rtk_summary={"fixed_samples": 0, "total_samples": 0, "fixed_pct": None,
                     "note": "manual run; no RTK gating expected at this venue"},
        classification={"pass": None, "note": "manual run (no sequencer verdict)"},
        wallclock={},
        controller_tuning={"controller_type": args.controller, "k_e": args.k_e,
                           "R_min": min(args.R, 0.5)},
        bag_path=os.path.abspath(args.bag_dir),
    )
    out = dl.write_sidecar(args.bag_dir, sidecar)
    print(f"[make_sidecar] wrote {out}")
    print(f"[make_sidecar] recipe: {recipe}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
