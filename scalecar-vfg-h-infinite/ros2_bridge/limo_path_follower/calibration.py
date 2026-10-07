# -*- coding: utf-8 -*-
"""Steering-envelope calibration: figure-8 geometry, matrix lock, sanity check.

Pure Python (no ROS) so the planner, the executor, the venue gate and the
laptop analysis tools share ONE definition. Field design 2026-10-08:

* Every session starts with an open-loop figure-8 at full steering lock
  (``calib_node``). The FIRST successful one (mode ``full``: 0.5 and 1.0 m/s)
  measures R_min and LOCKS the matrix radii once; later sessions run a short
  ``sanity`` figure-8 (1.0 m/s) and pause if the steering changed.
* The matrix is defined as fractions of the usable curvature instead of fixed
  radii (``matrix.radius_m: auto``): usable = margin x 1/R_min(v_lock), and
  R_i = 1 / (fraction_i x usable), rounded. The lock is written once and never
  moved, so all N reps of a cell are the same cell.
* The lock's ``epoch`` tags every scored leg recorded under it; progress counts
  only legs of the current epoch (stock-driver legs and any future re-lock can
  never be credited to the wrong matrix).

Files (all under the bag root, i.e. run artifacts, NUC -> laptop):
  <bag_root>/calibration/matrix_lock.json   the one-time lock
  <bag_root>/calibration/checks.jsonl       one line per calibration attempt
"""
import json
import math
import os
from datetime import datetime, timezone

RECIPE_TYPE = "calib_fig8"

# experiment.yaml `calibration:` defaults (deep-merged).
DEFAULTS = {
    "enabled": True,
    "full_speeds": [0.5, 1.0],
    "sanity_speeds": [1.0],
    "lock_speed": 1.0,          # the speed whose worst side sets the lock
    "steer_cmd_rad": 0.40,      # just under the driver clamp (0.408); the
                                # chassis caps ~0.35 = what "full lock" means
    "R_plan_m": 1.2,            # footprint bound the planner clears
    "turn_deg": 360.0,
    "approach_straight_m": 3.0,
    "approach_loop_r_m": 1.3,   # >= glue track floor; any heading can join it
    "margin": 0.8,              # steady-state <= 80% of full-lock curvature
    "curvature_fractions": [0.4, 0.57, 0.8, 1.0],
    "round_m": 0.05,
    "r_min_bounds_m": [0.4, 1.2],
    "sanity_tol": 0.15,         # |R_today - R_locked| / R_locked
    "recheck_after_h": 4.0,     # a pass younger than this skips the sanity run
    "rtk_crosscheck_tol": 0.15,
    "max_duration_s": 150.0,
    "min_clearance_m": 0.5,
    "radius_slack_m": 0.5,      # calib_node abort radius = 2*R_plan + slack
}


def config(doc):
    """experiment.yaml dict -> calibration config (defaults merged)."""
    cfg = dict(DEFAULTS)
    cfg.update((doc or {}).get("calibration") or {})
    return cfg


def lock_dir(bag_root):
    return os.path.join(bag_root, "calibration")


def lock_path(bag_root):
    return os.path.join(lock_dir(bag_root), "matrix_lock.json")


def checks_path(bag_root):
    return os.path.join(lock_dir(bag_root), "checks.jsonl")


# ----------------------------------------------------------------------
# Geometry
# ----------------------------------------------------------------------

class CalibFig8Path:
    """Planned figure-8 footprint (venue containment) in the analytic-path
    local frame: start at the origin heading +x, left = +y. Left circle
    centred (0, +R) then right circle centred (0, -R), each a full turn and
    tangent to +x at the origin. A real turn of radius r <= R tangent at the
    origin lies INSIDE the planned circle, so a smaller measured R stays
    inside what the planner cleared."""

    def __init__(self, R=1.2):
        self.R = float(R)
        self.total_length = 4.0 * math.pi * self.R

    def position(self, s):
        R = self.R
        s = max(0.0, min(float(s), self.total_length))
        if s <= 2.0 * math.pi * R:
            th = s / R
            return (R * math.sin(th), R - R * math.cos(th))
        th = (s - 2.0 * math.pi * R) / R
        return (R * math.sin(th), -(R - R * math.cos(th)))


def recipe_for(mode, cfg):
    """The calib_fig8 recipe dict carried in active.json."""
    speeds = cfg["full_speeds"] if mode == "full" else cfg["sanity_speeds"]
    return {"type": RECIPE_TYPE,
            "params": {"mode": str(mode),
                       "speeds": [float(v) for v in speeds],
                       "steer_cmd_rad": float(cfg["steer_cmd_rad"]),
                       "R_plan_m": float(cfg["R_plan_m"]),
                       "turn_deg": float(cfg["turn_deg"])}}


def is_calib_recipe(recipe):
    return str((recipe or {}).get("type", "")).lower() == RECIPE_TYPE


def approach_local(cfg, spacing=0.3):
    """Reposition glue INTO the calibration pin, local frame (pin at origin,
    heading +x): one full counter-clockwise loop of radius approach_loop_r_m
    centred left of the line, starting and ending at (-L, 0) heading +x, then
    the straight into the pin. The loop holds every heading, so the
    reposition join finds a forward-drivable segment from any robot pose."""
    L = float(cfg["approach_straight_m"])
    rg = float(cfg["approach_loop_r_m"])
    cx, cy = -L, rg
    n_loop = max(24, int(2.0 * math.pi * rg / spacing))
    pts = []
    for i in range(n_loop + 1):
        a = -math.pi / 2.0 + 2.0 * math.pi * i / n_loop   # bottom, CCW
        pts.append((cx + rg * math.cos(a), cy + rg * math.sin(a)))
    n_st = max(2, int(L / spacing))
    for i in range(1, n_st + 1):
        pts.append((-L + L * i / n_st, 0.0))
    return pts


def footprint_local(cfg, spacing=0.25):
    """Every point the calibration stage may drive (approach + planned
    figure-8), local frame — what the planner clears."""
    path = CalibFig8Path(cfg["R_plan_m"])
    n = max(8, int(path.total_length / spacing))
    fig = [path.position(path.total_length * i / n) for i in range(n + 1)]
    return approach_local(cfg, spacing) + fig


# ----------------------------------------------------------------------
# Lock / sanity
# ----------------------------------------------------------------------

def _segments(result, v):
    return [s for s in (result or {}).get("segments") or []
            if abs(float(s.get("v_cmd", -1)) - float(v)) < 1e-6
            and isinstance(s.get("R_imu_m"), (int, float))
            and math.isfinite(float(s["R_imu_m"]))]


def worst_radius(result, v):
    """(R_bind, {dir: R}) at speed v: the LARGER of left/right (the side that
    turns least sets what both directions can do). None if either side is
    missing."""
    segs = _segments(result, v)
    by_dir = {}
    for s in segs:
        by_dir[str(s.get("dir"))] = float(s["R_imu_m"])
    if "left" not in by_dir or "right" not in by_dir:
        return None, by_dir
    return max(by_dir.values()), by_dir


def radii_from_rmin(r_min, cfg):
    """Matrix radii (descending) for a locked R_min."""
    usable = float(cfg["margin"]) / float(r_min)
    step = float(cfg["round_m"])
    out = []
    for f in sorted(float(x) for x in cfg["curvature_fractions"]):
        R = 1.0 / (f * usable)
        R = round(round(R / step) * step, 6)
        if R not in out:
            out.append(R)
    return sorted(out, reverse=True)


def compute_lock(result, cfg, now_utc=None, source=None):
    """Full calibration result -> (lock dict, None) or (None, reason)."""
    if not (result or {}).get("ok"):
        return None, f"calibration result not ok: {(result or {}).get('reason')}"
    v_lock = float(cfg["lock_speed"])
    r_bind, by_dir = worst_radius(result, v_lock)
    if r_bind is None:
        return None, (f"no left+right measurement at {v_lock} m/s "
                      f"(got {sorted(by_dir)})")
    lo, hi = (float(x) for x in cfg["r_min_bounds_m"])
    if not (lo <= r_bind <= hi):
        return None, (f"measured R_min {r_bind:.2f} m at {v_lock} m/s is outside "
                      f"the sane range [{lo}, {hi}] m — steering not in direct "
                      "mode, or the measurement is bad")
    warnings = []
    rr = by_dir
    if abs(rr["left"] - rr["right"]) / r_bind > 0.25:
        warnings.append(f"left/right asymmetry: left {rr['left']:.2f} m, "
                        f"right {rr['right']:.2f} m")
    for s in _segments(result, v_lock):
        rt = s.get("R_rtk_rear_m")
        if isinstance(rt, (int, float)) and math.isfinite(rt):
            if abs(rt - s["R_imu_m"]) / s["R_imu_m"] > float(cfg["rtk_crosscheck_tol"]):
                warnings.append(f"{s.get('dir')}: RTK radius {rt:.2f} m vs IMU "
                                f"{s['R_imu_m']:.2f} m")
    now = now_utc or datetime.now(timezone.utc)
    stamp = now.strftime("%Y%m%dT%H%M%SZ")
    per_speed = {}
    for v in sorted({float(s.get("v_cmd")) for s in result.get("segments") or []
                     if s.get("v_cmd") is not None}):
        rb, d = worst_radius(result, v)
        per_speed[f"{v:g}"] = {"R_bind_m": rb, "by_dir": d}
    lock = {
        "schema": 1,
        "epoch": f"lock-{stamp}",
        "created_utc": now.isoformat(),
        "lock_speed": v_lock,
        "R_min_m": round(r_bind, 4),
        "delta_max_rad": round(math.atan(0.2 / r_bind), 4),
        "per_speed": per_speed,
        "margin": float(cfg["margin"]),
        "curvature_fractions": [float(x) for x in cfg["curvature_fractions"]],
        "round_m": float(cfg["round_m"]),
        "radius_m": radii_from_rmin(r_bind, cfg),
        "warnings": warnings,
        "source": source or {},
        "result": result,
    }
    return lock, None


def sanity_check(result, lock, cfg):
    """(ok, reason, r_today). A sanity figure-8 must agree with the lock."""
    if not (result or {}).get("ok"):
        return False, f"sanity figure-8 failed: {(result or {}).get('reason')}", None
    v = float(lock.get("lock_speed", cfg["lock_speed"]))
    r_today, by_dir = worst_radius(result, v)
    if r_today is None:
        return False, f"no left+right measurement at {v} m/s", None
    r_lock = float(lock["R_min_m"])
    dev = abs(r_today - r_lock) / r_lock
    if dev > float(cfg["sanity_tol"]):
        return False, (f"steering changed: R_min today {r_today:.2f} m vs locked "
                       f"{r_lock:.2f} m ({dev * 100:.0f}% > "
                       f"{float(cfg['sanity_tol']) * 100:.0f}%) — driver not in "
                       "direct mode, a mechanical change, or a bad surface"), r_today
    return True, (f"R_min today {r_today:.2f} m vs locked {r_lock:.2f} m "
                  f"({dev * 100:.0f}%)"), r_today


# ----------------------------------------------------------------------
# Persistence
# ----------------------------------------------------------------------

def load_lock(path):
    try:
        with open(path, "r", encoding="utf-8") as f:
            d = json.load(f)
    except (FileNotFoundError, ValueError, OSError):
        return None
    if not isinstance(d, dict) or not d.get("radius_m") or not d.get("epoch"):
        return None
    return d


def _atomic_write_json(path, obj):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(obj, f, indent=2, allow_nan=False)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, path)


def write_lock(path, lock):
    """Write the one-time lock. Refuses to overwrite an existing lock: a
    re-lock is a deliberate human act (delete the file), never automatic."""
    if load_lock(path) is not None:
        raise FileExistsError(f"matrix lock already exists: {path}")
    _atomic_write_json(path, lock)


def append_check(path, record):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "a", encoding="utf-8") as f:
        f.write(json.dumps(record, allow_nan=False) + "\n")


def last_pass_utc(path, epoch):
    """UTC datetime of the latest passing check for this epoch, or None."""
    best = None
    try:
        with open(path, "r", encoding="utf-8") as f:
            for line in f:
                try:
                    r = json.loads(line)
                except ValueError:
                    continue
                if r.get("ok") and r.get("epoch") == epoch and r.get("stamp_utc"):
                    try:
                        t = datetime.fromisoformat(r["stamp_utc"])
                    except ValueError:
                        continue
                    if best is None or t > best:
                        best = t
    except (FileNotFoundError, OSError):
        return None
    return best


def mode_due(cfg, lock, last_pass, now_utc=None):
    """'full' | 'sanity' | None — what the next session start must run."""
    if not cfg.get("enabled", True):
        return None
    if lock is None:
        return "full"
    now = now_utc or datetime.now(timezone.utc)
    if last_pass is None:
        return "sanity"
    age_h = (now - last_pass).total_seconds() / 3600.0
    return "sanity" if age_h >= float(cfg["recheck_after_h"]) else None


def resolve_matrix(doc, lock):
    """Return a COPY of the experiment doc with ``matrix.radius_m: auto``
    replaced by the locked radii ([] when there is no lock yet) and
    ``_matrix_epoch`` set (None without a lock). Fixed radius lists pass
    through unchanged (epoch None: legacy behaviour)."""
    import copy
    out = copy.deepcopy(doc or {})
    m = out.setdefault("matrix", {})
    if str(m.get("radius_m", "")).strip().lower() == "auto":
        if lock is not None:
            m["radius_m"] = [float(r) for r in lock["radius_m"]]
            out["_matrix_epoch"] = lock["epoch"]
        else:
            m["radius_m"] = []
            out["_matrix_epoch"] = None
        out["_radius_auto"] = True
    else:
        out["_matrix_epoch"] = None
        out["_radius_auto"] = False
    return out
