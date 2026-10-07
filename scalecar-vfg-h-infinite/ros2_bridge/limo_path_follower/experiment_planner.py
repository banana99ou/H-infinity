# -*- coding: utf-8 -*-
"""experiment_planner — auto-layout of experiment curves + glue inside a venue.

Pure geometry, no ROS, no I/O: trivially unit-testable off the robot (same
contract as venue_geom, which supplies all clearance math so a plan that is
green here is green at the venue_loader / run_executor containment gates).

Input: a venue dict (corners_wgs84 + safety_margin_m + exclusions, the same
schema the WebUI sends on /venue/load), the experiment.yaml document (matrix +
optional ``plan:`` block), and the per-cell pass counts the run_executor
already rebuilds from the bag manifest. Output: an ordered list of STAGES.
Each stage is a set of curve geometries (one per remaining (family, R) cell
group, as many as physically fit the venue at once) plus cyclic reposition
glue between them — exactly the lbExps/lbRepos shape the WebUI leg-batch
editor edits, so the operator can inspect and tweak the proposal before
Send + Start.

Placement is a deterministic coarse-to-fine grid search (position x heading
x direction); glue is a Bezier whose control mids force a straight,
heading-aligned tail (the reposition tracker achieves arrival heading by
geometry, not in-place rotation — see reposition_node). A geometry that fits
nowhere is NOT dropped (silent loss is how a 320-run matrix quietly becomes
fewer): the planner seats a BEST-EFFORT opposed pair and flags the stage
``needs_fix=True`` with a ``fix_reason`` so the operator drags it into spec in
the WebUI — the NUC venue_loader re-validates on Send, so an out-of-spec leg
never drives. ``ok`` is False while any stage needs_fix; the matrix is locked
spec, so the planner still never silently shrinks an R. (Truly unbuildable
recipes — bad family / vfg import — still land in ``unfittable``.)
"""
import math

import numpy as np

try:
    from limo_path_follower import venue_geom
except Exception:  # pragma: no cover - in-source / odd layout fallback
    import venue_geom
try:
    from limo_path_follower import calibration as calib_mod
except Exception:  # pragma: no cover - in-source / odd layout fallback
    import calibration as calib_mod


# Defaults for the optional ``plan:`` block in experiment.yaml. Shape params
# (L1/L2/L_mid/...) are GEOMETRY choices, not matrix axes — they live in the
# yaml so they are version-controlled and operator-editable, but the cell key
# only sweeps (family, R).
PLAN_DEFAULTS = {
    # >= 2: each stage is ONE geometry placed as an A/B opposed pair
    # (directional-bias removal, field decision 2026-06-11); <= 1: legacy
    # single-curve stages (the stage-advance shakedown forces this).
    "max_geometries_per_stage": 2,
    "geometry_order": "matrix",      # matrix | finish-nearest
    "grid_m": 1.0,                   # coarse placement grid pitch
    "heading_step_deg": 30.0,        # coarse heading pitch
    "refine_grid_m": 0.25,           # fine pitch around the coarse winner
    "refine_heading_deg": 7.5,
    "fit_buffer_m": 0.05,            # extra clearance over the loader's gate
    "step": {"L1": 1.0, "L2": 1.0, "theta_deg": 90.0},
    "slalom": {"n_arcs": 2, "theta_deg": 90.0, "L_mid": 0.5,
               "L1": 1.0, "L_end": 1.0},
    # wiggle_sine / wiggle_chirp / wiggle_square (wiggle_path.py): ~7 m each
    # whatever R, so an A/B pair seats in a ~14 m venue. wavelength_end only
    # shapes chirp.
    "wiggle": {"wavelength": 3.0, "wavelength_end": 1.5, "n_periods": 2,
               "L1": 0.5, "L_end": 0.5},
    "glue": {"v_const": 0.2, "pos_tol_m": 0.15,
             "tail_m": 2.2,          # straight, heading-aligned approach tail
             "lead_m": 1.0,          # straight exit along the curve's end heading
             # soft: preferred glue turn radius. Glues tighter than this pay a
             # smoothness penalty (was 0.5 m, which never bit: every glue then
             # sat at the trackable floor; 2026-10-02 -> 2.0 m).
             "min_radius_m": 2.0,
             # HARD floor: the chassis cannot steer tighter than ~0.37 m, so a
             # drawn glue below this would not be tracked (pure pursuit clamps
             # and leaves the checked corridor). Reject, don't warn.
             "hard_radius_m": 0.37,
             # TRACKABLE floor (B+ smoothness gate, field 2026-06-11): glue is
             # driven by reposition's pure pursuit with a 0.6 m look-ahead — a
             # legal-but-tight Dubins loop (R 0.45) breaks the steering cone
             # mid-arc and aborts. Glue must be drivable WITH MARGIN, not
             # merely chassis-possible. Raised 0.7 -> 1.0 (field 2026-06-16):
             # a w=1.8 racetrack turnaround (R 0.9) still UNDERSHOT — pure
             # pursuit cut the corner, landed short at the end pin facing
             # ~opposite, and the run failed. 1.0 m floor forces the closed-form
             # B candidates to w>=2.0 (R>=1.0), clearing the observed undershoot.
             "track_radius_m": 1.0,
             # Tiered floor (2026-10-02): a geometry or transit that cannot be
             # seated at track_radius_m retries at this floor (the old 0.85 m,
             # ~the measured steering limit) and is flagged in the notes, so a
             # tight venue never regresses to a best-effort stage.
             "track_radius_fallback_m": 0.85,
             # Straight-tail arrival condition: the final tail_check_m of every
             # glue must lie within tail_align_deg of the start-pin heading —
             # reposition converges heading through the tail, so this is what
             # turns "arrived (position + heading)" into a planned property.
             "tail_align_deg": 10.0,
             "tail_check_m": 0.8},
}


# ----------------------------------------------------------------------
# Small geometry helpers (EN frame, mirroring venue_geom conventions)
# ----------------------------------------------------------------------

def _en_to_latlon(e, n, lat0, lon0):
    mlat = 111320.0
    mlon = 111320.0 * math.cos(math.radians(lat0))
    return (lat0 + n / mlat, lon0 + e / mlon)


def _bearing_vec(h_deg):
    """Compass bearing (deg E-of-N) -> EN unit vector."""
    h = math.radians(h_deg)
    return (math.sin(h), math.cos(h))


def _clear(pt, poly, excl):
    cl = venue_geom.clearance(pt, poly)
    for (ce, cn, r) in excl:
        cl = min(cl, math.hypot(pt[0] - ce, pt[1] - cn) - r)
    return cl


def _clear_many(pts, poly, excl):
    """_clear for many points at once (ndarray). Same formulas as
    venue_geom.clearance / pt_in_poly / dist_to_seg, vectorized: the glue
    search evaluates ~10^7 points per plan, and the per-point Python loop was
    ~40% of planning time (profiled 2026-10-02)."""
    P = np.asarray(pts, dtype=float).reshape(-1, 2)
    px, py = P[:, 0], P[:, 1]
    d = np.full(len(P), np.inf)
    inside = np.zeros(len(P), dtype=bool)
    n = len(poly)
    for i in range(n):
        ax, ay = poly[i]
        bx, by = poly[(i + 1) % n]
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy
        if L2 == 0.0:
            dd = np.hypot(px - ax, py - ay)
        else:
            t = np.clip(((px - ax) * dx + (py - ay) * dy) / L2, 0.0, 1.0)
            dd = np.hypot(px - (ax + t * dx), py - (ay + t * dy))
        d = np.minimum(d, dd)
        crosses = (ay > py) != (by > py)
        with np.errstate(divide="ignore", invalid="ignore"):
            xint = (bx - ax) * (py - ay) / (by - ay) + ax
        inside ^= crosses & (px < xint)
    cl = np.where(inside, d, -d)
    for (ce, cn, r) in excl:
        cl = np.minimum(cl, np.hypot(px - ce, py - cn) - r)
    return cl


def _plan_cfg(doc):
    """PLAN_DEFAULTS deep-merged under the yaml's optional plan: block."""
    cfg = {k: (dict(v) if isinstance(v, dict) else v)
           for k, v in PLAN_DEFAULTS.items()}
    user = (doc or {}).get("plan") or {}
    for k, v in user.items():
        if isinstance(v, dict) and isinstance(cfg.get(k), dict):
            cfg[k].update(v)
        else:
            cfg[k] = v
    return cfg


def _cell_key(fam, R, c, v):
    """Mirror of tools/analysis manifest.cell_key (kept in sync by
    test_experiment_planner; run_executor passes counts keyed by the real
    manifest.cell_key, so the formats must match)."""
    def _num(x):
        try:
            return round(float(x), 6)
        except (TypeError, ValueError):
            return None
    return (str(c), _num(v), str(fam), _num(R))


def _cell_key_dict(cell_params):
    """manifest.cell_key call shape (dict in, tuple out) — the default for
    remaining_geometries so the real manifest.cell_key drops in unchanged."""
    return _cell_key(cell_params.get("path_family"),
                     cell_params.get("radius_m"),
                     cell_params.get("controller"),
                     cell_params.get("v_const"))


# ----------------------------------------------------------------------
# Remaining work (matrix x manifest counts)
# ----------------------------------------------------------------------

def remaining_geometries(doc, counts, key_fn=None):
    """Ordered [(family, R, remaining_runs), ...] with remaining > 0.

    ``counts`` maps cell_key -> passing runs (the run_executor's
    _completed_counts). ``key_fn`` has manifest.cell_key's call shape
    (cell_params dict -> tuple) and defaults to a local mirror of it; pass
    the real manifest.cell_key when available.
    """
    key_fn = key_fn or _cell_key_dict
    matrix = (doc or {}).get("matrix") or {}
    fams = [str(f) for f in (matrix.get("path_family") or [])]
    # 'auto' (calibration lock) must be resolved by the caller
    # (manifest.load_experiment); an unresolved 'auto' means no radii yet.
    raw = matrix.get("radius_m") or []
    radii = [] if isinstance(raw, str) else [float(r) for r in raw]
    by_fam = matrix.get("radius_m_by_family") or {}
    ctrls = [str(c) for c in (matrix.get("controller") or [])]
    speeds = [float(v) for v in (matrix.get("v_const") or [])]
    n = int((doc or {}).get("repetitions") or 0)
    out = []
    for fam in fams:
        for R in [float(r) for r in by_fam.get(fam, radii)]:
            rem = 0
            for c in ctrls:
                for v in speeds:
                    k = key_fn({"controller": c, "v_const": v,
                                "path_family": fam, "radius_m": R})
                    rem += max(0, n - int(counts.get(k, 0)))
            if rem > 0:
                out.append((fam, R, rem))
    cfg = _plan_cfg(doc)
    if cfg.get("geometry_order") == "finish-nearest":
        # Stable sort: ties keep matrix order (family, then its radius list).
        out.sort(key=lambda t: t[2])
    # plan.priority: [[family, R], ...] still-remaining cells go first, in the
    # listed order (stages run in plan order, so these run next). Used to
    # bring a representative pilot forward without touching the matrix.
    pri = [(str(f), round(float(r), 6)) for (f, r) in (cfg.get("priority") or [])]
    rank = {k: i for i, k in enumerate(pri)}
    out.sort(key=lambda t: rank.get((t[0], round(t[1], 6)), len(pri)))
    return out


def _recipe_for(fam, R, cfg, direction=1):
    if fam == "step":
        s = cfg["step"]
        return {"type": "step",
                "params": {"L1": float(s["L1"]), "R": float(R),
                           "theta_arc": math.radians(float(s["theta_deg"])),
                           "L2": float(s["L2"]), "direction": int(direction)}}
    if fam == "slalom":
        s = cfg["slalom"]
        # SlalomPath has no direction param — the first arc always turns left.
        return {"type": "slalom",
                "params": {"R": float(R),
                           "theta_arc": math.radians(float(s["theta_deg"])),
                           "L1": float(s["L1"]), "L_mid": float(s["L_mid"]),
                           "n_arcs": int(s["n_arcs"]),
                           "L_end": float(s["L_end"])}}
    if fam in ("wiggle_sine", "wiggle_chirp", "wiggle_square"):
        s = cfg["wiggle"]
        return {"type": fam,
                "params": {"R": float(R),
                           "wavelength": float(s["wavelength"]),
                           "wavelength_end": float(s["wavelength_end"]),
                           "n_periods": int(s["n_periods"]),
                           "L1": float(s["L1"]), "L_end": float(s["L_end"])}}
    return None


# ----------------------------------------------------------------------
# Curve sampling (local frame, then placed per candidate)
# ----------------------------------------------------------------------

def _local_samples(recipe, spacing_m):
    """[(x, y), ...] in the curve's local frame (+x fwd, +y left) plus the
    local end heading (rad CCW from +x). None if not buildable."""
    path = venue_geom.recipe_path(recipe)
    if path is None:
        return None, None
    total = float(path.total_length)
    n = max(2, int(total / max(0.01, spacing_m)))
    pts = []
    for i in range(n + 1):
        p = path.position(total * i / n)
        pts.append((float(p[0]), float(p[1])))
    try:
        end_yaw = float(path.heading(total))
    except Exception:
        (x0, y0), (x1, y1) = pts[-2], pts[-1]
        end_yaw = math.atan2(y1 - y0, x1 - x0)
    return pts, end_yaw


def _place(pts_local, x, y, h_deg):
    """Place local samples at EN (x, y) with +x at compass bearing h_deg."""
    h = math.radians(h_deg)
    fE, fN = math.sin(h), math.cos(h)
    lE, lN = -math.cos(h), math.sin(h)
    return [(x + px * fE + py * lE, y + px * fN + py * lN)
            for (px, py) in pts_local]


def _min_clear_placed(pts_local, x, y, h_deg, poly, excl, stop_below):
    """Min clearance of the placed curve. (stop_below is kept for callers;
    every caller only compares the result against it, and the vectorized
    full minimum answers that comparison identically.)"""
    h = math.radians(h_deg)
    fE, fN = math.sin(h), math.cos(h)
    lE, lN = -math.cos(h), math.sin(h)
    L = np.asarray(pts_local, dtype=float)
    E = x + L[:, 0] * fE + L[:, 1] * lE
    N = y + L[:, 0] * fN + L[:, 1] * lN
    return float(_clear_many(np.column_stack((E, N)), poly, excl).min())


# ----------------------------------------------------------------------
# Glue (Bezier with straight exit lead + straight aligned tail)
# ----------------------------------------------------------------------

def _bezier(ctrl, t):
    pts = list(ctrl)
    while len(pts) > 1:
        pts = [(a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)
               for a, b in zip(pts[:-1], pts[1:])]
    return pts[0]


def _bezier_samples(ctrl, spacing_m=0.3):
    length = sum(math.hypot(b[0] - a[0], b[1] - a[1])
                 for a, b in zip(ctrl[:-1], ctrl[1:]))
    n = max(12, min(300, int(length / max(0.05, spacing_m) + 1e-6)))
    # Bernstein form, all n+1 parameters in one matrix product (the per-point
    # de Casteljau loop was ~50% of planning time, profiled 2026-10-02; equal
    # to _bezier within 1e-12 m).
    m = len(ctrl) - 1
    t = np.arange(n + 1) / n
    j = np.arange(m + 1)
    coef = np.array([math.comb(m, k) for k in j], dtype=float)
    basis = coef * t[:, None] ** j * (1.0 - t[:, None]) ** (m - j)
    return [tuple(q) for q in (basis @ np.asarray(ctrl, dtype=float)).tolist()]


def _max_curvature(pts):
    """Max discrete (Menger) curvature over the sampled polyline."""
    if len(pts) < 3:
        return 0.0
    P = np.asarray(pts, dtype=float)
    a, b, c = P[:-2], P[1:-1], P[2:]
    ab = np.hypot(b[:, 0] - a[:, 0], b[:, 1] - a[:, 1])
    bc = np.hypot(c[:, 0] - b[:, 0], c[:, 1] - b[:, 1])
    ca = np.hypot(c[:, 0] - a[:, 0], c[:, 1] - a[:, 1])
    den = ab * bc * ca
    ok = den >= 1e-9
    if not ok.any():
        return 0.0
    area2 = np.abs((b[:, 0] - a[:, 0]) * (c[:, 1] - a[:, 1])
                   - (b[:, 1] - a[:, 1]) * (c[:, 0] - a[:, 0]))
    return float(max(0.0, (2.0 * area2[ok] / den[ok]).max()))


def _ang_diff_deg(a, b):
    return abs((a - b + 180.0) % 360.0 - 180.0)


def _self_intersects(pts):
    """True if the sampled polyline crosses itself (a teardrop / racetrack
    lap). A self-crossing reposition curve is NOT physically trackable even
    when its min radius is legal: near the crossing, pure pursuit's lookahead
    chord cuts across the loop and can latch the wrong branch, demanding a
    sub-R_min turn (field 2026-06-12: the stage-1 turnaround aborted exactly
    this way). Adjacent segments share an endpoint and are skipped; the
    first/last pair is skipped too (they may meet a shared pin by design)."""
    P = np.asarray(pts, dtype=float)
    n = len(P)
    if n < 4:
        return False
    A, B = P[:-1], P[1:]                         # segment i = A[i] -> B[i]
    i, j = np.triu_indices(n - 1, k=2)           # j >= i + 2: non-adjacent
    keep = ~((i == 0) & (j == n - 2))
    i, j = i[keep], j[keep]
    ax, ay, bx, by = A[i, 0], A[i, 1], B[i, 0], B[i, 1]
    cx, cy, dx, dy = A[j, 0], A[j, 1], B[j, 0], B[j, 1]
    d1 = (dx - cx) * (ay - cy) - (dy - cy) * (ax - cx)
    d2 = (dx - cx) * (by - cy) - (dy - cy) * (bx - cx)
    d3 = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)
    d4 = (bx - ax) * (dy - ay) - (by - ay) * (dx - ax)
    # Proper crossing only: each pair of cross products clearly on opposite
    # sides. Collinear pieces of one straight have d ~ 0 +- 1e-17, and a bare
    # sign test read that rounding noise as a crossing (2026-10-02: valid
    # straight glues were rejected at random, pushing the planner onto loops).
    eps = 1e-9

    def opposite(u, v):
        return ((u > eps) & (v < -eps)) | ((u < -eps) & (v > eps))
    return bool(np.any(opposite(d1, d2) & opposite(d3, d4)))


def check_glue_tracking(pts, start_b, g):
    """B+ smoothness gate over one sampled glue polyline (endpoints included).

    Treats the glue as the robot will drive it: (1) every point must be
    reachable at the TRACKABLE radius floor (track_radius_m — pure pursuit
    with margin, not the bare chassis limit), and (2) the final tail_check_m
    must be straight along the start-pin heading (start_b, compass deg E-of-N)
    within tail_align_deg, because reposition achieves arrival heading by
    tracking that tail. Exp curves are exempt (driven by the follower, the
    tight radii ARE the experiment); junction continuity to them holds because
    both generators start the glue along the previous curve's exit heading.

    Returns (ok, reason). Exposed for tests and for ad-hoc plan audits.
    """
    track_r = max(float(g.get("track_radius_m", 0.7)),
                  float(g.get("hard_radius_m", 0.37)))
    kmax = _max_curvature(pts)
    if kmax > 1.0 / track_r + 1e-6:
        return False, (f"min turn radius {1.0 / max(kmax, 1e-9):.2f}m is "
                       f"tighter than the trackable floor {track_r:.2f}m")
    tail_deg = float(g.get("tail_align_deg", 10.0))
    tail_m = float(g.get("tail_check_m", 0.8))
    acc = 0.0
    for a, b in zip(reversed(pts[:-1]), reversed(pts[1:])):
        de, dn = b[0] - a[0], b[1] - a[1]
        seg = math.hypot(de, dn)
        if seg < 1e-9:
            continue
        brg = math.degrees(math.atan2(de, dn)) % 360.0
        off = _ang_diff_deg(brg, start_b)
        if off > tail_deg:
            return False, (f"approach tail bends {off:.0f} deg off the "
                           f"start heading {acc:.1f}m before arrival "
                           f"(needs <= {tail_deg:.0f} deg for the last "
                           f"{tail_m:.1f}m)")
        acc += seg
        if acc >= tail_m:
            break
    if _self_intersects(pts):
        return False, ("glue self-intersects (teardrop/lap loop) — pure "
                       "pursuit can latch the wrong branch at the crossing")
    return True, None


def _dubins_paths(a_pose, b_pose, R):
    """All valid Dubins words from pose A to pose B at turn radius R.

    Poses are (x, y, yaw) in a metric frame, yaw CCW radians. Returns
    [(length, [(x, y), ...]), ...] — every candidate is END-VERIFIED by
    integration (position within 5 cm, heading within 2 deg of B), so a
    formula slip can only DROP a word, never emit a wrong path.
    """
    ax, ay, ath = a_pose
    bx, by, bth = b_pose
    dx, dy = bx - ax, by - ay
    D = math.hypot(dx, dy)
    d = D / R
    theta = math.atan2(dy, dx)
    alpha = (ath - theta) % (2 * math.pi)
    beta = (bth - theta) % (2 * math.pi)
    sa, ca = math.sin(alpha), math.cos(alpha)
    sb, cb = math.sin(beta), math.cos(beta)
    c_ab = math.cos(alpha - beta)
    two_pi = 2 * math.pi
    words = []

    def _m(x):
        # An arc of -1e-16 rad must stay ~0, not wrap to a full 2*pi lap
        # (rounding noise was turning "no arc" into a 360-deg loop).
        r = x % two_pi
        return 0.0 if r > two_pi - 1e-9 else r

    p_sq = 2 + d * d - 2 * c_ab + 2 * d * (sa - sb)
    if p_sq >= -1e-9:   # tangency (p = 0) must not hinge on rounding
        tmp = math.atan2(cb - ca, d + sa - sb)
        words.append((_m(tmp - alpha), math.sqrt(max(p_sq, 0.0)), _m(beta - tmp), "LSL"))
    p_sq = 2 + d * d - 2 * c_ab + 2 * d * (sb - sa)
    if p_sq >= -1e-9:   # tangency (p = 0) must not hinge on rounding
        tmp = math.atan2(ca - cb, d - sa + sb)
        words.append((_m(alpha - tmp), math.sqrt(max(p_sq, 0.0)), _m(tmp - beta), "RSR"))
    p_sq = -2 + d * d + 2 * c_ab + 2 * d * (sa + sb)
    if p_sq >= -1e-9:   # tangency (p = 0) must not hinge on rounding
        p = math.sqrt(max(p_sq, 0.0))
        tmp = math.atan2(-ca - cb, d + sa + sb) - math.atan2(-2.0, p)
        words.append((_m(tmp - alpha), p, _m(tmp - beta), "LSR"))
    p_sq = -2 + d * d + 2 * c_ab - 2 * d * (sa + sb)
    if p_sq >= -1e-9:   # tangency (p = 0) must not hinge on rounding
        p = math.sqrt(max(p_sq, 0.0))
        tmp = math.atan2(ca + cb, d - sa - sb) - math.atan2(2.0, p)
        words.append((_m(alpha - tmp), p, _m(beta - tmp), "RSL"))
    tmp = (6.0 - d * d + 2 * c_ab + 2 * d * (sa - sb)) / 8.0
    if abs(tmp) <= 1.0 + 1e-9:
        p = _m(two_pi - math.acos(max(-1.0, min(1.0, tmp))))
        t = _m(alpha - math.atan2(ca - cb, d - sa + sb) + p / 2.0)
        words.append((t, p, _m(alpha - beta - t + p), "RLR"))
    tmp = (6.0 - d * d + 2 * c_ab + 2 * d * (sb - sa)) / 8.0
    if abs(tmp) <= 1.0 + 1e-9:
        p = _m(two_pi - math.acos(max(-1.0, min(1.0, tmp))))
        t = _m(-alpha + math.atan2(-ca + cb, d + sa - sb) + p / 2.0)
        words.append((t, p, _m(beta - alpha - t + p), "LRL"))

    out = []
    for (t, p, q, mode) in words:
        segs = list(zip(mode, (t * R, p * R, q * R)))
        pts = [(ax, ay)]
        x, y, th = ax, ay, ath
        ok = True
        for (m, length) in segs:
            if length < -1e-9 or length > 4 * two_pi * R + D:
                ok = False
                break
            # ceil with slack: an exact 1.0 m straight must not sample as
            # 3 or 4 pieces depending on the last bit
            n = max(1, math.ceil(length / 0.25 - 1e-6))
            if m == "S":
                for i in range(1, n + 1):
                    pts.append((x + length * i / n * math.cos(th),
                                y + length * i / n * math.sin(th)))
                x, y = pts[-1]
            else:
                sgn = 1.0 if m == "L" else -1.0
                cx2 = x - sgn * R * math.sin(th)
                cy2 = y + sgn * R * math.cos(th)
                for i in range(1, n + 1):
                    th_i = th + sgn * (length * i / n) / R
                    pts.append((cx2 + sgn * R * math.sin(th_i),
                                cy2 - sgn * R * math.cos(th_i)))
                th = th + sgn * length / R
                x, y = pts[-1]
        if not ok:
            continue
        derr = math.hypot(x - bx, y - by)
        herr = abs((th - bth + math.pi) % (2 * math.pi) - math.pi)
        if derr < 0.05 and herr < math.radians(2.0):
            out.append(((t + p + q) * R, pts))
    out.sort(key=lambda w: w[0])
    return out


def _glue_ctrl(end, exit_bearing_deg, start, start_bearing_deg, cfg,
               lead_m, tail_m, dodges=()):
    """Control polygon end -> start. The final two mids sit on the approach
    heading line so the Bezier tail is straight and aligned (arrival heading
    is achieved by geometry per reposition_node); the first mid extends the
    previous curve's exit so the join is forward-drivable."""
    ex, ey = _bearing_vec(exit_bearing_deg)
    sx, sy = _bearing_vec(start_bearing_deg)
    ctrl = [end, (end[0] + ex * lead_m, end[1] + ey * lead_m)]
    ctrl += list(dodges)
    ctrl += [(start[0] - sx * tail_m, start[1] - sy * tail_m),
             (start[0] - sx * tail_m * 0.45, start[1] - sy * tail_m * 0.45),
             start]
    return ctrl


def _tail_straight(pts, start_b, align_deg):
    """Length of the final stretch of a sampled glue whose heading stays within
    align_deg of the start-pin heading: the straight the tracker gets to settle
    its heading on before the pin."""
    acc = 0.0
    for a, b in zip(reversed(pts[:-1]), reversed(pts[1:])):
        de, dn = b[0] - a[0], b[1] - a[1]
        seg = math.hypot(de, dn)
        if seg < 1e-9:
            continue
        if _ang_diff_deg(math.degrees(math.atan2(de, dn)) % 360.0,
                         start_b) > align_deg:
            break
        acc += seg
    return acc


def _total_turn(pts):
    """Total absolute heading change along a sampled polyline [rad]."""
    P = np.asarray(pts, dtype=float)
    d = np.diff(P, axis=0)
    d = d[np.hypot(d[:, 0], d[:, 1]) > 1e-9]
    if len(d) < 2:
        return 0.0
    h = np.arctan2(d[:, 1], d[:, 0])
    return float(np.abs((np.diff(h) + np.pi) % (2 * np.pi) - np.pi).sum())


def _beats(a, b, tol):
    """Lexicographic 'a is better than b' (higher is better), where a component
    only decides when it differs by more than its tolerance. Used instead of
    rounding: rounding flips on its .x5 boundaries — where quarter-metre-sampled
    lengths land exactly — so a nanometre of noise could change the plan."""
    for x, y, t in zip(a, b, tol):
        if x > y + t:
            return True
        if x < y - t:
            return False
    return False


def _tol_sorted(items, key, tol):
    """Stable sort, best first, under _beats (near-ties keep input order)."""
    import functools

    def cmp(i, j):
        a, b = key(i), key(j)
        return -1 if _beats(a, b, tol) else (1 if _beats(b, a, tol) else 0)
    return sorted(items, key=functools.cmp_to_key(cmp))


def _glue_quality(pts, start_b, g):
    """(turn radius, settled straight tail) of a sampled glue, each capped
    where more stops helping. Smoothness objective shared by the glue choice
    and the pair-layout ranking."""
    k = _max_curvature(pts)
    radius = min(1.0 / k if k > 1e-9 else math.inf,
                 float(g.get("quality_radius_cap_m", 2.5)))
    tail = min(_tail_straight(pts, start_b,
                              float(g.get("settle_align_deg", 5.0))),
               float(g.get("quality_tail_cap_m", 2.5)))
    return radius, tail


def _plan_glue(end, exit_b, start, start_b, poly, excl, req, cfg,
               objective="smooth"):
    """Plan one glue gap: the SMOOTHEST valid glue, not the most compact.

    Candidates are the operator-editable Bezier variants (lead/tail/dodge
    shapes) and exact bounded-curvature Dubins paths (gentle radii first, with
    a long straight approach before the pin). Every candidate must clear the
    venue, stay at or above the trackable radius and pass the tail gate; the
    survivors are ranked by one penalty that rewards a gentle turn (radius up
    to min_radius_m), a long settled straight tail before the pin and a short
    path. Field 2026-10-02: ranking by compactness alone put every glue right
    at the trackable floor with a 0.8-1.0 m tail, and the robot arrived a
    median 7 deg (p90 12 deg, RTK-measured) off the pin heading.

    poly=None plans in free space (no clearance terms): the result is then
    rotation-equivariant, which the pair packer uses to plan once per layout.
    objective="compact" ranks by shortness instead — the packer sizes a pair's
    block with the most compact valid glue (can ANY glue fit?) and leaves the
    smooth choice to the real-venue plan, where clearance bounds its size.

    Returns (glue_en, note) or (None, reason). glue_en is
    {"kind": "mids"|"waypoints", "pts": [(E, N), ...]} — mids exclude the
    snapped endpoints (WebUI lbRepos schema); waypoints include them."""
    g = cfg["glue"]
    free = poly is None
    track_r = max(float(g.get("track_radius_m", 0.7)),
                  float(g["hard_radius_m"]))
    track_k = 1.0 / track_r            # B+ gate: trackable, not just possible
    soft_k = 1.0 / float(g["min_radius_m"])
    chord_mid = ((end[0] + start[0]) / 2.0, (end[1] + start[1]) / 2.0)
    dx, dy = start[0] - end[0], start[1] - end[1]
    norm = math.hypot(dx, dy) or 1.0
    perp = (-dy / norm, dx / norm)
    # Variant sets: chord-perpendicular single dodges shape ordinary hooks;
    # paired lateral dodges (off the exit / approach headings, same side)
    # shape racetrack laps — needed when exit and entry headings coincide
    # (e.g. a slalom looping onto itself: ~360 deg of total turn).
    ex, ey = _bearing_vec(exit_b)
    sx, sy = _bearing_vec(start_b)
    rex, rey = ey, -ex          # right-perp of the exit heading (EN frame)
    rsx, rsy = sy, -sx          # right-perp of the approach heading
    single = [()] + [
        ((chord_mid[0] + perp[0] * off, chord_mid[1] + perp[1] * off),)
        for off in (0.6, -0.6, 1.2, -1.2, 1.8, -1.8, 2.4, -2.4, 3.2, -3.2)]
    pairs = []
    for sgn in (1.0, -1.0):
        for w in (1.5, 2.5, 3.5):
            pairs.append((
                (end[0] + ex * 1.0 + sgn * w * rex,
                 end[1] + ey * 1.0 + sgn * w * rey),
                (start[0] - sx * 1.0 + sgn * w * rsx,
                 start[1] - sy * 1.0 + sgn * w * rsy)))
    # Lap variants: control points spaced along a circle tangent to the exit
    # heading. A Bezier cannot shape a full 360-deg loop from 1-2 dodges (the
    # hull pulls it into a cusp), but ~60-deg-spaced points on a circle keep
    # the inscribed curve close to a constant-radius lap.
    laps = []
    for sgn in (1.0, -1.0):
        for r in (1.0, 1.4, 1.9):
            cx = end[0] + ex * 0.5 + sgn * r * rex
            cy = end[1] + ey * 0.5 + sgn * r * rey
            a0 = math.atan2(end[1] + ey * 0.5 - cy, end[0] + ex * 0.5 - cx)
            # Sweep the long way around (direction -sgn in math angle terms:
            # a right-side (sgn=+1) lap turns clockwise = decreasing angle).
            arc = []
            for k in range(1, 6):
                a = a0 - sgn * k * (2.0 * math.pi / 6.0)
                arc.append((cx + r * math.cos(a), cy + r * math.sin(a)))
            laps.append(tuple(arc))

    def _score(pts, spread):
        """Penalty of a cleared, trackable candidate (lower is better)."""
        length = sum(math.hypot(b[0] - a[0], b[1] - a[1])
                     for a, b in zip(pts[:-1], pts[1:]))
        if objective == "compact":
            return 0.3 * spread + length
        radius, tail = _glue_quality(pts, start_b, g)
        # Turning beyond what the gap needs (the exit -> pin heading change) is
        # a loop or detour: time, drift, another chance to miss the pin. The
        # tail/radius rewards alone preferred a 360-deg lap ending on a long
        # straight over a short direct turn (plots 2026-10-02).
        excess = max(0.0, _total_turn(pts)
                     - math.radians(_ang_diff_deg(exit_b, start_b)) - 0.35)
        mn = 0.0 if free else min(float(_clear_many(pts, poly, excl).min()), 2.0)
        return (0.3 * spread + max(0.0, 1.0 / radius - soft_k) * 3.0
                - 0.6 * tail + 0.1 * length - 0.2 * mn + 1.5 * excess)

    best = None     # (penalty, glue_en, kind, radius, length)
    tried = 0
    for lead in (float(g["lead_m"]), 0.5, 1.8):
        for tail in (float(g["tail_m"]), 1.2, 3.0):
            for dodges in single + pairs + laps:
                tried += 1
                ctrl = _glue_ctrl(end, exit_b, start, start_b, cfg,
                                  lead, tail, dodges)
                pts = _bezier_samples(ctrl)
                if not free and float(_clear_many(pts, poly, excl).min()) < req:
                    continue
                if _max_curvature(pts) > track_k:
                    continue        # not trackable with margin — reject (B+)
                ok_track, _why = check_glue_tracking(pts, start_b, g)
                if not ok_track:
                    continue        # bent tail / gate failure — reject (B+)
                spread = sum(math.hypot(d[0] - chord_mid[0],
                                        d[1] - chord_mid[1]) for d in dodges)
                pen = _score(pts, spread)
                if best is None or pen < best[0] - 1e-6:   # ties: first wins
                    best = (pen, {"kind": "mids", "pts": ctrl[1:-1]}, "bezier")
                    # NOTE: no good-enough early exit here — measured
                    # 2026-06-11: returning the first acceptable variant
                    # (instead of the penalty-best) degrades later
                    # placements enough that full-matrix planning got
                    # SLOWER overall (77-80 s vs 58 s). The polish pays
                    # for itself.

    # Dubins candidates: exact start/end poses, curvature bounded by R, each
    # ending in an explicit straight tail before the pin (the tracker settles
    # heading on it). Gentle radii and long tails are offered first-class, not
    # only as a last resort — a Bezier cannot shape a tight U-turn, so for A/B
    # turnarounds this is usually the smoothest drivable glue.
    a_yaw = math.radians(90.0 - exit_b)
    b_yaw = math.radians(90.0 - start_b)
    # B+ gate: never sweep below the TRACKABLE radius — a chassis-legal 0.45 m
    # Dubins loop breaks reposition's pure-pursuit cone mid-arc (field
    # 2026-06-11: "do a 180 at the E1 start").
    radii = sorted({r for r in (2.0, 1.5, 1.2, 1.0, 0.8, track_r)
                    if r >= track_r}, reverse=True)
    for R in radii:
        for tail in (2.0, 1.5, max(1.0, float(g.get("tail_check_m", 0.8)) + 0.2)):
            bx, by = start[0] - sx * tail, start[1] - sy * tail
            for (_length, pts) in _dubins_paths((end[0], end[1], a_yaw),
                                                (bx, by, b_yaw), R)[:3]:
                tried += 1
                full = pts + [(bx + sx * tail * i / 4.0,
                               by + sy * tail * i / 4.0) for i in range(1, 5)]
                if not free and float(_clear_many(full, poly, excl).min()) < req:
                    continue
                ok_track, _why = check_glue_tracking(full, start_b, g)
                if not ok_track:
                    continue
                pen = _score(full, 0.0)
                if best is None or pen < best[0] - 1e-6:   # ties: first wins
                    best = (pen, {"kind": "waypoints", "pts": full}, "dubins",
                            R, _length + tail)
    if best is not None:
        note = None
        if best[2] == "dubins":
            note = (f"glue is a Dubins path (R={best[3]:.2f}m, "
                    f"{best[4]:.1f}m) — regenerate the plan "
                    "rather than hand-editing it")
        return best[1], note
    return None, (f"no trackable glue ({tried} Bezier + Dubins candidates, "
                  f"R {radii[0]:.1f}..{radii[-1]:.2f}, all violate clearance "
                  f"of {req:.2f}m or the {track_r:.2f}m trackable radius)")


# ----------------------------------------------------------------------
# Placement search
# ----------------------------------------------------------------------

def _candidates(pts_local, poly, excl, req, cfg, prefer_near=None,
                prefer_heading=None):
    """Coarse-to-fine search. Returns scored [(score, x, y, h_deg), ...]
    best-first (at most ~20). prefer_heading (compass deg) biases toward a
    target heading — the A/B opposed-pair placement wants ~+180 deg."""
    xs = [p[0] for p in poly]
    ys = [p[1] for p in poly]
    grid = float(cfg["grid_m"])
    hstep = float(cfg["heading_step_deg"])
    target = req + float(cfg["fit_buffer_m"])

    def _scan(x0, x1, y0, y1, gx, h_list):
        found = []
        y = y0
        while y <= y1:
            x = x0
            while x <= x1:
                if _clear((x, y), poly, excl) >= req:   # cheap start gate
                    for h in h_list:
                        mn = _min_clear_placed(
                            pts_local, x, y, h, poly, excl, target)
                        if mn >= target:
                            # Distance band: a start ~4 m from the previous
                            # curve's end leaves the glue room for a trackable
                            # hook — touching-close is as bad as far away.
                            d = (abs(math.hypot(x - prefer_near[0],
                                                y - prefer_near[1]) - 4.0)
                                 if prefer_near else 0.0)
                            score = min(mn, req + 0.5) - 0.08 * d
                            if prefer_heading is not None:
                                # 1.8 at 180 deg off — dominates the band, so
                                # opposed placements sort first.
                                score -= 0.01 * _ang_diff_deg(h, prefer_heading)
                            found.append((score, x, y, h))
                x += gx
            y += gx
        return found

    h_coarse = [i * hstep for i in range(int(360.0 / hstep))]
    coarse = _scan(min(xs), max(xs), min(ys), max(ys), grid, h_coarse)
    coarse.sort(key=lambda c: -c[0])
    out = list(coarse[:8])
    fg = float(cfg["refine_grid_m"])
    fh = float(cfg["refine_heading_deg"])
    for (_s, cx, cy, ch) in coarse[:4]:
        hs = [ch + k * fh for k in (-2, -1, 1, 2)]
        out += _scan(cx - grid / 2, cx + grid / 2,
                     cy - grid / 2, cy + grid / 2, fg, hs + [ch])
    out.sort(key=lambda c: -c[0])
    # Thin near-duplicates so retry attempts are actually diverse.
    seen, uniq = set(), []
    for c in out:
        key = (round(c[1] / 0.5), round(c[2] / 0.5), round(c[3] / 15.0))
        if key in seen:
            continue
        seen.add(key)
        uniq.append(c)
        if len(uniq) >= 20:
            break
    return uniq


def _exit_bearing(h_deg, end_yaw_local):
    """Compass bearing of the curve end. Local yaw is CCW; bearings are CW."""
    return (h_deg - math.degrees(end_yaw_local)) % 360.0


def _glue_out(glue_en, cfg, lat0, lon0):
    """Planner-internal EN glue -> the WebUI lbRepos entry (lat/lon)."""
    pts = [dict(zip(("lat", "lon"), _en_to_latlon(e, n, lat0, lon0)))
           for (e, n) in glue_en["pts"]]
    out = {"mids": [], "v_const": float(cfg["glue"]["v_const"]),
           "pos_tol_m": float(cfg["glue"]["pos_tol_m"])}
    if glue_en["kind"] == "mids":
        out["mids"] = pts
    else:
        out["waypoints"] = pts
    return out


# ----------------------------------------------------------------------
# Best-effort placement (field 2026-06-15): when a geometry cannot be
# auto-placed to clear the margin with a trackable glue, the planner used to
# DROP it into ``unfittable`` (operator could then never touch it). Instead
# emit a rough opposed pair flagged ``needs_fix`` so the operator drags it
# into spec in the WebUI; the NUC venue_loader still validates on Send, so a
# red leg never drives.
# ----------------------------------------------------------------------

def _poly_centroid(poly):
    n = max(1, len(poly))
    return (sum(p[0] for p in poly) / n, sum(p[1] for p in poly) / n)


def _poly_major_bearing(poly):
    """Compass bearing of the polygon's PCA major axis (its long direction)."""
    cx, cy = _poly_centroid(poly)
    n = len(poly)
    sxx = sum((p[0] - cx) ** 2 for p in poly) / n
    syy = sum((p[1] - cy) ** 2 for p in poly) / n
    sxy = sum((p[0] - cx) * (p[1] - cy) for p in poly) / n
    t = sxx + syy
    disc = math.sqrt(max(t * t - 4.0 * (sxx * syy - sxy * sxy), 0.0))
    lam = (t + disc) / 2.0
    if abs(sxy) > 1e-12:
        ve, vn = sxy, lam - sxx
    else:
        ve, vn = (1.0, 0.0) if sxx >= syy else (0.0, 1.0)
    return math.degrees(math.atan2(ve, vn)) % 360.0


# ----------------------------------------------------------------------
# Venue-aligned packing (2026-10-02). The grid search above scores placements
# by clearance capped at req + 0.5, so in a roomy spot many candidates tie and
# the stable sort keeps scan order — headings scanned from 0 deg, i.e. curves
# pointing north/south regardless of how the venue is rotated, and an EN grid
# that does not line up with the walls. Here the A/B pair is instead built in
# the venue's own frame: curve A may only point along a wall direction, the
# whole pair (+ its glue) is treated as one rigid block, and the block is slid
# to where it clears the walls best.
# ----------------------------------------------------------------------

def _hull(points):
    """Convex hull (monotone chain), CCW."""
    pts = sorted(set((float(p[0]), float(p[1])) for p in points))
    if len(pts) <= 2:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def _venue_rect(poly):
    """Minimum-area rectangle around the venue (one side lies on a hull edge).
    The WebUI snaps corner 4 to a rectangle, so for a captured venue this IS
    the venue. Returns (bearing of the long side, centre (E, N), long, short)."""
    hull = _hull(poly)
    best = None
    for i in range(len(hull)):
        (x0, y0), (x1, y1) = hull[i], hull[(i + 1) % len(hull)]
        d = math.hypot(x1 - x0, y1 - y0)
        if d < 1e-9:
            continue
        ux, uy = (x1 - x0) / d, (y1 - y0) / d
        a = [p[0] * ux + p[1] * uy for p in hull]
        b = [-p[0] * uy + p[1] * ux for p in hull]
        area = (max(a) - min(a)) * (max(b) - min(b))
        if best is None or area < best[0]:
            best = (area, ux, uy, min(a), max(a), min(b), max(b))
    _, ux, uy, a0, a1, b0, b1 = best
    ca, cb = (a0 + a1) / 2.0, (b0 + b1) / 2.0
    centre = (ca * ux - cb * uy, ca * uy + cb * ux)
    if a1 - a0 >= b1 - b0:
        lv, long_, short = (ux, uy), a1 - a0, b1 - b0
    else:
        lv, long_, short = (-uy, ux), b1 - b0, a1 - a0
    return math.degrees(math.atan2(lv[0], lv[1])) % 360.0, centre, long_, short


def _glue_samples(g, a, b):
    """EN samples of a planned glue from a to b (mids are Bezier controls)."""
    if g["kind"] == "mids":
        return _bezier_samples([a] + list(g["pts"]) + [b])
    return list(g["pts"])


def _rot90(pts, k):
    """Rotate EN points about the origin by k x 90 deg of compass bearing
    (clockwise): bearing h -> h + 90 maps (E, N) -> (N, -E)."""
    out = list(pts)
    for _ in range(k % 4):
        out = [(n, -e) for (e, n) in out]
    return out


def _aligned_pairs(variants, entry, poly, excl, req, cfg, keep=4):
    """Wall-aligned A/B pair placements, best first (at most ``keep``).

    Each entry is (placed, glues_en, quality) where quality = (glue radius,
    settled tail, clearance) of the pair's worse glue/curve. Candidates are
    compared with tolerances (_beats), never on exact floats, and near-ties
    keep a fixed enumeration order (field 2026-10-02: an exact tie broken by
    the last ulp flipped a stage to a placement whose transit glue could not
    be planned) — so the plan is deterministic and robust to tiny noise."""
    g = cfg["glue"]
    beta, centre, long_, short = _venue_rect(poly)
    target = req + float(cfg["fit_buffer_m"])
    fu = _bearing_vec(beta)                 # long-axis unit vector (EN)
    fv = (fu[1], -fu[0])                    # short axis

    def uv(p):
        return (p[0] * fu[0] + p[1] * fu[1], p[0] * fv[0] + p[1] * fv[1])
    cu0, cv0 = uv(centre)

    # Phase 1 (cheap): every layout = (variant, k, side, w, s). The free-space
    # glue is planned ONCE per (variant, side, w, s) at k = 0 and turned for
    # the other three wall directions — free-space planning is rotation-
    # equivariant, so this is exact and 4x cheaper. Keep each layout's best
    # shift (centred first) by curve clearance; the compact glue must clear
    # too, so phase 2 always has at least that glue in the real venue.
    layouts = []
    for vi, (recipe, pts, end_yaw) in enumerate(variants):
        A0 = entry(recipe, pts, end_yaw, 0.0, 0.0, beta, 1)
        ex, ey = _bearing_vec(A0["exit_b"])
        for sgn in (1.0, -1.0):
            nx, ny = ey * sgn, -ex * sgn
            for w in (1.8, 2.0, 2.2, 2.4, 2.8, 3.2, 3.6):
                for s_ in (0.0, 1.0, 2.0):
                    b_start = (A0["end_en"][0] + nx * w + ex * s_,
                               A0["end_en"][1] + ny * w + ey * s_)
                    B0 = entry(recipe, pts, end_yaw, *b_start,
                               (beta + 180.0) % 360.0, 2)
                    g1, _n = _plan_glue(A0["end_en"], A0["exit_b"],
                                        B0["start_en"], B0["h"],
                                        None, [], req, cfg, objective="compact")
                    if g1 is None:
                        continue
                    # B is A turned 180 deg about P, so the return glue is g1
                    # turned the same way.
                    P = (B0["start_en"][0] / 2.0, B0["start_en"][1] / 2.0)
                    g2pts = [(2 * P[0] - q[0], 2 * P[1] - q[1]) for q in g1["pts"]]
                    curves0 = (_place(pts, 0.0, 0.0, beta)
                               + _place(pts, *B0["start_en"], B0["h"]))
                    glue0 = (_glue_samples(g1, A0["end_en"], B0["start_en"])
                             + _glue_samples({"kind": g1["kind"], "pts": g2pts},
                                             B0["end_en"], (0.0, 0.0)))
                    for k in range(4):
                        crv = np.asarray(_rot90(curves0, k))
                        glu = np.asarray(_rot90(glue0, k))
                        bu = [uv(q) for q in np.vstack((crv, glu)).tolist()]
                        u0, u1 = min(q[0] for q in bu), max(q[0] for q in bu)
                        v0, v1 = min(q[1] for q in bu), max(q[1] for q in bu)
                        su = long_ - 2.0 * target - (u1 - u0)
                        sv = short - 2.0 * target - (v1 - v0)
                        if su < 0.0 or sv < 0.0:
                            continue        # the block cannot fit this way round
                        hA = (beta + 90.0 * k) % 360.0
                        bs = _rot90([B0["start_en"]], k)[0]
                        du0, dv0 = cu0 - (u0 + u1) / 2.0, cv0 - (v0 + v1) / 2.0
                        shifts = [(du0, dv0)] + [
                            (du0 + su * i / 4.0, dv0 + sv * j / 4.0)
                            for i in (-2, -1, 0, 1, 2) for j in (-2, -1, 0, 1, 2)
                            if (i, j) != (0, 0)]
                        best_shift = None
                        for (du, dv) in shifts:
                            ox = du * fu[0] + dv * fv[0]
                            oy = du * fu[1] + dv * fv[1]
                            off = np.array([ox, oy])
                            cl = float(_clear_many(crv + off, poly, excl).min())
                            if cl < target:
                                continue
                            # The compact glue must clear too: then phase 2 has
                            # at least this glue available in the real venue.
                            if float(_clear_many(glu + off, poly, excl).min()) < req:
                                continue
                            if best_shift is None or cl > best_shift[0] + 0.005:
                                best_shift = (cl, ox, oy)
                        if best_shift is None:
                            continue
                        # Spare room around the compact block is where the
                        # real-venue glue can grow gentler: rank by it.
                        layouts.append(((min(su, sv), best_shift[0]),
                                        vi, hA, bs, best_shift))

    # Phase 2: validate the most promising layouts against the real walls
    # (both glues re-planned with clearance + trackable radius + tail gate).
    layouts = _tol_sorted(layouts, key=lambda t: t[0], tol=(0.05, 0.005))
    out = []
    for (_key, vi, hA, bs, (cl, ox, oy)) in layouts[:24]:
        recipe, pts, end_yaw = variants[vi]
        A = entry(recipe, pts, end_yaw, ox, oy, hA, 1)
        B = entry(recipe, pts, end_yaw, bs[0] + ox, bs[1] + oy,
                  (hA + 180.0) % 360.0, 2)
        r1 = _plan_glue(A["end_en"], A["exit_b"], B["start_en"], B["h"],
                        poly, excl, req, cfg)
        if r1[0] is None:
            continue
        r2 = _plan_glue(B["end_en"], B["exit_b"], A["start_en"], A["h"],
                        poly, excl, req, cfg)
        if r2[0] is None:
            continue
        q1 = _glue_quality(_glue_samples(r1[0], A["end_en"], B["start_en"]),
                           B["h"], g)
        q2 = _glue_quality(_glue_samples(r2[0], B["end_en"], A["start_en"]),
                           A["h"], g)
        quality = (min(q1[0], q2[0]), min(q1[1], q2[1]), cl)
        out.append((quality, len(out), ([A, B], [r1, r2], quality)))
        if len(out) >= 2 * keep:
            break       # roomiest layouts first; keep the smoothest of these
    out = _tol_sorted(out, key=lambda t: t[0], tol=(0.05, 0.05, 0.005))
    return [c for (_q, _i, c) in out[:keep]]


def _glue_floor(cfg):
    g = cfg["glue"]
    return max(float(g.get("track_radius_m", 0.7)), float(g["hard_radius_m"]))


def _floor_cfgs(cfg):
    """[cfg] plus, if the fallback floor is lower, a copy planned at it."""
    fb = float(cfg["glue"].get("track_radius_fallback_m", 0.0) or 0.0)
    if not fb or fb >= _glue_floor(cfg):
        return [cfg]
    lo = dict(cfg)
    lo["glue"] = dict(cfg["glue"], track_radius_m=fb)
    return [cfg, lo]


def _naive_glue(end, exit_b, start, start_b, cfg):
    """A simple un-validated hook end -> start, emitted so a best-effort stage
    always carries an editable glue (the operator reshapes it in the WebUI)."""
    g = cfg["glue"]
    ctrl = _glue_ctrl(end, exit_b, start, start_b, cfg,
                      float(g["lead_m"]), float(g["tail_m"]))
    return {"kind": "mids", "pts": ctrl[1:-1]}


# ----------------------------------------------------------------------
# Public entry
# ----------------------------------------------------------------------

def plan_stages(venue, doc, counts, footprint_r=0.30, track_margin=0.30,
                key_fn=None, calibration=None):
    """Plan stages covering every remaining (family, R) geometry.

    Returns the dict described in the module docstring; never raises on bad
    input (returns ok=False + notes instead) so a ROS host can publish the
    result verbatim.

    ``calibration`` (2026-10-08): None, or {"mode": "full"|"sanity",
    "pin": {lat, lon, heading_deg} | None}. Prepends a stage named
    "calibration" holding the open-loop figure-8 (calibration.py) and its
    approach loop; the matrix stages follow with the usual inter-stage glue,
    so the robot transits from the figure-8 into the first matrix stage
    unattended. "pin" re-seats an already-driven calibration at the same
    place (the executor's re-plan after it locks the radii).
    """
    notes, unfittable, stages = [], [], []
    corners = (venue or {}).get("corners_wgs84") or []
    if len(corners) < 3:
        return {"ok": False, "stages": [], "unfittable": [],
                "notes": ["venue has no polygon (need >= 3 corners_wgs84)"]}
    cfg = _plan_cfg(doc)
    floor_cfgs = _floor_cfgs(cfg)
    lat0, lon0 = corners[0]["lat"], corners[0]["lon"]
    poly = venue_geom.poly_en(corners, lat0, lon0)
    excl = []
    for ex in (venue.get("exclusions") or []):
        if str(ex.get("kind", "")).lower() == "circle":
            ce, cn = venue_geom.latlon_to_en(
                float(ex["lat"]), float(ex["lon"]), lat0, lon0)
            excl.append((ce, cn, float(ex["radius_m"])))
        else:
            return {"ok": False, "stages": [], "unfittable": [],
                    "notes": [f"unsupported exclusion kind {ex.get('kind')!r} "
                              "— planner fails closed like the loader"]}
    req = float(venue.get("safety_margin_m", 0.0)) \
        + float(footprint_r) + float(track_margin)
    # C (field 2026-06-16): lay each A/B racetrack ALONG the venue's long axis —
    # the curves point down the length, the lateral A/B separation opens across
    # the short axis (which has the most room for a wider, trackable turnaround).
    # Biases A's heading; B follows opposed, so the whole pair tracks the axis.
    major_bearing = _poly_major_bearing(poly)

    spacing = 0.25      # search-time sampling; final check is the loader's 0.10
    stage_geo = []      # per assembled stage: exit/entry poses (EN) for transit
    if calibration:
        cst, cgeo, cnote = _calibration_stage(
            calibration, doc, poly, excl, req, cfg, lat0, lon0, major_bearing)
        if cst is None:
            return {"ok": False, "stages": [], "unfittable": [],
                    "req_clearance_m": req, "notes": [cnote]}
        if cnote:
            notes.append(cnote)
        stages.append(cst)
        stage_geo.append(cgeo)

    remaining = remaining_geometries(doc, counts, key_fn=key_fn)
    if not remaining:
        if stages:
            notes.append(
                "no matrix cells to place yet" if (doc or {}).get("_radius_auto")
                and not ((doc or {}).get("matrix") or {}).get("radius_m")
                else "matrix complete — no remaining (family, R) cells")
            if (doc or {}).get("_radius_auto") and not (
                    (doc or {}).get("matrix") or {}).get("radius_m"):
                notes.append("matrix radii come from the calibration: the "
                             "executor plans the matrix itself once the "
                             "figure-8 has locked them")
            return {"ok": True, "stages": stages, "unfittable": [],
                    "req_clearance_m": req, "notes": notes}
        return {"ok": True, "stages": [], "unfittable": [],
                "req_clearance_m": req,
                "notes": ["matrix complete — no remaining (family, R) cells"]}

    queue = list(remaining)
    # Stage shape (field decision 2026-06-11): one GEOMETRY per stage, placed
    # as an A/B OPPOSED PAIR — the same recipe twice, headings ~180 deg apart,
    # glued into a racetrack (two ~180 turnarounds, far easier to keep above
    # the trackable radius than one 360 self-loop). The executor's least-done
    # treatment cycling then alternates runs between the two directions, so
    # slope/wind/mount bias averages out of every cell. Stages never mix
    # geometries any more. max_geometries_per_stage <= 1 keeps the legacy
    # single-curve (self-loop) stages — the stage-advance shakedown uses it.
    ab_pair = int(cfg["max_geometries_per_stage"]) >= 2
    guard = 0
    while queue and guard < 64:
        guard += 1
        fam, R, rem = queue.pop(0)
        variants = []
        # Right step (direction -1) disabled 2026-06-12 (field): its shape was
        # rejected on the roof. Left step (+1) only; slalom has no direction.
        for direction in (1,):
            recipe = _recipe_for(fam, R, cfg, direction)
            if recipe is None:
                break
            pts, end_yaw = _local_samples(recipe, spacing)
            if pts is None:
                break
            variants.append((recipe, pts, end_yaw))
        if not variants:
            unfittable.append({"family": fam, "R": R,
                               "reason": "recipe not buildable "
                                         "(unknown family or vfg import)"})
            continue

        def _entry(recipe, pts, end_yaw, x, y, h, idx):
            return {"id": f"exp{idx}", "recipe": recipe, "fam": fam, "R": R,
                    "rem": rem, "start_en": (x, y), "h": h,
                    "exit_b": _exit_bearing(h, end_yaw),
                    "end_en": _place(pts, x, y, h)[-1]}

        placed, glues_en, best_effort = None, None, False

        if ab_pair:
            target = req + float(cfg["fit_buffer_m"])

            def _try_B(A, recipe, pts, end_yaw, xB, yB, hB):
                if _min_clear_placed(pts, xB, yB, hB, poly, excl,
                                     target) < target:
                    return None
                B = _entry(recipe, pts, end_yaw, xB, yB, hB, 2)
                g1, n1 = _plan_glue(A["end_en"], A["exit_b"],
                                    B["start_en"], B["h"],
                                    poly, excl, req, cfg)
                if g1 is None:
                    return None
                g2, n2 = _plan_glue(B["end_en"], B["exit_b"],
                                    A["start_en"], A["h"],
                                    poly, excl, req, cfg)
                if g2 is None:
                    return None
                return B, [(g1, n1), (g2, n2)]

            # Wall-aligned rigid-block packing first (plan.packing, default
            # "aligned"); the free-heading grid search below is the fallback
            # for venues where no aligned block fits (odd shape, exclusions).
            if cfg.get("packing", "aligned") == "aligned":
                def _transit_tier(pp):
                    """Floor tier at which the inter-stage glue from the
                    previous stage's exit reaches pp's entry (None: neither)."""
                    if not stage_geo:
                        return 0
                    prev = stage_geo[-1]
                    for ti, tc in enumerate(floor_cfgs):
                        if _plan_glue(prev["exit_en"], prev["exit_b"],
                                      pp[0]["start_en"], pp[0]["h"],
                                      poly, excl, req, tc)[0] is not None:
                            return ti
                    return None
                for tier, tcfg in enumerate(floor_cfgs):
                    cands = _aligned_pairs(variants, _entry, poly, excl, req,
                                           tcfg)
                    if not cands:
                        continue
                    # Transit-aware pick: the smoothest candidate (either curve
                    # as the stage entry) whose transit from the previous stage
                    # plans, preferring a transit at the preferred floor. Without
                    # this a good pair could leave the transit best-effort.
                    opts = []
                    for ci, (pl, gl, _q) in enumerate(cands):
                        for oi, (pp, gg) in enumerate(
                                [(pl, gl), ([pl[1], pl[0]], [gl[1], gl[0]])]):
                            tt = _transit_tier(pp)
                            opts.append(((9 if tt is None else tt, ci, oi), pp, gg))
                            if tt == 0:
                                break
                        if opts[-1][0][0] == 0:
                            break
                    _k, placed, glues_en = min(opts, key=lambda o: o[0])
                    # ids follow the stage order (exp1 = entry curve)
                    placed = [dict(p, id=f"exp{i + 1}") for i, p in enumerate(placed)]
                    if tier > 0:
                        notes.append(
                            f"{fam} R{R}: venue too tight for the "
                            f"{_glue_floor(cfg):.2f}m glue floor — its turnarounds "
                            f"use {_glue_floor(tcfg):.2f}m, ~the steering limit "
                            "(expect larger arrival heading error)")
                    break

            for (recipe, pts, end_yaw) in (variants if placed is None else []):
                # align_major: try with major-axis heading bias first; if no
                # A/B pair fits from those positions, retry without the bias
                # so the venue constraint never fully blocks pairing.
                _prefer_list = ([major_bearing, None]
                                if cfg.get("align_major", True) else [None])
                for _a_prefer in _prefer_list:
                    for (_sA, xA, yA, hA) in _candidates(
                            pts, poly, excl, req, cfg,
                            prefer_heading=_a_prefer)[:6]:
                        A = _entry(recipe, pts, end_yaw, xA, yA, hA, 1)
                        opp = (hA + 180.0) % 360.0
                        # Closed-form racetrack slots first: B.start = A.end +
                        # lateral offset w (+ optional slide s along the exit),
                        # heading exactly opposed. The same recipe rotated 180
                        # then ENDS at A.start + the same offset, so BOTH glues
                        # are clean ~w/2-radius turnarounds by construction —
                        # the grid search rarely lands in this slot on its own.
                        ex_, ey_ = _bearing_vec(A["exit_b"])
                        cand_B = []
                        for sgn in (1.0, -1.0):
                            nx_, ny_ = ey_ * sgn, -ex_ * sgn
                            # w = lateral A/B offset; the return turnaround radius
                            # is ~w/2. Fine-grained 1.8..3.6 (field 2026-06-16): the
                            # track_radius_m gate rejects any w whose R=w/2 is below
                            # the floor, so the FIRST surviving w is the tightest
                            # TRACKABLE separation that still seats an A/B pair —
                            # gentler than the old undershooting 1.8 (R 0.9) but not
                            # so wide it forces a single-curve fallback.
                            for w in (1.8, 2.0, 2.2, 2.4, 2.8, 3.2, 3.6):
                                for s_ in (0.0, 1.0, 2.0):
                                    cand_B.append(
                                        (A["end_en"][0] + nx_ * w + ex_ * s_,
                                         A["end_en"][1] + ny_ * w + ey_ * s_,
                                         opp))
                        for (xB, yB, hB) in cand_B:
                            got = _try_B(A, recipe, pts, end_yaw, xB, yB, hB)
                            if got is not None:
                                placed, glues_en = [A, got[0]], got[1]
                                break
                        if placed is None:
                            # Grid fallback: anywhere opposed-ish that glues.
                            for (_sB, xB, yB, hB) in _candidates(
                                    pts, poly, excl, req, cfg,
                                    prefer_near=A["end_en"],
                                    prefer_heading=opp)[:6]:
                                if _ang_diff_deg(hB, opp) > 60.0:
                                    continue   # not opposed enough to de-bias
                                got = _try_B(A, recipe, pts, end_yaw, xB, yB, hB)
                                if got is not None:
                                    placed, glues_en = [A, got[0]], got[1]
                                    break
                        if placed:
                            break
                    if placed:
                        break
                if placed:
                    break
            if placed is None:
                notes.append(f"{fam} R{R}: no trackable A/B pair fits — "
                             "falling back to a single curve (directional "
                             "bias NOT cancelled for this geometry)")

        if placed is None:
            # Single curve with a self-loop glue (legacy shape; also the
            # fallback when the venue cannot host an opposed pair).
            for (recipe, pts, end_yaw) in variants:
                for (_s, x, y, h) in _candidates(pts, poly, excl,
                                                 req, cfg)[:20]:
                    P = _entry(recipe, pts, end_yaw, x, y, h, 1)
                    g, note = _plan_glue(P["end_en"], P["exit_b"],
                                         P["start_en"], P["h"],
                                         poly, excl, req, cfg)
                    if g is None:
                        continue
                    placed, glues_en = [P], [(g, note)]
                    break
                if placed:
                    break
        if placed is None and ab_pair:
            # Best-effort opposed pair: nothing here clears the margin with a
            # trackable glue, but DON'T drop the geometry. Seat a rough A/B
            # pair (closed-form racetrack slot) the operator drags to green;
            # flagged needs_fix and the loader re-validates on Send.
            recipe, pts, end_yaw = variants[0]
            xA = yA = hA = None
            for relax in (0.6, 0.3, 0.1):     # ease the margin to find a seat
                seats = _candidates(pts, poly, excl, req * relax, cfg)
                if seats:
                    _s, xA, yA, hA = seats[0]
                    break
            if xA is None:                    # curve larger than the polygon
                xA, yA = _poly_centroid(poly)
                hA = _poly_major_bearing(poly)
            A = _entry(recipe, pts, end_yaw, xA, yA, hA, 1)
            opp = (hA + 180.0) % 360.0
            ex_, ey_ = _bearing_vec(A["exit_b"])
            B = _entry(recipe, pts, end_yaw,
                       A["end_en"][0] + ey_ * 2.4,
                       A["end_en"][1] - ex_ * 2.4, opp, 2)
            g1, n1 = _plan_glue(A["end_en"], A["exit_b"], B["start_en"],
                                B["h"], poly, excl, req, cfg)
            if g1 is None:
                g1 = _naive_glue(A["end_en"], A["exit_b"], B["start_en"],
                                 B["h"], cfg)
                n1 = "best-effort hook — reshape in the WebUI"
            g2, n2 = _plan_glue(B["end_en"], B["exit_b"], A["start_en"],
                                A["h"], poly, excl, req, cfg)
            if g2 is None:
                g2 = _naive_glue(B["end_en"], B["exit_b"], A["start_en"],
                                 A["h"], cfg)
                n2 = "best-effort hook — reshape in the WebUI"
            placed, glues_en, best_effort = [A, B], [(g1, n1), (g2, n2)], True
            notes.append(
                f"{fam} R{R}: BEST-EFFORT opposed pair (does not clear "
                f"{req:.2f}m) — drag it to green in the WebUI before Send")
        if placed is None:
            unfittable.append({
                "family": fam, "R": R,
                "reason": ("no placement supports a trackable A/B pair or "
                           f"self-loop clearing {req:.2f}m")})
            continue

        glues = []
        for gi, (g, note) in enumerate(glues_en):
            if note:
                notes.append(f"stage {len(stages) + 1} glue {gi}: {note}")
            glues.append(_glue_out(g, cfg, lat0, lon0))

        stages.append({
            "name": f"stage_{len(stages) + 1}",
            # One geometry per stage (the A/B pair shares it): listed ONCE so
            # downstream accounting (fuzz invariant, executor scoring) sees
            # each (family, R) in exactly one place.
            "geometries": [{"family": fam, "R": R, "remaining_runs": rem}],
            "experiments": [{
                "id": p["id"], "recipe": p["recipe"],
                "start": dict(zip(("lat", "lon"),
                                  _en_to_latlon(*p["start_en"], lat0, lon0)),
                              heading_deg=round(p["h"], 1)),
            } for p in placed],
            "glues": glues,
            "needs_fix": best_effort,
        })
        if best_effort:
            stages[-1]["fix_reason"] = (
                "auto-placed best-effort: no trackable A/B pair clears "
                f"{req:.2f}m in this venue — adjust the curves/glue until the "
                "legs render green, then Send")
        stage_geo.append({"exit_en": placed[-1]["end_en"],
                          "exit_b": placed[-1]["exit_b"],
                          "entry_en": placed[0]["start_en"],
                          "entry_b": placed[0]["h"]})
    if queue:
        for (fam, R, rem) in queue:
            unfittable.append({"family": fam, "R": R,
                               "reason": "planner retry budget exhausted"})

    # One inter-stage transit glue per boundary. The robot finishes stage i at
    # SOME exp end (not statically known), walks the stage's own already-
    # validated loop to the stage EXIT pose (the last experiment's end), then
    # drives this glue to stage i+1's first start pin.
    #
    # B (2026-06-16): NEVER skipped — a boundary with no trackable glue gets a
    # best-effort editable hook (flagged needs_fix), not a silent drop to the
    # executor's blind path-join. C: emitted with editable control ``mids`` and
    # its START pin ``start_wgs84`` (the prev stage's exit, which is NOT in the
    # destination stage's experiments) so the WebUI can rebuild + hand-edit the
    # Bezier; ``waypoints_wgs84`` stays the source of truth for the loader /
    # executor (the WebUI regenerates it from the mids on Send). Dubins-fallback
    # glues carry waypoints only (non-editable, as the intra-stage ones do).
    for i in range(len(stages) - 1):
        a, b = stage_geo[i], stage_geo[i + 1]
        glue_en, note = _plan_glue(a["exit_en"], a["exit_b"],
                                   b["entry_en"], b["entry_b"],
                                   poly, excl, req, cfg)
        if glue_en is None and len(floor_cfgs) > 1:
            glue_en, note = _plan_glue(a["exit_en"], a["exit_b"],
                                       b["entry_en"], b["entry_b"],
                                       poly, excl, req, floor_cfgs[1])
            if glue_en is not None:
                note = (f"transit uses the {_glue_floor(floor_cfgs[1]):.2f}m "
                        "fallback glue floor (~the steering limit)"
                        + (f"; {note}" if note else ""))
        pair = f"{stages[i]['name']} -> {stages[i + 1]['name']}"
        eg_fix = False
        if glue_en is None:
            glue_en = _naive_glue(a["exit_en"], a["exit_b"],
                                  b["entry_en"], b["entry_b"], cfg)
            eg_fix = True
            notes.append(f"inter-stage glue {pair}: BEST-EFFORT hook (does not "
                         f"clear {req:.2f}m) — drag it to green before Send")
        elif note:
            notes.append(f"inter-stage glue {pair}: {note}")
        mids_en = list(glue_en["pts"]) if glue_en["kind"] == "mids" else None
        if mids_en is not None:
            pts_en = _bezier_samples([a["exit_en"]] + mids_en + [b["entry_en"]])
        else:
            pts_en = list(glue_en["pts"])     # Dubins: non-editable waypoints
        entry = {
            "from_stage": stages[i]["name"],
            "start_wgs84": dict(zip(("lat", "lon"),
                                    _en_to_latlon(*a["exit_en"], lat0, lon0))),
            "waypoints_wgs84": [
                dict(zip(("lat", "lon"), _en_to_latlon(e, n, lat0, lon0)))
                for (e, n) in pts_en],
            "v_const": float(cfg["glue"]["v_const"]),
            "pos_tol_m": float(cfg["glue"]["pos_tol_m"]),
            "end_heading_deg": round(b["entry_b"], 1),
            "needs_fix": eg_fix,
        }
        if mids_en is not None:
            entry["mids"] = [
                dict(zip(("lat", "lon"), _en_to_latlon(e, n, lat0, lon0)))
                for (e, n) in mids_en]
        stages[i + 1]["entry_glue"] = entry

    # First-pass wrap (2026-10-08, plan.first_pass_reps): the executor sweeps
    # every stage once at a low per-cell target, then wraps from the LAST stage
    # back to the FIRST matrix stage for the remaining reps. Plan that transit
    # like any inter-stage glue so the wrap is not a blind path-join. Skipped
    # (best-effort glue never emitted) when it does not plan cleanly.
    fm = 1 if (stages and stages[0].get("calibration")) else 0
    last = len(stages) - 1
    if int(cfg.get("first_pass_reps", 0) or 0) > 0 and last > fm:
        a, b = stage_geo[last], stage_geo[fm]
        glue_en, note = _plan_glue(a["exit_en"], a["exit_b"],
                                   b["entry_en"], b["entry_b"],
                                   poly, excl, req, cfg)
        if glue_en is None and len(floor_cfgs) > 1:
            glue_en, note = _plan_glue(a["exit_en"], a["exit_b"],
                                       b["entry_en"], b["entry_b"],
                                       poly, excl, req, floor_cfgs[1])
        if glue_en is None:
            notes.append(f"wrap glue {stages[last]['name']} -> "
                         f"{stages[fm]['name']}: none planned (the executor "
                         "path-joins on the first-pass wrap)")
        else:
            if glue_en["kind"] == "mids":
                pts_en = _bezier_samples([a["exit_en"]] + list(glue_en["pts"])
                                         + [b["entry_en"]])
            else:
                pts_en = list(glue_en["pts"])
            stages[fm]["wrap_glue"] = {
                "from_stage": stages[last]["name"],
                "start_wgs84": dict(zip(("lat", "lon"),
                                        _en_to_latlon(*a["exit_en"], lat0, lon0))),
                "waypoints_wgs84": [
                    dict(zip(("lat", "lon"), _en_to_latlon(e, n, lat0, lon0)))
                    for (e, n) in pts_en],
                "v_const": float(cfg["glue"]["v_const"]),
                "pos_tol_m": float(cfg["glue"]["pos_tol_m"]),
                "end_heading_deg": round(b["entry_b"], 1),
                "needs_fix": False,
            }

    needs_attention = (any(st.get("needs_fix") for st in stages)
                       or any((st.get("entry_glue") or {}).get("needs_fix")
                              for st in stages))
    return {"ok": bool(stages) and not unfittable and not needs_attention,
            "req_clearance_m": req,
            "stages": stages,
            "unfittable": unfittable,
            "notes": notes}



# ----------------------------------------------------------------------
# Calibration stage (2026-10-08) + planner stages -> active.json legs
# ----------------------------------------------------------------------

def _calibration_stage(calibration, doc, poly, excl, req, cfg, lat0, lon0,
                       major_bearing):
    """(stage, stage_geo, note) for the calibration figure-8, or
    (None, None, reason) when it fits nowhere. The cleared footprint is the
    approach loop + the figure-8 at the PLANNED radius bound (R_plan_m), so
    any real full-lock radius up to that bound stays inside it."""
    ccfg = calib_mod.config(doc)
    mode = str(calibration.get("mode") or "full")
    pts_local = calib_mod.footprint_local(ccfg)
    target = req + float(cfg["fit_buffer_m"])
    pin = calibration.get("pin")
    note = None
    if pin:
        x, y = venue_geom.latlon_to_en(float(pin["lat"]), float(pin["lon"]),
                                       lat0, lon0)
        h = float(pin.get("heading_deg", 0.0)) % 360.0
        clr = _min_clear_placed(pts_local, x, y, h, poly, excl, target)
        if clr < req:
            note = (f"calibration re-seated at its driven pin clears only "
                    f"{clr:.2f} m (< {req:.2f} m)")
    else:
        cands = _candidates(pts_local, poly, excl, req, cfg,
                            prefer_heading=major_bearing)
        if not cands:
            return None, None, (
                "calibration figure-8 (R_plan "
                f"{float(ccfg['R_plan_m']):.2f} m + approach loop) does not "
                f"fit the venue with {req:.2f} m clearance")
        _sc, x, y, h = cands[0]
    approach = _place(calib_mod.approach_local(ccfg), x, y, h)
    g = cfg["glue"]
    lat, lon = _en_to_latlon(x, y, lat0, lon0)
    stage = {
        "name": "calibration",
        "calibration": mode,
        "geometries": [],
        "experiments": [{
            "id": "cal1",
            "recipe": calib_mod.recipe_for(mode, ccfg),
            "start": {"lat": lat, "lon": lon, "heading_deg": round(h, 1)},
        }],
        "glues": [{
            "mids": [],
            "waypoints": [dict(zip(("lat", "lon"), _en_to_latlon(e, n, lat0, lon0)))
                          for (e, n) in approach],
            "v_const": float(g["v_const"]),
            "pos_tol_m": float(g["pos_tol_m"]),
        }],
        "needs_fix": False,
    }
    geo = {"exit_en": (x, y), "exit_b": h, "entry_en": (x, y), "entry_b": h}
    return stage, geo, note


def stages_to_legs(stages, venue, spacing_m=0.3):
    """Planner stages -> active.json ``plan_stages`` ([{name, legs,
    entry_glue}]), the exact shape the WebUI builds on Send (lbBuiltLegs /
    lbBuildPayload in tools/path_gen/interactive.html). Leg i = the glue that
    ARRIVES at experiment i (glue (i-1) mod n) + experiment i's recipe.
    Planner glues carry either verbatim waypoints (Dubins / calibration
    approach) or Bezier control mids between the previous experiment's END
    and this experiment's start; the mids are sampled with the planner's own
    _bezier_samples, i.e. the geometry the planner cleared. Used by the
    run_executor to re-plan without the browser (post-calibration)."""
    corners = (venue or {}).get("corners_wgs84") or []
    lat0, lon0 = float(corners[0]["lat"]), float(corners[0]["lon"])

    def _ll(e, n):
        la, lo = _en_to_latlon(e, n, lat0, lon0)
        return {"lat": la, "lon": lo}

    out = []
    for st in stages or []:
        exps = st.get("experiments") or []
        glues = st.get("glues") or []
        n = len(exps)
        if n == 0:
            continue
        ends = []
        for e in exps:
            pts = venue_geom.recipe_points_en(e["recipe"], e["start"], lat0, lon0,
                                              spacing_m=0.10)
            if not pts:
                raise ValueError(f"stage {st.get('name')}: recipe "
                                 f"{e['recipe'].get('type')} is not buildable")
            ends.append((pts[-1][0], pts[-1][1]))
        legs = []
        for i, e in enumerate(exps):
            gap = (i - 1 + n) % n
            g = glues[gap] if gap < len(glues) else {}
            if g.get("waypoints"):
                wps = [{"lat": float(w["lat"]), "lon": float(w["lon"])}
                       for w in g["waypoints"]]
            else:
                ctrl = [ends[gap]]
                ctrl += [venue_geom.latlon_to_en(float(m["lat"]), float(m["lon"]),
                                                 lat0, lon0)
                         for m in (g.get("mids") or [])]
                ctrl.append(venue_geom.latlon_to_en(float(e["start"]["lat"]),
                                                    float(e["start"]["lon"]),
                                                    lat0, lon0))
                wps = [_ll(a, b) for (a, b) in _bezier_samples(ctrl, spacing_m)]
            rec = e["recipe"]
            R = (rec.get("params") or {}).get("R")
            name = f"{e['id']}_{rec.get('type')}"
            if R is not None:
                name += "_R" + str(R).replace(".", "p")
            legs.append({"id": f"leg_{i + 1}", "curves": [
                {"name": f"repo_to_{e['id']}", "kind": "reposition",
                 "waypoints_wgs84": wps,
                 "end_heading_deg": float(e["start"]["heading_deg"]),
                 "v_const": float(g.get("v_const", PLAN_DEFAULTS["glue"]["v_const"])),
                 "pos_tol_m": float(g.get("pos_tol_m",
                                          PLAN_DEFAULTS["glue"]["pos_tol_m"]))},
                {"name": name, "kind": "recipe",
                 "scored": not calib_mod.is_calib_recipe(rec),
                 "start_pose": {"lat": float(e["start"]["lat"]),
                                "lon": float(e["start"]["lon"]),
                                "heading_deg": float(e["start"]["heading_deg"])},
                 "recipe": rec},
            ]})
        o = {"name": st.get("name"), "legs": legs}
        if st.get("entry_glue"):
            o["entry_glue"] = st["entry_glue"]
        if st.get("wrap_glue"):
            o["wrap_glue"] = st["wrap_glue"]
        out.append(o)
    return out
