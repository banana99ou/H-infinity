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
nowhere is reported in ``unfittable`` with the reason: the matrix is locked
spec, so the planner must fail loudly, never silently shrink an R.
"""
import math

try:
    from limo_path_follower import venue_geom
except Exception:  # pragma: no cover - in-source / odd layout fallback
    import venue_geom


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
    "glue": {"v_const": 0.2, "pos_tol_m": 0.15,
             "tail_m": 2.2,          # straight, heading-aligned approach tail
             "lead_m": 1.0,          # straight exit along the curve's end heading
             "min_radius_m": 0.5,    # soft: preferred glue turn radius
             # HARD floor: the chassis cannot steer tighter than ~0.37 m, so a
             # drawn glue below this would not be tracked (pure pursuit clamps
             # and leaves the checked corridor). Reject, don't warn.
             "hard_radius_m": 0.37,
             # TRACKABLE floor (B+ smoothness gate, field 2026-06-11): glue is
             # driven by reposition's pure pursuit with a 0.6 m look-ahead — a
             # legal-but-tight Dubins loop (R 0.45) breaks the steering cone
             # mid-arc and aborts. Glue must be drivable WITH MARGIN, not
             # merely chassis-possible.
             "track_radius_m": 0.7,
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
    radii = [float(r) for r in (matrix.get("radius_m") or [])]
    ctrls = [str(c) for c in (matrix.get("controller") or [])]
    speeds = [float(v) for v in (matrix.get("v_const") or [])]
    n = int((doc or {}).get("repetitions") or 0)
    out = []
    for fam in fams:
        for R in radii:
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
        out.sort(key=lambda t: (t[2], fams.index(t[0]), radii.index(t[1])))
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
    """Min clearance of the placed curve; early-exits below stop_below."""
    h = math.radians(h_deg)
    fE, fN = math.sin(h), math.cos(h)
    lE, lN = -math.cos(h), math.sin(h)
    mn = float("inf")
    for (px, py) in pts_local:
        cl = _clear((x + px * fE + py * lE, y + px * fN + py * lN), poly, excl)
        if cl < mn:
            mn = cl
            if mn < stop_below:
                return mn
    return mn


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
    n = max(12, min(300, int(length / max(0.05, spacing_m))))
    return [_bezier(ctrl, i / n) for i in range(n + 1)]


def _max_curvature(pts):
    """Max discrete (Menger) curvature over the sampled polyline."""
    k = 0.0
    for a, b, c in zip(pts[:-2], pts[1:-1], pts[2:]):
        ab = math.hypot(b[0] - a[0], b[1] - a[1])
        bc = math.hypot(c[0] - b[0], c[1] - b[1])
        ca = math.hypot(c[0] - a[0], c[1] - a[1])
        if ab * bc * ca < 1e-9:
            continue
        area2 = abs((b[0] - a[0]) * (c[1] - a[1])
                    - (b[1] - a[1]) * (c[0] - a[0]))
        k = max(k, 2.0 * area2 / (ab * bc * ca))
    return k


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
    n = len(pts)
    for i in range(n - 1):
        ax, ay = pts[i]
        bx, by = pts[i + 1]
        for j in range(i + 2, n - 1):
            if i == 0 and j == n - 2:
                continue
            cx, cy = pts[j]
            dx, dy = pts[j + 1]
            d1 = (dx - cx) * (ay - cy) - (dy - cy) * (ax - cx)
            d2 = (dx - cx) * (by - cy) - (dy - cy) * (bx - cx)
            d3 = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)
            d4 = (bx - ax) * (dy - ay) - (by - ay) * (dx - ax)
            if ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0)):
                return True
    return False


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
        return x % two_pi

    p_sq = 2 + d * d - 2 * c_ab + 2 * d * (sa - sb)
    if p_sq >= 0:
        tmp = math.atan2(cb - ca, d + sa - sb)
        words.append((_m(tmp - alpha), math.sqrt(p_sq), _m(beta - tmp), "LSL"))
    p_sq = 2 + d * d - 2 * c_ab + 2 * d * (sb - sa)
    if p_sq >= 0:
        tmp = math.atan2(ca - cb, d - sa + sb)
        words.append((_m(alpha - tmp), math.sqrt(p_sq), _m(tmp - beta), "RSR"))
    p_sq = -2 + d * d + 2 * c_ab + 2 * d * (sa + sb)
    if p_sq >= 0:
        p = math.sqrt(p_sq)
        tmp = math.atan2(-ca - cb, d + sa + sb) - math.atan2(-2.0, p)
        words.append((_m(tmp - alpha), p, _m(tmp - beta), "LSR"))
    p_sq = -2 + d * d + 2 * c_ab - 2 * d * (sa + sb)
    if p_sq >= 0:
        p = math.sqrt(p_sq)
        tmp = math.atan2(ca + cb, d - sa - sb) - math.atan2(2.0, p)
        words.append((_m(alpha - tmp), p, _m(beta - tmp), "RSL"))
    tmp = (6.0 - d * d + 2 * c_ab + 2 * d * (sa - sb)) / 8.0
    if abs(tmp) <= 1.0:
        p = _m(two_pi - math.acos(tmp))
        t = _m(alpha - math.atan2(ca - cb, d - sa + sb) + p / 2.0)
        words.append((t, p, _m(alpha - beta - t + p), "RLR"))
    tmp = (6.0 - d * d + 2 * c_ab + 2 * d * (sb - sa)) / 8.0
    if abs(tmp) <= 1.0:
        p = _m(two_pi - math.acos(tmp))
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
            n = max(1, int(length / 0.25))
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


def _plan_glue(end, exit_b, start, start_b, poly, excl, req, cfg):
    """Plan one glue gap. Tries operator-editable Bezier mids first
    (lead/tail/dodge variants); when no Bezier is both clear and chassis-
    trackable (e.g. a full lap back onto the same curve), falls back to an
    exact bounded-curvature Dubins path emitted as explicit waypoints.

    Returns (glue_en, note) or (None, reason). glue_en is
    {"kind": "mids"|"waypoints", "pts": [(E, N), ...]} — mids exclude the
    snapped endpoints (WebUI lbRepos schema); waypoints include them."""
    g = cfg["glue"]
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
    best = None     # (penalty, mids, kmax)
    tried = 0
    for lead in (float(g["lead_m"]), 0.5, 1.8):
        for tail in (float(g["tail_m"]), 1.2, 3.0):
            for dodges in single + pairs + laps:
                tried += 1
                ctrl = _glue_ctrl(end, exit_b, start, start_b, cfg,
                                  lead, tail, dodges)
                pts = _bezier_samples(ctrl)
                mn = min(_clear(p, poly, excl) for p in pts)
                if mn < req:
                    continue
                kmax = _max_curvature(pts)
                if kmax > track_k:
                    continue        # not trackable with margin — reject (B+)
                ok_track, _why = check_glue_tracking(pts, start_b, g)
                if not ok_track:
                    continue        # bent tail / gate failure — reject (B+)
                spread = sum(math.hypot(d[0] - chord_mid[0],
                                        d[1] - chord_mid[1]) for d in dodges)
                penalty = 0.3 * spread \
                    + max(0.0, kmax - soft_k) * 3.0 - mn * 0.2
                if best is None or penalty < best[0]:
                    best = (penalty, ctrl[1:-1], kmax)
                    # NOTE: no good-enough early exit here — measured
                    # 2026-06-11: returning the first acceptable variant
                    # (instead of the penalty-best) degrades later
                    # placements enough that full-matrix planning got
                    # SLOWER overall (77-80 s vs 58 s). The polish pays
                    # for itself.
    if best is not None:
        note = None
        if best[2] > soft_k:
            note = (f"glue turn radius {1.0 / best[2]:.2f}m tighter than "
                    f"preferred {g['min_radius_m']:.2f}m (drivable, but "
                    "watch tracking)")
        return {"kind": "mids", "pts": best[1]}, note

    # Dubins fallback: exact start/end poses, curvature bounded by R_turn.
    # Target a pose one straight tail-length BEFORE the start pin so the
    # tracker arrives on a straight, heading-aligned segment (reposition
    # achieves heading by geometry).
    tail = max(1.0, float(g.get("tail_check_m", 0.8)) + 0.2)
    a_yaw = math.radians(90.0 - exit_b)
    b_yaw = math.radians(90.0 - start_b)
    bx = start[0] - sx * tail
    by = start[1] - sy * tail
    # B+ gate: never sweep below the TRACKABLE radius — a chassis-legal 0.45 m
    # Dubins loop breaks reposition's pure-pursuit cone mid-arc (field
    # 2026-06-11: "do a 180 at the E1 start").
    radii = sorted({r for r in (1.2, 1.0, 0.8, track_r) if r >= track_r},
                   reverse=True)
    for R in radii:
        for (_length, pts) in _dubins_paths((end[0], end[1], a_yaw),
                                            (bx, by, b_yaw), R)[:3]:
            full = pts + [(bx + sx * tail * i / 4.0,
                           by + sy * tail * i / 4.0) for i in range(1, 5)]
            if min(_clear(p, poly, excl) for p in full) < req:
                continue
            ok_track, _why = check_glue_tracking(full, start_b, g)
            if not ok_track:
                continue
            note = (f"glue is a Dubins path (R={R:.2f}m, "
                    f"{_length + tail:.1f}m) — regenerate the plan "
                    "rather than hand-editing it")
            return {"kind": "waypoints", "pts": full}, note
    return None, (f"no trackable glue ({tried} Bezier variants + Dubins "
                  f"R {radii[0]:.1f}..{radii[-1]:.2f} all violate clearance "
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
# Public entry
# ----------------------------------------------------------------------

def plan_stages(venue, doc, counts, footprint_r=0.30, track_margin=0.30,
                key_fn=None):
    """Plan stages covering every remaining (family, R) geometry.

    Returns the dict described in the module docstring; never raises on bad
    input (returns ok=False + notes instead) so a ROS host can publish the
    result verbatim.
    """
    notes, unfittable, stages = [], [], []
    corners = (venue or {}).get("corners_wgs84") or []
    if len(corners) < 3:
        return {"ok": False, "stages": [], "unfittable": [],
                "notes": ["venue has no polygon (need >= 3 corners_wgs84)"]}
    cfg = _plan_cfg(doc)
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

    remaining = remaining_geometries(doc, counts, key_fn=key_fn)
    if not remaining:
        return {"ok": True, "stages": [], "unfittable": [],
                "req_clearance_m": req,
                "notes": ["matrix complete — no remaining (family, R) cells"]}

    spacing = 0.25      # search-time sampling; final check is the loader's 0.10
    stage_geo = []      # per assembled stage: exit/entry poses (EN) for transit
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

        placed, glues_en = None, None

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

            for (recipe, pts, end_yaw) in variants:
                for (_sA, xA, yA, hA) in _candidates(
                        pts, poly, excl, req, cfg)[:6]:
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
                        for w in (1.8, 2.4, 3.0, 3.6):
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
        })
        stage_geo.append({"exit_en": placed[-1]["end_en"],
                          "exit_b": placed[-1]["exit_b"],
                          "entry_en": placed[0]["start_en"],
                          "entry_b": placed[0]["h"]})
    if queue:
        for (fam, R, rem) in queue:
            unfittable.append({"family": fam, "R": R,
                               "reason": "planner retry budget exhausted"})

    # One inter-stage transit glue per boundary (C, field design 2026-06-11):
    # the robot finishes stage i at SOME exp end (not statically known), walks
    # the stage's own already-validated loop to the stage EXIT pose (the last
    # experiment's end), then drives this glue to stage i+1's first start pin.
    # Emitted as explicit waypoints (planner-sampled, not hand-editable) on
    # the DESTINATION stage. Failure degrades to the executor's path-join
    # fallback — noted, never fatal.
    for i in range(len(stages) - 1):
        a, b = stage_geo[i], stage_geo[i + 1]
        glue_en, note = _plan_glue(a["exit_en"], a["exit_b"],
                                   b["entry_en"], b["entry_b"],
                                   poly, excl, req, cfg)
        pair = f"{stages[i]['name']} -> {stages[i + 1]['name']}"
        if glue_en is None:
            notes.append(f"no inter-stage glue {pair} ({note}); the executor "
                         "falls back to a reposition path-join at the advance")
            continue
        if note:
            notes.append(f"inter-stage glue {pair}: {note}")
        if glue_en["kind"] == "mids":
            pts_en = _bezier_samples([a["exit_en"]] + list(glue_en["pts"])
                                     + [b["entry_en"]])
        else:
            pts_en = list(glue_en["pts"])
        stages[i + 1]["entry_glue"] = {
            "from_stage": stages[i]["name"],
            "waypoints_wgs84": [
                dict(zip(("lat", "lon"), _en_to_latlon(e, n, lat0, lon0)))
                for (e, n) in pts_en],
            "v_const": float(cfg["glue"]["v_const"]),
            "pos_tol_m": float(cfg["glue"]["pos_tol_m"]),
            "end_heading_deg": round(b["entry_b"], 1),
        }

    return {"ok": bool(stages) and not unfittable,
            "req_clearance_m": req,
            "stages": stages,
            "unfittable": unfittable,
            "notes": notes}
