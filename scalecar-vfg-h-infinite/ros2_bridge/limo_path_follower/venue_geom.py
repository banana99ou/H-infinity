# -*- coding: utf-8 -*-
"""Pure-geometry venue containment helpers for the operator-authored leg batch.

Shared by ``venue_loader_node`` (authoring-time validation of a Send-to-NUC
payload) and ``run_executor_node`` (pre-motion re-check at Go). No ROS, no I/O —
trivially unit-testable off the robot.

Every move in a leg batch is an EXPLICIT curve, so it can be checked against the
venue polygon BEFORE the robot moves:
  - a ``recipe`` curve is the follower's analytic path (step/uturn), sampled and
    placed at its ``start_pose`` exactly as the follower lays it down;
  - a ``reposition`` curve is the operator-drawn waypoint polyline.
Both are required to keep a clearance of
``safety_margin_m + robot_footprint + path_tracking_margin`` inside the polygon.

Frame: a local equirectangular East/North metric frame about the first venue
corner. Internally consistent for clearance geometry (this matches the frame the
experiment sequencer's own containment gate uses); it does NOT need to coincide
with path_overlay's anchor-rotated frame — the waypoints, recipe placement, and
polygon are all projected the same way, so distances are correct.
"""
import math


def latlon_to_en(lat, lon, lat0, lon0):
    """Equirectangular lat/lon -> local (East, North) metres about (lat0, lon0)."""
    mlat = 111320.0
    mlon = 111320.0 * math.cos(math.radians(lat0))
    return ((lon - lon0) * mlon, (lat - lat0) * mlat)


def pt_in_poly(pt, poly):
    """Ray-cast point-in-polygon. poly is a list of (x, y)."""
    x, y = pt
    inside = False
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        if ((y1 > y) != (y2 > y)) and \
                (x < (x2 - x1) * (y - y1) / (y2 - y1) + x1):
            inside = not inside
    return inside


def dist_to_seg(p, a, b):
    """Distance from point p to segment a->b (all (x, y))."""
    px, py = p
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    if dx == 0.0 and dy == 0.0:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def clearance(pt, poly):
    """Signed clearance to the polygon boundary: + inside, - outside (metres)."""
    d = min(dist_to_seg(pt, poly[i], poly[(i + 1) % len(poly)])
            for i in range(len(poly)))
    return d if pt_in_poly(pt, poly) else -d


def poly_en(corners_wgs84, lat0, lon0):
    """Project a venue corner list [{lat,lon},...] into the EN frame."""
    return [latlon_to_en(c["lat"], c["lon"], lat0, lon0) for c in corners_wgs84]


def recipe_points_en(recipe, start_pose, lat0, lon0, spacing_m=0.10):
    """Sample an analytic recipe and place it at start_pose, in EN metres.

    Mirrors path_follower.build_path_from_recipe + the path_overlay placement so
    the checked geometry is the exact curve the robot drives. ``start_pose`` is
    {lat, lon, heading_deg} (compass bearing E-of-N). Returns [(E, N, s), ...].
    Returns [] if StepCurvaturePath is unavailable or the type is not geometry-
    checkable here (e.g. slalom).
    """
    try:
        from vfg_pathfollowing.paths.step_curvature import StepCurvaturePath
    except Exception:
        return []
    params = recipe.get("params", {}) or {}
    ptype = str(recipe.get("type", "")).lower()

    def _f(k, d):
        return float(params.get(k, d))

    def _i(k, d):
        return int(params.get(k, d))

    if ptype == "step":
        path = StepCurvaturePath(L1=_f("L1", 5.0), R=_f("R", 0.5),
                                 theta_arc=_f("theta_arc", math.pi / 2),
                                 L2=_f("L2", 5.0), direction=_i("direction", 1))
    elif ptype == "uturn":
        path = StepCurvaturePath(L1=_f("L1", 1.0), R=_f("R", 0.5),
                                 theta_arc=math.pi, L2=_f("L2", 1.0),
                                 direction=_i("direction", 1))
    else:
        return []  # slalom / unknown: not geometry-checked here
    total = float(path.total_length)
    h = math.radians(float(start_pose.get("heading_deg", 0.0)))
    # local +x (forward) at compass bearing h E-of-N -> (E,N)=(sin h, cos h);
    # +y (left) is +90 deg CCW -> (-cos h, sin h). Matches path_overlay/follower.
    fE, fN = math.sin(h), math.cos(h)
    lE, lN = -math.cos(h), math.sin(h)
    e0, n0 = latlon_to_en(start_pose["lat"], start_pose["lon"], lat0, lon0)
    n = max(2, int(total / max(0.01, spacing_m)))
    out = []
    for i in range(n + 1):
        s = total * i / n
        p = path.position(s)
        px, py = float(p[0]), float(p[1])
        out.append((e0 + px * fE + py * lE, n0 + px * fN + py * lN, s))
    return out


def check_legs_containment(legs, venue, footprint_r, track_margin, spacing_m=0.25):
    """Verify every curve of every leg fits inside the venue polygon minus
    margins AND clear of every exclusion circle. Returns (ok: bool, report: str).

    Required clearance = safety_margin_m + footprint_r + track_margin, both to
    the polygon boundary and to each exclusion circle's edge. This is the ONLY
    layer that checks RECIPE curves against exclusions: the follower drives the
    recipe on dead-reckoned odom with no area guard, and the geofence watchdog
    checks the outer polygon only (reposition's own exclusion guard is dead
    during a recipe). Recipe curves are sampled+placed at their start_pose;
    reposition curves check each waypoint and the segments between them.
    """
    corners = (venue or {}).get("corners_wgs84") or []
    if len(corners) < 3:
        return True, "no venue polygon (corners_wgs84) — containment UNCHECKED"
    lat0, lon0 = corners[0]["lat"], corners[0]["lon"]
    poly = poly_en(corners, lat0, lon0)
    margin = float(venue.get("safety_margin_m", 0.0))
    req = margin + float(footprint_r) + float(track_margin)

    viol = []      # (label, deficit_m, clearance_m)
    worst = None

    # Exclusion circles in the same EN frame. Fail CLOSED on a malformed entry:
    # an exclusion we cannot parse must refuse the run, not silently vanish.
    excl = []      # (cE, cN, radius_m)
    for xi, ex in enumerate((venue or {}).get("exclusions") or []):
        kind = str((ex or {}).get("kind", "")).lower()
        if kind != "circle":
            viol.append((f"exclusion {xi}: unsupported kind '{kind}' — "
                         "cannot verify clearance", req, -req))
            continue
        try:
            ce, cn = latlon_to_en(float(ex["lat"]), float(ex["lon"]), lat0, lon0)
            excl.append((ce, cn, float(ex["radius_m"])))
        except (KeyError, TypeError, ValueError):
            viol.append((f"exclusion {xi}: malformed (lat/lon/radius_m)",
                         req, -req))

    def _check_pt(label, e, nn):
        nonlocal worst
        cl = clearance((e, nn), poly)
        for (ce, cn, r) in excl:
            cl = min(cl, math.hypot(e - ce, nn - cn) - r)
        worst = cl if worst is None else min(worst, cl)
        if cl < req:
            viol.append((label, req - cl, cl))

    for li, leg in enumerate(legs or []):
        leg_id = leg.get("id", f"leg_{li}")
        for ci, curve in enumerate(leg.get("curves", []) or []):
            kind = str(curve.get("kind", "")).lower()
            name = curve.get("name", f"curve_{ci}")
            if kind == "recipe":
                sp = curve.get("start_pose")
                if not sp:
                    viol.append((f"{leg_id}/{name}: recipe has no start_pose", req, -req))
                    continue
                pts = recipe_points_en(curve.get("recipe", {}) or {}, sp,
                                       lat0, lon0)
                if not pts:
                    # Geometry not checkable (import/type) — flag, do not silently pass.
                    viol.append((f"{leg_id}/{name}: recipe geometry UNVERIFIED "
                                 "(StepCurvaturePath unavailable or non-step type)",
                                 0.0, 0.0))
                    continue
                for (e, nn, s) in pts:
                    _check_pt(f"{leg_id}/{name} @s={s:.1f}m", e, nn)
            elif kind == "reposition":
                wps = curve.get("waypoints_wgs84") or []
                if len(wps) < 1:
                    viol.append((f"{leg_id}/{name}: reposition has no waypoints",
                                 req, -req))
                    continue
                en = [latlon_to_en(w["lat"], w["lon"], lat0, lon0) for w in wps]
                for k, (e, nn) in enumerate(en):
                    _check_pt(f"{leg_id}/{name} wp{k}", e, nn)
                # Sample the segments between waypoints (a sparse polyline could
                # cut a corner of the polygon between two in-bounds vertices).
                for k in range(len(en) - 1):
                    a, b = en[k], en[k + 1]
                    seg_len = math.hypot(b[0] - a[0], b[1] - a[1])
                    n = max(1, int(seg_len / max(0.05, spacing_m)))
                    for j in range(1, n):
                        t = j / n
                        _check_pt(f"{leg_id}/{name} seg{k}",
                                  a[0] + (b[0] - a[0]) * t,
                                  a[1] + (b[1] - a[1]) * t)
            else:
                viol.append((f"{leg_id}/{name}: unknown curve kind '{kind}'",
                             req, -req))

    head = (f"margin={margin:.2f} footprint={float(footprint_r):.2f} "
            f"track={float(track_margin):.2f} -> needs {req:.2f}m clearance")
    if viol:
        viol.sort(key=lambda v: -v[1])
        lines = [head,
                 f"{len(viol)} containment violation(s); worst {viol[0][1]:.2f}m short:"]
        lines += [f"  - {lbl}: {cl:+.2f}m clearance ({d:.2f}m short)"
                  for (lbl, d, cl) in viol[:8]]
        if len(viol) > 8:
            lines.append(f"  ... +{len(viol) - 8} more")
        return False, "\n".join(lines)
    tail = f"; OK (tightest clearance {worst:.2f}m)" if worst is not None else "; OK"
    return True, head + tail
