"""Reposition undershoot: does the stock driver's steering scale explain it?

H_gain : the chassis delivers only ~0.45x the commanded yaw rate (stock limo_base
         sends inner/2.47 and the chassis steers to that value as the bicycle
         angle). Pure pursuit then needs a large look-ahead angle to hold the
         curve -> steady OUTWARD offset on every arc -> near the pin it cuts in
         from outside and the position gate stops it while still rotated
         toward the inside of the turn.
Predictions (each can fail):
  P1 gyro yaw rate / commanded yaw rate on curves ~ the driver-chain model
     (~0.45), not ~1.
  P2 pure pursuit's own steady state on the arc: kappa_path / kappa_cmd ~ the
     same gain (independent of the gyro).
  P3 cross-track offset on arcs is OUTSIDE the turn.
  P4 signed arrival heading error has the SAME sign as the final turn
     (over-rotated toward the inside), not random.
"""
import math, sys
from collections import defaultdict
import numpy as np
from parse_repo import parse

L, T = 0.2, 0.172
INNER_MAX, SCALE = 0.48869, 2.47


def stock_yaw_rate(v, w):
    """yaw rate the chassis produces for cmd (v, w) through stock limo_base,
    given the measured fact that the chassis steers to the raw value."""
    if abs(w) < 1e-9 or abs(v) < 1e-9:
        return 0.0
    r = v / w
    if abs(r) < T / 2:
        r = math.copysign(T / 2 + 0.01, r)
    c = math.atan(L / r)
    a = abs(c)
    inner = math.copysign(math.atan(2 * L * math.sin(a) / (2 * L * math.cos(a) - T * math.sin(a))), c)
    inner = max(-INNER_MAX, min(INNER_MAX, inner))
    raw = inner / SCALE
    return v * math.tan(raw) / L


def wrapd(a):
    return (a + 180.0) % 360.0 - 180.0


def path_kappa(P):
    """signed curvature at each vertex (central difference of heading)."""
    d = np.diff(P, axis=0)
    h = np.unwrap(np.arctan2(d[:, 1], d[:, 0]))
    s = np.hypot(d[:, 0], d[:, 1])
    k = np.zeros(len(P))
    k[1:-1] = np.diff(h) / (0.5 * (s[:-1] + s[1:]))
    return k, h, s


def glue_shape(P):
    k, h, s = path_kappa(P)
    # straight tail: trailing length with |kappa| < 0.1
    tail = 0.0
    for i in range(len(P) - 2, 0, -1):
        if abs(k[i]) > 0.1:
            break
        tail += s[i]
    tail += s[-1]
    curved = np.abs(k) > 0.3
    r_arc = 1.0 / np.median(np.abs(k[curved])) if curved.any() else float("inf")
    # direction of the LAST curved stretch before the tail
    idx = np.where(curved)[0]
    last_dir = int(np.sign(k[idx[-1]])) if len(idx) else 0
    return dict(length=float(s.sum()), turn_deg=float(np.degrees(h[-1] - h[0])), r_arc=r_arc,
                tail=tail, last_dir=last_dir, k=k)


def side_of_path(p, P):
    """signed lateral offset of point p from polyline P (+ = left of travel)."""
    best = (1e9, 0.0, 0)
    for i in range(len(P) - 1):
        a, b = P[i], P[i + 1]
        ab = b - a
        t = np.clip(np.dot(p - a, ab) / max(np.dot(ab, ab), 1e-12), 0, 1)
        q = a + t * ab
        d = np.hypot(*(p - q))
        if d < best[0]:
            cross = ab[0] * (p - a)[1] - ab[1] * (p - a)[0]
            best = (d, math.copysign(d, cross), i)
    return best[1], best[2]


def main(log):
    eps, fit = parse(log)
    print(f"projection lat/lon->local fit: {fit['n_pairs']} pairs, residual p50 "
          f"{fit['res_p50']*100:.1f} cm, max {fit['res_max']*100:.1f} cm")
    drv = [e for e in eps if e["path"] is not None and e["outcome"] in ("arrived", "abort")]
    print(f"episodes: {len(eps)}  arrived {sum(e['outcome']=='arrived' for e in eps)}  "
          f"abort {sum(e['outcome']=='abort' for e in eps)}")

    # ---- glue shapes ----------------------------------------------------------
    shapes = {}
    for e in drv:
        key = (e["n"], e["end_yaw"])
        if key not in shapes:
            shapes[key] = glue_shape(e["path"])
    print("\nglues (n pts, end yaw): length / total turn / arc R / straight tail / last-turn dir")
    for k, g in shapes.items():
        cnt = sum((e["n"], e["end_yaw"]) == k for e in drv)
        print(f"  {k}: x{cnt:2d}  {g['length']:.2f} m  {g['turn_deg']:+6.0f} deg  R {g['r_arc']:.2f} m  "
              f"tail {g['tail']:.2f} m  last dir {g['last_dir']:+d}")

    # ---- P1 gyro gain ---------------------------------------------------------
    rows = []
    for e in drv:
        ctl = e["ctl"]
        if len(ctl) < 4 or len(e["hdg"]) < 2:
            continue
        tc = np.array([c[0] for c in ctl]); kc = np.array([c[7] for c in ctl])
        cmd = e["cmd"]
        tv = np.array([c[0] for c in cmd]) if cmd else np.array([e["t0"]])
        vv = np.array([c[1] for c in cmd]) if cmd else np.array([e["v"]])
        for (t1, h1), (t2, h2) in zip(e["hdg"][:-1], e["hdg"][1:]):
            if t2 - t1 > 3.0 or t1 < e["t0"]:
                continue
            m = (tc >= t1) & (tc <= t2)
            if m.sum() < 4:
                continue
            v = np.array([vv[max(0, np.searchsorted(tv, t, side="right") - 1)] for t in tc[m]])
            w_cmd = v * kc[m]
            if np.mean(v) < 0.15:          # skip the terminal slow-down
                continue
            w_meas = math.radians(wrapd(h2 - h1)) / (t2 - t1)
            w_model = np.mean([stock_yaw_rate(vi, wi) for vi, wi in zip(v, w_cmd)])
            rows.append((np.mean(w_cmd), w_meas, w_model, np.std(w_cmd)))
    R = np.array(rows)
    big = (np.abs(R[:, 0]) > 0.15) & (R[:, 3] < 0.05)      # steady, clearly turning
    print(f"\nP1 gyro: {big.sum()} steady 2-s windows with |w_cmd| > 0.15 rad/s")
    g_meas = R[big, 1] / R[big, 0]
    g_model = R[big, 2] / R[big, 0]
    print(f"   measured / commanded yaw rate : median {np.median(g_meas):.2f}  IQR "
          f"[{np.percentile(g_meas,25):.2f}, {np.percentile(g_meas,75):.2f}]")
    print(f"   stock-driver model / commanded: median {np.median(g_model):.2f}")
    print(f"   |meas - model| median {np.median(np.abs(R[big,1]-R[big,2])):.3f} rad/s vs "
          f"|meas - cmd| median {np.median(np.abs(R[big,1]-R[big,0])):.3f} rad/s")

    # ---- P2 pure-pursuit steady state ----------------------------------------
    ratios = []
    for e in drv:
        g = shapes[(e["n"], e["end_yaw"])]
        k = g["k"]
        for c in e["ctl"]:
            seg, Lla, kap = c[1], c[3], c[7]
            if Lla < 0.59 or seg < 2 or seg > len(k) - 4:
                continue
            win = k[max(1, seg - 2): seg + 4]
            if np.all(np.abs(win) > 0.5) and np.ptp(win) < 0.3 * np.mean(np.abs(win)) and abs(kap) > 0.2:
                if np.sign(kap) == np.sign(np.mean(win)):
                    ratios.append(np.mean(win) / kap)
    ratios = np.array(ratios)
    print(f"\nP2 pure pursuit on constant-curvature arcs: {len(ratios)} control ticks")
    print(f"   kappa_path / kappa_cmd: median {np.median(ratios):.2f}  IQR "
          f"[{np.percentile(ratios,25):.2f}, {np.percentile(ratios,75):.2f}]   (perfect actuation -> 1.0)")

    # ---- P3 offset side on arcs ----------------------------------------------
    out, inn = 0, 0
    offs = []
    for e in drv:
        g = shapes[(e["n"], e["end_yaw"])]
        k = g["k"]
        for (t, x, y) in e["fix"]:
            lat, i = side_of_path(np.array([x, y]), e["path"])
            if 1 <= i < len(k) - 2 and abs(k[i]) > 0.5 and abs(k[i + 1]) > 0.5 and abs(lat) > 0.03:
                outside = np.sign(lat) == -np.sign(k[i])   # left offset on a right turn = outside
                out += outside; inn += (not outside)
                offs.append(abs(lat))
    print(f"\nP3 RTK fixes on arcs (|offset| > 3 cm): outside {out}  inside {inn}  "
          f"(median |offset| {np.median(offs)*100:.0f} cm)")

    # ---- P4 arrival heading sign ---------------------------------------------
    same, opp = 0, 0
    errs = defaultdict(list)
    for e in drv:
        if e["outcome"] != "arrived" or e["err_deg"] is None:
            continue
        g = shapes[(e["n"], e["end_yaw"])]
        errs[(e["n"], e["end_yaw"])].append(e["err_deg"])
        if abs(e["err_deg"]) < 2:
            continue
        if np.sign(e["err_deg"]) == g["last_dir"]:
            same += 1
        else:
            opp += 1
    print(f"\nP4 arrivals with |heading err| >= 2 deg: same sign as last turn {same}, opposite {opp}")
    for k, v in errs.items():
        v = np.array(v)
        print(f"   glue {k}: n={len(v):2d} signed err median {np.median(v):+.1f} deg  "
              f"range [{v.min():+.1f}, {v.max():+.1f}]")

    # ---- aborts ---------------------------------------------------------------
    print("\naborts:")
    for e in eps:
        if e["outcome"] == "abort":
            print("  ", (e["n"], e["end_yaw"]), e["reason"][:140])
    return eps, shapes


if __name__ == "__main__":
    main(sys.argv[1])
