"""Counterfactual: replay every logged reposition with the node's REAL control
code (reposition_node._control_cb, imported with ROS stubbed out) on a
kinematic bicycle, under three actuation models:

  stock   : today's limo_base (inner angle / 2.47, chassis steers to that value)
  ideal   : the chassis gets the bicycle angle the node asked for (driver patch;
            mechanical limit 0.408 rad central)
  inverse : stock driver, but reposition pre-distorts omega so the chassis lands
            on the asked angle (reposition-only workaround; full lock R ~1.0 m)

Same start pose (the node's JOIN line), same polyline, same goto speed.
Impossibility check: 'stock' must reproduce the LOG (sign + size of arrival
heading error, aborts on the R=1.0 glue). If it doesn't, the model is wrong
and the counterfactuals mean nothing.
"""
import math, os, re, sys, types
import numpy as np

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
sys.path.insert(0, REPO + "/scalecar-vfg-h-infinite/ros2_bridge")


# ---- stub ROS so the node module imports on a laptop ------------------------
def _stub(name, **attrs):
    m = types.ModuleType(name)
    m.__dict__.update(attrs)
    sys.modules[name] = m
    return m


class _Any:
    def __init__(self, *a, **k):
        pass

    def __getattr__(self, k):
        return _Any()

    def __call__(self, *a, **k):
        return _Any()


_stub("rclpy", init=lambda *a, **k: None, shutdown=lambda *a, **k: None, spin=lambda *a, **k: None)
_stub("rclpy.node", Node=object)
_stub("rclpy.qos", QoSProfile=_Any, QoSDurabilityPolicy=_Any(), QoSReliabilityPolicy=_Any(),
      QoSHistoryPolicy=_Any())
for mod, names in {"sensor_msgs": [], "sensor_msgs.msg": ["NavSatFix"], "geometry_msgs": [],
                   "geometry_msgs.msg": ["Twist"], "std_msgs": [], "std_msgs.msg": ["String", "Float64"]}.items():
    _stub(mod, **{n: _Any for n in names})

from limo_path_follower import reposition_node as rn   # noqa: E402
from parse_repo import parse                          # noqa: E402

L, T = 0.2, 0.172
INNER_MAX, SCALE = 0.48869, 2.47
CENTRAL_MECH_MAX = 0.408       # stock inner 28 deg -> central


def central_to_inner(c):
    a = abs(c)
    return math.copysign(math.atan(2 * L * math.sin(a) / (2 * L * math.cos(a) - T * math.sin(a))), c)


def inner_to_central(i):
    r = L / math.tan(abs(i)) + T / 2
    return math.copysign(math.atan(L / r), i)


def stock_delta(v, w):
    if abs(w) < 1e-9 or abs(v) < 1e-9:
        return 0.0
    r = v / w
    if abs(r) < T / 2:
        r = math.copysign(T / 2 + 0.01, r)
    inner = max(-INNER_MAX, min(INNER_MAX, central_to_inner(math.atan(L / r))))
    return inner / SCALE


def ideal_delta(v, w):
    if abs(v) < 1e-9:
        return 0.0
    return max(-CENTRAL_MECH_MAX, min(CENTRAL_MECH_MAX, math.atan(L * w / v)))


def inverse_cmd(v, w):
    """omega to SEND through the stock driver so the chassis steers to atan(L w / v)."""
    if abs(v) < 1e-9 or abs(w) < 1e-9:
        return w
    want = math.atan(L * w / v)                       # bicycle angle the node asked for
    inner_cmd = max(-0.999 * INNER_MAX, min(0.999 * INNER_MAX, want * SCALE))
    return v * math.tan(inner_to_central(inner_cmd)) / L


class _Time:
    def __init__(self, ns):
        self.nanoseconds = ns

    def __sub__(self, o):
        return _Time(self.nanoseconds - o.nanoseconds)


class _Clock:
    def __init__(self):
        self.t = 0.0

    def now(self):
        return _Time(int(self.t * 1e9))


class _Log:
    def info(self, *a, **k):
        pass

    warn = warning = error = info


def make_node(path, end_yaw_deg, speed):
    n = rn.RepositionNode.__new__(rn.RepositionNode)
    defaults = dict(pos_tol=0.15, pos_tol_slack=0.15, pos_tol_float=0.40,
                    head_tol_fixed=math.radians(5), head_tol_float=math.radians(10),
                    min_speed=0.08, slowdown=0.50, lookahead=0.60, la_taper=0.60, la_min=0.25,
                    kappa_max=1 / 0.37, corridor=2.0, acquire_radius=1.0,
                    infeasible=math.radians(100), rtk_timeout=1.0)
    for k, v in defaults.items():
        setattr(n, "_" + k, v)
    n._la_arc_margin = 0.5 * n._lookahead
    n._rtk_quality = 4
    n._state = "driving"
    n._reason = ""
    n._waypoints = [tuple(p) for p in path]
    n._seg_i = 0
    n._la_branch = "-"
    n._la_eff = n._lookahead
    n._la_tgt_xy = None
    n._end_yaw = math.radians(end_yaw_deg)
    n._speed = speed
    n._acquired = False
    n._feasible_checked = False
    n._min_dfinal = None
    n._clock = _Clock()
    n.get_clock = lambda: n._clock
    n.get_logger = lambda: _Log()
    n._fused_is_fresh = lambda: True
    n._live_pose_safe = lambda: True
    n._publish_status = lambda *a, **k: None
    n._out = (0.0, 0.0)

    def _drive(v, w):
        n._out = (v, w)

    def _zero():
        n._out = (0.0, 0.0)

    def _abort(reason):
        n._state = "aborted"
        n._reason = reason
        n._out = (0.0, 0.0)

    n._drive, n._zero_cmd, n._abort = _drive, _zero, _abort
    return n


def run(path, end_yaw_deg, speed, x0, y0, psi0_deg, model, tau=0.2, dt=0.05, t_max=120.0):
    n = make_node(path, end_yaw_deg, speed)
    x, y, psi, delta = x0, y0, math.radians(psi0_deg), 0.0
    t = 0.0
    while t < t_max:
        n._clock.t = t
        n._fix_xy, n._fix_stamp, n._heading_est = (x, y), n._clock.now(), psi
        n._control_cb()
        if n._state != "driving":
            break
        v, w = n._out
        if model == "stock":
            d_tgt = stock_delta(v, w)
        elif model == "ideal":
            d_tgt = ideal_delta(v, w)
        elif model == "inverse":
            d_tgt = stock_delta(v, inverse_cmd(v, w))
        else:
            raise ValueError(model)
        sub = 5
        for _ in range(sub):
            h = dt / sub
            delta += (d_tgt - delta) * (h / tau if tau > 0 else 1.0)
            x += v * math.cos(psi) * h
            y += v * math.sin(psi) * h
            psi += v * math.tan(delta) / L * h
        t += dt
    err = math.degrees(rn._wrap(psi - math.radians(end_yaw_deg)))
    return n._state, err, n._reason


R_JOINR = re.compile(r"\[(\d+\.\d+)\] \[reposition_node\]: \[io-dbg\] JOIN robot=\(([-\d.]+),([-\d.]+)\) hdg=([-\d.]+)deg")


def main(log, tau=0.2):
    eps, _ = parse(log)
    joins = [(float(m.group(1)), float(m.group(2)), float(m.group(3)), float(m.group(4)))
             for m in map(R_JOINR.search, open(log, errors="replace")) if m]
    rows = []
    for e in eps:
        if e["path"] is None or e["outcome"] not in ("arrived", "abort"):
            continue
        j = min(joins, key=lambda r: abs(r[0] - e["t0"]))
        if abs(j[0] - e["t0"]) > 1.0:
            continue
        out = {}
        for model in ("stock", "ideal", "inverse"):
            out[model] = run(e["path"], e["end_yaw"], e["v"], j[1], j[2], j[3], model, tau=tau)
        rows.append((e, out))

    def fmt(state, err):
        return f"{'ARR' if state == 'arrived' else state[:5]:>5s} {err:+6.1f}"

    groups = {}
    for e, out in rows:
        groups.setdefault((e["n"], e["end_yaw"]), []).append((e, out))
    print(f"steering lag tau = {tau:.2f} s")
    print(f"{'glue':>14s} {'n':>3s} | {'LOG':^22s} | {'sim stock':^22s} | {'sim ideal':^22s} | {'sim inverse':^22s}")
    for k, lst in groups.items():
        def summ(get):
            st = [get(e, o)[0] for e, o in lst]
            er = np.array([get(e, o)[1] for e, o in lst if get(e, o)[0] == "arrived"])
            arr = sum(s == "arrived" for s in st)
            return f"arr {arr:2d}/{len(st):2d} err {np.median(er):+5.1f}" if len(er) else f"arr {arr:2d}/{len(st):2d} err   n/a"
        log_ = summ(lambda e, o: (e["outcome"], e["err_deg"] if e["err_deg"] is not None else float("nan")))
        print(f"{str(k):>14s} {len(lst):3d} | {log_:^22s} | "
              + " | ".join(f"{summ(lambda e, o, m=m: o[m][:2]):^22s}" for m in ("stock", "ideal", "inverse")))
    # per-episode agreement of stock-sim vs log on arrivals
    pairs = [(e["err_deg"], o["stock"][1]) for e, o in rows
             if e["outcome"] == "arrived" and o["stock"][0] == "arrived" and e["err_deg"] is not None]
    if pairs:
        p = np.array(pairs)
        print(f"stock-sim vs log, per arrival: median |diff| {np.median(np.abs(p[:,0]-p[:,1])):.1f} deg (n={len(p)})")
    return rows


if __name__ == "__main__":
    for tau in (0.0, 0.2, 0.4):
        main(sys.argv[1], tau)
        print()
