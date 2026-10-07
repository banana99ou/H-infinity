"""Parse reposition_node's [io-dbg] log into per-episode records.

Episode = one goto that reached 'driving'. Per episode:
  path    : goto polyline in the node's local frame (lat/lon -> local via an
            affine fit on the log's own 'IN fix (lat,lon) -> local (x,y)' pairs)
  end_yaw : 'end heading X deg (local)'
  fix     : [(t, x, y)]          (log-throttled ~2 s)
  hdg     : [(t, local_deg)]     (log-throttled ~2 s)
  cmd     : [(t, v, w)]          (log-throttled ~1 s)
  ctl     : [(t, seg, la, L, d_final, xtrack, alpha_deg, kappa, kappa_raw)] (~4 Hz)
  outcome : 'arrived' | 'abort' | 'cut'
  err_m, err_deg (signed, from the first status after arrival)
"""
import json, re, sys
import numpy as np

TS = re.compile(r"^\[(\w+)\] \[(\d+\.\d+)\] \[reposition_node\]: (.*)$")
R_FIX = re.compile(r"\[io-dbg\] IN  fix q=(\d) \(([-\d.]+), ([-\d.]+)\) -> local \(([-\d.]+), ([-\d.]+)\)")
R_FUSED = re.compile(r"\[io-dbg\] IN  fused=([-\d.]+) deg\(E-of-N\) -> heading_est=([-\d.]+) deg\(local\)")
R_CMD = re.compile(r"\[io-dbg\] OUT cmd_vel_raw v=([-\d.]+) w=([-\d.]+)")
R_CTL = re.compile(r"\[io-dbg\] CTL seg=(\d+) la=(\S+) tgt=\(([-\d.]+),([-\d.]+)\) L=([\d.]+) d_final=([\d.]+) "
                   r"xtrack=([\d.]+) alpha=([-\d.]+)deg kappa=([-\d.]+)/([-\d.]+)")
R_GOTO_IN = re.compile(r"\[io-dbg\] IN  goto <- (\{.*\})$")
R_DRIVE = re.compile(r"goto: (\d+) pts, end heading ([-\d.]+) deg \(local\), speed ([\d.]+) m/s, start err ([\d.]+) m -> driving")
R_JOIN = re.compile(r"\[io-dbg\] JOIN -> seg_i=(\d+)/(\d+)")
R_ARR = re.compile(r"arrived: err ([\d.]+) m\. (.*)$")
R_STATUS = re.compile(r"\[io-dbg\] OUT status -> (\{.*\})$")


def parse(path):
    lines = open(path, errors="replace").read().splitlines()
    pairs = []
    eps, cur, last_goto = [], None, None
    for ln in lines:
        m = TS.match(ln)
        if not m:
            continue
        t, msg = float(m.group(2)), m.group(3)
        if (g := R_FIX.search(msg)):
            lat, lon, x, y = map(float, g.group(2, 3, 4, 5))
            pairs.append((lat, lon, x, y))
            if cur is not None and cur["outcome"] is None:
                cur["fix"].append((t, x, y))
            continue
        if (g := R_GOTO_IN.search(msg)):
            try:
                last_goto = json.loads(g.group(1))
            except json.JSONDecodeError:
                last_goto = None
            continue
        if (g := R_DRIVE.search(msg)):
            if cur is not None and cur["outcome"] is None:
                cur["outcome"] = "cut"
            cur = dict(t0=t, n=int(g.group(1)), end_yaw=float(g.group(2)), v=float(g.group(3)),
                       start_err=float(g.group(4)), goto=last_goto, fix=[], hdg=[], cmd=[], ctl=[],
                       join=None, outcome=None, err_m=None, err_deg=None, reason=None, t_end=None)
            eps.append(cur)
            continue
        if cur is None:
            continue
        if cur["outcome"] is None:
            if (g := R_FUSED.search(msg)):
                cur["hdg"].append((t, float(g.group(2))))
            elif (g := R_CMD.search(msg)):
                cur["cmd"].append((t, float(g.group(1)), float(g.group(2))))
            elif (g := R_CTL.search(msg)):
                cur["ctl"].append((t, int(g.group(1)), g.group(2), float(g.group(5)), float(g.group(6)),
                                   float(g.group(7)), float(g.group(8)), float(g.group(9)), float(g.group(10)),
                                   float(g.group(3)), float(g.group(4))))
            elif (g := R_JOIN.search(msg)):
                cur["join"] = int(g.group(1))
            elif (g := R_ARR.search(msg)):
                cur["outcome"], cur["err_m"], cur["reason"], cur["t_end"] = "arrived", float(g.group(1)), g.group(2), t
            elif "ABORT" in msg or "reposition ABORT" in msg:
                cur["outcome"], cur["reason"], cur["t_end"] = "abort", msg, t
        elif cur["outcome"] == "arrived" and cur["err_deg"] is None and (g := R_STATUS.search(msg)):
            s = json.loads(g.group(1))
            if s.get("state") == "arrived" and s.get("err_deg") is not None:
                cur["err_deg"] = float(s["err_deg"])

    # lat/lon -> local affine fit from the node's own projections
    P = np.array(pairs)
    A = np.c_[P[:, 0], P[:, 1], np.ones(len(P))]
    cx, *_ = np.linalg.lstsq(A, P[:, 2], rcond=None)
    cy, *_ = np.linalg.lstsq(A, P[:, 3], rcond=None)
    res = np.hypot(A @ cx - P[:, 2], A @ cy - P[:, 3])
    for e in eps:
        if e["goto"]:
            W = np.array([(w["lat"], w["lon"], 1.0) for w in e["goto"]["waypoints"]])
            e["path"] = np.c_[W @ cx, W @ cy]
        else:
            e["path"] = None
    return eps, dict(n_pairs=len(P), res_p50=float(np.median(res)), res_max=float(res.max()))


if __name__ == "__main__":
    eps, fit = parse(sys.argv[1])
    print("projection fit:", fit)
    from collections import Counter
    print(Counter(e["outcome"] for e in eps))
