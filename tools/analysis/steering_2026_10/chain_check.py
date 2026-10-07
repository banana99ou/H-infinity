"""Is limo_driver.cpp's steering conversion the thing that loses 55% of the turn?

For each moving sample of today's scored legs (step R0.4 + slalom R1), follow the
steering value through every stage and ask where the turn is lost:

  S1 /cmd_vel_raw -> /cmd_vel       estop_cli pass-through (should be identical)
  S2 /cmd_vel -> raw_sent           limo_driver.cpp formula: inner(atan(L/r)),
                                    clamp 28 deg, / 2.47  (what goes on the wire)
  S3 raw_fb                         the chassis' own steering report, recovered
                                    from /wheel/odom (driver: inner = raw_fb*2.47)
  S4 delta_imu = atan(L*w_imu/v)    the bicycle angle the robot actually drove
  S5 delta_rtk                      same from RTK course rate (no IMU involved)

If the cpp's /2.47 is the loss:  raw_fb ~ raw_sent  and  delta_imu ~ raw_fb
(chassis steers to the value it got, as a bicycle angle), while the driver's
own /wheel/odom yaw rate ~ the commanded one (it believes the turn happened).
If the chassis instead honoured the driver's intent (steer the inner wheel to
raw*2.47): delta_imu ~ central(raw*2.47) ~ 2.2*raw, and imu yaw ~ commanded.
"""
import glob, math, sys
import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore
TS = get_typestore(Stores.ROS2_HUMBLE)
from pathlib import Path

L, T, INNER_MAX, SCALE = 0.2, 0.172, 0.48869, 2.47


def c2i(c):
    a = abs(c)
    return math.copysign(math.atan(2 * L * math.sin(a) / (2 * L * math.cos(a) - T * math.sin(a))), c)


def i2c(i):
    if abs(i) < 1e-9:
        return 0.0
    r = L / math.tan(abs(i)) + T / 2
    return math.copysign(math.atan(L / r), i)


def raw_sent(v, w):
    if abs(w) < 1e-9 or abs(v) < 1e-9:
        return 0.0
    r = v / w
    if abs(r) < T / 2:
        r = math.copysign(T / 2 + 0.01, r)
    inner = max(-INNER_MAX, min(INNER_MAX, c2i(math.atan(L / r))))
    return inner / SCALE


def series(reader, topic, f):
    cons = [c for c in reader.connections if c.topic == topic]
    out = []
    for c, t, raw in reader.messages(connections=cons):
        out.append((t * 1e-9, *f(reader.deserialize(raw, c.msgtype))))
    return np.array(out) if out else np.zeros((0, 3))


def hold(ts, arr, t):
    i = np.searchsorted(arr[:, 0], t, side="right") - 1
    return np.where(i >= 0, arr[np.clip(i, 0, None), 1:].T, np.nan).T


M = 111320.0
rows = []
same_cmd = []
for d in sorted(glob.glob(sys.argv[1] + "/26_1006_*")):
    with AnyReader([Path(d)], default_typestore=TS) as r:
        cmd = series(r, "/cmd_vel", lambda m: (m.linear.x, m.angular.z))
        cmdr = series(r, "/cmd_vel_raw", lambda m: (m.linear.x, m.angular.z))
        odo = series(r, "/wheel/odom", lambda m: (math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y),
                                                    m.twist.twist.angular.z))
        imu = series(r, "/imu", lambda m: (m.angular_velocity.z, 0.0))
        fix = series(r, "/gps_rtk_f9p_helical/gps/fix", lambda m: (m.latitude, m.longitude))
    if len(cmd) < 5 or len(odo) < 20:
        continue
    # S1: every /cmd_vel matches the latest /cmd_vel_raw
    if len(cmdr):
        prev = hold(None, cmdr, cmd[:, 0] - 1e-4)
        ok = np.isfinite(prev[:, 0])
        same_cmd.append(np.max(np.abs(prev[ok] - cmd[ok, 1:])) if ok.any() else np.nan)
    # RTK course rate (smoothed over 0.6 s), as a gyro-free yaw rate
    rtk_w = None
    if len(fix) > 10:
        lat0, lon0 = fix[0, 1], fix[0, 2]
        E = (fix[:, 2] - lon0) * M * math.cos(math.radians(lat0)); N = (fix[:, 1] - lat0) * M
        tt = fix[:, 0]
        k = 3
        if len(tt) > 2 * k + 2:
            hd = np.unwrap(np.arctan2(N[2 * k:] - N[:-2 * k], E[2 * k:] - E[:-2 * k]))
            th = tt[k:-k]
            rtk_w = np.c_[th[1:-1], (hd[2:] - hd[:-2]) / (th[2:] - th[:-2])]
            sp = np.hypot(np.diff(E), np.diff(N)) / np.diff(tt)
    for t, v_fb, w_odo in odo:
        if v_fb < 0.3:
            continue
        vc, wc = hold(None, cmd, np.array([t]))[0]
        wi = hold(None, imu, np.array([t - 0.0]))[0][0]
        if not np.isfinite(vc) or not np.isfinite(wi):
            continue
        rs = raw_sent(vc, wc)
        # invert the driver's odometry: |wz| = v / (L/tan|inner| + T/2)
        rr = v_fb / abs(w_odo) - T / 2 if abs(w_odo) > 1e-6 else float("inf")
        inner_fb = math.copysign(math.atan(L / rr), w_odo) if rr > 0 and math.isfinite(rr) else 0.0
        raw_fb = inner_fb / SCALE
        d_imu = math.atan(L * wi / v_fb)
        d_rtk = np.nan
        if rtk_w is not None and len(rtk_w):
            j = np.argmin(np.abs(rtk_w[:, 0] - t))
            if abs(rtk_w[j, 0] - t) < 0.15:
                d_rtk = math.atan(L * rtk_w[j, 1] / v_fb)
        rows.append((wc, rs, raw_fb, d_imu, d_rtk, w_odo, wi, vc))

R = np.array(rows)
print(f"bags: {len(same_cmd)}   moving samples (v >= 0.3 m/s): {len(R)}")
print(f"S1 /cmd_vel vs latest /cmd_vel_raw: max |diff| per bag, worst = {np.nanmax(same_cmd):.4f}")


def fit(x, y, mask, name):
    x, y = x[mask], y[mask]
    s = float(np.dot(x, y) / np.dot(x, x))
    res = np.median(np.abs(y - s * x))
    print(f"   {name:58s} slope {s:5.2f}   median |resid| {res:.4f}   n={mask.sum()}")
    return s


turning = np.abs(R[:, 1]) > 0.03
steady = turning & (np.abs(R[:, 1]) < 0.19)          # below the 28-deg clamp
print("\nS2->S3 does the chassis report what it was sent?")
fit(R[:, 1], R[:, 2], steady, "raw_fb vs raw_sent")
print("S3->S4 what angle does it actually drive, given its report?")
fit(R[:, 2], R[:, 3], steady, "delta_imu vs raw_fb        (cpp loss -> 1.0; intended -> ~2.2)")
rt = steady & np.isfinite(R[:, 4])
fit(R[:, 2], R[:, 4], rt, "delta_rtk vs raw_fb        (no IMU)")
print("whole chain, yaw rate:")
fit(R[:, 0], R[:, 6], steady, "IMU yaw rate vs /cmd_vel angular.z")
fit(R[:, 0], R[:, 5], steady, "driver's /wheel/odom yaw rate vs /cmd_vel angular.z")
sat = np.abs(R[:, 1]) > 0.197
print(f"\nat the 28-deg clamp (raw_sent = 0.198, n={sat.sum()}): raw_fb median {np.median(np.abs(R[sat,2])):.3f}, "
      f"delta_imu median {np.median(np.abs(R[sat,3])):.3f} rad -> R = {L/math.tan(np.median(np.abs(R[sat,3]))):.2f} m")
