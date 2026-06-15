#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Synthesize a tiny rosbag2 + sidecar for one leg, for laptop pipeline tests.

No robot, no ROS sourcing: writes a rosbag2 with the `rosbags` lib containing
the recorded topic set (the T7/D1 contract) for a known `step` recipe, plus a
paired ``<bag>.sidecar.json`` (the T7/D3 contract). The synthetic robot drives
the *exact* analytic StepCurvaturePath, so run_eval's RTK-truth error against
the rebuilt analytic reference should be ~0 — a clean correctness check for the
whole manifest -> qc -> run_eval -> aggregate -> export pipeline.

Usage::

    python3 tools/analysis/tests/make_fixture.py [out_dir] \
        [--rtk-bad]   # write quality=1 (NO-FIX-grade) status -> qc should FAIL it

Returns the bag directory path on stdout.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
from datetime import datetime, timezone

import numpy as np

_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
_VFG_ROOT = os.path.join(_REPO_ROOT, "scalecar-vfg-h-infinite")
_PATHGEN = os.path.join(_REPO_ROOT, "tools", "path_gen")
for _p in (_VFG_ROOT, _PATHGEN):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from vfg_pathfollowing.paths.step_curvature import StepCurvaturePath  # noqa: E402
from vfg_pathfollowing.paths.slalom import SlalomPath  # noqa: E402
import path_overlay as po  # noqa: E402

from rosbags.rosbag2 import Writer  # noqa: E402
from rosbags.typesys import Stores, get_typestore  # noqa: E402

TS = get_typestore(Stores.ROS2_HUMBLE)

Odometry = TS.types["nav_msgs/msg/Odometry"]
NavSatFix = TS.types["sensor_msgs/msg/NavSatFix"]
NavSatStatus = TS.types["sensor_msgs/msg/NavSatStatus"]
Twist = TS.types["geometry_msgs/msg/Twist"]
String = TS.types["std_msgs/msg/String"]
Bool = TS.types["std_msgs/msg/Bool"]
Float32 = TS.types["std_msgs/msg/Float32"]
Float64 = TS.types["std_msgs/msg/Float64"]
Float32MultiArray = TS.types["std_msgs/msg/Float32MultiArray"]
MultiArrayLayout = TS.types["std_msgs/msg/MultiArrayLayout"]
Header = TS.types["std_msgs/msg/Header"]
Time = TS.types["builtin_interfaces/msg/Time"]
Point = TS.types["geometry_msgs/msg/Point"]
Quaternion = TS.types["geometry_msgs/msg/Quaternion"]
Pose = TS.types["geometry_msgs/msg/Pose"]
PoseWithCovariance = TS.types["geometry_msgs/msg/PoseWithCovariance"]
Vector3 = TS.types["geometry_msgs/msg/Vector3"]
TwistWithCovariance = TS.types["geometry_msgs/msg/TwistWithCovariance"]


def _stamp(t_s):
    return Time(sec=int(t_s), nanosec=int((t_s - int(t_s)) * 1e9))


def _header(t_s, frame=""):
    return Header(stamp=_stamp(t_s), frame_id=frame)


def _quat_yaw(yaw):
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


def _odom(t_s, x, y, yaw, v):
    pose = Pose(position=Point(x=float(x), y=float(y), z=0.0),
                orientation=_quat_yaw(yaw))
    twist = Twist(linear=Vector3(x=float(v), y=0.0, z=0.0),
                  angular=Vector3(x=0.0, y=0.0, z=0.0))
    return Odometry(
        header=_header(t_s, "odom"),
        child_frame_id="base_link",
        pose=PoseWithCovariance(pose=pose, covariance=np.zeros(36, dtype=np.float64)),
        twist=TwistWithCovariance(twist=twist, covariance=np.zeros(36, dtype=np.float64)),
    )


def _navsatfix(t_s, lat, lon, quality):
    # status mirrors GPS-RTK_ROS2_pub_node mapping (q4/5 -> GBAS_FIX=2).
    if quality == 0:
        st = -1  # STATUS_NO_FIX
    elif quality in (4, 5):
        st = 2   # STATUS_GBAS_FIX
    else:
        st = 0   # STATUS_FIX
    return NavSatFix(
        header=_header(t_s, "gps"),
        status=NavSatStatus(status=np.int8(st), service=np.uint16(1)),
        latitude=float(lat), longitude=float(lon), altitude=50.0,
        position_covariance=np.zeros(9, dtype=np.float64),
        position_covariance_type=np.uint8(0),
    )


def _rtk_status_str(quality, lat, lon):
    desc = {0: "NO FIX", 1: "GPS", 4: "RTK FIXED", 5: "RTK FLOAT"}.get(quality, "GPS")
    return (f"FIX: {desc} (quality={quality}, sats=20, HDOP=0.80, rate=5.0Hz) | "
            f"Lat={lat:.8f}, Lon={lon:.8f} | RTCM: ACTIVE (bytes=1000, "
            f"fwd_age=0.2s, net_age=0.1s)")


def _status_array(x, y, yaw, v, s_star, total_len, kappa, rho, e_psi, delta_cmd):
    data = np.array([x, y, yaw, v, s_star, total_len, kappa, rho, e_psi,
                     delta_cmd, 1.0], dtype=np.float32)
    return Float32MultiArray(
        layout=MultiArrayLayout(dim=[], data_offset=np.uint32(0)),
        data=data,
    )


def make(out_dir, rtk_bad=False, controller="lpv-hinf", R=0.5, v=0.5,
         rep=0, leg="AtoB", family="step", err_amp=None,
         estop_fired=False, odom_gap=False, missing_topic=None,
         sidecar_pass_mismatch=False):
    # Controller/R-dependent lateral tracking error so the aggregate stats have
    # real signal: PID worse than LPV, both worse as R shrinks (mimics the sim
    # crossover). The synthetic drive follows the path offset by this error.
    # err_amp=0.0 (--clean) drives exactly on-path -> RTK-truth error ~0, a true
    # projection round-trip check.
    if err_amp is None:
        ctrl_factor = 0.4 if controller.startswith("lpv") else 1.0
        err_amp = 0.06 * ctrl_factor * (0.5 / max(R, 1e-3))  # m, along normal

    if family == "slalom":
        recipe = {"type": "slalom",
                  "params": {"R": R, "theta_arc": math.pi / 2, "L1": 2.0,
                             "L_mid": 1.0, "n_arcs": 2, "L_end": 2.0}}
        path = SlalomPath(R=R, theta_arc=math.pi / 2, L1=2.0, L_mid=1.0,
                          n_arcs=2, L_end=2.0)
    else:
        recipe = {"type": "step",
                  "params": {"L1": 2.0, "R": R, "theta_arc": math.pi / 2,
                             "L2": 2.0, "direction": 1}}
        path = StepCurvaturePath(L1=2.0, R=R, theta_arc=math.pi / 2, L2=2.0,
                                 direction=1)
    rate = 20.0
    dt = 1.0 / rate
    total = path.total_length
    T = total / v
    n = int(T * rate)
    anchor = po.Anchor(po.LAT0, po.LON0, po.BEARING_DEG)

    t0 = 1_700_000_000.0  # fixed epoch base
    quality = 1 if rtk_bad else 4

    os.makedirs(os.path.dirname(out_dir) or ".", exist_ok=True)
    if os.path.exists(out_dir):
        import shutil
        shutil.rmtree(out_dir)

    n_fixed = 0
    n_rtk = 0
    missing = {missing_topic} if missing_topic else set()
    with Writer(out_dir, version=8) as w:
        def add(topic, msgtype):
            if topic in missing:
                return None
            return w.add_connection(topic, msgtype, typestore=TS)

        c_odom = add("/wheel/odom", Odometry.__msgtype__)
        c_odomz = add("/wheel/odom_zeroed", Odometry.__msgtype__)
        c_fix = add("/gps_rtk_f9p_helical/gps/fix", NavSatFix.__msgtype__)
        c_rtks = add("/gps_rtk_f9p_helical/gps/rtk_status", String.__msgtype__)
        c_pix = add("/pixhawk/global_position/raw/fix", NavSatFix.__msgtype__)
        c_head = add("/heading/fused", Float64.__msgtype__)
        c_stat = add("/path_follower/status", Float32MultiArray.__msgtype__)
        c_tim = add("/path_follower/timing", Float32.__msgtype__)
        c_done = add("/path_follower/done", Bool.__msgtype__)
        c_cmd = add("/cmd_vel", Twist.__msgtype__)
        c_cmdr = add("/cmd_vel_raw", Twist.__msgtype__)
        c_estop = add("/estop", Bool.__msgtype__)

        rng = np.random.default_rng(0)
        for i in range(n):
            t = t0 + i * dt
            tns = int(t * 1e9)
            s = min(v * i * dt, total)
            p = path.position(s)
            yaw = path.heading(s)
            kappa = path.curvature(s)
            nrm = path.normal(s)
            # Driven trajectory = path offset along the normal by the
            # controller/R-dependent tracking error (ramped in over the arc).
            ramp = math.sin(math.pi * min(s / total, 1.0))
            drive = p + err_amp * ramp * nrm
            # odom: driven pose + tiny noise
            ox = drive[0] + rng.normal(0, 0.005)
            oy = drive[1] + rng.normal(0, 0.005)
            oyaw = yaw + err_amp * ramp * kappa + rng.normal(0, 0.002)
            in_gap = odom_gap and (n // 3 <= i <= n // 3 + int(rate * 1.0))
            if c_odom is not None and not in_gap:
                w.write(c_odom, tns, TS.serialize_cdr(_odom(t, ox, oy, oyaw, v),
                                                      Odometry.__msgtype__))
            # Re-anchored stream the controller actually tracked. In this
            # fixture the driven pose is already origin-anchored, so the zeroed
            # stream mirrors odom; run_eval prefers this for odom-belief.
            if c_odomz is not None:
                w.write(c_odomz, tns, TS.serialize_cdr(_odom(t, ox, oy, oyaw, v),
                                                       Odometry.__msgtype__))
            # /heading/fused: body heading as a compass bearing (deg E-of-N, CW+).
            # Inverse of run_eval.rtk_heading_reference's map
            # (yaw_vlocal = radians(bearing - fused_deg)) so it round-trips to the
            # driven venue-local yaw — a clean on-path leg keeps RTK-truth e_psi ~0
            # through the fused path, exactly as it did through course-over-ground.
            if c_head is not None:
                fused_deg = po.BEARING_DEG - math.degrees(oyaw)
                w.write(c_head, tns, TS.serialize_cdr(
                    Float64(data=float(fused_deg)), Float64.__msgtype__))
            # status telemetry (e_psi ~ 0 since on-path)
            if c_stat is not None:
                w.write(c_stat, tns, TS.serialize_cdr(
                    _status_array(ox, oy, oyaw, v, s, total, kappa, abs(kappa) * v,
                                  0.0, math.atan(0.2 * kappa)),
                    Float32MultiArray.__msgtype__))
            if c_tim is not None:
                w.write(c_tim, tns, TS.serialize_cdr(
                    Float32(data=np.float32(0.43 + rng.normal(0, 0.02))),
                    Float32.__msgtype__))
            omega = v * math.tan(math.atan(0.2 * kappa)) / 0.2
            tw = Twist(linear=Vector3(x=v, y=0.0, z=0.0),
                       angular=Vector3(x=0.0, y=0.0, z=float(omega)))
            if c_cmd is not None:
                w.write(c_cmd, tns, TS.serialize_cdr(tw, Twist.__msgtype__))
            if c_cmdr is not None:
                w.write(c_cmdr, tns, TS.serialize_cdr(tw, Twist.__msgtype__))
            if c_estop is not None:
                fired = estop_fired and i == max(1, n // 2)
                w.write(c_estop, tns, TS.serialize_cdr(Bool(data=bool(fired)),
                                                       Bool.__msgtype__))
            # RTK at 5 Hz (every 4th control step)
            if i % 4 == 0:
                lat, lon = po.local_to_latlon(np.array([drive]), anchor)[0]
                if c_fix is not None:
                    w.write(c_fix, tns, TS.serialize_cdr(
                        _navsatfix(t, lat, lon, quality), NavSatFix.__msgtype__))
                if c_rtks is not None:
                    w.write(c_rtks, tns, TS.serialize_cdr(
                        String(data=_rtk_status_str(quality, lat, lon)),
                        String.__msgtype__))
                # regular GPS: noisier, lat/lon only (the L5 product)
                if c_pix is not None:
                    w.write(c_pix, tns, TS.serialize_cdr(
                        _navsatfix(t, lat + rng.normal(0, 1e-5),
                                   lon + rng.normal(0, 1e-5), 1),
                        NavSatFix.__msgtype__))
                n_rtk += 1
                if quality == 4:
                    n_fixed += 1
        # latched done at end
        if c_done is not None:
            w.write(c_done, int((t0 + n * dt) * 1e9),
                    TS.serialize_cdr(Bool(data=True), Bool.__msgtype__))

    rtag = f"{R:g}".replace(".", "p")
    vtag = f"{v:g}".replace(".", "p")
    cell_id = f"R{rtag}-v{vtag}-{family}-{controller}"
    sidecar = {
        "schema_version": 1,
        "run_id": f"fixture_{cell_id}-rep{rep}-{leg}",
        "cell_id": cell_id,
        "leg": leg,
        "bag_path": out_dir,
        "cell_params": {"controller": controller, "v_const": v,
                        "radius_m": R, "path_family": family, "rep": rep},
        "path_recipe": recipe,
        "venue": {"venue_id": "rooftop", "start_pin_id": "A", "end_pin_id": "B",
                  "anchor": {"lat0": po.LAT0, "lon0": po.LON0,
                             "bearing_deg": po.BEARING_DEG}},
        "rtk_summary": {"fixed_samples": n_fixed, "total_samples": n_rtk,
                        "fixed_pct": (100.0 * n_fixed / n_rtk) if n_rtk else 0.0},
        "classification": {
            "pass": (True if sidecar_pass_mismatch else (not rtk_bad and not estop_fired and not odom_gap and not missing_topic)),
            "reasons": [],
        },
        "wallclock": {"start_utc": datetime.now(timezone.utc).isoformat(),
                      "duration_s": round(T, 3)},
        "controller_tuning": {"controller_type": controller, "k_e": 3.0,
                              "R_min": min(R, 0.5)},
        "git_commit": "fixture",
    }
    base = os.path.basename(out_dir.rstrip("/"))
    sidecar_path = os.path.join(os.path.dirname(out_dir) or ".",
                                f"{base}.sidecar.json")
    with open(sidecar_path, "w", encoding="utf-8") as f:
        json.dump(sidecar, f, indent=2)
        f.write("\n")
    return out_dir, sidecar_path


def make_batch(root):
    """Generate a small matrix of legs for aggregate/pipeline testing.

    2 controllers x 2 radii x 2 reps x {AtoB,BtoA}, v=1.0, family=step = 16 legs.
    Returns the list of bag dirs.
    """
    bags = []
    for controller in ("lpv-hinf", "pid"):
        for R in (0.5, 0.4):
            for rep in (0, 1):
                for leg in ("AtoB", "BtoA"):
                    rtag = f"{R:g}".replace(".", "p")
                    name = f"{controller}_R{rtag}_rep{rep}_{leg}"
                    out = os.path.join(root, name)
                    bag, _ = make(out, controller=controller, R=R, v=1.0,
                                  rep=rep, leg=leg, family="step")
                    bags.append(bag)
    return bags


def main(argv=None):
    ap = argparse.ArgumentParser(description="Make synthetic leg bag(s) + sidecar(s).")
    ap.add_argument("out_dir", nargs="?",
                    default=os.path.join(os.path.dirname(__file__), "fixtures",
                                         "fixture_step_AtoB"),
                    help="bag directory to create (single-leg mode)")
    ap.add_argument("--rtk-bad", action="store_true",
                    help="write quality=1 status so qc fails the RTK gate")
    ap.add_argument("--clean", action="store_true",
                    help="zero tracking error (drive exactly on-path)")
    ap.add_argument("--estop-fired", action="store_true",
                    help="write one /estop true sample so qc fails the leg")
    ap.add_argument("--odom-gap", action="store_true",
                    help="drop raw /wheel/odom for >1s so qc fails continuity")
    ap.add_argument("--missing-topic", default=None,
                    help="omit one topic from the bag, e.g. /path_follower/done")
    ap.add_argument("--sidecar-pass-mismatch", action="store_true",
                    help="force sidecar classification pass even for a bad leg")
    ap.add_argument("--batch", metavar="ROOT", default=None,
                    help="generate a small matrix of legs under ROOT instead")
    args = ap.parse_args(argv)
    if args.batch:
        bags = make_batch(args.batch)
        print(f"wrote {len(bags)} legs under {args.batch}")
        return 0
    bag, sidecar = make(args.out_dir, rtk_bad=args.rtk_bad,
                        err_amp=(0.0 if args.clean else None),
                        estop_fired=args.estop_fired,
                        odom_gap=args.odom_gap,
                        missing_topic=args.missing_topic,
                        sidecar_pass_mismatch=args.sidecar_pass_mismatch)
    print(bag)
    print(sidecar)
    return 0


if __name__ == "__main__":
    sys.exit(main())
