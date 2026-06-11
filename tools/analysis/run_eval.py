#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Per-leg evaluation: rosbag2 + sidecar -> metrics JSON (T11 / A1, A3).

Reads ONE recorded leg (a rosbag2 directory + its paired
``<bag>.sidecar.json``), reconstructs the driven trajectory two ways, rebuilds
the analytic reference path from the sidecar ``path_recipe`` (the *same*
mapping the follower used at drive time), recomputes cross-track / heading
error against that reference, and emits ``compute_metrics()`` **twice**:

  * **odom-belief**  — trajectory from ``/wheel/odom_zeroed`` (the re-anchored
                       stream the controller actually tracked), falling back to
                       raw ``/wheel/odom`` only when the zeroed stream is absent
  * **RTK-truth**    — trajectory from ``/gps_rtk_f9p_helical/gps/fix``
                       projected to the venue-local frame (ground truth)

On top of the library metrics it adds **terminal-pose-error** (Euclidean +
heading at the last sample vs the path endpoint) and **steering-effort**
(integral / RMS of ``delta_cmd`` from ``/path_follower/status``, with a
``/cmd_vel`` angular fallback), plus a compute-cost summary from
``/path_follower/timing``.

Design constraints (laptop authoring; see CLAUDE.md):
  * No ROS sourcing. Bags are read with the ``rosbags`` pip library, which
    parses rosbag2 sqlite3 / mcap directly. If it is not installed we fail with
    a clear, actionable message rather than fabricating numbers.
  * Real-bag validation is DEFERRED to the trip (no bags exist yet). This tool
    is written to run end-to-end on a single bag+sidecar and to error clearly
    when an input is missing.

Usage::

    python3 tools/analysis/run_eval.py /path/to/<bag_dir> \\
        [--sidecar /path/to/<bag>.sidecar.json] \\
        [--out /path/to/<bag>.metrics.json] \\
        [--t-transient 2.0] [--k-e 3.0]

If ``--sidecar`` is omitted it defaults to ``<bag_dir>.sidecar.json`` (the T7
naming) and then ``<bag_dir>/<basename>.sidecar.json``.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys

import numpy as np


# -----------------------------------------------------------------------------
# Imports of repo libraries (path classes, guidance, metrics). These are pure
# numpy and import fine on the laptop (verified: `vfg import OK`).
# -----------------------------------------------------------------------------

_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
_VFG_ROOT = os.path.join(_REPO_ROOT, "scalecar-vfg-h-infinite")
if _VFG_ROOT not in sys.path:
    sys.path.insert(0, _VFG_ROOT)
# path_overlay lives under tools/path_gen — reuse its lat/lon <-> local helpers.
_PATHGEN = os.path.join(_REPO_ROOT, "tools", "path_gen")
if _PATHGEN not in sys.path:
    sys.path.insert(0, _PATHGEN)

try:
    from vfg_pathfollowing.paths.step_curvature import StepCurvaturePath
    from vfg_pathfollowing.paths.slalom import SlalomPath
    from vfg_pathfollowing.guidance.vfg import VectorFieldGuidance
    from vfg_pathfollowing.simulation.result import SimResult
    from vfg_pathfollowing.simulation.metrics import compute_metrics
except Exception as exc:  # pragma: no cover - defensive
    raise SystemExit(
        "[run_eval] cannot import vfg_pathfollowing from "
        f"{_VFG_ROOT!r}: {exc}\n"
        "This tool needs the in-repo path/guidance/metrics library."
    )


# Topic names (the recorded set — see Data_Logger.py TOPICS / the T7 contract).
TOPIC_ODOM = "/wheel/odom"
# What the controller actually tracked: the orchestrator launches the follower
# with -r /wheel/odom:=/wheel/odom_zeroed. odom-belief metrics MUST use this
# stream when present; raw /wheel/odom is a different (un-anchored) frame and
# scoring it against the origin-anchored analytic path is meaningless.
TOPIC_ODOM_ZEROED = "/wheel/odom_zeroed"
TOPIC_RTK_FIX = "/gps_rtk_f9p_helical/gps/fix"
TOPIC_RTK_STATUS = "/gps_rtk_f9p_helical/gps/rtk_status"
TOPIC_PIX_FIX = "/pixhawk/global_position/raw/fix"
TOPIC_STATUS = "/path_follower/status"
TOPIC_TIMING = "/path_follower/timing"
TOPIC_CMD_VEL = "/cmd_vel"
TOPIC_CMD_VEL_RAW = "/cmd_vel_raw"
TOPIC_ESTOP = "/estop"

# RTK FIXED is quality==4 on the rtk_status String (NavSatFix.status cannot
# distinguish RTK on this F9P — feedback_rtk_and_venue). The driver formats
# "... (quality=N, sats=...) ...".
import re as _re
_RTK_QUALITY_RE = _re.compile(r"quality=(\d+)")
RTK_FIXED_QUALITY = 4


def parse_rtk_quality(status_str):
    """Extract the integer fix quality from an rtk_status String, or None."""
    if not status_str:
        return None
    m = _RTK_QUALITY_RE.search(str(status_str))
    return int(m.group(1)) if m else None

# Status Float32MultiArray layout (path_follower_node.py:172-179, hw-verified).
STATUS_LABELS = [
    "x", "y", "yaw", "v", "s_star", "total_length",
    "kappa", "rho", "e_psi", "delta_cmd", "has_path",
]
STATUS_DELTA_CMD = STATUS_LABELS.index("delta_cmd")


# =============================================================================
# Analytic reference reconstruction (mirror of path_follower_node.build_path_from_recipe)
# =============================================================================

def build_path_from_recipe(recipe, r_min_default=0.5):
    """Map a recipe dict -> analytic PathBase, identically to the follower node.

    Kept byte-for-byte equivalent to ``PathFollowerNode.build_path_from_recipe``
    (path_follower_node.py:294-357) so the analytic reference we score against is
    *exactly* the curve that was driven. The only difference: the node reads its
    U-turn radius from the live ``R_min`` param; here we take it from the recipe
    if present, else from ``r_min_default`` (which the caller can override from
    the sidecar's controller_tuning if it carries an R_min).

    Schema::  {"type": "step"|"slalom"|"uturn", "params": {...}}
    """
    if not isinstance(recipe, dict):
        raise ValueError(
            f"recipe must be a JSON object, got {type(recipe).__name__}")

    ptype = str(recipe.get("type", "")).lower().strip()
    params = recipe.get("params", {}) or {}
    if not isinstance(params, dict):
        raise ValueError("recipe 'params' must be a JSON object")

    def _f(key, default):
        return float(params.get(key, default))

    def _i(key, default):
        return int(params.get(key, default))

    if ptype == "step":
        return StepCurvaturePath(
            L1=_f("L1", 5.0),
            R=_f("R", 0.5),
            theta_arc=_f("theta_arc", math.pi / 2),
            L2=_f("L2", 5.0),
            direction=_i("direction", 1),
        )
    elif ptype == "slalom":
        return SlalomPath(
            R=_f("R", 0.5),
            theta_arc=_f("theta_arc", math.pi / 2),
            L1=_f("L1", 5.0),
            L_mid=_f("L_mid", 2.0),
            n_arcs=_i("n_arcs", 6),
            L_end=_f("L_end", 25.0),
        )
    elif ptype == "uturn":
        return StepCurvaturePath(
            L1=_f("L1", 1.0),
            R=_f("R", r_min_default),
            theta_arc=math.pi,
            L2=_f("L2", 1.0),
            direction=_i("direction", 1),
        )
    else:
        raise ValueError(
            f"unknown recipe type '{ptype}'; expected 'step', 'slalom', or 'uturn'")


# =============================================================================
# Bag reading (rosbags lib; degrade gracefully if absent)
# =============================================================================

def _require_rosbags():
    try:
        from rosbags.highlevel import AnyReader  # noqa: F401
        try:
            # rosbag2 Humble bags carry no embedded type definitions; newer
            # rosbags releases refuse to open them without an explicit
            # typestore.
            import functools

            from rosbags.typesys import Stores, get_typestore
            return functools.partial(
                AnyReader, default_typestore=get_typestore(Stores.ROS2_HUMBLE))
        except ImportError:
            return AnyReader
    except Exception as exc:
        raise SystemExit(
            "[run_eval] the 'rosbags' pip library is required to read rosbag2 "
            "files without sourcing ROS.\n"
            "  Install it on whatever host runs the analysis:  pip install rosbags\n"
            f"  (import failed with: {exc})\n"
            "Fallback: if 'rosbags' cannot be installed, decode the bag on the "
            "NUC with ROS sourced and `ros2 bag` / rosbag2_py, then feed the "
            "extracted arrays in — but do not invent values."
        )


def read_bag(bag_dir):
    """Read the topics we need from a rosbag2 directory.

    Returns a dict of arrays keyed by topic. Each entry holds parallel arrays
    keyed by field (e.g. 'stamp', 'x', 'y', ...). Missing topics map to None so
    callers can decide how to degrade.
    """
    AnyReader = _require_rosbags()
    from pathlib import Path

    bag = Path(bag_dir)
    if not bag.exists():
        raise SystemExit(f"[run_eval] bag path does not exist: {bag_dir}")

    out = {
        TOPIC_ODOM: None,
        TOPIC_ODOM_ZEROED: None,
        TOPIC_RTK_FIX: None,
        TOPIC_RTK_STATUS: None,
        TOPIC_PIX_FIX: None,
        TOPIC_STATUS: None,
        TOPIC_TIMING: None,
        TOPIC_CMD_VEL: None,
        TOPIC_CMD_VEL_RAW: None,
        TOPIC_ESTOP: None,
    }
    # Accumulators
    odom = {"stamp": [], "x": [], "y": [], "yaw": [], "v": []}
    odom_zeroed = {"stamp": [], "x": [], "y": [], "yaw": [], "v": []}
    rtk = {"stamp": [], "lat": [], "lon": [], "alt": [], "status": []}
    rtk_status = {"stamp": [], "quality": []}
    pix = {"stamp": [], "lat": [], "lon": [], "alt": []}
    status = {"stamp": [], "delta_cmd": [], "e_psi": [], "has_path": []}
    timing = {"stamp": [], "ms": []}
    cmd = {"stamp": [], "ang_z": [], "lin_x": []}
    cmd_raw = {"stamp": [], "ang_z": [], "lin_x": []}
    estop = {"stamp": [], "active": []}

    want = set(out.keys())

    with AnyReader([bag]) as reader:
        # Raw per-topic message counts for ALL recorded topics (not just the
        # decoded set) so the QC gate can verify required-topic presence.
        counts = {}
        for c in reader.connections:
            counts[c.topic] = counts.get(c.topic, 0) + (c.msgcount or 0)
        out["_counts"] = counts
        conns = [c for c in reader.connections if c.topic in want]
        for conn, t_ns, raw in reader.messages(connections=conns):
            msg = reader.deserialize(raw, conn.msgtype)
            t = t_ns * 1e-9
            topic = conn.topic
            if topic == TOPIC_ODOM:
                p = msg.pose.pose.position
                q = msg.pose.pose.orientation
                odom["stamp"].append(t)
                odom["x"].append(p.x)
                odom["y"].append(p.y)
                odom["yaw"].append(_yaw_from_quat(q.x, q.y, q.z, q.w))
                odom["v"].append(msg.twist.twist.linear.x)
            elif topic == TOPIC_ODOM_ZEROED:
                p = msg.pose.pose.position
                q = msg.pose.pose.orientation
                odom_zeroed["stamp"].append(t)
                odom_zeroed["x"].append(p.x)
                odom_zeroed["y"].append(p.y)
                odom_zeroed["yaw"].append(_yaw_from_quat(q.x, q.y, q.z, q.w))
                odom_zeroed["v"].append(msg.twist.twist.linear.x)
            elif topic == TOPIC_RTK_FIX:
                rtk["stamp"].append(t)
                rtk["lat"].append(msg.latitude)
                rtk["lon"].append(msg.longitude)
                rtk["alt"].append(getattr(msg, "altitude", float("nan")))
                rtk["status"].append(int(getattr(msg.status, "status", 0)))
            elif topic == TOPIC_RTK_STATUS:
                q = parse_rtk_quality(getattr(msg, "data", ""))
                rtk_status["stamp"].append(t)
                rtk_status["quality"].append(q if q is not None else -1)
            elif topic == TOPIC_PIX_FIX:
                pix["stamp"].append(t)
                pix["lat"].append(msg.latitude)
                pix["lon"].append(msg.longitude)
                pix["alt"].append(getattr(msg, "altitude", float("nan")))
            elif topic == TOPIC_STATUS:
                data = list(msg.data)
                status["stamp"].append(t)
                status["delta_cmd"].append(
                    data[STATUS_DELTA_CMD] if len(data) > STATUS_DELTA_CMD else float("nan"))
                status["e_psi"].append(
                    data[STATUS_LABELS.index("e_psi")]
                    if len(data) > STATUS_LABELS.index("e_psi") else float("nan"))
                status["has_path"].append(
                    data[STATUS_LABELS.index("has_path")]
                    if len(data) > STATUS_LABELS.index("has_path") else 0.0)
            elif topic == TOPIC_TIMING:
                timing["stamp"].append(t)
                timing["ms"].append(float(msg.data))
            elif topic == TOPIC_CMD_VEL:
                cmd["stamp"].append(t)
                cmd["ang_z"].append(msg.angular.z)
                cmd["lin_x"].append(msg.linear.x)
            elif topic == TOPIC_CMD_VEL_RAW:
                cmd_raw["stamp"].append(t)
                cmd_raw["ang_z"].append(msg.angular.z)
                cmd_raw["lin_x"].append(msg.linear.x)
            elif topic == TOPIC_ESTOP:
                estop["stamp"].append(t)
                estop["active"].append(1.0 if bool(msg.data) else 0.0)

    def _np(d):
        if not d["stamp"]:
            return None
        return {k: np.asarray(v, dtype=float) for k, v in d.items()}

    out[TOPIC_ODOM] = _np(odom)
    out[TOPIC_ODOM_ZEROED] = _np(odom_zeroed)
    out[TOPIC_RTK_FIX] = _np(rtk)
    out[TOPIC_RTK_STATUS] = _np(rtk_status)
    out[TOPIC_PIX_FIX] = _np(pix)
    out[TOPIC_STATUS] = _np(status)
    out[TOPIC_TIMING] = _np(timing)
    out[TOPIC_CMD_VEL] = _np(cmd)
    out[TOPIC_CMD_VEL_RAW] = _np(cmd_raw)
    out[TOPIC_ESTOP] = _np(estop)
    return out


def odom_belief_source(bag):
    """Pick the odom stream the controller actually tracked.

    Prefer ``/wheel/odom_zeroed`` (the re-anchored stream the follower is
    remapped onto); fall back to raw ``/wheel/odom`` for manual/indoor bags
    recorded without the odom-zeroing overlay. Returns ``(arr_or_None, label)``
    where label is the topic name used (for provenance / warnings).
    """
    zeroed = bag.get(TOPIC_ODOM_ZEROED)
    if zeroed is not None:
        return zeroed, TOPIC_ODOM_ZEROED
    return bag.get(TOPIC_ODOM), TOPIC_ODOM


def fixed_mask_for(fix_stamps, rtk_status, fixed_quality=RTK_FIXED_QUALITY,
                   max_dt=None):
    """Boolean mask over fix_stamps: True where the nearest rtk_status is FIXED.

    The rtk_status String topic carries the authoritative fix quality; match
    each NavSatFix sample to the temporally-nearest rtk_status sample. If there
    is no rtk_status topic at all, returns None (caller falls back to using
    every fix, as before).

    ``max_dt`` (seconds, optional): when set, a fix whose nearest rtk_status
    sample is more than ``max_dt`` away in time is treated as **not FIXED**
    (conservative) — guards against sparse/stale status mislabelling a fix as
    FIXED from a temporally-distant sample. Default ``None`` keeps the legacy
    nearest-neighbour behaviour (no time guard) so existing callers are
    unchanged; the threshold is a deliberate choice the caller must opt into.
    """
    if rtk_status is None or len(rtk_status.get("stamp", [])) == 0:
        return None
    qs = np.asarray(rtk_status["stamp"], float)
    qv = np.asarray(rtk_status["quality"], float)
    order = np.argsort(qs)
    qs, qv = qs[order], qv[order]
    fix_stamps = np.asarray(fix_stamps, float)
    idx = np.searchsorted(qs, fix_stamps)
    idx = np.clip(idx, 0, len(qs) - 1)
    # pick nearer of idx and idx-1
    left = np.clip(idx - 1, 0, len(qs) - 1)
    pick = np.where(np.abs(qs[idx] - fix_stamps) <= np.abs(fix_stamps - qs[left]),
                    idx, left)
    mask = qv[pick] == fixed_quality
    if max_dt is not None:
        mask = mask & (np.abs(qs[pick] - fix_stamps) <= float(max_dt))
    return mask


def _yaw_from_quat(x, y, z, w):
    """Yaw (Z) from a quaternion."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


# =============================================================================
# RTK -> local frame projection (reuse path_overlay helpers)
# =============================================================================

def project_rtk_to_local(lat, lon, anchor_spec):
    """Project RTK lat/lon arrays to the venue-local (x, y) frame.

    Reuses ``tools/path_gen/path_overlay.latlon_to_local`` with an ``Anchor``.
    ``anchor_spec`` is a dict {lat0, lon0, bearing_deg}; if None, falls back to
    the hardcoded rooftop anchor in path_overlay (LAT0/LON0/BEARING_DEG).

    Returns (x, y, yaw) arrays. yaw is the course-over-ground derived from
    successive RTK samples (forward-difference), so the first sample copies the
    second's heading. This is the same notion the reposition node uses.
    """
    import path_overlay as po

    if anchor_spec is None:
        anchor = po.Anchor(po.LAT0, po.LON0, po.BEARING_DEG)
    else:
        anchor = po.Anchor(
            float(anchor_spec["lat0"]),
            float(anchor_spec["lon0"]),
            float(anchor_spec.get("bearing_deg", po.BEARING_DEG)),
        )

    latlon = np.column_stack([np.asarray(lat, float), np.asarray(lon, float)])
    xy = po.latlon_to_local(latlon, anchor)  # (N, 2)
    x = xy[:, 0]
    y = xy[:, 1]

    yaw = _course_over_ground(x, y)
    return x, y, yaw, anchor


def transform_to_path_frame(x, y, yaw, venue_anchor, path_frame_anchor):
    """Re-anchor a venue-local trajectory into the per-leg path frame.

    The analytic reference path is anchored at the start-pin pose with +x along
    the start heading (the same frame odom_zero_node bakes into /wheel/odom_zeroed
    at the reset moment). To score an RTK-truth trajectory against that path,
    project RTK to venue-local first (project_rtk_to_local) and then apply the
    same SE(2) re-anchor odom_zero applies on the wheel side:
        P' = R(-anchor_yaw_local) * (P - anchor_local)

    `path_frame_anchor` is the leg's start pin: {lat, lon, heading_deg}. The
    compass-to-local-yaw conversion mirrors reposition_node._bearing_deg_to_local_yaw:
        anchor_yaw_local = radians(venue_anchor.bearing_deg - heading_deg)

    Returns (x', y', yaw') arrays. If `path_frame_anchor` is None this is a
    pass-through (legacy venue-local behaviour, kept for old bags).

    NOTE: the anchor here is the SCHEDULED pin pose, not the actual RTK fix at
    /odom_zero/reset time. For paper-grade precision the live RTK fix at reset
    is more accurate (reposition_node arrives within pos_tol_m = 0.15 m of the
    pin, so the pin-anchor introduces a sub-decimeter frame offset). The cleaner
    upgrade is to capture the live RTK fix in /odom_zero/status and surface that
    through the sidecar -- left as a follow-up.
    """
    if path_frame_anchor is None:
        return x, y, yaw

    import path_overlay as po

    a_lat = float(path_frame_anchor["lat"])
    a_lon = float(path_frame_anchor["lon"])
    a_hdg_deg = float(path_frame_anchor["heading_deg"])

    axy = po.latlon_to_local(np.array([[a_lat, a_lon]]), venue_anchor)
    ax, ay = float(axy[0, 0]), float(axy[0, 1])

    anchor_yaw_local = math.radians(venue_anchor.bearing_deg - a_hdg_deg)
    anchor_yaw_local = math.atan2(
        math.sin(anchor_yaw_local), math.cos(anchor_yaw_local))

    c = math.cos(anchor_yaw_local)
    s = math.sin(anchor_yaw_local)

    x_arr = np.asarray(x, float)
    y_arr = np.asarray(y, float)
    yaw_arr = np.asarray(yaw, float)

    dx = x_arr - ax
    dy = y_arr - ay
    x_p = c * dx + s * dy
    y_p = -s * dx + c * dy
    diff = yaw_arr - anchor_yaw_local
    yaw_p = np.arctan2(np.sin(diff), np.cos(diff))
    return x_p, y_p, yaw_p


def _course_over_ground(x, y):
    """Heading from forward differences of an (x, y) track [rad]."""
    n = len(x)
    yaw = np.zeros(n)
    if n < 2:
        return yaw
    dx = np.diff(x)
    dy = np.diff(y)
    cog = np.arctan2(dy, dx)
    yaw[:-1] = cog
    yaw[-1] = cog[-1]
    # Smooth out segments where the robot is essentially stationary (noise).
    step = np.hypot(dx, dy)
    for i in range(1, n - 1):
        if step[i] < 1e-3:  # under 1 mm between fixes -> reuse previous heading
            yaw[i] = yaw[i - 1]
    return yaw


# =============================================================================
# Error reconstruction against the analytic reference
# =============================================================================

def errors_along_path(path, x, y, yaw, k_e=3.0):
    """Compute (e_d, e_psi, kappa, rho, s_star, psi_des) along a trajectory.

    e_d via ``path.signed_distance`` (the spec's named primitive); e_psi and
    psi_des via ``VectorFieldGuidance`` (the same guidance the controller ran).
    rho = |kappa| (speed-agnostic here; the runtime rho folds v in, but for an
    analysis figure |kappa| is the path-intrinsic schedule).

    All arrays are length N (one per trajectory sample). Uses the previous
    sample's s_star as the warm-start for the next projection (monotone-ish
    arc-length tracking, robust on self-approaching paths like the U-turn).
    """
    guidance = VectorFieldGuidance(path, k_e=k_e)
    n = len(x)
    e_d = np.zeros(n)
    e_psi = np.zeros(n)
    kappa = np.zeros(n)
    s_star = np.zeros(n)
    psi_des = np.zeros(n)

    s_prev = None
    for i in range(n):
        q = (x[i], y[i])
        ed_i, s_i = path.signed_distance(q, s_init=s_prev)
        res = guidance.compute(q, yaw[i])
        e_d[i] = ed_i
        s_star[i] = s_i
        kappa[i] = res["kappa"]
        psi_des[i] = res["psi_des"]
        ep = res["psi_des"] - yaw[i]
        e_psi[i] = math.atan2(math.sin(ep), math.cos(ep))  # wrap to (-pi, pi]
        s_prev = s_i

    rho = np.abs(kappa)
    return e_d, e_psi, kappa, rho, s_star, psi_des


def build_sim_result(t, x, y, yaw, v, path, label, k_e=3.0):
    """Assemble a SimResult-shaped object so compute_metrics() can score it.

    ``t`` is re-zeroed to start at 0 so the metrics' ``t_transient`` window has
    a meaningful origin (bag stamps are absolute epoch seconds).
    """
    t = np.asarray(t, float)
    if len(t):
        t = t - t[0]
    e_d, e_psi, kappa, rho, s_star, psi_des = errors_along_path(
        path, x, y, yaw, k_e=k_e)
    return SimResult(
        time=t,
        X=np.asarray(x, float),
        Y=np.asarray(y, float),
        psi=np.asarray(yaw, float),
        v=np.asarray(v, float),
        delta=np.zeros(len(t)),
        psi_des=psi_des,
        e_psi=e_psi,
        e_d=e_d,
        kappa=kappa,
        rho=rho,
        delta_cmd=np.zeros(len(t)),
        label=label,
    )


# =============================================================================
# Extra metrics: terminal pose error + steering effort + compute cost
# =============================================================================

def terminal_pose_error(path, x, y, yaw):
    """Euclidean + heading error at the final trajectory sample vs path end."""
    if len(x) == 0:
        return {"terminal_pos_err_m": None, "terminal_heading_err_deg": None}
    s_end = path.total_length
    p_end = path.position(s_end)
    psi_end = path.heading(s_end)
    dx = x[-1] - p_end[0]
    dy = y[-1] - p_end[1]
    pos_err = float(math.hypot(dx, dy))
    dpsi = yaw[-1] - psi_end
    dpsi = math.atan2(math.sin(dpsi), math.cos(dpsi))
    return {
        "terminal_pos_err_m": pos_err,
        "terminal_heading_err_deg": float(math.degrees(abs(dpsi))),
    }


def steering_effort(status, cmd_raw, cmd):
    """Steering-effort metrics from delta_cmd (preferred) with cmd_vel fallback.

    Returns RMS, integral of |rate|, and total variation. delta_cmd from
    ``/path_follower/status`` is the true steering command (rad); if that topic
    is absent we fall back to angular.z on /cmd_vel_raw then /cmd_vel (the
    actuation surrogate).
    """
    src = None
    t = sig = None
    if status is not None and "delta_cmd" in status:
        d = status["delta_cmd"]
        if np.any(np.isfinite(d)):
            t = status["stamp"]
            sig = d
            src = "status.delta_cmd"
    if sig is None and cmd_raw is not None:
        t = cmd_raw["stamp"]
        sig = cmd_raw["ang_z"]
        src = "cmd_vel_raw.angular_z"
    if sig is None and cmd is not None:
        t = cmd["stamp"]
        sig = cmd["ang_z"]
        src = "cmd_vel.angular_z"
    if sig is None or len(sig) == 0:
        return {"source": None, "rms": None, "total_variation": None,
                "abs_rate_integral": None}

    sig = np.asarray(sig, float)
    finite = np.isfinite(sig)
    sig = sig[finite]
    t = np.asarray(t, float)[finite]
    if len(sig) == 0:
        return {"source": src, "rms": None, "total_variation": None,
                "abs_rate_integral": None}

    rms = float(np.sqrt(np.mean(sig ** 2)))
    tv = float(np.sum(np.abs(np.diff(sig)))) if len(sig) > 1 else 0.0
    # integral of |d sig / dt| dt = same as total variation in value; report the
    # time-weighted |rate| integral too for completeness.
    if len(sig) > 1:
        dt = np.diff(t)
        dt[dt <= 0] = np.nan
        rate = np.abs(np.diff(sig)) / dt
        abs_rate_integral = float(np.nansum(np.abs(np.diff(sig))))
    else:
        abs_rate_integral = 0.0
    return {"source": src, "rms": rms, "total_variation": tv,
            "abs_rate_integral": abs_rate_integral}


def compute_cost(timing):
    """Per-cycle controller compute-cost summary from /path_follower/timing."""
    if timing is None or len(timing.get("ms", [])) == 0:
        return {"n": 0, "mean_ms": None, "p50_ms": None, "p95_ms": None,
                "max_ms": None}
    ms = np.asarray(timing["ms"], float)
    ms = ms[np.isfinite(ms)]
    if len(ms) == 0:
        return {"n": 0, "mean_ms": None, "p50_ms": None, "p95_ms": None,
                "max_ms": None}
    return {
        "n": int(len(ms)),
        "mean_ms": float(np.mean(ms)),
        "p50_ms": float(np.percentile(ms, 50)),
        "p95_ms": float(np.percentile(ms, 95)),
        "max_ms": float(np.max(ms)),
    }


# =============================================================================
# Sidecar loading
# =============================================================================

def load_sidecar(bag_dir, sidecar_arg):
    """Resolve and load the paired sidecar JSON."""
    candidates = []
    if sidecar_arg:
        candidates.append(sidecar_arg)
    bag_dir = bag_dir.rstrip("/")
    base = os.path.basename(bag_dir)
    candidates.append(f"{bag_dir}.sidecar.json")
    candidates.append(os.path.join(bag_dir, f"{base}.sidecar.json"))
    for c in candidates:
        if c and os.path.isfile(c):
            with open(c, "r", encoding="utf-8") as f:
                return json.load(f), c
    raise SystemExit(
        "[run_eval] no sidecar JSON found. Tried:\n  " +
        "\n  ".join(candidates) +
        "\nPass --sidecar explicitly. The sidecar (T7 D3) carries the "
        "path_recipe needed to rebuild the analytic reference.")


# =============================================================================
# Main
# =============================================================================

def evaluate(bag_dir, sidecar, t_transient=2.0, k_e=3.0):
    """Run the full per-leg evaluation; return the metrics dict."""
    recipe = sidecar.get("path_recipe")
    if not recipe:
        raise SystemExit(
            "[run_eval] sidecar has no 'path_recipe'; cannot rebuild the "
            "analytic reference. (Is this a T7-produced sidecar?)")

    ctrl_tuning = sidecar.get("controller_tuning", {}) or {}
    r_min = float(ctrl_tuning.get("R_min", ctrl_tuning.get("r_min", 0.5)))
    eff_k_e = float(ctrl_tuning.get("k_e", k_e))
    venue = sidecar.get("venue", {}) or {}
    anchor_spec = venue.get("anchor")  # optional {lat0, lon0, bearing_deg}
    # Per-leg path-frame anchor (start pin pose, see _write_sidecar). Without
    # it RTK-truth would score against the fixed venue frame -- offset from the
    # analytic path's start-pin frame by tens of meters, the same class of bug
    # the odom-belief side had before /wheel/odom_zeroed was bagged.
    path_frame_anchor = venue.get("path_frame_anchor")

    path = build_path_from_recipe(recipe, r_min_default=r_min)

    bag = read_bag(bag_dir)

    result = {
        "bag_path": os.path.abspath(bag_dir),
        "run_id": sidecar.get("run_id"),
        "cell_id": sidecar.get("cell_id"),
        "leg": sidecar.get("leg"),
        "cell_params": sidecar.get("cell_params"),
        "path_recipe": recipe,
        "analytic_total_length_m": float(path.total_length),
        "controller_tuning": ctrl_tuning,
        "k_e_used": eff_k_e,
        "t_transient_s": t_transient,
        "metrics": {},
        "warnings": [],
    }

    # ---- odom-belief ----------------------------------------------------
    odom, odom_src = odom_belief_source(bag)
    result["odom_belief_source"] = odom_src
    if odom_src == TOPIC_ODOM and bag.get(TOPIC_ODOM_ZEROED) is None:
        result["warnings"].append(
            "no /wheel/odom_zeroed in bag; odom-belief scored against raw "
            "/wheel/odom (correct only for runs without the odom-zeroing overlay)")
    if odom is None:
        result["warnings"].append(
            "no odom messages in bag; odom-belief metrics skipped")
        result["metrics"]["odom_belief"] = None
    else:
        sr_odom = build_sim_result(
            odom["stamp"], odom["x"], odom["y"], odom["yaw"], odom["v"],
            path, label="odom-belief", k_e=eff_k_e)
        m = compute_metrics(sr_odom, t_transient=t_transient)
        m.update(terminal_pose_error(path, odom["x"], odom["y"], odom["yaw"]))
        m["n_samples"] = int(len(odom["stamp"]))
        result["metrics"]["odom_belief"] = m

    # ---- RTK-truth ------------------------------------------------------
    rtk = bag[TOPIC_RTK_FIX]
    if rtk is None:
        result["warnings"].append(
            f"no {TOPIC_RTK_FIX} messages in bag; RTK-truth metrics skipped")
        result["metrics"]["rtk_truth"] = None
        result["rtk_anchor"] = None
        result["rtk_fixed_fraction"] = None
    else:
        # RTK-truth is only ground truth where the fix is RTK FIXED (quality=4).
        # Filter to FIXED samples; fall back to all fixes if rtk_status absent.
        mask = fixed_mask_for(rtk["stamp"], bag[TOPIC_RTK_STATUS])
        if mask is None:
            result["rtk_fixed_fraction"] = None
            result["warnings"].append(
                f"no {TOPIC_RTK_STATUS} in bag; RTK-truth uses ALL fixes "
                "regardless of quality (cannot confirm FIXED).")
        else:
            frac = float(np.mean(mask)) if len(mask) else 0.0
            result["rtk_fixed_fraction"] = frac
            if int(np.sum(mask)) >= 2:
                rtk = {k: v[mask] for k, v in rtk.items()}
            else:
                result["warnings"].append(
                    f"only {int(np.sum(mask))} RTK-FIXED samples; RTK-truth "
                    "falls back to all fixes (metrics unreliable).")
        x, y, yaw, anchor = project_rtk_to_local(
            rtk["lat"], rtk["lon"], anchor_spec)
        # Re-anchor venue-local -> per-leg path frame using the start-pin pose
        # carried in the sidecar. Pass-through if absent (old bags).
        x, y, yaw = transform_to_path_frame(x, y, yaw, anchor, path_frame_anchor)
        # RTK has no native body velocity; approximate from successive fixes.
        v = _speed_from_track(rtk["stamp"], x, y)
        sr_rtk = build_sim_result(
            rtk["stamp"], x, y, yaw, v, path, label="rtk-truth", k_e=eff_k_e)
        m = compute_metrics(sr_rtk, t_transient=t_transient)
        m.update(terminal_pose_error(path, x, y, yaw))
        m["n_samples"] = int(len(rtk["stamp"]))
        result["metrics"]["rtk_truth"] = m
        result["rtk_anchor"] = {
            "lat0": anchor.lat0, "lon0": anchor.lon0,
            "bearing_deg": anchor.bearing_deg,
            "from_sidecar": anchor_spec is not None,
        }
        result["path_frame_anchor"] = (
            None if path_frame_anchor is None else dict(path_frame_anchor))
        if path_frame_anchor is None:
            result["warnings"].append(
                "no venue.path_frame_anchor in sidecar; RTK-truth scored "
                "against the venue frame, not the per-leg path frame "
                "(metrics may carry a fixed SE(2) offset)")
        if anchor_spec is None:
            result["warnings"].append(
                "no venue.anchor in sidecar; used hardcoded rooftop anchor "
                "from path_overlay (LAT0/LON0/BEARING_DEG). Confirm this matches "
                "the recording venue.")

    # ---- shared extras --------------------------------------------------
    result["steering_effort"] = steering_effort(
        bag[TOPIC_STATUS], bag[TOPIC_CMD_VEL_RAW], bag[TOPIC_CMD_VEL])
    result["compute_cost"] = compute_cost(bag[TOPIC_TIMING])

    return result


def _speed_from_track(stamp, x, y):
    """Approximate longitudinal speed from successive positions [m/s]."""
    n = len(x)
    v = np.zeros(n)
    if n < 2:
        return v
    dt = np.diff(np.asarray(stamp, float))
    dt[dt <= 0] = np.nan
    dist = np.hypot(np.diff(x), np.diff(y))
    spd = dist / dt
    v[:-1] = spd
    v[-1] = spd[-1]
    return np.nan_to_num(v, nan=0.0)


def build_per_sample_frame(bag_dir, sidecar, k_e=3.0, bag=None):
    """Per-sample aligned arrays for one leg, for the lossless export layer (T11).

    Returns ``{"odom": {...}, "rtk": {...}, "gnss_rtk": {...}, "gnss_pix": {...},
    "meta": {...}}`` where each sub-dict maps column -> 1-D array. Reuses the same
    path rebuild + ``errors_along_path`` as the metrics path, so the exported
    series are consistent with the scored metrics. ``gnss_*`` carry RAW lat/lon
    (the prof's separate GNSS product) — never projected, never filtered.
    """
    recipe = sidecar.get("path_recipe")
    if not recipe:
        raise ValueError("sidecar has no path_recipe")
    ctrl_tuning = sidecar.get("controller_tuning", {}) or {}
    r_min = float(ctrl_tuning.get("R_min", ctrl_tuning.get("r_min", 0.5)))
    eff_k_e = float(ctrl_tuning.get("k_e", k_e))
    venue = sidecar.get("venue", {}) or {}
    anchor_spec = venue.get("anchor")
    path_frame_anchor = venue.get("path_frame_anchor")
    path = build_path_from_recipe(recipe, r_min_default=r_min)

    if bag is None:
        bag = read_bag(bag_dir)

    frame = {"odom": {}, "rtk": {}, "gnss_rtk": {}, "gnss_pix": {},
             "meta": {"run_id": sidecar.get("run_id"),
                      "cell_id": sidecar.get("cell_id"),
                      "leg": sidecar.get("leg"),
                      "analytic_total_length_m": float(path.total_length)}}

    odom, odom_src = odom_belief_source(bag)
    frame["meta"]["odom_belief_source"] = odom_src
    if odom is not None:
        e_d, e_psi, kappa, rho, s_star, psi_des = errors_along_path(
            path, odom["x"], odom["y"], odom["yaw"], k_e=eff_k_e)
        t = odom["stamp"] - odom["stamp"][0] if len(odom["stamp"]) else odom["stamp"]
        frame["odom"] = {
            "t": t, "stamp": odom["stamp"], "x": odom["x"], "y": odom["y"],
            "yaw": odom["yaw"], "v": odom["v"], "e_d": e_d, "e_psi": e_psi,
            "kappa": kappa, "rho": rho, "s_star": s_star, "psi_des": psi_des,
        }

    rtk = bag[TOPIC_RTK_FIX]
    if rtk is not None:
        # RAW lat/lon product (unfiltered, unprojected) — the GNSS dataset (L2).
        q = None
        mask = fixed_mask_for(rtk["stamp"], bag[TOPIC_RTK_STATUS])
        if mask is not None:
            q = mask.astype(int)
        frame["gnss_rtk"] = {
            "stamp": rtk["stamp"], "lat": rtk["lat"], "lon": rtk["lon"],
            "alt": rtk.get("alt", np.full(len(rtk["stamp"]), np.nan)),
            "is_fixed": q if q is not None else np.full(len(rtk["stamp"]), -1),
        }
        # local-frame RTK-truth series (FIXED-filtered, scored against the path)
        r = rtk
        if mask is not None and int(np.sum(mask)) >= 2:
            r = {k: v[mask] for k, v in rtk.items()}
        x, y, yaw, _anchor = project_rtk_to_local(r["lat"], r["lon"], anchor_spec)
        x, y, yaw = transform_to_path_frame(x, y, yaw, _anchor, path_frame_anchor)
        e_d, e_psi, kappa, rho, s_star, psi_des = errors_along_path(
            path, x, y, yaw, k_e=eff_k_e)
        t = r["stamp"] - r["stamp"][0] if len(r["stamp"]) else r["stamp"]
        frame["rtk"] = {
            "t": t, "stamp": r["stamp"], "lat": r["lat"], "lon": r["lon"],
            "x": x, "y": y, "yaw": yaw, "e_d": e_d, "e_psi": e_psi,
            "kappa": kappa, "rho": rho, "s_star": s_star, "psi_des": psi_des,
        }

    pix = bag[TOPIC_PIX_FIX]
    if pix is not None:
        frame["gnss_pix"] = {
            "stamp": pix["stamp"], "lat": pix["lat"], "lon": pix["lon"],
            "alt": pix.get("alt", np.full(len(pix["stamp"]), np.nan)),
        }
    return frame


def default_out_path(bag_dir):
    bag_dir = bag_dir.rstrip("/")
    return f"{bag_dir}.metrics.json"


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="Per-leg evaluation: rosbag2 + sidecar -> metrics JSON.")
    ap.add_argument("bag", help="rosbag2 directory for one leg")
    ap.add_argument("--sidecar", default=None,
                    help="paired sidecar JSON (default: <bag>.sidecar.json)")
    ap.add_argument("--out", default=None,
                    help="output metrics JSON (default: <bag>.metrics.json)")
    ap.add_argument("--t-transient", type=float, default=2.0,
                    help="transient window excluded from steady-state RMS [s]")
    ap.add_argument("--k-e", type=float, default=3.0,
                    help="VFG convergence gain for e_psi reconstruction "
                         "(overridden by sidecar controller_tuning.k_e if set)")
    args = ap.parse_args(argv)

    sidecar, sidecar_path = load_sidecar(args.bag, args.sidecar)
    result = evaluate(args.bag, sidecar, t_transient=args.t_transient,
                      k_e=args.k_e)
    result["sidecar_path"] = os.path.abspath(sidecar_path)

    out = args.out or default_out_path(args.bag)
    tmp = out + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(result, f, indent=2)
        f.write("\n")
    os.replace(tmp, out)

    print(f"[run_eval] wrote {out}")
    for w in result["warnings"]:
        print(f"[run_eval] WARNING: {w}")
    # Brief stdout summary.
    for split in ("odom_belief", "rtk_truth"):
        m = result["metrics"].get(split)
        if m is None:
            print(f"[run_eval] {split}: (skipped)")
            continue
        print(f"[run_eval] {split}: "
              f"rms_e_d={m.get('rms_e_d')}, max_e_psi_deg={m.get('max_e_psi_deg')}, "
              f"term_pos={m.get('terminal_pos_err_m')}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
