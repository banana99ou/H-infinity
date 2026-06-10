# -*- coding: utf-8 -*-
"""venue_loader_node — receive an operator-authored venue+legs batch and persist it.

The WebUI "Send to NUC" button publishes the full venue JSON (polygon + anchor +
an ordered list of legs; each leg = a reposition glue curve + a scored experiment
recipe) on ``/venue/load``. This node:

  1. validates it — polygon >=3 corners, >=1 leg, each leg >=1 curve, valid curve
     kinds / controllers / recipes, AND a containment check that EVERY curve fits
     inside the venue polygon minus margins (the whole point: a move is only
     allowed if it was checkable before motion);
  2. atomically persists it to ``scenarios/venues/<name>.json`` and refreshes
     ``scenarios/venues/active.json`` — the single file run_executor + geofence read;
  3. if the polygon changed, auto-restarts the geofence proc (via the
     orchestrator) so the safety boundary reloads.

(There is no leg checkpoint file: run_executor resumes by rescanning the bag
manifest for passing runs; live progress is on /run/status.)

  sub  /venue/load         std_msgs/String  JSON   (the Send-to-NUC payload)
  pub  /venue/loaded       std_msgs/String  JSON (latched)
       {name, n_legs, total_scored, active_venue_file, error}
  pub  /venue/load_status  std_msgs/String  JSON (latched)  {ok, message}
  pub  /orchestrator/{kill,start}  std_msgs/String   (geofence reload on polygon change)
  sub  /orchestrator/status        std_msgs/String   (confirm geofence down before restart)
"""
import json
import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import (
    QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy,
)
from std_msgs.msg import String

try:
    from limo_path_follower import venue_geom
except Exception:  # pragma: no cover - in-source / odd layout fallback
    import venue_geom


_ROOT_CANDIDATES = [p for p in [
    os.environ.get('H_INFINITY_ROOT'),
    '/home/agilex/H-infinity',
    os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                 '..', '..', '..')),
] if p]
_REPO_ROOT = next(
    (p for p in _ROOT_CANDIDATES
     if os.path.isdir(os.path.join(p, 'scenarios', 'venues'))),
    _ROOT_CANDIDATES[-1])

VALID_KINDS = ("reposition", "recipe")
VALID_CONTROLLERS = ("lpv-hinf", "lpv_hinf", "lpv", "hinf", "pid-ff", "pid_ff", "pid")
VALID_RECIPE_TYPES = ("step", "slalom", "uturn")


class VenueLoaderNode(Node):

    def __init__(self):
        super().__init__("venue_loader_node")

        self.declare_parameter(
            "venues_dir", os.path.join(_REPO_ROOT, "scenarios", "venues"))
        self.declare_parameter(
            "active_file",
            os.path.join(_REPO_ROOT, "scenarios", "venues", "active.json"))
        self.declare_parameter("robot_footprint_radius_m", 0.30)
        self.declare_parameter("path_tracking_margin_m", 0.30)
        self.declare_parameter("geofence_proc", "geofence")
        # Auto-restart the geofence proc when the polygon changes (operator chose
        # this over an explicit button). Set false to disable the restart.
        self.declare_parameter("geofence_auto_restart", True)

        self._venues_dir = str(self.get_parameter("venues_dir").value)
        self._active_file = str(self.get_parameter("active_file").value)
        self._footprint_r = float(self.get_parameter("robot_footprint_radius_m").value)
        self._track_margin = float(self.get_parameter("path_tracking_margin_m").value)
        self._geofence_proc = str(self.get_parameter("geofence_proc").value)
        self._geofence_auto = bool(self.get_parameter("geofence_auto_restart").value)

        latched = QoSProfile(
            depth=1, history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        self.pub_loaded = self.create_publisher(String, "/venue/loaded", latched)
        self.pub_load_status = self.create_publisher(
            String, "/venue/load_status", latched)
        self.pub_orch_start = self.create_publisher(String, "/orchestrator/start", 10)
        self.pub_orch_kill = self.create_publisher(String, "/orchestrator/kill", 10)

        self.create_subscription(String, "/venue/load", self._on_load, 10)
        self.create_subscription(
            String, "/orchestrator/status", self._on_orch_status, 10)

        self._orch_status = {}
        # Pending geofence restart: kill sent, awaiting confirmation it is down
        # before publishing start (so it re-reads active.json). (None | float t0)
        self._geofence_restart_t0 = None
        self.create_timer(0.5, self._tick)

        # Publish what's already on disk so a fresh WebUI connect sees it.
        self._publish_loaded_from_disk()
        self.get_logger().info(
            f"venue_loader_node up. active={self._active_file}. "
            f"Listening on /venue/load.")

    # ------------------------------------------------------------------

    def _on_orch_status(self, msg):
        try:
            self._orch_status = json.loads(msg.data) or {}
        except (ValueError, TypeError):
            pass

    def _on_load(self, msg):
        try:
            v = json.loads(msg.data)
        except (ValueError, TypeError) as exc:
            self._load_status(False, f"bad /venue/load JSON: {exc}")
            return
        ok, why = self._validate(v)
        if not ok:
            self._load_status(False, f"validation failed: {why}")
            self.get_logger().warn(f"/venue/load rejected: {why}")
            return

        name = str(v.get("name") or "venue")
        # Detect a polygon change vs the current active venue (for geofence reload).
        polygon_changed = self._polygon_changed(v)

        # Persist: <name>.json + active.json (both atomic).
        try:
            os.makedirs(self._venues_dir, exist_ok=True)
            named = os.path.join(self._venues_dir, self._safe_name(name) + ".json")
            self._atomic_write(named, v)
            self._atomic_write(self._active_file, v)
        except Exception as exc:
            self._load_status(False, f"persist failed: {exc}")
            self.get_logger().error(f"persist failed: {exc}")
            return

        legs = v.get("legs") or []
        stages = v.get("plan_stages") or []
        total_scored = sum(
            1 for lgs in ([legs] if not stages
                          else [st.get("legs") or [] for st in stages])
            for lg in lgs
            for c in (lg.get("curves") or [])
            if str(c.get("kind", "")).lower() == "recipe" and c.get("scored", True))
        self._publish_loaded(name, len(legs), total_scored, error="",
                             n_stages=len(stages))
        staged = f" in {len(stages)} stages" if stages else ""
        self._load_status(
            True, f"loaded '{name}': {len(legs)} legs, {total_scored} scored "
            f"geometries{staged}. Press Start to run.")
        self.get_logger().info(
            f"venue '{name}' persisted: {len(legs)} legs ({total_scored} scored"
            f"{staged}). polygon_changed={polygon_changed}.")

        if polygon_changed and self._geofence_auto:
            self._restart_geofence_if_running()

    # ------------------------------------------------------------------
    # Validation
    # ------------------------------------------------------------------

    @staticmethod
    def _bad_latlon(obj):
        """True unless obj has finite numeric 'lat' and 'lon'. Guards the
        downstream consumers (run_executor goto build, venue_geom) against a
        hand-edited payload crashing them at run time."""
        if not isinstance(obj, dict):
            return True
        try:
            lat = float(obj["lat"])
            lon = float(obj["lon"])
        except (KeyError, TypeError, ValueError):
            return True
        return not (math.isfinite(lat) and math.isfinite(lon))

    def _validate_legs(self, legs, v, label=""):
        """Structural + containment checks for one leg list. ``label``
        prefixes errors so a multi-stage payload pinpoints the bad stage."""
        if not legs:
            return False, f"{label}no legs"
        for li, leg in enumerate(legs):
            curves = leg.get("curves") or []
            if not curves:
                return False, f"{label}leg {leg.get('id', li)} has no curves"
            for ci, c in enumerate(curves):
                kind = str(c.get("kind", "")).lower()
                if kind not in VALID_KINDS:
                    return False, f"{label}leg {li} curve {ci}: bad kind '{kind}'"
                if kind == "reposition":
                    wps = c.get("waypoints_wgs84") or []
                    if len(wps) < 1:
                        return False, (f"{label}leg {li} curve {ci}: "
                                       "reposition needs waypoints")
                    for wi, w in enumerate(wps):
                        if self._bad_latlon(w):
                            return False, (f"{label}leg {li} curve {ci}: "
                                           f"waypoint {wi} lacks finite lat/lon")
                elif kind == "recipe":
                    if not c.get("start_pose"):
                        return False, (f"{label}leg {li} curve {ci}: "
                                       "recipe needs start_pose")
                    if self._bad_latlon(c.get("start_pose")):
                        return False, (f"{label}leg {li} curve {ci}: "
                                       "start_pose lacks finite lat/lon")
                    rt = str((c.get("recipe") or {}).get("type", "")).lower()
                    if rt not in VALID_RECIPE_TYPES:
                        return False, (f"{label}leg {li} curve {ci}: "
                                       f"bad recipe type '{rt}'")
                    ctrl = str(c.get("controller", "lpv-hinf")).lower()
                    if ctrl not in VALID_CONTROLLERS:
                        return False, (f"{label}leg {li} curve {ci}: "
                                       f"bad controller '{ctrl}'")
        # Containment: every curve must fit the polygon minus margins.
        ok, report = venue_geom.check_legs_containment(
            legs, v, self._footprint_r, self._track_margin)
        if not ok:
            return False, f"{label}containment: " + report.replace("\n", " | ")
        return True, "ok"

    def _validate(self, v):
        if not isinstance(v, dict):
            return False, "payload is not an object"
        corners = v.get("corners_wgs84") or []
        if len(corners) < 3:
            return False, "polygon needs >= 3 corners_wgs84"
        ok, why = self._validate_legs(v.get("legs") or [], v)
        if not ok:
            return False, why
        # Auto-planned multi-stage batch: every LATER stage is validated with
        # the same gates NOW — the run_executor re-checks at each stage
        # advance, but a bad stage must be rejected at Send, not at 2 a.m.
        for si, st in enumerate(v.get("plan_stages") or []):
            ok, why = self._validate_legs(
                st.get("legs") or [], v,
                label=f"stage {st.get('name', si + 1)}: ")
            if not ok:
                return False, why
        return True, "ok"

    # ------------------------------------------------------------------
    # Persistence helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _safe_name(s):
        return "".join(c if (c.isalnum() or c in "-._") else "-" for c in str(s))

    @staticmethod
    def _atomic_write(path, obj):
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        tmp = path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(obj, f, indent=2)
            f.write("\n")
        os.replace(tmp, path)

    def _polygon_changed(self, new_v):
        try:
            with open(self._active_file, "r", encoding="utf-8") as f:
                cur = json.load(f)
        except (FileNotFoundError, ValueError):
            return True
        return (cur.get("corners_wgs84") or []) != (new_v.get("corners_wgs84") or [])

    # ------------------------------------------------------------------
    # Geofence reload
    # ------------------------------------------------------------------

    def _geofence_alive(self):
        return bool(self._orch_status.get(self._geofence_proc, False))

    def _restart_geofence_if_running(self):
        if not self._geofence_alive():
            self.get_logger().info(
                "polygon changed but geofence not running — it will read the new "
                "active.json when next started.")
            return
        self.get_logger().warn(
            "polygon changed — restarting geofence to reload the boundary.")
        self.pub_orch_kill.publish(String(data=self._geofence_proc))
        self._geofence_restart_t0 = time.monotonic()

    def _tick(self):
        # Complete a pending geofence restart once it is confirmed down.
        if self._geofence_restart_t0 is None:
            return
        if not self._geofence_alive():
            self.pub_orch_start.publish(String(data=self._geofence_proc))
            self.get_logger().info("geofence restarted on the new polygon.")
            self._geofence_restart_t0 = None
        elif time.monotonic() - self._geofence_restart_t0 > 6.0:
            # Give up waiting for 'down'; force a start anyway (orchestrator
            # ignores a start for an already-alive proc).
            self.pub_orch_start.publish(String(data=self._geofence_proc))
            self.get_logger().warn(
                "geofence restart: kill not confirmed in 6s; sent start anyway.")
            self._geofence_restart_t0 = None

    # ------------------------------------------------------------------
    # Status publishing
    # ------------------------------------------------------------------

    def _publish_loaded_from_disk(self):
        try:
            with open(self._active_file, "r", encoding="utf-8") as f:
                v = json.load(f)
        except (FileNotFoundError, ValueError):
            self._publish_loaded(None, 0, 0, error="no active venue loaded")
            return
        name = str(v.get("name") or "venue")
        legs = v.get("legs") or []
        stages = v.get("plan_stages") or []
        total_scored = sum(
            1 for lgs in ([legs] if not stages
                          else [st.get("legs") or [] for st in stages])
            for lg in lgs
            for c in (lg.get("curves") or [])
            if str(c.get("kind", "")).lower() == "recipe" and c.get("scored", True))
        self._publish_loaded(name, len(legs), total_scored, error="",
                             n_stages=len(stages))

    def _publish_loaded(self, name, n_legs, total_scored, error="", n_stages=0):
        self.pub_loaded.publish(String(data=json.dumps({
            "name": name,
            "n_legs": n_legs,
            "n_stages": n_stages,
            "total_scored": total_scored,
            "active_venue_file": self._active_file,
            "error": error,
        })))

    def _load_status(self, ok, message):
        self.pub_load_status.publish(String(data=json.dumps({
            "ok": bool(ok), "message": message,
        })))


def main(args=None):
    rclpy.init(args=args)
    node = VenueLoaderNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
