#!/usr/bin/env python3
"""Run-time venue geofence (safety watchdog).

The follower has no area guard during a recorded leg, and reposition's guard is
killed before the run — so nothing stops the robot before the venue/roof edge if a
leg drives off-axis. This node watches the live RTK fix and latches a hard E-stop
(`/estop_trigger` True, consumed by estop_cli) the moment the robot leaves the
venue polygon, or if RTK goes stale/blind (failsafe).

Reads the polygon from the venue JSON (`corners_wgs84`) so it tracks the venue in
use. Intended to run as a managed orchestrator PROC during autonomous legs.

  ros2 run ... OR: python3 geofence_watchdog.py --venue <venue.json>
"""
import argparse
import json
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool

_DEFAULT_VENUE = "/home/agilex/H-infinity/scenarios/venues/rooftop.json"
_FIX_TOPIC = "/gps_rtk_f9p_helical/gps/fix"


def _load_polygon(venue_path):
    """Return [(lat, lon), ...] from the venue's corners_wgs84."""
    with open(venue_path, "r", encoding="utf-8") as fh:
        v = json.load(fh)
    corners = v.get("corners_wgs84") or []
    poly = [(float(c["lat"]), float(c["lon"])) for c in corners]
    if len(poly) < 3:
        raise ValueError(f"venue {venue_path} has < 3 corners_wgs84")
    return poly


def point_in_polygon(lon, lat, poly):
    """Ray casting. poly is [(lat, lon), ...]; x=lon, y=lat."""
    inside = False
    n = len(poly)
    j = n - 1
    for i in range(n):
        yi, xi = poly[i]
        yj, xj = poly[j]
        if ((yi > lat) != (yj > lat)) and \
                (lon < (xj - xi) * (lat - yi) / (yj - yi) + xi):
            inside = not inside
        j = i
    return inside


class Geofence(Node):
    def __init__(self, poly, venue_path):
        super().__init__("geofence_watchdog")
        self._poly = poly
        self.pub = self.create_publisher(Bool, "/estop_trigger", 10)
        self.create_subscription(NavSatFix, _FIX_TOPIC, self._fix_cb, 10)
        self._last_t = None
        self._tripped = False
        self.create_timer(0.2, self._tick)
        self.get_logger().warn(
            f"GEOFENCE ARMED ({len(poly)}-corner venue {venue_path}, 5 Hz).")

    def _fix_cb(self, msg):
        self._last_t = time.time()
        if self._tripped:
            return
        lat, lon = msg.latitude, msg.longitude
        if lat != lat or lon != lon:      # NaN
            return
        if not point_in_polygon(lon, lat, self._poly):
            self._trip("LEFT VENUE POLYGON lat=%.7f lon=%.7f" % (lat, lon))

    def _tick(self):
        if self._tripped:
            self.pub.publish(Bool(data=True))   # keep latched
            return
        if self._last_t is not None and (time.time() - self._last_t) > 1.5:
            self._trip("RTK fix stale >1.5s -> blind geofence (failsafe estop)")

    def _trip(self, reason):
        self._tripped = True
        self.get_logger().error("GEOFENCE ESTOP: " + reason)
        for _ in range(10):
            self.pub.publish(Bool(data=True))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--venue", default=_DEFAULT_VENUE)
    args, _ = ap.parse_known_args()
    poly = _load_polygon(args.venue)
    rclpy.init()
    node = Geofence(poly, args.venue)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
