#!/usr/bin/env python3
"""Shared QC helpers that deliberately avoid ROS imports.

These helpers are used by both laptop-safe pytest tests and NUC-side harnesses.
Anything in this module must stay runnable on macOS without ROS installed.
"""

from __future__ import annotations

import json
import math
import os
import re
from dataclasses import dataclass
from typing import Iterable, Optional


REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
SYSTEM_SPEC = os.path.join(REPO_ROOT, "DOC", "system_spec.md")


@dataclass(frozen=True)
class RtkStatus:
    quality: Optional[int]
    fixed: bool
    float: bool
    rtcm_stale: bool
    fwd_age_s: Optional[float]
    raw: str


def parse_rtk_status(text: str) -> RtkStatus:
    """Parse the repo's human-readable RTK status string.

    The rover driver publishes quality in a String, e.g. ``quality=4``. This is
    the authoritative RTK gate; NavSatFix.status cannot distinguish FIXED vs
    FLOAT on the F9P path.
    """
    text = text or ""
    q = None
    m = re.search(r"\bquality\s*=\s*(-?\d+)", text)
    if m:
        q = int(m.group(1))
    age = None
    m = re.search(r"\bfwd_age\s*=\s*([0-9.]+)\s*s", text)
    if m:
        age = float(m.group(1))
    stale = "RTCM: STALE" in text or (age is not None and age > 5.0)
    return RtkStatus(
        quality=q,
        fixed=(q == 4),
        float=(q == 5),
        rtcm_stale=stale,
        fwd_age_s=age,
        raw=text,
    )


def point_in_polygon(pt: tuple[float, float], poly: list[tuple[float, float]]) -> bool:
    """Ray-cast point-in-polygon test."""
    x, y = pt
    inside = False
    j = len(poly) - 1
    for i, (xi, yi) in enumerate(poly):
        xj, yj = poly[j]
        if ((yi > y) != (yj > y)) and (
            x < (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi
        ):
            inside = not inside
        j = i
    return inside


def segments_intersect(
    a: tuple[float, float],
    b: tuple[float, float],
    c: tuple[float, float],
    d: tuple[float, float],
) -> bool:
    """Return True when line segments ab and cd strictly intersect."""

    def orient(p, q, r):
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])

    o1 = orient(a, b, c)
    o2 = orient(a, b, d)
    o3 = orient(c, d, a)
    o4 = orient(c, d, b)
    return (o1 * o2 < 0.0) and (o3 * o4 < 0.0)


def polygon_self_intersections(poly: list[tuple[float, float]]) -> list[tuple[int, int]]:
    """Return pairs of non-adjacent edges that intersect."""
    out: list[tuple[int, int]] = []
    n = len(poly)
    for i in range(n):
        a, b = poly[i], poly[(i + 1) % n]
        for j in range(i + 1, n):
            if j in (i, (i + 1) % n) or i == (j + 1) % n:
                continue
            c, d = poly[j], poly[(j + 1) % n]
            if segments_intersect(a, b, c, d):
                out.append((i, j))
    return out


def inset_polygon(corners: list[tuple[float, float]], margin: float) -> list[tuple[float, float]]:
    """Same conservative centroid inset used by reposition_node."""
    cx = sum(p[0] for p in corners) / len(corners)
    cy = sum(p[1] for p in corners) / len(corners)
    out = []
    for x, y in corners:
        dx = cx - x
        dy = cy - y
        d = math.hypot(dx, dy)
        if d < 1e-9:
            out.append((x, y))
        else:
            out.append((x + dx / d * margin, y + dy / d * margin))
    return out


def seg_in_polygon(
    p0: tuple[float, float],
    p1: tuple[float, float],
    poly: list[tuple[float, float]],
    n_samples: int = 24,
) -> bool:
    for i in range(n_samples + 1):
        t = i / n_samples
        p = (p0[0] + (p1[0] - p0[0]) * t, p0[1] + (p1[1] - p0[1]) * t)
        if not point_in_polygon(p, poly):
            return False
    return True


def seg_clears_circles(
    p0: tuple[float, float],
    p1: tuple[float, float],
    circles: Iterable[tuple[float, float, float]],
    n_samples: int = 24,
) -> bool:
    for i in range(n_samples + 1):
        t = i / n_samples
        x = p0[0] + (p1[0] - p0[0]) * t
        y = p0[1] + (p1[1] - p0[1]) * t
        for cx, cy, r in circles:
            if math.hypot(x - cx, y - cy) <= r:
                return False
    return True


def load_json(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def required_bag_topics_from_spec() -> set[str]:
    """Extract required bag topics from the locked system spec text.

    Kept intentionally simple: this is a contract smoke test, not a Markdown
    parser. It catches accidental drops from Data_Logger.TOPICS.
    """
    with open(SYSTEM_SPEC, "r", encoding="utf-8") as f:
        text = f.read()
    topics = set(re.findall(r"`(/[^`{} ,]+(?:/\{[^`]+?\})?)`", text))
    expanded: set[str] = set()
    for t in topics:
        if "{fix,nmea,rtk_status}" in t:
            prefix = t.split("{", 1)[0]
            expanded.update({prefix + "fix", prefix + "nmea", prefix + "rtk_status"})
        elif "{fix,satellites}" in t:
            prefix = t.split("{", 1)[0]
            expanded.update({prefix + "fix", prefix + "satellites"})
        else:
            expanded.add(t)
    return expanded
