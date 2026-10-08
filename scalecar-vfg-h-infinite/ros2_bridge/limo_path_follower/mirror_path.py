# -*- coding: utf-8 -*-
"""Mirrored (right-turn) analytic paths: ``step_m`` and ``slalom_m``.

The vendor's right step, ``StepCurvaturePath(direction=-1)``, runs backward at
the arc start (a cusp; disabled on the roof 2026-06-12). A right turn is
instead the LEFT path reflected across its own x-axis (the start heading):

    position (x, y)    -> (x, -y)        tangent (tx, ty) -> (tx, -ty)
    heading  psi       -> -psi           curvature kappa  -> -kappa
    left normal n      -> (-n_x, n_y)    (the left normal of the reflected curve)

Arc length, total length and the start pose (origin, heading +x) are
unchanged, so a freshly zeroed odom aligns it exactly like the left path.
closest_point / signed_distance come from PathBase and run on the reflected
methods. Used by the follower (path_follower_node), the geometry gate
(venue_geom) and the planner, so checked geometry == driven geometry.

Matrix bookkeeping (2026-10-08): ``step_m`` / ``slalom_m`` are their own
path families with half the repetitions (5 left + 5 right = 10 per pooled
cell), so a cell's directions stay balanced without new executor logic.
"""
import numpy as np

try:
    from vfg_pathfollowing.paths.path_base import PathBase
except Exception:  # pragma: no cover - geometry tools without vfg
    PathBase = object

MIRROR_SUFFIX = "_m"
MIRRORABLE = ("step", "slalom")
MIRROR_TYPES = tuple(t + MIRROR_SUFFIX for t in MIRRORABLE)


def base_type(ptype):
    """'step_m' -> 'step'; anything else unchanged."""
    p = str(ptype).lower()
    return p[:-len(MIRROR_SUFFIX)] if p in MIRROR_TYPES else p


def is_mirrored(ptype):
    return str(ptype).lower() in MIRROR_TYPES


class MirroredPath(PathBase):
    """``base`` reflected across its local x-axis (a right turn from a left)."""

    def __init__(self, base):
        self.base = base

    @property
    def total_length(self):
        return self.base.total_length

    def position(self, s):
        p = np.asarray(self.base.position(s), dtype=float)
        return np.array([p[0], -p[1]])

    def tangent(self, s):
        t = np.asarray(self.base.tangent(s), dtype=float)
        return np.array([t[0], -t[1]])

    def normal(self, s):
        n = np.asarray(self.base.normal(s), dtype=float)
        return np.array([-n[0], n[1]])

    def curvature(self, s):
        return -float(self.base.curvature(s))

    def heading(self, s):
        return -float(self.base.heading(s))

    def curvature_derivative(self, s):
        return -float(self.base.curvature_derivative(s))
