# -*- coding: utf-8 -*-
"""Bezier path: G1-continuous cubic Bezier spline through waypoints.

Author: Suwon Lee, Kookmin University
Created: 2026-03-01
Description: Cubic Bezier spline with automatic control point generation,
    analytic curvature, and arc-length parameterization.
"""

import numpy as np
from .path_base import PathBase


class BezierPath(PathBase):
    """G1-continuous cubic Bezier spline through waypoints.

    Generates n-1 cubic Bezier segments from n waypoints with
    automatic tangent-based control point placement.

    Parameters
    ----------
    waypoints : list of tuple (x, y)
        At least 2 waypoints.
    tension : float
        Control point tension [0, 0.5]. Default 0.33 (Catmull-Rom).
        Lower values produce tighter curves.
    n_samples_per_segment : int
        Points per segment for arc-length table. Default 500.
    """

    def __init__(self, waypoints, tension=0.33, n_samples_per_segment=500):
        pts = np.array(waypoints, dtype=float)
        if pts.shape[0] < 2:
            raise ValueError("Need at least 2 waypoints")

        self._n_seg = pts.shape[0] - 1
        self._tension = tension

        # Compute tangent directions at each waypoint
        tangents = self._compute_tangents(pts)

        # Build cubic Bezier segments: each has 4 control points [B0, B1, B2, B3]
        self._segments = []
        for i in range(self._n_seg):
            P0 = pts[i]
            P3 = pts[i + 1]
            chord = np.linalg.norm(P3 - P0)
            B1 = P0 + tension * chord * tangents[i]
            B2 = P3 - tension * chord * tangents[i + 1]
            self._segments.append(np.array([P0, B1, B2, P3]))

        # Build arc-length lookup table
        self._n_samp = n_samples_per_segment
        self._build_arc_length_table()

    @staticmethod
    def _compute_tangents(pts):
        """Compute unit tangent at each waypoint using neighbor averaging."""
        n = pts.shape[0]
        tangents = np.zeros_like(pts)

        if n == 2:
            d = pts[1] - pts[0]
            t = d / max(np.linalg.norm(d), 1e-12)
            tangents[0] = t
            tangents[1] = t
            return tangents

        # First point: chord direction
        d = pts[1] - pts[0]
        tangents[0] = d / max(np.linalg.norm(d), 1e-12)

        # Interior points: average of neighbor chords
        for i in range(1, n - 1):
            d = pts[i + 1] - pts[i - 1]
            tangents[i] = d / max(np.linalg.norm(d), 1e-12)

        # Last point: chord direction
        d = pts[-1] - pts[-2]
        tangents[-1] = d / max(np.linalg.norm(d), 1e-12)

        return tangents

    def _build_arc_length_table(self):
        """Build cumulative arc-length table using Gauss-Legendre quadrature."""
        # Per-segment arc lengths and cumulative sum
        self._seg_lengths = np.zeros(self._n_seg)
        self._seg_s_start = np.zeros(self._n_seg)

        # Per-segment parameter tables: t_values and cumulative arc lengths
        self._seg_t_tables = []
        self._seg_s_tables = []

        for i in range(self._n_seg):
            t_vals = np.linspace(0, 1, self._n_samp)
            # Compute positions along segment
            positions = np.array([self._eval_segment(i, t) for t in t_vals])
            # Chord lengths between consecutive samples
            diffs = np.diff(positions, axis=0)
            chord_lengths = np.linalg.norm(diffs, axis=1)
            # Cumulative arc length within segment
            s_local = np.zeros(self._n_samp)
            s_local[1:] = np.cumsum(chord_lengths)

            self._seg_lengths[i] = s_local[-1]
            self._seg_t_tables.append(t_vals)
            self._seg_s_tables.append(s_local)

        # Cumulative start positions
        for i in range(1, self._n_seg):
            self._seg_s_start[i] = self._seg_s_start[i - 1] + self._seg_lengths[i - 1]

        self._total_length = self._seg_s_start[-1] + self._seg_lengths[-1]

    def _eval_segment(self, seg_idx, t):
        """Evaluate position on segment seg_idx at parameter t in [0,1]."""
        B = self._segments[seg_idx]
        t1 = 1 - t
        return (t1**3 * B[0] + 3 * t * t1**2 * B[1]
                + 3 * t**2 * t1 * B[2] + t**3 * B[3])

    def _eval_segment_deriv1(self, seg_idx, t):
        """First derivative P'(t) on segment seg_idx."""
        B = self._segments[seg_idx]
        t1 = 1 - t
        return 3 * (t1**2 * (B[1] - B[0])
                     + 2 * t * t1 * (B[2] - B[1])
                     + t**2 * (B[3] - B[2]))

    def _eval_segment_deriv2(self, seg_idx, t):
        """Second derivative P''(t) on segment seg_idx."""
        B = self._segments[seg_idx]
        t1 = 1 - t
        return 6 * (t1 * (B[2] - 2 * B[1] + B[0])
                     + t * (B[3] - 2 * B[2] + B[1]))

    def _s_to_segment(self, s):
        """Convert global arc-length s to (segment_index, local_t).

        Returns
        -------
        seg_idx : int
        t : float in [0, 1]
        """
        s = np.clip(s, 0, self._total_length)

        # Find segment
        seg_idx = np.searchsorted(self._seg_s_start, s, side='right') - 1
        seg_idx = np.clip(seg_idx, 0, self._n_seg - 1)

        s_local = s - self._seg_s_start[seg_idx]
        s_local = np.clip(s_local, 0, self._seg_lengths[seg_idx])

        # Interpolate to find t from local arc length
        t = np.interp(s_local, self._seg_s_tables[seg_idx],
                       self._seg_t_tables[seg_idx])
        return seg_idx, t

    @property
    def total_length(self):
        return self._total_length

    def position(self, s):
        seg_idx, t = self._s_to_segment(s)
        return self._eval_segment(seg_idx, t)

    def tangent(self, s):
        seg_idx, t = self._s_to_segment(s)
        dp = self._eval_segment_deriv1(seg_idx, t)
        norm = np.linalg.norm(dp)
        if norm < 1e-12:
            return np.array([1.0, 0.0])
        return dp / norm

    def normal(self, s):
        t = self.tangent(s)
        return np.array([-t[1], t[0]])

    def heading(self, s):
        seg_idx, t = self._s_to_segment(s)
        dp = self._eval_segment_deriv1(seg_idx, t)
        return np.arctan2(dp[1], dp[0])

    def curvature(self, s):
        """Analytic curvature: kappa = (x'y'' - y'x'') / |P'|^3."""
        seg_idx, t = self._s_to_segment(s)
        dp = self._eval_segment_deriv1(seg_idx, t)
        ddp = self._eval_segment_deriv2(seg_idx, t)

        cross = dp[0] * ddp[1] - dp[1] * ddp[0]
        norm_dp = np.linalg.norm(dp)
        if norm_dp < 1e-12:
            return 0.0
        return cross / norm_dp**3

    @classmethod
    def circle(cls, radius=1.0, n_waypoints=16):
        """Create a circular path approximated by Bezier segments.

        Parameters
        ----------
        radius : float
            Circle radius [m].
        n_waypoints : int
            Number of waypoints on the circle. Default 16.

        Returns
        -------
        BezierPath
        """
        angles = np.linspace(0, 2 * np.pi, n_waypoints, endpoint=False)
        waypoints = [(radius * np.cos(a), radius * np.sin(a)) for a in angles]
        # Close the loop
        waypoints.append(waypoints[0])
        # Use tension that approximates circle well
        tension = 4 / 3 * np.tan(np.pi / n_waypoints) / (2 * np.pi / n_waypoints)
        tension = min(tension, 0.5)
        return cls(waypoints, tension=tension)

    @classmethod
    def figure_eight(cls, radius=2.0):
        """Create a figure-eight path.

        Parameters
        ----------
        radius : float
            Lobe radius [m].

        Returns
        -------
        BezierPath
        """
        r = radius
        waypoints = [
            (0, 0),
            (r, r),
            (2 * r, 0),
            (r, -r),
            (0, 0),
            (-r, r),
            (-2 * r, 0),
            (-r, -r),
            (0, 0),
        ]
        return cls(waypoints, tension=0.4)
