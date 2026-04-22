# -*- coding: utf-8 -*-
"""Slalom path: alternating left-right arcs with straight segments.

Author: Suwon Lee, Kookmin University
Created: 2026-03-01
Description: Slalom path with alternating curvature sign changes.
    No self-intersection, suitable for testing curvature sign transition.
"""

import numpy as np
from .path_base import PathBase


class SlalomPath(PathBase):
    """Slalom path with alternating curvature sign.

    Straight(L1) -> Arc(left) -> Straight(L_mid) -> Arc(right) -> ... -> Straight(L_end)

    Parameters
    ----------
    R : float
        Arc radius [m]. Default 0.5 (kappa = +/-2.0).
    theta_arc : float
        Arc angle [rad]. Default pi/2.
    L1 : float
        Initial straight length [m]. Default 5.0.
    L_mid : float
        Straight length between arcs [m]. Default 2.0.
    n_arcs : int
        Number of arcs (alternating left/right). Default 6.
    L_end : float
        Final straight length [m]. Default 25.0.
    """

    def __init__(self, R=0.5, theta_arc=np.pi / 2, L1=5.0, L_mid=2.0,
                 n_arcs=6, L_end=25.0):
        self.R = R
        self.theta_arc = theta_arc
        self.L1 = L1
        self.L_mid = L_mid
        self.n_arcs = n_arcs
        self.L_end = L_end

        arc_len = R * theta_arc

        # Build segment list: (type, length, direction)
        self._segments = []
        self._segments.append(('line', L1, 0))
        for i in range(n_arcs):
            direction = 1 if (i % 2 == 0) else -1
            self._segments.append(('arc', arc_len, direction))
            if i < n_arcs - 1:
                self._segments.append(('line', L_mid, 0))
        self._segments.append(('line', L_end, 0))

        # Cumulative arc-length boundaries
        self._s_bounds = np.zeros(len(self._segments) + 1)
        for k, (_, length, _) in enumerate(self._segments):
            self._s_bounds[k + 1] = self._s_bounds[k] + length
        self._total = self._s_bounds[-1]

        # Precompute start position + heading for each segment
        self._seg_pos = np.zeros((len(self._segments), 2))
        self._seg_hdg = np.zeros(len(self._segments))
        self._seg_pos[0] = np.array([0.0, 0.0])
        self._seg_hdg[0] = 0.0

        for k in range(len(self._segments) - 1):
            seg_type, seg_len, seg_dir = self._segments[k]
            pos_k = self._seg_pos[k]
            hdg_k = self._seg_hdg[k]

            if seg_type == 'line':
                end_pos = pos_k + seg_len * np.array([np.cos(hdg_k), np.sin(hdg_k)])
                end_hdg = hdg_k
            else:
                n_vec = np.array([-np.sin(hdg_k), np.cos(hdg_k)])
                center = pos_k + seg_dir * R * n_vec
                delta_hdg = seg_dir * theta_arc
                end_hdg = hdg_k + delta_hdg
                start_angle = np.arctan2(pos_k[1] - center[1],
                                         pos_k[0] - center[0])
                end_angle = start_angle + seg_dir * theta_arc
                end_pos = center + R * np.array([np.cos(end_angle),
                                                  np.sin(end_angle)])

            self._seg_pos[k + 1] = end_pos
            self._seg_hdg[k + 1] = end_hdg

    @property
    def total_length(self):
        return self._total

    def _find_segment(self, s):
        """Find segment index and local arc-length for given s."""
        s = np.clip(s, 0, self._total)
        idx = np.searchsorted(self._s_bounds[1:], s, side='right')
        idx = min(idx, len(self._segments) - 1)
        local_s = s - self._s_bounds[idx]
        return idx, local_s

    def position(self, s):
        idx, sl = self._find_segment(s)
        seg_type, _, seg_dir = self._segments[idx]
        pos0 = self._seg_pos[idx]
        hdg0 = self._seg_hdg[idx]

        if seg_type == 'line':
            return pos0 + sl * np.array([np.cos(hdg0), np.sin(hdg0)])
        else:
            n_vec = np.array([-np.sin(hdg0), np.cos(hdg0)])
            center = pos0 + seg_dir * self.R * n_vec
            start_angle = np.arctan2(pos0[1] - center[1],
                                     pos0[0] - center[0])
            angle = start_angle + seg_dir * sl / self.R
            return center + self.R * np.array([np.cos(angle), np.sin(angle)])

    def heading(self, s):
        idx, sl = self._find_segment(s)
        seg_type, _, seg_dir = self._segments[idx]
        hdg0 = self._seg_hdg[idx]

        if seg_type == 'line':
            return hdg0
        else:
            return hdg0 + seg_dir * sl / self.R

    def tangent(self, s):
        hdg = self.heading(s)
        return np.array([np.cos(hdg), np.sin(hdg)])

    def normal(self, s):
        t = self.tangent(s)
        return np.array([-t[1], t[0]])

    def curvature(self, s):
        idx, _ = self._find_segment(s)
        seg_type, _, seg_dir = self._segments[idx]
        if seg_type == 'arc':
            return seg_dir / self.R
        return 0.0
