# -*- coding: utf-8 -*-
"""Step curvature path: straight -> circular arc -> straight.

Author: Suwon Lee, Kookmin University
Created: 2026-03-01
Description: Path with an abrupt curvature transition, useful for
    testing transient response of path-following controllers.
"""

import numpy as np
from .path_base import PathBase


class StepCurvaturePath(PathBase):
    """Path with step curvature transition.

    Segments:
      1. Straight segment of length L1
      2. Circular arc of radius R, angle theta_arc
      3. Straight segment of length L2

    Parameters
    ----------
    L1 : float
        Length of first straight [m]. Default 5.0.
    R : float
        Radius of circular arc [m]. Default 0.5.
    theta_arc : float
        Arc angle [rad]. Default pi/2 (90 degrees).
    L2 : float
        Length of second straight [m]. Default 5.0.
    direction : int
        1 for left turn, -1 for right turn. Default 1.
    """

    def __init__(self, L1=5.0, R=0.5, theta_arc=np.pi / 2, L2=5.0, direction=1):
        self.L1 = L1
        self.R = R
        self.theta_arc = theta_arc
        self.L2 = L2
        self.dir = direction
        self._arc_length = R * theta_arc
        self._total = L1 + self._arc_length + L2

        # Arc center
        self._arc_center = np.array([L1, direction * R])
        # Heading at end of arc
        self._heading_after_arc = direction * theta_arc
        # Position at end of arc
        self._pos_end_arc = self._arc_center + R * np.array([
            np.sin(direction * theta_arc),
            -direction * np.cos(direction * theta_arc),
        ])

    @property
    def total_length(self):
        return self._total

    def _segment(self, s):
        """Determine which segment s falls in. Returns (seg_id, local_s)."""
        s = np.clip(s, 0, self._total)
        if s <= self.L1:
            return 0, s
        elif s <= self.L1 + self._arc_length:
            return 1, s - self.L1
        else:
            return 2, s - self.L1 - self._arc_length

    def position(self, s):
        seg, sl = self._segment(s)
        if seg == 0:
            return np.array([sl, 0.0])
        elif seg == 1:
            angle = self.dir * sl / self.R
            return self._arc_center + self.R * np.array([
                np.sin(angle),
                -self.dir * np.cos(angle),
            ])
        else:
            t = np.array([
                np.cos(self._heading_after_arc),
                np.sin(self._heading_after_arc),
            ])
            return self._pos_end_arc + sl * t

    def tangent(self, s):
        seg, sl = self._segment(s)
        if seg == 0:
            return np.array([1.0, 0.0])
        elif seg == 1:
            hdg = self.dir * sl / self.R
            return np.array([np.cos(hdg), np.sin(hdg)])
        else:
            hdg = self._heading_after_arc
            return np.array([np.cos(hdg), np.sin(hdg)])

    def normal(self, s):
        t = self.tangent(s)
        return np.array([-t[1], t[0]])

    def curvature(self, s):
        seg, _ = self._segment(s)
        if seg == 1:
            return self.dir / self.R
        return 0.0

    def heading(self, s):
        seg, sl = self._segment(s)
        if seg == 0:
            return 0.0
        elif seg == 1:
            return self.dir * sl / self.R
        else:
            return self._heading_after_arc
