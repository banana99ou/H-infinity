# -*- coding: utf-8 -*-
"""Sinusoidal path: y = A * sin(omega * x), arc-length parameterized.

Author: Suwon Lee, Kookmin University
Created: 2026-03-01
Description: Continuously varying curvature path for steady-state
    tracking performance evaluation.
"""

import numpy as np
from scipy.integrate import cumulative_trapezoid
from .path_base import PathBase


class SinusoidalPath(PathBase):
    """Sinusoidal path y = A * sin(omega * x).

    Arc-length parameterization via numerical integration.

    Parameters
    ----------
    A : float
        Amplitude [m]. Default 1.0.
    omega : float
        Spatial frequency [rad/m]. Default pi/5.
    x_end : float
        End x-coordinate [m]. Default 20.0.
    n_points : int
        Number of discretization points. Default 2000.
    """

    def __init__(self, A=1.0, omega=np.pi / 5, x_end=20.0, n_points=2000):
        self.A = A
        self.omega = omega

        self._x_pts = np.linspace(0, x_end, n_points)

        # dy/dx = A * omega * cos(omega * x)
        dydx = A * omega * np.cos(omega * self._x_pts)
        ds_dx = np.sqrt(1.0 + dydx**2)

        self._s_pts = np.zeros(n_points)
        self._s_pts[1:] = cumulative_trapezoid(ds_dx, self._x_pts)
        self._total_length = self._s_pts[-1]

    @property
    def total_length(self):
        return self._total_length

    def _x_from_s(self, s):
        """Inverse map: arc-length s -> x-coordinate."""
        s = np.clip(s, 0, self._total_length)
        return np.interp(s, self._s_pts, self._x_pts)

    def position(self, s):
        x = self._x_from_s(s)
        y = self.A * np.sin(self.omega * x)
        return np.array([x, y])

    def tangent(self, s):
        x = self._x_from_s(s)
        dydx = self.A * self.omega * np.cos(self.omega * x)
        norm = np.sqrt(1.0 + dydx**2)
        return np.array([1.0 / norm, dydx / norm])

    def normal(self, s):
        t = self.tangent(s)
        return np.array([-t[1], t[0]])

    def curvature(self, s):
        x = self._x_from_s(s)
        dydx = self.A * self.omega * np.cos(self.omega * x)
        d2ydx2 = -self.A * self.omega**2 * np.sin(self.omega * x)
        kappa = d2ydx2 / (1.0 + dydx**2)**1.5
        return kappa

    def heading(self, s):
        t = self.tangent(s)
        return np.arctan2(t[1], t[0])
