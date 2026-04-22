# -*- coding: utf-8 -*-
"""Kinematic bicycle model for LIMO scale car.

Author: Suwon Lee, Kookmin University
Created: 2026-03-01
Description: Kinematic bicycle model for low-speed path following.

The LIMO is a 1:10 scale Ackermann-steered mobile robot.
States: [X, Y, psi, v, delta] (full) or [psi, delta] (heading only).
Key parameters:
  - Wheelbase L = 0.2 m
  - Steering time constant tau_delta = 0.07 s
  - Max steering angle: 0.5 rad (~28 deg)
"""

import numpy as np


class KinematicBicycle:
    """Kinematic bicycle model for LIMO robot.

    Two operating modes:
    - Full 5-state model: [X, Y, psi, v, delta] for simulation
    - Heading 2-state model: [psi, delta] for controller design

    Parameters
    ----------
    params : dict or None
        Override default LIMO parameters. Keys: L, l_f, l_r, tau_delta, v0.
    """

    DEFAULT_PARAMS = {
        "L": 0.200,          # wheelbase [m]
        "l_f": 0.100,        # front axle to CG [m]
        "l_r": 0.100,        # rear axle to CG [m]
        "tau_delta": 0.07,   # steering actuator time constant [s]
        "v0": 1.0,           # nominal longitudinal speed [m/s]
    }

    def __init__(self, params=None):
        p = dict(self.DEFAULT_PARAMS)
        if params:
            p.update(params)
        self.L = p["L"]
        self.l_f = p["l_f"]
        self.l_r = p["l_r"]
        self.tau_delta = p["tau_delta"]
        self.v0 = p["v0"]

    def derivatives(self, t, x, u, v=None):
        """Full 5-state nonlinear ODE: dx/dt = f(x, u).

        Parameters
        ----------
        t : float
            Time (for ODE solver compatibility).
        x : array_like, shape (5,)
            State [X, Y, psi, v, delta].
        u : array_like, shape (2,)
            Input [a_cmd, delta_cmd].
        v : float or None
            Override longitudinal speed (ignore x[3]).

        Returns
        -------
        dx : ndarray, shape (5,)
        """
        X, Y, psi, vel, delta = x
        a_cmd, delta_cmd = u

        if v is not None:
            vel = v

        beta = np.arctan(self.l_r / self.L * np.tan(delta))

        dX = vel * np.cos(psi + beta)
        dY = vel * np.sin(psi + beta)
        dpsi = vel / self.L * np.cos(beta) * np.tan(delta)
        dv = a_cmd
        ddelta = (delta_cmd - delta) / self.tau_delta

        return np.array([dX, dY, dpsi, dv, ddelta])

    def derivatives_heading(self, t, x, u, v=None):
        """Heading 2-state ODE: x = [psi, delta].

        Parameters
        ----------
        t : float
        x : array_like, shape (2,)
            State [psi, delta].
        u : float
            Steering command delta_cmd.
        v : float or None
            Longitudinal speed. Uses self.v0 if None.

        Returns
        -------
        dx : ndarray, shape (2,)
        """
        psi, delta = x
        delta_cmd = u
        vel = v if v is not None else self.v0

        dpsi = vel / self.L * np.tan(delta)
        ddelta = (delta_cmd - delta) / self.tau_delta

        return np.array([dpsi, ddelta])
