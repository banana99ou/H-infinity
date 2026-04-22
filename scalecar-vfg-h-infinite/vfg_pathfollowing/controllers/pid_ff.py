# -*- coding: utf-8 -*-
"""PID + curvature feedforward controller (industry baseline).

Author: Suwon Lee, Kookmin University
Created: 2026-03-01
Description: Simple PID controller with curvature feedforward.
    delta_cmd = K_P * e_psi + K_D * de_psi/dt + atan(L * kappa)
"""

import numpy as np
from .base import ControllerBase


class PIDFeedforward(ControllerBase):
    """PID controller with curvature feedforward.

    Parameters
    ----------
    K_P : float
        Proportional gain. Default 2.0.
    K_I : float
        Integral gain. Default 0.0.
    K_D : float
        Derivative gain. Default 0.3.
    L : float
        Wheelbase [m]. Default 0.2.
    delta_max : float
        Max steering [rad]. Default 0.5.
    """

    def __init__(self, K_P=2.0, K_I=0.0, K_D=0.3, L=0.2, delta_max=0.5):
        self.K_P = K_P
        self.K_I = K_I
        self.K_D = K_D
        self.L = L
        self.delta_max = delta_max

        self._e_integral = 0.0
        self._e_prev = 0.0
        self._initialized = False

    def reset(self):
        """Reset internal PID state."""
        self._e_integral = 0.0
        self._e_prev = 0.0
        self._initialized = False

    def compute(self, e_psi, kappa=0.0, dt=0.01, **kwargs):
        """Compute PID+FF steering command.

        Parameters
        ----------
        e_psi : float
            Heading error [rad]. Convention: psi_des - psi.
        kappa : float
            Path curvature [1/m].
        dt : float
            Time step [s].

        Returns
        -------
        delta_cmd : float
            Steering command [rad].
        """
        # Derivative (backward difference)
        if self._initialized:
            e_dot = (e_psi - self._e_prev) / max(dt, 1e-6)
        else:
            e_dot = 0.0
            self._initialized = True

        # Integral
        self._e_integral += e_psi * dt

        # PID
        u_pid = self.K_P * e_psi + self.K_I * self._e_integral + self.K_D * e_dot

        # Feedforward
        u_ff = np.arctan(self.L * kappa)

        delta_cmd = u_pid + u_ff
        delta_cmd = np.clip(delta_cmd, -self.delta_max, self.delta_max)

        self._e_prev = e_psi
        return delta_cmd
