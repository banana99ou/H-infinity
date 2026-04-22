# -*- coding: utf-8 -*-
"""Abstract base class for path-following controllers.

Author: Suwon Lee, Kookmin University
Created: 2026-03-01
Description: Defines the common interface for all steering controllers.
"""

from abc import ABC, abstractmethod


class ControllerBase(ABC):
    """Abstract base class for steering controllers.

    All controllers must implement compute() and reset().
    The compute() method receives heading error e_psi (= psi_des - psi)
    and returns a steering command delta_cmd.
    """

    @abstractmethod
    def compute(self, e_psi, **kwargs):
        """Compute steering command.

        Parameters
        ----------
        e_psi : float
            Heading error [rad]. Convention: e_psi = psi_des - psi.
        **kwargs
            Controller-specific arguments (kappa, rho, dt, etc.).

        Returns
        -------
        delta_cmd : float
            Steering command [rad].
        """

    @abstractmethod
    def reset(self):
        """Reset internal controller state."""
