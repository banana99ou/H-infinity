# -*- coding: utf-8 -*-
"""Sensor noise generator for simulation.

Author: Suwon Lee, Kookmin University
Created: 2026-03-01
Description: Generates additive white Gaussian noise for heading
    and steering measurements.
"""

import numpy as np


class NoiseGenerator:
    """Sensor noise generator.

    Simulates IMU heading noise and steering encoder noise as
    additive white Gaussian noise (AWGN).

    Parameters
    ----------
    sigma_psi : float
        Heading measurement noise std [rad]. Default 0.03 (~1.7 deg).
    sigma_delta : float
        Steering measurement noise std [rad]. Default 0.015 (~0.9 deg).
    seed : int or None
        Random seed for reproducibility.
    """

    def __init__(self, sigma_psi=0.03, sigma_delta=0.015, seed=None):
        self.sigma_psi = sigma_psi
        self.sigma_delta = sigma_delta
        self._seed = seed
        self.rng = np.random.default_rng(seed)

    def reset(self, seed=None):
        """Reset random state. Uses the initial seed if none is provided."""
        self.rng = np.random.default_rng(seed if seed is not None else self._seed)

    def sample(self):
        """Generate one sample of sensor noise.

        Returns
        -------
        n_psi : float
            Heading noise [rad].
        n_delta : float
            Steering noise [rad].
        """
        n_psi = self.rng.normal(0, self.sigma_psi)
        n_delta = self.rng.normal(0, self.sigma_delta)
        return n_psi, n_delta

    def sample_batch(self, n):
        """Generate n samples.

        Returns
        -------
        noise : ndarray, shape (n, 2)
            Columns: [n_psi, n_delta].
        """
        n_psi = self.rng.normal(0, self.sigma_psi, n)
        n_delta = self.rng.normal(0, self.sigma_delta, n)
        return np.column_stack([n_psi, n_delta])
