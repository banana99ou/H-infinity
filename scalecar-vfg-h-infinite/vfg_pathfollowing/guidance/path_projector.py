# -*- coding: utf-8 -*-
"""Path projection using Newton's method for closest-point queries.

Author: Suwon Lee, Kookmin University
Created: 2026-03-01
Description: Maintains state for efficient sequential closest-point
    queries. Uses previous result as initial guess.
"""

import numpy as np


class PathProjector:
    """Sequential closest-point projector onto a reference path.

    Uses the previous closest-point as initial guess for Newton's method,
    making sequential queries O(1) instead of O(n).

    Parameters
    ----------
    path : PathBase
        Reference path object.
    """

    def __init__(self, path):
        self.path = path
        self._s_prev = None

    def reset(self):
        """Reset internal state."""
        self._s_prev = None

    def project(self, q):
        """Find closest point on path to position q.

        Parameters
        ----------
        q : array_like, shape (2,)
            Query point.

        Returns
        -------
        s_star : float
            Arc-length of closest point.
        p_star : ndarray, shape (2,)
            Position of closest point.
        """
        s_star, p_star = self.path.closest_point(q, s_init=self._s_prev)
        self._s_prev = s_star
        return s_star, p_star

    def compute_errors(self, q, psi):
        """Compute cross-track and heading errors.

        Parameters
        ----------
        q : array_like, shape (2,)
            Vehicle position [x, y].
        psi : float
            Vehicle heading [rad].

        Returns
        -------
        info : dict
            Keys: s_star, p_star, e_d, e_psi, kappa, psi_path.
        """
        s_star, p_star = self.project(q)
        n = self.path.normal(s_star)
        e_d = np.dot(np.asarray(q) - p_star, n)

        psi_path = self.path.heading(s_star)
        e_psi = _wrap_angle(psi - psi_path)

        kappa = self.path.curvature(s_star)

        return {
            "s_star": s_star,
            "p_star": p_star,
            "e_d": e_d,
            "e_psi": e_psi,
            "kappa": kappa,
            "psi_path": psi_path,
        }


def _wrap_angle(angle):
    """Wrap angle to [-pi, pi]."""
    return (angle + np.pi) % (2 * np.pi) - np.pi
