# -*- coding: utf-8 -*-
"""Abstract base class for reference paths.

Author: Suwon Lee, Kookmin University
Created: 2026-03-01
Description: Abstract base class for parametric curves P(s) in 2D,
    parameterized by arc-length s in [0, total_length].
"""

from abc import ABC, abstractmethod
import numpy as np


class PathBase(ABC):
    """Base class for parametric curves P(s) in 2D.

    All paths are parameterized by arc-length s in [0, total_length].
    Subclasses must implement: position, tangent, normal, curvature,
    heading, and total_length.
    """

    @abstractmethod
    def position(self, s):
        """Position on the path at arc-length s.

        Returns
        -------
        pos : ndarray, shape (2,)
        """

    @abstractmethod
    def tangent(self, s):
        """Unit tangent vector at arc-length s.

        Returns
        -------
        t : ndarray, shape (2,)
        """

    @abstractmethod
    def normal(self, s):
        """Unit normal vector at arc-length s (left-hand side).

        Returns
        -------
        n : ndarray, shape (2,)
        """

    @abstractmethod
    def curvature(self, s):
        """Signed curvature at arc-length s.

        Returns
        -------
        kappa : float
        """

    @abstractmethod
    def heading(self, s):
        """Path heading angle at arc-length s.

        Returns
        -------
        psi : float [rad]
        """

    @property
    @abstractmethod
    def total_length(self):
        """Total arc-length of the path.

        Returns
        -------
        L : float
        """

    def curvature_derivative(self, s):
        """Derivative of curvature with respect to arc-length: dkappa/ds.

        Default implementation uses central finite differences.
        Subclasses may override with analytical expressions.
        """
        ds = 1e-5
        L = self.total_length
        s_lo = max(0, s - ds)
        s_hi = min(L, s + ds)
        return (self.curvature(s_hi) - self.curvature(s_lo)) / (s_hi - s_lo)

    def closest_point(self, q, s_init=None, tol=1e-8, max_iter=20):
        """Find the closest point on the path to position q using Newton's method.

        Parameters
        ----------
        q : array_like, shape (2,)
            Query point.
        s_init : float or None
            Initial guess for arc-length. If None, uses coarse grid search.
        tol : float
            Convergence tolerance.
        max_iter : int
            Maximum Newton iterations.

        Returns
        -------
        s_star : float
            Arc-length parameter of closest point.
        p_star : ndarray, shape (2,)
            Position of closest point.
        """
        q = np.asarray(q, dtype=float)
        L = self.total_length

        if s_init is None:
            n_grid = 100
            s_grid = np.linspace(0, L, n_grid)
            dists = np.array([np.linalg.norm(q - self.position(s)) for s in s_grid])
            s_init = s_grid[np.argmin(dists)]

        s = s_init
        for _ in range(max_iter):
            p = self.position(s)
            t = self.tangent(s)
            diff = q - p
            f_val = np.dot(diff, t)
            n = self.normal(s)
            kappa = self.curvature(s)
            f_prime = -1.0 + kappa * np.dot(diff, n)

            if abs(f_prime) < 1e-12:
                break
            ds = -f_val / f_prime
            s = np.clip(s + ds, 0, L)

            if abs(ds) < tol:
                break

        return s, self.position(s)

    def signed_distance(self, q, s_init=None):
        """Compute signed cross-track error.

        Positive = left of path, negative = right of path.

        Parameters
        ----------
        q : array_like, shape (2,)
        s_init : float or None

        Returns
        -------
        e_d : float
            Signed distance.
        s_star : float
            Arc-length of closest point.
        """
        q = np.asarray(q, dtype=float)
        s_star, p_star = self.closest_point(q, s_init)
        n = self.normal(s_star)
        e_d = np.dot(q - p_star, n)
        return e_d, s_star
