# -*- coding: utf-8 -*-
"""Vector Field Guidance (VFG) with tanh weighting.

Author: Suwon Lee, Kookmin University
Created: 2026-03-01
Description: Computes desired heading by blending convergence and
    traversal vectors using a tanh weighting scheme.

The guidance law creates a vector field around the reference path:
  v_des = k_conv(d) * v_conv + k_trav(d) * v_trav
where k_conv = tanh(k_e * d), k_trav = sqrt(1 - k_conv^2).

Far from the path, the vehicle converges toward it.
Near the path, the vehicle follows it along the tangent direction.
"""

import numpy as np
from .path_projector import PathProjector


class VectorFieldGuidance:
    """Vector Field Guidance with tanh weighting.

    Parameters
    ----------
    path : PathBase
        Reference path.
    k_e : float
        Convergence gain. Higher values make the vehicle converge
        more aggressively. Default 3.0.
    """

    def __init__(self, path, k_e=3.0):
        self.path = path
        self.k_e = k_e
        self._projector = PathProjector(path)

    def reset(self):
        """Reset internal projector state."""
        self._projector.reset()

    def compute(self, q, psi_vehicle):
        """Compute VFG guidance command.

        Parameters
        ----------
        q : array_like, shape (2,)
            Vehicle position [x, y].
        psi_vehicle : float
            Vehicle heading [rad].

        Returns
        -------
        result : dict
            psi_des : desired heading [rad]
            kappa   : path curvature at closest point [1/m]
            e_d     : signed cross-track error [m]
            s_star  : arc-length of closest point [m]
            k_conv  : convergence weight [-]
            k_trav  : traversal weight [-]
        """
        q = np.asarray(q, dtype=float)
        s_star, p_star = self._projector.project(q)

        diff = q - p_star
        d = np.linalg.norm(diff)
        if d < 1e-10:
            d = 1e-10

        # Weighting coefficients
        k_conv = np.tanh(self.k_e * d)
        k_trav = np.sqrt(max(1.0 - k_conv**2, 0.0))

        # Direction vectors
        n = self.path.normal(s_star)
        e_d = np.dot(diff, n)
        v_conv = -np.sign(e_d) * n if abs(e_d) > 1e-10 else np.zeros(2)
        v_trav = self.path.tangent(s_star)

        # Desired velocity direction
        v_des = k_conv * v_conv + k_trav * v_trav
        norm_v = np.linalg.norm(v_des)
        if norm_v > 1e-10:
            v_des /= norm_v

        psi_des = np.arctan2(v_des[1], v_des[0])
        kappa = self.path.curvature(s_star)

        return {
            "psi_des": psi_des,
            "kappa": kappa,
            "e_d": e_d,
            "s_star": s_star,
            "k_conv": k_conv,
            "k_trav": k_trav,
        }
