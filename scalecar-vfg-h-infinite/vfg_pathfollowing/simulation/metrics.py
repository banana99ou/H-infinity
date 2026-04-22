# -*- coding: utf-8 -*-
"""Performance metrics for path-following evaluation.

Author: Suwon Lee, Kookmin University
Created: 2026-03-01
Description: Computes RMS errors, max errors, and settling time
    from simulation results.
"""

import numpy as np


def compute_metrics(result, t_transient=2.0):
    """Compute path-following performance metrics.

    Parameters
    ----------
    result : SimResult
        Simulation result.
    t_transient : float
        Transient period to exclude from steady-state metrics [s].

    Returns
    -------
    metrics : dict
        rms_e_psi_deg : RMS heading error (steady-state) [deg]
        rms_e_d       : RMS cross-track error (steady-state) [m]
        max_e_psi_deg : Max heading error [deg]
        max_e_d       : Max cross-track error [m]
        settling_time : Time to reach |e_psi| < 2 deg permanently [s]
    """
    t = result.time
    e_psi = result.e_psi
    e_d = result.e_d

    # Steady-state mask
    ss_mask = t >= t_transient
    if not np.any(ss_mask):
        import warnings
        warnings.warn(
            f"t_transient={t_transient}s exceeds simulation time "
            f"T={t[-1]:.1f}s. Using full time range.",
            stacklevel=2,
        )
        ss_mask = np.ones_like(t, dtype=bool)

    metrics = {}

    if len(e_psi) > 0:
        e_psi_deg = np.degrees(e_psi)
        metrics['rms_e_psi_deg'] = float(np.sqrt(np.mean(e_psi_deg[ss_mask]**2)))
        metrics['max_e_psi_deg'] = float(np.max(np.abs(e_psi_deg)))

        # Settling time: last time |e_psi| exceeds 2 deg
        threshold = 2.0  # deg
        above = np.abs(e_psi_deg) > threshold
        if np.any(above):
            last_above_idx = np.where(above)[0][-1]
            if last_above_idx < len(t) - 1:
                metrics['settling_time'] = float(t[last_above_idx + 1])
            else:
                metrics['settling_time'] = float(t[-1])
        else:
            metrics['settling_time'] = 0.0

    if len(e_d) > 0:
        metrics['rms_e_d'] = float(np.sqrt(np.mean(e_d[ss_mask]**2)))
        metrics['max_e_d'] = float(np.max(np.abs(e_d)))

    return metrics
