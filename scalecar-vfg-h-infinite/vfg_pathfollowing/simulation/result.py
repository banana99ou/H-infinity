# -*- coding: utf-8 -*-
"""Simulation result container with plotting utilities.

Author: Suwon Lee, Kookmin University
Created: 2026-03-01
Description: Simplified SimResult dataclass for educational use,
    with built-in 3-panel plot method.
"""

import numpy as np
from dataclasses import dataclass, field


@dataclass
class SimResult:
    """Simulation result container.

    Attributes
    ----------
    time : ndarray
        Time vector [s].
    X, Y : ndarray
        Position trajectory [m].
    psi : ndarray
        Heading angle [rad].
    v : ndarray
        Longitudinal speed [m/s].
    delta : ndarray
        Steering angle [rad].
    psi_des : ndarray
        Desired heading from VFG [rad].
    e_psi : ndarray
        Heading error [rad].
    e_d : ndarray
        Cross-track error [m].
    kappa : ndarray
        Path curvature [1/m].
    rho : ndarray
        Scheduling parameter |kappa|*v.
    delta_cmd : ndarray
        Steering command [rad].
    label : str
        Controller label for plotting.
    """
    time: np.ndarray
    X: np.ndarray
    Y: np.ndarray
    psi: np.ndarray
    v: np.ndarray
    delta: np.ndarray
    psi_des: np.ndarray = field(default_factory=lambda: np.array([]))
    e_psi: np.ndarray = field(default_factory=lambda: np.array([]))
    e_d: np.ndarray = field(default_factory=lambda: np.array([]))
    kappa: np.ndarray = field(default_factory=lambda: np.array([]))
    rho: np.ndarray = field(default_factory=lambda: np.array([]))
    delta_cmd: np.ndarray = field(default_factory=lambda: np.array([]))
    label: str = ""

    def plot(self, path=None, fig=None):
        """Plot 3-panel simulation result.

        Panel 1: XY trajectory (+ reference path if provided)
        Panel 2: Heading error e_psi over time
        Panel 3: Cross-track error e_d over time

        Parameters
        ----------
        path : PathBase or None
            Reference path to overlay on trajectory plot.
        fig : matplotlib.figure.Figure or None
            Existing figure. If None, creates new one.

        Returns
        -------
        fig : matplotlib.figure.Figure
        """
        import matplotlib.pyplot as plt

        if fig is None:
            fig, axes = plt.subplots(1, 3, figsize=(14, 4))
        else:
            axes = fig.subplots(1, 3)

        # Panel 1: XY trajectory
        ax = axes[0]
        if path is not None:
            s_arr = np.linspace(0, path.total_length, 500)
            px = [path.position(s)[0] for s in s_arr]
            py = [path.position(s)[1] for s in s_arr]
            ax.plot(px, py, 'k--', linewidth=1, label='Reference')
        ax.plot(self.X, self.Y, 'b-', linewidth=1.5, label=self.label or 'Vehicle')
        ax.set_xlabel('X [m]')
        ax.set_ylabel('Y [m]')
        ax.set_aspect('equal')
        ax.legend(fontsize=8)
        ax.set_title('Trajectory')

        # Panel 2: Heading error
        ax = axes[1]
        if len(self.e_psi) > 0:
            ax.plot(self.time, np.degrees(self.e_psi), 'b-', linewidth=1)
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('e_psi [deg]')
        ax.set_title('Heading Error')
        ax.grid(True, alpha=0.3)

        # Panel 3: Cross-track error
        ax = axes[2]
        if len(self.e_d) > 0:
            ax.plot(self.time, self.e_d, 'b-', linewidth=1)
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('e_d [m]')
        ax.set_title('Cross-track Error')
        ax.grid(True, alpha=0.3)

        fig.tight_layout()
        return fig
