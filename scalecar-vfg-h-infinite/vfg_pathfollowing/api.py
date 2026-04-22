# -*- coding: utf-8 -*-
"""High-level simulation facade for educational use.

Author: Suwon Lee, Kookmin University
Created: 2026-03-01
Description: Provides a simple 5-line interface for running path-following
    simulations and comparing controllers.

Example
-------
>>> from vfg_pathfollowing import Simulator, StepCurvaturePath
>>> path = StepCurvaturePath(R=0.5, theta_arc=1.57)
>>> sim = Simulator(path, controller='lpv-hinf', speed=1.5)
>>> result = sim.run(T=20.0)
>>> result.plot()
"""

import warnings
import numpy as np
from .models.kinematic import KinematicBicycle
from .guidance.vfg import VectorFieldGuidance
from .controllers.lpv_hinf import LPVHinfController
from .controllers.pid_ff import PIDFeedforward
from .simulation.engine import ClosedLoopSimulator
from .simulation.noise_gen import NoiseGenerator
from .simulation.result import SimResult


# Controller display labels
_CONTROLLER_LABELS = {
    'lpv-hinf': 'LPV H-inf',
    'pid-ff': 'PID-FF',
}


class Simulator:
    """High-level simulation facade.

    Parameters
    ----------
    path : PathBase
        Reference path.
    controller : str or ControllerBase
        Controller type: 'lpv-hinf' or 'pid-ff', or a controller instance.
    speed : float
        Constant longitudinal speed [m/s]. Default 1.0.
    k_e : float
        VFG convergence gain. Default 3.0.
    dt : float
        Simulation time step [s]. Default 0.01.
    noise : bool
        Enable sensor noise. Default False.
    noise_seed : int or None
        Random seed for noise generator.
    model_params : dict or None
        Override LIMO vehicle parameters.
    hinf_params : dict or None
        Tuning parameters for LPVHinfController. Keys: K_ff, rho_scale,
        output_gain, delta_max, L. Ignored when a controller instance is passed.
    """

    def __init__(self, path, controller='lpv-hinf', speed=1.0,
                 k_e=3.0, dt=0.01, noise=False, noise_seed=None,
                 model_params=None, hinf_params=None):
        if speed <= 0:
            raise ValueError(f"speed must be positive, got {speed}")
        self.path = path
        self.speed = speed
        self.dt = dt

        # Build vehicle model
        self._model = KinematicBicycle(params=model_params)

        # Build guidance
        self._guidance = VectorFieldGuidance(path, k_e=k_e)

        # Build controller
        if isinstance(controller, str):
            self._controller_key = self._normalize_key(controller)
            self._controller = self._make_controller(controller, dt, hinf_params)
        else:
            if hinf_params is not None:
                warnings.warn(
                    "hinf_params is ignored when a controller instance is provided.",
                    stacklevel=2,
                )
            self._controller_key = type(controller).__name__
            self._controller = controller

        # Build noise generator
        self._noise_gen = NoiseGenerator(seed=noise_seed) if noise else None

    @staticmethod
    def _normalize_key(key):
        """Normalize controller alias to canonical key."""
        key = key.lower().strip()
        if key in ('lpv-hinf', 'lpv_hinf', 'lpv', 'hinf'):
            return 'lpv-hinf'
        elif key in ('pid-ff', 'pid_ff', 'pid'):
            return 'pid-ff'
        else:
            raise ValueError(
                f"Unknown controller '{key}'. "
                f"Choose from: 'lpv-hinf', 'pid-ff'")

    @staticmethod
    def _make_controller(key, dt, hinf_params=None):
        """Create controller from canonical key."""
        key = Simulator._normalize_key(key)
        if key == 'lpv-hinf':
            return LPVHinfController.default(dt=dt, **(hinf_params or {}))
        else:
            return PIDFeedforward()

    def run(self, T=20.0, x0=None, y0=None, psi0=None):
        """Run a single simulation.

        Parameters
        ----------
        T : float
            Simulation duration [s]. Default 20.0.
        x0, y0, psi0 : float or None
            Initial conditions. Default (0, 0, 0).

        Returns
        -------
        result : SimResult
        """
        travel = self.speed * T
        path_len = self.path.total_length
        if travel > path_len * 1.2:
            import warnings
            T_rec = path_len / self.speed + 3.0
            warnings.warn(
                f"Vehicle will travel {travel:.1f}m but path is only "
                f"{path_len:.1f}m long. Consider T <= {T_rec:.0f}s.",
                stacklevel=2,
            )
        label = _CONTROLLER_LABELS.get(self._controller_key,
                                       self._controller_key)
        sim = ClosedLoopSimulator(
            model=self._model,
            guidance=self._guidance,
            controller=self._controller,
            path=self.path,
            noise_gen=self._noise_gen,
            dt_ctrl=self.dt,
        )
        return sim.run(T, self.speed, x0=x0, y0=y0, psi0=psi0, label=label)

    def compare(self, controllers=None, T=20.0, x0=None, y0=None, psi0=None):
        """Compare multiple controllers on the same path and speed.

        Parameters
        ----------
        controllers : list of str or None
            Controller keys to compare. Default ['lpv-hinf', 'pid-ff'].
        T : float
            Simulation duration [s].

        Returns
        -------
        results : dict
            Mapping controller_key -> SimResult.
        """
        if controllers is None:
            controllers = ['lpv-hinf', 'pid-ff']

        results = {}
        for raw_key in controllers:
            key = self._normalize_key(raw_key)
            ctrl = self._make_controller(key, self.dt)
            guidance = VectorFieldGuidance(self.path, k_e=self._guidance.k_e)
            label = _CONTROLLER_LABELS.get(key, key)

            sim = ClosedLoopSimulator(
                model=self._model,
                guidance=guidance,
                controller=ctrl,
                path=self.path,
                noise_gen=self._noise_gen,
                dt_ctrl=self.dt,
            )
            results[key] = sim.run(T, self.speed,
                                   x0=x0, y0=y0, psi0=psi0, label=label)

        return results

    @staticmethod
    def plot_comparison(results, path=None):
        """Plot comparison of multiple simulation results.

        Parameters
        ----------
        results : dict
            Mapping controller_key -> SimResult.
        path : PathBase or None
            Reference path to overlay.

        Returns
        -------
        fig : matplotlib.figure.Figure
        """
        import matplotlib.pyplot as plt

        fig, axes = plt.subplots(1, 3, figsize=(14, 4))

        _default_colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728']
        colors = {}
        for i, key in enumerate(results):
            if key == 'lpv-hinf':
                colors[key] = '#1f77b4'
            elif key == 'pid-ff':
                colors[key] = '#ff7f0e'
            else:
                colors[key] = _default_colors[i % len(_default_colors)]

        # Panel 1: XY trajectory
        ax = axes[0]
        if path is not None:
            s_arr = np.linspace(0, path.total_length, 500)
            px = [path.position(s)[0] for s in s_arr]
            py = [path.position(s)[1] for s in s_arr]
            ax.plot(px, py, 'k--', linewidth=1, label='Reference')

        for key, res in results.items():
            c = colors.get(key, None)
            ax.plot(res.X, res.Y, linewidth=1.5, color=c, label=res.label)

        ax.set_xlabel('X [m]')
        ax.set_ylabel('Y [m]')
        ax.set_aspect('equal')
        ax.legend(fontsize=8)
        ax.set_title('Trajectory')

        # Panel 2: Heading error
        ax = axes[1]
        for key, res in results.items():
            c = colors.get(key, None)
            if len(res.e_psi) > 0:
                ax.plot(res.time, np.degrees(res.e_psi),
                        linewidth=1, color=c, label=res.label)
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('e_psi [deg]')
        ax.set_title('Heading Error')
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

        # Panel 3: Cross-track error
        ax = axes[2]
        for key, res in results.items():
            c = colors.get(key, None)
            if len(res.e_d) > 0:
                ax.plot(res.time, res.e_d,
                        linewidth=1, color=c, label=res.label)
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('e_d [m]')
        ax.set_title('Cross-track Error')
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

        fig.tight_layout()
        return fig
