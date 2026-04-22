# -*- coding: utf-8 -*-
"""Closed-loop simulation engine for path following.

Author: Suwon Lee, Kookmin University
Created: 2026-03-01
Description: Integrates vehicle model, VFG guidance, and controller
    in a discrete-time simulation loop.

Controller dispatch uses isinstance checks (not string comparison)
with closures bound at construction time for clean separation.
"""

import numpy as np
from ..controllers.lpv_hinf import LPVHinfController
from ..controllers.pid_ff import PIDFeedforward
from .result import SimResult


def _wrap_angle(angle):
    """Wrap angle to [-pi, pi]."""
    return (angle + np.pi) % (2 * np.pi) - np.pi


class ClosedLoopSimulator:
    """Closed-loop simulation with discrete-time controller updates.

    Parameters
    ----------
    model : KinematicBicycle
        Vehicle model.
    guidance : VectorFieldGuidance
        VFG guidance law.
    controller : LPVHinfController or PIDFeedforward
        Feedback controller.
    path : PathBase
        Reference path.
    noise_gen : NoiseGenerator or None
        Sensor noise generator.
    dt_ctrl : float
        Controller update period [s]. Default 0.01 (100 Hz).
    """

    def __init__(self, model, guidance, controller, path,
                 noise_gen=None, dt_ctrl=0.01):
        self.model = model
        self.guidance = guidance
        self.controller = controller
        self.path = path
        self.noise_gen = noise_gen
        self.dt_ctrl = dt_ctrl

        # Bind controller call convention at construction time
        self._ctrl_fn = self._build_ctrl_fn(controller)

    @staticmethod
    def _build_ctrl_fn(controller):
        """Build controller call closure based on type.

        Returns a function: (psi_des, psi_meas, delta_meas, kappa, v, rho, dt) -> u
        """
        if isinstance(controller, LPVHinfController):
            def fn(psi_des, psi_meas, delta_meas, kappa, v, rho, dt):
                e_psi = _wrap_angle(psi_des - psi_meas)
                return controller.compute(e_psi, delta_meas, rho, dt, kappa=kappa)
            return fn
        elif isinstance(controller, PIDFeedforward):
            def fn(psi_des, psi_meas, delta_meas, kappa, v, rho, dt):
                e_psi = _wrap_angle(psi_des - psi_meas)
                return controller.compute(e_psi, kappa, dt)
            return fn
        else:
            # Generic fallback: assume compute(e_psi, **kwargs)
            def fn(psi_des, psi_meas, delta_meas, kappa, v, rho, dt):
                e_psi = _wrap_angle(psi_des - psi_meas)
                return controller.compute(e_psi, kappa=kappa, dt=dt)
            return fn

    def run(self, T, v0, x0=None, y0=None, psi0=None, label=""):
        """Run closed-loop simulation.

        Parameters
        ----------
        T : float
            Simulation duration [s].
        v0 : float
            Constant longitudinal speed [m/s].
        x0, y0 : float or None
            Initial position. Default (0, 0).
        psi0 : float or None
            Initial heading [rad]. Default 0.
        label : str
            Label for the result (used in plotting).

        Returns
        -------
        result : SimResult
        """
        dt = self.dt_ctrl
        n_steps = int(T / dt)

        if x0 is None:
            x0 = 0.0
        if y0 is None:
            y0 = 0.0
        if psi0 is None:
            psi0 = 0.0

        # Reset modules
        self.guidance.reset()
        self.controller.reset()
        if self.noise_gen is not None:
            self.noise_gen.reset()

        # Allocate result arrays
        time = np.zeros(n_steps)
        X = np.zeros(n_steps)
        Y = np.zeros(n_steps)
        psi = np.zeros(n_steps)
        vel = np.full(n_steps, v0)
        delta = np.zeros(n_steps)
        psi_des_arr = np.zeros(n_steps)
        e_psi_arr = np.zeros(n_steps)
        e_d_arr = np.zeros(n_steps)
        kappa_arr = np.zeros(n_steps)
        rho_arr = np.zeros(n_steps)
        delta_cmd_arr = np.zeros(n_steps)

        # Initial state: [psi, delta]
        state = np.array([psi0, 0.0])
        pos = np.array([x0, y0])

        for k in range(n_steps):
            t = k * dt
            time[k] = t

            cur_psi, cur_delta = state

            X[k] = pos[0]
            Y[k] = pos[1]
            psi[k] = cur_psi
            delta[k] = cur_delta

            # --- Guidance ---
            guid = self.guidance.compute(pos, cur_psi)
            psi_des = guid['psi_des']
            kappa_val = guid['kappa']
            e_d_val = guid['e_d']
            e_psi_val = _wrap_angle(psi_des - cur_psi)

            psi_des_arr[k] = psi_des
            e_psi_arr[k] = e_psi_val
            e_d_arr[k] = e_d_val
            kappa_arr[k] = kappa_val
            rho_val = abs(kappa_val) * v0
            rho_arr[k] = rho_val

            # --- Measurement with noise ---
            psi_meas = cur_psi
            delta_meas = cur_delta
            if self.noise_gen is not None:
                n_psi, n_delta = self.noise_gen.sample()
                psi_meas += n_psi
                delta_meas += n_delta

            # --- Controller (dispatch via closure) ---
            u = self._ctrl_fn(psi_des, psi_meas, delta_meas,
                              kappa_val, v0, rho_val, dt)
            delta_cmd_arr[k] = u

            # --- Plant dynamics (Euler integration) ---
            dx = self.model.derivatives_heading(t, state, u, v0)
            state = state + dt * dx

            # Position update
            new_psi = state[0]
            pos = pos + dt * v0 * np.array([np.cos(new_psi),
                                             np.sin(new_psi)])

        return SimResult(
            time=time, X=X, Y=Y, psi=psi, v=vel, delta=delta,
            psi_des=psi_des_arr, e_psi=e_psi_arr, e_d=e_d_arr,
            kappa=kappa_arr, rho=rho_arr, delta_cmd=delta_cmd_arr,
            label=label,
        )
