# -*- coding: utf-8 -*-
"""Tests for controller modules.

Author: Suwon Lee, Kookmin University
Created: 2026-03-01
Description: Unit tests for LPV H-infinity and PID controllers.
"""

import numpy as np
import pytest
from vfg_pathfollowing.controllers import LPVHinfController, PIDFeedforward
from vfg_pathfollowing import Simulator
from vfg_pathfollowing.paths import StepCurvaturePath


class TestPIDFeedforward:
    def test_zero_error_zero_curvature(self):
        pid = PIDFeedforward()
        u = pid.compute(e_psi=0.0, kappa=0.0, dt=0.01)
        assert u == 0.0

    def test_positive_error_positive_output(self):
        pid = PIDFeedforward()
        u = pid.compute(e_psi=0.1, kappa=0.0, dt=0.01)
        assert u > 0  # should steer toward desired heading

    def test_saturation(self):
        pid = PIDFeedforward(delta_max=0.5)
        u = pid.compute(e_psi=10.0, kappa=0.0, dt=0.01)
        assert abs(u) <= 0.5

    def test_feedforward(self):
        pid = PIDFeedforward()
        u_no_ff = pid.compute(e_psi=0.0, kappa=0.0, dt=0.01)
        pid.reset()
        u_with_ff = pid.compute(e_psi=0.0, kappa=2.0, dt=0.01)
        assert u_with_ff > u_no_ff

    def test_reset(self):
        pid = PIDFeedforward(K_I=0.1)
        pid.compute(e_psi=0.5, dt=0.01)
        pid.compute(e_psi=0.5, dt=0.01)
        pid.reset()
        assert pid._e_integral == 0.0
        assert pid._initialized is False


class TestLPVHinfController:
    def test_default_loads(self):
        ctrl = LPVHinfController.default()
        assert ctrl.n_vertices == 6
        assert ctrl.version == 'v3'

    def test_zero_input_zero_output_initially(self):
        ctrl = LPVHinfController.default()
        u = ctrl.compute(e_psi=0.0, delta_meas=0.0, rho=0.0, dt=0.01)
        assert abs(u) < 1e-6

    def test_nonzero_error_nonzero_output(self):
        ctrl = LPVHinfController.default()
        # Step several times to build up state
        for _ in range(10):
            u = ctrl.compute(e_psi=0.1, delta_meas=0.0, rho=1.0, dt=0.01)
        assert abs(u) > 0.001

    def test_rho_clipping(self):
        ctrl = LPVHinfController.default()
        # Should not crash with out-of-range rho
        u1 = ctrl.compute(e_psi=0.1, delta_meas=0.0, rho=-1.0, dt=0.01)
        ctrl.reset()
        u2 = ctrl.compute(e_psi=0.1, delta_meas=0.0, rho=100.0, dt=0.01)
        assert np.isfinite(u1)
        assert np.isfinite(u2)

    def test_reset(self):
        ctrl = LPVHinfController.default()
        ctrl.compute(e_psi=0.5, delta_meas=0.1, rho=2.0, dt=0.01)
        ctrl.reset()
        for state in ctrl._states:
            np.testing.assert_allclose(state, 0.0)

    def test_sign_convention(self):
        """Positive e_psi (psi_des > psi) should produce positive steering."""
        ctrl = LPVHinfController.default()
        # Run several steps to accumulate state response
        for _ in range(50):
            u = ctrl.compute(e_psi=0.2, delta_meas=0.0, rho=1.0, dt=0.01)
        # With positive heading error, controller should steer positively
        assert u > 0

    # ------------------------------------------------------------------
    # Tuning parameter tests
    # ------------------------------------------------------------------

    def test_kff_zero_when_no_kappa(self):
        """K_ff is ignored when kappa is not passed (backward compatible)."""
        ctrl_base = LPVHinfController.default()
        ctrl_kff = LPVHinfController.default(K_ff=1.0)
        u_base = ctrl_base.compute(e_psi=0.1, delta_meas=0.0, rho=1.0, dt=0.01)
        u_kff = ctrl_kff.compute(e_psi=0.1, delta_meas=0.0, rho=1.0, dt=0.01)
        assert u_base == u_kff

    def test_kff_adds_feedforward(self):
        """K_ff > 0 changes output when kappa is supplied."""
        ctrl_base = LPVHinfController.default(K_ff=0.0)
        ctrl_kff = LPVHinfController.default(K_ff=0.8)
        u_base = ctrl_base.compute(e_psi=0.0, delta_meas=0.0, rho=0.0, dt=0.01,
                                   kappa=1.0)
        u_kff = ctrl_kff.compute(e_psi=0.0, delta_meas=0.0, rho=0.0, dt=0.01,
                                 kappa=1.0)
        assert u_kff != u_base

    def test_kff_formula_at_zero_state(self):
        """At initial state (H-inf output == 0), u == K_ff * arctan(L * kappa)."""
        K_ff = 0.8
        L = 0.2
        kappa = 2.0
        ctrl = LPVHinfController.default(K_ff=K_ff, L=L)
        u = ctrl.compute(e_psi=0.0, delta_meas=0.0, rho=0.0, dt=0.01, kappa=kappa)
        expected = K_ff * np.arctan(L * kappa)
        assert abs(u - expected) < 1e-9

    def test_output_gain_scales(self):
        """output_gain=2.0 produces exactly 2x the output of gain=1.0."""
        ctrl1 = LPVHinfController.default(output_gain=1.0)
        ctrl2 = LPVHinfController.default(output_gain=2.0)
        # Prime both controllers with identical inputs so states match
        for _ in range(5):
            ctrl1.compute(e_psi=0.1, delta_meas=0.0, rho=1.0, dt=0.01)
            ctrl2.compute(e_psi=0.1, delta_meas=0.0, rho=1.0, dt=0.01)
        u1 = ctrl1.compute(e_psi=0.1, delta_meas=0.0, rho=1.0, dt=0.01)
        u2 = ctrl2.compute(e_psi=0.1, delta_meas=0.0, rho=1.0, dt=0.01)
        assert abs(u2 - 2.0 * u1) < 1e-9

    def test_rho_scale_changes_vertex(self):
        """rho_scale=2.0 causes different output than rho_scale=1.0."""
        ctrl1 = LPVHinfController.default(rho_scale=1.0)
        ctrl2 = LPVHinfController.default(rho_scale=2.0)
        # Use a rho that maps to different vertices when scaled
        u1 = ctrl1.compute(e_psi=0.1, delta_meas=0.0, rho=1.0, dt=0.01)
        u2 = ctrl2.compute(e_psi=0.1, delta_meas=0.0, rho=1.0, dt=0.01)
        assert u1 != u2

    def test_delta_max_clips(self):
        """Output must always be within [-delta_max, +delta_max]."""
        ctrl = LPVHinfController.default(delta_max=0.3)
        for _ in range(20):
            u = ctrl.compute(e_psi=1.0, delta_meas=0.0, rho=5.0, dt=0.01,
                             kappa=10.0)
            assert abs(u) <= 0.3 + 1e-9

    def test_backward_compat_defaults(self):
        """Default tuning params produce numerically identical output to old code."""
        ctrl_new = LPVHinfController.default(K_ff=0.0, rho_scale=1.0, output_gain=1.0)
        ctrl_ref = LPVHinfController.default()
        for e in [0.0, 0.1, 0.3]:
            u_new = ctrl_new.compute(e_psi=e, delta_meas=0.0, rho=1.0, dt=0.01)
            u_ref = ctrl_ref.compute(e_psi=e, delta_meas=0.0, rho=1.0, dt=0.01)
            assert abs(u_new - u_ref) < 1e-9

    def test_simulator_hinf_params(self):
        """Simulator with hinf_params passes params to LPVHinfController."""
        path = StepCurvaturePath(R=0.5, theta_arc=1.57)
        sim = Simulator(path, controller='lpv-hinf', speed=1.5,
                        hinf_params={'K_ff': 0.8, 'output_gain': 1.1})
        ctrl = sim._controller
        assert isinstance(ctrl, LPVHinfController)
        assert ctrl.K_ff == 0.8
        assert ctrl.output_gain == 1.1
