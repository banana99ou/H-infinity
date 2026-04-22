# -*- coding: utf-8 -*-
"""Tests for simulation engine and high-level API.

Author: Suwon Lee, Kookmin University
Created: 2026-03-01
Description: Integration tests for closed-loop simulation.
"""

import numpy as np
import pytest
from vfg_pathfollowing import (
    Simulator, StepCurvaturePath, SinusoidalPath, BezierPath,
    KinematicBicycle, VectorFieldGuidance,
    LPVHinfController, PIDFeedforward,
    ClosedLoopSimulator, compute_metrics,
)


class TestSimulator:
    def test_quickstart_pattern(self):
        """5-line quickstart should work."""
        path = StepCurvaturePath(R=0.5, theta_arc=1.57, L2=20)
        sim = Simulator(path, controller='lpv-hinf', speed=1.5)
        result = sim.run(T=10.0)
        assert len(result.time) > 0
        assert len(result.X) == len(result.time)
        assert result.label == 'LPV H-inf'

    def test_pid_controller(self):
        path = StepCurvaturePath()
        sim = Simulator(path, controller='pid-ff', speed=1.0)
        result = sim.run(T=10.0)
        assert len(result.time) > 0
        assert result.label == 'PID-FF'

    def test_compare(self):
        path = StepCurvaturePath()
        sim = Simulator(path, speed=1.0)
        results = sim.compare(T=10.0)
        assert 'lpv-hinf' in results
        assert 'pid-ff' in results

    def test_invalid_controller(self):
        path = StepCurvaturePath()
        with pytest.raises(ValueError):
            Simulator(path, controller='nonexistent')

    def test_custom_controller_instance(self):
        path = StepCurvaturePath()
        ctrl = PIDFeedforward(K_P=3.0)
        sim = Simulator(path, controller=ctrl, speed=1.0)
        result = sim.run(T=5.0)
        assert len(result.time) > 0

    def test_with_noise(self):
        path = StepCurvaturePath()
        sim = Simulator(path, controller='pid-ff', speed=1.0,
                        noise=True, noise_seed=42)
        result = sim.run(T=5.0)
        assert len(result.time) > 0


class TestClosedLoopSimulator:
    def test_lpv_reduces_error(self):
        """LPV should converge heading error to near zero."""
        path = StepCurvaturePath(L1=10, R=1.0, theta_arc=np.pi / 4, L2=10)
        model = KinematicBicycle()
        guidance = VectorFieldGuidance(path, k_e=3.0)
        ctrl = LPVHinfController.default()

        sim = ClosedLoopSimulator(model, guidance, ctrl, path)
        result = sim.run(T=15.0, v0=1.0)

        # Last 20% should have small error
        n = len(result.e_psi)
        tail = result.e_psi[int(0.8 * n):]
        rms = np.sqrt(np.mean(tail**2))
        assert np.degrees(rms) < 5.0  # less than 5 degrees

    def test_pid_works(self):
        path = StepCurvaturePath(L1=5, R=1.0, theta_arc=np.pi / 4, L2=25)
        model = KinematicBicycle()
        guidance = VectorFieldGuidance(path)
        ctrl = PIDFeedforward()

        sim = ClosedLoopSimulator(model, guidance, ctrl, path)
        result = sim.run(T=15.0, v0=1.0)

        n = len(result.e_psi)
        tail = result.e_psi[int(0.8 * n):]
        rms = np.sqrt(np.mean(tail**2))
        assert np.degrees(rms) < 10.0


class TestMetrics:
    def test_compute_metrics(self):
        path = StepCurvaturePath(L2=25)
        sim = Simulator(path, controller='pid-ff', speed=1.0)
        result = sim.run(T=15.0)
        metrics = compute_metrics(result, t_transient=3.0)

        assert 'rms_e_psi_deg' in metrics
        assert 'rms_e_d' in metrics
        assert 'settling_time' in metrics
        assert metrics['rms_e_psi_deg'] >= 0
        assert metrics['settling_time'] >= 0


class TestLPVBetterThanPID:
    def test_lpv_beats_pid_at_high_speed(self):
        """At v=2.0, LPV should have lower peak error than PID on step curvature."""
        path = StepCurvaturePath(R=0.5, theta_arc=np.pi / 2, L2=40)
        sim = Simulator(path, speed=2.0)
        results = sim.compare(T=15.0)

        m_lpv = compute_metrics(results['lpv-hinf'])
        m_pid = compute_metrics(results['pid-ff'])

        # LPV advantage is in peak (transient) error at curvature step
        assert m_lpv['max_e_psi_deg'] < m_pid['max_e_psi_deg']
