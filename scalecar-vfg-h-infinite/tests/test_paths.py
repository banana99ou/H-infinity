# -*- coding: utf-8 -*-
"""Tests for path modules.

Author: Suwon Lee, Kookmin University
Created: 2026-03-01
Description: Unit tests for all path classes.
"""

import numpy as np
import pytest
from vfg_pathfollowing.paths import (
    StepCurvaturePath, SinusoidalPath, SlalomPath, BezierPath,
)


class TestStepCurvaturePath:
    def test_start_at_origin(self):
        path = StepCurvaturePath()
        pos = path.position(0)
        np.testing.assert_allclose(pos, [0, 0], atol=1e-10)

    def test_total_length(self):
        path = StepCurvaturePath(L1=5, R=0.5, theta_arc=np.pi / 2, L2=5)
        expected = 5 + 0.5 * np.pi / 2 + 5
        assert abs(path.total_length - expected) < 1e-10

    def test_curvature_straight(self):
        path = StepCurvaturePath()
        assert path.curvature(0) == 0.0
        assert path.curvature(path.total_length) == 0.0

    def test_curvature_arc(self):
        R = 0.5
        path = StepCurvaturePath(R=R)
        kappa = path.curvature(path.L1 + 0.1)
        assert abs(kappa - 1.0 / R) < 1e-10

    def test_heading_monotonic_left_turn(self):
        path = StepCurvaturePath(direction=1)
        h0 = path.heading(path.L1)
        h1 = path.heading(path.L1 + path._arc_length / 2)
        assert h1 > h0

    def test_closest_point(self):
        path = StepCurvaturePath()
        q = np.array([2.5, 0.1])
        s_star, p_star = path.closest_point(q)
        assert abs(p_star[0] - 2.5) < 0.01
        assert abs(p_star[1]) < 1e-6


class TestSinusoidalPath:
    def test_start_at_origin(self):
        path = SinusoidalPath()
        pos = path.position(0)
        np.testing.assert_allclose(pos, [0, 0], atol=1e-6)

    def test_total_length_positive(self):
        path = SinusoidalPath()
        assert path.total_length > 0

    def test_curvature_varies(self):
        path = SinusoidalPath()
        k1 = path.curvature(0)
        k2 = path.curvature(path.total_length / 4)
        assert k1 != k2  # curvature should vary

    def test_tangent_unit(self):
        path = SinusoidalPath()
        t = path.tangent(path.total_length / 3)
        np.testing.assert_allclose(np.linalg.norm(t), 1.0, atol=1e-6)


class TestSlalomPath:
    def test_start_at_origin(self):
        path = SlalomPath()
        pos = path.position(0)
        np.testing.assert_allclose(pos, [0, 0], atol=1e-10)

    def test_curvature_alternates(self):
        path = SlalomPath()
        # First arc should be positive (left), second negative (right)
        s_arc1 = path.L1 + 0.01
        s_arc2 = path._s_bounds[3] + 0.01  # third segment (second arc)
        k1 = path.curvature(s_arc1)
        k2 = path.curvature(s_arc2)
        assert k1 > 0  # left turn
        assert k2 < 0  # right turn

    def test_total_length_positive(self):
        path = SlalomPath()
        assert path.total_length > 20


class TestBezierPath:
    def test_two_point_line(self):
        path = BezierPath([(0, 0), (10, 0)])
        pos = path.position(0)
        np.testing.assert_allclose(pos, [0, 0], atol=1e-4)
        # Should be roughly a straight line
        mid = path.position(path.total_length / 2)
        assert abs(mid[1]) < 0.1
        assert abs(mid[0] - 5) < 0.5

    def test_total_length_positive(self):
        path = BezierPath([(0, 0), (3, 0), (6, 2), (9, 2)])
        assert path.total_length > 0

    def test_curvature_nonzero(self):
        path = BezierPath([(0, 0), (3, 0), (6, 2), (9, 2)])
        k = path.curvature(path.total_length / 2)
        assert k != 0  # should have curvature at inflection

    def test_tangent_unit(self):
        path = BezierPath([(0, 0), (5, 3), (10, 0)])
        t = path.tangent(path.total_length / 2)
        np.testing.assert_allclose(np.linalg.norm(t), 1.0, atol=1e-6)

    def test_circle_factory(self):
        path = BezierPath.circle(radius=2.0)
        assert path.total_length > 10  # 2*pi*2 ~ 12.6

    def test_minimum_waypoints(self):
        with pytest.raises(ValueError):
            BezierPath([(0, 0)])

    def test_position_continuity(self):
        path = BezierPath([(0, 0), (3, 1), (6, 0), (9, 1)])
        # Check positions are continuous across segments
        ds = 0.001
        for i in range(1, path._n_seg):
            s_boundary = path._seg_s_start[i]
            p_before = path.position(s_boundary - ds)
            p_after = path.position(s_boundary + ds)
            assert np.linalg.norm(p_after - p_before) < 0.05
