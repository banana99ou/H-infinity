# -*- coding: utf-8 -*-
"""VFG Path Following for LIMO mobile robot.

Author: Suwon Lee, Kookmin University
Created: 2026-03-01
Description: Educational package for VFG-based path following
    with LPV H-infinity and PID controllers.

Quick start::

    from vfg_pathfollowing import Simulator, StepCurvaturePath

    path = StepCurvaturePath(R=0.5, theta_arc=1.57)
    sim = Simulator(path, controller='lpv-hinf', speed=1.5)
    result = sim.run(T=20.0)
    result.plot(path=path)
"""

# High-level API
from .api import Simulator

# Paths
from .paths import (
    PathBase,
    StepCurvaturePath,
    SinusoidalPath,
    SlalomPath,
    BezierPath,
)

# Models
from .models import KinematicBicycle

# Guidance
from .guidance import VectorFieldGuidance, PathProjector

# Controllers
from .controllers import ControllerBase, LPVHinfController, PIDFeedforward

# Simulation
from .simulation import SimResult, ClosedLoopSimulator, NoiseGenerator, compute_metrics

__version__ = "0.1.0"

__all__ = [
    # API
    "Simulator",
    # Paths
    "PathBase",
    "StepCurvaturePath",
    "SinusoidalPath",
    "SlalomPath",
    "BezierPath",
    # Models
    "KinematicBicycle",
    # Guidance
    "VectorFieldGuidance",
    "PathProjector",
    # Controllers
    "ControllerBase",
    "LPVHinfController",
    "PIDFeedforward",
    # Simulation
    "SimResult",
    "ClosedLoopSimulator",
    "NoiseGenerator",
    "compute_metrics",
]
