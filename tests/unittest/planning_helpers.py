"""Shared planning fixtures for RRT / RRT* contract tests."""

from __future__ import annotations

import numpy as np

from minilink.core.geometry import Sphere
from minilink.core.sets import BoxSet
from minilink.dynamics.catalog.vehicles.steering import HolonomicMobileRobot
from minilink.planning.problems import PlanningProblem
from minilink.planning.spatial.collision import bind, disc
from minilink.planning.spatial.scene import Scene

X_START = np.array([-4.0, -4.0])
X_GOAL = np.array([4.0, 4.0])


def make_holonomic_obstacle_problem():
    """Small holonomic robot with one spherical obstacle (RRT contract fixture)."""
    sys = HolonomicMobileRobot()  # dx = u
    sys.state.lower_bound = np.array([-6.0, -6.0])
    sys.state.upper_bound = np.array([6.0, 6.0])
    sys.inputs["u"].lower_bound = np.array([-1.0, -1.0])
    sys.inputs["u"].upper_bound = np.array([1.0, 1.0])

    scene = Scene(obstacles=(Sphere([0.0, 0.0], 1.0),))
    body = bind(sys, disc(0.2))
    X = BoxSet.from_system_state(sys) & scene.clearance_field(body).as_constraint()
    return PlanningProblem(sys=sys, x_start=X_START, x_goal=X_GOAL, X=X), X
