"""Linearize an inverted pendulum at the upright equilibrium, design an LQR,
and simulate the stabilized closed loop.

Workflow: analysis.linearize (tool) produces an LTISystem -> control.lqr
(design factory) returns a LinearFeedbackController block -> wire full-state
feedback and simulate the nonlinear plant.
"""

import numpy as np

from minilink.analysis.linearize import linearize
from minilink.control.lqr import lqr
from minilink.core.diagram import DiagramSystem
from minilink.dynamics.catalog.pendulum.pendulum import InvertedPendulum

# Nonlinear plant; zero angle is upright (unstable) for the inverted pendulum.
plant = InvertedPendulum()

# Linearize about the upright equilibrium and inspect the open-loop modes.
lti = linearize(plant, x_bar=[0.0, 0.0])
print("open-loop poles:", np.round(np.linalg.eigvals(lti.A()), 4))

# Design an LQR from the linear model; returns a ready-to-wire feedback block.
controller = lqr(
    lti.A(),
    lti.B(),
    Q=np.diag([10.0, 1.0]),
    R=np.array([[1.0]]),
    xbar=[0.0, 0.0],
    ubar=[0.0],
)

# Full-state feedback: plant state -> controller "x"; command -> plant "u".
diagram = DiagramSystem()
diagram.add_subsystem(controller, "lqr")
diagram.add_subsystem(plant, "plant")
diagram.connect("plant", "x", "lqr", "x")
diagram.connect("lqr", "u", "plant", "u")

# Release the pendulum tipped slightly off upright; the LQR should recover it.
plant.x0 = np.array([0.4, 0.0])
diagram.compute_trajectory(tf=8.0, n_steps=801)
diagram.plot_trajectory()
plant.animate()
