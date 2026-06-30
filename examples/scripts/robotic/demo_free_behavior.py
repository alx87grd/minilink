"""Manipulator free motion under gravity — no controller, zero joint torque.

Run from repo root::

    python examples/scripts/robotic/demo_free_behavior.py

Use this as a baseline before closed-loop demos: the arm falls, swings, and
settles (or keeps moving) from the initial pose with ``τ = 0``.
"""

import numpy as np

from minilink.dynamics.catalog.manipulators.arms import TwoLinkManipulator

# # --- One link: release from a raised angle ---
# from minilink.dynamics.catalog.manipulators.arms import OneLinkManipulator
# one = OneLinkManipulator()
# one.x0 = np.array([0.9, 0.0])

# one.compute_trajectory(tf=6.0)
# one.plot_trajectory()
# one.animate()

# --- Two links: release from a bent configuration ---
two = TwoLinkManipulator()

p_start = np.array([0.5, 0.5])
q0 = two.inverse_kinematics(p_start)
two.x0 = np.concatenate([q0, np.zeros(2)])

two.compute_trajectory(tf=10.0)
two.plot_trajectory()
two.animate()
