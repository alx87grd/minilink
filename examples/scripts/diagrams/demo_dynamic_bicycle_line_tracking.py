"""Closed-loop diagram: straight-line task, static tracker, NumPy dynamic bicycle.

Tracks the global x-axis (``y = 0``, ``theta = 0``) with a small initial lateral
offset. Run from repo root::

    conda run -n dev-h26 python examples/scripts/diagrams/demo_dynamic_bicycle_line_tracking.py

Uses :class:`~minilink.dynamics.catalog.vehicles.dynamic_bicycle.DynamicBicycle`
(no JAX). For meshcat instead of the default renderer, call
``diagram.animate(renderer="meshcat")``.
"""

import numpy as np

from minilink.core.blocks.sources import Source
from minilink.core.diagram import DiagramSystem
from minilink.core.system import StaticSystem
from minilink.dynamics.catalog.vehicles.dynamic_bicycle import DynamicBicycleCar3D


class StraightLinePathTask(Source):
    """Straight-line hold: output ``[y_ref, theta_ref, u_ref]`` (world frame).

    Parameters
    ----------
    y_ref :
        Lateral position reference on the path [m].
    theta_ref :
        Heading reference [rad].
    u_ref :
        Body longitudinal speed reference [m/s].
    """

    def __init__(
        self,
        y_ref: float = 0.0,
        theta_ref: float = 0.0,
        u_ref: float = 5.0,
    ):
        super().__init__(3)
        self.name = "Straight-line task"
        self.params["value"] = np.array([y_ref, theta_ref, u_ref], dtype=float)


class SimpleLineTracker(StaticSystem):
    """Feedback mapping vehicle output + references to ``w_rear`` and ``delta``.

    Inputs follow the concatenated port order ``y`` (6) then ``ref`` (3), with
    ``y`` the plant output ``[x, y, theta, u, v, r]`` and ``ref`` from
    :class:`StraightLinePathTask`.
    """

    def __init__(self, r_rear: float):
        super().__init__(9, 2)
        self.r_rear = float(r_rear)
        self.name = "Line tracker"
        self.params = {
            "k_y": 0.25,
            "k_psi": 2.0,
            "k_r": 1.0,
            "k_u": 0.8,
            "delta_max": 0.5,
            "w_rear_max": 120.0,
        }
        self.inputs = {}
        self.add_input_port(6, "y", nominal_value=np.zeros(6))
        self.add_input_port(3, "ref", nominal_value=np.zeros(3))
        self.outputs = {}
        self.add_output_port(
            1,
            "w_rear",
            function=self.ctl_w_rear,
            dependencies=["y", "ref"],
        )
        self.add_output_port(
            1,
            "delta",
            function=self.ctl_delta,
            dependencies=["y", "ref"],
        )

    def ctl_w_rear(self, x, u, t=0.0, params=None):
        p = self.params if params is None else params
        u_b, u_ref = u[3], u[8]
        e_u = u_ref - u_b
        w_cmd = u_ref / self.r_rear + p["k_u"] * e_u
        w_cmd = np.clip(w_cmd, 0.0, p["w_rear_max"])
        return np.array([w_cmd], dtype=float)

    def ctl_delta(self, x, u, t=0.0, params=None):
        p = self.params if params is None else params
        py, theta, r = u[1], u[2], u[5]
        y_ref, theta_ref = u[6], u[7]
        e_y = py - y_ref
        e_psi = theta - theta_ref
        delta_cmd = -(p["k_y"] * e_y + p["k_psi"] * e_psi + p["k_r"] * r)
        delta_cmd = np.clip(delta_cmd, -p["delta_max"], p["delta_max"])
        return np.array([delta_cmd], dtype=float)


vehicle = DynamicBicycleCar3D()
vehicle.x0 = np.array([0.0, 2.35, 0.6, 5.0, 0.0, 0.0])

task = StraightLinePathTask(y_ref=0.0, theta_ref=0.0, u_ref=5.0)
tracker = SimpleLineTracker(vehicle.r_r)

diagram = DiagramSystem()
diagram.name = "Dynamic bicycle line tracking"
diagram.add_subsystem(task, "task")
diagram.add_subsystem(tracker, "tracker")
diagram.add_subsystem(vehicle, "vehicle")

diagram.connect("task", "y", "tracker", "ref")
diagram.connect("vehicle", "y", "tracker", "y")
diagram.connect("tracker", "w_rear", "vehicle", "w_rear")
diagram.connect("tracker", "delta", "vehicle", "delta")

diagram.plot_graphe()
diagram.compute_trajectory(tf=5.0, dt=0.02, show=False, verbose=False)
diagram.plot_trajectory(signals=("x", "u"), backend="matplotlib")
diagram.animate(renderer="meshcat")
