"""Velocity PID demo for DynamicBicycle.

Controls rear wheel speed ``w_rear`` to track a constant longitudinal speed
reference ``u_ref``. No path tracking, no heading loop, no yaw-rate loop.

Run from repo root::

    conda run -n dev-h26 python examples/scripts/diagrams/demo_dynamic_bicycle_velocity_pid.py
"""

import types

import numpy as np

from minilink.core.diagram import DiagramSystem
from minilink.core.system import DynamicSystem, System
from minilink.dynamics.catalog.vehicles.dynamic_bicycle_ALEXS_THINGS import (
    DynamicBicycle,
)
from minilink.graphical.animation.primitives import camera_matrix

U_REF = 5.0


def attach_vehicle_centered_diagram_camera(
    diagram_sys, plant, *, plant_sys_id: str = "vehicle"
) -> None:
    """Animate the composed diagram using a camera target on the plant's ``(x, y)``."""

    ix = diagram_sys.state_index[plant_sys_id][0]

    def get_camera_transform(self, x, _u, _t):
        target = np.asarray(plant.camera_target, dtype=float).reshape(3).copy()
        target[0] += float(x[ix])
        target[1] += float(x[ix + 1])
        return camera_matrix(
            target=target,
            plot_axes=self.camera_plot_axes,
            scale=self.camera_scale,
        )

    diagram_sys.camera_plot_axes = tuple(plant.camera_plot_axes)
    diagram_sys.camera_scale = float(plant.camera_scale)
    diagram_sys.camera_target = (
        np.asarray(plant.camera_target, dtype=float).reshape(3).copy()
    )
    diagram_sys.get_camera_transform = types.MethodType(
        get_camera_transform, diagram_sys
    )


class SpeedReference(System):
    """Constant longitudinal speed reference.

    Output
    ------
    u_ref
        Desired body longitudinal speed [m/s].
    """

    def __init__(self, u_ref: float = U_REF):
        super().__init__(0, 0, 1)
        self.name = "Speed reference"
        self.inputs = {}
        self.outputs = {}
        self.recompute_input_properties()

        self._u_ref = float(u_ref)

        self.add_output_port(
            1,
            "u_ref",
            function=self.h_u_ref,
            dependencies=[],
        )

    def h_u_ref(self, x, u, t=0.0, params=None):
        return np.array([self._u_ref], dtype=float)


class VelocityPID(DynamicSystem):
    """PI + derivative-on-measurement for ``w_rear`` rear wheel rate.

    State ``x = [int_e, u_filt]``:
    - ``int_e``: integral of speed error
    - ``u_filt``: filtered body longitudinal speed

    Port order in ``u``:
    - ``u_ref``: desired longitudinal speed
    - plant ``y``: vehicle output vector

    The controller computes:

    ``w_rear = u_ref / r_rear + PID correction``
    """

    def __init__(self, r_rear: float):
        super().__init__(2, 7, 1)

        self.r_rear = float(r_rear)
        self.name = "Velocity PID"

        self.params = {
            "Kp": 2.0,
            "Ki": 1.0,
            "Kd": 0.05,
            "tau": 0.1,
            "w_max": 120.0,
            "i_max": 25.0,
        }

        self.state.labels = ["int_e", "u_filt"]
        self.x0 = np.array([0.0, U_REF], dtype=float)

        self.inputs = {}
        self.add_input_port(1, "u_ref", nominal_value=np.array([U_REF]))
        self.add_input_port(6, "y", nominal_value=np.zeros(6))

        self.outputs = {}
        self.add_output_port(
            1,
            "w_rear",
            function=self.h_w,
            dependencies=["u_ref", "y"],
        )
        self.add_output_port(
            2,
            "x",
            function=self.compute_state,
            dependencies=[],
        )

    def _speed_error(self, u):
        u_ref = u[0]
        u_b = u[4]
        return u_ref - u_b

    def f(self, x, u, t=0.0, params=None):
        p = self.params if params is None else params

        int_e = x[0]
        u_filt = x[1]

        e = self._speed_error(u)
        u_b = u[4]

        tau = max(p["tau"], 1e-6)

        w_ff = u[0] / self.r_rear
        d_filt = (u_b - u_filt) / tau

        w_pred = w_ff + p["Kp"] * e + p["Ki"] * int_e - p["Kd"] * d_filt

        stop_hi = (w_pred >= p["w_max"]) and (e > 0)
        stop_lo = (w_pred <= 0.0) and (e < 0)

        d_int = 0.0 if (stop_hi or stop_lo) else e

        if int_e >= p["i_max"] and e > 0:
            d_int = 0.0
        elif int_e <= -p["i_max"] and e < 0:
            d_int = 0.0

        d_u_filt = (u_b - u_filt) / tau

        return np.array([d_int, d_u_filt], dtype=float)

    def h_w(self, x, u, t=0.0, params=None):
        p = self.params if params is None else params

        int_e = x[0]
        u_filt = x[1]

        e = self._speed_error(u)
        u_b = u[4]

        tau = max(p["tau"], 1e-6)
        d_filt = (u_b - u_filt) / tau

        w_cmd = u[0] / self.r_rear + p["Kp"] * e + p["Ki"] * int_e - p["Kd"] * d_filt

        w_cmd = np.clip(w_cmd, 0.0, p["w_max"])

        return np.array([w_cmd], dtype=float)

    def get_kinematic_geometry(self):
        return []

    def get_kinematic_transforms(self, _x, _u, _t):
        return []


def main():
    vehicle = DynamicBicycle()

    # Initial state:
    # [px, py, theta, ?, u_body, ?, ?] depending on catalog convention.
    # Keeping your original initial condition.
    vehicle.x0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=float)

    speed_ref = SpeedReference(U_REF)
    vel_pid = VelocityPID(vehicle.r_r)

    diagram = DiagramSystem()
    diagram.name = "Velocity PID only - DynamicBicycle"

    diagram.add_subsystem(speed_ref, "speed_ref")
    diagram.add_subsystem(vel_pid, "vel_pid")
    diagram.add_subsystem(vehicle, "vehicle")

    diagram.connect("speed_ref", "u_ref", "vel_pid", "u_ref")
    diagram.connect("vehicle", "y", "vel_pid", "y")
    diagram.connect("vel_pid", "w_rear", "vehicle", "w_rear")

    # No yaw-rate / steering controller.
    # If the vehicle's delta input has a nominal/default value, it will remain zero.
    # If your DynamicBicycle requires delta to be explicitly connected, add a
    # constant-zero steering source.

    diagram.plot_graphe()

    diagram.compute_trajectory(
        tf=20.0,
        dt=0.02,
        show=False,
        verbose=False,
    )

    diagram.plot_trajectory(
        signals=("x", "u"),
        backend="matplotlib",
    )

    attach_vehicle_centered_diagram_camera(diagram, vehicle)

    diagram.animate(renderer="matplotlib")
    # diagram.animate(renderer="meshcat")
    # diagram.animate(renderer="plotly")


if __name__ == "__main__":
    main()
