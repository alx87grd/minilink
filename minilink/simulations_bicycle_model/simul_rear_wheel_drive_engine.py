"""Open-loop demo for DynamicBicycleRearWheelDrive.

No controller.
No velocity PID.
No path tracking.

The vehicle receives constant open-loop inputs:
- throotle: Normalized throttle [0, 1]
- delta: front steering angle [rad]
"""

import types

import numpy as np

from minilink.core.diagram import DiagramSystem
from minilink.core.system import System
from minilink.dynamics.catalog.vehicles.dynamic_bicycle import (
    DynamicBicycleRearWheelDriveEngine,
    Pacejka,
)
from minilink.graphical.primitives import camera_matrix

THR_REF = 1.0  # Normalized throttle [0, 1]
DELTA_REF = 0.0  # rad


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


class ConstantVehicleInput(System):
    """Constant open-loop inputs for DynamicBicycleRearWheelDriveEngine.

    This is not a controller. It only provides fixed inputs required by the model.

    Outputs
    -------
    thr
        Engine throttle, normalized [0, 1].
    delta
        Front steering angle [rad].
    """

    def __init__(self, thr: float = THR_REF, delta: float = DELTA_REF):
        super().__init__(0, 0, 1)

        self.name = "Constant vehicle input"

        self.inputs = {}
        self.outputs = {}
        self.recompute_input_properties()

        self.thr = float(thr)
        self.delta = float(delta)

        self.add_output_port(
            1,
            "thr",
            function=self.h_thr,
            dependencies=[],
        )

        self.add_output_port(
            1,
            "delta",
            function=self.h_delta,
            dependencies=[],
        )

        self.p = 2

    def h_thr(self, x, u, t=0.0, params=None):
        return np.array([self.thr], dtype=float)

    def h_delta(self, x, u, t=0.0, params=None):
        return np.array([self.delta], dtype=float)


def main():
    vehicle = DynamicBicycleRearWheelDriveEngine()

    vehicle.tire_model_f = Pacejka()
    vehicle.tire_model_r = Pacejka()

    # State convention for DynamicBicycleRearWheelDrive:
    # x = [X, Y, theta, phi_rear, phi_front, vx, vy, r, w_rear, w_front]

    vehicle.x0 = np.array(
        [
            0.0,  # X
            0.0,  # Y
            0.0,  # theta
            0.0,  # phi_rear
            0.0,  # phi_front
            0.0,  # vx
            0.0,  # vy
            0.0,  # r
            0.0,  # w_rear
            0.0,  # w_front
            0.0,  # tau_engine
            0.0,  # delta_act
        ],
        dtype=float,
    )

    constant_input = ConstantVehicleInput(
        thr=THR_REF,
        delta=DELTA_REF,
    )

    diagram = DiagramSystem()
    diagram.name = "Open-loop DynamicBicycleRearWheelDrive"

    diagram.add_subsystem(constant_input, "constant_input")
    diagram.add_subsystem(vehicle, "vehicle")

    diagram.connect("constant_input", "thr", "vehicle", "thr")
    diagram.connect("constant_input", "delta", "vehicle", "delta")

    diagram.plot_graphe()

    print("Starting trajectory computation...")

    diagram.compute_trajectory(
        tf=10.0,
        dt=0.02,
        show=False,
        verbose=False,
    )

    print("Trajectory computation done.")

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
