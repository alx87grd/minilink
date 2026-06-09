"""Open-loop demo for DynamicBicycleRearWheelDrive.

No controller.
No velocity PID.
No path tracking.

The vehicle receives constant open-loop inputs:
- t_rear: rear wheel torque [Nm]
- delta: front steering angle [rad]
"""

import types

import numpy as np

from minilink.core.diagram import DiagramSystem
from minilink.graphical.animation.primitives import camera_matrix
from projects.bicycle.models.dynamic_bicycle import DynamicBicycleRearWheelDrive
from projects.bicycle.models.tire_models import Pacejka

T_REAR_REF = 200.0  # Nm
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


def main():
    vehicle = DynamicBicycleRearWheelDrive()

    vehicle.tire_model_f = Pacejka()
    vehicle.tire_model_r = Pacejka()

    # State convention for DynamicBicycleRearWheelDrive:
    # x = [X, Y, theta, phi_rear, phi_front, vx, vy, r, w_rear, w_front]
    vehicle.x0 = np.array(
        [
            0.0,  # X
            0.0,  # Y
            0.0,  # theta
            0.0,  # vx
            0.0,  # vy
            0.0,  # r
            0.0,  # w_rear
            0.0,  # w_front
        ],
        dtype=float,
    )

    # Constant open-loop inputs as default (nominal) values on the vehicle ports.
    vehicle.inputs["t_rear"].nominal_value = np.array([T_REAR_REF])
    vehicle.inputs["delta"].nominal_value = np.array([DELTA_REF])

    diagram = DiagramSystem()
    diagram.name = "Open-loop DynamicBicycleRearWheelDrive"

    diagram.add_subsystem(vehicle, "vehicle")

    diagram.plot_diagram()

    diagram.compute_trajectory(
        tf=10,
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
