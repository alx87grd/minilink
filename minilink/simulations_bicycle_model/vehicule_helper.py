import types

import numpy as np

from minilink.dynamics.catalog.vehicles.dynamic_bicycle import (
    DynamicBicycleRearWheelDriveEngine,
    Pacejka,
)
from minilink.graphical.animation.primitives import camera_matrix


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


def create_vehicle(X=0.0, Y=0.0, theta=0.0, vx=0.0, vy=0.0, r=0.0):
    vehicle = DynamicBicycleRearWheelDriveEngine()

    vehicle.r_f = 0.3429
    vehicle.r_r = 0.3429

    vehicle.wheel_len_rear = vehicle.r_r * 2
    vehicle.wheel_width_rear = 0.2794
    vehicle.wheel_len_front = vehicle.r_f * 2
    vehicle.wheel_width_front = 0.2286

    vehicle.bw_rear = 0.0
    vehicle.bw_front = 0.0

    vehicle.Jw_rear = 1.6
    vehicle.Jw_front = 1.3

    vehicle.a = 1.16
    vehicle.b = 0.95
    vehicle.L = vehicle.a + vehicle.b

    vehicle.mass = 698.8
    vehicle.inertia = 700.0

    vehicle.gravity = 9.81
    vehicle.rho = 1.225

    vehicle.CdA = 0.0

    vehicle.tire_model_f = Pacejka()
    vehicle.tire_model_r = Pacejka()

    vehicle.engine_power_peak = 48470.5

    vehicle.transmission_ratio = 1.0

    vehicle.engine_dry_resistance = 8.0
    vehicle.engine_rolling_resistance = 0.025

    vehicle.engine_tau = 0.25
    vehicle.steering_tau = 0.15

    vehicle.x0 = np.array(
        [
            X,  # X
            Y,  # Y
            theta,  # theta
            0.0,  # phi_rear
            0.0,  # phi_front
            vx,  # vx
            vy,  # vy
            r,  # r
            vx / vehicle.r_r,  # w_rear
            vx / vehicle.r_f,  # w_front
            0.0,  # tau_engine
            0.0,  # delta_act
        ],
        dtype=float,
    )

    return vehicle
