"""Vehicle construction helpers for Simon's bicycle LOS project."""

import numpy as np

from examples.projects.simon_bicycle_los.engine_bicycle import (
    DynamicBicycleRearWheelDriveEngine,
)
from examples.projects.simon_bicycle_los.tire_models import Pacejka


def attach_vehicle_centered_diagram_camera(
    diagram_sys, plant, *, plant_sys_id: str = "vehicle"
) -> None:
    """Use the current minilink camera-follow hint for the vehicle body."""
    diagram_sys.camera_follow_frame = f"{plant_sys_id}:body"
    diagram_sys.camera_plot_axes = tuple(plant.camera_plot_axes)
    diagram_sys.camera_scale = float(plant.camera_scale)
    diagram_sys.camera_target = np.asarray(plant.camera_target, dtype=float).copy()


def create_vehicle(X=0.0, Y=0.0, theta=0.0, vx=0.0, vy=0.0, r=0.0, tire_slip_mode=None):
    """Create Simon's Unity-tuned engine bicycle."""
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

    vehicle.tire_model_f = Pacejka(combined_slip_mode=tire_slip_mode)
    vehicle.tire_model_r = Pacejka(combined_slip_mode=tire_slip_mode)

    vehicle.engine_power_peak = 48470.5
    vehicle.transmission_ratio = 1.0
    vehicle.engine_dry_resistance = 8.0
    vehicle.engine_rolling_resistance = 0.025

    vehicle.engine_tau = 0.25
    vehicle.steering_tau = 0.15

    vehicle.max_steer = np.pi / 4.0
    vehicle.min_steer = -np.pi / 4.0

    vehicle.x0 = np.array(
        [
            X,
            Y,
            theta,
            vx,
            vy,
            r,
            vx / vehicle.r_r,
            vx / vehicle.r_f,
            0.0,
            0.0,
        ],
        dtype=float,
    )

    return vehicle
