"""Open-loop demo for DynamicBicycleRearWheelDrive.

No controller.
No velocity PID.
No path tracking.

The vehicle receives constant open-loop inputs:
- throotle: Normalized throttle [0, 1]
- delta: front steering angle [rad]
"""

import numpy as np

from minilink.control.constant_ref import ConstantReference
from minilink.control.generic_meas import AccelerationMeasurement, Measurement
from minilink.control.generic_pid import PID, Sum, pid_result
from minilink.control.steering_map import AngularSpeedToSteeringMap
from minilink.core.diagram import DiagramSystem
from minilink.dynamics.catalog.vehicles.dynamic_bicycle import (
    DynamicBicycleRearWheelDriveEngine,
    Pacejka,
)
from minilink.simulations_bicycle_model.vehicule_helper import (
    attach_vehicle_centered_diagram_camera,
    create_vehicle,
)

ACC_REF = 1.0
VX_REF = 2.0  # m/s
R_REF = 0.6  # rad/s


def create_diagram(
    vehicle: DynamicBicycleRearWheelDriveEngine, vx_ref=VX_REF, r_ref=R_REF
):

    r_to_steering = AngularSpeedToSteeringMap(vehicle)
    steering = ConstantReference(ref=r_ref, name="Constant angular speed")
    speed_const = ConstantReference(ref=vx_ref, name="Constant linear speed")

    speed_meas = Measurement(name="Speed measurement", y_size=12, index=5, show=False)
    ang_speed_meas = Measurement(
        name="Angular speed measurement", y_size=12, index=7, show=False
    )

    r_pid = PID(
        Kp=33.0,
        Ki=15.0,
        Kd=0.25,
        cmd_min=-np.pi / 4.0,
        cmd_max=np.pi / 4.0,
        i_min=-np.pi / 4.0,
        i_max=np.pi / 4.0,
        name="Yaw rate PID",
    )

    sum_bloc = Sum()

    diagram = DiagramSystem()
    diagram.name = "Steering PID - DynamicBicycleRearWheelDriveEngine"

    diagram.add_subsystem(steering, "steering")
    # diagram.add_subsystem(speed_const, "speed_const")
    # diagram.add_subsystem(r_to_steering, "r_to_steering")
    diagram.add_subsystem(vehicle, "vehicle")
    diagram.add_subsystem(r_pid, "r_pid")
    # diagram.add_subsystem(speed_meas, "speed_meas")
    diagram.add_subsystem(ang_speed_meas, "ang_speed_meas")
    # diagram.add_subsystem(sum_bloc, "sum_bloc")

    # Constant steering command
    # diagram.connect("vehicle", "y", "speed_meas", "y")
    diagram.connect("vehicle", "y", "ang_speed_meas", "y")

    diagram.connect("steering", "ref", "r_pid", "ref")
    # diagram.connect("steering", "ref", "r_to_steering", "r_targ")

    # diagram.connect("speed_const", "ref", "r_to_steering", "vx_meas")
    diagram.connect("ang_speed_meas", "meas", "r_pid", "meas")

    # diagram.connect("r_pid", "cmd", "sum_bloc", "1")
    # diagram.connect("r_to_steering", "delta", "sum_bloc", "2")

    diagram.connect("r_pid", "cmd", "vehicle", "delta")

    return diagram, r_pid


def main():
    vx = VX_REF

    vehicle = create_vehicle(vx=vx)
    # To keep the constant Vx value seems like a reasonable solution (??)
    vehicle.engine_dry_resistance = 0.0
    vehicle.engine_rolling_resistance = 0.0

    diagram, r_pid = create_diagram(vehicle, vx_ref=vx)

    diagram.plot_graphe()

    print("Starting trajectory computation...")

    diagram.compute_trajectory(
        tf=10.0,
        dt=0.02,
        show=False,
        verbose=False,
    )

    print("Trajectory computation done.")

    # diagram.plot_trajectory(
    #     signals=("x", "u"),
    #     backend="matplotlib",
    # )

    pid_result(r_pid)

    attach_vehicle_centered_diagram_camera(diagram, vehicle)

    diagram.animate(renderer="matplotlib")


if __name__ == "__main__":
    main()
