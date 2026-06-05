import matplotlib.pyplot as plt
import numpy as np

from minilink.control.constant_ref import ConstantReference
from minilink.control.generic_meas import Measurement
from minilink.control.generic_pid import PID, Sum
from minilink.control.motor_map import AccToRearForce, ThrMap
from minilink.control.steering_map import AngularSpeedToSteeringMap
from minilink.core.diagram import DiagramSystem
from minilink.dynamics.catalog.vehicles.dynamic_bicycle import (
    DynamicBicycleRearWheelDriveEngine,
)
from minilink.simulations_bicycle_model.vehicule_helper import (
    attach_vehicle_centered_diagram_camera,
    create_vehicle,
)

VX_REF = 5.0  # m/s
R_REF = 0.5  # rad/s


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

    rear_speed_meas = Measurement(
        name="Rear wheel speed",
        y_size=12,
        index=8,
    )

    v_pid = PID(
        Kp=0.8,
        Ki=0.01,
        Kd=0.0,
        cmd_min=-10.0,
        cmd_max=10.0,
        i_min=-5.0,
        i_max=5.0,
        name="Speed PID",
    )

    thr_map = ThrMap(vehicle)
    acc_to_force = AccToRearForce(vehicle)

    r_pid = PID(
        Kp=0.0,  # 3.2,
        Ki=0.2,  # 1.5,
        Kd=0.0,  # 1.0,
        cmd_min=-np.pi / 4.0,
        cmd_max=np.pi / 4.0,
        i_min=-np.pi / 4.0,
        i_max=np.pi / 4.0,
        name="Yaw rate PID",
    )

    sum_bloc = Sum(max=np.pi / 2.0, min=-np.pi / 2.0)

    diagram = DiagramSystem()
    diagram.name = "Steering PID - DynamicBicycleRearWheelDriveEngine"

    diagram.add_subsystem(steering, "steering")
    diagram.add_subsystem(r_to_steering, "r_to_steering")
    diagram.add_subsystem(vehicle, "vehicle")
    diagram.add_subsystem(r_pid, "r_pid")

    diagram.add_subsystem(acc_to_force, "acc_to_force")
    diagram.add_subsystem(thr_map, "thr_map")
    diagram.add_subsystem(speed_const, "speed_const")
    diagram.add_subsystem(v_pid, "v_pid")
    diagram.add_subsystem(rear_speed_meas, "rear_speed_meas")
    diagram.add_subsystem(speed_meas, "speed_meas")

    diagram.add_subsystem(ang_speed_meas, "ang_speed_meas")
    diagram.add_subsystem(sum_bloc, "sum_bloc")

    # Constant steering command

    diagram.connect("speed_const", "ref", "v_pid", "ref")
    diagram.connect("speed_meas", "meas", "v_pid", "meas")
    diagram.connect("v_pid", "cmd", "acc_to_force", "acc_targ")

    diagram.connect("vehicle", "y", "speed_meas", "y")
    diagram.connect("vehicle", "y", "ang_speed_meas", "y")
    diagram.connect("vehicle", "y", "rear_speed_meas", "y")

    diagram.connect("steering", "ref", "r_pid", "ref")
    diagram.connect("steering", "ref", "r_to_steering", "r_targ")

    diagram.connect("speed_meas", "meas", "r_to_steering", "vx_meas")
    diagram.connect("ang_speed_meas", "meas", "r_pid", "meas")

    diagram.connect("r_pid", "cmd", "sum_bloc", "1")
    diagram.connect("r_to_steering", "delta", "sum_bloc", "2")

    diagram.connect("sum_bloc", "result", "vehicle", "delta")
    diagram.connect("acc_to_force", "F_rear", "thr_map", "F_rear")
    diagram.connect("thr_map", "thr", "vehicle", "thr")
    diagram.connect("rear_speed_meas", "meas", "thr_map", "w_rear")

    return diagram


def main():
    vx = VX_REF

    vehicle = create_vehicle(vx=vx)

    diagram = create_diagram(vehicle, vx_ref=vx)

    # diagram.plot_diagram()

    print("Starting trajectory computation...")

    diagram.compute_trajectory(
        tf=10,
        dt=0.02,
        show=False,
        verbose=False,
    )

    print("Trajectory computation done.")

    diagram.plot_trajectory(
        signals=("x"),
        backend="matplotlib",
    )

    traj = diagram.reconstruct_internal_signals(diagram.traj)
    pid_logs = traj.get_signal("r_pid:logs")
    t = traj.t

    ref = pid_logs[0, :]
    meas = pid_logs[1, :]

    plt.figure()
    plt.plot(t, ref, label="Reference speed")
    plt.plot(t, meas, label="Measured speed")
    plt.xlabel("Time [s]")
    plt.ylabel("Speed [rad/s]")
    plt.title("Ang speed tracking")
    plt.legend()
    plt.grid(True)
    plt.show()

    # pid_logs = traj.get_signal("r_pid:pid_int_value")
    # ref = pid_logs[0, :]
    # # meas = pid_logs[1, :]

    # plt.figure()
    # plt.plot(t, ref, label="Reference speed")
    # # plt.plot(t, meas, label="Measured speed")
    # plt.xlabel("Time [s]")
    # plt.ylabel("d*d_e [rad]")
    # plt.title("Pid values")
    # plt.legend()
    # plt.grid(True)
    # plt.show()

    attach_vehicle_centered_diagram_camera(diagram, vehicle)

    diagram.animate(renderer="matplotlib")


if __name__ == "__main__":
    main()
