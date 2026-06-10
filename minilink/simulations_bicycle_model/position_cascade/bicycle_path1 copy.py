import matplotlib.pyplot as plt
import numpy as np
from minilink.simulations_bicycle_model.path.path_plot import PathPlanner

from minilink.control.constant_ref import ConstantReference
from minilink.control.full_bicycle_meas import BicycleMeasurement
from minilink.control.generic_pid import PID, Sum
from minilink.control.motor_map import AccToRearForce, ThrMap
from minilink.control.steering_map import AngularSpeedToSteeringMap
from minilink.core.diagram import DiagramSystem
from minilink.core.system import System
from minilink.dynamics.catalog.vehicles.dynamic_bicycle import (
    DynamicBicycleRearWheelDriveEngine,
)
from minilink.graphical.animation.primitives import (
    Circle,
    CustomLine,
    Point,
    pose2d_matrix,
)
from minilink.simulations_bicycle_model.vehicule_helper import (
    attach_vehicle_centered_diagram_camera,
    create_vehicle,
)

VX_REF = 2.0  # m/s
R_REF = 0.0  # rad/s

SIMULATION_TIME = 10  # seconds

_PATH_X0 = -8.0
_PATH_X1 = 95.0


def x_pos(t):
    return t


def y_line(t):
    return 3.0


def create_diagram(
    vehicle: DynamicBicycleRearWheelDriveEngine, vx_ref=VX_REF, r_ref=R_REF
):

    r_to_steering = AngularSpeedToSteeringMap(vehicle)
    speed_const = ConstantReference(ref=vx_ref, name="Constant linear speed")

    full_state_meas = BicycleMeasurement(name="Meas states", y_size=10)

    v_pid = PID(
        Kp=0.8,
        Ki=0.01,
        Kd=0.0,
        cmd_min=-10.0,
        cmd_max=10.0,
        i_min=-0.0,
        i_max=5.0,
        name="Speed PID",
    )

    thr_map = ThrMap(vehicle)
    acc_to_force = AccToRearForce(vehicle)

    theta_pid = PID(
        Kp=2.5,
        Ki=0.2,
        Kd=1.3,
        cmd_min=-10.0,
        cmd_max=10.0,
        i_min=-1.0,
        i_max=1.0,
        name="Yaw rate PID",
    )

    r_pid = PID(
        Kp=0.0,
        Ki=0.2,
        Kd=0.0,
        cmd_min=-np.pi / 4.0,
        cmd_max=np.pi / 4.0,
        i_min=-np.pi / 4.0,
        i_max=np.pi / 4.0,
        name="Yaw rate PID",
    )

    sum_bloc = Sum(max=np.pi / 2.0, min=-np.pi / 2.0)

    trajectory = PathPlanner(x_traj=x_pos, y_traj=y_line)

    diagram = DiagramSystem()
    diagram.name = "Cascade PID - DynamicBicycleRearWheelDriveEngine"

    diagram.add_subsystem(r_to_steering, "r_to_steering")
    diagram.add_subsystem(vehicle, "vehicle")
    diagram.add_subsystem(r_pid, "r_pid")
    diagram.add_subsystem(theta_pid, "theta_pid")

    diagram.add_subsystem(acc_to_force, "acc_to_force")
    diagram.add_subsystem(thr_map, "thr_map")
    diagram.add_subsystem(v_pid, "v_pid")

    diagram.add_subsystem(full_state_meas, "full_state_meas")
    diagram.add_subsystem(sum_bloc, "sum_bloc")

    diagram.add_subsystem(trajectory, "trajectory")
    diagram.add_subsystem(speed_const, "speed_const")

    diagram.connect("speed_const", "ref", "v_pid", "ref")
    diagram.connect("trajectory", "y_targ", "theta_pid", "ref")

    # diagram.connect("speed_meas", "meas", "v_pid", "meas")
    diagram.connect("full_state_meas", "vx_meas", "v_pid", "meas")

    diagram.connect("v_pid", "cmd", "acc_to_force", "acc_targ")

    diagram.connect("vehicle", "y", "full_state_meas", "y")
    diagram.connect("theta_pid", "cmd", "r_pid", "ref")
    diagram.connect("theta_pid", "cmd", "r_to_steering", "r_targ")

    diagram.connect("full_state_meas", "vx_meas", "r_to_steering", "vx_meas")
    diagram.connect("full_state_meas", "r_meas", "r_pid", "meas")

    diagram.connect("full_state_meas", "y_meas", "theta_pid", "meas")

    diagram.connect("r_pid", "cmd", "sum_bloc", "1")
    diagram.connect("r_to_steering", "delta", "sum_bloc", "2")

    diagram.connect("sum_bloc", "result", "vehicle", "delta")
    diagram.connect("acc_to_force", "F_rear", "thr_map", "F_rear")
    diagram.connect("thr_map", "thr", "vehicle", "thr")
    diagram.connect("full_state_meas", "w_r_meas", "thr_map", "w_rear")

    return diagram


def main():
    vehicle = create_vehicle(vx=VX_REF)
    diagram = create_diagram(vehicle, vx_ref=VX_REF)

    # diagram.plot_diagram()

    print("Starting trajectory computation...")

    diagram.compute_trajectory(
        tf=SIMULATION_TIME,
        dt=0.02,
        show=False,
        verbose=False,
    )

    print("Trajectory computation done.")

    # diagram.plot_trajectory(
    #     signals=("x", "u"),
    #     backend="matplotlib",
    # )

    # traj = diagram.reconstruct_internal_signals(diagram.traj)
    # pid_logs = traj.get_signal("v_pid:logs")

    # ref = pid_logs[0, :]
    # meas = pid_logs[1, :]

    # t = traj.t

    # plt.figure()
    # plt.plot(t, ref, label="Goal Speed")
    # plt.plot(t, meas, label="Measured Speed")
    # plt.xlabel("Time [s]")
    # plt.ylabel("Speed [m/s]")
    # plt.title("Speed PID - Reference vs Measured")
    # plt.legend()
    # plt.grid(True)
    # plt.show()

    traj = diagram.reconstruct_internal_signals(diagram.traj)
    pid_logs = traj.get_signal("theta_pid:logs")

    ref = pid_logs[0, :]
    meas = pid_logs[1, :]

    t = traj.t

    plt.figure()
    plt.plot(t, ref, label="y meas")
    plt.plot(t, meas, label="y goal")
    plt.xlabel("Time [s]")
    plt.ylabel("Y pos [m]")
    plt.title("Theta POS - Reference vs Measured")
    plt.legend()
    plt.grid(True)
    plt.show()

    attach_vehicle_centered_diagram_camera(diagram, vehicle)

    diagram.animate(renderer="matplotlib")


if __name__ == "__main__":
    main()
