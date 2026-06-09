import matplotlib.pyplot as plt
import numpy as np

from projects.bicycle.control.measurement import Measurement
from projects.bicycle.control.pid import InstrumentedPID as PID
from minilink.core.diagram import DiagramSystem
from projects.bicycle.control.motor_map import AccToRearForce, ThrMap
from projects.bicycle.models.dynamic_bicycle import (
    DynamicBicycleRearWheelDriveEngine,
)
from projects.bicycle.simulations.vehicule_helper import (
    attach_vehicle_centered_diagram_camera,
    create_vehicle,
)

VX_REF = 30.0  # m/s
R_REF = 0.6  # rad/s


def create_diagram(
    vehicle: DynamicBicycleRearWheelDriveEngine, vx_ref=VX_REF, r_ref=R_REF
):

    rear_speed_meas = Measurement(
        name="Rear wheel speed",
        y_size=10,
        index=6,
    )

    speed_meas = Measurement(name="Speed measurement", y_size=10, index=3, show=False)
    ang_speed_meas = Measurement(
        name="Angular speed measurement", y_size=10, index=5, show=False
    )

    r_pid = PID(
        Kp=33.0,
        Ki=15.0,
        Kd=0.25,
        cmd_min=-np.pi / 4.0,
        cmd_max=np.pi / 4.0,
        name="Yaw rate PID",
    )

    v_pid = PID(
        Kp=0.8,
        Ki=0.01,
        Kd=0.0,
        cmd_min=-10.0,
        cmd_max=10.0,
        name="Speed PID",
    )

    # Constant references as default (nominal) values on the unconnected r ports.
    r_pid.inputs["r"].nominal_value = np.array([r_ref])
    v_pid.inputs["r"].nominal_value = np.array([vx_ref])

    thr_map = ThrMap(vehicle)
    acc_to_force = AccToRearForce(vehicle)

    diagram = DiagramSystem()
    diagram.name = "Steering PID - DynamicBicycleRearWheelDriveEngine"

    diagram.add_subsystem(vehicle, "vehicle")
    diagram.add_subsystem(r_pid, "r_pid")
    diagram.add_subsystem(ang_speed_meas, "ang_speed_meas")

    diagram.add_subsystem(acc_to_force, "acc_to_force")
    diagram.add_subsystem(thr_map, "thr_map")
    diagram.add_subsystem(v_pid, "v_pid")
    diagram.add_subsystem(rear_speed_meas, "rear_speed_meas")
    diagram.add_subsystem(speed_meas, "speed_meas")
    # Constant steering command

    diagram.connect("speed_meas", "meas", "v_pid", "y")
    diagram.connect("v_pid", "u", "acc_to_force", "acc_targ")

    diagram.connect("vehicle", "y", "speed_meas", "y")
    diagram.connect("vehicle", "y", "ang_speed_meas", "y")
    diagram.connect("vehicle", "y", "rear_speed_meas", "y")

    diagram.connect("ang_speed_meas", "meas", "r_pid", "y")

    diagram.connect("r_pid", "u", "vehicle", "delta")
    diagram.connect("acc_to_force", "F_rear", "thr_map", "F_rear")
    diagram.connect("thr_map", "thr", "vehicle", "thr")
    diagram.connect("rear_speed_meas", "meas", "thr_map", "w_rear")

    return diagram


def main():
    vx = VX_REF

    vehicle = create_vehicle(vx=vx)
    # To keep the constant Vx value seems like a reasonable solution (??)

    diagram = create_diagram(vehicle, vx_ref=vx)

    diagram.plot_diagram()

    print("Starting trajectory computation...")

    diagram.compute_trajectory(
        tf=10,
        dt=0.02,
        show=False,
        verbose=False,
    )

    print("Trajectory computation done.")

    # diagram.plot_trajectory(
    #     signals=("x", "u"),
    #     backend="matplotlib",
    # )

    traj = diagram.reconstruct_internal_signals(diagram.traj)
    pid_logs = traj.get_signal("r_pid:logs")

    t = traj.t
    ref = pid_logs[0, :]
    meas = pid_logs[1, :]

    plt.figure()
    plt.plot(t, ref, label="Reference speed")
    plt.plot(t, meas, label="Measured speed")
    plt.xlabel("Time [s]")
    plt.ylabel("Speed [m/s]")
    plt.title("Ang speed tracking")
    plt.legend()
    plt.grid(True)
    plt.show()

    pid_logs = traj.get_signal("v_pid:logs")
    ref = pid_logs[0, :]
    meas = pid_logs[1, :]

    plt.figure()
    plt.plot(t, ref, label="Reference speed")
    plt.plot(t, meas, label="Measured speed")
    plt.xlabel("Time [s]")
    plt.ylabel("Speed [m/s]")
    plt.title("Speed tracking")
    plt.legend()
    plt.grid(True)
    plt.show()

    attach_vehicle_centered_diagram_camera(diagram, vehicle)

    diagram.animate(renderer="matplotlib")


if __name__ == "__main__":
    main()
