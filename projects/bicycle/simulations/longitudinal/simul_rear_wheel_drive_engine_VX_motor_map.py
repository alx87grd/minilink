
from minilink.control.measurement import Measurement
from minilink.control.pid import PID
from minilink.control.references import ConstantReference
from minilink.core.diagram import DiagramSystem
from projects.bicycle.control.motor_map import AccToRearForce, ThrMap
from projects.bicycle.models.dynamic_bicycle import (
    DynamicBicycleRearWheelDriveEngine,
)
from projects.bicycle.simulations.vehicule_helper import (
    create_vehicle,
)

VX_REF = 5.0
DELTA_REF = 0.0


def create_diagram(vehicle: DynamicBicycleRearWheelDriveEngine, vx_ref=VX_REF):

    v_ref = ConstantReference(ref=vx_ref, name="Speed reference")

    speed_meas = Measurement(
        name="Speed measurement",
        y_size=10,
        index=3,
    )

    rear_speed_meas = Measurement(
        name="Rear wheel speed",
        y_size=10,
        index=6,
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

    steering = ConstantReference(ref=DELTA_REF, name="Constant steering")
    thr_map = ThrMap(vehicle)
    acc_to_force = AccToRearForce(vehicle)

    diagram = DiagramSystem()
    diagram.name = "Acceleration PID - DynamicBicycleRearWheelDriveEngine"

    diagram.add_subsystem(acc_to_force, "acc_to_force")
    diagram.add_subsystem(thr_map, "thr_map")

    diagram.add_subsystem(v_ref, "v_ref")
    diagram.add_subsystem(v_pid, "v_pid")
    diagram.add_subsystem(rear_speed_meas, "rear_speed_meas")
    diagram.add_subsystem(speed_meas, "speed_meas")

    diagram.add_subsystem(steering, "steering")
    diagram.add_subsystem(vehicle, "vehicle")

    diagram.connect("v_ref", "ref", "v_pid", "ref")
    diagram.connect("speed_meas", "meas", "v_pid", "meas")
    diagram.connect("v_pid", "cmd", "acc_to_force", "acc_targ")

    diagram.connect("vehicle", "y", "speed_meas", "y")
    diagram.connect("vehicle", "y", "rear_speed_meas", "y")

    diagram.connect("rear_speed_meas", "meas", "thr_map", "w_rear")

    diagram.connect("acc_to_force", "F_rear", "thr_map", "F_rear")
    diagram.connect("thr_map", "thr", "vehicle", "thr")

    diagram.connect("steering", "ref", "vehicle", "delta")

    return diagram


def main():
    vehicle = create_vehicle()
    diagram = create_diagram(vehicle)

    diagram.plot_diagram()

    print("Starting trajectory computation...")

    diagram.compute_trajectory(
        tf=10,
        dt=0.005,
        show=False,
        verbose=False,
    )

    print("Trajectory computation done.")

    diagram.plot_trajectory(
        signals=("x", "u"),
        backend="matplotlib",
    )

    traj = diagram.reconstruct_internal_signals(diagram.traj)
    pid_logs = traj.get_signal("v_pid:logs")

    ref = pid_logs[0, :]
    meas = pid_logs[1, :]

    import matplotlib.pyplot as plt

    plt.figure()
    plt.plot(diagram.traj.t, ref, label="Reference speed")
    plt.plot(diagram.traj.t, meas, label="Measured speed")
    plt.xlabel("Time [s]")
    plt.ylabel("Speed [m/s]")
    plt.title("Speed tracking")
    plt.legend()
    plt.grid(True)
    plt.show()

    # attach_vehicle_centered_diagram_camera(diagram, vehicle)

    # diagram.animate(renderer="matplotlib")


if __name__ == "__main__":
    main()
