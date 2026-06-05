import matplotlib.pyplot as plt
import numpy as np

from minilink.control.constant_ref import ConstantReference
from minilink.control.generic_meas import Measurement
from minilink.control.generic_pid import PID
from minilink.control.motor_map import AccToRearForce, ThrMap
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
VX_REF = 5.0  # m/s
DELTA_REF = 0.0  # rad


def create_diagram(vehicle: DynamicBicycleRearWheelDriveEngine, vx_ref=VX_REF):
    v_ref = ConstantReference(ref=vx_ref, name="Speed reference")

    speed_meas = Measurement(
        name="Speed measurement",
        y_size=12,
        index=5,
    )

    rear_speed_ref = ConstantReference(ref=14.5, name="Rear wheel speed")

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
    diagram.add_subsystem(rear_speed_ref, "rear_speed_ref")
    diagram.add_subsystem(speed_meas, "speed_meas")

    diagram.add_subsystem(steering, "steering")
    diagram.add_subsystem(vehicle, "vehicle")

    # Reference acceleration into PID
    diagram.connect("v_ref", "ref", "v_pid", "ref")
    diagram.connect("speed_meas", "meas", "v_pid", "meas")
    diagram.connect("v_pid", "cmd", "acc_to_force", "acc_targ")

    # Vehicle output vector into acceleration measurement block
    diagram.connect("vehicle", "y", "speed_meas", "y")

    # Scalar measured acceleration into PID
    diagram.connect("rear_speed_ref", "ref", "thr_map", "w_rear")

    # PID command drives throttle
    diagram.connect("acc_to_force", "F_rear", "thr_map", "F_rear")
    diagram.connect("thr_map", "thr", "vehicle", "thr")

    # Constant steering command
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
        verbose=False,
    )

    print("Trajectory computation done.")

    from minilink.graphical.signals import (
        build_data_signal_to_plot,
        build_signal_plot_spec,
    )
    from minilink.graphical.signals.matplotlib_backend import (
        render_matplotlib_signal_plot,
    )

    traj = diagram.reconstruct_internal_signals(diagram.traj)
    logs = traj.get_signal("vehicle:r_tire_datas")

    Fx = logs[0, :]
    kappa = logs[1, :]
    # idx = np.argsort(kappa)

    # my_spec = build_data_signal_to_plot(
    #     diagram,
    #     traj,
    #     datas=("vehicle:logs",),
    #     signals_names=("vehicle:logs[0]",),
    #     x_label="vehicle:logs[1]",
    # )
    # render_matplotlib_signal_plot(my_spec, show=True)
    tire = vehicle.tire_model_r

    x = np.linspace(np.min(kappa), np.max(kappa), 20000)

    Fz_r = (
        diagram.subsystems["vehicle"].mass
        * diagram.subsystems["vehicle"].gravity
        * (diagram.subsystems["vehicle"].a / diagram.subsystems["vehicle"].L)
    )

    Fx_model = (
        tire.Dx
        * Fz_r
        * np.sin(
            tire.Cx
            * np.arctan(tire.Bx * x - tire.Ex * (tire.Bx * x - np.arctan(tire.Bx * x)))
        )
    )

    plt.figure()
    # Magic Formula curve
    plt.plot(x, Fx_model, label="Magic Formula", linewidth=1, color="red")
    # Simulation data
    plt.scatter(
        kappa,
        Fx,
        s=5,
        alpha=0.3,
        label="Simulation",
    )
    plt.xlabel("Slip ratio κ")
    plt.ylabel("Longitudinal Force Fx")
    plt.title("Slip vs Magic Formula Comparison")
    plt.grid(True)
    plt.legend()
    plt.show()

    pid_logs = traj.get_signal("v_pid:logs")

    t = traj.t
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

    # spec = build_signal_plot_spec(diagram, traj, signals=("vehicle:logs"))
    # render_matplotlib_signal_plot(spec, show=True)

    # diagram.plot_trajectory(
    #     signals=("vehicle:logs"),
    #     backend="matplotlib",
    # )

    diagram.plot_trajectory(
        signals=("x"),
        backend="matplotlib",
    )

    # # diagram.plot_data()

    attach_vehicle_centered_diagram_camera(diagram, vehicle)

    diagram.animate(renderer="matplotlib")


if __name__ == "__main__":
    main()
