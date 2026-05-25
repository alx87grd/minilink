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
    # vehicle = create_vehicle()

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
        tau=0.1,
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

    return diagram, v_pid


def main():
    vehicle = create_vehicle()
    vehicle.tire_model_r = Pacejka(logs=True)
    diagram, v_pid = create_diagram(vehicle)

    # diagram.plot_graphe()

    print("Starting trajectory computation...")

    diagram.compute_trajectory(
        tf=10.0,
        dt=0.005,
        show=False,
        verbose=False,
    )

    print("Trajectory computation done.")

    tire = vehicle.tire_model_r

    kappa_sim = np.array(tire.kappa)
    alpha_sim = np.array(tire.alpha)
    Fx_sim = np.array(tire.Fx_log)

    x = np.linspace(np.min(kappa_sim), np.max(kappa_sim), 20000)

    Fx_model = (
        tire.Dx
        * tire.Fz
        * np.sin(
            tire.Cx
            * np.arctan(tire.Bx * x - tire.Ex * (tire.Bx * x - np.arctan(tire.Bx * x)))
        )
    )

    import matplotlib.pyplot as plt

    plt.figure()

    # Magic Formula curve
    plt.plot(x, Fx_model, label="Magic Formula", linewidth=1, color="red")

    # Simulation data
    plt.scatter(kappa_sim, Fx_sim, s=5, alpha=0.3, label="Simulation")

    plt.xlabel("Slip ratio κ")
    plt.ylabel("Longitudinal Force Fx")
    plt.title("Slip vs Magic Formula Comparison")
    plt.grid(True)
    plt.legend()
    plt.show()

    diagram.plot_trajectory(
        signals=("x", "u"),
        backend="matplotlib",
    )

    # t = np.array(v_pid.t_hist)
    # ref = np.array(v_pid.ref_hist)
    # meas = np.array(v_pid.meas_hist)

    # idx = np.argsort(t)
    # t = t[idx]
    # ref = ref[idx]
    # meas = meas[idx]

    # t_unique, unique_idx = np.unique(t, return_index=True)
    # ref = ref[unique_idx]
    # meas = meas[unique_idx]
    # t = t_unique

    # plt.figure()
    # plt.plot(t, ref, label="Reference speed")
    # plt.plot(t, meas, label="Measured speed")
    # plt.xlabel("Time [s]")
    # plt.ylabel("Speed [m/s]")
    # plt.title("Speed tracking")
    # plt.legend()
    # plt.grid(True)
    # plt.show()

    # attach_vehicle_centered_diagram_camera(diagram, vehicle)

    # diagram.animate(renderer="matplotlib")


if __name__ == "__main__":
    main()
