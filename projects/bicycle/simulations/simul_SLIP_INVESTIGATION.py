"""Open-loop demo for DynamicBicycleRearWheelDrive.

No controller.
No velocity PID.
No path tracking.

The vehicle receives constant open-loop inputs:
- throotle: Normalized throttle [0, 1]
- delta: front steering angle [rad]
"""

import matplotlib.pyplot as plt
import numpy as np

from minilink.core.diagram import DiagramSystem
from minilink.core.system import System
from projects.bicycle.simulations.vehicule_helper import (
    create_vehicle,
)

THR_REF = 1.0  # Normalized throttle [0, 1]
DELTA_REF = 0.0  # rad


class ConstantVehicleInput(System):
    """Constant open-loop inputs for DynamicBicycleRearWheelDriveEngine.

    This is not a controller. It only provides fixed inputs required by the model.

    Outputs
    -------
    thr
        Engine throttle, normalized [0, 1].
    delta
        Front steering angle [rad].
    """

    def __init__(self, thr: float = THR_REF, delta: float = DELTA_REF):
        super().__init__(0)

        self.name = "Constant vehicle input"

        self.inputs = {}
        self.outputs = {}
        self.recompute_input_properties()

        self.thr = float(thr)
        self.delta = float(delta)

        self.add_output_port(
            "thr",
            dim=1,
            function=self.h_thr,
            dependencies=[],
        )

        self.add_output_port(
            "delta",
            dim=1,
            function=self.h_delta,
            dependencies=[],
        )

        self.p = 2

    def h_thr(self, x, u, t=0.0, params=None):
        return np.array([self.thr], dtype=float)

    def h_delta(self, x, u, t=0.0, params=None):
        return np.array([self.delta], dtype=float)


def main():
    vehicle = create_vehicle()

    initial_vx = 1.0
    initial_wr = -1.0

    vehicle.x0 = np.array(
        [
            0.0,  # X
            0.0,  # Y
            0.0,  # theta
            initial_vx,  # vx
            0.0,  # vy
            0.0,  # r
            initial_wr,  # w_rear
            0.0,  # w_front
            0.0,  # tau_engine
            0.0,  # delta_act
        ],
        dtype=float,
    )

    constant_input = ConstantVehicleInput(
        thr=THR_REF,
        delta=DELTA_REF,
    )

    diagram = DiagramSystem()
    diagram.name = "Open-loop DynamicBicycleRearWheelDrive"

    diagram.add_subsystem(constant_input, "constant_input")
    diagram.add_subsystem(vehicle, "vehicle")

    # diagram.connect("constant_input", "thr", "vehicle", "thr")
    # diagram.connect("constant_input", "delta", "vehicle", "delta")

    diagram.plot_diagram()

    print("Starting trajectory computation...")

    diagram.compute_trajectory(
        tf=5,
        dt=0.005,
        show=False,
        verbose=False,
        solver="euler",
    )

    print("Trajectory computation done.")

    # diagram.plot_trajectory(
    #     signals=("x", "u"),
    #     backend="matplotlib",
    # )
    tire = vehicle.tire_model_r
    print(f"LEN ILLLEGAL:{len(tire.kappa_log)} ")

    traj = diagram.reconstruct_internal_signals(diagram.traj)
    logs = traj.get_signal("vehicle:r_tire_datas")

    Fx = logs[0, :]
    kappa = logs[1, :]

    # ================================================================================================================================
    wr = initial_wr * vehicle.r_r
    vx = initial_vx
    denom = np.maximum(
        np.maximum(np.abs(vx), np.abs(wr)), vehicle.tire_model_r.v_min_epsilon
    )

    kappa_true = (wr - vx) / denom

    print(f"TRAJ reconstruit:{kappa[0]}|| True value:{kappa_true} ")
    print(
        "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"
    )
    print(f"LEN TRAJ reconstruit:{len(kappa)}|| LEN ILLLEGAL:{len(tire.kappa_log)} ")
    # ================================================================================================================================

    all_kappa = tire.kappa_log + list(kappa)
    x = np.linspace(np.min(all_kappa), np.max(all_kappa), 20000)

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

    plt.plot(
        x,
        Fx_model,
        label="Magic Formula",
        linewidth=1,
        color="red",
        alpha=0.25,  # very transparent line
    )

    # Simulation data
    plt.scatter(
        tire.kappa_log,
        tire.Fx_log,
        s=2,  # smaller green points
        alpha=0.15,  # more transparent
        color="green",
        label="Append",
    )

    plt.scatter(
        kappa,
        Fx,
        s=12,  # bigger blue points
        alpha=1.0,  # solid color
        color="blue",
        label="Reconstruction",
    )

    plt.xlabel("Slip ratio κ")
    plt.ylabel("Longitudinal Force Fx")
    plt.title("LEGAL VS ILLEGAL")
    plt.grid(True)
    plt.legend()
    plt.show()

    # attach_vehicle_centered_diagram_camera(diagram, vehicle)

    # diagram.animate(renderer="matplotlib")
    # diagram.animate(renderer="meshcat")
    # diagram.animate(renderer="plotly")


if __name__ == "__main__":
    main()
