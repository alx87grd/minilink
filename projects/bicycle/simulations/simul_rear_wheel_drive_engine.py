"""Open-loop demo for DynamicBicycleRearWheelDrive.

No controller.
No velocity PID.
No path tracking.

The vehicle receives constant open-loop inputs:
- throotle: Normalized throttle [0, 1]
- delta: front steering angle [rad]
"""

import numpy as np

from minilink.core.diagram import DiagramSystem
from minilink.core.system import System
from projects.bicycle.simulations.vehicule_helper import (
    attach_vehicle_centered_diagram_camera,
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
        # thr = self.thr - 0.2 * t
        # if thr < 0.0:
        #     thr = 0.0
        if t > 5.0:
            thr = 0.0
        else:
            thr = self.thr
        return np.array([thr], dtype=float)

    def h_delta(self, x, u, t=0.0, params=None):
        return np.array([self.delta], dtype=float)


def main():
    vehicle = create_vehicle()
    # vehicle.engine_power_peak = vehicle.engine_power_peak / 1000

    vehicle.x0 = np.array(
        [
            0.0,  # X
            0.0,  # Y
            0.0,  # theta
            0.0,  # phi_rear
            0.0,  # phi_front
            0.0,  # vx
            0.0,  # vy
            0.0,  # r
            0.0,  # w_rear
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

    diagram.connect("constant_input", "thr", "vehicle", "thr")
    diagram.connect("constant_input", "delta", "vehicle", "delta")

    # diagram.plot_diagram()

    print("Starting trajectory computation...")

    diagram.compute_trajectory(
        tf=10,
        dt=0.005,
        show=False,
        verbose=False,
        solver="euler",
    )

    print("Trajectory computation done.")

    diagram.plot_trajectory(
        signals=("x", "u"),
        backend="matplotlib",
    )

    # traj = diagram.reconstruct_internal_signals(diagram.traj)
    # pid_logs = traj.get_signal("constant_input:thr")

    # thr = pid_logs[0, :]

    # t = traj.t

    # plt.figure()
    # plt.plot(t, thr, label="Goal speed")
    # plt.xlabel("Time [s]")
    # plt.ylabel("Throttle command [-]")
    # plt.title("Throttle command")
    # plt.legend()
    # plt.grid(True)
    # plt.show()

    attach_vehicle_centered_diagram_camera(diagram, vehicle)

    diagram.animate(renderer="matplotlib")
    # diagram.animate(renderer="meshcat")
    # diagram.animate(renderer="plotly")


if __name__ == "__main__":
    main()
