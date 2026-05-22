"""Open-loop demo for DynamicBicycleRearWheelDrive.

No controller.
No velocity PID.
No path tracking.

The vehicle receives constant open-loop inputs:
- throotle: Normalized throttle [0, 1]
- delta: front steering angle [rad]
"""

import types

import numpy as np

from minilink.core.diagram import DiagramSystem
from minilink.core.system import DynamicSystem, System
from minilink.dynamics.catalog.vehicles.combustion_engine_ESTIMATION_hd9 import (
    max_torque_curve,
)
from minilink.dynamics.catalog.vehicles.dynamic_bicycle import (
    DynamicBicycleRearWheelDriveEngine,
    LinearTire,
    Pacejka,
)
from minilink.graphical.primitives import camera_matrix

ACC_REF = 1.0
VX_REF = 5.0  # m/s
DELTA_REF = 0.0  # rad


def attach_vehicle_centered_diagram_camera(
    diagram_sys, plant, *, plant_sys_id: str = "vehicle"
) -> None:
    """Animate the composed diagram using a camera target on the plant's ``(x, y)``."""

    ix = diagram_sys.state_index[plant_sys_id][0]

    def get_camera_transform(self, x, _u, _t):
        target = np.asarray(plant.camera_target, dtype=float).reshape(3).copy()
        target[0] += float(x[ix])
        target[1] += float(x[ix + 1])
        return camera_matrix(
            target=target,
            plot_axes=self.camera_plot_axes,
            scale=self.camera_scale,
        )

    diagram_sys.camera_plot_axes = tuple(plant.camera_plot_axes)
    diagram_sys.camera_scale = float(plant.camera_scale)
    diagram_sys.camera_target = (
        np.asarray(plant.camera_target, dtype=float).reshape(3).copy()
    )
    diagram_sys.get_camera_transform = types.MethodType(
        get_camera_transform, diagram_sys
    )


class PID(DynamicSystem):
    """Generic scalar PID controller.

    Inputs
    ------
    ref
        Reference signal.
    meas
        Measured signal.

    Output
    ------
    cmd
        PID command.

    State
    -----
    x = [int_e, meas_filt]

    Notes
    -----
    Error is:

        e = ref - meas

    PID law is:

        cmd = Kp * e + Ki * int_e - Kd * d_meas_filt

    where the derivative is computed on the filtered measurement, not directly
    on the error. This avoids derivative kick when the reference changes.
    """

    def __init__(
        self,
        Kp: float = 1.0,
        Ki: float = 0.0,
        Kd: float = 0.0,
        tau: float = 0.05,
        cmd_min: float = -np.inf,
        cmd_max: float = np.inf,
        i_min: float = -np.inf,
        i_max: float = np.inf,
        name: str = "PID",
    ):
        # 2 states:
        # x[0] = integral of error
        # x[1] = filtered measurement
        #
        # 2 scalar inputs:
        # ref, meas
        #
        # 1 scalar output:
        # cmd
        super().__init__(2, 2, 1)

        self.name = name

        self.params = {
            "Kp": float(Kp),
            "Ki": float(Ki),
            "Kd": float(Kd),
            "tau": float(tau),
            "cmd_min": float(cmd_min),
            "cmd_max": float(cmd_max),
            "i_min": float(i_min),
            "i_max": float(i_max),
        }

        self.state.labels = ["int_e", "meas_filt"]
        self.state.units = ["", ""]

        self.x0 = np.array([0.0, 0.0], dtype=float)

        self.t_hist = []
        self.ref_hist = []
        self.meas_hist = []

        self.inputs = {}

        self.add_input_port(
            1,
            "ref",
            nominal_value=np.array([0.0]),
        )

        self.add_input_port(
            1,
            "meas",
            nominal_value=np.array([0.0]),
        )

        self.outputs = {}

        self.add_output_port(
            1,
            "cmd",
            function=self.h_cmd,
            dependencies=["ref", "meas"],
        )

        self.add_output_port(
            2,
            "x",
            function=self.compute_state,
            dependencies=[],
        )

    def _signals(self, u):
        ref = float(u[0])
        meas = float(u[1])
        e = ref - meas
        return ref, meas, e

    def f(self, x, u, t=0.0, params=None):
        p = self.params if params is None else params

        int_e = float(x[0])
        meas_filt = float(x[1])

        _, meas, e = self._signals(u)

        tau = max(float(p["tau"]), 1e-9)

        # Integrator with simple clamp protection.
        d_int_e = e

        if int_e >= p["i_max"] and e > 0.0:
            d_int_e = 0.0
        elif int_e <= p["i_min"] and e < 0.0:
            d_int_e = 0.0

        # First-order measurement filter.
        d_meas_filt = (meas - meas_filt) / tau

        return np.array(
            [
                d_int_e,
                d_meas_filt,
            ],
            dtype=float,
        )

    def h_cmd(self, x, u, t=0.0, params=None):
        p = self.params if params is None else params

        int_e = float(x[0])
        meas_filt = float(x[1])

        ref, meas, e = self._signals(u)

        tau = max(float(p["tau"]), 1e-9)

        # Derivative-on-measurement.
        d_meas_filt = (meas - meas_filt) / tau

        cmd = p["Kp"] * e + p["Ki"] * int_e - p["Kd"] * d_meas_filt

        # cmd += ref

        cmd = np.clip(
            cmd,
            p["cmd_min"],
            p["cmd_max"],
        )
        # DEBUG
        # print(f"acc_cmd={cmd}, e={e}")

        # Log signals
        self.t_hist.append(float(t))
        self.ref_hist.append(ref)
        self.meas_hist.append(meas)

        return np.array([cmd], dtype=float)

    def get_kinematic_geometry(self):
        return []

    def get_kinematic_transforms(self, _x, _u, _t):
        return []


class ConstantReference(System):
    """Constant scalar reference signal."""

    def __init__(self, ref: float = 1.0, name: str | None = None):
        super().__init__(0, 0, 1)

        self.name = name if name is not None else "Not named :("

        self.inputs = {}
        self.outputs = {}
        self.recompute_input_properties()

        self.ref = float(ref)

        self.add_output_port(
            1,
            "ref",
            function=self.h_ref,
            dependencies=[],
        )

    def h_ref(self, x, u, t=0.0, params=None):
        return np.array([self.ref], dtype=float)


class ConstantSteering(System):
    """Constant steering command."""

    def __init__(self, delta: float = DELTA_REF):
        super().__init__(0, 0, 1)

        self.name = "Constant steering"

        self.inputs = {}
        self.outputs = {}
        self.recompute_input_properties()

        self.delta = float(delta)

        self.add_output_port(
            1,
            "delta",
            function=self.h_delta,
            dependencies=[],
        )

    def h_delta(self, x, u, t=0.0, params=None):
        return np.array([self.delta], dtype=float)


class Measurement(System):
    """Extract longitudinal speed from vehicle output vector.

    For DynamicBicycleRearWheelDriveEngine:

        y = [
            X, Y, theta,
            phi_rear, phi_front,
            vx, vy, r,
            w_rear, w_front,
            tau_engine, delta_act
        ]

    """

    def __init__(self, name: str, y_size: int = 12, index: int = 5):
        super().__init__(0, y_size, 1)

        self.name = name

        self.y_size = int(y_size)
        self.index = int(index)

        self.inputs = {}
        self.add_input_port(
            self.y_size,
            "y",
            nominal_value=np.zeros(self.y_size),
        )

        self.outputs = {}
        self.add_output_port(
            1,
            "meas",
            function=self.h_meas,
            dependencies=["y"],
        )

    def h_meas(self, x, u, t=0.0, params=None):
        # print(f"w_rear: {u[self.index]}")
        return np.array([u[self.index]], dtype=float)

    def get_kinematic_geometry(self):
        return []

    def get_kinematic_transforms(self, _x, _u, _t):
        return []


class AccelerationMeasurement(DynamicSystem):
    """Estimate longitudinal acceleration from vehicle longitudinal speed.

    For DynamicBicycleRearWheelDriveEngine:

        y = [
            X, Y, theta,
            phi_rear, phi_front,
            vx, vy, r,
            w_rear, w_front,
            tau_engine, delta_act
        ]

    Therefore vx is at index 5.

    This block estimates acceleration as a filtered derivative:

        a_x ~= (vx - vx_filt) / tau

    where vx_filt is a first-order filtered version of vx.
    """

    def __init__(
        self,
        y_size: int = 12,
        speed_index: int = 5,
        tau: float = 0.05,
    ):
        # 1 state:
        # x[0] = filtered vx
        #
        # Input:
        # y = full vehicle output vector
        #
        # Output:
        # meas = estimated longitudinal acceleration
        super().__init__(1, y_size, 1)

        self.name = "Acceleration measurement"

        self.y_size = int(y_size)
        self.speed_index = int(speed_index)
        self.tau = float(tau)

        self.state.labels = ["vx_filt"]
        self.state.units = ["m/s"]

        self.x0 = np.array([0.0], dtype=float)

        self.inputs = {}
        self.add_input_port(
            self.y_size,
            "y",
            nominal_value=np.zeros(self.y_size),
        )

        self.outputs = {}
        self.add_output_port(
            1,
            "meas",
            function=self.h_meas,
            dependencies=["y"],
        )

    def f(self, x, u, t=0.0, params=None):
        vx = float(u[self.speed_index])
        vx_filt = float(x[0])

        tau = max(self.tau, 1e-9)

        d_vx_filt = (vx - vx_filt) / tau

        return np.array([d_vx_filt], dtype=float)

    def h_meas(self, x, u, t=0.0, params=None):
        vx = float(u[self.speed_index])
        vx_filt = float(x[0])

        tau = max(self.tau, 1e-9)

        acc = (vx - vx_filt) / tau

        # print(f"acc: {acc}")
        return np.array([acc], dtype=float)

    def get_kinematic_geometry(self):
        return []

    def get_kinematic_transforms(self, _x, _u, _t):
        return []


class ThrMap(System):
    """Map desired rear longitudinal force to normalized throttle.

    Input
    -----
    F_rear
        Desired rear longitudinal tire force [N]

    Output
    ------
    thr
        Normalized throttle command in [0, 1]

    Mapping
    -------
    tau_rear_required = F_rear * r_r

    thr = tau_rear_required / tau_rear_max
    """

    def __init__(
        self,
        vehicle: DynamicBicycleRearWheelDriveEngine,
        name: str = "Throttle map",
    ):
        super().__init__(0, 2, 1)

        self.name = name

        self.r_r = float(vehicle.r_r)
        self.engine_power_peak = vehicle.engine_power_peak
        self.transmission_ratio = vehicle.transmission_ratio

        self.inputs = {}
        self.add_input_port(
            1,
            "F_rear",
            nominal_value=np.array([0.0]),
        )

        self.add_input_port(
            1,
            "w_rear",
            nominal_value=np.array([0.0]),
        )

        self.outputs = {}

        self.add_output_port(
            1,
            "thr",
            function=self.h_thr,
            dependencies=["F_rear", "w_rear"],
        )

    def h_thr(self, x, u, t=0.0, params=None):
        F_rear = float(u[0])
        w_rear = float(u[1])

        # Ici j'assume que la roue ne glisse pas
        tau_rear_required = F_rear * self.r_r

        w_rear_num = max(w_rear, 1.0)
        w_moteur = w_rear_num * self.transmission_ratio

        # print(f"w_rear: {w_rear}, w_rear_num: {w_rear_num}")
        max_engine_torque = self.engine_power_peak / w_moteur

        thr = tau_rear_required / max_engine_torque

        thr = np.clip(thr, 0.0, 1.0)

        # print(f"thr: {thr}")
        # print(
        #     f"tau_rear_required: {tau_rear_required}, max_engine_torque: {max_engine_torque}rpm, thr: {thr}"
        # )

        # print(f"w_rear: {w_rear}, w_rear_num: {w_rear_num}, w_moteur: {w_moteur}")
        # print(f"F_rear: {F_rear}")

        return np.array([thr], dtype=float)

    def get_kinematic_geometry(self):
        return []

    def get_kinematic_transforms(self, _x, _u, _t):
        return []


class AccToRearForce(System):
    """Map desired longitudinal acceleration to rear wheel longitudinal force.

    Input
    -----
    acc
        Desired longitudinal acceleration [m/s^2]

    Output
    ------
    F_rear
        Desired rear longitudinal tire force [N]

    Mapping
    -------
    F_rear = mass * acc
    """

    def __init__(
        self,
        vehicle: DynamicBicycleRearWheelDriveEngine,
        name: str = "Acceleration to rear force",
    ):
        # 0 states
        # 1 scalar input: acc
        # 1 scalar output: F_rear
        super().__init__(0, 1, 1)

        self.name = name

        self.mass = float(vehicle.mass)

        self.inputs = {}

        self.add_input_port(
            1,
            "acc_targ",
            nominal_value=np.array([0.0]),
        )

        self.outputs = {}

        self.add_output_port(
            1,
            "F_rear",
            function=self.h_force,
            dependencies=["acc_targ"],
        )

    def h_force(self, x, u, t=0.0, params=None):
        acc_targ = float(u[0])

        F_rear = self.mass * acc_targ
        return np.array([F_rear], dtype=float)

    def get_kinematic_geometry(self):
        return []

    def get_kinematic_transforms(self, _x, _u, _t):
        return []


def main():
    vehicle = DynamicBicycleRearWheelDriveEngine()

    # ESTIMATION DE VALEUR POUR DEFENDER DPS HD9
    vehicle.r_f = 0.3429  # m
    vehicle.r_r = 0.3429  # m

    vehicle.wheel_len_rear = vehicle.r_r * 2
    vehicle.wheel_width_rear = 0.2794
    vehicle.wheel_len_front = vehicle.r_f * 2
    vehicle.wheel_width_front = 0.2286

    # Wheel viscous damping
    # NÉGLIGER
    vehicle.bw_rear = 0.0
    vehicle.bw_front = 0.0

    # Wheel inertias
    vehicle.Jw_rear = 1.6  # kgm^2
    vehicle.Jw_front = 1.3  # kgm^2

    vehicle.a = 1.16
    vehicle.b = 0.95
    vehicle.L = vehicle.a + vehicle.b

    vehicle.mass = 698.8  # kg
    vehicle.inertia = 700.0  # kgm^2

    vehicle.gravity = 9.81
    vehicle.rho = 1.225

    # Forces aero
    # NÉGLIGER
    vehicle.CdA = 0.0

    vehicle.tire_model_f = Pacejka()
    vehicle.tire_model_r = Pacejka()

    # Moteur HD9
    vehicle.engine_power_peak = 48470.5  # Watts from 65 HP

    vehicle.transmission_ratio = 1.0
    # print(f"transmission_ratio: {vehicle.transmission_ratio}")

    vehicle.engine_dry_resistance = 8.0  # N/m
    vehicle.engine_rolling_resistance = 0.025  # N/m/rad/s

    vehicle.engine_tau = 0.25

    # Temps de reponse direction
    vehicle.steering_tau = 0.15

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

    acc_ref = ConstantReference(ref=ACC_REF, name="Acceleration reference")
    v_ref = ConstantReference(ref=VX_REF, name="Speed reference")

    speed_meas = Measurement(
        name="Speed measurement",
        y_size=12,
        index=5,  # vx
    )

    acc_meas = AccelerationMeasurement(
        y_size=12,
        speed_index=5,  # vx
        tau=0.05,
    )

    rear_speed_meas = Measurement(
        name="Rear wheel speed",
        y_size=12,
        index=8,  # w_rear in y
    )

    acc_pid = PID(
        Kp=1000.0,
        Ki=1000.0,
        Kd=0.0,
        tau=0.1,
        cmd_min=0.0,
        cmd_max=1000000.0,
        i_min=-1000000.0,
        i_max=1000000.0,
        name="Acceleration PID",
    )

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

    x_pos_pid = PID(
        Kp=1.0,
        Ki=0.0,
        Kd=0.0,
        tau=0.1,
        cmd_min=-10.0,
        cmd_max=10.0,
        i_min=-5.0,
        i_max=5.0,
        name="Speed PID",
    )

    steering = ConstantSteering(delta=DELTA_REF)
    thr_map = ThrMap(vehicle)
    acc_to_force = AccToRearForce(vehicle)

    diagram = DiagramSystem()
    diagram.name = "Acceleration PID - DynamicBicycleRearWheelDriveEngine"

    # diagram.add_subsystem(acc_ref, "acc_ref")
    # diagram.add_subsystem(acc_pid, "acc_pid")
    # diagram.add_subsystem(acc_meas, "acc_meas")
    diagram.add_subsystem(acc_to_force, "acc_to_force")
    diagram.add_subsystem(thr_map, "thr_map")

    diagram.add_subsystem(v_ref, "v_ref")
    diagram.add_subsystem(v_pid, "v_pid")
    diagram.add_subsystem(rear_speed_meas, "rear_speed_meas")
    diagram.add_subsystem(speed_meas, "speed_meas")

    diagram.add_subsystem(steering, "steering")
    diagram.add_subsystem(vehicle, "vehicle")
    # diagram.add_subsystem(speed_meas, "speed_meas")

    # Reference acceleration into PID
    diagram.connect("v_ref", "ref", "v_pid", "ref")
    diagram.connect("speed_meas", "meas", "v_pid", "meas")
    diagram.connect("v_pid", "cmd", "acc_to_force", "acc_targ")

    # Vehicle output vector into acceleration measurement block
    diagram.connect("vehicle", "y", "speed_meas", "y")

    # Scalar measured acceleration into PID
    diagram.connect("vehicle", "y", "rear_speed_meas", "y")

    diagram.connect("rear_speed_meas", "meas", "thr_map", "w_rear")

    # PID command drives throttle
    diagram.connect("acc_to_force", "F_rear", "thr_map", "F_rear")
    diagram.connect("thr_map", "thr", "vehicle", "thr")

    # Constant steering command
    diagram.connect("steering", "delta", "vehicle", "delta")

    # diagram.plot_graphe()

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

    # Plot acceleration tracking
    import matplotlib.pyplot as plt

    t = np.array(v_pid.t_hist)
    ref = np.array(v_pid.ref_hist)
    meas = np.array(v_pid.meas_hist)

    # Sort (important for solver calls)
    idx = np.argsort(t)
    t = t[idx]
    ref = ref[idx]
    meas = meas[idx]

    # Optional: remove duplicates
    t_unique, unique_idx = np.unique(t, return_index=True)
    ref = ref[unique_idx]
    meas = meas[unique_idx]
    t = t_unique

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
    # diagram.animate(renderer="meshcat")
    # diagram.animate(renderer="plotly")


if __name__ == "__main__":
    main()
