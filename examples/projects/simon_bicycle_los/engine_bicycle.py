"""Simon rear-wheel-drive engine bicycle, adapted to current minilink ports.

The dynamics are intentionally kept faithful to Simon's
``DynamicBicycleRearWheelDriveEngine``: 10 states, throttle command, delayed
engine torque, delayed/rate-limited steering, rear-wheel-drive tire physics.
"""

import numpy as np

from examples.projects.simon_bicycle_los.tire_models import TireModel
from minilink.core.kinematics import SE2
from minilink.core.system import DynamicSystem
from minilink.graphical.animation.primitives import CustomLine


def _wheel_rectangle_pts(wl: float, ww: float) -> np.ndarray:
    """Closed polyline in wheel frame (x forward, y lateral)."""
    h, w = wl / 2, ww / 2
    return np.array(
        [
            [h, w, 0.0],
            [h, -w, 0.0],
            [-h, -w, 0.0],
            [-h, w, 0.0],
            [h, w, 0.0],
        ]
    )


class DynamicBicycleMagicForces(DynamicSystem):
    """Planar dynamic bicycle base with rear longitudinal force input."""

    def __init__(self, n=6):
        super().__init__(n=n)
        self.name = "Dynamic Bicycle Magic Forces"

        self.state.labels = ["x", "y", "theta", "u", "v", "r"]
        self.state.units = ["m", "m", "rad", "m/s", "m/s", "rad/s"]

        self.add_input_port("f_rear", nominal_value=np.array([0.0]))
        self.add_input_port("delta", nominal_value=np.array([0.0]))
        self.inputs["f_rear"].labels = ["f_rear"]
        self.inputs["f_rear"].units = ["N"]
        self.inputs["delta"].labels = ["delta"]
        self.inputs["delta"].units = ["rad"]

        self.add_output_port("y", dim=n, function=self.h, dependencies=())

        self.max_steer = 1.571
        self.min_steer = -1.571

        self.a = 1.0
        self.b = 1.0
        self.L = self.a + self.b
        self.r_f = 0.3
        self.r_r = 0.3

        self.wheel_len_rear = self.r_r * 2
        self.wheel_width_rear = 0.2
        self.wheel_len_front = self.r_f * 2
        self.wheel_width_front = 0.2

        self.mass = 1500.0
        self.inertia = 2500.0
        self.gravity = 9.81
        self.rho = 1.225
        self.CdA = 0.3 * 2.2

        self.tire_model_f = TireModel()
        self.tire_model_r = TireModel()
        self.camera_follow_frame = "body"
        self.camera_scale = 15.0

    def x2q(self, x):
        q = x[0:3]
        v = x[3:6]
        return q, v

    def q2x(self, dq, dv):
        return np.concatenate([dq, dv])

    def M_mat(self, q):
        return np.diag(np.array([self.mass, self.mass, self.inertia], dtype=float))

    def C_mat(self, q, v):
        yawrate = v[2]
        C = np.zeros((3, 3), dtype=float)
        C[1, 0] = self.mass * yawrate
        C[0, 1] = -self.mass * yawrate
        return C

    def N_mat(self, q):
        theta = q[2]
        c, s = np.cos(theta), np.sin(theta)
        return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=float)

    def compute_wheel_velocities(self, v_body, u_inputs):
        vx = v_body[0]
        vy = v_body[1]
        yawrate = v_body[2]
        delta = u_inputs[1]

        vx_f_b = vx
        vy_f_b = vy + self.a * yawrate
        vx_r_b = vx
        vy_r_b = vy - self.b * yawrate

        c_d, s_d = np.cos(delta), np.sin(delta)
        vx_f = c_d * vx_f_b + s_d * vy_f_b
        vy_f = -s_d * vx_f_b + c_d * vy_f_b
        vx_r = vx_r_b
        vy_r = vy_r_b

        w_r = vx_r / self.r_r
        w_f = vx_f / self.r_f

        return vx_f, vy_f, w_f, vx_r, vy_r, w_r

    def compute_tire_physics(self, v_body, u_inputs):
        vx_f, vy_f, w_f, vx_r, vy_r, w_r = self.compute_wheel_velocities(
            v_body, u_inputs
        )
        Fz_f = self.mass * self.gravity * (self.b / self.L)
        Fz_r = self.mass * self.gravity * (self.a / self.L)
        Fx_f, Fy_f = self.tire_model_f.vel2forces(vx_f, vy_f, w_f, self.r_f, Fz_f)
        Fx_r, Fy_r = self.tire_model_r.vel2forces(vx_r, vy_r, w_r, self.r_r, Fz_r)

        Fx_r = u_inputs[0]

        return Fx_f, Fy_f, Fx_r, Fy_r

    def tire_forces_body_frame_from_forces(self, Fx_f, Fy_f, Fx_r, Fy_r, u_in):
        delta = float(u_in[1])
        c_d = np.cos(delta)
        s_d = np.sin(delta)

        Fx_f_b = Fx_f * c_d - Fy_f * s_d
        Fy_f_b = Fx_f * s_d + Fy_f * c_d

        Fx_r_b = Fx_r
        Fy_r_b = Fy_r

        return Fx_f_b, Fy_f_b, Fx_r_b, Fy_r_b

    def generalized_d(self, q, v, u_in):
        Fx_front, Fy_front, Fx_rear, Fy_rear = self.compute_tire_physics(v, u_in)

        Fx_f_b, Fy_f_b, Fx_r_b, Fy_r_b = self.tire_forces_body_frame_from_forces(
            Fx_front,
            Fy_front,
            Fx_rear,
            Fy_rear,
            u_in,
        )

        Sum_Fx = Fx_f_b + Fx_r_b
        Sum_Fy = Fy_f_b + Fy_r_b
        Sum_Mz = self.a * Fy_f_b - self.b * Fy_r_b
        F_aero = 0.5 * self.rho * self.CdA * v[0] * abs(v[0])
        Sum_Fx -= F_aero
        F_ext = np.array([Sum_Fx, Sum_Fy, Sum_Mz], dtype=float)
        return -F_ext

    def f(self, x, u, t=0.0, params=None):
        q, v = self.x2q(x)
        f_rear, delta = self.get_port_values_from_u(u, "f_rear", "delta")
        u_in = np.array([f_rear[0], delta[0]])
        u_in[1] = np.clip(u_in[1], self.min_steer, self.max_steer)

        M = self.M_mat(q)
        C = self.C_mat(q, v)
        d_vec = self.generalized_d(q, v, u_in)
        dv = np.linalg.solve(M, -C @ v - d_vec)
        dq = self.N_mat(q) @ v

        dx = self.q2x(dq, dv)
        return dx

    def h(self, x, u, t=0.0, params=None):
        return x.copy()

    def tire_forces_body_frame(self, v_body, u_in):
        Fx_front, Fy_front, Fx_rear, Fy_rear = self.compute_tire_physics(v_body, u_in)
        Fx_f_b, Fy_f_b, Fx_r_b, Fy_r_b = self.tire_forces_body_frame_from_forces(
            Fx_front,
            Fy_front,
            Fx_rear,
            Fy_rear,
            u_in,
        )
        return Fx_f_b, Fy_f_b, Fx_r_b, Fy_r_b

    def get_u_int(self, x, u):
        return u

    def get_kinematic_geometry(self):
        return {
            "body": [
                CustomLine(
                    np.array([[-self.b, 0.0, 0.0], [self.a, 0.0, 0.0]]),
                    color="black",
                    linewidth=2,
                )
            ],
            "rear_wheel": [
                CustomLine(
                    _wheel_rectangle_pts(self.wheel_len_rear, self.wheel_width_rear),
                    color="black",
                    linewidth=1,
                )
            ],
            "front_wheel": [
                CustomLine(
                    _wheel_rectangle_pts(self.wheel_len_front, self.wheel_width_front),
                    color="black",
                    linewidth=1,
                )
            ],
        }

    def tf(self, x, u, t=0.0, params=None):
        X, Y, theta = float(x[0]), float(x[1]), float(x[2])
        u_in = self.get_u_int(x, u)
        delta = float(u_in[1])

        W_T_B = SE2(X, Y, theta)
        B_T_R = SE2(-self.b, 0.0, 0.0)
        B_T_F = SE2(self.a, 0.0, delta)
        return {
            "body": W_T_B,
            "rear_wheel": W_T_B @ B_T_R,
            "front_wheel": W_T_B @ B_T_F,
        }


class DynamicBicycleRearWheelDrive(DynamicBicycleMagicForces):
    """Dynamic bicycle with rear and front wheel spin states."""

    def __init__(self, n=8):
        super().__init__(n)
        self.name = "Dynamic Bicycle Rear Wheel Drive"

        self.state.labels = [
            "X",
            "Y",
            "theta",
            "vx",
            "vy",
            "yawrate",
            "w_rear",
            "w_front",
        ]
        self.state.units = [
            "m",
            "m",
            "rad",
            "m/s",
            "m/s",
            "rad/s",
            "rad/s",
            "rad/s",
        ]

        self.inputs = {}
        self.add_input_port("t_rear", nominal_value=np.array([0.0]))
        self.add_input_port("delta", nominal_value=np.array([0.0]))
        self.inputs["t_rear"].labels = ["t_rear"]
        self.inputs["t_rear"].units = ["Nm"]
        self.inputs["delta"].labels = ["delta"]
        self.inputs["delta"].units = ["rad"]

        self.outputs = {}
        self.add_output_port("y", dim=n, function=self.h, dependencies=())

        self.bw_rear = 0.0
        self.bw_front = 0.0
        self.rolling_friction_constant = 0.0
        self.Jw_rear = 1.0
        self.Jw_front = 1.0

    def x2q(self, x):
        q = x[0:3]
        v = x[3:8]
        return q, v

    def q2x(self, dq, dv):
        return np.concatenate([dq, dv])

    def M_mat(self, q):
        return np.diag(
            [self.mass, self.mass, self.inertia, self.Jw_rear, self.Jw_front]
        )

    def C_mat(self, q, v):
        yawrate = v[2]
        C = np.zeros((5, 5))
        C[0, 1] = -self.mass * yawrate
        C[1, 0] = self.mass * yawrate
        return C

    def N_mat(self, q):
        theta = q[2]
        c = np.cos(theta)
        s = np.sin(theta)

        N = np.array(
            [
                [c, -s, 0.0, 0.0, 0.0],
                [s, c, 0.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0, 0.0],
            ]
        )
        return N

    def g(self, q):
        return np.zeros(5)

    def compute_wheel_velocities(self, v_body, u_inputs):
        vx = v_body[0]
        vy = v_body[1]
        yawrate = v_body[2]

        w_rear = v_body[3]
        w_front = v_body[4]

        delta = u_inputs[1]

        vx_f_b = vx
        vy_f_b = vy + self.a * yawrate
        vx_r_b = vx
        vy_r_b = vy - self.b * yawrate

        c_d = np.cos(delta)
        s_d = np.sin(delta)
        vx_f = c_d * vx_f_b + s_d * vy_f_b
        vy_f = -s_d * vx_f_b + c_d * vy_f_b
        vx_r = vx_r_b
        vy_r = vy_r_b

        return vx_f, vy_f, w_front, vx_r, vy_r, w_rear

    def compute_tire_physics(self, v_body, u_inputs):
        vx_f, vy_f, w_f, vx_r, vy_r, w_r = self.compute_wheel_velocities(
            v_body, u_inputs
        )

        Fz_f = self.mass * self.gravity * (self.b / self.L)
        Fz_r = self.mass * self.gravity * (self.a / self.L)

        Fx_front, Fy_front = self.tire_model_f.vel2forces(
            vx_f, vy_f, w_f, self.r_f, Fz_f
        )
        Fx_front = 0.0
        Fx_rear, Fy_rear = self.tire_model_r.vel2forces(vx_r, vy_r, w_r, self.r_r, Fz_r)

        return Fx_front, Fy_front, Fx_rear, Fy_rear

    def generalized_d(self, q, v, u_in):
        w_rear = v[3]
        w_front = v[4]

        Fx_front, Fy_front, Fx_rear, Fy_rear = self.compute_tire_physics(v, u_in)

        Fx_f_b, Fy_f_b, Fx_r_b, Fy_r_b = self.tire_forces_body_frame_from_forces(
            Fx_front,
            Fy_front,
            Fx_rear,
            Fy_rear,
            u_in,
        )

        Sum_Fx = Fx_f_b + Fx_r_b
        Sum_Fy = Fy_f_b + Fy_r_b
        Sum_Mz = self.a * Fy_f_b - self.b * Fy_r_b

        F_aero = 0.5 * self.rho * self.CdA * v[0] * abs(v[0])
        Sum_Fx -= F_aero

        Fz_f = self.mass * self.gravity * (self.b / self.L)
        Fz_r = self.mass * self.gravity * (self.a / self.L)

        Tau_load_rear = (
            self.r_r * Fx_rear
            + self.bw_rear * w_rear
            + self.rolling_friction_constant * Fz_r * np.sign(w_rear)
        )
        Tau_load_front = (
            self.r_f * Fx_front
            + self.bw_front * w_front
            + self.rolling_friction_constant * Fz_f * np.sign(w_rear)
        )

        Q_ext = np.array([Sum_Fx, Sum_Fy, Sum_Mz, -Tau_load_rear, -Tau_load_front])
        return -Q_ext

    def u2e(self, u):
        tau_rear, delta = u
        e = np.array([0.0, 0.0, 0.0, tau_rear, 0.0])
        return e

    def B(self, q, u):
        return np.eye(5)

    def accelerations(self, q, v, u, t=0.0):
        M = self.M_mat(q)
        C = self.C_mat(q, v)
        g = self.g(q)
        d = self.generalized_d(q, v, u)
        B = self.B(q, u)
        e = self.u2e(u)

        dv = np.linalg.inv(M) @ (B @ e - C @ v - g - d)
        return dv

    def f(self, x, u, t=0.0, params=None):
        u[1] = np.clip(u[1], self.min_steer, self.max_steer)

        q, v = self.x2q(x)

        dv = self.accelerations(q, v, u, t)
        dq = self.N_mat(q) @ v

        dx = self.q2x(dq, dv)
        return dx

    def get_u_int(self, x, u):
        return u


class DynamicBicycleRearWheelDriveEngine(DynamicBicycleRearWheelDrive):
    """Rear-wheel-drive bicycle with throttle, engine lag, and steering lag.

    State ``x = [X, Y, theta, vx, vy, yawrate, w_rear, w_front,
    torque_engine, delta_act]``.
    """

    def __init__(self, n=10):
        super().__init__(n=n)
        self.name = "Dynamic Bicycle With Engine"

        self.state.labels = [
            "X",
            "Y",
            "theta",
            "vx",
            "vy",
            "yawrate",
            "w_rear",
            "w_front",
            "torque_engine",
            "delta_act",
        ]
        self.state.units = [
            "m",
            "m",
            "rad",
            "m/s",
            "m/s",
            "rad/s",
            "rad/s",
            "rad/s",
            "Nm",
            "rad",
        ]

        self.inputs = {}
        self.add_input_port("thr", nominal_value=np.array([0.0]))
        self.add_input_port("delta", nominal_value=np.array([0.0]))
        self.inputs["thr"].labels = ["thr"]
        self.inputs["thr"].units = ["normalized"]
        self.inputs["delta"].labels = ["delta"]
        self.inputs["delta"].units = ["rad"]

        self.outputs = {}
        self.add_output_port("y", dim=n, function=self.h, dependencies=())
        self.add_output_port(
            "rear_tire_data",
            dim=4,
            function=self.rear_tire_forces_and_slip,
            dependencies="all",
        )

        self.engine_power_peak = 40000.0
        self.transmission_ratio = 1.0
        self.engine_dry_resistance = 8.0
        self.engine_rolling_resistance = 0.025
        self.engine_viscous_resistance = 0.008
        self.engine_tau = 0.25
        self.steering_tau = 0.15
        self.steering_rate_max = 10.0

    def rear_tire_forces_and_slip(self, x, u, t=0.0, params=None):
        _, v = self.x2q(x)

        torque_engine, delta_act = self.get_u_int(x, u)
        u_drive = np.array([torque_engine, delta_act], dtype=float)

        vx_f, vy_f, w_f, vx_r, vy_r, w_r = self.compute_wheel_velocities(v, u_drive)

        Fz_r = self.mass * self.gravity * (self.a / self.L)
        alpha, kappa = self.tire_model_r.vel2slip(vx_r, vy_r, w_r, self.r_r)
        Fx, Fy = self.tire_model_r.slip2forces(alpha, kappa, Fz_r)
        return np.array([Fx, kappa, Fy, alpha], dtype=float)

    def x2q(self, x):
        q = x[0:3]
        v = x[3:8]
        return q, v

    def q2x(self, dq, dv, d_tau_engine=0.0, d_delta_act=0.0):
        return np.concatenate([dq, dv, [d_tau_engine, d_delta_act]])

    def engine_torque_from_throttle(self, v, throttle):
        w_rear = v[3]
        throttle = np.clip(throttle, 0.0, 1.0)

        w_rear_num = max(w_rear, 1e-6)
        w_moteur = w_rear_num * self.transmission_ratio

        available_torque = self.engine_power_peak / w_moteur
        available_torque = np.clip(available_torque, 0.0, self.engine_power_peak)

        tau_rear = throttle * available_torque
        if w_rear_num > 1e-6:
            tau_rear = (
                tau_rear
                - self.engine_dry_resistance * np.sign(w_moteur)
                - self.engine_rolling_resistance * w_moteur
                - (self.engine_viscous_resistance * w_moteur) ** 2
            )

        return tau_rear

    def f(self, x, u, t=0.0, params=None):
        q, v = self.x2q(x)

        throttle, delta_cmd = self.get_port_values_from_u(u, "thr", "delta")
        throttle = throttle[0]
        delta_cmd = delta_cmd[0]

        tau_cmd = self.engine_torque_from_throttle(v, throttle)
        torque_engine, delta_act = self.get_u_int(x, u)

        engine_tau = max(self.engine_tau, 1e-6)
        d_tau_engine = (tau_cmd - torque_engine) / engine_tau

        delta_cmd = float(np.clip(delta_cmd, self.min_steer, self.max_steer))
        delta_act_used = float(np.clip(delta_act, self.min_steer, self.max_steer))

        steering_tau = max(self.steering_tau, 1e-6)
        d_delta_act = (delta_cmd - delta_act) / steering_tau
        d_delta_act = float(
            np.clip(
                d_delta_act,
                -self.steering_rate_max,
                self.steering_rate_max,
            )
        )

        if delta_act >= self.max_steer and d_delta_act > 0.0:
            d_delta_act = 0.0

        if delta_act <= self.min_steer and d_delta_act < 0.0:
            d_delta_act = 0.0

        u_drive = np.array([torque_engine, delta_act_used], dtype=float)

        dv = self.accelerations(q, v, u_drive, t)
        dq = self.N_mat(q) @ v

        return np.concatenate([dq, dv, np.array([d_tau_engine, d_delta_act])])

    def get_u_int(self, x, u):
        torque_engine = float(x[8])
        delta_act = float(x[9])
        return np.array([torque_engine, delta_act], dtype=float)
