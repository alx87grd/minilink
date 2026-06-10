"""
Dynamic bicycle (planar rigid body + tire forces), ported minimally from pyro
``vehicle_dynamic.DynamicBicycle``.

State ``x = [x, y, theta, u, v, r]``: world pose and body velocities (surge, sway, yaw rate).
Inputs ``u = [w_rear, delta]``: rear wheel spin rate [rad/s], steer angle [rad].

:class:`DynamicBicycleCar3D` subclasses this model with identical dynamics and richer 3D graphics.
:class:`JaxDynamicBicycle` is a JAX-traceable variant of the same dynamics, suitable for
gradient-based trajectory optimization.
"""

import numpy as np

from minilink.core.system import DynamicSystem
from minilink.dynamics.catalog.vehicles.tire_models import (
    TireModel,
)
from minilink.graphical.animation.primitives import (
    Arrow,
    CustomLine,
    camera_matrix,
    pose2d_matrix,
    scale_pose2d_matrix,
)


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
    """
    Dynamic bicycle with magic force at rear wheel and steer inputs.

    Inputs
    ------
    f_rear : rear wheel applied force [N]
    delta  : front steer angle [rad]
    """

    def __init__(self, n=6):
        super().__init__(n=n)

        self.name = "Dynamic Bicycle majic forces"

        self.state.labels = ["x", "y", "theta", "u", "v", "r"]
        self.state.units = ["m", "m", "rad", "m/s", "m/s", "rad/s"]

        self.inputs = {}
        self.add_input_port("f_rear", nominal_value=np.array([0.0]))
        self.add_input_port("delta", nominal_value=np.array([0.0]))
        self.inputs["f_rear"].labels = ["f_rear"]
        self.inputs["f_rear"].units = ["N"]
        self.inputs["delta"].labels = ["delta"]
        self.inputs["delta"].units = ["rad"]

        self.max_steer = 1.571  # rad
        self.min_steer = -1.571  # rad

        self.outputs = {}
        self.add_output_port("y", dim=n, function=self.h, dependencies=[])

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

        self.camera_follow_vehicle = True

    def x2q(self, x):
        """
        Convert state vector to generalized coordinates and velocities
        """
        q = x[0:3]
        v = x[3:6]
        return q, v

    def q2x(self, dq, dv):
        """
        Convert qdot and vdot back to state derivative
        """
        return np.concatenate([dq, dv])

    def M_mat(self, q: np.ndarray) -> np.ndarray:
        return np.diag(np.array([self.mass, self.mass, self.inertia], dtype=float))

    def C_mat(self, q: np.ndarray, v: np.ndarray) -> np.ndarray:
        w = v[2]
        C = np.zeros((3, 3), dtype=float)
        C[1, 0] = self.mass * w
        C[0, 1] = -self.mass * w
        return C

    def N_mat(self, q: np.ndarray) -> np.ndarray:
        theta = q[2]
        c, s = np.cos(theta), np.sin(theta)
        return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=float)

    def compute_wheel_velocities(
        self, v_body: np.ndarray, u_inputs: np.ndarray
    ) -> tuple[float, float, float, float, float, float]:
        u = v_body[0]
        v = v_body[1]
        r = v_body[2]
        delta = u_inputs[1]

        vx_f_b = u
        vy_f_b = v + self.a * r
        vx_r_b = u
        vy_r_b = v - self.b * r

        c_d, s_d = np.cos(delta), np.sin(delta)
        vx_f = c_d * vx_f_b + s_d * vy_f_b
        vy_f = -s_d * vx_f_b + c_d * vy_f_b
        vx_r = vx_r_b
        vy_r = vy_r_b

        w_r = vx_r / self.r_r
        w_f = vx_f / self.r_f

        return vx_f, vy_f, w_f, vx_r, vy_r, w_r

    def compute_tire_physics(
        self, v_body: np.ndarray, u_inputs: np.ndarray
    ) -> tuple[float, float, float, float]:
        vx_f, vy_f, w_f, vx_r, vy_r, w_r = self.compute_wheel_velocities(
            v_body, u_inputs
        )
        Fz_f = self.mass * self.gravity * (self.b / self.L)
        Fz_r = self.mass * self.gravity * (self.a / self.L)
        Fx_f, Fy_f = self.tire_model_f.vel2forces(vx_f, vy_f, w_f, self.r_f, Fz_f)
        Fx_r, Fy_r = self.tire_model_r.vel2forces(vx_r, vy_r, w_r, self.r_r, Fz_r)

        Fx_r = u_inputs[0]

        return Fx_f, Fy_f, Fx_r, Fy_r

    def tire_forces_body_frame_from_forces(
        self,
        Fx_f: float,
        Fy_f: float,
        Fx_r: float,
        Fy_r: float,
        u_in: np.ndarray,
    ):
        delta = float(u_in[1])
        c_d = np.cos(delta)
        s_d = np.sin(delta)

        Fx_f_b = Fx_f * c_d - Fy_f * s_d
        Fy_f_b = Fx_f * s_d + Fy_f * c_d

        Fx_r_b = Fx_r
        Fy_r_b = Fy_r

        return Fx_f_b, Fy_f_b, Fx_r_b, Fy_r_b

    def generalized_d(
        self, q: np.ndarray, v: np.ndarray, u_in: np.ndarray
    ) -> np.ndarray:
        # Fx_f_b, Fy_f_b, Fx_r_b, Fy_r_b = self.tire_forces_body_frame(v, u_in)

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

    def f(
        self, x: np.ndarray, u: np.ndarray, t: float = 0.0, params=None
    ) -> np.ndarray:
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

    def h(
        self, x: np.ndarray, u: np.ndarray, t: float = 0.0, params=None
    ) -> np.ndarray:
        return x.copy()

    def get_camera_transform(self, x, u, t):
        """Return a camera centered on the vehicle pose by default.

        ``camera_target`` is treated as an offset from the vehicle position
        when ``camera_follow_vehicle`` is true. Set ``camera_follow_vehicle`` to
        false to use the base fixed-camera interpretation of ``camera_target``.
        """
        target = np.asarray(self.camera_target, dtype=float).reshape(3).copy()
        if self.camera_follow_vehicle:
            target[0] += float(x[0])
            target[1] += float(x[1])

        return camera_matrix(
            target=target,
            plot_axes=self.camera_plot_axes,
            scale=self.camera_scale,
        )

    def get_kinematic_geometry(self):
        wl_r, ww_r = self.wheel_len_rear, self.wheel_width_rear
        wl_f, ww_f = self.wheel_len_front, self.wheel_width_front

        chassis = CustomLine(
            np.array([[-self.b, 0.0, 0.0], [self.a, 0.0, 0.0]]),
            color="black",
            linewidth=2,
        )
        wpts_rear = _wheel_rectangle_pts(wl_r, ww_r)
        rear_w = CustomLine(wpts_rear, color="black", linewidth=1)
        wpts_front = _wheel_rectangle_pts(wl_f, ww_f)
        front_w = CustomLine(wpts_front, color="black", linewidth=1)
        arr_v = Arrow(color="blue", linewidth=2, origin="base")
        arr_f = Arrow(color="red", linewidth=2, origin="base")
        return [chassis, rear_w, front_w, arr_v, arr_v, arr_f, arr_f]

    def tire_forces_body_frame(self, v_body: np.ndarray, u_in: np.ndarray):
        Fx_front, Fy_front, Fx_rear, Fy_rear = self.compute_tire_physics(v_body, u_in)

        Fx_f_b, Fy_f_b, Fx_r_b, Fy_r_b = self.tire_forces_body_frame_from_forces(
            Fx_front,
            Fy_front,
            Fx_rear,
            Fy_rear,
            u_in,
        )

        return Fx_f_b, Fy_f_b, Fx_r_b, Fy_r_b

    def _world_arrow_pose(
        self, dx: float, dy: float, px: float, py: float, v_scale: float = 0.2
    ):
        mag = v_scale * np.hypot(dx, dy)
        if mag < 1e-9:
            mag = 1e-9
        th = np.arctan2(dy, dx)
        return scale_pose2d_matrix(px, py, th, mag)

    def _force_pose(
        self, Fx: float, Fy: float, px: float, py: float, f_scale: float = 0.001
    ):
        mag = f_scale * np.hypot(Fx, Fy)
        if mag < 1e-12:
            mag = 1e-12
        th = np.arctan2(Fy, Fx)
        return scale_pose2d_matrix(px, py, th, mag)

    def get_u_int(self, x: np.ndarray, u: np.ndarray) -> np.ndarray:
        return u

    def get_kinematic_transforms(self, x: np.ndarray, u: np.ndarray, t: float):
        X, Y, Theta = float(x[0]), float(x[1]), float(x[2])
        _, vb = self.x2q(x)
        u_in = self.get_u_int(x, u)
        delta = float(u_in[1])

        T_wb = pose2d_matrix(X, Y, Theta)
        T_rear = T_wb @ pose2d_matrix(-self.b, 0.0, 0.0)
        T_front = T_wb @ pose2d_matrix(self.a, 0.0, delta)

        uu, vv, wr = float(vb[0]), float(vb[1]), float(vb[2])
        v_f_loc = np.array([uu, vv + self.a * wr])
        v_r_loc = np.array([uu, vv - self.b * wr])

        c, s = np.cos(Theta), np.sin(Theta)
        rx = X + c * (-self.b) - s * 0.0
        ry = Y + s * (-self.b) + c * 0.0
        fx = X + c * self.a - s * 0.0
        fy = Y + s * self.a + c * 0.0

        vfx, vfy = c * v_f_loc[0] - s * v_f_loc[1], s * v_f_loc[0] + c * v_f_loc[1]
        vrx, vry = c * v_r_loc[0] - s * v_r_loc[1], s * v_r_loc[0] + c * v_r_loc[1]

        Fx_f_b, Fy_f_b, Fx_r_b, Fy_r_b = self.tire_forces_body_frame(vb, u_in)

        Ffx_w = c * Fx_f_b - s * Fy_f_b
        Ffy_w = s * Fx_f_b + c * Fy_f_b

        Frx_w = c * Fx_r_b - s * Fy_r_b
        Fry_w = s * Fx_r_b + c * Fy_r_b

        return [
            T_wb,
            T_rear,
            T_front,
            self._world_arrow_pose(vrx, vry, rx, ry),
            self._world_arrow_pose(vfx, vfy, fx, fy),
            self._force_pose(Frx_w, Fry_w, rx, ry),
            self._force_pose(Ffx_w, Ffy_w, fx, fy),
        ]


class DynamicBicycleRearWheelDrive(DynamicBicycleMagicForces):
    """
    Dynamic bicycle with rear wheel dynamics and steer inputs.

    Inputs
    ------
    f_rear : rear wheel applied force [N]
    delta  : front steer angle [rad]
    """

    def __init__(self, n=8):
        super().__init__(n)

        self.name = "Dynamic Bicycle"

        self.state.labels = [
            "X",
            "Y",
            "theta",
            "vx",
            "vy",
            "r",
            "w_l",
            "w_r",
        ]

        self.state.units = [
            "[m]",
            "[m]",
            "[rad]",
            "[m/s]",
            "[m/s]",
            "[rad/s]",
            "[rad/s]",
            "[rad/s]",
        ]

        self.inputs = {}
        self.add_input_port("t_rear", nominal_value=np.array([0.0]))
        self.add_input_port("delta", nominal_value=np.array([0.0]))
        self.inputs["t_rear"].labels = ["t_rear"]
        self.inputs["t_rear"].units = ["Nm"]
        self.inputs["delta"].labels = ["delta"]
        self.inputs["delta"].units = ["rad"]

        # Wheel viscous damping
        self.bw_rear = 0.0
        self.bw_front = 0.0

        # Wheel inertias
        self.Jw_rear = 1.0
        self.Jw_front = 1.0

    def x2q(self, x):
        """
        Convert state vector to generalized coordinates and velocities
        """
        q = x[0:3]
        v = x[3:8]
        return q, v

    def M_mat(self, q: np.ndarray) -> np.ndarray:
        """
        Generalized mass/inertia matrix in quasi-velocity coordinates
        v = [vx, vy, r, w_l, w_r]
        """
        M = np.diag([self.mass, self.mass, self.inertia, self.Jw_rear, self.Jw_front])
        return M

    def C_mat(self, q: np.ndarray, v: np.ndarray) -> np.ndarray:
        """
        Coriolis / convective matrix such that:

            M dv + C(q,v) v + g + d = B e
        """
        r = v[2]

        C = np.zeros((5, 5))

        # body-frame rigid-body convective coupling
        C[0, 1] = -self.mass * r  # gives -m*r*vy
        C[1, 0] = self.mass * r  # gives +m*r*vx

        return C

    def N_mat(self, q: np.ndarray) -> np.ndarray:
        """
        dq = N(q) @ v

        q = [X, Y, theta, phi_l, phi_r]
        v = [vx, vy, r, w_l, w_r]
        """
        theta = q[2]
        c = np.cos(theta)
        s = np.sin(theta)

        N = np.array(
            [
                [c, -s, 0.0, 0.0, 0.0],  # X_dot
                [s, c, 0.0, 0.0, 0.0],  # Y_dot
                [0.0, 0.0, 1.0, 0.0, 0.0],  # theta_dot
            ]
        )

        return N

    def g(self, q):
        """
        No generalized conservative force in planar motion
        """
        return np.zeros(5)

    def compute_wheel_velocities(
        self, v_body: np.ndarray, u_inputs: np.ndarray
    ) -> tuple[float, float, float, float, float, float]:
        """
        q = [X, Y, theta]
        v = [vx, vy, r, w_rear, w_front]
        """

        vx = v_body[0]
        vy = v_body[1]
        r = v_body[2]

        w_rear = v_body[3]
        w_front = v_body[4]

        delta = u_inputs[1]

        # Front wheel center velocity in body frame
        vx_f_b = vx
        vy_f_b = vy + self.a * r

        # Rear wheel center velocity in body frame
        vx_r_b = vx
        vy_r_b = vy - self.b * r

        # Rotate front wheel velocity into front wheel frame
        c_d = np.cos(delta)
        s_d = np.sin(delta)

        vx_f = c_d * vx_f_b + s_d * vy_f_b
        vy_f = -s_d * vx_f_b + c_d * vy_f_b

        # Rear wheel is not steered
        vx_r = vx_r_b
        vy_r = vy_r_b

        return vx_f, vy_f, w_front, vx_r, vy_r, w_rear

    def compute_tire_physics(
        self, v_body: np.ndarray, u_inputs: np.ndarray
    ) -> tuple[float, float, float, float]:
        """
        Compute tire forces from generalized state
        """
        vx_f, vy_f, w_f, vx_r, vy_r, w_r = self.compute_wheel_velocities(
            v_body, u_inputs
        )

        Fz_f = self.mass * self.gravity * (self.b / self.L)
        Fz_r = self.mass * self.gravity * (self.a / self.L)

        Fx_front, Fy_front = self.tire_model_f.vel2forces(
            vx_f, vy_f, w_f, self.r_f, Fz_f
        )
        # Roue avant ne produit aucune force en x
        Fx_front = 0.0

        Fx_rear, Fy_rear = self.tire_model_r.vel2forces(vx_r, vy_r, w_r, self.r_r, Fz_r)

        return Fx_front, Fy_front, Fx_rear, Fy_rear

    def generalized_d(
        self, q: np.ndarray, v: np.ndarray, u_in: np.ndarray
    ) -> np.ndarray:
        """
        Generalized non-conservative forces / loads

        Returned with sign convention consistent with:
            dv = inv(M) @ (B e - C v - g - d)
        so we return d = -Q_ext
        """
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

        # wheel resisting torques from tire longitudinal forces + viscous damping
        Tau_load_rear = self.r_r * Fx_rear + self.bw_rear * w_rear
        Tau_load_front = self.r_f * Fx_front + self.bw_front * w_front

        Q_ext = np.array([Sum_Fx, Sum_Fy, Sum_Mz, -Tau_load_rear, -Tau_load_front])

        return -Q_ext

    def u2e(self, u):
        """
        Map control input u = [tau_rear, delta] to generalized effort vector
        """
        tau_rear, delta = u
        e = np.array([0.0, 0.0, 0.0, tau_rear, 0.0])

        return e

    def B(self, q, u):
        """
        Generalized effort mapping matrix
        """
        return np.eye(5)

    def accelerations(self, q, v, u, t=0.0):
        """
        Compute generalized accelerations dv
        """
        M = self.M_mat(q)
        C = self.C_mat(q, v)
        g = self.g(q)
        d = self.generalized_d(q, v, u)
        B = self.B(q, u)
        e = self.u2e(u)

        dv = np.linalg.inv(M) @ (B @ e - C @ v - g - d)
        return dv

    def f(
        self, x: np.ndarray, u: np.ndarray, t: float = 0.0, params=None
    ) -> np.ndarray:
        """
        Continuous-time forward dynamics

        x = [q, v]
        """
        u[1] = np.clip(u[1], self.min_steer, self.max_steer)

        q, v = self.x2q(x)

        dv = self.accelerations(q, v, u, t)
        dq = self.N_mat(q) @ v

        dx = self.q2x(dq, dv)
        # print(u)
        return dx

    def get_u_int(self, x: np.ndarray, u: np.ndarray) -> np.ndarray:
        return u


class DynamicBicycleRearWheelDriveEngine(DynamicBicycleRearWheelDrive):
    """
    Dynamic bicycle with rear wheel dynamics and steer inputs.

    Inputs
    ------
    thr : engine throttle [Normalized]
    delta : front steer angle [rad]


    State
    -----
    x = [
        X, Y, theta,
        vx, vy, r,
        w_rear, w_front,
        torque_engine,
        delta_act,
    ]

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
            "r",
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

        self.add_output_port(
            "r_tire_datas",
            dim=4,
            function=self.rear_tire_forces_and_slip,
            dependencies="all",
        )

        self.engine_power_peak = 40000.0  # Watts
        self.transmission_ratio = 1.0

        self.engine_dry_resistance = 8.0  # N/m
        self.engine_rolling_resistance = 0.025  # N/m/rad/s

        self.engine_tau = 0.25
        self.steering_tau = 0.15

        # TODO: A tuner. C'est pour que la commande n'explosa pas
        self.steering_rate_max = 10.0  # rad/s

    def rear_tire_forces_and_slip(
        self, x: np.ndarray, u: np.ndarray, t: float = 0.0, params=None
    ) -> np.ndarray:
        _, v = self.x2q(x)

        torque_engine, delta_act = self.get_u_int(x, u)

        u_drive = np.array([torque_engine, delta_act], dtype=float)

        vx_f, vy_f, w_f, vx_r, vy_r, w_r = self.compute_wheel_velocities(v, u_drive)

        Fz_r = self.mass * self.gravity * (self.a / self.L)

        alpha, kappa = self.tire_model_r.vel2slip(vx_r, vy_r, w_r, self.r_r)
        Fx, Fy = self.tire_model_r.slip2forces(alpha, kappa, Fz_r, logs=False)
        return np.array([Fx, kappa, Fy, alpha], dtype=float)

    def x2q(self, x):
        """
        Convert state vector to generalized coordinates and velocities
        """
        q = x[0:3]
        v = x[3:8]
        return q, v

    def q2x(self, dq, dv, d_tau_engine=0.0, d_delta_act=0.0):
        return np.concatenate([dq, dv, [d_tau_engine, d_delta_act]])

    def engine_torque_from_throttle(self, v: np.ndarray, throttle: float):
        """
        Compute rear wheel drive torque from throttle and rear wheel speed.

        v = [vx, vy, r, w_rear, w_front]
        """
        w_rear = v[3]  # rad/s

        # Clamp throttle
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
            )

        return tau_rear

    def f(
        self, x: np.ndarray, u: np.ndarray, t: float = 0.0, params=None
    ) -> np.ndarray:
        """
        Continuous-time forward dynamics.


        x = [
            X, Y, theta,
            vx, vy, r,
            w_rear, w_front,
            torque_engine
        ]

        """

        q, v = self.x2q(x)

        throttle, delta_cmd = self.get_port_values_from_u(u, "thr", "delta")
        throttle = throttle[0]
        delta_cmd = delta_cmd[0]

        # Commanded torque from engine map
        tau_cmd = self.engine_torque_from_throttle(v, throttle)

        # Actual delayed engine torque state
        torque_engine, delta_act = self.get_u_int(x, u)

        # First-order low-pass engine response
        engine_tau = max(self.engine_tau, 1e-6)
        d_tau_engine = (tau_cmd - torque_engine) / engine_tau

        # First-order steering response
        # steering_tau = max(self.steering_tau, 1e-6)
        # d_delta_act = (delta_cmd - delta_act) / steering_tau

        # # Clamp steering command
        delta_cmd = float(np.clip(delta_cmd, self.min_steer, self.max_steer))

        # Clamp current state for use in dynamics
        delta_act_used = float(np.clip(delta_act, self.min_steer, self.max_steer))

        # First-order steering response
        steering_tau = max(self.steering_tau, 1e-6)
        d_delta_act = (delta_cmd - delta_act) / steering_tau

        # Physical steering rate limit
        d_delta_act = float(
            np.clip(
                d_delta_act,
                -self.steering_rate_max,
                self.steering_rate_max,
            )
        )

        # Prevent actuator state from integrating farther outside physical bounds
        if delta_act >= self.max_steer and d_delta_act > 0.0:
            d_delta_act = 0.0

        if delta_act <= self.min_steer and d_delta_act < 0.0:
            d_delta_act = 0.0

        # Use delayed/actual torque in wheel dynamics
        u_drive = np.array([torque_engine, delta_act_used], dtype=float)

        dv = self.accelerations(q, v, u_drive, t)
        dq = self.N_mat(q) @ v

        return np.concatenate([dq, dv, np.array([d_tau_engine, d_delta_act])])

    def get_u_int(self, x: np.ndarray, u: np.ndarray) -> np.ndarray:
        torque_engine = float(x[8])
        delta_act = float(x[9])
        return np.array([torque_engine, delta_act], dtype=float)
