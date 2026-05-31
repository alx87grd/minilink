import numpy as np

from minilink.dynamics.abstraction.generalized_mechanical import \
    GeneralizedMechanicalSystem
from minilink.graphical.animation.primitives import (Arrow, CustomLine,
                                                     camera_matrix,
                                                     pose2d_matrix,
                                                     scale_pose2d_matrix)


class TireModel:
    """Base tire-road interaction model.

    TRL: 1 - ready for user review.
    """

    def __init__(self):
        self.v_min_epsilon = 0.1

    def vel2slip(self, vx, vy, w, radius):
        vx_adjusted = np.abs(vx) + self.v_min_epsilon
        alpha = -np.arctan(vy / vx_adjusted)
        kappa = (w * radius - vx) / vx_adjusted
        return alpha, kappa

    def slip2forces(self, alpha, kappa, normal_force):
        raise NotImplementedError

    def vel2forces(self, vx, vy, w, radius, normal_force):
        alpha, kappa = self.vel2slip(vx, vy, w, radius)
        return self.slip2forces(alpha, kappa, normal_force)


class LinearTire(TireModel):
    """Linear tire with friction-circle saturation.

    TRL: 1 - ready for user review.
    """

    def __init__(self, Ca=60000.0, Ck=100000.0, mu=1.0):
        super().__init__()
        self.Ca = float(Ca)
        self.Ck = float(Ck)
        self.mu = float(mu)

    def slip2forces(self, alpha, kappa, normal_force):
        Fx = self.Ck * kappa
        Fy = self.Ca * alpha
        force_limit = self.mu * normal_force
        force_norm = np.sqrt(Fx**2 + Fy**2)
        if force_norm > force_limit:
            ratio = force_limit / force_norm
            Fx = ratio * Fx
            Fy = ratio * Fy
        return Fx, Fy


class Pacejka(TireModel):
    """Pacejka-style magic-formula tire.

    TRL: 1 - ready for user review.
    """

    def __init__(self, B=10.0, C=1.3, D=1.0, E=0.97):
        super().__init__()
        self.B = float(B)
        self.C = float(C)
        self.D = float(D)
        self.E = float(E)

    def _magic_formula(self, slip, normal_force):
        return self.D * normal_force * np.sin(
            self.C
            * np.arctan(
                self.B * slip
                - self.E * (self.B * slip - np.arctan(self.B * slip))
            )
        )

    def slip2forces(self, alpha, kappa, normal_force):
        return (
            self._magic_formula(kappa, normal_force),
            self._magic_formula(alpha, normal_force),
        )


class DynamicBicycle(GeneralizedMechanicalSystem):
    """Planar dynamic bicycle with rear wheel-speed and steering inputs.

    State: ``[x, y, theta, vx, vy, yaw_rate]`` where velocities are body-frame.
    Input: ``[w_rear, delta]``.

    TRL: 1 - ready for user review.
    """

    def __init__(self):
        super().__init__(dof=3, pos=3, actuators=2)
        self.name = "Dynamic Bicycle"
        self.state.labels = ["x", "y", "theta", "vx", "vy", "yaw_rate"]
        self.state.units = ["m", "m", "rad", "m/s", "m/s", "rad/s"]
        self.inputs["u"].labels = ["w_rear", "delta"]
        self.inputs["u"].units = ["rad/s", "rad"]
        self.inputs["u"].lower_bound[:] = [-200.0, -0.6]
        self.inputs["u"].upper_bound[:] = [200.0, 0.6]
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)

        self.a = 1.0
        self.b = 1.0
        self.r_f = 0.3
        self.r_r = 0.3
        self.mass = 1500.0
        self.inertia = 2500.0
        self.gravity = 9.81
        self.rho = 1.225
        self.CdA = 0.3 * 2.2
        self.tire_model_f = LinearTire()
        self.tire_model_r = LinearTire()
        self.wheel_len = 0.6
        self.wheel_width = 0.2
        self.camera_follow_vehicle = True
        self.camera_scale = 8.0

    @property
    def L(self):
        return self.a + self.b

    def M(self, q, params=None):
        return np.diag([self.mass, self.mass, self.inertia])

    def C(self, q, v, params=None):
        yaw_rate = v[2]
        return np.array(
            [
                [0.0, -self.mass * yaw_rate, 0.0],
                [self.mass * yaw_rate, 0.0, 0.0],
                [0.0, 0.0, 0.0],
            ]
        )

    def N(self, q, params=None):
        theta = q[2]
        c, s = np.cos(theta), np.sin(theta)
        return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])

    def B(self, q, params=None):
        return np.zeros((self.dof, self.m))

    def wheel_velocities(self, v, u):
        vx, vy, yaw_rate = v
        w_rear, delta = u
        vx_f_body = vx
        vy_f_body = vy + self.a * yaw_rate
        vx_r = vx
        vy_r = vy - self.b * yaw_rate
        c_delta, s_delta = np.cos(delta), np.sin(delta)
        vx_f = c_delta * vx_f_body + s_delta * vy_f_body
        vy_f = -s_delta * vx_f_body + c_delta * vy_f_body
        w_front = vx_f / self.r_f
        return vx_f, vy_f, w_front, vx_r, vy_r, w_rear

    def tire_forces(self, v, u):
        vx_f, vy_f, w_f, vx_r, vy_r, w_r = self.wheel_velocities(v, u)
        Fz_f = self.mass * self.gravity * self.b / self.L
        Fz_r = self.mass * self.gravity * self.a / self.L
        Fx_f, Fy_f = self.tire_model_f.vel2forces(vx_f, vy_f, w_f, self.r_f, Fz_f)
        Fx_r, Fy_r = self.tire_model_r.vel2forces(vx_r, vy_r, w_r, self.r_r, Fz_r)
        return Fx_f, Fy_f, Fx_r, Fy_r

    def d(self, q, v, u=None, t=0.0, params=None):
        Fx_f, Fy_f, Fx_r, Fy_r = self.tire_forces(v, u)
        delta = u[1]
        c_delta, s_delta = np.cos(delta), np.sin(delta)
        Fx_f_body = Fx_f * c_delta - Fy_f * s_delta
        Fy_f_body = Fx_f * s_delta + Fy_f * c_delta
        force_x = Fx_f_body + Fx_r
        force_y = Fy_f_body + Fy_r
        moment_z = self.a * Fy_f_body - self.b * Fy_r
        force_x -= 0.5 * self.rho * self.CdA * v[0] * abs(v[0])
        return -np.array([force_x, force_y, moment_z])

    def get_camera_transform(self, x, u, t):
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
        wl, ww = self.wheel_len / 2.0, self.wheel_width / 2.0
        wheel = np.array(
            [
                [wl, ww, 0.0],
                [wl, -ww, 0.0],
                [-wl, -ww, 0.0],
                [-wl, ww, 0.0],
                [wl, ww, 0.0],
            ]
        )
        return [
            CustomLine(
                np.array([[-self.b, 0.0, 0.0], [self.a, 0.0, 0.0]]),
                color="black",
                linewidth=2,
            ),
            CustomLine(wheel, color="black", linewidth=1),
            CustomLine(wheel, color="black", linewidth=1),
            Arrow(color="blue", linewidth=2, origin="base"),
            Arrow(color="blue", linewidth=2, origin="base"),
            Arrow(color="red", linewidth=2, origin="base"),
            Arrow(color="red", linewidth=2, origin="base"),
        ]

    def get_kinematic_transforms(self, x, u, t):
        q = x[:3]
        v = x[3:]
        delta = u[1]
        T_body = pose2d_matrix(q[0], q[1], q[2])
        Fx_f, Fy_f, Fx_r, Fy_r = self.tire_forces(v, u)
        v_f = np.array([v[0], v[1] + self.a * v[2]])
        v_r = np.array([v[0], v[1] - self.b * v[2]])
        velocity_scale = 0.2
        force_scale = 0.001
        return [
            T_body,
            T_body @ pose2d_matrix(-self.b, 0.0, 0.0),
            T_body @ pose2d_matrix(self.a, 0.0, delta),
            T_body
            @ scale_pose2d_matrix(
                self.a,
                0.0,
                np.arctan2(v_f[1], v_f[0]),
                velocity_scale * np.linalg.norm(v_f),
            ),
            T_body
            @ scale_pose2d_matrix(
                -self.b,
                0.0,
                np.arctan2(v_r[1], v_r[0]),
                velocity_scale * np.linalg.norm(v_r),
            ),
            T_body
            @ pose2d_matrix(self.a, 0.0, delta)
            @ scale_pose2d_matrix(
                0.0,
                0.0,
                np.arctan2(Fy_f, Fx_f),
                force_scale * np.hypot(Fx_f, Fy_f),
            ),
            T_body
            @ pose2d_matrix(-self.b, 0.0, 0.0)
            @ scale_pose2d_matrix(
                0.0,
                0.0,
                np.arctan2(Fy_r, Fx_r),
                force_scale * np.hypot(Fx_r, Fy_r),
            ),
        ]


if __name__ == "__main__":
    system = DynamicBicycle()
    system.x0 = np.array([0.0, 0.0, 0.0, 5.0, 0.0, 0.0])
    traj = system.compute_forced(
        lambda t: np.array([20.0, 0.1 * np.sin(t)]),
        tf=3.0,
        n_steps=120,
        show=True,
        verbose=False,
    )
    system.animate(traj)
