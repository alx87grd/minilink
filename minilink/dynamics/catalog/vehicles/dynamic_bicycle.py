import numpy as np

from minilink.dynamics.abstraction.generalized_mechanical import (
    GeneralizedMechanicalSystem,
)
from minilink.graphical.animation.primitives import (
    Arrow,
    CustomLine,
    camera_matrix,
    pose2d_matrix,
    scale_pose2d_matrix,
)


class TireModel:
    """Base tire-road interaction model.

    TRL: 1 - ready for user review.
    """

    def __init__(self):
        self.v_min_epsilon = 0.1

    def vel2slip(self, vx, vy, w, radius):
        vx_adjusted = np.abs(vx) + self.v_min_epsilon

        # lateral slip angle and longitudinal slip ratio
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
        Ca = self.Ca
        Ck = self.Ck
        mu = self.mu

        # linear slip-to-force law
        Fx = Ck * kappa
        Fy = Ca * alpha

        # friction-circle saturation
        force_limit = mu * normal_force
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
        B = self.B
        C = self.C
        D = self.D
        E = self.E

        # Pacejka magic formula
        return (
            D
            * normal_force
            * np.sin(C * np.arctan(B * slip - E * (B * slip - np.arctan(B * slip))))
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
        self.params = {
            "mass": 1500.0,
            "inertia": 2500.0,
            "a": 1.0,
            "b": 1.0,
            "gravity": 9.81,
            "rho": 1.225,
            "CdA": 0.3 * 2.2,
        }
        self.state.labels = ["x", "y", "theta", "vx", "vy", "yaw_rate"]
        self.state.units = ["m", "m", "rad", "m/s", "m/s", "rad/s"]
        self.inputs["u"].labels = ["w_rear", "delta"]
        self.inputs["u"].units = ["rad/s", "rad"]
        self.inputs["u"].lower_bound[:] = [-200.0, -0.6]
        self.inputs["u"].upper_bound[:] = [200.0, 0.6]
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)

        # Wheel radii enter the slip equations but are also part of the public
        # interface used by the cascade / trajopt demos, so keep them as plain
        # attributes alongside the graphic and camera parameters.
        self.r_f = 0.3
        self.r_r = 0.3
        self.tire_model_f = LinearTire()
        self.tire_model_r = LinearTire()
        self.wheel_len = 0.6
        self.wheel_width = 0.2
        self.camera_follow_vehicle = True
        self.camera_scale = 8.0

    @property
    def L(self):
        return self.params["a"] + self.params["b"]

    def M(self, q, params=None):
        params = self.params if params is None else params
        mass = params["mass"]
        inertia = params["inertia"]

        # body-frame translational mass and yaw inertia
        return np.diag([mass, mass, inertia])

    def C(self, q, v, params=None):
        params = self.params if params is None else params
        mass = params["mass"]
        yaw_rate = v[2]

        # Coriolis coupling from carrying body-frame momentum through the yaw
        # fmt: off
        return np.array([
            [            0.0, -mass * yaw_rate, 0.0],
            [mass * yaw_rate,              0.0, 0.0],
            [            0.0,              0.0, 0.0],
        ])
        # fmt: on

    def N(self, q, params=None):
        theta = q[2]
        c, s = np.cos(theta), np.sin(theta)

        # body-to-world map turning body velocities into world-frame rates
        # fmt: off
        return np.array([
            [  c,  -s, 0.0],
            [  s,   c, 0.0],
            [0.0, 0.0, 1.0],
        ])
        # fmt: on

    def B(self, q, params=None):
        # inputs act through the tire forces, not a linear actuator matrix
        return np.zeros((self.dof, self.m))

    def wheel_velocities(self, v, u):
        a = self.params["a"]
        b = self.params["b"]
        r_f = self.r_f
        vx, vy, yaw_rate = v
        w_rear, delta = u

        # hub velocities in the body frame
        vx_f_body = vx
        vy_f_body = vy + a * yaw_rate
        vx_r = vx
        vy_r = vy - b * yaw_rate

        # rotate the front hub velocity into the steered-wheel frame
        c_delta, s_delta = np.cos(delta), np.sin(delta)
        vx_f = c_delta * vx_f_body + s_delta * vy_f_body
        vy_f = -s_delta * vx_f_body + c_delta * vy_f_body
        w_front = vx_f / r_f
        return vx_f, vy_f, w_front, vx_r, vy_r, w_rear

    def tire_forces(self, v, u):
        mass = self.params["mass"]
        gravity = self.params["gravity"]
        a = self.params["a"]
        b = self.params["b"]
        L = a + b

        vx_f, vy_f, w_f, vx_r, vy_r, w_r = self.wheel_velocities(v, u)

        # static axle loads split by the center-of-mass position
        Fz_f = mass * gravity * b / L
        Fz_r = mass * gravity * a / L
        Fx_f, Fy_f = self.tire_model_f.vel2forces(vx_f, vy_f, w_f, self.r_f, Fz_f)
        Fx_r, Fy_r = self.tire_model_r.vel2forces(vx_r, vy_r, w_r, self.r_r, Fz_r)
        return Fx_f, Fy_f, Fx_r, Fy_r

    def d(self, q, v, u=None, t=0.0, params=None):
        params = self.params if params is None else params
        a = params["a"]
        b = params["b"]
        rho = params["rho"]
        CdA = params["CdA"]

        Fx_f, Fy_f, Fx_r, Fy_r = self.tire_forces(v, u)
        delta = u[1]
        c_delta, s_delta = np.cos(delta), np.sin(delta)
        Fx_f_body = Fx_f * c_delta - Fy_f * s_delta
        Fy_f_body = Fx_f * s_delta + Fy_f * c_delta

        # net body-frame force and yaw moment from tire and aerodynamic loads
        force_x = Fx_f_body + Fx_r - 0.5 * rho * CdA * v[0] * abs(v[0])
        force_y = Fy_f_body + Fy_r
        moment_z = a * Fy_f_body - b * Fy_r
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
        a = self.params["a"]
        b = self.params["b"]
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
                np.array([[-b, 0.0, 0.0], [a, 0.0, 0.0]]),
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
        a = self.params["a"]
        b = self.params["b"]
        q = x[:3]
        v = x[3:]
        delta = u[1]
        T_body = pose2d_matrix(q[0], q[1], q[2])
        Fx_f, Fy_f, Fx_r, Fy_r = self.tire_forces(v, u)
        v_f = np.array([v[0], v[1] + a * v[2]])
        v_r = np.array([v[0], v[1] - b * v[2]])
        velocity_scale = 0.2
        force_scale = 0.001
        return [
            T_body,
            T_body @ pose2d_matrix(-b, 0.0, 0.0),
            T_body @ pose2d_matrix(a, 0.0, delta),
            T_body
            @ scale_pose2d_matrix(
                a,
                0.0,
                np.arctan2(v_f[1], v_f[0]),
                velocity_scale * np.linalg.norm(v_f),
            ),
            T_body
            @ scale_pose2d_matrix(
                -b,
                0.0,
                np.arctan2(v_r[1], v_r[0]),
                velocity_scale * np.linalg.norm(v_r),
            ),
            T_body
            @ pose2d_matrix(a, 0.0, delta)
            @ scale_pose2d_matrix(
                0.0,
                0.0,
                np.arctan2(Fy_f, Fx_f),
                force_scale * np.hypot(Fx_f, Fy_f),
            ),
            T_body
            @ pose2d_matrix(-b, 0.0, 0.0)
            @ scale_pose2d_matrix(
                0.0,
                0.0,
                np.arctan2(Fy_r, Fx_r),
                force_scale * np.hypot(Fx_r, Fy_r),
            ),
        ]


if __name__ == "__main__":
    sys = DynamicBicycle()
    sys.x0 = np.array([0.0, 0.0, 0.0, 5.0, 0.0, 0.0])
    sys.compute_forced(lambda t: np.array([20.0, 0.1 * np.sin(t)]), tf=3.0)
    sys.animate()
