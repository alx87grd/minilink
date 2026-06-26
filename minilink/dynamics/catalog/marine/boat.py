import numpy as np

from minilink.core.backends import array_module
from minilink.core.kinematics import identity_matrix, pose2d_matrix
from minilink.dynamics.abstraction.generalized_mechanical import (
    GeneralizedMechanicalSystem,
)
from minilink.graphical.animation.legacy import (
    legacy_arrow_vector,
    legacy_body_arrow,
    legacy_torque_world,
)
from minilink.graphical.animation.primitives import CustomLine, Point


class Boat2D(GeneralizedMechanicalSystem):
    """Three-DoF planar boat with surge/sway force inputs.

    Low-speed manoeuvring model with linear plus quadratic hydrodynamic
    damping, following Fossen, *Handbook of Marine Craft Hydrodynamics and
    Motion Control* (2nd ed.), section 6.7."""

    def __init__(self):
        super().__init__(dof=3, pos=3, actuators=2)
        self.name = "Planar Boat"
        self.state.labels = ["x", "y", "theta", "surge", "sway", "yaw_rate"]
        self.state.units = ["m", "m", "rad", "m/s", "m/s", "rad/s"]
        self.inputs["u"].labels = ["Tx", "Ty"]
        self.inputs["u"].units = ["N", "N"]
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)

        l_t = 3.0
        self.params = {
            "mass": 10000.0,
            "inertia": 10000.0,
            "l_t": l_t,
            "damping_coef": np.array([10.0, 10.0, 100.0]),
            "Cx_max": 0.5,
            "Cy_max": 0.6,
            "Cm_max": 0.01,
            "N_max": 0.1,
            "rho": 1000.0,
            "Alc": 2.0 * l_t,
            "Afc": 0.5 * l_t,
            "loa": 2.0 * l_t,
        }

        # Graphic parameters (not part of the EoM)
        self.body_length = 2.0 * l_t
        self.body_width = self.params["Afc"]
        self.camera_follow_frame = "body"
        self.camera_scale = 3.0 * self.params["loa"]
        self.show_hydrodynamic_forces = False

    def M(self, q, params=None):
        params = self.params if params is None else params
        mass = params["mass"]
        inertia = params["inertia"]

        # rigid-body inertia: equal surge/sway mass, separate yaw inertia
        return np.diag([mass, mass, inertia])

    def C(self, q, v, params=None):
        params = self.params if params is None else params
        mass = params["mass"]
        yaw_rate = v[2]

        # Coriolis coupling induced by the body-frame yaw rate
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

        # body-to-world rotation mapping body velocities to world rates
        # fmt: off
        return np.array([
            [  c,  -s, 0.0],
            [  s,   c, 0.0],
            [0.0, 0.0, 1.0],
        ])
        # fmt: on

    def B(self, q, params=None):
        params = self.params if params is None else params
        l_t = params["l_t"]

        # thrust acts a distance l_t aft of the c.g., adding a yaw moment
        # fmt: off
        return np.array([
            [1.0,  0.0],
            [0.0,  1.0],
            [0.0, -l_t],
        ])
        # fmt: on

    def current_coefficients(self, alpha, params=None):
        params = self.params if params is None else params
        Cx_max = params["Cx_max"]
        Cy_max = params["Cy_max"]
        Cm_max = params["Cm_max"]

        # quadratic hydrodynamic coefficients vs. attack angle (Fossen Fig. 6.11)
        Cx = -Cx_max * np.cos(alpha) * np.abs(np.cos(alpha))
        Cy = Cy_max * np.sin(alpha) * np.abs(np.sin(alpha))
        Cm = Cm_max * np.sin(2.0 * alpha)
        return Cx, Cy, Cm

    def damping(self, relative_velocity, params=None):
        params = self.params if params is None else params
        damping_coef = params["damping_coef"]
        rho = params["rho"]
        Afc = params["Afc"]
        Alc = params["Alc"]
        loa = params["loa"]
        N_max = params["N_max"]

        vr = relative_velocity
        speed_squared = vr[0] ** 2 + vr[1] ** 2
        alpha = -np.arctan2(vr[1], vr[0])  # attack angle of the relative flow
        Cx, Cy, Cm = self.current_coefficients(alpha, params)

        # linear plus quadratic hydrodynamic damping (Fossen 6.7)
        d_linear = damping_coef * vr
        fx = -0.5 * rho * Afc * Cx * speed_squared
        fy = -0.5 * rho * Alc * Cy * speed_squared
        mz = -0.5 * rho * Alc * loa * Cm * speed_squared
        mz += N_max * rho * Alc * loa * np.abs(vr[2]) * vr[2]
        return d_linear + np.array([fx, fy, mz])

    def d(self, q, v, u=None, t=0.0, params=None):
        params = self.params if params is None else params
        return self.damping(v, params)

    def body_shape(self):
        """Top-view hull silhouette with the c.g. at the local origin.

        The hull spans ``body_length`` along local +X with a pointed bow and is
        centred so the body-frame origin (c.g.) sits at mid-hull.
        """
        l = self.body_length
        w = self.body_width
        pts = np.array(
            [
                [-0.5 * l, -0.5 * w, 0.0],
                [0.35 * l, -0.5 * w, 0.0],
                [0.5 * l, 0.0, 0.0],
                [0.35 * l, 0.5 * w, 0.0],
                [-0.5 * l, 0.5 * w, 0.0],
                [-0.5 * l, -0.5 * w, 0.0],
            ]
        )
        return CustomLine(pts, color="blue", linewidth=2)

    def get_kinematic_geometry(self):
        return {
            "body": [self.body_shape()],
            "cg": [Point(color="blue", marker="o", size=5)],
        }

    def tf(self, x, u, t=0, params=None):
        q = x[:3]
        return {
            "world": identity_matrix(x),
            "body": pose2d_matrix(q[0], q[1], q[2]),
            "cg": pose2d_matrix(q[0], q[1], 0.0),
        }

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        xp = array_module(x)
        q = x[:3]
        l_t = params["l_t"]
        force_scale = 0.0002
        thrust = xp.hypot(u[0], u[1])
        thrust_angle = xp.arctan2(u[1], u[0])
        t_body = pose2d_matrix(q[0], q[1], q[2])
        geometry = {
            "world": [
                legacy_body_arrow(
                    t_body,
                    -l_t,
                    0.0,
                    float(thrust_angle),
                    force_scale * float(thrust),
                    color="red",
                    linewidth=2,
                )
            ],
        }
        if self.show_hydrodynamic_forces:
            rho = params["rho"]
            Alc = params["Alc"]
            loa = params["loa"]
            Cm_max = params["Cm_max"]
            hydro_force = -self.d(q, x[3:], u, t)
            torque_max = abs(0.5 * rho * Alc * loa * Cm_max * 12.0)
            hydro_mag = force_scale * float(xp.hypot(hydro_force[0], hydro_force[1]))
            hydro_angle = float(xp.arctan2(hydro_force[1], hydro_force[0]))
            sweep = float(hydro_force[2] * (2.0 * np.pi) / torque_max)
            geometry["world"].append(
                legacy_body_arrow(
                    t_body,
                    0.0,
                    0.0,
                    hydro_angle,
                    hydro_mag,
                    color="black",
                    linewidth=2,
                )
            )
            geometry["world"].append(
                legacy_torque_world(
                    float(q[0]),
                    float(q[1]),
                    float(q[2]) - np.pi / 2.0,
                    sweep,
                    float(loa) / 5.0,
                    head_ratio=0.4,
                    color="black",
                    linewidth=2,
                )
            )
        return geometry


class Boat2DWithCurrent(Boat2D):
    """Planar boat with constant current velocity in world frame."""

    def __init__(self):
        super().__init__()
        self.name = "Planar Boat With Current"
        self.params["current_velocity"] = np.array([-1.0, -1.0])

    def d(self, q, v, u=None, t=0.0, params=None):
        params = self.params if params is None else params
        current_velocity = params["current_velocity"]

        # damp the water-relative velocity (current subtracted in body frame)
        world_current = np.array([current_velocity[0], current_velocity[1], 0.0])
        body_current = self.N(q, params).T @ world_current
        return self.damping(v - body_current, params)

    def tf(self, x, u, t=0, params=None):
        frames = super().tf(x, u, t, params)
        frames["world"] = identity_matrix(x)
        return frames

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        geometry = super().get_dynamic_geometry(x, u, t, params)
        current_velocity = self.params["current_velocity"]
        loa = self.params["loa"]
        q = x[:3]
        geometry.setdefault("world", []).append(
            legacy_arrow_vector(
                float(q[0]) - loa,
                float(q[1]) + loa,
                current_velocity[0],
                current_velocity[1],
                scale=0.5 * loa,
                color="green",
                linewidth=2,
                origin="tip",
            )
        )
        return geometry


if __name__ == "__main__":
    sys = Boat2D()
    # sys = Boat2DWithCurrent()
    # sys.show_hydrodynamic_forces = True

    sys.x0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    sys.compute_forced(
        lambda t: np.array([1000.0, 50.0 * np.sin(0.5 * t)]),
        tf=10.0,
    )
    sys.animate()
