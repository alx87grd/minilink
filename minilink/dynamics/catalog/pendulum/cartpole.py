"""
Pyro-ported cart-pole (linear cart, one pole).

Dynamics match SherbyRobotics/pyro ``CartPole`` in ``pyro/dynamic/cartpole.py``.

Kinematics: the ground + cart are drawn in the X–Y plane; the pole is offset
slightly in ``z`` for volumetric renderers (MeshCat) so the rod cylinder does
not pass through the cart body while matplotlib's default XY projection stays
visually the same.
"""

import numpy as np

from minilink.core.backends import require_jax_numpy
from minilink.dynamics.abstraction.mechanical import (
    JaxMechanicalSystem,
    MechanicalSystem,
)
from minilink.graphical.animation.primitives import (
    Arrow,
    Box,
    CustomLine,
    Point,
    Rod,
    Sphere,
    identity_matrix,
    point_transform,
    pose2d_matrix,
    rod_between_transform,
    scale_pose2d_matrix,
)


class RotatingCartPole(MechanicalSystem):
    """Rotating cart-pole with both joints actuated."""

    def __init__(self):
        super().__init__(dof=2, actuators=2)
        self.name = "Rotating Cart Pole"
        self.params = {
            "l1": 1.0,
            "l2": 1.0,
            "m2": 1.0,
            "I1": 1.0,
            "I2": 0.1,
            "gravity": 9.81,
            "d1": 0.1,
            "d2": 0.1,
        }
        self.state.labels = ["theta1", "theta2", "dtheta1", "dtheta2"]
        self.state.units = ["rad", "rad", "rad/s", "rad/s"]
        self.inputs["u"].labels = ["tau1", "tau2"]
        self.inputs["u"].units = ["Nm", "Nm"]
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)

        # graphic camera framing the spatial mechanism (animate with is_3d=True)
        self.camera_target = np.array([0.0, 0.0, 0.0])
        self.camera_scale = self.params["l1"] + self.params["l2"]

    def H(self, q, params=None):
        params = self.params if params is None else params
        m2 = params["m2"]
        l1 = params["l1"]
        l2 = params["l2"]
        I1 = params["I1"]
        I2 = params["I2"]
        c2 = np.cos(q[1])

        # coupled inertia of the two rotating links
        h01 = m2 * l1 * l2 * c2
        # fmt: off
        return np.array([
            [m2 * l1**2 + I1,             h01],
            [            h01, m2 * l2**2 + I2],
        ])
        # fmt: on

    def C(self, q, dq, params=None):
        params = self.params if params is None else params
        m2 = params["m2"]
        l1 = params["l1"]
        l2 = params["l2"]

        # Coriolis coupling driven by the second joint rate
        c01 = -m2 * l1 * l2 * np.sin(q[1]) * dq[1]
        # fmt: off
        return np.array([
            [0.0, c01],
            [0.0, 0.0],
        ])
        # fmt: on

    def g(self, q, params=None):
        params = self.params if params is None else params
        m2 = params["m2"]
        l2 = params["l2"]
        gravity = params["gravity"]

        # gravity torque acts on the second link only
        return np.array([0.0, -m2 * gravity * l2 * np.sin(q[1])])

    def d(self, q, dq, u=None, t=0.0, params=None):
        params = self.params if params is None else params
        d1 = params["d1"]
        d2 = params["d2"]

        # linear viscous joint damping
        return np.diag([d1, d2]) @ dq

    def get_kinematic_geometry(self):
        l1 = self.params["l1"]
        l2 = self.params["l2"]
        radius = 0.08 * max(l1, l2)
        half = l1 + l2
        ground_z = -l1
        return [
            CustomLine(
                [
                    [-half, -half, ground_z],
                    [-half, half, ground_z],
                    [half, half, ground_z],
                    [half, -half, ground_z],
                    [-half, -half, ground_z],
                ],
                color="black",
                style="--",
            ),
            Rod(length=l1, radius=0.03 * l1, color="black", linewidth=2),
            Rod(length=l1, radius=0.03 * l1, color="blue", linewidth=2),
            Rod(length=l2, radius=0.03 * l2, color="blue", linewidth=2),
            Sphere(radius=radius, color="blue", opacity=0.9),
            Sphere(radius=radius, color="blue", opacity=0.9),
            Sphere(radius=radius, color="blue", opacity=0.9),
        ]

    def get_kinematic_transforms(self, x, u, t):
        l1 = self.params["l1"]
        l2 = self.params["l2"]
        c1, s1 = np.cos(x[0]), np.sin(x[0])
        c2, s2 = np.cos(x[1]), np.sin(x[1])

        # theta1 rotates a horizontal arm about the vertical axis; theta2 swings
        # the pole up out of that arm's plane, so the tip lives in full 3D.
        p_support = np.array([0.0, 0.0, -l1])
        p_pivot = np.array([0.0, 0.0, 0.0])
        p_arm = np.array([l1 * s1, -l1 * c1, 0.0])
        p_tip = p_arm + l2 * np.array([s2 * c1, s2 * s1, c2])

        return [
            identity_matrix(),
            rod_between_transform(p_support, p_pivot),
            rod_between_transform(p_pivot, p_arm),
            rod_between_transform(p_arm, p_tip),
            point_transform(p_pivot),
            point_transform(p_arm),
            point_transform(p_tip),
        ]

    def get_camera_transform(self, x, u, t):
        # open on a 3/4 oblique view; the view-out column encodes (elev, azim)
        # and the interactive 3D toolbar lets the user orbit from there.
        camera = super().get_camera_transform(x, u, t)
        elevation, azimuth = np.radians(22.0), np.radians(-60.0)
        camera[:3, 2] = [
            np.cos(elevation) * np.cos(azimuth),
            np.cos(elevation) * np.sin(azimuth),
            np.sin(elevation),
        ]
        return camera


class UnderactuatedRotatingCartPole(RotatingCartPole):
    """Rotating cart-pole actuated only at the base."""

    def __init__(self):
        super().__init__()
        self.name = "Underactuated Rotating Cart Pole"
        self.inputs.clear()
        self.add_input_port(
            "u",
            dim=1,
            labels=["tau1"],
            units=["Nm"],
            lower_bound=[-5.0],
            upper_bound=[5.0],
        )

    def B(self, q, params=None):
        return np.array([[1.0], [0.0]])


def _configure_cartpole(sys, *, name):
    """Set the shared EoM params, graphic attributes, and port metadata.

    Used by both the NumPy :class:`CartPole` and JAX :class:`JaxCartPole` twins,
    which have different base classes but identical configuration.
    """
    sys.name = name
    sys.params = {
        "lcg": 0.5,
        "m1": 1.0,
        "m2": 0.1,
        "gravity": 9.81,
    }

    # Graphic parameters (not part of the EoM)
    sys.pole_length = 3.0
    sys.cart_length = 2.5
    sys.cart_height = 1.5
    sys.cart_depth = 0.8
    sys.ground_half_width = 10.0

    sys.state.labels = ["x", "theta", "dx", "dtheta"]
    sys.state.units = ["m", "rad", "m/s", "rad/s"]

    uport = sys.inputs["u"]
    uport.labels = ["F"]
    uport.units = ["N"]
    uport.lower_bound[0] = -10.0
    uport.upper_bound[0] = 10.0

    sys.outputs["y"].labels = list(sys.state.labels)
    sys.outputs["y"].units = list(sys.state.units)


class CartPole(MechanicalSystem):
    """Linear cart with one unactuated pendulum pole."""

    def __init__(self):
        super().__init__(dof=2, actuators=1)
        _configure_cartpole(self, name="Cart Pole")

    def H(self, q, params=None):
        params = self.params if params is None else params
        m1 = params["m1"]
        m2 = params["m2"]
        lcg = params["lcg"]
        theta = q[1]

        # cart+pole translation coupled to the pole rotation
        h01 = m2 * lcg * np.cos(theta)
        # fmt: off
        return np.array([
            [m1 + m2,        h01],
            [    h01, m2 * lcg**2],
        ])
        # fmt: on

    def C(self, q, dq, params=None):
        params = self.params if params is None else params
        m2 = params["m2"]
        lcg = params["lcg"]
        theta = q[1]
        dtheta = dq[1]

        # centrifugal term from the swinging pole
        c01 = -m2 * lcg * np.sin(theta) * dtheta
        # fmt: off
        return np.array([
            [0.0, c01],
            [0.0, 0.0],
        ])
        # fmt: on

    def B(self, q, params=None):
        # the force actuates the cart only
        return np.array([[1.0], [0.0]])

    def g(self, q, params=None):
        params = self.params if params is None else params
        m2 = params["m2"]
        lcg = params["lcg"]
        gravity = params["gravity"]
        theta = q[1]

        # gravity torque on the pole
        return np.array([0.0, m2 * gravity * lcg * np.sin(theta)])

    def d(self, q, dq, u=None, t=0.0, params=None):
        return np.zeros(self.dof)

    def get_kinematic_geometry(self):
        pole_length = self.pole_length
        cart_length = self.cart_length
        cart_height = self.cart_height
        cart_depth = self.cart_depth
        wheel_y = -cart_height / 2.0

        return [
            CustomLine(
                [
                    [-self.ground_half_width, 0.0, 0.0],
                    [self.ground_half_width, 0.0, 0.0],
                ],
                color="black",
                style="--",
            ),
            Box(
                length_x=cart_length,
                length_y=cart_height,
                length_z=cart_depth,
                color="black",
                opacity=0.85,
            ),
            Point([0.0, wheel_y, 0.0], color="black", marker="o", size=6),
            Point([0.0, wheel_y, 0.0], color="black", marker="o", size=6),
            Rod(
                length=pole_length,
                radius=0.03 * pole_length,
                color="blue",
                linewidth=2,
            ),
            Arrow(color="red", linewidth=2, origin="tip"),
        ]

    def get_kinematic_transforms(self, x, u, t):
        pos = x[0]
        theta = x[1]
        F = u[0]

        cart_length = self.cart_length
        cart_height = self.cart_height
        cart_depth = self.cart_depth
        wheel_dx = cart_length / 4.0
        pivot_y = cart_height / 2.0
        pole_z = cart_depth / 2.0 + 0.1

        force_len = abs(F) * 0.3
        if F >= 0.0:
            force_x = pos - cart_length / 2.0
            force_theta = 0.0
        else:
            force_x = pos + cart_length / 2.0
            force_theta = np.pi

        # offset the pole in z so the rod clears the cart body in 3D renderers
        pole_pose = pose2d_matrix(pos, pivot_y, theta)
        pole_pose[2, 3] = pole_z

        return [
            pose2d_matrix(0.0, 0.0, 0.0),
            pose2d_matrix(pos, pivot_y, 0.0),
            pose2d_matrix(pos - wheel_dx, pivot_y, 0.0),
            pose2d_matrix(pos + wheel_dx, pivot_y, 0.0),
            pole_pose,
            scale_pose2d_matrix(force_x, pivot_y, force_theta, force_len),
        ]


class JaxCartPole(JaxMechanicalSystem):
    """JAX-traceable linear cart with one unactuated pendulum pole."""

    def __init__(self):
        super().__init__(dof=2, actuators=1)
        _configure_cartpole(self, name="JAX Cart Pole")

    def H(self, q, params=None):
        params = self.params if params is None else params
        jnp = require_jax_numpy()
        m1 = params["m1"]
        m2 = params["m2"]
        lcg = params["lcg"]
        theta = q[1]

        # cart+pole translation coupled to the pole rotation
        h01 = m2 * lcg * jnp.cos(theta)
        # fmt: off
        return jnp.array([
            [m1 + m2,        h01],
            [    h01, m2 * lcg**2],
        ])
        # fmt: on

    def C(self, q, dq, params=None):
        params = self.params if params is None else params
        jnp = require_jax_numpy()
        m2 = params["m2"]
        lcg = params["lcg"]
        theta = q[1]
        dtheta = dq[1]

        # centrifugal term from the swinging pole
        c01 = -m2 * lcg * jnp.sin(theta) * dtheta
        # fmt: off
        return jnp.array([
            [0.0, c01],
            [0.0, 0.0],
        ])
        # fmt: on

    def B(self, q, params=None):
        jnp = require_jax_numpy()
        return jnp.array([[1.0], [0.0]])

    def g(self, q, params=None):
        params = self.params if params is None else params
        jnp = require_jax_numpy()
        m2 = params["m2"]
        lcg = params["lcg"]
        gravity = params["gravity"]
        theta = q[1]

        # gravity torque on the pole
        tau_g = m2 * gravity * lcg * jnp.sin(theta)
        return jnp.array([0.0, tau_g])

    def d(self, q, dq, u=None, t=0.0, params=None):
        jnp = require_jax_numpy()
        return jnp.zeros(self.dof)

    get_kinematic_geometry = CartPole.get_kinematic_geometry
    get_kinematic_transforms = CartPole.get_kinematic_transforms


if __name__ == "__main__":
    sys = RotatingCartPole()
    # sys = UnderactuatedRotatingCartPole()

    sys.x0 = np.array([1.0, 0.25, 0.0, 0.0])
    # sys.inputs["u"].nominal_value = np.array([1.0])
    sys.compute_trajectory(tf=12.0)
    sys.animate(is_3d=True, time_factor_video=1.0)
    # sys.animate()
    # sys.animate(renderer="meshcat")

    sys = CartPole()
    # sys = JaxCartPole()
    sys.params["m1"] = 1.0
    sys.params["m2"] = 2.0
    sys.params["lcg"] = 2.0

    sys.x0 = np.array([1.0, 2.0, 0.0, 0.0])
    # sys.inputs["u"].nominal_value = np.array([1.0])
    sys.compute_trajectory(tf=12.0)
    sys.animate()
