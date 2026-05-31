"""
Pyro-ported cart-pole (linear cart, one pole).

Dynamics match SherbyRobotics/pyro ``CartPole`` in ``pyro/dynamic/cartpole.py``.

Kinematics: the ground + cart are drawn in the X–Y plane; the pole is offset
slightly in ``z`` for volumetric renderers (MeshCat) so the rod cylinder does
not pass through the cart body while matplotlib's default XY projection stays
visually the same.
"""

import numpy as np

from minilink.compile.jax_utils import require_jax_numpy
from minilink.dynamics.abstraction.mechanical import (JaxMechanicalSystem,
                                                      MechanicalSystem)
from minilink.graphical.animation.primitives import (Arrow, Box, Circle,
                                                     CustomLine, Point, Rod,
                                                     TorqueArrow,
                                                     pose2d_matrix,
                                                     scale_pose2d_matrix,
                                                     torque_pose2d_matrix)


class RotatingCartPole(MechanicalSystem):
    """Rotating cart-pole with both joints actuated.

    TRL: 1 - ready for user review.
    """

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

    def H(self, q, params=None):
        params = self.params if params is None else params
        c2 = np.cos(q[1])
        m2 = params["m2"]
        l1 = params["l1"]
        l2 = params["l2"]
        h01 = m2 * l1 * l2 * c2
        return np.array(
            [
                [m2 * l1**2 + params["I1"], h01],
                [h01, m2 * l2**2 + params["I2"]],
            ]
        )

    def C(self, q, dq, params=None):
        params = self.params if params is None else params
        c01 = -params["m2"] * params["l1"] * params["l2"] * np.sin(q[1]) * dq[1]
        return np.array([[0.0, c01], [0.0, 0.0]])

    def g(self, q, params=None):
        params = self.params if params is None else params
        return np.array(
            [0.0, -params["m2"] * params["gravity"] * params["l2"] * np.sin(q[1])]
        )

    def d(self, q, dq, u=None, t=0.0, params=None):
        params = self.params if params is None else params
        return np.diag([params["d1"], params["d2"]]) @ dq

    def _joint_positions(self, q, params=None):
        params = self.params if params is None else params
        theta1, theta2 = q
        l1 = params["l1"]
        l2 = params["l2"]
        p0 = np.array([0.0, 0.0])
        p1 = l1 * np.array([np.cos(theta1), np.sin(theta1)])
        p2 = p1 + l2 * np.array(
            [np.cos(theta1 + theta2), np.sin(theta1 + theta2)]
        )
        return p0, p1, p2

    def get_kinematic_geometry(self):
        params = self.params
        radius = 0.08 * max(params["l1"], params["l2"])
        torque_radius = 0.2 * max(params["l1"], params["l2"])
        return [
            Circle(radius=radius, center=[0.0, 0.0], color="blue", fill=True),
            Rod(
                length=params["l1"],
                radius=0.03 * params["l1"],
                color="blue",
                linewidth=2,
            ),
            Circle(radius=radius, center=[0.0, 0.0], color="blue", fill=True),
            Rod(
                length=params["l2"],
                radius=0.03 * params["l2"],
                color="blue",
                linewidth=2,
            ),
            Circle(radius=radius, center=[0.0, 0.0], color="blue", fill=True),
            TorqueArrow(
                radius=torque_radius,
                head_ratio=0.4,
                color="red",
                linewidth=2,
            ),
            TorqueArrow(
                radius=torque_radius,
                head_ratio=0.4,
                color="red",
                linewidth=2,
            ),
        ]

    def get_kinematic_transforms(self, x, u, t):
        q = x[: self.dof]
        p0, p1, p2 = self._joint_positions(q)
        heading1 = q[0]
        heading2 = q[0] + q[1]
        u_lim = float(self.inputs["u"].upper_bound[0])
        torque_scale = 2.0 * np.pi / 3.0 / u_lim
        return [
            pose2d_matrix(p0[0], p0[1], 0.0),
            pose2d_matrix(p0[0], p0[1], heading1 + np.pi / 2.0),
            pose2d_matrix(p1[0], p1[1], 0.0),
            pose2d_matrix(p1[0], p1[1], heading2 + np.pi / 2.0),
            pose2d_matrix(p2[0], p2[1], 0.0),
            torque_pose2d_matrix(p0[0], p0[1], heading1, u[0] * torque_scale),
            torque_pose2d_matrix(p1[0], p1[1], heading2, u[1] * torque_scale),
        ]


class UnderactuatedRotatingCartPole(RotatingCartPole):
    """Rotating cart-pole actuated only at the base.

    TRL: 1 - ready for user review.
    """

    def __init__(self):
        super().__init__()
        self.name = "Underactuated Rotating Cart Pole"
        self.m = 0
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

    def get_kinematic_geometry(self):
        return super().get_kinematic_geometry()[:-1]

    def get_kinematic_transforms(self, x, u, t):
        transforms = super().get_kinematic_transforms(x, np.array([u[0], 0.0]), t)
        return transforms[:-1]


def _cartpole_params():
    return {
        "l": 3.0,
        "lcg": 0.5,
        "m1": 1.0,
        "m2": 0.1,
        "gravity": 9.81,
        "cart_length": 2.5,
        "cart_height": 1.5,
        "cart_depth": 0.8,
        "ground_half_width": 10.0,
    }


def _configure_cartpole_metadata(sys, *, name: str) -> None:
    sys.name = name
    sys.params = _cartpole_params()

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
    """Linear cart with one unactuated pendulum pole.

    TRL: 1 - ready for user review.
    """

    def __init__(self):
        super().__init__(dof=2, actuators=1)
        _configure_cartpole_metadata(self, name="Cart Pole")

    def H(self, q, params=None):
        params = self.params if params is None else params
        theta = q[1]
        m1 = params["m1"]
        m2 = params["m2"]
        lcg = params["lcg"]

        H = np.zeros((self.dof, self.dof))
        H[0, 0] = m1 + m2
        H[1, 0] = m2 * lcg * np.cos(theta)
        H[0, 1] = H[1, 0]
        H[1, 1] = m2 * lcg**2
        return H

    def C(self, q, dq, params=None):
        params = self.params if params is None else params
        theta = q[1]
        dtheta = dq[1]
        m2 = params["m2"]
        lcg = params["lcg"]

        C = np.zeros((self.dof, self.dof))
        C[0, 1] = -m2 * lcg * np.sin(theta) * dtheta
        return C

    def B(self, q, params=None):
        B = np.zeros((self.dof, self.m))
        B[0, 0] = 1.0
        return B

    def g(self, q, params=None):
        params = self.params if params is None else params
        theta = q[1]
        g = np.zeros(self.dof)
        g[1] = params["m2"] * params["gravity"] * params["lcg"] * np.sin(theta)
        return g

    def d(self, q, dq, u=None, t=0.0, params=None):
        return np.zeros(self.dof)

    def get_kinematic_geometry(self):
        params = self.params
        length = params["l"]

        cart_length = float(params["cart_length"])
        cart_height = float(params["cart_height"])
        cart_depth = float(params["cart_depth"])
        wheel_y = -cart_height / 2.0
        wheel_dx = cart_length / 4.0

        return [
            CustomLine(
                [
                    [-params["ground_half_width"], 0.0, 0.0],
                    [params["ground_half_width"], 0.0, 0.0],
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
            Point([-wheel_dx, wheel_y, 0.0], color="black", marker="o", size=6),
            Point([wheel_dx, wheel_y, 0.0], color="black", marker="o", size=6),
            Rod(length=length, radius=0.03 * length, color="blue", linewidth=2),
            Arrow(color="red", linewidth=2, origin="tip"),
        ]

    def get_kinematic_transforms(self, x, u, t):
        pos = x[0]
        theta = x[1]
        F = u[0]

        cart_length = float(self.params["cart_length"])
        cart_height = float(self.params["cart_height"])
        cart_depth = float(self.params["cart_depth"])
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

        return [
            pose2d_matrix(0.0, 0.0, 0.0),
            pose2d_matrix(pos, pivot_y, 0.0),
            pose2d_matrix(pos - wheel_dx, pivot_y, 0.0),
            pose2d_matrix(pos + wheel_dx, pivot_y, 0.0),
            _pose2d_offset_z(pos, pivot_y, theta, pole_z),
            scale_pose2d_matrix(force_x, pivot_y, force_theta, force_len),
        ]


class JaxCartPole(JaxMechanicalSystem):
    """JAX-traceable linear cart with one unactuated pendulum pole.

    TRL: 1 - ready for user review.
    """

    def __init__(self):
        super().__init__(dof=2, actuators=1)
        _configure_cartpole_metadata(self, name="JAX Cart Pole")

    def H(self, q, params=None):
        params = self.params if params is None else params
        jnp = require_jax_numpy()
        theta = q[1]
        m1 = params["m1"]
        m2 = params["m2"]
        lcg = params["lcg"]
        h01 = m2 * lcg * jnp.cos(theta)
        return jnp.array(
            [
                [m1 + m2, h01],
                [h01, m2 * lcg**2],
            ]
        )

    def C(self, q, dq, params=None):
        params = self.params if params is None else params
        jnp = require_jax_numpy()
        theta = q[1]
        dtheta = dq[1]
        m2 = params["m2"]
        lcg = params["lcg"]
        c01 = -m2 * lcg * jnp.sin(theta) * dtheta
        return jnp.array(
            [
                [0.0, c01],
                [0.0, 0.0],
            ]
        )

    def B(self, q, params=None):
        jnp = require_jax_numpy()
        return jnp.array([[1.0], [0.0]])

    def g(self, q, params=None):
        params = self.params if params is None else params
        jnp = require_jax_numpy()
        theta = q[1]
        tau_g = params["m2"] * params["gravity"] * params["lcg"] * jnp.sin(theta)
        return jnp.array([0.0, tau_g])

    def d(self, q, dq, u=None, t=0.0, params=None):
        jnp = require_jax_numpy()
        return jnp.zeros(self.dof)

    get_kinematic_geometry = CartPole.get_kinematic_geometry
    get_kinematic_transforms = CartPole.get_kinematic_transforms


def _pose2d_offset_z(x=0.0, y=0.0, theta=0.0, z=0.0):
    """Like :func:`pose2d_matrix`, but with an extra ``z`` translation (for 3D)."""
    T = pose2d_matrix(x, y, theta)
    T[2, 3] = z
    return T


if __name__ == "__main__":
    cartpole = CartPole()
    cartpole.x0 = np.array([0.0, 0.25, 0.0, 0.0])
    traj = cartpole.compute_forced(
        lambda t: np.array([2.0 * np.sin(2.0 * t)]),
        tf=4.0,
        n_steps=160,
        show=False,
        verbose=False,
    )
    cartpole.animate(traj)
