import numpy as np

from minilink.graphical.primitives import (
    Arrow,
    Box,
    CustomLine,
    Point,
    Rod,
    pose2d_matrix,
    scale_pose2d_matrix,
)
from minilink.mechanics.mechanical import MechanicalSystem


class CartPole(MechanicalSystem):
    """Linear cart with one unactuated pendulum pole."""

    def __init__(self):
        super().__init__(dof=2, actuators=1)

        self.name = "Cart Pole"
        self.params = {
            "l": 3.0,
            "lcg": 0.5,
            "m1": 1.0,
            "m2": 0.1,
            "gravity": 9.81,
        }

        self.state.labels = ["x", "theta", "dx", "dtheta"]
        self.state.units = ["m", "rad", "m/s", "rad/s"]

        uport = self.inputs["u"]
        uport.labels = ["F"]
        uport.units = ["N"]
        uport.lower_bound[0] = -10.0
        uport.upper_bound[0] = 10.0

        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)

    def H(self, q, params=None):
        params = params or self.params
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
        params = params or self.params
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
        params = params or self.params
        theta = q[1]
        g = np.zeros(self.dof)
        g[1] = params["m2"] * params["gravity"] * params["lcg"] * np.sin(theta)
        return g

    def d(self, q, dq, params=None):
        return np.zeros(self.dof)

    def get_kinematic_geometry(self):
        params = self.params
        length = params["l"]

        cart_length = 2.5
        cart_height = 1.5
        cart_depth = 0.8
        wheel_y = -cart_height / 2.0

        return [
            CustomLine(
                [[-10.0, 0.0, 0.0], [10.0, 0.0, 0.0]], color="black", style="--"
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
            Rod(length=length, radius=0.03 * length, color="blue", linewidth=2),
            Arrow(color="red", linewidth=2, origin="tip"),
        ]

    def get_kinematic_transforms(self, x, u, t):
        pos = x[0]
        theta = x[1]
        F = np.asarray(u)[0]

        cart_length = 2.5
        cart_height = 1.5
        cart_depth = 0.8
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


def _pose2d_offset_z(x=0.0, y=0.0, theta=0.0, z=0.0):
    T = pose2d_matrix(x, y, theta)
    T[2, 3] = z
    return T


if __name__ == "__main__":

    cartpole = CartPole()
    cartpole.x0 = np.array([0.0, 0.25, 0.0, 0.0])
    cart_traj = cartpole.compute_forced(
        lambda t: np.array([2.0 * np.sin(2.0 * t)]),
        tf=4.0,
        n_steps=160,
        show=False,
        verbose=False,
    )
    cartpole.animate(cart_traj)
    cartpole.game(renderer="meshcat")
