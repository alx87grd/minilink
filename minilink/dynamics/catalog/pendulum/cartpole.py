"""Cart-poles: the linear cart-pole and the rotating cart-pole, with a noise-port variant (pyro port)."""

import numpy as np

from minilink.core.backends import array_module
from minilink.core.kinematics import SE2, translation
from minilink.dynamics.abstraction.mechanical import MechanicalSystem
from minilink.graphical.animation.primitives import (
    Arrow,
    Box,
    Circle,
    CustomLine,
    Rod,
    Sphere,
    ground_line,
)
from minilink.graphical.catalog.shapes import link_pose_3d, point_pose

###############################################################################
# Public API
###############################################################################


class CartPole(MechanicalSystem):
    """Linear cart with one unactuated pendulum pole."""

    def __init__(self):
        super().__init__(dof=2, actuators=1)
        self.name = "Cart Pole"

        # Parameters
        self.params = {
            "lcg": 0.5,
            "m1": 1.0,
            "m2": 0.1,
            "gravity": 9.81,
        }

        # Signal labels, units and limits: the cart position and the pole angle, one
        # cart force within ±10 N
        self.state.labels = ["x", "theta", "dx", "dtheta"]
        self.state.units = ["m", "rad", "m/s", "rad/s"]
        self.inputs["u"].labels = ["F"]
        self.inputs["u"].units = ["N"]
        self.inputs["u"].lower_bound[0] = -10.0
        self.inputs["u"].upper_bound[0] = 10.0
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)

        # Graphics (not part of the EoM). Camera ±5 matches pyro's
        # ``forward_kinematic_domain``.
        self.pole_length = 3.0
        self.cart_length = 2.5
        self.cart_height = 1.5
        self.cart_depth = 0.8
        self.ground_half_width = 10.0
        self.wheel_radius = 0.15
        self.line_width = 1.5
        self.camera_scale = 10.0

    def H(self, q, params=None):
        params = self.params if params is None else params
        xp = array_module(q)

        m1 = params["m1"]
        m2 = params["m2"]
        lcg = params["lcg"]

        theta = q[1]

        # The cart and the pole translate together, coupled to the pole's rotation
        H11 = m1 + m2
        H12 = m2 * lcg * xp.cos(theta)
        H22 = m2 * lcg**2

        # fmt: off
        H = xp.array([
            [H11, H12],
            [H12, H22],
        ])
        # fmt: on

        return H

    def C(self, q, dq, params=None):
        params = self.params if params is None else params
        xp = array_module(q, dq)

        m2 = params["m2"]
        lcg = params["lcg"]

        theta = q[1]
        dtheta = dq[1]

        # Centrifugal term of the swinging pole
        C12 = -m2 * lcg * xp.sin(theta) * dtheta

        # fmt: off
        C = xp.array([
            [0.0, C12],
            [0.0, 0.0],
        ])
        # fmt: on

        return C

    def B(self, q, params=None):
        xp = array_module(q)

        # The force actuates the cart only
        B = xp.array([[1.0], [0.0]])

        return B

    def g(self, q, params=None):
        params = self.params if params is None else params
        xp = array_module(q)

        m2 = params["m2"]
        lcg = params["lcg"]
        gravity = params["gravity"]

        theta = q[1]

        # Gravity torque on the pole
        g = xp.array([0.0, m2 * gravity * lcg * xp.sin(theta)])

        return g

    def d(self, q, dq, u=None, t=0.0, params=None):
        xp = array_module(q)

        dof = self.dof

        # No damping: τ_d = 0
        tau_d = xp.zeros(dof)

        return tau_d

    def get_kinematic_geometry(self):
        pole_length = self.pole_length
        cart_length = self.cart_length
        cart_height = self.cart_height
        cart_depth = self.cart_depth
        ground_half_width = self.ground_half_width
        wheel_r = self.wheel_radius
        lw = self.line_width

        # The wheels under the cart, a quarter of its length from its centre
        wheel_y = -cart_height / 2.0
        wheel_dx = cart_length / 4.0

        return {
            "world": [ground_line(length=2.0 * ground_half_width)],
            "body": [
                Box(
                    length_x=cart_length,
                    length_y=cart_height,
                    length_z=cart_depth,
                    color="black",
                    opacity=0.85,
                ),
                Circle(
                    radius=wheel_r,
                    center=[-wheel_dx, wheel_y, 0.0],
                    color="black",
                    fill=True,
                ),
                Circle(
                    radius=wheel_r,
                    center=[wheel_dx, wheel_y, 0.0],
                    color="black",
                    fill=True,
                ),
            ],
            "pole": [
                Circle(radius=wheel_r, center=[0.0, 0.0], color="blue", fill=True),
                Rod(
                    length=pole_length,
                    radius=0.03 * pole_length,
                    color="blue",
                    linewidth=lw,
                ),
                Circle(
                    radius=wheel_r,
                    center=[0.0, -pole_length],
                    color="blue",
                    fill=True,
                ),
            ],
        }

    def tf(self, x, u, t=0, params=None):
        cart_length = self.cart_length
        cart_height = self.cart_height
        cart_depth = self.cart_depth

        pos = x[0]
        theta = x[1]
        F = u[0]

        # The pole pivots on top of the cart, drawn in front of it
        pivot_y = cart_height / 2.0
        pole_z = cart_depth / 2.0 + 0.1
        pole_pose = SE2(pos, pivot_y, theta)
        pole_pose[2, 3] = pole_z

        # The force arrow ends on the face of the cart it pushes
        force_x = pos - cart_length / 2.0 if F >= 0.0 else pos + cart_length / 2.0

        return {
            "body": SE2(pos, pivot_y, 0.0),
            "pole": pole_pose,
            "force": translation(force_x, pivot_y, 0.0),
        }

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        F = u[0]

        # The arrow along ±x, 0.3 m per newton, its head at the force frame
        force_len = abs(F) * 0.3
        force_theta = 0.0 if F >= 0.0 else np.pi
        d = np.array([np.cos(force_theta), np.sin(force_theta)])

        return {
            "force": [
                Arrow(
                    base=-force_len * d,
                    vector=d,
                    scale=force_len,
                    head_ratio=0.15,
                    color="red",
                    linewidth=self.line_width,
                )
            ]
        }


class CartPoleWithNoisePort(CartPole):
    """Linear cart-pole with process and measurement noise ports.

    ``d`` injects a disturbance force into the cart; ``n`` corrupts the
    measured state in ``y``.
    """

    def __init__(self):
        super().__init__()
        self.name = "Cart Pole"

        # Ports: the disturbance force d and the measurement noise n
        self.add_input_port("d", dim=1, nominal_value=np.array([0.0]))
        self.add_input_port("n", dim=4, nominal_value=np.zeros(4))

        # Signal labels and units
        self.inputs["d"].labels = ["disturbance"]
        self.inputs["d"].units = ["N"]
        self.inputs["n"].labels = list(self.state.labels)
        self.inputs["n"].units = list(self.state.units)

        # The measurement reads the noise port
        self.outputs["y"].dependencies = ("n",)

    def generalized_force(self, q, v, u, t=0.0, params=None):
        # The ports in order: the cart force, then the disturbance
        F = u[:1]
        d = u[1:2]

        B = self.B(q, params)

        # The disturbance enters with the cart force: B (F + d)
        force = B @ (F + d)

        return force

    def h(self, x, u, t=0.0, params=None):
        xp = array_module(x, u)

        # The third port: the measurement noise
        n = u[2:]

        # y = x + n
        y = xp.asarray(x) + xp.asarray(n)

        return y


class RotatingCartPole(MechanicalSystem):
    """Rotating cart-pole with both joints actuated."""

    def __init__(self):
        super().__init__(dof=2, actuators=2)
        self.name = "Rotating Cart Pole"

        # Parameters
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

        # Signal labels and units
        self.state.labels = ["theta1", "theta2", "dtheta1", "dtheta2"]
        self.state.units = ["rad", "rad", "rad/s", "rad/s"]
        self.inputs["u"].labels = ["tau1", "tau2"]
        self.inputs["u"].units = ["Nm", "Nm"]
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)

        # Graphics: the camera frames the spatial mechanism (animate with is_3d=True)
        self.camera_target = np.array([0.0, 0.0, 0.0])
        self.camera_scale = self.params["l1"] + self.params["l2"]

    def H(self, q, params=None):
        params = self.params if params is None else params
        xp = array_module(q)

        m2 = params["m2"]
        l1 = params["l1"]
        l2 = params["l2"]
        I1 = params["I1"]
        I2 = params["I2"]

        theta2 = q[1]
        c2 = xp.cos(theta2)

        # Coupled inertia of the rotating arm and the pole
        H11 = m2 * l1**2 + I1
        H12 = m2 * l1 * l2 * c2
        H22 = m2 * l2**2 + I2

        # fmt: off
        H = xp.array([
            [H11, H12],
            [H12, H22],
        ])
        # fmt: on

        return H

    def C(self, q, dq, params=None):
        params = self.params if params is None else params
        xp = array_module(q, dq)

        m2 = params["m2"]
        l1 = params["l1"]
        l2 = params["l2"]

        theta2 = q[1]
        dtheta2 = dq[1]

        # Coriolis coupling, driven by the pole's rate
        C12 = -m2 * l1 * l2 * xp.sin(theta2) * dtheta2

        # fmt: off
        C = xp.array([
            [0.0, C12],
            [0.0, 0.0],
        ])
        # fmt: on

        return C

    def g(self, q, params=None):
        params = self.params if params is None else params
        xp = array_module(q)

        m2 = params["m2"]
        l2 = params["l2"]
        gravity = params["gravity"]

        theta2 = q[1]

        # Gravity torque acts on the pole only
        g = xp.array([0.0, -m2 * gravity * l2 * xp.sin(theta2)])

        return g

    def d(self, q, dq, u=None, t=0.0, params=None):
        params = self.params if params is None else params
        xp = array_module(dq)

        d1 = params["d1"]
        d2 = params["d2"]

        # Linear viscous joint damping
        tau_d = xp.array([d1 * dq[0], d2 * dq[1]])

        return tau_d

    def get_kinematic_geometry(self):
        l1 = self.params["l1"]
        l2 = self.params["l2"]

        # A dashed square floor one support length below the base pivot
        radius = 0.08 * max(l1, l2)
        half = l1 + l2
        ground_z = -l1

        return {
            "world": [
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
                )
            ],
            "support": [Rod(length=l1, radius=0.03 * l1, color="black", linewidth=2)],
            "arm": [Rod(length=l1, radius=0.03 * l1, color="blue", linewidth=2)],
            "pole": [Rod(length=l2, radius=0.03 * l2, color="blue", linewidth=2)],
            "pivot": [Sphere(radius=radius, color="blue", opacity=0.9)],
            "armpt": [Sphere(radius=radius, color="blue", opacity=0.9)],
            "tip": [Sphere(radius=radius, color="blue", opacity=0.9)],
        }

    def tf(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        xp = array_module(x)

        l1 = params["l1"]
        l2 = params["l2"]

        theta1 = x[0]
        theta2 = x[1]
        c1 = xp.cos(theta1)
        s1 = xp.sin(theta1)
        c2 = xp.cos(theta2)
        s2 = xp.sin(theta2)

        # The arm sweeps the horizontal plane about the support post; the pole tilts
        # by θ₂ from the vertical, across the arm
        p_support = xp.array([0.0, 0.0, -l1])
        p_pivot = xp.array([0.0, 0.0, 0.0])
        p_arm = xp.array([l1 * s1, -l1 * c1, 0.0])
        p_tip = p_arm + l2 * xp.array([s2 * c1, s2 * s1, c2])

        return {
            "support": link_pose_3d(p_support, p_pivot),
            "arm": link_pose_3d(p_pivot, p_arm),
            "pole": link_pose_3d(p_arm, p_tip),
            "pivot": point_pose(p_pivot),
            "armpt": point_pose(p_arm),
            "tip": point_pose(p_tip),
        }


class UnderactuatedRotatingCartPole(RotatingCartPole):
    """Rotating cart-pole actuated only at the base."""

    def __init__(self):
        super().__init__()
        self.name = "Underactuated Rotating Cart Pole"

        # Ports: one torque, on the base joint, within ±5 N·m
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
        xp = array_module(q)

        # The torque acts on the base joint only
        B = xp.array([[1.0], [0.0]])

        return B


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
    sys.params["m1"] = 1.0
    sys.params["m2"] = 2.0
    sys.params["lcg"] = 2.0

    sys.x0 = np.array([1.0, 2.0, 0.0, 0.0])
    # sys.inputs["u"].nominal_value = np.array([1.0])
    sys.compute_trajectory(tf=12.0)
    sys.animate()
