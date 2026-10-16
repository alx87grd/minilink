"""Planar rocket with thrust magnitude and gimbal angle inputs."""

import numpy as np

from minilink.core.backends import array_module
from minilink.core.kinematics import SE2, translation
from minilink.dynamics.abstraction.mechanical import MechanicalSystem
from minilink.graphical.animation.primitives import (
    Arrow,
    CustomLine,
    Point,
    ground_line,
)

###############################################################################
# Public API
###############################################################################


class Rocket(MechanicalSystem):
    """Planar rocket with thrust magnitude and gimbal angle inputs."""

    def __init__(self):
        super().__init__(dof=3, actuators=2)
        self.name = "Planar Rocket"

        # Parameters
        self.params = {
            "mass": 1000.0,
            "inertia": 100.0,
            "ycg": 1.0,
            "gravity": 9.8,
            "cda": 1.0,
        }

        # Signal labels and units
        self.state.labels = ["x", "y", "theta", "vx", "vy", "omega"]
        self.state.units = ["m", "m", "rad", "m/s", "m/s", "rad/s"]
        self.inputs["u"].labels = ["thrust", "delta"]
        self.inputs["u"].units = ["N", "rad"]
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)

        # Graphics: the body's size, and a camera that follows it
        self.width = 0.4
        self.height = 2.0
        self.dynamic_range = 10.0
        self.camera_scale = self.dynamic_range
        self.camera_follow_frame = "body"

    def H(self, q, params=None):
        params = self.params if params is None else params
        xp = array_module(q)

        mass = params["mass"]
        inertia = params["inertia"]

        H = xp.diag(xp.array([mass, mass, inertia]))

        return H

    def C(self, q, dq, params=None):
        xp = array_module(q)

        # A constant inertia matrix: no Coriolis or centrifugal terms
        C = xp.zeros((3, 3))

        return C

    def g(self, q, params=None):
        params = self.params if params is None else params
        xp = array_module(q)

        mass = params["mass"]
        gravity = params["gravity"]

        # The weight along +y (g sits on the left side of the equation of motion)
        g = xp.array([0.0, mass * gravity, 0.0])

        return g

    def d(self, q, dq, u=None, t=0.0, params=None):
        params = self.params if params is None else params
        xp = array_module(dq)

        cda = params["cda"]

        vx = dq[0]
        vy = dq[1]
        omega = dq[2]

        # Quadratic aerodynamic drag c_dA v|v| on each velocity, plus a small linear
        # damping, 10⁻² on every axis
        tau_d = xp.array(
            [
                cda * vx * xp.abs(vx) + 0.01 * vx,
                cda * vy * xp.abs(vy) + 0.01 * vy,
                0.01 * omega,
            ]
        )

        return tau_d

    def generalized_force(self, q, dq, u, t=0.0, params=None):
        params = self.params if params is None else params
        xp = array_module(q, u)

        ycg = params["ycg"]

        thrust, delta = u
        theta = q[2]

        # Gimballed thrust: the force along the nozzle axis, and its torque about the
        # c.g. at the arm y_cg: τ = T [−sin(θ + δ), cos(θ + δ), −y_cg sin δ]
        tau = thrust * xp.array(
            [
                -xp.sin(theta + delta),
                xp.cos(theta + delta),
                -ycg * xp.sin(delta),
            ]
        )

        return tau

    def get_kinematic_geometry(self):
        return {
            "body": [self.body_shape()],
            "center": [Point(color="black", marker="o", size=5)],
            "world": [ground_line(length=200.0, y=0.0, color="black", style="--")],
        }

    def tf(self, x, u, t=0, params=None):
        q = x[:3]

        return {
            "body": SE2(q[0], q[1], q[2]),
            "center": SE2(q[0], q[1], 0.0),
        }

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        # The thrust arrow, 2·10⁻⁴ m per newton along the gimballed nozzle axis,
        # ending at the nozzle 1 m below the c.g.
        length = 0.0002 * u[0]
        angle = np.pi / 2.0 + u[1]
        d = np.array([np.cos(angle), np.sin(angle)])
        thrust = Arrow(
            base=-length * d,
            vector=d,
            scale=length,
            color="red",
            linewidth=2,
        )
        thrust.local_transform = translation(0.0, -1.0, 0.0)

        return {"body": [thrust]}

    ###########################################################################
    # Internal machinery
    ###########################################################################

    def body_shape(self):
        """Side-view rocket silhouette with the c.g. at the local origin."""
        w = self.width
        h = self.height
        pts = np.array(
            [
                [-0.5 * w, -0.5 * h, 0.0],
                [-0.5 * w, 0.35 * h, 0.0],
                [0.0, 0.5 * h, 0.0],
                [0.5 * w, 0.35 * h, 0.0],
                [0.5 * w, -0.5 * h, 0.0],
                [-0.5 * w, -0.5 * h, 0.0],
            ]
        )
        return CustomLine(pts, color="blue", linewidth=2)


if __name__ == "__main__":
    sys = Rocket()

    sys.x0 = np.array([0.0, 0.0, 0.1, 0.0, 0.0, 0.0])
    sys.inputs["u"].nominal_value = np.array([15000.0, 0.01])

    sys.compute_trajectory(tf=3.0)
    sys.animate()
