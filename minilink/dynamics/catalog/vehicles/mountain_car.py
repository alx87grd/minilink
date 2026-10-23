"""Mountain car: a one-coordinate mechanical system on a cosine hill."""

import numpy as np

from minilink.core.backends import array_module
from minilink.core.kinematics import SE2
from minilink.dynamics.abstraction.mechanical import MechanicalSystem
from minilink.graphical.animation.primitives import (
    Arrow,
    Circle,
    CustomLine,
)

###############################################################################
# Public API
###############################################################################


class MountainCar(MechanicalSystem):
    """Mountain car as a one-coordinate constrained mechanical system.

    Dynamics match SherbyRobotics/pyro ``MountainCar`` in
    ``pyro/dynamic/mountaincar.py``: a bead of mass ``mass`` slides along the
    fixed relief curve ``z(x) = a * cos(w * x)`` under gravity.
    """

    def __init__(self):
        super().__init__(dof=1, actuators=1)

        # Parameters: the bead's mass, gravity, and the hill z(x) = a cos(w x)
        self.params = {
            "mass": 1.0,
            "gravity": 1.0,
            "a": 0.5,
            "w": np.pi,
        }

        # Signal labels and units
        self.state.labels = ["x", "dx"]
        self.state.units = ["m", "m/s"]
        self.inputs["u"].labels = ["throttle"]
        self.inputs["u"].units = ["N"]
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)

    def z(self, x, params=None):
        params = self.params if params is None else params
        xp = array_module(x)

        a = params["a"]
        w = params["w"]

        # The hill profile
        z = a * xp.cos(w * x)

        return z

    def dz_dx(self, x, params=None):
        params = self.params if params is None else params
        xp = array_module(x)

        a = params["a"]
        w = params["w"]

        # The slope of the hill
        dz_dx = -a * w * xp.sin(w * x)

        return dz_dx

    def d2z_dx2(self, x, params=None):
        params = self.params if params is None else params
        xp = array_module(x)

        a = params["a"]
        w = params["w"]

        # The curvature of the hill
        d2z_dx2 = -a * w**2 * xp.cos(w * x)

        return d2z_dx2

    def H(self, q, params=None):
        params = self.params if params is None else params
        xp = array_module(q)

        mass = params["mass"]

        slope = self.dz_dx(q[0], params)

        # Effective inertia of the bead sliding along the curve z(x)
        H11 = mass * (1.0 + slope**2)

        H = xp.array([[H11]])

        return H

    def C(self, q, dq, params=None):
        params = self.params if params is None else params
        xp = array_module(q, dq)

        mass = params["mass"]

        slope = self.dz_dx(q[0], params)
        curvature = self.d2z_dx2(q[0], params)

        # Coriolis term from the slope changing along the path
        C11 = mass * slope * curvature * dq[0]

        C = xp.array([[C11]])

        return C

    def B(self, q, params=None):
        params = self.params if params is None else params
        xp = array_module(q)

        slope = self.dz_dx(q[0], params)

        # The throttle acts tangent to the curve; this projects it onto x
        B11 = xp.sqrt(1.0 + slope**2)

        B = xp.array([[B11]])

        return B

    def g(self, q, params=None):
        params = self.params if params is None else params
        xp = array_module(q)

        mass = params["mass"]
        gravity = params["gravity"]

        slope = self.dz_dx(q[0], params)

        # Gravity pulls the bead back down along the slope
        g = xp.array([mass * gravity * slope])

        return g

    def d(self, q, dq, u=None, t=0.0, params=None):
        xp = array_module(q)

        # No damping: τ_d = 0
        tau_d = xp.zeros(1)

        return tau_d

    def forward_kinematic_effector(self, q, params=None):
        """The bead's position ``(x, z(x))`` on the hill."""
        params = self.params if params is None else params
        xp = array_module(q)

        x = q[0]
        z = self.z(x, params)

        p = xp.array([x, z])

        return p

    def get_kinematic_geometry(self):
        # The terrain, sampled over the course
        xs = np.linspace(-1.7, 0.3, 240)
        terrain = np.column_stack([xs, [self.z(x) for x in xs], np.zeros_like(xs)])

        return {
            "world": [CustomLine(terrain, color="black", linewidth=2)],
            "body": [
                Circle(radius=0.05, center=[0.0, 0.0, 0.0], color="blue", fill=True)
            ],
        }

    def tf(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        xp = array_module(x)

        q = x[:1]
        p = self.forward_kinematic_effector(q, params)

        # The body frame rides the curve, its x axis along the terrain tangent
        slope = self.dz_dx(q[0], params)
        theta = xp.arctan(slope)

        return {"body": SE2(p[0], p[1], theta)}

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        throttle = u[0]

        # The throttle arrow along the body x axis (the terrain tangent; see tf)
        return {
            "body": [
                Arrow(
                    base=(0.0, 0.0),
                    vector=(throttle, 0.0),
                    scale=0.3,
                    color="red",
                    linewidth=2,
                )
            ]
        }


if __name__ == "__main__":
    sys = MountainCar()
    sys.x0 = np.array([-0.5, 0.0])
    sys.inputs["u"].nominal_value = np.array([0.7])
    sys.compute_trajectory(tf=5.0)
    sys.plot_phase_plane()
    sys.animate()
