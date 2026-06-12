import numpy as np

from minilink.dynamics.abstraction.mechanical import MechanicalSystem
from minilink.graphical.animation.primitives import (
    Arrow,
    Circle,
    CustomLine,
    arrow_transform,
    identity_matrix,
    translation_matrix,
)


class MountainCar(MechanicalSystem):
    """Mountain car as a one-coordinate constrained mechanical system.

    Dynamics match SherbyRobotics/pyro ``MountainCar`` in
    ``pyro/dynamic/mountaincar.py``: a bead of mass ``mass`` slides along the
    fixed relief curve ``z(x) = a * cos(w * x)`` under gravity."""

    def __init__(self):
        super().__init__(dof=1, actuators=1)
        self.name = "Mountain Car"
        self.params = {
            "mass": 1.0,
            "gravity": 1.0,
            "a": 0.5,
            "w": np.pi,
        }
        self.state.labels = ["x", "dx"]
        self.state.units = ["m", "m/s"]
        self.inputs["u"].labels = ["throttle"]
        self.inputs["u"].units = ["N"]
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)

    def z(self, x, params=None):
        params = self.params if params is None else params
        a = params["a"]
        w = params["w"]
        return a * np.cos(w * x)

    def dz_dx(self, x, params=None):
        params = self.params if params is None else params
        a = params["a"]
        w = params["w"]
        return -a * w * np.sin(w * x)

    def d2z_dx2(self, x, params=None):
        params = self.params if params is None else params
        a = params["a"]
        w = params["w"]
        return -a * w**2 * np.cos(w * x)

    def H(self, q, params=None):
        params = self.params if params is None else params
        mass = params["mass"]
        slope = self.dz_dx(q[0], params)

        # effective inertia of the bead sliding along the curve z(x)
        return np.array([[mass * (1.0 + slope**2)]])

    def C(self, q, dq, params=None):
        params = self.params if params is None else params
        mass = params["mass"]
        slope = self.dz_dx(q[0], params)
        curvature = self.d2z_dx2(q[0], params)

        # Coriolis term from the slope changing along the path
        return np.array([[mass * slope * curvature * dq[0]]])

    def B(self, q, params=None):
        params = self.params if params is None else params
        slope = self.dz_dx(q[0], params)

        # throttle acts tangent to the curve; this projects it onto x
        return np.array([[np.sqrt(1.0 + slope**2)]])

    def g(self, q, params=None):
        params = self.params if params is None else params
        mass = params["mass"]
        gravity = params["gravity"]
        slope = self.dz_dx(q[0], params)

        # gravity pulls the bead back down along the slope
        return np.array([mass * gravity * slope])

    def d(self, q, dq, u=None, t=0.0, params=None):
        return np.zeros(1)

    def forward_kinematic_effector(self, q):
        return np.array([q[0], self.z(q[0])])

    def get_kinematic_geometry(self):
        xs = np.linspace(-1.7, 0.3, 240)
        terrain = np.column_stack([xs, [self.z(x) for x in xs], np.zeros_like(xs)])
        return [
            CustomLine(terrain, color="black", linewidth=2),
            Circle(radius=0.05, center=[0.0, 0.0, 0.0], color="blue", fill=True),
            Arrow(color="red", linewidth=2, origin="base"),
        ]

    def get_kinematic_transforms(self, x, u, t):
        q = x[:1]
        p = self.forward_kinematic_effector(q)
        slope = self.dz_dx(q[0])
        tangent = np.array([1.0, slope])
        tangent = tangent / np.linalg.norm(tangent)
        return [
            identity_matrix(),
            translation_matrix(p[0], p[1], 0.0),
            arrow_transform(
                p[0],
                p[1],
                u[0] * tangent[0],
                u[0] * tangent[1],
                scale=0.3,
            ),
        ]


if __name__ == "__main__":
    sys = MountainCar()
    sys.x0 = np.array([-0.5, 0.0])
    sys.inputs["u"].nominal_value = np.array([0.7])
    sys.compute_trajectory(tf=5.0)
    sys.plot_phase_plane()
    sys.animate()
