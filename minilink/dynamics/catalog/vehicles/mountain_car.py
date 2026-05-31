import numpy as np

from minilink.dynamics.abstraction.mechanical import MechanicalSystem
from minilink.dynamics.catalog._graphics import (Arrow, Circle, CustomLine,
                                                 arrow_transform,
                                                 identity_matrix,
                                                 translation_matrix)


class MountainCar(MechanicalSystem):
    """Mountain car as a one-coordinate constrained mechanical system.

    TRL: 1 - ready for user review.
    """

    def __init__(self):
        super().__init__(dof=1, actuators=1)
        self.name = "Mountain Car"
        self.state.labels = ["x", "dx"]
        self.state.units = ["m", "m/s"]
        self.inputs["u"].labels = ["throttle"]
        self.inputs["u"].units = ["N"]
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)
        self.mass = 1.0
        self.gravity = 1.0
        self.a = 0.5
        self.w = np.pi

    def z(self, x):
        return self.a * np.cos(self.w * x)

    def dz_dx(self, x):
        return -self.a * self.w * np.sin(self.w * x)

    def d2z_dx2(self, x):
        return -self.a * self.w**2 * np.cos(self.w * x)

    def H(self, q, params=None):
        slope = self.dz_dx(q[0])
        return np.array([[self.mass * (1.0 + slope**2)]])

    def C(self, q, dq, params=None):
        slope = self.dz_dx(q[0])
        curvature = self.d2z_dx2(q[0])
        return np.array([[self.mass * slope * curvature * dq[0]]])

    def B(self, q, params=None):
        slope = self.dz_dx(q[0])
        return np.array([[np.sqrt(1.0 + slope**2)]])

    def g(self, q, params=None):
        return np.array([self.mass * self.gravity * self.dz_dx(q[0])])

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
    system = MountainCar()
    system.x0 = np.array([-0.5, 0.0])
    system.compute_forced(
        lambda t: np.array([0.7]),
        tf=5.0,
        n_steps=160,
        show=True,
        verbose=False,
    )
