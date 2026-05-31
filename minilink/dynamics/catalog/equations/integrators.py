import numpy as np

from minilink.core.system import DynamicSystem
from minilink.dynamics.catalog._graphics import (Arrow, Circle,
                                                 arrow_transform, ground_line,
                                                 pose2d_matrix)


def _body_geometry(*, arrows=1):
    geometry = [ground_line(length=12.0), Circle(radius=0.12, color="blue", fill=True)]
    for _ in range(arrows):
        geometry.append(Arrow(color="red", linewidth=2, origin="tail"))
    return geometry


class SimpleIntegrator(DynamicSystem):
    """Single integrator ``dx = u``.

    TRL: 1 - ready for user review.
    """

    def __init__(self):
        super().__init__(n=1, input_dim=1, output_dim=1, expose_state=True)
        self.name = "Simple Integrator"
        self.state.labels = ["position"]
        self.state.units = ["m"]
        self.inputs["u"].labels = ["speed"]
        self.inputs["u"].units = ["m/s"]
        self.outputs["y"].labels = ["position"]
        self.outputs["y"].units = ["m"]

    def f(self, x, u, t=0.0, params=None):
        return np.array([u[0]])

    def h(self, x, u, t=0.0, params=None):
        return np.array([x[0]])

    def get_kinematic_geometry(self):
        return _body_geometry()

    def get_kinematic_transforms(self, x, u, t):
        return [
            pose2d_matrix(0.0, 0.0, 0.0),
            pose2d_matrix(x[0], 0.0, 0.0),
            arrow_transform(x[0], 0.0, u[0], 0.0, scale=0.4),
        ]


class DoubleIntegrator(DynamicSystem):
    """Double integrator ``d(position)/dt = speed``, ``d(speed)/dt = u``.

    TRL: 1 - ready for user review.
    """

    def __init__(self):
        super().__init__(n=2, input_dim=1, output_dim=1, expose_state=True)
        self.name = "Double Integrator"
        self.state.labels = ["position", "speed"]
        self.state.units = ["m", "m/s"]
        self.inputs["u"].labels = ["force"]
        self.inputs["u"].units = ["N"]
        self.outputs["y"].labels = ["position"]
        self.outputs["y"].units = ["m"]

    def f(self, x, u, t=0.0, params=None):
        return np.array([x[1], u[0]])

    def h(self, x, u, t=0.0, params=None):
        return np.array([x[0]])

    def get_kinematic_geometry(self):
        return _body_geometry(arrows=2)

    def get_kinematic_transforms(self, x, u, t):
        return [
            pose2d_matrix(0.0, 0.0, 0.0),
            pose2d_matrix(x[0], 0.0, 0.0),
            arrow_transform(x[0], 0.18, x[1], 0.0, scale=0.4),
            arrow_transform(x[0], -0.18, u[0], 0.0, scale=0.25),
        ]


class TripleIntegrator(DynamicSystem):
    """Triple integrator with force as the third state.

    TRL: 1 - ready for user review.
    """

    def __init__(self):
        super().__init__(n=3, input_dim=1, output_dim=1, expose_state=True)
        self.name = "Triple Integrator"
        self.state.labels = ["position", "speed", "force"]
        self.state.units = ["m", "m/s", "N"]
        self.inputs["u"].labels = ["force_rate"]
        self.inputs["u"].units = ["N/s"]
        self.outputs["y"].labels = ["position"]
        self.outputs["y"].units = ["m"]

    def f(self, x, u, t=0.0, params=None):
        return np.array([x[1], x[2], u[0]])

    def h(self, x, u, t=0.0, params=None):
        return np.array([x[0]])

    def get_kinematic_geometry(self):
        return _body_geometry(arrows=3)

    def get_kinematic_transforms(self, x, u, t):
        return [
            pose2d_matrix(0.0, 0.0, 0.0),
            pose2d_matrix(x[0], 0.0, 0.0),
            arrow_transform(x[0], 0.24, x[1], 0.0, scale=0.35),
            arrow_transform(x[0], 0.0, x[2], 0.0, scale=0.2),
            arrow_transform(x[0], -0.24, u[0], 0.0, scale=0.12),
        ]


if __name__ == "__main__":
    system = DoubleIntegrator()
    system.x0 = np.array([0.0, 0.0])
    system.compute_forced(
        lambda t: np.array([1.0]),
        tf=2.0,
        n_steps=80,
        show=True,
        verbose=False,
    )
