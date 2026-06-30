import numpy as np

from minilink.core.kinematics import SE2
from minilink.core.system import DynamicSystem
from minilink.graphical.animation.primitives import Point


class SimpleIntegrator(DynamicSystem):
    """Single integrator ``dx = u``."""

    def __init__(self):
        super().__init__(n=1, input_dim=1, output_dim=1, expose_state=True)
        self.name = "Simple Integrator"
        self.state.labels = ["position"]
        self.state.units = ["m"]
        self.state.lower_bound[:] = [-10.0]
        self.state.upper_bound[:] = [10.0]
        self.inputs["u"].labels = ["speed"]
        self.inputs["u"].units = ["m/s"]
        self.outputs["y"].labels = ["position"]
        self.outputs["y"].units = ["m"]

    def f(self, x, u, t=0.0, params=None):
        return np.array([u[0]])

    def h(self, x, u, t=0.0, params=None):
        return np.array([x[0]])

    def get_kinematic_geometry(self):
        return {}

    def tf(self, x, u, t=0, params=None):
        return {}

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        marker = Point(color="blue", marker="o", size=8)
        marker.local_transform = SE2(x[0], 0.0, 0.0)
        return {"world": [marker]}


class DoubleIntegrator(DynamicSystem):
    """Double integrator ``d(position)/dt = speed``, ``d(speed)/dt = u``."""

    def __init__(self):
        super().__init__(n=2, input_dim=1, output_dim=1, expose_state=True)
        self.name = "Double Integrator"
        self.state.labels = ["position", "speed"]
        self.state.units = ["m", "m/s"]
        self.state.lower_bound[:] = [-10.0, -10.0]
        self.state.upper_bound[:] = [10.0, 10.0]
        self.inputs["u"].labels = ["force"]
        self.inputs["u"].units = ["N"]
        self.outputs["y"].labels = ["position"]
        self.outputs["y"].units = ["m"]

    def f(self, x, u, t=0.0, params=None):
        return np.array([x[1], u[0]])

    def h(self, x, u, t=0.0, params=None):
        return np.array([x[0]])

    def get_kinematic_geometry(self):
        return {}

    def tf(self, x, u, t=0, params=None):
        return {}

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        p0 = Point(color="blue", marker="o", size=8)
        p0.local_transform = SE2(x[0], 0.0, 0.0)
        p1 = Point(color="blue", marker="o", size=8)
        p1.local_transform = SE2(x[1], 0.5, 0.0)
        return {"world": [p0, p1]}


class TripleIntegrator(DynamicSystem):
    """Triple integrator with force as the third state."""

    def __init__(self):
        super().__init__(n=3, input_dim=1, output_dim=1, expose_state=True)
        self.name = "Triple Integrator"
        self.state.labels = ["position", "speed", "force"]
        self.state.units = ["m", "m/s", "N"]
        self.state.lower_bound[:] = [-10.0, -10.0, -10.0]
        self.state.upper_bound[:] = [10.0, 10.0, 10.0]
        self.inputs["u"].labels = ["force_rate"]
        self.inputs["u"].units = ["N/s"]
        self.outputs["y"].labels = ["position"]
        self.outputs["y"].units = ["m"]

    def f(self, x, u, t=0.0, params=None):
        return np.array([x[1], x[2], u[0]])

    def h(self, x, u, t=0.0, params=None):
        return np.array([x[0]])

    def get_kinematic_geometry(self):
        return {}

    def tf(self, x, u, t=0, params=None):
        return {}

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        offsets = (0.0, 0.5, 1.0)
        markers = []
        for xi, y_off in zip(x, offsets):
            marker = Point(color="blue", marker="o", size=8)
            marker.local_transform = SE2(xi, y_off, 0.0)
            markers.append(marker)
        return {"world": markers}


if __name__ == "__main__":
    sys = DoubleIntegrator()
    sys.plot_phase_plane()

    sys.x0 = np.array([-2.0, -2.0])
    sys.inputs["u"].nominal_value = np.array([1.0])
    sys.compute_trajectory(tf=20.0)
    sys.plot_phase_plane()
    sys.animate()
