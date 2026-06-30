import numpy as np

from minilink.core.backends import array_module
from minilink.core.kinematics import SE2
from minilink.dynamics.abstraction.mechanical import MechanicalSystem
from minilink.graphical.animation.primitives import (
    Circle,
    Rod,
    TorqueArrow,
)


class Pendulum(MechanicalSystem):
    """Single actuated pendulum."""

    def __init__(self, length=1.0, mass=1.0):
        super().__init__(dof=1, actuators=1)
        self.name = "Pendulum"
        self.params = {
            "m": mass,
            "l": length,
            "I": 1.0,
            "gravity": 9.81,
            "d": 0.0,
        }
        self.state.labels = ["theta", "dtheta"]
        self.state.units = ["rad", "rad/s"]
        self.inputs["u"].labels = ["tau"]
        self.inputs["u"].units = ["Nm"]
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)

        # Graphic parameters
        self.camera_target = np.array([0.0, 0.0, 0.0])
        self.camera_plot_axes = (0, 1)
        self.camera_scale = length * 1.5

    def H(self, q, params=None):
        params = self.params if params is None else params
        m = params["m"]
        l = params["l"]
        I = params["I"]
        xp = array_module(q)

        # rotational inertia of the bob about the pivot
        return xp.array([[m * l**2 + I]])

    def C(self, q, dq, params=None):
        xp = array_module(q)
        return xp.zeros((1, 1))

    def g(self, q, params=None):
        params = self.params if params is None else params
        m = params["m"]
        l = params["l"]
        gravity = params["gravity"]
        xp = array_module(q)

        # gravity restoring torque
        return xp.array([m * gravity * l * xp.sin(q[0])])

    def d(self, q, dq, u=None, t=0.0, params=None):
        params = self.params if params is None else params
        d = params["d"]
        xp = array_module(q)
        return xp.array([d * dq[0]])

    def get_kinematic_geometry(self):
        length = self.params["l"]
        radius = 0.08 * length
        pivot = Circle(radius=radius, center=[0.0, 0.0], color="blue", fill=True)
        return {
            "link": [
                pivot,
                Rod(length=length, radius=0.03 * length, color="blue", linewidth=2),
                Circle(radius=radius, center=[0.0, -length], color="blue", fill=True),
            ],
        }

    def tf(self, x, u, t=0, params=None):
        return {"link": SE2(0.0, 0.0, x[0])}

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        length = self.params["l"]
        torque = u[0]
        max_torque = self.inputs["u"].upper_bound[0]
        sweep = torque * (2.0 * np.pi / 3.0) / max_torque
        arc = TorqueArrow(
            sweep=sweep,
            radius=length / 5.0,
            head_ratio=0.4,
            color="red",
            linewidth=2,
        )
        arc.local_transform = SE2(0.0, 0.0, -np.pi / 2.0)
        return {"link": [arc]}


class PendulumWithNoisePort(Pendulum):
    """Single actuated pendulum with process and measurement noise ports.

    ``w`` injects disturbance torque into the dynamics; ``v`` corrupts the
    measured angular velocity in ``y``.
    """

    def __init__(self):
        super().__init__()
        self.name = "Pendulum"

        self.add_input_port("w", dim=1, nominal_value=np.array([0.0]))
        self.add_input_port("v", dim=1, nominal_value=np.array([0.0]))
        self.inputs["w"].labels = ["disturbance"]
        self.inputs["w"].units = ["Nm"]
        self.inputs["v"].labels = ["noise"]
        self.inputs["v"].units = ["rad/s"]

        self.outputs["y"].dependencies = ("v",)

    def B(self, q, params=None):
        xp = array_module(q)
        return xp.array([[1.0]])

    def generalized_force(self, q, v, u, t=0.0, params=None):
        tau, w = self.get_port_values_from_u(u, "u", "w")
        return self.B(q, params) @ (tau + w)

    def h(self, x, u, t=0.0, params=None):
        v_noise = self.get_port_values_from_u(u, "v")
        y = x.copy()
        y[self.dof] = x[self.dof] + v_noise[0]
        return y


class InvertedPendulum(Pendulum):
    """Single actuated pendulum with upward zero-angle equilibrium."""

    def __init__(self):
        super().__init__()
        self.name = "Inverted Pendulum"

    def g(self, q, params=None):
        return -super().g(q, params)

    def tf(self, x, u, t=0, params=None):
        # theta is measured from the upward vertical (zero-angle = upright); the
        # shared geometry draws angles from the downward vertical, so add pi.
        upright = np.array([x[0] + np.pi, x[1]])
        return super().tf(upright, u, t)


class TwoIndependentPendulums(MechanicalSystem):
    """Two uncoupled single pendulums sharing the same parameters."""

    def __init__(self):
        super().__init__(dof=2, actuators=2)
        self.name = "Two Independent Pendulums"
        self.params = {
            "m": 1.0,
            "l": 1.0,
            "I": 1.0,
            "gravity": 9.81,
            "d": 0.0,
        }
        self.state.labels = ["theta1", "theta2", "dtheta1", "dtheta2"]
        self.state.units = ["rad", "rad", "rad/s", "rad/s"]
        self.inputs["u"].labels = ["tau1", "tau2"]
        self.inputs["u"].units = ["Nm", "Nm"]
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)

        # Graphic parameters
        self.camera_target = np.array([0.0, 0.0, 0.0])
        self.camera_plot_axes = (0, 1)
        self.camera_scale = 2.0

    def H(self, q, params=None):
        params = self.params if params is None else params
        m = params["m"]
        l = params["l"]
        I = params["I"]
        inertia = m * l**2 + I
        xp = array_module(q)
        return xp.diag(xp.array([inertia, inertia]))

    def C(self, q, dq, params=None):
        xp = array_module(q)
        return xp.zeros((2, 2))

    def g(self, q, params=None):
        params = self.params if params is None else params
        m = params["m"]
        l = params["l"]
        gravity = params["gravity"]
        xp = array_module(q)
        return m * gravity * l * xp.sin(q)

    def d(self, q, dq, u=None, t=0.0, params=None):
        params = self.params if params is None else params
        d = params["d"]
        xp = array_module(q)
        return d * xp.asarray(dq)

    def get_kinematic_geometry(self):
        length = self.params["l"]
        radius = 0.08 * length
        geo = {}
        for i in (0, 1):
            pivot = Circle(radius=radius, center=[0.0, 0.0], color="blue", fill=True)
            geo[f"link{i}"] = [
                pivot,
                Rod(length=length, radius=0.03 * length, color="blue", linewidth=2),
                Circle(radius=radius, center=[0.0, -length], color="blue", fill=True),
            ]
        return geo

    def tf(self, x, u, t=0, params=None):
        length = self.params["l"]
        anchors = [-0.6 * length, 0.6 * length]
        return {
            f"link{i}": SE2(anchor_x, 0.0, x[i]) for i, anchor_x in enumerate(anchors)
        }

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        length = self.params["l"]
        torque_radius = length / 5.0
        max_torque = self.inputs["u"].upper_bound[0]
        sweep_scale = 2.0 * np.pi / 3.0 / max_torque
        dyn = {}
        for i in (0, 1):
            arc = TorqueArrow(
                sweep=u[i] * sweep_scale,
                radius=torque_radius,
                head_ratio=0.4,
                color="red",
                linewidth=2,
            )
            arc.local_transform = SE2(0.0, 0.0, -np.pi / 2.0)
            dyn[f"link{i}"] = [arc]
        return dyn


if __name__ == "__main__":
    sys = Pendulum()
    sys.x0 = np.array([0.7, 0.0])
    sys.compute_forced(
        lambda t: np.array([2.0 * np.sin(t)]),
        tf=5.0,
        n_steps=160,
        show=True,
        verbose=False,
    )
    sys.plot_phase_plane()
    sys.animate()
