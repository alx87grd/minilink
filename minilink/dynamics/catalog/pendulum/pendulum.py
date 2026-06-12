import numpy as np

from minilink.dynamics.abstraction.mechanical import MechanicalSystem
from minilink.graphical.animation.primitives import (
    Circle,
    Rod,
    TorqueArrow,
    pose2d_matrix,
    torque_pose2d_matrix,
)


class Pendulum(MechanicalSystem):
    """Single actuated pendulum."""

    def __init__(self):
        super().__init__(dof=1, actuators=1)
        self.name = "Pendulum"
        self.params = {
            "m": 1.0,
            "l": 1.0,
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
        self.camera_scale = 1.5

    def H(self, q, params=None):
        params = self.params if params is None else params
        m = params["m"]
        l = params["l"]
        I = params["I"]

        # rotational inertia of the bob about the pivot
        return np.array([[m * l**2 + I]])

    def C(self, q, dq, params=None):
        return np.zeros((1, 1))

    def g(self, q, params=None):
        params = self.params if params is None else params
        m = params["m"]
        l = params["l"]
        gravity = params["gravity"]

        # gravity restoring torque
        return np.array([m * gravity * l * np.sin(q[0])])

    def d(self, q, dq, u=None, t=0.0, params=None):
        params = self.params if params is None else params
        d = params["d"]
        return np.array([d * dq[0]])

    def get_kinematic_geometry(self):
        length = self.params["l"]
        radius = 0.08 * length
        return [
            Circle(radius=radius, center=[0.0, 0.0], color="blue", fill=True),
            Rod(length=length, radius=0.03 * length, color="blue", linewidth=2),
            Circle(radius=radius, center=[0.0, -length], color="blue", fill=True),
            TorqueArrow(radius=length / 5.0, head_ratio=0.4, color="red", linewidth=2),
        ]

    def get_kinematic_transforms(self, x, u, t):
        theta = x[0]
        torque = u[0]
        rod_angle = theta - np.pi / 2.0
        max_torque = self.inputs["u"].upper_bound[0]
        sweep = torque * (2.0 * np.pi / 3.0) / max_torque
        return [
            pose2d_matrix(0.0, 0.0, 0.0),
            pose2d_matrix(0.0, 0.0, theta),
            pose2d_matrix(0.0, 0.0, theta),
            torque_pose2d_matrix(0.0, 0.0, rod_angle, sweep),
        ]


class InvertedPendulum(Pendulum):
    """Single actuated pendulum with upward zero-angle equilibrium."""

    def __init__(self):
        super().__init__()
        self.name = "Inverted Pendulum"

    def g(self, q, params=None):
        return -super().g(q, params)

    def get_kinematic_transforms(self, x, u, t):
        # theta is measured from the upward vertical (zero-angle = upright); the
        # shared geometry draws angles from the downward vertical, so add pi.
        upright = np.array([x[0] + np.pi, x[1]])
        return super().get_kinematic_transforms(upright, u, t)


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
        return np.diag([inertia, inertia])

    def C(self, q, dq, params=None):
        return np.zeros((2, 2))

    def g(self, q, params=None):
        params = self.params if params is None else params
        m = params["m"]
        l = params["l"]
        gravity = params["gravity"]
        return m * gravity * l * np.sin(q)

    def d(self, q, dq, u=None, t=0.0, params=None):
        params = self.params if params is None else params
        d = params["d"]
        return d * np.asarray(dq)

    def get_kinematic_geometry(self):
        length = self.params["l"]
        radius = 0.08 * length
        torque_radius = length / 5.0
        return [
            Circle(radius=radius, center=[0.0, 0.0], color="blue", fill=True),
            Rod(length=length, radius=0.03 * length, color="blue", linewidth=2),
            Circle(radius=radius, center=[0.0, -length], color="blue", fill=True),
            TorqueArrow(
                radius=torque_radius,
                head_ratio=0.4,
                color="red",
                linewidth=2,
            ),
            Circle(radius=radius, center=[0.0, 0.0], color="blue", fill=True),
            Rod(length=length, radius=0.03 * length, color="blue", linewidth=2),
            Circle(radius=radius, center=[0.0, -length], color="blue", fill=True),
            TorqueArrow(
                radius=torque_radius,
                head_ratio=0.4,
                color="red",
                linewidth=2,
            ),
        ]

    def get_kinematic_transforms(self, x, u, t):
        length = self.params["l"]
        anchors = [-0.6 * length, 0.6 * length]
        max_torque = self.inputs["u"].upper_bound[0]
        sweep_scale = 2.0 * np.pi / 3.0 / max_torque
        transforms = []
        for i, anchor_x in enumerate(anchors):
            theta = x[i]
            rod_angle = theta - np.pi / 2.0
            transforms.extend(
                [
                    pose2d_matrix(anchor_x, 0.0, 0.0),
                    pose2d_matrix(anchor_x, 0.0, theta),
                    pose2d_matrix(anchor_x, 0.0, theta),
                    torque_pose2d_matrix(
                        anchor_x,
                        0.0,
                        rod_angle,
                        u[i] * sweep_scale,
                    ),
                ]
            )
        return transforms


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
