import numpy as np

from minilink.core.framework import DynamicSystem, StaticSystem
from minilink.graphical.primitives import (
    Circle,
    CustomLine,
    TorqueArrow,
    pose2d_matrix,
    torque_pose2d_matrix,
)


######################################################################
class Pendulum(DynamicSystem):
    def __init__(self):
        super().__init__(2, 1, 2)

        self.params = {"g": 9.81, "m": 1.0, "l": 1.0}

        self.name = "Pendulum"

        self.state.labels = ["theta", "theta_dot"]
        self.state.units = ["rad", "rad/s"]

        self.inputs = {}
        self.add_input_port(1, "u", nominal_value=np.array([0.0]))
        self.add_input_port(1, "w", nominal_value=np.array([0.0]))
        self.add_input_port(1, "v", nominal_value=np.array([0.0]))
        self.inputs["u"].labels = ["torque"]
        self.inputs["u"].units = ["Nm"]
        self.inputs["w"].labels = ["disturbance"]
        self.inputs["w"].units = ["Nm"]
        self.inputs["v"].labels = ["noise"]
        self.inputs["v"].units = ["rad"]

        self.outputs = {}
        # The output is the state with noise added to angular position
        self.add_output_port(self.p, "y", function=self.h, dependencies=["v"])

    ######################################################################
    def f(self, x, u, t=0, params=None):

        if params is None:
            params = self.params

        g = params["g"]
        m = params["m"]
        length = params["l"]

        theta = x[0]

        input_signals = self.get_port_values_from_u(u)
        u = input_signals["u"][0]
        w = input_signals["w"][0]

        dx = np.zeros(2)
        dx[0] = x[1]
        dx[1] = -g / length * np.sin(theta) + 1 / (m * length**2) * (u + w)

        return dx

    ######################################################################
    def h(self, x, u, t=0, params=None):

        # input_signals = self.get_port_values_from_u(u)
        # v = input_signals["v"]

        v = self.u2input_signal(u, "v")

        y = np.zeros(self.p)

        y[0] = x[0]
        y[1] = x[1] + v[0]

        return y

    ######################################################################
    def get_kinematic_geometry(self):
        primitives = []
        length = self.params["l"]

        radius = 0.1 * length

        # Hinge (black circle)
        primitives.append(
            Circle(radius=radius, center=[0, 0], color="black", fill=True)
        )
        # Rod (blue line)
        primitives.append(
            CustomLine(pts=[[0, 0], [0, -length]], color="black", linewidth=4)
        )
        # Bob (red circle)
        primitives.append(
            Circle(radius=radius, center=[0, -length], color="black", fill=True)
        )
        # Torque arc arrow at the hinge (radius = 1/5 of rod length, like pyro)
        primitives.append(
            TorqueArrow(radius=length / 5.0, head_ratio=0.4, color="red", linewidth=2)
        )

        return primitives

    ######################################################################
    def get_kinematic_transforms(self, x, u, t):
        transforms = []
        theta = x[0]
        # Hinge is static
        transforms.append(pose2d_matrix(0, 0, 0))
        # Rod rotates around hinge by theta
        transforms.append(pose2d_matrix(0, 0, theta))
        # Bob rotates around hinge by theta
        transforms.append(pose2d_matrix(0, 0, theta))

        # Torque arrow: arc at hinge, starting on the rod
        torque = self.u2input_signal(u, "u")[0]
        rod_angle = theta - np.pi / 2
        max_torque = 10.0
        sweep = torque * (2 * np.pi / 3) / max_torque
        transforms.append(torque_pose2d_matrix(0, 0, rod_angle, sweep))

        return transforms


######################################################################
class PendulumPDController(StaticSystem):
    def __init__(self):
        super().__init__(3, 1)

        self.params = {
            "Kp": 10.0,
            "Kd": 1.0,
        }

        self.name = "Controller"

        self.inputs = {}
        self.add_input_port(1, "ref", nominal_value=np.array([0.0]))
        self.add_input_port(2, "y", nominal_value=np.array([0.0, 0.0]))
        self.inputs["y"].labels = ["theta", "theta_dot"]
        self.inputs["y"].units = ["rad", "rad/s"]

        self.outputs = {}
        self.add_output_port(1, "u", function=self.ctl, dependencies=["ref", "y"])
        self.outputs["u"].labels = ["torque"]
        self.outputs["u"].units = ["Nm"]

    ######################################################################
    def ctl(self, x, u, t=0, params=None):

        if params is None:
            params = self.params

        Kp = params["Kp"]
        Kd = params["Kd"]

        ref = u[0]
        theta = u[1]
        theta_dot = u[2]

        torque = Kp * (ref - theta) - Kd * theta_dot

        u = np.array([torque])

        return u


######################################################################
class FloatingMass1D(DynamicSystem):
    """Point mass sliding along the x-axis: ``m * ddx = F``.

    States: ``[x, dx]``  (position, velocity)
    Input:  ``[F]``       (horizontal force)
    Output: ``[x]``       (position)

    The kinematic geometry consists of:
    - a filled circle (the mass),
    - an :class:`Arrow` primitive whose displayed length is proportional
      to the applied force  ``F`` via ``scale_pose2d_matrix``.
    """

    def __init__(self):
        super().__init__(n=2, m=1, p=1)

        self.params = {"m": 1.0}
        self.name = "FloatingMass1D"

        self.state.labels = ["x", "dx"]
        self.state.units = ["m", "m/s"]

        self.inputs = {}
        self.add_input_port(1, "F", nominal_value=np.array([0.0]))
        self.inputs["F"].labels = ["Force"]
        self.inputs["F"].units = ["N"]

        self.outputs = {}
        self.add_output_port(1, "y", function=self.h, dependencies=[])

    def f(self, x, u, t=0, params=None):
        params = params or self.params
        m = params["m"]
        F = self.u2input_signal(u, "F")[0]
        return np.array([x[1], F / m])

    def h(self, x, u, t=0, params=None):
        return np.array([x[0]])

    # -- Graphics ----------------------------------------------------------

    def get_kinematic_geometry(self):
        from minilink.graphical.primitives import Arrow, Circle

        return [
            Circle(radius=0.3, center=[0, 0], color="black", fill=True),
            Arrow(color="red", linewidth=2, origin="base"),
        ]

    def get_kinematic_transforms(self, x, u, t):
        from minilink.graphical.primitives import (
            scale_pose2d_matrix,
            translation_matrix,
        )

        pos = x[0]
        F = self.u2input_signal(u, "F")[0]

        force_scale = 0.3
        arrow_len = F * force_scale
        theta = 0.0 if arrow_len >= 0 else np.pi
        arrow_len = abs(arrow_len)

        return [
            translation_matrix(dx=pos, dy=0.0),
            scale_pose2d_matrix(x=pos, y=0.0, theta=theta, scale=arrow_len),
        ]


if __name__ == "__main__":
    pendulum = Pendulum()
    controller = PendulumPDController()

    pendulum.params["l"] = 5.0

    pendulum.x0[0] = np.pi / 4
    pendulum.compute_trajectory()
    pendulum.animate()
