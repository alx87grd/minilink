import numpy as np

from minilink.core.system import DynamicSystem
from minilink.graphical.primitives import (
    Circle,
    Rod,
    TorqueArrow,
    pose2d_matrix,
    torque_pose2d_matrix,
)


class Pendulum(DynamicSystem):
    """Simple pendulum with torque, disturbance, and sensor-noise ports."""

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

        # Default camera target and plot axes
        self.camera_target = np.array([0, 0, 0.0])
        self.camera_plot_axes = (0, 1)
        self.camera_scale = 2.0

    def f(self, x, u, t=0, params=None):
        params = self.params if params is None else params

        g = params["g"]
        m = params["m"]
        length = params["l"]
        theta = x[0]

        input_signals = self.get_port_values_from_u(u)
        tau = input_signals["u"][0]
        w = input_signals["w"][0]

        dx = np.zeros(2)
        dx[0] = x[1]
        dx[1] = -g / length * np.sin(theta) + (tau + w) / (m * length**2)

        return dx

    def h(self, x, u, t=0, params=None):
        v = self.u2input_signal(u, "v")

        y = np.zeros(self.p)
        y[0] = x[0]
        y[1] = x[1] + v[0]

        return y

    # Graphics
    def get_kinematic_geometry(self):
        primitives = []
        length = self.params["l"]

        radius = 0.08 * length

        # Hinge, rod, bob: blue (Pyro-style); torque arc stays red
        primitives.append(Circle(radius=radius, center=[0, 0], color="blue", fill=True))
        # Rod: radius is meshcat cylinder girth; matplotlib uses linewidth for the line
        primitives.append(
            Rod(
                length=length,
                radius=0.03 * length,
                color="blue",
                linewidth=2,
            )
        )
        primitives.append(
            Circle(radius=radius, center=[0, -length], color="blue", fill=True)
        )
        primitives.append(
            TorqueArrow(radius=length / 5.0, head_ratio=0.4, color="red", linewidth=2)
        )

        return primitives

    def get_kinematic_transforms(self, x, u, t):
        theta = x[0]
        torque = self.u2input_signal(u, "u")[0]
        rod_angle = theta - np.pi / 2
        max_torque = self.inputs["u"].upper_bound[0]
        sweep = torque * (2 * np.pi / 3) / max_torque

        return [
            pose2d_matrix(0, 0, 0),
            pose2d_matrix(0, 0, theta),
            pose2d_matrix(0, 0, theta),
            torque_pose2d_matrix(0, 0, rod_angle, sweep),
        ]


if __name__ == "__main__":

    pendulum = Pendulum()
    pendulum.plot_graphe()
    pendulum.plot_phase_plane()

    pendulum.x0[0] = np.pi / 2
    pendulum.compute_trajectory(tf=10.0, show=True)
    pendulum.plot_phase_plane(pendulum.traj)
    pendulum.animate()
