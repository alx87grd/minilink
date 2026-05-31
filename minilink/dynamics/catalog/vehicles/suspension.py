import numpy as np

from minilink.core.system import DynamicSystem
from minilink.dynamics.catalog._graphics import (Arrow, Box, CustomLine,
                                                 arrow_transform,
                                                 follow_xy_camera,
                                                 identity_matrix,
                                                 line_between_transform,
                                                 spring_line,
                                                 translation_matrix)


class QuarterCarOnRoughTerrain(DynamicSystem):
    """Quarter-car suspension moving over prescribed rough terrain.

    TRL: 1 - ready for user review.
    """

    def __init__(self):
        super().__init__(n=3, input_dim=1, output_dim=3, expose_state=True)
        self.name = "Quarter Car On Rough Terrain"
        self.state.labels = ["dy", "y", "x"]
        self.state.units = ["m/s", "m", "m"]
        self.inputs["u"].labels = ["force"]
        self.inputs["u"].units = ["N"]
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)
        self.mass = 1.0
        self.b = 1.0
        self.k = 1.0
        self.vx = 1.0
        self.a = np.array([0.5, 0.3, 0.7, 0.2, 0.2, 0.1])
        self.w = np.array([0.2, 0.4, 0.5, 1.0, 2.0, 3.0])
        self.phi = np.array([3.0, 2.0, 0.0, 0.0, 0.0, 0.0])
        self.camera_scale = 10.0

    def z(self, x):
        return np.sum(self.a * np.sin(self.w * (x - self.phi)))

    def dz(self, x):
        return np.sum(self.a * self.w * np.cos(self.w * (x - self.phi)))

    def f(self, x, u, t=0.0, params=None):
        ground = self.z(x[2])
        ground_slope = self.dz(x[2])
        acceleration = (
            u[0] - self.k * (x[1] - ground) - self.b * (x[0] - ground_slope)
        ) / self.mass
        return np.array([acceleration, x[0], self.vx])

    def h(self, x, u, t=0.0, params=None):
        return x

    def get_camera_transform(self, x, u, t):
        return follow_xy_camera(x[2], x[1], self.camera_scale)

    def get_kinematic_geometry(self):
        xs = np.linspace(-5.0, 15.0, 240)
        terrain = np.column_stack([xs, [self.z(x) for x in xs], np.zeros_like(xs)])
        return [
            CustomLine(terrain, color="black", linewidth=1),
            spring_line(color="black"),
            Box(length_x=0.8, length_y=0.35, length_z=0.2, color="blue", opacity=0.9),
            Arrow(color="red", linewidth=2, origin="base"),
        ]

    def get_kinematic_transforms(self, x, u, t):
        ground = self.z(x[2])
        mass_y = x[1]
        return [
            identity_matrix(),
            line_between_transform([x[2], ground], [x[2], mass_y - 0.2]),
            translation_matrix(x[2], mass_y, 0.0),
            arrow_transform(x[2] + 0.5, mass_y, 0.0, u[0], scale=0.2),
        ]


if __name__ == "__main__":
    system = QuarterCarOnRoughTerrain()
    system.compute_trajectory(tf=8.0, n_steps=240, show=True, verbose=False)
