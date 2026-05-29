import numpy as np

from minilink.core.system import DynamicSystem


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
        super().__init__(n=2)

        self.params = {"m": 1.0}
        self.name = "FloatingMass1D"

        self.state.labels = ["x", "dx"]
        self.state.units = ["m", "m/s"]

        self.add_input_port("F", nominal_value=0.0, labels=["Force"], units=["N"])

        self.add_output_port("y", dim=1, function=self.h, dependencies=())

    def f(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        m = params["m"]
        F = self.get_port_values_from_u(u, "F")[0]
        return np.array([x[1], F / m])

    def h(self, x, u, t=0, params=None):
        return np.array([x[0]])

    # Graphics
    def get_kinematic_geometry(self):
        from minilink.graphical.animation.primitives import Arrow, Circle

        return [
            Circle(radius=0.3, center=[0, 0], color="black", fill=True),
            Arrow(color="red", linewidth=2, origin="base"),
        ]

    def get_kinematic_transforms(self, x, u, t):
        from minilink.graphical.animation.primitives import (
            scale_pose2d_matrix,
            translation_matrix,
        )

        pos = x[0]
        F = self.get_port_values_from_u(u, "F")[0]

        force_scale = 0.3
        arrow_len = F * force_scale
        theta = 0.0 if arrow_len >= 0 else np.pi
        arrow_len = abs(arrow_len)

        return [
            translation_matrix(dx=pos, dy=0.0),
            scale_pose2d_matrix(x=pos, y=0.0, theta=theta, scale=arrow_len),
        ]
