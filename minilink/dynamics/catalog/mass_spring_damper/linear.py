import numpy as np

from minilink.dynamics.abstraction.state_space import StateSpaceSystem
from minilink.graphical.animation.primitives import (
    Arrow,
    Box,
    empty_transform,
    ground_line,
    identity_matrix,
    line_between_transform,
    scale_pose2d_matrix,
    spring_line,
    translation_matrix,
)


def _mass_box(size=0.5, color="blue", opacity=0.9):
    return Box(
        length_x=size,
        length_y=size,
        length_z=0.2 * size,
        color=color,
        opacity=opacity,
    )


def _mass_output_matrix(count, output_mass):
    if output_mass < 1 or output_mass > count:
        output_mass = count
    C = np.zeros((1, 2 * count))
    C[0, output_mass - 1] = 1.0
    return C, f"x{output_mass}"


def _force_arrow_transform(x, force):
    if abs(force) < 1e-12:
        return scale_pose2d_matrix(x, 0.0, 0.0, 0.0)
    theta = 0.0 if force >= 0.0 else np.pi
    return scale_pose2d_matrix(x, 0.0, theta, 0.3 * abs(force))


class SingleMass(StateSpaceSystem):
    """Single linear mass-spring-damper model.

    TRL: 1 - ready for user review.
    """

    def __init__(self, mass=1.0, k=2.0, b=0.0):
        self.mass = float(mass)
        self.k = float(k)
        self.b = float(b)
        A, B, C, D = self._abcd()
        super().__init__(A, B, C, D, name="Single Mass Spring Damper")
        self._set_metadata()
        self.camera_scale = 4.0

    def _abcd(self):
        return (
            np.array([[0.0, 1.0], [-self.k / self.mass, -self.b / self.mass]]),
            np.array([[0.0], [1.0 / self.mass]]),
            np.array([[1.0, 0.0]]),
            np.array([[0.0]]),
        )

    def _set_metadata(self):
        self.state.labels = ["x", "dx"]
        self.state.units = ["m", "m/s"]
        self.inputs["u"].labels = ["force"]
        self.inputs["u"].units = ["N"]
        self.outputs["y"].labels = ["x"]
        self.outputs["y"].units = ["m"]

    def refresh(self):
        self.A, self.B, self.C, self.D = self._abcd()

    def get_kinematic_geometry(self):
        return [
            ground_line(length=8.0),
            spring_line(),
            _mass_box(size=0.6, color="blue"),
            Arrow(color="red", linewidth=2, origin="base"),
        ]

    def get_kinematic_transforms(self, x, u, t):
        mass_x = x[0]
        anchor = -2.0
        left_face = mass_x - 0.3
        spring = (
            line_between_transform([anchor, 0.0], [left_face, 0.0])
            if self.k != 0.0
            else empty_transform()
        )
        return [
            identity_matrix(),
            spring,
            translation_matrix(mass_x, 0.0, 0.0),
            _force_arrow_transform(mass_x + 0.35, u[0]),
        ]


class TwoMass(StateSpaceSystem):
    """Two-mass linear spring-damper chain with force on the second mass.

    TRL: 1 - ready for user review.
    """

    def __init__(self, m=1.0, k=2.0, b=0.2, output_mass=2):
        self.m1 = self.m2 = float(m)
        self.k1 = self.k2 = float(k)
        self.b1 = self.b2 = float(b)
        self.output_mass = int(output_mass)
        A, B, C, D, output_label = self._abcd()
        super().__init__(A, B, C, D, name="Two Mass Spring Damper")
        self._set_metadata(output_label)
        self.camera_scale = 5.0

    def _abcd(self):
        A = np.array(
            [
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
                [
                    -(self.k1 + self.k2) / self.m1,
                    self.k2 / self.m1,
                    -self.b1 / self.m1,
                    0.0,
                ],
                [
                    self.k2 / self.m2,
                    -self.k2 / self.m2,
                    0.0,
                    -self.b2 / self.m2,
                ],
            ]
        )
        B = np.array([[0.0], [0.0], [0.0], [1.0 / self.m2]])
        C, output_label = _mass_output_matrix(2, self.output_mass)
        return A, B, C, np.array([[0.0]]), output_label

    def _set_metadata(self, output_label):
        self.state.labels = ["x1", "x2", "dx1", "dx2"]
        self.state.units = ["m", "m", "m/s", "m/s"]
        self.inputs["u"].labels = ["force"]
        self.inputs["u"].units = ["N"]
        self.outputs["y"].labels = [output_label]
        self.outputs["y"].units = ["m"]

    def refresh(self):
        self.A, self.B, self.C, self.D, output_label = self._abcd()
        self.outputs["y"].labels = [output_label]

    def get_kinematic_geometry(self):
        return [
            ground_line(length=10.0),
            spring_line(),
            spring_line(),
            _mass_box(size=0.55, color="green"),
            _mass_box(size=0.55, color="blue"),
            Arrow(color="red", linewidth=2, origin="base"),
        ]

    def _mass_centers(self, x):
        return np.array([x[0] - 2.0, x[1]])

    def get_kinematic_transforms(self, x, u, t):
        x1, x2 = self._mass_centers(x)
        anchor = -4.0
        spring1 = (
            line_between_transform([anchor, 0.0], [x1 - 0.3, 0.0])
            if self.k1 != 0.0
            else empty_transform()
        )
        return [
            identity_matrix(),
            spring1,
            line_between_transform([x1 + 0.3, 0.0], [x2 - 0.3, 0.0]),
            translation_matrix(x1, 0.0, 0.0),
            translation_matrix(x2, 0.0, 0.0),
            _force_arrow_transform(x2 + 0.35, u[0]),
        ]


class ThreeMass(StateSpaceSystem):
    """Three-mass linear spring-damper chain with force on the third mass.

    TRL: 1 - ready for user review.
    """

    def __init__(self, m=1.0, k=2.0, b=0.2, output_mass=2):
        self.m1 = self.m2 = self.m3 = float(m)
        self.k1 = self.k2 = self.k3 = float(k)
        self.b1 = self.b2 = self.b3 = float(b)
        self.output_mass = int(output_mass)
        A, B, C, D, output_label = self._abcd()
        super().__init__(A, B, C, D, name="Three Mass Spring Damper")
        self._set_metadata(output_label)
        self.camera_scale = 6.0

    def _abcd(self):
        A = np.array(
            [
                [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
                [
                    -(self.k1 + self.k2) / self.m1,
                    self.k2 / self.m1,
                    0.0,
                    -self.b1 / self.m1,
                    0.0,
                    0.0,
                ],
                [
                    self.k2 / self.m2,
                    -(self.k2 + self.k3) / self.m2,
                    self.k3 / self.m2,
                    0.0,
                    -self.b2 / self.m2,
                    0.0,
                ],
                [
                    0.0,
                    self.k3 / self.m3,
                    -self.k3 / self.m3,
                    0.0,
                    0.0,
                    -self.b3 / self.m3,
                ],
            ]
        )
        B = np.array([[0.0], [0.0], [0.0], [0.0], [0.0], [1.0 / self.m3]])
        C, output_label = _mass_output_matrix(3, self.output_mass)
        return A, B, C, np.array([[0.0]]), output_label

    def _set_metadata(self, output_label):
        self.state.labels = ["x1", "x2", "x3", "dx1", "dx2", "dx3"]
        self.state.units = ["m", "m", "m", "m/s", "m/s", "m/s"]
        self.inputs["u"].labels = ["force"]
        self.inputs["u"].units = ["N"]
        self.outputs["y"].labels = [output_label]
        self.outputs["y"].units = ["m"]

    def refresh(self):
        self.A, self.B, self.C, self.D, output_label = self._abcd()
        self.outputs["y"].labels = [output_label]

    def get_kinematic_geometry(self):
        return [
            ground_line(length=12.0),
            spring_line(),
            spring_line(),
            spring_line(),
            _mass_box(size=0.5, color="magenta"),
            _mass_box(size=0.5, color="green"),
            _mass_box(size=0.5, color="blue"),
            Arrow(color="red", linewidth=2, origin="base"),
        ]

    def _mass_centers(self, x):
        return np.array([x[0] - 2.0, x[1], x[2] + 2.0])

    def get_kinematic_transforms(self, x, u, t):
        x1, x2, x3 = self._mass_centers(x)
        anchor = -4.0
        spring1 = (
            line_between_transform([anchor, 0.0], [x1 - 0.28, 0.0])
            if self.k1 != 0.0
            else empty_transform()
        )
        return [
            identity_matrix(),
            spring1,
            line_between_transform([x1 + 0.28, 0.0], [x2 - 0.28, 0.0]),
            line_between_transform([x2 + 0.28, 0.0], [x3 - 0.28, 0.0]),
            translation_matrix(x1, 0.0, 0.0),
            translation_matrix(x2, 0.0, 0.0),
            translation_matrix(x3, 0.0, 0.0),
            _force_arrow_transform(x3 + 0.32, u[0]),
        ]


class FloatingSingleMass(SingleMass):
    """Single mass with no ground spring.

    TRL: 1 - ready for user review.
    """

    def __init__(self, m=1.0, b=0.0):
        super().__init__(mass=m, k=0.0, b=b)
        self.name = "Floating Single Mass"


class FloatingTwoMass(TwoMass):
    """Two masses with no ground spring on the first mass.

    TRL: 1 - ready for user review.
    """

    def __init__(self, m=1.0, k=1.0, b=0.0, output_mass=2):
        super().__init__(m=m, k=k, b=b, output_mass=output_mass)
        self.name = "Floating Two Mass"
        self.k1 = 0.0
        self.refresh()


class FloatingThreeMass(ThreeMass):
    """Three masses with no ground spring on the first mass.

    TRL: 1 - ready for user review.
    """

    def __init__(self, m=1.0, k=1.0, b=0.0, output_mass=3):
        super().__init__(m=m, k=k, b=b, output_mass=output_mass)
        self.name = "Floating Three Mass"
        self.k1 = 0.0
        self.refresh()


if __name__ == "__main__":
    # sys = SingleMass()
    # sys = TwoMass()
    # sys = FloatingThreeMass()
    sys = ThreeMass()
    sys.inputs["u"].nominal_value = np.array([1.0])
    sys.compute_trajectory(tf=15.0)
    sys.animate(time_factor_video=5.0)
