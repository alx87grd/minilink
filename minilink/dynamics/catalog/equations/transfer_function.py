import numpy as np
from scipy import signal

from minilink.dynamics.abstraction.state_space import StateSpaceSystem
from minilink.dynamics.catalog._graphics import (Arrow, Circle,
                                                 arrow_transform, ground_line,
                                                 pose2d_matrix)


class TransferFunction(StateSpaceSystem):
    """Continuous-time SISO transfer function in state-space realization.

    TRL: 1 - ready for user review.
    """

    def __init__(self, numerator, denominator, *, name="Transfer Function"):
        self.numerator = np.asarray(numerator, dtype=float)
        self.denominator = np.asarray(denominator, dtype=float)
        A, B, C, D = signal.tf2ss(self.numerator, self.denominator)
        super().__init__(A, B, C, D, name=name)
        self.inputs["u"].labels = ["u"]
        self.outputs["y"].labels = ["y"]
        self.poles = signal.TransferFunction(
            self.numerator, self.denominator
        ).poles
        self.zeros = signal.TransferFunction(
            self.numerator, self.denominator
        ).zeros

    def get_kinematic_geometry(self):
        return [
            ground_line(length=12.0),
            Circle(radius=0.1, color="blue", fill=True),
            Arrow(color="red", linewidth=2, origin="tail"),
        ]

    def get_kinematic_transforms(self, x, u, t):
        output = float(np.asarray(self.h(x, u, t)).reshape(-1)[0])
        input_value = float(np.asarray(u).reshape(-1)[0])
        return [
            pose2d_matrix(0.0, 0.0, 0.0),
            pose2d_matrix(output, 0.0, 0.0),
            arrow_transform(output, 0.0, input_value, 0.0, scale=0.35),
        ]


if __name__ == "__main__":
    system = TransferFunction([1.0], [1.0, 1.0])
    system.compute_forced(
        lambda t: np.array([1.0]),
        tf=5.0,
        n_steps=120,
        show=True,
        verbose=False,
    )
