import numpy as np
from scipy import signal

from minilink.dynamics.abstraction.state_space import LTISystem
from minilink.graphical.animation.primitives import (
    Arrow,
    Circle,
    arrow_transform,
    ground_line,
    pose2d_matrix,
)


class TransferFunction(LTISystem):
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
        tf = signal.TransferFunction(self.numerator, self.denominator)
        self.poles = tf.poles
        self.zeros = tf.zeros

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
    sys = TransferFunction([1.0], [1.0, 1.0])

    sys.x0 = np.array([2.0])
    sys.compute_trajectory(tf=5.0)
    sys.plot_trajectory()
