import numpy as np
from scipy import signal

from minilink.dynamics.abstraction.state_space import LTISystem
from minilink.core.kinematics import identity_matrix, translation_matrix
from minilink.graphical.animation.legacy import legacy_arrow_vector
from minilink.graphical.animation.primitives import Circle, ground_line


class TransferFunction(LTISystem):
    """Continuous-time SISO transfer function in state-space realization."""

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
        return {
            "world": [
                ground_line(length=12.0),
            ],
            "output": [
                Circle(radius=0.1, color="blue", fill=True),
            ],
        }

    def tf(self, x, u, t=0, params=None):
        output = np.asarray(self.h(x, u, t)).reshape(-1)[0]
        return {
            "world": identity_matrix(output),
            "output": translation_matrix(output, 0.0, 0.0),
        }

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        output = float(np.asarray(self.h(x, u, t)).reshape(-1)[0])
        input_value = float(np.asarray(u).reshape(-1)[0])
        return {
            "world": [
                legacy_arrow_vector(
                    output,
                    0.0,
                    input_value,
                    0.0,
                    scale=0.35,
                    color="red",
                    linewidth=2,
                )
            ],
        }


if __name__ == "__main__":
    sys = TransferFunction([1.0], [1.0, 1.0])

    sys.x0 = np.array([2.0])
    sys.compute_trajectory(tf=5.0)
    sys.plot_trajectory()
    sys.animate()
