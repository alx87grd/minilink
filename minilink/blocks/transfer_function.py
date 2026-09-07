import numpy as np
from scipy import signal

from minilink.core.feedback import ErrorDriven
from minilink.core.kinematics import translation
from minilink.dynamics.abstraction.state_space import LTISystem
from minilink.graphical.animation.primitives import (
    Arrow,
    Circle,
    ground_line,
)


class TransferFunction(ErrorDriven, LTISystem):
    """Continuous-time SISO transfer function in state-space realization.

    The default is a plant: input ``u``, outputs ``y`` and ``x``. Compensator
    layouts match the other classical blocks: ``ports="error"`` is ``e -> u``
    (``C @ plant`` inserts the Error block; ``C >> plant`` is the loop gain);
    ``ports="reference"`` is ``r, y -> u``.
    """

    def __init__(self, numerator, denominator, *, ports=None, name="Transfer Function"):
        self.numerator = np.asarray(numerator, dtype=float)
        self.denominator = np.asarray(denominator, dtype=float)
        A, B, C, D = signal.tf2ss(self.numerator, self.denominator)
        super().__init__(A, B, C, D, name=name)
        tf = signal.TransferFunction(self.numerator, self.denominator)
        self.poles = tf.poles
        self.zeros = tf.zeros

        if ports in ("error", "reference"):
            feedthrough = tuple(self.outputs["y"].dependencies)
            self.inputs = {}
            self.outputs = {}
            self.add_error_ports(ports, 1)
            self.add_output_port(
                "u",
                dim=1,
                function=self.h,
                dependencies=self.error_dependencies if feedthrough else (),
            )
            if ports == "reference":
                self.measurement_port, self.ref_port, self.control_port = "y", "r", "u"
                self.plot_space = "error"
        elif ports not in (None, "plant"):
            raise ValueError(
                f"ports must be None, 'plant', 'error', or 'reference', got {ports!r}"
            )
        else:
            # plant: ``error()`` is the identity so ``f`` / ``h`` stay ``u -> y``
            self.port_layout = "error"

    def f(self, x, u, t=0, params=None):
        return super().f(x, self.error(u), t, params)

    def h(self, x, u, t=0, params=None):
        return super().h(x, self.error(u), t, params)

    def get_kinematic_geometry(self):
        return {
            "world": [ground_line(length=12.0)],
            "body": [Circle(radius=0.1, color="blue", fill=True)],
        }

    def tf(self, x, u, t=0, params=None):
        output = float(np.asarray(self.h(x, u, t)).reshape(-1)[0])
        return {
            "body": translation(output, 0.0, 0.0),
            "force": translation(output, 0.0, 0.0),
        }

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        input_value = float(np.asarray(self.error(u)).reshape(-1)[0])
        return {
            "force": [
                Arrow(
                    base=(0.0, 0.0),
                    vector=(input_value, 0.0),
                    scale=0.35,
                    color="red",
                    linewidth=2,
                )
            ]
        }


if __name__ == "__main__":
    sys = TransferFunction([1.0], [1.0, 1.0])

    sys.x0 = np.array([2.0])
    sys.compute_trajectory(tf=5.0)
    sys.plot_trajectory()
    sys.animate()


class Lead(TransferFunction):
    """Lead compensator ``C(s) = K (s + z) / (s + p)`` with ``z < p``: phase lead between ``z`` and ``p``."""

    def __init__(self, K=1.0, z=1.0, p=10.0, *, ports="error"):
        if not 0.0 < z < p:
            raise ValueError(f"a lead compensator has 0 < z < p, got z={z}, p={p}")
        super().__init__([K, K * z], [1.0, p], ports=ports, name="Lead")


class Lag(TransferFunction):
    """Lag compensator ``C(s) = K (s + z) / (s + p)`` with ``p < z``: gain at low frequency."""

    def __init__(self, K=1.0, z=1.0, p=0.1, *, ports="error"):
        if not 0.0 < p < z:
            raise ValueError(f"a lag compensator has 0 < p < z, got z={z}, p={p}")
        super().__init__([K, K * z], [1.0, p], ports=ports, name="Lag")
