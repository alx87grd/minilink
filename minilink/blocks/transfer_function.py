"""Transfer-function blocks: a SISO ``TransferFunction`` in state space, and ``Lead`` / ``Lag``."""

import numpy as np
from scipy import signal

from minilink.core.feedback import ErrorDriven
from minilink.core.inspect import inspect_text
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
        if ports not in (None, "plant", "error", "reference"):
            raise ValueError(
                f"ports must be None, 'plant', 'error', or 'reference', got {ports!r}"
            )
        compensator = ports in ("error", "reference")
        self.numerator = np.asarray(numerator, dtype=float)
        self.denominator = np.asarray(denominator, dtype=float)
        A, B, C, D = signal.tf2ss(self.numerator, self.denominator)
        # a plant takes LTISystem's ports u -> y, x; a compensator declares its own below
        super().__init__(A, B, C, D, name=name, declare_ports=not compensator)
        tf = signal.TransferFunction(self.numerator, self.denominator)
        self.poles = tf.poles
        self.zeros = tf.zeros

        if compensator:
            feedthrough = bool(np.any(D))
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
        else:
            # plant: ``error()`` is the identity so ``f`` / ``h`` stay ``u -> y``
            self.port_layout = "error"

    def __str__(self):
        num = polynomial_text(self.numerator)
        den = polynomial_text(self.denominator)
        return f"{inspect_text(self)}\n  G(s) = {num} / {den}"

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


# =============================================================================
# Internal machinery
# =============================================================================

_SUPERSCRIPT = str.maketrans("0123456789", "⁰¹²³⁴⁵⁶⁷⁸⁹")


def polynomial_text(coefficients):
    """``(s² + 3 s + 2)`` from the coefficients, highest power first; one term needs no brackets."""
    c = np.trim_zeros(np.atleast_1d(np.asarray(coefficients, dtype=float)), "f")
    degree = c.size - 1
    terms = []
    for k, a in enumerate(c):
        power = degree - k
        if a == 0.0:
            continue
        coefficient = "" if abs(a) == 1.0 and power > 0 else f"{abs(a):.4g}"
        variable = (
            ""
            if power == 0
            else "s" + (str(power).translate(_SUPERSCRIPT) if power > 1 else "")
        )
        terms.append(
            (
                "-" if a < 0 else "+",
                " ".join(part for part in (coefficient, variable) if part),
            )
        )
    if not terms:
        return "0"
    text = ("-" if terms[0][0] == "-" else "") + terms[0][1]
    for sign, term in terms[1:]:
        text += f" {sign} {term}"
    return f"({text})" if len(terms) > 1 else text


if __name__ == "__main__":
    sys = TransferFunction([1.0], [1.0, 1.0])

    sys.x0 = np.array([2.0])
    sys.compute_trajectory(tf=5.0)
    sys.plot_trajectory()
    sys.animate()
