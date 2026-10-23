"""Transfer-function blocks: a SISO ``TransferFunction`` in state space, and the ``Lead`` compensator."""

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

###############################################################################
# Public API
###############################################################################


class TransferFunction(ErrorDriven, LTISystem):
    """Continuous-time SISO transfer function in state-space realization.

    The default is a plant: input ``u``, outputs ``y`` and ``x``;
    ``expose_state=False`` keeps ``u -> y`` only (a filter is not a plant).
    Compensator layouts match the other classical blocks: ``ports="error"``
    is ``e -> u`` (``C @ plant`` inserts the Error block; ``C >> plant`` is the
    loop gain); ``ports="reference"`` is ``r, y -> u``.
    """

    def __init__(
        self,
        numerator,
        denominator,
        *,
        ports=None,
        name=None,
        expose_state=True,
    ):
        require_tf_ports(ports)
        compensator = ports in ("error", "reference")

        # The coefficients of G(s) = num(s) / den(s), highest power first
        self.numerator = np.asarray(numerator, dtype=float)
        self.denominator = np.asarray(denominator, dtype=float)

        # The realization ẋ = A x + B u, y = C x + D u of G(s); a plant takes
        # LTISystem's ports u -> y, x, a compensator declares its own below
        A, B, C, D = signal.tf2ss(self.numerator, self.denominator)
        super().__init__(
            A,
            B,
            C,
            D,
            name=name,
            declare_ports=not compensator,
            expose_state=expose_state,
        )

        # The poles and zeros of G(s)
        G = signal.TransferFunction(self.numerator, self.denominator)
        self.poles = G.poles
        self.zeros = G.zeros

        # Ports: a compensator reads its error layout and outputs u, fed through
        # from the error only when D ≠ 0
        if compensator:
            feedthrough = bool(np.any(D))
            self.port_layout = self.add_error_ports(ports, 1)
            self.add_output_port(
                "u",
                dim=1,
                function=self.h,
                dependencies=self.error_dependencies if feedthrough else (),
            )
        else:
            # A plant: error() is the identity, so f and h stay u -> y
            self.port_layout = "error"

    def f(self, x, u, t=0, params=None):
        # ẋ = A x + B e, the error e being u itself for a plant
        return super().f(x, self.error(u), t, params)

    def h(self, x, u, t=0, params=None):
        # y = C x + D e
        return super().h(x, self.error(u), t, params)

    def __str__(self):
        num = polynomial_text(self.numerator)
        den = polynomial_text(self.denominator)

        return f"{inspect_text(self)}\n  G(s) = {num} / {den}"

    def get_kinematic_geometry(self):
        return {
            "world": [ground_line(length=12.0)],
            "body": [Circle(radius=0.1, color="blue", fill=True)],
        }

    def tf(self, x, u, t=0, params=None):
        y = self.h(x, u, t)

        # The body slides along the ground by the output
        output = float(np.asarray(y).reshape(-1)[0])

        return {
            "body": translation(output, 0.0, 0.0),
            "force": translation(output, 0.0, 0.0),
        }

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        e = self.error(u)

        # The arrow draws the block's input: the error, u itself for a plant
        input_value = float(np.asarray(e).reshape(-1)[0])

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
        require_lead_corners(z, p)

        # C(s) = (K s + K z) / (s + p)
        super().__init__([K, K * z], [1.0, p], ports=ports)


###############################################################################
# Internal machinery
###############################################################################


def require_tf_ports(ports):
    """Refuse a port layout other than a plant's (``None``, ``"plant"``) or a compensator's."""
    if ports not in (None, "plant", "error", "reference"):
        raise ValueError(
            f"ports must be None, 'plant', 'error', or 'reference', got {ports!r}"
        )


_SUPERSCRIPT = str.maketrans("0123456789", "⁰¹²³⁴⁵⁶⁷⁸⁹")


def polynomial_text(coefficients):
    """``(s² + 3 s + 2)`` from the coefficients, highest power first; one term needs no brackets."""
    c = np.trim_zeros(np.atleast_1d(np.asarray(coefficients, dtype=float)), "f")
    degree = c.size - 1

    # One signed term per nonzero coefficient; a unit coefficient before s is implied
    terms = []
    for k, a in enumerate(c):
        power = degree - k
        if a == 0.0:
            continue

        coefficient = "" if abs(a) == 1.0 and power > 0 else f"{abs(a):.4g}"
        superscript = str(power).translate(_SUPERSCRIPT) if power > 1 else ""
        variable = "" if power == 0 else "s" + superscript
        sign = "-" if a < 0 else "+"
        term = " ".join(part for part in (coefficient, variable) if part)
        terms.append((sign, term))

    if not terms:
        return "0"

    # The leading sign is written only when negative, the others between the terms
    text = ("-" if terms[0][0] == "-" else "") + terms[0][1]
    for sign, term in terms[1:]:
        text += f" {sign} {term}"

    return f"({text})" if len(terms) > 1 else text


def require_lead_corners(z, p):
    """Refuse a lead compensator without ``0 < z < p``."""
    if not 0.0 < z < p:
        raise ValueError(f"a lead compensator has 0 < z < p, got z={z}, p={p}")


if __name__ == "__main__":
    sys = TransferFunction([1.0], [1.0, 1.0])

    sys.x0 = np.array([2.0])
    sys.compute_trajectory(tf=5.0)
    sys.plot_trajectory()
    sys.animate()
