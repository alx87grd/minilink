"""The state observer: a copy of the linear model, corrected by the output it mispredicts."""

import re

import numpy as np

from minilink.analysis.linearization import design_point
from minilink.core.discrete.step_system import StepSystem
from minilink.core.system import DynamicSystem, fastest_time_constant
from minilink.dynamics.abstraction.state_space import ConstantMatrices

###############################################################################
# Public API
###############################################################################


class StateObserver(DynamicSystem):
    """Linear state observer ``dx̂ = A (x̂ − x̄) + B (u − ū) + L (y − ŷ)``, the estimate on ``x``.

    The observer runs ``model``, the plant's linear model ``(A, B, C, D)``
    about its operating point ``(x̄, ū, ȳ)``, and corrects it with the
    innovation, the measured ``y`` minus the predicted ``ŷ = ȳ + C (x̂ − x̄) +
    D (u − ū)``, through the gain ``L``. ``model`` is a
    :class:`~minilink.dynamics.abstraction.state_space.LinearizedSystem`, its
    point read from it (``ȳ = h(x̄, ū)``), or an
    :class:`~minilink.dynamics.abstraction.state_space.LTISystem`, about the
    origin: the matrix form is ``StateObserver(LTISystem(A, B, C, D), L)``.
    The observer is continuous: a step model (a ``DiscreteLTISystem``, a
    ``LinearizedStepSystem``) is refused; design on the continuous plant,
    then put the observer on a clock (``observer % dt``).
    The estimation error then follows ``A − L C``, so the gain sets its poles:
    :func:`~minilink.estimation.design.luenberger` places them and
    :func:`~minilink.estimation.design.kalman` makes them optimal for white noise.
    Both return this block.

    Ports: the plant's command ``u`` (dimension ``m``) and measurement ``y``
    (dimension ``p``) in; the estimate ``x`` (dimension ``n``, the model's
    state labels with a hat) out. The initial estimate ``x0`` starts at x̄.
    The matrices, the gain and the operating point live in ``params``.
    A box holding this block under the key ``observer`` and a state-feedback
    gain, with ``r``, ``y`` -> ``u`` on its edge, is a controller for ``@``.
    """

    def __init__(self, model, L):
        # The model (A, B, C, D) and its operating point (x̄, ū, ȳ), the origin
        # of a linear model; the gain L
        require_linear_model(model)
        require_continuous_observer(model)
        A = np.asarray(model.A(), dtype=float)
        B = np.atleast_2d(np.asarray(model.B(), dtype=float))
        C = np.atleast_2d(np.asarray(model.C(), dtype=float))
        D = np.atleast_2d(np.asarray(model.D(), dtype=float))
        L = np.atleast_2d(np.asarray(L, dtype=float))
        n, m = B.shape
        p = C.shape[0]
        check_dimensions(A, B, C, D, L)
        x_bar, u_bar, y_bar = (
            np.asarray(component, dtype=float).reshape(-1)
            for component in design_point(model)
        )
        labels = [hat(label) for label in model.state.labels]

        super().__init__(n)
        self.id = "observer"

        # Parameters
        self.params = {
            "A": A,
            "B": B,
            "C": C,
            "D": D,
            "L": L,
            "x_bar": x_bar,
            "u_bar": u_bar,
            "y_bar": y_bar,
        }

        # Signal labels: the model's state, each entry with a hat
        self.state.labels = labels

        # Initial estimate: the operating point, where the model holds
        self.state.nominal_value = np.array(x_bar, dtype=float).reshape(-1)
        self.x0 = x_bar.copy()

        # Ports: the command u and the measurement y in, the estimate x̂ out
        self.add_input_port("u", dim=m, nominal_value=u_bar)
        self.add_input_port("y", dim=p, nominal_value=y_bar)
        self.add_output_port(
            "x",
            dim=n,
            function=self.estimate,
            dependencies=(),
            labels=self.state.labels,
        )

    @property
    def poles(self):
        """Eigenvalues of ``A − L C``: the poles of the estimation error."""
        A = self.params["A"]
        C = self.params["C"]
        L = self.params["L"]

        poles = np.linalg.eigvals(A - L @ C)

        return poles

    @property
    def solver_info(self) -> dict:
        """The solver hints, the smallest time constant ``1 / max |λᵢ(A − L C)|`` read from the current ``params``."""
        info = self.own_solver_info
        info["smallest_time_constant"] = fastest_time_constant(self.poles)

        return info

    @solver_info.setter
    def solver_info(self, value: dict):
        self.own_solver_info = dict(value)

    def f(self, x, u, t=0, params=None):
        params = self.params if params is None else params

        A = params["A"]
        B = params["B"]
        C = params["C"]
        D = params["D"]
        L = params["L"]
        x_bar = params["x_bar"]
        u_bar = params["u_bar"]
        y_bar = params["y_bar"]

        m = B.shape[1]
        x_hat = x
        u_in = u[:m]
        y = u[m:]

        # The predicted output ŷ, then the model corrected by the innovation y − ŷ
        y_hat = y_bar + C @ (x_hat - x_bar) + D @ (u_in - u_bar)
        dx_hat = A @ (x_hat - x_bar) + B @ (u_in - u_bar) + L @ (y - y_hat)

        return dx_hat

    def estimate(self, x, u, t=0, params=None):
        """The estimate ``x̂``: the observer's state."""
        return x

    def __str__(self):
        prefix = "  L = "
        L = np.array2string(
            self.params["L"], precision=4, suppress_small=True, prefix=prefix
        )
        poles = ", ".join(f"{pole:.3g}" for pole in np.round(self.poles, 4))

        return f"{super().__str__()}\n{prefix}{L}\n  poles of A − L C: {poles}"


###############################################################################
# Internal machinery
###############################################################################


def require_linear_model(model):
    """Refuse anything but a linear model, one that stores its matrices: the observer runs them."""
    if not isinstance(model, ConstantMatrices):
        raise TypeError(
            f"StateObserver(model, L) runs a linear model, got {type(model).__name__}: "
            "pass LTISystem(A, B, C, D) or linearize(plant, x_bar), or design on the "
            "plant with luenberger(plant, poles) or kalman(plant, Q_n, R_n)"
        )


def require_continuous_observer(model):
    """Refuse a step model: the observer runs its matrices as a continuous ``dx̂``."""
    if isinstance(model, StepSystem):
        raise TypeError(
            f"{model.name!r} is a step model: a StateObserver is continuous, "
            "dx̂ = A (x̂ − x̄) + B (u − ū) + L (y − ŷ); design on the continuous plant, "
            "then put the observer on a clock (observer % dt). On a step model, "
            "luenberger_gain(A, C, poles) gives the gain, its poles in z"
        )


def hat(label):
    """``θ`` -> ``θ̂``, ``x[0]`` -> ``x̂[0]``, ``theta`` -> ``theta_hat``."""
    component = re.fullmatch(r"(.)\[(\d+)\]", label)
    if component:
        return f"{component[1]}̂[{component[2]}]"
    return f"{label}̂" if len(label) == 1 else f"{label}_hat"


def check_dimensions(A, B, C, D, L):
    """Refuse matrices whose shapes do not describe one plant and one gain."""
    n, m = B.shape
    p = C.shape[0]
    if A.shape != (n, n):
        raise ValueError(f"A must be ({n}, {n}) to match B's {n} rows, got {A.shape}")
    if C.shape[1] != n:
        raise ValueError(f"C must have {n} columns, got {C.shape}")
    if D.shape != (p, m):
        raise ValueError(f"D must be ({p}, {m}), got {D.shape}")
    if L.shape != (n, p):
        raise ValueError(
            f"L must be ({n}, {p}): one row per state, one column per output, got {L.shape}"
        )
