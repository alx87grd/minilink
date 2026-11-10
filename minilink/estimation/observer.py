"""The state observer: a copy of the linear model, corrected by the output it mispredicts."""

import numpy as np

from minilink.core.system import DynamicSystem, fastest_time_constant
from minilink.dynamics.abstraction.state_space import point_component

###############################################################################
# Public API
###############################################################################


class StateObserver(DynamicSystem):
    """Linear state observer ``dx̂ = A (x̂ − x̄) + B (u − ū) + L (y − ŷ)``, the estimate on ``x``.

    The observer runs the model ``(A, B, C, D)`` of the plant about the operating
    point ``(x̄, ū, ȳ)`` and corrects it with the innovation, the measured ``y``
    minus the predicted ``ŷ = ȳ + C (x̂ − x̄) + D (u − ū)``, through the gain ``L``.
    The estimation error then follows ``A − L C``, so the gain sets its poles:
    :func:`~minilink.estimation.design.luenberger` places them and
    :func:`~minilink.estimation.design.kalman` makes them optimal for white noise.
    Both return this block.

    Ports: the plant's command ``u`` (dimension ``m``) and measurement ``y``
    (dimension ``p``) in; the estimate ``x`` (dimension ``n``, labelled with a
    hat) out. The initial estimate ``x0`` starts at x̄. The matrices, the gain and the
    operating point live in ``params``; ``ȳ`` defaults to ``C x̄ + D ū``, the
    linear model's own output, and the plant-taking designs pass ``h(x̄, ū)``.
    A box holding this block under the key ``observer`` and a state-feedback
    gain, with ``r``, ``y`` -> ``u`` on its edge, is a controller for ``@``.
    """

    def __init__(
        self, A, B, C, L, D=None, *, x_bar=None, u_bar=None, y_bar=None, labels=None
    ):
        # The model (A, B, C, D), the gain L and the operating point (x̄, ū, ȳ),
        # with ȳ = C x̄ + D ū by default: the linear model's own output
        A = np.asarray(A, dtype=float)
        B = np.atleast_2d(np.asarray(B, dtype=float))
        C = np.atleast_2d(np.asarray(C, dtype=float))
        L = np.atleast_2d(np.asarray(L, dtype=float))
        n, m = B.shape
        p = C.shape[0]
        D = np.zeros((p, m)) if D is None else np.atleast_2d(np.asarray(D, dtype=float))
        check_dimensions(A, B, C, D, L)
        x_bar = point_component(x_bar, n, "x_bar")
        u_bar = point_component(u_bar, m, "u_bar")
        y_bar = (
            C @ x_bar + D @ u_bar
            if y_bar is None
            else point_component(y_bar, p, "y_bar")
        )

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

        # Signal labels: the estimate x̂, each entry with a hat
        self.state.labels = (
            [f"x̂[{i}]" for i in range(n)] if labels is None else list(labels)
        )

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
