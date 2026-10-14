"""The state observer: a copy of the linear model, corrected by the output it mispredicts."""

import numpy as np

from minilink.core.system import DynamicSystem


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
    hat) out. The initial estimate is ``x0``. The matrices, the gain and the
    operating point live in ``params``; ``ȳ`` defaults to ``C x̄ + D ū``, the
    linear model's own output, and the plant-taking designs pass ``h(x̄, ū)``.
    A box holding this block under the key ``observer`` and a state-feedback
    gain, with ``r``, ``y`` -> ``u`` on its edge, is a controller for ``@``.
    """

    def __init__(
        self, A, B, C, L, D=None, *, xbar=None, ubar=None, ybar=None, labels=None
    ):
        A = np.asarray(A, dtype=float)
        B = np.atleast_2d(np.asarray(B, dtype=float))
        C = np.atleast_2d(np.asarray(C, dtype=float))
        L = np.atleast_2d(np.asarray(L, dtype=float))
        n, m = B.shape
        p = C.shape[0]
        D = np.zeros((p, m)) if D is None else np.atleast_2d(np.asarray(D, dtype=float))
        check_dimensions(A, B, C, D, L)
        xbar = (
            np.zeros(n) if xbar is None else np.asarray(xbar, dtype=float).reshape(-1)
        )
        ubar = (
            np.zeros(m) if ubar is None else np.asarray(ubar, dtype=float).reshape(-1)
        )
        ybar = (
            C @ xbar + D @ ubar
            if ybar is None
            else np.asarray(ybar, dtype=float).reshape(-1)
        )

        super().__init__(n)
        self.name = "State Observer"
        self.id = "observer"
        self.params = {
            "A": A,
            "B": B,
            "C": C,
            "D": D,
            "L": L,
            "xbar": xbar,
            "ubar": ubar,
            "ybar": ybar,
        }
        self.state.labels = (
            [f"x̂[{i}]" for i in range(n)] if labels is None else list(labels)
        )

        self.add_input_port("u", dim=m, nominal_value=ubar)
        self.add_input_port("y", dim=p, nominal_value=ybar)
        self.add_output_port(
            "x",
            dim=n,
            function=self.estimate,
            dependencies=(),
            labels=self.state.labels,
        )

    def f(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        A, B, C, D, L = params["A"], params["B"], params["C"], params["D"], params["L"]
        xbar, ubar, ybar = params["xbar"], params["ubar"], params["ybar"]
        m = B.shape[1]
        x_hat = x
        u_in, y = u[:m], u[m:]

        # dx̂ = A (x̂ − x̄) + B (u − ū) + L (y − ŷ),   ŷ = ȳ + C (x̂ − x̄) + D (u − ū)
        e_x, e_u = x_hat - xbar, u_in - ubar
        y_hat = ybar + C @ e_x + D @ e_u
        dx_hat = A @ e_x + B @ e_u + L @ (y - y_hat)

        return dx_hat

    def estimate(self, x, u, t=0, params=None):
        """The estimate ``x̂``: the observer's state."""
        return x

    @property
    def poles(self):
        """Eigenvalues of ``A − L C``: the poles of the estimation error."""
        A, C, L = self.params["A"], self.params["C"], self.params["L"]
        return np.linalg.eigvals(A - L @ C)

    def __str__(self):
        prefix = "  L = "
        L = np.array2string(
            self.params["L"], precision=4, suppress_small=True, prefix=prefix
        )
        poles = ", ".join(f"{pole:.3g}" for pole in np.round(self.poles, 4))
        return f"{super().__str__()}\n{prefix}{L}\n  poles of A − L C: {poles}"


# Internal machinery


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
