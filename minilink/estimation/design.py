"""State-observer design: a model or arrays in, a ``StateObserver`` out, the duals of the state-feedback designs."""

import re

import numpy as np

from minilink.analysis.linearization import design_model
from minilink.analysis.riccati import care
from minilink.analysis.structural import observability_matrix, rank
from minilink.control.design import place_gain
from minilink.estimation.observer import StateObserver

###############################################################################
# Public API
###############################################################################


def luenberger_gain(A, C, poles):
    """Return the observer gain ``L`` that puts the eigenvalues of ``A − L C`` at ``poles``."""
    A = np.asarray(A, dtype=float)
    C = np.atleast_2d(np.asarray(C, dtype=float))

    # 𝒪 = [C; CA; …; CAⁿ⁻¹]: every error pole can be moved only when rank 𝒪 = n
    obsv = observability_matrix(A, C)
    require_observable(obsv, A.shape[0])

    # eig(A − L C) = eig(Aᵀ − Cᵀ Lᵀ): the placement for the pair (Aᵀ, Cᵀ), transposed
    L = place_gain(A.T, C.T, poles).T

    return L


def kalman_gain(A, C, Q_n, R_n):
    """Return the Kalman gain ``L`` of the stationary filter for process noise ``Q_n`` and measurement noise ``R_n``."""
    A = np.asarray(A, dtype=float)
    C = np.atleast_2d(np.asarray(C, dtype=float))
    Q_n = np.asarray(Q_n, dtype=float)
    R_n = np.atleast_2d(np.asarray(R_n, dtype=float))

    # A P + P Aᵀ − P Cᵀ R_n⁻¹ C P + Q_n = 0: the control equation on the dual pair (Aᵀ, Cᵀ)
    P = care(A.T, C.T, Q_n, R_n)

    # L = P Cᵀ R_n⁻¹   (P and R_n symmetric: the transpose of R_n⁻¹ C P)
    L = np.linalg.solve(R_n, C @ P).T

    return L


def luenberger(
    sys, poles, *, x_bar=None, u_bar=None, t=0.0, params=None, method="auto", eps=1e-6
):
    """Design an observer for ``sys`` by pole placement and return a ``StateObserver``.

    ``sys`` is a linear model, used as is with the operating point it carries,
    or any plant, linearized first about its command port at ``(x_bar, u_bar)``,
    the nominal state and inputs when omitted. The block runs the model about
    that point and corrects it with ``L (y − ŷ)``, so the estimation error has
    the requested poles. Arguments as in :func:`~minilink.control.design.lqr`,
    with ``poles`` in place of ``Q, R``. On arrays the gain is
    :func:`luenberger_gain`, and the model ``LTISystem(A, B, C)``.

    The observer reads the plant's output ``y``: a plant without one is
    refused (make the sensor part of the plant box, ``plant >> Gain(...)``),
    and a design on another output is made on ``linearize(plant, x_bar,
    of=...)``.
    """
    model = design_model(
        sys, x_bar, u_bar, t, params, method=method, eps=eps, measured=True
    )
    A = model.A()
    C = model.C()

    L = luenberger_gain(A, C, poles)

    return build_observer(model, L, "Luenberger Observer")


def kalman(
    sys,
    Q_n,
    R_n,
    *,
    x_bar=None,
    u_bar=None,
    t=0.0,
    params=None,
    method="auto",
    eps=1e-6,
):
    """Design a stationary Kalman filter for ``sys`` and return a ``StateObserver``.

    ``sys`` is a linear model or any plant measured through ``y``, as in
    :func:`luenberger`. On arrays the gain is :func:`kalman_gain`. ``Q_n`` is
    the intensity of the white process noise on the state, ``n × n`` (a noise
    ``w`` entering through ``B_w`` has ``Q_n = B_w W B_wᵀ``), and ``R_n`` the
    intensity of the white measurement noise, ``p × p``. The gain minimizes the
    steady-state estimation error covariance, and the block is the stationary
    Kalman–Bucy filter.
    """
    model = design_model(
        sys, x_bar, u_bar, t, params, method=method, eps=eps, measured=True
    )
    A = model.A()
    C = model.C()

    L = kalman_gain(A, C, Q_n, R_n)

    return build_observer(model, L, "Kalman Filter")


###############################################################################
# Internal machinery
###############################################################################


def require_observable(obsv, n):
    """Refuse an unobservable pair, naming the rank of the observability matrix."""
    r = rank(obsv)
    if r < n:
        raise ValueError(
            f"luenberger: (A, C) is not observable (rank 𝒪 = {r} < n = {n}): only "
            "the observable modes' error poles can be moved."
        )


def build_observer(model, L, name):
    """The ``StateObserver`` called ``name`` running ``model`` with gain ``L`` at the model's operating point, its labels with a hat."""
    labels = [hat(label) for label in model.state.labels]

    observer = StateObserver(
        model.A(),
        model.B(),
        model.C(),
        L,
        model.D(),
        xbar=model.xbar,
        ubar=model.ubar,
        ybar=model.ybar,
        labels=labels,
    )
    observer.name = name

    return observer


def hat(label):
    """``θ`` -> ``θ̂``, ``x[0]`` -> ``x̂[0]``, ``theta`` -> ``theta_hat``; a linearized model's ``Delta θ`` reads as its plant's label."""
    label = label.removeprefix("Delta ")
    component = re.fullmatch(r"(.)\[(\d+)\]", label)
    if component:
        return f"{component[1]}̂[{component[2]}]"
    return f"{label}̂" if len(label) == 1 else f"{label}_hat"


if __name__ == "__main__":
    from minilink.dynamics.catalog.pendulum.pendulum import Pendulum

    plant = Pendulum()
    observer = kalman(plant, Q_n=np.diag([0.01, 0.1]), R_n=1e-3 * np.eye(2))
    print(observer)
