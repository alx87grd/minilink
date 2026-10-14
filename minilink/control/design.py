"""Linear state-feedback design: a model or arrays in, a controller block out."""

import numpy as np
from scipy.signal import place_poles

from minilink.analysis.derivatives import jacobian
from minilink.analysis.linearization import design_model
from minilink.analysis.riccati import (
    care,
    hamiltonian_matrix,
    riccati_map,
    riccati_step,
    riccati_transition,
    substeps,
)
from minilink.analysis.structural import controllability
from minilink.control.state import (
    StateFeedbackController,
    TimeVaryingStateFeedbackController,
    TrajectoryFeedbackController,
)
from minilink.core.diagram import DiagramSystem

# Public API


def lqr_gain(A, B, Q, R):
    """Return the optimal feedback gain ``K`` of the law ``u = -K x`` minimizing ``∫ xᵀQx + uᵀRu dt``."""
    A = np.asarray(A, dtype=float)
    B = np.atleast_2d(np.asarray(B, dtype=float))
    Q = np.asarray(Q, dtype=float)
    R = np.atleast_2d(np.asarray(R, dtype=float))

    # Aᵀ P + P A − P B R⁻¹ Bᵀ P + Q = 0
    P = care(A, B, Q, R)

    # K = R⁻¹ Bᵀ P
    K = np.linalg.solve(R, B.T @ P)

    return K


def place_gain(A, B, poles):
    """Return the gain ``K`` of the law ``u = -K x`` that puts the eigenvalues of ``A − B K`` at ``poles``."""
    A = np.asarray(A, dtype=float)
    B = np.atleast_2d(np.asarray(B, dtype=float))
    n, m = B.shape
    poles = requested_poles(poles, n)

    # 𝒞 = [B, AB, …, Aⁿ⁻¹B]: every pole can be moved only when rank 𝒞 = n
    ctrb = controllability(A, B)
    require_controllable(ctrb)

    if m == 1:
        # φ(s) = ∏ (s − pᵢ) = sⁿ + a₁ sⁿ⁻¹ + … + aₙ
        a = np.real(np.poly(poles))

        # φ(A) = Aⁿ + a₁ Aⁿ⁻¹ + … + aₙ I   (Horner)
        phi_A = np.zeros((n, n))
        for a_k in a:
            phi_A = phi_A @ A + a_k * np.eye(n)

        # Ackermann: K = [0 … 0 1] 𝒞⁻¹ φ(A)
        e_n = np.eye(n)[-1]
        K = np.linalg.solve(ctrb.matrix.T, e_n)[np.newaxis, :] @ phi_A
    else:
        # Several inputs: many K place these poles; take the robust one (Kautz–Nichols–Van Dooren)
        K = robust_gain(A, B, poles)

    require_placed(A, B, K, poles)

    return K


def lqr(
    sys, Q, R, *, x_bar=None, u_bar=None, t=0.0, params=None, method="auto", eps=1e-6
):
    """Design an LQR on ``sys`` and return a ``StateFeedbackController``.

    ``sys`` is a linear model (an ``LTISystem``, used as is with the operating
    point it carries) or any plant, linearized first about its command port at
    ``(x_bar, u_bar)``, the nominal state and inputs when omitted. The block
    implements ``u = ubar - K (x - r)`` with ``r`` defaulting to ``xbar``, the
    operating point of the model; wire the plant state into its ``x`` port. On
    arrays the gain is :func:`lqr_gain`, and the model ``LTISystem(A, B)``.

    Parameters
    ----------
    sys : System
        The linear model or the plant.
    Q, R : array-like
        State and input weights of ``∫ xᵀQx + uᵀRu dt``.
    x_bar, u_bar : array-like, optional
        Operating point to linearize a plant about, the nominal by default; a
        linear model carries its own.
    t, params, method, eps : optional
        Linearization time, parameter set, differentiation backend and
        central-difference step, as in :func:`~minilink.analysis.linearize`.
    """
    model = design_model(sys, x_bar, u_bar, t, params, method=method, eps=eps)
    A, B = model.A(), model.B()

    K = lqr_gain(A, B, Q, R)

    return StateFeedbackController(K, xbar=model.xbar, ubar=model.ubar)


def place(
    sys, poles, *, x_bar=None, u_bar=None, t=0.0, params=None, method="auto", eps=1e-6
):
    """Place the closed-loop poles of ``sys`` and return a ``StateFeedbackController``.

    ``sys`` is a linear model or any plant, linearized first about its command
    port at ``(x_bar, u_bar)``, as in :func:`lqr`, with ``poles`` (``n`` of
    them, complex ones in conjugate pairs) in place of ``Q, R``. The block
    implements ``u = ubar - K (x - r)`` with ``r`` defaulting to ``xbar``. On
    arrays the gain is :func:`place_gain`.
    """
    model = design_model(sys, x_bar, u_bar, t, params, method=method, eps=eps)
    A, B = model.A(), model.B()

    K = place_gain(A, B, poles)

    return StateFeedbackController(K, xbar=model.xbar, ubar=model.ubar)


def lqg(
    sys,
    Q,
    R,
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
    """Design an LQG compensator on ``sys``, an LQR gain on a Kalman estimate, and return it as one controller box.

    ``sys`` is a linear model or any plant, as in :func:`lqr`. ``Q, R`` are the LQR
    weights, ``Q_n, R_n`` the process and measurement noise intensities of
    :func:`~minilink.estimation.design.kalman`. The box holds the
    :class:`~minilink.estimation.observer.StateObserver` under ``observer`` and
    the :class:`~minilink.control.state.StateFeedbackController` under ``gain``,
    the estimate wired into the gain and the command back into the observer,
    with ``r``, ``y`` -> ``u`` on its edge: ``box @ plant`` closes the loop, and
    the closed-loop poles are the gain's and the observer's together (the
    separation principle).
    """
    from minilink.estimation.design import kalman  # the dual design, for this verb only

    model = design_model(sys, x_bar, u_bar, t, params, method=method, eps=eps)

    gain = lqr(model, Q, R)
    observer = kalman(model, Q_n, R_n)

    # the estimate feeds the gain, the command feeds the observer; r, y in and u out
    box = DiagramSystem()
    box.name = "LQG Controller"
    box.add_subsystem(observer, "observer")
    box.add_subsystem(gain, "gain")
    box.connect("observer", "x", "gain", "x")
    box.connect("gain", "u", "observer", "u")
    box.expose("r", "y", "u")

    return box


def lqr_gain_schedule(A, B, Q, R, S_f, tf, *, n_steps=1001):
    """Return ``(t, K, S)``: the finite-horizon gain ``K(t)`` of the law ``u = -K(t) x`` and the Riccati matrix ``S(t)``.

    ``S`` is swept backward from the terminal weight ``S(t_f) = S_f``, exactly on
    a uniform grid, through the Hamiltonian system of the co-state
    (:func:`~minilink.analysis.riccati.riccati_step`): no integration error, no
    stiffness, and ``S`` stays symmetric. ``t`` holds ``n_steps`` samples from
    ``0`` to ``tf``; ``K`` has shape ``(n_steps, m, n)`` and ``S`` shape
    ``(n_steps, n, n)``.
    """
    A = np.asarray(A, dtype=float)
    B = np.atleast_2d(np.asarray(B, dtype=float))
    Q = np.asarray(Q, dtype=float)
    R = np.atleast_2d(np.asarray(R, dtype=float))
    S_f = np.asarray(S_f, dtype=float)
    n = A.shape[0]
    n_steps = int(n_steps)
    if n_steps < 2 or tf <= 0.0:
        raise ValueError("lqr_gain_schedule needs tf > 0 and n_steps >= 2")
    R_inv = np.linalg.inv(R)
    t = np.linspace(0.0, float(tf), n_steps)
    dt = float(tf) / (n_steps - 1)

    # −dS/dt = S A + Aᵀ S − S B R⁻¹ Bᵀ S + Q, swept backward from S(t_f) = S_f; the
    # dynamics are constant, so one transition over a substep serves the whole sweep
    H = hamiltonian_matrix(A, B, Q, R_inv)
    n_sub = substeps(H, dt)
    E = riccati_transition(A, B, Q, R_inv, dt / n_sub)
    S = np.empty((n_steps, n, n))
    S[-1] = S_f
    for k in range(n_steps - 1, 0, -1):
        S_k = S[k]
        for _ in range(n_sub):
            S_k = riccati_map(E, S_k)
        S[k - 1] = S_k

    # K(t) = R⁻¹ Bᵀ S(t)
    K = R_inv @ B.T @ S

    return t, K, S


def lqr_finite_horizon(
    sys,
    Q,
    R,
    S_f,
    tf,
    *,
    n_steps=1001,
    after="hold",
    x_bar=None,
    u_bar=None,
    t=0.0,
    params=None,
    method="auto",
    eps=1e-6,
):
    """Design a finite-horizon LQR on ``sys`` and return a ``TimeVaryingStateFeedbackController``.

    ``sys`` is a linear model or any plant, as in :func:`lqr`. The block
    implements ``u = ubar - K(t) (x - r)`` with
    ``K(t)`` from :func:`lqr_gain_schedule`, minimizing
    ``∫₀^tf xᵀQx + uᵀRu dt + x(tf)ᵀ S_f x(tf)`` for the linear model. Past the
    horizon the block keeps ``K(tf)`` (``after="hold"``) or switches to the
    infinite-horizon gain of :func:`lqr_gain` (``after="stationary"``).
    """
    if after not in ("hold", "stationary"):
        raise ValueError("after must be 'hold' or 'stationary'")
    model = design_model(sys, x_bar, u_bar, t, params, method=method, eps=eps)
    A, B = model.A(), model.B()

    t_schedule, K, _ = lqr_gain_schedule(A, B, Q, R, S_f, tf, n_steps=n_steps)

    # past the horizon: K(t_f) held, or the stationary gain
    K_after = lqr_gain(A, B, Q, R) if after == "stationary" else None

    return TimeVaryingStateFeedbackController(
        t_schedule, K, xbar=model.xbar, ubar=model.ubar, K_after=K_after
    )


def trajectory_lqr(
    sys, trajectory, Q, R, S_f=None, *, params=None, method="auto", eps=1e-6
):
    """Design an LQR along a reference trajectory and return a ``TrajectoryFeedbackController``.

    ``sys`` is linearized at every sample ``(x_d(t_k), u_d(t_k))`` of
    ``trajectory``, and the Riccati differential equation of the error
    ``x - x_d`` is swept backward along the samples, the dynamics frozen on
    each interval and each interval exact, as in :func:`lqr_gain_schedule`.
    The gains give the law ``u = u_d(t) - K(t) (x - x_d(t))``. ``S_f``
    defaults to the algebraic Riccati solution at the final sample, so the
    gain held past the horizon is the stationary LQR of the end point.

    Parameters
    ----------
    sys : System
        Plant to stabilize.
    trajectory : Trajectory
        Reference ``(t, x_d, u_d)``, ``N`` samples at increasing times.
    Q, R : array-like
        LQR weights on the tracking error and on the input deviation.
    S_f : array-like, optional
        Terminal weight ``S(t_f)``; default the stationary solution at the end.
    params : dict, optional
        Parameter dict forwarded to ``sys.f``.
    method : {"auto", "fd", "jax"}, optional
        Differentiation backend, see :func:`~minilink.analysis.derivatives.jacobian`.
    eps : float, optional
        Central-difference step.
    """
    t = np.asarray(trajectory.t, dtype=float).reshape(-1)
    X = np.asarray(trajectory.x, dtype=float).T
    U = np.asarray(trajectory.u, dtype=float).T
    Q = np.asarray(Q, dtype=float)
    R = np.atleast_2d(np.asarray(R, dtype=float))
    N, n = X.shape
    m = U.shape[1]
    R_inv = np.linalg.inv(R)

    # A_k = ∂f/∂x and B_k = ∂f/∂u at every sample of the reference
    A = np.empty((N, n, n))
    B = np.empty((N, n, m))
    for k in range(N):
        A[k] = jacobian(sys, "f", "x", X[k], U[k], t[k], params, method=method, eps=eps)
        B[k] = jacobian(sys, "f", "u", X[k], U[k], t[k], params, method=method, eps=eps)

    # −dS/dt = S A_k + A_kᵀ S − S B_k R⁻¹ B_kᵀ S + Q, swept backward from S(t_f) = S_f
    # with the dynamics frozen on each interval
    S = np.empty((N, n, n))
    S[-1] = care(A[-1], B[-1], Q, R) if S_f is None else S_f
    for k in range(N - 1, 0, -1):
        S[k - 1] = riccati_step(A[k - 1], B[k - 1], Q, R_inv, S[k], t[k] - t[k - 1])

    # K_k = R⁻¹ B_kᵀ S_k
    K = R_inv @ np.swapaxes(B, 1, 2) @ S

    return TrajectoryFeedbackController(trajectory, K)


# Internal machinery: the request checked, the result checked


def requested_poles(poles, n):
    """The requested poles as a complex array: exactly ``n``, complex ones in conjugate pairs."""
    poles = np.atleast_1d(np.asarray(poles, dtype=complex))
    if poles.shape != (n,):
        raise ValueError(f"place: {n} poles needed for {n} states, got {poles.size}.")
    if not np.allclose(np.sort_complex(poles), np.sort_complex(poles.conj())):
        raise ValueError(
            "place: complex poles must come in conjugate pairs (a real gain K "
            f"gives a real characteristic polynomial), got {poles}."
        )
    return poles


def require_controllable(ctrb):
    """Refuse an uncontrollable pair, naming the rank of the controllability matrix."""
    if ctrb.rank < ctrb.n:
        raise ValueError(
            f"place: (A, B) is not controllable (rank 𝒞 = {ctrb.rank} < n = "
            f"{ctrb.n}): only the controllable modes can be moved."
        )


def robust_gain(A, B, poles):
    """``scipy.signal.place_poles``, its multiplicity limit said in plain words."""
    try:
        return place_poles(A, B, poles).gain_matrix
    except ValueError as error:
        raise ValueError(
            f"place: with {B.shape[1]} inputs a pole can be repeated at most "
            f"{np.linalg.matrix_rank(B)} times; spread the repeated poles ({error})."
        ) from error


def require_placed(A, B, K, poles):
    """Check ``det(sI − (A − B K)) = ∏ (s − pᵢ)``; a badly conditioned design fails loudly.

    The characteristic polynomials are compared, not the eigenvalues: a repeated
    pole is computed only to about the cube root of machine precision, while its
    polynomial stays exact.
    """
    placed = np.real(np.poly(A - B @ K))
    requested = np.real(np.poly(poles))
    if not np.allclose(
        placed, requested, rtol=1e-6, atol=1e-9 * np.max(np.abs(requested))
    ):
        raise ValueError(
            f"place: the closed loop landed at {np.linalg.eigvals(A - B @ K)}, not at "
            f"the requested {poles}; the design is badly conditioned."
        )


if __name__ == "__main__":
    from minilink.dynamics.catalog.pendulum.cartpole import CartPole

    plant = CartPole()
    x_bar = np.array([0.0, np.pi, 0.0, 0.0])  # pole inverted, cart at origin

    controller = lqr(
        plant, Q=np.diag([1.0, 10.0, 1.0, 1.0]), R=np.array([[0.1]]), x_bar=x_bar
    )
    print("LQR gain K =\n", np.round(controller.params["K"], 4))

    diagram = controller @ plant
    plant.x0 = np.array([-1.0, np.pi + 0.3, 0.0, 0.0])
    diagram.compute_trajectory(tf=8.0)
    diagram.plot_trajectory()
    diagram.animate()
