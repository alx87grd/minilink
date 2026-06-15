"""Equilibrium linearization of any :class:`~minilink.core.system.System`.

``linearize`` returns the first-order model

    dΔx = A Δx + B Δu
    Δy  = C Δx + D Δu

about an operating point ``(x_bar, u_bar)``, where ``Δx = x - x_bar`` etc.
Jacobians use central finite differences (``method='fd'``) or exact values from
a compiled evaluator (``method='jax'``). The result is an
:class:`~minilink.dynamics.abstraction.state_space.LTISystem` ready for design
tools (``control.lqr``), structural analysis, and ``analysis.modal``.
"""

import numpy as np

from minilink.dynamics.abstraction.state_space import LTISystem


def linearize(
    sys,
    x_bar,
    u_bar=None,
    *,
    t=0.0,
    params=None,
    epsilon=1e-6,
    method="fd",
    compile_backend="jax",
):
    """Linearize ``sys`` about ``(x_bar, u_bar)`` and return an ``LTISystem``.

    Parameters
    ----------
    sys : System
        System with ``f`` (and ``h``) to linearize.
    x_bar : array of shape (n,)
        Operating-point state.
    u_bar : array of shape (m,), optional
        Operating-point input. Defaults to the system's nominal port values.
    t : float, optional
        Time at which to evaluate the (assumed time-invariant) Jacobians.
    params : dict, optional
        Parameter set forwarded to ``f`` / ``h`` when ``method='fd'``.
    epsilon : float, optional
        Central-difference step when ``method='fd'``.
    method : str, optional
        ``'fd'`` for central finite differences, ``'jax'`` for exact Jacobians
        from a compiled evaluator.
    compile_backend : str, optional
        Backend passed to :meth:`~minilink.core.system.System.compile` when
        ``method='jax'``.

    Returns
    -------
    LTISystem
        Constant-matrix model with ``Delta``-prefixed labels carried from ``sys``.
    """
    x_bar = np.asarray(x_bar, dtype=float).reshape(-1)
    if u_bar is None:
        u_bar = sys.get_u_from_input_ports()
    u_bar = np.asarray(u_bar, dtype=float).reshape(-1)

    key = str(method).strip().lower()
    if key == "fd":
        A, B, C, D = _linearize_fd(sys, x_bar, u_bar, t=t, params=params, epsilon=epsilon)
    elif key == "jax":
        A, B, C, D = _linearize_jax(
            sys, x_bar, u_bar, t=t, compile_backend=compile_backend
        )
    else:
        raise ValueError(f"linearize method must be 'fd' or 'jax', got {method!r}")

    lti = LTISystem(A, B, C, D, name=f"Linearized {sys.name}")
    lti.state.labels = [f"Delta {label}" for label in sys.state.labels]
    return lti


def _linearize_fd(sys, x_bar, u_bar, *, t, params, epsilon):
    def f(x, u):
        return np.asarray(sys.f(x, u, t, params), dtype=float).reshape(-1)

    A = _jacobian(lambda x: f(x, u_bar), x_bar, epsilon)
    B = _jacobian(lambda u: f(x_bar, u), u_bar, epsilon)

    if sys.p > 0:

        def h(x, u):
            return np.asarray(sys.h(x, u, t, params), dtype=float).reshape(-1)

        C = _jacobian(lambda x: h(x, u_bar), x_bar, epsilon)
        D = _jacobian(lambda u: h(x_bar, u), u_bar, epsilon)
    else:
        C = np.eye(x_bar.size)
        D = np.zeros((x_bar.size, u_bar.size))
    return A, B, C, D


def _linearize_jax(sys, x_bar, u_bar, *, t, compile_backend):
    evaluator = sys.compile(backend=compile_backend)
    A, B, C, D = evaluator.linearize(x_bar, u_bar, t=t)
    return (
        np.asarray(A, dtype=float),
        np.asarray(B, dtype=float),
        np.asarray(C, dtype=float),
        np.asarray(D, dtype=float),
    )


def _jacobian(func, x0, epsilon):
    """Central-difference Jacobian of ``func`` at ``x0`` (shape ``(out, in)``)."""
    x0 = np.asarray(x0, dtype=float)
    n = x0.size
    out = np.asarray(func(x0)).reshape(-1).size

    jac = np.zeros((out, n))
    for j in range(n):
        step = np.zeros(n)
        step[j] = epsilon
        jac[:, j] = (func(x0 + step) - func(x0 - step)) / (2.0 * epsilon)
    return jac


if __name__ == "__main__":
    from minilink.dynamics.catalog.pendulum.pendulum import Pendulum

    pendulum = Pendulum()
    # Linearize about the downward rest; A should show the pendulum mode.
    lti = linearize(pendulum, x_bar=[0.0, 0.0])
    print("A =\n", np.round(lti.A(), 4))
    print("B =\n", np.round(lti.B(), 4))
    print("open-loop poles:", np.round(np.linalg.eigvals(lti.A()), 4))
