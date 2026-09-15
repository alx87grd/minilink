"""Equilibria of :class:`~minilink.core.system.DynamicSystem` models."""

from __future__ import annotations

import numpy as np
from scipy.optimize import fsolve


def find_equilibrium(sys, x_guess, u_bar=None, t=0.0, params=None, *, tol=1e-9):
    """Return a state ``x_eq`` near ``x_guess`` with ``f(x_eq, u_bar, t) ≈ 0``.

    Parameters
    ----------
    sys : DynamicSystem
        Continuous-time model whose ``f`` defines the dynamics (leaf subclass
        or wired diagram with stacked ``f``).
    x_guess : array of shape (n,)
        Initial guess for the equilibrium state.
    u_bar : array of shape (m,), optional
        Held input. Defaults to the system's nominal port values.
    t : float, optional
        Time at which to evaluate ``f``.
    params : dict, optional
        Parameter set forwarded to ``f``.
    tol : float, optional
        Tolerance on ``‖f‖`` for the success check.

    Returns
    -------
    x_eq : np.ndarray
        The equilibrium state.

    Raises
    ------
    RuntimeError
        If the solver does not drive ``‖f‖`` below ``tol``.
    """
    x_guess = np.asarray(x_guess, dtype=float).reshape(-1)
    if u_bar is None:
        u_bar = sys.get_u_from_input_ports()
    u_bar = np.asarray(u_bar, dtype=float).reshape(-1)

    def residual(x):
        return np.asarray(sys.f(x, u_bar, t, params), dtype=float).reshape(-1)

    x_eq = fsolve(residual, x_guess, xtol=tol)

    if np.linalg.norm(residual(x_eq)) > tol * 1e3:
        raise RuntimeError("find_equilibrium did not converge to f(x) = 0")
    return x_eq


if __name__ == "__main__":
    from minilink.dynamics.catalog.pendulum.pendulum import Pendulum

    pendulum = Pendulum()
    # With zero torque, the upright start relaxes to the hanging equilibrium.
    x_eq = find_equilibrium(pendulum, x_guess=[0.3, 0.0])
    print("equilibrium:", x_eq)
