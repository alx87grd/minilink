"""Find equilibria (trim points) of a system by root-finding on ``f``.

An equilibrium is a state where the dynamics vanish for a held input:
``f(x_eq, u, t) = 0``. :func:`find_equilibrium` solves this with
``scipy.optimize.fsolve`` from a user guess, the usual first step before
linearizing and designing a controller about that point.
"""

import numpy as np
from scipy.optimize import fsolve


def find_equilibrium(sys, x_guess, u=None, *, t=0.0, params=None, tol=1e-9):
    """Return a state ``x_eq`` near ``x_guess`` with ``f(x_eq, u, t) ≈ 0``.

    Parameters
    ----------
    sys : System
        System whose ``f`` defines the dynamics.
    x_guess : array of shape (n,)
        Initial guess for the equilibrium state.
    u : array of shape (m,), optional
        Held input. Defaults to the system's nominal port values.
    t : float, optional
        Time at which to evaluate ``f``.
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
    if u is None:
        u = sys.get_u_from_input_ports()
    u = np.asarray(u, dtype=float).reshape(-1)

    def residual(x):
        return np.asarray(sys.f(x, u, t, params), dtype=float).reshape(-1)

    x_eq = fsolve(residual, x_guess, xtol=tol)

    if np.linalg.norm(residual(x_eq)) > tol * 1e3:
        raise RuntimeError("find_equilibrium did not converge to f(x) = 0")
    return x_eq


if __name__ == "__main__":
    from minilink.dynamics.catalog.pendulum.pendulum import Pendulum

    pendulum = Pendulum()
    # With zero torque, the upright start relaxes to the hanging equilibrium.
    x_eq = find_equilibrium(pendulum, x_guess=[0.3, 0.0])
    print("equilibrium:", np.round(x_eq, 6))
