"""Adaptive SciPy :func:`scipy.integrate.solve_ivp` on the simulation time grid."""

import numpy as np
from scipy.integrate import solve_ivp

from minilink.simulation.input_interpolation import INPUT_INTERP_KEY, build_u_at_t
from minilink.simulation.solvers.solver import SolverBackend

###############################################################################
# Public API
###############################################################################


class SciPySolverBackend(SolverBackend):
    """Adaptive ``solve_ivp``, its solution reported on the ``times`` grid."""

    def __init__(self) -> None:
        self.last_debug = None
        self.last_solve_ivp_solution = None

    def integrate(
        self,
        evaluator,
        times: np.ndarray,
        x0: np.ndarray,
        args=None,
        params=None,
    ) -> np.ndarray:
        t0 = times[0]
        tf = times[-1]
        f = evaluator.f
        u_nominal = evaluator.nominal_input

        def rhs(t, x):
            return np.asarray(f(x, u_nominal, t, params))

        # Solver options: the analytic Jacobian only from a JAX evaluator
        kw = dict(args or {})
        use_jac = kw.pop("use_jac", False)
        method = kw.get("method", "RK45")
        if use_jac and evaluator.backend == "jax":
            kw["jac"] = nominal_jacobian_of(evaluator, params)
            jac_applied = True
        else:
            jac_applied = False

        # x(t) = x₀ + ∫ f(x(s), ū, s) ds, solved adaptively and reported on the grid
        sol = solve_ivp(rhs, [t0, tf], x0, t_eval=times, **kw)

        # The run's record; a failed solve raises
        return self.finalize_solution(
            sol,
            mode="nominal",
            method=method,
            extra_debug={"jac_applied": jac_applied},
        )

    def integrate_forced(
        self,
        evaluator,
        times: np.ndarray,
        u: np.ndarray,
        x0: np.ndarray,
        args=None,
        params=None,
    ) -> np.ndarray:
        # Solver options: the input scheme is ours, not solve_ivp's, and a forced
        # run takes no analytic Jacobian
        kw = dict(args or {})
        scheme = kw.pop(INPUT_INTERP_KEY, "linear")
        kw.pop("use_jac", None)
        method = kw.get("method", "RK45")

        t0 = times[0]
        tf = times[-1]

        # u(t) read between the samples by the scheme
        u_at_t = build_u_at_t(times, u, scheme)
        f = evaluator.f

        def rhs(t, x):
            return np.asarray(f(x, u_at_t(t), t, params))

        # x(t) = x₀ + ∫ f(x(s), u(s), s) ds, solved adaptively and reported on the grid
        sol = solve_ivp(rhs, [t0, tf], x0, t_eval=times, **kw)

        # The run's record; a failed solve raises
        return self.finalize_solution(
            sol,
            mode="forced",
            method=method,
            extra_debug={"interp": scheme},
        )

    ###########################################################################
    # Internal machinery
    ###########################################################################

    def finalize_solution(self, sol, *, mode, method, extra_debug=None):
        """Keep ``sol`` and its debug record, raise on a failed solve, return ``sol.y``."""
        self.last_solve_ivp_solution = sol

        # The run's record, kept even when the solve failed
        debug = {
            "solver": "scipy",
            "mode": mode,
            "method": method,
            "success": sol.success,
            "status": sol.status,
            "message": sol.message,
            "nfev": sol.nfev,
            "njev": sol.njev,
            "nlu": sol.nlu,
            "n_t": len(sol.t),
        }
        if extra_debug:
            debug.update(extra_debug)
        self.last_debug = debug

        require_success(sol, mode, method)

        return sol.y


###############################################################################
# Internal machinery
###############################################################################


def require_success(sol, mode, method):
    """Refuse a failed solve, naming the mode, the method and SciPy's message."""
    if not sol.success:
        raise RuntimeError(
            f"SciPy solver failed in {mode} mode with method {method}: {sol.message}"
        )


def nominal_jacobian_of(evaluator, params=None):
    """``(t, x) -> ∂f/∂x`` at the nominal input, compiled once per JAX evaluator (``params=None``: the compiled ones)."""
    forms = evaluator.compiled_forms
    if "scipy_jacobian" not in forms:
        jax = evaluator.jax
        f = evaluator.f
        u_nominal = evaluator.nominal_input
        forms["scipy_jacobian"] = jax.jit(
            jax.jacfwd(lambda x, t, p: f(x, u_nominal, t, p), argnums=0)
        )
    jacobian = forms["scipy_jacobian"]
    tree = evaluator.call_tree(params)

    return lambda t, x: np.asarray(
        jacobian(evaluator.jnp.asarray(x), np.float64(t), tree)
    )
