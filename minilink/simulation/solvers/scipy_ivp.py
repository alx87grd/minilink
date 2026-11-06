"""Adaptive SciPy :func:`scipy.integrate.solve_ivp` on the simulation time grid."""

import numpy as np
from scipy.integrate import solve_ivp

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
        # Solver options: a forced run takes no analytic Jacobian
        kw = dict(args or {})
        kw.pop("use_jac", None)
        method = kw.get("method", "RK45")
        f = evaluator.f

        # Each sample held over its period: the plant integrated adaptively from
        # sample to sample, x_{k+1} = x_k + ∫ f(x(s), u_k, s) ds over [t_k, t_{k+1}]
        n_pts = times.shape[0]
        x_traj = np.zeros((np.size(x0), n_pts), dtype=float)
        x_traj[:, 0] = x0
        nfev = 0
        for k in range(n_pts - 1):
            u_k = u[:, k]

            def rhs(t, x, u_k=u_k):
                return np.asarray(f(x, u_k, t, params))

            sol = solve_ivp(rhs, [times[k], times[k + 1]], x_traj[:, k], **kw)
            require_success(sol, "forced", method)
            x_traj[:, k + 1] = sol.y[:, -1]
            nfev += sol.nfev

        # The run's record: the periods' evaluations summed
        self.last_solve_ivp_solution = None
        self.last_debug = {
            "solver": "scipy",
            "mode": "forced",
            "method": method,
            "success": True,
            "nfev": nfev,
            "n_t": n_pts,
        }

        return x_traj

    def integrate_input(self, evaluator, times, u_of_t, x0, args=None, params=None):
        """``u(t)`` read at the solver's own evaluation points."""
        # Solver options: a forced run takes no analytic Jacobian
        kw = dict(args or {})
        kw.pop("use_jac", None)
        method = kw.get("method", "RK45")

        t0 = times[0]
        tf = times[-1]
        f = evaluator.f

        def rhs(t, x):
            return np.asarray(f(x, u_of_t(t), t, params))

        # x(t) = x₀ + ∫ f(x(s), u(s), s) ds, solved adaptively and reported on the grid
        sol = solve_ivp(rhs, [t0, tf], x0, t_eval=times, **kw)

        # The run's record; a failed solve raises
        return self.finalize_solution(sol, mode="forced", method=method)

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
