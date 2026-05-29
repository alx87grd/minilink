"""SciPy :func:`scipy.integrate.solve_ivp` integrator backend."""

import numpy as np
from scipy.integrate import solve_ivp

from minilink.simulation.input_interpolation import INPUT_INTERP_KEY, build_u_at_t
from minilink.simulation.solvers.solver import SolverBackend


class SciPySolverBackend(SolverBackend):
    """SciPy solve_ivp backend."""

    def __init__(self) -> None:
        self.last_debug = None
        self.last_solve_ivp_solution = None

    def _finalize_solution(self, sol, *, mode, method, extra_debug=None):
        self.last_solve_ivp_solution = sol
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
        if not sol.success:
            raise RuntimeError(
                f"SciPy solver failed in {mode} mode with method "
                f"{method}: {sol.message}"
            )
        return sol.y

    def integrate(
        self,
        evaluator,
        times: np.ndarray,
        x0: np.ndarray,
        args=None,
    ) -> np.ndarray:

        # Get the time interval
        t0, tf = times[0], times[-1]

        # Build the right-hand side of the ODE
        rhs = evaluator.as_scipy_rhs()

        # Get the integration parameters
        kw = dict(args or {})
        use_jac = kw.pop("use_jac", False)

        # Build the Jacobian if needed
        if use_jac and getattr(evaluator, "backend", None) == "jax":
            kw["jac"] = evaluator.as_scipy_jac()
            jac_applied = True
        else:
            jac_applied = False

        # Integrate the system
        sol = solve_ivp(rhs, [t0, tf], x0, t_eval=times, **kw)

        # Debug information
        method = kw.get("method", "RK45")
        return self._finalize_solution(
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
    ) -> np.ndarray:

        # Get the integration parameters
        kw = dict(args or {})
        scheme = kw.pop(INPUT_INTERP_KEY, "linear")
        kw.pop("use_jac", None)

        # Debug information
        method = kw.get("method", "RK45")

        # Get the time interval
        t0, tf = times[0], times[-1]

        # Build the forced right-hand side of the ODE
        u_at_t = build_u_at_t(times, u, scheme)
        rhs = evaluator.as_scipy_rhs_forced(u_at_t)

        # Integrate the system
        sol = solve_ivp(rhs, [t0, tf], x0, t_eval=times, **kw)

        # Debug information
        return self._finalize_solution(
            sol,
            mode="forced",
            method=method,
            extra_debug={"interp": scheme},
        )
