import logging

import numpy as np

from minilink.core.trajectory import Trajectory
from minilink.simulation.solver_backends import (
    EulerSolverBackend,
    RK4SolverBackend,
    SciPySolverBackend,
)

# (solver backend key, options). Used by :meth:`Simulator._parse_solver`.
_USER_SOLVER_MODES: dict[str, tuple[str, dict]] = {
    "scipy": (
        "scipy",
        {"method": "RK45", "use_jac": False},
    ),
    "scipy_stiff": (
        "scipy",
        {"method": "Radau", "use_jac": True},
    ),
    "scipy_max": (
        "scipy",
        {
            "method": "DOP853",
            "rtol": 1e-8,
            "atol": 1e-10,
            "use_jac": False,
        },
    ),
    "scipy_ultra": (
        "scipy",
        {
            "method": "DOP853",
            "rtol": 1e-8,
            "atol": 1e-11,
            "use_jac": False,
        },
    ),
    "scipy_lsoda": (
        "scipy",
        {
            "method": "LSODA",
            "rtol": 3e-7,
            "atol": 1e-11,
            "use_jac": False,
        },
    ),
    "euler": ("euler", {}),
    "rk4_fixedsteps": ("rk4", {}),
}


class Simulator:
    """Prototype simulator using pluggable solver backends."""

    def __init__(
        self,
        sys,
        x0=None,  # initial state, if None, use sys.x0
        t0=0,
        tf=10,
        n_steps=None,
        dt=None,
        solver=None,
        verbose=True,
        compile_backend="numpy",
    ):
        self.verbose = verbose
        self.sys = sys
        self.compile_backend = compile_backend
        self.scipy_last_solution = None
        self.sys.refresh()

        if self.verbose:
            print(
                f"Simulator:\n"
                "--------------\n"
                f"Simulating system {sys.name} from t={t0} to t={tf}"
            )

        self.t, dt, n_steps = self.select_time_vector(t0, tf, n_steps, dt, sys)
        self.times = self.t
        self.n_pts = len(self.times)
        self.x0 = self._validate_x0(sys.x0 if x0 is None else x0, sys.n)

        self.solver_mode = self.select_solver(sys, solver)
        solver_backend_key, self.solver_backend_options = self._parse_solver(
            self.solver_mode
        )
        self.solver_backend = self._select_backend(solver_backend_key)
        self.evaluator = self._build_evaluator(sys, compile_backend)
        self.last_debug = None

        if self.verbose:
            print(f"Time steps = {n_steps}, dt={dt} and solver= {self.solver_mode}")

    def _build_evaluator(self, sys, compile_backend):
        if self.verbose:
            print(f"Auto-compiling backend={compile_backend}.")
        return sys.compile(backend=compile_backend)

    def _validate_x0(self, x0, state_dim):
        x0_arr = np.asarray(x0, dtype=float)
        if x0_arr.ndim != 1:
            raise ValueError(f"x0 must be a 1-D array with shape ({state_dim},)")
        if x0_arr.shape[0] != state_dim:
            raise ValueError(f"x0 must have shape ({state_dim},)")
        if not np.all(np.isfinite(x0_arr)):
            raise ValueError("x0 must contain only finite values")
        return x0_arr

    def _validate_n_steps(self, n_steps):
        if isinstance(n_steps, bool) or not isinstance(n_steps, (int, np.integer)):
            raise ValueError("n_steps must be an integer greater than or equal to 2")
        if n_steps < 2:
            raise ValueError("n_steps must be greater than or equal to 2")

    def _validate_dt(self, dt, *, label="dt"):
        if not np.isscalar(dt):
            raise ValueError(f"{label} must be a positive finite scalar")
        dt_value = float(dt)
        if not np.isfinite(dt_value) or dt_value <= 0.0:
            raise ValueError(f"{label} must be a positive finite scalar")
        return dt_value

    def _validate_forced_u_traj(self, u_traj):
        u_arr = np.asarray(u_traj, dtype=float)
        expected_shape = (self.sys.m, self.n_pts)
        if u_arr.ndim != 2:
            raise ValueError(f"u_traj must have shape {expected_shape}")
        if u_arr.shape != expected_shape:
            raise ValueError(f"u_traj must have shape {expected_shape}")
        if not np.all(np.isfinite(u_arr)):
            raise ValueError("u_traj must contain only finite values")
        return u_arr

    def _supports_forced_mode(self):
        return not isinstance(self.solver_backend, RK4SolverBackend)

    def _select_backend(self, solver_backend_key):
        if solver_backend_key == "scipy":
            return SciPySolverBackend()
        if solver_backend_key == "euler":
            return EulerSolverBackend()
        if solver_backend_key == "rk4":
            return RK4SolverBackend()
        raise ValueError(f"Unknown solver '{solver_backend_key}'")

    def select_solver(self, sys, user_solver=None):
        """
        Intelligently select the solver backend based on the system's properties.
        - If the user has specified a solver, return it.
        - If the system has discontinuous behavior, return "scipy_stiff".
        - Otherwise, return "scipy".
        """
        if user_solver is not None:
            return user_solver
        if not sys.solver_info["continuous_time_equation"]:
            raise ValueError("Prototype Simulator does not support discrete solver")
        if sys.solver_info["discontinuous_behavior"]:
            return "scipy_stiff"
        return "scipy"

    def select_time_vector(self, t0, tf, n_steps, dt, sys):
        if not np.isscalar(t0) or not np.isscalar(tf):
            raise ValueError("t0 and tf must be finite scalars")
        t0 = float(t0)
        tf = float(tf)
        if not np.isfinite(t0) or not np.isfinite(tf):
            raise ValueError("t0 and tf must be finite scalars")
        if tf <= t0:
            raise ValueError("tf must be greater than t0")

        if n_steps is None and dt is None:
            dt = self._validate_dt(
                sys.solver_info["smallest_time_constant"] * 0.1,
                label="automatic dt",
            )
            time_vector = np.arange(t0, tf + dt, dt)
            if self.verbose:
                print("Automatic dt based on the smallest time constant of the system")
        elif dt is None:
            self._validate_n_steps(n_steps)
            time_vector = np.linspace(t0, tf, n_steps)
        elif n_steps is None:
            dt = self._validate_dt(dt)
            time_vector = np.arange(t0, tf + dt, dt)
        else:
            self._validate_n_steps(n_steps)
            logging.warning(
                "You must choose between n_steps and dt: using the specified n_steps"
            )
            time_vector = np.linspace(t0, tf, n_steps)

        if time_vector.size < 2:
            raise ValueError("Time vector must contain at least two points")
        dt = time_vector[1] - time_vector[0]
        n_steps = len(time_vector)
        return time_vector, dt, n_steps

    def solve(self):
        try:
            x_traj = self.solver_backend.integrate(
                self.evaluator, self.times, self.x0, args=self.solver_backend_options
            )
        except Exception:
            self.last_debug = self.solver_backend.last_debug
            self.scipy_last_solution = getattr(
                self.solver_backend, "last_solve_ivp_solution", None
            )
            raise

        self.scipy_last_solution = getattr(
            self.solver_backend, "last_solve_ivp_solution", None
        )

        # Build the input trajectory
        u_traj = np.zeros((self.sys.m, self.n_pts))
        if self.sys.m > 0:
            u_nom = self.evaluator._u_nominal
            u_traj[:, :] = u_nom.reshape(self.sys.m, 1)

        traj = Trajectory(t=self.times, x=x_traj, u=u_traj)

        # Memory for debugging purposes
        self.last_debug = self.solver_backend.last_debug
        self.last_traj = traj

        return traj

    def solve_forced(self, u_traj):
        u_traj = self._validate_forced_u_traj(u_traj)
        if not self._supports_forced_mode():
            raise ValueError(
                f"Solver '{self.solver_mode}' does not support forced simulations"
            )
        try:
            x_traj = self.solver_backend.integrate_forced(
                self.evaluator,
                self.times,
                u_traj,
                self.x0,
                args=self.solver_backend_options,
            )
        except Exception:
            self.last_debug = self.solver_backend.last_debug
            self.scipy_last_solution = getattr(
                self.solver_backend, "last_solve_ivp_solution", None
            )
            raise

        self.scipy_last_solution = getattr(
            self.solver_backend, "last_solve_ivp_solution", None
        )
        self.last_debug = self.solver_backend.last_debug
        traj = Trajectory(t=self.times, x=x_traj, u=u_traj)

        # Memory for debugging purposes
        self.last_debug = self.solver_backend.last_debug
        self.last_traj = traj

        return traj

    def _parse_solver(self, solver):
        if solver not in _USER_SOLVER_MODES:
            raise ValueError(f"Unknown solver '{solver}'")
        return _USER_SOLVER_MODES[solver]
