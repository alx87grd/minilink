import logging

import numpy as np

from minilink.core.analysis import Trajectory
from minilink.simulation.solver_backends import (
    EulerSolverBackend,
    RK4SolverBackend,
    SciPySolverBackend,
)


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
        solver="scipy",
        verbose=True,
        compile_backend="numpy",
    ):
        self.verbose = verbose
        self.sys = sys
        self.compile_backend = compile_backend
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
        self.x0 = sys.x0 if x0 is None else x0
        assert self.x0.shape[0] == sys.n, "x0 has the wrong dimension"

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
        if n_steps is None and dt is None:
            dt = sys.solver_info["smallest_time_constant"] * 0.1
            time_vector = np.arange(t0, tf + dt, dt)
            if self.verbose:
                print("Automatic dt based on the smallest time constant of the system")
        elif dt is None:
            time_vector = np.linspace(t0, tf, n_steps)
        elif n_steps is None:
            time_vector = np.arange(t0, tf + dt, dt)
        else:
            logging.warning(
                "You must choose between n_steps and dt: using the specified n_steps"
            )
            time_vector = np.linspace(t0, tf, n_steps)

        dt = time_vector[1] - time_vector[0]
        n_steps = len(time_vector)
        return time_vector, dt, n_steps

    def solve(self, show=False):
        x_traj = self.solver_backend.integrate(
            self.evaluator, self.times, self.x0, args=self.solver_backend_options
        )

        # Build the input trajectory
        u_traj = np.zeros((self.sys.m, self.n_pts))
        if self.sys.m > 0:
            u_nom = self.evaluator._u_nominal
            u_traj[:, :] = u_nom.reshape(self.sys.m, 1)

        traj = Trajectory(x_traj, u_traj, self.times)

        # Memory for debugging purposes
        self.last_debug = self.solver_backend.last_debug
        self.last_traj = traj

        return traj

    def solve_forced(self, u_traj, show=False):
        x_traj = self.solver_backend.integrate_forced(
            self.evaluator,
            self.times,
            u_traj,
            self.x0,
            args=self.solver_backend_options,
        )
        self.last_debug = self.solver_backend.last_debug
        traj = Trajectory(x_traj, u_traj, self.times)

        # Memory for debugging purposes
        self.last_debug = self.solver_backend.last_debug
        self.last_traj = traj

        return traj

    def _parse_solver(self, solver):
        mapping = {
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
                    "rtol": 1e-9,
                    "atol": 1e-11,
                    "use_jac": False,
                },
            ),
            "euler": ("euler", {}),
            "rk4_fixedsteps": ("rk4", {}),
        }
        if solver not in mapping:
            raise ValueError(f"Unknown solver '{solver}'")
        return mapping[solver]
