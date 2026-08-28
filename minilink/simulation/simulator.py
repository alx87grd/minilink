"""
Time-domain ODE simulation of :class:`~minilink.core.system.DynamicSystem` models
(including :class:`~minilink.core.diagram.DiagramSystem`).

Integrates ``dx/dt = f(x, u, t)`` along a time grid using pluggable solver backends
(SciPy, Euler, fixed-step RK4). Static ``System`` leaves use
:class:`~minilink.simulation.static_simulator.StaticSimulator` instead.

Discontinuous closed-loop diagrams (e.g. sliding-mode ``sign(s)``): grid-point
``Trajectory`` / ``reconstruct_internal_signals`` torques need not match RK4
sub-step behavior — see `DESIGN.md` §5 (*Discontinuous closed loops — known issues*).

Public module symbols :data:`COMPILE_BACKEND_AUTO` and :data:`RK4_AUTO_MIN_TIME_POINTS`
control automatic compile backend selection and optional fixed-step RK4 on long
uniform grids when using the JAX compiler.
"""

import time

import numpy as np

from minilink.core.backends import (
    BACKEND_AUTO,
    BACKEND_JAX,
    BACKEND_NUMPY,
)
from minilink.core.trajectory import Trajectory
from minilink.simulation.compile_backend import resolve_auto_backend
from minilink.simulation.input_coercion import coerce_forced_input
from minilink.simulation.sim_reporting import (
    print_simulation_preamble,
    print_simulation_report,
)
from minilink.simulation.solver_warnings import emit_discontinuous_solver_warnings
from minilink.simulation.solvers.euler import EulerSolverBackend
from minilink.simulation.solvers.euler_fixed import EulerFixedStepSolverBackend
from minilink.simulation.solvers.rk4_fixed import RK4SolverBackend
from minilink.simulation.solvers.scipy_ivp import SciPySolverBackend
from minilink.simulation.time_grid import build_time_grid

# Internal: user-facing solver labels to backend keys and options
# (solver backend key, options)
_USER_SOLVER_MODES: dict[str, tuple[str, dict]] = {
    "scipy": (
        "scipy",
        {
            "method": "RK45",
            "rtol": 1e-4,
            "atol": 1e-7,
            "use_jac": False,
        },
    ),
    "scipy_stiff": (
        "scipy",
        {"method": "Radau", "use_jac": True},
    ),
    "scipy_max": (
        "scipy",
        {
            "method": "DOP853",
            "rtol": 1e-6,
            "atol": 1e-8,
            "use_jac": False,
        },
    ),
    "scipy_ultra": (
        "scipy",
        {
            "method": "DOP853",
            "rtol": 1e-8,
            "atol": 1e-10,
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
    "euler_fixedsteps": ("euler_fixed", {}),
    "rk4_fixedsteps": ("rk4", {}),
}

# Long uniform rollouts can use fixed-step RK4 (JIT) instead of
# SciPy when the output grid is long enough
RK4_AUTO_MIN_TIME_POINTS = 10_000

# Default automatic dt scale relative to ``solver_info["smallest_time_constant"]``
SMOOTH_AUTO_DT_SCALE = 0.1
DISCONTINUOUS_AUTO_DT_SCALE = 0.1

# Pass ``compile_backend=COMPILE_BACKEND_AUTO`` to try JAX first, then NumPy.
# Re-exported from :mod:`minilink.core.backends` so legacy callers
# importing it from the simulator keep working.
COMPILE_BACKEND_AUTO = BACKEND_AUTO


def _time_grid_is_uniform(times: np.ndarray) -> bool:
    """True if ``times`` are evenly spaced (required for :class:`RK4SolverBackend`)."""
    if times.size < 2:
        return True
    d = np.diff(times)
    return bool(np.allclose(d, d[0]))


class Simulator:
    """
    Integrate a compiled system along a discrete time grid.

    The dynamics follow ``dx/dt = f(x, u, t)``. Inputs are nominal for
    :meth:`solve`; :meth:`solve_forced` accepts callables, constants, or sampled
    input trajectories and converts them to the solver's sampled input matrix.
    The system has state dimension ``n`` and input dimension ``m`` (from
    ``sys.n`` and ``sys.m``).

    Parameters
    ----------
    sys : System
        Model to simulate; compiled with ``compile_backend`` as below.
    x0 : array_like, optional
        Initial state in :math:`\\mathbb{R}^n`. If ``None``, uses ``sys.x0``.
    t0, tf : float
        Start and end time.
    n_steps : int, optional
        Number of time samples (including endpoints) when ``dt`` is not set.
    dt : float, optional
        Step size when ``n_steps`` is not set.
    solver : str, optional
        Solver mode; if ``None``, chosen by :meth:`select_solver`. Use
        ``"euler"`` for variable ``dt`` per output knot, or
        ``"euler_fixedsteps"`` for uniform grids via compiled Euler rollouts
        (same pattern as ``"rk4_fixedsteps"``).
    verbose : bool
        Print setup information (default quiet).
    compile_backend : str
        Name passed as ``backend`` to :meth:`~minilink.core.system.System.compile`.
        Typical values are ``numpy`` (default) or ``jax``. Use :data:`COMPILE_BACKEND_AUTO`
        (the string ``auto``) to try JAX if importable, then fall back to NumPy.
    solver_warnings : str
        ``"warn"`` (default), ``"error"``, or ``"ignore"`` for discontinuous-loop
        :class:`UserWarning` messages from :mod:`minilink.simulation.solver_warnings`.
    """

    def __init__(
        self,
        sys,
        x0=None,  # initial state, if None, use sys.x0
        t0=0,
        tf=10,
        n_steps=None,
        dt=None,
        solver=None,
        verbose=False,
        compile_backend=BACKEND_NUMPY,
        solver_warnings="warn",
    ):
        from minilink.core.system import DynamicSystem

        if not isinstance(sys, DynamicSystem):
            if sys.n == 0:
                raise TypeError(
                    "Static System leaves use StaticSimulator via "
                    "compute_trajectory, or construct StaticSimulator directly."
                )
            raise TypeError(
                f"Cannot simulate {type(sys).__name__} with n={sys.n}; "
                "use DynamicSystem for state evolution."
            )

        self.verbose = verbose
        self.sys = sys
        self.sys.refresh()
        self.user_solver = solver
        self.user_specified_dt = dt is not None
        self.auto_time_grid = n_steps is None and dt is None
        self.solver_warnings = solver_warnings
        self.t0 = float(t0)
        self.tf = float(tf)
        self.last_solve_time_s = None
        self.last_traj = None
        self.last_debug = None

        # Select the time vector
        self.t, dt, n_steps = self.select_time_vector(t0, tf, n_steps, dt, sys)
        self.n_pts = len(self.t)
        self.x0 = self._validate_x0(sys.x0 if x0 is None else x0, sys.n)

        # Compile the system
        self.compile_backend, self.evaluator = self._resolve_and_build_evaluator(
            sys, compile_backend
        )

        # Select the solver
        self.solver_mode = self.select_solver(sys, solver)
        solver_backend_key, self.solver_backend_options = self._parse_solver(
            self.solver_mode
        )
        self.solver_backend = self._select_backend(solver_backend_key)
        self.dt = dt

        setup_notes = emit_discontinuous_solver_warnings(
            solver_mode=self.solver_mode,
            solver_info=sys.solver_info,
            dt=dt,
            user_solver=self.user_solver,
            user_specified_dt=self.user_specified_dt,
            solver_warnings=self.solver_warnings,
            verbose=self.verbose,
        )

        if self.verbose:
            print_simulation_preamble(
                title="===               Time-Domain Simulation                  ===",
                system_name=sys.name,
                n=sys.n,
                m=sys.m,
                x0=self.x0,
                t0=self.t0,
                tf=self.tf,
                n_pts=self.n_pts,
                dt=dt,
                solver_mode=self.solver_mode,
                user_solver=self.user_solver,
                compile_backend=self.compile_backend,
                auto_time_grid=self.auto_time_grid,
                solver_options=self.solver_backend_options or None,
                notes=setup_notes or None,
            )

    def select_time_vector(self, t0, tf, n_steps, dt, sys):
        """
        Build time samples on [t0, tf] and return (time_vector, dt, len).

        Delegates to :func:`minilink.simulation.time_grid.build_time_grid`;
        the automatic ``dt`` (neither ``n_steps`` nor ``dt`` given) comes from
        the system's smallest time constant scaled by the smooth or
        discontinuous auto-dt policy. If both ``n_steps`` and ``dt`` are set,
        ``n_steps`` wins (a warning is logged).
        """
        if sys.solver_info.get("discontinuous_behavior", False):
            scale = DISCONTINUOUS_AUTO_DT_SCALE
        else:
            scale = SMOOTH_AUTO_DT_SCALE
        default_dt = sys.solver_info["smallest_time_constant"] * scale
        return build_time_grid(t0, tf, n_steps=n_steps, dt=dt, default_dt=default_dt)

    def select_solver(self, sys, user_solver=None):
        """
        Choose the solver backend label from the system and options.

        - If the user has specified a solver, return it.
        - If the system has discontinuous behavior, return ``"euler"``.
        - If ``compile_backend`` is ``"jax"``, the time grid is uniform, and the
          number of evaluation points is at least :data:`RK4_AUTO_MIN_TIME_POINTS`,
          return ``"rk4_fixedsteps"`` (fast JIT rollout).
        - Otherwise, return ``"scipy"``.
        """
        if user_solver is not None:
            return user_solver
        if sys.solver_info.get("discontinuous_behavior", False):
            return "euler"
        if (
            self.compile_backend == BACKEND_JAX
            and self.n_pts >= RK4_AUTO_MIN_TIME_POINTS
            and _time_grid_is_uniform(self.t)
        ):
            return "rk4_fixedsteps"
        return "scipy"

    # Core methods for integration

    def solve(self):
        """
        Run simulation with **nominal** input (constant :math:`\\bar{u}` from the
        compiled evaluator) over ``self.t``.

        Returns
        -------
        Trajectory
            State and input time series, ``(n, n_pts)`` and ``(m, n_pts)``.
        """

        # Integrate the system using the selected solver backend
        time_solve = self.verbose
        if time_solve:
            t_start = time.perf_counter()

        x_traj = self.solver_backend.integrate(
            self.evaluator, self.t, self.x0, args=self.solver_backend_options
        )

        if time_solve:
            self.last_solve_time_s = time.perf_counter() - t_start

        # Build the input trajectory

        m = self.sys.m
        u_traj = np.zeros((m, self.n_pts))
        if m > 0:
            u_bar = self.evaluator._u_nominal
            u_traj[:, :] = u_bar.reshape(m, 1)

        # Build the trajectory object

        traj = Trajectory(t=self.t, x=x_traj, u=u_traj)

        self.last_debug = self.solver_backend.last_debug
        self.last_traj = traj

        if self.verbose:
            print_simulation_report(
                elapsed_s=self.last_solve_time_s,
                n_samples=traj.n_samples,
                x_final=traj.x[:, -1] if traj.n > 0 else None,
                last_debug=self.last_debug,
            )

        return traj

    def solve_forced(self, u, input_port_id=None):
        """
        Run simulation with a prescribed input.

        ``u`` may be a full sampled input matrix, a callable ``u(t)``, a constant
        vector, or a scalar for one-dimensional inputs. If ``input_port_id`` is
        given, ``u`` describes only that port and the other ports use nominal
        values.

        Returns
        -------
        Trajectory
            State and input time series.
        """
        u_traj = coerce_forced_input(self.sys, self.t, u, input_port_id=input_port_id)
        if not self._supports_forced_mode():
            raise ValueError(
                f"Solver '{self.solver_mode}' does not support forced simulations"
            )
        time_solve = self.verbose
        if time_solve:
            t_start = time.perf_counter()

        x_traj = self.solver_backend.integrate_forced(
            self.evaluator,
            self.t,
            u_traj,
            self.x0,
            args=self.solver_backend_options,
        )

        if time_solve:
            self.last_solve_time_s = time.perf_counter() - t_start

        traj = Trajectory(t=self.t, x=x_traj, u=u_traj)
        self.last_debug = self.solver_backend.last_debug
        self.last_traj = traj

        if self.verbose:
            print_simulation_report(
                elapsed_s=self.last_solve_time_s,
                n_samples=traj.n_samples,
                x_final=traj.x[:, -1] if traj.n > 0 else None,
                last_debug=self.last_debug,
            )

        return traj

    # Private: compile, validation, and backend wiring
    def _build_evaluator(self, sys, compile_backend):
        return sys.compile(backend=compile_backend)

    def _resolve_and_build_evaluator(self, sys, compile_backend):
        """
        Compile with *compile_backend*, or if it is :data:`COMPILE_BACKEND_AUTO`, try JAX
        then NumPy.
        """
        return resolve_auto_backend(
            lambda backend: self._build_evaluator(sys, backend), compile_backend
        )

    def _validate_x0(self, x0, n):
        x0_arr = np.asarray(x0, dtype=float)
        if x0_arr.ndim != 1:
            raise ValueError(f"x0 must be a 1-D array with shape ({n},)")
        if x0_arr.shape[0] != n:
            raise ValueError(f"x0 must have shape ({n},)")
        if not np.all(np.isfinite(x0_arr)):
            raise ValueError("x0 must contain only finite values")
        return x0_arr

    def _supports_forced_mode(self):
        return True

    def _select_backend(self, solver_backend_key):
        if solver_backend_key == "scipy":
            return SciPySolverBackend()
        if solver_backend_key == "euler":
            return EulerSolverBackend()
        if solver_backend_key == "euler_fixed":
            return EulerFixedStepSolverBackend()
        if solver_backend_key == "rk4":
            return RK4SolverBackend()
        raise ValueError(f"Unknown solver '{solver_backend_key}'")

    def _parse_solver(self, solver):
        if solver not in _USER_SOLVER_MODES:
            raise ValueError(f"Unknown solver '{solver}'")
        return _USER_SOLVER_MODES[solver]
