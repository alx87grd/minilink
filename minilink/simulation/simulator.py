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

import logging
import time

import numpy as np

from minilink.core.backends import (
    BACKEND_AUTO,
    BACKEND_JAX,
    BACKEND_NUMPY,
)
from minilink.core.trajectory import Trajectory
from minilink.simulation.sim_reporting import (
    print_simulation_preamble,
    print_simulation_report,
)
from minilink.simulation.solver_warnings import emit_discontinuous_solver_warnings
from minilink.simulation.solvers.euler import EulerSolverBackend
from minilink.simulation.solvers.euler_fixed import EulerFixedStepSolverBackend
from minilink.simulation.solvers.rk4_fixed import RK4SolverBackend
from minilink.simulation.solvers.scipy_ivp import SciPySolverBackend

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

        # Select the time vector
        self.t, dt, n_steps = self.select_time_vector(t0, tf, n_steps, dt, sys)
        self.times = self.t
        self.n_pts = len(self.times)
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

        self.scipy_last_solution = None
        self.last_debug = None

    def select_time_vector(self, t0, tf, n_steps, dt, sys):
        """
        Build time samples on [t0, tf] and return (time_vector, dt, len).

        If n_steps is set, uses a uniform grid of that many points. If only dt is
        set, uses that step with arange. If neither, picks dt from the system's
        smallest time constant. If both n_steps and dt are set, n_steps wins (a
        warning is logged).
        """

        # Validate the time vector
        try:
            t0, tf = float(t0), float(tf)
        except (TypeError, ValueError) as exc:
            raise ValueError("t0 and tf must be real scalars") from exc
        if not (np.isfinite(t0) and np.isfinite(tf)):
            raise ValueError("t0 and tf must be finite")
        if tf <= t0:
            raise ValueError("tf must be greater than t0")

        # Automatically
        if n_steps is None and dt is None:
            if sys.solver_info.get("discontinuous_behavior", False):
                scale = DISCONTINUOUS_AUTO_DT_SCALE
            else:
                scale = SMOOTH_AUTO_DT_SCALE
            dt = self._validate_dt(
                sys.solver_info["smallest_time_constant"] * scale,
                label="automatic dt",
            )
            time_vector = np.arange(t0, tf + dt, dt)

        # If only dt is set, use a uniform grid with that step size
        elif dt is None:
            self._validate_n_steps(n_steps)
            time_vector = np.linspace(t0, tf, n_steps)

        # If only n_steps is set, use a uniform grid with that many steps
        elif n_steps is None:
            dt = self._validate_dt(dt)
            time_vector = np.arange(t0, tf + dt, dt)

        # If both n_steps and dt are set, use a uniform grid with that many steps and step size
        else:
            self._validate_n_steps(n_steps)
            logging.warning(
                "You must choose between n_steps and dt: using the specified n_steps"
            )
            time_vector = np.linspace(t0, tf, n_steps)

        if time_vector.size < 2:
            raise ValueError("Time vector must contain at least two points")

        # Return the time vector, step size, and number of steps
        dt = time_vector[1] - time_vector[0]
        n_steps = len(time_vector)
        return time_vector, dt, n_steps

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
            and _time_grid_is_uniform(self.times)
        ):
            return "rk4_fixedsteps"
        return "scipy"

    # Core methods for integration

    def solve(self):
        """
        Run simulation with **nominal** input (constant :math:`\\bar{u}` from the
        compiled evaluator) over ``self.times``.

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
            self.evaluator, self.times, self.x0, args=self.solver_backend_options
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

        traj = Trajectory(t=self.times, x=x_traj, u=u_traj)

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
        u_traj = self._coerce_forced_input(u, input_port_id=input_port_id)
        if not self._supports_forced_mode():
            raise ValueError(
                f"Solver '{self.solver_mode}' does not support forced simulations"
            )
        time_solve = self.verbose
        if time_solve:
            t_start = time.perf_counter()

        x_traj = self.solver_backend.integrate_forced(
            self.evaluator,
            self.times,
            u_traj,
            self.x0,
            args=self.solver_backend_options,
        )

        if time_solve:
            self.last_solve_time_s = time.perf_counter() - t_start

        traj = Trajectory(t=self.times, x=x_traj, u=u_traj)
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
        if compile_backend != BACKEND_AUTO:
            return compile_backend, self._build_evaluator(sys, compile_backend)

        try:
            import jax  # noqa: F401
        except ImportError:
            return BACKEND_NUMPY, self._build_evaluator(sys, BACKEND_NUMPY)
        try:
            return BACKEND_JAX, self._build_evaluator(sys, BACKEND_JAX)
        except Exception:
            logging.getLogger(__name__).debug(
                "JAX compile failed, falling back to numpy", exc_info=True
            )
            return BACKEND_NUMPY, self._build_evaluator(sys, BACKEND_NUMPY)

    def _validate_x0(self, x0, n):
        x0_arr = np.asarray(x0, dtype=float)
        if x0_arr.ndim != 1:
            raise ValueError(f"x0 must be a 1-D array with shape ({n},)")
        if x0_arr.shape[0] != n:
            raise ValueError(f"x0 must have shape ({n},)")
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
        u = np.asarray(u_traj, dtype=float)
        m, n_pts = self.sys.m, self.n_pts
        expected_shape = (m, n_pts)
        if u.ndim != 2:
            raise ValueError(f"u_traj must have shape {expected_shape}")
        if u.shape != expected_shape:
            raise ValueError(f"u_traj must have shape {expected_shape}")
        if not np.all(np.isfinite(u)):
            raise ValueError("u_traj must contain only finite values")
        return u

    def _coerce_forced_input(self, u, input_port_id=None):
        if input_port_id is None:
            return self._coerce_forced_signal(u, self.sys.m, "u")

        port = self.sys.inputs[input_port_id]
        port_slice = self.sys.get_input_port_slice(input_port_id)

        u_nominal = self.sys.get_u_from_input_ports().reshape(self.sys.m, 1)
        u_traj = np.repeat(u_nominal, self.n_pts, axis=1)
        u_traj[port_slice, :] = self._coerce_forced_signal(
            u,
            port.dim,
            f"u for input port '{input_port_id}'",
        )
        return self._validate_forced_u_traj(u_traj)

    def _coerce_forced_signal(self, data, expected_dim, label):
        if callable(data):
            signal = self._sample_forced_callable(data, expected_dim)
        else:
            signal = self._coerce_forced_array(data, expected_dim, label)

        if not np.all(np.isfinite(signal)):
            raise ValueError(f"{label} must contain only finite values")
        return signal

    def _sample_forced_callable(self, fn, expected_dim):
        samples = np.zeros((expected_dim, self.n_pts), dtype=float)

        for i, ti in enumerate(self.times):
            value = np.asarray(fn(float(ti)), dtype=float)
            if expected_dim == 1 and value.ndim == 0:
                samples[0, i] = float(value)
                continue
            samples[:, i] = value.reshape(expected_dim)

        return samples

    def _coerce_forced_array(self, data, expected_dim, label):
        arr = np.asarray(data, dtype=float)
        expected_shape = (expected_dim, self.n_pts)

        if arr.ndim == 0:
            if expected_dim != 1:
                raise ValueError(f"{label} must have shape {expected_shape}")
            return np.full((1, self.n_pts), float(arr), dtype=float)

        if arr.ndim == 1:
            if expected_dim == 1 and arr.shape[0] == self.n_pts:
                return arr.reshape(1, self.n_pts)
            if arr.shape[0] == expected_dim:
                column = arr.reshape(expected_dim, 1)
                return np.repeat(column, self.n_pts, axis=1)
            raise ValueError(f"{label} must have shape {expected_shape}")

        if arr.ndim == 2 and arr.shape == expected_shape:
            return arr

        raise ValueError(f"{label} must have shape {expected_shape}")

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
