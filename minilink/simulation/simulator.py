"""The continuous-time simulator: integrate ``dx = f(x, u, t)`` of a ``DynamicSystem`` or a diagram on a time grid."""

import time

import numpy as np

from minilink.core.backends import (
    BACKEND_AUTO,
    BACKEND_JAX,
    BACKEND_NUMPY,
)
from minilink.core.inspect import body_line, format_inspect, inspect_inline
from minilink.core.params import checked
from minilink.core.system import DynamicSystem, automatic_step_of
from minilink.core.trajectory import Trajectory
from minilink.simulation.compile_backend import resolve_auto_backend
from minilink.simulation.input_coercion import coerce_forced_input, require_x0
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

# User-facing solver labels: (solver backend key, options)
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
        {
            "method": "Radau",
            "rtol": 1e-4,
            "atol": 1e-7,
            "use_jac": True,
        },
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

# Long *explicit* uniform rollouts can use fixed-step RK4 (JIT) instead of
# SciPy when the output grid is long enough; the automatic grid never
# triggers this rule.
RK4_AUTO_MIN_TIME_POINTS = 10_000

# Solvers that integrate on the output grid itself (every non-SciPy backend):
# their automatic dt comes from the plant time constant instead of
# DEFAULT_N_STEPS.
FIXED_STEP_SOLVERS = tuple(
    label for label, (backend, _) in _USER_SOLVER_MODES.items() if backend != "scipy"
)

# Pass ``backend=COMPILE_BACKEND_AUTO`` to try JAX first, then NumPy.
# Re-exported from :mod:`minilink.core.backends` so legacy callers
# importing it from the simulator keep working.
COMPILE_BACKEND_AUTO = BACKEND_AUTO


###############################################################################
# Public API
###############################################################################


class Simulator:
    """
    Integrate a compiled system along a discrete time grid.

    The dynamics follow ``dx/dt = f(x, u, t)``. Inputs are nominal for
    :meth:`solve`, whose ``u=`` accepts callables, constants, or sampled
    input trajectories and converts them to the solver's sampled input matrix.
    The system has state dimension ``n`` and input dimension ``m`` (from
    ``sys.n`` and ``sys.m``). ``compute_trajectory`` is the facade; use the class
    directly for repeated runs with the same solver, grid and backend.

    Parameters
    ----------
    sys : System
        Model to simulate; compiled with ``backend`` as below.
    x0 : array_like, optional
        Initial state in :math:`\\mathbb{R}^n`. If ``None``, uses ``sys.x0``.
    t0, tf : float
        Start and end time.
    n_steps : int, optional
        Number of time samples (including endpoints) when ``dt`` is not set.
    dt : float, optional
        Step size when ``n_steps`` is not set. The grid ends at ``tf`` when
        ``dt`` divides ``tf - t0``; otherwise its last sample is the first
        step past ``tf``.
    solver : str, optional
        Solver mode; if ``None``, chosen by :meth:`select_solver`. Use
        ``"euler"`` for variable ``dt`` per output knot, or
        ``"euler_fixedsteps"`` for uniform grids via compiled Euler rollouts
        (same pattern as ``"rk4_fixedsteps"``).
    verbose : bool
        Print setup information (default quiet).
    backend : str
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
        tf=10,
        *,
        x0=None,  # Initial state; sys.x0 when None
        t0=0,
        n_steps=None,
        dt=None,
        solver=None,
        verbose=False,
        backend=BACKEND_NUMPY,
        solver_warnings="warn",
    ):
        require_dynamic_system(sys)

        # The model and the options as given
        self.verbose = verbose
        self.sys = sys
        self.user_solver = solver
        self.user_specified_dt = dt is not None
        self.auto_time_grid = n_steps is None and dt is None
        self.solver_warnings = solver_warnings
        self.t0 = float(t0)
        self.tf = float(tf)

        # The latest run, filled by solve
        self.last_solve_time_s = None
        self.last_traj = None
        self.last_debug = None

        # The initial state, checked
        self.x0 = require_x0(sys.x0 if x0 is None else x0, sys.n)

        # Compile the system: its numbers, taken now, are what a solve runs on unless
        # it is given a tree (solve(params=...))
        self.backend, self.evaluator = self.resolve_and_build_evaluator(sys, backend)

        # Solver and time grid. With an automatic grid the solver is chosen
        # first and the grid is sized to it; with an explicit grid the solver
        # may still be chosen from the grid (JAX auto-RK4 on long uniform grids).
        self.t = None
        self.n_pts = None
        self.solver_mode = None
        if self.auto_time_grid:
            self.solver_mode = self.select_solver(sys, solver)
            self.t, dt, n_steps = self.select_time_vector(
                t0, tf, None, None, sys, solver_mode=self.solver_mode
            )
            self.n_pts = len(self.t)
        else:
            self.t, dt, n_steps = self.select_time_vector(t0, tf, n_steps, dt, sys)
            self.n_pts = len(self.t)
            self.solver_mode = self.select_solver(sys, solver)

        # The solver backend and its options
        solver_backend_key, self.solver_backend_options = self.parse_solver(
            self.solver_mode
        )
        self.solver_backend = self.select_backend(solver_backend_key)
        self.dt = dt

        # Advisories for discontinuous loops and held signals: warned, or listed
        # in the setup panel when verbose
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
            print_setup_panel(self, setup_notes)

    def __str__(self):
        return simulator_text(self)

    def solve(self, *, u=None, input_port_id=None, params=None, record=()):
        """
        Run the simulation over ``self.t``: the nominal input, or ``u`` prescribed.

        ``u`` is ``None`` for the constant :math:`\\bar{u}` of the compiled
        evaluator; else a sampled matrix ``(m, n_pts)``, a function ``u(t)``, a
        constant vector, or a scalar for a one-input system. A sampled input is
        held over each sample period and a function of time is read where the
        solver evaluates ``f``, on every solver; ``input_port_id`` prescribes one
        port and keeps the others at their nominal values. ``params=None`` runs
        on the numbers taken when the simulator was built (the compiled form); a
        tree, the system's complete tree at other numbers (``p = sim.params``, a
        copy, edited), runs on those without a recompile: a sweep on one
        simulator compiles once.

        ``record`` names a diagram's signals to keep in the trajectory's
        ``signals`` (``"block:port"``, a block's whole input ``"block:u"``, a
        nested block by its path): a fixed-step run reads them off each step's
        own first stage, any other run computes them at the saved samples after
        the run, both on the numbers the run was made with.

        Returns
        -------
        Trajectory
            State and input time series, ``(n, n_pts)`` and ``(m, n_pts)``.
        """
        params = self.tree_of(params)
        local, nested = split_record(record)
        from_pass = bool(local) and self.solver_backend.records_from_pass
        records = None

        # Integrate with the selected solver backend, timed when verbose
        time_solve = self.verbose
        if time_solve:
            t0 = time.perf_counter()

        if u is None:
            # The nominal input ū, held over the whole grid
            x_traj = self.solver_backend.integrate(
                self.evaluator,
                self.t,
                self.x0,
                args=self.solver_backend_options,
                params=params,
                **({"record": local} if from_pass else {}),
            )
            m = self.sys.m
            u_traj = np.zeros((m, self.n_pts))
            if m > 0:
                u_bar = self.evaluator._u_nominal
                u_traj[:, :] = u_bar.reshape(m, 1)
        elif callable(u):
            # A function of time, read where the solver evaluates f: recorded after
            from_pass = False
            sys = self.sys

            def u_of_t(t):
                return coerce_forced_input(sys, np.array([t]), u, input_port_id)[:, 0]

            x_traj = self.solver_backend.integrate_input(
                self.evaluator,
                self.t,
                u_of_t,
                self.x0,
                args=self.solver_backend_options,
                params=params,
            )
            u_traj = coerce_forced_input(
                self.sys, self.t, u, input_port_id=input_port_id
            )
        else:
            # A sampled input, each sample held over its period
            u_traj = coerce_forced_input(
                self.sys, self.t, u, input_port_id=input_port_id
            )
            x_traj = self.solver_backend.integrate_forced(
                self.evaluator,
                self.t,
                u_traj,
                self.x0,
                args=self.solver_backend_options,
                params=params,
                **({"record": local} if from_pass else {}),
            )
        if from_pass:
            x_traj, records = x_traj

        # The named signals: off the steps' own pass, or at the saved samples
        signals = self.recorded_signals(x_traj, u_traj, params, local, nested, records)

        if time_solve:
            self.last_solve_time_s = time.perf_counter() - t0

        # The trajectory, kept with the solver's debug record
        return self.store_trajectory(x_traj, u_traj, signals)

    @property
    def params(self) -> dict:
        """The numbers the simulator runs on, taken at its build: a copy, the tree ``solve(params=)`` takes edited."""
        return self.evaluator.params

    def tree_of(self, params):
        """The tree a run takes: ``None`` (the compiled numbers), or ``params`` checked complete against them."""
        return None if params is None else checked(params, self.evaluator.params)

    # Solver and time-grid selection

    def select_time_vector(self, t0, tf, n_steps, dt, sys, solver_mode=None):
        """
        Build time samples on [t0, tf] and return (time_vector, dt, len).

        Explicit ``n_steps`` or ``dt`` win (``n_steps`` if both are set, with a
        logged warning). With neither, the automatic grid depends on the
        solver: fixed-step solvers (:data:`FIXED_STEP_SOLVERS`) integrate on
        the grid, so ``dt`` is :func:`~minilink.core.system.automatic_step_of`:
        the system's smallest time constant scaled by
        :data:`~minilink.core.system.AUTO_DT_SCALE`, rounded down to a divisor
        of the sample period when a block holds a signal; adaptive solvers pick
        their own steps, so the grid is only a reporting resolution with
        :data:`~minilink.simulation.time_grid.DEFAULT_N_STEPS` points.
        """
        # An adaptive solver picks its own steps: the DEFAULT_N_STEPS reporting grid
        default_dt = None

        # A fixed-step solver steps on the grid: Δt = 0.1 τ_min, dividing the hold period
        if solver_mode in FIXED_STEP_SOLVERS:
            default_dt = automatic_step_of(sys.solver_info)

        return build_time_grid(t0, tf, n_steps=n_steps, dt=dt, default_dt=default_dt)

    def select_solver(self, sys, user_solver=None):
        """
        Choose the solver backend label from the system and options.

        - If the user has specified a solver, return it.
        - If the system has discontinuous behavior, return ``"euler"``.
        - If a block holds a signal over a sample period (``solver_info
          ["sample_period"]``, a noise source), return ``"rk4_fixedsteps"``:
          the automatic grid then steps at a divisor of that period.
        - If the time grid was given explicitly, ``backend`` is
          ``"jax"``, the grid is uniform, and it has at least
          :data:`RK4_AUTO_MIN_TIME_POINTS` points, return ``"rk4_fixedsteps"``
          (fast JIT rollout). The automatic grid never triggers this rule.
        - Otherwise, return ``"scipy"``.
        """
        if user_solver is not None:
            return user_solver
        if sys.solver_info.get("discontinuous_behavior", False):
            return "euler"
        if sys.solver_info.get("sample_period") is not None:
            return "rk4_fixedsteps"
        if (
            not self.auto_time_grid
            and self.backend == BACKEND_JAX
            and self.n_pts >= RK4_AUTO_MIN_TIME_POINTS
            and time_grid_is_uniform(self.t)
        ):
            return "rk4_fixedsteps"
        return "scipy"

    ###########################################################################
    # Internal machinery
    ###########################################################################

    def resolve_and_build_evaluator(self, sys, backend):
        """``(backend, evaluator)``: ``sys`` compiled with ``backend``, or JAX then NumPy on :data:`COMPILE_BACKEND_AUTO`."""
        return resolve_auto_backend(
            lambda backend: self.build_evaluator(sys, backend), backend
        )

    def build_evaluator(self, sys, backend):
        """``sys`` compiled with ``backend``."""
        return sys.compile(backend=backend)

    def parse_solver(self, solver):
        """``(backend key, options)`` of a solver label."""
        if solver not in _USER_SOLVER_MODES:
            raise ValueError(f"Unknown solver '{solver}'")
        return _USER_SOLVER_MODES[solver]

    def select_backend(self, solver_backend_key):
        """A fresh solver backend for its key."""
        if solver_backend_key == "scipy":
            return SciPySolverBackend()
        if solver_backend_key == "euler":
            return EulerSolverBackend()
        if solver_backend_key == "euler_fixed":
            return EulerFixedStepSolverBackend()
        if solver_backend_key == "rk4":
            return RK4SolverBackend()
        raise ValueError(f"Unknown solver '{solver_backend_key}'")

    def recorded_signals(self, x_traj, u_traj, params, local, nested, records):
        """
        The run's named signals by name, ``(dim, n_pts)`` each, on the run's numbers.

        ``records`` are the fixed-step run's own, at the start of each step, the
        last sample one more signal pass; without them every saved sample takes
        one pass (vectorized on JAX). A nested block's signal is read one level at
        a time by the diagram.
        """
        signals = {}
        if local:
            ev = self.evaluator
            layout = ev.record_layout(local)
            xs = np.asarray(x_traj, dtype=float).T
            us = np.asarray(u_traj, dtype=float).T
            if records is None:
                values = values_at_samples(ev, layout, xs, us, self.t, params)
            else:
                last = ev.record_at(xs[-1], us[-1], self.t[-1], params, layout)
                values = np.concatenate(
                    [np.asarray(records, dtype=float), np.asarray(last)[None, :]]
                )

            # Each name's columns of the stacked record
            start = 0
            for name, sl, op in layout:
                dim = (sl.stop - sl.start) if op is None else int(op.u_dim)
                signals[name] = values[:, start : start + dim].T
                start += dim

        if nested:
            tree = self.evaluator.params if params is None else params
            run = Trajectory(t=self.t, x=x_traj, u=u_traj)
            for name in nested:
                signals[name] = self.sys.port_samples(run, name, tree)

        return signals

    def store_trajectory(self, x_traj, u_traj, signals=None):
        """The run's trajectory, kept as ``last_traj`` with the solver's debug record."""
        traj = Trajectory(t=self.t, x=x_traj, u=u_traj, signals=signals or {})
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


###############################################################################
# Internal machinery
###############################################################################


def split_record(record):
    """``(local, nested)``: the names a run's own signal pass reads, and those of a block inside a nested diagram."""
    names = tuple(record or ())
    local = tuple(name for name in names if "/" not in name.rpartition(":")[0])
    nested = tuple(name for name in names if name not in local)

    return local, nested


def values_at_samples(ev, layout, xs, us, ts, params):
    """The recorded values at every saved sample, ``(n_pts, r)``: one signal pass each, one ``vmap`` on JAX."""
    if ev.backend == "jax":
        jax = ev.jax
        at = jax.vmap(
            lambda x, u, t: ev.record_at(x, u, t, params, layout), in_axes=(0, 0, 0)
        )
        return np.asarray(jax.jit(at)(xs, us, ts))

    return np.stack(
        [ev.record_at(x, u, t, params, layout) for x, u, t in zip(xs, us, ts)]
    )


def require_dynamic_system(sys) -> None:
    """Refuse a system without continuous state: a static leaf or a step system."""
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


def time_grid_is_uniform(times: np.ndarray) -> bool:
    """True if ``times`` are evenly spaced (required for :class:`RK4SolverBackend`)."""
    if times.size < 2:
        return True
    d = np.diff(times)
    return bool(np.allclose(d, d[0], atol=0.0))


def print_setup_panel(sim: Simulator, notes: list[str]) -> None:
    """The verbose setup panel: model, grid, solver, backend and the advisories."""
    print_simulation_preamble(
        title="===               Time-Domain Simulation                  ===",
        system_name=sim.sys.name,
        n=sim.sys.n,
        m=sim.sys.m,
        x0=sim.x0,
        t0=sim.t0,
        tf=sim.tf,
        n_pts=sim.n_pts,
        dt=sim.dt,
        solver_mode=sim.solver_mode,
        user_solver=sim.user_solver,
        backend=sim.backend,
        auto_time_grid=sim.auto_time_grid,
        solver_options=sim.solver_backend_options or None,
        notes=notes or None,
    )


def simulator_text(sim):
    """The print text of a simulator: its solver, step and grid, and the system."""
    headline = f"Simulator, solver={sim.solver_mode!r}, dt={sim.dt:g}, N={sim.n_pts}"
    body = [body_line("sys", inspect_inline(sim.sys))]

    return format_inspect(headline, body)
