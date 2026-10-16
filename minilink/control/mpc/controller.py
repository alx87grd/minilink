"""Model predictive control as a System block: one trajectory-optimization solve per replan tick."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np

from minilink.control.mpc.utilities import (
    NominalCache,
    build_nominal_cache,
    eval_signal,
    mpc_default_computer_x0,
    mpc_warm_start_guess,
)
from minilink.core.backends import BACKEND_JAX, normalize_backend
from minilink.core.system import StepSystem, System
from minilink.core.trajectory import Trajectory
from minilink.planning.results import PlanningSolution
from minilink.planning.trajectory_optimization.planner import (
    TrajectoryOptimizationPlanner,
    reject_unknown_online_params,
)

if TYPE_CHECKING:
    from minilink.simulation.computer import Computer, StepSchedule


###############################################################################
# Public API
###############################################################################


def ModelPredictiveController(
    planner: TrajectoryOptimizationPlanner,
    *,
    dt_mpc: float,
    warm_start: bool = True,
    verbose: bool = False,
    t0: float = 0.0,
    debug: bool = False,
):
    """
    Build the product MPC System family block.

    Parameters
    ----------
    planner : TrajectoryOptimizationPlanner
        Trajopt planner (auto-compiles parametric NLP when needed).
    dt_mpc : float
        Replan period (required for both warm-start modes).
    warm_start : bool, optional
        If True, return a :class:`~minilink.core.system.StepSystem` with packed
        ``z`` on ``Computer.x``. If False, algebraic :class:`~minilink.core.system.System`.
    verbose, t0, debug
        Latch printouts, absolute-time epoch, and live debug figure auto-update.
    """
    # Warm start: the last solve's packed z is the block's state
    if warm_start:
        return MPCStatefulController(
            planner,
            dt_mpc=dt_mpc,
            verbose=verbose,
            t0=t0,
            debug=debug,
        )

    # Cold start: an algebraic block, no solver state kept between ticks
    return MPCStatelessController(
        planner,
        dt_mpc=dt_mpc,
        verbose=verbose,
        t0=t0,
        debug=debug,
    )


@dataclass(frozen=True)
class Command:
    """
    One replan tick result for deploy or debugging.

    ``solution.trajectory.t`` is plan-local time with zero at the solve
    instant. Absolute solve time is ``t_solve``.
    """

    solution: PlanningSolution
    k: int
    t_solve: float
    u_ff: np.ndarray
    x_ff: np.ndarray
    z: np.ndarray
    success: bool

    @property
    def solver(self):
        """The optimizer's record of this tick's solve (``solution.solver``)."""
        return self.solution.solver


class ModelPredictiveControllerMixin:
    """
    Shared deploy / export / debug surface for MPC System family blocks.

    Its constructor holds the state both blocks share: the planner, the replan
    clock (``t0``, ``dt_mpc``), the deploy memory and the tick latch. A block
    calls it after its own ``System`` constructor, declares its ports with
    :meth:`add_mpc_ports`, defines ``tick(x, u, t)`` (the replan tick's solve
    every port reads), and overrides :meth:`z_warm_for_command` when it
    warm-starts.
    """

    # The command is a solve call, not a static law: nothing to sweep.
    plot_space = None

    def __init__(
        self,
        planner: TrajectoryOptimizationPlanner,
        *,
        dt_mpc: float,
        verbose: bool = False,
        t0: float = 0.0,
        debug: bool = False,
    ) -> None:
        # The planner, the replan clock and the debug flag
        self._planner = planner
        self._dt_mpc = float(dt_mpc)
        self._t0 = float(t0)
        self._debug = bool(debug)

        # Deploy memory: the tick counter, the last command, the debug figure and
        # the nominal cache
        self._deploy_k = 0
        self._last_command: Command | None = None
        self._debug_handles = None
        self._debug_sys = None
        self._nominal_cache: NominalCache | None = None

        # Base ticks per replan tick (a dual-rate computer sets it)
        self._replan_divisor = 1

        # One NLP per replan tick, shared by every port and the step
        self._latch = MPCTickLatch(
            planner,
            verbose=verbose,
            dt_mpc=self._dt_mpc,
            t0=self._t0,
        )

    @property
    def planner(self) -> TrajectoryOptimizationPlanner:
        """Trajopt planner owned by this block."""
        return self._planner

    @property
    def dt_mpc(self) -> float:
        """Replan period (warm-start shift and Computer schedule)."""
        return float(self._dt_mpc)

    @property
    def t0(self) -> float:
        """Absolute-time epoch: ``t_solve = t0 + k * dt_mpc``."""
        return float(self._t0)

    @property
    def last_command(self) -> Command | None:
        """Most recent :meth:`compute_command` result, if any."""
        return self._last_command

    # Deploy: one command per replan tick

    def compute_command(
        self,
        y,
        *,
        params=None,
        k: int | None = None,
        t: float | None = None,
    ) -> Command:
        """
        Deploy replan tick: measure ``y`` → NLP → :class:`Command`.

        Does **not** build the nominal interpolator; call
        :meth:`generate_nominal_interpolator` explicitly for dual-rate / broadcast.

        Parameters
        ----------
        y : array_like
            Measured plant state (diagram input ``y``).
        params : mapping, optional
            Forwarded to
            :meth:`~minilink.planning.trajectory_optimization.planner.TrajectoryOptimizationPlanner.solve_trajectory_from`
            via the tick latch (``None``/``{}`` = x0-only; ``scene`` reserved /
            NotImplemented until pipeline B).
        k : int, optional
            Tick index. Default: increment internal deploy counter.
        t : float, optional
            Absolute solve time. Default: ``t0 + k * dt_mpc``.
        """
        planner = self._planner
        latch = self._latch
        t0 = self._t0
        dt_mpc = self._dt_mpc

        # The replan tick: the deploy counter's next one, unless given
        if k is None:
            k_int = int(self._deploy_k)
            self._deploy_k = k_int + 1
        else:
            k_int = int(k)

        # The solve instant t_solve = t0 + k Δt_mpc, unless given
        if t is None:
            t_solve = float(t0 + k_int * dt_mpc)
        else:
            t_solve = float(t)

        # Gate early for clear deploy errors; the latch also forwards to the planner
        reject_unknown_online_params(params)
        y_arr = np.asarray(y, dtype=float).reshape(-1)
        z_warm = self.z_warm_for_command()

        # One NLP from the measurement, warm-started from the last command's z when
        # the block keeps it
        tick = latch.solve_for_tick(
            k_int,
            y_arr,
            z_warm=z_warm,
            dt_mpc=dt_mpc if z_warm is not None else None,
            params=params,
        )
        solution = planner.require_solution()

        # The command: the solve's first input, next state and packed z, copied
        cmd = Command(
            solution=solution,
            k=k_int,
            t_solve=t_solve,
            u_ff=np.asarray(tick.u_ff, dtype=float).copy(),
            x_ff=np.asarray(tick.x_ff, dtype=float).copy(),
            z=np.asarray(tick.z, dtype=float).copy(),
            success=bool(solution.success),
        )
        self._last_command = cmd

        # The live debug figure follows every command
        if self._debug:
            self.update_debug_figure(cmd)

        return cmd

    def get_solver_record(self):
        """
        The last NLP solve's record for deploy / logging (ROS-agnostic).

        Prefers :attr:`last_command`; otherwise the planner's
        :attr:`~minilink.planning.planner.Planner.last_solution` (e.g. after
        a hybrid tick with no deploy call). ``None`` if nothing has been
        solved yet.
        """
        if self._last_command is not None:
            return self._last_command.solver

        solution = self._planner.last_solution
        if solution is not None:
            return solution.solver

        return None

    def reset(self) -> None:
        """Clear deploy counter, last command, nominal cache, and tick latch."""
        self._deploy_k = 0
        self._last_command = None
        self._nominal_cache = None
        self._latch.reset_latch()

    # Nominal interpolator: the latched plan between replan ticks

    def generate_nominal_interpolator(
        self, *, derivatives: bool = True
    ) -> NominalCache:
        """
        Build the fast nominal cache from the last latched plan (opt-in).

        Call after :meth:`compute_command` (or a hybrid replan tick) when using
        high-rate :meth:`get_nominal_u` / dual-rate broadcast. Not invoked by
        :meth:`compute_command` itself.

        Parameters
        ----------
        derivatives : bool, optional
            If True, attach FD knot rates for ``get_nominal_*_dot``.
        """
        solution = self._planner.last_solution
        require_latched_solution(solution)

        # The solve instant: the last command's, else the latched tick's, else the
        # epoch
        if self._last_command is not None:
            t_solve = float(self._last_command.t_solve)
        else:
            t_solve = self._latch.last_t_solve
            if t_solve is None:
                t_solve = float(self._t0)

        # The plan's knots on absolute time, with their rates when asked
        cache = build_nominal_cache(
            solution.trajectory, t_solve, derivatives=bool(derivatives)
        )
        self._nominal_cache = cache

        return cache

    def get_nominal_u(self, t: float) -> np.ndarray:
        """Linear interp of latched plan ``u`` at absolute time ``t``."""
        cache = self.require_nominal_cache()
        return eval_signal(cache, t, cache.u)

    def get_nominal_x(self, t: float) -> np.ndarray:
        """Linear interp of latched plan ``x`` at absolute time ``t``."""
        cache = self.require_nominal_cache()
        return eval_signal(cache, t, cache.x)

    def get_nominal_u_dot(self, t: float) -> np.ndarray:
        """Interp of precomputed ``u`` rates (requires ``derivatives=True``)."""
        cache = self.require_nominal_cache()
        require_knot_rates(cache.u_dot, "u_dot")
        return eval_signal(cache, t, cache.u_dot)

    def get_nominal_x_dot(self, t: float) -> np.ndarray:
        """Interp of precomputed ``x`` rates (requires ``derivatives=True``)."""
        cache = self.require_nominal_cache()
        require_knot_rates(cache.x_dot, "x_dot")
        return eval_signal(cache, t, cache.x_dot)

    # Computer export

    def export_to_computer(self, schedule=None):
        """Build a single-rate :class:`~minilink.simulation.computer.Computer` (``u_ff`` ZOH)."""
        return export_mpc_to_computer(self, schedule, dt_mpc=self._dt_mpc)

    def dual_rate_computer(self, dt_broadcast: float) -> Computer:
        """
        Advanced multi-rate :class:`~minilink.simulation.computer.Computer`.

        Replan leaf at :attr:`dt_mpc`; broadcast leaf at ``dt_broadcast`` applying
        :meth:`get_nominal_u`. Requires ``dt_mpc / dt_broadcast`` to be a positive
        integer. Does not change default :meth:`export_to_computer` / ``@``.

        Leaves share this controller's latch / nominal cache (no port edge
        between them). Deploy truth remains
        :meth:`compute_command` + :meth:`generate_nominal_interpolator` +
        :meth:`get_nominal_u`.
        """
        return export_mpc_dual_rate_computer(self, dt_broadcast=float(dt_broadcast))

    def __matmul__(self, plant):
        """``mpc @ plant`` → hybrid via ``export_to_computer() @ plant`` (ZOH)."""
        return self.export_to_computer() @ plant

    def __mod__(self, schedule):
        """``mpc % schedule`` → :meth:`export_to_computer` (checks ``dt_mpc``)."""
        return self.export_to_computer(schedule)

    # Debug figure

    def init_debug_figure(self, sys, **kwargs):
        """
        Create a matplotlib figure for live plan visualization.

        Under a non-interactive backend (e.g. ``Agg``), still builds axes so
        tests can smoke the path; ``update_debug_figure`` redraws plan knots.
        """
        del kwargs
        import matplotlib.pyplot as plt

        # One axis: the latched plan against plan-local time
        fig, ax = plt.subplots(1, 1, figsize=(6, 4))
        ax.set_xlabel("plan-local time τ")
        ax.set_ylabel("state / input")
        ax.set_title("MPC debug: latched plan")

        self._debug_handles = {"fig": fig, "ax": ax, "lines": []}
        self._debug_sys = sys

        return fig

    def update_debug_figure(self, cmd: Command | None = None):
        """Redraw debug axes from ``cmd`` or :attr:`last_command`."""
        if self._debug_handles is None:
            return None

        use = cmd if cmd is not None else self._last_command
        if use is None:
            return self._debug_handles["fig"]

        # The axis cleared and titled with this tick
        ax = self._debug_handles["ax"]
        ax.cla()
        ax.set_xlabel("plan-local time τ")
        ax.set_ylabel("state / input")
        ax.set_title(f"MPC debug k={use.k} t_solve={use.t_solve:.3g}")

        # The plan's states solid, its inputs dashed
        traj = use.solution.trajectory
        for i in range(int(traj.x.shape[0])):
            ax.plot(traj.t, traj.x[i], label=f"x[{i}]")
        for i in range(int(traj.u.shape[0])):
            ax.plot(traj.t, traj.u[i], "--", label=f"u[{i}]")
        ax.legend(loc="best", fontsize=8)

        fig = self._debug_handles["fig"]
        fig.canvas.draw_idle()

        return fig

    ###########################################################################
    # Internal machinery
    ###########################################################################

    def add_mpc_ports(self, n, m, n_z) -> None:
        """The measurement ``y`` in; the plan's first input, its next state and the packed ``z`` out."""
        self.add_input_port("y", dim=n)
        self.add_output_port(
            "u_ff",
            dim=m,
            function=self.compute_u_ff,
            dependencies=("y",),
        )
        self.add_output_port(
            "x_ff",
            dim=n,
            function=self.compute_x_ff,
            dependencies=("y",),
        )
        self.add_output_port(
            "z",
            dim=n_z,
            function=self.compute_z,
            dependencies=("y",),
        )

    def compute_u_ff(self, x, u, t=0, params=None):
        """The plan's first input ``u_ff``, from the replan tick's solve."""
        del params
        return self.tick(x, u, t).u_ff

    def compute_x_ff(self, x, u, t=0, params=None):
        """The plan's next state ``x_ff``, from the replan tick's solve."""
        del params
        return self.tick(x, u, t).x_ff

    def compute_z(self, x, u, t=0, params=None):
        """The packed decision vector ``z`` of the replan tick's solve."""
        del params
        return self.tick(x, u, t).z

    def replan_k(self, k_tick) -> int:
        """Map Computer base tick to replan index (dual-rate divisor)."""
        # The divisor d: base ticks per replan tick, at least one
        d = int(self._replan_divisor)
        if d < 1:
            d = 1

        # k_replan = ⌊k / d⌋
        k_replan = int(k_tick) // d

        return k_replan

    def measurement_from_u(self, u) -> np.ndarray:
        """The measured state ``y`` from the block input."""
        return np.asarray(u, dtype=float).reshape(self.inputs["y"].dim)

    def z_warm_for_command(self):
        """The previous packed ``z`` to warm-start a deploy command (none here)."""
        return None

    def require_nominal_cache(self) -> NominalCache:
        """Refuse a nominal read before the interpolator is built; return the cache."""
        cache = self._nominal_cache
        if cache is None:
            raise RuntimeError(
                "nominal interpolator not built; call "
                "generate_nominal_interpolator() after a replan."
            )
        return cache


class MPCStatelessController(ModelPredictiveControllerMixin, System):
    """
    Algebraic MPC feedforward block (``warm_start=False`` product sibling).

    Owns ``dt_mpc`` for Computer schedule / ``@``. Outputs ``u_ff``, ``x_ff``,
    and ``z`` come from one NLP per integer tick ``k`` (latch-memoized).
    """

    def __init__(
        self,
        planner: TrajectoryOptimizationPlanner,
        *,
        dt_mpc: float,
        verbose: bool = False,
        t0: float = 0.0,
        debug: bool = False,
    ) -> None:
        n, m, n_z = validate_mpc_planner(planner)

        # Each base by name: super() would reach the mixin's constructor first
        System.__init__(self)
        self.name = "MPC Stateless Controller"

        # The planner, the replan clock, the deploy memory and the tick latch
        ModelPredictiveControllerMixin.__init__(
            self, planner, dt_mpc=dt_mpc, verbose=verbose, t0=t0, debug=debug
        )

        # Ports: the measurement y in; u_ff, x_ff and the packed z out
        self.add_mpc_ports(n, m, n_z)

    ###########################################################################
    # Internal machinery
    ###########################################################################

    def tick(self, x, u, t):
        """The replan tick's solve from the measurement, cold-started."""
        del x
        return self._latch.solve_for_tick(self.replan_k(t), self.measurement_from_u(u))


class MPCStatefulController(ModelPredictiveControllerMixin, StepSystem):
    """
    Warm-start MPC block with packed optimizer state on ``Computer.x``.

    ``warm_start=True`` product sibling. State ``x`` is packed ``z`` from the
    previous tick. Ports and :meth:`step` share one NLP per tick via the latch.
    """

    def __init__(
        self,
        planner: TrajectoryOptimizationPlanner,
        *,
        dt_mpc: float,
        verbose: bool = False,
        t0: float = 0.0,
        debug: bool = False,
    ) -> None:
        n, m, n_z = validate_mpc_planner(planner)

        # A step block on the packed decision vector z; each base by name, since
        # super() would reach the mixin's constructor first
        StepSystem.__init__(self, n_z, expose_state=False)
        self.name = "MPC Stateful Controller"

        # The planner, the replan clock, the deploy memory and the tick latch
        ModelPredictiveControllerMixin.__init__(
            self, planner, dt_mpc=dt_mpc, verbose=verbose, t0=t0, debug=debug
        )

        # The state at reset: the default collocation guess, packed
        self.x0 = mpc_default_computer_x0(planner)

        # Ports: the measurement y in; u_ff, x_ff and the packed z out
        self.add_mpc_ports(n, m, n_z)

    @property
    def latch(self) -> MPCTickLatch:
        """Per-tick solve memo (reset via :meth:`MPCTickLatch.reset_latch`)."""
        return self._latch

    def step(self, x, u, k=0, params=None):
        del params
        y = self.measurement_from_u(u)

        # z_{k+1}: the replan tick's NLP from the measurement, warm-started from z_k = x
        z_new = self._latch.solve_for_tick(
            self.replan_k(k),
            y,
            z_warm=x,
            dt_mpc=self._dt_mpc,
        ).z
        z_next = np.asarray(z_new, dtype=float).reshape(self.n).copy()

        return z_next

    ###########################################################################
    # Internal machinery
    ###########################################################################

    def tick(self, x, u, t):
        """The replan tick's solve from the measurement, warm-started from ``z = x``."""
        return self._latch.solve_for_tick(
            self.replan_k(t),
            self.measurement_from_u(u),
            z_warm=x,
            dt_mpc=self._dt_mpc,
        )

    def z_warm_for_command(self):
        """The last command's packed ``z``, to warm-start the next one."""
        if self._last_command is not None:
            return self._last_command.z
        return None


class MPCTickLatch:
    """
    Memoize one planner solve per integer replan tick ``k`` and measurement ``y``.

    Port ``compute`` paths on MPC blocks call :meth:`solve_for_tick`; the first
    call at a new ``(k, y)`` runs the NLP via
    :meth:`~TrajectoryOptimizationPlanner.solve_trajectory_from`, later calls
    with the same ``k`` and an equal ``y`` read the latch. Inside a
    :class:`~minilink.simulation.computer.Computer` tick every port sees the same
    ``y``, so the block still solves once per tick.
    """

    def __init__(
        self,
        planner: TrajectoryOptimizationPlanner,
        *,
        verbose: bool = False,
        dt_mpc: float | None = None,
        t0: float = 0.0,
    ) -> None:
        self._planner = planner
        self._verbose = bool(verbose)
        self._dt_mpc = None if dt_mpc is None else float(dt_mpc)
        self._t0 = float(t0)

        # The latched tick: its index, its measurement and its solve
        self._latch_k: int | None = None
        self._latch_y: np.ndarray | None = None
        self._latch: MPCTickSolve | None = None

        # Called after every new solve (the dual-rate nominal cache)
        self._after_solve = None

    @property
    def last_t_solve(self) -> float | None:
        """Absolute solve time of the latched tick, if any."""
        if self._latch is None:
            return None
        return float(self._latch.t_solve)

    def solve_for_tick(
        self,
        k,
        y,
        *,
        z_warm=None,
        dt_mpc=None,
        initial_guess=None,
        params=None,
    ) -> MPCTickSolve:
        """The tick's solve: the latched one at the same ``(k, y)``, else a new NLP."""
        planner = self._planner
        t0 = self._t0
        dt_default = self._dt_mpc

        k_int = int(k)
        y_arr = np.asarray(y, dtype=float).reshape(-1)

        # Same tick, same measurement: the latched solve
        if (
            self._latch is not None
            and self._latch_k == k_int
            and np.array_equal(self._latch_y, y_arr, equal_nan=True)
        ):
            return self._latch

        # The seed: the given guess, else the previous z shifted by one replan
        guess = initial_guess
        if guess is None and z_warm is not None:
            require_warm_start_period(dt_mpc)
            guess = mpc_warm_start_guess(
                z_warm,
                y_arr,
                planner,
                dt_mpc=float(dt_mpc),
                k=k_int,
            )

        # One NLP from the measured state
        traj_plan = planner.solve_trajectory_from(
            y_arr, params=params, initial_guess=guess
        )
        traj = traj_plan.trajectory
        result = planner.last_optimization_result
        require_optimization_result(result)

        # The plan's dimensions, and a next state for x_ff
        n = int(planner.problem.sys.n)
        m = int(planner.problem.sys.m)
        require_plan_next_state(traj)

        # The solve instant t_solve = t0 + k Δt_mpc (the epoch when no period is
        # known)
        dt = float(dt_mpc) if dt_mpc is not None else dt_default
        if dt is None:
            t_solve = float(t0)
        else:
            t_solve = float(t0 + k_int * dt)

        # The feedforward: the plan's first input u_ff = u(τ₀) and its next state
        # x_ff = x(τ₁)
        latch = MPCTickSolve(
            plan=traj,
            z=np.asarray(result.z, dtype=float).reshape(-1),
            u_ff=np.asarray(traj.u[:, 0], dtype=float).reshape(m),
            x_ff=np.asarray(traj.x[:, 1], dtype=float).reshape(n),
            t_solve=t_solve,
            k=k_int,
        )

        # Latch it, then call the hook and print the tick
        self._latch_k = k_int
        self._latch_y = y_arr.copy()
        self._latch = latch
        if self._after_solve is not None:
            self._after_solve()
        if self._verbose:
            self.print_tick(k_int, result)

        return latch

    def set_after_solve(self, callback) -> None:
        """Optional hook after a new NLP latch (e.g. dual-rate interpolator)."""
        self._after_solve = callback

    def reset_latch(self) -> None:
        """Clear tick memo (e.g. after ``Computer.reset``)."""
        self._latch_k = None
        self._latch_y = None
        self._latch = None

    ###########################################################################
    # Internal machinery
    ###########################################################################

    def print_tick(self, k_int, result) -> None:
        """One line per new solve: its time (or tick), success, cost and wall times."""
        planner = self._planner
        t0 = self._t0
        dt_mpc = self._dt_mpc

        # Wall times: solve is the optimizer's, step the whole from-tick call (bind +
        # solve + reconstruct)
        solve_s = result.solve_time_s
        if solve_s is None:
            solve_s = planner.last_solve_time_s
        step_s = planner.last_step_time_s
        solve_txt = "n/a" if solve_s is None else f"{float(solve_s):.3f}s"
        step_txt = "n/a" if step_s is None else f"{float(step_s):.3f}s"
        cost = result.cost
        j_txt = "n/a" if cost is None else f"{float(cost):.6g}"

        # The fire time t = t0 + k Δt_mpc, or the tick index when no period is known
        if dt_mpc is not None:
            t_fire = t0 + k_int * dt_mpc
            print(
                f"MPC @ t={t_fire:.2f}s  success={result.success}  "
                f"J={j_txt}  solve={solve_txt}  step={step_txt}"
            )
        else:
            print(
                f"MPC @ k={k_int}  success={result.success}  "
                f"J={j_txt}  solve={solve_txt}  step={step_txt}"
            )


@dataclass
class MPCTickSolve:
    """One MPC fire: reconstructed plan and feedforward slices."""

    plan: Trajectory
    z: np.ndarray
    u_ff: np.ndarray
    x_ff: np.ndarray
    t_solve: float
    k: int


def export_mpc_to_computer(
    block,
    schedule: StepSchedule | float | None = None,
    *,
    dt_mpc: float | None = None,
) -> Computer:
    """
    Build a single-rate :class:`~minilink.simulation.computer.Computer` from an MPC block.

    ``schedule`` defaults to ``dt_mpc`` (the block's own when not passed); a
    given ``schedule`` must tick at ``dt_mpc``. Once the schedule is accepted,
    the export undoes the dual-rate hooks a ``dual_rate_computer`` call left on
    the block; a rejected schedule leaves the block as it was.
    """
    # Tools import core (DESIGN §3): the simulation band loads on first use
    from minilink.simulation.computer import StepSchedule, build_computer

    # The schedule: one base tick per replan, a given one checked against the period
    block_dt = dt_mpc if dt_mpc is not None else getattr(block, "_dt_mpc", None)
    if schedule is None:
        require_replan_period(block_dt)
        schedule = StepSchedule(dt_base=float(block_dt))
    elif block_dt is not None:
        dt_sched = (
            schedule.dt_base if isinstance(schedule, StepSchedule) else float(schedule)
        )
        require_matching_period(dt_sched, block_dt)

    computer = build_computer(block, schedule)

    # Single-rate path: undo any dual-rate hooks left on the block
    if hasattr(block, "_replan_divisor"):
        block._replan_divisor = 1
    latch = getattr(block, "_latch", None)
    if latch is not None and hasattr(latch, "set_after_solve"):
        latch.set_after_solve(None)

    return computer


def export_mpc_dual_rate_computer(block, *, dt_broadcast: float) -> Computer:
    """
    Multi-rate Computer: replan at ``block.dt_mpc``, broadcast at ``dt_broadcast``.

    Boundary ports: ``y`` → replan, ``u_nom`` from broadcast (primary control).

    Coupling between ``replan`` and ``broadcast`` is a **shared latch** on
    ``block`` (``generate_nominal_interpolator`` after NLP, then
    ``get_nominal_*``) — not a diagram port. Graphviz correctly shows no edge.
    """
    # Tools import core (DESIGN §3): the simulation band loads on first use
    from minilink.core.diagram import StepDiagramSystem
    from minilink.simulation.computer import Computer, StepSchedule

    # The replan divisor d = Δt_mpc / Δt_broadcast: base ticks per replan, a positive
    # integer
    dt_mpc = float(block.dt_mpc)
    dt_b = float(dt_broadcast)
    require_positive_broadcast(dt_b)
    ratio = dt_mpc / dt_b
    d = int(round(ratio))
    require_whole_ratio(ratio, d, dt_mpc, dt_b)

    # The replan block maps base ticks to replan ticks and rebuilds the nominal
    # cache after every new solve
    block._replan_divisor = d
    block._latch.set_after_solve(
        lambda: block.generate_nominal_interpolator(derivatives=True)
    )

    # Two leaves: the replan block, and the broadcast sampling its nominal cache
    broadcast = MPCBroadcastController(block, dt_broadcast=dt_b, t0=float(block.t0))
    diagram = StepDiagramSystem()
    diagram.name = "MPC Dual-Rate"
    diagram.add_subsystem(block, "replan")
    diagram.add_subsystem(broadcast, "broadcast")

    # Boundary y → replan; u_nom / extras on the diagram
    y_port = block.inputs["y"]
    diagram.add_input_port("y", dim=y_port.dim, nominal_value=y_port.nominal_value)
    diagram.connect("input", "y", "replan", "y")
    diagram.connect_new_output_port("broadcast", "u_nom", "u_nom")
    diagram.connect_new_output_port("broadcast", "x_nom", "x_nom")
    for extra in ("u_ff", "x_ff", "z"):
        if extra in block.outputs:
            diagram.connect_new_output_port("replan", extra, extra)

    # The broadcast fires every base tick, the replan every d of them
    schedule = StepSchedule(
        dt_base=dt_b,
        fire={"replan": d, "broadcast": 1},
    )

    return Computer(diagram, schedule)


class MPCBroadcastController(System):
    """
    Sample latched nominals at Computer base ticks (no NLP).

    Absolute time is ``t0 + k * dt_broadcast`` where ``k`` is the Computer tick
    passed as the port ``t`` argument.

    Holds a reference to the replan MPC (shared ``NominalCache``) — not a
    port input. See dual-rate packaging in DESIGN.md (MPC hybrid block).
    """

    def __init__(self, mpc, *, dt_broadcast: float, t0: float = 0.0) -> None:
        super().__init__()
        self.name = "MPC Broadcast"

        # The replan block it samples, and the broadcast clock
        self._mpc = mpc
        self._dt_broadcast = float(dt_broadcast)
        self._t0 = float(t0)

        # Ports: the latched plan's input and state, sampled at every base tick
        n = int(mpc._planner.problem.sys.n)
        m = int(mpc._planner.problem.sys.m)
        self.add_output_port("u_nom", dim=m, function=self.compute_u_nom)
        self.add_output_port("x_nom", dim=n, function=self.compute_x_nom)

    ###########################################################################
    # Internal machinery
    ###########################################################################

    def abs_t(self, k_tick) -> float:
        """The absolute time of a base tick: t = t0 + k Δt_broadcast."""
        t0 = self._t0
        dt_broadcast = self._dt_broadcast

        t = float(t0 + float(k_tick) * dt_broadcast)

        return t

    _abs_t = abs_t  # ripple

    def compute_u_nom(self, x, u, t=0, params=None):
        """The latched plan's input at the base tick ``t``."""
        del x, u, params
        return self._mpc.get_nominal_u(self.abs_t(t))

    def compute_x_nom(self, x, u, t=0, params=None):
        """The latched plan's state at the base tick ``t``."""
        del x, u, params
        return self._mpc.get_nominal_x(self.abs_t(t))


###############################################################################
# Internal machinery
###############################################################################


def validate_mpc_planner(planner) -> tuple[int, int, int]:
    """Validate a plan-producing planner and return ``(n, m, n_z)``.

    Accepts a duck-typed
    :class:`~minilink.planning.trajectory_optimization.planner.TrajectoryOptimizationPlanner`
    (or compatible) with ``compile_parametric_program``,
    ``has_parametric_program``, and ``solve_trajectory_from``.

    ``compile_backend='jax'`` auto-compiles a parametric NLP once (bind + solve
    each tick). ``compile_backend='numpy'`` or ``'direct'`` skips parametric
    compile; each tick rebuilds via :meth:`~TrajectoryOptimizationPlanner.solve_trajectory_from`.
    """
    # The planner's interface: a parametric compile and a solve from a measured state
    missing = [
        name
        for name in (
            "compile_parametric_program",
            "has_parametric_program",
            "solve_trajectory_from",
        )
        if not hasattr(planner, name)
    ]
    if missing:
        raise TypeError(
            "MPC block requires a TrajectoryOptimizationPlanner-compatible "
            f"planner with {missing}; got {type(planner).__name__}."
        )

    # At least two knots: x_ff is the plan's next state
    n_steps = int(planner.transcription.options.n_steps)
    if n_steps < 2:
        raise ValueError(
            f"MPC block requires transcription n_steps >= 2 for x_ff, got {n_steps}"
        )

    # A JAX planner compiles its parametric NLP once, here
    if (
        normalize_backend(planner.options.compile_backend, allow_direct=True)
        == BACKEND_JAX
    ):
        if not planner.has_parametric_program:
            planner.compile_parametric_program()

        if not planner.has_parametric_program:
            raise RuntimeError(
                "planner.compile_parametric_program() did not produce a parametric "
                "program (check compile_backend='jax' and transcription support)."
            )

    # The block's sizes: the plant's state and input, the packed decision vector
    sys = planner.problem.sys
    n = int(sys.n)
    m = int(sys.m)
    n_z = int(planner.transcription.decision_dimension(planner.problem))

    return n, m, n_z


def require_latched_solution(solution):
    """Refuse a nominal cache before any solve has latched a plan."""
    if solution is None:
        raise RuntimeError(
            "generate_nominal_interpolator requires a latched solution; "
            "call compute_command (or run a replan tick) first."
        )


def require_knot_rates(rates, name):
    """Refuse a rate read from a cache built without its rates (``derivatives=False``)."""
    if rates is None:
        raise RuntimeError(
            f"{name} not in cache; call generate_nominal_interpolator(derivatives=True)."
        )


def require_warm_start_period(dt_mpc):
    """Refuse a warm start without the replan period it shifts the plan by."""
    if dt_mpc is None:
        raise ValueError("dt_mpc is required when z_warm is provided")


def require_optimization_result(result):
    """Refuse a solve that left no optimization result on the planner."""
    if result is None:
        raise RuntimeError(
            "solve_trajectory_from did not store last_optimization_result"
        )


def require_plan_next_state(traj):
    """Refuse a plan of fewer than two samples: ``x_ff`` is its second state."""
    if traj.n_samples < 2:
        raise RuntimeError(
            "MPC plan must have at least two samples for x_ff = plan.x[:, 1]"
        )


def require_replan_period(block_dt):
    """Refuse a default schedule without a replan period to tick at."""
    if block_dt is None:
        raise ValueError(
            "schedule is required when the block has no dt_mpc; "
            "pass a schedule or dt_mpc=..."
        )


def require_matching_period(dt_sched, block_dt):
    """Refuse a schedule that does not tick at the replan period (10⁻¹² of slack)."""
    if abs(dt_sched - float(block_dt)) > 1e-12:
        raise ValueError(
            f"schedule dt_base={dt_sched} does not match block dt_mpc={block_dt}"
        )


def require_positive_broadcast(dt_b):
    """Refuse a broadcast period that is not positive."""
    if dt_b <= 0.0:
        raise ValueError(f"dt_broadcast must be positive, got {dt_b}")


def require_whole_ratio(ratio, d, dt_mpc, dt_b):
    """Refuse a replan period that is not a whole number of broadcast periods (10⁻⁹ of slack)."""
    if d < 1 or abs(ratio - d) > 1e-9:
        raise ValueError(
            f"dt_mpc / dt_broadcast must be a positive integer "
            f"(got dt_mpc={dt_mpc}, dt_broadcast={dt_b}, ratio={ratio})"
        )
