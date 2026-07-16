"""
ModelPredictiveController — product MPC System family (control/mpc).

Factory, mixin, algebraic / warm-start backends, Command, tick latch,
Computer export, and dual-rate broadcast leaf.
"""

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
from minilink.core.system import StepSystem, System
from minilink.core.trajectory import Trajectory
from minilink.planning.results import SolveMetadata, TrajectoryPlan
from minilink.planning.trajectory_optimization.planner import (
    TrajectoryOptimizationPlanner,
    reject_unknown_online_params,
)

if TYPE_CHECKING:
    from minilink.simulation.computer import Computer, StepSchedule


# Public API — Command


@dataclass(frozen=True)
class Command:
    """
    One replan tick result for deploy or debugging.

    ``plan.trajectory.t`` is plan-local time with zero at the solve instant.
    Absolute solve time is ``t_solve``.
    """

    plan: TrajectoryPlan
    k: int
    t_solve: float
    u_ff: np.ndarray
    x_ff: np.ndarray
    z: np.ndarray
    success: bool

    @property
    def plan_flat(self) -> np.ndarray:
        """Flattened ``(t, x, u)`` from :meth:`TrajectoryPlan.to_flat`."""
        return self.plan.to_flat()

    @property
    def metadata(self) -> SolveMetadata:
        """Solve extras for this tick (alias of ``plan.metadata``)."""
        return self.plan.metadata


# Public API — factory + mixin


class ModelPredictiveControllerMixin:
    """
    Shared deploy / export / debug surface for MPC System family blocks.

    Concrete classes provide ``_planner``, ``_latch``, ``_dt_mpc``, ``_t0``,
    ``_debug``, and optional warm-start via ``_z_warm_for_command``.
    """

    _planner: TrajectoryOptimizationPlanner
    _latch: object
    _dt_mpc: float
    _t0: float
    _debug: bool
    _deploy_k: int
    _last_command: Command | None
    _debug_handles: object | None
    _debug_sys: object | None
    _nominal_cache: NominalCache | None
    _replan_divisor: int

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

    def _replan_k(self, k_tick) -> int:
        """Map Computer base tick to replan index (dual-rate divisor)."""
        d = int(self._replan_divisor)
        if d < 1:
            d = 1
        return int(k_tick) // d

    def get_solve_metadata(self) -> SolveMetadata | None:
        """
        Last NLP solve extras for deploy / logging (ROS-agnostic).

        Prefers :attr:`last_command` metadata; otherwise the planner's
        :attr:`~minilink.planning.planner.Planner.last_trajectory_plan`
        (e.g. after a hybrid tick with no deploy call). ``None`` if nothing
        has been solved yet.
        """
        if self._last_command is not None:
            return self._last_command.metadata
        plan = self._planner.last_trajectory_plan
        if plan is not None:
            return plan.metadata
        return None

    def reset(self) -> None:
        """Clear deploy counter, last command, nominal cache, and tick latch."""
        self._deploy_k = 0
        self._last_command = None
        self._nominal_cache = None
        self._latch.reset_latch()

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
        plan = self._planner.last_trajectory_plan
        if plan is None:
            raise RuntimeError(
                "generate_nominal_interpolator requires a latched plan; "
                "call compute_command (or run a replan tick) first."
            )
        if self._last_command is not None:
            t_solve = float(self._last_command.t_solve)
        else:
            t_solve = self._latch.last_t_solve
            if t_solve is None:
                t_solve = float(self._t0)
        cache = build_nominal_cache(plan, t_solve, derivatives=bool(derivatives))
        self._nominal_cache = cache
        return cache

    def _require_nominal_cache(self) -> NominalCache:
        cache = self._nominal_cache
        if cache is None:
            raise RuntimeError(
                "nominal interpolator not built; call "
                "generate_nominal_interpolator() after a replan."
            )
        return cache

    def get_nominal_u(self, t: float) -> np.ndarray:
        """Linear interp of latched plan ``u`` at absolute time ``t``."""
        cache = self._require_nominal_cache()
        return eval_signal(cache, t, cache.u)

    def get_nominal_x(self, t: float) -> np.ndarray:
        """Linear interp of latched plan ``x`` at absolute time ``t``."""
        cache = self._require_nominal_cache()
        return eval_signal(cache, t, cache.x)

    def get_nominal_u_dot(self, t: float) -> np.ndarray:
        """Interp of precomputed ``u`` rates (requires ``derivatives=True``)."""
        cache = self._require_nominal_cache()
        if cache.u_dot is None:
            raise RuntimeError(
                "u_dot not in cache; call generate_nominal_interpolator(derivatives=True)."
            )
        return eval_signal(cache, t, cache.u_dot)

    def get_nominal_x_dot(self, t: float) -> np.ndarray:
        """Interp of precomputed ``x`` rates (requires ``derivatives=True``)."""
        cache = self._require_nominal_cache()
        if cache.x_dot is None:
            raise RuntimeError(
                "x_dot not in cache; call generate_nominal_interpolator(derivatives=True)."
            )
        return eval_signal(cache, t, cache.x_dot)

    def _z_warm_for_command(self):
        """Override on StepSystem: previous packed ``z`` for warm-start."""
        return None

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
        if k is None:
            k_int = int(self._deploy_k)
            self._deploy_k = k_int + 1
        else:
            k_int = int(k)

        if t is None:
            t_solve = float(self._t0 + k_int * self._dt_mpc)
        else:
            t_solve = float(t)

        # Gate early for clear deploy errors; latch also forwards to the planner.
        reject_unknown_online_params(params)
        y_arr = np.asarray(y, dtype=float).reshape(-1)
        z_warm = self._z_warm_for_command()
        tick = self._latch.solve_for_tick(
            k_int,
            y_arr,
            z_warm=z_warm,
            dt_mpc=self._dt_mpc if z_warm is not None else None,
            params=params,
        )
        plan = self._planner.require_trajectory_plan()
        cmd = Command(
            plan=plan,
            k=k_int,
            t_solve=t_solve,
            u_ff=np.asarray(tick.u_ff, dtype=float).copy(),
            x_ff=np.asarray(tick.x_ff, dtype=float).copy(),
            z=np.asarray(tick.z, dtype=float).copy(),
            success=bool(plan.metadata.success),
        )
        self._last_command = cmd
        if self._debug:
            self.update_debug_figure(cmd)
        return cmd

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

    def init_debug_figure(self, sys, **kwargs):
        """
        Create a matplotlib figure for live plan visualization.

        Under a non-interactive backend (e.g. ``Agg``), still builds axes so
        tests can smoke the path; ``update_debug_figure`` redraws plan knots.
        """
        del kwargs
        import matplotlib.pyplot as plt

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

        ax = self._debug_handles["ax"]
        ax.cla()
        ax.set_xlabel("plan-local time τ")
        ax.set_ylabel("state / input")
        ax.set_title(f"MPC debug k={use.k} t_solve={use.t_solve:.3g}")
        traj = use.plan.trajectory
        for i in range(int(traj.x.shape[0])):
            ax.plot(traj.t, traj.x[i], label=f"x[{i}]")
        for i in range(int(traj.u.shape[0])):
            ax.plot(traj.t, traj.u[i], "--", label=f"u[{i}]")
        ax.legend(loc="best", fontsize=8)
        fig = self._debug_handles["fig"]
        fig.canvas.draw_idle()
        return fig


def ModelPredictiveController(
    planner: TrajectoryOptimizationPlanner,
    *,
    dt_mpc: float,
    warm_start: bool = True,
    step_disp: bool = False,
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
    step_disp, t0, debug
        Latch printouts, absolute-time epoch, and live debug figure auto-update.
    """
    if warm_start:
        return MPCStatefulController(
            planner,
            dt_mpc=dt_mpc,
            step_disp=step_disp,
            t0=t0,
            debug=debug,
        )
    return MPCStatelessController(
        planner,
        dt_mpc=dt_mpc,
        step_disp=step_disp,
        t0=t0,
        debug=debug,
    )


# System backends


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
        step_disp: bool = False,
        t0: float = 0.0,
        debug: bool = False,
    ) -> None:
        n, m, n_z = validate_mpc_planner(planner)

        super().__init__()
        self.name = "MPC Stateless Controller"
        self._planner = planner
        self._dt_mpc = float(dt_mpc)
        self._t0 = float(t0)
        self._debug = bool(debug)
        self._deploy_k = 0
        self._last_command = None
        self._debug_handles = None
        self._debug_sys = None
        self._nominal_cache = None
        self._replan_divisor = 1
        self._latch = MPCTickLatch(
            planner,
            step_disp=step_disp,
            dt_mpc=self._dt_mpc,
            t0=self._t0,
        )

        self.add_input_port("y", dim=n)
        self.add_output_port(
            "u_ff",
            dim=m,
            function=self._compute_u_ff,
            dependencies=("y",),
        )
        self.add_output_port(
            "x_ff",
            dim=n,
            function=self._compute_x_ff,
            dependencies=("y",),
        )
        self.add_output_port(
            "z",
            dim=n_z,
            function=self._compute_z,
            dependencies=("y",),
        )

    @property
    def planner(self) -> TrajectoryOptimizationPlanner:
        """Compiled trajopt planner (parametric NLP)."""
        return self._planner

    def _measurement(self, u) -> np.ndarray:
        return np.asarray(u, dtype=float).reshape(self.inputs["y"].dim)

    def _compute_u_ff(self, x, u, t=0, params=None):
        del x, params
        return self._latch.solve_for_tick(self._replan_k(t), self._measurement(u)).u_ff

    def _compute_x_ff(self, x, u, t=0, params=None):
        del x, params
        return self._latch.solve_for_tick(self._replan_k(t), self._measurement(u)).x_ff

    def _compute_z(self, x, u, t=0, params=None):
        del x, params
        return self._latch.solve_for_tick(self._replan_k(t), self._measurement(u)).z


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
        step_disp: bool = False,
        t0: float = 0.0,
        debug: bool = False,
    ) -> None:
        n, m, n_z = validate_mpc_planner(planner)
        super().__init__(n_z, expose_state=False)
        self.name = "MPC Stateful Controller"
        self._planner = planner
        self._dt_mpc = float(dt_mpc)
        self._t0 = float(t0)
        self._debug = bool(debug)
        self._deploy_k = 0
        self._last_command = None
        self._debug_handles = None
        self._debug_sys = None
        self._nominal_cache = None
        self._replan_divisor = 1
        self._latch = MPCTickLatch(
            planner,
            step_disp=step_disp,
            dt_mpc=self._dt_mpc,
            t0=self._t0,
        )
        self.x0 = mpc_default_computer_x0(planner)

        self.add_input_port("y", dim=n)
        self.add_output_port(
            "u_ff",
            dim=m,
            function=self._compute_u_ff,
            dependencies=("y",),
        )
        self.add_output_port(
            "x_ff",
            dim=n,
            function=self._compute_x_ff,
            dependencies=("y",),
        )
        self.add_output_port(
            "z",
            dim=n_z,
            function=self._compute_z,
            dependencies=("y",),
        )

    @property
    def planner(self) -> TrajectoryOptimizationPlanner:
        """Compiled trajopt planner (parametric NLP)."""
        return self._planner

    @property
    def latch(self) -> MPCTickLatch:
        """Per-tick solve memo (reset via :meth:`MPCTickLatch.reset_latch`)."""
        return self._latch

    def _z_warm_for_command(self):
        if self._last_command is not None:
            return self._last_command.z
        return None

    def _measurement(self, u) -> np.ndarray:
        return np.asarray(u, dtype=float).reshape(self.inputs["y"].dim)

    def _compute_u_ff(self, x, u, t=0, params=None):
        del params
        return self._latch.solve_for_tick(
            self._replan_k(t),
            self._measurement(u),
            z_warm=x,
            dt_mpc=self._dt_mpc,
        ).u_ff

    def _compute_x_ff(self, x, u, t=0, params=None):
        del params
        return self._latch.solve_for_tick(
            self._replan_k(t),
            self._measurement(u),
            z_warm=x,
            dt_mpc=self._dt_mpc,
        ).x_ff

    def _compute_z(self, x, u, t=0, params=None):
        del params
        return self._latch.solve_for_tick(
            self._replan_k(t),
            self._measurement(u),
            z_warm=x,
            dt_mpc=self._dt_mpc,
        ).z

    def step(self, x, u, k=0, params=None):
        del params
        y = self._measurement(u)
        z_new = self._latch.solve_for_tick(
            self._replan_k(k),
            y,
            z_warm=x,
            dt_mpc=self._dt_mpc,
        ).z
        return np.asarray(z_new, dtype=float).reshape(self.n).copy()


# Tick latch


@dataclass
class MPCTickSolve:
    """One MPC fire: reconstructed plan and feedforward slices."""

    plan: Trajectory
    z: np.ndarray
    u_ff: np.ndarray
    x_ff: np.ndarray
    t_solve: float
    k: int


class MPCTickLatch:
    """
    Memoize one planner solve per integer replan tick ``k``.

    Port ``compute`` paths on MPC blocks call :meth:`solve_for_tick`; the first
    call at a new ``k`` runs the NLP via
    :meth:`~TrajectoryOptimizationPlanner.solve_trajectory_from`, later calls
    at the same ``k`` read the latch.
    """

    def __init__(
        self,
        planner: TrajectoryOptimizationPlanner,
        *,
        step_disp: bool = False,
        dt_mpc: float | None = None,
        t0: float = 0.0,
    ) -> None:
        self._planner = planner
        self._step_disp = bool(step_disp)
        self._dt_mpc = None if dt_mpc is None else float(dt_mpc)
        self._t0 = float(t0)
        self._latch_k: int | None = None
        self._latch: MPCTickSolve | None = None
        self._after_solve = None

    @property
    def last_t_solve(self) -> float | None:
        """Absolute solve time of the latched tick, if any."""
        if self._latch is None:
            return None
        return float(self._latch.t_solve)

    def set_after_solve(self, callback) -> None:
        """Optional hook after a new NLP latch (e.g. dual-rate interpolator)."""
        self._after_solve = callback

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
        k_int = int(k)
        if self._latch_k == k_int and self._latch is not None:
            return self._latch

        y_arr = np.asarray(y, dtype=float).reshape(-1)
        guess = initial_guess
        if guess is None and z_warm is not None:
            if dt_mpc is None:
                raise ValueError("dt_mpc is required when z_warm is provided")
            guess = mpc_warm_start_guess(
                z_warm,
                y_arr,
                self._planner,
                dt_mpc=float(dt_mpc),
                k=k_int,
            )

        traj_plan = self._planner.solve_trajectory_from(
            y_arr, params=params, initial_guess=guess
        )
        traj = traj_plan.trajectory
        result = self._planner.last_optimization_result
        if result is None:
            raise RuntimeError(
                "solve_trajectory_from did not store last_optimization_result"
            )

        n = int(self._planner.problem.sys.n)
        m = int(self._planner.problem.sys.m)
        if traj.n_samples < 2:
            raise RuntimeError(
                "MPC plan must have at least two samples for x_ff = plan.x[:, 1]"
            )

        dt = float(dt_mpc) if dt_mpc is not None else self._dt_mpc
        if dt is None:
            t_solve = float(self._t0)
        else:
            t_solve = float(self._t0 + k_int * dt)

        latch = MPCTickSolve(
            plan=traj,
            z=np.asarray(result.z, dtype=float).reshape(-1),
            u_ff=np.asarray(traj.u[:, 0], dtype=float).reshape(m),
            x_ff=np.asarray(traj.x[:, 1], dtype=float).reshape(n),
            t_solve=t_solve,
            k=k_int,
        )
        self._latch_k = k_int
        self._latch = latch
        if self._after_solve is not None:
            self._after_solve()
        if self._step_disp:
            self._print_step_disp(k_int, result)
        return latch

    def _print_step_disp(self, k_int, result) -> None:
        solve_s = result.solve_time_s
        if solve_s is None:
            solve_s = self._planner.last_solve_time_s
        step_s = self._planner.last_step_time_s
        solve_txt = "n/a" if solve_s is None else f"{float(solve_s):.3f}s"
        step_txt = "n/a" if step_s is None else f"{float(step_s):.3f}s"
        if self._dt_mpc is not None:
            t_fire = self._t0 + k_int * self._dt_mpc
            print(
                f"MPC @ t={t_fire:.2f}s  success={result.success}  "
                f"solve={solve_txt}  step={step_txt}"
            )
        else:
            print(
                f"MPC @ k={k_int}  success={result.success}  "
                f"solve={solve_txt}  step={step_txt}"
            )

    def reset_latch(self) -> None:
        """Clear tick memo (e.g. after ``Computer.reset``)."""
        self._latch_k = None
        self._latch = None


# Export helpers


def validate_mpc_planner(planner) -> tuple[int, int, int]:
    """Validate a plan-producing planner and return ``(n, m, n_z)``.

    Accepts a duck-typed
    :class:`~minilink.planning.trajectory_optimization.planner.TrajectoryOptimizationPlanner`
    (or compatible) with ``compile_parametric_program``,
    ``has_parametric_program``, and ``solve_trajectory_from``. Auto-compiles
    when needed so MPC ticks never re-transcribe.
    """
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

    n_steps = int(planner.transcription.options.n_steps)
    if n_steps < 2:
        raise ValueError(
            f"MPC block requires transcription n_steps >= 2 for x_ff, got {n_steps}"
        )

    if not planner.has_parametric_program:
        planner.compile_parametric_program()

    if not planner.has_parametric_program:
        raise RuntimeError(
            "planner.compile_parametric_program() did not produce a parametric "
            "program (check compile_backend='jax' and transcription support)."
        )

    sys = planner.problem.sys
    n = int(sys.n)
    m = int(sys.m)
    n_z = int(planner.transcription.decision_dimension(planner.problem))
    return n, m, n_z


def export_mpc_to_computer(
    block,
    schedule: "StepSchedule | float | None" = None,
    *,
    dt_mpc: float | None = None,
) -> "Computer":
    """
    Build a :class:`~minilink.simulation.computer.Computer` from an MPC block.

    Warm-start blocks (``ModelPredictiveController(..., warm_start=True)``)
    default ``schedule`` from ``dt_mpc``. Algebraic (``warm_start=False``)
    blocks require an explicit ``schedule``.
    """
    from minilink.simulation.computer import StepSchedule, as_computer

    # Single-rate path: undo any dual-rate hooks left on the block.
    if hasattr(block, "_replan_divisor"):
        block._replan_divisor = 1
    latch = getattr(block, "_latch", None)
    if latch is not None and hasattr(latch, "set_after_solve"):
        latch.set_after_solve(None)

    block_dt = dt_mpc if dt_mpc is not None else getattr(block, "_dt_mpc", None)
    if schedule is None:
        if block_dt is None:
            raise ValueError(
                "schedule is required for algebraic ModelPredictiveController "
                "(warm_start=False); pass export_to_computer(dt_mpc) or use mpc % dt"
            )
        schedule = StepSchedule(dt_base=float(block_dt))
    elif block_dt is not None:
        dt_sched = (
            schedule.dt_base if isinstance(schedule, StepSchedule) else float(schedule)
        )
        if abs(dt_sched - float(block_dt)) > 1e-12:
            raise ValueError(
                f"schedule dt_base={dt_sched} does not match block dt_mpc={block_dt}"
            )
    return as_computer(block, schedule)


def export_mpc_dual_rate_computer(block, *, dt_broadcast: float) -> "Computer":
    """
    Multi-rate Computer: replan at ``block.dt_mpc``, broadcast at ``dt_broadcast``.

    Boundary ports: ``y`` → replan, ``u_nom`` from broadcast (primary control).

    Coupling between ``replan`` and ``broadcast`` is a **shared latch** on
    ``block`` (``generate_nominal_interpolator`` after NLP, then
    ``get_nominal_*``) — not a diagram port. Graphviz correctly shows no edge.
    """
    from minilink.core.diagram import StepDiagramSystem
    from minilink.simulation.computer import Computer, StepSchedule

    dt_mpc = float(block.dt_mpc)
    dt_b = float(dt_broadcast)
    if dt_b <= 0.0:
        raise ValueError(f"dt_broadcast must be positive, got {dt_b}")
    ratio = dt_mpc / dt_b
    d = int(round(ratio))
    if d < 1 or abs(ratio - d) > 1e-9:
        raise ValueError(
            f"dt_mpc / dt_broadcast must be a positive integer "
            f"(got dt_mpc={dt_mpc}, dt_broadcast={dt_b}, ratio={ratio})"
        )

    block._replan_divisor = d
    block._latch.set_after_solve(
        lambda: block.generate_nominal_interpolator(derivatives=True)
    )

    broadcast = MPCBroadcastController(block, dt_broadcast=dt_b, t0=float(block.t0))
    diagram = StepDiagramSystem()
    diagram.name = "MPC Dual-Rate"
    diagram.add_subsystem(block, "replan")
    diagram.add_subsystem(broadcast, "broadcast")

    # Boundary y → replan; u_nom / extras on the diagram.
    y_port = block.inputs["y"]
    diagram.add_input_port("y", dim=y_port.dim, nominal_value=y_port.nominal_value)
    diagram.connect("input", "y", "replan", "y")
    diagram.connect_new_output_port("broadcast", "u_nom", "u_nom")
    diagram.connect_new_output_port("broadcast", "x_nom", "x_nom")
    for extra in ("u_ff", "x_ff", "z"):
        if extra in block.outputs:
            diagram.connect_new_output_port("replan", extra, extra)

    schedule = StepSchedule(
        dt_base=dt_b,
        fire={"replan": d, "broadcast": 1},
    )
    return Computer(diagram, schedule)


# Dual-rate broadcast leaf


class MPCBroadcastController(System):
    """
    Sample latched nominals at Computer base ticks (no NLP).

    Absolute time is ``t0 + k * dt_broadcast`` where ``k`` is the Computer tick
    passed as the port ``t`` argument.

    Holds a reference to the replan MPC (shared ``NominalCache``) — not a
    port input. See dual-rate packaging option A in DESIGN / phase-E8.
    """

    def __init__(self, mpc, *, dt_broadcast: float, t0: float = 0.0) -> None:
        super().__init__()
        self.name = "MPC Broadcast"
        self._mpc = mpc
        self._dt_broadcast = float(dt_broadcast)
        self._t0 = float(t0)
        n = int(mpc._planner.problem.sys.n)
        m = int(mpc._planner.problem.sys.m)
        self.add_output_port("u_nom", dim=m, function=self._compute_u_nom)
        self.add_output_port("x_nom", dim=n, function=self._compute_x_nom)

    def _abs_t(self, k_tick) -> float:
        return float(self._t0 + float(k_tick) * self._dt_broadcast)

    def _compute_u_nom(self, x, u, t=0, params=None):
        del x, u, params
        return self._mpc.get_nominal_u(self._abs_t(t))

    def _compute_x_nom(self, x, u, t=0, params=None):
        del x, u, params
        return self._mpc.get_nominal_x(self._abs_t(t))
