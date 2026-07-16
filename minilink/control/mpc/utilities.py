"""MPC pure helpers: warm-start guesses and nominal plan cache."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from minilink.core.trajectory import Trajectory
from minilink.planning.initial_guess import default_initial_trajectory
from minilink.planning.results import TrajectoryPlan
from minilink.planning.trajectory_optimization.planner import (
    TrajectoryOptimizationPlanner,
)


def _shift_plan_trajectory(
    plan: Trajectory,
    x_meas,
    *,
    dt_shift: float,
    horizon: float,
    t_anchor: float = 0.0,
) -> Trajectory | None:
    """
    Shift a previous MPC plan forward by ``dt_shift`` for warm-starting.

    Mirrors a hand-loop ``compute_command`` + RK4 ZOH deploy pattern.
    Returns ``None`` when the shifted window has fewer than three samples.
    """
    if plan.n_samples < 3:
        return None

    t_shift = np.asarray(plan.t, dtype=float).reshape(-1) + float(dt_shift)
    mask = t_shift <= float(horizon) + 1e-9
    if np.count_nonzero(mask) < 3:
        return None

    x_guess = np.asarray(plan.x[:, mask], dtype=float).copy()
    x_guess[:, 0] = np.asarray(x_meas, dtype=float).reshape(-1)
    return Trajectory(
        t=t_shift[mask] - float(t_anchor),
        x=x_guess,
        u=np.asarray(plan.u[:, mask], dtype=float),
    )


def mpc_warm_start_guess(
    z_prev,
    y,
    planner: TrajectoryOptimizationPlanner,
    *,
    dt_mpc: float,
    k: int = 0,
) -> Trajectory:
    """
    Build an ``initial_guess`` Trajectory for
    :meth:`~TrajectoryOptimizationPlanner.solve_trajectory_from`.

    At tick ``k == 0`` (or when shift fails), returns the default collocation guess.
    Otherwise unpacks ``z_prev``, shifts on the transcription grid, and pins the
    first state node to the current measurement ``y``.
    """
    problem = planner.problem
    transcription = planner.transcription
    t_grid = transcription.initial_guess_time_grid(problem)
    horizon = problem.require_finite_tf()

    if int(k) <= 0:
        return default_initial_trajectory(problem, t_grid)

    if z_prev is None:
        return default_initial_trajectory(problem, t_grid)

    z_arr = np.asarray(z_prev, dtype=float).reshape(-1)
    x_mat, u_mat = transcription.unpack(z_arr, problem)
    plan = Trajectory(
        t=np.asarray(transcription.options.t(problem), dtype=float).reshape(-1),
        x=x_mat,
        u=u_mat,
    )
    t_anchor = float(k) * float(dt_mpc)
    shifted = _shift_plan_trajectory(
        plan,
        y,
        dt_shift=dt_mpc,
        horizon=horizon,
        t_anchor=t_anchor,
    )
    if shifted is None:
        return default_initial_trajectory(problem, t_grid)
    return shifted


def warm_start_guess_from_prev_plan(
    prev_plan: Trajectory | None,
    x_meas,
    planner: TrajectoryOptimizationPlanner,
    *,
    dt_shift: float,
    horizon: float,
    t_anchor: float,
) -> Trajectory:
    """
    Warm-start guess from a previous solved plan (manual MPC demo contract).

    Mirrors a hand-loop ``compute_command`` + RK4 ZOH deploy pattern.
    lines 122–137: shift ``prev_plan`` by ``dt_shift``, pin ``x[:, 0] = x_meas``,
    re-time with ``t_anchor`` (simulation time at the MPC fire).
    """
    problem = planner.problem
    t_grid = planner.transcription.initial_guess_time_grid(problem)
    if prev_plan is not None and prev_plan.n_samples >= 3:
        shifted = _shift_plan_trajectory(
            prev_plan,
            x_meas,
            dt_shift=dt_shift,
            horizon=horizon,
            t_anchor=t_anchor,
        )
        if shifted is not None:
            return shifted
    return default_initial_trajectory(problem, t_grid)


def mpc_default_computer_x0(planner: TrajectoryOptimizationPlanner) -> np.ndarray:
    """Packed default decision vector for ``Computer.reset`` / ``x0_computer``."""
    problem = planner.problem
    guess = default_initial_trajectory(
        problem,
        planner.transcription.initial_guess_time_grid(problem),
    )
    return np.asarray(
        planner.transcription.pack_initial_guess(problem, guess),
        dtype=float,
    ).reshape(-1)


@dataclass
class NominalCache:
    """Plan-local interpolant data built after an NLP replan."""

    t_solve: float
    tau: np.ndarray
    x: np.ndarray
    u: np.ndarray
    x_dot: np.ndarray | None = None
    u_dot: np.ndarray | None = None

    @property
    def tau_end(self) -> float:
        return float(self.tau[-1]) if self.tau.size else 0.0


def _finite_diff_knots(tau: np.ndarray, y: np.ndarray) -> np.ndarray:
    """Central / one-sided FD of ``y`` (n, N) vs ``tau`` (N,)."""
    n, n_t = y.shape
    out = np.zeros_like(y)
    if n_t == 1:
        return out
    dtau = np.diff(tau)
    dtau = np.where(np.abs(dtau) < 1e-15, 1e-15, dtau)
    # Forward / backward at ends; central interior.
    out[:, 0] = (y[:, 1] - y[:, 0]) / dtau[0]
    out[:, -1] = (y[:, -1] - y[:, -2]) / dtau[-1]
    if n_t > 2:
        for i in range(1, n_t - 1):
            out[:, i] = (y[:, i + 1] - y[:, i - 1]) / (tau[i + 1] - tau[i - 1])
    return out


def build_nominal_cache(
    plan: TrajectoryPlan,
    t_solve: float,
    *,
    derivatives: bool = True,
) -> NominalCache:
    """
    Build a :class:`NominalCache` from a latched traj plan.

    Parameters
    ----------
    plan : TrajectoryPlan
        Latched solve result (plan-local ``trajectory.t``).
    t_solve : float
        Absolute solve time (τ = 0).
    derivatives : bool, optional
        If True, attach FD knot rates ``x_dot`` / ``u_dot``.
    """
    traj = plan.trajectory
    tau = np.asarray(traj.t, dtype=float).reshape(-1)
    x = np.asarray(traj.x, dtype=float)
    u = np.asarray(traj.u, dtype=float)
    if tau.size < 1:
        raise ValueError("plan trajectory must have at least one sample")
    x_dot = _finite_diff_knots(tau, x) if derivatives else None
    u_dot = _finite_diff_knots(tau, u) if derivatives else None
    return NominalCache(
        t_solve=float(t_solve),
        tau=tau.copy(),
        x=x.copy(),
        u=u.copy(),
        x_dot=None if x_dot is None else x_dot.copy(),
        u_dot=None if u_dot is None else u_dot.copy(),
    )


def _clamp_tau(cache: NominalCache, t: float) -> float:
    """Absolute ``t`` → plan-local τ clamped to ``[0, T]``."""
    tau = float(t) - float(cache.t_solve)
    return float(np.clip(tau, 0.0, cache.tau_end))


def eval_signal(cache: NominalCache, t: float, signal: np.ndarray) -> np.ndarray:
    """Linear interp of rows of ``signal`` (n, N) at absolute ``t``."""
    tau = _clamp_tau(cache, t)
    rows = []
    for i in range(int(signal.shape[0])):
        rows.append(np.interp(tau, cache.tau, signal[i]))
    return np.asarray(rows, dtype=float).reshape(-1)
