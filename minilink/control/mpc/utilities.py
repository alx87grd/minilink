"""Receding-horizon bookkeeping: the warm-start guesses and the nominal plan cache of MPC."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from minilink.core.trajectory import Trajectory
from minilink.planning.initial_guess import default_initial_trajectory
from minilink.planning.trajectory_optimization.planner import (
    TrajectoryOptimizationPlanner,
)

###############################################################################
# Public API
###############################################################################


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

    # The first tick, or no previous solution: the default collocation guess
    if int(k) <= 0 or z_prev is None:
        return default_initial_trajectory(problem, t_grid)

    # The previous solution as a plan on the transcription grid
    z_arr = np.asarray(z_prev, dtype=float).reshape(-1)
    x_mat, u_mat = transcription.unpack(z_arr, problem)
    t_plan = np.asarray(transcription.options.t(problem), dtype=float).reshape(-1)
    plan = Trajectory(t=t_plan, x=x_mat, u=u_mat)

    # Shifted one MPC period ahead, re-timed from the tick k Δt, its first node at y
    t_anchor = float(k) * float(dt_mpc)
    shifted = shift_plan_trajectory(
        plan,
        y,
        dt_shift=dt_mpc,
        horizon=horizon,
        t_anchor=t_anchor,
    )

    # Too short a shifted window: the default collocation guess
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

    Mirrors a hand-loop ``compute_command`` + RK4 ZOH deploy pattern: shift
    ``prev_plan`` by ``dt_shift``, pin ``x[:, 0] = x_meas``, re-time with
    ``t_anchor`` (simulation time at the MPC fire). Without a previous plan of
    at least three samples, or when the shifted window is too short, returns
    the default collocation guess.
    """
    problem = planner.problem
    t_grid = planner.transcription.initial_guess_time_grid(problem)

    # The previous plan shifted by dt_shift, its first node at the measurement
    if prev_plan is not None and prev_plan.n_samples >= 3:
        shifted = shift_plan_trajectory(
            prev_plan,
            x_meas,
            dt_shift=dt_shift,
            horizon=horizon,
            t_anchor=t_anchor,
        )
        if shifted is not None:
            return shifted

    # No usable previous plan: the default collocation guess
    return default_initial_trajectory(problem, t_grid)


def mpc_default_computer_x0(planner: TrajectoryOptimizationPlanner) -> np.ndarray:
    """Packed default decision vector for ``Computer.reset`` / ``x0_computer``."""
    problem = planner.problem
    transcription = planner.transcription
    t_grid = transcription.initial_guess_time_grid(problem)

    # The default collocation guess, packed as the decision vector
    guess = default_initial_trajectory(problem, t_grid)
    z0 = np.asarray(transcription.pack_initial_guess(problem, guess), dtype=float)
    z0 = z0.reshape(-1)

    return z0


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
        """The plan-local horizon ``T``: the last knot, ``0`` for an empty plan."""
        return float(self.tau[-1]) if self.tau.size else 0.0


def build_nominal_cache(
    trajectory: Trajectory,
    t_solve: float,
    *,
    derivatives: bool = True,
) -> NominalCache:
    """
    Build a :class:`NominalCache` from a latched solution's trajectory.

    Parameters
    ----------
    trajectory : Trajectory
        The latched solve's schedule (plan-local ``t``).
    t_solve : float
        Absolute solve time (τ = 0).
    derivatives : bool, optional
        If True, attach FD knot rates ``x_dot`` / ``u_dot``.
    """
    tau = np.asarray(trajectory.t, dtype=float).reshape(-1)
    x = np.asarray(trajectory.x, dtype=float)
    u = np.asarray(trajectory.u, dtype=float)
    require_plan_samples(tau)

    # The knot rates ẋ and u̇, by finite differences
    x_dot = finite_diff_knots(tau, x) if derivatives else None
    u_dot = finite_diff_knots(tau, u) if derivatives else None

    return NominalCache(
        t_solve=float(t_solve),
        tau=tau.copy(),
        x=x.copy(),
        u=u.copy(),
        x_dot=None if x_dot is None else x_dot.copy(),
        u_dot=None if u_dot is None else u_dot.copy(),
    )


def eval_signal(cache: NominalCache, t: float, signal: np.ndarray) -> np.ndarray:
    """Linear interp of rows of ``signal`` (n, N) at absolute ``t``."""
    tau = clamp_tau(cache, t)

    # Each row interpolated linearly between the knots, at τ
    rows = []
    for i in range(int(signal.shape[0])):
        rows.append(np.interp(tau, cache.tau, signal[i]))
    sample = np.asarray(rows, dtype=float).reshape(-1)

    return sample


###############################################################################
# Internal machinery
###############################################################################


def shift_plan_trajectory(
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

    # The knots moved by dt_shift, kept up to the horizon (10⁻⁹ of slack)
    t_shift = np.asarray(plan.t, dtype=float).reshape(-1) + float(dt_shift)
    mask = t_shift <= float(horizon) + 1e-9
    if np.count_nonzero(mask) < 3:
        return None

    # The kept states, the first one pinned to the measurement
    x_guess = np.asarray(plan.x[:, mask], dtype=float).copy()
    x_guess[:, 0] = np.asarray(x_meas, dtype=float).reshape(-1)

    # Re-timed from the anchor, with the kept inputs
    t_guess = t_shift[mask] - float(t_anchor)
    u_guess = np.asarray(plan.u[:, mask], dtype=float)

    return Trajectory(t=t_guess, x=x_guess, u=u_guess)


_shift_plan_trajectory = shift_plan_trajectory  # ripple


def require_plan_samples(tau):
    """Refuse a plan without a knot."""
    if tau.size < 1:
        raise ValueError("plan trajectory must have at least one sample")


def finite_diff_knots(tau: np.ndarray, y: np.ndarray) -> np.ndarray:
    """Central / one-sided FD of ``y`` (n, N) vs ``tau`` (N,)."""
    _, n_t = y.shape
    y_dot = np.zeros_like(y)

    # A single knot has no rate
    if n_t == 1:
        return y_dot

    # Knot spacings Δτ, any |Δτ| < 10⁻¹⁵ set to 10⁻¹⁵ so a repeated knot never
    # divides by zero
    dtau = np.diff(tau)
    dtau = np.where(np.abs(dtau) < 1e-15, 1e-15, dtau)

    # One-sided at the ends: ẏ₀ = (y₁ − y₀) / Δτ₀, and backward at the last knot
    y_dot[:, 0] = (y[:, 1] - y[:, 0]) / dtau[0]
    y_dot[:, -1] = (y[:, -1] - y[:, -2]) / dtau[-1]

    # Central inside: ẏᵢ = (y_{i+1} − y_{i−1}) / (τ_{i+1} − τ_{i−1})
    if n_t > 2:
        for i in range(1, n_t - 1):
            y_dot[:, i] = (y[:, i + 1] - y[:, i - 1]) / (tau[i + 1] - tau[i - 1])

    return y_dot


def clamp_tau(cache: NominalCache, t: float) -> float:
    """Absolute ``t`` → plan-local τ clamped to ``[0, T]``."""
    # τ = t − t_solve, clamped to the plan [0, T]
    tau = float(t) - float(cache.t_solve)
    tau = float(np.clip(tau, 0.0, cache.tau_end))

    return tau
