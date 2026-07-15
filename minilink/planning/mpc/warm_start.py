"""MPC warm-start guess construction for step blocks."""

from __future__ import annotations

import numpy as np

from minilink.core.trajectory import Trajectory
from minilink.planning.initial_guess import default_initial_trajectory
from minilink.planning.mpc.planner import MPCPlanner


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

    Mirrors the hand loop in ``examples/scripts/mpc/demo_dynamic_bicycle_rate_mpc_straight_line.py``.
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
    planner: MPCPlanner,
    *,
    dt_mpc: float,
    k: int = 0,
) -> Trajectory:
    """
    Build an ``initial_guess`` Trajectory for :meth:`~MPCPlanner.step`.

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
    shifted = shift_plan_trajectory(
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
    planner: MPCPlanner,
    *,
    dt_shift: float,
    horizon: float,
    t_anchor: float,
) -> Trajectory:
    """
    Warm-start guess from a previous solved plan (manual MPC demo contract).

    Mirrors ``examples/scripts/mpc/demo_dynamic_bicycle_rate_mpc_straight_line.py``
    lines 122–137: shift ``prev_plan`` by ``dt_shift``, pin ``x[:, 0] = x_meas``,
    re-time with ``t_anchor`` (simulation time at the MPC fire).
    """
    problem = planner.problem
    t_grid = planner.transcription.initial_guess_time_grid(problem)
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
    return default_initial_trajectory(problem, t_grid)


def mpc_default_computer_x0(planner: MPCPlanner) -> np.ndarray:
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
