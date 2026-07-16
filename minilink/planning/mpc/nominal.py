"""Nominal plan cache: slow build, fast linear eval."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from minilink.planning.results import TrajectoryPlan


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


def clamp_tau(cache: NominalCache, t: float) -> float:
    """Absolute ``t`` → plan-local τ clamped to ``[0, T]``."""
    tau = float(t) - float(cache.t_solve)
    return float(np.clip(tau, 0.0, cache.tau_end))


def eval_signal(cache: NominalCache, t: float, signal: np.ndarray) -> np.ndarray:
    """Linear interp of rows of ``signal`` (n, N) at absolute ``t``."""
    tau = clamp_tau(cache, t)
    rows = []
    for i in range(int(signal.shape[0])):
        rows.append(np.interp(tau, cache.tau, signal[i]))
    return np.asarray(rows, dtype=float).reshape(-1)
