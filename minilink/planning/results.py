"""
Typed planning results for traj-family and policy-family solvers.

Keeps :class:`~minilink.core.trajectory.Trajectory` as a pure ``(t, x, u)``
schedule; metadata and warm-start extras live on the wrappers.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from minilink.core.trajectory import Trajectory


@dataclass(frozen=True)
class SolveMetadata:
    """Extras from a planner or NLP solve (not part of the schedule)."""

    success: bool
    message: str = ""
    cost: float | None = None
    solve_time_s: float | None = None
    stats: dict[str, object] = field(default_factory=dict)

    def __post_init__(self) -> None:
        object.__setattr__(self, "stats", dict(self.stats))
        if self.solve_time_s is not None:
            object.__setattr__(self, "solve_time_s", float(self.solve_time_s))


@dataclass(frozen=True)
class TrajectoryPlan:
    """
    Traj-family planning result.

    Parameters
    ----------
    trajectory : Trajectory
        Sampled ``(t, x, u)`` schedule.
    metadata : SolveMetadata
        Success / timing / solver stats.
    warm_state : array_like, optional
        Packed warm-start vector for the next solve (e.g. NLP ``z``).
    x_dot, u_dot : ndarray, optional
        Reserved knot rates; default ``None``.
    """

    trajectory: Trajectory
    metadata: SolveMetadata
    warm_state: np.ndarray | None = None
    x_dot: np.ndarray | None = None
    u_dot: np.ndarray | None = None

    def __post_init__(self) -> None:
        if self.warm_state is not None:
            object.__setattr__(
                self,
                "warm_state",
                np.asarray(self.warm_state, dtype=float).reshape(-1).copy(),
            )
        if self.x_dot is not None:
            object.__setattr__(
                self, "x_dot", np.asarray(self.x_dot, dtype=float).copy()
            )
        if self.u_dot is not None:
            object.__setattr__(
                self, "u_dot", np.asarray(self.u_dot, dtype=float).copy()
            )

    def to_flat(self) -> np.ndarray:
        """Flatten ``(t, x, u)`` into one vector: ``t``, then ``x`` row-major, then ``u``."""
        traj = self.trajectory
        return np.concatenate(
            [
                traj.t.reshape(-1),
                traj.x.reshape(-1),
                traj.u.reshape(-1),
            ]
        )

    @classmethod
    def from_flat(
        cls,
        flat: np.ndarray,
        *,
        n: int,
        m: int,
        n_samples: int,
        metadata: SolveMetadata | None = None,
    ) -> TrajectoryPlan:
        """Inverse of :meth:`to_flat` for a known ``(n, m, N)`` layout."""
        flat = np.asarray(flat, dtype=float).reshape(-1)
        n_t = int(n_samples)
        n_x = int(n) * n_t
        n_u = int(m) * n_t
        expected = n_t + n_x + n_u
        if flat.size != expected:
            raise ValueError(
                f"flat has length {flat.size}, expected {expected} for "
                f"n={n}, m={m}, N={n_samples}"
            )
        t = flat[:n_t]
        x = flat[n_t : n_t + n_x].reshape(n, n_t)
        u = flat[n_t + n_x :].reshape(m, n_t)
        if metadata is None:
            metadata = SolveMetadata(success=True)
        return cls(trajectory=Trajectory(t=t, x=x, u=u), metadata=metadata)


@dataclass(frozen=True)
class PolicyPlan:
    """
    Policy-family planning result.

    Parameters
    ----------
    policy : object
        Solver-specific policy payload (e.g. DP tables, lookup table).
    metadata : SolveMetadata
        Success / timing / solver stats.
    """

    policy: object
    metadata: SolveMetadata
