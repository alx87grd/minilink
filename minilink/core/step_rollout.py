"""Step rollouts: sampled state and input on integer step indices."""

from __future__ import annotations

from dataclasses import dataclass, field
from types import MappingProxyType

import numpy as np

from minilink.core.trajectory import SampledSignals, Trajectory

###############################################################################
# Public API
###############################################################################


@dataclass(frozen=True)
class StepRollout(SampledSignals):
    """
    Sampled state-input rollout on integer step indices ``k``.

    The discrete-time counterpart of :class:`~minilink.core.trajectory.Trajectory`.

    Parameters
    ----------
    k : np.ndarray
        Step indices with shape ``(N,)``.
    x : np.ndarray
        State samples with shape ``(n, N)``.
    u : np.ndarray
        Input samples with shape ``(m, N)``.
    signals : dict of str to np.ndarray, optional
        Additional sampled signals, each with shape ``(dim, N)``.
    """

    k: np.ndarray
    x: np.ndarray
    u: np.ndarray
    signals: dict[str, np.ndarray] = field(default_factory=dict)

    def __post_init__(self) -> None:
        # The samples k (N,), x (n, N) and u (m, N), as float copies
        k = np.asarray(self.k, dtype=float).reshape(-1).copy()
        x = np.asarray(self.x, dtype=float).copy()
        u = np.asarray(self.u, dtype=float).copy()
        check_samples(k, x, u)

        # The extra channels, each (dim, N), never redefining x or u
        signals = {}
        for name, values in dict(self.signals).items():
            require_extra_channel(name)
            arr = np.asarray(values, dtype=float).copy()
            check_signal(name, arr, k)
            signals[name] = arr

        object.__setattr__(self, "k", k)
        object.__setattr__(self, "x", x)
        object.__setattr__(self, "u", u)
        object.__setattr__(self, "signals", MappingProxyType(signals))

    @property
    def n_samples(self) -> int:
        """Number of step samples."""
        return int(self.k.size)

    def as_trajectory(self) -> Trajectory:
        """View for signal plotting only (``k`` mapped to the ``t`` array)."""
        return Trajectory(
            t=np.asarray(self.k, dtype=float),
            x=self.x,
            u=self.u,
            signals=dict(self.signals),
        )

    def copy(self) -> StepRollout:
        """Return a deep copy of the rollout."""
        return StepRollout(
            k=self.k.copy(),
            x=self.x.copy(),
            u=self.u.copy(),
            signals={name: values.copy() for name, values in self.signals.items()},
        )


###############################################################################
# Internal machinery
###############################################################################


def check_samples(k, x, u):
    """Refuse samples that are not ``k (N,)``, ``x (n, N)`` and ``u (m, N)`` with ``N ≥ 1``."""
    if k.ndim != 1:
        raise ValueError("k must be a 1-D array with shape (N,)")
    if x.ndim != 2:
        raise ValueError("x must be a 2-D array with shape (n, N)")
    if u.ndim != 2:
        raise ValueError("u must be a 2-D array with shape (m, N)")
    if x.shape[1] != k.size:
        raise ValueError("x must have shape (n, N) where N == k.size")
    if u.shape[1] != k.size:
        raise ValueError("u must have shape (m, N) where N == k.size")
    if k.size == 0:
        raise ValueError("StepRollout must contain at least one sample")


def require_extra_channel(name):
    """Refuse an extra signal named after a core channel."""
    if name in {"x", "u"}:
        raise ValueError("signals cannot redefine core channels 'x' or 'u'")


def check_signal(name, values, k):
    """Refuse an extra signal that is not ``(dim, N)`` on the rollout's samples."""
    if values.ndim != 2:
        raise ValueError(f"Signal {name!r} must have shape (dim, N)")
    if values.shape[1] != k.size:
        raise ValueError(f"Signal {name!r} must have shape (dim, N) where N == k.size")
