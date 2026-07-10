"""
Step rollout container for sampled state-input data on integer step indices.

This module defines :class:`StepRollout`, the discrete-time counterpart to
:class:`~minilink.core.trajectory.Trajectory`.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from types import MappingProxyType

import numpy as np

from minilink.core.trajectory import Trajectory


@dataclass(frozen=True)
class StepRollout:
    """
    Sampled state-input rollout on integer step indices ``k``.

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
        k = np.asarray(self.k, dtype=float).reshape(-1).copy()
        x = np.asarray(self.x, dtype=float).copy()
        u = np.asarray(self.u, dtype=float).copy()

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

        signals = {}
        for name, values in dict(self.signals).items():
            if name in {"x", "u"}:
                raise ValueError("signals cannot redefine core channels 'x' or 'u'")
            arr = np.asarray(values, dtype=float).copy()
            if arr.ndim != 2:
                raise ValueError(f"Signal {name!r} must have shape (dim, N)")
            if arr.shape[1] != k.size:
                raise ValueError(
                    f"Signal {name!r} must have shape (dim, N) where N == k.size"
                )
            signals[name] = arr

        object.__setattr__(self, "k", k)
        object.__setattr__(self, "x", x)
        object.__setattr__(self, "u", u)
        object.__setattr__(self, "signals", MappingProxyType(signals))

    @property
    def n(self) -> int:
        """State dimension."""
        return int(self.x.shape[0])

    @property
    def m(self) -> int:
        """Input dimension."""
        return int(self.u.shape[0])

    @property
    def n_samples(self) -> int:
        """Number of step samples."""
        return int(self.k.size)

    @property
    def signal_names(self) -> tuple[str, ...]:
        """All available sampled channel names."""
        return ("x", "u", *tuple(self.signals.keys()))

    def has_signal(self, name: str) -> bool:
        """Return ``True`` when a named signal is available."""
        return name in {"x", "u"} or name in self.signals

    def get_signal(self, name: str) -> np.ndarray:
        """Return a sampled signal by name."""
        if name == "x":
            return self.x
        if name == "u":
            return self.u
        return self.signals[name]

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
