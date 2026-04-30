"""
Trajectory container for sampled state-input data.

This module defines the canonical :class:`Trajectory` object used across
simulation, planning, control, and rendering workflows.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from types import MappingProxyType

import numpy as np


@dataclass(frozen=True)
class Trajectory:
    """
    Sampled state-input trajectory on a shared time grid.

    Parameters
    ----------
    t : np.ndarray
        Time samples with shape ``(N,)``.
    x : np.ndarray
        State samples with shape ``(n, N)``.
    u : np.ndarray
        Input samples with shape ``(m, N)``.
    signals : dict of str to np.ndarray, optional
        Additional sampled signals, each with shape ``(dim, N)``.

    Notes
    -----
    The trajectory semantic core is ``(t, x, u)``. Extra signals such as
    outputs, derivatives, or reconstructed internal ports can be attached
    through :meth:`with_signal` or :meth:`with_signals`.
    """

    t: np.ndarray
    x: np.ndarray
    u: np.ndarray
    signals: dict[str, np.ndarray] = field(default_factory=dict)

    def __post_init__(self) -> None:
        t = np.asarray(self.t, dtype=float).reshape(-1).copy()
        x = np.asarray(self.x, dtype=float).copy()
        u = np.asarray(self.u, dtype=float).copy()

        if t.ndim != 1:
            raise ValueError("t must be a 1-D array with shape (N,)")
        if x.ndim != 2:
            raise ValueError("x must be a 2-D array with shape (n, N)")
        if u.ndim != 2:
            raise ValueError("u must be a 2-D array with shape (m, N)")
        if x.shape[1] != t.size:
            raise ValueError("x must have shape (n, N) where N == t.size")
        if u.shape[1] != t.size:
            raise ValueError("u must have shape (m, N) where N == t.size")
        if t.size == 0:
            raise ValueError("Trajectory must contain at least one sample")
        if np.any(np.diff(t) < 0.0):
            raise ValueError("t must be monotonically nondecreasing")

        signals = {}
        for name, values in dict(self.signals).items():
            if name in {"x", "u"}:
                raise ValueError("signals cannot redefine core channels 'x' or 'u'")
            arr = np.asarray(values, dtype=float).copy()
            if arr.ndim != 2:
                raise ValueError(f"Signal {name!r} must have shape (dim, N)")
            if arr.shape[1] != t.size:
                raise ValueError(
                    f"Signal {name!r} must have shape (dim, N) where N == t.size"
                )
            signals[name] = arr

        object.__setattr__(self, "t", t)
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
        """Number of time samples."""
        return int(self.t.size)

    @property
    def time_steps(self) -> int:
        """Compatibility alias for the number of samples."""
        return self.n_samples

    @property
    def t0(self) -> float:
        """Initial time."""
        return float(self.t[0])

    @property
    def tf(self) -> float:
        """Final time."""
        return float(self.t[-1])

    @property
    def time_duration(self) -> float:
        """Total covered time span ``tf - t0``."""
        return float(self.tf - self.t0)

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

    def with_signal(self, name: str, values: np.ndarray) -> Trajectory:
        """
        Return a new trajectory with one additional sampled signal.

        Parameters
        ----------
        name : str
            Signal name. Cannot be ``"x"`` or ``"u"``.
        values : np.ndarray
            Signal samples with shape ``(dim, N)``.
        """
        if name in {"x", "u"}:
            raise ValueError("Use the constructor to define core channels 'x' and 'u'")
        new_signals = dict(self.signals)
        new_signals[name] = values
        return Trajectory(t=self.t, x=self.x, u=self.u, signals=new_signals)

    def with_signals(self, values: dict[str, np.ndarray]) -> Trajectory:
        """Return a new trajectory with multiple additional sampled signals."""
        new_signals = dict(self.signals)
        for name, arr in values.items():
            if name in {"x", "u"}:
                raise ValueError(
                    "Use the constructor to define core channels 'x' and 'u'"
                )
            new_signals[name] = arr
        return Trajectory(t=self.t, x=self.x, u=self.u, signals=new_signals)

    def _nearest_index(self, t: float) -> int:
        return int(np.abs(self.t - t).argmin())

    def sample(self, name: str, t: float) -> np.ndarray:
        """Sample a signal by nearest time index."""
        i = self._nearest_index(t)
        return self.get_signal(name)[:, i]

    def t2x(self, t: float) -> np.ndarray:
        """Return the state sample nearest to time ``t``."""
        return self.sample("x", t)

    def t2u(self, t: float) -> np.ndarray:
        """Return the input sample nearest to time ``t``."""
        return self.sample("u", t)

    def copy(self) -> Trajectory:
        """Return a deep copy of the trajectory."""
        return Trajectory(
            t=self.t.copy(),
            x=self.x.copy(),
            u=self.u.copy(),
            signals={name: values.copy() for name, values in self.signals.items()},
        )

    def save(self, path) -> None:
        """Save the sampled trajectory to an ``.npz`` file."""
        signal_names = list(self.signals)
        arrays = {
            "t": self.t,
            "x": self.x,
            "u": self.u,
            "signal_names": np.asarray(signal_names),
        }
        for i, name in enumerate(signal_names):
            arrays[f"signal_{i}"] = self.signals[name]
        np.savez(path, **arrays)

    @classmethod
    def load(cls, path) -> Trajectory:
        """Load a trajectory saved with :meth:`save`."""
        with np.load(path, allow_pickle=False) as data:
            names = [str(name) for name in data["signal_names"]]
            signals = {name: data[f"signal_{i}"] for i, name in enumerate(names)}
            return cls(t=data["t"], x=data["x"], u=data["u"], signals=signals)

    def resample(
        self,
        *,
        t_new: np.ndarray | None = None,
        n_samples: int | None = None,
    ) -> Trajectory:
        """
        Return a new trajectory interpolated on a new time grid.

        Parameters
        ----------
        t_new : np.ndarray, optional
            New 1-D time vector.
        n_samples : int, optional
            Number of uniformly spaced samples between ``t0`` and ``tf``.
        """
        if (t_new is None) == (n_samples is None):
            raise ValueError("Provide exactly one of t_new or n_samples")

        if n_samples is not None:
            if n_samples < 1:
                raise ValueError("n_samples must be >= 1")
            t_new = np.linspace(self.t0, self.tf, int(n_samples))
        else:
            t_new = np.asarray(t_new, dtype=float).reshape(-1)
            if t_new.size == 0:
                raise ValueError("t_new must contain at least one sample")
            if np.any(np.diff(t_new) < 0.0):
                raise ValueError("t_new must be monotonically nondecreasing")

        def _interp(arr: np.ndarray) -> np.ndarray:
            out = np.zeros((arr.shape[0], t_new.size), dtype=float)
            for i in range(arr.shape[0]):
                out[i, :] = np.interp(t_new, self.t, arr[i, :])
            return out

        return Trajectory(
            t=t_new,
            x=_interp(self.x),
            u=_interp(self.u),
            signals={name: _interp(values) for name, values in self.signals.items()},
        )
