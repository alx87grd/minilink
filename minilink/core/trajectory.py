"""Trajectory: sampled state and input on a shared time grid."""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from types import MappingProxyType

import numpy as np

from minilink.core.backends import array_module
from minilink.core.inspect import inspect_text, repr_pretty

###############################################################################
# Public API
###############################################################################


class SampledSignals:
    """The named channels of a sampled record: ``x``, ``u`` and the extra ``signals``."""

    @property
    def n(self) -> int:
        """State dimension."""
        return int(self.x.shape[0])

    @property
    def m(self) -> int:
        """Input dimension."""
        return int(self.u.shape[0])

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


@dataclass(frozen=True)
class Trajectory(SampledSignals):
    """
    Sampled state-input trajectory on a shared time grid.

    The canonical sampled record of simulation, planning, control, and rendering
    workflows.

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
    roles : dict, optional
        The static layout of a loop's run: for each role (``"plant"``), the
        ``(start, stop)`` of its state in ``x`` and the name of the signal that
        records its input, read by :meth:`of`.

    Notes
    -----
    The trajectory semantic core is ``(t, x, u)``. Extra signals such as
    outputs, derivatives, or reconstructed internal ports can be attached
    through :meth:`with_signal` or :meth:`with_signals`. The samples keep their
    array module: NumPy arrays are copied as floats, JAX arrays are kept, and a
    trajectory is a JAX pytree, so ``jit``, ``grad`` and ``vmap`` pass through a
    function that returns one.
    """

    t: np.ndarray
    x: np.ndarray
    u: np.ndarray
    signals: dict[str, np.ndarray] = field(default_factory=dict)
    roles: dict = field(default_factory=dict)

    def __post_init__(self) -> None:
        xp = array_module(self.t, self.x, self.u)

        # The core samples as floats on their own backend (copies on NumPy), the times flat
        t = samples_of(xp, self.t).reshape(-1)
        x = samples_of(xp, self.x)
        u = samples_of(xp, self.u)
        check_samples(t, x, u)

        # The extra signals as floats, each on the same time grid
        signals = {}
        for name, values in dict(self.signals).items():
            signals[name] = signal_samples(name, values, t.size)

        # The frozen fields take the samples, the roles a read-only layout
        object.__setattr__(self, "t", t)
        object.__setattr__(self, "x", x)
        object.__setattr__(self, "u", u)
        object.__setattr__(self, "signals", MappingProxyType(signals))
        object.__setattr__(self, "roles", MappingProxyType(dict(self.roles)))

    def with_cost(self, cost, *, discount_rate=0.0, params=None) -> Trajectory:
        """The trajectory with the signals ``"cost_rate"`` and ``"cost"`` of ``cost`` (shape ``(1, N)``)."""
        cost_rate, running = cost.integrate(
            self, discount_rate=discount_rate, params=params
        )

        return self.with_signals(
            {"cost_rate": cost_rate.reshape(1, -1), "cost": running.reshape(1, -1)}
        )

    def of(self, role: str) -> Trajectory:
        """
        The run of one block of a loop: its state slice and its recorded input.

        ``traj.of("plant")`` indexes the loop's samples by the static layout in
        :attr:`roles`, so it traces under JAX like any slice.
        """
        require_role(self, role)
        start, stop = self.roles[role]["x"]
        u_name = self.roles[role]["u"]

        x = self.x[start:stop]
        u = self.signals[u_name]

        return Trajectory(t=self.t, x=x, u=u)

    @property
    def n_samples(self) -> int:
        """Number of time samples."""
        return int(self.t.size)

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

    # Signals

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
        require_extra_channel(name)

        new_signals = dict(self.signals)
        new_signals[name] = values

        return Trajectory(
            t=self.t, x=self.x, u=self.u, signals=new_signals, roles=self.roles
        )

    def with_signals(self, values: dict[str, np.ndarray]) -> Trajectory:
        """Return a new trajectory with multiple additional sampled signals."""
        new_signals = dict(self.signals)
        for name, arr in values.items():
            require_extra_channel(name)
            new_signals[name] = arr

        return Trajectory(
            t=self.t, x=self.x, u=self.u, signals=new_signals, roles=self.roles
        )

    # Samples at a time

    def sample(self, name: str, t: float) -> np.ndarray:
        """Sample a signal by nearest time index."""
        i = self.nearest_index(t)
        return self.get_signal(name)[:, i]

    def t2x(self, t: float) -> np.ndarray:
        """Return the state sample nearest to time ``t``."""
        return self.sample("x", t)

    def t2u(self, t: float) -> np.ndarray:
        """Return the input sample nearest to time ``t``."""
        return self.sample("u", t)

    # Copies and files

    def copy(self) -> Trajectory:
        """Return a deep copy of the trajectory."""
        return Trajectory(
            t=self.t.copy(),
            x=self.x.copy(),
            u=self.u.copy(),
            signals={name: values.copy() for name, values in self.signals.items()},
            roles=self.roles,
        )

    def save(self, path) -> None:
        """Save the sampled trajectory to an ``.npz`` file, its role layout with it."""
        signal_names = list(self.signals)

        # The core samples, the signal names and the role layout, then one array per signal
        arrays = {
            "t": np.asarray(self.t),
            "x": np.asarray(self.x),
            "u": np.asarray(self.u),
            "signal_names": np.asarray(signal_names),
            "roles": np.asarray(json.dumps(roles_as_json(self.roles))),
        }
        for i, name in enumerate(signal_names):
            arrays[f"signal_{i}"] = np.asarray(self.signals[name])

        np.savez(path, **arrays)

    @classmethod
    def load(cls, path) -> Trajectory:
        """Load a trajectory saved with :meth:`save`."""
        with np.load(path, allow_pickle=False) as data:
            names = [str(name) for name in data["signal_names"]]
            signals = {name: data[f"signal_{i}"] for i, name in enumerate(names)}
            roles = json.loads(str(data["roles"])) if "roles" in data else {}
            roles = {
                role: {"x": tuple(layout["x"]), "u": layout["u"]}
                for role, layout in roles.items()
            }
            return cls(
                t=data["t"], x=data["x"], u=data["u"], signals=signals, roles=roles
            )

    # Resampling

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
        t_new = time_grid_of(self, t_new, n_samples)
        t = self.t

        # Each channel linear between the samples, row by row on the new grid
        def _interp(arr: np.ndarray) -> np.ndarray:
            out = np.zeros((arr.shape[0], t_new.size), dtype=float)
            for i in range(arr.shape[0]):
                out[i, :] = np.interp(t_new, t, arr[i, :])
            return out

        x_new = _interp(self.x)
        u_new = _interp(self.u)
        signals_new = {name: _interp(values) for name, values in self.signals.items()}

        return Trajectory(
            t=t_new, x=x_new, u=u_new, signals=signals_new, roles=self.roles
        )

    # Display

    def __str__(self):
        return inspect_text(self)

    def _repr_pretty_(self, p, cycle):
        repr_pretty(self, p, cycle)

    ###########################################################################
    # Internal machinery
    ###########################################################################

    def nearest_index(self, t: float) -> int:
        """Index of the time sample nearest to ``t``."""
        t_samples = self.t

        # i = argminₖ |t_k − t|
        i = np.abs(t_samples - t).argmin()

        return int(i)


def interpolate_schedule(xp, t_samples, values, t):
    """Value of a sampled schedule at time ``t``: linear between samples, held outside.

    ``t_samples`` holds ``N`` nondecreasing sample times and ``values`` the
    samples, shape ``(N, ...)``; the result has the shape of one sample. At a
    repeated time the later sample holds, and a single sample holds at every
    ``t``. ``xp`` is the array module of the caller (NumPy or JAX).
    """
    ts = xp.asarray(t_samples)
    values = xp.asarray(values)
    last = len(t_samples) - 1

    # The interval [t_i, t_j] holding t, clipped to the schedule (i = j = 0 for one sample)
    i = xp.clip(xp.searchsorted(ts, t, side="right") - 1, 0, max(last - 1, 0))
    j = xp.minimum(i + 1, last)
    width = ts[j] - ts[i]

    # w = (t − t_i) / (t_j − t_i), held in [0, 1] outside the samples; on a
    # repeated time (t_j = t_i) w = 1 from t_i on, the later sample
    w = xp.where(
        width > 0.0,
        xp.clip((t - ts[i]) / xp.where(width > 0.0, width, 1.0), 0.0, 1.0),
        xp.where(t >= ts[i], 1.0, 0.0),
    )

    # v(t) = (1 − w) v_i + w v_j
    value = (1.0 - w) * values[i] + w * values[j]

    return value


###############################################################################
# Internal machinery
###############################################################################


def samples_of(xp, values):
    """Samples as floats: a fresh NumPy copy, or the JAX array as it is (a tracer stays one)."""
    if xp is np:
        return np.array(values, dtype=float)

    return xp.asarray(values, dtype=float)


def check_samples(t, x, u):
    """Refuse core samples off one nonempty, nondecreasing grid of ``N`` times."""
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
    if isinstance(t, np.ndarray) and np.any(np.diff(t) < 0.0):
        raise ValueError("t must be monotonically nondecreasing")


def signal_samples(name, values, N):
    """An extra signal as a float copy, refused on a core name or off the grid."""
    if name in {"x", "u"}:
        raise ValueError("signals cannot redefine core channels 'x' or 'u'")

    arr = samples_of(array_module(values), values)
    if arr.ndim != 2:
        raise ValueError(f"Signal {name!r} must have shape (dim, N)")
    if arr.shape[1] != N:
        raise ValueError(f"Signal {name!r} must have shape (dim, N) where N == t.size")

    return arr


def require_extra_channel(name):
    """Refuse ``x`` or ``u`` as an added signal: the constructor owns them."""
    if name in {"x", "u"}:
        raise ValueError("Use the constructor to define core channels 'x' and 'u'")


def time_grid_of(traj, t_new, n_samples):
    """The resampling grid: ``t_new``, or ``n_samples`` uniform times over ``[t0, tf]``."""
    if (t_new is None) == (n_samples is None):
        raise ValueError("Provide exactly one of t_new or n_samples")

    if n_samples is not None:
        if n_samples < 1:
            raise ValueError("n_samples must be >= 1")
        t_new = np.linspace(traj.t0, traj.tf, int(n_samples))
    else:
        t_new = np.asarray(t_new, dtype=float).reshape(-1)
        if t_new.size == 0:
            raise ValueError("t_new must contain at least one sample")
        if np.any(np.diff(t_new) < 0.0):
            raise ValueError("t_new must be monotonically nondecreasing")

    return t_new


def require_role(traj, role):
    """Refuse a role the trajectory's layout does not record."""
    if role not in traj.roles:
        recorded = ", ".join(traj.roles) or "none"
        raise ValueError(
            f"this trajectory records no {role!r} role (roles: {recorded}); a loop's "
            "run from problem.compile(...).trajectory() records its plant"
        )


def roles_as_json(roles):
    """The role layout as plain lists and strings."""
    return {
        role: {"x": list(layout["x"]), "u": layout["u"]}
        for role, layout in roles.items()
    }


def trajectory_flatten(traj):
    """The pytree children (the samples, the signals by name) and the static layout."""
    names = tuple(sorted(traj.signals))
    children = (traj.t, traj.x, traj.u, *(traj.signals[name] for name in names))
    layout = tuple(
        (role, tuple(roles["x"]), roles["u"])
        for role, roles in sorted(traj.roles.items())
    )

    return children, (names, layout)


def trajectory_unflatten(aux, children):
    """A trajectory from its children, without validation (a transformation's leaves)."""
    names, layout = aux
    t, x, u, *signal_values = children

    traj = object.__new__(Trajectory)
    object.__setattr__(traj, "t", t)
    object.__setattr__(traj, "x", x)
    object.__setattr__(traj, "u", u)
    object.__setattr__(
        traj, "signals", MappingProxyType(dict(zip(names, signal_values)))
    )
    object.__setattr__(
        traj,
        "roles",
        MappingProxyType(
            {role: {"x": x_slice, "u": u_name} for role, x_slice, u_name in layout}
        ),
    )

    return traj


def register_trajectory_pytree():
    """Register :class:`Trajectory` as a JAX pytree (called once, when minilink loads JAX)."""
    # Lazy import: JAX is optional, and loads with the first JAX tool
    import jax

    jax.tree_util.register_pytree_node(
        Trajectory, trajectory_flatten, trajectory_unflatten
    )
