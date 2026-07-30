"""Shared forced-input coercion for simulators.

``solve_forced``-style APIs accept a callable ``u(t)``, a constant scalar or
vector, or a full sampled matrix — optionally for a single named input port
(other ports hold their nominal values). :func:`coerce_forced_input` converts
any of these into the solver's sampled ``(m, n_pts)`` input matrix.
"""

import numpy as np


def coerce_forced_input(sys, times, u, input_port_id=None):
    """Return the sampled ``(m, n_pts)`` input matrix for a forced simulation.

    Parameters
    ----------
    sys : System
        Provides ``m``, input ports, and nominal port values.
    times : ndarray
        Time samples; callables are evaluated at each entry.
    u : callable, scalar, or array_like
        Full input ``u(t)`` / constant / ``(dim, n_pts)`` matrix — for the
        whole input vector, or for one port when ``input_port_id`` is given.
    input_port_id : str, optional
        Name of the single forced input port.
    """
    n_pts = len(times)
    if input_port_id is None:
        return _coerce_forced_signal(u, sys.m, times, "u")

    port = sys.inputs[input_port_id]
    port_slice = sys.get_input_port_slice(input_port_id)

    u_nominal = sys.get_u_from_input_ports().reshape(sys.m, 1)
    u_traj = np.repeat(u_nominal, n_pts, axis=1)
    u_traj[port_slice, :] = _coerce_forced_signal(
        u,
        port.dim,
        times,
        f"u for input port '{input_port_id}'",
    )
    return validate_forced_u_traj(u_traj, sys.m, n_pts)


def validate_forced_u_traj(u_traj, m, n_pts):
    """Validate a full sampled input matrix against ``(m, n_pts)``."""
    u = np.asarray(u_traj, dtype=float)
    expected_shape = (m, n_pts)
    if u.ndim != 2 or u.shape != expected_shape:
        raise ValueError(f"u_traj must have shape {expected_shape}")
    if not np.all(np.isfinite(u)):
        raise ValueError("u_traj must contain only finite values")
    return u


def _coerce_forced_signal(data, expected_dim, times, label):
    if callable(data):
        signal = _sample_forced_callable(data, expected_dim, times)
    else:
        signal = _coerce_forced_array(data, expected_dim, len(times), label)

    if not np.all(np.isfinite(signal)):
        raise ValueError(f"{label} must contain only finite values")
    return signal


def _sample_forced_callable(fn, expected_dim, times):
    samples = np.zeros((expected_dim, len(times)), dtype=float)

    for i, ti in enumerate(times):
        value = np.asarray(fn(float(ti)), dtype=float)
        if expected_dim == 1 and value.ndim == 0:
            samples[0, i] = float(value)
            continue
        samples[:, i] = value.reshape(expected_dim)

    return samples


def _coerce_forced_array(data, expected_dim, n_pts, label):
    arr = np.asarray(data, dtype=float)
    expected_shape = (expected_dim, n_pts)

    if arr.ndim == 0:
        if expected_dim != 1:
            raise ValueError(f"{label} must have shape {expected_shape}")
        return np.full((1, n_pts), float(arr), dtype=float)

    if arr.ndim == 1:
        if expected_dim == 1 and arr.shape[0] == n_pts:
            return arr.reshape(1, n_pts)
        if arr.shape[0] == expected_dim:
            column = arr.reshape(expected_dim, 1)
            return np.repeat(column, n_pts, axis=1)
        raise ValueError(f"{label} must have shape {expected_shape}")

    if arr.ndim == 2 and arr.shape == expected_shape:
        return arr

    raise ValueError(f"{label} must have shape {expected_shape}")
