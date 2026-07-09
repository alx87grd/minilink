"""Shared time-grid construction for simulators."""

import logging

import numpy as np


def build_time_grid(
    t0,
    tf,
    *,
    n_steps=None,
    dt=None,
    default_dt=0.001,
    verbose=False,
) -> tuple[np.ndarray, float, int]:
    """
    Build time samples on ``[t0, tf]`` and return ``(time_vector, dt, n_steps)``.

    If ``n_steps`` is set, uses a uniform grid of that many points. If only
    ``dt`` is set, uses ``arange``. If neither, uses ``default_dt``.
    """
    try:
        t0, tf = float(t0), float(tf)
    except (TypeError, ValueError) as exc:
        raise ValueError("t0 and tf must be real scalars") from exc
    if not (np.isfinite(t0) and np.isfinite(tf)):
        raise ValueError("t0 and tf must be finite")
    if tf <= t0:
        raise ValueError("tf must be greater than t0")

    if n_steps is None and dt is None:
        dt = _validate_dt(default_dt, label="automatic dt")
        time_vector = np.arange(t0, tf + dt, dt)
        if verbose:
            print("Automatic dt from default time-grid policy")

    elif dt is None:
        _validate_n_steps(n_steps)
        time_vector = np.linspace(t0, tf, n_steps)

    elif n_steps is None:
        dt = _validate_dt(dt)
        time_vector = np.arange(t0, tf + dt, dt)

    else:
        _validate_n_steps(n_steps)
        logging.warning(
            "You must choose between n_steps and dt: using the specified n_steps"
        )
        time_vector = np.linspace(t0, tf, n_steps)

    if time_vector.size < 2:
        raise ValueError("Time vector must contain at least two points")

    dt_out = time_vector[1] - time_vector[0]
    return time_vector, float(dt_out), int(len(time_vector))


def _validate_n_steps(n_steps):
    if isinstance(n_steps, bool) or not isinstance(n_steps, (int, np.integer)):
        raise ValueError("n_steps must be an integer greater than or equal to 2")
    if n_steps < 2:
        raise ValueError("n_steps must be greater than or equal to 2")


def _validate_dt(dt, *, label="dt"):
    if not np.isscalar(dt):
        raise ValueError(f"{label} must be a positive finite scalar")
    dt_value = float(dt)
    if not np.isfinite(dt_value) or dt_value <= 0.0:
        raise ValueError(f"{label} must be a positive finite scalar")
    return dt_value
