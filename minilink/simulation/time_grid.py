"""The simulation time grid: the output samples on ``[t0, tf]``."""

import numpy as np

# Automatic output grid (points on [t0, tf]) when neither ``n_samples`` nor ``dt``
# is given and the solver picks its own steps: a reporting resolution, not an
# integration step, fine enough (1 ms over 10 s) for smooth plots of fast
# oscillations. Fixed-step solvers derive ``dt`` from the plant instead.
DEFAULT_N_SAMPLES = 10001


###############################################################################
# Public API
###############################################################################


def build_time_grid(
    t0,
    tf,
    *,
    n_samples=None,
    dt=None,
    default_dt=None,
    verbose=False,
) -> tuple[np.ndarray, float, int]:
    """
    Build time samples on ``[t0, tf]`` and return ``(time_vector, dt, n_samples)``.

    If ``n_samples`` is set, uses a uniform grid of that many points. If
    ``dt`` is set, samples ``t0 + k dt``; both together are refused. The grid
    ends at ``tf``: when ``dt`` divides ``tf - t0`` (to a relative tolerance
    of 1e-9, which absorbs round-off such as ``1.1 / 0.1``), it has
    ``round((tf - t0) / dt) + 1`` points; otherwise its last step is shorter
    than ``dt`` and lands on ``tf`` (``[0, 0.3, 0.6, 0.9, 1.0]`` for ``dt =
    0.3`` on ``[0, 1]``). If neither, uses ``default_dt`` the same way when
    the caller supplies one (fixed-step solvers) and otherwise the
    :data:`DEFAULT_N_SAMPLES` reporting grid. The ``dt`` returned is the
    grid's step, its first.
    """
    t0, tf = require_time_span(t0, tf)

    if n_samples is None and dt is None and default_dt is None:
        # The solver picks its own steps: the reporting grid
        time_vector = np.linspace(t0, tf, DEFAULT_N_SAMPLES)
        if verbose:
            print(f"Automatic {DEFAULT_N_SAMPLES}-point output grid")

    elif n_samples is None and dt is None:
        # The caller's default step (fixed-step solvers): t₀ + k dt
        dt = require_dt(default_dt, label="automatic dt")
        time_vector = dt_grid(t0, tf, dt)
        if verbose:
            print("Automatic dt from default time-grid policy")

    elif dt is None:
        # n_samples points, uniform on [t₀, t_f]
        require_n_samples(n_samples)
        time_vector = np.linspace(t0, tf, n_samples)

    elif n_samples is None:
        # The given step: t₀ + k dt
        dt = require_dt(dt)
        time_vector = dt_grid(t0, tf, dt)

    else:
        # Both given: refused, once each is valid on its own
        require_n_samples(n_samples)
        require_dt(dt)
        raise ValueError(
            f"a time grid takes n_samples or dt, not both (n_samples={n_samples}, "
            f"dt={dt}): n_samples points on [t0, tf], or steps of dt from t0"
        )

    require_two_samples(time_vector)

    # The step and the sample count of the grid as built
    dt_out = time_vector[1] - time_vector[0]

    return time_vector, float(dt_out), int(len(time_vector))


###############################################################################
# Internal machinery
###############################################################################


# Relative tolerance under which ``(tf - t0) / dt`` counts as a whole number of
# steps, so float round-off (1.1 / 0.1 = 11.000000000000002) adds no sample.
_WHOLE_STEPS_RTOL = 1e-9


def require_time_span(t0, tf):
    """Refuse a span unless real, finite and ``t0 < tf``; return ``t0`` and ``tf`` as floats."""
    try:
        t0, tf = float(t0), float(tf)
    except (TypeError, ValueError) as exc:
        raise ValueError("t0 and tf must be real scalars") from exc
    if not (np.isfinite(t0) and np.isfinite(tf)):
        raise ValueError("t0 and tf must be finite")
    if tf <= t0:
        raise ValueError("tf must be greater than t0")
    return t0, tf


def require_dt(dt, *, label="dt"):
    """Refuse a ``dt`` that is not a positive finite scalar (a 0-d array is one); return it as a float."""
    if np.ndim(dt) != 0:
        raise ValueError(f"{label} must be a positive finite scalar")
    try:
        dt_value = float(dt)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} must be a positive finite scalar") from exc
    if not np.isfinite(dt_value) or dt_value <= 0.0:
        raise ValueError(f"{label} must be a positive finite scalar")
    return dt_value


def dt_grid(t0, tf, dt):
    """``t0, t0 + dt, …``, ending at ``tf``: a ``dt`` that does not divide the run shortens the last step."""
    n_float = (tf - t0) / dt
    n = round(n_float)

    # dt divides the horizon: t₀ … t₀ + n dt, dropping the sample round-off can
    # add past t_f. Slicing arange keeps every kept sample bit-identical
    # (t0 + dt * arange rounds differently when t0 != 0).
    if n >= 1 and abs(n_float - n) <= _WHOLE_STEPS_RTOL * n:
        time_vector = np.arange(t0, tf + dt, dt)[: n + 1]

    # It does not: the steps of dt short of t_f, then a shorter last one onto t_f
    else:
        time_vector = np.append(np.arange(t0, tf, dt), tf)

    return time_vector


def require_n_samples(n_samples):
    """Refuse an ``n_samples`` that is not an integer of at least 2."""
    if isinstance(n_samples, bool) or not isinstance(n_samples, (int, np.integer)):
        raise ValueError("n_samples must be an integer greater than or equal to 2")
    if n_samples < 2:
        raise ValueError("n_samples must be greater than or equal to 2")


def require_two_samples(time_vector):
    """Refuse a grid of fewer than two samples."""
    if time_vector.size < 2:
        raise ValueError("Time vector must contain at least two points")
