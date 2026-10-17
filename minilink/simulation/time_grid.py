"""The simulation time grid: the output samples on ``[t0, tf]``."""

import logging

import numpy as np

# Automatic output grid (points on [t0, tf]) when neither ``n_steps`` nor ``dt``
# is given and the solver picks its own steps: a reporting resolution, not an
# integration step, fine enough (1 ms over 10 s) for smooth plots of fast
# oscillations. Fixed-step solvers derive ``dt`` from the plant instead.
DEFAULT_N_STEPS = 10001

# Automatic step of a fixed-step solver, smooth or discontinuous loop alike:
# dt = AUTO_DT_SCALE × solver_info["smallest_time_constant"]
AUTO_DT_SCALE = 0.1


###############################################################################
# Public API
###############################################################################


def build_time_grid(
    t0,
    tf,
    *,
    n_steps=None,
    dt=None,
    default_dt=None,
    verbose=False,
) -> tuple[np.ndarray, float, int]:
    """
    Build time samples on ``[t0, tf]`` and return ``(time_vector, dt, n_steps)``.

    If ``n_steps`` is set, uses a uniform grid of that many points. If only
    ``dt`` is set, samples ``t0 + k dt``. When ``dt`` divides ``tf - t0`` (to
    a relative tolerance of 1e-9, which absorbs round-off such as
    ``1.1 / 0.1``), the grid has ``round((tf - t0) / dt) + 1`` points and
    ends at ``tf``. Otherwise it steps on until it covers ``tf``: the last
    sample is the first step past ``tf``, up to one ``dt`` beyond it. If
    neither, uses ``default_dt`` the same way when the caller supplies one
    (fixed-step solvers) and otherwise the :data:`DEFAULT_N_STEPS` reporting
    grid.
    """
    t0, tf = require_time_span(t0, tf)

    if n_steps is None and dt is None and default_dt is None:
        # The solver picks its own steps: the reporting grid
        time_vector = np.linspace(t0, tf, DEFAULT_N_STEPS)
        if verbose:
            print(f"Automatic {DEFAULT_N_STEPS}-point output grid")

    elif n_steps is None and dt is None:
        # The caller's default step (fixed-step solvers): t₀ + k dt
        dt = require_dt(default_dt, label="automatic dt")
        time_vector = dt_grid(t0, tf, dt)
        if verbose:
            print("Automatic dt from default time-grid policy")

    elif dt is None:
        # n_steps points, uniform on [t₀, t_f]
        require_n_steps(n_steps)
        time_vector = np.linspace(t0, tf, n_steps)

    elif n_steps is None:
        # The given step: t₀ + k dt
        dt = require_dt(dt)
        time_vector = dt_grid(t0, tf, dt)

    else:
        # Both given: n_steps wins, once both are valid
        require_n_steps(n_steps)
        require_dt(dt)
        logging.warning(
            "You must choose between n_steps and dt: using the specified n_steps"
        )
        time_vector = np.linspace(t0, tf, n_steps)

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
    """``t0, t0 + dt, …`` until ``tf`` is covered, ending at ``tf`` when ``dt`` divides the horizon."""
    n_float = (tf - t0) / dt
    n = round(n_float)
    time_vector = np.arange(t0, tf + dt, dt)

    # dt divides the horizon: keep t₀ … t₀ + n dt and drop the sample round-off can
    # add past t_f. Slicing arange keeps every kept sample bit-identical
    # (t0 + dt * arange rounds differently when t0 != 0).
    if n >= 1 and abs(n_float - n) <= _WHOLE_STEPS_RTOL * n:
        time_vector = time_vector[: n + 1]

    return time_vector


def require_n_steps(n_steps):
    """Refuse an ``n_steps`` that is not an integer of at least 2."""
    if isinstance(n_steps, bool) or not isinstance(n_steps, (int, np.integer)):
        raise ValueError("n_steps must be an integer greater than or equal to 2")
    if n_steps < 2:
        raise ValueError("n_steps must be greater than or equal to 2")


def require_two_samples(time_vector):
    """Refuse a grid of fewer than two samples."""
    if time_vector.size < 2:
        raise ValueError("Time vector must contain at least two points")
