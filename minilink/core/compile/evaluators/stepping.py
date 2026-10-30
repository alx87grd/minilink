"""The step model: one sample period of a continuous plant, its input held, and a run of them."""

from __future__ import annotations

import operator

import numpy as np

from minilink.core.backends import array_module

SOLVERS = ("rk4", "euler")
HOLDS = ("zoh", "foh")

# Substeps traced one by one under JAX up to this count; more run in a rolled loop,
# so a plant at the automatic step (a thousand steps a period) traces as one body
UNROLLED_SUBSTEPS = 64

###############################################################################
# Public API
###############################################################################


def rk4(f, x, ua, ub, t, h):
    """One RK4 step over ``h``, the input linear from ``ua`` to ``ub`` (equal for a hold)."""
    um = 0.5 * (ua + ub)

    k1 = f(x, ua, t)
    k2 = f(x + 0.5 * h * k1, um, t + 0.5 * h)
    k3 = f(x + 0.5 * h * k2, um, t + 0.5 * h)
    k4 = f(x + h * k3, ub, t + h)

    # x_{k+1} = x_k + (h/6)(k₁ + 2 k₂ + 2 k₃ + k₄)
    x_next = x + (h / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

    return x_next


def euler(f, x, ua, ub, t, h):
    """One explicit Euler step over ``h``, the input at the start of the step."""
    # x_{k+1} = x_k + h f(x_k, u_k, t_k)
    x_next = x + h * f(x, ua, t)

    return x_next


def step_body(f, x, u0, u1, t, dt, solver, substeps):
    """``x(t + dt)`` from ``x(t)``: ``substeps`` steps of ``solver``, the input linear from ``u0`` to ``u1``."""
    integrate = rk4 if solver == "rk4" else euler
    h = dt / substeps
    if rolled(x, substeps):
        return rolled_steps(integrate, f, x, u0, u1, t, h, substeps)[0]

    # The substeps, the input at their ends read off the line from u0 to u1
    for i in range(substeps):
        ua = u0 if i == 0 else u0 + (i / substeps) * (u1 - u0)
        ub = u1 if i == substeps - 1 else u0 + ((i + 1) / substeps) * (u1 - u0)
        x = integrate(f, x, ua, ub, t + i * h, h)

    return x


def step_with_cost(f, rate, x, u, t, dt, solver, substeps):
    """
    ``(x(t + dt), c)``: one period of :func:`step_body` with the stage cost integrated beside the state.

    The state is augmented with ``c``, ``ċ = rate(x, u, τ)``, ``c(t) = 0``,
    and stepped by the same scheme and substeps, the input held: the state
    rows are :func:`step_body`'s, and under one Euler step ``c = dt · rate(x,
    u, t)``. One plant evaluation per stage serves both.
    """
    xp = array_module(x, u)
    n = x.shape[0]

    # One Euler step in its closed form: x + dt f(x, u, t) and c = dt rate(x, u, t),
    # the augmented step's own two rows (the state's is euler's, to the bit)
    if is_one_euler_step(solver, substeps):
        x_next = euler(f, x, u, u, t, dt)
        c = euler_stage_cost(rate, x, u, t, dt)
        return x_next, c

    # ż = [f(x, u, τ); rate(x, u, τ)], the cost a state of its own
    def f_aug(z, u, tau):
        x_s = z[:n]
        c_dot = xp.reshape(rate(x_s, u, tau), (1,))
        z_dot = xp.concatenate([f(x_s, u, tau), c_dot])
        return z_dot

    # z(t) = [x; 0], stepped over one period
    z0 = xp.concatenate([x, xp.zeros(1)])
    z = step_body(f_aug, z0, u, u, t, dt, solver, substeps)

    return z[:n], z[n]


def stage_cost(f, rate, x, u, t, dt, solver, substeps):
    """The stage cost alone of :func:`step_with_cost`: under one Euler step ``dt · rate(x, u, t)``, the plant not evaluated."""
    if is_one_euler_step(solver, substeps):
        return euler_stage_cost(rate, x, u, t, dt)

    _, c = step_with_cost(f, rate, x, u, t, dt, solver, substeps)

    return c


def euler_stage_cost(rate, x, u, t, dt):
    """``c = dt · rate(x, u, t)``: the stage cost of one Euler step, the one rule every stage-cost path shares."""
    xp = array_module(x, u)

    # c = dt ċ(x, u, t), the rate at the period's start
    c = dt * xp.reshape(rate(x, u, t), ())

    return c


def is_one_euler_step(solver, substeps) -> bool:
    """Whether a period is one explicit Euler step, whose stage cost has the closed form."""
    return solver == "euler" and int(substeps) == 1


def step_path(f, x, u0, u1, t, dt, solver, substeps):
    """``[x(t + dt/s), …, x(t + dt)]``: the end of every substep of :func:`step_body`, ``(s, n)``."""
    xp = array_module(x)
    integrate = rk4 if solver == "rk4" else euler
    h = dt / substeps
    if rolled(x, substeps):
        return rolled_steps(integrate, f, x, u0, u1, t, h, substeps)[1]

    # The substeps, each end kept, the input at their ends read off the line from u0 to u1
    path = []
    for i in range(substeps):
        ua = u0 if i == 0 else u0 + (i / substeps) * (u1 - u0)
        ub = u1 if i == substeps - 1 else u0 + ((i + 1) / substeps) * (u1 - u0)
        x = integrate(f, x, ua, ub, t + i * h, h)
        path.append(x)

    x_path = xp.stack(path)

    return x_path


def rolled(x, substeps) -> bool:
    """Whether the substeps run in a rolled loop: JAX arrays, more than :data:`UNROLLED_SUBSTEPS`."""
    return substeps > UNROLLED_SUBSTEPS and array_module(x) is not np


def rolled_steps(integrate, f, x, u0, u1, t, h, substeps):
    """``(x(t + s h), [x(t + h), …, x(t + s h)])``: the substeps in one ``lax.scan``, the input read off the line from u0 to u1."""
    # Lazy import: JAX loads with the first traced plant
    import jax

    def substep(x, i):
        ua = u0 + (i / substeps) * (u1 - u0)
        ub = u0 + ((i + 1) / substeps) * (u1 - u0)
        x_next = integrate(f, x, ua, ub, t + i * h, h)
        return x_next, x_next

    x_end, path = jax.lax.scan(substep, x, np.arange(substeps, dtype=float))

    return x_end, path


def substeps_of(dt_hold, dt_inner=None):
    """The number of integration steps of about ``dt_inner`` in one hold of ``dt_hold``."""
    if dt_inner is None:
        return 1

    return max(1, int(round(float(dt_hold) / float(dt_inner))))


def iterate(evaluator, transition, x0, sequences, *, records=False):
    """``[x_0, …, x_N]`` of ``x_{k+1} = transition(x_k, *row_k)`` over the rows of ``sequences``.

    With ``records``, ``transition`` returns ``(x_{k+1}, record_k)`` (a vector
    computed at tick ``k``) and the run returns ``(x_samples, records)``,
    ``records`` ``(N, r)``. A Python loop on NumPy, one ``lax.scan`` on JAX.
    """
    if evaluator.backend != "jax":
        x = x0
        x_path = [x]
        record_path = []
        for row in zip(*sequences):
            if records:
                x, record = transition(x, *row)
                record_path.append(record)
            else:
                x = transition(x, *row)
            x_path.append(x)
        x_samples = np.array(x_path)
        if not records:
            return x_samples

        return x_samples, np.stack(record_path)

    def scan_body(x, row):
        if records:
            x_next, record = transition(x, *row)
            return x_next, (x_next, record)
        x_next = transition(x, *row)
        return x_next, x_next

    _, outputs = evaluator.jax.lax.scan(scan_body, x0, sequences)
    xs = outputs[0] if records else outputs
    x_samples = evaluator.jnp.concatenate((x0[None, :], xs), axis=0)
    if not records:
        return x_samples

    return x_samples, outputs[1]


def iterate_dense(evaluator, period, x0, sequences):
    """``[x_0, …]`` of every point a run keeps: ``period(x_k, *row_k) -> (x_{k+1}, points_k[, record_k])``.

    ``points_k`` is ``(S, n)``, the state at the ``S`` points of the period after
    its start, its last row ``x_{k+1}``; the run returns ``(N·S + 1, n)``. A
    period that also returns ``record_k`` (a vector, the signals recorded at
    tick ``k``) makes the run return ``(x_samples, records)``, ``records``
    ``(N, r)``. A Python loop on NumPy, one ``lax.scan`` on JAX.
    """
    if evaluator.backend != "jax":
        x = x0
        x_path = [x0[None, :]]
        records = []
        for row in zip(*sequences):
            x, points, *record = period(x, *row)
            x_path.append(points)
            records.extend(record)
        x_samples = np.concatenate(x_path, axis=0)
        if not records:
            return x_samples

        return x_samples, np.stack(records)

    def scan_body(x, row):
        x_next, *outputs = period(x, *row)
        return x_next, tuple(outputs)

    _, outputs = evaluator.jax.lax.scan(scan_body, x0, sequences)
    n = x0.shape[0]
    x_samples = evaluator.jnp.concatenate(
        (x0[None, :], outputs[0].reshape(-1, n)), axis=0
    )
    if len(outputs) == 1:
        return x_samples

    return x_samples, outputs[1]


def held_inputs(u_seq, hold):
    """The input at the start and at the end of each period: ``(u_k, u_k)`` or ``(u_k, u_{k+1})``."""
    if hold == "zoh":
        return u_seq, u_seq

    return u_seq[:-1], u_seq[1:]


###############################################################################
# Internal machinery
###############################################################################


def require_step_knobs(solver, substeps, hold="zoh"):
    """Refuse a solver, a substep count or a hold the step model does not have."""
    if solver not in SOLVERS:
        raise ValueError(
            f"solver must be one of {SOLVERS} for a step, got {solver!r}; "
            "an adaptive solver integrates through Simulator(solver='scipy')"
        )
    if isinstance(substeps, bool) or not is_integer(substeps) or substeps < 1:
        raise ValueError(f"substeps must be a positive integer, got {substeps!r}")
    if hold not in HOLDS:
        raise ValueError(f"hold must be one of {HOLDS}, got {hold!r}")


def is_integer(value) -> bool:
    """Whether ``value`` is an integer type (``int``, a NumPy integer), not a float holding one."""
    try:
        operator.index(value)
    except TypeError:
        return False

    return True
