"""The step model: one sample period of a continuous plant, its input held, and a run of them."""

from __future__ import annotations

SOLVERS = ("rk4", "euler")
HOLDS = ("zoh", "foh")

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

    # The substeps, the input at their ends read off the line from u0 to u1
    for i in range(substeps):
        ua = u0 if i == 0 else u0 + (i / substeps) * (u1 - u0)
        ub = u1 if i == substeps - 1 else u0 + ((i + 1) / substeps) * (u1 - u0)
        x = integrate(f, x, ua, ub, t + i * h, h)

    return x


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
    if int(substeps) != substeps or substeps < 1:
        raise ValueError(f"substeps must be a positive integer, got {substeps!r}")
    if hold not in HOLDS:
        raise ValueError(f"hold must be one of {HOLDS}, got {hold!r}")
