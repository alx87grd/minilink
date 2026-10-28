"""The host clock runner: a loop's blocks ticked on the host, each continuous plant integrated adaptively between the ticks."""

from __future__ import annotations

import numpy as np
from scipy.integrate import solve_ivp

from minilink.core.discrete.sampled_diagram import SampledDiagramSystem

# The adaptive integrator's tolerances between two ticks
RTOL = 1e-10
ATOL = 1e-12

###############################################################################
# Public API
###############################################################################


def run_on_the_clock(
    system, u_steps, *, x0, t_start=0.0, params=None, points=1, rtol=RTOL, atol=ATOL
):
    """
    The run of a system on a clock with an adaptive plant: ``(N·points + 1, n)``.

    The blocks are the pure ones the step diagram runs: at each tick the plan
    forms the signals and steps every discrete block; the loop's plant (the
    continuous block on the base clock, :attr:`~minilink.core.compile.plan.SampledExecutionPlan.plant_op`;
    or the system itself)
    is integrated by SciPy's adaptive ``solve_ivp`` from ``t_k`` to ``t_k + Δ``,
    its input held, restarted at every tick (the sample times are known), and
    reported at ``points`` instants of the period. The step diagram is the
    runner's test oracle at the ticks. ``u_steps`` is the boundary input of
    each tick, ``(N, m)``.
    """
    ev = system.compile(backend="numpy")
    x = np.asarray(x0, dtype=float).reshape(ev.n)
    rows = [x[None, :]]

    for k, u_k in enumerate(np.asarray(u_steps, dtype=float)):
        x, inside = tick_on_the_host(
            ev, system, x, u_k, k, t_start, params, points, rtol, atol
        )
        rows.append(inside)

    x_samples = np.concatenate(rows, axis=0)

    return x_samples


###############################################################################
# Internal machinery
###############################################################################


def tick_on_the_host(ev, system, x, u, k, t_start, params, points, rtol, atol):
    """One tick: the next state and the state at ``points`` instants of the period."""
    h = system.sample_period
    paths = []

    def advance(block, x_i, u_i, t, p):
        # ẋ = f(x, u, t) over [t, t + Δ] from x_i, the input held
        f = block.source.f
        times = t + h * np.arange(1, points + 1) / points
        solution = solve_ivp(
            lambda tau, z: f(z, u_i, tau, p),
            (float(t), float(t + h)),
            np.asarray(x_i, dtype=float),
            t_eval=times,
            rtol=rtol,
            atol=atol,
        )
        paths.append(solution.y.T)
        return solution.y[:, -1]

    if isinstance(system, SampledDiagramSystem):
        params = ev.params if params is None else params
        signals = ev.port_signals(x, u, k, params, t_start=t_start)
        x_next = ev.next_from_signals(
            x, u, k, params, signals, t_start=t_start, advance=advance
        )
        inside = inside_points(ev, x, x_next, paths, points)
    else:
        params = system.params if params is None else params
        if system.resolves:
            x_next = advance(system, x, u, t_start + k * h, params)
            inside = paths[0]
        else:
            # A block with nothing to integrate inside its period: its own update
            x_next = ev.step(x, u, k, params, t_start=t_start)
            inside = x_next[None, :]

    return x_next, inside


def inside_points(ev, x, x_next, paths, points):
    """The loop's state at the period's ``points`` instants: the plants' paths, the rest at the tick."""
    inside = np.tile(x, (points, 1))
    op = ev.plan.plant_op
    if op is not None:
        inside[:, op.local_x_slice] = paths[0]
    inside[-1] = x_next

    return inside
