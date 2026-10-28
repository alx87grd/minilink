"""Discretization: a system on a clock, the plant one sample period later."""

from __future__ import annotations

from minilink.core.discrete.sampled_system import SampledSystem
from minilink.core.system import System

###############################################################################
# Public API
###############################################################################


def discretize(
    system: System,
    dt: float,
    *,
    solver: str = "rk4",
    substeps: int = 1,
) -> SampledSystem:
    """
    The system on a clock of period ``dt``, its input held: a :class:`SampledSystem`.

    A continuous system becomes ``x_{k+1} = Φ(x_k, u_k, t_k)``, the input held
    over each period ``[t_k, t_k + dt]``: ``solver`` is ``"rk4"`` or
    ``"euler"``, taking ``substeps`` steps per period, or ``"exact"`` for a
    linear model, the matrix exponential
    ``x_{k+1} = e^{A dt} x_k + ∫₀^{dt} e^{A s} ds B u_k``. A static block is
    read at the sampling instants, a tick law keeps its count. The view reads
    the source's live ``params`` (``step(x, u, k, params)`` overrides them),
    and the sample period is ``dt`` alone: the source's ``params`` may not carry
    a ``"dt"`` key. ``sys % dt`` is ``discretize(sys, dt, solver="euler")``.
    """
    return SampledSystem(system, dt, solver=solver, substeps=substeps)
