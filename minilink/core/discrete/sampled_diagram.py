"""The sampled diagram: a step diagram with a clocked block, the computer and ``computer @ plant``."""

from __future__ import annotations

from math import gcd

import numpy as np

from minilink.core.backends import array_module
from minilink.core.discrete.sampled_system import SampledSystem
from minilink.core.discrete.step_diagram import StepDiagramSystem
from minilink.core.discrete.step_system import StepSystem

# The clocks' grid: every period is a whole number of nanoseconds, to a relative
# tolerance, and their common base is the greatest common divisor on that grid
NANOSECONDS_PER_SECOND = 10**9
PERIOD_RATIO_TOL = 1e-9


###############################################################################
# Public API
###############################################################################


class SampledDiagramSystem(StepDiagramSystem):
    """
    A step diagram on a clock: its blocks fire on a common base tick ``Δ``.

    The base period ``Δ`` (:attr:`sample_period`) is the greatest common divisor
    of the clocked blocks' periods; a block of period ``d·Δ`` fires on the ticks
    ``k mod d = 0`` and holds its outputs in between, each held output a state
    slot after the blocks' states (a zero-order hold is a memory). One pure map
    on the base tick, ``X_{k+1} = F(X_k, U_k, k)``, whose plan hands every
    block the time of the loop: ``t_k = t_start + k·Δ`` in seconds to a block
    whose equations are in seconds (a static block, a system on a clock), its
    own count ``k // d`` to a tick law on a clock, the base count ``k`` to a bare
    tick law. Signals propagate within the tick. A sampled box nested inside is
    inlined into the one plan at compile.

    ``(filt % 0.01) >> (ctl % 0.1)`` is a sampled diagram (a multi-rate
    computer); ``(pid % 0.01) @ plant`` is one, its plant block the plant on the
    base clock.
    """

    @property
    def sample_period(self):
        """The base period ``Δ`` in seconds: the greatest common divisor of the clocked blocks' periods."""
        return base_period_of(clock_periods(self))

    def add_subsystem(self, sys, sys_id):
        """Add a step, static or clocked block (or a box of them) under a unique id."""
        super().add_subsystem(sys, sys_id)
        self.add_hold_slots()

    def refresh(self):
        super().refresh()
        self.add_hold_slots()

    def divisor_of(self, sys) -> int:
        """The block's divisor ``d``: it fires on the ticks ``k mod d = 0`` of the base clock."""
        if not isinstance(sys, SampledSystem):
            return 1

        return divisor_of(sys.sample_period, self.sample_period)

    def step(self, x, u, k=0, params=None):
        """
        ``X_{k+1}`` of the loop at its tick ``k``, the clock's origin at 0.

        The compiled plan is the one owner of the schedule (the divisors, the
        holds and each block's time): this runs it on the array's backend.
        """
        xp = array_module(x, u)
        backend = "numpy" if xp is np else "jax"
        ev = self.compile(backend=backend)

        x_next = ev.step(x, u, k, params)

        return x_next

    def compile(self, backend="numpy", verbose=False):
        """Compile the sampled diagram into its stateless evaluator, the schedule in its plan."""
        # Lazy import: a cycle, the compiler reads SampledDiagramSystem from here
        from minilink.core.compile.compiler import compile_sampled_diagram

        return compile_sampled_diagram(self, backend=backend, verbose=verbose)

    ###########################################################################
    # Internal machinery
    ###########################################################################

    def require_subsystem(self, sys, sys_id):
        """Refuse a continuous block with states: it joins the loop on a clock."""
        require_sampled_subsystem(sys, sys_id)

    def add_hold_slots(self):
        """Append one state slot per output of a block that fires slower than the base tick.

        A sampled box inside holds its own slow blocks in its own state; this
        level adds the blocks it makes slower than they are in their box.
        """
        holds = held_ports(self)

        # The slots after the blocks' states, zero at the start (every block fires at k = 0)
        start = self.n
        self.hold_index = {}
        labels = []
        for path, port_id, dim in holds:
            self.hold_index[(path, port_id)] = (start, start + dim)
            labels.extend(
                f"{'/'.join(path)}:{port_id}[{i}] (held)"
                if dim > 1
                else f"{'/'.join(path)}:{port_id} (held)"
                for i in range(dim)
            )
            start += dim
        self.hold_dim = start - self.n
        if not self.hold_dim:
            return

        state = self.state
        state.dim = start
        state.labels = list(state.labels) + labels
        state.units = list(state.units) + [""] * self.hold_dim
        state.lower_bound = np.concatenate(
            [state.lower_bound, np.full(self.hold_dim, -np.inf)]
        )
        state.upper_bound = np.concatenate(
            [state.upper_bound, np.full(self.hold_dim, np.inf)]
        )
        state.nominal_value = np.concatenate(
            [state.nominal_value, np.zeros(self.hold_dim)]
        )
        self.x0 = np.concatenate([self.x0, np.zeros(self.hold_dim)])
        self.derived_x0 = np.copy(self.x0)
        self.n = start


def clock_periods(diagram):
    """The periods of every clocked block of ``diagram``, nested boxes included."""
    periods = []
    for sys in diagram.subsystems.values():
        if isinstance(sys, SampledSystem):
            periods.append(sys.sample_period)
        elif isinstance(sys, StepDiagramSystem):
            periods.extend(clock_periods(sys))
    return periods


def base_period_of(periods):
    """The greatest common divisor of ``periods`` on a nanosecond grid, refused off the grid."""
    if not periods:
        return None

    # Δ = gcd of the periods counted in nanoseconds
    ticks = [round(float(p) * NANOSECONDS_PER_SECOND) for p in periods]
    for p, n in zip(periods, ticks):
        off_grid = abs(
            float(p) - n / NANOSECONDS_PER_SECOND
        ) > PERIOD_RATIO_TOL * float(p)
        if n < 1 or off_grid:
            raise ValueError(
                f"a block's period {p!r} s is not a whole number of nanoseconds; "
                "give the clocks commensurate periods"
            )
    base = 0
    for n in ticks:
        base = gcd(base, n)

    return base / NANOSECONDS_PER_SECOND


def divisor_of(period, base) -> int:
    """``d = period / Δ``, a whole number to :data:`PERIOD_RATIO_TOL`, else refused."""
    ratio = float(period) / float(base)
    d = round(ratio)
    if d < 1 or abs(ratio - d) > PERIOD_RATIO_TOL * max(1.0, ratio):
        raise ValueError(
            f"a block's period {period:g} s is not a whole multiple of the base "
            f"period {base:g} s; give the clocks commensurate periods"
        )
    return int(d)


###############################################################################
# Internal machinery
###############################################################################


def held_ports(diagram):
    """``(block path, port, dim)`` of every output this level holds: slower than its base, not held by a box inside."""
    base = diagram.sample_period
    holds = []
    for path, sys, held_inside in leaves_of(diagram):
        if held_inside or not isinstance(sys, SampledSystem):
            continue
        if divisor_of(sys.sample_period, base) == 1:
            continue
        for port_id, port in sys.outputs.items():
            holds.append((path, port_id, port.dim))
    return holds


def leaves_of(diagram, prefix=()):
    """``(path, leaf, held inside)`` of every leaf block, nested boxes walked; a box's own holds marked."""
    for sys_id, sys in diagram.subsystems.items():
        path = (*prefix, sys_id)
        if not isinstance(sys, StepDiagramSystem):
            yield path, sys, False
            continue
        for leaf_path, leaf, held in leaves_of(sys, path):
            inside = leaf_path[len(path) :]
            held_here = held or any(
                (inside, port) in sys.hold_index for port in leaf.outputs
            )
            yield leaf_path, leaf, held_here


def require_sampled_subsystem(sys, sys_id):
    """Refuse a continuous block with states: it joins the loop on a clock (``% dt``, or as the plant of ``@``)."""
    if isinstance(sys, StepSystem):
        return
    if sys.n > 0:
        raise TypeError(
            f"{type(sys).__name__} {sys_id!r} has continuous states and cannot "
            "join a SampledDiagramSystem: put it on a clock (block % dt, or "
            "discretize(plant, dt)), or close the loop with (block % dt) @ plant"
        )
