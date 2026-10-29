"""The sampled diagram: a step diagram with a clocked block, the computer and ``computer @ plant``."""

from __future__ import annotations

import copy
from math import gcd

import numpy as np

from minilink.core.backends import array_module
from minilink.core.discrete.sampled_system import SampledSystem, steps_per_period
from minilink.core.discrete.step_diagram import StepDiagramSystem
from minilink.core.discrete.step_system import StepSystem
from minilink.core.wiring import (
    WiredDiagramMixin,
    require_x0_size,
    stacked_x0,
    write_through_x0,
)

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

    @property
    def x0(self):
        """The loop's initial state: its blocks' ``x0`` (a view), then the held outputs' own start."""
        x0 = np.concatenate([stacked_x0(self.subsystems), self.hold_x0])
        x0.flags.writeable = False

        return x0

    @x0.setter
    def x0(self, value):
        value = np.array(value, dtype=float).reshape(-1)
        require_x0_size(self, value, self.n)

        # The blocks' slices to the blocks, the held outputs' to this level
        n_blocks = self.n - self.hold_dim
        write_through_x0(self, value[:n_blocks], n_blocks)
        self.hold_x0 = value[n_blocks:]

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

    def compile(self, backend="numpy", verbose=False, *, substeps=None, solver=None):
        """
        Compile the sampled diagram into its stateless evaluator, the schedule in its plan.

        ``substeps`` and ``solver`` choose the plant's integration inside each
        base period (``None`` keeps the automatic step, ``⌈Δ / Δt⌉`` RK4 steps);
        a diagram with no continuous plant takes neither.
        """
        # Lazy import: a cycle, the compiler reads SampledDiagramSystem from here
        from minilink.core.compile.compiler import compile_sampled_diagram

        diagram = with_plant_steps(self, substeps=substeps, solver=solver)

        return compile_sampled_diagram(diagram, backend=backend, verbose=verbose)

    def animate(self, traj=None, **options):
        """
        Animate the loop's plant on its own run: its states, and the input the loop held on it each period.

        The loop's blocks are evaluated once per tick to read the plant's input
        (a pure MPC solves once per tick), never once per frame. A loop with no
        plant (a computer alone) animates as any diagram. ``options`` are
        :meth:`~minilink.core.facades.animation.SystemAnimation.animate`'s.
        """
        # The run passed, else the last one computed, else a fresh one
        if traj is None:
            traj = self.traj if self.traj is not None else self.compute_trajectory()

        # The plant's own run, drawn by the continuous plant it samples
        plant, plant_traj = plant_run_of(self, traj)
        if plant is None:
            return super().animate(traj, **options)

        return plant.animate(plant_traj, **options)

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

        # The slots after the blocks' states, zero at the start (every block fires at
        # k = 0); a start set on them stays while their size does, and a layout change
        # under a start that is not zero is refused
        start = self.n
        self.hold_index = {}
        labels = []
        for path, port_id, dim in holds:
            self.hold_index[(path, port_id)] = (start, start + dim)
            labels.extend(
                f"{'/'.join(path)}:{port_id}[{i}] (memory)"
                if dim > 1
                else f"{'/'.join(path)}:{port_id} (memory)"
                for i in range(dim)
            )
            start += dim
        self.hold_dim = start - self.n
        if self.hold_x0.size != self.hold_dim:
            require_no_hold_start(self, labels)
            self.hold_x0 = np.zeros(self.hold_dim)
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
        self.n = start


def require_no_hold_start(diagram, labels) -> None:
    """Refuse a change of the held outputs' layout under a start set on them: it would be dropped."""
    if np.any(diagram.hold_x0 != 0.0):
        raise ValueError(
            f"{diagram.name} holds a start on its held outputs "
            f"({diagram.hold_x0.size} slots) and its layout changed to {labels}: set "
            "the start (x0) again after adding blocks"
        )


def diagram_on_clock(diagram, dt):
    """
    ``diagram % dt``: a sampled diagram of the same blocks and wiring, each block ``% dt``.

    A sampled diagram keeps its kind and its own attributes (a zero-delay MPC
    its planner's face); a tick diagram becomes a :class:`SampledDiagramSystem`.
    """
    require_no_clocks(diagram)

    # The same kind with empty wiring, or a sampled diagram for a tick diagram
    if isinstance(diagram, SampledDiagramSystem):
        clocked = copy.copy(diagram)
        StepDiagramSystem.__init__(clocked)
    else:
        clocked = SampledDiagramSystem()

    # Each block on the clock: a static block reads seconds, a tick law its count
    for sys_id, sys in diagram.subsystems.items():
        clocked.add_subsystem(sys % dt, sys_id)

    # The edge inputs, the wiring and the edge outputs, as they were
    for port_id, port in diagram.inputs.items():
        clocked.inputs[port_id] = copy.copy(port)
    for target_sys_id, sources in diagram.connections.items():
        if target_sys_id == "output":
            continue
        for target_port_id, source in sources.items():
            if source is not None:
                clocked.connect(*source, target_sys_id, target_port_id)
    for port_id, port in diagram.outputs.items():
        clocked.connect_new_output_port(
            *port.source,
            port_id,
            port.declared_dependencies,
            nominal_value=port.nominal_value,
            labels=port.labels,
            units=port.units,
            lower_bound=port.lower_bound,
            upper_bound=port.upper_bound,
        )
    clocked.name = f"{diagram.name} % {float(dt):g}"

    return clocked


def with_plant_steps(diagram, *, substeps=None, solver=None):
    """
    The diagram with its plant integrated in ``substeps`` steps of ``solver`` per period: a view sharing every block.

    ``None`` keeps each. The plant is the one continuous system on the base
    clock, at any depth; the copy shares the blocks, their numbers and their
    starts, so only the plant's integration step differs. A diagram with no
    plant refuses the knobs: it has nothing integrated inside its periods.
    """
    if substeps is None and solver is None:
        return diagram

    stepped = replace_plant(diagram, substeps, solver)
    if stepped is None:
        raise ValueError(
            f"{diagram.name} has no continuous plant: its blocks run on the clock "
            "alone, there is no integration step to choose (dt=, substeps=, solver=)"
        )

    return stepped


def plant_steps_of(diagram, dt=None, solver=None) -> dict:
    """``{"substeps", "solver"}`` from a run's ``dt`` (the plant's step, ``⌈Δ / dt⌉`` per period) and ``solver``."""
    substeps = None if dt is None else steps_per_period(diagram.sample_period, dt)

    return {"substeps": substeps, "solver": solver}


def replace_plant(diagram, substeps, solver):
    """A shallow copy of ``diagram`` with its plant view restepped, at any depth; ``None`` when it holds no plant."""
    for sys_id, sys in diagram.subsystems.items():
        if isinstance(sys, SampledSystem) and sys.is_plant:
            new = sys.with_steps(substeps=substeps, solver=solver)
        elif isinstance(sys, WiredDiagramMixin):
            new = replace_plant(sys, substeps, solver)
        else:
            continue
        if new is None:
            continue
        stepped = copy.copy(diagram)
        stepped.subsystems = {**diagram.subsystems, sys_id: new}
        return stepped

    return None


def plant_run_of(loop, rollout, params=None):
    """
    ``(plant, run)``: the loop's plant and its run inside the loop's dense run, ``(None, None)`` with no plant.

    ``run.x`` is the plant's state slice; ``run.u`` the input the loop fed it,
    held over each period: the run's recorded ``"plant:u"`` when it has one (a
    dense run records it, nothing is computed again), else gathered by the
    compiled plan at each tick; the end sample holds the last tick's input, as
    the record does. ``params`` is the tree the run was made with (``None``:
    the loop's own).
    """
    # Lazy import: a real cycle, the trajectory module reads the systems
    from minilink.core.trajectory import Trajectory

    ev = loop.compile()
    op = ev.plan.plant_op
    if op is None:
        return None, None
    plant = getattr(op.system, "source", op.system)
    if "plant" in rollout.roles:
        return plant, rollout.of("plant")
    t_start = float(rollout.t[0])
    n_samples = rollout.t.size

    # S = the samples per period of this run: the dense run's substeps, a ticks run's one
    S = samples_per_period(rollout, loop.sample_period)

    # u_k = the plant's gathered input at tick k, held over the samples of its period
    # and, for the last tick, over the end sample
    ticks = np.minimum(np.arange(n_samples) // S, (n_samples - 1) // S - 1)
    u = np.zeros((int(op.u_dim), n_samples))
    for k in np.unique(ticks):
        samples = np.flatnonzero(ticks == k)
        first = samples[0]
        u_k = ev.plant_input(
            rollout.x[:, first], rollout.u[:, first], int(k), params, t_start=t_start
        )
        u[:, samples] = np.asarray(u_k).reshape(-1, 1)

    x = rollout.x[op.local_x_slice]

    return plant, Trajectory(t=rollout.t, x=x, u=u)


def samples_per_period(rollout, period) -> int:
    """``S``, the samples per base period of a run: its own spacing read (the dense run's substeps, a ticks run's one)."""
    n_samples = rollout.t.size
    period = float(period)
    spacing = float(rollout.t[1] - rollout.t[0]) if n_samples > 1 else period

    return max(1, int(round(period / spacing)))


def clock_periods(diagram):
    """The periods of every clocked block of ``diagram``, nested boxes included."""
    periods = []
    for sys in diagram.subsystems.values():
        if isinstance(sys, SampledSystem):
            periods.append(sys.sample_period)
        elif isinstance(sys, WiredDiagramMixin):
            periods.extend(clock_periods(sys))
    return periods


def base_period_of(periods):
    """The greatest common divisor of ``periods`` on a nanosecond grid, refused off the grid."""
    if not periods:
        return None
    require_clocks(periods)

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
        if not isinstance(sys, WiredDiagramMixin):
            yield path, sys, False
            continue
        hold_index = getattr(sys, "hold_index", {})
        for leaf_path, leaf, held in leaves_of(sys, path):
            inside = leaf_path[len(path) :]
            held_here = held or any(
                (inside, port) in hold_index for port in leaf.outputs
            )
            yield leaf_path, leaf, held_here


def require_no_clocks(diagram):
    """Refuse to clock a diagram that already holds a clocked block: clock each block once."""
    periods = clock_periods(diagram)
    if periods:
        raise TypeError(
            f"{diagram.name} already holds blocks on a clock of {periods[0]:g} s: "
            "clock the unclocked diagram, or clock its blocks one by one"
        )


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


def require_clocks(periods):
    """Refuse a block on a clock with no period: a native sampled law takes its period from ``%``."""
    if any(p is None for p in periods):
        raise TypeError(
            "a sampled law in the loop has no period: put it on its clock "
            "(law % dt), e.g. (mpc % 0.05) @ plant"
        )
