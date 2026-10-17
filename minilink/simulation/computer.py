"""Scheduled step simulation: a compiled step diagram ticked on a multi-rate schedule."""

from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass, field

import numpy as np

from minilink.core.backends import BACKEND_NUMPY, normalize_backend
from minilink.core.compile.evaluators.step_rollout import gather_u
from minilink.core.compile.execution_plan import PortOperation
from minilink.core.compile.step_compiler import build_step_execution_plan
from minilink.core.compile.step_execution_plan import StepExecutionPlan, StepOperation
from minilink.core.diagram import DiagramSystem, StepDiagramSystem
from minilink.core.system import DynamicSystem, System

###############################################################################
# Public API
###############################################################################


class Computer:
    """
    Stateful scheduled runtime for a :class:`StepDiagramSystem`.

    Compiles the same
    :class:`~minilink.core.compile.step_execution_plan.StepExecutionPlan` as the
    synchronous ``compute_rollout``, then advances with :meth:`tick` under
    :attr:`schedule` divisors (multi-rate firing) and double-buffered internal
    port signals.

    Within each tick, fired blocks gather inputs from the committed read buffer
    only (parallel / order-agnostic semantics). Skipped blocks hold via the
    write-buffer copy taken at tick start.
    """

    def __init__(self, diagram: StepDiagramSystem, schedule: StepSchedule):
        require_step_diagram(diagram)

        # The diagram and its firing schedule
        self.diagram = diagram
        self.schedule = schedule

        # The compiled plan and each block's operations, filled by compile()
        self.plan: StepExecutionPlan | None = None
        self.port_ops_by_sys: dict[str, tuple[PortOperation, ...]] = {}
        self.step_op_by_sys: dict[str, StepOperation | None] = {}
        self.all_sys_ids: tuple[str, ...] = ()

        # The run: the tick index, the state and the two signal buffers, set by reset()
        self.k = 0
        self.x: np.ndarray | None = None
        self.signal_read: np.ndarray | None = None
        self.signal_write: np.ndarray | None = None

    def compile(
        self,
        backend: str = BACKEND_NUMPY,
        *,
        bind_params: bool = False,
        verbose: bool = False,
    ) -> None:
        """Build the step execution plan and per-subsystem lookup tables."""
        _ = verbose  # Reserved for parity with ``StepDiagramSystem.compile``
        require_numpy_backend(backend)
        require_scheduled_subsystems(self.schedule, self.diagram)

        # The plan: the same lowering as the synchronous rollout
        plan = build_step_execution_plan(self.diagram, bind_params=bind_params)
        self.plan = plan

        # Each block's port operations and its one step operation, by block id
        self.port_ops_by_sys = group_port_ops_by_sys(plan)
        self.step_op_by_sys = {sys_id: None for sys_id in self.diagram.subsystems}
        for op in plan.step_ops:
            self.step_op_by_sys[op.sys_id] = op
        self.all_sys_ids = tuple(self.diagram.subsystems)

    def reset(self, x0=None) -> None:
        """Reset tick index, state, and internal signal buffers."""
        self.require_compiled()

        plan = self.plan

        # The first tick, from x0 or else the diagram's own initial state
        self.k = 0
        if x0 is None:
            x_init = np.asarray(self.diagram.x0, dtype=float).reshape(plan.state_dim)
        else:
            x_init = np.asarray(x0, dtype=float).reshape(plan.state_dim)
        self.x = x_init.copy()

        # Both signal buffers, zeroed
        self.signal_read = np.zeros(plan.signal_dim, dtype=float)
        self.signal_write = np.zeros(plan.signal_dim, dtype=float)

        # A block's tick latch (an MPC block's memoized solve) forgets its last tick
        for sys in self.diagram.subsystems.values():
            latch = getattr(sys, "_latch", None)
            if latch is not None and hasattr(latch, "reset_latch"):
                latch.reset_latch()

    def tick(self, u) -> dict[str, np.ndarray]:
        """
        Advance one base tick.

        Parameters
        ----------
        u : array_like
            Diagram boundary input vector (layout ``diagram.m``).

        Returns
        -------
        dict[str, ndarray]
            Boundary output port values after the tick commit.
        """
        self.require_reset()

        plan = self.plan
        k_tick = self.k
        read = self.signal_read
        write = self.signal_write

        # Skipped blocks hold: the write buffer starts as the committed signals
        write[:] = read
        x_work = self.x.copy()
        u_arr = np.asarray(u, dtype=float).reshape(self.diagram.m)

        for sys_id in self.all_sys_ids:
            # A block fires on the ticks where k mod d = 0
            if k_tick % self.divisor(sys_id) != 0:
                continue

            # Its output ports, fed from the committed read buffer only
            for op in self.port_ops_by_sys.get(sys_id, ()):
                local_x = x_work[op.local_x_slice]
                local_u = gather_u(op.gather_sources, op.u_dim, read, u_arr)
                write[op.out_slice] = op.compute_func(
                    local_x, local_u, k_tick, op.bound_params
                )

            # Its state step: x_{k+1} = step(x_k, u_k, k)
            step_op = self.step_op_by_sys.get(sys_id)
            if step_op is not None:
                local_x = x_work[step_op.local_x_slice]
                local_u = gather_u(step_op.gather_sources, step_op.u_dim, read, u_arr)
                x_work[step_op.local_x_slice] = step_op.step_func(
                    local_x, local_u, k_tick, step_op.bound_params
                )

        # Commit: the written buffer becomes the read one, and the state advances
        self.signal_read, self.signal_write = write, read
        self.x = x_work
        outs = {
            port_id: self.signal_read[sl].copy()
            for port_id, sl in plan.external_output_slices.items()
        }
        self.k += 1

        return outs

    def __matmul__(self, plant):
        """
        Hybrid feedback using standard port auto-wiring.

        ``computer @ plant`` returns a :class:`~minilink.core.hybrid_diagram.HybridDiagram`
        with the same port-resolution rules as continuous ``ctl @ plant``.
        Like the continuous ``@``, it leaves a plant diagram unchanged.
        """
        # A real cycle: core.hybrid_composition imports Computer from this module
        from minilink.core.hybrid_composition import (
            hybrid_closed_loop,
            resolve_hybrid_feedback_ports,
        )

        require_plant_side(plant)

        # The boundary ports, resolved as for the continuous ``ctl @ plant``
        computer_out, computer_in, plant_in, plant_out = resolve_hybrid_feedback_ports(
            self.diagram,
            plant,
        )

        return hybrid_closed_loop(
            self.diagram,
            plant,
            schedule=self.schedule,
            computer=self,
            computer_out=computer_out,
            computer_in=computer_in,
            plant_in=plant_in,
            plant_out=plant_out,
        )

    def divisor(self, sys_id: str) -> int:
        """Return the fire divisor for ``sys_id`` (default 1)."""
        return self.schedule.fire.get(sys_id, 1)

    ###########################################################################
    # Internal machinery
    ###########################################################################

    def require_compiled(self) -> None:
        """Refuse a reset before :meth:`compile` has built the plan."""
        if self.plan is None:
            raise RuntimeError("Computer.compile() must be called before reset()")

    def require_reset(self) -> None:
        """Refuse a tick before :meth:`compile` and :meth:`reset` have built the plan and the buffers."""
        if (
            self.plan is None
            or self.x is None
            or self.signal_read is None
            or self.signal_write is None
        ):
            raise RuntimeError(
                "Computer.compile() and reset() must be called before tick()"
            )


@dataclass
class StepSchedule:
    """Firing schedule for a :class:`Computer` runtime."""

    dt_base: float
    # Each block's divisor d: the block fires on the ticks where k mod d = 0
    fire: dict[str, int] = field(default_factory=dict)

    def __post_init__(self) -> None:
        self.dt_base = validate_dt_base(self.dt_base)

        # The divisors as whole numbers, in a dict of the schedule's own
        fire = {}
        for sys_id, divisor in self.fire.items():
            d = int(divisor)
            require_fire_divisor(sys_id, d, divisor)
            fire[sys_id] = d
        self.fire = fire

    @classmethod
    def from_rates(cls, dt_base: float, rates_hz: dict[str, float]) -> StepSchedule:
        """Build ``fire`` divisors from subsystem rates in Hz."""
        base = base_rate_hz(dt_base)

        # d = f_base / f of each block, a whole number
        fire = {
            sys_id: rate_to_divisor(base, rate, sys_id)
            for sys_id, rate in rates_hz.items()
        }

        return cls(dt_base=dt_base, fire=fire)


def as_computer(
    side: System | StepDiagramSystem,
    schedule: StepSchedule | float,
) -> Computer:
    """
    Build a :class:`Computer` from a step block or :class:`StepDiagramSystem`.

    Wraps a leaf in a step diagram, exposes standard boundary ports, and attaches
    ``schedule``. Use the ``%`` operator as shorthand: ``computer = block % dt``.
    A block whose class defines its own ``%`` builds the Computer there (an MPC
    block checks ``schedule`` against its ``dt_mpc``).
    """
    # A real cycle: core.hybrid_composition imports Computer from this module
    from minilink.core.hybrid_composition import builds_own_computer

    # A plain block: wrapped in a step diagram, its boundary ports exposed
    if not builds_own_computer(side):
        return build_computer(side, schedule)

    # A block whose class defines its own ``%`` builds its Computer there
    computer = side % schedule
    require_own_computer(side, computer)

    return computer


###############################################################################
# Internal machinery
###############################################################################


def require_step_diagram(diagram) -> None:
    """Refuse a runtime over anything but a :class:`StepDiagramSystem`."""
    if not isinstance(diagram, StepDiagramSystem):
        raise TypeError(
            f"Computer requires StepDiagramSystem, got {type(diagram).__name__}"
        )


def require_numpy_backend(backend) -> None:
    """Refuse a compile backend other than NumPy, the only one the runtime ticks on."""
    key = normalize_backend(backend)
    if key != BACKEND_NUMPY:
        raise NotImplementedError("Computer supports backend='numpy' only")


def require_scheduled_subsystems(schedule, diagram) -> None:
    """Refuse a schedule that names a block the diagram does not have."""
    unknown = set(schedule.fire) - set(diagram.subsystems)
    if unknown:
        names = ", ".join(sorted(unknown))
        raise ValueError(f"StepSchedule.fire keys not in diagram subsystems: {names}")


def group_port_ops_by_sys(
    plan: StepExecutionPlan,
) -> dict[str, tuple[PortOperation, ...]]:
    """The plan's port operations, grouped by the block that owns them."""
    grouped: dict[str, list[PortOperation]] = defaultdict(list)
    for op in plan.port_ops:
        grouped[op.sys_id].append(op)
    return {sys_id: tuple(ops) for sys_id, ops in grouped.items()}


def require_plant_side(plant) -> None:
    """Refuse a plant that is neither a block nor a diagram."""
    if not isinstance(plant, (System, DiagramSystem)):
        raise TypeError(
            f"Computer @ plant requires System or DiagramSystem, "
            f"got {type(plant).__name__}"
        )


def validate_dt_base(dt_base: float) -> float:
    """``dt_base`` as a float, refused unless positive."""
    dt = float(dt_base)
    if dt <= 0.0:
        raise ValueError(f"dt_base must be positive, got {dt_base}")
    return dt


def require_fire_divisor(sys_id, d, divisor) -> None:
    """Refuse a fire divisor below one tick."""
    if d < 1:
        raise ValueError(f"fire[{sys_id!r}] must be >= 1, got {divisor}")


def base_rate_hz(dt_base: float) -> float:
    """The base tick rate ``1 / dt_base`` in Hz, refused unless a whole number."""
    dt = validate_dt_base(dt_base)

    # f_base = 1 / Δt_base, whole to 10⁻⁹
    base_hz = 1.0 / dt
    if abs(base_hz - round(base_hz)) > 1e-9:
        raise ValueError(
            f"dt_base={dt_base} implies non-integer base rate Hz={base_hz}; "
            "use a dt_base that yields an integer base rate for from_rates"
        )

    return base_hz


def rate_to_divisor(base_hz: float, rate_hz: float, sys_id: str) -> int:
    """The fire divisor ``base_hz / rate_hz`` of one block, refused unless a whole number."""
    rate = float(rate_hz)
    if rate <= 0.0:
        raise ValueError(f"rates_hz[{sys_id!r}] must be positive, got {rate_hz}")
    if rate > base_hz + 1e-9:
        raise ValueError(
            f"rates_hz[{sys_id!r}]={rate_hz} exceeds base rate {base_hz} Hz"
        )

    # d = f_base / f, a whole number of ticks to 10⁻⁹
    divisor = base_hz / rate
    rounded = round(divisor)
    if abs(divisor - rounded) > 1e-9 or rounded < 1:
        raise ValueError(
            f"rates_hz[{sys_id!r}]={rate_hz} is not an integer divisor of "
            f"base rate {base_hz} Hz"
        )

    return int(rounded)


def build_computer(
    side: System | StepDiagramSystem,
    schedule: StepSchedule | float,
) -> Computer:
    """Plain build behind ``block % schedule``: never defers to the block's own ``%``."""
    # A real cycle: core.hybrid_composition imports Computer from this module
    from minilink.core.hybrid_composition import (
        as_step_diagram,
        expose_computer_boundary_ports,
    )

    require_computer_side(side)
    step_schedule = as_schedule(schedule)

    # The block in a step diagram, with the standard boundary ports exposed
    diagram = as_step_diagram(side)
    expose_computer_boundary_ports(diagram)

    return Computer(diagram, step_schedule)


def require_computer_side(side) -> None:
    """Refuse what cannot run on a computer: a Computer, a flow diagram, a continuous plant."""
    if isinstance(side, Computer):
        raise TypeError("operand is already a Computer")
    if isinstance(side, DiagramSystem) and not isinstance(side, StepDiagramSystem):
        raise TypeError(
            f"as_computer() requires a step block or StepDiagramSystem, "
            f"got flow {type(side).__name__}"
        )
    if isinstance(side, DynamicSystem) and side.n > 0:
        raise TypeError(
            f"continuous plant {side.name!r} belongs in Simulator, not Computer"
        )
    if not isinstance(side, (StepDiagramSystem, System)):
        raise TypeError(
            f"as_computer() expected System or StepDiagramSystem, "
            f"got {type(side).__name__}"
        )


def as_schedule(schedule: StepSchedule | float) -> StepSchedule:
    """The schedule as given, or a single-rate one ticking every ``schedule`` seconds."""
    if isinstance(schedule, StepSchedule):
        return schedule

    return StepSchedule(dt_base=float(schedule))


def require_own_computer(side, computer) -> None:
    """Refuse a block's own ``%`` that returned something other than a Computer."""
    if not isinstance(computer, Computer):
        raise TypeError(
            f"{type(side).__name__} % schedule returned "
            f"{type(computer).__name__}, not a Computer"
        )
