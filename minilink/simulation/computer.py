"""
Scheduled discrete simulation for compiled step diagrams.

Stateful :class:`Computer` runtime driven by :class:`StepSchedule` — same
:class:`~minilink.core.compile.step_execution_plan.StepExecutionPlan` lowering as
synchronous ``compute_rollout``, with multi-rate firing and double-buffered signals.
"""

from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass, field

import numpy as np

from minilink.core.backends import BACKEND_NUMPY, normalize_backend
from minilink.core.compile.evaluators.numpy_evaluator import _gather_u
from minilink.core.compile.execution_plan import PortOperation
from minilink.core.compile.step_compiler import build_step_execution_plan
from minilink.core.compile.step_execution_plan import StepExecutionPlan, StepOperation
from minilink.core.diagram import StepDiagramSystem

# Public API


@dataclass
class StepSchedule:
    """Firing schedule for a :class:`Computer` runtime."""

    dt_base: float
    fire: dict[str, int] = field(default_factory=dict)
    # sys_id -> divisor d; block fires when k % d == 0

    def __post_init__(self) -> None:
        self.dt_base = _validate_dt_base(self.dt_base)
        for sys_id, divisor in self.fire.items():
            d = int(divisor)
            if d < 1:
                raise ValueError(f"fire[{sys_id!r}] must be >= 1, got {divisor}")
            self.fire[sys_id] = d

    @classmethod
    def from_rates(cls, dt_base: float, rates_hz: dict[str, float]) -> StepSchedule:
        """Build ``fire`` divisors from subsystem rates in Hz."""
        base = _base_hz(dt_base)
        fire = {
            sys_id: _rate_to_divisor(base, rate, sys_id)
            for sys_id, rate in rates_hz.items()
        }
        return cls(dt_base=dt_base, fire=fire)


class Computer:
    """
    Stateful scheduled runtime for a :class:`StepDiagramSystem`.

    Compiles the same :class:`StepExecutionPlan` as synchronous rollout, then
    advances with :meth:`tick` under :attr:`schedule` divisors and double-buffered
    internal port signals.

    Within each tick, fired blocks gather inputs from the committed read buffer
    only (parallel / order-agnostic semantics). Skipped blocks hold via the
    write-buffer copy taken at tick start.
    """

    def __init__(self, diagram: StepDiagramSystem, schedule: StepSchedule):
        if not isinstance(diagram, StepDiagramSystem):
            raise TypeError(
                f"Computer requires StepDiagramSystem, got {type(diagram).__name__}"
            )
        self.diagram = diagram
        self.schedule = schedule
        self.plan: StepExecutionPlan | None = None
        self.port_ops_by_sys: dict[str, tuple[PortOperation, ...]] = {}
        self.step_op_by_sys: dict[str, StepOperation | None] = {}
        self.all_sys_ids: tuple[str, ...] = ()
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
        _ = verbose  # reserved for parity with ``StepDiagramSystem.compile``
        key = normalize_backend(backend)
        if key != BACKEND_NUMPY:
            raise NotImplementedError("Computer supports backend='numpy' only")

        unknown = set(self.schedule.fire) - set(self.diagram.subsystems)
        if unknown:
            names = ", ".join(sorted(unknown))
            raise ValueError(
                f"StepSchedule.fire keys not in diagram subsystems: {names}"
            )

        plan = build_step_execution_plan(self.diagram, bind_params=bind_params)
        self.plan = plan
        self.port_ops_by_sys = _group_port_ops_by_sys(plan)
        self.step_op_by_sys = {sys_id: None for sys_id in self.diagram.subsystems}
        for op in plan.step_ops:
            self.step_op_by_sys[op.sys_id] = op
        self.all_sys_ids = tuple(self.diagram.subsystems)

    def reset(self, x0=None) -> None:
        """Reset tick index, state, and internal signal buffers."""
        if self.plan is None:
            raise RuntimeError("Computer.compile() must be called before reset()")

        self.k = 0
        if x0 is None:
            x_init = np.asarray(self.diagram.x0, dtype=float).reshape(
                self.plan.state_dim
            )
        else:
            x_init = np.asarray(x0, dtype=float).reshape(self.plan.state_dim)
        self.x = x_init.copy()
        self.signal_read = np.zeros(self.plan.signal_dim, dtype=float)
        self.signal_write = np.zeros(self.plan.signal_dim, dtype=float)

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
        if (
            self.plan is None
            or self.x is None
            or self.signal_read is None
            or self.signal_write is None
        ):
            raise RuntimeError(
                "Computer.compile() and reset() must be called before tick()"
            )

        plan = self.plan
        k_tick = self.k
        read = self.signal_read
        write = self.signal_write
        write[:] = read
        x_work = self.x.copy()
        u_arr = np.asarray(u, dtype=float).reshape(self.diagram.m)

        for sys_id in self.all_sys_ids:
            if k_tick % self.divisor(sys_id) != 0:
                continue

            for op in self.port_ops_by_sys.get(sys_id, ()):
                local_x = x_work[op.local_x_slice]
                local_u = _gather_u(op.gather_sources, op.u_dim, read, u_arr)
                write[op.out_slice] = op.compute_func(
                    local_x, local_u, k_tick, op.bound_params
                )

            step_op = self.step_op_by_sys.get(sys_id)
            if step_op is not None:
                local_x = x_work[step_op.local_x_slice]
                local_u = _gather_u(step_op.gather_sources, step_op.u_dim, read, u_arr)
                x_work[step_op.local_x_slice] = step_op.step_func(
                    local_x, local_u, k_tick, step_op.bound_params
                )

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
        Default hybrid feedback: computer boundary ``u`` → plant ``u``,
        plant ``y`` → computer ``y``.

        Requires ``schedule`` on this runtime; does not cross raw step/flow
        diagrams — only ``Computer @ DiagramSystem``.
        """
        from minilink.core.diagram import DiagramSystem
        from minilink.core.hybrid_composition import hybrid_closed_loop

        if not isinstance(plant, DiagramSystem):
            raise TypeError(
                f"Computer @ plant requires DiagramSystem plant, "
                f"got {type(plant).__name__}"
            )
        return hybrid_closed_loop(
            self.diagram,
            plant,
            schedule=self.schedule,
            computer=self,
        )

    def divisor(self, sys_id: str) -> int:
        """Return the fire divisor for ``sys_id`` (default 1)."""
        return self.schedule.fire.get(sys_id, 1)


# Internal machinery


def _validate_dt_base(dt_base: float) -> float:
    dt = float(dt_base)
    if dt <= 0.0:
        raise ValueError(f"dt_base must be positive, got {dt_base}")
    return dt


def _base_hz(dt_base: float) -> float:
    dt = _validate_dt_base(dt_base)
    base_hz = 1.0 / dt
    if abs(base_hz - round(base_hz)) > 1e-9:
        raise ValueError(
            f"dt_base={dt_base} implies non-integer base rate Hz={base_hz}; "
            "use a dt_base that yields an integer base rate for from_rates"
        )
    return base_hz


def _rate_to_divisor(base_hz: float, rate_hz: float, sys_id: str) -> int:
    rate = float(rate_hz)
    if rate <= 0.0:
        raise ValueError(f"rates_hz[{sys_id!r}] must be positive, got {rate_hz}")
    if rate > base_hz + 1e-9:
        raise ValueError(
            f"rates_hz[{sys_id!r}]={rate_hz} exceeds base rate {base_hz} Hz"
        )
    divisor = base_hz / rate
    rounded = round(divisor)
    if abs(divisor - rounded) > 1e-9 or rounded < 1:
        raise ValueError(
            f"rates_hz[{sys_id!r}]={rate_hz} is not an integer divisor of "
            f"base rate {base_hz} Hz"
        )
    return int(rounded)


def _group_port_ops_by_sys(
    plan: StepExecutionPlan,
) -> dict[str, tuple[PortOperation, ...]]:
    grouped: dict[str, list[PortOperation]] = defaultdict(list)
    for op in plan.port_ops:
        grouped[op.sys_id].append(op)
    return {sys_id: tuple(ops) for sys_id, ops in grouped.items()}
