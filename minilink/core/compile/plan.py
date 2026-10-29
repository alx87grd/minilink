"""
The execution plans of compiled diagrams: a flow diagram's and a step diagram's.

A plan is the output of the compilation step: it captures, in an immutable
and flattened form, all the information needed to evaluate a diagram without
any dictionary lookups or recursive calls. :class:`ExecutionPlan` (a
:class:`~minilink.core.diagram.DiagramSystem`) holds ``state_ops``, one ``dx``
per stateful block; :class:`StepExecutionPlan` (a
:class:`~minilink.core.discrete.step_diagram.StepDiagramSystem`) holds ``step_ops``, one
``x_{k+1}`` per stateful block. Both share the port topology.

:func:`build_execution_plan` and :func:`build_step_execution_plan` build them;
the evaluators
(:class:`~minilink.core.compile.evaluators.diagram.DiagramEvaluator`,
:class:`~minilink.core.compile.evaluators.discrete.StepDiagramEvaluator`)
walk the operation lists in topological order.
"""

from __future__ import annotations

import copy
from collections.abc import Callable
from dataclasses import dataclass
from functools import cached_property
from typing import TYPE_CHECKING

import numpy as np

from minilink.core.discrete.step_diagram import StepDiagramSystem
from minilink.core.discrete.step_system import StepSystem
from minilink.core.system import DynamicSystem
from minilink.core.topology import evaluation_order_of
from minilink.core.wiring import WiredDiagramMixin

if TYPE_CHECKING:
    from minilink.core.diagram import DiagramSystem


# The name of the plant's input among a run's signals: the plant role's u
PLANT_INPUT = "plant:u"

###############################################################################
# Public API
###############################################################################

# Source-type constants
# Used in the ``gather_sources`` tuples of PortOperation / StateOperation.

NOMINAL = 0
"""Source is a constant value (port not connected, or not a dependency)."""

EXTERNAL_INPUT = 1
"""Source is an external input to the diagram (from the diagram's ``u`` vector)."""

INTERNAL_SIGNAL = 2
"""Source is an output from another subsystem (from the internal signal buffer)."""


# Operation dataclasses
@dataclass(frozen=True)
class PortOperation:
    """One output-port evaluation step in topological order.

    When a diagram is compiled, every output port becomes a PortOperation.
    During evaluation, operations execute sequentially in topological order:
    for each operation, gather the local input vector ``u`` from the signal
    buffer, call the compute function, and write the result into the buffer.

    Attributes
    ----------
    compute_func : Callable[..., np.ndarray]
        The port's compute function ``(x, u, t, params) → y``; ``params`` is
        :attr:`bound_params` unless the caller passes its own.
    local_x_slice : slice
        Index slice into the global state vector ``x`` that extracts this
        subsystem's local state.  For static systems (``n = 0``): ``slice(0, 0)``.
    gather_sources : tuple[tuple[int, Any, int], ...]
        Recipe for assembling the local input vector ``u`` from various sources.
        Each entry is ``(source_type, source_value, dim)`` where:

        - ``source_type = NOMINAL (0)``:
          Port not connected or not a dependency.
          ``source_value`` is the constant ``np.ndarray`` (nominal value).
        - ``source_type = EXTERNAL_INPUT (1)``:
          From the diagram's external input vector.
          ``source_value`` is a ``slice`` into the diagram's ``u``.
        - ``source_type = INTERNAL_SIGNAL (2)``:
          From another subsystem's output.
          ``source_value`` is a ``slice`` into the internal signal buffer.
    out_slice : slice
        Where to write this port's output in the internal signal buffer.
    u_dim : int
        Total dimension of the assembled local input vector.
        Equal to the sum of all ``dim`` values in ``gather_sources``.
    bound_params : dict | None
        The subsystem's ``params`` as compiled, a deep copy. The evaluator
        collects these into ``evaluator.params``; ``params=None`` reads that
        tree and each op takes ``params[sys_id]``. It does not freeze other
        instance attributes; purity of ``compute_func`` is not enforced.
    sys_id : str
        Id of the owning subsystem in the diagram. Used by the parametric
        evaluator tier to route nested diagram params (``params[sys_id]``).
    """

    compute_func: Callable[..., np.ndarray]
    local_x_slice: slice
    gather_sources: tuple[tuple[int, object, int], ...]
    out_slice: slice
    u_dim: int
    bound_params: dict | None = None
    label: str = ""
    sys_id: str = ""


@dataclass(frozen=True)
class StateOperation:
    """One state-derivative evaluation step.

    Similar to :class:`PortOperation`, but calls ``f(x, u, t) → dx`` instead
    of ``h(x, u, t) → y``.  The result is written into the global derivative
    vector ``dx`` at ``local_x_slice``.

    Attributes
    ----------
    f_func : Callable[..., np.ndarray]
        The subsystem's state-derivative function ``(x, u, t, params) → dx``;
        ``params`` is :attr:`bound_params` unless the caller passes its own.
    local_x_slice : slice
        Index slice into the global state vector ``x`` and derivative vector
        ``dx`` for this subsystem's local state.
    gather_sources : tuple[tuple[int, Any, int], ...]
        Recipe for assembling the local input vector ``u``.
        Same format as :attr:`PortOperation.gather_sources` — the ``f``
        function always needs **all** inputs (no dependency filtering).
    u_dim : int
        Total dimension of the assembled local input vector.
    bound_params : dict | None
        Same semantics as :attr:`PortOperation.bound_params` for ``f_func``
        (``params`` dict snapshot only; see :class:`minilink.core.system.System`).
    sys_id : str
        Same semantics as :attr:`PortOperation.sys_id`.
    """

    f_func: Callable[..., np.ndarray]
    local_x_slice: slice
    gather_sources: tuple[tuple[int, object, int], ...]
    u_dim: int
    bound_params: dict | None = None
    label: str = ""
    sys_id: str = ""


# Execution plan
@dataclass(frozen=True)
class ExecutionPlan:
    """Immutable flattened execution schedule for a compiled diagram.

    Built by :func:`~minilink.core.compile.plan.build_execution_plan`.
    Consumed by evaluator backends to compute state derivatives and outputs
    without any dictionary lookups or recursive traversals.

    Attributes
    ----------
    state_dim : int
        Total state dimension of the diagram (sum of all subsystem ``n`` values).
    signal_dim : int
        Total size of the internal signal buffer (sum of all output port
        dimensions across all subsystems).
    port_ops : tuple[PortOperation, ...]
        Output-port operations in topological order.  Evaluating them
        sequentially fills the internal signal buffer.
    state_ops : tuple[StateOperation, ...]
        State-derivative operations.  Evaluating them sequentially (after
        the port operations) fills the derivative vector ``dx``.
    output_slices : dict[tuple[str, str], slice]
        Map from ``(subsystem_id, port_id)`` to the corresponding ``slice``
        in the internal signal buffer.  Used to extract specific port outputs
        after evaluation.
    external_output_slices : dict[str, slice]
        Diagram **boundary** outputs only: maps diagram output port id (keys of
        :attr:`~minilink.core.diagram.DiagramSystem.outputs` on the diagram) to
        the slice in the internal buffer that holds the connected source
        subsystem port's signal. Empty when the diagram has no external outputs.
    """

    state_dim: int
    signal_dim: int
    port_ops: tuple[PortOperation, ...]
    state_ops: tuple[StateOperation, ...]
    output_slices: dict[tuple[str, str], slice]
    external_output_slices: dict[str, slice]


@dataclass(frozen=True)
class StepOperation:
    """One discrete state-advance step in a compiled step diagram.

    Calls ``step(x, u, k, params) -> x_new`` and writes the result into the
    global state vector at ``local_x_slice``; the fields read as on
    :class:`StateOperation`.
    """

    step_func: Callable[..., object]
    local_x_slice: slice
    gather_sources: tuple[tuple[int, object, int], ...]
    u_dim: int
    bound_params: dict | None = None
    label: str = ""
    sys_id: str = ""


@dataclass(frozen=True)
class StepExecutionPlan:
    """Immutable flattened schedule for a compiled step diagram.

    Built by :func:`~minilink.core.compile.plan.build_step_execution_plan`;
    the fields read as on :class:`ExecutionPlan`, ``step_ops`` in place of
    ``state_ops``.
    """

    state_dim: int
    signal_dim: int
    port_ops: tuple[PortOperation, ...]
    step_ops: tuple[StepOperation, ...]
    output_slices: dict[tuple[str, str], slice]
    external_output_slices: dict[str, slice]


def build_execution_plan(diagram: DiagramSystem) -> ExecutionPlan:
    """Build an immutable ExecutionPlan from a DiagramSystem.

    This is the core compilation step.  It:

    1. Orders the output ports, refusing an algebraic loop, via
       :func:`~minilink.core.topology.evaluation_order_of`.
    2. Assigns each output port a slice in the flat signal buffer.
    3. Builds the gather-source recipes for each port and state operation, each
       with its block's ``params`` as compiled (one deep copy per block).
    4. Returns a frozen :class:`ExecutionPlan`.

    Parameters
    ----------
    diagram : DiagramSystem
        Must have subsystems added and connections wired.

    Returns
    -------
    ExecutionPlan
    """
    port_execution_order = evaluation_order_of(diagram)
    return execution_plan_from_order(diagram, port_execution_order)


def build_step_execution_plan(diagram: StepDiagramSystem) -> StepExecutionPlan:
    """Build an immutable :class:`StepExecutionPlan` from a step diagram."""
    port_execution_order = evaluation_order_of(diagram)
    return step_execution_plan_from_order(diagram, port_execution_order)


def execution_plan_from_order(
    diagram: DiagramSystem,
    port_execution_order: list[tuple[str, str]],
) -> ExecutionPlan:
    """Build the execution plan given a pre-computed topological order.

    This is the inner implementation of :func:`build_execution_plan`,
    split out so that :func:`compile_diagram` can time each step
    individually.
    """
    output_slices, signal_dim = signal_layout(diagram)
    snapshot = params_snapshot(diagram)
    port_ops = port_operations(
        diagram, port_execution_order, output_slices, DynamicSystem, snapshot
    )

    # One dx per continuous block
    state_ops = tuple(
        StateOperation(f_func=sys.f, **fields)
        for sys, fields in state_fields(diagram, output_slices, DynamicSystem, snapshot)
    )

    return ExecutionPlan(
        state_dim=diagram.n,
        signal_dim=signal_dim,
        port_ops=port_ops,
        state_ops=state_ops,
        output_slices=output_slices,
        external_output_slices=boundary_output_slices(diagram, output_slices),
    )


def step_execution_plan_from_order(
    diagram: StepDiagramSystem,
    port_execution_order: list[tuple[str, str]],
) -> StepExecutionPlan:
    """The step twin of :func:`execution_plan_from_order`: ``step_ops`` for ``state_ops``."""
    output_slices, signal_dim = signal_layout(diagram)
    snapshot = params_snapshot(diagram)
    port_ops = port_operations(
        diagram, port_execution_order, output_slices, StepSystem, snapshot
    )

    # One x_{k+1} per discrete block
    step_ops = tuple(
        StepOperation(step_func=sys.step, **fields)
        for sys, fields in state_fields(diagram, output_slices, StepSystem, snapshot)
    )

    return StepExecutionPlan(
        state_dim=diagram.n,
        signal_dim=signal_dim,
        port_ops=port_ops,
        step_ops=step_ops,
        output_slices=output_slices,
        external_output_slices=boundary_output_slices(diagram, output_slices),
    )


###############################################################################
# Internal machinery
###############################################################################


def signal_layout(diagram):
    """Every output port's slice of the flat signal buffer, and the buffer's size."""
    output_slices: dict[tuple[str, str], slice] = {}
    current_idx = 0
    for sys_id, sys in diagram.subsystems.items():
        for port_id, port in sys.outputs.items():
            dim = port.dim
            output_slices[(sys_id, port_id)] = slice(current_idx, current_idx + dim)
            current_idx += dim

    return output_slices, current_idx


def params_snapshot(diagram):
    """Every block's ``params`` as compiled: one deep copy per block, by block id."""
    return {
        sys_id: copy.deepcopy(getattr(sys, "params", {}))
        for sys_id, sys in diagram.subsystems.items()
    }


def port_operations(diagram, port_execution_order, output_slices, stateful, snapshot):
    """One :class:`PortOperation` per output port, in topological order."""
    port_ops: list[PortOperation] = []
    for sys_id, port_id in port_execution_order:
        sys = diagram.subsystems[sys_id]
        port = sys.outputs[port_id]

        gather_sources, u_dim = gather_sources_of(
            diagram, sys_id, output_slices, dependencies=port.dependencies
        )

        out_slice = output_slices[(sys_id, port_id)]
        local_x_slice = state_slice_of(diagram, sys_id, stateful)

        port_ops.append(
            PortOperation(
                compute_func=port.compute,
                local_x_slice=local_x_slice,
                gather_sources=tuple(gather_sources),
                out_slice=out_slice,
                u_dim=u_dim,
                bound_params=snapshot[sys_id],
                label=f"{sys_id}:{port_id}",
                sys_id=sys_id,
            )
        )

    return tuple(port_ops)


def state_fields(diagram, output_slices, stateful, snapshot):
    """``(block, fields)`` for every ``stateful`` block: the fields its state operation shares."""
    for sys_id, sys in diagram.subsystems.items():
        if not isinstance(sys, stateful):
            continue
        gather_sources, u_dim = gather_sources_of(
            diagram, sys_id, output_slices, dependencies="all"
        )
        fields = {
            "local_x_slice": state_slice_of(diagram, sys_id, stateful),
            "gather_sources": tuple(gather_sources),
            "u_dim": u_dim,
            "bound_params": snapshot[sys_id],
            "label": sys_id,
            "sys_id": sys_id,
        }
        yield sys, fields


def boundary_output_slices(diagram, output_slices):
    """Each diagram output port's slice: the slice of the block port wired to it."""
    external_output_slices: dict[str, slice] = {}
    out_conns = diagram.connections.get("output", {})
    for out_port_id in diagram.outputs:
        src = out_conns.get(out_port_id)
        if src is None:
            continue
        source_sys_id, source_port_id = src
        key = (source_sys_id, source_port_id)
        if key not in output_slices:
            raise RuntimeError(
                f"Diagram output {out_port_id!r} source {key} missing from output_slices"
            )
        external_output_slices[out_port_id] = output_slices[key]

    return external_output_slices


def gather_sources_of(
    diagram: DiagramSystem,
    sys_id: str,
    output_slices: dict[tuple[str, str], slice],
    dependencies="all",
) -> tuple[list[tuple[int, object, int]], int]:
    """Build the gather-source recipe for a subsystem's input ports.

    This is the **single implementation** of the signal-gathering logic that
    was previously duplicated across the codebase.

    Parameters
    ----------
    diagram : DiagramSystem
    sys_id : str
        The subsystem whose inputs to gather.
    output_slices : dict
        Pre-computed ``(sys_id, port_id) → slice`` map.
    dependencies : list or ``"all"``
        Which input ports are required.  ``"all"`` means every input port.

    Returns
    -------
    gather_sources : list[tuple[int, Any, int]]
        List of ``(source_type, source_value, dim)`` entries.
    u_dim : int
        Total dimension of the assembled local input vector.
    """
    sys = diagram.subsystems[sys_id]
    gather_sources: list[tuple[int, object, int]] = []
    u_dim = 0

    for in_port_id, in_port in sys.inputs.items():
        # If this input port is not a dependency, use the nominal value
        if dependencies != "all" and in_port_id not in dependencies:
            source_type = NOMINAL
            source_val = in_port.nominal_value
        else:
            source = diagram.connections[sys_id].get(in_port_id)
            if source is None:
                # Not connected → use nominal value
                source_type = NOMINAL
                source_val = in_port.nominal_value
            else:
                src_sys_id, src_port_id = source
                if src_sys_id == "input":
                    source_type = EXTERNAL_INPUT
                    u_idx = 0
                    for input_port_id, input_port in diagram.inputs.items():
                        if input_port_id == src_port_id:
                            source_val = slice(u_idx, u_idx + input_port.dim)
                            break
                        u_idx += input_port.dim
                else:
                    source_type = INTERNAL_SIGNAL
                    source_val = output_slices[(src_sys_id, src_port_id)]

        dim = in_port.dim
        gather_sources.append((source_type, source_val, dim))
        u_dim += dim

    return gather_sources, u_dim


def state_slice_of(diagram, sys_id: str, stateful=DynamicSystem) -> slice:
    """Return the global state-vector slice for a subsystem (empty unless ``stateful``)."""
    sys = diagram.subsystems[sys_id]
    if isinstance(sys, stateful):
        start, end = diagram.state_index[sys_id]
        return slice(start, end)
    return slice(0, 0)


# The sampled plan: a step plan on a clock


@dataclass(frozen=True)
class SampledPortOperation:
    """One output-port evaluation of a sampled plan: a :class:`PortOperation` and its clock.

    ``face`` says what the block reads as time: ``"seconds"`` (``t_k``, a static
    block or a system on a clock), ``"count"`` (``k // divisor``, a tick law on a
    clock) or ``"base"`` (``k``, a bare tick law). A block of ``divisor > 1``
    computes its output on the ticks ``k mod divisor = 0`` and reads it from
    ``hold_slice`` of the state in between. ``params_path`` is the block's keys
    in the diagram's nested parameter tree.
    """

    compute_func: Callable[..., np.ndarray]
    local_x_slice: slice
    gather_sources: tuple[tuple[int, object, int], ...]
    out_slice: slice
    u_dim: int
    face: str
    divisor: int
    hold_slice: slice | None
    params_path: tuple[str, ...]
    bound_params: dict | None = None
    label: str = ""
    sys_id: str = ""


@dataclass(frozen=True)
class SampledStepOperation:
    """One state advance of a sampled plan: ``update(x, u, t_k)`` or ``step(x, u, k)``, on its clock.

    ``system`` is the block itself, which the dense run and the host clock
    runner read (its substeps, its continuous source).
    """

    step_func: Callable[..., object]
    local_x_slice: slice
    gather_sources: tuple[tuple[int, object, int], ...]
    u_dim: int
    face: str
    divisor: int
    params_path: tuple[str, ...]
    system: object = None
    bound_params: dict | None = None
    label: str = ""
    sys_id: str = ""


@dataclass(frozen=True)
class SampledExecutionPlan:
    """Immutable flattened schedule of a sampled diagram: its blocks' operations on one base clock.

    ``base_period`` is ``Δ``; nested boxes are inlined (``output_slices`` keyed by
    the flat ids ``box__block``), while every slice of the state reads the
    nested layout of the diagram, held outputs included.
    """

    state_dim: int
    signal_dim: int
    base_period: float
    port_ops: tuple[SampledPortOperation, ...]
    step_ops: tuple[SampledStepOperation, ...]
    output_slices: dict[tuple[str, str], slice]
    external_output_slices: dict[str, slice]
    paths: dict[str, tuple[str, ...]] = None

    def output_slice_of(self, path, port_id) -> slice:
        """The signal buffer's slice of the leaf at ``path`` (its keys in the nested diagram), port ``port_id``."""
        flat_id = next(f for f, p in self.paths.items() if p == tuple(path))
        return self.output_slices[(flat_id, port_id)]

    @cached_property
    def plant_op(self):
        """The state advance of the loop's plant: the block integrated inside every base period, else ``None``.

        A continuous block on the base clock is the plant whatever its id; of
        several, the one whose id is ``plant``, at any depth.
        """
        candidates = [
            op
            for op in self.step_ops
            if op.divisor == 1 and op.face == "seconds" and op.system.resolves
        ]
        if len(candidates) > 1:
            candidates = [op for op in candidates if op.params_path[-1] == "plant"]
            require_one_plant(candidates, self.step_ops)

        return candidates[0] if candidates else None

    @property
    def dense_points(self) -> int:
        """``S``, the points per base period the dense run keeps: the plant's substeps (1 with no plant)."""
        op = self.plant_op
        return 1 if op is None else op.system.substeps


def build_sampled_execution_plan(diagram) -> SampledExecutionPlan:
    """Build the :class:`SampledExecutionPlan` of a sampled diagram, its nested boxes inlined."""
    # A real cycle: the assembly imports the diagrams, which import this module's users
    from minilink.core.assembly import flatten_diagram
    from minilink.core.discrete.sampled_system import SampledSystem

    base = diagram.sample_period
    require_clock(diagram, base)
    require_tick_laws_on_their_box_clock(diagram, base)

    # The wiring of one flat level; the slices of the nested layout
    flat, report = flatten_diagram(diagram)
    layout = nested_layout(diagram)
    paths = {flat_id: tuple(path.split("/")) for path, flat_id in report.id_map.items()}
    order = evaluation_order_of(flat)
    output_slices, signal_dim = signal_layout(flat)
    snapshot = params_snapshot(flat)

    port_ops = []
    for sys_id, port_id in order:
        sys = flat.subsystems[sys_id]
        path = paths[sys_id]
        x_slice, holds = layout[path]
        gather_sources, u_dim = gather_sources_of(
            flat, sys_id, output_slices, dependencies=sys.outputs[port_id].dependencies
        )
        port_ops.append(
            SampledPortOperation(
                compute_func=sys.outputs[port_id].compute,
                local_x_slice=x_slice,
                gather_sources=tuple(gather_sources),
                out_slice=output_slices[(sys_id, port_id)],
                u_dim=u_dim,
                face=face_of(sys, SampledSystem),
                divisor=diagram.divisor_of(sys),
                hold_slice=holds.get(port_id),
                params_path=path,
                bound_params=snapshot[sys_id],
                label=f"{'/'.join(path)}:{port_id}",
                sys_id=sys_id,
            )
        )

    # One state advance per block with a state: the seconds face on a clock, the step otherwise
    step_ops = []
    for sys_id, sys in flat.subsystems.items():
        if not isinstance(sys, StepSystem) or sys.n == 0:
            continue
        path = paths[sys_id]
        face = face_of(sys, SampledSystem)
        gather_sources, u_dim = gather_sources_of(
            flat, sys_id, output_slices, dependencies="all"
        )
        step_ops.append(
            SampledStepOperation(
                step_func=sys.update if face == "seconds" else sys.step,
                local_x_slice=layout[path][0],
                gather_sources=tuple(gather_sources),
                u_dim=u_dim,
                face=face,
                divisor=diagram.divisor_of(sys),
                params_path=path,
                system=sys,
                bound_params=snapshot[sys_id],
                label="/".join(path),
                sys_id=sys_id,
            )
        )

    external = {}
    for out_port_id in diagram.outputs:
        src = flat.connections.get("output", {}).get(out_port_id)
        if src is not None:
            external[out_port_id] = output_slices[tuple(src)]

    return SampledExecutionPlan(
        state_dim=diagram.n,
        signal_dim=signal_dim,
        base_period=base,
        port_ops=tuple(port_ops),
        step_ops=tuple(step_ops),
        output_slices=output_slices,
        external_output_slices=external,
        paths=paths,
    )


def require_tick_laws_on_their_box_clock(diagram, base, prefix=()):
    """
    Refuse a bare tick law in a nested box whose own clock is slower than the loop's base.

    A bare tick law ticks on the base clock of the loop it runs in (R158): in a
    box clocked at ``Δ_box`` nested in a loop of finer base ``Δ``, it would tick
    ``Δ_box / Δ`` times faster than in its box alone. It takes its own clock
    instead (``law % Δ_box``).
    """
    for sys_id, sys in diagram.subsystems.items():
        if not isinstance(sys, WiredDiagramMixin):
            continue
        box_base = getattr(sys, "sample_period", None)
        for inner_id, inner in sys.subsystems.items():
            bare = isinstance(inner, StepSystem) and inner.sample_period is None
            bare = bare and not isinstance(inner, WiredDiagramMixin)
            if bare and box_base is not None and abs(box_base - base) > 1e-12 * base:
                path = "/".join((*prefix, sys_id, inner_id))
                raise TypeError(
                    f"the tick law {path!r} has no clock of its own: in its box it ticks "
                    f"every {box_base:g} s, in this loop every {base:g} s; give it its "
                    f"box's clock ({inner.name} % {box_base:g})"
                )
        require_tick_laws_on_their_box_clock(sys, base, (*prefix, sys_id))


def face_of(sys, sampled_kind) -> str:
    """What the block reads as time in a sampled plan: ``"seconds"``, ``"count"`` or ``"base"``."""
    if isinstance(sys, sampled_kind):
        return "count" if sys.counts else "seconds"
    if isinstance(sys, StepSystem):
        return "base"

    return "seconds"


def nested_layout(diagram, offset=0, prefix=()):
    """``path → (state slice, {port: hold slice})`` of every leaf, in the diagram's nested layout."""
    layout = {}
    for sys_id, sys in diagram.subsystems.items():
        start, end = diagram.state_index[sys_id]
        path = (*prefix, sys_id)
        if isinstance(sys, WiredDiagramMixin):
            layout.update(nested_layout(sys, offset + start, path))
        else:
            layout[path] = (slice(offset + start, offset + end), {})
    for (path, port_id), (start, end) in getattr(diagram, "hold_index", {}).items():
        layout[(*prefix, *path)][1][port_id] = slice(offset + start, offset + end)

    return layout


def require_one_plant(candidates, step_ops) -> None:
    """Refuse a loop of several continuous blocks on the base clock none of which, or more than one, is the plant."""
    if len(candidates) != 1:
        integrated = [
            op.label
            for op in step_ops
            if op.divisor == 1 and op.face == "seconds" and op.system.resolves
        ]
        raise ValueError(
            f"the blocks {integrated} are all integrated inside the base period; the "
            "dense run keeps one plant's substeps: name it 'plant'"
        )


def require_clock(diagram, base) -> None:
    """Refuse a sampled diagram with no block on a clock: a tick diagram is a StepDiagramSystem."""
    if base is None:
        raise ValueError(
            f"{diagram.name} has no block on a clock: put one on a clock (block % dt) "
            "or build a StepDiagramSystem"
        )
