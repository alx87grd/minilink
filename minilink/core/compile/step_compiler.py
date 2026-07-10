"""
Step diagram compiler: topology analysis and step execution plan construction.
"""

from __future__ import annotations

import copy
import time
from typing import TYPE_CHECKING

from minilink.core.backends import BACKEND_NUMPY, normalize_backend, require_jax_numpy
from minilink.core.compile.compiler import _build_gather_sources
from minilink.core.compile.execution_plan import PortOperation
from minilink.core.compile.step_execution_plan import StepExecutionPlan, StepOperation
from minilink.core.system import DynamicSystem, StepSystem
from minilink.core.wiring import check_algebraic_loops

if TYPE_CHECKING:
    from minilink.core.diagram import StepDiagramSystem


def build_step_execution_plan(
    diagram: StepDiagramSystem, *, bind_params: bool = False
) -> StepExecutionPlan:
    """Build an immutable :class:`StepExecutionPlan` from a step diagram."""
    port_execution_order = check_algebraic_loops(diagram)
    return _build_step_execution_plan_from_order(
        diagram, port_execution_order, bind_params=bind_params
    )


def compile_step_diagram(
    diagram: StepDiagramSystem,
    backend: str = BACKEND_NUMPY,
    *,
    bind_params: bool = False,
    verbose: bool = False,
):
    """Compile a :class:`~minilink.core.diagram.StepDiagramSystem` into an evaluator."""
    from minilink.core.diagram import StepDiagramSystem as StepDiagramSystemType

    if not isinstance(diagram, StepDiagramSystemType):
        raise TypeError(
            f"compile_step_diagram requires StepDiagramSystem, got {type(diagram).__name__}"
        )

    for sys_id, subsystem in diagram.subsystems.items():
        if isinstance(subsystem, DynamicSystem):
            raise TypeError(
                f"DynamicSystem leaf '{sys_id}' cannot be compiled inside a step "
                "diagram; use DiagramSystem for continuous subsystems."
            )

    t_total = time.perf_counter() if verbose else None

    if verbose:
        t0 = time.perf_counter()
        print("[compile] Step 1: Checking for algebraic loops...", end="", flush=True)

    port_execution_order = check_algebraic_loops(diagram)

    if verbose:
        print(f"  ({time.perf_counter() - t0:.3f}s)")

    if verbose:
        t0 = time.perf_counter()
        n_ports = len(port_execution_order)
        n_states = sum(
            1 for s in diagram.subsystems.values() if isinstance(s, StepSystem)
        )
        print(
            f"[compile] Step 2: Building step execution plan "
            f"({n_ports} ports, {n_states} step states)...",
            end="",
            flush=True,
        )

    plan = _build_step_execution_plan_from_order(
        diagram, port_execution_order, bind_params=bind_params
    )

    if verbose:
        print(f"  ({time.perf_counter() - t0:.3f}s)")

    key = normalize_backend(backend)
    if key == BACKEND_NUMPY:
        from minilink.core.compile.evaluators.step_diagram_evaluator import (
            NumpyStepDiagramEvaluator,
        )

        evaluator = NumpyStepDiagramEvaluator(plan, diagram)
    else:
        require_jax_numpy()
        from minilink.core.compile.evaluators.step_diagram_evaluator import (
            JaxStepDiagramEvaluator,
        )

        evaluator = JaxStepDiagramEvaluator(plan, diagram, verbose=verbose)

    if verbose:
        print(f"[compile] Done.  ({time.perf_counter() - t_total:.3f}s total)")

    return evaluator


def _build_step_execution_plan_from_order(
    diagram,
    port_execution_order: list[tuple[str, str]],
    *,
    bind_params: bool = False,
) -> StepExecutionPlan:
    output_slices: dict[tuple[str, str], slice] = {}
    current_idx = 0
    for sys_id, sys in diagram.subsystems.items():
        for port_id, port in sys.outputs.items():
            dim = port.dim
            output_slices[(sys_id, port_id)] = slice(current_idx, current_idx + dim)
            current_idx += dim
    signal_dim = current_idx

    port_ops: list[PortOperation] = []
    for sys_id, port_id in port_execution_order:
        sys = diagram.subsystems[sys_id]
        port = sys.outputs[port_id]

        gather_sources, u_dim = _build_gather_sources(
            diagram, sys_id, output_slices, dependencies=port.dependencies
        )

        out_slice = output_slices[(sys_id, port_id)]
        local_x_slice = _step_state_slice(diagram, sys_id)

        bound = copy.deepcopy(getattr(sys, "params", {})) if bind_params else None

        port_ops.append(
            PortOperation(
                compute_func=port.compute,
                local_x_slice=local_x_slice,
                gather_sources=tuple(gather_sources),
                out_slice=out_slice,
                u_dim=u_dim,
                bound_params=bound,
                label=f"{sys_id}:{port_id}",
                sys_id=sys_id,
            )
        )

    step_ops: list[StepOperation] = []
    for sys_id, sys in diagram.subsystems.items():
        if not isinstance(sys, StepSystem):
            continue
        gather_sources, u_dim = _build_gather_sources(
            diagram, sys_id, output_slices, dependencies="all"
        )
        local_x_slice = _step_state_slice(diagram, sys_id)
        bound = copy.deepcopy(getattr(sys, "params", {})) if bind_params else None
        step_ops.append(
            StepOperation(
                step_func=sys.step,
                local_x_slice=local_x_slice,
                gather_sources=tuple(gather_sources),
                u_dim=u_dim,
                bound_params=bound,
                label=sys_id,
                sys_id=sys_id,
            )
        )

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

    return StepExecutionPlan(
        state_dim=diagram.n,
        signal_dim=signal_dim,
        port_ops=tuple(port_ops),
        step_ops=tuple(step_ops),
        output_slices=output_slices,
        external_output_slices=external_output_slices,
    )


def _step_state_slice(diagram, sys_id: str) -> slice:
    """Global state-vector slice for a step-diagram subsystem."""
    sys = diagram.subsystems[sys_id]
    if isinstance(sys, StepSystem):
        start, end = diagram.state_index[sys_id]
        return slice(start, end)
    return slice(0, 0)
