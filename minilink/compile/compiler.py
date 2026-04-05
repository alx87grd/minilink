"""
Diagram compiler: topology analysis and execution plan construction.

This module provides the complete compilation pipeline:

1. :func:`check_algebraic_loops` — Standalone depth-first search-based
   algebraic loop detection.  Returns the topological execution order
   of output ports.
   Can be imported independently by :class:`~minilink.core.diagram.DiagramSystem`
   for use after manual wiring.

2. :func:`build_execution_plan` — Constructs an immutable
   :class:`~minilink.compile.execution_plan.ExecutionPlan` from a diagram.
   Calls :func:`check_algebraic_loops` internally.

3. :func:`compile_diagram` — Top-level entry point that builds the plan and
   wraps it in a backend evaluator.
"""

from __future__ import annotations

import copy
from typing import TYPE_CHECKING

from minilink.compile.execution_plan import (
    EXTERNAL_INPUT,
    INTERNAL_SIGNAL,
    NOMINAL,
    ExecutionPlan,
    PortOperation,
    StateOperation,
)

if TYPE_CHECKING:
    from minilink.core.diagram import DiagramSystem


# ── Public API ──────────────────���────────────────────────────────────


def compile(system, backend="numpy"):
    """Compile a System into a :class:`DynamicsEvaluator`.

    For leaf systems (non-diagram), wraps ``f``/``h`` with frozen params
    and nominal u snapshot, providing the full evaluator API (RK4, rollout,
    linearize, etc.).

    For diagrams, delegates to :func:`compile_diagram`.

    Parameters
    ----------
    system : System or DiagramSystem
        The system to compile.
    backend : str
        ``'numpy'`` or ``'jax'``.

    Returns
    -------
    DynamicsEvaluator
    """
    from minilink.core.diagram import DiagramSystem

    if isinstance(system, DiagramSystem):
        return compile_diagram(system, backend=backend)

    key = backend.strip().lower()
    if key == "numpy":
        from minilink.compile.numpy_evaluator import NumpyLeafEvaluator

        return NumpyLeafEvaluator(system)
    elif key == "jax":
        try:
            import jax  # noqa: F401
        except ImportError:
            raise ImportError(
                "JAX is required for the 'jax' backend. "
                "Install with: pip install jax jaxlib"
            )
        from minilink.compile.jax_evaluator import JaxLeafEvaluator

        return JaxLeafEvaluator(system)
    else:
        raise ValueError(f"Unknown backend {backend!r}. Expected 'numpy' or 'jax'.")


def compile_diagram(
    diagram: DiagramSystem, backend: str = "numpy", *, bind_params: bool = False
):
    """Compile a DiagramSystem into a backend evaluator.

    Parameters
    ----------
    diagram : DiagramSystem
        The wired diagram to compile.
    backend : str
        ``'numpy'`` or ``'jax'``.
    bind_params : bool, optional
        If ``True``, each plan operation stores a deep copy of that subsystem's
        ``params`` and evaluators pass it into ``f`` / port ``compute``. If
        ``False`` (default), ``None`` is passed and blocks use live ``self.params``.

    Returns
    -------
    NumpyDiagramEvaluator or JaxDiagramEvaluator
        A stateless evaluator that can compute state derivatives and outputs.

    Examples
    --------
    >>> from minilink.compile import compile_diagram
    >>> evaluator = compile_diagram(diagram)
    >>> dx = evaluator.f(x, u, t)

    Notes
    -----
    ``bind_params=True`` snapshots only each subsystem's ``params`` dict into the plan.
    It does **not** make user ``f`` / port ``compute`` implementations pure if they still
    read or mutate other instance state; see :class:`minilink.core.framework.System`.
    """
    plan = build_execution_plan(diagram, bind_params=bind_params)

    key = backend.strip().lower()
    if key == "numpy":
        from minilink.compile.numpy_evaluator import NumpyDiagramEvaluator

        return NumpyDiagramEvaluator(plan, diagram)
    elif key == "jax":
        try:
            import jax  # noqa: F401
        except ImportError:
            raise ImportError(
                "JAX is required for the 'jax' backend. "
                "Install with: pip install jax jaxlib"
            )
        from minilink.compile.jax_evaluator import JaxDiagramEvaluator

        return JaxDiagramEvaluator(plan, diagram)
    else:
        raise ValueError(f"Unknown backend {backend!r}. Expected 'numpy' or 'jax'.")


def build_execution_plan(
    diagram: DiagramSystem, *, bind_params: bool = False
) -> ExecutionPlan:
    """Build an immutable ExecutionPlan from a DiagramSystem.

    This is the core compilation step.  It:
    1. Runs algebraic loop detection via :func:`check_algebraic_loops`.
    2. Assigns each output port a slice in the flat signal buffer.
    3. Builds the gather-source recipes for each port and state operation.
    4. Returns a frozen :class:`ExecutionPlan`.

    Parameters
    ----------
    diagram : DiagramSystem
        Must have subsystems added and connections wired.
    bind_params : bool, optional
        When ``True``, set :attr:`~minilink.compile.execution_plan.PortOperation.bound_params`
        / :attr:`~minilink.compile.execution_plan.StateOperation.bound_params` on each row
        (deep copy of ``subsystem.params`` only; see :func:`compile_diagram` Notes).

    Returns
    -------
    ExecutionPlan
    """
    port_execution_order = check_algebraic_loops(diagram)

    # ── 1. Map all output ports to slices in the flat signal buffer ───
    output_slices: dict[tuple[str, str], slice] = {}
    current_idx = 0
    for sys_id, sys in diagram.subsystems.items():
        for port_id, port in sys.outputs.items():
            dim = port.dim
            output_slices[(sys_id, port_id)] = slice(current_idx, current_idx + dim)
            current_idx += dim
    signal_dim = current_idx

    # ── 2. Build PortOperation list (output ports, topological order) ─
    port_ops: list[PortOperation] = []
    for sys_id, port_id in port_execution_order:
        sys = diagram.subsystems[sys_id]
        port = sys.outputs[port_id]

        # Determine the 'recipe' for gathering all inputs required by this output port
        # This includes mapping which signals come from global 'u', which from the
        # internal signal buffer, and which use nominal constant values.
        # gather_sources: list[tuple[int, object, int]] — list of (source_type, source_val, dim)
        # u_dim: int — total flattened dimension of the local subsystem input vector 'u'
        gather_sources, u_dim = _build_gather_sources(
            diagram, sys_id, output_slices, dependencies=port.dependencies
        )

        # Pre-calculated index in the flat signal buffer where this port writes its result
        out_slice = output_slices[(sys_id, port_id)]

        # Mapping into the global state vector 'x' for this subsystem's local state
        local_x_slice = _state_slice(diagram, sys_id)

        bound = (
            copy.deepcopy(getattr(sys, "params", {})) if bind_params else None
        )

        port_ops.append(
            PortOperation(
                compute_func=port.compute,
                local_x_slice=local_x_slice,
                gather_sources=tuple(gather_sources),
                out_slice=out_slice,
                u_dim=u_dim,
                bound_params=bound,
            )
        )

    # ── 3. Build StateOperation list (subsystems with state) ─────────
    state_ops: list[StateOperation] = []
    for sys_id, sys in diagram.subsystems.items():
        if sys.n > 0:
            gather_sources, u_dim = _build_gather_sources(
                diagram, sys_id, output_slices, dependencies="all"
            )
            local_x_slice = _state_slice(diagram, sys_id)
            bound = (
                copy.deepcopy(getattr(sys, "params", {})) if bind_params else None
            )
            state_ops.append(
                StateOperation(
                    f_func=sys.f,
                    local_x_slice=local_x_slice,
                    gather_sources=tuple(gather_sources),
                    u_dim=u_dim,
                    bound_params=bound,
                )
            )

    return ExecutionPlan(
        state_dim=diagram.n,
        signal_dim=signal_dim,
        port_ops=tuple(port_ops),
        state_ops=tuple(state_ops),
        output_slices=output_slices,
    )


def check_algebraic_loops(
    diagram: DiagramSystem,
) -> list[tuple[str, str]]:
    """Detect algebraic loops and return the topological port execution order.

    Uses depth-first search over output-port dependency edges.  An algebraic
    loop exists when a cycle contains only direct-feedthrough paths (i.e.,
    output ports whose ``dependencies`` include the input ports that close
    the cycle).

    This function is **standalone** and can be called independently of
    :func:`build_execution_plan` — for example, right after wiring a diagram
    to validate its topology before simulation.

    Parameters
    ----------
    diagram : DiagramSystem
        The wired diagram to analyse.

    Returns
    -------
    list[tuple[str, str]]
        Output ports ``(sys_id, port_id)`` in valid topological order
        (dependencies before dependents).

    Raises
    ------
    RuntimeError
        If an algebraic loop is detected, with the full cycle path in the
        error message.
    """
    # Tracks ports that have been FULLY explored (including all dependencies)
    visited: set[tuple[str, str]] = set()
    # Path of the current recursion to provide a trace if a loop is found
    stack: list[tuple[str, str]] = []
    # Set mirror of stack for O(1) cycle detection
    stack_set: set[tuple[str, str]] = set()
    # The resulting topological order
    order: list[tuple[str, str]] = []

    def visit_port(sys_id: str, port_id: str) -> None:
        node = (sys_id, port_id)

        # 1. Algebraic loop detection: if node is already on the recursion stack
        if node in stack_set:
            cycle_start_idx = stack.index(node)
            cycle_path = stack[cycle_start_idx:] + [node]
            cycle_str = " -> ".join(f"{s}:{p}" for s, p in cycle_path)
            raise RuntimeError(f"Algebraic loop detected: {cycle_str}")

        # 2. End-Point A: if we've already fully analyzed this node, stop here (Memoization)
        if node in visited:
            return

        # 3. Mark as currently visiting
        stack.append(node)
        stack_set.add(node)

        # 4. Explore dependencies (upstream crawl)
        # Get the specific output port object we are visiting
        port = diagram.subsystems[sys_id].outputs.get(port_id)
        if port is not None:
            # Get the list of input ports that this output depends on (feedthrough)
            deps = port.dependencies
            sys_inputs = diagram.subsystems[sys_id].inputs
            # If "all", this output depends on every input port of the subsystem
            input_deps = sys_inputs.keys() if deps == "all" else deps

            # Iterate over all input ports that this output depends on
            for in_port_id in input_deps:
                # Find what is connected to this input port (the 'source')
                source = diagram.connections[sys_id].get(in_port_id)
                # End-Point B: if not connected, the chain ends here
                if source is not None:
                    # A source is a tuple: (source_subsystem_id, source_output_port_id)
                    src_sys_id, src_port_id = source
                    # Recursively visit source subsystem if it's not a diagram input
                    # End-Point C: Diagram inputs are terminal nodes for this search
                    if src_sys_id != "input":
                        visit_port(src_sys_id, src_port_id)

        # 5. Finishing node: backtrack and finalize the execution order
        # Remove the current node from the path list; we've finished this branch
        stack.pop()
        # Remove from set; this node is no longer part of the "active" path
        stack_set.remove(node)
        # Mark as visited; ensures we never waste time re-exploring this port
        visited.add(node)
        # Store result; adding it LAST means all its dependencies are already
        # in the 'order' list. This transforms a complex graph into a simple,
        # optimized sequence where every signal is calculated before it's needed.
        order.append(node)

    # --- STARTING POINT: EXPLORE ALL OUTPUT PORTS ---
    # Initiate the recursive depth-first search from every subsystem output port.
    for sys_id, sys in diagram.subsystems.items():
        for port_id in sys.outputs:
            visit_port(sys_id, port_id)

    return order


# ── Private helpers ──────────────────────────────────────────────────


def _build_gather_sources(
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
                    # External diagram input → slice into u
                    source_type = EXTERNAL_INPUT
                    u_idx = 0
                    for input_port_id, input_port in diagram.inputs.items():
                        # We found the matching diagram input port.
                        # Calculate its 'slice' in the flat global input vector 'u'
                        if input_port_id == src_port_id:
                            source_val = slice(u_idx, u_idx + input_port.dim)
                            break
                        u_idx += input_port.dim
                else:
                    # Internal connection → slice into signal buffer
                    source_type = INTERNAL_SIGNAL
                    source_val = output_slices[(src_sys_id, src_port_id)]

        dim = in_port.dim
        gather_sources.append((source_type, source_val, dim))
        u_dim += dim

    return gather_sources, u_dim


def _state_slice(diagram: DiagramSystem, sys_id: str) -> slice:
    """Return the global state-vector slice for a subsystem."""
    sys = diagram.subsystems[sys_id]
    if sys.n > 0:
        start, end = diagram.state_index[sys_id]
        return slice(start, end)
    return slice(0, 0)
