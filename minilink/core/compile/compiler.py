"""
The compiler: a system turned into its evaluator, a diagram through its execution plan.

:func:`compile` picks the evaluator by system kind; a diagram goes through
the pipeline:

1. :func:`~minilink.core.wiring.evaluation_order_of` — Depth-first search over
   the port dependencies (implemented in ``wiring.py``): the topological
   execution order of the output ports, an algebraic loop refused. The
   standalone check after manual wiring is
   :func:`~minilink.core.wiring.check_algebraic_loops`.

2. :func:`build_execution_plan` (:func:`build_step_execution_plan` for a step
   diagram) — Constructs an immutable
   :class:`~minilink.core.compile.plan.ExecutionPlan`
   (:class:`~minilink.core.compile.plan.StepExecutionPlan`) from a diagram.
   Calls :func:`~minilink.core.wiring.evaluation_order_of` internally.

3. :func:`compile_diagram` (:func:`compile_step_diagram`) — Top-level entry
   point that builds the plan and wraps it in the evaluator.
"""

from __future__ import annotations

import copy
import time
import warnings
from typing import TYPE_CHECKING

import numpy as np

from minilink.core.backends import (
    BACKEND_NUMPY,
    normalize_backend,
    require_jax_numpy,
)
from minilink.core.compile.plan import (
    EXTERNAL_INPUT,
    INTERNAL_SIGNAL,
    NOMINAL,
    ExecutionPlan,
    PortOperation,
    StateOperation,
    StepExecutionPlan,
    StepOperation,
)
from minilink.core.discrete.step_system import StepSystem
from minilink.core.system import DynamicSystem
from minilink.core.wiring import evaluation_order_of, iter_subsystems

if TYPE_CHECKING:
    from minilink.core.diagram import DiagramSystem
    from minilink.core.discrete.step_diagram import StepDiagramSystem


# Public API
def validate_equation_shapes(system, label=None):
    """Probe the textbook hooks once and check their declared shapes.

    ``f`` (continuous leaves), ``step`` (discrete leaves), and the standard
    output ports created by the constructor (``y`` -> ``h``, ``x`` -> state)
    are evaluated at ``(x0, u_nominal, t=0)``; a wrong-length result raises
    ``ValueError`` naming the block, the hook, and both shapes. Custom port
    computes are not probed (they may be expensive or stateful).
    """
    from minilink.core.diagram import DiagramSystem
    from minilink.core.discrete.step_diagram import StepDiagramSystem
    from minilink.core.discrete.step_system import StepSystem
    from minilink.core.system import DynamicSystem

    if isinstance(system, (DiagramSystem, StepDiagramSystem)):
        return
    name = label or getattr(system, "name", type(system).__name__)
    n = int(system.n)
    x0 = np.asarray(system.x0, dtype=float).reshape(-1)
    if x0.shape != (n,):
        raise ValueError(
            f"x0 of '{name}' has shape {x0.shape}; expected ({n},) for n={n}"
        )
    u0 = np.asarray(system.get_u_from_input_ports(), dtype=float).reshape(-1)

    if isinstance(system, DynamicSystem) and n > 0:
        _check_shape(system.f(x0, u0, 0.0), (n,), f"f() of '{name}'", f"for n={n}")
    elif isinstance(system, StepSystem):
        _check_shape(system.step(x0, u0, 0), (n,), f"step() of '{name}'", f"for n={n}")

    for port_id, port in system.outputs.items():
        function = port.compute
        if getattr(function, "__self__", None) is not system:
            continue
        hook = getattr(function, "__name__", "")
        if hook not in ("h", "compute_state"):
            continue
        _check_shape(
            function(x0, u0, 0.0),
            (port.dim,),
            f"{hook}() of '{name}'",
            f"for output port '{port_id}'",
        )


def _check_shape(value, expected, what, context):
    shape = np.shape(value)
    if shape == expected or (shape == () and expected == (1,)):
        return  # a bare scalar is fine for a one-dimensional equation
    raise ValueError(f"{what} returned shape {shape}; expected {expected} {context}")


def compile(system, backend=BACKEND_NUMPY, verbose=False):
    """Compile a system into a typed evaluator.

    Returns a :class:`DynamicEvaluator` for a :class:`DynamicSystem`, a
    :class:`DiagramEvaluator` for a diagram, a :class:`StepEvaluator` for a
    :class:`StepSystem`, a :class:`StepDiagramEvaluator` for a step diagram and a
    :class:`StaticEvaluator` for a static ``System`` (``n=0``).

    Parameters
    ----------
    system : System or DiagramSystem
        The system to compile.
    backend : str
        ``'numpy'`` or ``'jax'``.
    verbose : bool
        If ``True``, print timed compilation steps.
    """
    from minilink.core.diagram import DiagramSystem
    from minilink.core.discrete.step_diagram import StepDiagramSystem
    from minilink.core.discrete.step_system import StepSystem
    from minilink.core.system import DynamicSystem

    if isinstance(system, DiagramSystem):
        return compile_diagram(system, backend=backend, verbose=verbose)

    if isinstance(system, StepDiagramSystem):
        return compile_step_diagram(system, backend=backend, verbose=verbose)

    key = normalize_backend(backend)
    validate_equation_shapes(system)
    if key != BACKEND_NUMPY:
        require_jax_numpy()

    # One evaluator class per system kind, on either backend
    if isinstance(system, StepSystem):
        from minilink.core.compile.evaluators.discrete import StepEvaluator as kind
    elif isinstance(system, DynamicSystem):
        from minilink.core.compile.evaluators.dynamic import DynamicEvaluator as kind
    elif system.n > 0:
        raise TypeError(
            f"Cannot compile {type(system).__name__} with n={system.n}; "
            "subclass DynamicSystem and implement f(), or StepSystem for discrete evolution."
        )
    else:
        from minilink.core.compile.evaluators.static import StaticEvaluator as kind

    t_total = time.perf_counter()
    evaluator = kind(system, backend=key, verbose=verbose)
    if verbose:
        print(f"[compile] Done.  ({time.perf_counter() - t_total:.3f}s total)")

    return evaluator


def compile_auto(system, verbose=False):
    """Compile with JAX when it is installed and ``system`` traces, else with NumPy.

    Returns ``(backend, evaluator)``. Only two verdicts fall back to NumPy:
    JAX cannot be imported, or the system is "not JAX-traceable"; any other
    error from the JAX compile surfaces.
    """
    from minilink.core.backends import BACKEND_JAX, jax_installed

    if jax_installed():
        try:
            return BACKEND_JAX, compile(system, backend=BACKEND_JAX, verbose=verbose)
        except ImportError:
            pass  # present on disk but not importable (e.g. blocked, broken install)
        except RuntimeError as exc:
            if "JAX-traceable" not in str(exc):
                raise
    return BACKEND_NUMPY, compile(system, backend=BACKEND_NUMPY, verbose=verbose)


def compile_diagram(
    diagram: DiagramSystem,
    backend: str = BACKEND_NUMPY,
    *,
    bind_params: bool = False,
    verbose: bool = False,
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
    verbose : bool
        If ``True``, print timed compilation steps.

    Returns
    -------
    DiagramEvaluator
        A stateless evaluator that can compute state derivatives and outputs.

    Examples
    --------
    >>> from minilink.core.compile.compiler import compile_diagram
    >>> evaluator = compile_diagram(diagram)
    >>> dx = evaluator.f(x, u, t)

    Notes
    -----
    ``bind_params=True`` snapshots only each subsystem's ``params`` dict into the plan.
    It does **not** make user ``f`` / port ``compute`` implementations pure if they still
    read or mutate other instance state; see :class:`minilink.core.system.System`.
    """
    from minilink.core.discrete.step_diagram import StepDiagramSystem
    from minilink.core.discrete.step_system import StepSystem

    if isinstance(diagram, StepDiagramSystem):
        raise TypeError(
            "StepDiagramSystem must be compiled via compile_step_diagram; "
            "use StepDiagramSystem.compile() or compile(step_diagram)."
        )

    for sys_id, subsystem in diagram.subsystems.items():
        if isinstance(subsystem, StepSystem):
            raise TypeError(
                f"StepSystem leaf '{sys_id}' cannot be compiled inside a flow "
                "DiagramSystem; use StepDiagramSystem."
            )

    t_total = time.perf_counter() if verbose else None

    # Step 1: Algebraic loop detection
    if verbose:
        t0 = time.perf_counter()
        print("[compile] Step 1: Checking for algebraic loops...", end="", flush=True)

    for sys_id, subsystem in diagram.subsystems.items():
        validate_equation_shapes(subsystem, label=f"{subsystem.name} ({sys_id})")
    warn_shared_seeds(diagram)
    port_execution_order = evaluation_order_of(diagram)

    if verbose:
        print(f"  ({time.perf_counter() - t0:.3f}s)")

    # Step 2: Build execution plan
    if verbose:
        t0 = time.perf_counter()
        n_ports = len(port_execution_order)
        n_states = sum(
            1 for s in diagram.subsystems.values() if isinstance(s, DynamicSystem)
        )
        print(
            f"[compile] Step 2: Building execution plan "
            f"({n_ports} ports, {n_states} states)...",
            end="",
            flush=True,
        )

    plan = _build_execution_plan_from_order(
        diagram, port_execution_order, bind_params=bind_params
    )

    if verbose:
        print(f"  ({time.perf_counter() - t0:.3f}s)")

    # Create evaluator (steps 0, 3, 4 handled inside for JAX)
    key = normalize_backend(backend)
    if key != BACKEND_NUMPY:
        require_jax_numpy()
    from minilink.core.compile.evaluators.diagram import DiagramEvaluator

    evaluator = DiagramEvaluator(plan, diagram, backend=key, verbose=verbose)

    if verbose:
        print(f"[compile] Done.  ({time.perf_counter() - t_total:.3f}s total)")

    return evaluator


def build_execution_plan(
    diagram: DiagramSystem, *, bind_params: bool = False
) -> ExecutionPlan:
    """Build an immutable ExecutionPlan from a DiagramSystem.

    This is the core compilation step.  It:

    1. Orders the output ports, refusing an algebraic loop, via
       :func:`~minilink.core.wiring.evaluation_order_of`.
    2. Assigns each output port a slice in the flat signal buffer.
    3. Builds the gather-source recipes for each port and state operation.
    4. Returns a frozen :class:`ExecutionPlan`.

    Parameters
    ----------
    diagram : DiagramSystem
        Must have subsystems added and connections wired.
    bind_params : bool, optional
        When ``True``, set :attr:`~minilink.core.compile.plan.PortOperation.bound_params`
        / :attr:`~minilink.core.compile.plan.StateOperation.bound_params` on each row
        (deep copy of ``subsystem.params`` only; see :func:`compile_diagram` Notes).

    Returns
    -------
    ExecutionPlan
    """
    port_execution_order = evaluation_order_of(diagram)
    return _build_execution_plan_from_order(
        diagram, port_execution_order, bind_params=bind_params
    )


def compile_step_diagram(
    diagram: StepDiagramSystem,
    backend: str = BACKEND_NUMPY,
    *,
    bind_params: bool = False,
    verbose: bool = False,
):
    """Compile a :class:`~minilink.core.discrete.step_diagram.StepDiagramSystem` into an evaluator.

    The step twin of :func:`compile_diagram`: same arguments, a
    :class:`~minilink.core.compile.evaluators.discrete.StepDiagramEvaluator` back.
    """
    from minilink.core.discrete.step_diagram import (
        StepDiagramSystem as StepDiagramSystemType,
    )

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
        validate_equation_shapes(subsystem, label=f"{subsystem.name} ({sys_id})")

    t_total = time.perf_counter() if verbose else None

    if verbose:
        t0 = time.perf_counter()
        print("[compile] Step 1: Checking for algebraic loops...", end="", flush=True)

    port_execution_order = evaluation_order_of(diagram)

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
    if key != BACKEND_NUMPY:
        require_jax_numpy()
    from minilink.core.compile.evaluators.discrete import StepDiagramEvaluator

    evaluator = StepDiagramEvaluator(plan, diagram, backend=key, verbose=verbose)

    if verbose:
        print(f"[compile] Done.  ({time.perf_counter() - t_total:.3f}s total)")

    return evaluator


def build_step_execution_plan(
    diagram: StepDiagramSystem, *, bind_params: bool = False
) -> StepExecutionPlan:
    """Build an immutable :class:`StepExecutionPlan` from a step diagram."""
    port_execution_order = evaluation_order_of(diagram)
    return _build_step_execution_plan_from_order(
        diagram, port_execution_order, bind_params=bind_params
    )


# Private helpers
def warn_shared_seeds(diagram):
    """Warn when two random blocks hold one seed: they would draw the same signal."""
    blocks_by_seed = {}
    for path, subsystem in iter_subsystems(diagram):
        seed = subsystem.params.get("seed") if subsystem.is_random else None
        if seed is not None:
            blocks_by_seed.setdefault(int(seed), []).append(".".join(path))
    for seed, ids in blocks_by_seed.items():
        if len(ids) > 1:
            warnings.warn(
                f"random blocks {', '.join(ids)} share seed {seed} and draw the same "
                "signal; give each its own seed, or realize the diagram under one key",
                stacklevel=2,
            )


def _build_execution_plan_from_order(
    diagram: DiagramSystem,
    port_execution_order: list[tuple[str, str]],
    *,
    bind_params: bool = False,
) -> ExecutionPlan:
    """Build the execution plan given a pre-computed topological order.

    This is the inner implementation of :func:`build_execution_plan`,
    split out so that :func:`compile_diagram` can time each step
    individually.
    """
    output_slices, signal_dim = signal_layout(diagram)
    port_ops = port_operations(
        diagram, port_execution_order, output_slices, DynamicSystem, bind_params
    )

    # One dx per continuous block
    state_ops = tuple(
        StateOperation(f_func=sys.f, **fields)
        for sys, fields in state_fields(
            diagram, output_slices, DynamicSystem, bind_params
        )
    )

    return ExecutionPlan(
        state_dim=diagram.n,
        signal_dim=signal_dim,
        port_ops=port_ops,
        state_ops=state_ops,
        output_slices=output_slices,
        external_output_slices=boundary_output_slices(diagram, output_slices),
    )


def _build_step_execution_plan_from_order(
    diagram: StepDiagramSystem,
    port_execution_order: list[tuple[str, str]],
    *,
    bind_params: bool = False,
) -> StepExecutionPlan:
    """The step twin of :func:`_build_execution_plan_from_order`: ``step_ops`` for ``state_ops``."""
    output_slices, signal_dim = signal_layout(diagram)
    port_ops = port_operations(
        diagram, port_execution_order, output_slices, StepSystem, bind_params
    )

    # One x_{k+1} per discrete block
    step_ops = tuple(
        StepOperation(step_func=sys.step, **fields)
        for sys, fields in state_fields(diagram, output_slices, StepSystem, bind_params)
    )

    return StepExecutionPlan(
        state_dim=diagram.n,
        signal_dim=signal_dim,
        port_ops=port_ops,
        step_ops=step_ops,
        output_slices=output_slices,
        external_output_slices=boundary_output_slices(diagram, output_slices),
    )


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


def port_operations(
    diagram, port_execution_order, output_slices, stateful, bind_params
):
    """One :class:`PortOperation` per output port, in topological order."""
    port_ops: list[PortOperation] = []
    for sys_id, port_id in port_execution_order:
        sys = diagram.subsystems[sys_id]
        port = sys.outputs[port_id]

        gather_sources, u_dim = _build_gather_sources(
            diagram, sys_id, output_slices, dependencies=port.dependencies
        )

        out_slice = output_slices[(sys_id, port_id)]
        local_x_slice = _state_slice(diagram, sys_id, stateful)

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

    return tuple(port_ops)


def state_fields(diagram, output_slices, stateful, bind_params):
    """``(block, fields)`` for every ``stateful`` block: the fields its state operation shares."""
    for sys_id, sys in diagram.subsystems.items():
        if not isinstance(sys, stateful):
            continue
        gather_sources, u_dim = _build_gather_sources(
            diagram, sys_id, output_slices, dependencies="all"
        )
        bound = copy.deepcopy(getattr(sys, "params", {})) if bind_params else None
        fields = {
            "local_x_slice": _state_slice(diagram, sys_id, stateful),
            "gather_sources": tuple(gather_sources),
            "u_dim": u_dim,
            "bound_params": bound,
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


def _state_slice(diagram, sys_id: str, stateful=DynamicSystem) -> slice:
    """Return the global state-vector slice for a subsystem (empty unless ``stateful``)."""
    sys = diagram.subsystems[sys_id]
    if isinstance(sys, stateful):
        start, end = diagram.state_index[sys_id]
        return slice(start, end)
    return slice(0, 0)
