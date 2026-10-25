"""The compiler: a system turned into its evaluator, a diagram through its execution plan."""

from __future__ import annotations

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
    execution_plan_from_order,
    step_execution_plan_from_order,
)
from minilink.core.discrete.step_system import StepSystem
from minilink.core.system import DynamicSystem
from minilink.core.topology import evaluation_order_of
from minilink.core.wiring import iter_subsystems

if TYPE_CHECKING:
    from minilink.core.diagram import DiagramSystem
    from minilink.core.discrete.step_diagram import StepDiagramSystem


###############################################################################
# Public API
###############################################################################


def compile(system, backend=BACKEND_NUMPY, verbose=False, *, flatten=False):
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
        return compile_diagram(
            system, backend=backend, verbose=verbose, flatten=flatten
        )

    if isinstance(system, StepDiagramSystem):
        return compile_step_diagram(
            system, backend=backend, verbose=verbose, flatten=flatten
        )

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
    flatten: bool = False,
    verbose: bool = False,
):
    """Compile a DiagramSystem into a backend evaluator.

    Parameters
    ----------
    diagram : DiagramSystem
        The wired diagram to compile.
    backend : str
        ``'numpy'`` or ``'jax'``.
    flatten : bool
        If ``True``, inline nested diagram blocks into one level (``parent__child``
        ids) before building the execution plan. Default ``False``.
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
    Each plan operation carries a deep copy of its subsystem's ``params``, taken
    here, so ``params=None`` reads the parameters as compiled on both backends; an
    edit after compile needs a recompile or an explicit ``params``. Only the
    ``params`` dicts are copied: it does **not** make user ``f`` / port ``compute``
    implementations pure if they still read or mutate other instance state; see
    :class:`minilink.core.system.System`.
    """
    from minilink.core.assembly import flatten_diagram
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

    if flatten:
        if verbose:
            t0 = time.perf_counter()
            n_before = len(diagram.subsystems)
            print("[compile] Step 0: Flattening nested diagrams...", end="", flush=True)
        diagram, _flatten_report = flatten_diagram(diagram)
        if verbose:
            print(
                f"  ({n_before} → {len(diagram.subsystems)} blocks, "
                f"{time.perf_counter() - t0:.3f}s)"
            )

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

    plan = execution_plan_from_order(diagram, port_execution_order)

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


def compile_step_diagram(
    diagram: StepDiagramSystem,
    backend: str = BACKEND_NUMPY,
    *,
    flatten: bool = False,
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

    if flatten:
        raise NotImplementedError(
            "compile_step_diagram(flatten=True) is not supported yet; "
            "use a flow DiagramSystem or flatten=False."
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

    plan = step_execution_plan_from_order(diagram, port_execution_order)

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
        require_shape(system.f(x0, u0, 0.0), (n,), f"f() of '{name}'", f"for n={n}")
    elif isinstance(system, StepSystem):
        require_shape(system.step(x0, u0, 0), (n,), f"step() of '{name}'", f"for n={n}")

    for port_id, port in system.outputs.items():
        function = port.compute
        if getattr(function, "__self__", None) is not system:
            continue
        hook = getattr(function, "__name__", "")
        if hook not in ("h", "compute_state"):
            continue
        require_shape(
            function(x0, u0, 0.0),
            (port.dim,),
            f"{hook}() of '{name}'",
            f"for output port '{port_id}'",
        )


###############################################################################
# Internal machinery
###############################################################################


def require_shape(value, expected, what, context):
    """Refuse a hook's result whose shape is not the declared one (a bare scalar passes for ``(1,)``)."""
    shape = np.shape(value)
    if shape == expected or (shape == () and expected == (1,)):
        return
    raise ValueError(f"{what} returned shape {shape}; expected {expected} {context}")


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
