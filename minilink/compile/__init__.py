"""
Compilation framework.

Provides tools for compiling systems and diagrams into fast evaluators,
and analysing diagram topology (algebraic loop detection).

Quick start (leaf system)::

    from minilink.compile import compile

    eval = compile(pendulum, backend="numpy")
    dx = eval.f(x, u, t)
    y  = eval.h(x, u, t)

Quick start (diagram)::

    from minilink.compile import compile_diagram

    evaluator = compile_diagram(diagram, backend="numpy")
    dx = evaluator.f(x, u, t)

For topology analysis only::

    from minilink.compile import check_algebraic_loops

    port_order = check_algebraic_loops(diagram)  # raises on loops
"""

from minilink.compile.compiler import (
    build_execution_plan,
    check_algebraic_loops,
    compile,
    compile_diagram,
)
from minilink.compile.evaluator import DynamicsEvaluator
from minilink.compile.execution_plan import (
    ExecutionPlan,
    PortOperation,
    StateOperation,
)
from minilink.compile.numpy_evaluator import NumpyDiagramEvaluator, NumpyLeafEvaluator

# JAX backends are exported lazily — JAX is optional.
try:
    from minilink.compile.jax_evaluator import JaxDiagramEvaluator, JaxLeafEvaluator
except ImportError:
    JaxDiagramEvaluator = None  # type: ignore[assignment,misc]
    JaxLeafEvaluator = None  # type: ignore[assignment,misc]

__all__ = [
    # Entry points
    "compile",
    "compile_diagram",
    "build_execution_plan",
    "check_algebraic_loops",
    # Public interface
    "DynamicsEvaluator",
    # Data structures
    "ExecutionPlan",
    "PortOperation",
    "StateOperation",
    # Leaf evaluators
    "NumpyLeafEvaluator",
    "JaxLeafEvaluator",  # None when JAX is not installed
    # Diagram evaluators
    "NumpyDiagramEvaluator",
    "JaxDiagramEvaluator",  # None when JAX is not installed
]
