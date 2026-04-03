"""
Diagram compilation framework.

Provides tools for analysing diagram topology (algebraic loop detection)
and compiling diagrams into fast, stateless evaluators.

Quick start::

    from minilink.compile import compile_diagram

    evaluator = compile_diagram(diagram, backend="numpy")
    dx = evaluator.compute_dx(x, u, t)
    y  = evaluator.compute_outputs(x, u, t, ports=[("plant", "y")])

For topology analysis only::

    from minilink.compile import check_algebraic_loops

    port_order = check_algebraic_loops(diagram)  # raises on loops
"""

from minilink.compile.compiler import (
    build_execution_plan,
    check_algebraic_loops,
    compile_diagram,
)
from minilink.compile.execution_plan import (
    ExecutionPlan,
    PortOperation,
    StateOperation,
)
from minilink.compile.numpy_backend import NumpyEvaluator

# JaxEvaluator is exported lazily — JAX is optional.
# Import it here so `from minilink.compile import JaxEvaluator` works when JAX is installed.
try:
    from minilink.compile.jax_backend import JaxEvaluator
except ImportError:
    JaxEvaluator = None  # type: ignore[assignment,misc]

__all__ = [
    # Entry points
    "compile_diagram",
    "build_execution_plan",
    "check_algebraic_loops",
    # Data structures
    "ExecutionPlan",
    "PortOperation",
    "StateOperation",
    # Backends
    "NumpyEvaluator",
    "JaxEvaluator",  # None when JAX is not installed
]

