"""
Compilation framework.

Import from defining modules, for example::

    from minilink.core.compile.compiler import compile, compile_diagram, check_algebraic_loops, build_execution_plan
    from minilink.core.compile.evaluators.evaluator import DynamicsEvaluator
    from minilink.core.compile.execution_plan import ExecutionPlan, PortOperation, StateOperation
    from minilink.core.compile.evaluators.numpy_evaluator import NumpyDiagramEvaluator, NumpyLeafEvaluator

Optional JAX evaluators (requires ``pip install minilink[jax]``)::

    from minilink.core.compile.evaluators.jax_evaluator import JaxDiagramEvaluator, JaxLeafEvaluator
"""
