"""
Compilation framework.

Import from defining modules, for example::

    from minilink.compile.compiler import compile, compile_diagram, check_algebraic_loops, build_execution_plan
    from minilink.compile.evaluator import DynamicsEvaluator
    from minilink.compile.execution_plan import ExecutionPlan, PortOperation, StateOperation
    from minilink.compile.numpy_evaluator import NumpyDiagramEvaluator, NumpyLeafEvaluator

Optional JAX evaluators (requires ``pip install minilink[jax]``)::

    from minilink.compile.jax_evaluator import JaxDiagramEvaluator, JaxLeafEvaluator
"""
