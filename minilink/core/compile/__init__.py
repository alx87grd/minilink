"""
Compilation framework: a system compiled into an evaluator, one class per system kind.

Import from defining modules, for example::

    from minilink.core.compile.compiler import compile, compile_diagram, build_execution_plan
    from minilink.core.compile.evaluators.dynamic import DynamicEvaluator
    from minilink.core.compile.evaluators.diagram import DiagramEvaluator
    from minilink.core.compile.evaluators.static import StaticEvaluator
    from minilink.core.compile.evaluators.discrete import StepEvaluator, StepDiagramEvaluator
    from minilink.core.compile.execution_plan import ExecutionPlan, PortOperation, StateOperation

Every evaluator runs on NumPy or JAX (``compile(system, backend=...)``); JAX is
imported only by a JAX evaluator.
"""
