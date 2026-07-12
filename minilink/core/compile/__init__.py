"""
Compilation framework.

Import from defining modules, for example::

    from minilink.core.compile.compiler import compile, compile_diagram, check_algebraic_loops, build_execution_plan
    from minilink.core.compile.evaluators.evaluators import DynamicsEvaluator
    from minilink.core.compile.evaluators.step_rollout import StepRolloutMixin, gather_u
    from minilink.core.compile.evaluators.numpy_evaluators import (
        NumpyDiagramEvaluator,
        NumpyDynamicEvaluator,
        NumpyStaticEvaluator,
    )
    from minilink.core.compile.execution_plan import ExecutionPlan, PortOperation, StateOperation

Optional JAX evaluators (requires ``pip install minilink[jax]``)::

    from minilink.core.compile.evaluators.jax_evaluators import (
        JaxDiagramEvaluator,
        JaxDynamicEvaluator,
        JaxStaticEvaluator,
    )
"""
