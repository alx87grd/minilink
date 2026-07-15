"""Compatibility re-export — prefer ``trajectory_optimization.parametric_evaluator``."""

from minilink.planning.trajectory_optimization.parametric_evaluator import (
    JaxParametricProgramEvaluator,
)

__all__ = ["JaxParametricProgramEvaluator"]
