"""Compatibility re-export — prefer ``trajectory_optimization.parametric_program``."""

from minilink.planning.trajectory_optimization.parametric_program import (
    ParametricMathematicalProgram,
)

__all__ = ["ParametricMathematicalProgram"]
