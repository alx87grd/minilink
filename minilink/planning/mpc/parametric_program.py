"""Parametric finite-dimensional programs for compile-once MPC."""

from collections.abc import Callable, Mapping
from dataclasses import dataclass, field
from typing import Any, TypeAlias

from minilink.core.backends import BACKEND_JAX

NativeArray: TypeAlias = Any
NativeScalarExpression: TypeAlias = Any
ObjectiveFunction: TypeAlias = Callable[[NativeArray], NativeScalarExpression]
ParametricEqualityFunction: TypeAlias = Callable[
    [NativeArray, NativeArray], NativeArray
]
InequalityFunction: TypeAlias = Callable[[NativeArray], NativeArray]


@dataclass(frozen=True)
class ParametricMathematicalProgram:
    """
    Pure NLP with a runtime initial-state parameter ``x0``.

    The generic form is ``minimize J(z)`` subject to ``h(z, x0) = 0``,
    ``g(z) >= 0``, and optional box bounds on ``z``. Only ``x0`` may vary
    between solves; all other problem data is frozen at compile time.
    """

    n_z: int
    n_x0: int
    J: ObjectiveFunction
    h: ParametricEqualityFunction
    g: InequalityFunction | None = None
    lower: Any | None = None
    upper: Any | None = None
    backend: str = BACKEND_JAX
    metadata: Mapping[str, object] = field(default_factory=dict)
