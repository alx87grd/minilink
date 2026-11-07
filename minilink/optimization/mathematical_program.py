"""Mathematical programs: the finite-dimensional NLP, and the result an optimizer returns."""

from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Any, TypeAlias

import numpy as np

from minilink.core.backends import BACKEND_NUMPY, array_module
from minilink.core.inspect import body_line, format_inspect, repr_pretty
from minilink.core.reporting import preview_vector

# These stay loose on purpose: the runtime contract is native-backend behavior
# plus shape checks in evaluators, without making JAX a core dependency.
NativeArray: TypeAlias = Any
NativeScalarExpression: TypeAlias = Any
ObjectiveFunction: TypeAlias = Callable[[NativeArray], NativeScalarExpression]
ArrayFunction: TypeAlias = Callable[[NativeArray], NativeArray]


###############################################################################
# Public API
###############################################################################


@dataclass(frozen=True)
class MathematicalProgram:
    """
    Pure finite-dimensional nonlinear program description.

    The generic form is the textbook NLP
    ``minimize J(z)`` subject to ``h(z) = 0``, ``g(z) >= 0``, and optional
    box bounds ``lower <= z <= upper``. The program stores pure callables of
    the decision vector ``z`` and no solver state such as an initial guess;
    backend casting, JAX compilation and SciPy-friendly wrappers live on the
    program evaluators the optimizers own.

    A program may declare parameters: with ``params`` a tree of
    numbers, every callable takes ``(z, params)`` (``J(z, params)``,
    ``h(z, params)``, ``grad_J(z, params)``, …), ``params`` the frozen default
    the values of a solve are merged onto
    (``Optimizer(program).solve(params=...)``), so one compiled program is
    solved for many values without a recompile. Without ``params`` the
    callables take ``z`` alone, as a hand-written program does. The tree's
    structure is fixed; its sizes are structural, never parameters.

    Native function contract
    ------------------------
    The mathematical callables are backend-native functions. With NumPy input,
    they should return NumPy scalar/array expressions; with JAX input, they
    should return JAX scalar/array expressions. Do not cast with ``float(...)``
    or force NumPy arrays inside these functions if JAX tracing may be used.

    Shapes are:

    ``J : (n_z,) -> scalar`` (0-D scalar expression),
    ``h : (n_z,) -> (n_h,)``,
    ``g : (n_z,) -> (n_g,)``.

    Parameters
    ----------
    n_z : int
        Dimension of the decision vector ``z``.
    J : callable
        Objective function ``J(z) -> scalar``. The input and output should stay
        native to the active array backend: with NumPy input, return a NumPy
        scalar expression; with JAX input, return a JAX scalar expression.
        Prefer code like ``return z @ z`` over ``return float(z @ z)``.
    h : callable, optional
        Equality residual ``h(z) -> array`` with shape ``(n_h,)``. The returned
        array should be native to the active backend. Missing means no
        equalities.
    g : callable, optional
        Inequality margin ``g(z) -> array`` with shape ``(n_g,)`` and
        convention ``g(z) >= 0``. The returned array should be native to the
        active backend. Missing means no inequalities.
    lower, upper : array_like, optional
        Lower and upper box bounds on ``z``. ``None`` means unbounded on that
        side.
    grad_J : callable, optional
        Objective gradient ``grad_J(z) -> array`` with shape ``(n_z,)``.
    hess_J : callable, optional
        Objective Hessian ``hess_J(z) -> array`` with shape ``(n_z, n_z)``.
    jac_h, jac_g : callable, optional
        Constraint Jacobians ``dh/dz`` and ``dg/dz`` with shapes
        ``(n_h, n_z)`` and ``(n_g, n_z)``.
    backend : str
        Native array backend of the callables (``"numpy"`` or ``"jax"``) —
        the program-evaluator backend they should be compiled with.
    metadata : dict
        Optional transcription or diagnostic metadata.
    params : dict, optional
        The default values of the parameters the callables read, a nested
        dict of numbers; ``None`` (the default) for callables of ``z`` alone.
    """

    n_z: int
    J: ObjectiveFunction
    h: ArrayFunction | None = None
    g: ArrayFunction | None = None
    lower: np.ndarray | None = None
    upper: np.ndarray | None = None
    grad_J: ArrayFunction | None = None
    hess_J: ArrayFunction | None = None
    jac_h: ArrayFunction | None = None
    jac_g: ArrayFunction | None = None
    backend: str = BACKEND_NUMPY
    metadata: dict[str, object] = field(default_factory=dict)
    params: dict | None = None

    def __post_init__(self) -> None:
        n_z = int(self.n_z)
        require_positive_size(n_z)

        # The box bounds as flat float copies of shape (n_z,), with lower ≤ upper
        lower = as_vector_or_none("lower", self.lower, n_z)
        upper = as_vector_or_none("upper", self.upper, n_z)
        require_ordered_bounds(lower, upper)

        # The frozen fields take their checked values, the metadata its own copy
        object.__setattr__(self, "n_z", n_z)
        object.__setattr__(self, "lower", lower)
        object.__setattr__(self, "upper", upper)
        object.__setattr__(self, "metadata", dict(self.metadata))
        require_params_tree(self.params)

    # Display

    def __str__(self):
        return program_text(self)

    def _repr_pretty_(self, p, cycle):
        repr_pretty(self, p, cycle)

    def _repr_latex_(self):
        return program_latex(self)


@dataclass(frozen=True)
class OptimizationResult:
    """
    Result returned by an :class:`~minilink.optimization.optimizer.Optimizer`.

    Parameters
    ----------
    solve_time_s : float, optional
        Wall-clock time in seconds of the backend solve alone, measured by
        :meth:`~minilink.optimization.optimizer.Optimizer.solve`; ``None`` on a
        traced solve (a compiled face, a fixed count of native steps), which
        has no wall clock.
    """

    z: np.ndarray
    success: bool
    cost: float | None = None
    message: str = ""
    stats: dict[str, object] = field(default_factory=dict)
    raw_result: object | None = None
    solve_time_s: float | None = None

    def __post_init__(self) -> None:
        # The solution as a flat float copy (a JAX array kept as it is, so a solve
        # traces), the statistics as their own dict
        if array_module(self.z) is not np:
            z = self.z.reshape(-1)
        else:
            z = np.asarray(self.z, dtype=float).reshape(-1).copy()
        object.__setattr__(self, "z", z)
        object.__setattr__(self, "stats", dict(self.stats))

        # The measured time as a float, when one was measured
        if self.solve_time_s is not None:
            object.__setattr__(self, "solve_time_s", float(self.solve_time_s))

    def __str__(self):
        return optimization_result_text(self)

    def _repr_pretty_(self, p, cycle):
        repr_pretty(self, p, cycle)


###############################################################################
# Internal machinery
###############################################################################


def require_params_tree(params):
    """Refuse declared parameters that are not a tree (a dict)."""
    if params is not None and not isinstance(params, dict):
        raise TypeError(
            f"params must be a dict of the program's parameters, got {type(params).__name__}"
        )


def require_positive_size(n_z):
    """Refuse a program without a decision variable."""
    if n_z <= 0:
        raise ValueError("n_z must be a positive integer")


def as_vector_or_none(name: str, value, n_z: int) -> np.ndarray | None:
    """A bound as a flat float copy of shape ``(n_z,)``, or ``None`` when absent."""
    if value is None:
        return None
    arr = np.asarray(value, dtype=float).reshape(-1).copy()
    if arr.shape != (n_z,):
        raise ValueError(f"{name} bounds must have shape ({n_z},)")
    return arr


def require_ordered_bounds(lower, upper):
    """Refuse a lower bound above its upper bound."""
    if lower is not None and upper is not None and np.any(lower > upper):
        raise ValueError("lower bounds must be less than or equal to upper bounds")


def program_text(program):
    """The print text of a program: its class, constraint rows, backend and bounds."""
    # A real cycle: optimizer imports this module
    from minilink.optimization.optimizer import program_class

    kind = program_class(program)
    has_h = program.h is not None
    has_g = program.g is not None
    headline = f"MathematicalProgram, n_z={program.n_z}"
    body = [
        body_line("class", kind),
        body_line("h", f"{'yes' if has_h else 'no'} (n={constraint_rows(program.h)})"),
        body_line("g", f"{'yes' if has_g else 'no'} (n={constraint_rows(program.g)})"),
        body_line("backend", program.backend),
    ]

    sides = []
    if program.lower is not None:
        sides.append("lower")
    if program.upper is not None:
        sides.append("upper")
    if sides:
        body.append(body_line("bounds", ", ".join(sides)))

    return format_inspect(headline, body)


def constraint_rows(fn):
    """The number of rows of a constraint function (``?`` when it does not say)."""
    if fn is None:
        return 0

    n_out = getattr(fn, "n_out", None)
    if n_out is not None:
        return int(n_out)

    return "?"


def optimization_result_text(result):
    """The print text of a result: success, ``J*``, ``z*``, the message and the time."""
    success = "yes" if result.success else "no"
    headline = f"OptimizationResult, success={success}"
    body = []

    if result.cost is not None:
        body.append(body_line("J*", f"{float(result.cost):.6g}"))
    body.append(body_line("z*", preview_vector(result.z)))
    if result.message:
        message = result.message.strip()
        if len(message) > 72:
            message = message[:69] + "..."
        body.append(body_line("message", message))
    if result.solve_time_s is not None:
        body.append(body_line("solve_time_s", f"{result.solve_time_s:g}"))

    return format_inspect(headline, body)


def program_latex(program):
    """The program's LaTeX summary: ``min J(z)`` and the kinds of constraint it holds."""
    parts = [r"\min_{z} J(z)"]
    if program.h is not None:
        parts.append(r"\text{subject to } h(z) = 0")
    if program.g is not None:
        parts.append(r"g(z) \geq 0")
    if program.lower is not None or program.upper is not None:
        parts.append(r"\text{box bounds on } z")

    body = r" \\ ".join(parts)

    return rf"\begin{{align*}}{body}\end{{align*}}"
