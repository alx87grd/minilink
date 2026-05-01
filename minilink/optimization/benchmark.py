"""Benchmarks for finite-dimensional :class:`MathematicalProgram` solvers.

Sweep a list of optimizer backends (currently SciPy ``minimize`` and Ipopt via
``cyipopt``) over a set of textbook nonlinear-program test cases and compare
solve time, iteration counts, final cost, and constraint feasibility. Mirrors
the layout of :mod:`minilink.simulation.benchmark` and
:mod:`minilink.compile.benchmark`.

Usage::

    from minilink.optimization.benchmark import (
        benchmark_optimizer_backends,
        default_optimizer_variants,
        print_optimizer_benchmark,
        STANDARD_OPTIMIZATION_CASES,
    )

    variants = default_optimizer_variants()
    result = benchmark_optimizer_backends(STANDARD_OPTIMIZATION_CASES, variants)
    print_optimizer_benchmark(result)
"""

from __future__ import annotations

import time
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass, field

import numpy as np

from minilink.optimization.mathematical_program import (
    EqualityConstraint,
    InequalityConstraint,
    MathematicalProgram,
    OptimizationResult,
    VariableBounds,
)
from minilink.optimization.optimizers.optimizer import Optimizer
from minilink.optimization.optimizers.scipy_minimize import ScipyMinimizeOptimizer


# --- Variant / Case / Result records ---------------------------------------


@dataclass(frozen=True)
class OptimizerBenchmarkVariant:
    """One optimizer backend configuration to benchmark.

    Parameters
    ----------
    name : str
        Short label used in the benchmark table.
    backend : str
        Backend identifier, ``"scipy"`` or ``"ipopt"``.
    method : str
        Backend-specific method (e.g. SciPy ``"SLSQP"``, ``"trust-constr"``).
        Ignored by Ipopt (which uses interior-point).
    options : Mapping
        Backend-specific options dictionary.
    """

    name: str
    backend: str
    method: str = ""
    options: Mapping[str, object] = field(default_factory=dict)


@dataclass(frozen=True)
class OptimizerBenchmarkCase:
    """One named mathematical-program test problem.

    Parameters
    ----------
    id : str
        Short machine-readable id.
    label : str
        Human-readable label for tables.
    build : callable
        Returns a fresh :class:`MathematicalProgram` (built per run so internal
        state is never shared between solvers).
    z_star : np.ndarray, optional
        Known optimum used to report a relative ``z`` error. ``None`` skips it.
    cost_star : float, optional
        Known optimal cost. ``None`` skips the cost-error column.
    """

    id: str
    label: str
    build: Callable[[], MathematicalProgram]
    z_star: np.ndarray | None = None
    cost_star: float | None = None


@dataclass(frozen=True)
class OptimizerBenchmarkRow:
    """One measured (case, variant) combination."""

    case_id: str
    case_label: str
    variant: OptimizerBenchmarkVariant
    success: bool
    solve_s: float
    cost: float | None
    cost_err: float | None
    z_err: float | None
    eq_inf: float
    min_ineq: float | None
    bound_inf: float
    nit: int | None
    nfev: int | None
    njev: int | None
    message: str


@dataclass(frozen=True)
class OptimizerBenchmarkResult:
    """All rows for a sweep of cases and optimizer variants."""

    cases: tuple[OptimizerBenchmarkCase, ...]
    variants: tuple[OptimizerBenchmarkVariant, ...]
    n_runs: int
    rows: tuple[OptimizerBenchmarkRow, ...]


# --- Standard variants and cases -------------------------------------------


def default_optimizer_variants(
    *,
    maxiter: int = 200,
    ftol: float = 1e-8,
    tol: float = 1e-8,
) -> tuple[OptimizerBenchmarkVariant, ...]:
    """Default SciPy and (when available) Ipopt benchmark sweep."""
    variants: list[OptimizerBenchmarkVariant] = [
        OptimizerBenchmarkVariant(
            name="scipy-SLSQP",
            backend="scipy",
            method="SLSQP",
            options={"maxiter": int(maxiter), "ftol": float(ftol), "disp": False},
        ),
        OptimizerBenchmarkVariant(
            name="scipy-trust-constr",
            backend="scipy",
            method="trust-constr",
            options={"maxiter": int(maxiter), "xtol": float(ftol), "disp": False},
        ),
    ]
    if ipopt_optimizer_available():
        variants.append(
            OptimizerBenchmarkVariant(
                name="ipopt",
                backend="ipopt",
                method="ipopt",
                options={"max_iter": int(maxiter), "tol": float(tol), "print_level": 0},
            )
        )
    return tuple(variants)


def ipopt_optimizer_available() -> bool:
    """Return ``True`` when ``cyipopt`` can be imported."""
    try:
        import cyipopt  # noqa: F401

        return True
    except ImportError:
        return False


# --- Standard test problems (textbook NLPs) --------------------------------
#
# Small, well-known nonlinear programs for solver comparison. The problems are
# defined explicitly with gradients so SciPy and Ipopt receive identical
# information.


def _quadratic_problem() -> MathematicalProgram:
    # min 0.5 (z - c)^T (z - c)  with c = [1, 2, 3]; z* = c, cost* = 0
    c = np.array([1.0, 2.0, 3.0])

    def J(z: np.ndarray) -> float:
        return 0.5 * float(np.sum((z - c) ** 2))

    def grad(z: np.ndarray) -> np.ndarray:
        return z - c

    return MathematicalProgram(J=J, grad=grad, z0=np.zeros(3))


def _rosenbrock_problem(n: int = 5) -> MathematicalProgram:
    # Classic Rosenbrock; z* = ones(n), cost* = 0
    def J(z: np.ndarray) -> float:
        return float(np.sum(100.0 * (z[1:] - z[:-1] ** 2) ** 2 + (1.0 - z[:-1]) ** 2))

    def grad(z: np.ndarray) -> np.ndarray:
        g = np.zeros_like(z)
        g[:-1] = -400.0 * z[:-1] * (z[1:] - z[:-1] ** 2) - 2.0 * (1.0 - z[:-1])
        g[1:] += 200.0 * (z[1:] - z[:-1] ** 2)
        return g

    z0 = -1.2 * np.ones(n)
    z0[1::2] = 1.0
    return MathematicalProgram(J=J, grad=grad, z0=z0)


def _box_constrained_quadratic_problem() -> MathematicalProgram:
    # min 0.5 z^T z subject to z >= 1; z* = ones, cost* = 0.5 * n
    n = 4

    def J(z: np.ndarray) -> float:
        return 0.5 * float(np.sum(z**2))

    def grad(z: np.ndarray) -> np.ndarray:
        return z

    bounds = VariableBounds(lower=np.ones(n), upper=np.full(n, np.inf))
    return MathematicalProgram(J=J, grad=grad, z0=2.0 * np.ones(n), bounds=bounds)


def _equality_circle_problem() -> MathematicalProgram:
    # min z[0] + z[1]  s.t.  z[0]^2 + z[1]^2 = 1
    # KKT optimum at z* = (-1, -1) / sqrt(2), cost* = -sqrt(2)
    def J(z: np.ndarray) -> float:
        return float(z[0] + z[1])

    def grad(z: np.ndarray) -> np.ndarray:
        return np.array([1.0, 1.0])

    def h(z: np.ndarray) -> np.ndarray:
        return np.array([z[0] ** 2 + z[1] ** 2 - 1.0])

    def jac_h(z: np.ndarray) -> np.ndarray:
        return np.array([[2.0 * z[0], 2.0 * z[1]]])

    eq = EqualityConstraint(h=h, jac=jac_h, name="unit_circle")
    return MathematicalProgram(
        J=J,
        grad=grad,
        z0=np.array([-0.5, -0.5]),
        equalities=(eq,),
    )


def _inequality_quadratic_problem() -> MathematicalProgram:
    # min (z[0] - 2)^2 + (z[1] - 1)^2  s.t.  z[0] + z[1] <= 1, z >= 0
    # i.e. g1(z) = 1 - z[0] - z[1] >= 0, g2(z) = z >= 0 (as box bounds)
    def J(z: np.ndarray) -> float:
        return float((z[0] - 2.0) ** 2 + (z[1] - 1.0) ** 2)

    def grad(z: np.ndarray) -> np.ndarray:
        return np.array([2.0 * (z[0] - 2.0), 2.0 * (z[1] - 1.0)])

    def g(z: np.ndarray) -> np.ndarray:
        return np.array([1.0 - z[0] - z[1]])

    def jac_g(z: np.ndarray) -> np.ndarray:
        return np.array([[-1.0, -1.0]])

    ineq = InequalityConstraint(g=g, jac=jac_g, name="sum_le_one")
    bounds = VariableBounds(lower=np.zeros(2), upper=np.full(2, np.inf))
    return MathematicalProgram(
        J=J,
        grad=grad,
        z0=np.array([0.5, 0.5]),
        bounds=bounds,
        inequalities=(ineq,),
    )


STANDARD_OPTIMIZATION_CASES: tuple[OptimizerBenchmarkCase, ...] = (
    OptimizerBenchmarkCase(
        id="quadratic",
        label="Unconstrained quadratic",
        build=_quadratic_problem,
        z_star=np.array([1.0, 2.0, 3.0]),
        cost_star=0.0,
    ),
    OptimizerBenchmarkCase(
        id="rosenbrock",
        label="Rosenbrock (n=5)",
        build=lambda: _rosenbrock_problem(5),
        z_star=np.ones(5),
        cost_star=0.0,
    ),
    OptimizerBenchmarkCase(
        id="box_qp",
        label="Box-constrained QP",
        build=_box_constrained_quadratic_problem,
        z_star=np.ones(4),
        cost_star=0.5 * 4,
    ),
    OptimizerBenchmarkCase(
        id="circle_eq",
        label="Equality circle",
        build=_equality_circle_problem,
        z_star=np.array([-1.0, -1.0]) / np.sqrt(2.0),
        cost_star=-np.sqrt(2.0),
    ),
    OptimizerBenchmarkCase(
        id="ineq_qp",
        label="Inequality QP",
        build=_inequality_quadratic_problem,
        # KKT solution: project (2, 1) onto z[0] + z[1] = 1 with z >= 0
        z_star=np.array([1.0, 0.0]),
        cost_star=(1.0 - 2.0) ** 2 + (0.0 - 1.0) ** 2,
    ),
)


# --- Sweep ------------------------------------------------------------------


def benchmark_optimizer_backends(
    cases: Sequence[OptimizerBenchmarkCase] = STANDARD_OPTIMIZATION_CASES,
    variants: Sequence[OptimizerBenchmarkVariant] | None = None,
    *,
    n_runs: int = 1,
) -> OptimizerBenchmarkResult:
    """Run each variant on each case and return a flat row table.

    The mathematical program is rebuilt for every (case, variant, run) so
    optimizers do not see warmed-up internal state.
    """
    variant_tuple = (
        default_optimizer_variants() if variants is None else tuple(variants)
    )
    case_tuple = tuple(cases)

    rows: list[OptimizerBenchmarkRow] = []
    for case in case_tuple:
        for variant in variant_tuple:
            rows.append(_run_case(case, variant, n_runs=n_runs))
    return OptimizerBenchmarkResult(
        cases=case_tuple,
        variants=variant_tuple,
        n_runs=int(n_runs),
        rows=tuple(rows),
    )


def _run_case(
    case: OptimizerBenchmarkCase,
    variant: OptimizerBenchmarkVariant,
    *,
    n_runs: int,
) -> OptimizerBenchmarkRow:
    optimizer = _make_optimizer(variant)

    durations: list[float] = []
    last_result: OptimizationResult | None = None
    last_program: MathematicalProgram | None = None
    for _ in range(int(n_runs)):
        program = case.build()
        last_program = program
        t0 = time.perf_counter()
        last_result = optimizer.solve(program)
        durations.append(time.perf_counter() - t0)

    assert last_result is not None and last_program is not None
    eq_inf, min_ineq, bound_inf = _constraint_metrics(last_program, last_result.z)
    return OptimizerBenchmarkRow(
        case_id=case.id,
        case_label=case.label,
        variant=variant,
        success=bool(last_result.success),
        solve_s=float(np.mean(durations)),
        cost=last_result.cost,
        cost_err=_cost_error(last_result.cost, case.cost_star),
        z_err=_z_error(last_result.z, case.z_star),
        eq_inf=eq_inf,
        min_ineq=min_ineq,
        bound_inf=bound_inf,
        nit=_stat_int(last_result, "nit"),
        nfev=_stat_int(last_result, "nfev"),
        njev=_stat_int(last_result, "njev"),
        message=last_result.message,
    )


def _make_optimizer(variant: OptimizerBenchmarkVariant) -> Optimizer:
    if variant.backend == "scipy":
        method = variant.method or "SLSQP"
        return ScipyMinimizeOptimizer(method=method, options=dict(variant.options))
    if variant.backend == "ipopt":
        from minilink.optimization.optimizers.ipopt import IpoptOptimizer

        opts = dict(variant.options)
        tol = opts.pop("tol", None)
        return IpoptOptimizer(options=opts, tol=tol)
    raise ValueError(f"Unknown optimizer backend {variant.backend!r}")


# --- Metrics ---------------------------------------------------------------


def _constraint_metrics(
    program: MathematicalProgram,
    z: np.ndarray,
) -> tuple[float, float | None, float]:
    eq_inf = 0.0
    if program.equalities:
        eq_inf = max(
            float(np.max(np.abs(equality.residual(z))))
            for equality in program.equalities
        )

    min_ineq: float | None = None
    if program.inequalities:
        min_ineq = min(
            float(np.min(inequality.margin(z))) for inequality in program.inequalities
        )

    bound_inf = 0.0
    if program.bounds is not None:
        if program.bounds.lower is not None:
            bound_inf = max(
                bound_inf,
                float(np.max(np.maximum(program.bounds.lower - z, 0.0))),
            )
        if program.bounds.upper is not None:
            bound_inf = max(
                bound_inf,
                float(np.max(np.maximum(z - program.bounds.upper, 0.0))),
            )
    return eq_inf, min_ineq, bound_inf


def _cost_error(cost: float | None, cost_star: float | None) -> float | None:
    if cost is None or cost_star is None:
        return None
    denom = max(1.0, abs(float(cost_star)))
    return float(abs(float(cost) - float(cost_star)) / denom)


def _z_error(z: np.ndarray, z_star: np.ndarray | None) -> float | None:
    if z_star is None:
        return None
    diff = np.linalg.norm(np.asarray(z, dtype=float) - np.asarray(z_star, dtype=float))
    ref = np.linalg.norm(np.asarray(z_star, dtype=float))
    if ref > 0.0:
        return float(diff / ref)
    return float(diff)


def _stat_int(result: OptimizationResult, name: str) -> int | None:
    value = result.stats.get(name)
    if value is None:
        return None
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


# --- Table formatting ------------------------------------------------------

_COL_CASE = 24
_COL_VARIANT = 22


def print_optimizer_benchmark(result: OptimizerBenchmarkResult) -> None:
    """Print a compact (case, variant) table for an optimizer benchmark."""
    width = _COL_CASE + _COL_VARIANT + 90
    print()
    print("-" * width)
    print(
        "  optimizer-backend benchmark   "
        f"variants={len(result.variants)}  "
        f"cases={len(result.cases)}  runs={result.n_runs}"
    )
    print("-" * width)
    print(_header_line())
    print("-" * width)
    for row in result.rows:
        print(_format_row(row))
    print("-" * width)
    failures = [row for row in result.rows if not row.success]
    if failures:
        print()
        print("  failures:")
        for row in failures:
            print(f"    {row.case_id:<14} {row.variant.name:<18} : {row.message}")


def _header_line() -> str:
    return (
        f"  {'case':<{_COL_CASE}} {'variant':<{_COL_VARIANT}} "
        f"{'ok':>3} {'solve_s':>9} {'cost':>13} {'|dcost|':>10} "
        f"{'|dz|/|z*|':>10} {'eq_inf':>10} {'bnd_inf':>9} "
        f"{'nit':>5} {'nfev':>6} {'njev':>6}"
    )


def _format_row(row: OptimizerBenchmarkRow) -> str:
    cost = "n/a" if row.cost is None else f"{row.cost:13.5g}"
    cost_err = "n/a" if row.cost_err is None else f"{row.cost_err:10.2e}"
    z_err = "n/a" if row.z_err is None else f"{row.z_err:10.2e}"
    nit = "n/a" if row.nit is None else f"{row.nit:5d}"
    nfev = "n/a" if row.nfev is None else f"{row.nfev:6d}"
    njev = "n/a" if row.njev is None else f"{row.njev:6d}"
    return (
        f"  {row.case_label:<{_COL_CASE}} {row.variant.name:<{_COL_VARIANT}} "
        f"{('Y' if row.success else 'N'):>3} {row.solve_s:9.5f} {cost} "
        f"{cost_err} {z_err} "
        f"{row.eq_inf:10.2e} {row.bound_inf:9.2e} "
        f"{nit} {nfev} {njev}"
    )
