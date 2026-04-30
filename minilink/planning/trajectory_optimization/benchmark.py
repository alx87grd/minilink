"""Benchmarks for trajectory-optimization transcriptions and optimizers."""

import time
from collections.abc import Mapping
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from minilink.compile.jax_utils import configure_jax
from minilink.core.costs import JaxQuadraticCost, QuadraticCost
from minilink.core.trajectory import Trajectory
from minilink.dynamics.catalog.pendulum.cartpole import CartPole
from minilink.optimization.mathematical_program import MathematicalProgram
from minilink.optimization.optimizers.scipy_minimize import ScipyMinimizeOptimizer
from minilink.planning.initial_guess import default_initial_trajectory
from minilink.planning.problems import PlanningProblem
from minilink.planning.trajectory_optimization.direct_collocation import (
    DirectCollocationOptions,
    DirectCollocationTranscription,
)
from minilink.planning.trajectory_optimization.multiple_shooting import (
    MultipleShootingOptions,
    MultipleShootingTranscription,
)
from minilink.planning.trajectory_optimization.planner import (
    TrajectoryOptimizationOptions,
    TrajectoryOptimizationPlanner,
)
from minilink.planning.trajectory_optimization.shooting import (
    ShootingOptions,
    ShootingTranscription,
)

try:
    from minilink.dynamics.catalog.pendulum.cartpole import JaxCartPole
    from minilink.planning.trajectory_optimization.jax_direct_collocation import (
        JaxDirectCollocationOptions,
        JaxDirectCollocationTranscription,
    )
    from minilink.planning.trajectory_optimization.jax_multiple_shooting import (
        JaxMultipleShootingOptions,
        JaxMultipleShootingTranscription,
    )
    from minilink.planning.trajectory_optimization.jax_shooting import (
        JaxShootingOptions,
        JaxShootingTranscription,
    )

    HAS_JAX_TRAJOPT = True
except ImportError:
    JaxCartPole = None
    JaxDirectCollocationOptions = None
    JaxDirectCollocationTranscription = None
    JaxMultipleShootingOptions = None
    JaxMultipleShootingTranscription = None
    JaxShootingOptions = None
    JaxShootingTranscription = None
    HAS_JAX_TRAJOPT = False


@dataclass(frozen=True)
class TrajectoryOptimizationBenchmarkConfig:
    """Grid and optimizer defaults for one trajopt benchmark run."""

    case: str = "cartpole"
    tf: float = 5.0
    n_steps: int = 20
    maxiter: int = 200
    ftol: float = 1e-2
    n_runs: int = 1
    warm_guess_path: str | Path | None = None


@dataclass(frozen=True)
class TrajectoryOptimizationBenchmarkVariant:
    """One trajopt transcription/backend/optimizer option."""

    name: str
    transcription: str
    compile_backend: str
    derivative: str
    precision: str = "n/a"
    start: str = "cold"
    optimizer_backend: str = "scipy"
    optimizer_method: str = "SLSQP"
    optimizer_options: Mapping[str, object] = field(default_factory=dict)


@dataclass(frozen=True)
class TrajectoryOptimizationBenchmarkRow:
    """One measured trajectory-optimization variant."""

    variant: TrajectoryOptimizationBenchmarkVariant
    success: bool
    total_s: float
    transcribe_s: float
    solve_s: float
    nit: int | None
    nfev: int | None
    njev: int | None
    cost: float | None
    eq_inf: float
    min_ineq: float | None
    bound_inf: float
    message: str


@dataclass(frozen=True)
class TrajectoryOptimizationBenchmarkResult:
    """Benchmark result for one trajectory-optimization case."""

    config: TrajectoryOptimizationBenchmarkConfig
    rows: tuple[TrajectoryOptimizationBenchmarkRow, ...]


def jax_trajopt_available() -> bool:
    """Return whether JAX trajectory-optimization variants can run."""
    return HAS_JAX_TRAJOPT


def default_trajectory_optimization_variants(
    *,
    starts: tuple[str, ...] = ("cold", "warm"),
    ftol: float = 1e-2,
) -> tuple[TrajectoryOptimizationBenchmarkVariant, ...]:
    """Return the default cartpole trajopt benchmark sweep."""
    variants = [
        TrajectoryOptimizationBenchmarkVariant(
            name="numpy-collocation-slsqp",
            transcription="collocation",
            compile_backend="numpy",
            derivative="finite-diff",
        ),
        TrajectoryOptimizationBenchmarkVariant(
            name="numpy-shooting-slsqp",
            transcription="shooting",
            compile_backend="numpy",
            derivative="finite-diff",
        ),
        TrajectoryOptimizationBenchmarkVariant(
            name="numpy-multiple-slsqp",
            transcription="multiple_shooting",
            compile_backend="numpy",
            derivative="finite-diff",
        ),
    ]

    if jax_trajopt_available():
        variants += [
            TrajectoryOptimizationBenchmarkVariant(
                name="jax-collocation-grad-x64",
                transcription="collocation",
                compile_backend="jax",
                derivative="jax",
                precision="x64",
            ),
            TrajectoryOptimizationBenchmarkVariant(
                name="jax-shooting-grad-x64",
                transcription="shooting",
                compile_backend="jax",
                derivative="jax",
                precision="x64",
            ),
            TrajectoryOptimizationBenchmarkVariant(
                name="jax-multiple-grad-x64",
                transcription="multiple_shooting",
                compile_backend="jax",
                derivative="jax",
                precision="x64",
            ),
            TrajectoryOptimizationBenchmarkVariant(
                name="jax-collocation-grad-f32-strict",
                transcription="collocation",
                compile_backend="jax",
                derivative="jax",
                precision="f32",
            ),
            TrajectoryOptimizationBenchmarkVariant(
                name="jax-collocation-grad-f32-relaxed",
                transcription="collocation",
                compile_backend="jax",
                derivative="jax",
                precision="f32",
                optimizer_options={"ftol": max(float(ftol), 1e-5)},
            ),
            TrajectoryOptimizationBenchmarkVariant(
                name="jax-collocation-fd-x64",
                transcription="collocation",
                compile_backend="jax",
                derivative="finite-diff",
                precision="x64",
            ),
        ]

    return _with_starts(variants, starts)


def benchmark_trajectory_optimization(
    config: TrajectoryOptimizationBenchmarkConfig,
    variants: tuple[TrajectoryOptimizationBenchmarkVariant, ...] | None = None,
) -> TrajectoryOptimizationBenchmarkResult:
    """Run a trajectory-optimization benchmark sweep."""
    if config.case != "cartpole":
        raise NotImplementedError(f"Unknown trajopt benchmark case {config.case!r}")
    if variants is None:
        variants = default_trajectory_optimization_variants(ftol=config.ftol)

    warm_guess = None
    if any(variant.start == "warm" for variant in variants):
        warm_guess = _warm_initial_trajectory(config)

    rows = tuple(
        _summarize(
            variant,
            [
                _run_trajopt_variant(variant, config, warm_guess)
                for _ in range(config.n_runs)
            ],
        )
        for variant in variants
    )
    return TrajectoryOptimizationBenchmarkResult(config=config, rows=rows)


def print_trajectory_optimization_benchmark(
    result: TrajectoryOptimizationBenchmarkResult,
) -> None:
    """Print a compact table for a trajectory-optimization benchmark result."""
    widths = _trajopt_table_widths(result.rows)
    rule_width = widths["name"] + widths["transcription"] + widths["start"] + 126
    c = result.config

    print()
    print("-" * rule_width)
    print(
        "trajectory optimization benchmark "
        f"case={c.case} tf={c.tf:g} steps={c.n_steps} "
        f"maxiter={c.maxiter} runs={c.n_runs}"
    )
    print("-" * rule_width)
    print(_trajopt_table_header(widths))
    print("-" * rule_width)
    for row in result.rows:
        print(_trajopt_table_row(row, widths))
    print("-" * rule_width)
    for row in result.rows:
        if not row.success:
            print(f"{row.variant.name} {row.variant.start} failed: {row.message}")


# TrajOpt Run
def _run_trajopt_variant(
    variant: TrajectoryOptimizationBenchmarkVariant,
    config: TrajectoryOptimizationBenchmarkConfig,
    warm_guess: Trajectory | None,
) -> TrajectoryOptimizationBenchmarkRow:
    planner = _planner(variant, config)
    guess = _initial_guess(planner, variant, warm_guess)

    t0 = time.perf_counter()
    transcribe_t0 = time.perf_counter()
    program = planner.transcription.transcribe(
        planner.problem,
        initial_guess=guess,
        compile_backend=planner.options.compile_backend,
    )
    transcribe_s = time.perf_counter() - transcribe_t0

    solve_t0 = time.perf_counter()
    solution = planner.optimizer.solve(program)
    solve_s = time.perf_counter() - solve_t0

    max_eq, min_ineq, max_bound = _constraint_metrics(program, solution.z)
    return TrajectoryOptimizationBenchmarkRow(
        variant=variant,
        success=bool(solution.success),
        total_s=time.perf_counter() - t0,
        transcribe_s=transcribe_s,
        solve_s=solve_s,
        nit=solution.stats.get("nit"),
        nfev=solution.stats.get("nfev"),
        njev=solution.stats.get("njev"),
        cost=solution.cost,
        eq_inf=max_eq,
        min_ineq=min_ineq,
        bound_inf=max_bound,
        message=solution.message,
    )


def _initial_guess(
    planner: TrajectoryOptimizationPlanner,
    variant: TrajectoryOptimizationBenchmarkVariant,
    warm_guess: Trajectory | None,
) -> Trajectory | None:
    if variant.start == "warm":
        return warm_guess
    t = planner.transcription.initial_guess_time_grid(planner.problem)
    return default_initial_trajectory(planner.problem, t)


def _summarize(
    variant: TrajectoryOptimizationBenchmarkVariant,
    runs: list[TrajectoryOptimizationBenchmarkRow],
) -> TrajectoryOptimizationBenchmarkRow:
    last = runs[-1]
    return TrajectoryOptimizationBenchmarkRow(
        variant=variant,
        success=all(run.success for run in runs),
        total_s=float(np.mean([run.total_s for run in runs])),
        transcribe_s=float(np.mean([run.transcribe_s for run in runs])),
        solve_s=float(np.mean([run.solve_s for run in runs])),
        nit=_mean_int_or_none(run.nit for run in runs),
        nfev=_mean_int_or_none(run.nfev for run in runs),
        njev=_mean_int_or_none(run.njev for run in runs),
        cost=_mean_or_none(run.cost for run in runs),
        eq_inf=float(np.mean([run.eq_inf for run in runs])),
        min_ineq=_mean_or_none(run.min_ineq for run in runs),
        bound_inf=float(np.mean([run.bound_inf for run in runs])),
        message=last.message,
    )


def _mean_or_none(values) -> float | None:
    kept = [value for value in values if value is not None]
    if not kept:
        return None
    return float(np.mean(kept))


def _mean_int_or_none(values) -> int | None:
    value = _mean_or_none(values)
    if value is None:
        return None
    return int(round(value))


def _constraint_metrics(
    program: MathematicalProgram,
    z: np.ndarray,
) -> tuple[float, float | None, float]:
    max_eq = 0.0
    if program.equalities:
        max_eq = max(
            float(np.max(np.abs(equality.residual(z))))
            for equality in program.equalities
        )

    min_ineq = None
    if program.inequalities:
        min_ineq = min(
            float(np.min(inequality.margin(z))) for inequality in program.inequalities
        )

    max_bound = 0.0
    if program.bounds is not None:
        if program.bounds.lower is not None:
            max_bound = max(
                max_bound,
                float(np.max(np.maximum(program.bounds.lower - z, 0.0))),
            )
        if program.bounds.upper is not None:
            max_bound = max(
                max_bound,
                float(np.max(np.maximum(z - program.bounds.upper, 0.0))),
            )

    return max_eq, min_ineq, max_bound


# Planner Construction
def _planner(
    variant: TrajectoryOptimizationBenchmarkVariant,
    config: TrajectoryOptimizationBenchmarkConfig,
) -> TrajectoryOptimizationPlanner:
    return TrajectoryOptimizationPlanner(
        _problem(variant),
        transcription=_transcription(variant, config),
        optimizer=_optimizer(variant, config),
        options=TrajectoryOptimizationOptions(compile_backend=variant.compile_backend),
    )


def _problem(variant: TrajectoryOptimizationBenchmarkVariant) -> PlanningProblem:
    if variant.compile_backend == "jax":
        _configure_jax(variant)
        return _cartpole_problem(JaxCartPole(), jax_cost=True)
    return _cartpole_problem(CartPole(), jax_cost=False)


def _transcription(
    variant: TrajectoryOptimizationBenchmarkVariant,
    config: TrajectoryOptimizationBenchmarkConfig,
):
    use_gradient = variant.derivative == "jax"
    if variant.transcription == "collocation":
        if variant.compile_backend == "jax":
            return JaxDirectCollocationTranscription(
                JaxDirectCollocationOptions(
                    tf=config.tf,
                    n_steps=config.n_steps,
                    use_gradient=use_gradient,
                )
            )
        return DirectCollocationTranscription(
            DirectCollocationOptions(tf=config.tf, n_steps=config.n_steps)
        )
    if variant.transcription == "shooting":
        if variant.compile_backend == "jax":
            return JaxShootingTranscription(
                JaxShootingOptions(
                    tf=config.tf,
                    n_steps=config.n_steps,
                    use_gradient=use_gradient,
                )
            )
        return ShootingTranscription(
            ShootingOptions(tf=config.tf, n_steps=config.n_steps)
        )
    if variant.transcription == "multiple_shooting":
        if variant.compile_backend == "jax":
            return JaxMultipleShootingTranscription(
                JaxMultipleShootingOptions(
                    tf=config.tf,
                    n_steps=config.n_steps,
                    use_gradient=use_gradient,
                )
            )
        return MultipleShootingTranscription(
            MultipleShootingOptions(tf=config.tf, n_steps=config.n_steps)
        )
    raise NotImplementedError(f"Unknown transcription {variant.transcription!r}")


def _optimizer(
    variant: TrajectoryOptimizationBenchmarkVariant,
    config: TrajectoryOptimizationBenchmarkConfig,
) -> ScipyMinimizeOptimizer:
    if variant.optimizer_backend != "scipy":
        raise NotImplementedError(
            f"Unknown optimizer backend {variant.optimizer_backend!r}"
        )

    options = {
        "disp": False,
        "maxiter": int(config.maxiter),
        "ftol": float(config.ftol),
        **dict(variant.optimizer_options),
    }
    return ScipyMinimizeOptimizer(method=variant.optimizer_method, options=options)


def _warm_initial_trajectory(
    config: TrajectoryOptimizationBenchmarkConfig,
) -> Trajectory:
    if config.warm_guess_path:
        path = Path(config.warm_guess_path)
        if path.exists():
            return Trajectory.load(path)

    if jax_trajopt_available():
        configure_jax(enable_x64=True)
        problem = _cartpole_problem(JaxCartPole(), jax_cost=True)
        transcription = JaxDirectCollocationTranscription(
            JaxDirectCollocationOptions(tf=config.tf, n_steps=config.n_steps)
        )
        compile_backend = "jax"
    else:
        problem = _cartpole_problem(CartPole(), jax_cost=False)
        transcription = DirectCollocationTranscription(
            DirectCollocationOptions(tf=config.tf, n_steps=config.n_steps)
        )
        compile_backend = "numpy"

    planner = TrajectoryOptimizationPlanner(
        problem,
        transcription=transcription,
        optimizer=_warm_optimizer(config),
        options=TrajectoryOptimizationOptions(compile_backend=compile_backend),
    )
    traj = planner.compute_solution()
    if config.warm_guess_path:
        traj.save(config.warm_guess_path)
    return traj


def _warm_optimizer(
    config: TrajectoryOptimizationBenchmarkConfig,
) -> ScipyMinimizeOptimizer:
    return ScipyMinimizeOptimizer(
        options={
            "disp": False,
            "maxiter": max(int(config.maxiter), 200),
            "ftol": float(config.ftol),
        }
    )


def _cartpole_problem(sys, *, jax_cost: bool) -> PlanningProblem:
    sys.inputs["u"].lower_bound[0] = -10.0
    sys.inputs["u"].upper_bound[0] = 10.0

    x_start = np.array([-2.0, 1.0, 0.0, 0.0])
    x_goal = np.array([0.0, np.pi, 0.0, 0.0])
    cost_cls = JaxQuadraticCost if jax_cost else QuadraticCost
    cost = cost_cls.from_system(
        sys,
        Q=np.diag([1.0, 1.0, 0.0, 0.0]),
        R=np.diag([1.0]),
        S=np.zeros((sys.n, sys.n)),
        xbar=x_goal,
        ubar=np.zeros(sys.m),
    )
    return PlanningProblem(sys=sys, x_start=x_start, x_goal=x_goal, cost=cost)


def _configure_jax(variant: TrajectoryOptimizationBenchmarkVariant) -> None:
    if not jax_trajopt_available():
        raise ModuleNotFoundError("JAX trajopt benchmark components are not installed")
    configure_jax(enable_x64=variant.precision == "x64")


def _with_starts(
    variants: list[TrajectoryOptimizationBenchmarkVariant],
    starts: tuple[str, ...],
) -> tuple[TrajectoryOptimizationBenchmarkVariant, ...]:
    return tuple(
        TrajectoryOptimizationBenchmarkVariant(
            name=variant.name,
            transcription=variant.transcription,
            compile_backend=variant.compile_backend,
            derivative=variant.derivative,
            precision=variant.precision,
            start=start,
            optimizer_backend=variant.optimizer_backend,
            optimizer_method=variant.optimizer_method,
            optimizer_options=variant.optimizer_options,
        )
        for variant in variants
        for start in starts
    )


# Table Formatting
def _trajopt_table_widths(
    rows: tuple[TrajectoryOptimizationBenchmarkRow, ...],
) -> dict[str, int]:
    return {
        "name": max([21, *(len(row.variant.name) for row in rows)]),
        "transcription": max([13, *(len(row.variant.transcription) for row in rows)]),
        "start": max([5, *(len(row.variant.start) for row in rows)]),
    }


def _trajopt_table_header(widths: dict[str, int]) -> str:
    return (
        f"{'variant':<{widths['name']}} "
        f"{'transcription':<{widths['transcription']}} "
        f"{'start':<{widths['start']}} {'backend':<7} {'deriv':<11} {'prec':<4} "
        f"{'opt':<7} {'method':<12} "
        f"{'ok':>3} {'total':>8} {'trans':>8} {'solve':>8} "
        f"{'nit':>5} {'nfev':>6} {'njev':>6} {'cost':>11} "
        f"{'eq_inf':>10} {'bound':>9}"
    )


def _trajopt_table_row(
    row: TrajectoryOptimizationBenchmarkRow,
    widths: dict[str, int],
) -> str:
    variant = row.variant
    cost = "n/a" if row.cost is None else f"{row.cost:11.4g}"
    nit = "n/a" if row.nit is None else str(row.nit)
    nfev = "n/a" if row.nfev is None else str(row.nfev)
    njev = "n/a" if row.njev is None else str(row.njev)
    return (
        f"{variant.name:<{widths['name']}} "
        f"{variant.transcription:<{widths['transcription']}} "
        f"{variant.start:<{widths['start']}} {variant.compile_backend:<7} "
        f"{variant.derivative:<11} {variant.precision:<4} "
        f"{variant.optimizer_backend:<7} {variant.optimizer_method:<12} "
        f"{str(row.success):>3} {row.total_s:8.3f} "
        f"{row.transcribe_s:8.3f} {row.solve_s:8.3f} "
        f"{nit:>5} {nfev:>6} {njev:>6} {cost} "
        f"{row.eq_inf:10.2e} {row.bound_inf:9.2e}"
    )
