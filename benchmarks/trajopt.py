"""Which trajopt transcription, backend, and solver combination wins on cartpole?

Sweeps transcription (collocation / shooting / multiple shooting), compile
backend (NumPy finite differences vs JAX analytic derivatives), solver preset
(SLSQP / trust-constr / Ipopt), precision, and cold/warm starts on a cartpole
swing-up, comparing total, transcription, and solve wall time plus constraint
feasibility.
"""

import time
from collections.abc import Mapping
from dataclasses import dataclass, field, replace
from pathlib import Path

import numpy as np

from benchmarks.common import ansi, ipopt_available
from minilink.core.backends import configure_jax
from minilink.core.costs import QuadraticCost
from minilink.core.trajectory import Trajectory
from minilink.dynamics.catalog.pendulum.cartpole import CartPole
from minilink.optimization.evaluators.program_evaluator import (
    MathematicalProgramEvaluator,
)
from minilink.optimization.mathematical_program import MathematicalProgram
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
    #: If true, print :class:`~minilink.optimization.mathematical_program.MathematicalProgram`
    #: details once per distinct ``(transcription, compile_backend, n_steps, tf)``.
    print_mathematical_program_details: bool = False


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
    optimizer_method: str = "scipy_slsqp"
    optimizer_options: Mapping[str, object] = field(default_factory=dict)
    use_hessian: bool = False


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
    try:
        from minilink.dynamics.catalog.pendulum.cartpole import (
            JaxCartPole,  # noqa: F401
        )

        return True
    except ImportError:
        return False


def default_trajectory_optimization_variants(
    *,
    starts: tuple[str, ...] = ("cold", "warm"),
    ftol: float = 1e-2,
    include_ipopt: bool = True,
) -> tuple[TrajectoryOptimizationBenchmarkVariant, ...]:
    """Return the default cartpole trajopt benchmark sweep.

    When ``include_ipopt`` is true and ``cyipopt`` is importable, appends Ipopt
    variants for each transcription, using JAX analytic derivatives if JAX is
    available and otherwise NumPy finite differences.
    """
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
        ]

        if include_ipopt and ipopt_available():
            variants += [
                TrajectoryOptimizationBenchmarkVariant(
                    name="jax-collocation-ipopt",
                    transcription="collocation",
                    compile_backend="jax",
                    derivative="jax",
                    precision="x64",
                    optimizer_backend="ipopt",
                    optimizer_method="ipopt",
                    optimizer_options={"tol": float(ftol)},
                    use_hessian=False,
                ),
                TrajectoryOptimizationBenchmarkVariant(
                    name="jax-shooting-ipopt",
                    transcription="shooting",
                    compile_backend="jax",
                    derivative="jax",
                    precision="x64",
                    optimizer_backend="ipopt",
                    optimizer_method="ipopt",
                    optimizer_options={"tol": float(ftol)},
                    use_hessian=False,
                ),
                TrajectoryOptimizationBenchmarkVariant(
                    name="jax-multiple-ipopt",
                    transcription="multiple_shooting",
                    compile_backend="jax",
                    derivative="jax",
                    precision="x64",
                    optimizer_backend="ipopt",
                    optimizer_method="ipopt",
                    optimizer_options={"tol": float(ftol)},
                    use_hessian=False,
                ),
            ]
    elif include_ipopt and ipopt_available():
        variants += [
            TrajectoryOptimizationBenchmarkVariant(
                name="numpy-collocation-ipopt",
                transcription="collocation",
                compile_backend="numpy",
                derivative="finite-diff",
                optimizer_backend="ipopt",
                optimizer_method="ipopt",
                optimizer_options={"tol": float(ftol)},
            ),
            TrajectoryOptimizationBenchmarkVariant(
                name="numpy-shooting-ipopt",
                transcription="shooting",
                compile_backend="numpy",
                derivative="finite-diff",
                optimizer_backend="ipopt",
                optimizer_method="ipopt",
                optimizer_options={"tol": float(ftol)},
            ),
            TrajectoryOptimizationBenchmarkVariant(
                name="numpy-multiple-ipopt",
                transcription="multiple_shooting",
                compile_backend="numpy",
                derivative="finite-diff",
                optimizer_backend="ipopt",
                optimizer_method="ipopt",
                optimizer_options={"tol": float(ftol)},
            ),
        ]

    return _with_starts(variants, starts)


def default_trajectory_optimization_solver_variants(
    *,
    starts: tuple[str, ...] = ("cold",),
    ftol: float = 1e-2,
    include_trust_constr: bool = True,
    include_ipopt: bool = True,
) -> tuple[TrajectoryOptimizationBenchmarkVariant, ...]:
    """Return a compact direct-collocation solver-preset sweep.

    The default sweep keeps the benchmark tractable: one transcription
    (direct collocation), cold start only, and a small set of solver presets.
    JAX variants use the JAX program evaluator for analytic derivatives.
    """
    variants = [
        TrajectoryOptimizationBenchmarkVariant(
            name="numpy-collocation-slsqp",
            transcription="collocation",
            compile_backend="numpy",
            derivative="finite-diff",
            optimizer_backend="scipy",
            optimizer_method="scipy_slsqp",
        ),
    ]

    if jax_trajopt_available():
        variants.append(
            TrajectoryOptimizationBenchmarkVariant(
                name="jax-collocation-slsqp",
                transcription="collocation",
                compile_backend="jax",
                derivative="jax",
                precision="x64",
                optimizer_backend="scipy",
                optimizer_method="scipy_slsqp",
            )
        )
        if include_trust_constr:
            variants.append(
                TrajectoryOptimizationBenchmarkVariant(
                    name="jax-collocation-trust",
                    transcription="collocation",
                    compile_backend="jax",
                    derivative="jax",
                    precision="x64",
                    optimizer_backend="scipy",
                    optimizer_method="scipy_trust_constr",
                    use_hessian=True,
                )
            )
        if include_ipopt and ipopt_available():
            variants.append(
                TrajectoryOptimizationBenchmarkVariant(
                    name="jax-collocation-ipopt",
                    transcription="collocation",
                    compile_backend="jax",
                    derivative="jax",
                    precision="x64",
                    optimizer_backend="ipopt",
                    optimizer_method="ipopt",
                    optimizer_options={"tol": float(ftol)},
                    use_hessian=False,
                )
            )
    elif include_ipopt and ipopt_available():
        variants.append(
            TrajectoryOptimizationBenchmarkVariant(
                name="numpy-collocation-ipopt",
                transcription="collocation",
                compile_backend="numpy",
                derivative="finite-diff",
                optimizer_backend="ipopt",
                optimizer_method="ipopt",
                optimizer_options={"tol": float(ftol)},
            )
        )

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

    program_detail_keys: set[tuple[str, str, int, float]] | None = (
        set() if config.print_mathematical_program_details else None
    )

    rows = tuple(
        _summarize(
            variant,
            [
                _run_trajopt_variant(
                    variant,
                    config,
                    warm_guess,
                    program_detail_keys=program_detail_keys,
                )
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
    rule_width = widths["name"] + widths["transcription"] + widths["start"] + 131
    c = result.config
    fast_idx = _fastest_successful_row_indices(result.rows)

    print()
    print("-" * rule_width)
    print(
        "trajectory optimization benchmark "
        f"case={c.case} tf={c.tf:g} steps={c.n_steps} "
        f"maxiter={c.maxiter} runs={c.n_runs} "
        "green=success red=failure bold-green=fastest-success"
    )
    print("-" * rule_width)
    print(_trajopt_table_header(widths))
    print("-" * rule_width)
    for i, row in enumerate(result.rows):
        _print_trajopt_table_row(row, widths, is_winner=i in fast_idx)
    print("-" * rule_width)
    _print_repeated_winner_rows(result, widths, fast_idx)
    _print_fastest_successful_footer(result.rows, fast_idx)
    for row in result.rows:
        if not row.success:
            print(f"{row.variant.name} {row.variant.start} failed: {row.message}")


def _mathematical_program_detail_key(
    variant: TrajectoryOptimizationBenchmarkVariant,
    config: TrajectoryOptimizationBenchmarkConfig,
) -> tuple[str, str, int, float]:
    return (
        variant.transcription,
        str(variant.compile_backend),
        int(config.n_steps),
        float(config.tf),
    )


def _mathematical_program_bounds_summary(program: MathematicalProgram) -> str:
    lo, up = program.lower, program.upper
    if lo is None and up is None:
        return "box_bounds: none"
    parts: list[str] = []
    if lo is not None:
        lo_arr = np.asarray(lo, dtype=float)
        parts.append(
            f"lower len={lo_arr.size} min={float(np.min(lo_arr)):.4g} "
            f"max={float(np.max(lo_arr)):.4g}"
        )
    if up is not None:
        up_arr = np.asarray(up, dtype=float)
        parts.append(
            f"upper len={up_arr.size} min={float(np.min(up_arr)):.4g} "
            f"max={float(np.max(up_arr)):.4g}"
        )
    return "box_bounds: " + "; ".join(parts)


def _print_mathematical_program_details(
    program: MathematicalProgram,
    ev: MathematicalProgramEvaluator,
    *,
    variant: TrajectoryOptimizationBenchmarkVariant,
    config: TrajectoryOptimizationBenchmarkConfig,
) -> None:
    print()
    print("=" * 72)
    print(
        "MathematicalProgram detail (once per transcription / compile_backend / grid)"
    )
    print("=" * 72)
    print(f"representative variant: {variant.name!r} start={variant.start!r}")
    print(
        f"transcription={variant.transcription!r} "
        f"compile_backend={variant.compile_backend!r}"
    )
    print(f"grid: tf={config.tf:g} n_steps={config.n_steps}")
    print(f"n_z={program.n_z}")
    print(f"n_h={ev.n_h} n_g={ev.n_g}")
    print(f"program_evaluator.backend={ev.backend!r}")
    print(_mathematical_program_bounds_summary(program))
    print(
        "analytic callbacks on program: "
        f"grad_J={program.grad_J is not None} "
        f"hess_J={program.hess_J is not None} "
        f"jac_h={program.jac_h is not None} "
        f"jac_g={program.jac_g is not None}"
    )
    if program.metadata:
        print("metadata:")
        for key in sorted(program.metadata, key=str):
            print(f"  {key!r}: {program.metadata[key]!r}")
    print("=" * 72)


# TrajOpt Run
def _run_trajopt_variant(
    variant: TrajectoryOptimizationBenchmarkVariant,
    config: TrajectoryOptimizationBenchmarkConfig,
    warm_guess: Trajectory | None,
    *,
    program_detail_keys: set[tuple[str, str, int, float]] | None = None,
) -> TrajectoryOptimizationBenchmarkRow:
    planner = _planner(variant, config)
    guess = _initial_guess(planner, variant, warm_guess)

    t0 = time.perf_counter()
    transcribe_t0 = time.perf_counter()
    program = planner.transcription.transcribe(
        planner.problem,
        compile_backend=planner.options.compile_backend,
    )
    z0 = planner.transcription.pack_initial_guess(planner.problem, guess)
    transcribe_s = time.perf_counter() - transcribe_t0

    solve_t0 = time.perf_counter()
    optimizer = planner._make_optimizer(program, z0)
    if program_detail_keys is not None:
        key = _mathematical_program_detail_key(variant, config)
        if key not in program_detail_keys:
            program_detail_keys.add(key)
            _print_mathematical_program_details(
                program,
                optimizer.program_evaluator,
                variant=variant,
                config=config,
            )
    solution = optimizer.solve()
    solve_s = time.perf_counter() - solve_t0

    max_eq, min_ineq, max_bound = optimizer.program_evaluator.constraint_violations(
        solution.z
    )
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


# Planner Construction
def _planner(
    variant: TrajectoryOptimizationBenchmarkVariant,
    config: TrajectoryOptimizationBenchmarkConfig,
) -> TrajectoryOptimizationPlanner:
    return TrajectoryOptimizationPlanner(
        _problem(variant),
        transcription=_transcription(variant, config),
        options=TrajectoryOptimizationOptions(
            compile_backend=variant.compile_backend,
            optimizer_method=variant.optimizer_method,
            use_hessian=variant.use_hessian,
            optimizer_options=_optimizer_options(variant, config),
        ),
    )


def _problem(variant: TrajectoryOptimizationBenchmarkVariant) -> PlanningProblem:
    if variant.compile_backend == "jax":
        from minilink.dynamics.catalog.pendulum.cartpole import JaxCartPole

        _configure_jax(variant)
        return _cartpole_problem(JaxCartPole())
    return _cartpole_problem(CartPole())


def _transcription(
    variant: TrajectoryOptimizationBenchmarkVariant,
    config: TrajectoryOptimizationBenchmarkConfig,
):
    if variant.transcription == "collocation":
        return DirectCollocationTranscription(
            DirectCollocationOptions(tf=config.tf, n_steps=config.n_steps)
        )
    if variant.transcription == "shooting":
        return ShootingTranscription(
            ShootingOptions(tf=config.tf, n_steps=config.n_steps)
        )
    if variant.transcription == "multiple_shooting":
        return MultipleShootingTranscription(
            MultipleShootingOptions(tf=config.tf, n_steps=config.n_steps)
        )
    raise NotImplementedError(f"Unknown transcription {variant.transcription!r}")


def _optimizer_options(
    variant: TrajectoryOptimizationBenchmarkVariant,
    config: TrajectoryOptimizationBenchmarkConfig,
) -> dict[str, object]:
    if variant.optimizer_method == "scipy_trust_constr":
        return {
            "maxiter": int(config.maxiter),
            "xtol": float(config.ftol),
            "gtol": float(config.ftol),
            "barrier_tol": float(config.ftol),
            "verbose": 0,
            **dict(variant.optimizer_options),
        }

    if variant.optimizer_method == "ipopt":
        return {
            "print_level": 0,
            "max_iter": int(config.maxiter),
            "tol": float(config.ftol),
            **dict(variant.optimizer_options),
        }

    if variant.optimizer_method != "scipy_slsqp":
        raise NotImplementedError(
            f"Unknown optimizer method {variant.optimizer_method!r}"
        )

    return {
        "disp": False,
        "maxiter": int(config.maxiter),
        "ftol": float(config.ftol),
        **dict(variant.optimizer_options),
    }


def _warm_initial_trajectory(
    config: TrajectoryOptimizationBenchmarkConfig,
) -> Trajectory:
    if config.warm_guess_path:
        path = Path(config.warm_guess_path)
        if path.exists():
            return Trajectory.load(path)

    if jax_trajopt_available():
        from minilink.dynamics.catalog.pendulum.cartpole import JaxCartPole

        configure_jax(enable_x64=True)
        problem = _cartpole_problem(JaxCartPole())
        transcription = DirectCollocationTranscription(
            DirectCollocationOptions(tf=config.tf, n_steps=config.n_steps)
        )
        compile_backend = "jax"
    else:
        problem = _cartpole_problem(CartPole())
        transcription = DirectCollocationTranscription(
            DirectCollocationOptions(tf=config.tf, n_steps=config.n_steps)
        )
        compile_backend = "numpy"

    planner = TrajectoryOptimizationPlanner(
        problem,
        transcription=transcription,
        options=TrajectoryOptimizationOptions(
            compile_backend=compile_backend,
            optimizer_options=_warm_optimizer_options(config),
        ),
    )
    traj = planner.compute_solution()
    if config.warm_guess_path:
        traj.save(config.warm_guess_path)
    return traj


def _warm_optimizer_options(
    config: TrajectoryOptimizationBenchmarkConfig,
) -> dict[str, object]:
    return {
        "disp": False,
        "maxiter": max(int(config.maxiter), 200),
        "ftol": float(config.ftol),
    }


def _cartpole_problem(sys) -> PlanningProblem:
    sys.inputs["u"].lower_bound[0] = -10.0
    sys.inputs["u"].upper_bound[0] = 10.0

    x_start = np.array([-2.0, 1.0, 0.0, 0.0])
    x_goal = np.array([0.0, np.pi, 0.0, 0.0])
    cost = QuadraticCost.from_system(
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
        replace(variant, start=start) for variant in variants for start in starts
    )


# Table Formatting
def _fastest_successful_row_indices(
    rows: tuple[TrajectoryOptimizationBenchmarkRow, ...],
) -> set[int]:
    successful = [i for i, row in enumerate(rows) if row.success]
    if not successful:
        return set()
    best = min(rows[i].total_s for i in successful)
    return {i for i in successful if rows[i].total_s <= best + 1e-15}


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
        f"{'hess':<4} {'opt':<7} {'method':<12} "
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
        f"{str(variant.use_hessian):<4} "
        f"{variant.optimizer_backend:<7} {variant.optimizer_method:<12} "
        f"{str(row.success):>3} {row.total_s:8.3f} "
        f"{row.transcribe_s:8.3f} {row.solve_s:8.3f} "
        f"{nit:>5} {nfev:>6} {njev:>6} {cost} "
        f"{row.eq_inf:10.2e} {row.bound_inf:9.2e}"
    )


def _print_trajopt_table_row(
    row: TrajectoryOptimizationBenchmarkRow,
    widths: dict[str, int],
    *,
    is_winner: bool = False,
    flush: bool = False,
) -> None:
    line = _trajopt_table_row(row, widths)
    if is_winner:
        line = ansi(line, 1, 92)
    elif row.success:
        line = ansi(line, 32)
    else:
        line = ansi(line, 31)
    print(line, flush=flush)


def _print_repeated_winner_rows(
    result: TrajectoryOptimizationBenchmarkResult,
    widths: dict[str, int],
    fast_idx: set[int],
) -> None:
    if not fast_idx:
        return
    print(flush=True)
    label = "Winner row" if len(fast_idx) == 1 else "Tied winner rows"
    print(ansi(f"-- {label} (repeated) --", 90), flush=True)
    for i in sorted(fast_idx):
        _print_trajopt_table_row(
            result.rows[i],
            widths,
            is_winner=True,
            flush=True,
        )


def _print_fastest_successful_footer(
    rows: tuple[TrajectoryOptimizationBenchmarkRow, ...],
    fast_idx: set[int],
) -> None:
    if not fast_idx:
        return
    best = min(rows[i].total_s for i in fast_idx)
    labels = ", ".join(rows[i].variant.name for i in sorted(fast_idx))
    msg = f"Fastest successful total_s: {labels}  total={best:.3f}s"
    print(ansi(msg, 1, 92), flush=True)
