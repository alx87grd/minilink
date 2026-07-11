"""End-to-end trajopt timing breakdown (single cold solve).

Measures every phase from problem construction through reconstruct, including
JAX program-evaluator JIT compile inside ``Optimizer`` construction.

Run from repo root::

    python benchmarks/run_trajopt_timing_breakdown.py

The benchmark harness in ``benchmarks/trajopt.py`` reports only::

    transcribe_s  = transcribe + pack_initial_guess
    solve_s       = Optimizer(...) + optimizer.solve()
                    ^ includes JaxMathematicalProgramEvaluator JIT warm-start

JAX transcriptions (collocation / shooting / multiple shooting) call
``problem.sys.f`` directly in the NLP — they do **not** use
``dynamics_function`` or compiled evaluators on the hot path.
"""

from __future__ import annotations

import time
from dataclasses import dataclass

from benchmarks.trajopt import (
    TrajectoryOptimizationBenchmarkConfig,
    TrajectoryOptimizationBenchmarkVariant,
    _configure_jax,
    _initial_guess,
    _optimizer_options,
    _planner,
)
from minilink.core.backends import configure_jax
from minilink.optimization.optimizer import Optimizer


@dataclass(frozen=True)
class TrajoptTimingBreakdown:
    """Wall-clock seconds for one trajopt workflow."""

    problem_s: float
    transcribe_s: float
    pack_guess_s: float
    optimizer_compile_s: float
    solve_s: float
    reconstruct_s: float
    total_s: float
    benchmark_transcribe_s: float
    benchmark_solve_s: float

    def as_table(self) -> str:
        lines = [
            f"{'phase':<24} {'seconds':>10}",
            f"{'-' * 24} {'-' * 10}",
            f"{'problem build':<24} {self.problem_s:10.4f}",
            f"{'transcribe':<24} {self.transcribe_s:10.4f}",
            f"{'pack initial guess':<24} {self.pack_guess_s:10.4f}",
            f"{'optimizer compile':<24} {self.optimizer_compile_s:10.4f}",
            f"{'solve (scipy only)':<24} {self.solve_s:10.4f}",
            f"{'reconstruct':<24} {self.reconstruct_s:10.4f}",
            f"{'TOTAL (workflow)':<24} {self.total_s:10.4f}",
            "",
            f"{'benchmark transcribe_s':<24} {self.benchmark_transcribe_s:10.4f}"
            f"  (= transcribe + pack)",
            f"{'benchmark solve_s':<24} {self.benchmark_solve_s:10.4f}"
            f"  (= optimizer compile + solve)",
        ]
        return "\n".join(lines)


def _variant(
    name: str, *, precision: str = "f32-relaxed"
) -> TrajectoryOptimizationBenchmarkVariant:
    if name == "jax-collocation-grad-x64":
        return TrajectoryOptimizationBenchmarkVariant(
            name=name,
            transcription="collocation",
            compile_backend="jax",
            derivative="grad",
            precision="x64",
        )
    if name == "jax-collocation-grad-f32-strict":
        return TrajectoryOptimizationBenchmarkVariant(
            name=name,
            transcription="collocation",
            compile_backend="jax",
            derivative="grad",
            precision="f32-strict",
        )
    return TrajectoryOptimizationBenchmarkVariant(
        name="jax-collocation-grad-f32-relaxed",
        transcription="collocation",
        compile_backend="jax",
        derivative="grad",
        precision="f32-relaxed",
    )


def run_breakdown(
    variant: TrajectoryOptimizationBenchmarkVariant,
    config: TrajectoryOptimizationBenchmarkConfig,
) -> TrajoptTimingBreakdown:
    """One cold solve with per-phase wall-clock timing."""
    workflow_t0 = time.perf_counter()

    problem_t0 = time.perf_counter()
    if variant.compile_backend == "jax":
        _configure_jax(variant)
    problem_s = time.perf_counter() - problem_t0

    planner = _planner(variant, config)
    guess = _initial_guess(planner, variant, None)

    transcribe_t0 = time.perf_counter()
    program = planner.transcription.transcribe(
        planner.problem,
        compile_backend=planner.options.compile_backend,
    )
    transcribe_s = time.perf_counter() - transcribe_t0

    pack_t0 = time.perf_counter()
    z0 = planner.transcription.pack_initial_guess(planner.problem, guess)
    pack_guess_s = time.perf_counter() - pack_t0

    compile_t0 = time.perf_counter()
    optimizer = Optimizer(
        program,
        z0=z0,
        method=planner.options.optimizer_method,
        use_hessian=planner.options.use_hessian,
        options=_optimizer_options(variant, config),
    )
    optimizer_compile_s = time.perf_counter() - compile_t0

    solve_t0 = time.perf_counter()
    result = optimizer.solve(record_solve_time=True)
    solve_s = time.perf_counter() - solve_t0

    reconstruct_t0 = time.perf_counter()
    planner.transcription.reconstruct_result(
        result,
        problem=planner.problem,
        compile_backend=planner.options.compile_backend,
    )
    reconstruct_s = time.perf_counter() - reconstruct_t0

    total_s = time.perf_counter() - workflow_t0
    benchmark_transcribe_s = transcribe_s + pack_guess_s
    benchmark_solve_s = optimizer_compile_s + solve_s

    return TrajoptTimingBreakdown(
        problem_s=problem_s,
        transcribe_s=transcribe_s,
        pack_guess_s=pack_guess_s,
        optimizer_compile_s=optimizer_compile_s,
        solve_s=solve_s,
        reconstruct_s=reconstruct_s,
        total_s=total_s,
        benchmark_transcribe_s=benchmark_transcribe_s,
        benchmark_solve_s=benchmark_solve_s,
    )


def _print_pair(
    label: str, cold: TrajoptTimingBreakdown, warm: TrajoptTimingBreakdown
) -> None:
    print(f"\n{label}")
    print(f"{'phase':<24} {'cold':>10} {'2nd run':>10} {'ratio':>8}")
    print(f"{'-' * 24} {'-' * 10} {'-' * 10} {'-' * 8}")
    for field in (
        "problem_s",
        "transcribe_s",
        "pack_guess_s",
        "optimizer_compile_s",
        "solve_s",
        "reconstruct_s",
        "total_s",
        "benchmark_transcribe_s",
        "benchmark_solve_s",
    ):
        c = getattr(cold, field)
        w = getattr(warm, field)
        ratio = c / w if w > 1e-9 else float("nan")
        print(f"{field:<24} {c:10.4f} {w:10.4f} {ratio:8.2f}x")


def main() -> None:
    configure_jax(enable_x64=False)
    config = TrajectoryOptimizationBenchmarkConfig(
        case="cartpole",
        tf=5.0,
        n_steps=20,
        maxiter=200,
        ftol=1e-2,
        n_runs=1,
    )

    print("Trajopt timing breakdown — cartpole direct collocation (JAX)")
    print("Config: tf=5.0 n_steps=20 maxiter=200 ftol=1e-2")
    print()
    print(
        "Note: JAX collocation transcribe uses problem.sys.f directly; "
        "evaluator trace tier does not affect this hot path."
    )

    for variant_name in (
        "jax-collocation-grad-x64",
        "jax-collocation-grad-f32-strict",
        "jax-collocation-grad-f32-relaxed",
    ):
        variant = _variant(variant_name)
        cold = run_breakdown(variant, config)
        warm = run_breakdown(variant, config)
        _print_pair(variant.name, cold, warm)
        print()
        print("Cold run detail:")
        print(cold.as_table())


if __name__ == "__main__":
    main()
