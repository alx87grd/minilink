"""How fast is one ``step(x, u, k)`` call?

Times repeated discrete updates of one :class:`~minilink.core.system.StepSystem`
across the native Python path and the compiled NumPy/JAX evaluators, reporting
compile time, per-call time, and speedup versus native.
"""

from __future__ import annotations

import time
from dataclasses import dataclass

from benchmarks.common import format_benchmark_backend_label


@dataclass(frozen=True)
class StepEvaluatorBenchmarkVariant:
    """One way to evaluate ``step(x, u, k)``."""

    name: str
    backend: str | None


@dataclass(frozen=True)
class StepEvaluatorBenchmarkRow:
    """One benchmark row for a compiled or native step evaluator."""

    variant: StepEvaluatorBenchmarkVariant
    compile_s: float | None
    loop_s: float | None
    us_per_call: float | None
    speedup_vs_native: float | None
    message: str = ""


@dataclass(frozen=True)
class StepEvaluatorBenchmarkResult:
    """Benchmark result for repeated ``step`` evaluations on one system."""

    system_name: str
    n_calls: int
    k: int
    rows: tuple[StepEvaluatorBenchmarkRow, ...]


DEFAULT_STEP_EVALUATOR_VARIANTS: tuple[StepEvaluatorBenchmarkVariant, ...] = (
    StepEvaluatorBenchmarkVariant("native", None),
    StepEvaluatorBenchmarkVariant("numpy", "numpy"),
    StepEvaluatorBenchmarkVariant("jax", "jax"),
)


def benchmark_step_evaluators(
    system,
    x,
    u,
    k: int = 0,
    *,
    n_calls: int = 100_000,
    variants: tuple[
        StepEvaluatorBenchmarkVariant, ...
    ] = DEFAULT_STEP_EVALUATOR_VARIANTS,
) -> StepEvaluatorBenchmarkResult:
    """Benchmark ``system.step`` and compiled evaluator variants on one sample."""
    rows: list[StepEvaluatorBenchmarkRow] = []
    native_loop_s = None

    for variant in variants:
        if variant.backend is None:
            row = _benchmark_native(system, variant, x, u, k, n_calls)
            native_loop_s = row.loop_s
            rows.append(row)
            continue
        rows.append(
            _benchmark_compiled(system, variant, x, u, k, n_calls, native_loop_s)
        )

    return StepEvaluatorBenchmarkResult(
        system_name=str(getattr(system, "name", type(system).__name__)),
        n_calls=n_calls,
        k=k,
        rows=tuple(rows),
    )


def print_step_benchmark(result: StepEvaluatorBenchmarkResult) -> None:
    """Print a compact table for ``benchmark_step_evaluators``."""
    width = 72
    print()
    print("-" * width)
    print("  step() evaluator benchmark")
    print(
        f"  system: {result.system_name}    n_calls: {result.n_calls}    k: {result.k}"
    )
    print("-" * width)
    print(
        f"  {'backend':<12}  {'compile(s)':>10}  {'loop(s)':>11}  "
        f"{'us/call':>9}  {'vs native':>10}"
    )
    print(f"  {'-' * 12}  {'-' * 10}  {'-' * 11}  {'-' * 9}  {'-' * 10}")
    for row in result.rows:
        print(_format_step_row(row))
    print("-" * width)


def _benchmark_native(
    system,
    variant: StepEvaluatorBenchmarkVariant,
    x,
    u,
    k: int,
    n_calls: int,
) -> StepEvaluatorBenchmarkRow:
    loop_s = None
    message = ""
    try:
        loop_s = _loop_time(system.step, x, u, k, n_calls)
    except RecursionError:
        message = "skipped: recursive native step"
    return _row(
        variant,
        compile_s=None,
        loop_s=loop_s,
        n_calls=n_calls,
        native_loop_s=loop_s,
        message=message,
    )


def _benchmark_compiled(
    system,
    variant: StepEvaluatorBenchmarkVariant,
    x,
    u,
    k: int,
    n_calls: int,
    native_loop_s: float | None,
) -> StepEvaluatorBenchmarkRow:
    values = _evaluator_inputs(variant, x, u)
    if values is None:
        return _row(
            variant,
            compile_s=None,
            loop_s=None,
            n_calls=n_calls,
            native_loop_s=native_loop_s,
            message="skipped: jax not installed",
        )
    x_eval, u_eval = values

    t0 = time.perf_counter()
    evaluator = system.compile(backend=variant.backend, verbose=False)
    compile_s = time.perf_counter() - t0

    step = evaluator.step
    _block_if_needed(step(x_eval, u_eval, k))
    loop_s = _loop_time(step, x_eval, u_eval, k, n_calls)
    return _row(
        variant,
        compile_s=compile_s,
        loop_s=loop_s,
        n_calls=n_calls,
        native_loop_s=native_loop_s,
    )


def _evaluator_inputs(
    variant: StepEvaluatorBenchmarkVariant,
    x,
    u,
):
    if variant.backend != "jax":
        return x, u
    try:
        import jax.numpy as jnp
    except ImportError:
        return None
    return jnp.asarray(x), jnp.asarray(u)


def _loop_time(step, x, u, k: int, n_calls: int) -> float:
    t0 = time.perf_counter()
    for _ in range(n_calls):
        y = step(x, u, k)
    _block_if_needed(y)
    return time.perf_counter() - t0


def _block_if_needed(value) -> None:
    if hasattr(value, "block_until_ready"):
        value.block_until_ready()


def _row(
    variant: StepEvaluatorBenchmarkVariant,
    *,
    compile_s: float | None,
    loop_s: float | None,
    n_calls: int,
    native_loop_s: float | None,
    message: str = "",
) -> StepEvaluatorBenchmarkRow:
    us_per_call = None if loop_s is None else (loop_s / n_calls) * 1e6
    speedup = None
    if native_loop_s is not None and loop_s is not None and loop_s > 0.0:
        speedup = native_loop_s / loop_s
    return StepEvaluatorBenchmarkRow(
        variant=variant,
        compile_s=compile_s,
        loop_s=loop_s,
        us_per_call=us_per_call,
        speedup_vs_native=speedup,
        message=message,
    )


def _format_step_row(row: StepEvaluatorBenchmarkRow) -> str:
    backend = row.variant.name
    if row.variant.backend == "jax" and not row.message:
        backend = format_benchmark_backend_label("jax")
    compile_s = "         -" if row.compile_s is None else f"{row.compile_s:>10.5f}"
    loop_s = "   skipped" if row.loop_s is None else f"{row.loop_s:>10.5f}"
    us = "     n/a" if row.us_per_call is None else f"{row.us_per_call:>8.3f}"
    speedup = (
        "    n/a"
        if row.speedup_vs_native is None
        else f"{row.speedup_vs_native:>7.2f}x"
    )
    suffix = f"  ({row.message})" if row.message else ""
    return f"  {backend:<12}  {compile_s}  {loop_s}  {us}  {speedup}{suffix}"
