"""Benchmarks for native and compiled system dynamics evaluators."""

from __future__ import annotations

import time
from dataclasses import dataclass

from minilink.compile.jax_utils import format_benchmark_backend_label


@dataclass(frozen=True)
class FEvaluatorBenchmarkVariant:
    """One way to evaluate ``f(x, u, t)``."""

    name: str
    backend: str | None


@dataclass(frozen=True)
class FEvaluatorBenchmarkRow:
    """One benchmark row for a compiled or native evaluator."""

    variant: FEvaluatorBenchmarkVariant
    compile_s: float | None
    loop_s: float | None
    us_per_call: float | None
    speedup_vs_native: float | None
    message: str = ""


@dataclass(frozen=True)
class FEvaluatorBenchmarkResult:
    """Benchmark result for repeated ``f`` evaluations on one system."""

    system_name: str
    n_calls: int
    t: float
    rows: tuple[FEvaluatorBenchmarkRow, ...]


DEFAULT_F_EVALUATOR_VARIANTS: tuple[FEvaluatorBenchmarkVariant, ...] = (
    FEvaluatorBenchmarkVariant("native", None),
    FEvaluatorBenchmarkVariant("numpy", "numpy"),
    FEvaluatorBenchmarkVariant("jax", "jax"),
)


def benchmark_f_evaluators(
    system,
    x,
    u,
    t: float = 0.0,
    *,
    n_calls: int = 100_000,
    variants: tuple[FEvaluatorBenchmarkVariant, ...] = DEFAULT_F_EVALUATOR_VARIANTS,
) -> FEvaluatorBenchmarkResult:
    """Benchmark ``system.f`` and compiled evaluator variants on one sample."""
    rows: list[FEvaluatorBenchmarkRow] = []
    native_loop_s = None

    for variant in variants:
        if variant.backend is None:
            row = _benchmark_native(system, variant, x, u, t, n_calls)
            native_loop_s = row.loop_s
            rows.append(row)
            continue
        rows.append(
            _benchmark_compiled(system, variant, x, u, t, n_calls, native_loop_s)
        )

    return FEvaluatorBenchmarkResult(
        system_name=str(getattr(system, "name", type(system).__name__)),
        n_calls=n_calls,
        t=t,
        rows=tuple(rows),
    )


def print_f_benchmark(result: FEvaluatorBenchmarkResult) -> None:
    """Print a compact table for ``benchmark_f_evaluators``."""
    width = 72
    print()
    print("-" * width)
    print("  f() evaluator benchmark")
    print(
        f"  system: {result.system_name}    n_calls: {result.n_calls}    t: {result.t}"
    )
    print("-" * width)
    print(
        f"  {'backend':<12}  {'compile(s)':>10}  {'loop(s)':>11}  "
        f"{'us/call':>9}  {'vs native':>10}"
    )
    print(f"  {'-' * 12}  {'-' * 10}  {'-' * 11}  {'-' * 9}  {'-' * 10}")
    for row in result.rows:
        print(_format_f_row(row))
    print("-" * width)


# Evaluator Runs
def _benchmark_native(
    system,
    variant: FEvaluatorBenchmarkVariant,
    x,
    u,
    t: float,
    n_calls: int,
) -> FEvaluatorBenchmarkRow:
    loop_s = None
    message = ""
    try:
        loop_s = _loop_time(system.f, x, u, t, n_calls)
    except RecursionError:
        message = "skipped: recursive native f"
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
    variant: FEvaluatorBenchmarkVariant,
    x,
    u,
    t: float,
    n_calls: int,
    native_loop_s: float | None,
) -> FEvaluatorBenchmarkRow:
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

    f = _compiled_f(evaluator, variant.backend)
    _block_if_needed(f(x_eval, u_eval, t))
    loop_s = _loop_time(f, x_eval, u_eval, t, n_calls)
    return _row(
        variant,
        compile_s=compile_s,
        loop_s=loop_s,
        n_calls=n_calls,
        native_loop_s=native_loop_s,
    )


def _evaluator_inputs(
    variant: FEvaluatorBenchmarkVariant,
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


def _compiled_f(evaluator, backend: str | None):
    if backend != "jax":
        return evaluator.f
    get_f_jit = getattr(evaluator, "get_f_jit", None)
    return get_f_jit() if callable(get_f_jit) else evaluator.f


def _loop_time(f, x, u, t: float, n_calls: int) -> float:
    t0 = time.perf_counter()
    for _ in range(n_calls):
        y = f(x, u, t)
    _block_if_needed(y)
    return time.perf_counter() - t0


def _block_if_needed(value) -> None:
    if hasattr(value, "block_until_ready"):
        value.block_until_ready()


def _row(
    variant: FEvaluatorBenchmarkVariant,
    *,
    compile_s: float | None,
    loop_s: float | None,
    n_calls: int,
    native_loop_s: float | None,
    message: str = "",
) -> FEvaluatorBenchmarkRow:
    us_per_call = None if loop_s is None else (loop_s / n_calls) * 1e6
    speedup = None
    if native_loop_s is not None and loop_s is not None and loop_s > 0.0:
        speedup = native_loop_s / loop_s
    return FEvaluatorBenchmarkRow(
        variant=variant,
        compile_s=compile_s,
        loop_s=loop_s,
        us_per_call=us_per_call,
        speedup_vs_native=speedup,
        message=message,
    )


# Table Formatting
def _format_f_row(row: FEvaluatorBenchmarkRow) -> str:
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
    return f"  {backend:<12}  {compile_s}  {loop_s}  {us}  {speedup}"
