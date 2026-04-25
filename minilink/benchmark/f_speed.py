"""Minimal timing of ``system.f`` vs compiled NumPy vs compiled JAX ``f``."""

from __future__ import annotations

import time
from typing import Any, NamedTuple

import numpy as np

from minilink.benchmark.simulation_speed import format_benchmark_backend_label


class FSpeedTimings(NamedTuple):
    """Seconds for ``n_calls`` evaluations (compile_* is evaluator construction only)."""

    n_calls: int
    native_loop_s: float | None
    numpy_compile_s: float
    numpy_loop_s: float
    jax_compile_s: float | None
    jax_loop_s: float | None


def print_f_speed_table(
    b: FSpeedTimings,
    *,
    system_name: str,
    t: float = 0.0,
) -> None:
    """Print a fixed-layout table for :class:`FSpeedTimings` (no diagram-specific fields)."""
    ref = b.native_loop_s

    def _speedup(loop_s: float | None) -> str:
        if loop_s is None:
            return "    n/a"
        if ref is None or ref <= 0 or loop_s <= 0:
            return "    n/a"
        return f"{ref / loop_s:>7.2f}x"

    def _us_per_call(loop_s: float | None) -> str:
        if loop_s is None or b.n_calls <= 0:
            return "     n/a"
        return f"{(loop_s / b.n_calls) * 1e6:>8.3f}"

    def _row(
        backend: str,
        compile_s: float | None,
        loop_s: float | None,
        *,
        native_speedup: bool = False,
    ) -> None:
        c = f"{compile_s:>10.5f}" if compile_s is not None else "         -"
        if loop_s is None and backend == "native" and ref is None:
            lo = "   skipped"
            us = "     n/a"
            sp = "    n/a"
        elif loop_s is None:
            lo = "   skipped"
            us = "     n/a"
            sp = "    n/a"
        else:
            lo = f"{loop_s:>10.5f}"
            us = _us_per_call(loop_s)
            if native_speedup and ref is not None and loop_s is not None and loop_s > 0:
                sp = f"{ref / loop_s:>7.2f}x"
            else:
                sp = _speedup(loop_s)
        print(f"  {backend:<12}  {c}  {lo}  {us}  {sp}")

    width = 72
    print()
    print("-" * width)
    print("  f() speed benchmark")
    print(f"  system: {system_name}    n_calls: {b.n_calls}    t: {t}")
    print("-" * width)
    print(
        f"  {'backend':<12}  {'compile(s)':>10}  {'loop(s)':>11}  {'us/call':>9}  {'vs native':>10}"
    )
    print(f"  {'-' * 12}  {'-' * 10}  {'-' * 11}  {'-' * 9}  {'-' * 10}")

    _row("native", None, ref, native_speedup=True)
    _row("numpy", b.numpy_compile_s, b.numpy_loop_s)
    if b.jax_loop_s is not None:
        _row(format_benchmark_backend_label("jax"), b.jax_compile_s, b.jax_loop_s)
    else:
        print(
            f"  {'jax':<12}  {'-':>10}  {'(no jax)':>11}  {'n/a':>9}  {'n/a':>10}"
        )
    print("-" * width)


def benchmark_f_speeds(
    system: Any,
    x_np: np.ndarray,
    u_np: np.ndarray,
    t: float = 0.0,
    *,
    n_calls: int = 100_000,
    print_table: bool = True,
) -> FSpeedTimings:
    """Time ``system.f``, then ``compile('numpy').f``, then ``compile('jax').f``.

    If ``print_table`` is ``True``, prints a standard timing table to stdout.

    If JAX is not installed, ``jax_compile_s`` and ``jax_loop_s`` are ``None``.
    If native ``f`` hits :exc:`RecursionError` (e.g. deep recursive ``diag.f``),
    ``native_loop_s`` is ``None`` and NumPy/JAX paths still run.
    """
    name = str(getattr(system, "name", type(system).__name__))

    native_loop_s: float | None
    try:
        t0 = time.perf_counter()
        for _ in range(n_calls):
            system.f(x_np, u_np, t)
        native_loop_s = time.perf_counter() - t0
    except RecursionError:
        native_loop_s = None

    t0 = time.perf_counter()
    ev_np = system.compile(backend="numpy", verbose=False)
    numpy_compile_s = time.perf_counter() - t0
    ev_np.f(x_np, u_np, t)
    t0 = time.perf_counter()
    for _ in range(n_calls):
        ev_np.f(x_np, u_np, t)
    numpy_loop_s = time.perf_counter() - t0

    jax_compile_s: float | None = None
    jax_loop_s: float | None = None

    try:
        import jax.numpy as jnp
    except ImportError:
        b = FSpeedTimings(
            n_calls,
            native_loop_s,
            numpy_compile_s,
            numpy_loop_s,
            None,
            None,
        )
        if print_table:
            print_f_speed_table(b, system_name=name, t=t)
        return b

    x_j = jnp.asarray(np.asarray(x_np, dtype=float))
    u_j = jnp.asarray(np.asarray(u_np, dtype=float))
    t0 = time.perf_counter()
    ev_j = system.compile(backend="jax", verbose=False)
    jax_compile_s = time.perf_counter() - t0
    getter = getattr(ev_j, "get_f_jit", None)
    f_j = getter() if callable(getter) else ev_j.f
    y = f_j(x_j, u_j, t)
    y.block_until_ready()
    t0 = time.perf_counter()
    for _ in range(n_calls):
        y = f_j(x_j, u_j, t)
    y.block_until_ready()
    jax_loop_s = time.perf_counter() - t0

    b = FSpeedTimings(
        n_calls,
        native_loop_s,
        numpy_compile_s,
        numpy_loop_s,
        jax_compile_s,
        jax_loop_s,
    )
    if print_table:
        print_f_speed_table(b, system_name=name, t=t)
    return b
