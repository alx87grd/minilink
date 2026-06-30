"""Manual dynamic-programming backend benchmark runner.

Compares the ``loop``, ``numpy``, and ``jax`` value-iteration backends on a
pendulum swing-up. The loop backend is only timed on the small grid (it is
orders of magnitude slower); ``numpy`` and ``jax`` are also timed on a large
grid to show the device speedup at scale.

Run from the repository root::

    python benchmarks/run_dp_backends.py
"""

from __future__ import annotations

from benchmarks.dynamic_programming import (
    benchmark_backend,
    check_agreement,
    print_dp_benchmark,
)

# Edit these, then re-run.
small_grid = (21, 21)
small_u = (5,)
large_grid = (151, 151)
large_u = (15,)
runs = {"loop": 1, "numpy": 3, "jax": 3}  # loop is slow; time it once

jax_available = True
try:
    import jax  # noqa: F401
except ImportError:
    jax_available = False

# Small grid: all three backends (loop included).
small_backends = ["loop", "numpy"] + (["jax"] if jax_available else [])
small_rows = [
    benchmark_backend(be, small_grid, small_u, runs=runs[be]) for be in small_backends
]
print(f"\n=== Small grid {small_grid}x{small_u} (loop included) ===")
print_dp_benchmark(small_rows, reference="loop")
check_agreement(small_rows)

# Large grid: numpy vs jax (loop is impractical here).
large_backends = ["numpy"] + (["jax"] if jax_available else [])
large_rows = [
    benchmark_backend(be, large_grid, large_u, runs=runs[be]) for be in large_backends
]
print(f"\n=== Large grid {large_grid}x{large_u} (numpy vs jax) ===")
print_dp_benchmark(large_rows, reference="numpy")
check_agreement(large_rows)
