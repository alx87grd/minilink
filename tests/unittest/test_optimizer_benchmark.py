"""Smoke tests for :mod:`minilink.optimization.benchmark`."""

import numpy as np

from minilink.optimization.benchmark import (
    STANDARD_OPTIMIZATION_CASES,
    OptimizerBenchmarkVariant,
    benchmark_optimizer_backends,
    print_optimizer_benchmark,
)


def test_standard_cases_build_and_run_with_scipy():
    variants = (
        OptimizerBenchmarkVariant(
            name="scipy-SLSQP",
            method="scipy_slsqp",
            options={"maxiter": 200, "ftol": 1e-9, "disp": False},
        ),
    )
    result = benchmark_optimizer_backends(STANDARD_OPTIMIZATION_CASES, variants)
    assert len(result.rows) == len(STANDARD_OPTIMIZATION_CASES)
    for row in result.rows:
        assert row.solve_s >= 0.0
        if row.cost_err is not None:
            assert np.isfinite(row.cost_err)


def test_print_optimizer_benchmark_does_not_crash(capsys):
    variants = (
        OptimizerBenchmarkVariant(
            name="scipy-SLSQP",
            method="scipy_slsqp",
            options={"maxiter": 200, "ftol": 1e-9, "disp": False},
        ),
    )
    result = benchmark_optimizer_backends(STANDARD_OPTIMIZATION_CASES, variants)
    print_optimizer_benchmark(result)
    captured = capsys.readouterr()
    assert "optimizer-backend benchmark" in captured.out
    assert "case" in captured.out
