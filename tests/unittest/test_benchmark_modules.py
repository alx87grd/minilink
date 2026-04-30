"""Smoke tests for importable benchmark modules."""

import unittest

import numpy as np

from minilink.compile.benchmark import (
    FEvaluatorBenchmarkVariant,
    benchmark_f_evaluators,
)
from minilink.core.system import DynamicSystem
from minilink.planning.trajectory_optimization.benchmark import (
    TrajectoryOptimizationBenchmarkConfig,
    TrajectoryOptimizationBenchmarkVariant,
    benchmark_trajectory_optimization,
)


class _TinyStable(DynamicSystem):
    def __init__(self):
        super().__init__(1, 1, 1)
        self.name = "TinyStable"
        self.x0 = np.array([1.0])

    def f(self, x, u, t=0, params=None):
        return np.asarray([-x[0] + u[0]], dtype=float)

    def h(self, x, u, t=0, params=None):
        return x


class TestBenchmarkModules(unittest.TestCase):
    def test_compile_benchmark_returns_rows(self):
        result = benchmark_f_evaluators(
            _TinyStable(),
            np.array([1.0]),
            np.array([0.0]),
            n_calls=2,
            variants=(
                FEvaluatorBenchmarkVariant("native", None),
                FEvaluatorBenchmarkVariant("numpy", "numpy"),
            ),
        )
        self.assertEqual(len(result.rows), 2)
        self.assertEqual(result.rows[1].variant.backend, "numpy")
        self.assertGreaterEqual(result.rows[1].loop_s, 0.0)

    def test_trajopt_benchmark_returns_rows(self):
        config = TrajectoryOptimizationBenchmarkConfig(
            n_steps=3,
            maxiter=1,
            n_runs=1,
        )
        variant = TrajectoryOptimizationBenchmarkVariant(
            name="unit-numpy-collocation",
            transcription="collocation",
            compile_backend="numpy",
            derivative="finite-diff",
            start="cold",
        )
        result = benchmark_trajectory_optimization(config, (variant,))
        self.assertEqual(len(result.rows), 1)
        self.assertEqual(result.rows[0].variant.optimizer_backend, "scipy")


if __name__ == "__main__":
    unittest.main()
