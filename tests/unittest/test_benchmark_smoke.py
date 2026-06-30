"""Smoke tests that benchmark helpers import and return structured results.

Benchmarks measure performance, not correctness; these tests only guard the
repo-root ``benchmarks/`` package against import and API drift.
"""

from __future__ import annotations

import io
import unittest
from contextlib import redirect_stdout

import numpy as np

from benchmarks.dynamic_programming import benchmark_backend
from benchmarks.f_evaluators import FEvaluatorBenchmarkVariant, benchmark_f_evaluators
from benchmarks.optimization import (
    STANDARD_OPTIMIZATION_CASES,
    OptimizerBenchmarkVariant,
    benchmark_optimizer_backends,
    print_optimizer_benchmark,
)
from benchmarks.planning_rrt import benchmark_nearest_backend, holonomic_problem
from benchmarks.simulation import (
    TRUTH_SIMULATION_VARIANT,
    SimulationBenchmarkVariant,
    benchmark_simulation_backend,
    benchmark_simulation_matrix,
)
from benchmarks.trajopt import (
    TrajectoryOptimizationBenchmarkConfig,
    TrajectoryOptimizationBenchmarkVariant,
    benchmark_trajectory_optimization,
)
from minilink.core.system import DynamicSystem
from minilink.planning.search.rrt import RRTPlanner


class _TinyStable(DynamicSystem):
    def __init__(self):
        super().__init__(n=1, input_dim=1, output_dim=1, y_dependencies=())
        self.name = "TinyStable"
        self.x0 = np.array([1.0])

    def f(self, x, u, t=0, params=None):
        return np.asarray([-x[0] + u[0]], dtype=float)

    def h(self, x, u, t=0, params=None):
        return x


class TestBenchmarkSmoke(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls._sys = _TinyStable()
        cls._tgrid = dict(t0=0.0, tf=0.1, dt=0.01)

    def test_f_evaluator_benchmark_returns_rows(self):
        result = benchmark_f_evaluators(
            self._sys,
            np.array([1.0]),
            np.array([0.0]),
            n_calls=2,
            variants=(
                FEvaluatorBenchmarkVariant("native", None),
                FEvaluatorBenchmarkVariant("numpy", "numpy"),
            ),
        )
        self.assertEqual(len(result.rows), 2)
        self.assertGreaterEqual(result.rows[1].loop_s, 0.0)

    def test_trajopt_benchmark_returns_row(self):
        config = TrajectoryOptimizationBenchmarkConfig(n_steps=3, maxiter=1, n_runs=1)
        variant = TrajectoryOptimizationBenchmarkVariant(
            name="unit-numpy-collocation",
            transcription="collocation",
            compile_backend="numpy",
            derivative="finite-diff",
            start="cold",
        )
        result = benchmark_trajectory_optimization(config, (variant,))
        self.assertEqual(len(result.rows), 1)

    def test_optimizer_benchmark_runs_standard_cases(self):
        variants = (
            OptimizerBenchmarkVariant(
                name="scipy-SLSQP",
                method="scipy_slsqp",
                options={"maxiter": 200, "ftol": 1e-9, "disp": False},
            ),
        )
        result = benchmark_optimizer_backends(STANDARD_OPTIMIZATION_CASES, variants)
        self.assertEqual(len(result.rows), len(STANDARD_OPTIMIZATION_CASES))
        buf = io.StringIO()
        with redirect_stdout(buf):
            print_optimizer_benchmark(result)
        self.assertIn("optimizer-backend benchmark", buf.getvalue())

    def test_simulation_benchmark_compile_once_split(self):
        result = benchmark_simulation_backend(
            self._sys,
            candidate=SimulationBenchmarkVariant("euler", "numpy"),
            truth=TRUTH_SIMULATION_VARIANT,
            n_runs=2,
            **self._tgrid,
        )
        self.assertEqual(result.mean_time, result.mean_solve_time)
        self.assertGreater(result.mean_solve_time, 0.0)

        matrix = benchmark_simulation_matrix(
            self._sys,
            case_name="unit",
            variants=(
                SimulationBenchmarkVariant("euler", "numpy"),
                TRUTH_SIMULATION_VARIANT,
            ),
            n_runs=1,
            **self._tgrid,
        )
        self.assertTrue(matrix.compile_once)
        self.assertEqual(len(matrix.rows), 2)

    def test_dp_benchmark_backend_returns_row(self):
        row = benchmark_backend("loop", (5, 5), (3,), n_steps=2, runs=1)
        self.assertEqual(row.backend, "loop")
        self.assertGreater(row.build_s, 0.0)
        self.assertEqual(row.iterations, 2)

    def test_planning_rrt_fixture_and_benchmark_row(self):
        problem, extender, x_goal = holonomic_problem()
        self.assertEqual(problem.sys.n, 2)
        self.assertIsNotNone(extender)
        self.assertEqual(x_goal.shape, (2,))
        row = benchmark_nearest_backend(RRTPlanner, "brute_force", seed=0)
        self.assertEqual(row.planner, "rrt")
        self.assertEqual(row.backend, "brute_force")
        self.assertGreater(row.elapsed_s, 0.0)


if __name__ == "__main__":
    unittest.main()
