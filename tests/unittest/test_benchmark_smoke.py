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
from benchmarks.step_evaluators import (
    StepEvaluatorBenchmarkVariant,
    benchmark_step_evaluators,
)
from benchmarks.trajopt import (
    TrajectoryOptimizationBenchmarkConfig,
    TrajectoryOptimizationBenchmarkVariant,
    benchmark_trajectory_optimization,
)
from minilink.core.system import DynamicSystem, StepSystem
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


class _TinyStep(StepSystem):
    def __init__(self):
        super().__init__(n=1, input_dim=1, output_dim=1, y_dependencies=())
        self.name = "TinyStep"
        self.x0 = np.array([1.0])

    def step(self, x, u, k=0, params=None):
        return np.asarray([x[0] + u[0]], dtype=float)

    def h(self, x, u, k=0, params=None):
        return np.asarray(x, dtype=float)


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

    def test_step_evaluator_benchmark_returns_rows(self):
        step_sys = _TinyStep()
        result = benchmark_step_evaluators(
            step_sys,
            np.array([1.0]),
            np.array([0.0]),
            k=0,
            n_calls=2,
            variants=(
                StepEvaluatorBenchmarkVariant("native", None),
                StepEvaluatorBenchmarkVariant("numpy", "numpy"),
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
        self.assertEqual(result.candidate_x_final.shape, (1,))
        self.assertEqual(result.truth_x_final.shape, (1,))

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


class TestBenchmarkRegression(unittest.TestCase):
    def test_baseline_json_loads(self):
        from pathlib import Path

        from benchmarks.baseline import load_baseline

        fixture = (
            Path(__file__).resolve().parents[1]
            / "fixtures"
            / "benchmark_baseline"
            / "minimal_core_perf.json"
        )
        baseline = load_baseline(fixture)
        self.assertEqual(baseline.suite, "core_perf")
        self.assertEqual(len(baseline.metrics), 3)

    def test_compare_metrics_passes_when_equal(self):
        from benchmarks.baseline import BaselineFile, MetricRecord, compare_metrics

        baseline = BaselineFile(
            schema_version=1,
            suite="core_perf",
            description="",
            regression_factor=4.0,
            recorded_at="",
            host_hint="",
            metrics=(
                MetricRecord(
                    id="test.speed.ratio",
                    gate="speed",
                    direction="higher_better",
                    value=8.0,
                    unit="ratio",
                ),
            ),
        )
        recorded = [
            MetricRecord(
                id="test.speed.ratio",
                gate="speed",
                direction="higher_better",
                value=8.0,
                unit="ratio",
            )
        ]
        result = compare_metrics(recorded, baseline, factor=4.0)
        self.assertFalse(result.failed)

    def test_compare_metrics_fails_on_large_drop(self):
        from benchmarks.baseline import BaselineFile, MetricRecord, compare_metrics

        baseline = BaselineFile(
            schema_version=1,
            suite="core_perf",
            description="",
            regression_factor=4.0,
            recorded_at="",
            host_hint="",
            metrics=(
                MetricRecord(
                    id="test.speed.ratio",
                    gate="speed",
                    direction="higher_better",
                    value=10.0,
                    unit="ratio",
                ),
            ),
        )
        recorded = [
            MetricRecord(
                id="test.speed.ratio",
                gate="speed",
                direction="higher_better",
                value=1.0,
                unit="ratio",
            )
        ]
        result = compare_metrics(recorded, baseline, factor=4.0)
        self.assertTrue(result.failed)

    def test_compare_accuracy_fails_above_ceiling(self):
        from benchmarks.baseline import BaselineFile, MetricRecord, compare_metrics

        baseline = BaselineFile(
            schema_version=1,
            suite="core_perf",
            description="",
            regression_factor=4.0,
            recorded_at="",
            host_hint="",
            metrics=(
                MetricRecord(
                    id="test.accuracy.percent",
                    gate="accuracy",
                    direction="lower_better",
                    value=0.1,
                    max_allowed=1.0,
                    unit="percent",
                ),
            ),
        )
        recorded = [
            MetricRecord(
                id="test.accuracy.percent",
                gate="accuracy",
                direction="lower_better",
                value=2.0,
                max_allowed=1.0,
                unit="percent",
            )
        ]
        result = compare_metrics(recorded, baseline, factor=4.0)
        self.assertTrue(result.failed)

    def test_compare_vector_match_x_tf(self):
        from benchmarks.baseline import BaselineFile, MetricRecord, compare_metrics

        baseline = BaselineFile(
            schema_version=1,
            suite="core_perf",
            description="",
            regression_factor=4.0,
            recorded_at="",
            host_hint="",
            metrics=(
                MetricRecord(
                    id="test.truth.x_tf",
                    gate="accuracy",
                    direction="vector_match",
                    value=[1.0, 0.0],
                    atol=0.05,
                    rtol=0.05,
                    unit="state",
                ),
            ),
        )
        recorded = [
            MetricRecord(
                id="test.truth.x_tf",
                gate="accuracy",
                direction="vector_match",
                value=[1.0, 0.0],
                atol=0.05,
                rtol=0.05,
                unit="state",
            )
        ]
        result = compare_metrics(recorded, baseline, factor=4.0)
        self.assertFalse(result.failed)

    def test_core_perf_suite_returns_metrics(self):
        from benchmarks.suites.core_perf import CorePerfSuiteConfig, run_core_perf_suite

        metrics = run_core_perf_suite(
            CorePerfSuiteConfig(
                pendulum_n_calls=2,
                diagram_n_calls=2,
                sim_n_runs=1,
                static_n_steps=5,
            )
        )
        ids = {metric.id for metric in metrics}
        self.assertIn("diagram_dense_f.numpy.dx_residual", ids)
        self.assertIn("sim.diagram_dense.truth.x_tf", ids)


if __name__ == "__main__":
    unittest.main()
