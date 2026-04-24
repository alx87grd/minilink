"""Unit tests for :mod:`minilink.benchmark.simulation_speed` compile/solve split and ``compile_once``."""

import contextlib
import io
import unittest

import numpy as np

from minilink.benchmark import (
    TRUTH_SOLVER,
    TRUTH_BACKEND,
    benchmark_sim_backend,
    benchmark_sim_speed_matrix,
    format_benchmark_backend_label,
)
from minilink.core.framework import DynamicSystem


class _TinyStable(DynamicSystem):
    """1-state linear system for fast benchmark smoke tests."""

    def __init__(self):
        super().__init__(1, 1, 1)
        self.name = "TinyStable"
        self.x0 = np.array([1.0])

    def f(self, x, u, t=0, params=None):
        return np.asarray([-x[0] + u[0]], dtype=float)

    def h(self, x, u, t=0, params=None):
        return x


class TestSimulationSpeedBenchmark(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls._sys = _TinyStable()
        cls._tgrid = dict(t0=0.0, tf=0.1, dt=0.01)

    def test_format_benchmark_backend_label_numpy_unchanged(self):
        self.assertEqual(format_benchmark_backend_label("numpy"), "numpy")

    def test_format_benchmark_backend_label_jax_shape(self):
        label = format_benchmark_backend_label("jax")
        try:
            import jax  # noqa: F401
        except ImportError:
            self.assertEqual(label, "jax")
        else:
            self.assertRegex(label, r"^jax\([a-z0-9_]+\)$")

    def test_compile_each_run_total_exceeds_split_parts(self):
        r = benchmark_sim_backend(
            self._sys,
            candidate_solver="euler",
            candidate_backend="numpy",
            truth_solver=TRUTH_SOLVER,
            truth_backend=TRUTH_BACKEND,
            n_runs=2,
            compile_once=False,
            **self._tgrid,
        )
        # ``mean_time`` is full per-iteration wall; split is ``_build_evaluator`` + ``solve`` only.
        self.assertGreaterEqual(
            r.mean_time + 1e-9,
            r.mean_compile_time + r.mean_solve_time,
        )

    def test_compile_once_mean_time_is_solve_only(self):
        r = benchmark_sim_backend(
            self._sys,
            candidate_solver="euler",
            candidate_backend="numpy",
            truth_solver=TRUTH_SOLVER,
            truth_backend=TRUTH_BACKEND,
            n_runs=2,
            **self._tgrid,
        )
        self.assertEqual(r.mean_time, r.mean_solve_time)
        self.assertEqual(r.truth_mean_time, r.truth_mean_solve_time)
        self.assertGreaterEqual(r.mean_compile_time, 0.0)
        self.assertGreater(r.mean_solve_time, 0.0)

    def test_matrix_reuses_truth_row_and_has_compile_solve(self):
        pairs = (("euler", "numpy"), (TRUTH_SOLVER, TRUTH_BACKEND))
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            m = benchmark_sim_speed_matrix(
                self._sys,
                case_name="unit",
                pairs=pairs,
                n_runs=1,
                **self._tgrid,
            )
        r0, r1 = m.rows[0], m.rows[1]
        # Truth pair is listed first when present in ``pairs``.
        self.assertEqual((r0.solver, r0.backend), (TRUTH_SOLVER, TRUTH_BACKEND))
        self.assertEqual(
            (r0.mean_time, r0.mean_compile_time, r0.mean_solve_time),
            (
                m.truth_mean_time,
                m.truth_mean_compile_time,
                m.truth_mean_solve_time,
            ),
        )
        self.assertEqual((r1.solver, r1.backend), ("euler", "numpy"))
        self.assertTrue(m.compile_once)
        self.assertIsInstance(r0.mean_compile_time, float)
        self.assertIsInstance(r0.mean_solve_time, float)


if __name__ == "__main__":
    unittest.main()
