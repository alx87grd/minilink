"""Tests for MPC :meth:`export_to_computer`."""

import unittest

import pytest

pytest.importorskip("jax")

from minilink.core.backends import configure_jax  # noqa: E402
from minilink.core.costs import QuadraticCost  # noqa: E402
from minilink.dynamics.catalog.vehicles.dynamic_bicycle import (  # noqa: E402
    JaxDynamicBicycleRateInputsUY,
)
from minilink.planning.mpc import ModelPredictiveController
from minilink.planning.problems import PlanningProblem  # noqa: E402
from minilink.planning.trajectory_optimization.direct_collocation import (
    DirectCollocationOptions,
    DirectCollocationTranscription,
)
from minilink.planning.trajectory_optimization.planner import (
    TrajectoryOptimizationOptions,
    TrajectoryOptimizationPlanner,
)
from minilink.simulation.computer import Computer  # noqa: E402


def _planner():
    configure_jax(enable_x64=True)
    sys = JaxDynamicBicycleRateInputsUY()
    x0 = sys.x0.copy()
    return TrajectoryOptimizationPlanner(
        PlanningProblem(
            sys=sys,
            tf=1.0,
            x_start=x0,
            cost=QuadraticCost.from_system(sys, xbar=x0),
        ),
        transcription=DirectCollocationTranscription(
            DirectCollocationOptions(n_steps=5)
        ),
        options=TrajectoryOptimizationOptions(
            compile_backend="jax",
            record_solve_time=True,
            optimizer_method="scipy_slsqp",
            optimizer_options={"maxiter": 5, "ftol": 1e-1},
        ),
    )


class TestMpcExportComputer(unittest.TestCase):
    def test_step_block_defaults_schedule(self):
        planner = _planner()
        mpc = ModelPredictiveController(planner, dt_mpc=0.2, warm_start=True)
        computer = mpc.export_to_computer()
        self.assertIsInstance(computer, Computer)
        self.assertAlmostEqual(computer.schedule.dt_base, 0.2)

    def test_step_block_rejects_mismatched_schedule(self):
        planner = _planner()
        mpc = ModelPredictiveController(planner, dt_mpc=0.2, warm_start=True)
        with self.assertRaises(ValueError):
            mpc.export_to_computer(0.1)

    def test_algebraic_defaults_schedule_from_dt_mpc(self):
        planner = _planner()
        mpc = ModelPredictiveController(planner, dt_mpc=0.2, warm_start=False)
        computer = mpc.export_to_computer()
        self.assertAlmostEqual(computer.schedule.dt_base, 0.2)
        with self.assertRaises(ValueError):
            mpc.export_to_computer(0.1)

    def test_dual_rate_computer_schedule_and_u_nom(self):
        planner = _planner()
        mpc = ModelPredictiveController(planner, dt_mpc=0.2, warm_start=True)
        computer = mpc.dual_rate_computer(dt_broadcast=0.05)
        self.assertIsInstance(computer, Computer)
        self.assertAlmostEqual(computer.schedule.dt_base, 0.05)
        self.assertEqual(computer.schedule.fire["replan"], 4)
        self.assertEqual(computer.schedule.fire["broadcast"], 1)
        self.assertIn("u_nom", computer.diagram.outputs)
        self.assertIn("y", computer.diagram.inputs)
        with self.assertRaises(ValueError):
            mpc.dual_rate_computer(dt_broadcast=0.07)

    def test_dual_rate_matmul_hybrid(self):
        from minilink.core.hybrid_diagram import HybridDiagram

        planner = _planner()
        plant = planner.problem.sys
        mpc = ModelPredictiveController(planner, dt_mpc=0.2, warm_start=True)
        computer = mpc.dual_rate_computer(dt_broadcast=0.1)
        hybrid = computer @ plant
        self.assertIsInstance(hybrid, HybridDiagram)


if __name__ == "__main__":
    unittest.main()
