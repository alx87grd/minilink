"""Tests for MPC :meth:`export_to_computer`."""

import unittest

import pytest

pytest.importorskip("jax")

from minilink.core.backends import configure_jax  # noqa: E402
from minilink.core.costs import QuadraticCost  # noqa: E402
from minilink.dynamics.catalog.vehicles.dynamic_bicycle import (  # noqa: E402
    JaxDynamicBicycleRateInputsUY,
)
from minilink.planning.mpc import (
    mpc_stateful_controller,
    mpc_stateless_controller,
)
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
        mpc = mpc_stateful_controller(planner, dt_mpc=0.2)
        computer = mpc.export_to_computer()
        self.assertIsInstance(computer, Computer)
        self.assertAlmostEqual(computer.schedule.dt_base, 0.2)

    def test_step_block_rejects_mismatched_schedule(self):
        planner = _planner()
        mpc = mpc_stateful_controller(planner, dt_mpc=0.2)
        with self.assertRaises(ValueError):
            mpc.export_to_computer(0.1)

    def test_algebraic_defaults_schedule_from_dt_mpc(self):
        planner = _planner()
        mpc = mpc_stateless_controller(planner, dt_mpc=0.2)
        computer = mpc.export_to_computer()
        self.assertAlmostEqual(computer.schedule.dt_base, 0.2)
        with self.assertRaises(ValueError):
            mpc.export_to_computer(0.1)


if __name__ == "__main__":
    unittest.main()
