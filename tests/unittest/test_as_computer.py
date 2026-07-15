"""Tests for :func:`~minilink.simulation.computer.as_computer` and ``%``."""

import unittest

from minilink.blocks.basic import Integrator
from minilink.control.output import ProportionalController
from minilink.core.diagram import StepDiagramSystem
from minilink.simulation.computer import Computer, StepSchedule, as_computer


def _build_step_diagram():
    diagram = StepDiagramSystem()
    diagram.add_subsystem(ProportionalController(0.5), "ctl")
    diagram.add_input_port("r")
    diagram.add_input_port("y")
    diagram.connect("input", "r", "ctl", "r")
    diagram.connect("input", "y", "ctl", "y")
    diagram.connect_new_output_port("ctl", "u", "u")
    return diagram


class TestAsComputer(unittest.TestCase):
    def test_leaf_mod_float(self):
        computer = ProportionalController(0.3) % 0.02
        self.assertIsInstance(computer, Computer)
        self.assertAlmostEqual(computer.schedule.dt_base, 0.02)
        self.assertIn("ctl", computer.diagram.subsystems)

    def test_step_diagram_as_computer(self):
        diagram = _build_step_diagram()
        computer = as_computer(diagram, StepSchedule(dt_base=0.01))
        self.assertIs(diagram, computer.diagram)

    def test_rejects_continuous_plant(self):
        with self.assertRaises(TypeError):
            Integrator() % 0.01

    def test_mpc_mod_exposes_u_ff(self):
        pytest = __import__("pytest")
        jax = pytest.importorskip("jax")
        del jax

        from minilink.core.backends import configure_jax
        from minilink.core.costs import QuadraticCost
        from minilink.dynamics.catalog.vehicles.dynamic_bicycle import (
            JaxDynamicBicycleRateInputsUY,
        )
        from minilink.planning.mpc import ModelPredictiveController
        from minilink.planning.problems import PlanningProblem
        from minilink.planning.trajectory_optimization.direct_collocation import (
            DirectCollocationOptions,
            DirectCollocationTranscription,
        )
        from minilink.planning.trajectory_optimization.planner import (
            TrajectoryOptimizationOptions,
            TrajectoryOptimizationPlanner,
        )

        configure_jax(enable_x64=True)
        sys = JaxDynamicBicycleRateInputsUY()
        x0 = sys.x0.copy()
        planner = TrajectoryOptimizationPlanner(
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
        computer = (
            ModelPredictiveController(planner, dt_mpc=0.2, warm_start=False) % 0.2
        )
        self.assertIn("y", computer.diagram.inputs)
        self.assertIn("u_ff", computer.diagram.outputs)


if __name__ == "__main__":
    unittest.main()
