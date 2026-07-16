"""Unit tests for TrajectoryOptimizationPlanner.solve_trajectory_from."""

import unittest

import numpy as np
import pytest

pytest.importorskip("jax")

import jax.numpy as jnp  # noqa: E402

from minilink.core.backends import configure_jax  # noqa: E402
from minilink.core.costs import QuadraticCost  # noqa: E402
from minilink.core.system import DynamicSystem  # noqa: E402
from minilink.planning.initial_guess import default_initial_trajectory  # noqa: E402
from minilink.planning.problems import PlanningProblem  # noqa: E402
from minilink.planning.trajectory_optimization.direct_collocation import (  # noqa: E402
    DirectCollocationOptions,
    DirectCollocationTranscription,
)
from minilink.planning.trajectory_optimization.planner import (  # noqa: E402
    TrajectoryOptimizationOptions,
    TrajectoryOptimizationPlanner,
)


class JaxSingleIntegrator(DynamicSystem):
    def __init__(self):
        super().__init__(n=1, input_dim=1, output_dim=1, y_dependencies=())
        self.state.lower_bound = np.array([-10.0])
        self.state.upper_bound = np.array([10.0])
        self.inputs["u"].lower_bound = np.array([-10.0])
        self.inputs["u"].upper_bound = np.array([10.0])

    def f(self, x, u, t=0, params=None):
        return jnp.array([u[0]])

    def h(self, x, u, t=0, params=None):
        return x


@pytest.mark.optional
@pytest.mark.jax
class TestMPCSolveTrajectoryFrom(unittest.TestCase):
    def setUp(self):
        configure_jax(enable_x64=True)

    def make_problem(self, x_start=0.0):
        sys = JaxSingleIntegrator()
        cost = QuadraticCost.from_system(
            sys,
            Q=np.zeros((1, 1)),
            R=np.eye(1),
            S=np.zeros((1, 1)),
        )
        return PlanningProblem(
            sys=sys,
            tf=1.0,
            x_start=np.array([x_start]),
            cost=cost,
        )

    def make_planner(self, problem):
        transcription = DirectCollocationTranscription(
            DirectCollocationOptions(n_steps=5)
        )
        planner = TrajectoryOptimizationPlanner(
            problem,
            transcription=transcription,
            options=TrajectoryOptimizationOptions(
                compile_backend="jax",
                record_solve_time=True,
                optimizer_options={"maxiter": 50, "ftol": 1e-4},
            ),
        )
        planner.compile_parametric_program()
        return planner

    def test_parity_with_step(self):
        x0 = np.array([0.2])
        planner_step = self.make_planner(self.make_problem(0.0))
        traj = planner_step.step(x0)

        planner_from = self.make_planner(self.make_problem(0.0))
        plan = planner_from.solve_trajectory_from(x0)
        np.testing.assert_allclose(plan.trajectory.x, traj.x, atol=1e-8)
        np.testing.assert_allclose(plan.trajectory.u, traj.u, atol=1e-8)
        np.testing.assert_allclose(
            plan.warm_state,
            planner_from.last_optimization_result.z,
            atol=1e-8,
        )
        self.assertIs(planner_from.last_trajectory_plan, plan)

    def test_metadata_present(self):
        planner = self.make_planner(self.make_problem(0.1))
        plan = planner.solve_trajectory_from(np.array([0.1]))
        self.assertTrue(hasattr(plan.metadata, "success"))
        self.assertIsInstance(plan.metadata.success, bool)
        self.assertIsNotNone(plan.metadata.message)
        self.assertIsNotNone(plan.warm_state)

    def test_params_none_and_empty_ok(self):
        planner = self.make_planner(self.make_problem(0.0))
        x0 = np.array([0.0])
        plan_none = planner.solve_trajectory_from(x0, params=None)
        plan_empty = planner.solve_trajectory_from(x0, params={})
        np.testing.assert_allclose(plan_none.trajectory.x[:, 0], x0, atol=1e-5)
        np.testing.assert_allclose(plan_empty.trajectory.x[:, 0], x0, atol=1e-5)

    def test_params_scene_not_implemented(self):
        planner = self.make_planner(self.make_problem(0.0))
        with self.assertRaises(NotImplementedError):
            planner.solve_trajectory_from(np.array([0.0]), params={"scene": {}})

    def test_params_unknown_keys_rejected(self):
        planner = self.make_planner(self.make_problem(0.0))
        with self.assertRaises(ValueError):
            planner.solve_trajectory_from(np.array([0.0]), params={"foo": 1})

    def test_solve_trajectory_rebuild_and_from_compiled(self):
        """Offline rebuild and compiled from-solve both honor ``x_start``."""
        problem = self.make_problem(0.15)
        planner_from = self.make_planner(problem)
        plan_from = planner_from.solve_trajectory_from(problem.x_start)
        np.testing.assert_allclose(
            plan_from.trajectory.x[:, 0], problem.x_start, atol=1e-5
        )

        planner_off = TrajectoryOptimizationPlanner(
            problem,
            transcription=DirectCollocationTranscription(
                DirectCollocationOptions(n_steps=5)
            ),
            options=TrajectoryOptimizationOptions(
                compile_backend="jax",
                record_solve_time=True,
                optimizer_options={"maxiter": 50, "ftol": 1e-4},
            ),
        )
        plan_off = planner_off.solve_trajectory()
        np.testing.assert_allclose(
            plan_off.trajectory.x[:, 0], problem.x_start, atol=1e-5
        )

    def test_initial_guess_accepted(self):
        problem = self.make_problem(0.0)
        planner = self.make_planner(problem)
        guess = default_initial_trajectory(
            problem,
            planner.transcription.initial_guess_time_grid(problem),
        )
        plan = planner.solve_trajectory_from(np.array([0.05]), initial_guess=guess)
        np.testing.assert_allclose(plan.trajectory.x[:, 0], [0.05], atol=1e-5)
        self.assertIsNotNone(plan.warm_state)
