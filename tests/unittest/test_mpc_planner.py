"""Optional beta MPC planner tests."""

import unittest

import numpy as np
import pytest

pytest.importorskip("jax")

import jax.numpy as jnp  # noqa: E402

from minilink.core.backends import configure_jax  # noqa: E402
from minilink.core.costs import QuadraticCost  # noqa: E402
from minilink.core.system import DynamicSystem  # noqa: E402
from minilink.planning.mpc import (  # noqa: E402
    MPCDirectCollocationTranscription,
    MPCOptions,
    MPCPlanner,
)
from minilink.planning.problems import PlanningProblem  # noqa: E402
from minilink.planning.trajectory_optimization.direct_collocation import (  # noqa: E402
    DirectCollocationOptions,
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
class TestMPCPlanner(unittest.TestCase):
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
            x_start=np.array([x_start]),
            cost=cost,
        )

    def make_planner(self, problem):
        transcription = MPCDirectCollocationTranscription(
            DirectCollocationOptions(tf=1.0, n_steps=5)
        )
        return MPCPlanner(
            problem,
            transcription=transcription,
            options=MPCOptions(
                optimizer_options={"maxiter": 50, "ftol": 1e-4},
            ),
        )

    def test_step_smoke(self):
        planner = self.make_planner(self.make_problem(0.0))
        traj = planner.step(np.array([0.0]))
        self.assertEqual(traj.x.shape, (1, 5))
        self.assertEqual(traj.u.shape, (1, 5))
        self.assertIsNotNone(planner.last_optimization_result)

        traj2 = planner.step(np.array([0.2]))
        self.assertEqual(traj2.x.shape, (1, 5))

    def test_compile_once_per_planner(self):
        planner = self.make_planner(self.make_problem(0.0))
        compile_s = planner.compile_time_s
        self.assertIsNotNone(compile_s)
        self.assertGreater(compile_s, 0.0)

        planner.step(np.array([0.0]))
        first_step_s = planner.last_step_time_s
        planner.step(np.array([0.15]))
        second_step_s = planner.last_step_time_s

        self.assertIsNotNone(first_step_s)
        self.assertIsNotNone(second_step_s)
        self.assertLess(second_step_s, first_step_s)
        self.assertLess(second_step_s, 0.5 * compile_s)

    def test_initial_boundary_satisfied(self):
        x_start = np.array([0.35])
        planner = self.make_planner(self.make_problem(float(x_start[0])))
        traj = planner.step(x_start)
        np.testing.assert_allclose(traj.x[:, 0], x_start, atol=1e-5)

    def test_compute_solution_alias(self):
        problem = self.make_problem(0.1)
        planner = self.make_planner(problem)
        traj = planner.compute_solution()
        np.testing.assert_allclose(traj.x[:, 0], problem.x_start, atol=1e-5)
