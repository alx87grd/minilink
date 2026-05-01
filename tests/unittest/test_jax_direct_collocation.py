"""Optional JAX direct-collocation smoke tests."""

import unittest

import numpy as np
import pytest

pytest.importorskip("jax")

import jax  # noqa: E402
import jax.numpy as jnp  # noqa: E402

from minilink.compile.jax_utils import configure_jax  # noqa: E402
from minilink.core.costs import JaxQuadraticCost, QuadraticCost  # noqa: E402
from minilink.core.system import DynamicSystem  # noqa: E402
from minilink.dynamics.catalog.pendulum.cartpole import (  # noqa: E402
    CartPole,
    JaxCartPole,
)
from minilink.optimization.optimizers.scipy_minimize import (
    ScipyMinimizeOptimizer,  # noqa: E402
)
from minilink.planning.initial_guess import default_initial_trajectory  # noqa: E402
from minilink.planning.problems import PlanningProblem  # noqa: E402
from minilink.planning.trajectory_optimization.jax_direct_collocation import (  # noqa: E402
    JaxDirectCollocationOptions,
    JaxDirectCollocationTranscription,
)
from minilink.planning.trajectory_optimization.jax_multiple_shooting import (  # noqa: E402
    JaxMultipleShootingOptions,
    JaxMultipleShootingTranscription,
)
from minilink.planning.trajectory_optimization.jax_shooting import (  # noqa: E402
    JaxShootingOptions,
    JaxShootingTranscription,
)
from minilink.planning.trajectory_optimization.planner import (  # noqa: E402
    TrajectoryOptimizationOptions,
    TrajectoryOptimizationPlanner,
)


class JaxSingleIntegrator(DynamicSystem):
    def __init__(self):
        super().__init__(n=1, m=1, p=1)
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
class TestJaxDirectCollocation(unittest.TestCase):
    def setUp(self):
        configure_jax(enable_x64=True)

    def make_single_integrator_problem(self, cost_cls=JaxQuadraticCost):
        sys = JaxSingleIntegrator()
        cost = cost_cls.from_system(
            sys,
            Q=np.zeros((1, 1)),
            R=np.eye(1),
            S=np.zeros((1, 1)),
        )
        return PlanningProblem(
            sys=sys,
            x_start=np.array([0.0]),
            x_goal=np.array([1.0]),
            cost=cost,
        )

    def test_jax_cartpole_matches_numpy_dynamics(self):
        sys_np = CartPole()
        sys_jax = JaxCartPole()
        x = np.array([-0.3, 1.2, 0.4, -0.5])
        u = np.array([2.0])

        dx_np = sys_np.f(x, u)
        dx_jax = sys_jax.f(jnp.asarray(x), jnp.asarray(u))

        np.testing.assert_allclose(np.asarray(dx_jax), dx_np, rtol=1e-5, atol=1e-5)
        jax.make_jaxpr(lambda xx, uu: sys_jax.f(xx, uu))(
            jnp.asarray(x),
            jnp.asarray(u),
        )

    def test_jax_direct_collocation_solves_single_integrator(self):
        problem = self.make_single_integrator_problem()
        planner = TrajectoryOptimizationPlanner(
            problem,
            transcription=JaxDirectCollocationTranscription(
                JaxDirectCollocationOptions(tf=1.0, n_steps=5)
            ),
            optimizer=ScipyMinimizeOptimizer(options={"maxiter": 100, "ftol": 1e-9}),
            options=TrajectoryOptimizationOptions(compile_backend="jax"),
        )

        guess = default_initial_trajectory(
            problem,
            planner.transcription.initial_guess_time_grid(problem),
        )
        program = planner.transcription.transcribe(
            problem,
            initial_guess=guess,
            compile_backend="jax",
        )
        self.assertIsNotNone(program.grad)
        self.assertIsNotNone(program.equalities[0].jac)

        traj = planner.compute_solution()

        self.assertTrue(planner.last_optimization_result.success)
        np.testing.assert_allclose(traj.x[:, 0], [0.0], atol=1e-7)
        np.testing.assert_allclose(traj.x[:, -1], [1.0], atol=1e-7)

    def test_jax_shooting_solves_single_integrator(self):
        problem = self.make_single_integrator_problem()
        planner = TrajectoryOptimizationPlanner(
            problem,
            transcription=JaxShootingTranscription(
                JaxShootingOptions(tf=1.0, n_steps=5)
            ),
            optimizer=ScipyMinimizeOptimizer(options={"maxiter": 100, "ftol": 1e-9}),
            options=TrajectoryOptimizationOptions(compile_backend="jax"),
        )

        guess = default_initial_trajectory(
            problem,
            planner.transcription.initial_guess_time_grid(problem),
        )
        program = planner.transcription.transcribe(
            problem,
            initial_guess=guess,
            compile_backend="jax",
        )
        self.assertEqual(program.n_z, problem.sys.m * 5)
        self.assertIsNotNone(program.grad)
        self.assertIsNotNone(program.equalities[0].jac)
        self.assertIsNotNone(program.inequalities[0].jac)

        traj = planner.compute_solution()

        self.assertTrue(planner.last_optimization_result.success)
        np.testing.assert_allclose(traj.x[:, 0], [0.0], atol=1e-7)
        np.testing.assert_allclose(traj.x[:, -1], [1.0], atol=1e-7)

    def test_jax_multiple_shooting_solves_single_integrator(self):
        problem = self.make_single_integrator_problem()
        planner = TrajectoryOptimizationPlanner(
            problem,
            transcription=JaxMultipleShootingTranscription(
                JaxMultipleShootingOptions(tf=1.0, n_steps=5)
            ),
            optimizer=ScipyMinimizeOptimizer(options={"maxiter": 100, "ftol": 1e-9}),
            options=TrajectoryOptimizationOptions(compile_backend="jax"),
        )

        guess = default_initial_trajectory(
            problem,
            planner.transcription.initial_guess_time_grid(problem),
        )
        program = planner.transcription.transcribe(
            problem,
            initial_guess=guess,
            compile_backend="jax",
        )
        self.assertEqual(program.n_z, (problem.sys.n + problem.sys.m) * 5)
        self.assertIsNotNone(program.grad)
        self.assertIsNotNone(program.equalities[0].jac)

        traj = planner.compute_solution()

        self.assertTrue(planner.last_optimization_result.success)
        np.testing.assert_allclose(traj.x[:, 0], [0.0], atol=1e-7)
        np.testing.assert_allclose(traj.x[:, -1], [1.0], atol=1e-7)

    def test_rejects_numpy_quadratic_cost(self):
        problem = self.make_single_integrator_problem(QuadraticCost)
        opts = JaxDirectCollocationOptions(
            tf=1.0,
            n_steps=5,
        )
        tr = JaxDirectCollocationTranscription(opts)
        guess = default_initial_trajectory(
            problem,
            tr.initial_guess_time_grid(problem),
        )
        with self.assertRaisesRegex(ValueError, "JaxQuadraticCost"):
            tr.transcribe(
                problem,
                initial_guess=guess,
                compile_backend="direct",
            )

    def test_jax_evaluator_forced_rk4_rollout_smoke(self):
        sys = JaxSingleIntegrator()
        evaluator = sys.compile(backend="jax", verbose=False)

        x = evaluator.rk4_rollout_forced(
            np.array([0.0]),
            np.ones((5, 1)),
            0.0,
            0.25,
        )

        np.testing.assert_allclose(np.asarray(x).reshape(-1), np.linspace(0.0, 1.0, 5))


if __name__ == "__main__":
    unittest.main()
