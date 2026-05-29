"""Optional JAX direct-collocation smoke tests."""

import unittest

import numpy as np
import pytest

pytest.importorskip("jax")

import jax  # noqa: E402
import jax.numpy as jnp  # noqa: E402

from minilink.compile.jax_utils import configure_jax  # noqa: E402
from minilink.core.costs import QuadraticCost  # noqa: E402
from minilink.core.sets import BallSet  # noqa: E402
from minilink.core.system import DynamicSystem  # noqa: E402
from minilink.dynamics.catalog.pendulum.cartpole import (  # noqa: E402
    CartPole,
    JaxCartPole,
)
from minilink.optimization.evaluators.compiler import (  # noqa: E402
    compile_program_evaluator,
)
from minilink.planning.initial_guess import default_initial_trajectory  # noqa: E402
from minilink.planning.problems import PlanningProblem  # noqa: E402
from minilink.planning.trajectory_optimization.direct_collocation import (  # noqa: E402
    DirectCollocationOptions,
    DirectCollocationTranscription,
)
from minilink.planning.trajectory_optimization.multiple_shooting import (  # noqa: E402
    MultipleShootingOptions,
    MultipleShootingTranscription,
)
from minilink.planning.trajectory_optimization.planner import (  # noqa: E402
    TrajectoryOptimizationOptions,
    TrajectoryOptimizationPlanner,
)
from minilink.planning.trajectory_optimization.shooting import (  # noqa: E402
    ShootingOptions,
    ShootingTranscription,
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
class TestJaxDirectCollocation(unittest.TestCase):
    def setUp(self):
        configure_jax(enable_x64=True)

    def make_single_integrator_problem(self, cost_cls=QuadraticCost):
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
            transcription=DirectCollocationTranscription(
                DirectCollocationOptions(tf=1.0, n_steps=5)
            ),
            options=TrajectoryOptimizationOptions(
                compile_backend="jax",
                optimizer_options={"maxiter": 100, "ftol": 1e-9},
            ),
        )

        guess = default_initial_trajectory(
            problem,
            planner.transcription.initial_guess_time_grid(problem),
        )
        program = planner.transcription.transcribe(
            problem,
            compile_backend="jax",
        )
        z0 = planner.transcription.pack_initial_guess(problem, guess)
        program_evaluator = compile_program_evaluator(
            program,
            backend=program.metadata["program_backend"],
            sample_z=z0,
        )
        self.assertTrue(program_evaluator.has_gradient)
        self.assertTrue(program_evaluator.has_jacobian_h)

        traj = planner.compute_solution()

        self.assertTrue(planner.last_optimization_result.success)
        np.testing.assert_allclose(traj.x[:, 0], [0.0], atol=1e-7)
        np.testing.assert_allclose(traj.x[:, -1], [1.0], atol=1e-7)

    def test_jax_shooting_solves_single_integrator(self):
        problem = self.make_single_integrator_problem()
        planner = TrajectoryOptimizationPlanner(
            problem,
            transcription=ShootingTranscription(
                ShootingOptions(tf=1.0, n_steps=5)
            ),
            options=TrajectoryOptimizationOptions(
                compile_backend="jax",
                optimizer_options={"maxiter": 100, "ftol": 1e-9},
            ),
        )

        guess = default_initial_trajectory(
            problem,
            planner.transcription.initial_guess_time_grid(problem),
        )
        program = planner.transcription.transcribe(
            problem,
            compile_backend="jax",
        )
        z0 = planner.transcription.pack_initial_guess(problem, guess)
        program_evaluator = compile_program_evaluator(
            program,
            backend=program.metadata["program_backend"],
            sample_z=z0,
        )
        self.assertEqual(program.n_z, problem.sys.m * 5)
        self.assertTrue(program_evaluator.has_gradient)
        self.assertTrue(program_evaluator.has_jacobian_h)
        self.assertTrue(program_evaluator.has_jacobian_g)

        traj = planner.compute_solution()

        self.assertTrue(planner.last_optimization_result.success)
        np.testing.assert_allclose(traj.x[:, 0], [0.0], atol=1e-7)
        np.testing.assert_allclose(traj.x[:, -1], [1.0], atol=1e-7)

    def test_jax_multiple_shooting_solves_single_integrator(self):
        problem = self.make_single_integrator_problem()
        planner = TrajectoryOptimizationPlanner(
            problem,
            transcription=MultipleShootingTranscription(
                MultipleShootingOptions(tf=1.0, n_steps=5)
            ),
            options=TrajectoryOptimizationOptions(
                compile_backend="jax",
                optimizer_options={"maxiter": 100, "ftol": 1e-9},
            ),
        )

        guess = default_initial_trajectory(
            problem,
            planner.transcription.initial_guess_time_grid(problem),
        )
        program = planner.transcription.transcribe(
            problem,
            compile_backend="jax",
        )
        z0 = planner.transcription.pack_initial_guess(problem, guess)
        program_evaluator = compile_program_evaluator(
            program,
            backend=program.metadata["program_backend"],
            sample_z=z0,
        )
        self.assertEqual(program.n_z, (problem.sys.n + problem.sys.m) * 5)
        self.assertTrue(program_evaluator.has_gradient)
        self.assertTrue(program_evaluator.has_jacobian_h)

        traj = planner.compute_solution()

        self.assertTrue(planner.last_optimization_result.success)
        np.testing.assert_allclose(traj.x[:, 0], [0.0], atol=1e-7)
        np.testing.assert_allclose(traj.x[:, -1], [1.0], atol=1e-7)

    def test_jax_direct_collocation_accepts_traceable_terminal_set(self):
        base = self.make_single_integrator_problem()
        problem = PlanningProblem(
            sys=base.sys,
            x_start=np.array([0.0]),
            Xf=BallSet(center=np.array([1.0]), radius=0.1),
            cost=base.cost,
        )
        tr = DirectCollocationTranscription(
            DirectCollocationOptions(tf=1.0, n_steps=5)
        )
        guess = default_initial_trajectory(problem, tr.initial_guess_time_grid(problem))
        program = tr.transcribe(problem, compile_backend="jax")
        z0 = tr.pack_initial_guess(problem, guess)
        program_evaluator = compile_program_evaluator(
            program,
            backend=program.metadata["program_backend"],
            sample_z=z0,
        )

        self.assertEqual(program_evaluator.n_g, 1)
        self.assertTrue(program_evaluator.has_jacobian_g)

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
