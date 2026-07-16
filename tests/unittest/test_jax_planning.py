"""Optional JAX direct-collocation smoke tests."""

import unittest
import numpy as np
import pytest

pytest.importorskip("jax")
import jax
import jax.numpy as jnp
from minilink.core.backends import configure_jax
from minilink.core.costs import QuadraticCost
from minilink.core.sets import BallSet
from minilink.core.system import DynamicSystem
from minilink.dynamics.catalog.pendulum.cartpole import CartPole, JaxCartPole
from minilink.optimization.evaluators.compiler import compile_program_evaluator
from minilink.planning.initial_guess import default_initial_trajectory
from minilink.planning.problems import PlanningProblem
from minilink.planning.trajectory_optimization.direct_collocation import (
    DirectCollocationOptions,
    DirectCollocationTranscription,
)
from minilink.planning.trajectory_optimization.multiple_shooting import (
    MultipleShootingOptions,
    MultipleShootingTranscription,
)
from minilink.planning.trajectory_optimization.planner import (
    TrajectoryOptimizationOptions,
    TrajectoryOptimizationPlanner,
)
from minilink.planning.trajectory_optimization.shooting import (
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
            sys, Q=np.zeros((1, 1)), R=np.eye(1), S=np.zeros((1, 1))
        )
        return PlanningProblem(
            sys=sys, tf=1.0, x_start=np.array([0.0]), x_goal=np.array([1.0]), cost=cost
        )

    def test_jax_cartpole_matches_numpy_dynamics(self):
        sys_np = CartPole()
        sys_jax = JaxCartPole()
        x = np.array([-0.3, 1.2, 0.4, -0.5])
        u = np.array([2.0])
        dx_np = sys_np.f(x, u)
        dx_jax = sys_jax.f(jnp.asarray(x), jnp.asarray(u))
        np.testing.assert_allclose(np.asarray(dx_jax), dx_np, rtol=1e-05, atol=1e-05)
        jax.make_jaxpr(lambda xx, uu: sys_jax.f(xx, uu))(jnp.asarray(x), jnp.asarray(u))

    def test_jax_direct_collocation_solves_single_integrator(self):
        problem = self.make_single_integrator_problem()
        planner = TrajectoryOptimizationPlanner(
            problem,
            transcription=DirectCollocationTranscription(
                DirectCollocationOptions(n_steps=5)
            ),
            options=TrajectoryOptimizationOptions(
                compile_backend="jax", optimizer_options={"maxiter": 100, "ftol": 1e-09}
            ),
        )
        guess = default_initial_trajectory(
            problem, planner.transcription.initial_guess_time_grid(problem)
        )
        program = planner.transcription.transcribe(problem, compile_backend="jax")
        z0 = planner.transcription.pack_initial_guess(problem, guess)
        program_evaluator = compile_program_evaluator(
            program, backend=program.backend, sample_z=z0
        )
        self.assertTrue(program_evaluator.has_gradient)
        self.assertTrue(program_evaluator.has_jacobian_h)
        traj = planner.solve().trajectory
        self.assertTrue(planner.last_optimization_result.success)
        np.testing.assert_allclose(traj.x[:, 0], [0.0], atol=1e-07)
        np.testing.assert_allclose(traj.x[:, -1], [1.0], atol=1e-07)

    def test_jax_shooting_solves_single_integrator(self):
        problem = self.make_single_integrator_problem()
        planner = TrajectoryOptimizationPlanner(
            problem,
            transcription=ShootingTranscription(ShootingOptions(n_steps=5)),
            options=TrajectoryOptimizationOptions(
                compile_backend="jax", optimizer_options={"maxiter": 100, "ftol": 1e-09}
            ),
        )
        guess = default_initial_trajectory(
            problem, planner.transcription.initial_guess_time_grid(problem)
        )
        program = planner.transcription.transcribe(problem, compile_backend="jax")
        z0 = planner.transcription.pack_initial_guess(problem, guess)
        program_evaluator = compile_program_evaluator(
            program, backend=program.backend, sample_z=z0
        )
        self.assertEqual(program.n_z, problem.sys.m * 5)
        self.assertTrue(program_evaluator.has_gradient)
        self.assertTrue(program_evaluator.has_jacobian_h)
        self.assertTrue(program_evaluator.has_jacobian_g)
        traj = planner.solve().trajectory
        self.assertTrue(planner.last_optimization_result.success)
        np.testing.assert_allclose(traj.x[:, 0], [0.0], atol=1e-07)
        np.testing.assert_allclose(traj.x[:, -1], [1.0], atol=1e-07)

    def test_jax_multiple_shooting_solves_single_integrator(self):
        problem = self.make_single_integrator_problem()
        planner = TrajectoryOptimizationPlanner(
            problem,
            transcription=MultipleShootingTranscription(
                MultipleShootingOptions(n_steps=5)
            ),
            options=TrajectoryOptimizationOptions(
                compile_backend="jax", optimizer_options={"maxiter": 100, "ftol": 1e-09}
            ),
        )
        guess = default_initial_trajectory(
            problem, planner.transcription.initial_guess_time_grid(problem)
        )
        program = planner.transcription.transcribe(problem, compile_backend="jax")
        z0 = planner.transcription.pack_initial_guess(problem, guess)
        program_evaluator = compile_program_evaluator(
            program, backend=program.backend, sample_z=z0
        )
        self.assertEqual(program.n_z, (problem.sys.n + problem.sys.m) * 5)
        self.assertTrue(program_evaluator.has_gradient)
        self.assertTrue(program_evaluator.has_jacobian_h)
        traj = planner.solve().trajectory
        self.assertTrue(planner.last_optimization_result.success)
        np.testing.assert_allclose(traj.x[:, 0], [0.0], atol=1e-07)
        np.testing.assert_allclose(traj.x[:, -1], [1.0], atol=1e-07)

    def test_jax_direct_collocation_accepts_traceable_terminal_set(self):
        base = self.make_single_integrator_problem()
        problem = PlanningProblem(
            sys=base.sys,
            tf=1.0,
            x_start=np.array([0.0]),
            Xf=BallSet(center=np.array([1.0]), radius=0.1),
            cost=base.cost,
        )
        tr = DirectCollocationTranscription(DirectCollocationOptions(n_steps=5))
        guess = default_initial_trajectory(problem, tr.initial_guess_time_grid(problem))
        program = tr.transcribe(problem, compile_backend="jax")
        z0 = tr.pack_initial_guess(problem, guess)
        program_evaluator = compile_program_evaluator(
            program, backend=program.backend, sample_z=z0
        )
        self.assertEqual(program_evaluator.n_g, 1)
        self.assertTrue(program_evaluator.has_jacobian_g)

    def test_jax_evaluator_forced_rk4_rollout_smoke(self):
        sys = JaxSingleIntegrator()
        evaluator = sys.compile(backend="jax", verbose=False)
        x = evaluator.rk4_integrate_linear(np.array([0.0]), np.ones((5, 1)), 0.0, 0.25)
        np.testing.assert_allclose(np.asarray(x).reshape(-1), np.linspace(0.0, 1.0, 5))


# from test_jax_kinematic_bicycle.py
pytest.importorskip("jax")
from minilink.dynamics.catalog.vehicles.steering import (
    JaxKinematicBicycle,
    JaxKinematicBicycleRateInputs,
    KinematicBicycle,
)


@pytest.mark.optional
@pytest.mark.jax
class TestJaxKinematicBicycle(unittest.TestCase):
    def setUp(self):
        configure_jax(enable_x64=True)

    def test_regular_f_is_jaxpr_traceable(self):
        sys = JaxKinematicBicycle()
        x = jnp.zeros(3)
        u = jnp.zeros(2)
        jax.make_jaxpr(lambda xx, uu: sys.f(xx, uu))(x, u)

    def test_regular_f_matches_numpy_nominal(self):
        sys_j = JaxKinematicBicycle()
        sys_n = KinematicBicycle()
        x = jnp.zeros(3)
        u = jnp.array([2.0, 0.0])
        dx_j = sys_j.f(x, u)
        dx_n = sys_n.f(np.zeros(3), np.array([2.0, 0.0]))
        np.testing.assert_allclose(np.asarray(dx_j), dx_n, rtol=1e-05, atol=1e-05)

    def test_regular_f_matches_numpy_nontrivial(self):
        sys_j = JaxKinematicBicycle()
        sys_n = KinematicBicycle()
        x = jnp.array([1.0, -0.5, 0.4])
        u = jnp.array([3.0, 0.25])
        dx_j = sys_j.f(x, u)
        dx_n = sys_n.f(np.array([1.0, -0.5, 0.4]), np.array([3.0, 0.25]))
        np.testing.assert_allclose(np.asarray(dx_j), dx_n, rtol=1e-05, atol=1e-05)

    def test_rate_f_is_jaxpr_traceable(self):
        sys = JaxKinematicBicycleRateInputs()
        x = jnp.zeros(5)
        u = jnp.zeros(2)
        jax.make_jaxpr(lambda xx, uu: sys.f(xx, uu))(x, u)

    def test_rate_f_integrates_command_rates(self):
        sys = JaxKinematicBicycleRateInputs()
        x = jnp.array([0.0, 0.0, 0.0, 2.0, 0.1])
        u = jnp.array([0.5, -0.2])
        dx = np.asarray(sys.f(x, u))
        self.assertEqual(dx.shape, (5,))
        np.testing.assert_allclose(dx[3:], np.array([0.5, -0.2]))

    def test_rate_f_pose_derivative_matches_regular_model(self):
        sys_rate = JaxKinematicBicycleRateInputs()
        sys_reg = JaxKinematicBicycle()
        x_rate = jnp.array([1.0, 2.0, 0.3, 4.0, -0.15])
        u_rate = jnp.zeros(2)
        dx_rate = np.asarray(sys_rate.f(x_rate, u_rate))
        dx_reg = np.asarray(sys_reg.f(x_rate[:3], jnp.array([x_rate[3], x_rate[4]])))
        np.testing.assert_allclose(dx_rate[:3], dx_reg, rtol=1e-05, atol=1e-05)
