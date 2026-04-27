"""Optional JAX direct-collocation smoke tests."""

import unittest

import numpy as np
import pytest

try:
    import jax
    import jax.numpy as jnp

    from minilink.core.costs import JaxQuadraticCost, QuadraticCost
    from minilink.core.system import DynamicSystem
    from minilink.dynamics.catalog.pendulum.cartpole import CartPole, JaxCartPole
    from minilink.optimization.optimizers.scipy_minimize import ScipyMinimizeOptimizer
    from minilink.planning.problems import PlanningProblem
    from minilink.planning.trajectory_optimization.jax_direct_collocation import (
        JaxDirectCollocationOptions,
        JaxDirectCollocationPlanner,
        JaxDirectCollocationTranscription,
    )

    HAS_JAX = True
except ImportError:
    HAS_JAX = False


@pytest.mark.optional
@pytest.mark.jax
@unittest.skipUnless(HAS_JAX, "jax not installed")
class TestJaxDirectCollocation(unittest.TestCase):
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

        sys = JaxSingleIntegrator()
        cost = JaxQuadraticCost.from_system(
            sys,
            Q=np.zeros((1, 1)),
            R=np.eye(1),
            S=np.zeros((1, 1)),
        )
        problem = PlanningProblem(
            sys=sys,
            x_start=np.array([0.0]),
            x_goal=np.array([1.0]),
            cost=cost,
        )
        planner = JaxDirectCollocationPlanner(
            problem,
            tf=1.0,
            n_steps=5,
            optimizer=ScipyMinimizeOptimizer(options={"maxiter": 100, "ftol": 1e-9}),
        )

        program = planner.transcription.transcribe(problem)
        self.assertIsNotNone(program.grad)
        self.assertIsNotNone(program.equalities[0].jac)

        traj = planner.compute_solution()

        self.assertTrue(planner.last_optimization_result.success)
        np.testing.assert_allclose(traj.x[:, 0], [0.0], atol=1e-7)
        np.testing.assert_allclose(traj.x[:, -1], [1.0], atol=1e-7)

    def test_rejects_numpy_quadratic_cost(self):
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

        sys = JaxSingleIntegrator()
        cost = QuadraticCost.from_system(
            sys,
            Q=np.zeros((1, 1)),
            R=np.eye(1),
            S=np.zeros((1, 1)),
        )
        problem = PlanningProblem(
            sys=sys,
            x_start=np.array([0.0]),
            x_goal=np.array([1.0]),
            cost=cost,
        )
        opts = JaxDirectCollocationOptions(
            tf=1.0,
            n_steps=5,
            compile_backend="direct",
        )
        tr = JaxDirectCollocationTranscription(opts)
        with self.assertRaisesRegex(ValueError, "JaxQuadraticCost"):
            tr.transcribe(problem)


if __name__ == "__main__":
    unittest.main()
