"""JAX dynamic-bicycle smoke tests."""

import unittest

import numpy as np
import pytest

try:
    import jax
    import jax.numpy as jnp

    from minilink.compile.jax_utils import configure_jax
    from minilink.core.costs import JaxQuadraticCost
    from minilink.dynamics.catalog.vehicles.dynamic_bicycle import (
        DynamicBicycle,
        JaxDynamicBicycle,
    )
    from minilink.optimization.optimizers.scipy_minimize import ScipyMinimizeOptimizer
    from minilink.planning.problems import PlanningProblem
    from minilink.planning.trajectory_optimization.jax_direct_collocation import (
        JaxDirectCollocationOptions,
        JaxDirectCollocationTranscription,
    )
    from minilink.planning.trajectory_optimization.planner import (
        TrajectoryOptimizationOptions,
        TrajectoryOptimizationPlanner,
    )

    HAS_JAX = True
except ImportError:
    HAS_JAX = False


@pytest.mark.optional
@pytest.mark.jax
@unittest.skipUnless(HAS_JAX, "jax not installed")
class TestJaxDynamicBicycle(unittest.TestCase):
    def setUp(self):
        configure_jax(enable_x64=True)

    def test_jax_matches_numpy_dynamics_linear(self):
        sys_np = DynamicBicycle()
        sys_jx = JaxDynamicBicycle()
        x = np.array([0.1, -0.2, 0.3, 5.0, 0.4, 0.05])
        u = np.array([20.0, 0.1])

        dx_np = sys_np.f(x, u)
        dx_jx = np.asarray(sys_jx.f(jnp.asarray(x), jnp.asarray(u)))
        np.testing.assert_allclose(dx_jx, dx_np, rtol=1e-9, atol=1e-9)

    def test_jax_matches_numpy_dynamics_saturated(self):
        # Large wheel speed + steering pushes both tires onto the friction
        # circle, exercising the saturating branch of vel2forces.
        sys_np = DynamicBicycle()
        sys_jx = JaxDynamicBicycle()
        x = np.array([0.0, 0.0, 0.0, 5.0, 0.0, 0.0])
        u = np.array([100.0, 0.5])

        dx_np = sys_np.f(x, u)
        dx_jx = np.asarray(sys_jx.f(jnp.asarray(x), jnp.asarray(u)))
        np.testing.assert_allclose(dx_jx, dx_np, rtol=1e-9, atol=1e-9)

    def test_jax_dynamics_traces_and_grads(self):
        sys_jx = JaxDynamicBicycle()
        x = jnp.asarray([0.1, -0.2, 0.3, 5.0, 0.4, 0.05])
        u = jnp.asarray([20.0, 0.1])

        jax.make_jaxpr(lambda xx, uu: sys_jx.f(xx, uu))(x, u)

        def loss(xx, uu):
            return jnp.sum(sys_jx.f(xx, uu) ** 2)

        gx, gu = jax.grad(loss, argnums=(0, 1))(x, u)
        self.assertEqual(gx.shape, (6,))
        self.assertEqual(gu.shape, (2,))

    def test_jax_direct_collocation_smoke(self):
        sys = JaxDynamicBicycle()
        sys.inputs["w_rear"].lower_bound[0] = 0.0
        sys.inputs["w_rear"].upper_bound[0] = 80.0
        sys.inputs["delta"].lower_bound[0] = -0.6
        sys.inputs["delta"].upper_bound[0] = 0.6

        u_target = 12.0
        tf = 1.5
        x_start = np.array([0.0, 0.0, 0.0, u_target, 0.0, 0.0])
        x_goal = np.array([u_target * tf, 1.5, 0.0, u_target, 0.0, 0.0])
        ubar = np.array([u_target / sys.r_r, 0.0])

        cost = JaxQuadraticCost.from_system(
            sys,
            Q=np.diag([0.0, 4.0, 5.0, 0.1, 1.0, 1.0]),
            R=np.diag([1e-4, 50.0]),
            S=np.diag([0.0, 50.0, 50.0, 1.0, 10.0, 10.0]),
            xbar=x_goal,
            ubar=ubar,
        )
        problem = PlanningProblem(sys=sys, x_start=x_start, x_goal=x_goal, cost=cost)
        planner = TrajectoryOptimizationPlanner(
            problem,
            transcription=JaxDirectCollocationTranscription(
                JaxDirectCollocationOptions(tf=tf, n_steps=10, use_gradient=True)
            ),
            optimizer=ScipyMinimizeOptimizer(
                options={"maxiter": 60, "ftol": 1e-2, "disp": False}
            ),
            options=TrajectoryOptimizationOptions(compile_backend="jax"),
        )
        traj = planner.compute_solution()
        # Boundary conditions are enforced as equality constraints, so the
        # transcription should always honor them within optimizer tolerances.
        np.testing.assert_allclose(traj.x[:, 0], x_start, atol=1e-5)


if __name__ == "__main__":
    unittest.main()
