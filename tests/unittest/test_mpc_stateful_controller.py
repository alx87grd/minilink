"""Unit tests for warm-start MPC stateful controller."""

import unittest
from unittest.mock import patch

import numpy as np
import pytest

pytest.importorskip("jax")

import jax.numpy as jnp  # noqa: E402

from minilink.core.backends import configure_jax  # noqa: E402
from minilink.core.costs import QuadraticCost  # noqa: E402
from minilink.core.system import DynamicSystem  # noqa: E402
from minilink.core.trajectory import Trajectory  # noqa: E402
from minilink.planning.mpc import (  # noqa: E402
    MPCDirectCollocationTranscription,
    MPCOptions,
    MPCPlanner,
    MPCStatefulController,
    mpc_stateful_controller,
)
from minilink.planning.mpc.warm_start import (  # noqa: E402
    mpc_warm_start_guess,
    shift_plan_trajectory,
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
class TestMPCStatefulController(unittest.TestCase):
    def setUp(self):
        configure_jax(enable_x64=True)

    def _make_planner(self, x_start=0.0):
        sys = JaxSingleIntegrator()
        cost = QuadraticCost.from_system(
            sys,
            Q=np.zeros((1, 1)),
            R=np.eye(1),
            S=np.zeros((1, 1)),
        )
        problem = PlanningProblem(sys=sys, x_start=np.array([x_start]), cost=cost)
        transcription = MPCDirectCollocationTranscription(
            DirectCollocationOptions(tf=1.0, n_steps=5)
        )
        planner = MPCPlanner(
            problem,
            transcription=transcription,
            options=MPCOptions(optimizer_options={"maxiter": 50, "ftol": 1e-4}),
        )
        return planner

    def test_factory_and_state_dim(self):
        planner = self._make_planner()
        dt_mpc = 0.2
        block = mpc_stateful_controller(planner, dt_mpc=dt_mpc)
        self.assertIsInstance(block, MPCStatefulController)
        n_z = planner.transcription.decision_dimension(planner.problem)
        self.assertEqual(block.n, n_z)
        self.assertEqual(block.outputs["z"].dim, n_z)
        self.assertNotIn("x", block.outputs)

    def test_shift_plan_trajectory_matches_demo_mask(self):
        t = np.linspace(0.0, 1.0, 6)
        plan = Trajectory(
            t=t,
            x=np.vstack([t, t**2]),
            u=np.ones((1, t.size)),
        )
        x_meas = np.array([0.5, 0.25])
        shifted = shift_plan_trajectory(
            plan,
            x_meas,
            dt_shift=0.2,
            horizon=1.0,
            t_anchor=0.4,
        )
        self.assertIsNotNone(shifted)
        assert shifted is not None
        np.testing.assert_allclose(shifted.x[:, 0], x_meas)
        expected_t = plan.t + 0.2
        mask = expected_t <= 1.0 + 1e-9
        np.testing.assert_allclose(shifted.t, expected_t[mask] - 0.4)

    def test_warm_start_guess_first_tick_uses_default(self):
        planner = self._make_planner()
        z0 = np.ones(planner.transcription.decision_dimension(planner.problem))
        guess = mpc_warm_start_guess(
            z0,
            np.array([0.1]),
            planner,
            dt_mpc=0.2,
            k=0,
        )
        default = mpc_warm_start_guess(
            None,
            np.array([0.1]),
            planner,
            dt_mpc=0.2,
            k=0,
        )
        np.testing.assert_allclose(guess.t, default.t)
        np.testing.assert_allclose(guess.x, default.x)

    def test_feedforward_slices_match_planner_step(self):
        planner = self._make_planner(0.2)
        block = mpc_stateful_controller(planner, dt_mpc=0.2)
        z_prev = block.x0.copy()
        y = np.array([0.2])

        plan = planner.step(
            y,
            initial_guess=mpc_warm_start_guess(z_prev, y, planner, dt_mpc=0.2, k=1),
        )

        u_ff = block.outputs["u_ff"].compute(z_prev, y, t=1)
        x_ff = block.outputs["x_ff"].compute(z_prev, y, t=1)
        z_out = block.outputs["z"].compute(z_prev, y, t=1)

        np.testing.assert_allclose(u_ff, plan.u[:, 0])
        np.testing.assert_allclose(x_ff, plan.x[:, 1])
        np.testing.assert_allclose(
            z_out, planner.last_optimization_result.z.reshape(-1)
        )

    def test_single_planner_step_per_tick_across_ports_and_step(self):
        planner = self._make_planner(0.0)
        block = mpc_stateful_controller(planner, dt_mpc=0.2)
        z_prev = block.x0.copy()
        y = np.array([0.1])

        with patch.object(planner, "step", wraps=planner.step) as step_mock:
            block.outputs["u_ff"].compute(z_prev, y, t=3)
            block.outputs["x_ff"].compute(z_prev, y, t=3)
            block.outputs["z"].compute(z_prev, y, t=3)
            z_new = block.step(z_prev, y, k=3)
            self.assertEqual(step_mock.call_count, 1)

        z_port = block.outputs["z"].compute(z_prev, y, t=3)
        np.testing.assert_allclose(z_new, z_port)

    def test_n_steps_one_rejected(self):
        planner = self._make_planner()
        planner.transcription.options.n_steps = 1
        with self.assertRaises(ValueError):
            mpc_stateful_controller(planner, dt_mpc=0.2)


if __name__ == "__main__":
    unittest.main()
