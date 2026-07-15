"""Unit tests for stateless MPC block."""

import unittest
from unittest.mock import patch

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
    MPCStatelessController,
    mpc_stateless_controller,
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
class TestMPCStatelessController(unittest.TestCase):
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
        problem = PlanningProblem(
            sys=sys, x_start=np.array([x_start]), cost=cost, tf=1.0
        )
        transcription = MPCDirectCollocationTranscription(
            DirectCollocationOptions(n_steps=5)
        )
        planner = MPCPlanner(
            problem,
            transcription=transcription,
            options=MPCOptions(optimizer_options={"maxiter": 50, "ftol": 1e-4}),
        )
        return planner

    def test_factory_and_port_dims(self):
        planner = self._make_planner()
        block = mpc_stateless_controller(planner)
        self.assertIsInstance(block, MPCStatelessController)
        self.assertEqual(block.n, 0)
        self.assertEqual(block.inputs["y"].dim, 1)
        self.assertEqual(block.outputs["u_ff"].dim, 1)
        self.assertEqual(block.outputs["x_ff"].dim, 1)
        n_z = planner.transcription.decision_dimension(planner.problem)
        self.assertEqual(block.outputs["z"].dim, n_z)

    def test_feedforward_slices_match_planner_step(self):
        planner = self._make_planner(0.2)
        block = mpc_stateless_controller(planner)
        y = np.array([0.2])
        plan = planner.step(y, initial_guess=None)

        u_ff = block.outputs["u_ff"].compute(None, y, t=0)
        x_ff = block.outputs["x_ff"].compute(None, y, t=1)
        z = block.outputs["z"].compute(None, y, t=1)

        np.testing.assert_allclose(u_ff, plan.u[:, 0])
        np.testing.assert_allclose(x_ff, plan.x[:, 1])
        np.testing.assert_allclose(z, planner.last_optimization_result.z.reshape(-1))

    def test_single_planner_step_per_tick_across_ports(self):
        planner = self._make_planner(0.0)
        block = mpc_stateless_controller(planner)
        y = np.array([0.1])

        with patch.object(
            planner,
            "step",
            wraps=planner.step,
        ) as step_mock:
            block.outputs["u_ff"].compute(None, y, t=3)
            block.outputs["x_ff"].compute(None, y, t=3)
            block.outputs["z"].compute(None, y, t=3)
            self.assertEqual(step_mock.call_count, 1)

    def test_bad_measurement_dim_raises(self):
        planner = self._make_planner()
        block = mpc_stateless_controller(planner)
        with self.assertRaises(ValueError):
            block.outputs["u_ff"].compute(None, np.array([0.0, 1.0]), t=0)

    def test_n_steps_one_rejected(self):
        planner = self._make_planner()
        planner.transcription.options.n_steps = 1
        with self.assertRaises(ValueError):
            mpc_stateless_controller(planner)


if __name__ == "__main__":
    unittest.main()
