"""Unit tests for ModelPredictiveController (phase E2)."""

import unittest

import numpy as np
import pytest

pytest.importorskip("jax")

import jax.numpy as jnp  # noqa: E402
import matplotlib  # noqa: E402

matplotlib.use("Agg")

from minilink.core.backends import configure_jax  # noqa: E402
from minilink.core.costs import QuadraticCost  # noqa: E402
from minilink.core.hybrid_diagram import HybridDiagram  # noqa: E402
from minilink.core.system import DynamicSystem, StepSystem, System  # noqa: E402
from minilink.planning.mpc import (  # noqa: E402
    Command,
    ModelPredictiveController,
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
class TestModelPredictiveController(unittest.TestCase):
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
        return MPCPlanner(
            problem,
            transcription=MPCDirectCollocationTranscription(
                DirectCollocationOptions(n_steps=5)
            ),
            options=MPCOptions(optimizer_options={"maxiter": 50, "ftol": 1e-4}),
        )

    def test_factory_warm_start_types(self):
        planner = self._make_planner()
        step = ModelPredictiveController(planner, dt_mpc=0.2, warm_start=True)
        alg = ModelPredictiveController(planner, dt_mpc=0.2, warm_start=False)
        self.assertIsInstance(step, StepSystem)
        self.assertIsInstance(alg, System)
        self.assertNotIsInstance(alg, StepSystem)
        self.assertEqual(step.dt_mpc, 0.2)
        self.assertEqual(alg.dt_mpc, 0.2)

    def test_compute_command_fields(self):
        planner = self._make_planner(0.1)
        mpc = ModelPredictiveController(planner, dt_mpc=0.2, warm_start=True)
        cmd = mpc.compute_command(np.array([0.1]), k=0)
        self.assertIsInstance(cmd, Command)
        self.assertEqual(cmd.k, 0)
        self.assertAlmostEqual(cmd.t_solve, 0.0)
        self.assertTrue(hasattr(cmd.plan.metadata, "success"))
        self.assertIsNotNone(cmd.plan.warm_state)
        np.testing.assert_allclose(cmd.u_ff, cmd.plan.trajectory.u[:, 0])
        self.assertIs(mpc.last_command, cmd)

    def test_compute_command_parity_with_ports(self):
        planner = self._make_planner(0.0)
        mpc = ModelPredictiveController(planner, dt_mpc=0.2, warm_start=False)
        y = np.array([0.15])
        cmd = mpc.compute_command(y, k=2)
        u_ff = mpc.outputs["u_ff"].compute(None, y, t=2)
        np.testing.assert_allclose(cmd.u_ff, u_ff)

    def test_params_rejected(self):
        planner = self._make_planner()
        mpc = ModelPredictiveController(planner, dt_mpc=0.2, warm_start=False)
        with self.assertRaises(ValueError):
            mpc.compute_command(np.array([0.0]), params={"scene": {}})

    def test_matmul_returns_hybrid(self):
        planner = self._make_planner()
        sys = planner.problem.sys
        mpc = ModelPredictiveController(planner, dt_mpc=0.2, warm_start=True)
        hybrid = mpc @ sys
        self.assertIsInstance(hybrid, HybridDiagram)

    def test_debug_figure_smoke(self):
        planner = self._make_planner()
        mpc = ModelPredictiveController(
            planner, dt_mpc=0.2, warm_start=False, debug=True
        )
        fig = mpc.init_debug_figure(planner.problem.sys)
        self.assertIsNotNone(fig)
        cmd = mpc.compute_command(np.array([0.0]), k=0)
        fig2 = mpc.update_debug_figure(cmd)
        self.assertIsNotNone(fig2)

    def test_hybrid_one_nlp_per_tick_and_compile_once(self):
        """Latch + compile-once: no per-tick re-prepare / re-compile / multi-solve."""
        from unittest.mock import patch

        from minilink.planning.mpc.warm_start import mpc_default_computer_x0
        from minilink.simulation.computer import Computer
        from minilink.simulation.hybrid_simulator import HybridSimulator

        planner = self._make_planner(0.0)
        plant = JaxSingleIntegrator()
        plant.x0 = np.array([0.0])
        dt_mpc = 0.2
        tf = 1.0
        n_ticks = int(round(tf / dt_mpc))

        mpc = ModelPredictiveController(planner, dt_mpc=dt_mpc, warm_start=True)
        hybrid = mpc @ plant
        x0 = np.array([0.0])
        x0_computer = mpc_default_computer_x0(planner)

        n_solve = []
        n_prepare = []
        n_plant_compile = []
        n_computer_compile = []

        _solve = planner.solve_trajectory_from
        _prepare = planner.prepare
        _plant_compile = plant.compile
        _computer_compile = Computer.compile
        _hs_init = HybridSimulator.__init__

        def solve_w(*args, **kwargs):
            n_solve.append(1)
            return _solve(*args, **kwargs)

        def prepare_w(*args, **kwargs):
            n_prepare.append(1)
            return _prepare(*args, **kwargs)

        def plant_compile_w(*args, **kwargs):
            n_plant_compile.append(1)
            return _plant_compile(*args, **kwargs)

        def computer_compile_w(self, *args, **kwargs):
            n_computer_compile.append(1)
            return _computer_compile(self, *args, **kwargs)

        def hs_init_w(self, *args, **kwargs):
            hyb = args[0] if args else kwargs["hybrid"]
            hyb.plant.compile = plant_compile_w
            return _hs_init(self, *args, **kwargs)

        planner.solve_trajectory_from = solve_w
        planner.prepare = prepare_w
        Computer.compile = computer_compile_w
        HybridSimulator.__init__ = hs_init_w
        try:
            hybrid.compute_trajectory(
                tf=tf,
                x0_plant=x0,
                x0_computer=x0_computer,
                compile_backend="numpy",
                verbose=False,
            )
        finally:
            planner.solve_trajectory_from = _solve
            planner.prepare = _prepare
            Computer.compile = _computer_compile
            HybridSimulator.__init__ = _hs_init

        self.assertEqual(len(n_solve), n_ticks)
        self.assertEqual(len(n_prepare), 0)
        self.assertEqual(len(n_plant_compile), 1)
        self.assertEqual(len(n_computer_compile), 1)

        # Ports + step on one discrete tick share the latch.
        leaf = ModelPredictiveController(planner, dt_mpc=dt_mpc, warm_start=True)
        z = leaf.x0.copy()
        y = np.array([0.05])
        with patch.object(
            planner, "solve_trajectory_from", wraps=planner.solve_trajectory_from
        ) as mock_solve:
            leaf.outputs["u_ff"].compute(z, y, t=4)
            leaf.outputs["x_ff"].compute(z, y, t=4)
            leaf.outputs["z"].compute(z, y, t=4)
            leaf.step(z, y, k=4)
            self.assertEqual(mock_solve.call_count, 1)
