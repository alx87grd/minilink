"""Unit tests for ModelPredictiveController."""

import unittest
import numpy as np
import pytest

pytest.importorskip("jax")
import jax.numpy as jnp
import matplotlib

matplotlib.use("Agg")
from minilink.control.mpc import Command, ModelPredictiveController
from minilink.core.backends import configure_jax
from minilink.core.costs import QuadraticCost
from minilink.core.hybrid_diagram import HybridDiagram
from minilink.core.system import DynamicSystem, StepSystem, System
from minilink.planning.problems import PlanningProblem
from minilink.planning.trajectory_optimization.direct_collocation import (
    DirectCollocationOptions,
    DirectCollocationTranscription,
)
from minilink.planning.trajectory_optimization.planner import (
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
class TestModelPredictiveController(unittest.TestCase):
    def setUp(self):
        configure_jax(enable_x64=True)

    def _make_planner(self, x_start=0.0):
        sys = JaxSingleIntegrator()
        cost = QuadraticCost.from_system(
            sys, Q=np.zeros((1, 1)), R=np.eye(1), S=np.zeros((1, 1))
        )
        problem = PlanningProblem(
            sys=sys, x_start=np.array([x_start]), cost=cost, tf=1.0
        )
        return TrajectoryOptimizationPlanner(
            problem,
            transcription=DirectCollocationTranscription(
                DirectCollocationOptions(n_steps=5)
            ),
            options=TrajectoryOptimizationOptions(
                compile_backend="jax",
                record_solve_time=True,
                optimizer_options={"maxiter": 50, "ftol": 0.0001},
            ),
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
        self.assertIsNone(mpc.get_solve_metadata())
        cmd = mpc.compute_command(np.array([0.1]), k=0)
        self.assertIsInstance(cmd, Command)
        self.assertEqual(cmd.k, 0)
        self.assertAlmostEqual(cmd.t_solve, 0.0)
        self.assertTrue(hasattr(cmd.plan.metadata, "success"))
        self.assertIs(cmd.metadata, cmd.plan.metadata)
        self.assertIs(mpc.get_solve_metadata(), cmd.metadata)
        self.assertIsNotNone(cmd.plan.warm_state)
        np.testing.assert_allclose(cmd.u_ff, cmd.plan.trajectory.u[:, 0])
        self.assertIs(mpc.last_command, cmd)

    def test_reset_clears_deploy_state(self):
        planner = self._make_planner(0.1)
        mpc = ModelPredictiveController(planner, dt_mpc=0.2, warm_start=False)
        cmd0 = mpc.compute_command(np.array([0.1]))
        self.assertEqual(cmd0.k, 0)
        self.assertIsNotNone(mpc.get_solve_metadata())
        mpc.reset()
        self.assertIsNone(mpc.last_command)
        self.assertIsNotNone(mpc.get_solve_metadata())
        cmd1 = mpc.compute_command(np.array([0.1]))
        self.assertEqual(cmd1.k, 0)

    def test_compute_command_parity_with_ports(self):
        planner = self._make_planner(0.0)
        mpc = ModelPredictiveController(planner, dt_mpc=0.2, warm_start=False)
        y = np.array([0.15])
        cmd = mpc.compute_command(y, k=2)
        u_ff = mpc.outputs["u_ff"].compute(None, y, t=2)
        np.testing.assert_allclose(cmd.u_ff, u_ff)

    def test_params_scene_not_implemented(self):
        planner = self._make_planner()
        mpc = ModelPredictiveController(planner, dt_mpc=0.2, warm_start=False)
        with self.assertRaises(NotImplementedError):
            mpc.compute_command(np.array([0.0]), params={"scene": {}})

    def test_params_empty_ok(self):
        planner = self._make_planner(0.0)
        mpc = ModelPredictiveController(planner, dt_mpc=0.2, warm_start=False)
        cmd = mpc.compute_command(np.array([0.0]), params={})
        self.assertIsInstance(cmd, Command)

    def test_params_unknown_rejected(self):
        planner = self._make_planner()
        mpc = ModelPredictiveController(planner, dt_mpc=0.2, warm_start=False)
        with self.assertRaises(ValueError):
            mpc.compute_command(np.array([0.0]), params={"foo": 1})

    def test_nominal_interpolator_and_getters(self):
        planner = self._make_planner(0.1)
        mpc = ModelPredictiveController(planner, dt_mpc=0.2, warm_start=False)
        with self.assertRaises(RuntimeError):
            mpc.get_nominal_u(0.0)
        cmd = mpc.compute_command(np.array([0.1]), k=0, t=0.0)
        mpc.generate_nominal_interpolator(derivatives=True)
        u0 = mpc.get_nominal_u(0.0)
        np.testing.assert_allclose(u0, cmd.u_ff, atol=1e-05)
        x0 = mpc.get_nominal_x(0.0)
        np.testing.assert_allclose(x0, cmd.plan.trajectory.x[:, 0], atol=1e-05)
        t_mid = 0.5
        u_m = mpc.get_nominal_u(t_mid)
        self.assertEqual(u_m.shape, (1,))
        du = mpc.get_nominal_u_dot(t_mid)
        dx = mpc.get_nominal_x_dot(t_mid)
        self.assertEqual(du.shape, (1,))
        self.assertEqual(dx.shape, (1,))
        u_end = mpc.get_nominal_u(1000.0)
        np.testing.assert_allclose(u_end, cmd.plan.trajectory.u[:, -1], atol=1e-05)

    def test_deploy_shaped_two_rate_hand_loop(self):
        """Mirrors RAS: replan + interpolator, then many get_nominal_u."""
        planner = self._make_planner(0.0)
        mpc = ModelPredictiveController(planner, dt_mpc=0.2, warm_start=True)
        n_solve = []
        _solve = planner.solve_trajectory_from

        def solve_w(*args, **kwargs):
            n_solve.append(1)
            return _solve(*args, **kwargs)

        planner.solve_trajectory_from = solve_w
        try:
            mpc.compute_command(np.array([0.0]), t=0.0)
            mpc.generate_nominal_interpolator(derivatives=True)
            for i in range(10):
                t = 0.01 * i
                u = mpc.get_nominal_u(t)
                self.assertEqual(u.shape, (1,))
            self.assertEqual(len(n_solve), 1)
        finally:
            planner.solve_trajectory_from = _solve

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
        from minilink.control.mpc.utilities import mpc_default_computer_x0
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
        n_compile_parametric = []
        n_plant_compile = []
        n_computer_compile = []
        _solve = planner.solve_trajectory_from
        _compile_parametric = planner.compile_parametric_program
        _plant_compile = plant.compile
        _computer_compile = Computer.compile
        _hs_init = HybridSimulator.__init__

        def solve_w(*args, **kwargs):
            n_solve.append(1)
            return _solve(*args, **kwargs)

        def compile_parametric_w(*args, **kwargs):
            n_compile_parametric.append(1)
            return _compile_parametric(*args, **kwargs)

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
        planner.compile_parametric_program = compile_parametric_w
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
            planner.compile_parametric_program = _compile_parametric
            Computer.compile = _computer_compile
            HybridSimulator.__init__ = _hs_init
        self.assertEqual(len(n_solve), n_ticks)
        self.assertEqual(len(n_compile_parametric), 0)
        self.assertEqual(len(n_plant_compile), 1)
        self.assertEqual(len(n_computer_compile), 1)
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


from unittest.mock import patch

pytest.importorskip("jax")
from minilink.control.mpc import ModelPredictiveController
from minilink.core.system import DynamicSystem, System


@pytest.mark.optional
@pytest.mark.jax
class TestMPCAlgebraicController(unittest.TestCase):
    def setUp(self):
        configure_jax(enable_x64=True)

    def _make_planner(self, x_start=0.0):
        sys = JaxSingleIntegrator()
        cost = QuadraticCost.from_system(
            sys, Q=np.zeros((1, 1)), R=np.eye(1), S=np.zeros((1, 1))
        )
        problem = PlanningProblem(
            sys=sys, x_start=np.array([x_start]), cost=cost, tf=1.0
        )
        transcription = DirectCollocationTranscription(
            DirectCollocationOptions(n_steps=5)
        )
        planner = TrajectoryOptimizationPlanner(
            problem,
            transcription=transcription,
            options=TrajectoryOptimizationOptions(
                compile_backend="jax",
                record_solve_time=True,
                optimizer_options={"maxiter": 50, "ftol": 0.0001},
            ),
        )
        return planner

    def test_factory_and_port_dims(self):
        planner = self._make_planner()
        block = ModelPredictiveController(planner, dt_mpc=0.2, warm_start=False)
        self.assertIsInstance(block, System)
        self.assertNotIsInstance(block, DynamicSystem)
        self.assertEqual(block.n, 0)
        self.assertEqual(block.inputs["y"].dim, 1)
        self.assertEqual(block.outputs["u_ff"].dim, 1)
        self.assertEqual(block.outputs["x_ff"].dim, 1)
        n_z = planner.transcription.decision_dimension(planner.problem)
        self.assertEqual(block.outputs["z"].dim, n_z)

    def test_feedforward_slices_match_planner_step(self):
        planner = self._make_planner(0.2)
        block = ModelPredictiveController(planner, dt_mpc=0.2, warm_start=False)
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
        block = ModelPredictiveController(planner, dt_mpc=0.2, warm_start=False)
        y = np.array([0.1])
        with patch.object(
            planner, "solve_trajectory_from", wraps=planner.solve_trajectory_from
        ) as step_mock:
            block.outputs["u_ff"].compute(None, y, t=3)
            block.outputs["x_ff"].compute(None, y, t=3)
            block.outputs["z"].compute(None, y, t=3)
            self.assertEqual(step_mock.call_count, 1)

    def test_bad_measurement_dim_raises(self):
        planner = self._make_planner()
        block = ModelPredictiveController(planner, dt_mpc=0.2, warm_start=False)
        with self.assertRaises(ValueError):
            block.outputs["u_ff"].compute(None, np.array([0.0, 1.0]), t=0)

    def test_n_steps_one_rejected(self):
        planner = self._make_planner()
        planner.transcription.options.n_steps = 1
        with self.assertRaises(ValueError):
            ModelPredictiveController(planner, dt_mpc=0.2, warm_start=False)


pytest.importorskip("jax")
from minilink.control.mpc.utilities import _shift_plan_trajectory, mpc_warm_start_guess
from minilink.core.system import DynamicSystem, StepSystem
from minilink.core.trajectory import Trajectory


@pytest.mark.optional
@pytest.mark.jax
class TestMPCWarmStartController(unittest.TestCase):
    def setUp(self):
        configure_jax(enable_x64=True)

    def _make_planner(self, x_start=0.0):
        sys = JaxSingleIntegrator()
        cost = QuadraticCost.from_system(
            sys, Q=np.zeros((1, 1)), R=np.eye(1), S=np.zeros((1, 1))
        )
        problem = PlanningProblem(
            sys=sys, x_start=np.array([x_start]), cost=cost, tf=1.0
        )
        transcription = DirectCollocationTranscription(
            DirectCollocationOptions(n_steps=5)
        )
        planner = TrajectoryOptimizationPlanner(
            problem,
            transcription=transcription,
            options=TrajectoryOptimizationOptions(
                compile_backend="jax",
                record_solve_time=True,
                optimizer_options={"maxiter": 50, "ftol": 0.0001},
            ),
        )
        return planner

    def test_factory_and_state_dim(self):
        planner = self._make_planner()
        dt_mpc = 0.2
        block = ModelPredictiveController(planner, dt_mpc=dt_mpc, warm_start=True)
        self.assertIsInstance(block, StepSystem)
        n_z = planner.transcription.decision_dimension(planner.problem)
        self.assertEqual(block.n, n_z)
        self.assertEqual(block.outputs["z"].dim, n_z)
        self.assertNotIn("x", block.outputs)

    def test_shift_plan_trajectory_matches_demo_mask(self):
        t = np.linspace(0.0, 1.0, 6)
        plan = Trajectory(t=t, x=np.vstack([t, t**2]), u=np.ones((1, t.size)))
        x_meas = np.array([0.5, 0.25])
        shifted = _shift_plan_trajectory(
            plan, x_meas, dt_shift=0.2, horizon=1.0, t_anchor=0.4
        )
        self.assertIsNotNone(shifted)
        assert shifted is not None
        np.testing.assert_allclose(shifted.x[:, 0], x_meas)
        expected_t = plan.t + 0.2
        mask = expected_t <= 1.0 + 1e-09
        np.testing.assert_allclose(shifted.t, expected_t[mask] - 0.4)

    def test_warm_start_guess_first_tick_uses_default(self):
        planner = self._make_planner()
        z0 = np.ones(planner.transcription.decision_dimension(planner.problem))
        guess = mpc_warm_start_guess(z0, np.array([0.1]), planner, dt_mpc=0.2, k=0)
        default = mpc_warm_start_guess(None, np.array([0.1]), planner, dt_mpc=0.2, k=0)
        np.testing.assert_allclose(guess.t, default.t)
        np.testing.assert_allclose(guess.x, default.x)

    def test_feedforward_slices_match_planner_step(self):
        planner = self._make_planner(0.2)
        block = ModelPredictiveController(planner, dt_mpc=0.2, warm_start=True)
        z_prev = block.x0.copy()
        y = np.array([0.2])
        plan = planner.step(
            y, initial_guess=mpc_warm_start_guess(z_prev, y, planner, dt_mpc=0.2, k=1)
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
        block = ModelPredictiveController(planner, dt_mpc=0.2, warm_start=True)
        z_prev = block.x0.copy()
        y = np.array([0.1])
        with patch.object(
            planner, "solve_trajectory_from", wraps=planner.solve_trajectory_from
        ) as step_mock:
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
            ModelPredictiveController(planner, dt_mpc=0.2, warm_start=True)


pytest.importorskip("jax")
from minilink.core.system import DynamicSystem


@pytest.mark.optional
@pytest.mark.jax
class TestTrajectoryOptimizationPlanner(unittest.TestCase):
    def setUp(self):
        configure_jax(enable_x64=True)

    def make_problem(self, x_start=0.0):
        sys = JaxSingleIntegrator()
        cost = QuadraticCost.from_system(
            sys, Q=np.zeros((1, 1)), R=np.eye(1), S=np.zeros((1, 1))
        )
        return PlanningProblem(sys=sys, tf=1.0, x_start=np.array([x_start]), cost=cost)

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
                optimizer_options={"maxiter": 50, "ftol": 0.0001},
            ),
        )
        planner.compile_parametric_program()
        return planner

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
        np.testing.assert_allclose(traj.x[:, 0], x_start, atol=1e-05)

    def test_solve_returns_trajectory_plan(self):
        problem = self.make_problem(0.1)
        planner = self.make_planner(problem)
        plan = planner.solve()
        traj = plan.trajectory
        np.testing.assert_allclose(traj.x[:, 0], problem.x_start, atol=1e-05)
        self.assertIs(planner.last_trajectory_plan, plan)


pytest.importorskip("jax")
from minilink.planning.initial_guess import default_initial_trajectory


@pytest.mark.optional
@pytest.mark.jax
class TestMPCSolveTrajectoryFrom(unittest.TestCase):
    def setUp(self):
        configure_jax(enable_x64=True)

    def make_problem(self, x_start=0.0):
        sys = JaxSingleIntegrator()
        cost = QuadraticCost.from_system(
            sys, Q=np.zeros((1, 1)), R=np.eye(1), S=np.zeros((1, 1))
        )
        return PlanningProblem(sys=sys, tf=1.0, x_start=np.array([x_start]), cost=cost)

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
                optimizer_options={"maxiter": 50, "ftol": 0.0001},
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
        np.testing.assert_allclose(plan.trajectory.x, traj.x, atol=1e-08)
        np.testing.assert_allclose(plan.trajectory.u, traj.u, atol=1e-08)
        np.testing.assert_allclose(
            plan.warm_state, planner_from.last_optimization_result.z, atol=1e-08
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
        np.testing.assert_allclose(plan_none.trajectory.x[:, 0], x0, atol=1e-05)
        np.testing.assert_allclose(plan_empty.trajectory.x[:, 0], x0, atol=1e-05)

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
            plan_from.trajectory.x[:, 0], problem.x_start, atol=1e-05
        )
        planner_off = TrajectoryOptimizationPlanner(
            problem,
            transcription=DirectCollocationTranscription(
                DirectCollocationOptions(n_steps=5)
            ),
            options=TrajectoryOptimizationOptions(
                compile_backend="jax",
                record_solve_time=True,
                optimizer_options={"maxiter": 50, "ftol": 0.0001},
            ),
        )
        plan_off = planner_off.solve_trajectory()
        np.testing.assert_allclose(
            plan_off.trajectory.x[:, 0], problem.x_start, atol=1e-05
        )

    def test_initial_guess_accepted(self):
        problem = self.make_problem(0.0)
        planner = self.make_planner(problem)
        guess = default_initial_trajectory(
            problem, planner.transcription.initial_guess_time_grid(problem)
        )
        plan = planner.solve_trajectory_from(np.array([0.05]), initial_guess=guess)
        np.testing.assert_allclose(plan.trajectory.x[:, 0], [0.05], atol=1e-05)
        self.assertIsNotNone(plan.warm_state)


pytest.importorskip("jax")
from minilink.dynamics.catalog.vehicles.jax_vehicles import (
    BicycleDynRate,
)
from minilink.simulation.computer import Computer


def _planner():
    configure_jax(enable_x64=True)
    sys = BicycleDynRate()
    x0 = sys.x0.copy()
    return TrajectoryOptimizationPlanner(
        PlanningProblem(
            sys=sys, tf=1.0, x_start=x0, cost=QuadraticCost.from_system(sys, xbar=x0)
        ),
        transcription=DirectCollocationTranscription(
            DirectCollocationOptions(n_steps=5)
        ),
        options=TrajectoryOptimizationOptions(
            compile_backend="jax",
            record_solve_time=True,
            optimizer_method="scipy_slsqp",
            optimizer_options={"maxiter": 5, "ftol": 0.1},
        ),
    )


class TestMpcExportComputer(unittest.TestCase):
    def test_step_block_defaults_schedule(self):
        planner = _planner()
        mpc = ModelPredictiveController(planner, dt_mpc=0.2, warm_start=True)
        computer = mpc.export_to_computer()
        self.assertIsInstance(computer, Computer)
        self.assertAlmostEqual(computer.schedule.dt_base, 0.2)

    def test_step_block_rejects_mismatched_schedule(self):
        planner = _planner()
        mpc = ModelPredictiveController(planner, dt_mpc=0.2, warm_start=True)
        with self.assertRaises(ValueError):
            mpc.export_to_computer(0.1)

    def test_algebraic_defaults_schedule_from_dt_mpc(self):
        planner = _planner()
        mpc = ModelPredictiveController(planner, dt_mpc=0.2, warm_start=False)
        computer = mpc.export_to_computer()
        self.assertAlmostEqual(computer.schedule.dt_base, 0.2)
        with self.assertRaises(ValueError):
            mpc.export_to_computer(0.1)

    def test_dual_rate_computer_schedule_and_u_nom(self):
        planner = _planner()
        mpc = ModelPredictiveController(planner, dt_mpc=0.2, warm_start=True)
        computer = mpc.dual_rate_computer(dt_broadcast=0.05)
        self.assertIsInstance(computer, Computer)
        self.assertAlmostEqual(computer.schedule.dt_base, 0.05)
        self.assertEqual(computer.schedule.fire["replan"], 4)
        self.assertEqual(computer.schedule.fire["broadcast"], 1)
        self.assertIn("u_nom", computer.diagram.outputs)
        self.assertIn("y", computer.diagram.inputs)
        with self.assertRaises(ValueError):
            mpc.dual_rate_computer(dt_broadcast=0.07)

    def test_dual_rate_matmul_hybrid(self):
        from minilink.core.hybrid_diagram import HybridDiagram

        planner = _planner()
        plant = planner.problem.sys
        mpc = ModelPredictiveController(planner, dt_mpc=0.2, warm_start=True)
        computer = mpc.dual_rate_computer(dt_broadcast=0.1)
        hybrid = computer @ plant
        self.assertIsInstance(hybrid, HybridDiagram)


import warnings
from minilink.control.mpc import ModelPredictiveController, mpc_default_computer_x0
from minilink.planning.trajectory_optimization.planner import (
    TrajectoryOptimizationPlanner,
)


class SingleIntegrator(DynamicSystem):
    def __init__(self):
        super().__init__(n=1, input_dim=1, output_dim=1, y_dependencies=())
        self.state.lower_bound = np.array([-10.0])
        self.state.upper_bound = np.array([10.0])
        self.inputs["u"].lower_bound = np.array([-10.0])
        self.inputs["u"].upper_bound = np.array([10.0])

    def f(self, x, u, t=0, params=None):
        return np.array([u[0]])

    def h(self, x, u, t=0, params=None):
        return x


def _make_numpy_planner(x_start=0.0):
    sys = SingleIntegrator()
    cost = QuadraticCost.from_system(
        sys, Q=np.zeros((1, 1)), R=np.eye(1), S=np.zeros((1, 1))
    )
    problem = PlanningProblem(sys=sys, x_start=np.array([x_start]), cost=cost, tf=1.0)
    return TrajectoryOptimizationPlanner(
        problem,
        n_steps=5,
        transcription="direct_collocation",
        compile_backend="numpy",
        optimizer_options={"maxiter": 50, "ftol": 0.0001},
    )


class TestMPCNumPyRebuild(unittest.TestCase):
    def test_controller_init_without_parametric_compile(self):
        planner = _make_numpy_planner()
        n_compile = []
        with patch.object(
            planner,
            "compile_parametric_program",
            side_effect=lambda: n_compile.append(1),
        ):
            mpc = ModelPredictiveController(planner, dt_mpc=0.2, warm_start=True)
        self.assertEqual(len(n_compile), 0)
        self.assertFalse(planner.has_parametric_program)
        self.assertIsInstance(mpc, StepSystem)

    def test_algebraic_numpy_controller_init(self):
        planner = _make_numpy_planner()
        mpc = ModelPredictiveController(planner, dt_mpc=0.2, warm_start=False)
        self.assertIsInstance(mpc, System)
        self.assertNotIsInstance(mpc, StepSystem)
        self.assertFalse(planner.has_parametric_program)

    def test_compute_command_rebuilds_plan(self):
        planner = _make_numpy_planner(0.1)
        mpc = ModelPredictiveController(planner, dt_mpc=0.2, warm_start=True)
        with warnings.catch_warnings(record=True) as caught:
            warnings.simplefilter("always")
            cmd = mpc.compute_command(np.array([0.1]), k=0)
        rebuild_msgs = [
            w.message
            for w in caught
            if "rebuilding the NLP each call" in str(w.message)
        ]
        self.assertEqual(rebuild_msgs, [])
        self.assertTrue(cmd.success)
        self.assertIsNotNone(planner.last_program)
        np.testing.assert_allclose(cmd.u_ff, cmd.plan.trajectory.u[:, 0])

    def test_rebuild_per_tick_new_program(self):
        planner = _make_numpy_planner(0.0)
        mpc = ModelPredictiveController(planner, dt_mpc=0.2, warm_start=True)
        programs = []
        original_solve = planner.solve_trajectory

        def solve_and_track(*args, **kwargs):
            plan = original_solve(*args, **kwargs)
            programs.append(planner.last_program)
            return plan

        with patch.object(planner, "solve_trajectory", side_effect=solve_and_track):
            mpc.compute_command(np.array([0.0]), k=0)
            mpc.compute_command(np.array([0.05]), k=1)
        self.assertEqual(len(programs), 2)
        self.assertIsNot(programs[0], programs[1])

    def test_hybrid_closed_loop_smoke(self):
        planner = _make_numpy_planner(0.0)
        plant = SingleIntegrator()
        plant.x0 = np.array([0.0])
        dt_mpc = 0.2
        tf = 0.6
        n_ticks = int(round(tf / dt_mpc))
        mpc = ModelPredictiveController(planner, dt_mpc=dt_mpc, warm_start=True)
        hybrid = mpc @ plant
        self.assertIsInstance(hybrid, HybridDiagram)
        n_solve = []
        n_compile_parametric = []
        _solve = planner.solve_trajectory_from
        _compile_parametric = planner.compile_parametric_program

        def solve_w(*args, **kwargs):
            n_solve.append(1)
            return _solve(*args, **kwargs)

        def compile_parametric_w(*args, **kwargs):
            n_compile_parametric.append(1)
            return _compile_parametric(*args, **kwargs)

        planner.solve_trajectory_from = solve_w
        planner.compile_parametric_program = compile_parametric_w
        try:
            hybrid.compute_trajectory(
                tf=tf,
                x0_plant=plant.x0,
                x0_computer=mpc_default_computer_x0(planner),
                compile_backend="numpy",
                verbose=False,
            )
        finally:
            planner.solve_trajectory_from = _solve
            planner.compile_parametric_program = _compile_parametric
        self.assertEqual(len(n_solve), n_ticks)
        self.assertEqual(len(n_compile_parametric), 0)


pytest.importorskip("jax")
from minilink.control.mpc import (
    ModelPredictiveController,
    mpc_animation_overlays,
    mpc_plans_from_rollout,
)
from minilink.simulation.hybrid_simulator import HybridSimulator


def _build_bicycle_hybrid(*, mpc_hz=5.0):
    configure_jax(enable_x64=True)
    u_target = 4.0
    mpc_horizon = 1.0
    mpc_steps = 8
    sys_mpc = BicycleDynRate()
    sys_sim = BicycleDynRate()
    sys_sim.params["mass"] = 1.03 * sys_mpc.params["mass"]
    w_rear_max = 90.0
    delta_max = 0.55
    w_rear_dot_max = 80.0
    delta_dot_max = 2.0
    for sys in (sys_mpc, sys_sim):
        sys.state.lower_bound[6] = 0.0
        sys.state.upper_bound[6] = w_rear_max
        sys.state.lower_bound[7] = -delta_max
        sys.state.upper_bound[7] = delta_max
        sys.inputs["u"].lower_bound = np.array([-w_rear_dot_max, -delta_dot_max])
        sys.inputs["u"].upper_bound = np.array([w_rear_dot_max, delta_dot_max])
    r_r = sys_mpc.params["r_r"]
    w_rear_ref = u_target / r_r
    x_ref = np.array([0.0, 0.0, 0.0, u_target, 0.0, 0.0, w_rear_ref, 0.0])
    cost = QuadraticCost.from_system(
        sys_mpc,
        Q=np.diag([0.0, 12.0, 18.0, 0.5, 4.0, 6.0, 0.1, 100.0]),
        R=np.diag([1.0, 25.0]),
        S=np.diag([0.0, 30.0, 40.0, 2.0, 12.0, 18.0, 0.1, 100.0]),
        xbar=x_ref,
        ubar=np.zeros(2),
    )
    x0 = np.array([0.0, 3.0, 0.0, u_target * 0.8, 0.0, 0.0, u_target * 0.8 / r_r, 0.0])
    sys_sim.x0 = x0.copy()
    template_problem = PlanningProblem(
        sys=sys_mpc, x_start=x0, cost=cost, tf=mpc_horizon
    )
    transcription = DirectCollocationTranscription(
        DirectCollocationOptions(n_steps=mpc_steps)
    )
    mpc_planner = TrajectoryOptimizationPlanner(
        template_problem,
        transcription=transcription,
        options=TrajectoryOptimizationOptions(
            compile_backend="jax",
            record_solve_time=True,
            optimizer_method="scipy_slsqp",
            optimizer_options={"maxiter": 80, "ftol": 0.01},
        ),
    )
    mpc_dt = 1.0 / mpc_hz
    mpc = ModelPredictiveController(mpc_planner, dt_mpc=mpc_dt, warm_start=False)
    hybrid = mpc % mpc_dt @ sys_sim
    return (hybrid, mpc_planner, transcription, template_problem)


@pytest.mark.optional
@pytest.mark.jax
class TestMpcHybridStraightLine(unittest.TestCase):
    def test_stateless_u_ff_matches_planner_in_hybrid_sim(self):
        hybrid, mpc_planner, transcription, problem = _build_bicycle_hybrid()
        n_steps = 4
        sim = HybridSimulator(hybrid, t0=0.0, n_steps=n_steps, plant_dt_inner=0.005)
        result = sim.solve()
        plant_eval = hybrid.plant.compile()
        u_nom = hybrid.plant.get_u_from_input_ports()
        y0 = np.asarray(
            plant_eval.outputs(hybrid.plant.x0, u_nom, 0.0)["y"], dtype=float
        )
        y_hist = result.computer.signals["y"]
        for col in range(n_steps):
            y_k = y0 if col == 0 else y_hist[:, col - 1]
            plan = mpc_planner.step(y_k, initial_guess=None)
            np.testing.assert_allclose(
                result.computer.signals["u_ff"][:, col],
                plan.u[:, 0],
                rtol=0.0001,
                atol=0.0001,
            )

    def test_mpc_plans_from_rollout_smoke(self):
        hybrid, _planner, transcription, problem = _build_bicycle_hybrid()
        n_steps = 3
        result = HybridSimulator(
            hybrid, t0=0.0, n_steps=n_steps, plant_dt_inner=0.005
        ).solve()
        plans = mpc_plans_from_rollout(
            result.computer,
            transcription,
            problem,
            t0=0.0,
            dt_mpc=hybrid.computer.schedule.dt_base,
        )
        self.assertEqual(len(plans), n_steps)
        self.assertEqual(plans[0][1].n_samples, transcription.options.n_steps)

    def test_mpc_animation_overlays_smoke(self):
        hybrid, planner, _transcription, _problem = _build_bicycle_hybrid()
        n_steps = 3
        result = HybridSimulator(
            hybrid, t0=0.0, n_steps=n_steps, plant_dt_inner=0.005
        ).solve()
        overlays = mpc_animation_overlays(result, planner, reference_pad=5.0)
        self.assertEqual(len(overlays), 1)
        self.assertEqual(type(overlays[0]).__name__, "SceneHistory")


pytest.importorskip("jax")
from minilink.blocks.routing import Demux
from minilink.control.mpc.utilities import mpc_default_computer_x0, mpc_warm_start_guess
from minilink.core.diagram import DiagramSystem, StepDiagramSystem
from minilink.dynamics.catalog.vehicles.jax_vehicles import (
    BicycleDynRatePorts,
)
from minilink.simulation.computer import Computer, StepSchedule


def _build_bicycle_hybrid_warm(*, mpc_hz=5.0):
    configure_jax(enable_x64=True)
    u_target = 4.0
    mpc_horizon = 1.0
    mpc_steps = 8
    sys_mpc = BicycleDynRatePorts()
    sys_sim = BicycleDynRatePorts()
    sys_sim.params["mass"] = 1.03 * sys_mpc.params["mass"]
    w_rear_max = 90.0
    delta_max = 0.55
    w_rear_dot_max = 80.0
    delta_dot_max = 2.0
    for sys in (sys_mpc, sys_sim):
        sys.state.lower_bound[6] = 0.0
        sys.state.upper_bound[6] = w_rear_max
        sys.state.lower_bound[7] = -delta_max
        sys.state.upper_bound[7] = delta_max
        sys.inputs["w_rear_dot"].lower_bound[0] = -w_rear_dot_max
        sys.inputs["w_rear_dot"].upper_bound[0] = w_rear_dot_max
        sys.inputs["delta_dot"].lower_bound[0] = -delta_dot_max
        sys.inputs["delta_dot"].upper_bound[0] = delta_dot_max
    r_r = sys_mpc.params["r_r"]
    w_rear_ref = u_target / r_r
    x_ref = np.array([0.0, 0.0, 0.0, u_target, 0.0, 0.0, w_rear_ref, 0.0])
    cost = QuadraticCost.from_system(
        sys_mpc,
        Q=np.diag([0.0, 12.0, 18.0, 0.5, 4.0, 6.0, 0.1, 100.0]),
        R=np.diag([1.0, 25.0]),
        S=np.diag([0.0, 30.0, 40.0, 2.0, 12.0, 18.0, 0.1, 100.0]),
        xbar=x_ref,
        ubar=np.zeros(2),
    )
    x0 = np.array([0.0, 3.0, 0.0, u_target * 0.8, 0.0, 0.0, u_target * 0.8 / r_r, 0.0])
    sys_sim.x0 = x0.copy()
    template_problem = PlanningProblem(
        sys=sys_mpc, x_start=x0, cost=cost, tf=mpc_horizon
    )
    transcription = DirectCollocationTranscription(
        DirectCollocationOptions(n_steps=mpc_steps)
    )
    mpc_planner = TrajectoryOptimizationPlanner(
        template_problem,
        transcription=transcription,
        options=TrajectoryOptimizationOptions(
            compile_backend="jax",
            record_solve_time=True,
            optimizer_method="scipy_slsqp",
            optimizer_options={"maxiter": 80, "ftol": 0.01},
        ),
    )
    mpc_dt = 1.0 / mpc_hz
    mpc = ModelPredictiveController(mpc_planner, dt_mpc=mpc_dt, warm_start=True)
    step_diagram = StepDiagramSystem()
    step_diagram.add_subsystem(mpc, "mpc")
    step_diagram.add_input_port("y", dim=int(sys_mpc.n))
    step_diagram.connect("input", "y", "mpc", "y")
    step_diagram.connect_new_output_port("mpc", "u_ff", "u_ff")
    step_diagram.connect_new_output_port("mpc", "x_ff", "x_ff")
    step_diagram.connect_new_output_port("mpc", "z", "z")
    plant_diagram = DiagramSystem()
    plant_diagram.add_subsystem(sys_sim, "bike")
    plant_diagram.add_subsystem(Demux(dims=(1, 1)), "split")
    plant_diagram.add_input_port("u", dim=2)
    plant_diagram.connect("input", "u", "split", "u")
    plant_diagram.connect("split", "out0", "bike", "w_rear_dot")
    plant_diagram.connect("split", "out1", "bike", "delta_dot")
    plant_diagram.connect_new_output_port("bike", "y", "y")
    hybrid = HybridDiagram(
        computer=Computer(step_diagram, StepSchedule(dt_base=mpc_dt)),
        plant=plant_diagram,
    )
    hybrid.connect_boundary(
        direction="computer_to_plant", computer_port="u_ff", plant_port="u"
    )
    hybrid.connect_boundary(
        direction="plant_to_computer", computer_port="y", plant_port="y"
    )
    z0 = mpc_default_computer_x0(mpc_planner)
    return (hybrid, mpc_planner, transcription, template_problem, mpc_dt, z0)


@pytest.mark.optional
@pytest.mark.jax
class TestMpcHybridWarmStartParity(unittest.TestCase):
    def test_warm_started_u_ff_matches_hand_loop(self):
        hybrid, mpc_planner, _transcription, _problem, mpc_dt, z0 = (
            _build_bicycle_hybrid_warm()
        )
        n_steps = 4
        result = HybridSimulator(
            hybrid, t0=0.0, n_steps=n_steps, plant_dt_inner=0.005, x0_computer=z0
        ).solve()
        plant_eval = hybrid.plant.compile()
        u_nom = hybrid.plant.get_u_from_input_ports()
        y0 = np.asarray(
            plant_eval.outputs(hybrid.plant.x0, u_nom, 0.0)["y"], dtype=float
        )
        y_hist = result.computer.signals["y"]
        z_hist = result.computer.x
        for col in range(n_steps):
            y_k = y0 if col == 0 else y_hist[:, col - 1]
            z_prev = z0 if col == 0 else z_hist[:, col - 1]
            guess = mpc_warm_start_guess(z_prev, y_k, mpc_planner, dt_mpc=mpc_dt, k=col)
            plan = mpc_planner.step(y_k, initial_guess=guess)
            np.testing.assert_allclose(
                result.computer.signals["u_ff"][:, col],
                plan.u[:, 0],
                rtol=0.0001,
                atol=0.0001,
            )


pytest.importorskip("jax")
from minilink.control.mpc.utilities import (
    mpc_default_computer_x0,
    warm_start_guess_from_prev_plan,
)

U_TARGET = 4.0
TF_SIM = 5.0
MPC_HZ = 5.0
SIM_HZ = 200.0
MPC_HORIZON = 2.0
MPC_STEPS = 20
MPC_MAXITER = 150
MPC_FTOL = 0.01
MPC_DT = 1.0 / MPC_HZ
SIM_DT = 1.0 / SIM_HZ
SUBSTEPS = max(1, int(round(MPC_DT / SIM_DT)))
W_REAR_MAX = 90.0
DELTA_MAX = 0.55
W_REAR_DOT_MAX = 80.0
DELTA_DOT_MAX = 2.0


def _configure_bicycle_systems():
    configure_jax(enable_x64=True)
    sys_mpc = BicycleDynRatePorts()
    sys_sim = BicycleDynRatePorts()
    sys_sim.params["mass"] = 1.03 * sys_mpc.params["mass"]
    sys_sim.params["inertia"] = 1.02 * sys_mpc.params["inertia"]
    for sys in (sys_mpc, sys_sim):
        sys.state.lower_bound[6] = 0.0
        sys.state.upper_bound[6] = W_REAR_MAX
        sys.state.lower_bound[7] = -DELTA_MAX
        sys.state.upper_bound[7] = DELTA_MAX
        sys.inputs["w_rear_dot"].lower_bound[0] = -W_REAR_DOT_MAX
        sys.inputs["w_rear_dot"].upper_bound[0] = W_REAR_DOT_MAX
        sys.inputs["delta_dot"].lower_bound[0] = -DELTA_DOT_MAX
        sys.inputs["delta_dot"].upper_bound[0] = DELTA_DOT_MAX
    r_r = sys_mpc.params["r_r"]
    w_rear_ref = U_TARGET / r_r
    x_ref = np.array([0.0, 0.0, 0.0, U_TARGET, 0.0, 0.0, w_rear_ref, 0.0])
    cost = QuadraticCost.from_system(
        sys_mpc,
        Q=np.diag([0.0, 12.0, 18.0, 0.5, 4.0, 6.0, 0.1, 100.0]),
        R=np.diag([1.0, 25.0]),
        S=np.diag([0.0, 30.0, 40.0, 2.0, 12.0, 18.0, 0.1, 100.0]),
        xbar=x_ref,
        ubar=np.zeros(2),
    )
    x0 = np.array([0.0, 3.0, 0.0, U_TARGET * 0.8, 0.0, 0.0, U_TARGET * 0.8 / r_r, 0.0])
    return (sys_mpc, sys_sim, cost, x0)


def _make_planner(sys_mpc, cost, x0):
    problem = PlanningProblem(sys=sys_mpc, x_start=x0, cost=cost, tf=MPC_HORIZON)
    transcription = DirectCollocationTranscription(
        DirectCollocationOptions(n_steps=MPC_STEPS)
    )
    planner = TrajectoryOptimizationPlanner(
        problem,
        transcription=transcription,
        options=TrajectoryOptimizationOptions(
            compile_backend="jax",
            record_solve_time=True,
            optimizer_method="scipy_slsqp",
            optimizer_options={"maxiter": MPC_MAXITER, "ftol": MPC_FTOL},
        ),
    )
    return (planner, transcription, problem)


def run_hand_loop_warm_mpc(*, sys_sim, planner, problem):
    """
    Run a warm-started manual ``compute_command`` + RK4 ZOH loop.

    Returns MPC-fire times, ``u_hold`` commands, plant states at fires, and the fine plant traj.
    """
    sim_evaluator = sys_sim.compile(backend="jax", verbose=False)
    planner.compile_parametric_program()
    x0 = np.asarray(problem.x_start, dtype=float).reshape(-1)
    t_mpc = []
    u_mpc = []
    x_mpc = []
    t_hist = [0.0]
    x_hist = [x0.copy()]
    u_hist = [np.zeros(sys_sim.m)]
    x = x0.copy()
    t = 0.0
    u_hold = np.zeros(sys_sim.m)
    prev_plan = None
    next_mpc_t = 0.0
    while t < TF_SIM - 1e-12:
        if t >= next_mpc_t - 1e-12:
            guess = warm_start_guess_from_prev_plan(
                prev_plan, x, planner, dt_shift=MPC_DT, horizon=MPC_HORIZON, t_anchor=t
            )
            plan = planner.step(x, initial_guess=guess)
            prev_plan = plan
            u_hold = np.asarray(plan.u[:, 0], dtype=float).reshape(-1).copy()
            t_mpc.append(float(t))
            u_mpc.append(u_hold.copy())
            x_mpc.append(x.copy())
            next_mpc_t += MPC_DT
        for _ in range(SUBSTEPS):
            if t >= TF_SIM:
                break
            x = sim_evaluator.rk4_step(x, u_hold, t, SIM_DT)
            t += SIM_DT
            t_hist.append(t)
            x_hist.append(x.copy())
            u_hist.append(u_hold.copy())
    traj = Trajectory(
        t=np.asarray(t_hist, dtype=float),
        x=np.asarray(x_hist, dtype=float).T,
        u=np.asarray(u_hist, dtype=float).T,
    )
    return {
        "t_mpc": np.asarray(t_mpc, dtype=float),
        "u_mpc": np.asarray(u_mpc, dtype=float).T,
        "x_mpc": np.asarray(x_mpc, dtype=float).T,
        "traj": traj,
    }


def build_hybrid_warm_mpc(*, sys_mpc, sys_sim, planner):
    mpc = ModelPredictiveController(planner, dt_mpc=MPC_DT, warm_start=True)
    z0 = mpc_default_computer_x0(planner)
    step_diagram = StepDiagramSystem()
    step_diagram.add_subsystem(mpc, "mpc")
    step_diagram.add_input_port("y", dim=int(sys_mpc.n))
    step_diagram.connect("input", "y", "mpc", "y")
    step_diagram.connect_new_output_port("mpc", "u_ff", "u_ff")
    step_diagram.connect_new_output_port("mpc", "x_ff", "x_ff")
    step_diagram.connect_new_output_port("mpc", "z", "z")
    plant_diagram = DiagramSystem()
    plant_diagram.add_subsystem(sys_sim, "bike")
    plant_diagram.add_subsystem(Demux(dims=(1, 1)), "split")
    plant_diagram.add_input_port("u", dim=2)
    plant_diagram.connect("input", "u", "split", "u")
    plant_diagram.connect("split", "out0", "bike", "w_rear_dot")
    plant_diagram.connect("split", "out1", "bike", "delta_dot")
    plant_diagram.connect_new_output_port("bike", "y", "y")
    hybrid = HybridDiagram(
        computer=Computer(step_diagram, StepSchedule(dt_base=MPC_DT)),
        plant=plant_diagram,
    )
    hybrid.connect_boundary(
        direction="computer_to_plant", computer_port="u_ff", plant_port="u"
    )
    hybrid.connect_boundary(
        direction="plant_to_computer", computer_port="y", plant_port="y"
    )
    return (hybrid, z0)


def _plant_state_at_times(traj: Trajectory, times):
    """Nearest fine-grid plant state at each requested time."""
    t_arr = np.asarray(traj.t, dtype=float).reshape(-1)
    states = []
    for t_q in times:
        idx = int(np.argmin(np.abs(t_arr - float(t_q))))
        states.append(np.asarray(traj.x[:, idx], dtype=float).reshape(-1))
    return np.asarray(states, dtype=float).T


@pytest.mark.optional
@pytest.mark.jax
class TestMpcHybridDemoParity(unittest.TestCase):
    def test_warm_hybrid_matches_hand_loop_baseline(self):
        sys_mpc, sys_sim, cost, x0 = _configure_bicycle_systems()
        sys_sim.x0 = x0.copy()
        planner_hand, _transcription, problem = _make_planner(sys_mpc, cost, x0)
        hand = run_hand_loop_warm_mpc(
            sys_sim=sys_sim, planner=planner_hand, problem=problem
        )
        planner_hybrid, _, _ = _make_planner(sys_mpc, cost, x0)
        hybrid, z0 = build_hybrid_warm_mpc(
            sys_mpc=sys_mpc, sys_sim=sys_sim, planner=planner_hybrid
        )
        result = HybridSimulator(
            hybrid,
            t0=0.0,
            tf=TF_SIM,
            plant_dt_inner=SIM_DT,
            x0_plant=x0,
            x0_computer=z0,
            compile_backend="jax",
        ).solve()
        n_mpc = hand["t_mpc"].size
        self.assertEqual(result.computer.n_samples, n_mpc)
        u_hybrid = np.asarray(result.computer.signals["u_ff"][:, :n_mpc], dtype=float)
        np.testing.assert_allclose(
            u_hybrid,
            hand["u_mpc"],
            rtol=0.0001,
            atol=0.0001,
            err_msg="u_ff at MPC fires must match hand-loop u_hold",
        )
        plant_eval = hybrid.plant.compile(backend="jax")
        u_nom = hybrid.plant.get_u_from_input_ports()
        y0 = np.asarray(
            plant_eval.outputs(hybrid.plant.x0, u_nom, 0.0)["y"], dtype=float
        )
        y_hist = result.computer.signals["y"]
        for col in range(n_mpc):
            y_k = y0 if col == 0 else y_hist[:, col - 1]
            np.testing.assert_allclose(
                y_k,
                hand["x_mpc"][:, col],
                rtol=0.0001,
                atol=0.0001,
                err_msg=f"measurement at MPC fire {col}",
            )
        x_hybrid_at_mpc = _plant_state_at_times(result.plant, hand["t_mpc"])
        np.testing.assert_allclose(
            x_hybrid_at_mpc,
            hand["x_mpc"],
            rtol=0.001,
            atol=0.001,
            err_msg="plant state at MPC fire times must match hand loop",
        )
