"""The planner's compiled face and the MPC on it: one solve per tick, the plan one period late."""

import unittest
from unittest import mock

import matplotlib
import numpy as np
import pytest

matplotlib.use("Agg")

from minilink.analysis import discretize
from minilink.core.backends import configure_jax, require_jax_numpy
from minilink.core.costs import QuadraticCost
from minilink.core.system import DynamicSystem
from minilink.planning.initial_guess import default_initial_trajectory
from minilink.planning.problems import PlanningProblem
from minilink.planning.trajectory_optimization.direct_collocation import (
    DirectCollocationOptions,
    DirectCollocationTranscription,
)
from minilink.planning.trajectory_optimization.mpc import (
    ModelPredictiveController,
    PlanFollower,
    mpc_plans_from_rollout,
    plant_run,
)
from minilink.planning.trajectory_optimization.mpc.predictive import (
    DelayedPredictiveController,
    PurePredictiveController,
    WarmStartedPredictiveController,
    warm_guess,
)
from minilink.planning.trajectory_optimization.compiled import (
    CompiledTrajectoryPlanner,
)
from minilink.planning.trajectory_optimization.planner import (
    TrajectoryOptimizationOptions,
    TrajectoryOptimizationPlanner,
    TrajectoryOptimizationRecord,
)

DT = 0.2  # the MPC's period in the loop tests


class JaxSingleIntegrator(DynamicSystem):
    def __init__(self):
        super().__init__(n=1, input_dim=1, output_dim=1, y_dependencies=())
        self.state.lower_bound = np.array([-10.0])
        self.state.upper_bound = np.array([10.0])
        self.inputs["u"].lower_bound = np.array([-10.0])
        self.inputs["u"].upper_bound = np.array([10.0])

    def f(self, x, u, t=0, params=None):
        jnp = require_jax_numpy()
        return jnp.array([u[0]])

    def h(self, x, u, t=0, params=None):
        return x


@pytest.mark.optional
@pytest.mark.jax
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
        return PlanningProblem(
            sys=sys, tf=1.0, x_start=np.array([x_start]), cost=cost, U=sys.input_box
        )

    def make_planner(self, problem):
        transcription = DirectCollocationTranscription(
            DirectCollocationOptions(n_steps=5)
        )
        planner = TrajectoryOptimizationPlanner(
            problem,
            transcription=transcription,
            options=TrajectoryOptimizationOptions(
                backend="jax",
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
        self.assertIs(planner.last_solution, plan)
        self.assertIsInstance(plan.solver, TrajectoryOptimizationRecord)


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
        return PlanningProblem(
            sys=sys, tf=1.0, x_start=np.array([x_start]), cost=cost, U=sys.input_box
        )

    def make_planner(self, problem):
        transcription = DirectCollocationTranscription(
            DirectCollocationOptions(n_steps=5)
        )
        planner = TrajectoryOptimizationPlanner(
            problem,
            transcription=transcription,
            options=TrajectoryOptimizationOptions(
                backend="jax",
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
            planner_from.last_optimization_result.z,
            planner_step.last_optimization_result.z,
            atol=1e-08,
        )
        self.assertIs(planner_from.last_solution, plan)

    def test_solver_record_present(self):
        planner = self.make_planner(self.make_problem(0.1))
        plan = planner.solve_trajectory_from(np.array([0.1]))
        self.assertIsInstance(plan.success, bool)
        self.assertIsNotNone(plan.solver.message)
        self.assertIsNone(plan.solver.feasible)  # an online tick runs no residual check
        self.assertEqual(plan.success, plan.solver.solver_success)
        self.assertIsNone(plan.evaluation)  # not asked
        self.assertIsNotNone(planner.last_optimization_result.z)

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
                backend="jax",
                record_solve_time=True,
                optimizer_options={"maxiter": 50, "ftol": 0.0001},
            ),
        )
        plan_off = planner_off.solve_trajectory()
        np.testing.assert_allclose(
            plan_off.trajectory.x[:, 0], problem.x_start, atol=1e-05
        )

    def test_a_set_and_the_start_time_bind_like_a_rebuild(self):
        from minilink.core.sets import BallSet

        def problem_of(radius, t_start=0.0):
            sys = JaxSingleIntegrator()
            cost = QuadraticCost.from_system(
                sys, Q=np.eye(1), R=0.1 * np.eye(1), xbar=np.array([2.0])
            )
            return PlanningProblem(
                sys=sys,
                tf=1.0,
                t_start=t_start,
                x_start=np.array([0.0]),
                cost=cost,
                X=BallSet(np.zeros(1), radius),
                U=sys.input_box,
            )

        planner = self.make_planner(problem_of(1.0))
        self.assertIn("X", planner.program.params)  # a set read as margins is a leaf
        x0 = np.array([0.0])
        plan = planner.solve_trajectory_from(
            x0, params={"X": {"radius": 0.5}, "t_start": 3.0}
        )
        rebuilt = self.make_planner(problem_of(0.5, t_start=3.0)).solve_trajectory_from(
            x0
        )
        np.testing.assert_allclose(plan.trajectory.x, rebuilt.trajectory.x, atol=1e-6)
        np.testing.assert_allclose(plan.trajectory.t, 3.0 + np.linspace(0.0, 1.0, 5))
        self.assertLessEqual(plan.trajectory.x.max(), 0.5 + 1e-6)

        # A box lowered to the decision bounds is structure, not a leaf
        self.assertNotIn("U", planner.program.params)

        # A moved ball takes its own bounds: none of the compile-time box stays
        def moved(center):
            problem = problem_of(1.0)
            problem.X = BallSet(np.array([center]), 1.0)
            return problem

        start = np.array([1.5])
        plan = planner.solve_trajectory_from(start, params={"X": {"center": [1.0]}})
        rebuilt = self.make_planner(moved(1.0)).solve_trajectory_from(start)
        self.assertTrue(plan.success)
        np.testing.assert_allclose(plan.trajectory.x, rebuilt.trajectory.x, atol=1e-6)

    def test_the_compiled_ticks_evaluation_is_the_ticks_problem(self):
        sys = JaxSingleIntegrator()
        cost = QuadraticCost.from_system(sys, Q=np.eye(1), R=0.1 * np.eye(1))
        problem = PlanningProblem(
            sys=sys, tf=1.0, x_start=np.array([0.0]), cost=cost, U=sys.input_box
        )
        values = {"t_start": 3.0, "cost": {"Q": 4.0 * np.eye(1)}}
        x0 = np.array([0.3])
        compiled = self.make_planner(problem).solve_trajectory_from(
            x0, params=values, evaluate=True
        )
        rebuild = TrajectoryOptimizationPlanner(
            problem,
            transcription=DirectCollocationTranscription(
                DirectCollocationOptions(n_steps=5)
            ),
            options=TrajectoryOptimizationOptions(
                backend="numpy", optimizer_options={"maxiter": 50, "ftol": 0.0001}
            ),
        ).solve_trajectory_from(x0, params=values, evaluate=True)
        self.assertEqual(compiled.problem.t_start, 3.0)
        np.testing.assert_allclose(compiled.problem.x_start, x0)
        np.testing.assert_allclose(
            compiled.evaluation.J, rebuild.evaluation.J, rtol=1e-3
        )
        self.assertEqual(problem.t_start, 0.0)  # the caller's problem as it was

    def test_the_compiled_tick_solves_with_ipopt(self):
        pytest.importorskip("cyipopt")
        problem = self.make_problem(0.0)
        planner = TrajectoryOptimizationPlanner(
            problem,
            transcription=DirectCollocationTranscription(
                DirectCollocationOptions(n_steps=5)
            ),
            options=TrajectoryOptimizationOptions(
                backend="jax", method="ipopt", optimizer_options={"max_iter": 100}
            ),
        )
        planner.compile_parametric_program()
        plan = planner.solve_trajectory_from(np.array([0.2]))
        reference = self.make_planner(problem).solve_trajectory_from(np.array([0.2]))
        self.assertTrue(plan.success)
        np.testing.assert_allclose(plan.trajectory.x[:, 0], [0.2], atol=1e-6)
        np.testing.assert_allclose(plan.trajectory.u, reference.trajectory.u, atol=1e-3)

    def test_initial_guess_accepted(self):
        problem = self.make_problem(0.0)
        planner = self.make_planner(problem)
        guess = default_initial_trajectory(
            problem, planner.transcription.initial_guess_time_grid(problem)
        )
        plan = planner.solve_trajectory_from(np.array([0.05]), guess=guess)
        np.testing.assert_allclose(plan.trajectory.x[:, 0], [0.05], atol=1e-05)
        self.assertIsNotNone(planner.last_optimization_result.z)


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
    problem = PlanningProblem(
        sys=sys, x_start=np.array([x_start]), cost=cost, tf=1.0, U=sys.input_box
    )
    return TrajectoryOptimizationPlanner(
        problem,
        n_steps=5,
        transcription="direct_collocation",
        backend="numpy",
        optimizer_options={"maxiter": 50, "ftol": 0.0001},
    )


def integrator_planner(x_start=0.5, goal=2.0, backend="jax"):
    """The single integrator brought to ``goal`` over 1 s on 6 collocation knots (5 periods of 0.2 s)."""
    if backend == "jax":
        configure_jax(enable_x64=True)
    sys = JaxSingleIntegrator() if backend == "jax" else SingleIntegrator()
    cost = QuadraticCost.from_system(
        sys, Q=np.eye(1), R=0.1 * np.eye(1), xbar=np.array([goal])
    )
    problem = PlanningProblem(
        sys=sys, tf=1.0, x_start=np.array([x_start]), cost=cost, U=sys.input_box
    )
    return TrajectoryOptimizationPlanner(
        problem,
        n_steps=5,
        transcription="direct_collocation",
        backend=backend,
        optimizer_options={"maxiter": 100, "ftol": 1e-10},
    )


@pytest.mark.jax
class TestTheDelayedMPC(unittest.TestCase):
    def setUp(self):
        self.planner = integrator_planner()
        self.mpc = ModelPredictiveController(self.planner)
        self.pv = self.mpc.pv

    def test_its_state_is_the_plan_and_its_start_the_default_guess(self):
        mpc, pv = self.mpc, self.pv
        assert isinstance(mpc, DelayedPredictiveController)
        assert mpc.n == pv.n_z and list(mpc.inputs) == ["y"]
        assert list(mpc.outputs) == ["u", "z", "x_ref", "u_ref"]
        np.testing.assert_array_equal(mpc.x0, pv.initial())
        assert mpc.sample_period is None
        clocked = mpc % DT
        assert clocked.sample_period == DT and mpc.sample_period is None
        assert set(mpc.params) >= {"x_start", "t_start", "plant", "cost"}

    def test_one_tick_solves_from_the_measurement_and_applies_the_stored_plan(self):
        mpc, pv = self.mpc, self.pv
        z = pv.initial()
        y = np.array([0.8])
        z_next = mpc.update(z, y, 0.4)
        expected, _ = pv.solve({"x_start": y, "t_start": 0.4}, z0=pv.shift(z, 0.4))
        np.testing.assert_allclose(z_next, expected, atol=1e-12)
        assert z_next[-1] == 0.4
        np.testing.assert_array_equal(mpc.compute_u(z, y, 0.4), pv.command(z, y, 0.4))

    def test_a_failed_solve_keeps_the_old_plan(self):
        mpc, pv = self.mpc, self.pv
        z = pv.initial()

        def failing(params=None, z0=None):
            return z0 + 1.0, {"success": False, "iterations": 0, "cost": np.nan}

        pv.solve = failing
        np.testing.assert_array_equal(mpc.update(z, np.array([0.8]), 0.2), z)

    def test_the_loop_applies_each_plan_one_period_late(self):
        mpc = self.mpc
        plant = self.planner.problem.sys
        loop = (mpc % DT) @ discretize(plant, DT, substeps=4)
        loop.subsystems["plant"].x0 = np.array([0.5])
        run = loop.compute_rollout(5)
        start, end = loop.state_index["controller"]
        origins = run.x[start:end][-1]
        np.testing.assert_allclose(origins, [0.0, 0.0, 0.2, 0.4, 0.6, 0.8])

        # The first period applies the guess's first input; the plant moves toward the goal
        p0, p1 = loop.state_index["plant"]
        assert abs(run.x[p0:p1, -1][0] - 2.0) < abs(0.5 - 2.0)

    def test_a_bound_leaf_reads_its_port(self):
        planner = integrator_planner()
        mpc = ModelPredictiveController(
            planner, inputs={"x_start": "y", "cost.xbar": "r"}
        )
        assert list(mpc.inputs) == ["y", "r"] and mpc.inputs["r"].dim == 1
        z = mpc.pv.initial()
        near = mpc.update(z, np.array([0.5, 0.6]), 0.0)
        far = mpc.update(z, np.array([0.5, 3.0]), 0.0)
        u_near = mpc.compute_u(near, np.array([0.5, 0.6]), 0.0)
        u_far = mpc.compute_u(far, np.array([0.5, 3.0]), 0.0)
        assert u_far[0] > u_near[0]
        with pytest.raises(ValueError, match="its own port"):
            ModelPredictiveController(
                planner, inputs={"x_start": "y", "cost.xbar": "y"}
            )

    def test_lookahead_starts_the_plan_ahead_from_the_predicted_state(self):
        mpc = ModelPredictiveController(self.planner, lookahead=DT)
        pv = mpc.pv
        z, _ = pv.solve({"x_start": np.array([0.5]), "t_start": 0.0})
        z_next = mpc.update(z, np.array([0.5]), 0.0)
        assert z_next[-1] == DT

        # x̂(Δ) = x + Δ u_k, the applied command held (the integrator: exact)
        x_hat = 0.5 + DT * pv.command(z, None, 0.0)[0]
        expected, _ = pv.solve(
            {"x_start": np.array([x_hat]), "t_start": DT}, z0=pv.shift(z, DT)
        )
        np.testing.assert_allclose(z_next, expected, atol=1e-6)

    def test_the_plans_of_a_run_are_read_back_on_their_knots(self):
        mpc = self.mpc
        plant = self.planner.problem.sys
        loop = (mpc % DT) @ discretize(plant, DT, substeps=4)
        run = loop.compute_trajectory(tf=1.0)
        plans = mpc_plans_from_rollout(run, mpc, loop=loop)
        # One plan per solve, labelled by its origin (the tick-0 guess is no solve)
        assert [round(t, 9) for t, _ in plans] == [0.0, 0.2, 0.4, 0.6, 0.8]
        t_k, plan = plans[1]
        np.testing.assert_allclose(plan.t, plan.t[0] + mpc.pv.offsets)
        with pytest.raises(ValueError, match="loop="):
            mpc_plans_from_rollout(run, mpc)


def tree_at(pv, x_start, t_start):
    """The planner's tree with the measured start and the plan's origin, as a tick binds them."""
    return {**pv.params, "x_start": x_start, "t_start": t_start}


@pytest.mark.jax
class TestTheWarmStartedMPC(unittest.TestCase):
    def setUp(self):
        self.planner = integrator_planner()
        self.mpc = ModelPredictiveController(self.planner, delay=False)
        self.pv = self.mpc.pv

    def test_it_is_a_solve_a_command_and_a_delay_holding_the_plan(self):
        mpc, pv = self.mpc, self.pv
        assert isinstance(mpc, WarmStartedPredictiveController)
        assert list(mpc.subsystems) == ["solve", "command", "memory"]
        assert list(mpc.inputs) == ["y"] and list(mpc.outputs) == ["u", "z"]
        assert mpc.n == pv.n_z and mpc.sample_period is None
        np.testing.assert_array_equal(mpc.x0, pv.initial())
        clocked = mpc % DT
        assert clocked.sample_period == DT and mpc.sample_period is None
        np.testing.assert_array_equal(clocked.x0, pv.initial())
        with pytest.raises(TypeError, match="clock"):
            self.planner.problem.compile(mpc, dt=DT)

    def test_the_warm_guess_is_the_last_plan_shifted_with_its_first_state_measured(
        self,
    ):
        pv = self.pv
        z, _ = pv.solve({"x_start": np.array([0.5]), "t_start": 0.0})
        y = np.array([0.9])
        guess = warm_guess(pv, z, y, tree_at(pv, y, DT), pv.initial())
        shifted = pv.shift(z, DT)
        assert guess[0] == 0.9 and guess[-1] == DT
        np.testing.assert_array_equal(guess[1:], shifted[1:])

    def test_the_first_guess_is_the_default_guess_from_the_measured_start(self):
        pv = self.pv
        y = np.array([1.7])
        P = tree_at(pv, y, 0.0)
        guess = warm_guess(pv, pv.initial(), y, P, pv.initial())
        np.testing.assert_array_equal(guess, pv.initial(P))
        assert guess[0] == 1.7

    def test_the_solve_block_solves_from_the_measurement_warm_started(self):
        pv = self.pv
        solve = self.mpc.subsystems["solve"]
        z, _ = pv.solve({"x_start": np.array([0.5]), "t_start": 0.2})
        y = np.array([0.8])
        z_star = solve.compute_z(np.zeros(0), np.concatenate([y, z]), 0.4)
        expected, _ = pv.solve(
            {"x_start": y, "t_start": 0.4},
            z0=warm_guess(pv, z, y, tree_at(pv, y, 0.4), pv.initial()),
        )
        np.testing.assert_allclose(z_star, expected, atol=1e-12)
        assert z_star[-1] == 0.4

    def test_the_loop_is_the_hand_loop_with_one_solve_per_tick(self):
        pv = self.pv
        plant = self.planner.problem.sys
        loop = (self.mpc % DT) @ discretize(plant, DT, substeps=4)
        loop.subsystems["plant"].x0 = np.array([0.5])
        ev = loop.compile()  # the compile's own checks solve; the run is counted
        with mock.patch.object(
            CompiledTrajectoryPlanner,
            "solve",
            autospec=True,
            side_effect=pv.solve.__func__,
        ) as solve:
            run = loop.compute_trajectory(tf=1.0)
            solves = solve.call_count
        del ev

        # The hand loop: solve from x_k warm-started, apply u_k = u*_0 over the period
        x = np.array([0.5])
        z = pv.initial()
        for k in range(5):
            t_k = k * DT
            z, _ = pv.solve(
                {"x_start": x, "t_start": t_k},
                z0=warm_guess(pv, z, x, tree_at(pv, x, t_k), pv.initial()),
            )
            u_k = pv.command(z, x, t_k)
            x = x + DT * u_k  # the integrator under a held input: exact
        p0, p1 = loop.state_index["plant"]
        np.testing.assert_allclose(run.x[p0:p1, -1], x, atol=1e-9)

        # Five ticks, five solves: each signal once per tick, no second solve
        assert solves == 5

        # The plan readers find the boxed MPC; the plant's input is the command
        plans = mpc_plans_from_rollout(run, self.mpc, loop=loop)
        assert [round(t, 9) for t, _ in plans] == [0.0, 0.2, 0.4, 0.6, 0.8]
        assert plant_run(run, loop).u.shape == (1, run.t.size)

    def test_a_failed_solve_keeps_the_last_plan(self):
        pv = self.pv
        solve = self.mpc.subsystems["solve"]
        z, _ = pv.solve({"x_start": np.array([0.5]), "t_start": 0.0})

        def failing(params=None, z0=None):
            return z0 + 1.0, {"success": False, "iterations": 0, "cost": np.nan}

        pv.solve = failing
        z_star = solve.compute_z(np.zeros(0), np.concatenate([[0.8], z]), 0.2)
        np.testing.assert_array_equal(z_star, z)

    def test_the_dual_rate_follower_gets_the_plan_at_the_slow_tick(self):
        follower = PlanFollower(self.planner)
        computer = (self.mpc % DT) >> (follower % 0.05)
        loop = computer @ discretize(self.planner.problem.sys, 0.05)
        run = loop.compute_trajectory(tf=0.6)
        plans = mpc_plans_from_rollout(run, self.mpc, loop=loop)
        assert [round(t, 9) for t, _ in plans] == [0.0, 0.2, 0.4]
        assert plant_run(run, loop).u.shape == (1, run.t.size)


@pytest.mark.optional
@pytest.mark.jax
def test_the_zero_delay_form_traces_on_a_native_planner():
    jax = pytest.importorskip("jax")
    from minilink.planning.trajectory_optimization.shooting import (
        ShootingOptions,
        ShootingTranscription,
    )

    configure_jax(enable_x64=True)
    sys = JaxSingleIntegrator()
    cost = QuadraticCost.from_system(
        sys, Q=np.eye(1), R=0.1 * np.eye(1), xbar=np.array([2.0])
    )
    problem = PlanningProblem(
        sys=sys, tf=1.0, x_start=np.array([0.5]), cost=cost, U=sys.input_box
    )
    planner = TrajectoryOptimizationPlanner(
        problem,
        transcription=ShootingTranscription(ShootingOptions(n_steps=6)),
        backend="jax",
        method="adam",
        iterations=20,
        optimizer_options={"learning_rate": 0.05},
    )
    mpc = ModelPredictiveController(planner, delay=False)
    pe = problem.compile(mpc % DT, backend="jax")
    params = pe.params

    # The closed loop is one traced scan: its J and its gradient in the start
    def J_of(x_start):
        return pe.evaluate({**params, "x_start": x_start}).objective

    J = J_of(params["x_start"])
    g = jax.grad(J_of)(params["x_start"])
    assert np.isfinite(J) and np.all(np.isfinite(g))


@pytest.mark.jax
class TestThePureMPCAndTheFollower(unittest.TestCase):
    def test_the_pure_form_solves_cold_at_each_tick_and_needs_a_clock(self):
        planner = integrator_planner()
        mpc = ModelPredictiveController(planner, delay=False, warm_start=False)
        assert isinstance(mpc, PurePredictiveController) and mpc.n == 0
        pv = mpc.pv
        y = np.array([0.8])
        z, _ = pv.solve({"x_start": y, "t_start": 0.4})
        np.testing.assert_allclose(
            mpc.compute_u(np.zeros(0), y, 0.4), pv.command(z, y, 0.4), atol=1e-12
        )
        with pytest.raises(TypeError, match="clock"):
            planner.problem.compile(mpc, dt=DT)

    def test_dual_rate_applies_the_slow_plan_at_the_fast_rate(self):
        planner = integrator_planner()
        mpc = ModelPredictiveController(planner)
        follower = PlanFollower(planner)
        assert list(follower.inputs) == ["z"] and list(follower.outputs) == ["u"]
        computer = (mpc % DT) >> (follower % 0.05)
        assert computer.sample_period == 0.05
        pe = planner.problem.compile(computer, backend="numpy")
        run = pe.trajectory()
        u = run.get_signal("plant:u")[0]

        # The first slow period applies the guess (zero input); the plan solved at
        # t = 0 arrives at t = 0.2 and is read at the fast ticks
        np.testing.assert_array_equal(u[:4], 0.0)
        assert np.all(u[4:8] > 0.0) and len(set(np.round(u[4:8], 12))) == 4

    def test_a_planner_with_a_start_law_is_refused(self):
        from minilink import Uniform

        planner = integrator_planner()
        problem = planner.problem.replace(x_start=Uniform([0.0], [1.0]))
        planner = TrajectoryOptimizationPlanner(
            problem, n_steps=5, transcription="direct_collocation", backend="jax"
        )
        with pytest.raises(ValueError, match="point start"):
            ModelPredictiveController(planner)


class TestTheMPCOnANumPyPlanner(unittest.TestCase):
    def test_the_numpy_program_is_rebuilt_each_tick(self):
        planner = integrator_planner(backend="numpy")
        mpc = ModelPredictiveController(planner)
        plant = planner.problem.sys
        loop = (mpc % DT) @ discretize(plant, DT, substeps=4)
        loop.subsystems["plant"].x0 = np.array([0.5])
        run = loop.compute_rollout(5)
        p0, p1 = loop.state_index["plant"]
        assert abs(run.x[p0:p1, -1][0] - 2.0) < 0.6


@pytest.mark.jax
class TestTheMPCsSeamFixes(unittest.TestCase):
    def test_an_iteration_limit_stop_is_used_and_a_non_finite_solve_is_not(self):
        planner = integrator_planner()
        mpc = ModelPredictiveController(planner)
        pv = mpc.pv
        z = pv.initial()
        solve = pv.solve

        def capped(params=None, z0=None):
            z_new, status = solve(params, z0)
            return z_new, {**status, "success": False}

        pv.solve = capped
        assert not np.array_equal(mpc.update(z, np.array([0.8]), 0.2), z)

        def diverged(params=None, z0=None):
            return z0 * np.nan, {"success": False, "iterations": 1, "cost": np.nan}

        pv.solve = diverged
        np.testing.assert_array_equal(mpc.update(z, np.array([0.8]), 0.2), z)

    def test_an_unwired_bound_port_reads_the_trees_value(self):
        planner = integrator_planner()
        mpc = ModelPredictiveController(
            planner, inputs={"x_start": "y", "cost.xbar": "r"}
        )
        np.testing.assert_array_equal(mpc.inputs["r"].nominal_value, [2.0])
        with pytest.raises(ValueError, match="subtree"):
            ModelPredictiveController(planner, inputs={"x_start": "y", "cost": "c"})

    def test_the_controllers_tree_is_its_own(self):
        planner = integrator_planner()
        mpc = ModelPredictiveController(planner)
        mpc.params["cost"]["Q"][0, 0] = 9.0
        assert planner.problem.cost.params["Q"][0, 0] == 1.0
        clocked = mpc % DT
        clocked.params["cost"]["Q"][0, 0] = 7.0
        assert mpc.params["cost"]["Q"][0, 0] == 9.0
        with pytest.raises(TypeError, match="already on a clock"):
            clocked % 0.1

    def test_an_unclocked_mpc_in_a_loop_is_refused_naming_the_clock(self):
        planner = integrator_planner()
        mpc = ModelPredictiveController(planner)
        with pytest.raises(TypeError, match="% dt"):
            mpc @ planner.problem.sys

    def test_a_faces_z_seeds_the_offline_solve(self):
        planner = integrator_planner()
        pv = planner.compile()
        z, _ = pv.solve()
        warm = planner.solve(guess=z)
        np.testing.assert_allclose(
            warm.trajectory.x, pv.solution(z).trajectory.x, atol=1e-6
        )

    def test_the_plan_readers_find_the_mpc_in_a_dual_rate_loop(self):
        planner = integrator_planner()
        mpc = ModelPredictiveController(planner)
        computer = (mpc % DT) >> (PlanFollower(planner) % 0.05)
        loop = computer @ discretize(planner.problem.sys, 0.05)
        run = loop.compute_trajectory(tf=0.6)
        plans = mpc_plans_from_rollout(run, mpc, loop=loop)
        assert [round(t, 9) for t, _ in plans] == [0.0, 0.2, 0.4]


def test_plant_run_carries_the_input_the_loop_fed_the_plant():
    from minilink import P, Integrator
    from minilink.planning.trajectory_optimization.mpc import plant_run

    loop = (P(2.0, ports="reference") % 0.1) @ discretize(Integrator(), 0.1, substeps=2)
    run = loop.compute_trajectory(tf=0.5, u=np.array([1.0]))
    plant = plant_run(run, loop)

    # u = 2 (r − y) at each tick, held over its period; x the plant's own slice
    p0, p1 = loop.state_index["plant"]
    np.testing.assert_array_equal(plant.x, run.x[p0:p1])
    ticks = np.floor(run.t / 0.1 + 1e-9).astype(int)
    tick_y = {k: run.x[p0, np.flatnonzero(ticks == k)[0]] for k in set(ticks)}
    np.testing.assert_allclose(plant.u[0], [2.0 * (1.0 - tick_y[k]) for k in ticks])


def test_plant_run_reads_open_ports_boxes_and_a_late_clock():
    from minilink import PD, Gain, NoisyPendulum, P, closed_loop
    from minilink import Integrator
    from minilink.planning.trajectory_optimization.mpc import plant_run

    # An open disturbance port reads its nominal value
    noisy = closed_loop(PD(2.0, 0.1) % 0.1, NoisyPendulum(), noise=True)
    run = noisy.compute_trajectory(tf=0.5)
    assert plant_run(run, noisy).u.shape == (3, run.t.size)

    # A command from a box: u = 3 · 2 (r − y) at each tick
    boxed = closed_loop((P(2.0) % 0.1) >> (Gain([3.0]) % 0.1), Integrator())
    run = boxed.compute_trajectory(tf=0.5, u=np.array([1.0]))
    plant = plant_run(run, boxed)
    np.testing.assert_allclose(plant.u[0], 6.0 * (1.0 - plant.x[0]), atol=1e-12)

    # A clock started late: the tick of a sample is its index
    late = (P(2.0) % 1e-3) @ Integrator()
    run = late.compute_trajectory(tf=1e5 + 0.01, u=np.array([1.0]), t_start=1e5)
    plant = plant_run(run, late)
    np.testing.assert_allclose(plant.u[0], 2.0 * (1.0 - plant.x[0]), atol=1e-12)
