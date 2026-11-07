"""The planner's compiled face and the MPC on it: one architecture, one solve per tick, the delay and the warm start two switches."""

import contextlib
import io
import unittest
from unittest import mock

import matplotlib
import numpy as np

from tests.unittest.planning_helpers import edited
import pytest

matplotlib.use("Agg")

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
    mpc_animation_overlays,
    mpc_plans_from_rollout,
    plant_run,
)
from minilink.core.compile.plan import nested_layout
from minilink.planning.trajectory_optimization.mpc.predictive import (
    PredictiveController,
    warm_guess,
)
from minilink.planning.trajectory_optimization.compiled import (
    CompiledTrajectoryPlanner,
)
from minilink.planning.trajectory_optimization.planner import (
    TrajectoryOptimizationPlanner,
    TrajectoryOptimizationRecord,
)

DT = 0.2  # the MPC's period in the loop tests


class JaxSingleIntegrator(DynamicSystem):
    def __init__(self):
        super().__init__(state_dim=1, input_dim=1, output_dim=1, y_dependencies=())
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
class TestTwoReplanningRoutes(unittest.TestCase):
    """Offline, ``planner.solve(params=)`` with the new start in the tree; in real time, the compiled face."""

    def setUp(self):
        configure_jax(enable_x64=True)

    def make_problem(self, x0=0.0):
        sys = JaxSingleIntegrator()
        cost = QuadraticCost.from_system(
            sys, Q=np.zeros((1, 1)), R=np.eye(1), S=np.zeros((1, 1))
        )
        return PlanningProblem(
            sys=sys,
            horizon=1.0,
            x0=np.array([x0]),
            cost=cost,
            U=sys.input_box,
        )

    def make_planner(self, problem):
        transcription = DirectCollocationTranscription(
            DirectCollocationOptions(n_steps=5)
        )
        return TrajectoryOptimizationPlanner(
            problem,
            transcription=transcription,
            backend="jax",
            options={"maxiter": 50, "ftol": 0.0001},
        )

    def test_the_face_solves_from_a_new_start_and_reads_the_plan_back(self):
        pv = self.make_planner(self.make_problem(0.0)).compile()
        tree = edited(pv.params, {"x0": np.array([0.35])})
        z, status = pv.solve(tree)
        traj = pv.solution(z, params=tree, status=status).trajectory
        self.assertEqual(traj.x.shape, (1, 5))
        self.assertEqual(traj.u.shape, (1, 5))
        np.testing.assert_allclose(traj.x[:, 0], [0.35], atol=1e-05)

        # The next tick from the last plan, a new start
        z2, _ = pv.solve(edited(pv.params, {"x0": np.array([0.2])}), z)
        self.assertEqual(np.asarray(z2).shape, np.asarray(z).shape)

    def test_the_two_routes_agree_on_a_new_start(self):
        problem = self.make_problem(0.0)
        planner = self.make_planner(problem)
        x0 = np.array([0.2])

        offline = planner.solve(params=edited(planner.params, {"x0": x0}))
        pv = planner.compile()
        tree = edited(pv.params, {"x0": x0})
        z, status = pv.solve(tree)
        online = pv.solution(z, params=tree, status=status)

        np.testing.assert_allclose(online.trajectory.x, offline.trajectory.x, atol=1e-6)
        np.testing.assert_allclose(online.trajectory.u, offline.trajectory.u, atol=1e-6)
        np.testing.assert_allclose(offline.trajectory.x[:, 0], x0, atol=1e-05)
        self.assertIs(planner.last_solution, offline)  # the face keeps no solution
        self.assertIsInstance(online.record, TrajectoryOptimizationRecord)
        np.testing.assert_allclose(problem.x0, [0.0])  # the caller's problem as it was

    def test_params_none_or_the_planners_tree(self):
        pv = self.make_planner(self.make_problem(0.0)).compile()
        z_none, _ = pv.solve(None)
        z_tree, _ = pv.solve(pv.params)
        np.testing.assert_allclose(np.asarray(z_none), np.asarray(z_tree), atol=1e-8)
        with self.assertRaises(ValueError):  # an empty tree is no tree: None
            pv.solve({})
        with self.assertRaises(ValueError):
            pv.solve({"foo": 1})

    def test_a_set_and_the_start_time_bind_like_a_rebuild(self):
        from minilink.core.sets import BallSet

        def problem_of(radius, t0=0.0):
            sys = JaxSingleIntegrator()
            cost = QuadraticCost.from_system(
                sys, Q=np.eye(1), R=0.1 * np.eye(1), x_bar=np.array([2.0])
            )
            return PlanningProblem(
                sys=sys,
                horizon=1.0,
                t0=t0,
                x0=np.array([0.0]),
                cost=cost,
                X=BallSet(np.zeros(1), radius),
                U=sys.input_box,
            )

        pv = self.make_planner(problem_of(1.0)).compile()
        self.assertIn("X", pv.params)  # a set read as margins is a leaf
        tree = edited(pv.params, {"X": {"radius": 0.5}, "t0": 3.0})
        z, status = pv.solve(tree)
        plan = pv.solution(z, params=tree, status=status)
        rebuilt = self.make_planner(problem_of(0.5, t0=3.0)).solve()
        np.testing.assert_allclose(plan.trajectory.x, rebuilt.trajectory.x, atol=1e-6)
        np.testing.assert_allclose(plan.trajectory.t, 3.0 + np.linspace(0.0, 1.0, 5))
        self.assertLessEqual(plan.trajectory.x.max(), 0.5 + 1e-6)

        # A moved ball takes its own bounds: none of the compile-time box stays
        def moved(center):
            problem = problem_of(1.0)
            problem.X = BallSet(np.array([center]), 1.0)
            return problem

        start = np.array([1.5])
        tree = edited(pv.params, {"X": {"center": [1.0]}, "x0": start})
        z, status = pv.solve(tree)
        plan = pv.solution(z, params=tree, status=status)
        rebuilt = self.make_planner(moved(1.0)).solve(
            params=edited(self.make_planner(moved(1.0)).params, {"x0": start})
        )
        self.assertTrue(plan.success)
        np.testing.assert_allclose(plan.trajectory.x, rebuilt.trajectory.x, atol=1e-6)

    def test_the_compiled_face_solves_with_ipopt(self):
        pytest.importorskip("cyipopt")
        planner = TrajectoryOptimizationPlanner(
            self.make_problem(0.0),
            transcription=DirectCollocationTranscription(
                DirectCollocationOptions(n_steps=5)
            ),
            backend="jax",
            method="ipopt",
            options={"max_iter": 100},
        )
        pv = planner.compile()
        tree = edited(pv.params, {"x0": np.array([0.2])})
        z, status = pv.solve(tree)
        plan = pv.solution(z, params=tree, status=status)
        self.assertTrue(plan.success)
        np.testing.assert_allclose(plan.trajectory.x[:, 0], [0.2], atol=1e-6)

        # J = ∫ u² dt with no state cost and a free end: u* = 0, the start held
        np.testing.assert_allclose(plan.trajectory.u, 0.0, atol=1e-6)
        np.testing.assert_allclose(plan.trajectory.x, 0.2, atol=1e-6)

    def test_a_guess_seeds_the_face(self):
        problem = self.make_problem(0.0)
        pv = self.make_planner(problem).compile()
        guess = default_initial_trajectory(
            problem, pv.planner.transcription.initial_guess_time_grid(problem)
        )
        tree = edited(pv.params, {"x0": np.array([0.05])})
        z, status = pv.solve(tree, pv.pack(guess, tree))
        plan = pv.solution(z, params=tree, status=status)
        np.testing.assert_allclose(plan.trajectory.x[:, 0], [0.05], atol=1e-05)
        self.assertIsNotNone(plan.record.optimization_result.z)


class SingleIntegrator(DynamicSystem):
    def __init__(self):
        super().__init__(state_dim=1, input_dim=1, output_dim=1, y_dependencies=())
        self.state.lower_bound = np.array([-10.0])
        self.state.upper_bound = np.array([10.0])
        self.inputs["u"].lower_bound = np.array([-10.0])
        self.inputs["u"].upper_bound = np.array([10.0])

    def f(self, x, u, t=0, params=None):
        return np.array([u[0]])

    def h(self, x, u, t=0, params=None):
        return x


def _make_numpy_planner(x0=0.0):
    sys = SingleIntegrator()
    cost = QuadraticCost.from_system(
        sys, Q=np.zeros((1, 1)), R=np.eye(1), S=np.zeros((1, 1))
    )
    problem = PlanningProblem(
        sys=sys, x0=np.array([x0]), cost=cost, horizon=1.0, U=sys.input_box
    )
    return TrajectoryOptimizationPlanner(
        problem,
        n_steps=5,
        transcription="direct_collocation",
        backend="numpy",
        options={"maxiter": 50, "ftol": 0.0001},
    )


def integrator_planner(x0=0.5, goal=2.0, backend="jax"):
    """The single integrator brought to ``goal`` over 1 s on 6 collocation knots (5 periods of 0.2 s)."""
    if backend == "jax":
        configure_jax(enable_x64=True)
    sys = JaxSingleIntegrator() if backend == "jax" else SingleIntegrator()
    cost = QuadraticCost.from_system(
        sys, Q=np.eye(1), R=0.1 * np.eye(1), x_bar=np.array([goal])
    )
    problem = PlanningProblem(
        sys=sys, horizon=1.0, x0=np.array([x0]), cost=cost, U=sys.input_box
    )
    return TrajectoryOptimizationPlanner(
        problem,
        n_steps=5,
        transcription="direct_collocation",
        backend=backend,
        options={"maxiter": 100, "ftol": 1e-10},
    )


@pytest.mark.jax
class TestTheDelayedMPC(unittest.TestCase):
    def setUp(self):
        self.planner = integrator_planner()
        self.mpc = ModelPredictiveController(self.planner, delay=True)
        self.pv = self.mpc.pv

    def test_the_delay_is_a_visible_unit_delay_before_the_command(self):
        mpc, pv = self.mpc, self.pv
        assert isinstance(mpc, PredictiveController)
        assert list(mpc.subsystems) == ["solve", "command", "memory", "delay"]
        assert list(mpc.inputs) == ["y"]
        assert list(mpc.outputs) == ["u", "z", "x_ref", "u_ref"]
        assert mpc.connections["command"]["z"] == ("delay", "y")
        np.testing.assert_array_equal(mpc.subsystems["delay"].x0, pv.initial())
        assert mpc.sample_period is None
        clocked = mpc % DT
        assert clocked.sample_period == DT and mpc.sample_period is None
        assert set(mpc.params["solve"]) >= {"x0", "t0", "plant", "cost"}

    def test_the_loop_applies_each_plan_one_period_late(self):
        mpc = self.mpc
        plant = self.planner.problem.sys
        loop = (mpc % DT) @ plant
        loop.subsystems["plant"].x0 = np.array([0.5])
        run = loop.compute_rollout(5, dt=DT / 4)
        delay_slice, _ = nested_layout(loop)[("controller", "delay")]
        origins = run.x[delay_slice][-1]
        np.testing.assert_allclose(origins, [0.0, 0.0, 0.2, 0.4, 0.6, 0.8])

        # The first period applies the guess's first input; the plant moves toward the goal
        p0, p1 = loop.state_index["plant"]
        assert abs(run.x[p0:p1, -1][0] - 2.0) < abs(0.5 - 2.0)

    def test_a_bound_leaf_reads_its_port(self):
        planner = integrator_planner()
        mpc = ModelPredictiveController(planner, inputs={"x0": "y", "cost.x_bar": "r"})
        assert list(mpc.inputs) == ["y", "r"] and mpc.inputs["r"].dim == 1
        pv = mpc.pv
        solve = mpc.subsystems["solve"]
        z = pv.initial()

        def command_at(r):
            z_star = solve.compute_z(np.zeros(0), np.concatenate([[0.5, r], z]), 0.0)
            return pv.command(z_star, None, 0.0)

        assert command_at(3.0)[0] > command_at(0.6)[0]
        with pytest.raises(ValueError, match="its own port"):
            ModelPredictiveController(planner, inputs={"x0": "y", "cost.x_bar": "y"})

    def test_the_face_predicts_the_start_for_a_users_compensation(self):
        pv = self.pv
        x, u = np.array([0.5]), np.array([1.5])

        # x̂(t + Δ) = x + Δ u under a held input: the integrator is exact
        x_hat = pv.predict_start(x, u, 0.0, DT)
        np.testing.assert_allclose(x_hat, x + DT * u, atol=1e-12)

    def test_the_plans_of_a_run_are_read_back_on_their_knots(self):
        mpc = self.mpc
        plant = self.planner.problem.sys
        loop = (mpc % DT) @ plant
        run = loop.compute_trajectory(tf=1.0, dt=DT / 4)
        plans = mpc_plans_from_rollout(run, mpc, loop=loop)
        # One plan per solve, labelled by its origin
        assert [round(t, 9) for t, _ in plans] == [0.0, 0.2, 0.4, 0.6, 0.8]
        t_k, plan = plans[1]
        np.testing.assert_allclose(plan.t, plan.t[0] + mpc.pv.offsets)
        with pytest.raises(ValueError, match="loop="):
            mpc_plans_from_rollout(run, mpc)


def tree_at(pv, x0, t0):
    """The planner's tree with the measured start and the plan's origin, as a tick binds them."""
    return {**pv.params, "x0": x0, "t0": t0}


@pytest.mark.jax
class TestTheWarmStartedMPC(unittest.TestCase):
    def setUp(self):
        self.planner = integrator_planner()
        self.mpc = ModelPredictiveController(self.planner, delay=False)
        self.pv = self.mpc.pv

    def test_it_is_a_solve_a_command_and_a_memory_holding_the_plan(self):
        mpc, pv = self.mpc, self.pv
        assert isinstance(mpc, PredictiveController)
        assert list(mpc.subsystems) == ["solve", "command", "memory"]
        assert list(mpc.inputs) == ["y"]
        assert list(mpc.outputs) == ["u", "z", "x_ref", "u_ref"]
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
        z, _ = pv.solve(edited(pv.params, {"x0": np.array([0.5]), "t0": 0.0}))
        y = np.array([0.9])
        guess = warm_guess(pv, z, y, tree_at(pv, y, DT), pv.initial())
        shifted = pv.shift(z, DT)
        assert guess[0] == 0.9 and guess[-1] == DT
        np.testing.assert_array_equal(guess[1:], shifted[1:])
        np.testing.assert_array_equal(guess, pv.shift(z, DT, x0=y))

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
        z, _ = pv.solve(edited(pv.params, {"x0": np.array([0.5]), "t0": 0.2}))
        y = np.array([0.8])
        z_star = solve.compute_z(np.zeros(0), np.concatenate([y, z]), 0.4)
        expected, _ = pv.solve(
            edited(pv.params, {"x0": y, "t0": 0.4}),
            z0=warm_guess(pv, z, y, tree_at(pv, y, 0.4), pv.initial()),
        )
        np.testing.assert_allclose(z_star, expected, atol=1e-12)
        assert z_star[-1] == 0.4

    def test_the_loop_is_the_hand_loop_with_one_solve_per_tick(self):
        pv = self.pv
        plant = self.planner.problem.sys
        loop = (self.mpc % DT) @ plant
        loop.subsystems["plant"].x0 = np.array([0.5])
        ev = loop.compile(
            substeps=4
        )  # the compile's own checks solve; the run is counted
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
                edited(pv.params, {"x0": x, "t0": t_k}),
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

    def test_the_read_backs_solve_nothing_again(self):
        # The run records the plant's input: the plant's run, the plan readers, the
        # overlays and the animation read the record, so a run solves once per tick
        plant = self.planner.problem.sys
        loop = (self.mpc % DT) @ plant
        loop.subsystems["plant"].x0 = np.array([0.5])
        run = loop.compute_trajectory(tf=1.0, dt=DT / 4)
        assert run.has_signal("plant:u") and "plant" in run.roles
        with mock.patch.object(
            CompiledTrajectoryPlanner,
            "solve",
            autospec=True,
            side_effect=self.pv.solve.__func__,
        ) as solve:
            plant_run(run, loop)
            mpc_plans_from_rollout(run, self.mpc, loop=loop)
            mpc_animation_overlays(run, self.mpc, loop=loop)
            loop.animate(run, show=False, html=False)
            solves = solve.call_count
        assert solves == 0

    def test_a_planner_with_a_start_set_is_refused(self):
        from minilink.core.sets import BoxSet

        problem = self.planner.problem
        boxed = PlanningProblem(
            sys=problem.sys,
            cost=problem.cost,
            horizon=problem.horizon,
            x0=np.array([0.5]),
            X0=BoxSet(np.array([0.4]), np.array([0.6])),
            U=problem.U,
        )
        planner = TrajectoryOptimizationPlanner(
            boxed, n_steps=5, transcription="direct_collocation", backend="jax"
        )
        with pytest.raises(ValueError, match="point start"):
            ModelPredictiveController(planner, delay=False)

    def test_a_failed_solve_keeps_the_last_plan(self):
        pv = self.pv
        solve = self.mpc.subsystems["solve"]
        z, _ = pv.solve(edited(pv.params, {"x0": np.array([0.5]), "t0": 0.0}))

        def failing(params=None, z0=None):
            return z0 + 1.0, {"success": False, "iterations": 0, "cost": np.nan}

        pv.solve = failing
        z_star = solve.compute_z(np.zeros(0), np.concatenate([[0.8], z]), 0.2)
        np.testing.assert_array_equal(z_star, z)

    def test_the_dual_rate_follower_gets_the_plan_at_the_slow_tick(self):
        follower = PlanFollower(self.planner)
        computer = (self.mpc % DT) >> (follower % 0.05)
        loop = computer @ self.planner.problem.sys
        run = loop.compute_trajectory(tf=0.6, dt=0.05)
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
        sys, Q=np.eye(1), R=0.1 * np.eye(1), x_bar=np.array([2.0])
    )
    problem = PlanningProblem(
        sys=sys, horizon=1.0, x0=np.array([0.5]), cost=cost, U=sys.input_box
    )
    planner = TrajectoryOptimizationPlanner(
        problem,
        transcription=ShootingTranscription(ShootingOptions(n_steps=6)),
        backend="jax",
        method="adam",
        n_iterations=20,
        learning_rate=0.05,
    )
    mpc = ModelPredictiveController(planner, delay=False)
    pe = problem.compile(mpc % DT, backend="jax")
    params = pe.params

    # The closed loop is one traced scan: its J and its gradient in the start
    def J_of(x0):
        return pe.evaluate({**params, "x0": x0}).objective

    J = J_of(params["x0"])
    g = jax.grad(J_of)(params["x0"])
    assert np.isfinite(J) and np.all(np.isfinite(g))

    # A verbose block prints on the host only: the traced run stays silent and traceable
    talkative = ModelPredictiveController(planner, delay=False, verbose=True)
    pe_v = problem.compile(talkative % DT, backend="jax")
    with contextlib.redirect_stdout(io.StringIO()) as out:
        J_v = pe_v.evaluate(pe_v.params).objective
    assert np.isclose(float(J_v), float(J)) and out.getvalue() == ""


@pytest.mark.jax
class TestThePureMPCAndTheFollower(unittest.TestCase):
    def test_the_cold_form_solves_from_the_default_guess_at_each_tick(self):
        planner = integrator_planner()
        mpc = ModelPredictiveController(planner, warm_start=False)
        assert list(mpc.subsystems) == ["solve", "command", "memory"]
        pv = mpc.pv
        solve = mpc.subsystems["solve"]
        y = np.array([0.8])
        z_prev, _ = pv.solve(edited(pv.params, {"x0": np.array([0.5]), "t0": 0.2}))
        P, z0 = solve.solve_inputs(np.zeros(0), np.concatenate([y, z_prev]), 0.4)
        np.testing.assert_array_equal(z0, pv.initial(tree_at(pv, y, 0.4)))
        with pytest.raises(TypeError, match="clock"):
            planner.problem.compile(mpc, dt=DT)

    def test_dual_rate_applies_the_slow_plan_at_the_fast_rate(self):
        from minilink.blocks.step import UnitDelay

        planner = integrator_planner()
        mpc = ModelPredictiveController(planner)
        follower = PlanFollower(planner)
        assert list(follower.inputs) == ["z"] and list(follower.outputs) == ["u"]

        # The plan one slow period late: the user's unit delay between the two rates
        late = UnitDelay(dim=mpc.pv.n_z).rename_ports(u="z")
        late.x0 = mpc.pv.initial()
        computer = (mpc % DT) >> (late % DT) >> (follower % 0.05)
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
        problem = planner.problem.replace(x0=Uniform([0.0], [1.0]))
        planner = TrajectoryOptimizationPlanner(
            problem, n_steps=5, transcription="direct_collocation", backend="jax"
        )
        with pytest.raises(ValueError, match="point start"):
            ModelPredictiveController(planner)


class TestTheMPCOnANumPyPlanner(unittest.TestCase):
    def test_a_delayed_mpc_compiles_with_no_solve(self):
        # The compile's shape probe does not run the solve block
        planner = integrator_planner(backend="numpy")
        mpc = ModelPredictiveController(planner, delay=True)
        loop = (mpc % DT) @ planner.problem.sys
        with mock.patch.object(
            CompiledTrajectoryPlanner, "solve", autospec=True
        ) as solve:
            loop.compile(substeps=1)
        assert solve.call_count == 0

    def test_the_face_keeps_the_tree_it_was_compiled_with(self):
        # planner.compile() is a build: a later edit of the problem is not the face's
        planner = integrator_planner(backend="numpy")
        pv = planner.compile()
        planner.problem.params = {**planner.problem.params, "t0": 5.0}
        assert pv.params["t0"] == 0.0 and pv.initial()[-1] == 0.0
        z, _ = pv.solve()
        assert z[-1] == 0.0

        # A tree at other numbers is the compiled one edited, not the edited problem's
        partial = edited(pv.params, {"x0": np.array([0.3])})
        z_partial, _ = pv.solve(partial)
        assert z_partial[-1] == 0.0 and pv.initial(partial)[-1] == 0.0
        assert pv.solution(z_partial, partial).trajectory.t[0] == 0.0

    def test_the_numpy_program_is_rebuilt_each_tick(self):
        planner = integrator_planner(backend="numpy")
        mpc = ModelPredictiveController(planner, delay=True)
        plant = planner.problem.sys
        loop = (mpc % DT) @ plant
        loop.subsystems["plant"].x0 = np.array([0.5])
        run = loop.compute_rollout(5, dt=DT / 4)
        p0, p1 = loop.state_index["plant"]
        assert abs(run.x[p0:p1, -1][0] - 2.0) < 0.6

    def test_verbose_prints_one_line_per_solve_and_is_off_by_default(self):
        def printed(verbose):
            planner = integrator_planner(backend="numpy")
            mpc = ModelPredictiveController(planner, delay=False, verbose=verbose)
            loop = (mpc % DT) @ planner.problem.sys
            loop.subsystems["plant"].x0 = np.array([0.5])
            out = io.StringIO()
            with contextlib.redirect_stdout(out):
                loop.compute_trajectory(tf=1.0, dt=DT / 4)
            return [line for line in out.getvalue().splitlines() if "MPC @" in line]

        # Five ticks, one line each, at the ticks' times
        lines = printed(True)
        assert [line.split()[2] for line in lines] == [
            "t=0.00s",
            "t=0.20s",
            "t=0.40s",
            "t=0.60s",
            "t=0.80s",
        ]
        assert printed(False) == []


@pytest.mark.jax
class TestTheMPCsSeamFixes(unittest.TestCase):
    def test_an_iteration_limit_stop_is_used_and_a_non_finite_solve_is_not(self):
        planner = integrator_planner()
        mpc = ModelPredictiveController(planner)
        pv = mpc.pv
        block = mpc.subsystems["solve"]
        z = pv.initial()
        u = np.concatenate([[0.8], z])
        solve = pv.solve

        def capped(params=None, z0=None):
            z_new, status = solve(params, z0)
            return z_new, {**status, "success": False}

        pv.solve = capped
        assert not np.array_equal(block.compute_z(np.zeros(0), u, 0.2), z)

        def diverged(params=None, z0=None):
            return z0 * np.nan, {"success": False, "iterations": 1, "cost": np.nan}

        pv.solve = diverged
        np.testing.assert_array_equal(block.compute_z(np.zeros(0), u, 0.2), z)

    def test_an_unwired_bound_port_reads_the_trees_value(self):
        planner = integrator_planner()
        mpc = ModelPredictiveController(planner, inputs={"x0": "y", "cost.x_bar": "r"})
        np.testing.assert_array_equal(mpc.inputs["r"].nominal_value, [2.0])
        with pytest.raises(ValueError, match="subtree"):
            ModelPredictiveController(planner, inputs={"x0": "y", "cost": "c"})

    def test_the_controllers_tree_is_its_own(self):
        planner = integrator_planner()
        mpc = ModelPredictiveController(planner)
        mpc.params["solve"]["cost"]["Q"][0, 0] = 9.0
        assert planner.problem.cost.params["Q"][0, 0] == 1.0
        with pytest.raises(TypeError, match="already holds blocks on a clock"):
            (mpc % DT) % 0.1

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
        loop = computer @ planner.problem.sys
        run = loop.compute_trajectory(tf=0.6, dt=0.05)
        plans = mpc_plans_from_rollout(run, mpc, loop=loop)
        assert [round(t, 9) for t, _ in plans] == [0.0, 0.2, 0.4]


def test_plant_run_carries_the_input_the_loop_fed_the_plant():
    from minilink import P, Integrator
    from minilink.planning.trajectory_optimization.mpc import plant_run

    loop = (P(2.0, ports="reference") % 0.1) @ Integrator()
    run = loop.compute_trajectory(tf=0.5, u=np.array([1.0]), dt=0.1 / 2)
    plant = plant_run(run, loop)

    # u = 2 (r − y) at each tick, held over its period and the last over the end
    # sample; x the plant's own slice
    p0, p1 = loop.state_index["plant"]
    np.testing.assert_array_equal(plant.x, run.x[p0:p1])
    ticks = np.minimum(np.floor(run.t / 0.1 + 1e-9).astype(int), 4)
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
    run = boxed.compute_trajectory(tf=0.5, u=np.array([1.0]), dt=0.1)
    plant = plant_run(run, boxed)
    np.testing.assert_allclose(
        plant.u[0, :-1], 6.0 * (1.0 - plant.x[0, :-1]), atol=1e-12
    )
    assert plant.u[0, -1] == plant.u[0, -2]  # the end sample holds the last tick's

    # A clock started late: the tick of a sample is its index
    late = (P(2.0) % 1e-3) @ Integrator()
    run = late.compute_trajectory(tf=1e5 + 0.01, u=np.array([1.0]), t0=1e5, dt=1e-3)
    plant = plant_run(run, late)
    np.testing.assert_allclose(
        plant.u[0, :-1], 2.0 * (1.0 - plant.x[0, :-1]), atol=1e-12
    )
    assert plant.u[0, -1] == plant.u[0, -2]  # the end sample holds the last tick's


@pytest.mark.optional
@pytest.mark.jax
def test_the_traced_default_guess_follows_the_traced_start():
    # The review's D-4: a traced loop's first tick starts from the host's guess
    jax = pytest.importorskip("jax")
    configure_jax(enable_x64=True)
    pv = integrator_planner().compile()
    P = tree_at(pv, np.array([1.3]), 0.4)
    host = pv.initial(P)
    traced = jax.jit(lambda x, t: pv.initial(edited(pv.params, {"x0": x, "t0": t})))(
        np.array([1.3]), 0.4
    )
    np.testing.assert_allclose(np.asarray(traced), host, rtol=0, atol=1e-12)
    assert host[0] == 1.3 and host[-1] == 0.4
