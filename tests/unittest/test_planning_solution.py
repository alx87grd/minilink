"""One result for every planner: the policy reproduces the trajectory, and the evaluation is shared."""

import numpy as np
import pytest

from minilink import DynamicProgrammingPlanner, Pendulum, PlanningProblem, QuadraticCost
from minilink.blocks import TrajectorySource
from minilink.core.trajectory import Trajectory
from minilink.dynamics.catalog.equations import DoubleIntegrator
from minilink.planning import (
    MonteCarloEvaluator,
    PlanningSolution,
    StochasticPlanningProblem,
    Uniform,
)
from minilink.planning.evaluation import control_law, nominal_trajectory
from minilink.planning.reinforcement_learning import TabularLearningPlanner
from minilink.planning.search.extenders import KinodynamicExtender
from minilink.planning.search.rrt import RRTOptions, RRTPlanner
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
from tests.unittest.planning_helpers import make_holonomic_obstacle_problem


def pendulum_problem(stochastic=False):
    plant = Pendulum()
    plant.inputs["u"].lower_bound = np.array([-5.0])
    plant.inputs["u"].upper_bound = np.array([5.0])
    cost = QuadraticCost.from_system(
        plant, Q=np.eye(2), R=0.1 * np.eye(1), S=np.eye(2), xbar=np.array([np.pi, 0.0])
    )
    if stochastic:
        return StochasticPlanningProblem(
            plant, cost=cost, tf=2.0, x0_distribution=Uniform([-0.2, -0.2], [0.2, 0.2])
        )
    return PlanningProblem(
        plant, x_start=[0.0, 0.0], x_goal=[np.pi, 0.0], cost=cost, tf=2.0
    )


def move_problem(stochastic=False):
    """A double integrator brought to rest at the origin in two seconds."""
    plant = DoubleIntegrator()
    plant.inputs["u"].lower_bound = np.array([-2.0])
    plant.inputs["u"].upper_bound = np.array([2.0])
    cost = QuadraticCost.from_system(plant, Q=np.eye(2), R=0.1 * np.eye(1))
    if stochastic:
        return StochasticPlanningProblem(
            plant,
            cost=cost,
            tf=2.0,
            x_goal=[0.0, 0.0],
            x0_distribution=Uniform([0.8, -0.2], [1.2, 0.2]),
        )
    return PlanningProblem(
        plant, x_start=[1.0, 0.0], x_goal=[0.0, 0.0], cost=cost, tf=2.0
    )


def trajopt(problem, transcription, n_steps=10):
    return TrajectoryOptimizationPlanner(
        problem,
        transcription=transcription(n_steps),
        options=TrajectoryOptimizationOptions(
            compile_backend="numpy", optimizer_options={"maxiter": 200, "ftol": 1e-10}
        ),
    )


def collocation(n_steps):
    return DirectCollocationTranscription(DirectCollocationOptions(n_steps=n_steps))


def shooting(n_steps):
    return MultipleShootingTranscription(MultipleShootingOptions(n_steps=n_steps))


def replay(problem, solution, solver="rk4_fixedsteps"):
    """The open-loop policy driving the plant in series, sampled on the solution's grid."""
    plant = problem.sys
    planned = solution.trajectory
    plant.x0 = np.asarray(planned.x[:, 0])
    loop = solution.policy >> plant
    dt = float(planned.t[1] - planned.t[0])
    traj = loop.compute_trajectory(
        tf=float(planned.tf), dt=dt, solver=solver, verbose=False
    )
    driven = loop.trajectory_of(plant, traj)
    n = planned.n_samples  # the simulator may append one rounding sample past tf
    np.testing.assert_allclose(driven.t[:n], planned.t, atol=1e-9)
    return driven.x[:, :n]


def held_input_rollout(plant, law, t, x0):
    """The law sampled at each grid time and held over the step, one RK4 step per period."""
    evaluator = plant.compile(backend="numpy", verbose=False)
    x = np.zeros((int(plant.n), t.size))
    x[:, 0] = x0
    for k in range(t.size - 1):
        u = law(x[:, k], t[k])
        x[:, k + 1] = evaluator.rk4_step(x[:, k], u, t[k], t[k + 1] - t[k])
    return x


# --- open-loop planners: the source block replays the schedule ---


def test_multiple_shooting_policy_reproduces_its_trajectory():
    problem = move_problem()
    solution = trajopt(problem, shooting).solve()
    assert isinstance(solution, PlanningSolution) and solution.success
    assert solution.open_loop and isinstance(solution.policy, TrajectorySource)
    assert solution.trajectory is not None and solution.evaluation is None
    np.testing.assert_allclose(
        replay(problem, solution), solution.trajectory.x, atol=1e-8
    )


def test_collocation_policy_reproduces_its_trajectory_to_its_own_discretization():
    problem = move_problem()
    gaps = []
    for n_steps in (8, 32):
        solution = trajopt(problem, collocation, n_steps).solve()
        assert solution.success
        gaps.append(
            float(np.max(np.abs(replay(problem, solution) - solution.trajectory.x)))
        )
    assert gaps[1] < 0.1 * gaps[0]  # the trapezoidal defect vanishes with the grid


def test_trajopt_evaluation_is_one_trial_deterministic_and_n_trials_stochastic():
    solution = trajopt(move_problem(), collocation, 128).solve(evaluate=True)
    assert solution.evaluation.n_trials == 1 and not solution.evaluation.failed[0]
    assert str(solution.evaluation).startswith("J = ")
    # the replayed schedule scores the optimizer's cost up to the discretization gap
    assert (
        abs(solution.evaluation.mean - solution.solver.cost)
        < 0.05 * solution.solver.cost
    )

    with pytest.warns(UserWarning):  # a deterministic planner on a stochastic problem
        planner = trajopt(move_problem(stochastic=True), shooting)
    solution = planner.solve(evaluate=True, n_trials=5)
    assert solution.evaluation.n_trials == 5
    assert "open loop" in str(solution) and "feasible" in str(solution.solver)


def test_rrt_policy_holds_each_control_and_reproduces_the_path():
    """The held source sampled on the grid replays the tree path (the evaluator's contract)."""
    problem, _ = make_holonomic_obstacle_problem()
    planner = RRTPlanner(
        problem,
        extender=KinodynamicExtender(
            controls=[
                np.array(c, dtype=float) for c in ((1, 0), (0, 1), (-1, 0), (0, -1))
            ],
            horizon=0.6,
            n_substeps=6,
        ),
        options=RRTOptions(seed=0, goal_tolerance=0.5, max_nodes=4000),
    )
    solution = planner.solve()
    assert solution.success and solution.solver.reached_goal
    assert solution.open_loop and solution.policy.interpolation == "previous"
    assert solution.evaluation is None  # the problem declares no cost
    planned = solution.trajectory
    law = control_law(solution.policy, planned.t, "numpy")
    x_hand = held_input_rollout(problem.sys, law, planned.t, planned.x[:, 0])
    np.testing.assert_allclose(x_hand, planned.x, atol=1e-8)


# --- feedback planners: the rollout is the held-input closed loop on the grid ---


def double_integrator_problem():
    plant = DoubleIntegrator()
    plant.state.lower_bound = np.array([-2.0, -2.0])
    plant.state.upper_bound = np.array([2.0, 2.0])
    plant.inputs["u"].lower_bound = np.array([-1.0])
    plant.inputs["u"].upper_bound = np.array([1.0])
    cost = QuadraticCost.from_system(plant, Q=np.diag([1.0, 0.1]), R=np.diag([0.01]))
    return PlanningProblem(
        plant,
        x_start=[1.0, 0.0],
        cost=cost,
        tf=np.inf,
        infeasible_cost=20.0,
    )


@pytest.mark.parametrize("make", ["dp", "tabular"])
def test_grid_policies_roll_out_as_held_input_closed_loops(make):
    problem = double_integrator_problem()
    grid = dict(x_grid=(11, 11), u_grid=(3,), dt=0.2)
    if make == "dp":
        planner = DynamicProgrammingPlanner(problem, alpha=0.9, tol=1e-6, **grid)
        solution = planner.solve(evaluate=True)
    else:
        planner = TabularLearningPlanner(
            problem, alpha=0.9, integrator="euler", seed=0, **grid
        )
        solution = planner.solve(episodes=300, evaluate=True)
    assert not solution.open_loop and solution.evaluation.n_trials == 1
    planned = solution.trajectory

    def law(x, t):
        return solution.policy.action(x)

    x_hand = held_input_rollout(problem.sys, law, planned.t, problem.x_start)
    np.testing.assert_allclose(planned.x, x_hand, atol=1e-10)
    # the same rollout is what plot_solution draws, and the cost-to-go is the planner's table
    np.testing.assert_array_equal(
        planner.solution_trajectory().x, solution.trajectory.x
    )
    assert solution.cost_to_go(problem.x_start) == planner.value_at(problem.x_start)
    assert str(solution.evaluation).startswith("J = ")


def test_one_evaluator_scores_an_open_loop_and_a_feedback_law_on_the_same_draws():
    problem = pendulum_problem(stochastic=True)
    with pytest.warns(UserWarning):
        open_loop = trajopt(problem, shooting).solve()
    feedback = DynamicProgrammingPlanner(
        problem.nominal(), x_grid=(21, 21), u_grid=(3,), dt=0.1, alpha=0.95, tol=0.1
    ).solve()
    evaluator = MonteCarloEvaluator(
        problem, dt=0.1, n_trials=4, backend="numpy", seed=3
    )
    scored = [evaluator.evaluate(s.policy) for s in (open_loop, feedback)]
    np.testing.assert_array_equal(scored[0].x0, scored[1].x0)
    assert all(np.all(np.isfinite(report.J)) for report in scored)


def test_nominal_trajectory_verb_matches_the_rl_planner_rollout():
    pytest.importorskip("jax")
    from minilink.planning.reinforcement_learning import ReinforcementLearningPlanner

    plant = Pendulum()
    plant.inputs["u"].lower_bound = np.array([-4.0])
    plant.inputs["u"].upper_bound = np.array([4.0])
    plant.state.lower_bound = np.array([-np.pi, -8.0])
    plant.state.upper_bound = np.array([np.pi, 8.0])
    cost = QuadraticCost.from_system(plant, Q=np.eye(2), R=0.01 * np.eye(1))
    problem = StochasticPlanningProblem(
        plant, cost=cost, tf=np.inf, x0_distribution=Uniform([-0.5, -0.5], [0.5, 0.5])
    )
    planner = ReinforcementLearningPlanner(
        problem,
        dt=0.1,
        hidden=(8, 8),
        n_envs=4,
        n_steps=16,
        batch_size=32,
        gamma=0.9,
        verbose=0,
    )
    solution = planner.solve(timesteps=64, evaluate=True, n_trials=3)
    verb = nominal_trajectory(problem, solution.policy, dt=0.1, tf=planner.env.tf)
    np.testing.assert_allclose(verb.x, solution.trajectory.x, atol=1e-10)
    assert isinstance(verb, Trajectory) and solution.evaluation.n_trials == 3
