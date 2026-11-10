"""One result for every planner: the controller handed out is the one scored and drawn, and the evaluation is shared."""

import numpy as np
import pytest

from minilink import DynamicProgrammingPlanner, Pendulum, PlanningProblem, QuadraticCost
from minilink.blocks import TrajectorySource
from minilink.core.sets import SingletonSet
from minilink.dynamics.catalog.equations import DoubleIntegrator
from minilink.learning.reinforcement_learning import TabularLearningPlanner
from minilink.planning import (
    PlanningSolution,
    Uniform,
    evaluate,
)
from minilink.planning.search.extenders import KinodynamicExtender
from minilink.planning.search.rrt import RRTPlanner
from minilink.planning.trajectory_optimization.direct_collocation import (
    DirectCollocationTranscription,
)
from minilink.planning.trajectory_optimization.multiple_shooting import (
    MultipleShootingTranscription,
)
from minilink.planning.trajectory_optimization.planner import (
    TrajectoryOptimizationPlanner,
)
from tests.unittest.planning_helpers import make_holonomic_obstacle_problem


def pendulum_problem(stochastic=False):
    plant = Pendulum()
    plant.inputs["u"].lower_bound = np.array([-5.0])
    plant.inputs["u"].upper_bound = np.array([5.0])
    cost = QuadraticCost.from_system(
        plant, Q=np.eye(2), R=0.1 * np.eye(1), S=np.eye(2), x_bar=np.array([np.pi, 0.0])
    )
    if stochastic:
        return PlanningProblem(
            plant,
            cost=cost,
            horizon=2.0,
            x0=Uniform([-0.2, -0.2], [0.2, 0.2]),
            U=plant.input_box,
        )
    return PlanningProblem(
        plant,
        x0=[0.0, 0.0],
        Xf=SingletonSet([np.pi, 0.0]),
        cost=cost,
        horizon=2.0,
        U=plant.input_box,
    )


def move_problem(stochastic=False):
    """A double integrator brought to rest at the origin in two seconds."""
    plant = DoubleIntegrator()
    plant.inputs["u"].lower_bound = np.array([-2.0])
    plant.inputs["u"].upper_bound = np.array([2.0])
    cost = QuadraticCost.from_system(plant, Q=np.eye(2), R=0.1 * np.eye(1))
    if stochastic:
        return PlanningProblem(
            plant,
            cost=cost,
            horizon=2.0,
            Xf=SingletonSet([0.0, 0.0]),
            x0=Uniform([0.8, -0.2], [1.2, 0.2]),
            U=plant.input_box,
        )
    return PlanningProblem(
        plant,
        x0=[1.0, 0.0],
        Xf=SingletonSet([0.0, 0.0]),
        cost=cost,
        horizon=2.0,
        U=plant.input_box,
    )


def trajopt(problem, transcription, n_steps=10):
    return TrajectoryOptimizationPlanner(
        problem,
        transcription=transcription(n_steps),
        backend="numpy",
        options={"maxiter": 200, "ftol": 1e-10},
    )


def collocation(n_steps):
    return DirectCollocationTranscription(n_steps=n_steps)


def shooting(n_steps):
    return MultipleShootingTranscription(n_steps=n_steps)


def replay(problem, solution, solver="rk4"):
    """The open-loop policy driving the plant in series, sampled on the solution's grid."""
    plant = problem.sys
    planned = solution.trajectory
    plant.x0 = np.asarray(planned.x[:, 0])
    loop = solution.controller >> plant
    dt = float(planned.t[1] - planned.t[0])
    plant_id = loop.subsystem_id(plant)
    traj = loop.compute_trajectory(
        tf=float(planned.tf),
        dt=dt,
        solver=solver,
        record=(f"{plant_id}:u",),
        verbose=False,
    )
    driven = traj.of(plant_id)
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
        x[:, k + 1] = evaluator.step(x[:, k], u, t[k], dt=t[k + 1] - t[k])
    return x


# --- open-loop planners: the source block replays the schedule ---


def test_multiple_shooting_policy_reproduces_its_trajectory():
    problem = move_problem()
    solution = trajopt(problem, shooting).solve()
    assert isinstance(solution, PlanningSolution) and solution.success
    assert solution.open_loop and isinstance(solution.controller, TrajectorySource)
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


@pytest.mark.slow
def test_trajopt_evaluation_is_one_trial_deterministic_and_n_trials_stochastic():
    solution = trajopt(move_problem(), collocation, 128).solve(evaluate=True)
    assert solution.evaluation.n_trials == 1 and not solution.evaluation.failed[0]
    assert str(solution.evaluation).startswith("J = ")
    # the replayed schedule scores the optimizer's cost up to the discretization gap
    assert (
        abs(solution.evaluation.mean - solution.record.cost)
        < 0.05 * solution.record.cost
    )

    with pytest.warns(UserWarning):  # a deterministic planner on a stochastic problem
        planner = trajopt(move_problem(stochastic=True), shooting)
    solution = planner.solve(evaluate=True, n_trials=5)
    assert solution.evaluation.n_trials == 5
    assert "open loop" in str(solution) and "feasible" in str(solution.record)


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
            n_steps=6,
        ),
        seed=0,
        max_nodes=4000,
    )
    solution = planner.solve()
    assert solution.success and solution.record.reached_goal
    assert solution.open_loop and solution.controller.hold == "zoh"
    assert solution.evaluation is None  # the problem declares no cost
    planned = solution.trajectory

    def law(x, t):
        return np.asarray(solution.controller.h(np.zeros(0), np.zeros(0), t))

    x_hand = held_input_rollout(problem.sys, law, planned.t, planned.x[:, 0])
    np.testing.assert_allclose(x_hand, planned.x, atol=1e-8)


# --- feedback planners: the law handed out is the law scored and drawn ---


def double_integrator_problem():
    plant = DoubleIntegrator()
    plant.state.lower_bound = np.array([-2.0, -2.0])
    plant.state.upper_bound = np.array([2.0, 2.0])
    plant.inputs["u"].lower_bound = np.array([-1.0])
    plant.inputs["u"].upper_bound = np.array([1.0])
    cost = QuadraticCost.from_system(plant, Q=np.diag([1.0, 0.1]), R=np.diag([0.01]))
    return PlanningProblem(
        plant,
        x0=[1.0, 0.0],
        cost=cost,
        horizon=np.inf,
        penalize_failure=True,
        failure_cost=20.0,
        U=plant.input_box,
    )


@pytest.mark.parametrize("make", ["dp", "tabular"])
def test_grid_laws_are_scored_and_drawn_as_handed_out(make):
    from minilink.planning.results import drawn_trajectory_of

    problem = double_integrator_problem()
    grid = dict(x_grid=(11, 11), u_grid=(3,), dt=0.2)
    if make == "dp":
        planner = DynamicProgrammingPlanner(problem, gamma=0.9, tol=1e-6, **grid)
        solution = planner.solve(evaluate=True)
    else:
        planner = TabularLearningPlanner(
            problem, gamma=0.9, solver="euler", seed=0, **grid
        )
        solution = planner.solve(n_episodes=300, evaluate=True)

    # A feedback law: no plan, the controller handed out as it is, a lookup block
    assert not solution.open_loop and solution.trajectory is None
    assert solution.controller is planner.controller
    assert solution.evaluation.n_trials == 1

    # The score is the controller's continuous loop integrated on the planner's dt
    assert planner.dt == 0.2
    alone = evaluate(problem, solution.controller, dt=planner.dt)
    np.testing.assert_array_equal(solution.evaluation.J, alone.J)

    # The views draw that loop's run from the start, the plant's own run: on dt
    # when given (the planner's mirrors give its own), else on the automatic step
    run = problem.compile(solution.controller).trajectory().of("plant")
    np.testing.assert_array_equal(drawn_trajectory_of(solution).x, np.asarray(run.x))
    np.testing.assert_array_equal(drawn_trajectory_of(solution).x[:, 0], problem.x0)
    on_dt = problem.compile(solution.controller, planner.dt).trajectory().of("plant")
    drawn = drawn_trajectory_of(solution, planner.dt)
    np.testing.assert_array_equal(drawn.x, np.asarray(on_dt.x))
    np.testing.assert_allclose(np.diff(drawn.t), planner.dt, rtol=1e-9)
    assert solution.cost_to_go(problem.x0) == planner.cost_to_go(problem.x0)
    assert str(solution.evaluation).startswith("J = ")


def test_value_iteration_scores_the_law_it_hands_out_continuously():
    """solve(evaluate=True), evaluate(problem, controller, dt) and compare(dt) agree: the planner's dt, no hold."""
    from minilink.planning import compare

    problem = pendulum_problem().replace(Xf=None)
    vi = DynamicProgrammingPlanner(problem, x_grid=(41, 41), u_grid=(5,), dt=0.1)
    solution = vi.solve(evaluate=True)

    # The three scores of the law handed out, integrated on the planner's dt, are one
    # number
    assert vi.dt == 0.1
    alone = evaluate(problem, solution.controller, dt=vi.dt)
    raced = compare(problem, VI=solution, dt=vi.dt).evaluations["VI"]
    np.testing.assert_array_equal(solution.evaluation.J, alone.J)
    np.testing.assert_array_equal(solution.evaluation.J, raced.J)

    # The continuous law integrated on the grid's dt (ruled 2026-11-08), not the
    # law held over each period of the grid, a different controller with its own
    # score, nor the law on the automatic step (dt = 2 / 45 s here)
    held = evaluate(problem, solution.controller % 0.1)
    automatic = evaluate(problem, solution.controller)
    np.testing.assert_allclose(solution.evaluation.J, [23.38524887320534], rtol=1e-9)
    np.testing.assert_allclose(held.J, [23.36262222847119], rtol=1e-9)
    np.testing.assert_allclose(automatic.J, [23.37795258655731], rtol=1e-9)


def test_one_evaluation_scores_an_open_loop_and_a_feedback_law_on_the_same_draws():
    problem = pendulum_problem(stochastic=True)
    with pytest.warns(UserWarning):
        open_loop = trajopt(problem, shooting).solve()
    feedback = DynamicProgrammingPlanner(
        problem.nominal(), x_grid=(21, 21), u_grid=(3,), dt=0.1, gamma=0.95, tol=0.1
    ).solve()
    scored = [
        evaluate(problem, s, dt=0.1, n_trials=4, backend="numpy", key=3)
        for s in (open_loop, feedback)
    ]
    np.testing.assert_array_equal(scored[0].x0, scored[1].x0)
    assert all(np.all(np.isfinite(report.J)) for report in scored)


@pytest.mark.optional
@pytest.mark.jax
def test_the_rl_planner_scores_the_law_it_hands_out():
    pytest.importorskip("jax")
    from minilink.learning.reinforcement_learning import ReinforcementLearningPlanner

    plant = Pendulum()
    plant.inputs["u"].lower_bound = np.array([-4.0])
    plant.inputs["u"].upper_bound = np.array([4.0])
    plant.state.lower_bound = np.array([-np.pi, -8.0])
    plant.state.upper_bound = np.array([np.pi, 8.0])
    cost = QuadraticCost.from_system(plant, Q=np.eye(2), R=0.01 * np.eye(1))
    problem = PlanningProblem(
        plant,
        cost=cost,
        horizon=np.inf,
        x0=Uniform([-0.5, -0.5], [0.5, 0.5]),
        U=plant.input_box,
    )
    planner = ReinforcementLearningPlanner(
        problem,
        dt=0.1,
        hidden=(8, 8),
        n_envs=4,
        n_steps=16,
        batch_size=32,
        gamma=0.9,
    )
    solution = planner.solve(n_timesteps=64, evaluate=True, n_trials=3)

    # A feedback law: no plan; its score is the continuous loop of the copy handed
    # out, integrated on the planner's dt
    assert solution.trajectory is None and solution.evaluation.n_trials == 3
    alone = evaluate(
        problem,
        solution.controller,
        dt=planner.dt,
        n_trials=3,
        key=0,
        backend="jax",
        episode_length=planner.env.episode_length,
    )
    np.testing.assert_array_equal(solution.evaluation.J, alone.J)
    assert isinstance(solution.plot_trajectory(show=False).figure, object)


# --- the solution knows its problem, shows what the planner produced, and compares ---


def solved_pendulum():
    """One problem, two feedback solutions (value iteration and LQR) with their rollouts."""
    import matplotlib

    matplotlib.use("Agg")
    from minilink import LQRPlanner

    plant = Pendulum()
    plant.state.lower_bound = np.array([-2.0 * np.pi, -8.0])
    plant.state.upper_bound = np.array([2.0 * np.pi, 8.0])
    plant.inputs["u"].lower_bound = np.array([-5.0])
    plant.inputs["u"].upper_bound = np.array([5.0])
    cost = QuadraticCost.from_system(plant, Q=np.eye(2), R=np.eye(1), x_bar=[np.pi, 0])
    problem = PlanningProblem(
        plant,
        x0=[0.0, 0.0],
        Xf=SingletonSet([np.pi, 0.0]),
        cost=cost,
        horizon=np.inf,
        X=plant.state.box,
        penalize_failure=True,
        failure_cost=500.0,
        U=plant.input_box,
    )
    vi = DynamicProgrammingPlanner(
        problem, x_grid=(21, 21), u_grid=(3,), dt=0.1, gamma=0.95, tol=0.5
    )
    return (
        problem,
        vi.solve(evaluate=True),
        LQRPlanner(problem, dt=0.1).solve(evaluate=True),
    )


def test_every_solution_carries_its_problem():
    problem = move_problem()
    solution = trajopt(problem, shooting).solve()
    assert solution.problem is problem and solution.family == "trajectory optimization"
    problem, vi, lqr = solved_pendulum()
    assert vi.problem is problem and lqr.problem is problem
    assert vi.family == "value iteration" and lqr.family == "riccati"


def test_solution_verbs_draw_what_the_planner_produced():
    from minilink.planning.comparison import sample_cost_to_go

    problem, vi, lqr = solved_pendulum()
    for solution in (vi, lqr):
        fig, ax = solution.plot_cost_to_go(show=False, grid_shape=(11, 11))
        assert ax.get_title().startswith("cost-to-go")
        result = solution.plot_control_law(show=False)
        # the sweep is the problem's state box and the colour scale its input box
        x_lim, y_lim = result.axes.get_xlim(), result.axes.get_ylim()
        assert x_lim[0] == pytest.approx(
            -2.0 * np.pi, abs=0.2
        )  # half a cell of padding
        assert y_lim[1] == pytest.approx(8.0, abs=0.2)
        assert result.axes.collections[0].get_clim() == (-5.0, 5.0)
        assert solution.plot_trajectory(show=False).figure is not None
        assert solution.plot_cost(show=False).figure is not None
    x_level, y_level, J = sample_cost_to_go(lqr, grid_shape=(5, 7))
    assert J.shape == (5, 7) and J[2, 3] == pytest.approx(lqr.cost_to_go([0.0, 0.0]))

    plan = trajopt(move_problem(), shooting).solve()
    with pytest.raises(ValueError, match="open-loop"):
        plan.plot_control_law()
    with pytest.raises(ValueError, match="cost_to_go"):
        plan.plot_cost_to_go()
    # A feedback law with no score yet: the views run it from the start all the same
    unscored = DynamicProgrammingPlanner(
        problem, x_grid=(11, 11), u_grid=(3,), dt=0.2, gamma=0.9, tol=1.0
    ).solve()
    result = unscored.plot_trajectory(show=False)
    assert result.figure._suptitle.get_text() == "run (value iteration)"
    assert (
        plan.plot_trajectory(show=False).figure._suptitle.get_text().startswith("plan")
    )


def test_planner_shortcuts_draw_the_latest_solution():
    """A planner's plots draw its latest solution; refused before any solve."""
    from minilink import LQRPlanner

    problem, vi, lqr = solved_pendulum()
    planner = LQRPlanner(problem)
    with pytest.raises(ValueError):
        planner.plot_control_law()
    planner.solve()
    assert planner.plot_control_law(show=False).figure is not None
    assert planner.plot_cost_to_go(show=False, grid_shape=(9, 9))[0] is not None


def test_value_iteration_plots_its_own_tables_under_the_solution_names():
    """On the grid planner, plot_cost_to_go and plot_control_law draw the J and π tables."""
    problem, vi, lqr = solved_pendulum()
    planner = DynamicProgrammingPlanner(
        problem, x_grid=(11, 11), u_grid=(3,), dt=0.2, gamma=0.9, tol=1.0
    )
    with pytest.raises(ValueError):
        planner.plot_control_law()
    planner.solve()
    assert planner.plot_cost_to_go(show=False)[0] is not None
    assert planner.plot_control_law(show=False)[0] is not None


def test_compare_reads_the_solutions_side_by_side():
    from minilink.planning import compare

    problem, vi, lqr = solved_pendulum()
    race = compare(VI=vi, LQR=lqr)
    assert len(race) == 2 and race["LQR"] is lqr and list(race) == ["VI", "LQR"]
    table = str(race)
    assert table.splitlines()[0].split() == ["success", "record", "evaluation"]
    assert "converged" in table and "closed-loop poles" in table and "J = " in table

    fig, axes = race.plot_control_law(show=False)
    assert [ax.get_title() for ax in axes] == ["VI", "LQR"]
    assert axes[0].collections[0].get_clim() == axes[1].collections[0].get_clim()
    fig, axes = race.plot_cost_to_go(show=False, grid_shape=(9, 9))
    assert axes[0].collections[0].get_clim() == axes[1].collections[0].get_clim()
    fig, axes = race.plot_trajectory(show=False)
    assert len(axes) == 3 and len(axes[0].get_lines()) == 2

    # one evaluation scores every policy on the same draws; a solution stands for its policy
    scored = compare(problem, VI=vi, LQR=lqr, dt=0.1, backend="numpy")
    assert list(scored) == ["VI", "LQR"] and scored["LQR"] is lqr
    J_lqr = evaluate(problem, lqr.controller, dt=0.1, backend="numpy").J
    np.testing.assert_array_equal(scored.evaluations["LQR"].J, J_lqr)
    assert "J = " in str(scored)
    np.testing.assert_array_equal(
        scored.difference("VI", "LQR"),
        scored.evaluations["VI"].J - scored.evaluations["LQR"].J,
    )

    # a bare controller and an evaluation already computed take part too
    mixed = compare(problem, LQR=lqr.controller, VI=scored.evaluations["VI"], dt=0.1)
    assert list(mixed) == ["LQR"] and "given" in str(mixed)
    on_grid = DynamicProgrammingPlanner(problem, x_grid=(11, 11), u_grid=(3,), dt=0.2)
    assert np.all(np.isfinite(on_grid.evaluate_policy(lqr).params["values"]))

    with pytest.raises(TypeError):
        compare(VI=vi.controller)
    with pytest.raises(ValueError):
        compare()


@pytest.mark.optional
@pytest.mark.jax
def test_compare_scores_every_policy_on_one_backend_and_the_same_draws():
    """The default backend is chosen once per race: NumPy for all when one law does not trace."""
    from minilink.planning import ProblemEvaluator, compare

    problem, vi, lqr = solved_pendulum()
    spread = PlanningProblem(
        problem.sys,
        cost=problem.cost,
        horizon=np.inf,
        X=problem.X,
        penalize_failure=True,
        failure_cost=500.0,
        x0=Uniform([-0.5, -0.5], [0.5, 0.5]),
        U=problem.sys.input_box,
    )
    settings = dict(dt=0.1, n_trials=3, episode_length=1.0)

    # a law that runs on NumPy only puts the LQR law there too, on the same draws
    assert ProblemEvaluator(spread, numpy_only_law(lqr), 0.1).backend == "numpy"
    scored = compare(spread, NumPy=numpy_only_law(lqr), LQR=lqr, **settings)
    np.testing.assert_array_equal(
        scored.evaluations["NumPy"].x0, scored.evaluations["LQR"].x0
    )
    on_numpy = evaluate(spread, lqr, backend="numpy", **settings)
    np.testing.assert_array_equal(scored.evaluations["LQR"].J, on_numpy.J)

    # the value-iteration law traces: scored beside LQR on JAX, the same draws
    scored = compare(spread, VI=vi, LQR=lqr, **settings)
    np.testing.assert_array_equal(
        scored.evaluations["VI"].x0, scored.evaluations["LQR"].x0
    )

    # laws that all trace keep the JAX backend and its numbers
    scored = compare(spread, LQR=lqr, **settings)
    on_jax = evaluate(spread, lqr, backend="jax", **settings)
    np.testing.assert_array_equal(scored.evaluations["LQR"].J, on_jax.J)


def numpy_only_law(solution):
    """The LQR law of ``solution``, written so it runs on NumPy only (``float`` of its command)."""
    from minilink.core.system import System

    K = np.asarray(solution.record.K)
    x_bar = np.asarray(solution.problem.cost.x_bar)

    law = System(0)
    law.add_input_port("x", dim=K.shape[1])
    law.add_output_port(
        "u",
        dim=1,
        function=lambda x, u, t=0, params=None: np.array([-float(K[0] @ (u - x_bar))]),
        dependencies=("x",),
    )

    return law


def test_compare_reads_dt_as_evaluate_does():
    """A sampled controller runs on its own clock, dt omitted; a continuous one on dt, else its automatic step."""
    from minilink.control import lqr as lqr_design
    from minilink.planning import compare

    plant = Pendulum()
    cost = QuadraticCost.from_system(plant)
    problem = PlanningProblem(plant, x0=np.array([0.3, 0.0]), cost=cost, horizon=2.0)
    law = lqr_design(plant, np.eye(2), np.eye(1))

    on_clock = compare(problem, LQR=law % 0.1, backend="numpy").evaluations["LQR"]
    alone = evaluate(problem, law % 0.1, backend="numpy")
    np.testing.assert_array_equal(on_clock.J, alone.J)

    continuous = compare(problem, LQR=law, backend="numpy").evaluations["LQR"]
    np.testing.assert_array_equal(
        continuous.J, evaluate(problem, law, backend="numpy").J
    )
    with pytest.raises(ValueError, match="own clock"):
        compare(problem, LQR=law % 0.1, dt=0.05, backend="numpy")


def test_compare_returns_a_comparison_of_the_solutions_and_their_scores():
    from minilink.planning import Comparison, compare

    problem, vi, lqr = solved_pendulum()
    race = compare(problem, VI=vi, LQR=lqr, dt=0.1, backend="numpy")

    # The named solutions, in order, and one evaluation each on the same draws
    assert isinstance(race, Comparison)
    assert list(race.solutions) == ["VI", "LQR"] and list(race.evaluations) == [
        "VI",
        "LQR",
    ]
    assert race.difference("VI", "LQR").shape == race.evaluations["VI"].J.shape


def test_compare_scores_a_sampled_loop_as_evaluate_does():
    """The plant inside a clocked law keeps its own integration steps in both."""
    from minilink.control import lqr as lqr_design
    from minilink.planning import compare

    plant = Pendulum()
    cost = QuadraticCost.from_system(plant)
    problem = PlanningProblem(plant, x0=np.array([0.3, 0.0]), cost=cost, horizon=5.0)
    law = lqr_design(plant, np.eye(2), np.eye(1)) % 0.2

    alone = evaluate(problem, law, dt=0.2)
    compared = compare(problem, LQR=law, dt=0.2).evaluations["LQR"]

    np.testing.assert_allclose(compared.J, alone.J)


# --- the controller is the block handed out; the function is the compiled face's ---


def test_a_feedback_solution_hands_out_its_controller_block_and_no_function():
    from minilink import LQRPlanner
    from minilink.planning.policies import policy_from_block

    x = np.array([0.4, -0.3])

    # The stationary LQR law, u = −K x about the origin: the block's command
    stationary = LQRPlanner(move_problem().replace(horizon=np.inf)).solve()
    K = stationary.controller.params["K"]
    command = policy_from_block(stationary.controller)
    np.testing.assert_allclose(command(x), -K @ x, rtol=1e-12)
    assert command(x).shape == (1,)

    # The finite-horizon LQR law reads the time: u = −K(t_k) x at a node of its schedule
    finite = LQRPlanner(move_problem()).solve()
    t_k = finite.controller.params["t"]
    K_k = finite.controller.params["K"]
    command = policy_from_block(finite.controller)
    for k in (0, 30):
        np.testing.assert_allclose(command(x, t_k[k]), -K_k[k] @ x, rtol=1e-12)

    # The solution and the planner hand out the block, not a function of it
    for retired in ("policy", "require_state", "require_trajectory"):
        assert not hasattr(stationary, retired)
    assert stationary.trajectory is None and stationary.evaluation is None


def test_an_open_loop_solutions_controller_replays_its_plan():
    solution = trajopt(move_problem(), collocation).solve()
    plan = solution.trajectory
    assert solution.open_loop

    # u = π(t): the source's output, the plan's input linear between its samples
    t = 0.5 * (plan.t[3] + plan.t[4])
    u_plan = np.interp(t, plan.t, plan.u[0])
    u = solution.controller.h(np.zeros(0), np.zeros(0), t)
    np.testing.assert_allclose(u, [u_plan], rtol=1e-12)


def test_a_solution_is_built_by_keyword_with_its_optional_evidence():
    problem = move_problem()
    solution = trajopt(problem, shooting).solve()
    bare = PlanningSolution(
        problem=problem, controller=solution.controller, record=solution.record
    )
    assert bare.trajectory is None and bare.cost_to_go is None
    assert bare.evaluation is None and bare.params is not problem.params
    with pytest.raises(TypeError):
        PlanningSolution(problem, solution.controller, solution.record)


@pytest.mark.parametrize("make", ["dp", "lqr"])
def test_the_planner_mirrors_its_latest_solution(make, monkeypatch):
    import matplotlib.pyplot as plt

    from minilink import LQRPlanner

    problem = move_problem().replace(horizon=np.inf)
    if make == "dp":
        planner = DynamicProgrammingPlanner(
            problem, x_grid=(11, 11), u_grid=(5,), dt=0.2
        )
    else:
        planner = LQRPlanner(problem)
    x = np.array([0.4, -0.3])

    # Before a solve, the mirrors refuse
    with pytest.raises(ValueError, match="No solution"):
        planner.controller  # noqa: B018

    # After it, each reads the latest solution
    solution = planner.solve()
    assert planner.controller is planner.last_solution.controller
    assert planner.cost_to_go(x) == float(solution.cost_to_go(x))
    assert isinstance(planner.cost_to_go(x), float)
    with pytest.raises(AttributeError):
        planner.controller = None

    # The views carry the solution's words; the retired ones are gone
    assert callable(planner.plot_trajectory) and callable(planner.animate)
    retired = (
        "plot_solution",
        "animate_solution",
        "get_controller",
        "value_at",
        "policy",
        "nominal_trajectory",
        "holds_policy",
    )
    for name in retired:
        assert not hasattr(planner, name)
    planner.plot_trajectory(show=False)
    planner.plot_cost(show=False)
    plt.close("all")

    # The views run the law on the planner's own dt, the step of its evidence
    given = []
    monkeypatch.setattr(
        PlanningSolution, "plot_cost", lambda self, **kwargs: given.append(kwargs)
    )
    planner.plot_cost(show=False)
    planner.plot_cost(dt=0.01)
    assert planner.dt is not None
    assert given == [{"dt": planner.dt, "show": False}, {"dt": 0.01}]


def test_the_planners_cost_to_go_reads_the_time_on_a_finite_horizon():
    from minilink import LQRPlanner

    planner = LQRPlanner(move_problem())
    solution = planner.solve()
    x = np.array([0.4, -0.3])

    assert planner.cost_to_go(x, t=1.0) == float(solution.cost_to_go(x, t=1.0))
    assert planner.cost_to_go(x, t=1.0) != planner.cost_to_go(x, t=0.0)


def test_a_tree_search_plan_starts_at_the_problem_start_time():
    """The path's times run from the problem's ``t0``, and its evidence over the path's own length."""
    from minilink import BallSet, Pendulum, PlanningProblem, QuadraticCost, RRTPlanner

    plant = Pendulum()
    x_up = np.array([np.pi, 0.0])
    problem = PlanningProblem(
        plant,
        x0=np.zeros(2),
        Xf=BallSet(x_up, 0.3),
        cost=QuadraticCost.from_system(plant, x_bar=x_up),
        X=plant.state.box,
        U=plant.input_box,
        t0=2.0,
    )

    solution = RRTPlanner(problem, seed=0).solve()
    plan = solution.trajectory

    assert plan.t0 == pytest.approx(2.0)
    np.testing.assert_allclose(solution.controller.params["t"], plan.t)
