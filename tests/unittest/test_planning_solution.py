"""One result for every planner: the controller handed out is the one scored and drawn, and the evaluation is shared."""

import numpy as np
import pytest

from minilink import DynamicProgrammingPlanner, Pendulum, PlanningProblem, QuadraticCost
from minilink.blocks import TrajectorySource
from minilink.core.sets import BallSet, SingletonSet
from minilink.core.trajectory import Trajectory
from minilink.dynamics.catalog.equations import DoubleIntegrator
from minilink.learning.reinforcement_learning import TabularLearningPlanner
from minilink.planning import (
    PlanningSolution,
    Uniform,
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
    plant.inputs["u"].lower = np.array([-5.0])
    plant.inputs["u"].upper = np.array([5.0])
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
    plant.inputs["u"].lower = np.array([-2.0])
    plant.inputs["u"].upper = np.array([2.0])
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
    assert abs(solution.evaluation.mean - solution.record.J) < 0.05 * solution.record.J

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
        max_iterations=4000,
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
    plant.state.lower = np.array([-2.0, -2.0])
    plant.state.upper = np.array([2.0, 2.0])
    plant.inputs["u"].lower = np.array([-1.0])
    plant.inputs["u"].upper = np.array([1.0])
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
    alone = problem.evaluate(solution.controller, dt=planner.dt)
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
    """solve(evaluate=True), problem.evaluate(controller, dt) and problem.compare(..., dt=) agree: the planner's dt, no hold."""

    problem = pendulum_problem().replace(Xf=None)
    vi = DynamicProgrammingPlanner(problem, x_grid=(41, 41), u_grid=(5,), dt=0.1)
    solution = vi.solve(evaluate=True)

    # The three scores of the law handed out, integrated on the planner's dt, are one
    # number
    assert vi.dt == 0.1
    alone = problem.evaluate(solution.controller, dt=vi.dt)
    raced = problem.compare(VI=solution, dt=vi.dt).evaluations["VI"]
    np.testing.assert_array_equal(solution.evaluation.J, alone.J)
    np.testing.assert_array_equal(solution.evaluation.J, raced.J)

    # The continuous law integrated on the grid's dt (ruled 2026-11-08), not the
    # law held over each period of the grid, a different controller with its own
    # score, nor the law on the automatic step (dt = 2 / 45 s here)
    held = problem.evaluate(solution.controller % 0.1)
    automatic = problem.evaluate(solution.controller)
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
        problem.evaluate(s, dt=0.1, n_trials=4, backend="numpy", key=3)
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
    plant.inputs["u"].lower = np.array([-4.0])
    plant.inputs["u"].upper = np.array([4.0])
    plant.state.lower = np.array([-np.pi, -8.0])
    plant.state.upper = np.array([np.pi, 8.0])
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
        n_rollouts=4,
        n_steps_per_update=16,
        batch_size=32,
        gamma=0.9,
    )
    solution = planner.solve(n_timesteps=64, evaluate=True, n_trials=3)

    # A feedback law: no plan; its score is the continuous loop of the copy handed
    # out, integrated on the planner's dt
    assert solution.trajectory is None and solution.evaluation.n_trials == 3
    alone = problem.evaluate(
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
    plant.state.lower = np.array([-2.0 * np.pi, -8.0])
    plant.state.upper = np.array([2.0 * np.pi, 8.0])
    plant.inputs["u"].lower = np.array([-5.0])
    plant.inputs["u"].upper = np.array([5.0])
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
        drawn = solution.plot_cost_to_go(show=False, grid_shape=(11, 11))
        assert drawn.axes.get_title().startswith("cost-to-go")
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
    assert result.figure._suptitle.get_text() == "simulation (value iteration)"
    title = plan.plot_trajectory(show=False).figure._suptitle.get_text()
    assert title.startswith("nominal trajectory")


def test_planner_shortcuts_draw_the_latest_solution():
    """A planner's plots draw its latest solution; refused before any solve."""
    from minilink import LQRPlanner

    problem, vi, lqr = solved_pendulum()
    planner = LQRPlanner(problem)
    with pytest.raises(ValueError):
        planner.plot_control_law()
    planner.solve()
    assert planner.plot_control_law(show=False).figure is not None
    assert planner.plot_cost_to_go(show=False, grid_shape=(9, 9)).figure is not None


def test_value_iteration_plots_its_own_tables_under_the_solution_names():
    """On the grid planner, plot_cost_to_go and plot_control_law draw the J and π tables."""
    problem, vi, lqr = solved_pendulum()
    planner = DynamicProgrammingPlanner(
        problem, x_grid=(11, 11), u_grid=(3,), dt=0.2, gamma=0.9, tol=1.0
    )
    with pytest.raises(ValueError):
        planner.plot_control_law()
    planner.solve()
    assert planner.plot_cost_to_go(show=False).figure is not None
    assert planner.plot_control_law(show=False).figure is not None


def test_compare_reads_the_solutions_side_by_side():

    problem, vi, lqr = solved_pendulum()
    race = problem.compare(VI=vi, LQR=lqr, dt=0.1, backend="numpy")
    assert len(race) == 2 and race["LQR"] is lqr and list(race) == ["VI", "LQR"]
    table = str(race)
    assert table.splitlines()[1].startswith("Comparison, ")
    titles = [half.strip() for half in table.splitlines()[3].split("│")]
    assert titles == ["", "nominal trajectory", "simulation"]
    assert "converged" not in table and "converged" in str(race["VI"])

    axes = race.plot_control_law(show=False).axes
    assert [ax.get_title() for ax in axes] == ["VI", "LQR"]
    assert axes[0].collections[0].get_clim() == axes[1].collections[0].get_clim()
    axes = race.plot_cost_to_go(show=False, grid_shape=(9, 9)).axes
    assert axes[0].collections[0].get_clim() == axes[1].collections[0].get_clim()
    axes = race.plot_simulated_trajectory(show=False).axes
    assert len(axes) == 3 and len(axes[0].get_lines()) == 2

    # The same panels with Plotly, titled above them
    pytest.importorskip("plotly")
    laws = race.plot_control_law(title="laws", renderer="plotly", show=False)
    assert [type(trace).__name__ for trace in laws.figure.data] == ["Heatmap"] * 2
    assert laws.figure.layout.title.text == "laws"
    runs = race.plot_simulated_trajectory(renderer="plotly", show=False)
    assert [trace.name for trace in runs.figure.data[:2]] == ["VI", "LQR"]

    # one evaluation scores every policy on the same draws; a solution stands for its policy
    scored = race
    J_lqr = problem.evaluate(lqr.controller, dt=0.1, backend="numpy").J
    np.testing.assert_array_equal(scored.evaluations["LQR"].J, J_lqr)
    assert f"{float(J_lqr[0]):.4g}" in str(scored) and scored.problem is problem
    np.testing.assert_array_equal(
        scored.difference("VI", "LQR"),
        scored.evaluations["VI"].J - scored.evaluations["LQR"].J,
    )

    # a bare controller and an evaluation already computed take part too
    mixed = problem.compare(LQR=lqr.controller, VI=scored.evaluations["VI"], dt=0.1)
    assert list(mixed) == ["LQR"] and str(mixed["LQR"].record) == "given"
    assert [line.split()[0] for line in str(mixed).splitlines()[6:-1]] == ["LQR", "VI"]
    on_grid = DynamicProgrammingPlanner(problem, x_grid=(11, 11), u_grid=(3,), dt=0.2)
    assert np.all(np.isfinite(on_grid.evaluate_policy(lqr).params["values"]))

    with pytest.raises(TypeError, match="named entries"):
        problem.compare(vi, dt=0.1)
    with pytest.raises(ValueError, match="at least one entry"):
        problem.compare(dt=0.1)


@pytest.mark.optional
@pytest.mark.jax
def test_compare_scores_every_policy_on_one_backend_and_the_same_draws():
    """The default backend is chosen once per race: NumPy for all when one law does not trace."""
    from minilink.planning import ProblemEvaluator

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
    scored = spread.compare(NumPy=numpy_only_law(lqr), LQR=lqr, **settings)
    np.testing.assert_array_equal(
        scored.evaluations["NumPy"].x0, scored.evaluations["LQR"].x0
    )
    on_numpy = spread.evaluate(lqr, backend="numpy", **settings)
    np.testing.assert_array_equal(scored.evaluations["LQR"].J, on_numpy.J)

    # the value-iteration law traces: scored beside LQR on JAX, the same draws
    scored = spread.compare(VI=vi, LQR=lqr, **settings)
    np.testing.assert_array_equal(
        scored.evaluations["VI"].x0, scored.evaluations["LQR"].x0
    )

    # laws that all trace keep the JAX backend and its numbers
    scored = spread.compare(LQR=lqr, **settings)
    on_jax = spread.evaluate(lqr, backend="jax", **settings)
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

    plant = Pendulum()
    cost = QuadraticCost.from_system(plant)
    problem = PlanningProblem(plant, x0=np.array([0.3, 0.0]), cost=cost, horizon=2.0)
    law = lqr_design(plant, np.eye(2), np.eye(1))

    on_clock = problem.compare(LQR=law % 0.1, backend="numpy").evaluations["LQR"]
    alone = problem.evaluate(law % 0.1, backend="numpy")
    np.testing.assert_array_equal(on_clock.J, alone.J)

    continuous = problem.compare(LQR=law, backend="numpy").evaluations["LQR"]
    np.testing.assert_array_equal(
        continuous.J, problem.evaluate(law, backend="numpy").J
    )
    with pytest.raises(ValueError, match="own clock"):
        problem.compare(LQR=law % 0.1, dt=0.05, backend="numpy")


def test_compare_returns_a_comparison_of_the_solutions_and_their_scores():
    from minilink.planning import Comparison

    problem, vi, lqr = solved_pendulum()
    race = problem.compare(VI=vi, LQR=lqr, dt=0.1, backend="numpy")

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

    plant = Pendulum()
    cost = QuadraticCost.from_system(plant)
    problem = PlanningProblem(plant, x0=np.array([0.3, 0.0]), cost=cost, horizon=5.0)
    law = lqr_design(plant, np.eye(2), np.eye(1)) % 0.2

    alone = problem.evaluate(law, dt=0.2)
    compared = problem.compare(LQR=law, dt=0.2).evaluations["LQR"]

    np.testing.assert_allclose(compared.J, alone.J)


# --- a comparison reads each answer's plan beside its run ---


class HandPlanRecord:
    """The record of a plan written by hand."""

    success = True

    def __str__(self):
        return "written by hand"


def judged_move(t0=0.0, horizon=2.0, radius=0.2, x0=(1.0, 0.0)):
    """A double integrator brought into a ball around the origin by t0 + horizon, |u| ≤ 2."""
    plant = DoubleIntegrator()
    plant.inputs["u"].lower = np.array([-2.0])
    plant.inputs["u"].upper = np.array([2.0])
    cost = QuadraticCost.from_system(plant, Q=np.eye(2), R=0.1 * np.eye(1))

    return PlanningProblem(
        plant,
        x0=x0,
        Xf=BallSet(np.zeros(2), radius),
        cost=cost,
        horizon=horizon,
        t0=t0,
        U=plant.input_box,
    )


def hand_plan(problem, length=2.5):
    """A plan of ``length`` seconds from the problem's start, longer than its horizon: x₁ = 1 − 0.4 (t − t0), u = 0."""
    t = problem.t0 + np.linspace(0.0, length, int(round(10 * length)) + 1)
    x = np.vstack([1.0 - 0.4 * (t - problem.t0), np.full(t.size, -0.4)])
    u = np.zeros((1, t.size))
    plan = Trajectory(t, x, u)

    return PlanningSolution(
        problem=problem,
        controller=TrajectorySource(t, u, hold="zoh"),
        record=HandPlanRecord(),
        trajectory=plan,
    )


def halves_of(table, name):
    """The nominal half and the simulation half of ``name``'s line in a comparison's table."""
    line = next(line for line in table.splitlines() if line.startswith(f"{name} "))
    _, plan, run = (half.strip() for half in line.split("│"))

    return plan, run


def test_the_table_scores_the_nominal_trajectory_cut_at_the_horizon_beside_the_simulation():
    """A plan of 2.5 s on a 2 s horizon is scored on [t0, t0 + 2], the state at t0 + 2 interpolated."""
    from minilink.control import lqr as lqr_design
    from minilink.planning.evaluation import evaluate_trajectory

    problem = judged_move()
    plan = hand_plan(problem)
    law = lqr_design(problem.sys, np.eye(2), 0.1 * np.eye(1))
    comparison = problem.compare(Law=law, Plan=plan, dt=0.05, backend="numpy")
    table = str(comparison)

    # The plan scored on its first 2 s, the state at t = 2 a sample of the plan
    p = plan.trajectory
    kept = p.t <= 2.0 + 1e-12
    cut = evaluate_trajectory(
        problem, Trajectory(p.t[kept], p.x[:, kept], p.u[:, kept])
    )
    # A framed panel: the title between two rules, a closing rule, as wide as the table
    lines = table.splitlines()
    assert lines[1] == "Comparison, DoubleIntegrator, horizon=2"
    assert lines[0] == lines[2] == lines[-1] == "=" * max(map(len, lines))
    titles = [half.strip() for half in lines[3].split("│")]
    assert titles == ["", "nominal trajectory", "simulation"]
    assert lines[4].split("│")[2].split() == ["J", "goal", "constraints"]
    plan_half, run_half = halves_of(table, "Plan")
    assert plan_half.split() == [f"{float(cut.J[0]):.4g}", "missed", "ok"]
    assert len(run_half.split()) == 3

    # The law has no nominal trajectory; its simulation is the judge's
    plan_half, run_half = halves_of(table, "Law")
    assert plan_half == "-"
    assert run_half.startswith(f"{float(comparison.evaluations['Law'].J[0]):.4g}")

    # The records stay on the solutions, out of the table
    assert "given" not in table and "written by hand" not in table


def test_a_plan_that_starts_after_the_deadline_says_so_and_the_table_prints():
    """A plan beginning at t0 + T or later has nothing on the horizon: its plan half says so, its run is still judged."""

    problem = judged_move()
    late = hand_plan(judged_move(t0=5.0))
    table = str(problem.compare(Plan=late, dt=0.05))

    plan_half, run_half = halves_of(table, "Plan")
    assert plan_half == "starts after the deadline"
    assert len(run_half.split()) == 3


def test_the_table_and_the_figure_keep_the_order_the_entries_were_given():
    """An entry given as an evaluation keeps its place: the rows and the colours follow the call."""
    from minilink.control import lqr as lqr_design

    problem = judged_move()
    law = lqr_design(problem.sys, np.eye(2), 0.1 * np.eye(1))
    scored = problem.evaluate(law, dt=0.05, trajectories=True)
    comparison = problem.compare(Scored=scored, Plan=hand_plan(problem), dt=0.05)

    rows = [line.split()[0] for line in str(comparison).splitlines()[6:8]]
    assert rows == ["Scored", "Plan"]
    labels = list(
        dict.fromkeys(
            line.get_label()
            for line in comparison.plot_simulated_trajectory(show=False)
            .axes[0]
            .get_lines()
            if line.get_label() in ("Scored", "Plan")
        )
    )
    assert labels == ["Scored", "Plan"]


def test_a_run_that_reaches_the_goal_outside_the_constraints_reads_both():
    """The table states the numbers and ranks nothing: no star, no footnote."""
    from minilink.control import lqr as lqr_design

    problem = judged_move(radius=0.5)
    soft = lqr_design(problem.sys, np.eye(2), 1.0 * np.eye(1))
    optimal = lqr_design(problem.sys, np.eye(2), 0.1 * np.eye(1))
    table = str(problem.compare(Soft=soft, Optimal=optimal, dt=0.05))

    # The cost's own LQR law costs less but leaves the input limits
    assert halves_of(table, "Optimal")[1].split()[1:] == ["reached", "violated"]
    assert halves_of(table, "Soft")[1].split()[1:] == ["reached", "ok"]
    J = {
        name: float(halves_of(table, name)[1].split()[0])
        for name in ("Soft", "Optimal")
    }
    assert J["Optimal"] < J["Soft"]
    assert "*" not in table and "best" not in table


def test_the_table_reads_several_draws_an_evaluation_and_a_diverged_run():
    """Several draws: the criterion heads J, goal and constraints are counts; an Evaluation entry has no nominal trajectory; a run not finite reads diverged."""
    from dataclasses import replace

    from minilink.control import lqr as lqr_design
    from minilink.planning.evaluation import evaluate_trajectory

    # Several draws of the start
    problem = judged_move(x0=Uniform([0.8, -0.2], [1.2, 0.2]))
    law = lqr_design(problem.sys, np.eye(2), 0.1 * np.eye(1))
    comparison = problem.compare(Law=law, dt=0.05, n_trials=5, backend="numpy")
    table = str(comparison)
    assert table.splitlines()[1].endswith("criterion=expectation, 5 draws")
    header = table.splitlines()[4].split("│")[2].split()
    assert header == ["E[J]", "σ(J)", "max", "J", "goal", "constraints"]
    _, run_half = halves_of(table, "Law")
    evaluation = comparison.evaluations["Law"]
    statistics = [evaluation.mean, evaluation.std, evaluation.worst]
    assert run_half.split() == [
        *(f"{float(value):.4g}" for value in statistics),
        f"{int(np.sum(evaluation.reached))}/5",
        f"{int(np.sum(~evaluation.failed))}/5",
    ]

    # Another criterion comes first, each statistic once
    for criterion, labels in (
        (("cvar", 0.2), ["CVaR_0.2(J)", "E[J]", "σ(J)", "max", "J"]),
        ("worst_case", ["max", "J", "E[J]", "σ(J)"]),
    ):
        judged = problem.replace(criterion=criterion)
        table = str(judged.compare(Law=law, dt=0.05, n_trials=5, backend="numpy"))
        header = table.splitlines()[4].split("│")[2].split()
        assert header == [*labels, "goal", "constraints"]

    # A run not finite, given as an evaluation that keeps no run
    t = np.linspace(0.0, 2.0, 21)
    x = np.zeros((2, t.size))
    x[0, 5:] = np.nan
    diverged = evaluate_trajectory(
        judged_move(), Trajectory(t, x, np.zeros((1, t.size)))
    )
    mixed = judged_move().compare(
        Law=law, Diverged=replace(diverged, trajectories=None), dt=0.05
    )
    plan_half, run_half = halves_of(str(mixed), "Diverged")
    assert plan_half == "-"
    assert run_half.split()[1:] == ["missed", "diverged"]


def test_the_judged_criterion_heads_the_table_whatever_the_entries_carry():
    """An evaluation made under another criterion is read under the judged one; draws that differ read as a range; one run among draws has no spread."""
    from minilink.control import lqr as lqr_design
    from minilink.planning.evaluation import reduce_criterion

    problem = judged_move(x0=Uniform([0.8, -0.2], [1.2, 0.2]))
    law = lqr_design(problem.sys, np.eye(2), 0.1 * np.eye(1))
    soft = lqr_design(problem.sys, np.eye(2), 1.0 * np.eye(1))

    # Judged in expectation: a CVaR evaluation given first does not set the columns
    given = problem.replace(criterion=("cvar", 0.4)).evaluate(soft, dt=0.05, n_trials=5)
    table = str(problem.compare(Given=given, Law=law, dt=0.05, n_trials=5))
    assert table.splitlines()[1].endswith("criterion=expectation, 5 draws")
    header = table.splitlines()[4].split("│")[2].split()
    assert header[:3] == ["E[J]", "σ(J)", "max"]
    assert halves_of(table, "Given")[1].split()[0] == f"{float(given.mean):.4g}"

    # Judged under a CVaR: every row's first column is that CVaR of its own J
    judged = problem.replace(criterion=("cvar", 0.4))
    given = problem.evaluate(soft, dt=0.05, n_trials=5)
    comparison = judged.compare(Given=given, Law=law, dt=0.05, n_trials=5)
    table = str(comparison)
    assert "criterion=cvar, alpha=0.4" in table.splitlines()[1]
    for name, evaluation in (("Given", given), ("Law", comparison.evaluations["Law"])):
        cvar = float(reduce_criterion(evaluation.J, ("cvar", 0.4)))
        assert halves_of(table, name)[1].split()[0] == f"{cvar:.4g}"

    # Ten draws beside five: a range; one run beside them: its J, no spread
    ten = problem.evaluate(soft, dt=0.05, n_trials=10)
    one = judged_move().evaluate(soft, dt=0.05)
    table = str(problem.compare(Ten=ten, One=one, Law=law, dt=0.05, n_trials=5))
    assert table.splitlines()[1].endswith("5–10 draws")
    J, *spread = halves_of(table, "One")[1].split()[:3]
    assert J == f"{float(one.J[0]):.4g}" and spread == ["-", "-"]


def test_the_plan_is_scored_on_the_judged_problem_whatever_the_entries_order():
    """The judged problem scores the plan and marks the figure, not the first entry's own."""
    from minilink.control import lqr as lqr_design
    from minilink.planning.evaluation import evaluate_trajectory

    own = judged_move(horizon=2.5, radius=1.0)
    judged = judged_move(t0=1.0, horizon=2.0)
    plan = hand_plan(own)
    law = lqr_design(judged.sys, np.eye(2), 0.1 * np.eye(1))

    for entries in ({"Plan": plan, "Law": law}, {"Law": law, "Plan": plan}):
        comparison = judged.compare(**entries, dt=0.05, backend="numpy")
        assert comparison.problem is judged

        # Scored on the judged problem, whole: the plan's [0, 2.5] ends before the judged
        # deadline t0 + T = 3; it misses the small goal
        expected = evaluate_trajectory(judged, plan.trajectory)
        plan_half, _ = halves_of(str(comparison), "Plan")
        assert plan_half.split()[:2] == [f"{float(expected.J[0]):.4g}", "missed"]

        # Both figures' horizon is the judged t0 + T on every row
        axes = [
            *comparison.plot_nominal_trajectory(show=False).axes,
            *comparison.plot_simulated_trajectory(show=False).axes,
        ]
        for ax in axes:
            verticals = [
                line.get_xdata()[0]
                for line in ax.get_lines()
                if line.get_linestyle() == ":"
                and line.get_xdata()[0] == line.get_xdata()[-1]
            ]
            assert verticals == [3.0]


def test_the_figures_marks_follow_each_rows_signal():
    """The goal's extent on the x rows at t0 + T, the input limits on the u rows, the horizon on every row."""
    from minilink.control import lqr as lqr_design
    from minilink.planning.comparison import GOAL_COLOR, LIMIT_COLOR

    problem = judged_move(t0=1.0)
    law = lqr_design(problem.sys, np.eye(2), 0.1 * np.eye(1))
    comparison = problem.compare(Law=law, Plan=hand_plan(problem), dt=0.05)

    def marks(ax, color):
        return [
            (tuple(line.get_xdata()), tuple(line.get_ydata()))
            for line in ax.get_lines()
            if line.get_color() == color
        ]

    # On the input row alone: the two limits and the horizon, no goal
    (u_row,) = [comparison.plot_simulated_trajectory(signals=("u",), show=False).axes]
    assert marks(u_row, GOAL_COLOR) == []
    limits = [ys for xs, ys in marks(u_row, LIMIT_COLOR) if ys[0] == ys[-1]]
    assert sorted(ys[0] for ys in limits) == [-2.0, 2.0]

    # x rows: the goal's extent at t0 + T = 3, the ball of radius 0.2; the u row: no
    # goal; on both figures
    for plot in (
        comparison.plot_nominal_trajectory,
        comparison.plot_simulated_trajectory,
    ):
        axes = plot(show=False).axes
        for k in range(2):
            assert marks(axes[k], GOAL_COLOR) == [((3.0, 3.0), (-0.2, 0.2))]
        assert marks(axes[2], GOAL_COLOR) == []


def test_the_legend_sits_beside_the_panels_and_given_axes_are_left_as_they_are():
    """A figure drawn here: one legend of the names and the limits beside the panels; on given axes: each panel's own, the axes not moved."""
    import matplotlib.pyplot as plt

    from minilink.control import lqr as lqr_design

    problem = judged_move()
    law = lqr_design(problem.sys, np.eye(2), 0.1 * np.eye(1))
    comparison = problem.compare(Law=law, Plan=hand_plan(problem), dt=0.05)

    result = comparison.plot_simulated_trajectory(show=False)
    (legend,) = result.figure.legends
    assert [text.get_text() for text in legend.get_texts()] == ["Law", "Plan", "limits"]
    assert all(ax.get_legend() is None for ax in result.axes)
    right = max(ax.get_position().x1 for ax in result.axes)
    assert right < legend.get_window_extent().x0 / result.figure.bbox.width + 1e-6

    fig, given = plt.subplots(3, 1)
    before = [ax.get_position().bounds for ax in given]
    comparison.plot_simulated_trajectory(ax=list(given), show=False)
    assert fig.legends == []
    assert [ax.get_position().bounds for ax in given] == before
    assert given[0].get_legend() is not None
    plt.close(fig)


def test_an_evaluation_entry_is_drawn_when_it_keeps_its_runs():
    """Kept runs are drawn; without them it is left out; alone without them, the figure is refused."""
    from minilink.control import lqr as lqr_design

    problem = judged_move()
    law = lqr_design(problem.sys, np.eye(2), 0.1 * np.eye(1))
    scored = problem.evaluate(law, dt=0.05)
    kept = problem.evaluate(law, dt=0.05, trajectories=True)

    def labels(comparison):
        axes = comparison.plot_simulated_trajectory(show=False).axes
        return [line.get_label() for line in axes[0].get_lines()]

    assert "B" in labels(problem.compare(A=scored, B=law, dt=0.05))
    assert "A" not in labels(problem.compare(A=scored, B=law, dt=0.05))
    assert "A" in labels(problem.compare(A=kept, dt=0.05))
    with pytest.raises(ValueError, match="not drawn"):
        problem.compare(A=scored).plot_simulated_trajectory(show=False)


def plant_part(run):
    """The plant's part of a loop's run."""
    return run.of("plant") if "plant" in run.roles else run


def test_the_evaluations_keep_the_judges_runs_and_the_simulated_figure_draws_them():
    """One run per trial in each evaluation; one run is the solid line, several are faint lines; no nominal trajectory among them."""
    from minilink.control import lqr as lqr_design
    from minilink.planning.comparison import FAINT_ALPHA, FAINT_WIDTH

    law = lqr_design(DoubleIntegrator(), np.eye(2), 0.1 * np.eye(1))

    # Without draws: one run, the line on every row is that run
    problem = judged_move()
    comparison = problem.compare(Law=law, Plan=hand_plan(problem), dt=0.05)
    for evaluation in comparison.evaluations.values():
        assert len(evaluation.trajectories) == 1
    axes = comparison.plot_simulated_trajectory(show=False).axes
    for name in ("Law", "Plan"):
        run = plant_part(comparison.evaluations[name].trajectories[0])
        (line,) = [line for line in axes[0].get_lines() if line.get_label() == name]
        np.testing.assert_array_equal(line.get_xdata(), np.asarray(run.t))
        np.testing.assert_array_equal(line.get_ydata(), np.asarray(run.x)[0])
        assert line.get_alpha() in (None, 1.0)

    # Five draws: five faint lines per answer, and nothing else of theirs
    stochastic = judged_move(x0=Uniform([0.8, -0.2], [1.2, 0.2]))
    entries = dict(Law=law, Plan=hand_plan(stochastic))
    comparison = stochastic.compare(**entries, dt=0.05, n_trials=5)
    for evaluation in comparison.evaluations.values():
        assert len(evaluation.trajectories) == 5
    result = comparison.plot_simulated_trajectory(show=False)
    lines = result.axes[0].get_lines()
    answers = [line for line in lines if line.get_label() in ("Law", "Plan")]
    assert len(answers) == 2 * 5
    assert all(line.get_alpha() == FAINT_ALPHA for line in answers)
    assert all(line.get_linewidth() == FAINT_WIDTH for line in answers)

    # The legend's keys at full opacity, on matplotlib and on Plotly
    (legend,) = result.figure.legends
    assert all(handle.get_alpha() == 1.0 for handle in legend.legend_handles)
    pytest.importorskip("plotly")
    figure = comparison.plot_simulated_trajectory(renderer="plotly", show=False).figure
    assert figure.layout.legend.x > 1.0 and figure.layout.legend.xanchor == "left"
    keys = [trace for trace in figure.data if trace.showlegend]
    assert [trace.name for trace in keys] == ["Law", "Plan", "limits"]
    assert all(trace.opacity in (None, 1.0) for trace in keys)
    # A key shows or hides every trace of its answer: five runs on each of the three
    # rows (x₁, x₂, u)
    for name in ("Law", "Plan"):
        group = [trace for trace in figure.data if trace.legendgroup == name]
        faint = [trace for trace in group if trace.opacity == FAINT_ALPHA]
        assert len(faint) == 5 * 3


def test_the_nominal_figure_draws_the_answers_that_carry_one_in_their_colour():
    """The plan whole and solid, in the colour the simulated figure gives it; a feedback law left out; none at all is refused."""
    from minilink.control import lqr as lqr_design

    problem = judged_move()
    law = lqr_design(problem.sys, np.eye(2), 0.1 * np.eye(1))
    plan = hand_plan(problem)
    comparison = problem.compare(Law=law, Plan=plan, dt=0.05)

    def answer_lines(result):
        return {
            line.get_label(): line
            for line in result.axes[0].get_lines()
            if line.get_label() in ("Law", "Plan")
        }

    nominal = answer_lines(comparison.plot_nominal_trajectory(show=False))
    simulated = answer_lines(comparison.plot_simulated_trajectory(show=False))
    assert list(nominal) == ["Plan"]
    line = nominal["Plan"]
    np.testing.assert_array_equal(line.get_xdata(), np.asarray(plan.trajectory.t))
    np.testing.assert_array_equal(line.get_ydata(), np.asarray(plan.trajectory.x)[0])
    assert line.get_linestyle() == "-" and line.get_alpha() in (None, 1.0)
    assert line.get_color() == simulated["Plan"].get_color()

    with pytest.raises(ValueError, match="no answer carries a nominal trajectory"):
        problem.compare(Law=law, dt=0.05).plot_nominal_trajectory(show=False)


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
    planner.plot_trajectory(dt=0.05, show=False)
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
    assert given == [
        {
            "dt": planner.dt,
            "figsize": None,
            "renderer": "matplotlib",
            "show": False,
            "ax": None,
        },
        {
            "dt": 0.01,
            "figsize": None,
            "renderer": "matplotlib",
            "show": True,
            "ax": None,
        },
    ]


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


def test_the_planning_plots_take_one_keyword_set_and_return_a_plot_result():
    """Ruling 16: one written-out keyword set on the solution and every planner, a PlotResult back."""
    import inspect

    from minilink import LQRPlanner
    from minilink.graphical.common.plot_result import PlotResult
    from minilink.planning import PlanningSolution
    from minilink.planning.planner import Planner

    for name in ("plot_cost_to_go", "plot_control_law"):
        solution_set = inspect.signature(getattr(PlanningSolution, name)).parameters
        assert not any(p.kind is p.VAR_KEYWORD for p in solution_set.values()), name
        assert {"x_axis", "y_axis", "is_3d", "ax", "renderer", "show"} <= set(
            solution_set
        )
        for planner in (
            Planner,
            LQRPlanner,
            DynamicProgrammingPlanner,
            TabularLearningPlanner,
        ):
            planner_set = dict(inspect.signature(getattr(planner, name)).parameters)
            planner_set.pop("field", None)  # value iteration's one extra
            assert [(p.name, p.default) for p in planner_set.values()] == [
                (p.name, p.default) for p in solution_set.values()
            ], (planner.__name__, name)

    problem, vi, lqr = solved_pendulum()
    for solution in (vi, lqr):
        for drawn in (
            solution.plot_cost_to_go(show=False),
            solution.plot_cost_to_go(is_3d=True, show=False),
            solution.plot_control_law(show=False),
        ):
            assert isinstance(drawn, PlotResult) and drawn.figure is not None


def test_the_3d_cost_to_go_draws_the_surface_alone():
    """is_3d draws one figure, the surface: on the grid's nodes and on a sampled field."""
    import matplotlib.pyplot as plt

    problem, vi, lqr = solved_pendulum()
    for solution in (vi, lqr):
        plt.close("all")
        drawn = solution.plot_cost_to_go(is_3d=True, show=False)
        assert [panel.is_3d for panel in drawn.spec.panels] == [True]
        assert plt.get_fignums() == [drawn.figure.number]
    plt.close("all")


def test_a_grid_solution_is_drawn_on_its_nodes_unless_asked_for_another_mesh():
    problem, vi, lqr = solved_pendulum()
    grid = vi.controller.grid

    # On the nodes: the table's own extent and the failure cost atop the scale
    table = vi.plot_cost_to_go(show=False)
    assert table.axes.get_title() == "Cost-to-go"
    assert table.axes.collections[0].get_clim()[1] == 500.0
    np.testing.assert_allclose(
        table.axes.collections[0].get_array().max(),
        min(500.0, float(np.max(vi.record.J))),
    )
    # Another mesh, or a time: the field sampled
    sampled = vi.plot_cost_to_go(bounds=((-1.0, 1.0), (-1.0, 1.0)), show=False)
    assert sampled.axes.get_title() == "cost-to-go (value iteration)"
    assert sampled.axes.get_xlim() == (-1.0, 1.0)
    assert grid.n_nodes == 21 * 21

    # A title replaced on the law's table
    law = vi.plot_control_law(title="π", show=False)
    assert law.axes.get_title() == "π"


def test_the_planning_plots_draw_with_plotly_and_take_a_run_over_them():
    pytest.importorskip("plotly")
    problem, vi, lqr = solved_pendulum()
    loop = lqr.controller @ problem.sys
    traj = loop.compute_trajectory(tf=1.0, verbose=False).of("plant")
    for solution in (vi, lqr):
        J = solution.plot_cost_to_go(renderer="plotly", show=False)
        assert [type(trace).__name__ for trace in J.figure.data] == ["Heatmap"]
        law = solution.plot_control_law(trajectory=traj, renderer="plotly", show=False)
        kinds = [type(trace).__name__ for trace in law.figure.data]
        assert kinds == ["Heatmap", "Scatter", "Scatter", "Scatter"]
    surface = lqr.plot_cost_to_go(is_3d=True, renderer="plotly", show=False)
    assert [type(trace).__name__ for trace in surface.figure.data] == ["Surface"]


def test_the_planning_plots_overlay_a_run_and_refuse_what_they_cannot_draw():
    problem, vi, lqr = solved_pendulum()
    loop = lqr.controller @ problem.sys
    traj = loop.compute_trajectory(tf=1.0, verbose=False).of("plant")
    for solution in (vi, lqr):
        drawn = solution.plot_cost_to_go(trajectory=traj, show=False)
        assert any(line.get_label() == "trajectory" for line in drawn.axes.get_lines())
        drawn = solution.plot_control_law(trajectory=traj, show=False)
        assert any(line.get_label() == "trajectory" for line in drawn.axes.get_lines())
        with pytest.raises(ValueError, match="cannot be used together"):
            solution.plot_cost_to_go(is_3d=True, trajectory=traj, show=False)
    with pytest.raises(ValueError, match="anchor"):
        lqr.plot_control_law(anchor=[np.pi, 0.0], show=False)
    assert vi.plot_control_law(anchor=[np.pi, 0.0], show=False).figure is not None


# --- the solution's params: the planner's complete tree, solved again ---


ROUND_TRIP_NAMES = (
    "value iteration",
    "lqr",
    "rrt",
    "rrt*",
    "trajectory optimization",
    "mppi",
    "tabular learning",
)
NEEDS_JAX = ("mppi",)


def round_trip_planner(name):
    """``(planner, solve keywords)`` for the planner ``name``, small and seeded (built only when asked: MPPI needs JAX)."""
    from minilink import LQRPlanner
    from minilink.planning import MPPIPlanner
    from minilink.planning.search.rrt_star import RRTStarPlanner

    grid = dict(x_grid=(11, 11), u_grid=(3,), dt=0.2)
    infinite = move_problem().replace(horizon=np.inf, discount_rate=0.5)
    compass = [
        np.array([np.cos(a), np.sin(a)])
        for a in np.linspace(0.0, 2.0 * np.pi, 8, endpoint=False)
    ]

    def obstacles():
        problem, _ = make_holonomic_obstacle_problem()
        return problem

    def extender():
        return KinodynamicExtender(controls=compass, horizon=0.6, n_steps=6)

    builders = {
        "value iteration": lambda: (DynamicProgrammingPlanner(infinite, **grid), {}),
        "lqr": lambda: (LQRPlanner(move_problem()), {}),
        "rrt": lambda: (
            RRTPlanner(obstacles(), extender(), max_iterations=400, seed=3),
            {},
        ),
        "rrt*": lambda: (
            RRTStarPlanner(obstacles(), extender(), max_iterations=200, seed=3),
            {},
        ),
        "trajectory optimization": lambda: (trajopt(move_problem(), collocation), {}),
        "mppi": lambda: (
            MPPIPlanner(move_problem(), dt=0.1, n_rollouts=64, n_iterations=2),
            {},
        ),
        "tabular learning": lambda: (
            TabularLearningPlanner(infinite, solver="euler", seed=2, **grid),
            {"n_episodes": 20},
        ),
    }

    return builders[name]()


def solved_numbers(solution):
    """What a solve computes, as bytes: the controller's numbers, the plan and the record but its timings."""
    import pickle
    from dataclasses import fields

    record = solution.record
    numbers = {
        name: getattr(record, name)
        for name in (f.name for f in fields(record))
        if not name.endswith("time_s") and name not in ("optimization_result",)
    }
    controller = {
        key: value
        for key, value in (solution.controller.params or {}).items()
        if not isinstance(value, dict)
    }
    plan = solution.trajectory
    plan = None if plan is None else (plan.t, plan.x, plan.u)

    return pickle.dumps((sorted(controller.items()), plan, sorted(numbers.items())))


@pytest.mark.parametrize(
    "name",
    [
        pytest.param(name, marks=pytest.mark.jax) if name in NEEDS_JAX else name
        for name in ROUND_TRIP_NAMES
    ],
)
def test_a_solution_is_solved_again_from_its_params(name):
    """Ruling 24: solution.params is the planner's complete tree; solve(params=) reproduces it."""
    planner, keywords = round_trip_planner(name)
    first = planner.solve(**keywords)

    assert set(first.params) == set(planner.params), name
    assert {"method", "seed"} <= set(first.params)
    again = planner.solve(params=first.params, **keywords)

    assert solved_numbers(again) == solved_numbers(first), name


@pytest.mark.jax
def test_a_learned_solution_is_learned_again_from_its_params():
    pytest.importorskip("jax")
    from minilink.learning.reinforcement_learning import ReinforcementLearningPlanner

    problem = move_problem().replace(horizon=np.inf, discount_rate=0.5)
    problem.sys.state.lower = np.array([-3.0, -3.0])
    problem.sys.state.upper = np.array([3.0, 3.0])
    problem = problem.replace(X=problem.sys.state.box)
    planner = ReinforcementLearningPlanner(
        problem,
        dt=0.1,
        hidden=(8,),
        n_rollouts=2,
        n_steps_per_update=16,
        batch_size=16,
        episode_length=2.0,
    )
    first = planner.solve(n_timesteps=64)
    again = planner.solve(n_timesteps=64, params=first.params)

    assert set(first.params) == set(planner.params)
    assert {"plant", "cost", "method", "seed"} <= set(first.params)
    for key, weights in first.controller.params["mlp"].items():
        np.testing.assert_array_equal(again.controller.params["mlp"][key], weights)

    # The tree's problem leaves reach the planner rebuilt there (the maintainer, 2026-11-11)
    import copy

    from minilink.planning.planner import planner_at

    edited = copy.deepcopy(first.params)
    edited["x0"] = np.asarray(edited["x0"]) + 0.5
    rebuilt = planner_at(planner, edited)
    np.testing.assert_array_equal(rebuilt.problem.x0, edited["x0"])
