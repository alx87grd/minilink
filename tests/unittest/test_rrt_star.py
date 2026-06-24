"""Tests for RRT* motion planning."""

import numpy as np
import pytest

from minilink.planning.search.extenders import SteeringExtender
from minilink.planning.search.rrt import RRTOptions, RRTPlanner
from minilink.planning.search.rrt_star import RRTStarOptions, RRTStarPlanner
from minilink.planning.search.steering import StraightLineSteering
from tests.unittest.test_rrt import X_GOAL, make_problem


def make_steering_extender():
    return SteeringExtender(
        StraightLineSteering(speed=1.0), max_distance=0.6, resolution=0.05
    )


def path_cost(planner) -> float:
    node = planner.solution_node
    if node is None:
        return float("inf")
    return float(node.cost)


def test_rrt_star_reaches_goal():
    problem, X = make_problem()
    planner = RRTStarPlanner(
        problem,
        extender=make_steering_extender(),
        options=RRTStarOptions(seed=0, goal_tolerance=0.5, max_nodes=4000),
    )
    traj = planner.compute_solution()
    assert planner.reached_goal
    assert np.linalg.norm(traj.x[:, -1] - X_GOAL) < 0.5
    assert all(X.contains(traj.x[:, i]) for i in range(traj.x.shape[1]))


def test_rrt_star_reaches_goal_with_kdtree_backend():
    problem, X = make_problem()
    planner = RRTStarPlanner(
        problem,
        extender=make_steering_extender(),
        options=RRTStarOptions(
            seed=0, goal_tolerance=0.5, max_nodes=4000, nearest_backend="kd_tree"
        ),
    )
    traj = planner.compute_solution()
    assert planner.reached_goal
    assert np.linalg.norm(traj.x[:, -1] - X_GOAL) < 0.5
    assert all(X.contains(traj.x[:, i]) for i in range(traj.x.shape[1]))


def test_rrt_star_improves_path_cost_over_rrt():
    problem, _ = make_problem()
    extender = make_steering_extender()
    star_costs = []
    rrt_costs = []
    for seed in range(12):
        options = RRTStarOptions(seed=seed, goal_tolerance=0.5, max_nodes=3000)
        rrt = RRTPlanner(problem, extender=extender, options=options)
        rrt.compute_solution()
        star = RRTStarPlanner(problem, extender=extender, options=options)
        star.compute_solution()
        if rrt.reached_goal and star.reached_goal:
            rrt_costs.append(path_cost(rrt))
            star_costs.append(path_cost(star))

    assert len(star_costs) >= 8
    assert np.mean(star_costs) <= np.mean(rrt_costs)
    assert sum(s <= r for s, r in zip(star_costs, rrt_costs)) >= 4


def test_rewire_false_is_at_least_as_costly():
    problem, _ = make_problem()
    extender = make_steering_extender()
    with_rewire = RRTStarPlanner(
        problem,
        extender=extender,
        options=RRTStarOptions(
            seed=5, goal_tolerance=0.5, max_nodes=2500, rewire=True
        ),
    )
    without_rewire = RRTStarPlanner(
        problem,
        extender=extender,
        options=RRTStarOptions(
            seed=5, goal_tolerance=0.5, max_nodes=2500, rewire=False
        ),
    )
    with_rewire.compute_solution()
    without_rewire.compute_solution()
    assert with_rewire.reached_goal
    assert without_rewire.reached_goal
    assert path_cost(with_rewire) <= path_cost(without_rewire) + 1e-9


def test_rrt_star_is_deterministic():
    problem, _ = make_problem()
    extender = make_steering_extender()
    options = RRTStarOptions(seed=11, goal_tolerance=0.5, max_nodes=2000)

    def run():
        planner = RRTStarPlanner(problem, extender=extender, options=options)
        traj = planner.compute_solution()
        return traj, path_cost(planner)

    (traj_a, cost_a), (traj_b, cost_b) = run(), run()
    assert np.allclose(traj_a.x, traj_b.x)
    assert cost_a == pytest.approx(cost_b)


def test_rrt_star_infers_rewire_eta_from_extender():
    problem, _ = make_problem()
    planner = RRTStarPlanner(
        problem,
        extender=make_steering_extender(),
        options=RRTStarOptions(seed=0, max_nodes=10),
    )
    assert planner._rewire_eta() == pytest.approx(0.6)


def test_rrt_star_requires_rewire_eta_for_unknown_extender():
    problem, _ = make_problem()

    class DummyExtender:
        def propose(self, *args, **kwargs):
            return []

    planner = RRTStarPlanner(
        problem,
        extender=DummyExtender(),
        options=RRTStarOptions(seed=0),
    )
    with pytest.raises(ValueError, match="rewire_eta"):
        planner._rewire_eta()


def test_search_callback_invoked_on_rrt():
    calls = []

    class RecordingCallback:
        def __call__(self, step):
            calls.append(step)

    problem, _ = make_problem()
    planner = RRTPlanner(
        problem,
        extender=make_steering_extender(),
        options=RRTStarOptions(
            seed=0,
            goal_tolerance=0.5,
            max_nodes=200,
            callback=RecordingCallback(),
            live_plot_every=1,
        ),
    )
    planner.compute_solution()
    assert calls
    assert all(step.iteration > 0 for step in calls)
    assert calls[-1].phase == "explore"


def test_live_plot_after_goal_only_skips_explore_phase():
    calls = []

    class RecordingCallback:
        def __call__(self, step):
            calls.append(step.phase)

    problem, _ = make_problem()
    planner = RRTStarPlanner(
        problem,
        extender=make_steering_extender(),
        options=RRTStarOptions(
            seed=1,
            goal_tolerance=0.5,
            max_nodes=2500,
            optimize_after_goal=True,
            convergence_patience=100,
            callback=RecordingCallback(),
            live_plot_every=1,
            live_plot_after_goal_only=True,
        ),
    )
    planner.compute_solution()
    assert planner.reached_goal
    assert calls
    assert all(phase == "optimize" for phase in calls)


def test_live_plot_option_builds_callback():
    problem, _ = make_problem()
    planner = RRTPlanner(
        problem,
        extender=make_steering_extender(),
        options=RRTOptions(
            seed=0,
            max_nodes=10,
            live_plot=True,
            live_plot_every=1,
        ),
    )
    callback = planner._resolve_search_callback()
    from minilink.planning.search.live_plot import LiveSearchPlotCallback

    assert isinstance(callback, LiveSearchPlotCallback)


def test_optimize_after_goal_runs_longer_and_refines_cost():
    problem, _ = make_problem()
    extender = make_steering_extender()
    base = dict(seed=4, goal_tolerance=0.5, max_nodes=3500, goal_bias=0.05)

    first_hit = RRTStarPlanner(
        problem,
        extender=extender,
        options=RRTStarOptions(**base, optimize_after_goal=False),
    )
    optimized = RRTStarPlanner(
        problem,
        extender=extender,
        options=RRTStarOptions(
            **base,
            optimize_after_goal=True,
            cost_tol=0.05,
            convergence_patience=400,
        ),
    )
    first_hit.compute_solution()
    optimized.compute_solution()

    assert first_hit.reached_goal
    assert optimized.reached_goal
    assert optimized.iterations > first_hit.iterations
    assert path_cost(optimized) <= path_cost(first_hit) + 1e-9


def test_convergence_patience_stops_search():
    problem, _ = make_problem()
    planner = RRTStarPlanner(
        problem,
        extender=make_steering_extender(),
        options=RRTStarOptions(
            seed=6,
            goal_tolerance=0.5,
            max_nodes=8000,
            optimize_after_goal=True,
            cost_tol=0.05,
            convergence_patience=50,
        ),
    )
    planner.compute_solution()
    assert planner.reached_goal
    assert planner.converged
    assert planner.iterations < 8000


def test_record_history_for_animation():
    problem, _ = make_problem()
    planner = RRTStarPlanner(
        problem,
        extender=make_steering_extender(),
        options=RRTStarOptions(
            seed=0,
            goal_tolerance=0.5,
            max_nodes=800,
            record_history=True,
            history_stride=25,
        ),
    )
    planner.compute_solution()
    assert len(planner.history) >= 2
    assert planner.history[0].iteration == 1
    assert all(frame.tree_edges for frame in planner.history[1:])
