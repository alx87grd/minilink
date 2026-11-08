"""Every planner's numbers: one tree, overridden per solve (DESIGN §6, the four kinds of argument)."""

from unittest import mock

import numpy as np
import pytest

from minilink import (
    DynamicProgrammingPlanner,
    LQRPlanner,
    Pendulum,
    PlanningProblem,
    QuadraticCost,
    RRTPlanner,
    RRTStarPlanner,
)
from minilink.core.sets import BallSet
from minilink.learning import TabularLearningPlanner
from minilink.planning.planner import planner_at
from tests.unittest.planning_helpers import edited

X_UP = np.array([np.pi, 0.0])


def bounded_pendulum():
    plant = Pendulum()
    plant.state.lower_bound = np.array([-np.pi, -6.0])
    plant.state.upper_bound = np.array([np.pi, 6.0])
    return plant


def regulation():
    plant = bounded_pendulum()
    cost = QuadraticCost.from_system(plant, Q=np.eye(2), R=0.1 * np.eye(1))
    return PlanningProblem(
        plant,
        x0=np.zeros(2),
        cost=cost,
        X=plant.state.box,
        U=plant.input_box,
        discount_rate=0.5,
    )


def reach():
    plant = bounded_pendulum()
    cost = QuadraticCost.from_system(plant, x_bar=X_UP)
    return PlanningProblem(
        plant,
        x0=np.zeros(2),
        cost=cost,
        X=plant.state.box,
        U=plant.input_box,
        Xf=BallSet(X_UP, 0.5),
        horizon=5.0,
    )


GRID = dict(x_grid=(9, 9), u_grid=(3,), dt=0.1)

PLANNERS = {
    "value iteration": (
        lambda **k: DynamicProgrammingPlanner(
            regulation(), max_iterations=30, **GRID, **k
        ),
        {"gamma": 0.9},
    ),
    "tabular": (
        lambda **k: TabularLearningPlanner(regulation(), **GRID, **k),
        {"eta": 0.5},
    ),
    "LQR": (
        lambda **k: LQRPlanner(reach().replace(Xf=None), **k),
        {"x_bar": X_UP},
    ),
    "RRT": (lambda **k: RRTPlanner(reach(), max_nodes=60, **k), {"goal_bias": 0.3}),
    "RRT*": (
        lambda **k: RRTStarPlanner(reach(), max_nodes=40, **k),
        {"radius_constant": 2.0},
    ),
}


def solve_of(planner, **kwargs):
    """One solve: the tabular learner learns a few episodes, the others solve."""
    if isinstance(planner, TabularLearningPlanner):
        return planner.solve(20, **kwargs)
    return planner.solve(**kwargs)


def fingerprint(solution):
    """The numbers a solution carries: its cost-to-go at a few states, else its plan."""
    states = [np.array([a, 0.0]) for a in (-1.0, 0.5, 2.0)]
    if solution.cost_to_go is not None:
        return np.array([solution.cost_to_go(x) for x in states])
    return np.asarray(solution.trajectory.x)


@pytest.mark.parametrize("name", list(PLANNERS))
def test_every_planner_answers_its_tree(name):
    build, numbers = PLANNERS[name]
    planner = build()
    tree = planner.params

    assert {"plant", "cost", "x0", "t0", "method", "seed"} <= set(tree)
    assert set(numbers) <= set(tree["method"])


@pytest.mark.parametrize("name", list(PLANNERS))
def test_a_solve_at_a_tree_is_the_planner_built_there(name):
    build, numbers = PLANNERS[name]
    planner = build()
    before = {
        key: np.copy(value)
        for key, value in planner.params["method"].items()
        if value is not None
    }

    at_tree = solve_of(planner, params=edited(planner.params, {"method": numbers}))
    built_there = solve_of(build(**numbers))

    np.testing.assert_allclose(fingerprint(at_tree), fingerprint(built_there))
    for key, value in before.items():
        np.testing.assert_array_equal(planner.params["method"][key], value)


def test_a_changed_problem_derives_its_own_numbers():
    """The rebuild keeps the numbers given, never the defaults derived: the discount follows the rate."""
    vi = DynamicProgrammingPlanner(regulation(), max_iterations=30, **GRID)
    at_faster_rate = planner_at(vi, edited(vi.params, {"discount_rate": 1.0}))

    np.testing.assert_allclose(at_faster_rate.gamma, np.exp(-1.0 * GRID["dt"]))
    np.testing.assert_allclose(vi.gamma, np.exp(-0.5 * GRID["dt"]))


def test_an_unknown_number_is_refused_with_the_known_ones():
    vi = DynamicProgrammingPlanner(regulation(), max_iterations=30, **GRID)
    with pytest.raises(ValueError, match="its numbers are"):
        vi.solve(params={"method": {"learning_rate": 0.1}})


@pytest.mark.parametrize("name", ["value iteration", "tabular"])
def test_the_shortcuts_read_the_solution_a_solve_at_a_tree_returned(name):
    """One last answer (RULES 2.4): the planner's shortcuts follow ``last_solution``, never older tables."""
    build, numbers = PLANNERS[name]
    planner = build()
    solve_of(planner)
    solution = solve_of(planner, params=edited(planner.params, {"method": numbers}))
    x = np.array([0.5, 0.0])

    assert solution is planner.last_solution
    assert planner.cost_to_go(x) == solution.cost_to_go(x)
    assert planner.controller is solution.controller


def test_every_planner_takes_params_by_keyword_only():
    """One ``solve`` shape (DESIGN §6): ``params=`` by keyword on every planner that takes it."""
    import inspect

    from minilink import TrajectoryOptimizationPlanner
    from minilink.planning import MPPIPlanner

    planners = (
        DynamicProgrammingPlanner,
        LQRPlanner,
        RRTPlanner,
        RRTStarPlanner,
        TabularLearningPlanner,
        TrajectoryOptimizationPlanner,
        MPPIPlanner,
    )
    for planner in planners:
        parameter = inspect.signature(planner.solve).parameters["params"]
        assert parameter.kind is inspect.Parameter.KEYWORD_ONLY, planner.__name__


WORKFLOW = {"verbose", "callback", "live_plot", "record_history", "history_stride"}


@pytest.mark.parametrize("name", list(PLANNERS))
def test_a_rebuilt_planner_inherits_no_display(name):
    build, _ = PLANNERS[name]
    kwargs = build().build_kwargs()

    assert not {key for key in kwargs if key in WORKFLOW or key.startswith("live_plot")}


def test_a_verbose_solve_at_a_tree_reports_once_from_the_verb(capsys):
    """``solve(params=, verbose=True)``: the rebuilt planner gets no display keyword, its solve prints once."""
    from unittest import mock

    build, numbers = PLANNERS["value iteration"]
    planner = build()
    constructor = DynamicProgrammingPlanner.__init__
    with mock.patch.object(
        DynamicProgrammingPlanner, "__init__", autospec=True, side_effect=constructor
    ) as built:
        planner.solve(params=edited(planner.params, {"method": numbers}), verbose=True)

    assert built.call_count == 1
    assert not set(built.call_args.kwargs) & WORKFLOW
    assert capsys.readouterr().out.count("Bellman equation solved!") == 1


def test_the_grids_table_options_survive_a_discount_and_a_tree():
    """A grid built with precompute=False keeps it through gamma= and solve(params=)."""
    from minilink.planning.dynamic_programming.discretizer import StateSpaceGrid

    problem = regulation()
    grid = StateSpaceGrid(problem, x_grid=(9, 9), u_grid=(3,), dt=0.1, precompute=False)
    planner = DynamicProgrammingPlanner(problem, grid=grid, gamma=0.9, max_iterations=5)
    assert planner.grid is not grid and not planner.grid.precompute

    with mock.patch.object(
        DynamicProgrammingPlanner,
        "__init__",
        autospec=True,
        side_effect=DynamicProgrammingPlanner.__init__,
    ) as built:
        planner.solve(params=edited(planner.params, {"method": {"gamma": 0.8}}))
    rebuilt = built.call_args.args[0]
    assert rebuilt.grid.precompute is False and rebuilt.grid.precomputed is False
    assert rebuilt.gamma == 0.8
