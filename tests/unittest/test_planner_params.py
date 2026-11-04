"""Every planner's numbers: one tree, overridden per solve (DESIGN §6, the four kinds of argument)."""

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

    at_tree = solve_of(planner, params={"method": numbers})
    built_there = solve_of(build(**numbers))

    np.testing.assert_allclose(fingerprint(at_tree), fingerprint(built_there))
    for key, value in before.items():
        np.testing.assert_array_equal(planner.params["method"][key], value)


def test_a_changed_problem_derives_its_own_numbers():
    """The rebuild keeps the numbers given, never the defaults derived: the discount follows the rate."""
    vi = DynamicProgrammingPlanner(regulation(), max_iterations=30, **GRID)
    at_faster_rate = vi.at({"discount_rate": 1.0})

    np.testing.assert_allclose(at_faster_rate.gamma, np.exp(-1.0 * GRID["dt"]))
    np.testing.assert_allclose(vi.gamma, np.exp(-0.5 * GRID["dt"]))


def test_an_unknown_number_is_refused_with_the_known_ones():
    vi = DynamicProgrammingPlanner(regulation(), max_iterations=30, **GRID)
    with pytest.raises(ValueError, match="its numbers are"):
        vi.solve(params={"method": {"learning_rate": 0.1}})
