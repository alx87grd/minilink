"""One last answer per object (RULES 2.4): only the shortcuts read ``sys.traj`` and ``planner.last_solution``."""

import ast
from pathlib import Path

PACKAGE = Path(__file__).resolve().parents[2] / "minilink"

# Where the library reads each last answer: the object's own convenience verbs, which
# default to it, and nothing else. A tool takes the result it needs, or the one a run
# returned; a new reader here is a tool leaning on hidden state.
SHORTCUTS = {
    "last_solution": {
        "planning/planner.py": {"require_solution"},
        "planning/dynamic_programming/dp.py": {"DynamicProgrammingPlanner.tables"},
        "learning/reinforcement_learning/tabular.py": {"TabularLearningPlanner.tables"},
        "planning/search/plotting.py": {"plot_tree", "animate_search.update"},
    },
    "traj": {
        "core/discrete/sampled_diagram.py": {"SampledDiagramSystem.animate"},
        "core/facades/animation.py": {"SystemAnimation.animate"},
        "core/facades/plotting.py": {
            "SystemPlotting.plot_trajectory",
            "DynamicPlotting.plot_phase_plane",
            "StepPlotting.plot_rollout",
        },
        "core/facades/simulation.py": {"scored_trajectory"},
        # The law's snapshot shares its last run instead of copying it
        "learning/reinforcement_learning/planner.py": {"snapshot_of"},
    },
}


def readers_of(attribute):
    """``{module: {function}}``: where the library reads ``.<attribute>``, demos under ``__main__`` aside."""
    found = {}
    for path in sorted(PACKAGE.rglob("*.py")):
        tree = ast.parse(path.read_text())
        module = path.relative_to(PACKAGE).as_posix()
        for where, node in reads(tree, attribute):
            found.setdefault(module, set()).add(where)
    return found


def reads(tree, attribute, scope=()):
    """``(qualified function, node)`` for every load of ``.<attribute>`` under ``tree``."""
    for node in ast.iter_child_nodes(tree):
        if isinstance(node, ast.If) and is_main_guard(node):
            continue
        inner = scope
        if isinstance(node, (ast.ClassDef, ast.FunctionDef, ast.AsyncFunctionDef)):
            inner = (*scope, node.name)
        if (
            isinstance(node, ast.Attribute)
            and node.attr == attribute
            and isinstance(node.ctx, ast.Load)
        ):
            yield ".".join(scope), node
        yield from reads(node, attribute, inner)


def is_main_guard(node):
    test = node.test
    return (
        isinstance(test, ast.Compare)
        and isinstance(test.left, ast.Name)
        and test.left.id == "__name__"
    )


def test_only_the_shortcuts_read_the_last_answer():
    for attribute, allowed in SHORTCUTS.items():
        assert readers_of(attribute) == allowed, attribute


def test_a_kept_trajectory_optimization_solution_keeps_its_own_account():
    """The solve's raw answer, history and timings live in its record, not on the planner."""
    import numpy as np

    from minilink import (
        Pendulum,
        PlanningProblem,
        QuadraticCost,
        TrajectoryOptimizationPlanner,
    )

    plant = Pendulum()
    problem = PlanningProblem(
        plant,
        x0=np.zeros(2),
        cost=QuadraticCost.from_system(plant, x_bar=np.array([0.5, 0.0])),
        U=plant.input_box,
        horizon=1.0,
    )
    planner = TrajectoryOptimizationPlanner(problem, n_steps=8, options={"maxiter": 20})
    first = planner.solve(record_history=True)
    second = planner.solve(guess=first, record_history=True)

    assert second is planner.last_solution
    for solution in (first, second):
        record = solution.record
        assert record.optimization_result is not None
        assert record.cost == record.optimization_result.cost
        assert len(record.history) > 0
        assert record.step_time_s is not None
    assert first.record.history is not second.record.history
    assert first.record.optimization_result is not second.record.optimization_result


# Every planner's record carries the solve's wall time under one name (the maintainer's
# ruling 17 of 2026-11-10), measured at the user's verb
RECORDS = (
    "minilink.planning.dynamic_programming.dp.ValueIterationRecord",
    "minilink.planning.dynamic_programming.lqr.RiccatiRecord",
    "minilink.planning.search.rrt.TreeSearchRecord",
    "minilink.planning.trajectory_optimization.path_integral.MPPIRecord",
    "minilink.planning.trajectory_optimization.planner.TrajectoryOptimizationRecord",
    "minilink.learning.reinforcement_learning.tabular.TabularLearningRecord",
    "minilink.learning.reinforcement_learning.planner.ReinforcementLearningRecord",
)


def test_every_planner_record_carries_solve_time_s():
    import dataclasses
    import importlib

    for path in RECORDS:
        module, name = path.rsplit(".", 1)
        record = getattr(importlib.import_module(module), name)
        fields = {f.name for f in dataclasses.fields(record)}
        assert "solve_time_s" in fields, path


def test_a_solve_times_itself_on_its_record():
    import numpy as np

    from minilink import (
        DynamicProgrammingPlanner,
        LQRPlanner,
        Pendulum,
        PlanningProblem,
        QuadraticCost,
    )
    from minilink.dynamics.catalog.equations import DoubleIntegrator
    from minilink.learning.reinforcement_learning import TabularLearningPlanner
    from minilink.planning.search.extenders import KinodynamicExtender
    from minilink.planning.search.rrt import RRTPlanner
    from tests.unittest.planning_helpers import make_holonomic_obstacle_problem

    plant = DoubleIntegrator()
    plant.state.lower, plant.state.upper = np.array([-2.0, -2.0]), np.array([2.0, 2.0])
    plant.inputs["u"].lower, plant.inputs["u"].upper = np.array([-1.0]), np.array([1.0])
    grid_problem = PlanningProblem(
        plant,
        x0=[1.0, 0.0],
        cost=QuadraticCost.from_system(plant),
        X=plant.state.box,
        U=plant.input_box,
    )
    grid = dict(x_grid=(5, 5), u_grid=(3,), dt=0.2)
    pendulum = Pendulum()
    regulator = PlanningProblem(
        pendulum, x0=[0.1, 0.0], cost=QuadraticCost.from_system(pendulum)
    )
    search_problem, _ = make_holonomic_obstacle_problem()
    extender = KinodynamicExtender(controls=[[1.0, 0.0]], horizon=0.5, n_steps=2)

    solutions = (
        DynamicProgrammingPlanner(grid_problem, gamma=0.9, **grid).solve(),
        DynamicProgrammingPlanner(grid_problem, gamma=0.9, **grid).solve_steps(3),
        LQRPlanner(regulator).solve(),
        RRTPlanner(search_problem, extender=extender, max_nodes=5).solve(),
        TabularLearningPlanner(grid_problem, gamma=0.9, **grid).solve(n_episodes=3),
    )
    for solution in solutions:
        solve_time_s = solution.record.solve_time_s
        assert isinstance(solve_time_s, float) and solve_time_s >= 0.0
        assert "solved in" in str(solution.record)
