"""A held compiled planner never compiles again: ``pv.solve`` at new numbers runs its executables (DESIGN §4, rule 6)."""

import numpy as np
import pytest

jax = pytest.importorskip("jax")

from minilink import Pendulum, PlanningProblem, QuadraticCost  # noqa: E402
from minilink.core.backends import configure_jax  # noqa: E402
from minilink.core.sets import SingletonSet  # noqa: E402
from minilink.planning.trajectory_optimization.direct_collocation import (  # noqa: E402
    DirectCollocationTranscription,
)
from minilink.planning.trajectory_optimization.path_integral import (  # noqa: E402
    MPPIPlanner,
)
from minilink.planning.trajectory_optimization.planner import (  # noqa: E402
    TrajectoryOptimizationPlanner,
)

pytestmark = [pytest.mark.optional, pytest.mark.jax]

COMPILES = []


def count_compile(name, secs, **kwargs):
    """Record one XLA compilation (JAX reports every event duration here)."""
    if name == "/jax/core/compile/backend_compile_duration":
        COMPILES.append(name)


jax.monitoring.register_event_duration_secs_listener(count_compile)


def swing_problem():
    plant = Pendulum()
    plant.inputs["u"].lower_bound[0] = -20.0
    plant.inputs["u"].upper_bound[0] = 20.0
    goal = np.array([np.pi, 0.0])
    return PlanningProblem(
        sys=plant,
        horizon=1.0,
        x0=np.zeros(2),
        Xf=SingletonSet(goal),
        cost=QuadraticCost.from_system(plant, R=np.array([[0.01]]), x_bar=goal),
        X=plant.state.box,
        U=plant.input_box,
    )


def at(tree, k):
    """The tree at the k-th new numbers: a new start and a new mass."""
    tree["x0"] = np.array([0.02 * k, -0.01 * k])
    tree["plant"]["m"] = 1.0 + 0.05 * k
    return tree


def compiles_of(pv):
    """The compiles of two warm-started solves at new numbers, after the first."""
    z, _ = pv.solve(at(pv.params, 1))
    COMPILES.clear()
    for k in (2, 3):
        z, _ = pv.solve(at(pv.params, k), z)
    return len(COMPILES)


def test_the_compiled_trajectory_planner_at_new_numbers_compiles_nothing():
    configure_jax(enable_x64=True)
    planner = TrajectoryOptimizationPlanner(
        swing_problem(),
        transcription=DirectCollocationTranscription(n_steps=8),
        backend="jax",
        method="slsqp",
        options={"maxiter": 20, "disp": False},
    )

    assert compiles_of(planner.compile()) == 0


def test_the_compiled_path_integral_planner_at_new_numbers_compiles_nothing():
    pv = MPPIPlanner(swing_problem(), dt=0.1, n_rollouts=32, noise=4.0).compile()

    assert compiles_of(pv) == 0
    assert pv.params["plant"]["m"] == 1.0  # its params is a copy


def test_the_problem_evaluator_at_new_numbers_compiles_nothing():
    from minilink import PID

    configure_jax(enable_x64=True)
    pe = swing_problem().compile(PID(Kp=5.0, Kd=1.0), dt=0.05, backend="jax")
    p = pe.params
    pe.evaluate(p)
    COMPILES.clear()
    for k in (2, 3):
        p["plant"]["m"] = 1.0 + 0.05 * k
        p["controller"]["Kp"] = np.array([5.0 + k])
        pe.evaluate(p)

    assert len(COMPILES) == 0


def test_a_cost_edit_inside_the_problem_is_seen_by_the_next_value_iteration_solve():
    from minilink import DynamicProgrammingPlanner

    problem = swing_problem().replace(Xf=None, horizon=np.inf)
    for backend in ("numpy", "jax"):
        planner = DynamicProgrammingPlanner(
            problem,
            x_grid=(9, 9),
            u_grid=(3,),
            dt=0.1,
            backend=backend,
            max_iterations=5,
        )
        planner.solve()
        J_before = planner.result.J.copy()
        problem.cost.params["R"] = 100.0 * problem.cost.params["R"]
        planner.solve()
        J_after = planner.result.J.copy()
        problem.cost.params["R"] = problem.cost.params["R"] / 100.0

        assert not np.allclose(J_before, J_after), backend


def test_one_planner_solve_compiles_the_plant_once():
    from unittest import mock

    problem = swing_problem()
    planner = TrajectoryOptimizationPlanner(
        problem,
        transcription=DirectCollocationTranscription(n_steps=8),
        backend="numpy",
        method="slsqp",
        options={"maxiter": 5, "disp": False},
    )
    compile_system = problem.sys.compile
    with mock.patch.object(
        type(problem.sys),
        "compile",
        autospec=True,
        side_effect=lambda self, *a, **k: compile_system(*a, **k),
    ) as compiles:
        planner.solve(record_history=True)

    assert compiles.call_count == 1
