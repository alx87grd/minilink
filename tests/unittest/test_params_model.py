"""One parameters model (docs/plans/one-params-model.md): the defects its first step closes, one test each."""

import copy
import pickle

import numpy as np
import pytest

from minilink import (
    PID,
    Pendulum,
    PlanningProblem,
    QuadraticCost,
    TimeCost,
)


def regulation_problem(plant):
    cost = QuadraticCost.from_system(plant, R=0.1 * np.eye(1))
    return PlanningProblem(plant, x0=np.array([0.5, 0.0]), cost=cost, horizon=2.0)


# A system deep-copies and pickles after a run


def test_a_simulated_plant_deep_copies_and_pickles():
    plant = Pendulum()
    plant.compute_trajectory(tf=0.1, verbose=False)

    twin = copy.deepcopy(plant)
    again = pickle.loads(pickle.dumps(plant))

    np.testing.assert_array_equal(twin.traj.x, plant.traj.x)
    np.testing.assert_array_equal(again.traj.x, plant.traj.x)
    with pytest.raises(TypeError):
        twin.traj.signals["new"] = np.zeros((1, 2))  # the record stays read-only


@pytest.mark.optional
@pytest.mark.jax
def test_a_plant_simulated_on_jax_deep_copies():
    pytest.importorskip("jax")
    plant = Pendulum()
    plant.compute_trajectory(tf=0.1, backend="jax", verbose=False)

    twin = copy.deepcopy(plant)

    np.testing.assert_array_equal(np.asarray(twin.traj.x), np.asarray(plant.traj.x))


# A composite refuses a part's tree that would wipe its other numbers


def test_a_problem_refuses_a_part_tree_missing_entries_and_keeps_them():
    plant = Pendulum()
    problem = regulation_problem(plant)
    keys = sorted(problem.cost.params)

    with pytest.raises(ValueError, match="missing"):
        problem.params = {"cost": {"R": np.array([[10.0]])}}
    with pytest.raises(ValueError, match="unknown \\['mass'\\]"):
        problem.params = {"plant": {**plant.params, "mass": 2.0}}

    assert sorted(problem.cost.params) == keys
    assert "mass" not in plant.params


def test_a_problem_takes_a_complete_part_tree():
    plant = Pendulum()
    problem = regulation_problem(plant)

    problem.params = {"plant": {**plant.params, "m": 2.0}}

    assert plant.params["m"] == 2.0


def test_a_diagram_refuses_a_partial_block_and_changes_nothing():
    loop = PID(Kp=10.0, Ki=1.0, Kd=2.0) @ Pendulum()
    before = copy.deepcopy(loop.params)

    with pytest.raises(ValueError, match="missing"):
        loop.params = {
            **loop.params,
            "controller": {**loop.params["controller"], "Kp": np.array([3.0])},
            "plant": {"m": 2.0},
        }

    assert loop.params["controller"]["Kp"] == before["controller"]["Kp"]
    assert sorted(loop.params["plant"]) == sorted(before["plant"])


def test_a_diagram_takes_its_realization():
    loop = PID(Kp=10.0, Ki=1.0, Kd=2.0) @ Pendulum()

    loop.params = loop.realize(3)


def test_a_sum_refuses_a_partial_term():
    total = QuadraticCost.from_system(Pendulum()) + TimeCost(np.zeros(2))
    term = next(iter(total.params))

    with pytest.raises(ValueError, match="missing"):
        total.params = {term: {"R": np.array([[10.0]])}}


def test_a_new_shape_through_a_composite_is_refused():
    plant = Pendulum()
    problem = regulation_problem(plant)

    with pytest.raises(ValueError, match="shape"):
        problem.params = {"cost": {**problem.cost.params, "R": np.eye(2)}}


# The program's form is decided once per bind, never per call


@pytest.mark.optional
@pytest.mark.jax
def test_the_program_form_is_decided_at_bind():
    pytest.importorskip("jax")
    from minilink.optimization import MathematicalProgram
    from minilink.optimization.evaluators.compiler import compile_program_evaluator

    program = MathematicalProgram(
        n_z=2,
        J=lambda z, p: (z[0] - p["a"]) ** 2 + (z[1] - p["b"]) ** 2,
        params={"a": np.array(1.0), "b": np.array(2.0)},
    )
    evaluator = compile_program_evaluator(program, "jax")
    evaluator.bind(None)
    assert evaluator.frozen
    evaluator.bind({"a": 3.0})
    assert not evaluator.frozen
    evaluator.bind(None)
    assert evaluator.frozen


# A set's numbers key the offline program, a lowered box included


@pytest.mark.optional
@pytest.mark.jax
def test_an_in_place_edit_of_a_lowered_box_builds_a_new_program():
    pytest.importorskip("jax")
    from minilink import TrajectoryOptimizationPlanner

    plant = Pendulum()
    problem = PlanningProblem(
        plant,
        x0=np.array([0.5, 0.0]),
        cost=QuadraticCost.from_system(plant, R=0.1 * np.eye(1)),
        horizon=1.0,
        U=plant.input_box,
    )
    planner = TrajectoryOptimizationPlanner(
        problem,
        n_steps=8,
        transcription="direct_collocation",
        backend="jax",
        options={"maxiter": 10},
    )
    planner.solve(evaluate=False)
    first = planner.last_program

    problem.U.box.params["upper"][:] = 0.5 * problem.U.box.params["upper"]
    planner.solve(evaluate=False)

    assert planner.last_program is not first


# A sum reads its own numbers when given none


def test_a_sum_given_none_reads_its_terms_own_numbers():
    total = QuadraticCost.from_system(Pendulum()) + 2.0 * TimeCost(np.zeros(2))
    x, u = np.array([0.3, -0.2]), np.array([0.4])

    assert total.g(x, u, 0.1) == total.g(x, u, 0.1, total.params)


# One params property: an assignment keeps the dict a composite holds


def test_a_held_tree_stays_current_after_an_assignment():
    plant = Pendulum()
    loop = PID(Kp=10.0, Ki=1.0, Kd=2.0) @ plant
    held = {"plant": plant.params}

    plant.params = {**plant.params, "m": 7.0}

    assert held["plant"]["m"] == 7.0
    assert loop.params["plant"] is plant.params


def test_assigning_an_object_its_own_tree_keeps_it():
    plant = Pendulum()
    numbers = dict(plant.params)

    plant.params = plant.params

    assert plant.params == numbers
