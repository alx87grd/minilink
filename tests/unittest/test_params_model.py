"""One parameters model (DESIGN §4, "Parameters"): its seven rules and the defects they closed, one test each."""

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
from tests.unittest.planning_helpers import edited


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


# A composite takes its whole tree: a partial one is refused and changes nothing


def test_a_problem_refuses_a_partial_tree_and_keeps_its_numbers():
    plant = Pendulum()
    problem = regulation_problem(plant)
    keys = sorted(problem.cost.params)

    with pytest.raises(ValueError, match="missing"):
        problem.params = {"cost": {"R": np.array([[10.0]])}}
    with pytest.raises(ValueError, match="unknown \\['mass'\\]"):
        problem.params = edited(problem.params, {"plant": {"mass": 2.0}})

    assert sorted(problem.cost.params) == keys
    assert "mass" not in plant.params


def test_a_problem_takes_its_complete_tree_edited():
    plant = Pendulum()
    problem = regulation_problem(plant)

    problem.params = edited(problem.params, {"plant": {"m": 2.0}})

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
        problem.params = edited(problem.params, {"cost": {"R": np.eye(2)}})


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


# planner.solve builds every call: it sees an in-place edit, a lowered box included


@pytest.mark.optional
@pytest.mark.jax
def test_a_planner_solve_builds_every_call_and_sees_an_edit_of_a_lowered_box():
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

    upper = 0.5 * problem.U.box.params["upper"]
    problem.U.box.params["upper"][:] = upper
    solution = planner.solve(evaluate=False)

    np.testing.assert_array_equal(planner.last_program.upper[-8:], np.repeat(upper, 8))
    assert np.all(solution.trajectory.u <= upper + 1e-8)


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


# Values hold their numbers: one home, edited in place


def test_a_set_moves_with_an_in_place_edit_and_its_attributes_are_views():
    from minilink import BoxSet

    X = BoxSet([-1.0, -1.0], [1.0, 1.0])
    X.params["upper"] = np.array([3.0, 3.0])

    np.testing.assert_array_equal(X.upper, [3.0, 3.0])
    assert X.contains(np.array([2.0, 0.0]))
    X.upper = np.array([1.0, 1.0])
    assert not X.contains(np.array([2.0, 0.0]))


def test_a_problem_write_lands_in_the_users_set():
    from minilink import BoxSet

    plant = Pendulum()
    X = BoxSet([-1.0, -1.0], [1.0, 1.0])
    problem = PlanningProblem(plant, x0=np.zeros(2), X=X, horizon=1.0)

    problem.params["X"]["upper"] = np.array([2.0, 2.0])
    problem.params = edited(problem.params, {"X": {"lower": np.array([-2.0, -2.0])}})

    assert problem.X is X
    np.testing.assert_array_equal(X.lower, [-2.0, -2.0])
    np.testing.assert_array_equal(X.upper, [2.0, 2.0])


def test_a_field_set_and_a_field_cost_hold_their_fields_numbers():
    from minilink.core.fields import QuadraticField

    V = QuadraticField(np.zeros(2), np.eye(2))
    band = V.as_set(lower=0.1, upper=2.0)
    cost = V.as_cost(weight=3.0)
    x = np.array([1.0, 0.0])

    band.params["field"]["center"] = np.array([1.0, 0.0])
    cost.params = edited(cost.params, {"weight": 2.0})

    np.testing.assert_array_equal(V.center, [1.0, 0.0])
    assert float(cost.g(x, np.zeros(1))) == 0.0 and cost.weight == 2.0
    np.testing.assert_allclose(band.margin(x), [-0.1, 2.0])


def test_a_gaussian_keeps_its_spread_when_its_mean_moves():
    from minilink import Gaussian

    law = Gaussian([0.0, 0.0], std=[0.1, 0.2])
    law.params["mean"] = np.array([1.0, 0.0])

    np.testing.assert_array_equal(law.mean, [1.0, 0.0])
    np.testing.assert_array_equal(law.std, [0.1, 0.2])
    assert law.sample(0).shape == (2,)


def test_a_composite_values_unknown_entry_is_refused_before_any_write():
    from minilink import BoxSet
    from minilink.core.sets import BallSet

    both = BoxSet([-1.0], [1.0]) & BallSet([0.0], 1.0)

    with pytest.raises(ValueError, match="unknown \\['2'\\]"):
        both.params = edited(both.params, {"2": {"radius": 1.0}})
    np.testing.assert_array_equal(both.sets[0].upper, [1.0])


# Composites hold their own numbers: a write to one lands


def test_a_problems_own_numbers_land_where_they_are_written():
    plant = Pendulum()
    problem = regulation_problem(plant)

    problem.params["discount_rate"] = 0.5
    problem.params["t0"] = 1.0

    assert problem.discount_rate == 0.5 and problem.t0 == 1.0
    problem.discount_rate = 0.25
    assert problem.params["discount_rate"] == 0.25


def test_a_scaled_terms_weight_lands_through_the_sum():
    base = QuadraticCost.from_system(Pendulum())
    obstacle = 10.0 * TimeCost(np.zeros(2))
    total = base + obstacle

    total.params["TimeCost"]["weight"] = 20.0

    assert obstacle.weight == 20.0


def test_a_problem_holds_numbers_and_its_laws_are_its_distributions():
    from minilink import Uniform

    plant = Pendulum()
    law = Uniform([0.8], [1.2])
    problem = PlanningProblem(
        plant,
        x0=np.zeros(2),
        horizon=1.0,
        distributions={"plant": {"m": law}},
    )

    assert problem.distributions["plant"]["m"] is law
    assert problem.params["plant"] is plant.params
    with pytest.raises(TypeError, match="distributions"):
        problem.params = {"plant": {**plant.params, "m": Uniform([1.0], [2.0])}}


def test_a_sampled_view_reads_and_writes_its_sources_numbers():
    plant = Pendulum()
    sampled = plant.discretize(0.1)

    sampled.params["m"] = 2.0

    assert sampled.params is plant.params and plant.params["m"] == 2.0


def test_a_diagram_refuses_a_block_added_twice():
    from minilink import DiagramSystem

    plant = Pendulum()
    diagram = DiagramSystem()
    diagram.add_subsystem(plant, "plant")

    with pytest.raises(ValueError, match="already in the diagram as 'plant'"):
        diagram.add_subsystem(plant, "plant2")


# Complete trees everywhere: a verb takes None or its tool's whole tree


def test_a_verb_refuses_a_partial_tree_and_takes_its_tools_tree_edited():
    from minilink import DynamicProgrammingPlanner

    plant = Pendulum()
    plant.state.lower_bound = np.array([-np.pi, -6.0])
    plant.state.upper_bound = np.array([np.pi, 6.0])
    problem = regulation_problem(plant).replace(X=plant.state.box, U=plant.input_box)
    planner = DynamicProgrammingPlanner(
        problem, max_iterations=5, x_grid=(5, 5), u_grid=(3,), dt=0.1
    )

    with pytest.raises(ValueError, match="missing"):
        planner.solve(params={"plant": {"m": 2.0}})
    solution = planner.solve(params=edited(planner.params, {"plant": {"m": 2.0}}))

    assert solution is planner.last_solution
    assert plant.params["m"] == 1.0  # the user's problem untouched


def test_an_optimizer_refuses_a_partial_tree():
    from minilink.optimization import MathematicalProgram, Optimizer

    program = MathematicalProgram(
        n_z=1,
        J=lambda z, p: (z[0] - p["c"]) ** 2,
        params={"c": np.array(1.0)},
    )
    optimizer = Optimizer(program, z0=[0.0], method="l-bfgs")

    with pytest.raises(ValueError, match="missing \\['method'\\]"):
        optimizer.solve(params={"c": np.array(3.0)})
    result = optimizer.solve(params=edited(optimizer.params, {"c": np.array(3.0)}))

    assert result.z[0] == pytest.approx(3.0, abs=1e-6)


# A held simulator sweeps a parameter: one compile, each value read


@pytest.mark.optional
@pytest.mark.jax
def test_a_held_simulator_sweeps_a_parameter_with_one_compile():
    jax = pytest.importorskip("jax")
    from minilink import Simulator

    compiles = []
    jax.monitoring.register_event_duration_secs_listener(
        lambda name, secs, **kw: (
            compiles.append(name)
            if name == "/jax/core/compile/backend_compile_duration"
            else None
        )
    )
    plant = Pendulum()
    plant.x0 = np.array([0.5, 0.0])
    sim = Simulator(plant, tf=1.0, n_samples=101, solver="rk4", backend="jax")

    finals = [
        sim.solve(params=edited(sim.params, {"m": m})).x[0, -1] for m in (0.5, 1.0)
    ]
    compiles.clear()
    more = [sim.solve(params=edited(sim.params, {"m": m})).x[0, -1] for m in (1.5, 2.0)]

    assert len(compiles) == 0
    assert len({*finals, *more}) == 4
    assert plant.params["m"] == 1.0


def test_a_solution_records_the_numbers_it_was_solved_on():
    from minilink import LQRPlanner

    plant = Pendulum()
    problem = PlanningProblem(
        plant,
        x0=np.array([0.1, 0.0]),
        cost=QuadraticCost.from_system(plant, R=0.1 * np.eye(1)),
    )
    solution = LQRPlanner(problem).solve()

    plant.params["m"] = 3.0

    assert solution.params["plant"]["m"] == 1.0
    assert solution.problem.params["plant"]["m"] == 3.0  # the live problem moved


# A held problem evaluator runs on the numbers taken at its compile


def test_a_held_problem_evaluator_snapshots_and_a_new_compile_reads_the_edit():
    from minilink import lqr

    plant = Pendulum()
    problem = regulation_problem(plant)
    law = lqr(plant, np.eye(2), 0.1 * np.eye(1))
    pe = problem.compile(law, dt=0.02, backend="numpy")
    before = pe.evaluate().objective

    plant.params["m"] = 3.0

    assert pe.evaluate().objective == before
    assert pe.params["plant"]["m"] == 1.0
    assert problem.compile(law, dt=0.02, backend="numpy").evaluate().objective != before
    heavy = edited(pe.params, {"plant": {"m": 3.0}})
    assert pe.evaluate(heavy).objective != before


# The RRT extender reads the plant's numbers at each solve


def test_the_kinodynamic_extender_reads_an_edit_of_the_plant():
    from minilink.planning.search.extenders import KinodynamicExtender

    plant = Pendulum()
    problem = PlanningProblem(plant, x0=np.zeros(2), U=plant.input_box)
    extender = KinodynamicExtender(horizon=0.2, n_steps=4)
    start = np.array([0.3, 0.0])

    light = extender.propose(start, start, problem, np.random.default_rng(0))
    plant.params["m"] = 3.0
    heavy = extender.propose(start, start, problem, np.random.default_rng(0))

    assert not np.allclose(light[1].states, heavy[1].states)


# A tool's params is a copy of its snapshot: editing it never changes the tool


@pytest.mark.parametrize(
    "backend", ["numpy", pytest.param("jax", marks=pytest.mark.jax)]
)
def test_a_tools_params_is_a_copy_and_passed_back_runs_there(backend):
    from minilink import Simulator
    from minilink.optimization import MathematicalProgram, Optimizer

    plant = Pendulum()
    x, u = np.array([0.5, 0.0]), np.zeros(1)
    ev = plant.compile(backend=backend)
    before = np.asarray(ev.f(x, u))

    p = ev.params
    p["m"] = 3.0

    np.testing.assert_array_equal(np.asarray(ev.f(x, u)), before)
    assert ev.params["m"] == 1.0
    assert not np.allclose(np.asarray(ev.f(x, u, 0.0, p)), before)

    plant.x0 = np.array([0.5, 0.0])  # tilted: the mass shows in the swing
    sim = Simulator(plant, tf=0.5, n_samples=51, solver="rk4", backend=backend)
    q = sim.params
    q["m"] = 3.0
    assert sim.params["m"] == 1.0
    assert not np.allclose(sim.solve(params=q).x, sim.solve().x)

    program = MathematicalProgram(
        n_z=1, J=lambda z, prm: (z[0] - prm["c"]) ** 2, params={"c": np.array(1.0)}
    )
    optimizer = Optimizer(program, z0=[0.0], method="l-bfgs")
    r = optimizer.params
    r["c"][...] = 5.0  # even an in-place edit of a leaf stays in the copy
    assert float(optimizer.params["c"]) == 1.0
    assert optimizer.solve(params=r).z[0] == pytest.approx(5.0, abs=1e-6)
