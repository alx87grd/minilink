"""The seeded baseline of the parameters run: every path the one-parameters-model steps touch, as JSON.

Run: ``PYTHONPATH=. python benchmarks/params_baseline.py OUT.json`` (needs JAX). It records
numbers only, never timings, so two runs on one commit are byte-identical (``cmp``); the run of
a change to the parameters model (DESIGN §4) compares every commit against the capture taken at its base.
It uses only the surface every step keeps: ``params=None`` or a complete tree, built as a deep
copy of an object's own ``params`` and edited like a dict.

What it records:

- every public core object the contract test walks: its tree's key order, each leaf's dtype and
  shape, its one call at ``params=None`` and at its own tree;
- compiled rollouts (the pendulum, a PID loop, a nested diagram flattened, the racecar): the
  facade, the frozen form, the parametric form at an edited tree, a parameter family;
- one solve per planner family, and one solve at an edited complete tree;
- the problem evaluator on a stochastic problem, its program over two leaves packed and unpacked;
- a set-as-cost and a field-as-constraint problem solved;
- the compile counts of a held tool's repeats and sweeps.

Each section is caught on its own: an error is recorded as its type and message, so a later
commit that breaks one path shows as a difference instead of stopping the capture.
"""

import copy
import json
import sys
import warnings

import numpy as np

sys.path.insert(0, ".")
warnings.filterwarnings("ignore")

import jax  # noqa: E402

from minilink.core.backends import configure_jax  # noqa: E402

configure_jax(enable_x64=True)

import minilink  # noqa: E402
from minilink import (  # noqa: E402
    PID,
    DynamicProgrammingPlanner,
    LQRPlanner,
    Pendulum,
    PlanningProblem,
    QuadraticCost,
    RRTPlanner,
    RRTStarPlanner,
    TrajectoryOptimizationPlanner,
    Uniform,
)
from minilink.core.sets import BallSet  # noqa: E402

COMPILES = []
jax.monitoring.register_event_duration_secs_listener(
    lambda name, secs, **kw: (
        COMPILES.append(secs)
        if name == "/jax/core/compile/backend_compile_duration"
        else None
    )
)


###############################################################################
# Recording
###############################################################################


def number(value):
    """A JSON value of numbers: arrays as nested lists, scalars as floats, ``None`` kept."""
    if value is None or isinstance(value, (bool, str)):
        return value
    if isinstance(value, dict):
        return {str(k): number(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [number(v) for v in value]
    array = np.asarray(value)
    if array.dtype == object:
        return repr(type(value).__name__)
    if array.dtype.kind in "biu":
        return array.tolist()
    return np.asarray(array, dtype=float).tolist()


def structure(tree):
    """The tree's key order, and each leaf's dtype and shape."""
    if isinstance(tree, dict):
        return {str(k): structure(v) for k, v in tree.items()}
    if tree is None:
        return None
    if not isinstance(tree, (int, float, complex, np.ndarray, np.generic, list, tuple)):
        if not type(tree).__module__.startswith("jax"):
            return type(tree).__name__
    array = np.asarray(tree)
    return f"{array.dtype}{tuple(array.shape)}"


def section(name, out, build):
    """Record ``build()`` under ``name``, or the error it raised."""
    try:
        out[name] = build()
    except Exception as exc:  # noqa: BLE001 - every failure is a recorded difference
        out[name] = {"error": f"{type(exc).__name__}: {exc}"}


def compiles(call):
    """The XLA compilations ``call`` triggers."""
    COMPILES.clear()
    out = call()
    jax.block_until_ready(jax.tree_util.tree_leaves(out) if out is not None else 0)
    return len(COMPILES)


def edited(tree, path, value):
    """A deep copy of ``tree`` with the leaf at ``path`` (a tuple of keys) replaced."""
    copy_ = copy.deepcopy(tree)
    node = copy_
    for key in path[:-1]:
        node = node[key]
    node[path[-1]] = value
    return copy_


###############################################################################
# Sections
###############################################################################


def contract_objects():
    """Every object the contract test walks: structure, and its call at ``None`` and at its own tree."""
    sys.path.insert(0, "tests/unittest")
    import test_contract as contract

    out = {}
    for name, obj, call in contract.CASES:

        def one(name=name, obj=obj, call=call):
            built, call_ = contract.case_of(name, obj, call)
            params = built.params
            return {
                "structure": structure(params),
                "at_none": number(call_(built, None, np)),
                "at_own": number(call_(built, params, np)),
                "jax_at_none": number(call_(built, None, jax.numpy))
                if "Gaussian" not in name and name not in ("Uniform", "Particles")
                else None,
            }

        section(name, out, one)

    return out


def loop_of():
    return PID(Kp=10.0, Ki=1.0, Kd=2.0) @ Pendulum()


def nested_of():
    inner = PID(Kp=4.0, Ki=0.5, Kd=1.0) @ Pendulum()
    outer = minilink.DiagramSystem()
    outer.add_subsystem(inner, "inner")
    outer.refresh()
    return outer


def racecar_of():
    from minilink.dynamics.catalog.vehicles.racecar import UdeSRacecarDyn

    return UdeSRacecarDyn()


def rollouts():
    """The facade, the frozen and parametric compiled forms, a family, on four systems."""
    systems = {
        "pendulum": (Pendulum, ("m",)),
        "pid_loop": (loop_of, ("plant", "m")),
        "racecar": (racecar_of, None),
    }
    out = {}
    for label, (build, path) in systems.items():

        def one(build=build, path=path):
            sys_ = build()
            sys_.x0 = np.asarray(sys_.x0, dtype=float) + 0.1
            traj = sys_.compute_trajectory(tf=1.0, n_steps=101, verbose=False)
            ev = sys_.compile(backend="jax")
            u = np.zeros((200, ev.m))
            frozen = ev.rollout(sys_.x0, u, 0.0, 0.01)
            tree = copy.deepcopy(ev.params)
            if path is None:
                keys = [k for k, v in tree.items() if np.ndim(v) == 0]
                path = (keys[0],)
            node = tree
            for key in path:
                node = node[key]
            moved = edited(tree, path, 1.3 * float(node))
            live = ev.rollout(sys_.x0, u, 0.0, 0.01, moved)
            family = edited(tree, path, np.array([0.9, 1.0, 1.1]) * float(node))
            batch = ev.rollout_batch(
                np.tile(sys_.x0, (3, 1)),
                np.zeros((3, 50, ev.m)),
                dt=0.01,
                params=family,
            )
            return {
                "structure": structure(ev.params),
                "facade_x": number(traj.x[:, ::20]),
                "frozen": number(np.asarray(frozen)[::40]),
                "live": number(np.asarray(live)[::40]),
                "family": number(np.asarray(batch)[:, ::10]),
            }

        section(label, out, one)

    def nested():
        outer = nested_of()
        ev = outer.compile(backend="jax", flatten=True)
        x0 = np.asarray(outer.x0, dtype=float) + 0.1
        u = np.zeros((100, ev.m))
        tree = copy.deepcopy(ev.params)
        return {
            "structure": structure(ev.params),
            "frozen": number(np.asarray(ev.rollout(x0, u, 0.0, 0.01))[::25]),
            "live": number(np.asarray(ev.rollout(x0, u, 0.0, 0.01, tree))[::25]),
        }

    section("nested_flattened", out, nested)

    return out


def regulation():
    plant = Pendulum()
    plant.state.lower_bound = np.array([-np.pi, -6.0])
    plant.state.upper_bound = np.array([np.pi, 6.0])
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
    plant = Pendulum()
    plant.state.lower_bound = np.array([-np.pi, -6.0])
    plant.state.upper_bound = np.array([np.pi, 6.0])
    up = np.array([np.pi, 0.0])
    cost = QuadraticCost.from_system(plant, x_bar=up)
    return PlanningProblem(
        plant,
        x0=np.zeros(2),
        cost=cost,
        X=plant.state.box,
        U=plant.input_box,
        Xf=BallSet(up, 0.5),
        horizon=5.0,
    )


def fingerprint(solution):
    """The numbers a solution carries: its cost-to-go at a few states, its plan."""
    states = [np.array([a, 0.0]) for a in (-1.0, 0.5, 2.0)]
    record = {}
    if solution.cost_to_go is not None:
        record["cost_to_go"] = number([solution.cost_to_go(x) for x in states])
    if solution.trajectory is not None:
        record["x"] = number(np.asarray(solution.trajectory.x)[:, ::5])
        if solution.trajectory.u is not None:
            record["u"] = number(np.asarray(solution.trajectory.u)[:, ::5])
    return record


def planners():
    """One solve per family, and one at an edited complete tree (the plant's mass)."""
    grid = dict(x_grid=(9, 9), u_grid=(3,), dt=0.1)
    builds = {
        "value_iteration": lambda: DynamicProgrammingPlanner(
            regulation(), max_iterations=30, **grid
        ),
        "lqr": lambda: LQRPlanner(reach().replace(Xf=None)),
        "rrt": lambda: RRTPlanner(reach(), max_nodes=60, seed=0),
        "rrt_star": lambda: RRTStarPlanner(reach(), max_nodes=40, seed=0),
        "trajopt_collocation": lambda: TrajectoryOptimizationPlanner(
            reach().replace(Xf=None, horizon=2.0),
            n_steps=15,
            transcription="direct_collocation",
            backend="jax",
            options={"maxiter": 40},
        ),
        "trajopt_adam": lambda: TrajectoryOptimizationPlanner(
            reach().replace(Xf=None, X=None, horizon=1.0),
            n_steps=10,
            transcription="single_shooting",
            backend="jax",
            method="adam",
            n_iterations=20,
            learning_rate=0.05,
        ),
    }
    out = {}
    for label, build in builds.items():

        def one(build=build):
            planner = build()
            base = fingerprint(planner.solve(evaluate=False))
            tree = edited(planner.params, ("plant", "m"), 1.2)
            moved = fingerprint(planner.solve(params=tree, evaluate=False))
            return {
                "structure": structure(planner.params),
                "base": base,
                "moved": moved,
            }

        section(label, out, one)

    def mppi():
        from minilink.planning import MPPIPlanner

        plant = Pendulum()
        plant.inputs["u"].lower_bound = np.array([-4.0])
        plant.inputs["u"].upper_bound = np.array([4.0])
        cost = QuadraticCost.from_system(plant, x_bar=np.array([np.pi, 0.0]))
        problem = PlanningProblem(
            plant, cost=cost, horizon=1.0, x0=[0.0, 0.0], U=plant.input_box
        )
        planner = MPPIPlanner(
            problem, dt=0.05, n_rollouts=64, temperature=0.1, noise=2.0, n_iterations=2
        )
        pv = planner.compile()
        z, _ = pv.solve()
        tree = edited(pv.params, ("plant", "m"), 1.2)
        z_moved, _ = pv.solve(tree)
        return {
            "structure": structure(pv.params),
            "z": number(np.asarray(z)[::4]),
            "z_moved": number(np.asarray(z_moved)[::4]),
        }

    section("mppi", out, mppi)

    def mpc_tick():
        planner = TrajectoryOptimizationPlanner(
            reach().replace(Xf=None, horizon=1.0),
            n_steps=8,
            transcription="direct_collocation",
            backend="jax",
            options={"maxiter": 30},
        )
        planner.compile_parametric_program()
        first = planner.solve_trajectory_from(np.array([0.1, 0.0]))
        second = planner.solve_trajectory_from(np.array([0.2, 0.0]))
        return {
            "first": number(np.asarray(first.trajectory.x)),
            "second": number(np.asarray(second.trajectory.x)),
        }

    section("mpc_tick", out, mpc_tick)

    def compiled_face():
        planner = TrajectoryOptimizationPlanner(
            reach().replace(Xf=None, X=None, horizon=1.0),
            n_steps=10,
            transcription="single_shooting",
            backend="jax",
            method="adam",
            n_iterations=20,
            learning_rate=0.05,
        )
        pv = planner.compile()
        tick = copy.deepcopy(pv.params)
        tick["x0"] = np.array([0.3, 0.0])
        tick["t0"] = 0.0
        z, status = pv.solve(tick, z0=pv.initial())
        return {
            "structure": structure(pv.params),
            "z": number(np.asarray(z)),
            "cost": number(status["cost"]),
        }

    section("compiled_face", out, compiled_face)

    return out


def stochastic_problem():
    plant = Pendulum()
    cost = QuadraticCost.from_system(plant, R=0.1 * np.eye(1))
    laws = {"plant": {"m": Uniform([0.8], [1.2])}}
    common = dict(cost=cost, x0=np.array([0.5, 0.0]), horizon=2.0, U=plant.input_box)
    try:
        return PlanningProblem(plant, distributions=laws, **common), plant
    except TypeError:
        return PlanningProblem(plant, params=laws, **common), plant


def problem_evaluator():
    """The problem evaluator on a stochastic problem: one run, trials, a program over two leaves."""
    from minilink import lqr

    problem, plant = stochastic_problem()
    controller = lqr(plant, np.eye(2), 0.1 * np.eye(1))
    pe = problem.compile(controller, dt=0.02, backend="jax")
    params = pe.params
    traj = pe.trajectory()
    evaluation = pe.evaluate(key=0, n_trials=8)
    program = pe.program(["controller.K", "cost.R"], key=0, n_trials=4)
    z = pe.pack(params, ["controller.K", "cost.R"])
    back = pe.unpack(z, ["controller.K", "cost.R"])
    grad = jax.grad(lambda p: pe.evaluate(p, key=0, n_trials=4).objective)(params)
    return {
        "structure": structure(params),
        "trajectory": number(np.asarray(traj.x)[:, ::10]),
        "objective": number(evaluation.objective),
        "program_z0": number(program.metadata.get("z0")),
        "program_J": number(
            program.J(z, program.params) if program.params else program.J(z)
        ),
        "z": number(z),
        "unpacked": number(back["controller"]),
        "grad_K": number(grad["controller"]["K"]),
    }


def sets_and_fields_in_programs():
    """A set as a running cost, a field's sublevel set as the state constraint, both solved."""
    from minilink.core.fields import QuadraticField

    plant = Pendulum()
    base = QuadraticCost.from_system(plant, R=0.1 * np.eye(1))
    box = minilink.BoxSet([-0.4, -2.0], [0.4, 2.0])
    as_cost = PlanningProblem(
        plant, cost=base + box.as_cost(weight=5.0), x0=np.array([0.3, 0.0]), horizon=1.0
    )
    V = QuadraticField([0.0, 0.0], np.eye(2))
    as_constraint = PlanningProblem(
        plant, cost=base, x0=np.array([0.3, 0.0]), horizon=1.0, X=V.as_set(upper=1.0)
    )
    out = {}
    for label, problem in (
        ("set_as_cost", as_cost),
        ("field_as_constraint", as_constraint),
    ):

        def one(problem=problem):
            planner = TrajectoryOptimizationPlanner(
                problem,
                n_steps=10,
                transcription="direct_collocation",
                backend="jax",
                options={"maxiter": 30},
            )
            return {
                "structure": structure(problem.params),
                "solution": fingerprint(planner.solve(evaluate=False)),
            }

        section(label, out, one)

    return out


def compile_counts():
    """A held tool: its repeat, a sweep of values, the frozen form after the sweep."""
    plant = Pendulum()
    ev = plant.compile(backend="jax")
    x0 = np.array([0.3, 0.0])
    u = np.zeros((100, 1))
    counts = {"first_frozen": compiles(lambda: ev.rollout(x0, u, 0.0, 0.01))}
    counts["repeat_frozen"] = compiles(lambda: ev.rollout(x0, u, 0.0, 0.01))
    trees = [edited(ev.params, ("m",), m) for m in (0.8, 1.0, 1.2, 1.4)]
    counts["first_live"] = compiles(lambda: ev.rollout(x0, u, 0.0, 0.01, trees[0]))
    counts["sweep"] = sum(
        compiles(lambda t=t: ev.rollout(x0, u, 0.0, 0.01, t)) for t in trees[1:]
    )
    counts["alternating"] = sum(
        compiles(lambda t=t: ev.rollout(x0, u, 0.0, 0.01, t)) for t in trees * 2
    )
    counts["frozen_after"] = compiles(lambda: ev.rollout(x0, u, 0.0, 0.01))
    return counts


###############################################################################
# Main
###############################################################################


def main(path):
    record = {}
    for name, build in (
        ("contract", contract_objects),
        ("rollouts", rollouts),
        ("planners", planners),
        ("problem_evaluator", problem_evaluator),
        ("programs", sets_and_fields_in_programs),
        ("compile_counts", compile_counts),
    ):
        section(name, record, build)

    with open(path, "w", encoding="utf-8") as file:
        json.dump(record, file, indent=1, sort_keys=False)
        file.write("\n")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "params_baseline.json")
