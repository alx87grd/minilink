"""The v0.3 core's budget: five timings against the bars of the core's definition of done.

    python benchmarks/v03_budget.py [OUT.json]

A report, not a gate (the numbers go in the run's report): each path runs
once to compile, then its best of ``REPEATS`` calls is taken (CPU, float64).

1. The one score on a continuous loop, 1000 starts × 200 RK4 steps: within 2× of 44 ms.
2. The sampled loop, 100 starts × 3000 base ticks: within 2× of 93 ms.
3. The native MPC's closed loop, 60 ticks × 15 iterations × a 20-step horizon: within 2× of
   134 ms a call.
4. The parametric program's solve (a native fixed-iteration solve, jitted with the parameters
   an argument) within 2× of the same solve with its parameters frozen (constants of the trace).
5. A parameter family (1000 draws of the pendulum's gravity) within 2× of the plain batch
   (1000 starts). A drawn mass (5b) is reported, not judged: each trial then factors its
   own mass matrix in the plant's ``solve(H, τ − C v − g)``, the plant's cost, not the
   evaluator's.
"""

from __future__ import annotations

import json
import sys
import time
import warnings

import numpy as np

from minilink import (
    PID,
    Gaussian,
    Pendulum,
    PlanningProblem,
    QuadraticCost,
    StateFeedback,
    Uniform,
)
from minilink.control import ModelPredictiveController
from minilink.core.backends import configure_jax
from minilink.planning import TrajectoryOptimizationPlanner

configure_jax(enable_x64=True)
warnings.simplefilter("ignore")

REPEATS = 5
FACTOR = 2.0


def best_time(call):
    """Seconds of the fastest of ``REPEATS`` calls, after one call that compiles."""
    import jax

    jax.block_until_ready(call())
    times = []
    for _ in range(REPEATS):
        start = time.perf_counter()
        jax.block_until_ready(call())
        times.append(time.perf_counter() - start)

    return min(times)


def pendulum_task(tf, x_start, params=None):
    """The pendulum regulated to the bottom over ``tf``, within its torque box."""
    plant = Pendulum()
    cost = QuadraticCost.from_system(plant, R=0.1 * np.eye(1))
    problem = PlanningProblem(
        plant,
        cost=cost,
        x_start=x_start,
        tf=tf,
        U=plant.input_box,
        params=params or {},
    )
    return plant, problem


def spread_of_starts():
    """The start law of the batched entries: about the pendulum's 0.5 rad."""
    return Gaussian(mean=np.array([0.5, 0.0]), std=np.array([0.1, 0.1]))


def pd_law():
    """A PD on the pendulum's state."""
    return StateFeedback(np.array([[2.0, 0.5]]))


def score_entry():
    """1000 starts × 200 RK4 steps of a continuous loop, one score."""
    _, problem = pendulum_task(2.0, spread_of_starts())
    pe = problem.compile(pd_law(), dt=0.01, backend="jax")
    params = pe.params

    seconds = best_time(lambda: pe.evaluate(params, n_trials=1000).objective)

    return {"seconds": seconds, "bar": 0.044}


def sampled_entry():
    """100 starts × 3000 base ticks of a digital PID on its clock."""
    _, problem = pendulum_task(30.0, spread_of_starts())
    pid = PID(Kp=2.0, Ki=0.1, Kd=0.5)
    pe = problem.compile(pid % 0.01, substeps=1, backend="jax")
    params = pe.params

    seconds = best_time(lambda: pe.evaluate(params, n_trials=100).objective)

    return {"seconds": seconds, "bar": 0.093}


def mpc_planner(problem, n_steps, iterations):
    """A native fixed-iteration planner: single shooting, Adam, ``iterations`` steps a solve."""
    planner = TrajectoryOptimizationPlanner(
        problem,
        n_steps=n_steps,
        transcription="single_shooting",
        backend="jax",
        method="adam",
        iterations=iterations,
        optimizer_options={"learning_rate": 0.05},
    )
    return planner


def mpc_entry():
    """60 ticks × 15 Adam iterations × a 20-step horizon, the closed loop one call."""
    plant, task = pendulum_task(3.0, np.array([0.5, 0.0]))
    _, plan = pendulum_task(1.0, np.array([0.5, 0.0]))
    mpc = ModelPredictiveController(mpc_planner(plan, 20, 15))
    pe = task.compile(mpc % 0.05, substeps=1, backend="jax")
    params = pe.params

    seconds = best_time(lambda: pe.evaluate(params).objective)

    return {"seconds": seconds, "bar": 0.134}


def program_entry():
    """One native solve jitted with its parameters an argument, against the same solve with them frozen."""
    import jax

    _, problem = pendulum_task(2.0, np.array([0.5, 0.0]))
    pv = mpc_planner(problem, 20, 15).compile()
    P = {"x_start": problem.x_start, "t_start": 0.0}
    z0 = pv.initial()

    # The parameters a traced argument (one trace for every value), or constants of the trace
    parametric_solve = jax.jit(lambda P, z0: pv.solve(P, z0=z0)[0])
    frozen_solve = jax.jit(lambda z0: pv.solve(P, z0=z0)[0])

    parametric = best_time(lambda: parametric_solve(P, z0))
    frozen = best_time(lambda: frozen_solve(z0))

    return {"seconds": parametric, "reference_seconds": frozen}


def family_entry(leaf="gravity", low=9.0, high=10.6):
    """1000 draws of one plant parameter against 1000 drawn starts, the same loop."""
    law = {"plant": {leaf: Uniform([low], [high])}}
    _, family = pendulum_task(2.0, np.array([0.5, 0.0]), params=law)
    _, batch = pendulum_task(2.0, spread_of_starts())
    pe_family = family.compile(pd_law(), dt=0.01, backend="jax")
    pe_batch = batch.compile(pd_law(), dt=0.01, backend="jax")
    p_family = pe_family.params
    p_batch = pe_batch.params

    seconds = best_time(lambda: pe_family.evaluate(p_family, n_trials=1000).objective)
    reference = best_time(lambda: pe_batch.evaluate(p_batch, n_trials=1000).objective)

    return {"seconds": seconds, "reference_seconds": reference}


def mass_family_entry():
    """The same family on the mass: each trial factors its own mass matrix, reported, not judged."""
    entry = family_entry("m", 0.8, 1.2)
    entry["judged"] = False

    return entry


ENTRIES = {
    "1 the one score, 1000 starts x 200 RK4 steps": score_entry,
    "2 the sampled loop, 100 starts x 3000 ticks": sampled_entry,
    "3 the native MPC, 60 ticks x 15 iterations x 20 steps": mpc_entry,
    "4 the parametric program against the frozen one": program_entry,
    "5 a parameter family (gravity) against the plain batch": family_entry,
    "5b the family on the mass (its own mass matrix per trial)": mass_family_entry,
}


def verdict(entry):
    """Within budget: ``FACTOR`` times the bar, or the reference path's time."""
    limit = FACTOR * entry.get("bar", entry.get("reference_seconds"))
    entry["limit_seconds"] = limit
    entry["within"] = bool(entry["seconds"] <= limit) or not entry.get("judged", True)

    return entry


def main(argv):
    report = {}
    for name, entry_of in ENTRIES.items():
        entry = verdict(entry_of())
        report[name] = entry
        reference = entry.get("bar", entry.get("reference_seconds"))
        mark = "within" if entry["within"] else "OVER"
        mark = mark if entry.get("judged", True) else "reported"
        print(
            f"{name}: {1e3 * entry['seconds']:.1f} ms "
            f"(reference {1e3 * reference:.1f} ms, limit {1e3 * entry['limit_seconds']:.1f} ms) {mark}"
        )
    if len(argv) > 1:
        with open(argv[1], "w") as handle:
            json.dump(report, handle, indent=2)


if __name__ == "__main__":
    main(sys.argv)
