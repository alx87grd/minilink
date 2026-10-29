"""Compile once, in the form the call needs: the first call, a repeat and the compile count of each held JAX tool.

    python benchmarks/compile_once.py [OUT.json]

A report, not a gate (CPU, float64). Part 1, for each pipeline's held tool (a simulator, a
compiled loop, an evaluation, a trajectory optimization, a native MPC, value iteration): the
first call's seconds, a repeat's and the XLA compilations the repeat costs (0 is the
contract; a convenience call compiles each time). Part 2, at the problem and the program layers: the parametric form (the
tree an argument, held on the device) against a frozen one (the tree closed over, constants
of the trace); a frozen form is worth building at a layer only if it is steadily more than
15 % faster.
"""

from __future__ import annotations

import json
import sys
import time
import warnings

import numpy as np

from minilink import (
    CartPole,
    DynamicProgrammingPlanner,
    Pendulum,
    PlanningProblem,
    QuadraticCost,
    Simulator,
    StateFeedback,
    lqr,
)
from minilink.control import ModelPredictiveController
from minilink.core.backends import configure_jax
from minilink.planning import TrajectoryOptimizationPlanner

jax = configure_jax(enable_x64=True)
warnings.simplefilter("ignore")

REPEATS = 5
GAIN = 0.15

# The XLA compilations of the call being timed
COMPILES = []
jax.monitoring.register_event_duration_secs_listener(
    lambda name, secs, **kw: (
        COMPILES.append(secs)
        if name == "/jax/core/compile/backend_compile_duration"
        else None
    )
)


def done(out):
    """``out`` computed: every JAX array of it ready."""
    leaves = jax.tree_util.tree_leaves(out)
    return jax.block_until_ready(
        [leaf for leaf in leaves if isinstance(leaf, jax.Array)]
    )


def timed(call):
    """``(first, repeat, compiles)``: the first call's seconds, the best repeat's and its compile count."""
    start = time.perf_counter()
    done(call())
    first = time.perf_counter() - start
    repeats = []
    COMPILES.clear()
    for _ in range(REPEATS):
        start = time.perf_counter()
        done(call())
        repeats.append(time.perf_counter() - start)

    return first, min(repeats), len(COMPILES) / REPEATS


def pipelines():
    """The held tool of each pipeline, by name."""
    cart = CartPole()
    cart.x0 = np.array([0.1, 0.2, 0.0, 0.0])
    pend = Pendulum()
    cost = QuadraticCost.from_system(pend, R=0.1 * np.eye(1))
    sim = dict(tf=5.0, n_steps=5001, solver="rk4_fixedsteps", backend="jax")
    loop = lqr(cart, np.eye(4), np.eye(1)) @ cart
    sampled = (lqr(cart, np.eye(4), np.eye(1)) % 0.01) @ cart
    plant_sim = Simulator(cart, **sim)
    scipy_sim = Simulator(cart, tf=2.0, solver="scipy", backend="jax")
    loop_sim = Simulator(loop, **sim)
    sampled_ev = sampled.compile(backend="jax")

    problem = PlanningProblem(
        pend, cost=cost, x_start=np.array([0.5, 0.0]), tf=5.0, U=pend.input_box
    )
    pe = problem.compile(StateFeedback(np.array([[2.0, 0.5]])), dt=0.01, backend="jax")

    swing = PlanningProblem(
        cart,
        cost=QuadraticCost.from_system(cart),
        x_start=np.array([0.0, np.pi - 0.5, 0.0, 0.0]),
        tf=2.0,
    )
    collocation = TrajectoryOptimizationPlanner(
        swing,
        n_steps=30,
        transcription="direct_collocation",
        backend="jax",
        method="scipy_slsqp",
        optimizer_options={"maxiter": 50},
    )
    horizon = PlanningProblem(
        pend, cost=cost, x_start=np.array([0.5, 0.0]), tf=1.0, U=pend.input_box
    )
    native = TrajectoryOptimizationPlanner(
        horizon,
        n_steps=20,
        transcription="single_shooting",
        backend="jax",
        method="adam",
        iterations=15,
        optimizer_options={"learning_rate": 0.05},
    )
    mpc_loop = (ModelPredictiveController(native) % 0.05) @ pend
    mpc_ev = mpc_loop.compile(backend="jax")
    dp = DynamicProgrammingPlanner(
        PlanningProblem(
            pend, cost=cost, x_start=np.array([0.5, 0.0]), U=pend.input_box
        ),
        x_grid=(41, 41),
        u_grid=(5,),
        dt=0.05,
        backend="jax",
        tol=1.0,
        max_iterations=200,
        verbose=False,
    )

    return {
        "sim  Simulator(plant) (rk4)": lambda: plant_sim.solve().x,
        "sim  Simulator(plant) (scipy)": lambda: scipy_sim.solve().x,
        "sim  Simulator(lqr @ plant)": lambda: loop_sim.solve().x,
        "sim  sampled loop, compiled": lambda: sampled_ev.rollout_dense(
            sampled.x0, None, None, n_steps=500
        ),
        "eval pe.evaluate()": lambda: pe.evaluate().objective,
        "eval pe.evaluate(params)": lambda: pe.evaluate(pe.params).objective,
        "eval pe.trajectory()": lambda: pe.trajectory().x,
        "opt  collocation planner.solve()": lambda: (
            collocation.solve(evaluate=False).trajectory.x
        ),
        "opt  native planner.solve()": lambda: (
            native.solve(evaluate=False).trajectory.x
        ),
        "mpc  (mpc % dt) @ plant, compiled": lambda: mpc_ev.rollout_dense(
            mpc_loop.x0, None, None, n_steps=20
        ),
        "dp   value iteration planner.solve()": lambda: dp.solve(evaluate=False),
    }


def forms():
    """The parametric form against a frozen one, at the problem and the program layers."""
    pend = Pendulum()
    cost = QuadraticCost.from_system(pend, R=0.1 * np.eye(1))
    problem = PlanningProblem(
        pend, cost=cost, x_start=np.array([0.5, 0.0]), tf=5.0, U=pend.input_box
    )
    law = StateFeedback(np.array([[2.0, 0.5]]))
    pe = problem.compile(law, dt=0.01, backend="jax")
    tree = pe.params

    horizon = PlanningProblem(
        pend, cost=cost, x_start=np.array([0.5, 0.0]), tf=1.0, U=pend.input_box
    )
    pv = TrajectoryOptimizationPlanner(
        horizon,
        n_steps=20,
        transcription="single_shooting",
        backend="jax",
        method="adam",
        iterations=200,
        optimizer_options={"learning_rate": 0.05},
    ).compile()
    start = {"x_start": horizon.x_start, "t_start": 0.0}
    z0 = pv.initial()
    parametric = jax.jit(lambda P, z: pv.solve(P, z0=z)[0])
    frozen = jax.jit(lambda z: pv.solve(start, z0=z)[0])

    return {
        "problem": (
            lambda: pe.evaluate().objective,
            jax.jit(lambda: pe.evaluate(tree).objective),
        ),
        "program": (lambda: parametric(start, z0), lambda: frozen(z0)),
    }


def main(path=None):
    out = {"pipelines": {}, "forms": {}}
    for name, call in pipelines().items():
        first, repeat, compiles = timed(call)
        out["pipelines"][name] = {
            "first_s": round(first, 4),
            "repeat_s": round(repeat, 5),
            "repeat_compiles": compiles,
        }
        print(
            f"{name:46s} first {1e3 * first:8.1f} ms  repeat {1e3 * repeat:8.2f} ms"
            f"  compiles {compiles:g}",
            flush=True,
        )

    for layer, (parametric, frozen) in forms().items():
        _, run_p, _ = timed(parametric)
        _, run_f, _ = timed(frozen)
        gain = 1.0 - run_f / run_p
        out["forms"][layer] = {
            "parametric_s": round(run_p, 5),
            "frozen_s": round(run_f, 5),
            "frozen_gain": round(gain, 3),
            "frozen_worth_it": bool(gain > GAIN),
        }
        print(
            f"{layer:8s} parametric {1e3 * run_p:8.2f} ms  frozen {1e3 * run_f:8.2f} ms"
            f"  gain {100 * gain:+.0f} %",
            flush=True,
        )

    if path:
        with open(path, "w") as handle:
            json.dump(out, handle, indent=1)

    return out


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else None)
