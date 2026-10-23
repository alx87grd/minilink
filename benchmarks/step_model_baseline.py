"""Numbers of every sampled consumer of the step model, for the refactor's before/after check.

    python benchmarks/step_model_baseline.py OUT.json            # capture
    python benchmarks/step_model_baseline.py OUT.json --compare REF.json [--rtol 1e-6]

Each entry is one consumer on a pendulum or a cart-pole, NumPy and JAX where both exist,
with and without explicit parameters. No timing is recorded, so two captures of the same
code are byte-identical. ``--compare`` reports every entry that differs beyond ``rtol``.
"""

from __future__ import annotations

import json
import sys
import warnings

import numpy as np

from minilink import CartPole, Pendulum, PlanningProblem, QuadraticCost
from minilink.core.backends import configure_jax

configure_jax(enable_x64=True)
warnings.simplefilter("ignore")

DT = 0.05
N = 20
X0 = np.array([0.4, -0.2])
X0_CP = np.array([0.1, 0.5, 0.0, 0.0])
PARAMS = {"m": 1.3, "l": 0.8}


def u_sequence(m, n=N):
    """A deterministic input sequence, one row per step."""
    k = np.arange(n)[:, None]
    return 0.5 * np.sin(0.3 * k + np.arange(m)[None, :])


def as_list(value):
    """Arrays, scalars and nested containers as JSON lists of floats."""
    if isinstance(value, dict):
        return {str(k): as_list(v) for k, v in sorted(value.items())}
    if isinstance(value, (list, tuple)):
        return [as_list(v) for v in value]
    if value is None or isinstance(value, (bool, str)):
        return value
    return np.asarray(value, dtype=float).tolist()


# Entries


def evaluator_entries(out):
    """The compiled step model, both backends, the compiled and explicit parameters."""
    for name, plant, x0 in (
        ("pendulum", Pendulum(), X0),
        ("cartpole", CartPole(), X0_CP),
    ):
        m = plant.m
        us = u_sequence(m)
        knots = u_sequence(m, N + 1)
        p = dict(plant.params)
        p.update({k: v for k, v in PARAMS.items() if k in p})
        for backend in ("numpy", "jax"):
            ev = plant.compile(backend)
            key = f"ev.{name}.{backend}"
            u0 = us[0]
            out[f"{key}.rk4_step"] = ev.jit.step(x0, u0, 0.1, dt=DT)
            out[f"{key}.euler_step"] = ev.jit.step(x0, u0, 0.1, dt=DT, solver="euler")
            out[f"{key}.rk4_step_p"] = ev.jit.step(x0, u0, 0.1, p, dt=DT)
            out[f"{key}.euler_step_p"] = ev.jit.step(
                x0, u0, 0.1, p, dt=DT, solver="euler"
            )
            out[f"{key}.rk4_integrate_zoh"] = ev.jit.rollout(x0, us, 0.0, DT)
            out[f"{key}.rk4_integrate_zoh_p"] = ev.jit.rollout(x0, us, 0.0, DT, p)
            out[f"{key}.euler_integrate_zoh"] = ev.jit.rollout(
                x0, us, 0.0, DT, solver="euler"
            )
            out[f"{key}.rk4_integrate_linear"] = ev.jit.rollout(
                x0, knots, 0.0, DT, hold="foh"
            )
            out[f"{key}.rk4_integrate_linear_p"] = ev.jit.rollout(
                x0, knots, 0.0, DT, p, hold="foh"
            )
            out[f"{key}.rk4_integrate_ivp"] = ev.jit.rollout(
                x0, None, 0.0, DT, n_steps=N
            )
            out[f"{key}.euler_integrate_ivp"] = ev.jit.rollout(
                x0, None, 0.0, DT, solver="euler", n_steps=N
            )
            out[f"{key}.integrate_zoh_inner"] = ev.jit.step(
                x0, u0, 0.0, dt=DT, substeps=4
            )
            if backend == "jax":
                out[f"{key}.rollout_batch"] = ev.rollout_batch(
                    np.stack([x0, 0.5 * x0]), us, dt=DT
                )


def discretize_entries(out):
    """``discretize`` with RK4 and Euler, the default and explicit parameters."""
    from minilink.analysis import discretize

    plant = Pendulum()
    us = u_sequence(1)
    explicit = {**plant.params, **PARAMS}
    for integrator in ("rk4", "euler"):
        for label, params in (("nominal", None), ("params", explicit)):
            step = discretize(plant, DT, solver=integrator)
            x = X0.copy()
            xs = [x]
            for k in range(N):
                x = step.step(x, us[k], k, params)
                xs.append(x)
            out[f"discretize.{integrator}.{label}"] = np.array(xs)


def problem_of(plant, x0):
    cost = QuadraticCost.from_system(plant)
    return PlanningProblem(plant, cost=cost, x_start=x0, tf=2.0, X=plant.state.box)


def rl_environment_entries(out):
    """The RL environment's control period, both backends."""
    from minilink.learning.reinforcement_learning.environment import RolloutEnvironment

    plant = Pendulum()
    problem = problem_of(plant, X0)
    us = u_sequence(1)
    for backend in ("numpy", "jax"):
        env = RolloutEnvironment(problem, dt=DT, backend=backend)
        x, t = X0.copy(), 0.0
        rows = []
        for k in range(N):
            x, t, r, terminated, truncated = env.step(x, t, us[k], None)
            rows.append(np.concatenate([np.asarray(x), [t, r]]))
        out[f"rl_env.{backend}"] = np.array(rows)


def gym_entries(out):
    """The gym bridge's steps, RK4 and Euler."""
    from minilink.interfaces import Sys2Gym

    us = u_sequence(1)
    for integrator in ("rk4", "euler"):
        plant = Pendulum()
        plant.x0 = X0.copy()
        cost = QuadraticCost.from_system(plant)
        env = Sys2Gym(
            plant, cost, dt=DT, tf=10.0, solver=integrator, reset_mode="nominal"
        )
        env.reset(seed=0)
        rows = []
        for k in range(N):
            y, r, terminated, truncated, info = env.step(us[k])
            rows.append(np.concatenate([np.asarray(y), [r]]))
        out[f"gym.{integrator}"] = np.array(rows)


def dp_entries(out):
    """Value iteration and policy evaluation on a coarse pendulum grid."""
    from minilink import DynamicProgrammingPlanner

    plant = Pendulum()
    plant.state.lower_bound = np.array([-4.0, -6.0])
    plant.state.upper_bound = np.array([4.0, 6.0])
    plant.inputs["u"].lower_bound = np.array([-3.0])
    plant.inputs["u"].upper_bound = np.array([3.0])
    problem = PlanningProblem(plant, cost=QuadraticCost.from_system(plant))
    planner = DynamicProgrammingPlanner(
        problem, x_grid=(21, 21), u_grid=(5,), dt=DT, max_iterations=30, verbose=False
    )
    solution = planner.solve()
    grid_points = (
        np.stack(np.meshgrid(np.linspace(-3, 3, 7), np.linspace(-5, 5, 7)))
        .reshape(2, -1)
        .T
    )
    out["dp.cost_to_go"] = [solution.cost_to_go(x) for x in grid_points]
    out["dp.policy"] = [solution.policy.ctl(None, x) for x in grid_points]


def trajopt_entries(out):
    """The three transcriptions on a short pendulum problem."""
    from minilink import TrajectoryOptimizationPlanner

    for transcription in ("direct_collocation", "shooting", "multiple_shooting"):
        plant = Pendulum()
        plant.inputs["u"].lower_bound = np.array([-5.0])
        plant.inputs["u"].upper_bound = np.array([5.0])
        cost = QuadraticCost.from_system(plant, xbar=np.array([np.pi, 0.0]))
        problem = PlanningProblem(plant, cost=cost, x_start=np.zeros(2), tf=1.0)
        if transcription == "shooting":
            from minilink.planning.trajectory_optimization.shooting import (
                ShootingOptions,
                ShootingTranscription,
            )

            transcription = ShootingTranscription(ShootingOptions(n_steps=10))
        planner = TrajectoryOptimizationPlanner(
            problem, n_steps=10, transcription=transcription, verbose=False
        )
        transcription = getattr(transcription, "name", None) or "shooting"
        solution = planner.solve()
        traj = solution.trajectory
        out[f"trajopt.{transcription}.x"] = traj.x
        out[f"trajopt.{transcription}.u"] = traj.u


def compute_forced_entries(out):
    """``compute_forced`` on each fixed-step solver, a callable and a sampled input."""
    plant = Pendulum()
    plant.x0 = X0.copy()
    t = np.linspace(0.0, 1.0, 20)
    u_matrix = 0.5 * np.sin(2.0 * t)[None, :]
    for solver in ("rk4_fixedsteps", "euler_fixedsteps", "euler"):
        traj = plant.compute_forced(
            lambda tt: np.array([0.5 * np.sin(2.0 * tt)]),
            tf=1.0,
            n_steps=20,
            solver=solver,
            verbose=False,
        )
        out[f"forced.{solver}.callable"] = traj.x
        traj = plant.compute_forced(
            u_matrix, tf=1.0, n_steps=20, solver=solver, verbose=False
        )
        out[f"forced.{solver}.sampled"] = traj.x


def rrt_entries(out):
    """One seeded RRT search on the pendulum."""
    from minilink import BallSet, KinodynamicExtender, RRTOptions, RRTPlanner

    plant = Pendulum()
    x_goal = np.array([np.pi, 0.0])
    problem = PlanningProblem(
        plant,
        x_start=np.zeros(2),
        x_goal=x_goal,
        Xf=BallSet(x_goal, 0.5),
        X=plant.state.box,
    )
    torques = [np.array([tau]) for tau in (-5.0, 0.0, 5.0)]
    planner = RRTPlanner(
        problem,
        extender=KinodynamicExtender(controls=torques, horizon=0.3, n_substeps=6),
        options=RRTOptions(seed=0, goal_bias=0.05, max_nodes=200),
    )
    solution = planner.solve()
    out["rrt.x"] = solution.trajectory.x


def lyapunov_entries(out):
    """The region of attraction of an LQR loop, from its sampler."""
    from minilink import InvertedPendulum, lqr
    from minilink.analysis import region_of_attraction

    plant = InvertedPendulum()
    controller = lqr(plant, Q=np.eye(2), R=np.eye(1))
    roa = region_of_attraction(controller @ plant, samples=200)
    out["lyapunov.level"] = roa.level


ENTRIES = (
    evaluator_entries,
    discretize_entries,
    rl_environment_entries,
    gym_entries,
    dp_entries,
    trajopt_entries,
    compute_forced_entries,
    rrt_entries,
    lyapunov_entries,
)


def capture():
    out = {}
    for entry in ENTRIES:
        try:
            entry(out)
        except Exception as error:  # noqa: BLE001  (an entry that breaks is a result)
            out[f"{entry.__name__}.error"] = f"{type(error).__name__}: {error}"
    return {key: as_list(value) for key, value in out.items()}


def compare(current, reference, rtol):
    """Every key that is missing, new, an error, or differs beyond ``rtol``."""
    report = []
    for key in sorted(set(current) | set(reference)):
        if key not in current or key not in reference:
            report.append(
                f"{key}: only in {'reference' if key in reference else 'current'}"
            )
            continue
        a, b = current[key], reference[key]
        if isinstance(a, str) or isinstance(b, str):
            if a != b:
                report.append(f"{key}: {a!r} != {b!r}")
            continue
        a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
        if a.shape != b.shape:
            report.append(f"{key}: shape {a.shape} != {b.shape}")
            continue
        scale = np.maximum(np.abs(b), 1.0)
        error = float(np.max(np.abs(a - b) / scale)) if a.size else 0.0
        if error > rtol:
            report.append(f"{key}: relative error {error:.3g}")
    return report


if __name__ == "__main__":
    path = sys.argv[1]
    data = capture()
    with open(path, "w", encoding="utf-8") as file:
        json.dump(data, file, indent=1, sort_keys=True)
    if "--compare" in sys.argv:
        reference_path = sys.argv[sys.argv.index("--compare") + 1]
        rtol = (
            float(sys.argv[sys.argv.index("--rtol") + 1])
            if "--rtol" in sys.argv
            else 1e-6
        )
        with open(reference_path, encoding="utf-8") as file:
            reference = json.load(file)
        report = compare(data, reference, rtol)
        print(
            "\n".join(report)
            if report
            else f"all {len(data)} entries agree (rtol {rtol:g})"
        )
        sys.exit(1 if report else 0)
    print(f"{len(data)} entries; errors: {[k for k in data if k.endswith('.error')]}")
