"""Flat trajectory-optimization backend/options benchmark.

Run from the repository root::

    python tests/benchmark/benchmark_trajopt_backends.py --case cartpole --starts both

JAX variants are skipped when JAX is not installed.
"""

from __future__ import annotations

import argparse
import time
from functools import lru_cache
from pathlib import Path

import numpy as np

from minilink.compile.jax_utils import configure_jax
from minilink.core.costs import JaxQuadraticCost, QuadraticCost
from minilink.core.trajectory import Trajectory
from minilink.dynamics.catalog.pendulum.cartpole import CartPole
from minilink.optimization.mathematical_program import MathematicalProgram
from minilink.optimization.optimizers.scipy_minimize import ScipyMinimizeOptimizer
from minilink.planning.initial_guess import default_initial_trajectory
from minilink.planning.problems import PlanningProblem
from minilink.planning.trajectory_optimization.direct_collocation import (
    DirectCollocationOptions,
    DirectCollocationTranscription,
)
from minilink.planning.trajectory_optimization.multiple_shooting import (
    MultipleShootingOptions,
    MultipleShootingTranscription,
)
from minilink.planning.trajectory_optimization.planner import (
    TrajectoryOptimizationOptions,
    TrajectoryOptimizationPlanner,
)
from minilink.planning.trajectory_optimization.shooting import (
    ShootingOptions,
    ShootingTranscription,
)

try:
    from minilink.dynamics.catalog.pendulum.cartpole import JaxCartPole
    from minilink.planning.trajectory_optimization.jax_direct_collocation import (
        JaxDirectCollocationOptions,
        JaxDirectCollocationTranscription,
    )
    from minilink.planning.trajectory_optimization.jax_multiple_shooting import (
        JaxMultipleShootingOptions,
        JaxMultipleShootingTranscription,
    )
    from minilink.planning.trajectory_optimization.jax_shooting import (
        JaxShootingOptions,
        JaxShootingTranscription,
    )

    HAS_JAX_CODE = True
except ImportError:
    JaxCartPole = None
    JaxDirectCollocationOptions = None
    JaxDirectCollocationTranscription = None
    JaxMultipleShootingOptions = None
    JaxMultipleShootingTranscription = None
    JaxShootingOptions = None
    JaxShootingTranscription = None
    HAS_JAX_CODE = False


parser = argparse.ArgumentParser()
parser.add_argument("--case", choices=["cartpole"], default="cartpole")
parser.add_argument("--tf", type=float, default=5.0)
parser.add_argument("--steps", type=int, default=20)
parser.add_argument("--maxiter", type=int, default=200)
parser.add_argument("--ftol", type=float, default=1e-2)
parser.add_argument("--runs", type=int, default=1)
parser.add_argument("--starts", choices=["cold", "warm", "both"], default="both")
parser.add_argument("--warm-guess-path", default="")
args = parser.parse_args()


def jax_available() -> bool:
    if not HAS_JAX_CODE:
        return False
    try:
        import jax  # noqa: F401
    except ImportError:
        return False
    return True


def make_cartpole_problem(sys, *, jax_cost: bool) -> PlanningProblem:
    sys.inputs["u"].lower_bound[0] = -10.0
    sys.inputs["u"].upper_bound[0] = 10.0

    x_start = np.array([-2.0, 1.0, 0.0, 0.0])
    x_goal = np.array([0.0, np.pi, 0.0, 0.0])
    cost_cls = JaxQuadraticCost if jax_cost else QuadraticCost
    cost = cost_cls.from_system(
        sys,
        Q=np.diag([1.0, 1.0, 0.0, 0.0]),
        R=np.diag([1.0]),
        S=np.zeros((sys.n, sys.n)),
        xbar=x_goal,
        ubar=np.zeros(sys.m),
    )
    return PlanningProblem(
        sys=sys,
        x_start=x_start,
        x_goal=x_goal,
        cost=cost,
    )


def make_optimizer(variant: dict[str, object]) -> ScipyMinimizeOptimizer:
    return ScipyMinimizeOptimizer(
        options={
            "disp": False,
            "maxiter": int(args.maxiter),
            "ftol": float(variant.get("ftol", args.ftol)),
        }
    )


def make_warm_optimizer() -> ScipyMinimizeOptimizer:
    return ScipyMinimizeOptimizer(
        options={
            "disp": False,
            "maxiter": max(int(args.maxiter), 200),
            "ftol": float(args.ftol),
        }
    )


@lru_cache(maxsize=1)
def warm_initial_trajectory() -> Trajectory:
    if args.warm_guess_path:
        path = Path(args.warm_guess_path)
        if path.exists():
            return Trajectory.load(path)

    if jax_available():
        configure_jax(enable_x64=True)
        problem = make_cartpole_problem(JaxCartPole(), jax_cost=True)
        transcription = JaxDirectCollocationTranscription(
            JaxDirectCollocationOptions(tf=args.tf, n_steps=args.steps)
        )
        compile_backend = "jax"
    else:
        problem = make_cartpole_problem(CartPole(), jax_cost=False)
        transcription = DirectCollocationTranscription(
            DirectCollocationOptions(tf=args.tf, n_steps=args.steps)
        )
        compile_backend = "numpy"

    planner = TrajectoryOptimizationPlanner(
        problem,
        transcription=transcription,
        optimizer=make_warm_optimizer(),
        options=TrajectoryOptimizationOptions(compile_backend=compile_backend),
    )
    traj = planner.compute_solution()
    if args.warm_guess_path:
        traj.save(args.warm_guess_path)
    return traj


def build_planner(variant: dict[str, object]):
    method = variant["method"]

    if method == "shooting":
        if variant["backend"] == "jax":
            if not jax_available():
                raise ModuleNotFoundError("JAX is not installed")

            configure_jax(enable_x64=variant["x64"] == "yes")
            return TrajectoryOptimizationPlanner(
                make_cartpole_problem(JaxCartPole(), jax_cost=True),
                transcription=JaxShootingTranscription(
                    JaxShootingOptions(
                        tf=args.tf,
                        n_steps=args.steps,
                        use_gradient=variant["gradient"] == "jax",
                    )
                ),
                optimizer=make_optimizer(variant),
                options=TrajectoryOptimizationOptions(compile_backend="jax"),
            )

        return TrajectoryOptimizationPlanner(
            make_cartpole_problem(CartPole(), jax_cost=False),
            transcription=ShootingTranscription(
                ShootingOptions(tf=args.tf, n_steps=args.steps)
            ),
            optimizer=make_optimizer(variant),
            options=TrajectoryOptimizationOptions(compile_backend="numpy"),
        )

    if method == "multiple-shooting":
        if variant["backend"] == "jax":
            if not jax_available():
                raise ModuleNotFoundError("JAX is not installed")

            configure_jax(enable_x64=variant["x64"] == "yes")
            return TrajectoryOptimizationPlanner(
                make_cartpole_problem(JaxCartPole(), jax_cost=True),
                transcription=JaxMultipleShootingTranscription(
                    JaxMultipleShootingOptions(
                        tf=args.tf,
                        n_steps=args.steps,
                        use_gradient=variant["gradient"] == "jax",
                    )
                ),
                optimizer=make_optimizer(variant),
                options=TrajectoryOptimizationOptions(compile_backend="jax"),
            )

        return TrajectoryOptimizationPlanner(
            make_cartpole_problem(CartPole(), jax_cost=False),
            transcription=MultipleShootingTranscription(
                MultipleShootingOptions(tf=args.tf, n_steps=args.steps)
            ),
            optimizer=make_optimizer(variant),
            options=TrajectoryOptimizationOptions(compile_backend="numpy"),
        )

    if variant["backend"] == "numpy":
        transcription = DirectCollocationTranscription(
            DirectCollocationOptions(tf=args.tf, n_steps=args.steps)
        )
        problem = make_cartpole_problem(CartPole(), jax_cost=False)
        compile_backend = "numpy"
    else:
        if not jax_available():
            raise ModuleNotFoundError("JAX is not installed")

        configure_jax(enable_x64=variant["x64"] == "yes")
        transcription = JaxDirectCollocationTranscription(
            JaxDirectCollocationOptions(
                tf=args.tf,
                n_steps=args.steps,
                use_gradient=variant["gradient"] == "jax",
            )
        )
        problem = make_cartpole_problem(JaxCartPole(), jax_cost=True)
        compile_backend = "jax"

    return TrajectoryOptimizationPlanner(
        problem,
        transcription=transcription,
        optimizer=make_optimizer(variant),
        options=TrajectoryOptimizationOptions(compile_backend=compile_backend),
    )


def constraint_metrics(
    program: MathematicalProgram,
    z: np.ndarray,
) -> tuple[float, float | None, float]:
    if program.equalities:
        max_eq = max(
            float(np.max(np.abs(equality.residual(z))))
            for equality in program.equalities
        )
    else:
        max_eq = 0.0

    min_ineq = None
    if program.inequalities:
        min_ineq = min(
            float(np.min(inequality.margin(z))) for inequality in program.inequalities
        )

    max_bound = 0.0
    if program.bounds is not None:
        if program.bounds.lower is not None:
            max_bound = max(
                max_bound,
                float(np.max(np.maximum(program.bounds.lower - z, 0.0))),
            )
        if program.bounds.upper is not None:
            max_bound = max(
                max_bound,
                float(np.max(np.maximum(z - program.bounds.upper, 0.0))),
            )

    return max_eq, min_ineq, max_bound


def run_once(variant: dict[str, object]) -> dict[str, object]:
    warm_guess = warm_initial_trajectory() if variant["start"] == "warm" else None

    planner = build_planner(variant)
    if warm_guess is None:
        guess = default_initial_trajectory(
            planner.problem,
            planner.transcription.initial_guess_time_grid(planner.problem),
        )
    else:
        guess = warm_guess

    t0 = time.perf_counter()
    transcribe_t0 = time.perf_counter()
    program = planner.transcription.transcribe(
        planner.problem,
        initial_guess=guess,
        compile_backend=planner.options.compile_backend,
    )
    transcribe_s = time.perf_counter() - transcribe_t0

    solve_t0 = time.perf_counter()
    result = planner.optimizer.solve(program)
    solve_s = time.perf_counter() - solve_t0

    max_eq, min_ineq, max_bound = constraint_metrics(program, result.z)
    return {
        **variant,
        "ok": bool(result.success),
        "total": time.perf_counter() - t0,
        "transcribe": transcribe_s,
        "solve": solve_s,
        "nit": result.stats.get("nit"),
        "nfev": result.stats.get("nfev"),
        "njev": result.stats.get("njev"),
        "cost": result.cost,
        "eq_inf": max_eq,
        "min_ineq": min_ineq,
        "bound": max_bound,
        "message": result.message,
    }


def mean_or_none(values) -> float | None:
    values = [value for value in values if value is not None]
    if not values:
        return None
    return float(np.mean(values))


def mean_int_or_none(values) -> int | None:
    value = mean_or_none(values)
    if value is None:
        return None
    return int(round(value))


def summarize(
    variant: dict[str, object], runs: list[dict[str, object]]
) -> dict[str, object]:
    last = runs[-1]
    return {
        **variant,
        "ok": all(bool(run["ok"]) for run in runs),
        "total": float(np.mean([run["total"] for run in runs])),
        "transcribe": float(np.mean([run["transcribe"] for run in runs])),
        "solve": float(np.mean([run["solve"] for run in runs])),
        "nit": mean_int_or_none(run["nit"] for run in runs),
        "nfev": mean_int_or_none(run["nfev"] for run in runs),
        "njev": mean_int_or_none(run["njev"] for run in runs),
        "cost": mean_or_none(run["cost"] for run in runs),
        "eq_inf": float(np.mean([run["eq_inf"] for run in runs])),
        "bound": float(np.mean([run["bound"] for run in runs])),
        "message": last["message"],
    }


variants = [
    {
        "name": "numpy-slsqp",
        "method": "collocation",
        "backend": "numpy",
        "gradient": "finite-diff",
        "x64": "n/a",
    },
    {
        "name": "numpy-shooting-slsqp",
        "method": "shooting",
        "backend": "numpy",
        "gradient": "finite-diff",
        "x64": "n/a",
    },
    {
        "name": "numpy-multiple-slsqp",
        "method": "multiple-shooting",
        "backend": "numpy",
        "gradient": "finite-diff",
        "x64": "n/a",
    },
]
if jax_available():
    variants.extend(
        [
            {
                "name": "jax-slsqp-grad-x64",
                "method": "collocation",
                "backend": "jax",
                "gradient": "jax",
                "x64": "yes",
            },
            {
                "name": "jax-shooting-grad-x64",
                "method": "shooting",
                "backend": "jax",
                "gradient": "jax",
                "x64": "yes",
            },
            {
                "name": "jax-multiple-grad-x64",
                "method": "multiple-shooting",
                "backend": "jax",
                "gradient": "jax",
                "x64": "yes",
            },
            {
                "name": "jax-slsqp-grad-f32-strict",
                "method": "collocation",
                "backend": "jax",
                "gradient": "jax",
                "x64": "no",
            },
            {
                "name": "jax-slsqp-grad-f32-relaxed",
                "method": "collocation",
                "backend": "jax",
                "gradient": "jax",
                "x64": "no",
                "ftol": max(float(args.ftol), 1e-5),
            },
            {
                "name": "jax-slsqp-fd-x64",
                "method": "collocation",
                "backend": "jax",
                "gradient": "finite-diff",
                "x64": "yes",
            },
        ]
    )

if args.starts == "cold":
    variants = [{**variant, "start": "cold"} for variant in variants]
elif args.starts == "warm":
    variants = [{**variant, "start": "warm"} for variant in variants]
else:
    variants = [
        {**variant, "start": start}
        for variant in variants
        for start in ("cold", "warm")
    ]

rows = []
for variant in variants:
    print(f"running {variant['name']} {variant['start']}...", flush=True)
    try:
        rows.append(summarize(variant, [run_once(variant) for _ in range(args.runs)]))
    except Exception as exc:
        print(f"  skipped: {type(exc).__name__}: {exc}")

print()
name_width = max([21, *(len(str(row["name"])) for row in rows)])
method_width = max([11, *(len(str(row["method"])) for row in rows)])
start_width = max([5, *(len(str(row["start"])) for row in rows)])
rule_width = name_width + method_width + start_width + 97
print("-" * rule_width)
print(
    "trajectory optimization benchmark "
    f"case={args.case} tf={args.tf:g} steps={args.steps} "
    f"maxiter={args.maxiter} starts={args.starts} runs={args.runs}"
)
print("-" * rule_width)
print(
    f"{'variant':<{name_width}} {'method':<{method_width}} "
    f"{'start':<{start_width}} {'backend':<7} {'grad':<11} {'x64':<4} "
    f"{'ok':>3} {'total':>8} {'trans':>8} {'solve':>8} "
    f"{'nit':>5} {'nfev':>6} {'njev':>6} {'cost':>11} "
    f"{'eq_inf':>10} {'bound':>9}"
)
print("-" * rule_width)
for row in rows:
    cost = "n/a" if row["cost"] is None else f"{row['cost']:11.4g}"
    nit = "n/a" if row["nit"] is None else str(row["nit"])
    nfev = "n/a" if row["nfev"] is None else str(row["nfev"])
    njev = "n/a" if row["njev"] is None else str(row["njev"])
    print(
        f"{row['name']:<{name_width}} {row['method']:<{method_width}} "
        f"{row['start']:<{start_width}} {row['backend']:<7} {row['gradient']:<11} "
        f"{row['x64']:<4} {str(row['ok']):>3} {row['total']:8.3f} "
        f"{row['transcribe']:8.3f} {row['solve']:8.3f} "
        f"{nit:>5} {nfev:>6} {njev:>6} {cost} "
        f"{row['eq_inf']:10.2e} {row['bound']:9.2e}"
    )
print("-" * rule_width)
if not jax_available():
    print("JAX variants skipped because JAX is not installed.")
for row in rows:
    if not row["ok"]:
        print(f"{row['name']} {row['start']} failed: {row['message']}")
