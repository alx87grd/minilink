"""Flat trajectory-optimization backend/options benchmark.

Run from the repository root::

    python tests/benchmark/benchmark_trajopt_backends.py --case cartpole

JAX variants are skipped when JAX is not installed.
"""

from __future__ import annotations

import argparse
import time

import numpy as np

from minilink.core.costs import JaxQuadraticCost, QuadraticCost
from minilink.dynamics.catalog.pendulum.cartpole import CartPole
from minilink.optimization.mathematical_program import MathematicalProgram
from minilink.optimization.optimizers.scipy_minimize import ScipyMinimizeOptimizer
from minilink.planning.problems import PlanningProblem
from minilink.planning.trajectory_optimization.direct_collocation import (
    DirectCollocationPlanner,
)

try:
    from minilink.dynamics.catalog.pendulum.cartpole import JaxCartPole
    from minilink.planning.trajectory_optimization.jax_direct_collocation import (
        JaxDirectCollocationPlanner,
    )

    HAS_JAX_CODE = True
except ImportError:
    JaxCartPole = None
    JaxDirectCollocationPlanner = None
    HAS_JAX_CODE = False


parser = argparse.ArgumentParser()
parser.add_argument("--case", choices=["cartpole"], default="cartpole")
parser.add_argument("--tf", type=float, default=5.0)
parser.add_argument("--steps", type=int, default=20)
parser.add_argument("--maxiter", type=int, default=200)
parser.add_argument("--ftol", type=float, default=1e-6)
parser.add_argument("--runs", type=int, default=1)
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


def make_optimizer() -> ScipyMinimizeOptimizer:
    return ScipyMinimizeOptimizer(
        options={
            "disp": False,
            "maxiter": int(args.maxiter),
            "ftol": float(args.ftol),
        }
    )


def build_planner(variant: dict[str, str]):
    if variant["backend"] == "numpy":
        return DirectCollocationPlanner(
            make_cartpole_problem(CartPole(), jax_cost=False),
            tf=args.tf,
            n_steps=args.steps,
            optimizer=make_optimizer(),
            compile_backend="numpy",
        )

    if not jax_available():
        raise ModuleNotFoundError("JAX is not installed")

    return JaxDirectCollocationPlanner(
        make_cartpole_problem(JaxCartPole(), jax_cost=True),
        tf=args.tf,
        n_steps=args.steps,
        optimizer=make_optimizer(),
        compile_backend="jax",
        use_gradient=variant["gradient"] == "jax",
        enable_x64=variant["x64"] == "yes",
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


def run_once(variant: dict[str, str]) -> dict[str, object]:
    t0 = time.perf_counter()
    planner = build_planner(variant)

    transcribe_t0 = time.perf_counter()
    program = planner.transcription.transcribe(planner.problem)
    transcribe_s = time.perf_counter() - transcribe_t0

    solve_t0 = time.perf_counter()
    result = planner.options.optimizer.solve(program)
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


def summarize(variant: dict[str, str], runs: list[dict[str, object]]) -> dict[str, object]:
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
        "backend": "numpy",
        "gradient": "finite-diff",
        "x64": "n/a",
    }
]
if jax_available():
    variants.extend(
        [
            {
                "name": "jax-slsqp-grad-x64",
                "backend": "jax",
                "gradient": "jax",
                "x64": "yes",
            },
            {
                "name": "jax-slsqp-grad-f32",
                "backend": "jax",
                "gradient": "jax",
                "x64": "no",
            },
            {
                "name": "jax-slsqp-fd-x64",
                "backend": "jax",
                "gradient": "finite-diff",
                "x64": "yes",
            },
        ]
    )

rows = []
for variant in variants:
    print(f"running {variant['name']}...", flush=True)
    try:
        rows.append(summarize(variant, [run_once(variant) for _ in range(args.runs)]))
    except Exception as exc:
        print(f"  skipped: {type(exc).__name__}: {exc}")

print()
print("-" * 116)
print(
    "trajectory optimization benchmark "
    f"case={args.case} tf={args.tf:g} steps={args.steps} "
    f"maxiter={args.maxiter} runs={args.runs}"
)
print("-" * 116)
print(
    f"{'variant':<21} {'backend':<7} {'grad':<11} {'x64':<4} "
    f"{'ok':>3} {'total':>8} {'trans':>8} {'solve':>8} "
    f"{'nit':>5} {'nfev':>6} {'njev':>6} {'cost':>11} "
    f"{'eq_inf':>10} {'bound':>9}"
)
print("-" * 116)
for row in rows:
    cost = "n/a" if row["cost"] is None else f"{row['cost']:11.4g}"
    nit = "n/a" if row["nit"] is None else str(row["nit"])
    nfev = "n/a" if row["nfev"] is None else str(row["nfev"])
    njev = "n/a" if row["njev"] is None else str(row["njev"])
    print(
        f"{row['name']:<21} {row['backend']:<7} {row['gradient']:<11} "
        f"{row['x64']:<4} {str(row['ok']):>3} {row['total']:8.3f} "
        f"{row['transcribe']:8.3f} {row['solve']:8.3f} "
        f"{nit:>5} {nfev:>6} {njev:>6} {cost} "
        f"{row['eq_inf']:10.2e} {row['bound']:9.2e}"
    )
print("-" * 116)
if not jax_available():
    print("JAX variants skipped because JAX is not installed.")
