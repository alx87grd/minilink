"""Manual simulator benchmark across multiple system factories.

Compares runtime and precision relative to a configurable reference run.
"""

from __future__ import annotations

import time
from dataclasses import dataclass

import jax
import numpy as np

from minilink.simulation import Simulator
from tests.manual.benchmark_systems import (
    make_dense_network,
    make_pendulum,
    make_physics_many_spheres,
)

# Single precision policy switch for this benchmark script.
# False -> float32 default, True -> float64 default (if JAX supports x64).
USE_X64 = False
jax.config.update("jax_enable_x64", USE_X64)


@dataclass(frozen=True)
class BenchmarkConfig:
    t0: float
    tf: float
    dt: float
    n_runs: int
    reference_solver: str
    reference_backend: str
    solvers: list[str]
    backends: list[str]


@dataclass(frozen=True)
class SystemCase:
    name: str
    factory: callable
    config: BenchmarkConfig


def _run_case_once(sys, t0, tf, dt, solver, compile_backend):
    tic = time.perf_counter()
    sim = Simulator(
        sys,
        t0=t0,
        tf=tf,
        dt=dt,
        solver=solver,
        verbose=False,
        compile_backend=compile_backend,
    )
    traj = sim.solve()
    elapsed = time.perf_counter() - tic
    x_end = np.asarray(traj.x[:, -1], dtype=float)
    return elapsed, sim.last_debug, x_end


def _benchmark_system(case: SystemCase):
    cfg = case.config
    rows = []

    print(f"\n=== {case.name} ===")
    print(
        f"t0={cfg.t0} tf={cfg.tf} dt={cfg.dt} runs={cfg.n_runs} "
        f"ref=({cfg.reference_solver}, {cfg.reference_backend}) "
        f"precision={'float64' if USE_X64 else 'float32'}"
    )
    print(
        "solver_mode      backend          mean [s]    std [s]   nfev   njev    n_t"
        "     rel_err_l2 speed_vs_ref"
    )
    print(
        "--------------- --------------- ---------- ---------- ------ ------ ------"
        " -------------- ------------"
    )

    for solver in cfg.solvers:
        for compile_backend in cfg.backends:
            backend_label = (
                f"jax({jax.default_backend()})"
                if compile_backend == "jax"
                else compile_backend
            )
            t_runs = []
            nfev_runs = []
            njev_runs = []
            nt_runs = []
            x_end_runs = []
            for _ in range(cfg.n_runs):
                sys = case.factory()
                elapsed, dbg, x_end = _run_case_once(
                    sys,
                    t0=cfg.t0,
                    tf=cfg.tf,
                    dt=cfg.dt,
                    solver=solver,
                    compile_backend=compile_backend,
                )
                t_runs.append(elapsed)
                nfev_runs.append(dbg["nfev"])
                njev_runs.append(dbg["njev"])
                nt_runs.append(dbg["n_t"])
                x_end_runs.append(x_end)

            rows.append(
                {
                    "solver_mode": solver,
                    "compile_backend": compile_backend,
                    "backend": backend_label,
                    "mean": float(np.mean(t_runs)),
                    "std": float(np.std(t_runs)),
                    "nfev": int(np.mean(nfev_runs)),
                    "njev": int(np.mean(njev_runs)),
                    "nt": int(np.mean(nt_runs)),
                    "x_end": np.mean(np.stack(x_end_runs), axis=0),
                }
            )

    ref_row = None
    for row in rows:
        if (row["solver_mode"], row["compile_backend"]) == (
            cfg.reference_solver,
            cfg.reference_backend,
        ):
            ref_row = row
            break
    if ref_row is None:
        raise ValueError("Reference row not found in benchmark grid.")

    ref_norm = np.linalg.norm(ref_row["x_end"])
    ref_time = ref_row["mean"]

    for row in rows:
        diff_norm = np.linalg.norm(row["x_end"] - ref_row["x_end"])
        if ref_norm > 0:
            row["rel_err_l2"] = 100.0 * diff_norm / ref_norm
        else:
            row["rel_err_l2"] = 0.0 if diff_norm == 0 else np.inf
        row["speed_vs_ref"] = ref_time / row["mean"] if row["mean"] > 0 else np.inf
        print(
            f"{row['solver_mode']:<15} {row['backend']:<15} {row['mean']:>10.6f}"
            f" {row['std']:>10.6f} {row['nfev']:>6d} {row['njev']:>6d} {row['nt']:>6d}"
            f" {row['rel_err_l2']:>14.4f}% {row['speed_vs_ref']:>12.2f}x"
        )


######################################################################
pendulum_cfg = BenchmarkConfig(
    t0=0.0,
    tf=10.0,
    dt=0.01,
    n_runs=3,
    reference_solver="scipy_ultra",
    reference_backend="numpy",
    solvers=[
        "euler",
        "rk4_fixedsteps",
        "scipy",
        "scipy_stiff",
        "scipy_max",
        "scipy_ultra",
    ],
    backends=["numpy", "jax"],
)

dense_cfg = BenchmarkConfig(
    t0=0.0,
    tf=5.0,
    dt=0.1,
    n_runs=3,
    reference_solver="scipy_ultra",
    reference_backend="numpy",
    solvers=[
        "euler",
        "rk4_fixedsteps",
        "scipy",
        "scipy_stiff",
        "scipy_max",
        "scipy_ultra",
    ],
    backends=["numpy", "jax"],
)

physics_cfg = BenchmarkConfig(
    t0=0.0,
    tf=2.0,
    dt=0.01,
    n_runs=2,
    reference_solver="scipy_ultra",
    reference_backend="numpy",
    solvers=[
        "euler",
        "rk4_fixedsteps",
        "scipy",
        "scipy_stiff",
        "scipy_max",
        "scipy_ultra",
    ],
    backends=["numpy", "jax"],
)

cases = [
    SystemCase("Pendulum", make_pendulum, pendulum_cfg),
    SystemCase(
        "Dense Network (50 nodes, 5 conn/node)",
        lambda: make_dense_network(num_nodes=50, connections_per_node=5),
        dense_cfg,
    ),
    SystemCase(
        "PhysicsManySpheres (24 bodies)",
        lambda: make_physics_many_spheres(nx=6, ny=4),
        physics_cfg,
    ),
]

for case in cases:
    _benchmark_system(case)
