import sys
import time

import jax
import numpy as np

from minilink.benchmark import relative_l2_error
from minilink.blocks.testing.basic import make_pendulum
from minilink.blocks.testing.network import build_deep_network
from minilink.blocks.testing.engine import make_physics_many_spheres
from minilink.simulation import Simulator


# system = make_pendulum(initial_angle=2.0, damping=0.0)
system = make_physics_many_spheres(nx=12, ny=12)


# system = build_deep_network(depth=50)

t0 = 0.0
tf = 2.0
dt = 0.005
n_runs = 1

system.compute_trajectory(tf=tf, dt=dt, solver="scipy", show=False)
system.animate(renderer="meshcat")

# Truth for err% and speed_vs_ref: same as `benchmark_simulator_speed_matrix.py` (scipy_ultra + numpy).
GROUND_TRUTH = ("scipy_ultra", "numpy")
ACCU_PCT = 1.0  # final table: green if err < this (%), else red; highlight max speed_vs_ref in green


########################################################
# Helper functions
########################################################


def _stdout_color() -> bool:
    return bool(sys.stdout.isatty())


def _ansi(s: str, *codes: int) -> str:
    if not _stdout_color() or not codes:
        return s
    return f"\033[{';'.join(str(c) for c in codes)}m{s}\033[0m"


def _fastest_accurate_indices(
    rows: list[dict], err_key: str, speed_key: str, thr: float
) -> set[int]:
    acc = [i for i, r in enumerate(rows) if r[err_key] < thr]
    if not acc:
        return set()
    best = max(rows[i][speed_key] for i in acc)
    return {i for i in acc if rows[i][speed_key] >= best - 1e-15}


print("\n=== Simulator speed comparison ===")
print(f"system={system.name}  t0={t0}  tf={tf}  dt={dt}  runs={n_runs}")
_tty = _stdout_color()
print(
    f"ref={GROUND_TRUTH[0]}+{GROUND_TRUTH[1]}  err=relative L2 vs ref final x  "
    f"green if err<{ACCU_PCT:g}% else red; bold bright green = max speed_vs_ref among green"
    + ("" if _tty else " (no ANSI — not a TTY)")
)
row_fmt_live = (
    "{solver_mode:<15} {backend:<8} {method:<12} "
    "{mean:>10.6f} {std:>10.6f} {nfev:>6d} {njev:>6d} {nt:>6d} {x0f:>12.6f}"
)
row_fmt = (
    "{solver_mode:<15} {backend:<8} {method:<12} "
    "{mean:>10.6f} {std:>10.6f} {nfev:>6d} {njev:>6d} {nt:>6d} {x0f:>12.6f} "
    "{err:>11.2f}% {ok:>3} {speed:>8.2f}x"
)
print(
    "\nsolver_mode      backend  scipy_method   mean [s]    std [s]   nfev   njev    n_t     x0_final"
)
print(
    "--------------- -------  ------------  ---------- ---------- ------ ------ ------ ------------"
)
# print(f"reference={GROUND_TRUTH[0]} + {GROUND_TRUTH[1]}")

solver_cases = {
    "euler": ["numpy", "jax"],
    "rk4_fixedsteps": ["numpy", "jax"],
    "scipy": ["numpy", "jax"],
    "scipy_stiff": ["numpy", "jax"],
    "scipy_max": ["numpy", "jax"],
    "scipy_ultra": ["numpy", "jax"],
    "scipy_lsoda": ["numpy", "jax"],
}

rows: list[dict] = []

for solver in [
    "euler",
    "scipy",
    "scipy_stiff",
    "scipy_max",
    "rk4_fixedsteps",
    "scipy_ultra",
    "scipy_lsoda",
]:
    cases = solver_cases[solver]

    for compile_backend in cases:
        backend_label = compile_backend
        if compile_backend == "jax":
            backend_label = f"jax({jax.default_backend()})"

        t_runs: list[float] = []
        nfev_runs: list[int] = []
        njev_runs: list[int] = []
        nt_runs: list[int] = []
        x_end_runs: list[np.ndarray] = []
        scipy_methods: list[str] = []
        for _ in range(n_runs):
            tic = time.perf_counter()
            sim = Simulator(
                system,
                t0=t0,
                tf=tf,
                dt=dt,
                solver=solver,
                verbose=False,
                compile_backend=compile_backend,
            )
            traj = sim.solve()
            t_runs.append(time.perf_counter() - tic)
            nfev_runs.append(sim.last_debug["nfev"])
            njev_runs.append(sim.last_debug["njev"])
            nt_runs.append(sim.last_debug["n_t"])
            x_end_runs.append(np.asarray(traj.x[:, -1], dtype=float))
            scipy_methods.append(sim.last_debug.get("method", "NA"))

        x_mean = np.mean(np.stack(x_end_runs, axis=0), axis=0)
        scipy_method = scipy_methods[0] if solver.startswith("scipy") else "NA"

        rows.append(
            {
                "solver_mode": solver,
                "compile_backend": compile_backend,
                "backend": backend_label,
                "method": scipy_method,
                "mean": float(np.mean(t_runs)),
                "std": float(np.std(t_runs)),
                "nfev": int(np.mean(nfev_runs)),
                "njev": int(np.mean(njev_runs)),
                "nt": int(np.mean(nt_runs)),
                "x0f": float(x_mean[0]),
                "x_mean": x_mean,
            }
        )
        print(
            row_fmt_live.format(
                solver_mode=rows[-1]["solver_mode"],
                backend=rows[-1]["backend"],
                method=rows[-1]["method"],
                mean=rows[-1]["mean"],
                std=rows[-1]["std"],
                nfev=rows[-1]["nfev"],
                njev=rows[-1]["njev"],
                nt=rows[-1]["nt"],
                x0f=rows[-1]["x0f"],
            )
        )

ref_x = None
ref_mean = None
for row in rows:
    if (row["solver_mode"], row["compile_backend"]) == GROUND_TRUTH:
        ref_x = row["x_mean"]
        ref_mean = row["mean"]
        break

if ref_x is None or ref_mean is None:
    raise ValueError(f"Ground truth row not found: {GROUND_TRUTH}")

for row in rows:
    row["err"] = float(relative_l2_error(row["x_mean"], ref_x))
    row["speed"] = ref_mean / row["mean"] if row["mean"] > 0 else float("inf")
    row["ok"] = "Y" if row["err"] < ACCU_PCT else "N"

fast_idx = _fastest_accurate_indices(rows, "err", "speed", ACCU_PCT)

print("\n=== Final precision table ===")
print(
    "solver_mode      backend  scipy_method   mean [s]    std [s]   nfev   njev    n_t     "
    "x0_final   err_vs_ref  ok speed_vs_ref"
)
print(
    "--------------- -------  ------------  ---------- ---------- ------ ------ ------ "
    "------------ ----------- --- -----------"
)

for i, row in enumerate(rows):
    line = row_fmt.format(
        solver_mode=row["solver_mode"],
        backend=row["backend"],
        method=row["method"],
        mean=row["mean"],
        std=row["std"],
        nfev=row["nfev"],
        njev=row["njev"],
        nt=row["nt"],
        x0f=row["x0f"],
        err=row["err"],
        ok=row["ok"],
        speed=row["speed"],
    )
    if i in fast_idx:
        line = _ansi(line, 1, 92)
    elif row["err"] < ACCU_PCT:
        line = _ansi(line, 32)
    else:
        line = _ansi(line, 31)
    print(line)
