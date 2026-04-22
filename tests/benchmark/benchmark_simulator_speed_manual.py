import time

import numpy as np
import jax

from minilink.blocks.testing.basic import make_pendulum
from minilink.blocks.testing.network import build_deep_network
from minilink.simulation import Simulator

sys = make_pendulum(initial_angle=2.0, damping=0.0)

# sys = build_deep_network(depth=50)

t0 = 0.0
tf = 10.0
dt = 0.01
n_runs = 1
GROUND_TRUTH = ("scipy_ultra", "jax")

print("\n=== Simulator speed comparison ===")
print(f"system={sys.name}  t0={t0}  tf={tf}  dt={dt}  runs={n_runs}")
row_fmt_live = (
    "{solver_mode:<15} {backend:<8} {method:<12} "
    "{mean:>10.6f} {std:>10.6f} {nfev:>6d} {njev:>6d} {nt:>6d} {x0f:>12.6f}"
)
row_fmt = (
    "{solver_mode:<15} {backend:<8} {method:<12} "
    "{mean:>10.6f} {std:>10.6f} {nfev:>6d} {njev:>6d} {nt:>6d} {x0f:>12.6f} {err:>11.2f}% {speed:>8.2f}x"
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
}

rows = []

for solver in [
    "euler",
    "scipy",
    "scipy_stiff",
    "scipy_max",
    "rk4_fixedsteps",
    "scipy_ultra",
]:
    cases = solver_cases[solver]

    for compile_backend in cases:
        backend_label = compile_backend
        if compile_backend == "jax":
            backend_label = f"jax({jax.default_backend()})"

        t_runs = []
        nfev_runs = []
        njev_runs = []
        nt_runs = []
        x0f_runs = []
        scipy_methods = []
        for _ in range(n_runs):
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
            t_runs.append(time.perf_counter() - tic)
            nfev_runs.append(sim.last_debug["nfev"])
            njev_runs.append(sim.last_debug["njev"])
            nt_runs.append(sim.last_debug["n_t"])
            x0f_runs.append(float(traj.x[0, -1]))
            scipy_methods.append(sim.last_debug.get("method", "NA"))

        scipy_method = scipy_methods[0] if solver.startswith("scipy") else "NA"

        rows.append(
            {
                "solver_mode": solver,
                "compile_backend": compile_backend,
                "backend": backend_label,
                "method": scipy_method,
                "mean": np.mean(t_runs),
                "std": np.std(t_runs),
                "nfev": int(np.mean(nfev_runs)),
                "njev": int(np.mean(njev_runs)),
                "nt": int(np.mean(nt_runs)),
                "x0f": np.mean(x0f_runs),
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

ref_x0f = None
ref_mean = None
for row in rows:
    if (row["solver_mode"], row["compile_backend"]) == GROUND_TRUTH:
        ref_x0f = row["x0f"]
        ref_mean = row["mean"]
        break

if ref_x0f is None:
    raise ValueError(f"Ground truth row not found: {GROUND_TRUTH}")

print("\n=== Final precision table ===")
print(
    "solver_mode      backend  scipy_method   mean [s]    std [s]   nfev   njev    n_t     x0_final   err_vs_ref speed_vs_ref"
)
print(
    "--------------- -------  ------------  ---------- ---------- ------ ------ ------ ------------ ----------- -----------"
)

for row in rows:
    if abs(ref_x0f) > 0:
        row["err"] = 100.0 * abs((row["x0f"] - ref_x0f) / ref_x0f)
    else:
        row["err"] = 0.0 if row["x0f"] == ref_x0f else np.inf
    row["speed"] = ref_mean / row["mean"] if row["mean"] > 0 else np.inf
    print(
        row_fmt.format(
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
            speed=row["speed"],
        )
    )
