import time

import numpy as np
import jax
import jax.numpy as jnp

from minilink.core.framework import DynamicSystem
from minilink.simulation import Simulator


class Pendulum(DynamicSystem):
    def __init__(self):
        super().__init__(n=2, m=1, p=2)
        self.name = "Pendulum"

    def f(self, x, u, t=0, params=None):
        # Pure JAX implementation
        gravity = 9.81
        length = 1.0
        damping = 0.0
        q, dq = x[0], x[1]
        ddq = -(gravity / length) * jnp.sin(q) - damping * dq + u[0]
        return jnp.array([dq, ddq])


sys = Pendulum()
sys.x0[0] = 2.0

t0 = 0.0
tf = 1000.0
dt = 0.01
n_runs = 1

print("\n=== Simulator speed comparison ===")
print(f"system={sys.name}  t0={t0}  tf={tf}  dt={dt}  runs={n_runs}")
row_fmt = (
    "{solver_mode:<15} {backend:<8} {method:<12} "
    "{mean:>10.6f} {std:>10.6f} {nfev:>6d} {njev:>6d} {nt:>6d} {x0f:>12.6f}"
)
print(
    "\nsolver_mode      backend  scipy_method   mean [s]    std [s]   nfev   njev    n_t     x0_final"
)
print(
    "--------------- -------  ------------  ---------- ---------- ------ ------ ------ ------------"
)

solver_cases = {
    "euler": ["numpy", "jax"],
    "rk4_fixedsteps": ["numpy", "jax"],
    "scipy": ["numpy", "jax"],
    "scipy_stiff": ["numpy", "jax"],
    "scipy_max": ["numpy", "jax"],
    "scipy_ultra": ["numpy", "jax"],
}

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

        print(
            row_fmt.format(
                solver_mode=solver,
                backend=backend_label,
                method=scipy_method,
                mean=np.mean(t_runs),
                std=np.std(t_runs),
                nfev=int(np.mean(nfev_runs)),
                njev=int(np.mean(njev_runs)),
                nt=int(np.mean(nt_runs)),
                x0f=np.mean(x0f_runs),
            )
        )
