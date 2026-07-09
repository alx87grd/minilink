"""Pendulum regulation: continuous Pyro SMC vs hybrid sampled SMC.

Compares fixed-step RK4 closed loop with
:class:`~minilink.control.modelbased.SlidingModeController` against the same law
sampled at ``TS`` via :func:`~minilink.analysis.discretize.sample_static` and
:class:`~minilink.simulation.hybrid_simulator.HybridSimulator`.

**Known issues (continuous path):** same as
``examples/scripts/control/demo_sliding_mode_pendulum.py`` — auto **Euler** with
finer ``dt`` is the recommended continuous path; see
`DESIGN.md` (*Discontinuous closed loops — known issues*).

Run from the repo root::

    python examples/scripts/hybrid/demo_smc_pendulum_compare.py
"""

import matplotlib.pyplot as plt
import numpy as np

from minilink.analysis.discretize import sample_static
from minilink.control.modelbased import SlidingModeController
from minilink.core.composition import closed_loop_qdq
from minilink.core.hybrid_composition import hybrid_closed_loop
from minilink.dynamics.catalog.pendulum.pendulum import Pendulum

TF = 5.0
SIM_DT = 0.002
TS = 0.02
REF = np.array([0.0, 0.0])

plant = Pendulum(length=1.0, mass=1.0)
plant.x0 = np.array([np.pi + 0.25, 0.0])

smc_params = {"lam": 2.0, "gain": 8.0, "nab": 0.15}
ctl = SlidingModeController(plant, **smc_params)

diagram_ct = closed_loop_qdq(ctl, plant)
result_ct = diagram_ct.compute_forced(
    REF,
    input_port_id="r",
    t0=0.0,
    tf=TF,
    dt=SIM_DT,
    solver="rk4_fixedsteps",
)
result_ct = diagram_ct.reconstruct_internal_signals(result_ct)

# diagram_ct.plot_trajectory()

smc_step = sample_static(
    SlidingModeController(plant, **smc_params),
    dt=TS,
)
hybrid = hybrid_closed_loop(
    smc_step,
    plant,
    schedule=TS,
    computer_in="y",
    plant_out="y",
)
result_hy = hybrid.compute_forced(
    REF,
    input_port_id="r",
    t0=0.0,
    tf=TF,
    plant_dt_inner=SIM_DT,
)

# hybrid.plot_trajectory()

fig, axes = plt.subplots(2, 1, sharex=True, figsize=(8, 5))

t_ct = result_ct.t
q_ct = result_ct.signals["sys:q"][0, :]
u_ct = result_ct.signals["ctl:u"][0, :]
axes[0].plot(t_ct, q_ct, label="continuous q")
axes[1].plot(t_ct, u_ct, label="continuous u", alpha=0.85)

t_hy = result_hy.t
q_hy = result_hy.signals["y"][0, :]
u_hy = result_hy.signals["u"][0, :]
axes[0].step(t_hy, q_hy, where="post", label=f"hybrid q (TS={TS})", linewidth=1.5)
axes[1].step(t_hy, u_hy, where="post", label=f"hybrid u (TS={TS})", linewidth=1.5)

for ax in axes:
    ax.axvline(TS, color="0.7", linestyle=":", linewidth=0.8)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")

axes[0].set_ylabel("theta [rad]")
axes[1].set_ylabel("tau [Nm]")
axes[1].set_xlabel("time [s]")
axes[0].set_title("Pyro SMC: continuous vs hybrid sampled control")
fig.tight_layout()
plt.show()
