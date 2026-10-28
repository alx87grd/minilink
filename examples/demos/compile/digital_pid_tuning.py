"""Tune a digital PID by gradient: the law on its clock, the loop compiled once, the gains optimized."""

import jax
import numpy as np

from minilink import PID, Optimizer, Pendulum, PlanningProblem, QuadraticCost

TF = 5.0
DT = 0.01  # the computer's period, s
R_REF = 1.0  # setpoint, rad
RHO = 0.01  # effort weight

# The task: J = (1/T) ∫₀ᵀ ((θ − r)² + ρ (u − u_ss)²) dt, u_ss = m g l sin r
plant = Pendulum()
U_SS = plant.params["m"] * plant.params["gravity"] * plant.params["l"] * np.sin(R_REF)
cost = QuadraticCost.from_system(
    plant,
    Q=np.diag([1.0 / TF, 0.0]),
    R=RHO / TF * np.eye(1),
    xbar=np.array([R_REF, 0.0]),
    ubar=np.array([U_SS]),
)
problem = PlanningProblem(plant, cost=cost, tf=TF)

# The digital PID: its difference equation on a 100 Hz clock, closed on the plant
pid = PID(Kp=1.0, Ki=0.0, Kd=1.0)
pe = problem.compile(pid % DT)
start = pe.params
start["r"] = np.array([R_REF])
print(pe.evaluate(start))

# Every gain's gradient through the sampled loop, by reverse mode
grad = jax.grad(lambda p: pe.evaluate(p).objective)(start)
print(grad["controller"])

# The tuning program over the three gains, solved by the class default (L-BFGS)
wrt = ["controller.Kp", "controller.Ki", "controller.Kd"]
best = Optimizer(pe.program(wrt, params=start), method="auto").solve()
print(best)

# The tuned loop, judged and drawn
tuned = pe.unpack(best.z, wrt, params=start)
print(pe.evaluate(tuned))
pe.loop.plot_trajectory(pe.trajectory(tuned))
