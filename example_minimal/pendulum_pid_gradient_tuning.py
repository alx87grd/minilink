"""A digital PID tuned by gradient: the law on its clock, the loop compiled once, the gains optimized."""

import jax
import numpy as np

from minilink import PID, Optimizer, Pendulum, PlanningProblem, QuadraticCost

########################################################
### Tunables
########################################################

TF = 5.0  # [s]
DT = 0.01  # [s]
R_REF = 1.0  # [rad]

########################################################
### The task
########################################################

plant = Pendulum()
u_ss = plant.params["m"] * plant.params["gravity"] * plant.params["l"] * np.sin(R_REF)
rho = 0.01

df_dp = plant.jacobian(of="f", wrt="params")
print(df_dp)

cost = QuadraticCost.from_system(
    plant,
    Q=np.diag([1.0 / TF, 0.0]),
    R=rho / TF * np.eye(1),
    x_bar=np.array([R_REF, 0.0]),
    u_bar=np.array([u_ss]),
)
problem = PlanningProblem(plant, cost=cost, horizon=TF)

########################################################
### Digital PID on the plant
########################################################

pid = PID(Kp=1.0, Ki=0.0, Kd=1.0)
diagram = pid % DT @ plant

diagram.plot_diagram()

J = problem.compile(diagram, substeps=1)

########################################################
### Baseline cost before tuning
########################################################

start = J.params
start["r"] = np.array([R_REF])
print(J.evaluate(start))
traj = J.trajectory(start)
J.loop.plot_trajectory(traj)
J.loop.animate(traj)

########################################################
### Gradient of the cost in the gains
########################################################

grad = jax.grad(J)(start)
print(grad["controller"])

########################################################
### Gain optimization
########################################################

wrt = ["controller.Kp", "controller.Ki", "controller.Kd"]
math_program = J.program(wrt, params=start)


best = Optimizer(math_program, method="auto").solve()
print(best)

########################################################
### Tuned loop
########################################################

tuned = J.unpack(best.z, wrt, params=start)
print(J.evaluate(tuned))

traj = J.trajectory(tuned)
J.loop.plot_trajectory(traj)
J.loop.animate(traj)
