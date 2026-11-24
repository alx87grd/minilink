"""A digital PID tuned by gradient: the law on its clock, the loop compiled once, the gains optimized."""

import jax
import numpy as np

from minilink import PID, Optimizer, Pendulum, PlanningProblem, QuadraticCost

TF = 5.0  # [s]
DT = 0.01  # [s] the computer's period
R_REF = 1.0  # [rad] the setpoint

# The task: J = (1/T) ∫₀ᵀ ((θ − r)² + ρ (u − u_ss)²) dt, u_ss = m g l sin r

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

# The digital PID: its difference equation on the computer's clock, closed on the plant

pid = PID(Kp=1.0, Ki=0.0, Kd=1.0)
diagram = pid % DT @ plant

diagram.plot_diagram()

J = problem.compile(diagram, substeps=1)


start = J.params
start["r"] = np.array([R_REF])
print(J.evaluate(start))
traj = J.trajectory(start)
J.loop.plot_trajectory(traj)
J.loop.animate(traj)

# The gradient: ∂J/∂(Kp, Ki, Kd) through the sampled loop, by reverse mode

grad = jax.grad(J)(start)
print(grad["controller"])

# The tuning: min over (Kp, Ki, Kd) of J

wrt = ["controller.Kp", "controller.Ki", "controller.Kd"]
math_program = J.program(wrt, params=start)


best = Optimizer(math_program, method="auto").solve()
print(best)

# The tuned loop, judged, drawn and played

tuned = J.unpack(best.z, wrt, params=start)
print(J.evaluate(tuned))

traj = J.trajectory(tuned)
J.loop.plot_trajectory(traj)
J.loop.animate(traj)
