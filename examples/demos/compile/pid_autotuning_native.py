"""Autotune PID gains with the native planning pipeline and live trajectory plot."""

import numpy as np

from minilink import (
    DiagramSystem,
    IntegralImpedance,
    Optimizer,
    Pendulum,
    PlanningProblem,
    QuadraticCost,
)

# Rollout horizon: N_STEPS periods of DT over TF
TF = 10.0
DT = 0.05
N_STEPS = 200

# Gradient descent on PID gains.
N_LEARN_STEPS = 1000
LR = 1.0

# Live trajectory plot during tuning.
PLOT_EVERY = 5
PLOT_PAUSE = 0.05
PLOT_BACKEND = "matplotlib"

# The tuning cost: the mean over the run of the squared tracking error and of the
# effort above the torque that holds the setpoint,
#   J = (1/T) ∫₀ᵀ ((r − θ)² + ρ (u − u_ss)²) dt,   u_ss = m g l sin r
R_REF = 1.0  # setpoint, rad
RHO = 0.01  # effort weight

plant = Pendulum()
ctl = IntegralImpedance(Kp=1.0, Ki=0.0, Kd=1.0)
U_SS = plant.params["m"] * plant.params["gravity"] * plant.params["l"] * np.sin(R_REF)

diagram = DiagramSystem()
diagram.add_subsystem(plant, "plant")
diagram.add_subsystem(ctl, "controller")
diagram.add_input_port("r", dim=1)
diagram.connect("input", "r", "controller", "r")
diagram.connect("plant", "y", "controller", "y")
diagram.connect("controller", "u", "plant", "u")
diagram.inputs["r"].nominal_value = np.array([R_REF])

# The task: the cost as a quadratic on the plant, Q on θ − r, R on u − u_ss
cost = QuadraticCost.from_system(
    plant,
    Q=np.diag([1.0 / TF, 0.0]),
    R=RHO / TF * np.eye(1),
    x_bar=np.array([R_REF, 0.0]),
    u_bar=np.array([U_SS]),
)
problem = PlanningProblem(plant, cost=cost, horizon=N_STEPS * DT)

# The run compiled once, and its program: z = (Kp, Ki, Kd), J(z) the run's cost
pe = problem.compile(diagram, DT)
wrt = ["controller.Kp", "controller.Ki", "controller.Kd"]
start = pe.unpack(np.array([1.0, 0.0, 1.0]), wrt)
program = pe.program(wrt, params=start)

# 1. Native gradient descent with live closed-loop trajectory plot
descent = Optimizer(
    program,
    method="gd",
    options={"maxiter": N_LEARN_STEPS},
    learning_rate=LR,
).solve(
    callback=pe.live_plot(
        wrt,
        params=start,
        every=PLOT_EVERY,
        pause=PLOT_PAUSE,
        backend=PLOT_BACKEND,
    )
)
print(descent)

# 2. The class default (L-BFGS) finishes the tuning in a few steps
best = Optimizer(program, method="auto").solve()
print(best)

# The tuned loop, judged and drawn by the problem's evaluator
tuned = pe.unpack(best.z, wrt, start)
print(pe.evaluate(tuned))
plant.plot_trajectory(pe.trajectory(tuned).of("plant"))
