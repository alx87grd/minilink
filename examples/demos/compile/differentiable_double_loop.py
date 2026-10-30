"""One differentiable double loop: a neural inner loop, an MPC around it, co-designed by gradient and analyzed in z."""

import numpy as np

from minilink import Optimizer, Pendulum, PlanningProblem, QuadraticCost, closed_loop
from minilink.analysis import plot_bode
from minilink.blocks import MLP
from minilink.control import ModelPredictiveController
from minilink.planning import TrajectoryOptimizationPlanner

DT = 0.1  # the MPC's period, s
X0 = np.array([1.0, 0.0])  # the start, rad and rad/s

# The inner loop: a small network u = MLP(x) on the pendulum, a free input d added to u
plant = Pendulum()
net = MLP(input_dim=2, output_dim=1, hidden=(4,), output_gain=0.1).rename_ports(u="x")
inner = closed_loop(net, plant, reference=False, disturbance=True)
inner.inputs["d"].lower_bound = np.array([-2.0])
inner.inputs["d"].upper_bound = np.array([2.0])
inner.plot_diagram()

# The outer loop: an MPC that plans d on the inner loop, solved by 5 Adam steps a tick
# (a fixed count, so the whole loop stays one pure function); its setpoint an input r
cost = QuadraticCost.from_system(inner, Q=np.diag([1.0, 0.1]), R=0.01 * np.eye(1))
planner = TrajectoryOptimizationPlanner(
    PlanningProblem(inner, cost=cost, x_start=X0, tf=0.5, U=inner.input_box),
    n_steps=5,
    transcription="single_shooting",
    backend="jax",
    method="adam",
    iterations=5,
)
mpc = ModelPredictiveController(planner, inputs={"x_start": "y", "cost.xbar": "r"})

mpc.plot_diagram()

# The double loop compiled once on JAX: J(p), the 2 s run's cost, a pure function
problem = PlanningProblem(inner, cost=cost, x_start=X0, tf=2.0)
pe = problem.compile(mpc % DT, backend="jax")
loop = pe.loop
loop.plot_diagram()

# 1. One gradient through everything: a plant parameter, a network weight, an MPC weight
wrt = ["plant.plant.m", "plant.controller.b1", "controller.solve.cost.R"]
start = pe.params
program = pe.program(wrt, params=start)
z0 = pe.pack(start, wrt)

# 2. Co-design: the three numbers optimized together
best = Optimizer(program, method="adam", iterations=20).solve()
print(best)
tuned = pe.unpack(best.z, wrt, params=start)

# 3. The whole closed loop linearized in z (network and solver included): its Bode
# plot from the setpoint r to the angle, G(e^{jωΔ})
loop.jacobian(of="controller__command:u", wrt="x")
plot_bode(loop, of=("y", 0), wrt=("r", 0), method="jax", margins=False)
plant.plot_trajectory(pe.trajectory(tuned).of("plant"))
