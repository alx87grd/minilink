"""Textbook objects report through ``print(obj)``; a diagram also draws inline in Jupyter."""

import numpy as np

from minilink import (
    BallSet,
    BoxSet,
    DiagramSystem,
    DoubleIntegrator,
    Gaussian,
    Integrator,
    LQRPlanner,
    MathematicalProgram,
    Optimizer,
    PlanningProblem,
    QuadraticCost,
    Simulator,
    Step,
    linearize,
)

plant = Integrator()
cost = QuadraticCost.from_system(plant)

print(plant)

diagram = DiagramSystem()
step = Step(final_value=0.5, step_time=1.0)
diagram.add_subsystem(step, "step")
diagram.add_subsystem(plant, "plant")
diagram.connect("step", "y", "plant", "u")
diagram.name = "step_to_integrator"
print(diagram)

sim = Simulator(plant, tf=1.0, n_steps=11)
traj = sim.solve()
print(traj)
print(sim)

print(BoxSet([-1.0], [1.0]))
print(BallSet([0.0], 2.0))
print(Gaussian([0.0], [0.2]))

print(cost)
print(cost + QuadraticCost.from_system(DoubleIntegrator()))

print(PlanningProblem(plant, tf=2.0, cost=cost))

print(
    PlanningProblem(
        plant,
        tf=1.0,
        x_start=Gaussian([0.0], [0.1]),
        cost=cost,
        criterion="expectation",
    )
)

print(linearize(plant))

di = DoubleIntegrator()
di_problem = PlanningProblem(
    di,
    tf=5.0,
    cost=QuadraticCost.from_system(di),
    U=di.input_box,
)
solution = LQRPlanner(di_problem).solve(evaluate=True)
print(solution)

program = MathematicalProgram(
    n_z=2,
    J=lambda z: float(z @ z),
    grad_J=lambda z: 2.0 * z,
    lower=np.array([-1.0, -1.0]),
    upper=np.array([1.0, 1.0]),
)
print(program)

opt = Optimizer(program, z0=[0.5, -0.5], method="l-bfgs")
print(opt)
