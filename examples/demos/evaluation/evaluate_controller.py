"""A controller scored on a problem: one run, Monte Carlo over the problem's draws, a comparison, the compiled evaluator."""

import numpy as np

from minilink import (
    Pendulum,
    PlanningProblem,
    QuadraticCost,
    Uniform,
    compare,
    evaluate,
    lqr,
)

N_TRIALS = 100  # Monte Carlo runs per controller

# The plant: a pendulum, to be held at the upright x̄ = (π, 0)

plant = Pendulum()
x_up = np.array([np.pi, 0.0])

# The task: from near the upright, J = ∫ (x − x̄)ᵀ(x − x̄) + uᵀu dt over 5 s

problem = PlanningProblem(
    plant,
    x0=np.array([np.pi - 0.3, 0.0]),
    cost=QuadraticCost.from_system(plant, x_bar=x_up),
    horizon=5.0,
)

# Two controllers: LQR state feedbacks at the upright, u = −K (x − x̄), one designed
# on the task's own weights and one that weighs the state ten times more

controller = lqr(plant, np.eye(2), np.eye(1), x_bar=x_up)
stiff = lqr(plant, 10 * np.eye(2), np.eye(1), x_bar=x_up)

# One run: the controller closed on the plant from the problem's start, scored by its cost

print(evaluate(problem, controller))
print(evaluate(problem, stiff))

# A digital controller runs on its own clock: the same law held at 20 Hz

print(evaluate(problem, controller % 0.05))

# A random problem: the start drawn from a box around the upright, scored over its draws

stochastic = problem.replace(
    x0=Uniform(np.array([np.pi - 0.5, -0.5]), np.array([np.pi + 0.5, 0.5]))
)
print(evaluate(stochastic, controller, n_trials=N_TRIALS, key=0))

# The two controllers side by side, on the same draws

print(compare(stochastic, LQR=controller, stiff=stiff, n_trials=N_TRIALS))

# The compiled evaluator: built once, scored again, its run drawn

pe = problem.compile(controller)
print(pe.evaluate())
plant.plot_trajectory(pe.trajectory().of("plant"))
