"""Lane-change trajectory optimization with servo + torque bicycle inputs.

Same setup as ``car_trajopt_lanechange.py``, but the plant is
:class:`~minilink.dynamics.catalog.vehicles.jax_vehicles.BicycleDynServo`
with ``u = [tau_cmd, delta_cmd]`` instead of wheel/steer rate commands.

Run from repo root::

    PYTHONPATH=. python examples/projects/car_trajopt/car_trajopt_lanechange_servo.py
"""

import numpy as np

from minilink.core.backends import configure_jax
from minilink.core.costs import QuadraticCost
from minilink.dynamics.catalog.vehicles.jax_vehicles import BicycleDynServo
from minilink.planning.problems import PlanningProblem
from minilink.planning.trajectory_optimization.planner import (
    TrajectoryOptimizationPlanner,
)

configure_jax(enable_x64=True)

# --- Problem setup ---
PRINT_SOLVE_REPORT = True
PRINT_RESULT_SUMMARY = not PRINT_SOLVE_REPORT
SCIPY_DISP = False
TF = 3.0
N_STEPS = 30
U_0 = 5.0
U_TARGET = U_0 * 2.0
Y_GOAL = 2.5
HEADING_TARGET = 0.0

sys = BicycleDynServo()

x_start = np.array([0.0, 0.0, 0.0, U_0, 0.0, 0.0, U_0 / sys.params["r_r"], 0.0, 0.0])
x_ref = np.array(
    [
        0.0,
        Y_GOAL,
        HEADING_TARGET,
        U_TARGET,
        0.0,
        0.0,
        U_TARGET / sys.params["r_r"],
        0.0,
        0.0,
    ]
)

Q = np.diag([0.0, 10.0, 1.0, 10.0, 0.1, 0.1, 0.01, 1.0, 0.0])
R = np.diag([1e-4, 10.0])
S = np.diag([0.0, 10.0, 100.0, 10.0, 0.0, 0.1, 0.01, 1.0, 0.0])

ubar = np.array([0.0, 0.0])

cost = QuadraticCost.from_system(
    sys,
    Q=Q,
    R=R,
    S=S,
    xbar=x_ref,
    ubar=ubar,
)
problem = PlanningProblem(
    sys=sys,
    tf=TF,
    x_start=x_start,
    cost=cost,
)

planner = TrajectoryOptimizationPlanner(
    problem,
    n_steps=N_STEPS,
    transcription="direct_collocation",
    compile_backend="jax",
    solve_disp=PRINT_SOLVE_REPORT,
    optimizer_options={
        "disp": SCIPY_DISP,
        "maxiter": 500,
        "ftol": 1e-1,
    },
)

traj = planner.solve().trajectory
planner.plot_solution(signals=("x", "u"))
sys.traj = traj
sys.animate()
