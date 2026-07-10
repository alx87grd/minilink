"""Straight-line rate-MPC via layered hybrid shortcuts.

Uses ``mpc % MPC_DT`` (or ``mpc.export_to_computer()`` for warm-start) and
``computer @ plant`` with :class:`JaxDynamicBicycleRateInputsUY`.

Run from repo root::

    python examples/scripts/hybrid/demo_dynamic_bicycle_rate_mpc_straight_line.py
"""

import numpy as np

from minilink.core.backends import configure_jax
from minilink.core.composition import _propagate_animation_camera
from minilink.core.costs import QuadraticCost
from minilink.dynamics.catalog.vehicles.dynamic_bicycle import (
    JaxDynamicBicycleRateInputsUY,
)
from minilink.graphical.animation.primitives import (
    CustomLine,
    HorizonPolyline,
    TrajectoryPolyline,
)
from minilink.graphical.catalog import SceneHistory
from minilink.planning.mpc import (
    MPCDirectCollocationTranscription,
    MPCOptions,
    MPCPlanner,
    mpc_plans_from_rollout,
    mpc_stateful_controller,
    mpc_stateless_controller,
)
from minilink.planning.mpc.warm_start import mpc_default_computer_x0
from minilink.planning.problems import PlanningProblem
from minilink.planning.trajectory_optimization.direct_collocation import (
    DirectCollocationOptions,
)

USE_WARM_START = True
STEP_DISP = True

U_TARGET = 4.0
TF_SIM = 5.0

MPC_HZ = 5.0
SIM_HZ = 200.0
MPC_HORIZON = 2.0
MPC_STEPS = 20
MPC_MAXITER = 150
MPC_FTOL = 1e-2
MPC_DT = 1.0 / MPC_HZ
SIM_DT = 1.0 / SIM_HZ

W_REAR_MAX = 90.0
DELTA_MAX = 0.55
W_REAR_DOT_MAX = 80.0
DELTA_DOT_MAX = 2.0

REF_X_PAD = 20.0
CAMERA_SCALE = 12.0

configure_jax(enable_x64=True)

sys_mpc = JaxDynamicBicycleRateInputsUY()
sys_sim = JaxDynamicBicycleRateInputsUY()
sys_sim.params["mass"] = 1.03 * sys_mpc.params["mass"]
sys_sim.params["inertia"] = 1.02 * sys_mpc.params["inertia"]

for sys in (sys_mpc, sys_sim):
    sys.state.lower_bound[6] = 0.0
    sys.state.upper_bound[6] = W_REAR_MAX
    sys.state.lower_bound[7] = -DELTA_MAX
    sys.state.upper_bound[7] = DELTA_MAX
    sys.inputs["u"].lower_bound = np.array([-W_REAR_DOT_MAX, -DELTA_DOT_MAX])
    sys.inputs["u"].upper_bound = np.array([W_REAR_DOT_MAX, DELTA_DOT_MAX])

r_r = sys_mpc.params["r_r"]
w_rear_ref = U_TARGET / r_r
x_ref = np.array([0.0, 0.0, 0.0, U_TARGET, 0.0, 0.0, w_rear_ref, 0.0])
cost = QuadraticCost.from_system(
    sys_mpc,
    Q=np.diag([0.0, 12.0, 18.0, 0.5, 4.0, 6.0, 0.1, 100.0]),
    R=np.diag([1.0, 25.0]),
    S=np.diag([0.0, 30.0, 40.0, 2.0, 12.0, 18.0, 0.1, 100.0]),
    xbar=x_ref,
    ubar=np.zeros(2),
)

x0 = np.array([0.0, 3.0, 0.0, U_TARGET * 0.8, 0.0, 0.0, (U_TARGET * 0.8) / r_r, 0.0])
sys_sim.x0 = x0.copy()

template_problem = PlanningProblem(sys=sys_mpc, x_start=x0, cost=cost)
transcription = MPCDirectCollocationTranscription(
    DirectCollocationOptions(tf=MPC_HORIZON, n_steps=MPC_STEPS)
)
mpc_planner = MPCPlanner(
    template_problem,
    transcription=transcription,
    options=MPCOptions(
        compile_backend="jax",
        optimizer_method="scipy_slsqp",
        record_solve_time=True,
        optimizer_options={"maxiter": MPC_MAXITER, "ftol": MPC_FTOL},
    ),
)

mpc = (
    mpc_stateful_controller(mpc_planner, dt_mpc=MPC_DT, step_disp=STEP_DISP)
    if USE_WARM_START
    else mpc_stateless_controller(mpc_planner, step_disp=STEP_DISP)
)
x0_computer = mpc_default_computer_x0(mpc_planner) if USE_WARM_START else None

if USE_WARM_START:
    computer = mpc.export_to_computer()
else:
    computer = mpc % MPC_DT
hybrid = computer @ sys_sim


hybrid.plot_diagram()

mode = "warm-start" if USE_WARM_START else "stateless"
print(f"Hybrid MPC straight-line tracking (rate inputs, {mode})")
print(f"  compile={mpc_planner.compile_time_s:.3f}s (once)")
print(f"  mpc_hz={MPC_HZ}, sim_hz={SIM_HZ}, horizon={MPC_HORIZON}s")

result = hybrid.compute_trajectory(
    t0=0.0,
    tf=TF_SIM,
    x0_plant=x0,
    x0_computer=x0_computer,
    plant_dt_inner=SIM_DT,
    verbose=True,
)

traj = hybrid.traj
x0_ref = float(traj.x[0, 0]) - REF_X_PAD
x1_ref = float(traj.x[0, -1]) + REF_X_PAD
mpc_plans = mpc_plans_from_rollout(
    result.computer,
    transcription,
    template_problem,
    t0=0.0,
    dt_mpc=MPC_DT,
)
history = SceneHistory(
    reference=CustomLine(
        np.array([[x0_ref, 0.0, 0.0], [x1_ref, 0.0, 0.0]]),
        color="k",
        linewidth=1.0,
        style="--",
    ),
    trail=TrajectoryPolyline(
        traj,
        window="prefix",
        color="#1565c0",
        style="--",
        linewidth=1.0,
    ),
    horizon=HorizonPolyline(
        mpc_plans,
        color="#ef6c00",
        linewidth=2.0,
        style="--",
    ),
)

sys_sim.params = dict(sys_sim.params)
sys_sim.camera_scale = CAMERA_SCALE
_propagate_animation_camera(hybrid.plant, sys_sim)
hybrid.plot_trajectory(signals=("x", "u"))
hybrid.animate(overlays=[history])
