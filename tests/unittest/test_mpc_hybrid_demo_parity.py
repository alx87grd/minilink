"""Hybrid warm-start MPC vs hand-rolled baseline from mpc/ straight-line demo."""

from __future__ import annotations

import unittest

import numpy as np
import pytest

pytest.importorskip("jax")

from minilink.blocks.routing import Demux  # noqa: E402
from minilink.control.mpc import (
    ModelPredictiveController,
)
from minilink.control.mpc.utilities import (  # noqa: E402
    mpc_default_computer_x0,
    warm_start_guess_from_prev_plan,
)
from minilink.core.backends import configure_jax  # noqa: E402
from minilink.core.costs import QuadraticCost  # noqa: E402
from minilink.core.diagram import DiagramSystem, StepDiagramSystem  # noqa: E402
from minilink.core.hybrid_diagram import HybridDiagram  # noqa: E402
from minilink.core.trajectory import Trajectory  # noqa: E402
from minilink.dynamics.catalog.vehicles.dynamic_bicycle import (  # noqa: E402
    JaxDynamicBicycleRateInputs,
)
from minilink.planning.problems import PlanningProblem  # noqa: E402
from minilink.planning.trajectory_optimization.direct_collocation import (
    DirectCollocationOptions,
    DirectCollocationTranscription,
)
from minilink.planning.trajectory_optimization.planner import (
    TrajectoryOptimizationOptions,
    TrajectoryOptimizationPlanner,
)
from minilink.simulation.computer import Computer, StepSchedule  # noqa: E402
from minilink.simulation.hybrid_simulator import HybridSimulator  # noqa: E402

# Match examples/scripts/mpc/demo_dynamic_bicycle_rate_mpc_straight_line.py
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
SUBSTEPS = max(1, int(round(MPC_DT / SIM_DT)))

W_REAR_MAX = 90.0
DELTA_MAX = 0.55
W_REAR_DOT_MAX = 80.0
DELTA_DOT_MAX = 2.0


def _configure_bicycle_systems():
    configure_jax(enable_x64=True)

    sys_mpc = JaxDynamicBicycleRateInputs()
    sys_sim = JaxDynamicBicycleRateInputs()
    sys_sim.params["mass"] = 1.03 * sys_mpc.params["mass"]
    sys_sim.params["inertia"] = 1.02 * sys_mpc.params["inertia"]

    for sys in (sys_mpc, sys_sim):
        sys.state.lower_bound[6] = 0.0
        sys.state.upper_bound[6] = W_REAR_MAX
        sys.state.lower_bound[7] = -DELTA_MAX
        sys.state.upper_bound[7] = DELTA_MAX
        sys.inputs["w_rear_dot"].lower_bound[0] = -W_REAR_DOT_MAX
        sys.inputs["w_rear_dot"].upper_bound[0] = W_REAR_DOT_MAX
        sys.inputs["delta_dot"].lower_bound[0] = -DELTA_DOT_MAX
        sys.inputs["delta_dot"].upper_bound[0] = DELTA_DOT_MAX

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
    x0 = np.array(
        [0.0, 3.0, 0.0, U_TARGET * 0.8, 0.0, 0.0, (U_TARGET * 0.8) / r_r, 0.0]
    )
    return sys_mpc, sys_sim, cost, x0


def _make_planner(sys_mpc, cost, x0):
    problem = PlanningProblem(sys=sys_mpc, x_start=x0, cost=cost, tf=MPC_HORIZON)
    transcription = DirectCollocationTranscription(
        DirectCollocationOptions(n_steps=MPC_STEPS)
    )
    planner = TrajectoryOptimizationPlanner(
        problem,
        transcription=transcription,
        options=TrajectoryOptimizationOptions(
            compile_backend="jax",
            record_solve_time=True,
            optimizer_method="scipy_slsqp",
            optimizer_options={"maxiter": MPC_MAXITER, "ftol": MPC_FTOL},
        ),
    )
    return planner, transcription, problem


def run_hand_loop_warm_mpc(*, sys_sim, planner, problem):
    """
    Run the warm-started manual loop from ``mpc/demo_dynamic_bicycle_rate_mpc_straight_line.py``.

    Returns MPC-fire times, ``u_hold`` commands, plant states at fires, and the fine plant traj.
    """
    sim_evaluator = sys_sim.compile(backend="jax", verbose=False)
    planner.compile_parametric_program()
    x0 = np.asarray(problem.x_start, dtype=float).reshape(-1)

    t_mpc = []
    u_mpc = []
    x_mpc = []
    t_hist = [0.0]
    x_hist = [x0.copy()]
    u_hist = [np.zeros(sys_sim.m)]

    x = x0.copy()
    t = 0.0
    u_hold = np.zeros(sys_sim.m)
    prev_plan = None
    next_mpc_t = 0.0

    while t < TF_SIM - 1e-12:
        if t >= next_mpc_t - 1e-12:
            guess = warm_start_guess_from_prev_plan(
                prev_plan,
                x,
                planner,
                dt_shift=MPC_DT,
                horizon=MPC_HORIZON,
                t_anchor=t,
            )
            plan = planner.step(x, initial_guess=guess)
            prev_plan = plan
            u_hold = np.asarray(plan.u[:, 0], dtype=float).reshape(-1).copy()
            t_mpc.append(float(t))
            u_mpc.append(u_hold.copy())
            x_mpc.append(x.copy())
            next_mpc_t += MPC_DT

        for _ in range(SUBSTEPS):
            if t >= TF_SIM:
                break
            x = sim_evaluator.rk4_step(x, u_hold, t, SIM_DT)
            t += SIM_DT
            t_hist.append(t)
            x_hist.append(x.copy())
            u_hist.append(u_hold.copy())

    traj = Trajectory(
        t=np.asarray(t_hist, dtype=float),
        x=np.asarray(x_hist, dtype=float).T,
        u=np.asarray(u_hist, dtype=float).T,
    )
    return {
        "t_mpc": np.asarray(t_mpc, dtype=float),
        "u_mpc": np.asarray(u_mpc, dtype=float).T,
        "x_mpc": np.asarray(x_mpc, dtype=float).T,
        "traj": traj,
    }


def build_hybrid_warm_mpc(*, sys_mpc, sys_sim, planner):
    mpc = ModelPredictiveController(planner, dt_mpc=MPC_DT, warm_start=True)
    z0 = mpc_default_computer_x0(planner)

    step_diagram = StepDiagramSystem()
    step_diagram.add_subsystem(mpc, "mpc")
    step_diagram.add_input_port("y", dim=int(sys_mpc.n))
    step_diagram.connect("input", "y", "mpc", "y")
    step_diagram.connect_new_output_port("mpc", "u_ff", "u_ff")
    step_diagram.connect_new_output_port("mpc", "x_ff", "x_ff")
    step_diagram.connect_new_output_port("mpc", "z", "z")

    plant_diagram = DiagramSystem()
    plant_diagram.add_subsystem(sys_sim, "bike")
    plant_diagram.add_subsystem(Demux(dims=(1, 1)), "split")
    plant_diagram.add_input_port("u", dim=2)
    plant_diagram.connect("input", "u", "split", "u")
    plant_diagram.connect("split", "out0", "bike", "w_rear_dot")
    plant_diagram.connect("split", "out1", "bike", "delta_dot")
    plant_diagram.connect_new_output_port("bike", "y", "y")

    hybrid = HybridDiagram(
        computer=Computer(step_diagram, StepSchedule(dt_base=MPC_DT)),
        plant=plant_diagram,
    )
    hybrid.connect_boundary(
        direction="computer_to_plant", computer_port="u_ff", plant_port="u"
    )
    hybrid.connect_boundary(
        direction="plant_to_computer", computer_port="y", plant_port="y"
    )
    return hybrid, z0


def _plant_state_at_times(traj: Trajectory, times):
    """Nearest fine-grid plant state at each requested time."""
    t_arr = np.asarray(traj.t, dtype=float).reshape(-1)
    states = []
    for t_q in times:
        idx = int(np.argmin(np.abs(t_arr - float(t_q))))
        states.append(np.asarray(traj.x[:, idx], dtype=float).reshape(-1))
    return np.asarray(states, dtype=float).T


@pytest.mark.optional
@pytest.mark.jax
class TestMpcHybridDemoParity(unittest.TestCase):
    def test_warm_hybrid_matches_hand_loop_baseline(self):
        sys_mpc, sys_sim, cost, x0 = _configure_bicycle_systems()
        sys_sim.x0 = x0.copy()

        planner_hand, _transcription, problem = _make_planner(sys_mpc, cost, x0)
        hand = run_hand_loop_warm_mpc(
            sys_sim=sys_sim,
            planner=planner_hand,
            problem=problem,
        )

        planner_hybrid, _, _ = _make_planner(sys_mpc, cost, x0)
        hybrid, z0 = build_hybrid_warm_mpc(
            sys_mpc=sys_mpc,
            sys_sim=sys_sim,
            planner=planner_hybrid,
        )
        result = HybridSimulator(
            hybrid,
            t0=0.0,
            tf=TF_SIM,
            plant_dt_inner=SIM_DT,
            x0_plant=x0,
            x0_computer=z0,
            compile_backend="jax",
        ).solve()

        n_mpc = hand["t_mpc"].size
        self.assertEqual(result.computer.n_samples, n_mpc)

        u_hybrid = np.asarray(result.computer.signals["u_ff"][:, :n_mpc], dtype=float)
        np.testing.assert_allclose(
            u_hybrid,
            hand["u_mpc"],
            rtol=1e-4,
            atol=1e-4,
            err_msg="u_ff at MPC fires must match hand-loop u_hold",
        )

        plant_eval = hybrid.plant.compile(backend="jax")
        u_nom = hybrid.plant.get_u_from_input_ports()
        y0 = np.asarray(
            plant_eval.outputs(hybrid.plant.x0, u_nom, 0.0)["y"],
            dtype=float,
        )
        y_hist = result.computer.signals["y"]
        for col in range(n_mpc):
            y_k = y0 if col == 0 else y_hist[:, col - 1]
            np.testing.assert_allclose(
                y_k,
                hand["x_mpc"][:, col],
                rtol=1e-4,
                atol=1e-4,
                err_msg=f"measurement at MPC fire {col}",
            )

        x_hybrid_at_mpc = _plant_state_at_times(result.plant, hand["t_mpc"])
        np.testing.assert_allclose(
            x_hybrid_at_mpc,
            hand["x_mpc"],
            rtol=1e-3,
            atol=1e-3,
            err_msg="plant state at MPC fire times must match hand loop",
        )


if __name__ == "__main__":
    unittest.main()
