"""Warm-start MPC hybrid straight-line bicycle vs warm-started hand loop."""

import unittest

import numpy as np
import pytest

pytest.importorskip("jax")

from minilink.blocks.routing import Demux  # noqa: E402
from minilink.core.backends import configure_jax  # noqa: E402
from minilink.core.costs import QuadraticCost  # noqa: E402
from minilink.core.diagram import DiagramSystem, StepDiagramSystem  # noqa: E402
from minilink.core.hybrid_diagram import HybridDiagram  # noqa: E402
from minilink.dynamics.catalog.vehicles.dynamic_bicycle import (  # noqa: E402
    JaxDynamicBicycleRateInputs,
)
from minilink.planning.mpc import (  # noqa: E402
    MPCDirectCollocationTranscription,
    MPCOptions,
    MPCPlanner,
    mpc_stateful_controller,
)
from minilink.planning.mpc.warm_start import (  # noqa: E402
    mpc_default_computer_x0,
    mpc_warm_start_guess,
)
from minilink.planning.problems import PlanningProblem  # noqa: E402
from minilink.planning.trajectory_optimization.direct_collocation import (  # noqa: E402
    DirectCollocationOptions,
)
from minilink.simulation.computer import Computer, StepSchedule  # noqa: E402
from minilink.simulation.hybrid_simulator import HybridSimulator  # noqa: E402


def _build_bicycle_hybrid_warm(*, mpc_hz=5.0):
    configure_jax(enable_x64=True)

    u_target = 4.0
    mpc_horizon = 1.0
    mpc_steps = 8

    sys_mpc = JaxDynamicBicycleRateInputs()
    sys_sim = JaxDynamicBicycleRateInputs()
    sys_sim.params["mass"] = 1.03 * sys_mpc.params["mass"]

    w_rear_max = 90.0
    delta_max = 0.55
    w_rear_dot_max = 80.0
    delta_dot_max = 2.0

    for sys in (sys_mpc, sys_sim):
        sys.state.lower_bound[6] = 0.0
        sys.state.upper_bound[6] = w_rear_max
        sys.state.lower_bound[7] = -delta_max
        sys.state.upper_bound[7] = delta_max
        sys.inputs["w_rear_dot"].lower_bound[0] = -w_rear_dot_max
        sys.inputs["w_rear_dot"].upper_bound[0] = w_rear_dot_max
        sys.inputs["delta_dot"].lower_bound[0] = -delta_dot_max
        sys.inputs["delta_dot"].upper_bound[0] = delta_dot_max

    r_r = sys_mpc.params["r_r"]
    w_rear_ref = u_target / r_r
    x_ref = np.array([0.0, 0.0, 0.0, u_target, 0.0, 0.0, w_rear_ref, 0.0])
    cost = QuadraticCost.from_system(
        sys_mpc,
        Q=np.diag([0.0, 12.0, 18.0, 0.5, 4.0, 6.0, 0.1, 100.0]),
        R=np.diag([1.0, 25.0]),
        S=np.diag([0.0, 30.0, 40.0, 2.0, 12.0, 18.0, 0.1, 100.0]),
        xbar=x_ref,
        ubar=np.zeros(2),
    )

    x0 = np.array(
        [0.0, 3.0, 0.0, u_target * 0.8, 0.0, 0.0, (u_target * 0.8) / r_r, 0.0]
    )
    sys_sim.x0 = x0.copy()

    template_problem = PlanningProblem(
        sys=sys_mpc, x_start=x0, cost=cost, tf=mpc_horizon
    )
    transcription = MPCDirectCollocationTranscription(
        DirectCollocationOptions(n_steps=mpc_steps)
    )
    mpc_planner = MPCPlanner(
        template_problem,
        transcription=transcription,
        options=MPCOptions(
            compile_backend="jax",
            optimizer_method="scipy_slsqp",
            optimizer_options={"maxiter": 80, "ftol": 1e-2},
        ),
    )

    mpc_dt = 1.0 / mpc_hz
    mpc = mpc_stateful_controller(mpc_planner, dt_mpc=mpc_dt)
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
        computer=Computer(step_diagram, StepSchedule(dt_base=mpc_dt)),
        plant=plant_diagram,
    )
    hybrid.connect_boundary(
        direction="computer_to_plant", computer_port="u_ff", plant_port="u"
    )
    hybrid.connect_boundary(
        direction="plant_to_computer", computer_port="y", plant_port="y"
    )

    z0 = mpc_default_computer_x0(mpc_planner)
    return hybrid, mpc_planner, transcription, template_problem, mpc_dt, z0


@pytest.mark.optional
@pytest.mark.jax
class TestMpcHybridWarmStartParity(unittest.TestCase):
    def test_warm_started_u_ff_matches_hand_loop(self):
        hybrid, mpc_planner, _transcription, _problem, mpc_dt, z0 = (
            _build_bicycle_hybrid_warm()
        )
        n_steps = 4
        result = HybridSimulator(
            hybrid,
            t0=0.0,
            n_steps=n_steps,
            plant_dt_inner=0.005,
            x0_computer=z0,
        ).solve()

        plant_eval = hybrid.plant.compile()
        u_nom = hybrid.plant.get_u_from_input_ports()
        y0 = np.asarray(
            plant_eval.outputs(hybrid.plant.x0, u_nom, 0.0)["y"],
            dtype=float,
        )
        y_hist = result.computer.signals["y"]
        z_hist = result.computer.x

        for col in range(n_steps):
            y_k = y0 if col == 0 else y_hist[:, col - 1]
            z_prev = z0 if col == 0 else z_hist[:, col - 1]
            guess = mpc_warm_start_guess(
                z_prev,
                y_k,
                mpc_planner,
                dt_mpc=mpc_dt,
                k=col,
            )
            plan = mpc_planner.step(y_k, initial_guess=guess)
            np.testing.assert_allclose(
                result.computer.signals["u_ff"][:, col],
                plan.u[:, 0],
                rtol=1e-4,
                atol=1e-4,
            )


if __name__ == "__main__":
    unittest.main()
