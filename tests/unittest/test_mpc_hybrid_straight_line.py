"""Stateless MPC hybrid straight-line bicycle vs stateless planner."""

import unittest

import numpy as np
import pytest

pytest.importorskip("jax")

from minilink.core.backends import configure_jax  # noqa: E402
from minilink.core.costs import QuadraticCost  # noqa: E402
from minilink.dynamics.catalog.vehicles.dynamic_bicycle import (  # noqa: E402
    JaxDynamicBicycleRateInputsUY,
)
from minilink.planning.mpc import (  # noqa: E402
    MPCDirectCollocationTranscription,
    MPCOptions,
    MPCPlanner,
    mpc_animation_overlays,
    mpc_plans_from_rollout,
    mpc_stateless_controller,
)
from minilink.planning.problems import PlanningProblem  # noqa: E402
from minilink.planning.trajectory_optimization.direct_collocation import (  # noqa: E402
    DirectCollocationOptions,
)
from minilink.simulation.hybrid_simulator import HybridSimulator  # noqa: E402


def _build_bicycle_hybrid(*, mpc_hz=5.0):
    configure_jax(enable_x64=True)

    u_target = 4.0
    mpc_horizon = 1.0
    mpc_steps = 8

    sys_mpc = JaxDynamicBicycleRateInputsUY()
    sys_sim = JaxDynamicBicycleRateInputsUY()
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
        sys.inputs["u"].lower_bound = np.array([-w_rear_dot_max, -delta_dot_max])
        sys.inputs["u"].upper_bound = np.array([w_rear_dot_max, delta_dot_max])

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
    mpc = mpc_stateless_controller(mpc_planner)
    hybrid = (mpc % mpc_dt) @ sys_sim
    return hybrid, mpc_planner, transcription, template_problem


@pytest.mark.optional
@pytest.mark.jax
class TestMpcHybridStraightLine(unittest.TestCase):
    def test_stateless_u_ff_matches_planner_in_hybrid_sim(self):
        hybrid, mpc_planner, transcription, problem = _build_bicycle_hybrid()
        n_steps = 4
        sim = HybridSimulator(
            hybrid,
            t0=0.0,
            n_steps=n_steps,
            plant_dt_inner=0.005,
        )
        result = sim.solve()

        plant_eval = hybrid.plant.compile()
        u_nom = hybrid.plant.get_u_from_input_ports()
        y0 = np.asarray(
            plant_eval.outputs(hybrid.plant.x0, u_nom, 0.0)["y"],
            dtype=float,
        )
        y_hist = result.computer.signals["y"]

        for col in range(n_steps):
            y_k = y0 if col == 0 else y_hist[:, col - 1]
            plan = mpc_planner.step(y_k, initial_guess=None)
            np.testing.assert_allclose(
                result.computer.signals["u_ff"][:, col],
                plan.u[:, 0],
                rtol=1e-4,
                atol=1e-4,
            )

    def test_mpc_plans_from_rollout_smoke(self):
        hybrid, _planner, transcription, problem = _build_bicycle_hybrid()
        n_steps = 3
        result = HybridSimulator(
            hybrid,
            t0=0.0,
            n_steps=n_steps,
            plant_dt_inner=0.005,
        ).solve()
        plans = mpc_plans_from_rollout(
            result.computer,
            transcription,
            problem,
            t0=0.0,
            dt_mpc=hybrid.computer.schedule.dt_base,
        )
        self.assertEqual(len(plans), n_steps)
        self.assertEqual(plans[0][1].n_samples, transcription.options.n_steps)

    def test_mpc_animation_overlays_smoke(self):
        hybrid, planner, _transcription, _problem = _build_bicycle_hybrid()
        n_steps = 3
        result = HybridSimulator(
            hybrid,
            t0=0.0,
            n_steps=n_steps,
            plant_dt_inner=0.005,
        ).solve()
        overlays = mpc_animation_overlays(
            result,
            planner,
            reference_pad=5.0,
        )
        self.assertEqual(len(overlays), 1)
        self.assertEqual(type(overlays[0]).__name__, "SceneHistory")


if __name__ == "__main__":
    unittest.main()
