"""Regression tests: discontinuous closed-loop solver behavior (SMC pendulum).

SciPy adaptive solvers (``scipy``, ``scipy_stiff``, …) on this loop can hang
for minutes near switching surfaces — do **not** include them in default smoke
runs. Euler vs RK4 comparisons use short horizons and fixed ``dt`` only.
"""

from __future__ import annotations

import unittest

import numpy as np
import pytest

from minilink.blocks.sources import Step
from minilink.control.modelbased import SlidingModeController
from minilink.core.composition import closed_loop_qdq
from minilink.dynamics.catalog.pendulum.pendulum import Pendulum
from minilink.simulation.simulator import DISCONTINUOUS_AUTO_DT_SCALE, Simulator

TF = 1.0
DT_COARSE = 0.1


def _build_smc_diagram():
    """Match ``examples/scripts/control/demo_sliding_mode_pendulum.py`` setup."""
    plant = Pendulum(length=1.0, mass=1.0)
    model = Pendulum(length=1.0, mass=0.5)
    plant.x0 = np.array([1.0, 0.0])
    ref = Step(
        initial_value=np.array([np.pi, 0.0]),
        final_value=np.array([0.0, 0.0]),
        step_time=0.5,
    )
    smc = SlidingModeController(model, lam=20.0, gain=8.0, nab=0.15)
    return ref >> closed_loop_qdq(smc, plant)


def _max_ddq_consistency_error(diagram, traj):
    """Max |Δdq/Δt − ddq_from_f| on interior grid points."""
    q = traj.x[0]
    dq = traj.x[1]
    dt = np.diff(traj.t)
    ddq_num = np.diff(dq) / dt
    errors = []
    for i in range(len(ddq_num)):
        x_i = np.array([q[i], dq[i]])
        k = diagram.f(x_i, traj.u[:, i], traj.t[i])
        errors.append(abs(ddq_num[i] - float(k[1])))
    return float(np.max(errors)) if errors else 0.0


def _integrate(diagram, *, solver, dt=DT_COARSE, tf=TF):
    return diagram.compute_trajectory(
        tf=tf,
        dt=dt,
        solver=solver,
        solver_warnings="ignore",
        show=False,
        verbose=False,
    )


class TestDiscontinuousSolvers(unittest.TestCase):
    def setUp(self):
        self.diagram = _build_smc_diagram()
        self.assertTrue(self.diagram.solver_info["discontinuous_behavior"])

    def test_auto_solver_selects_euler(self):
        sim = Simulator(
            self.diagram,
            tf=TF,
            dt=DT_COARSE,
            solver_warnings="ignore",
            verbose=False,
        )
        self.assertEqual(sim.solver_mode, "euler")

    def test_auto_dt_uses_discontinuous_scale(self):
        sim = Simulator(
            self.diagram,
            tf=0.01,
            solver_warnings="ignore",
            verbose=False,
        )
        expected_dt = 0.001 * DISCONTINUOUS_AUTO_DT_SCALE
        self.assertAlmostEqual(sim.t[1] - sim.t[0], expected_dt)

    def test_euler_matches_f_based_ddq_better_than_rk4_on_coarse_dt(self):
        traj_euler = _integrate(self.diagram, solver="euler")
        traj_rk4 = _integrate(self.diagram, solver="rk4_fixedsteps")

        err_euler = _max_ddq_consistency_error(self.diagram, traj_euler)
        err_rk4 = _max_ddq_consistency_error(self.diagram, traj_rk4)

        self.assertLess(
            err_euler,
            err_rk4,
            "Euler should align Δdq/Δt with f-based ddq better than RK4 sub-steps",
        )

    def test_rk4_coarse_dt_can_freeze_state_while_euler_moves(self):
        traj_euler = _integrate(self.diagram, solver="euler")
        traj_rk4 = _integrate(self.diagram, solver="rk4_fixedsteps")

        dq_motion_euler = float(np.max(np.abs(np.diff(traj_euler.x[1]))))
        dq_motion_rk4 = float(np.max(np.abs(np.diff(traj_rk4.x[1]))))

        self.assertGreater(
            dq_motion_euler,
            dq_motion_rk4,
            "RK4 sub-step cancellation can stall dq updates on a coarse grid",
        )

    @pytest.mark.skip(
        reason=(
            "SciPy adaptive solvers on discontinuous SMC closed loops can hang "
            "for minutes (Zeno-like refinement near sign(s) switches); not for smoke."
        )
    )
    def test_scipy_adaptive_on_smc_not_for_smoke(self):
        """Documented skip — run manually only with a long timeout if needed."""
        _integrate(self.diagram, solver="scipy_stiff", tf=0.5)


if __name__ == "__main__":
    unittest.main()
