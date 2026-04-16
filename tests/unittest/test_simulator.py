import unittest
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from minilink.core.framework import DynamicSystem
from minilink.simulation import Simulator


class StableLinearSystem(DynamicSystem):
    def __init__(self):
        super().__init__(1, 1, 1)
        self.name = "StableLinearSystem"
        self.x0 = np.array([1.0])
        self.inputs["u"].nominal_value = np.array([0.0])

    def f(self, x, u, t=0, params=None):
        return np.array([-x[0] + u[0]])

    def h(self, x, u, t=0, params=None):
        return np.array([x[0]])


class DiscontinuousLinearSystem(StableLinearSystem):
    def __init__(self):
        super().__init__()
        self.solver_info["discontinuous_behavior"] = True


class TestNewSimulator(unittest.TestCase):
    def test_default_solver_auto_selects_stiff_for_discontinuous_system(self):
        sim = Simulator(DiscontinuousLinearSystem(), tf=1.0, n_steps=5, verbose=False)
        self.assertEqual(sim.solver_mode, "scipy_stiff")

    def test_invalid_time_grid_arguments_raise_value_error(self):
        sys = StableLinearSystem()
        with self.assertRaises(ValueError):
            Simulator(sys, tf=0.0, n_steps=5, verbose=False)
        with self.assertRaises(ValueError):
            Simulator(sys, tf=1.0, n_steps=1, verbose=False)
        with self.assertRaises(ValueError):
            Simulator(sys, tf=1.0, dt=0.0, verbose=False)

    def test_invalid_x0_shape_raises_value_error(self):
        with self.assertRaises(ValueError):
            Simulator(StableLinearSystem(), x0=np.array([[1.0]]), tf=1.0, n_steps=5, verbose=False)

    def test_scipy_failure_raises_and_keeps_debug_information(self):
        failed_solution = SimpleNamespace(
            success=False,
            status=-1,
            message="integration failed",
            nfev=12,
            njev=0,
            nlu=0,
            t=np.array([0.0]),
            y=np.array([[1.0]]),
        )
        sim = Simulator(StableLinearSystem(), tf=1.0, n_steps=5, verbose=False)

        with patch(
            "minilink.simulation.solver_backends.solve_ivp",
            return_value=failed_solution,
        ):
            with self.assertRaises(RuntimeError) as ctx:
                sim.solve()

        self.assertIn("integration failed", str(ctx.exception))
        self.assertIs(sim.scipy_last_solution, failed_solution)
        self.assertFalse(sim.last_debug["success"])
        self.assertEqual(sim.last_debug["message"], "integration failed")
        self.assertFalse(hasattr(sim, "last_traj"))

    def test_solve_populates_last_traj_and_last_debug(self):
        sim = Simulator(StableLinearSystem(), tf=0.2, n_steps=3, solver="euler", verbose=False)
        traj = sim.solve()

        self.assertIs(sim.last_traj, traj)
        self.assertEqual(sim.last_debug["solver"], "euler")
        self.assertEqual(traj.x.shape, (1, 3))
        self.assertEqual(traj.u.shape, (1, 3))

    def test_solve_forced_validates_shape(self):
        sim = Simulator(StableLinearSystem(), tf=0.2, n_steps=3, solver="euler", verbose=False)
        bad_u = np.zeros((3, 1))

        with self.assertRaises(ValueError):
            sim.solve_forced(bad_u)

    def test_solve_forced_rejects_unsupported_solver(self):
        sim = Simulator(
            StableLinearSystem(),
            tf=0.2,
            n_steps=3,
            solver="rk4_fixedsteps",
            verbose=False,
        )
        u_traj = np.zeros((1, 3))

        with self.assertRaises(ValueError) as ctx:
            sim.solve_forced(u_traj)

        self.assertIn("does not support forced simulations", str(ctx.exception))

    def test_wrapper_compute_trajectory_uses_new_simulator_path(self):
        sys = StableLinearSystem()
        traj = sys.compute_trajectory(tf=0.2, n_steps=3, solver="euler", show=False)

        self.assertIs(sys.traj, traj)
        self.assertEqual(traj.x.shape, (1, 3))
        self.assertEqual(traj.u.shape, (1, 3))
        self.assertEqual(traj.t.shape, (3,))


if __name__ == "__main__":
    unittest.main()
