import unittest
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
import pytest

from minilink.compile.jax_utils import array_module
from minilink.core.system import DynamicSystem
from minilink.simulation.simulator import COMPILE_BACKEND_AUTO, Simulator


def _have_jax() -> bool:
    try:
        import jax  # noqa: F401

        return True
    except ImportError:
        return False


class StableLinearSystem(DynamicSystem):
    def __init__(self):
        super().__init__(1, 1, 1)
        self.name = "StableLinearSystem"
        self.x0 = np.array([1.0])
        self.inputs["u"].nominal_value = np.array([0.0])

    def f(self, x, u, t=0, params=None):
        xp = array_module(x)
        return xp.array([-x[0] + u[0]])

    def h(self, x, u, t=0, params=None):
        xp = array_module(x)
        return xp.array([x[0]])


class TwoPortLinearSystem(DynamicSystem):
    def __init__(self):
        super().__init__(1, 0, 1)
        self.name = "TwoPortLinearSystem"
        self.inputs = {}
        self.add_input_port(1, "left", nominal_value=np.array([1.0]))
        self.add_input_port(1, "right", nominal_value=np.array([2.0]))
        self.m = 2
        self.x0 = np.array([0.0])

    def f(self, x, u, t=0, params=None):
        xp = array_module(x)
        return xp.array([-x[0] + u[0] + 2.0 * u[1]])

    def h(self, x, u, t=0, params=None):
        xp = array_module(x)
        return xp.array([x[0]])


class DiscontinuousLinearSystem(StableLinearSystem):
    def __init__(self):
        super().__init__()
        self.solver_info["discontinuous_behavior"] = True


class TestNewSimulator(unittest.TestCase):
    def test_default_solver_auto_selects_stiff_for_discontinuous_system(self):
        sim = Simulator(DiscontinuousLinearSystem(), tf=1.0, n_steps=5, verbose=False)
        self.assertEqual(sim.solver_mode, "scipy_stiff")

    def test_large_grid_numpy_stays_scipy(self):
        sim = Simulator(
            StableLinearSystem(),
            tf=1.0,
            n_steps=10_000,
            compile_backend="numpy",
            verbose=False,
        )
        self.assertEqual(sim.solver_mode, "scipy")

    def test_default_compile_backend_is_numpy(self):
        sim = Simulator(StableLinearSystem(), tf=1.0, n_steps=5, verbose=False)
        self.assertEqual(sim.compile_backend, "numpy")

    @pytest.mark.optional
    @pytest.mark.jax
    @unittest.skipUnless(_have_jax(), "jax not installed")
    def test_compile_backend_auto_resolves_to_jax_when_available(self):
        sim = Simulator(
            StableLinearSystem(),
            tf=1.0,
            n_steps=5,
            compile_backend=COMPILE_BACKEND_AUTO,
            verbose=False,
        )
        self.assertEqual(sim.compile_backend, "jax")

    @pytest.mark.optional
    @pytest.mark.jax
    @unittest.skipUnless(_have_jax(), "jax not installed")
    def test_auto_selects_rk4_for_large_uniform_grid_with_jax(self):
        sim = Simulator(
            StableLinearSystem(),
            tf=1.0,
            n_steps=10_000,
            compile_backend="jax",
            verbose=False,
        )
        self.assertEqual(sim.solver_mode, "rk4_fixedsteps")

    @pytest.mark.optional
    @pytest.mark.jax
    @unittest.skipUnless(_have_jax(), "jax not installed")
    def test_jax_grid_below_threshold_stays_scipy(self):
        sim = Simulator(
            StableLinearSystem(),
            tf=1.0,
            n_steps=9999,
            compile_backend="jax",
            verbose=False,
        )
        self.assertEqual(sim.solver_mode, "scipy")

    @pytest.mark.optional
    @pytest.mark.jax
    @unittest.skipUnless(_have_jax(), "jax not installed")
    def test_discontinuous_jax_large_grid_stays_stiff(self):
        sim = Simulator(
            DiscontinuousLinearSystem(),
            tf=1.0,
            n_steps=10_000,
            compile_backend="jax",
            verbose=False,
        )
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
            Simulator(
                StableLinearSystem(),
                x0=np.array([[1.0]]),
                tf=1.0,
                n_steps=5,
                verbose=False,
            )

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
            "minilink.simulation.solvers.scipy_ivp.solve_ivp",
            return_value=failed_solution,
        ):
            with self.assertRaises(RuntimeError) as ctx:
                sim.solve()

        self.assertIn("integration failed", str(ctx.exception))
        # Failure path: debug lives on the SciPy backend (no try/finally on Simulator)
        sb = sim.solver_backend
        self.assertIs(sb.last_solve_ivp_solution, failed_solution)
        self.assertFalse(sb.last_debug["success"])
        self.assertEqual(sb.last_debug["message"], "integration failed")
        self.assertFalse(hasattr(sim, "last_traj"))

    def test_solve_populates_last_traj_and_last_debug(self):
        sim = Simulator(
            StableLinearSystem(), tf=0.2, n_steps=3, solver="euler", verbose=False
        )
        traj = sim.solve()

        self.assertIs(sim.last_traj, traj)
        self.assertEqual(sim.last_debug["solver"], "euler")
        self.assertEqual(traj.x.shape, (1, 3))
        self.assertEqual(traj.u.shape, (1, 3))

    def test_solve_forced_validates_shape(self):
        sim = Simulator(
            StableLinearSystem(), tf=0.2, n_steps=3, solver="euler", verbose=False
        )
        bad_u = np.zeros((3, 1))

        with self.assertRaises(ValueError):
            sim.solve_forced(bad_u)

    def test_solve_forced_accepts_callable_full_input(self):
        sim = Simulator(
            StableLinearSystem(), tf=0.2, n_steps=3, solver="euler", verbose=False
        )

        traj = sim.solve_forced(lambda t: 10.0 * t)

        np.testing.assert_allclose(traj.u, np.array([[0.0, 1.0, 2.0]]))

    def test_solve_forced_accepts_constant_vector(self):
        sim = Simulator(
            TwoPortLinearSystem(), tf=0.2, n_steps=3, solver="euler", verbose=False
        )

        traj = sim.solve_forced(np.array([3.0, 4.0]))

        np.testing.assert_allclose(
            traj.u,
            np.array(
                [
                    [3.0, 3.0, 3.0],
                    [4.0, 4.0, 4.0],
                ]
            ),
        )

    def test_solve_forced_accepts_scalar_on_one_named_port(self):
        sim = Simulator(
            TwoPortLinearSystem(), tf=0.2, n_steps=3, solver="euler", verbose=False
        )

        traj = sim.solve_forced(5.0, input_port_id="left")

        np.testing.assert_allclose(traj.u[0, :], np.array([5.0, 5.0, 5.0]))
        np.testing.assert_allclose(traj.u[1, :], np.array([2.0, 2.0, 2.0]))

    def test_solve_forced_supports_fixed_step_rk4(self):
        sim = Simulator(
            StableLinearSystem(),
            tf=0.2,
            n_steps=3,
            solver="rk4_fixedsteps",
            verbose=False,
        )
        u_traj = np.zeros((1, 3))

        traj = sim.solve_forced(u_traj)

        self.assertEqual(sim.last_debug["solver"], "rk4_fixedsteps")
        np.testing.assert_allclose(traj.u, u_traj)
        np.testing.assert_allclose(traj.x[:, 0], [1.0])
        self.assertLess(traj.x[0, -1], 1.0)

    def test_wrapper_compute_trajectory_uses_new_simulator_path(self):
        sys = StableLinearSystem()
        traj = sys.compute_trajectory(tf=0.2, n_steps=3, solver="euler", show=False)

        self.assertIs(sys.traj, traj)
        self.assertEqual(traj.x.shape, (1, 3))
        self.assertEqual(traj.u.shape, (1, 3))
        self.assertEqual(traj.t.shape, (3,))

    def test_wrapper_compute_forced_accepts_full_input_trajectory(self):
        sys = StableLinearSystem()
        u_traj = np.array([[0.0, 0.5, 1.0]])

        traj = sys.compute_forced(
            u_traj,
            tf=0.2,
            n_steps=3,
            solver="euler",
            show=False,
            verbose=False,
        )

        self.assertIs(sys.traj, traj)
        np.testing.assert_allclose(traj.u, u_traj)

    def test_wrapper_compute_forced_samples_callable_on_one_named_port(self):
        sys = TwoPortLinearSystem()

        traj = sys.compute_forced(
            lambda t: 10.0 * t,
            input_port_id="left",
            tf=0.2,
            n_steps=3,
            solver="euler",
            show=False,
            verbose=False,
        )

        np.testing.assert_allclose(traj.u[0, :], np.array([0.0, 1.0, 2.0]))
        np.testing.assert_allclose(traj.u[1, :], np.array([2.0, 2.0, 2.0]))


if __name__ == "__main__":
    unittest.main()
