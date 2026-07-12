"""StepRollout container, gather_u, and evaluator rollout tests."""

import unittest

import numpy as np
import pytest

from minilink.core.compile.compiler import compile
from minilink.core.compile.evaluators.step_rollout import gather_u
from minilink.core.compile.execution_plan import (
    EXTERNAL_INPUT,
    INTERNAL_SIGNAL,
    NOMINAL,
)
from minilink.core.step_rollout import StepRollout
from minilink.core.system import StepSystem
from minilink.core.trajectory import Trajectory

try:
    import jax  # noqa: F401

    _JAX_AVAILABLE = True
except ImportError:
    _JAX_AVAILABLE = False


class AffineStep(StepSystem):
    def __init__(self):
        super().__init__(n=1, input_dim=1, output_dim=1)
        self.x0 = np.array([0.0])

    def step(self, x, u, k=0, params=None):
        return np.array([x[0] + u[0] + float(k)])

    def h(self, x, u, k=0, params=None):
        return np.array([x[0]])


class TestGatherU(unittest.TestCase):
    def test_nominal_source(self):
        sources = ((NOMINAL, np.array([1.0, 2.0]), 2),)
        local_u = gather_u(sources, 2, np.zeros(0), np.zeros(0))
        np.testing.assert_allclose(local_u, [1.0, 2.0])

    def test_external_input_source(self):
        sources = ((EXTERNAL_INPUT, slice(1, 2), 1), (EXTERNAL_INPUT, slice(0, 2), 2))
        boundary_u = np.array([3.0, 4.0, 5.0])
        local_u = gather_u(sources, 3, np.zeros(0), boundary_u)
        np.testing.assert_allclose(local_u, [4.0, 3.0, 4.0])

    def test_internal_signal_source(self):
        sources = ((INTERNAL_SIGNAL, slice(2, 3), 1), (INTERNAL_SIGNAL, slice(0, 2), 2))
        signals = np.array([9.0, 8.0, 7.0])
        local_u = gather_u(sources, 3, signals, np.zeros(0))
        np.testing.assert_allclose(local_u, [7.0, 9.0, 8.0])

    def test_mixed_sources(self):
        sources = (
            (NOMINAL, np.array([0.5]), 1),
            (EXTERNAL_INPUT, slice(0, 1), 1),
            (INTERNAL_SIGNAL, slice(1, 2), 1),
        )
        signals = np.array([1.0, 2.0])
        boundary_u = np.array([3.0])
        local_u = gather_u(sources, 3, signals, boundary_u)
        np.testing.assert_allclose(local_u, [0.5, 3.0, 2.0])

    def test_zero_dim_returns_empty(self):
        local_u = gather_u((), 0, np.zeros(0), np.zeros(0))
        self.assertEqual(local_u.shape, (0,))

    def test_unknown_source_raises(self):
        with self.assertRaises(RuntimeError):
            gather_u(((99, 0, 1),), 1, np.zeros(1), np.zeros(1))


class TestStepRollout(unittest.TestCase):
    def test_shapes(self):
        rollout = StepRollout(
            k=np.arange(4, dtype=float),
            x=np.zeros((1, 4)),
            u=np.zeros((1, 4)),
        )
        self.assertEqual(rollout.n_samples, 4)
        self.assertEqual(rollout.n, 1)
        self.assertEqual(rollout.m, 1)

    def test_as_trajectory(self):
        rollout = StepRollout(
            k=np.array([0.0, 1.0, 2.0]),
            x=np.array([[0.0, 1.0, 3.0]]),
            u=np.array([[0.0, 1.0, 1.0]]),
        )
        traj = rollout.as_trajectory()
        self.assertIsInstance(traj, Trajectory)
        np.testing.assert_allclose(traj.t, [0.0, 1.0, 2.0])

    def test_rollout_constant_u(self):
        plant = AffineStep()
        ev = compile(plant)
        rollout = ev.rollout(plant.x0, n_steps=3, u=np.array([1.0]))
        self.assertEqual(rollout.x.shape, (1, 4))
        self.assertEqual(rollout.u.shape, (1, 4))
        np.testing.assert_allclose(rollout.x[0], [0.0, 1.0, 3.0, 6.0])

    def test_rollout_sequence_u(self):
        plant = AffineStep()
        ev = compile(plant)
        u_seq = np.array([[1.0], [0.0], [2.0]])
        rollout = ev.rollout(plant.x0, n_steps=3, u=u_seq)
        np.testing.assert_allclose(rollout.u[0, :3], [1.0, 0.0, 2.0])
        np.testing.assert_allclose(rollout.u[0, 3], 2.0)

    def test_rollout_callable_u(self):
        plant = AffineStep()
        ev = compile(plant)

        def u_of_k(k):
            return np.array([float(k)])

        rollout = ev.rollout(plant.x0, n_steps=2, u=u_of_k)
        np.testing.assert_allclose(rollout.u[0, :2], [0.0, 1.0])

    def test_rollout_state_only_by_default(self):
        plant = AffineStep()
        ev = compile(plant)
        rollout = ev.rollout(plant.x0, n_steps=2, u=np.array([0.0]))
        self.assertEqual(len(rollout.signals), 0)

    def test_record_boundary_outputs(self):
        from minilink.simulation.step_recording import record_boundary_outputs

        plant = AffineStep()
        ev = compile(plant)
        rollout = ev.rollout(plant.x0, n_steps=2, u=np.array([0.0]))
        logged = record_boundary_outputs(rollout, ev)
        self.assertIn("y", logged.signals)
        np.testing.assert_allclose(logged.signals["y"][0], rollout.x[0])


@pytest.mark.optional
@pytest.mark.jax
@unittest.skipUnless(_JAX_AVAILABLE, "JAX not installed")
class TestStepRolloutJax(unittest.TestCase):
    def test_jax_rollout_scan_smoke(self):
        from minilink.core.backends import array_module

        class JaxFriendlyStep(StepSystem):
            def __init__(self):
                super().__init__(n=1, input_dim=1)
                self.x0 = np.array([0.0])

            def step(self, x, u, k=0, params=None):
                xp = array_module(x)
                x = xp.asarray(x, dtype=float).reshape(1)
                u = xp.asarray(u, dtype=float).reshape(1)
                return xp.array([x[0] + u[0]])

        plant = JaxFriendlyStep()
        ev = compile(plant, backend="jax")
        rollout = ev.rollout(plant.x0, n_steps=4, u=np.array([1.0]))
        ev_np = compile(plant, backend="numpy")
        ref = ev_np.rollout(plant.x0, n_steps=4, u=np.array([1.0]))
        np.testing.assert_allclose(rollout.x, ref.x, atol=1e-5)


if __name__ == "__main__":
    unittest.main()
