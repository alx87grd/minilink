"""StepRollout container and evaluator rollout tests."""

import unittest

import numpy as np
import pytest

from minilink.core.compile.compiler import compile
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

    def test_record_outputs(self):
        plant = AffineStep()
        ev = compile(plant)
        rollout = ev.rollout(plant.x0, n_steps=2, u=np.array([0.0]))
        self.assertIn("y", rollout.signals)
        np.testing.assert_allclose(rollout.signals["y"][0], rollout.x[0])


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
        rollout = ev.rollout(
            plant.x0, n_steps=4, u=np.array([1.0]), record_outputs=False
        )
        ev_np = compile(plant, backend="numpy")
        ref = ev_np.rollout(
            plant.x0, n_steps=4, u=np.array([1.0]), record_outputs=False
        )
        np.testing.assert_allclose(rollout.x, ref.x, atol=1e-5)


if __name__ == "__main__":
    unittest.main()
