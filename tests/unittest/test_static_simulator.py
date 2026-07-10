"""StaticSimulator records boundary outputs on a time grid."""

import unittest

import numpy as np
import pytest

from minilink.blocks.routing import Gain
from minilink.blocks.sources import Step
from minilink.simulation.static_simulator import StaticSimulator

try:
    import jax  # noqa: F401

    _JAX_AVAILABLE = True
except ImportError:
    _JAX_AVAILABLE = False


class TestStaticSimulator(unittest.TestCase):
    def test_gain_compute_trajectory_shape(self):
        gain = Gain(K=2.0, dim=1)
        traj = gain.compute_trajectory(t0=0, tf=1, n_steps=5, verbose=False)
        self.assertEqual(traj.x.shape, (0, 5))
        self.assertEqual(traj.u.shape, (1, 5))
        self.assertIn("y", traj.signals)
        np.testing.assert_allclose(traj.signals["y"], np.zeros((1, 5)))

    def test_step_source_signals_on_grid(self):
        step = Step(
            initial_value=np.array([0.0]),
            final_value=np.array([1.0]),
            step_time=0.5,
        )
        traj = step.compute_trajectory(t0=0, tf=1, n_steps=3, verbose=False)
        self.assertEqual(traj.x.shape, (0, 3))
        y = traj.signals["y"]
        np.testing.assert_array_equal(y[:, 0], [0.0])
        np.testing.assert_array_equal(y[:, 1], [1.0])
        np.testing.assert_array_equal(y[:, 2], [1.0])

    def test_static_simulator_direct(self):
        gain = Gain(K=3.0, dim=1)
        sim = StaticSimulator(gain, t0=0, tf=0.2, n_steps=3)
        traj = sim.solve_forced(np.array([[1.0, 2.0, 3.0]]))
        np.testing.assert_allclose(traj.signals["y"], [[3.0, 6.0, 9.0]])


@pytest.mark.optional
@pytest.mark.jax
@unittest.skipUnless(_JAX_AVAILABLE, "JAX not installed")
class TestStaticSimulatorJax(unittest.TestCase):
    def test_gain_jax_compile_backend(self):
        gain = Gain(K=2.0, dim=1)
        traj = gain.compute_trajectory(
            t0=0, tf=0.5, n_steps=3, compile_backend="jax", verbose=False
        )
        self.assertEqual(traj.x.shape, (0, 3))
        self.assertIn("y", traj.signals)
