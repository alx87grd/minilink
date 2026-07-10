"""Step rollout facade tests."""

import unittest

import numpy as np

from minilink.blocks.basic import Integrator
from minilink.core.system import StepSystem


class Counter(StepSystem):
    def __init__(self):
        super().__init__(n=1)
        self.x0 = np.zeros(1)

    def step(self, x, u, k=0, params=None):
        return np.array([x[0] + 1.0])


class TestFacadesRollout(unittest.TestCase):
    def test_compute_rollout_caches_on_system(self):
        plant = Counter()
        rollout = plant.compute_rollout(n_steps=5)
        self.assertIs(plant.rollout, rollout)
        self.assertEqual(rollout.n_samples, 6)
        np.testing.assert_allclose(rollout.x[0, -1], 5.0)

    def test_plot_rollout_headless(self):
        import matplotlib

        matplotlib.use("Agg")
        plant = Counter()
        rollout = plant.compute_rollout(n_steps=3)
        result = plant.plot_rollout(rollout, show=False)
        self.assertEqual(result.backend, "matplotlib")
        fig, axes = result.payload
        self.assertEqual(axes[-1].get_xlabel(), "Step [k]")

    def test_compute_rollout_only_on_step_system(self):
        with self.assertRaises(AttributeError):
            Integrator().compute_rollout(n_steps=3)


if __name__ == "__main__":
    unittest.main()
