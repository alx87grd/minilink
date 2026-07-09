"""Tests for continuous-to-step discretization helpers."""

import unittest

import numpy as np

from minilink.analysis.discretize import discretize, sample_static
from minilink.control.modelbased import SlidingModeController
from minilink.control.output import ProportionalController
from minilink.dynamics.catalog.equations.integrators import DoubleIntegrator
from minilink.dynamics.catalog.pendulum.pendulum import Pendulum


class TestDiscretize(unittest.TestCase):
    def test_discretize_rk4_matches_evaluator_step(self):
        plant = DoubleIntegrator()
        dt = 0.05
        step_leaf = discretize(plant, dt)
        evaluator = plant.compile(backend="numpy")

        x0 = np.array([0.2, -0.1])
        u = np.array([0.4])
        x1_ref = evaluator.rk4_step(x0, u, 0.0, dt)
        x1 = step_leaf.step(x0, u, k=0)
        np.testing.assert_allclose(x1, x1_ref, rtol=1e-9, atol=1e-9)

    def test_sample_static_delegates_output(self):
        plant = Pendulum()
        ctl = SlidingModeController(plant, lam=1.5, gain=2.0, nab=0.05)
        sampled = sample_static(ctl, dt=0.02)

        self.assertEqual(sampled.n, 0)
        self.assertEqual(sampled.params["dt"], 0.02)
        self.assertIn("r", sampled.inputs)
        self.assertIn("y", sampled.inputs)

        q = np.array([0.1])
        dq = np.array([0.0])
        r = np.array([0.0, 0.0])
        boundary = np.concatenate([r, q, dq])
        np.testing.assert_allclose(
            sampled.outputs["u"].compute(None, boundary, 0.0),
            ctl.ctl(None, boundary),
        )

    def test_sample_static_rejects_dynamic_system(self):
        with self.assertRaises(ValueError):
            sample_static(DoubleIntegrator(), dt=0.01)

    def test_discretize_rejects_static_system(self):
        with self.assertRaises(TypeError):
            discretize(ProportionalController(1.0), dt=0.01)


if __name__ == "__main__":
    unittest.main()
