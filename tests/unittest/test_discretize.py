"""Tests for continuous-to-step discretization helpers."""

import unittest

import numpy as np

from minilink.analysis.discretize import discretize
from minilink.control.output import ProportionalController
from minilink.dynamics.catalog.equations.integrators import DoubleIntegrator


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

    def test_discretize_h_delegates_to_source(self):
        plant = DoubleIntegrator()
        dt = 0.05
        step_leaf = discretize(plant, dt)

        x = np.array([0.2, -0.1])
        u = np.array([0.4])
        np.testing.assert_allclose(
            step_leaf.h(x, u, k=2),
            plant.h(x, u, t=2 * dt),
        )

    def test_discretize_rejects_static_system(self):
        with self.assertRaises(TypeError):
            discretize(ProportionalController(1.0), dt=0.01)


if __name__ == "__main__":
    unittest.main()
