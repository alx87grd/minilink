"""Tests for continuous-to-step discretization helpers."""

import unittest

import numpy as np

from minilink.analysis.discretize import discretize
from minilink.control.output import ProportionalController
from minilink.core.system import DynamicSystem
from minilink.dynamics.catalog.equations.integrators import DoubleIntegrator


def _rk4_step(f, x, u, t, dt, params):
    k1 = f(x, u, t, params)
    k2 = f(x + 0.5 * dt * k1, u, t + 0.5 * dt, params)
    k3 = f(x + 0.5 * dt * k2, u, t + 0.5 * dt, params)
    k4 = f(x + dt * k3, u, t + dt, params)
    return x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)


class _GainIntegrator(DynamicSystem):
    def __init__(self, gain=1.0):
        super().__init__(n=1, input_dim=1, output_dim=1, expose_state=True)
        self.params = {"gain": float(gain), "dt": 0.05}

    def f(self, x, u, t=0.0, params=None):
        p = self.params if params is None else params
        gain = p["gain"]
        return np.array([gain * u[0]])

    def h(self, x, u, t=0.0, params=None):
        return np.array([x[0]])


class TestDiscretize(unittest.TestCase):
    def test_discretize_rk4_matches_source_step(self):
        plant = DoubleIntegrator()
        dt = 0.05
        step_leaf = discretize(plant, dt)
        p = step_leaf.params

        x0 = np.array([0.2, -0.1])
        u = np.array([0.4])
        x1_ref = _rk4_step(plant.f, x0, u, 0.0, dt, p)
        x1 = step_leaf.step(x0, u, k=0)
        np.testing.assert_allclose(x1, x1_ref, rtol=1e-9, atol=1e-9)
        self.assertEqual(step_leaf.params["dt"], dt)

    def test_discretize_h_delegates_to_source(self):
        plant = DoubleIntegrator()
        dt = 0.05
        step_leaf = discretize(plant, dt)
        p = step_leaf.params

        x = np.array([0.2, -0.1])
        u = np.array([0.4])
        np.testing.assert_allclose(
            step_leaf.h(x, u, k=2),
            plant.h(x, u, t=2 * dt, params=p),
        )

    def test_discretize_euler_matches_source_step(self):
        plant = DoubleIntegrator()
        dt = 0.05
        step_leaf = discretize(plant, dt, method="euler")
        p = step_leaf.params

        x0 = np.array([0.2, -0.1])
        u = np.array([0.4])
        x1_ref = x0 + dt * plant.f(x0, u, 0.0, p)
        x1 = step_leaf.step(x0, u, k=0)
        np.testing.assert_allclose(x1, x1_ref, rtol=1e-9, atol=1e-9)
        self.assertEqual(step_leaf.method, "euler")

    def test_discretize_accepts_dt_in_params_only(self):
        step_leaf = discretize(_GainIntegrator(), params={"dt": 0.02})
        self.assertEqual(step_leaf.params["dt"], 0.02)

    def test_step_params_override_gain(self):
        plant = _GainIntegrator(gain=1.0)
        step_leaf = discretize(plant, 0.1)
        u = np.array([1.0])
        x0 = np.array([0.0])

        x_nom = step_leaf.step(x0, u, k=0)
        p_fast = {**step_leaf.params, "gain": 3.0}
        x_fast = step_leaf.step(x0, u, k=0, params=p_fast)
        self.assertGreater(x_fast[0], x_nom[0])

    def test_step_params_override_dt(self):
        plant = _GainIntegrator(gain=1.0)
        step_leaf = discretize(plant, 0.1)
        u = np.array([1.0])
        x0 = np.array([0.0])
        p_short = {**step_leaf.params, "dt": 0.02}
        p_long = {**step_leaf.params, "dt": 0.2}
        x_short = step_leaf.step(x0, u, k=0, params=p_short)
        x_long = step_leaf.step(x0, u, k=0, params=p_long)
        self.assertLess(x_short[0], x_long[0])

    def test_discretize_rejects_unknown_method(self):
        plant = DoubleIntegrator()
        with self.assertRaises(ValueError):
            discretize(plant, 0.01, method="bdf")

    def test_discretize_rejects_missing_dt(self):
        plant = DoubleIntegrator()
        with self.assertRaises(ValueError):
            discretize(plant)

    def test_discretize_rejects_static_system(self):
        with self.assertRaises(TypeError):
            discretize(ProportionalController(1.0), dt=0.01)


if __name__ == "__main__":
    unittest.main()
