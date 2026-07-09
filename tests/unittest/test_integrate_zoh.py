"""Tests for :meth:`IntegrationMixin.integrate_zoh`."""

import unittest

import numpy as np

from minilink.blocks.basic import Integrator


class TestIntegrateZoh(unittest.TestCase):
    def test_single_step_matches_integrate(self):
        plant = Integrator()
        evaluator = plant.compile()

        x0 = np.array([0.0])
        u_hold = np.array([2.0])
        x_final = evaluator.integrate_zoh(x0, u_hold, t0=0.0, dt_hold=0.1)

        x_seq = evaluator.integrate(x0, u_hold.reshape(1, 1), t0=0.0, dt=0.1)
        np.testing.assert_allclose(x_final, x_seq[-1])

    def test_subdivided_dt_inner(self):
        plant = Integrator()
        evaluator = plant.compile()

        x0 = np.array([0.0])
        u_hold = np.array([1.0])
        dt_hold = 0.1
        dt_inner = 0.02

        x_final = evaluator.integrate_zoh(
            x0, u_hold, t0=0.0, dt_hold=dt_hold, dt_inner=dt_inner
        )
        n_sub = int(round(dt_hold / dt_inner))
        u_sequence = np.ones((n_sub, 1))
        dt_step = dt_hold / n_sub
        x_seq = evaluator.integrate(x0, u_sequence, t0=0.0, dt=dt_step)
        np.testing.assert_allclose(x_final, x_seq[-1], rtol=1e-10, atol=1e-10)

    def test_invalid_dt_raises(self):
        plant = Integrator()
        evaluator = plant.compile()
        with self.assertRaises(ValueError):
            evaluator.integrate_zoh(np.array([0.0]), np.array([1.0]), 0.0, 0.0)


if __name__ == "__main__":
    unittest.main()
