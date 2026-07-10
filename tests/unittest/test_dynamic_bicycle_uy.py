"""Tests for :class:`JaxDynamicBicycleRateInputsUY`."""

import unittest

import numpy as np
import pytest

pytest.importorskip("jax")

from minilink.core.backends import configure_jax  # noqa: E402
from minilink.dynamics.catalog.vehicles.dynamic_bicycle import (  # noqa: E402
    JaxDynamicBicycleRateInputs,
    JaxDynamicBicycleRateInputsUY,
)


class TestDynamicBicycleUY(unittest.TestCase):
    def setUp(self):
        configure_jax(enable_x64=True)
        self.named = JaxDynamicBicycleRateInputs()
        self.uy = JaxDynamicBicycleRateInputsUY()

    def test_standard_ports(self):
        self.assertIn("u", self.uy.inputs)
        self.assertEqual(self.uy.inputs["u"].dim, 2)
        self.assertIn("y", self.uy.outputs)
        self.assertEqual(self.uy.outputs["y"].dim, self.uy.n)

    def test_f_matches_named_rate_inputs(self):
        x = np.array([1.0, 2.0, 0.1, 3.0, 0.2, 0.05, 4.0, 0.1])
        u = np.array([0.5, -0.1])
        dx_named = np.asarray(self.named.f(x, u))
        dx_uy = np.asarray(self.uy.f(x, u))
        np.testing.assert_allclose(dx_named, dx_uy, rtol=1e-9, atol=1e-9)


if __name__ == "__main__":
    unittest.main()
