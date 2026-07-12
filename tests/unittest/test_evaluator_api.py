"""Compiled evaluator public API contracts."""

import unittest

import numpy as np
import pytest

from minilink.blocks.basic import Integrator
from minilink.core.compile.compiler import compile
from minilink.core.compile.evaluators.evaluators import DynamicsEvaluator
from minilink.core.compile.evaluators.numpy_evaluators import NumpyStepEvaluator
from minilink.core.system import StepSystem

try:
    import jax.numpy as jnp

    _JAX_AVAILABLE = True
except ImportError:
    _JAX_AVAILABLE = False


class TestEvaluatorApi(unittest.TestCase):
    def test_dynamic_evaluator_has_no_h(self):
        ev = compile(Integrator())
        self.assertFalse(hasattr(ev, "h"))
        self.assertFalse(hasattr(ev, "h_p"))

    def test_static_evaluator_outputs_y(self):
        from minilink.blocks.routing import Gain

        gain = Gain(K=2.0, dim=1)
        ev = compile(gain)
        out = ev.outputs(np.array([]), np.array([3.0]), 0.0)
        self.assertIn("y", out)
        np.testing.assert_allclose(out["y"], [6.0])

    def test_no_get_f_jit_on_jax_dynamic(self):
        if not _JAX_AVAILABLE:
            self.skipTest("JAX not installed")
        ev = compile(Integrator(), backend="jax")
        self.assertFalse(hasattr(ev, "get_f_jit"))

    def test_step_evaluator_has_rollout_not_integrate(self):
        class IdentityStep(StepSystem):
            def __init__(self):
                super().__init__(n=1)

            def step(self, x, u, k=0, params=None):
                return x

        ev = compile(IdentityStep())
        self.assertIsInstance(ev, NumpyStepEvaluator)
        self.assertTrue(hasattr(ev, "rollout"))
        self.assertFalse(hasattr(ev, "f"))
        self.assertFalse(hasattr(ev, "integrate"))
        self.assertFalse(hasattr(ev, "h"))
        self.assertFalse(isinstance(ev, DynamicsEvaluator))


@pytest.mark.optional
@pytest.mark.jax
@unittest.skipUnless(_JAX_AVAILABLE, "JAX not installed")
class TestEvaluatorApiJax(unittest.TestCase):
    def test_f_matches_numpy(self):
        plant = Integrator()
        ev_np = compile(plant, backend="numpy")
        ev_jx = compile(plant, backend="jax")
        x_np, u_np = np.array([0.2]), np.array([1.0])
        x_j, u_j = jnp.array(x_np), jnp.array(u_np)
        np.testing.assert_allclose(
            np.asarray(ev_jx.f(x_j, u_j, 0.0)), ev_np.f(x_np, u_np, 0.0), atol=1e-5
        )
