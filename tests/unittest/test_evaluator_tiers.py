"""Evaluator fast vs trace tier contracts."""

import unittest

import numpy as np
import pytest

from minilink.blocks.basic import Integrator
from minilink.core.compile.compiler import compile
from minilink.core.compile.evaluators.tiers import TRACE_TIER_MSG
from minilink.core.system import StepSystem

try:
    import jax
    import jax.numpy as jnp

    _JAX_AVAILABLE = True
except ImportError:
    _JAX_AVAILABLE = False


@pytest.mark.optional
@pytest.mark.jax
@unittest.skipUnless(_JAX_AVAILABLE, "JAX not installed")
class TestEvaluatorTiersJax(unittest.TestCase):
    def setUp(self):
        self.plant = Integrator()
        self.ev = compile(self.plant, backend="jax")
        self.x = jnp.array([0.2])
        self.u = jnp.array([1.0])
        self.t = 0.0

    def test_f_trace_parity_with_f(self):
        np.testing.assert_allclose(
            np.asarray(self.ev.f_trace(self.x, self.u, self.t)),
            np.asarray(self.ev.f(self.x, self.u, self.t)),
            atol=1e-6,
        )

    def test_f_jit_alias_identity(self):
        from minilink.core.compile.evaluators.jax_evaluator import JaxDynamicEvaluator

        self.assertIs(JaxDynamicEvaluator.f_jit, JaxDynamicEvaluator.f)
        self.assertIs(JaxDynamicEvaluator.f_jit_p, JaxDynamicEvaluator.f_p)

    def test_has_trace_tier(self):
        self.assertTrue(self.ev.has_trace_tier)

    def test_f_trace_p_with_frozen_params(self):
        frozen = self.ev._frozen_params
        np.testing.assert_allclose(
            np.asarray(self.ev.f_trace_p(self.x, self.u, self.t, frozen)),
            np.asarray(self.ev.f(self.x, self.u, self.t)),
            atol=1e-6,
        )

    def test_trace_tier_composable_in_outer_jit(self):
        x, u, t = self.x, self.u, self.t

        @jax.jit
        def loss(theta):
            p = {"k": theta[0]}
            return jnp.sum((self.ev.f_trace_p(x, u, t, p) - jnp.array([1.0])) ** 2)

        grad = jax.jit(jax.grad(loss))(jnp.array([1.0]))
        self.assertEqual(grad.shape, (1,))


class TestEvaluatorTiersNumPy(unittest.TestCase):
    def test_numpy_f_trace_raises(self):
        ev = compile(Integrator(), backend="numpy")
        with self.assertRaises(AttributeError) as ctx:
            _ = ev.f_trace
        self.assertIn(TRACE_TIER_MSG, str(ctx.exception))


@pytest.mark.optional
@pytest.mark.jax
@unittest.skipUnless(_JAX_AVAILABLE, "JAX not installed")
class TestStepEvaluatorTiersJax(unittest.TestCase):
    def test_step_trace_parity(self):
        class IdentityStep(StepSystem):
            def __init__(self):
                super().__init__(n=1)

            def step(self, x, u, k=0, params=None):
                return x

        ev = compile(IdentityStep(), backend="jax")
        x = jnp.array([1.0])
        u = jnp.array([0.0])
        from minilink.core.compile.evaluators.step_evaluator import JaxStepEvaluator

        np.testing.assert_allclose(
            np.asarray(ev.step_trace(x, u, 0)),
            np.asarray(ev.step(x, u, 0)),
            atol=1e-6,
        )
        self.assertIs(JaxStepEvaluator.step_jit, JaxStepEvaluator.step)
