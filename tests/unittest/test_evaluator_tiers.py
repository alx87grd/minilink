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

    def test_numpy_rk4_step_trace_raises(self):
        ev = compile(Integrator(), backend="numpy")
        with self.assertRaises(AttributeError) as ctx:
            _ = ev.rk4_step_trace
        self.assertIn(TRACE_TIER_MSG, str(ctx.exception))


@pytest.mark.optional
@pytest.mark.jax
@unittest.skipUnless(_JAX_AVAILABLE, "JAX not installed")
class TestIntegrationTiersJax(unittest.TestCase):
    def setUp(self):
        self.ev = compile(Integrator(), backend="jax")
        self.x = jnp.array([0.2])
        self.u = jnp.array([1.0])
        self.t = 0.0
        self.dt = 0.01
        self.params = {"k": 1.0}

    def test_rk4_step_trace_parity(self):
        np.testing.assert_allclose(
            np.asarray(self.ev.rk4_step_trace(self.x, self.u, self.t, self.dt)),
            np.asarray(self.ev.rk4_step(self.x, self.u, self.t, self.dt)),
            atol=1e-6,
        )

    def test_integrate_trace_parity(self):
        u_seq = jnp.array([[1.0], [0.5], [0.0]])
        np.testing.assert_allclose(
            np.asarray(self.ev.integrate_trace(self.x, u_seq, self.t, self.dt)),
            np.asarray(self.ev.integrate(self.x, u_seq, self.t, self.dt)),
            atol=1e-6,
        )

    def test_rk4_integrate_forced_p_fast_tier(self):
        u_knots = jnp.array([[1.0], [0.5], [0.0]])
        out = self.ev.rk4_integrate_forced_p(
            self.x, u_knots, self.t, self.dt, self.params
        )
        self.assertEqual(out.shape, (3, 1))


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
