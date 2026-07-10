"""Typed compile() dispatch for static System leaves."""

import unittest

import numpy as np
import pytest

from minilink.blocks.routing import Gain
from minilink.core.compile.compiler import compile
from minilink.core.compile.evaluators.dynamics_evaluator import DynamicsEvaluator
from minilink.core.compile.evaluators.static_evaluator import NumpyStaticEvaluator

try:
    import jax  # noqa: F401

    from minilink.core.compile.evaluators.static_evaluator import JaxStaticEvaluator

    _JAX_AVAILABLE = True
except ImportError:
    JaxStaticEvaluator = None  # type: ignore[misc, assignment]
    _JAX_AVAILABLE = False


class TestCompileStatic(unittest.TestCase):
    def test_compile_gain_returns_numpy_static_evaluator(self):
        gain = Gain(K=3.0, dim=1)
        ev = compile(gain)
        self.assertIsInstance(ev, NumpyStaticEvaluator)
        self.assertFalse(isinstance(ev, DynamicsEvaluator))
        self.assertFalse(hasattr(ev, "f") or callable(getattr(ev, "f", None)))

    def test_static_outputs_match_block(self):
        gain = Gain(K=2.0, dim=1)
        ev = compile(gain)
        u = np.array([4.0])
        out = ev.outputs(np.array([]), u, 0.0)
        np.testing.assert_allclose(out["y"], [8.0])


@pytest.mark.optional
@pytest.mark.jax
@unittest.skipUnless(_JAX_AVAILABLE, "JAX not installed")
class TestCompileStaticJax(unittest.TestCase):
    def test_compile_gain_jax_parity(self):
        gain = Gain(K=2.5, dim=1)
        ev_np = compile(gain, backend="numpy")
        ev_jx = compile(gain, backend="jax")
        self.assertIsInstance(ev_jx, JaxStaticEvaluator)
        u = np.array([2.0])
        out_np = ev_np.outputs(np.array([]), u, 0.0)
        out_jx = ev_jx.outputs(np.array([]), u, 0.0)
        np.testing.assert_allclose(np.asarray(out_jx["y"]), out_np["y"], atol=1e-5)
