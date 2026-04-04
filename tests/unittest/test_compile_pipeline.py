"""Tests for the new compilation pipeline.

Validates that:
- ``compile_diagram()`` produces a working ``NumpyEvaluator``
- ``compute_dx()`` matches ``diagram.f()`` (the slow recursive reference)
- ``compute_outputs()`` returns correct port signals
- ``compute_internal_signals()`` exposes the full signal buffer
- ``check_algebraic_loops()`` detects loops and returns valid order
- ``build_execution_plan()`` produces a valid ``ExecutionPlan``
"""

import unittest

import numpy as np

from minilink.blocks.basic import Integrator, PropController
from minilink.compile import (
    ExecutionPlan,
    NumpyEvaluator,
    build_execution_plan,
    check_algebraic_loops,
    compile_diagram,
)
from minilink.core.diagram import DiagramSystem
from minilink.core.framework import StaticSystem, System


# ── Helper: reusable test diagrams ───────────────────────────────────


def _build_small_closed_loop():
    """Step → PropController → Integrator → feedback to controller."""
    diag = DiagramSystem()
    diag.graphe_building_verbose = False

    ctl = PropController()
    plant = Integrator()
    ctl.params["Kp"] = 2.5

    diag.add_subsystem(ctl, "ctl")
    diag.add_subsystem(plant, "plant")
    diag.add_input_port(1, "ref")

    diag.connect("input", "ref", "ctl", "ref")
    diag.connect("plant", "y", "ctl", "y")
    diag.connect("ctl", "u", "plant", "u")
    return diag


def _build_feedthrough_loop():
    """Three feedthrough blocks wired in a cycle (algebraic loop)."""

    class FeedthroughSystem(System):
        def __init__(self, name):
            super().__init__(0, 1, 1)
            self.name = name
            self.inputs = {}
            self.add_input_port(1, "u")
            self.outputs = {}
            self.add_output_port(1, "y", function=self.h, dependencies=["u"])

        def h(self, x, u, t=0, params=None):
            return u * 2

    diag = DiagramSystem()
    diag.graphe_building_verbose = False
    diag.add_subsystem(FeedthroughSystem("A"), "A")
    diag.add_subsystem(FeedthroughSystem("B"), "B")
    diag.add_subsystem(FeedthroughSystem("C"), "C")

    diag.connect("A", "y", "B", "u")
    diag.connect("B", "y", "C", "u")
    diag.connect("C", "y", "A", "u")
    return diag


# ── Tests ────────────────────────────────────────────────────────────


class TestCheckAlgebraicLoops(unittest.TestCase):
    """Test standalone algebraic loop detection."""

    def test_no_loop_returns_order(self):
        diag = _build_small_closed_loop()
        order = check_algebraic_loops(diag)
        self.assertIsInstance(order, list)
        self.assertGreaterEqual(len(order), 1)
        # All entries are (sys_id, port_id) tuples
        for sys_id, port_id in order:
            self.assertIn(sys_id, diag.subsystems)

    def test_loop_raises(self):
        diag = _build_feedthrough_loop()
        with self.assertRaises(RuntimeError) as ctx:
            check_algebraic_loops(diag)
        self.assertIn("Algebraic loop detected", str(ctx.exception))

    def test_chain_no_loop(self):
        """A → B → C (no loop) should succeed."""
        diag = _build_feedthrough_loop()
        # Remove the closing edge C → A to break the loop
        diag.connections["A"]["u"] = None
        order = check_algebraic_loops(diag)
        self.assertGreaterEqual(len(order), 3)


class TestBuildExecutionPlan(unittest.TestCase):
    """Test execution plan construction."""

    def test_returns_execution_plan(self):
        diag = _build_small_closed_loop()
        plan = build_execution_plan(diag)
        self.assertIsInstance(plan, ExecutionPlan)

    def test_plan_dimensions(self):
        diag = _build_small_closed_loop()
        plan = build_execution_plan(diag)
        self.assertEqual(plan.state_dim, diag.n)
        self.assertGreater(plan.signal_dim, 0)
        self.assertGreaterEqual(len(plan.port_ops), 1)
        self.assertGreaterEqual(len(plan.state_ops), 1)

    def test_output_slices_keys(self):
        diag = _build_small_closed_loop()
        plan = build_execution_plan(diag)
        # Every subsystem output port should have a slice
        for sys_id, sys in diag.subsystems.items():
            for port_id in sys.outputs:
                self.assertIn((sys_id, port_id), plan.output_slices)


class TestCompileDiagram(unittest.TestCase):
    """Test the top-level compile_diagram entry point."""

    def test_returns_numpy_evaluator(self):
        diag = _build_small_closed_loop()
        evaluator = compile_diagram(diag, backend="numpy")
        self.assertIsInstance(evaluator, NumpyEvaluator)

    def test_invalid_backend_raises(self):
        diag = _build_small_closed_loop()
        with self.assertRaises(ValueError):
            compile_diagram(diag, backend="unknown")


class TestNumpyEvaluator(unittest.TestCase):
    """Test NumpyEvaluator against the reference diagram.f()."""

    def setUp(self):
        self.diag = _build_small_closed_loop()
        self.evaluator = compile_diagram(self.diag)

    def test_compute_dx_matches_diagram_f(self):
        x = np.array([0.3])
        u = np.array([1.2])
        t = 0.1

        dx_reference = self.diag.f(x, u, t)
        dx_compiled = self.evaluator.compute_dx(x, u, t)

        np.testing.assert_allclose(dx_compiled, dx_reference, atol=1e-10)

    def test_compute_dx_multiple_points(self):
        """Test at several state/input combinations."""
        for x_val, u_val in [(0.0, 0.0), (1.0, 2.0), (-5.0, 10.0)]:
            x = np.array([x_val])
            u = np.array([u_val])
            dx_ref = self.diag.f(x, u, 0.0)
            dx_comp = self.evaluator.compute_dx(x, u, 0.0)
            np.testing.assert_allclose(dx_comp, dx_ref, atol=1e-10)

    def test_compute_outputs_all(self):
        x = np.array([0.5])
        u = np.array([1.0])
        y = self.evaluator.compute_outputs(x, u, 0.0)
        # Should return signals from all output ports
        self.assertGreater(y.size, 0)

    def test_compute_outputs_specific_port(self):
        x = np.array([0.5])
        u = np.array([1.0])
        y = self.evaluator.compute_outputs(x, u, 0.0, ports=[("plant", "y")])
        # Plant output y = x for Integrator
        np.testing.assert_allclose(y, np.array([0.5]), atol=1e-10)

    def test_compute_internal_signals(self):
        x = np.array([0.5])
        u = np.array([1.0])
        signals = self.evaluator.compute_internal_signals(x, u, 0.0)
        self.assertEqual(signals.shape[0], self.evaluator.plan.signal_dim)

    def test_compute_dx_zero_state(self):
        """Diagram with no dynamic subsystems should return empty dx."""
        diag = DiagramSystem()
        diag.graphe_building_verbose = False

        class Gain(StaticSystem):
            def __init__(self):
                super().__init__(1, 1)
                self.add_output_port(1, "y", function=self.h, dependencies=["u"])

            def h(self, x, u, t=0, params=None):
                return u * 2.0

        diag.add_subsystem(Gain(), "gain")
        diag.add_input_port(1, "u")
        diag.connect("input", "u", "gain", "u")

        evaluator = compile_diagram(diag)
        dx = evaluator.compute_dx(np.array([]), np.array([3.0]), 0.0)
        self.assertEqual(dx.shape, (0,))

    def test_compute_internal_signals_dict(self):
        """compute_internal_signals_dict returns {sys_id:port_id -> array}."""
        x = np.array([0.5])
        u = np.array([1.0])
        signals = self.evaluator.compute_internal_signals_dict(x, u, 0.0)
        self.assertIsInstance(signals, dict)
        # All subsystem output ports should be present
        for sys_id, sys in self.diag.subsystems.items():
            for port_id in sys.outputs:
                self.assertIn(f"{sys_id}:{port_id}", signals)

    def test_bind_params_freezes_snapshot(self):
        """bound_params in plan: mutating subsystem.params does not change dx."""
        diag = _build_small_closed_loop()
        plant = diag.subsystems["plant"]
        x = np.array([0.3])
        u = np.array([1.0])
        t = 0.0

        ev = compile_diagram(diag, bind_params=True)
        dx1 = ev.compute_dx(x, u, t)
        plant.params["k"] = 99.0
        dx2 = ev.compute_dx(x, u, t)
        np.testing.assert_allclose(dx1, dx2, atol=1e-10)

    def test_bind_params_false_follows_live_params(self):
        """Default compile: dx changes when subsystem.params changes (no recompile)."""
        diag = _build_small_closed_loop()
        plant = diag.subsystems["plant"]
        x = np.array([0.3])
        u = np.array([1.0])
        t = 0.0

        ev = compile_diagram(diag, bind_params=False)
        dx1 = ev.compute_dx(x, u, t)
        plant.params["k"] = 99.0
        dx2 = ev.compute_dx(x, u, t)
        self.assertGreater(np.abs(dx2 - dx1).max(), 1e-6)

    def test_as_dx_callable_matches_compute_dx(self):
        fn = self.evaluator.as_dx_callable()
        x = np.array([0.2])
        u = np.array([0.7])
        t = 0.05
        np.testing.assert_allclose(
            fn(x, u, t), self.evaluator.compute_dx(x, u, t), atol=1e-10
        )

    def test_as_scipy_ivp_fun_order(self):
        x = np.array([0.4])
        u = np.array([1.0])
        t = 0.15
        rhs = self.evaluator.as_scipy_ivp_fun(u=u)
        np.testing.assert_allclose(
            rhs(t, x), self.evaluator.compute_dx(x, u, t), atol=1e-10
        )

    def test_build_execution_plan_bind_params_sets_bound_params(self):
        diag = _build_small_closed_loop()
        plan_free = build_execution_plan(diag, bind_params=False)
        plan_bound = build_execution_plan(diag, bind_params=True)
        self.assertIsNone(plan_free.port_ops[0].bound_params)
        self.assertIsNotNone(plan_bound.port_ops[0].bound_params)


# TODO: add JAX tests with compatible JAX block in a separate test file
# # ── JAX tests (skipped if JAX not installed) ─────────────────────────

# try:
#     import jax
#     import jax.numpy as jnp
#     from minilink.compile import JaxEvaluator
#     _JAX_AVAILABLE = True
# except ImportError:
#     _JAX_AVAILABLE = False


# @unittest.skipUnless(_JAX_AVAILABLE, "JAX not installed")
# class TestJaxEvaluator(unittest.TestCase):
#     """Test JaxEvaluator against the reference diagram.f() and for JAX traceability."""

#     def setUp(self):
#         self.diag = _build_small_closed_loop()
#         self.evaluator = compile_diagram(self.diag, backend="jax")
#         self.x = jnp.array([0.3], dtype=jnp.float32)
#         self.u = jnp.array([1.2], dtype=jnp.float32)

#     def test_returns_jax_evaluator(self):
#         self.assertIsInstance(self.evaluator, JaxEvaluator)

#     def test_compute_dx_matches_numpy(self):
#         """JaxEvaluator.compute_dx must match NumpyEvaluator exactly."""
#         np_evaluator = compile_diagram(self.diag, backend="numpy")
#         x_np = np.array([0.3])
#         u_np = np.array([1.2])

#         dx_numpy = np_evaluator.compute_dx(x_np, u_np, 0.1)
#         dx_jax = np.array(self.evaluator.compute_dx(self.x, self.u, 0.1))

#         np.testing.assert_allclose(dx_jax, dx_numpy, atol=1e-5)

#     def test_compute_dx_multiple_points(self):
#         """Test at several state/input combinations."""
#         np_evaluator = compile_diagram(self.diag, backend="numpy")
#         for x_val, u_val in [(0.0, 0.0), (1.0, 2.0), (-5.0, 10.0)]:
#             x_np = np.array([x_val])
#             u_np = np.array([u_val])
#             x_jax = jnp.array(x_np, dtype=jnp.float32)
#             u_jax = jnp.array(u_np, dtype=jnp.float32)
#             dx_ref = np_evaluator.compute_dx(x_np, u_np, 0.0)
#             dx_jax = np.array(self.evaluator.compute_dx(x_jax, u_jax, 0.0))
#             np.testing.assert_allclose(dx_jax, dx_ref, atol=1e-5)

#     def test_compute_dx_is_differentiable(self):
#         """jax.grad must flow through compute_dx."""
#         def dx0_of_u(u0):
#             return self.evaluator.compute_dx(
#                 self.x, jnp.array([u0], dtype=jnp.float32), 0.0
#             )[0]

#         # For an integrator dx/dt = u, so d(dx)/du = 1
#         grad_val = float(jax.grad(dx0_of_u)(jnp.array(1.2, dtype=jnp.float32)))
#         self.assertAlmostEqual(abs(grad_val), 1.0, places=4)

#     def test_compute_outputs_specific_port(self):
#         """JaxEvaluator.compute_outputs with port selection."""
#         y = self.evaluator.compute_outputs(
#             self.x, self.u, 0.0, ports=[("plant", "y")]
#         )
#         # Plant output y = x for Integrator
#         self.assertAlmostEqual(float(y[0]), 0.3, places=5)

#     def test_compute_outputs_is_differentiable(self):
#         """jax.grad must flow through compute_outputs."""
#         def y_of_x(x0):
#             return self.evaluator.compute_outputs(
#                 jnp.array([x0], dtype=jnp.float32),
#                 self.u, 0.0, ports=[("plant", "y")]
#             )[0]

#         grad_val = float(jax.grad(y_of_x)(jnp.array(0.3, dtype=jnp.float32)))
#         self.assertAlmostEqual(abs(grad_val), 1.0, places=4)

#     def test_jit_compute_dx(self):
#         """get_jit_compute_dx returns a callable that matches eager result."""
#         jit_dx = self.evaluator.get_jit_compute_dx()
#         dx_eager = self.evaluator.compute_dx(self.x, self.u, 0.0)
#         dx_jit = jit_dx(self.x, self.u, 0.0)
#         np.testing.assert_allclose(
#             np.array(dx_jit), np.array(dx_eager), atol=1e-5
#         )

#     def test_jit_compute_outputs(self):
#         """get_jit_compute_outputs returns a callable that matches eager result."""
#         jit_out = self.evaluator.get_jit_compute_outputs()
#         y_eager = self.evaluator.compute_outputs(self.x, self.u, 0.0)
#         y_jit = jit_out(self.x, self.u, 0.0)
#         np.testing.assert_allclose(
#             np.array(y_jit), np.array(y_eager), atol=1e-5
#         )

#     def test_compute_internal_signals_dict(self):
#         """compute_internal_signals_dict returns {sys_id:port_id -> jax array}."""
#         signals = self.evaluator.compute_internal_signals_dict(self.x, self.u, 0.0)
#         self.assertIsInstance(signals, dict)
#         for sys_id, sys in self.diag.subsystems.items():
#             for port_id in sys.outputs:
#                 self.assertIn(f"{sys_id}:{port_id}", signals)


if __name__ == "__main__":
    unittest.main()
