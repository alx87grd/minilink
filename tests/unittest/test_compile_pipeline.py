"""Tests for the compilation pipeline.

Validates that:
- ``compile_diagram()`` produces a working ``NumpyDiagramEvaluator``
- ``f()`` matches ``diagram.f()`` (the slow recursive reference)
- ``compute_internal_signals()`` / ``compute_internal_signals_dict()`` expose the buffer
- ``check_algebraic_loops()`` detects loops and returns valid order
- ``build_execution_plan()`` produces a valid ``ExecutionPlan``
"""

import unittest

import numpy as np

from minilink.blocks.basic import Integrator, PropController
from minilink.blocks.sources import _array_module
from minilink.compile.compiler import (
    build_execution_plan,
    check_algebraic_loops,
    compile_diagram,
)
from minilink.compile.execution_plan import ExecutionPlan
from minilink.compile.numpy_evaluator import NumpyDiagramEvaluator
from minilink.core.diagram import DiagramSystem
from minilink.core.framework import DynamicSystem, StaticSystem, System


# ── Helper: reusable test diagrams ───────────────────────────────────


class _JAXFriendlyPropController(StaticSystem):
    """P controller using NumPy or JAX ops so ``compile(..., backend='jax')`` traces."""

    def __init__(self):
        super().__init__(2, 1)
        self.params = {"Kp": 10.0}
        self.name = "Controller"
        self.inputs = {}
        self.add_input_port(1, "ref", nominal_value=np.array([0.0]))
        self.add_input_port(1, "y", nominal_value=np.array([0.0]))
        self.outputs = {}
        self.add_output_port(1, "u", function=self.ctl, dependencies=["ref", "y"])

    def ctl(self, x, u, t=0, params=None):
        if params is None:
            params = self.params
        Kp = params["Kp"]
        xp = _array_module(u)
        r = u[0]
        y_ = u[1]
        return xp.array([Kp * (r - y_)])


class _JAXFriendlyIntegrator(DynamicSystem):
    """``dx = k u`` integrator; mirrors :class:`Integrator` with JAX-friendly arrays."""

    def __init__(self):
        super().__init__(1, 1, 1)
        self.params = {"k": 1.0}
        self.name = "Integrator"
        self.outputs = {}
        self.add_output_port(1, "y", function=self.h, dependencies=[])

    def f(self, x, u, t=0, params=None):
        if params is None:
            params = self.params
        k = params["k"]
        xp = _array_module(x)
        return xp.array([k * u[0]])

    def h(self, x, u, t=0, params=None):
        xp = _array_module(x)
        return xp.array([x[0]])


def _build_small_closed_loop_jax():
    """Same wiring as :func:`_build_small_closed_loop` with JAX-traceable blocks."""
    diag = DiagramSystem()
    diag.graphe_building_verbose = False

    ctl = _JAXFriendlyPropController()
    plant = _JAXFriendlyIntegrator()
    ctl.params["Kp"] = 2.5

    diag.add_subsystem(ctl, "ctl")
    diag.add_subsystem(plant, "plant")
    diag.add_input_port(1, "ref")

    diag.connect("input", "ref", "ctl", "ref")
    diag.connect("plant", "y", "ctl", "y")
    diag.connect("ctl", "u", "plant", "u")
    return diag


def _build_closed_loop_with_external_output_jax():
    diag = _build_small_closed_loop_jax()
    diag.connect_new_output_port("plant", "y", "y_meas")
    return diag


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


def _build_closed_loop_with_external_output():
    """Same as ``_build_small_closed_loop`` plus one diagram output port."""
    diag = _build_small_closed_loop()
    diag.connect_new_output_port("plant", "y", "y_meas")
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

    def test_external_output_slices_empty_without_boundary_outputs(self):
        diag = _build_small_closed_loop()
        plan = build_execution_plan(diag)
        self.assertEqual(plan.external_output_slices, {})

    def test_external_output_slices_with_connect_new_output_port(self):
        diag = _build_closed_loop_with_external_output()
        plan = build_execution_plan(diag)
        self.assertIn("y_meas", plan.external_output_slices)
        self.assertEqual(
            plan.external_output_slices["y_meas"],
            plan.output_slices[("plant", "y")],
        )


class TestCompileDiagram(unittest.TestCase):
    """Test the top-level compile_diagram entry point."""

    def test_returns_numpy_evaluator(self):
        diag = _build_small_closed_loop()
        evaluator = compile_diagram(diag, backend="numpy")
        self.assertIsInstance(evaluator, NumpyDiagramEvaluator)

    def test_invalid_backend_raises(self):
        diag = _build_small_closed_loop()
        with self.assertRaises(ValueError):
            compile_diagram(diag, backend="unknown")


class TestNumpyDiagramEvaluator(unittest.TestCase):
    """Test NumpyDiagramEvaluator against the reference diagram.f()."""

    def setUp(self):
        self.diag = _build_small_closed_loop()
        self.evaluator = compile_diagram(self.diag)

    def test_f_matches_diagram_f(self):
        x = np.array([0.3])
        u = np.array([1.2])
        t = 0.1

        dx_reference = self.diag.f(x, u, t)
        dx_compiled = self.evaluator.f(x, u, t)

        np.testing.assert_allclose(dx_compiled, dx_reference, atol=1e-10)

    def test_f_multiple_points(self):
        """Test at several state/input combinations."""
        for x_val, u_val in [(0.0, 0.0), (1.0, 2.0), (-5.0, 10.0)]:
            x = np.array([x_val])
            u = np.array([u_val])
            dx_ref = self.diag.f(x, u, 0.0)
            dx_comp = self.evaluator.f(x, u, 0.0)
            np.testing.assert_allclose(dx_comp, dx_ref, atol=1e-10)

    def test_compute_internal_signals_non_empty(self):
        x = np.array([0.5])
        u = np.array([1.0])
        buf = self.evaluator.compute_internal_signals(x, u, 0.0)
        self.assertGreater(buf.size, 0)

    def test_compute_internal_signals_dict_plant_y(self):
        x = np.array([0.5])
        u = np.array([1.0])
        d = self.evaluator.compute_internal_signals_dict(x, u, 0.0)
        # Plant output y = x for Integrator
        np.testing.assert_allclose(d["plant:y"], np.array([0.5]), atol=1e-10)

    def test_compute_internal_signals(self):
        x = np.array([0.5])
        u = np.array([1.0])
        signals = self.evaluator.compute_internal_signals(x, u, 0.0)
        self.assertEqual(signals.shape[0], self.evaluator.plan.signal_dim)

    def test_f_zero_state(self):
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
        dx = evaluator.f(np.array([]), np.array([3.0]), 0.0)
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

    def test_outputs_empty_without_external_boundary_ports(self):
        """outputs() is boundary-only; closed loop without connect_new_output_port is {}."""
        x = np.array([0.5])
        u = np.array([1.0])
        self.assertEqual(self.evaluator.outputs(x, u, 0.0), {})

    def test_outputs_external_matches_plant_y_measurement(self):
        """With connect_new_output_port, outputs() maps diagram port id → signal."""
        diag = _build_closed_loop_with_external_output()
        ev = compile_diagram(diag)
        x = np.array([0.5])
        u = np.array([1.0])
        y = ev.outputs(x, u, 0.0)
        self.assertIn("y_meas", y)
        np.testing.assert_allclose(y["y_meas"], np.array([0.5]), atol=1e-10)

    def test_h_matches_single_boundary_output(self):
        """h() returns the single external output when exactly one boundary port exists."""
        diag = _build_closed_loop_with_external_output()
        ev = compile_diagram(diag)
        x = np.array([0.5])
        u = np.array([1.0])
        np.testing.assert_allclose(ev.h(x, u, 0.0), np.array([0.5]), atol=1e-10)

    def test_h_raises_when_multiple_output_ports(self):
        """h() is only defined when the diagram exposes exactly one boundary output."""
        x = np.array([0.5])
        u = np.array([1.0])
        with self.assertRaises(NotImplementedError):
            self.evaluator.h(x, u, 0.0)

    def test_bind_params_freezes_snapshot(self):
        """bound_params in plan: mutating subsystem.params does not change dx."""
        diag = _build_small_closed_loop()
        plant = diag.subsystems["plant"]
        x = np.array([0.3])
        u = np.array([1.0])
        t = 0.0

        ev = compile_diagram(diag, bind_params=True)
        dx1 = ev.f(x, u, t)
        plant.params["k"] = 99.0
        dx2 = ev.f(x, u, t)
        np.testing.assert_allclose(dx1, dx2, atol=1e-10)

    def test_bind_params_false_follows_live_params(self):
        """Default compile: dx changes when subsystem.params changes (no recompile)."""
        diag = _build_small_closed_loop()
        plant = diag.subsystems["plant"]
        x = np.array([0.3])
        u = np.array([1.0])
        t = 0.0

        ev = compile_diagram(diag, bind_params=False)
        dx1 = ev.f(x, u, t)
        plant.params["k"] = 99.0
        dx2 = ev.f(x, u, t)
        self.assertGreater(np.abs(dx2 - dx1).max(), 1e-6)

    def test_f_as_callable(self):
        fn = self.evaluator.f
        x = np.array([0.2])
        u = np.array([0.7])
        t = 0.05
        np.testing.assert_allclose(
            fn(x, u, t), self.evaluator.f(x, u, t), atol=1e-10
        )

    def test_as_scipy_rhs(self):
        x = np.array([0.4])
        u = np.array([1.0])
        t = 0.15
        rhs = self.evaluator.as_scipy_rhs()
        np.testing.assert_allclose(
            rhs(t, x), self.evaluator.f(x, self.evaluator._u_nominal, t), atol=1e-10
        )

    def test_build_execution_plan_bind_params_sets_bound_params(self):
        diag = _build_small_closed_loop()
        plan_free = build_execution_plan(diag, bind_params=False)
        plan_bound = build_execution_plan(diag, bind_params=True)
        self.assertIsNone(plan_free.port_ops[0].bound_params)
        self.assertIsNotNone(plan_bound.port_ops[0].bound_params)


# ── JAX tests (skipped if JAX not installed) ─────────────────────────

try:
    import jax.numpy as jnp

    from minilink.compile.jax_evaluator import JaxDiagramEvaluator

    _JAX_AVAILABLE = True
except ImportError:
    _JAX_AVAILABLE = False


@unittest.skipUnless(_JAX_AVAILABLE, "JAX not installed")
class TestJaxDiagramEvaluatorOutputs(unittest.TestCase):
    """JaxDiagramEvaluator boundary outputs are JIT-compiled and match NumPy."""

    def test_outputs_matches_numpy_no_boundary_ports(self):
        diag = _build_small_closed_loop_jax()
        ev_np = compile_diagram(diag, backend="numpy")
        ev_jax = compile_diagram(diag, backend="jax")
        self.assertIsInstance(ev_jax, JaxDiagramEvaluator)

        x_np, u_np = np.array([0.5]), np.array([1.0])
        x_j = jnp.array([0.5], dtype=jnp.float32)
        u_j = jnp.array([1.0], dtype=jnp.float32)
        t = 0.0

        d_np = ev_np.outputs(x_np, u_np, t)
        d_jx = ev_jax.outputs(x_j, u_j, t)
        self.assertEqual(d_np, {})
        self.assertEqual(d_jx, {})

    def test_outputs_matches_numpy_with_external_port(self):
        diag = _build_closed_loop_with_external_output_jax()
        ev_np = compile_diagram(diag, backend="numpy")
        ev_jax = compile_diagram(diag, backend="jax")
        x_j = jnp.array([0.5], dtype=jnp.float32)
        u_j = jnp.array([1.0], dtype=jnp.float32)
        t = 0.0
        d_np = ev_np.outputs(np.array([0.5]), np.array([1.0]), t)
        d_jx = ev_jax.outputs(x_j, u_j, t)
        self.assertEqual(set(d_np.keys()), set(d_jx.keys()))
        for k in d_np:
            np.testing.assert_allclose(np.asarray(d_jx[k]), d_np[k], atol=1e-5)

    def test_get_outputs_jit_matches_outputs(self):
        diag = _build_closed_loop_with_external_output_jax()
        ev = compile_diagram(diag, backend="jax")
        x_j = jnp.array([0.3], dtype=jnp.float32)
        u_j = jnp.array([1.2], dtype=jnp.float32)
        t = 0.1
        out1 = ev.outputs(x_j, u_j, t)
        out2 = ev.get_outputs_jit()(x_j, u_j, t)
        for k in out1:
            np.testing.assert_allclose(np.asarray(out1[k]), np.asarray(out2[k]), atol=1e-6)

    def test_get_internal_signals_jit_matches_compute_internal_signals(self):
        diag = _build_small_closed_loop_jax()
        ev = compile_diagram(diag, backend="jax")
        x_j = jnp.array([0.3], dtype=jnp.float32)
        u_j = jnp.array([1.2], dtype=jnp.float32)
        t = 0.1
        buf1 = ev.compute_internal_signals(x_j, u_j, t)
        buf2 = ev.get_internal_signals_jit()(x_j, u_j, t)
        np.testing.assert_allclose(np.asarray(buf1), np.asarray(buf2), atol=1e-6)


# @unittest.skipUnless(_JAX_AVAILABLE, "JAX not installed")
# class TestJaxDiagramEvaluator(unittest.TestCase):
#     """Test JaxDiagramEvaluator against the reference diagram.f() and for JAX traceability."""

#     def setUp(self):
#         self.diag = _build_small_closed_loop()
#         self.evaluator = compile_diagram(self.diag, backend="jax")
#         self.x = jnp.array([0.3], dtype=jnp.float32)
#         self.u = jnp.array([1.2], dtype=jnp.float32)

#     def test_returns_jax_evaluator(self):
#         self.assertIsInstance(self.evaluator, JaxDiagramEvaluator)

#     def test_f_matches_numpy(self):
#         """JaxDiagramEvaluator.f must match NumpyDiagramEvaluator exactly."""
#         np_evaluator = compile_diagram(self.diag, backend="numpy")
#         x_np = np.array([0.3])
#         u_np = np.array([1.2])

#         dx_numpy = np_evaluator.f(x_np, u_np, 0.1)
#         dx_jax = np.array(self.evaluator.f(self.x, self.u, 0.1))

#         np.testing.assert_allclose(dx_jax, dx_numpy, atol=1e-5)

#     def test_f_multiple_points(self):
#         """Test at several state/input combinations."""
#         np_evaluator = compile_diagram(self.diag, backend="numpy")
#         for x_val, u_val in [(0.0, 0.0), (1.0, 2.0), (-5.0, 10.0)]:
#             x_np = np.array([x_val])
#             u_np = np.array([u_val])
#             x_jax = jnp.array(x_np, dtype=jnp.float32)
#             u_jax = jnp.array(u_np, dtype=jnp.float32)
#             dx_ref = np_evaluator.f(x_np, u_np, 0.0)
#             dx_jax = np.array(self.evaluator.f(x_jax, u_jax, 0.0))
#             np.testing.assert_allclose(dx_jax, dx_ref, atol=1e-5)

#     def test_f_is_differentiable(self):
#         """jax.grad must flow through f."""
#         def dx0_of_u(u0):
#             return self.evaluator.f(
#                 self.x, jnp.array([u0], dtype=jnp.float32), 0.0
#             )[0]

#         # For an integrator dx/dt = u, so d(dx)/du = 1
#         grad_val = float(jax.grad(dx0_of_u)(jnp.array(1.2, dtype=jnp.float32)))
#         self.assertAlmostEqual(abs(grad_val), 1.0, places=4)

#     def test_internal_signals_dict_plant_y(self):
#         """JaxDiagramEvaluator.compute_internal_signals_dict for one port."""
#         d = self.evaluator.compute_internal_signals_dict(self.x, self.u, 0.0)
#         y = d["plant:y"]
#         self.assertAlmostEqual(float(y[0]), 0.3, places=5)

#     def test_internal_signals_dict_is_differentiable(self):
#         """jax.grad through compute_internal_signals_dict slice."""
#         def y_of_x(x0):
#             d = self.evaluator.compute_internal_signals_dict(
#                 jnp.array([x0], dtype=jnp.float32), self.u, 0.0
#             )
#             return d["plant:y"][0]

#         grad_val = float(jax.grad(y_of_x)(jnp.array(0.3, dtype=jnp.float32)))
#         self.assertAlmostEqual(abs(grad_val), 1.0, places=4)

#     def test_get_f_jit(self):
#         """get_f_jit returns a callable that matches eager result."""
#         jit_dx = self.evaluator.get_f_jit()
#         dx_eager = self.evaluator.f(self.x, self.u, 0.0)
#         dx_jit = jit_dx(self.x, self.u, 0.0)
#         np.testing.assert_allclose(
#             np.array(dx_jit), np.array(dx_eager), atol=1e-5
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
