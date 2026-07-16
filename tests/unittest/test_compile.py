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
import pytest

from minilink.blocks.basic import Integrator
from minilink.control.output import ProportionalController
from minilink.core.backends import array_module
from minilink.core.compile.compiler import (
    build_execution_plan,
    check_algebraic_loops,
    compile_diagram,
)
from minilink.core.compile.evaluators.numpy_evaluators import NumpyDiagramEvaluator
from minilink.core.compile.execution_plan import ExecutionPlan
from minilink.core.diagram import DiagramSystem
from minilink.core.system import DynamicSystem, System


# Helper: reusable test diagrams
class _JAXFriendlyPController(System):
    """P controller using NumPy or JAX ops so ``compile(..., backend='jax')`` traces."""

    def __init__(self):
        super().__init__()
        self.params = {"Kp": 10.0}
        self.name = "Controller"
        self.add_input_port("r", nominal_value=0.0)
        self.add_input_port("y", nominal_value=0.0)
        self.add_output_port("u", function=self.ctl, dependencies=("r", "y"))

    def ctl(self, x, u, t=0, params=None):
        if params is None:
            params = self.params
        Kp = params["Kp"]
        xp = array_module(u)
        r, y_ = self.get_port_values_from_u(u, "r", "y")
        return xp.array([Kp * (r[0] - y_[0])])


class _JAXFriendlyIntegrator(DynamicSystem):
    """``dx = k u`` integrator; mirrors :class:`Integrator` with JAX-friendly arrays."""

    def __init__(self):
        super().__init__(n=1, input_dim=1, output_dim=1, y_dependencies=())
        self.params = {"k": 1.0}
        self.name = "Integrator"

    def f(self, x, u, t=0, params=None):
        if params is None:
            params = self.params
        k = params["k"]
        xp = array_module(x)
        return xp.array([k * u[0]])

    def h(self, x, u, t=0, params=None):
        xp = array_module(x)
        return xp.array([x[0]])


def _build_small_closed_loop_jax():
    """Same wiring as :func:`_build_small_closed_loop` with JAX-traceable blocks."""
    diag = DiagramSystem()
    diag.connection_verbose = False

    ctl = _JAXFriendlyPController()
    plant = _JAXFriendlyIntegrator()
    ctl.params["Kp"] = 2.5

    diag.add_subsystem(ctl, "ctl")
    diag.add_subsystem(plant, "plant")
    diag.add_input_port("r")

    diag.connect("input", "r", "ctl", "r")
    diag.connect("plant", "y", "ctl", "y")
    diag.connect("ctl", "u", "plant", "u")
    return diag


def _build_closed_loop_with_external_output_jax():
    diag = _build_small_closed_loop_jax()
    diag.connect_new_output_port("plant", "y", "y_meas")
    return diag


def _build_small_closed_loop():
    """Step → ProportionalController → Integrator → feedback to controller."""
    diag = DiagramSystem()
    diag.connection_verbose = False

    ctl = ProportionalController(2.5)
    plant = Integrator()

    diag.add_subsystem(ctl, "ctl")
    diag.add_subsystem(plant, "plant")
    diag.add_input_port("r")

    diag.connect("input", "r", "ctl", "r")
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
            super().__init__()
            self.name = name
            self.add_input_port("u")
            self.add_output_port("y", function=self.h, dependencies=("u",))

        def h(self, x, u, t=0, params=None):
            return u * 2

    diag = DiagramSystem()
    diag.connection_verbose = False
    diag.add_subsystem(FeedthroughSystem("A"), "A")
    diag.add_subsystem(FeedthroughSystem("B"), "B")
    diag.add_subsystem(FeedthroughSystem("C"), "C")

    diag.connect("A", "y", "B", "u")
    diag.connect("B", "y", "C", "u")
    diag.connect("C", "y", "A", "u")
    return diag


# Tests
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

    def test_f_matches_diagram_f_for_representative_points(self):
        """Compiled f matches the recursive reference over several operating points."""
        for x_val, u_val, t in [
            (0.3, 1.2, 0.1),
            (0.0, 0.0, 0.0),
            (1.0, 2.0, 0.0),
            (-5.0, 10.0, 0.0),
        ]:
            x = np.array([x_val])
            u = np.array([u_val])
            dx_ref = self.diag.f(x, u, t)
            dx_comp = self.evaluator.f(x, u, t)
            np.testing.assert_allclose(dx_comp, dx_ref, atol=1e-10)

    def test_reference_path_forwards_explicit_subsystem_params(self):
        """The recursive reference path honors explicit per-subsystem params."""
        diag = _build_small_closed_loop()
        x = np.array([0.5])
        u = np.array([2.0])
        params = {
            "ctl": {"K": np.array([[4.0]])},
            "plant": {"k": 3.0},
        }

        dx = diag.f(x, u, 0.0, params=params)

        np.testing.assert_allclose(dx, np.array([18.0]), atol=1e-10)

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
        diag.connection_verbose = False

        class Gain(System):
            def __init__(self):
                super().__init__()
                self.add_input_port("u")
                self.add_output_port("y", function=self.h, dependencies=("u",))

            def h(self, x, u, t=0, params=None):
                return u * 2.0

        diag.add_subsystem(Gain(), "gain")
        diag.add_input_port("u")
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
        np.testing.assert_allclose(fn(x, u, t), self.evaluator.f(x, u, t), atol=1e-10)

    def test_as_scipy_rhs(self):
        x = np.array([0.4])
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


# JAX tests (skipped if JAX not installed)
try:
    import jax
    import jax.numpy as jnp

    from minilink.core.compile.evaluators.jax_evaluators import JaxDiagramEvaluator

    _JAX_AVAILABLE = True
except ImportError:
    _JAX_AVAILABLE = False


@pytest.mark.optional
@pytest.mark.jax
@unittest.skipUnless(_JAX_AVAILABLE, "JAX not installed")
class TestJaxDiagramEvaluatorOutputs(unittest.TestCase):
    """JaxDiagramEvaluator boundary outputs are JIT-compiled and match NumPy."""

    def test_outputs_match_numpy_for_boundary_configurations(self):
        for diag in [
            _build_small_closed_loop_jax(),
            _build_closed_loop_with_external_output_jax(),
        ]:
            ev_np = compile_diagram(diag, backend="numpy")
            ev_jax = compile_diagram(diag, backend="jax")
            self.assertIsInstance(ev_jax, JaxDiagramEvaluator)

            x_np, u_np = np.array([0.5]), np.array([1.0])
            x_j = jnp.array([0.5], dtype=jnp.float32)
            u_j = jnp.array([1.0], dtype=jnp.float32)
            t = 0.0

            d_np = ev_np.outputs(x_np, u_np, t)
            d_jx = ev_jax.outputs(x_j, u_j, t)
            self.assertEqual(set(d_np.keys()), set(d_jx.keys()))
            for k in d_np:
                np.testing.assert_allclose(np.asarray(d_jx[k]), d_np[k], atol=1e-5)

    def test_f_jax_matches_numpy(self):
        diag = _build_small_closed_loop_jax()
        ev_np = compile_diagram(diag, backend="numpy")
        ev_jax = compile_diagram(diag, backend="jax")
        x_np, u_np = np.array([0.3]), np.array([1.2])
        x_j = jnp.array(x_np, dtype=jnp.float32)
        u_j = jnp.array(u_np, dtype=jnp.float32)
        t = 0.1

        dx_np = ev_np.f(x_np, u_np, t)
        np.testing.assert_allclose(np.asarray(ev_jax.f(x_j, u_j, t)), dx_np, atol=1e-5)

    def test_internal_signals_jax_matches_numpy(self):
        diag = _build_small_closed_loop_jax()
        ev_np = compile_diagram(diag, backend="numpy")
        ev_jax = compile_diagram(diag, backend="jax")
        x_np, u_np = np.array([0.3]), np.array([1.2])
        x_j = jnp.array(x_np, dtype=jnp.float32)
        u_j = jnp.array(u_np, dtype=jnp.float32)
        t = 0.1
        buf_np = ev_np.compute_internal_signals(x_np, u_np, t)
        buf_jx = ev_jax.compute_internal_signals(x_j, u_j, t)
        np.testing.assert_allclose(np.asarray(buf_jx), buf_np, atol=1e-5)

    def test_reference_diagram_f_is_jittable(self):
        diag = _build_small_closed_loop_jax()
        ev_np = compile_diagram(diag, backend="numpy")
        x_np = np.array([0.3])
        u_np = np.array([1.2])
        x_j = jnp.array(x_np, dtype=jnp.float32)
        u_j = jnp.array(u_np, dtype=jnp.float32)
        t = 0.1

        dx_jit = jax.jit(lambda xx, uu: diag.f(xx, uu, t))(x_j, u_j)

        np.testing.assert_allclose(
            np.asarray(dx_jit),
            ev_np.f(x_np, u_np, t),
            atol=1e-5,
        )


# Diagram params contract (nested {sys_id: {...}})
class TestDiagramParamsContract(unittest.TestCase):
    """Nested params routing and the ``DiagramSystem.params`` live view."""

    def setUp(self):
        self.diag = _build_small_closed_loop()  # ctl K=2.5, plant k=1.0
        self.x = np.array([0.5])
        self.u = np.array([2.0])
        # Closed loop: dx = k * K * (r - x), with r - x = 1.5 here.

    def test_params_property_is_nested_live_view(self):
        params = self.diag.params
        self.assertEqual(set(params), {"ctl", "plant"})
        self.assertIs(params["plant"], self.diag.subsystems["plant"].params)

    def test_params_setter_distributes_by_sys_id(self):
        self.diag.params = {"plant": {"k": 3.0}}
        self.assertEqual(self.diag.subsystems["plant"].params["k"], 3.0)
        np.testing.assert_allclose(self.diag.f(self.x, self.u), [11.25], atol=1e-10)

    def test_params_setter_unknown_id_raises(self):
        with self.assertRaises(ValueError):
            self.diag.params = {"plnt": {"k": 3.0}}

    def test_mutating_params_view_affects_f(self):
        self.diag.params["plant"]["k"] = 5.0
        np.testing.assert_allclose(self.diag.f(self.x, self.u), [18.75], atol=1e-10)

    def test_partial_params_fall_back_to_defaults(self):
        dx = self.diag.f(self.x, self.u, 0.0, params={"plant": {"k": 3.0}})
        np.testing.assert_allclose(dx, [11.25], atol=1e-10)

    def test_unknown_sys_id_raises(self):
        with self.assertRaises(ValueError):
            self.diag.f(self.x, self.u, 0.0, params={"plnt": {"k": 3.0}})

    def test_flat_leaf_style_dict_raises(self):
        # The old pass-through heuristic is gone: leaf-style keys are
        # unknown subsystem ids and must fail loudly.
        with self.assertRaises(ValueError):
            self.diag.f(self.x, self.u, 0.0, params={"Kp": 4.0})

    def test_non_mapping_params_raises_typeerror(self):
        with self.assertRaises(TypeError):
            self.diag.f(self.x, self.u, 0.0, params=[1.0, 2.0])


class TestNumpyDiagramParametricTier(unittest.TestCase):
    """NumpyDiagramEvaluator ``f_p`` / ``h_p`` / ``outputs_p`` with nested params."""

    def setUp(self):
        self.diag = _build_closed_loop_with_external_output()
        self.evaluator = compile_diagram(self.diag, backend="numpy")
        self.x = np.array([0.5])
        self.u = np.array([2.0])

    def test_f_p_matches_recursive_reference(self):
        params = {"ctl": {"K": np.array([[4.0]])}, "plant": {"k": 3.0}}
        for x_val, u_val, t in [(0.3, 1.2, 0.1), (0.0, 0.0, 0.0), (-5.0, 10.0, 0.0)]:
            x, u = np.array([x_val]), np.array([u_val])
            np.testing.assert_allclose(
                self.evaluator.f_p(x, u, t, params),
                self.diag.f(x, u, t, params=params),
                atol=1e-10,
            )

    def test_f_p_none_matches_f(self):
        np.testing.assert_allclose(
            self.evaluator.f_p(self.x, self.u, 0.0, None),
            self.evaluator.f(self.x, self.u, 0.0),
            atol=1e-10,
        )

    def test_f_p_partial_params(self):
        dx = self.evaluator.f_p(self.x, self.u, 0.0, {"plant": {"k": 3.0}})
        np.testing.assert_allclose(dx, [11.25], atol=1e-10)

    def test_f_p_unknown_sys_id_raises(self):
        with self.assertRaises(ValueError):
            self.evaluator.f_p(self.x, self.u, 0.0, {"plnt": {"k": 3.0}})

    def test_outputs_p_single_boundary(self):
        params = {"ctl": {"K": np.array([[4.0]])}}
        out = self.evaluator.outputs_p(self.x, self.u, 0.0, params)
        self.assertEqual(set(out), {"y_meas"})
        np.testing.assert_allclose(out["y_meas"], [0.5], atol=1e-10)


@pytest.mark.optional
@pytest.mark.jax
@unittest.skipUnless(_JAX_AVAILABLE, "JAX not installed")
class TestJaxDiagramParametricTier(unittest.TestCase):
    """JAX parametric tier: ``f_p``, ``outputs_p``, gradients w.r.t. params."""

    def setUp(self):
        self.diag = _build_closed_loop_with_external_output_jax()  # Kp=2.5, k=1.0
        self.ev = compile_diagram(self.diag, backend="jax")
        self.ev_np = compile_diagram(self.diag, backend="numpy")
        self.x_j = jnp.array([0.5], dtype=jnp.float32)
        self.u_j = jnp.array([2.0], dtype=jnp.float32)
        self.x_np = np.array([0.5])
        self.u_np = np.array([2.0])

    def test_f_p_matches_numpy(self):
        params = {"ctl": {"Kp": 4.0}, "plant": {"k": 3.0}}
        np.testing.assert_allclose(
            np.asarray(self.ev.f_p(self.x_j, self.u_j, 0.0, params)),
            self.ev_np.f_p(self.x_np, self.u_np, 0.0, params),
            atol=1e-5,
        )

    def test_outputs_p_matches_numpy(self):
        params = {"plant": {"k": 3.0}}
        out_j = self.ev.outputs_p(self.x_j, self.u_j, 0.0, params)
        out_n = self.ev_np.outputs_p(self.x_np, self.u_np, 0.0, params)
        self.assertEqual(set(out_j), set(out_n))
        for key in out_n:
            np.testing.assert_allclose(np.asarray(out_j[key]), out_n[key], atol=1e-5)

    def test_jacobian_f_params_analytic(self):
        # dx = k * Kp * (r - x): d(dx)/dk = Kp (r - x), d(dx)/dKp = k (r - x)
        params = {"ctl": {"Kp": 2.5}, "plant": {"k": 1.0}}
        jac = self.ev.jacobian_f_params(self.x_j, self.u_j, 0.0, params)
        np.testing.assert_allclose(
            np.asarray(jac["plant"]["k"]), [2.5 * 1.5], atol=1e-5
        )
        np.testing.assert_allclose(np.asarray(jac["ctl"]["Kp"]), [1.5], atol=1e-5)

    def test_grad_flows_through_f_p(self):
        f_p = self.ev.f_p

        def dx0(params):
            return f_p(self.x_j, self.u_j, 0.0, params)[0]

        g = jax.grad(dx0)({"plant": {"k": 1.0}})
        self.assertAlmostEqual(float(g["plant"]["k"]), 2.5 * 1.5, places=4)

    def test_jacobian_f_params_requires_params(self):
        with self.assertRaises(ValueError):
            self.ev.jacobian_f_params(self.x_j, self.u_j, 0.0, None)


if __name__ == "__main__":
    unittest.main()


# --- merged from test_compile_static.py ---

"""Typed compile() dispatch for static System leaves."""

import unittest

import numpy as np
import pytest

from minilink.blocks.routing import Gain
from minilink.core.compile.compiler import compile
from minilink.core.compile.evaluators.evaluators import DynamicsEvaluator
from minilink.core.compile.evaluators.numpy_evaluators import NumpyStaticEvaluator

try:
    import jax  # noqa: F401

    from minilink.core.compile.evaluators.jax_evaluators import JaxStaticEvaluator

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


# --- merged from test_compile_step_leaf.py ---

"""compile() dispatch for StepSystem leaves."""

import unittest

import numpy as np

from minilink.core.compile.compiler import compile
from minilink.core.compile.evaluators.numpy_evaluators import NumpyStepEvaluator
from minilink.core.system import StepSystem


class LinearStep(StepSystem):
    def __init__(self):
        super().__init__(n=1, input_dim=1)
        self.params = {"a": 0.5}
        self.x0 = np.array([1.0])

    def step(self, x, u, k=0, params=None):
        params = self.params if params is None else params
        return np.array([params["a"] * x[0] + u[0]])


class TestCompileStepLeaf(unittest.TestCase):
    def test_compile_returns_numpy_step_evaluator(self):
        ev = compile(LinearStep())
        self.assertIsInstance(ev, NumpyStepEvaluator)

    def test_frozen_params_step_parity(self):
        plant = LinearStep()
        ev = compile(plant)
        x = np.array([2.0])
        u = np.array([1.0])
        np.testing.assert_allclose(ev.step(x, u, 0), plant.step(x, u, 0))
        np.testing.assert_allclose(
            ev.step_p(x, u, 0, {"a": 0.25}), plant.step(x, u, 0, {"a": 0.25})
        )


if __name__ == "__main__":
    unittest.main()


# --- merged from test_evaluator_api.py ---

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


# --- merged from test_evaluator_tiers.py ---

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
        from minilink.core.compile.evaluators.jax_evaluators import JaxDynamicEvaluator

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

    def test_rk4_integrate_zoh_trace_parity(self):
        u_seq = jnp.array([[1.0], [0.5], [0.0]])
        np.testing.assert_allclose(
            np.asarray(self.ev.rk4_integrate_zoh_trace(self.x, u_seq, self.t, self.dt)),
            np.asarray(self.ev.rk4_integrate_zoh(self.x, u_seq, self.t, self.dt)),
            atol=1e-6,
        )

    def test_rk4_integrate_linear_p_fast_tier(self):
        u_knots = jnp.array([[1.0], [0.5], [0.0]])
        out = self.ev.rk4_integrate_linear_p(
            self.x, u_knots, self.t, self.dt, self.params
        )
        self.assertEqual(out.shape, (3, 1))

    def test_euler_integrate_zoh_trace_parity(self):
        u_seq = jnp.array([[1.0], [0.5], [0.0]])
        np.testing.assert_allclose(
            np.asarray(
                self.ev.euler_integrate_zoh_trace(self.x, u_seq, self.t, self.dt)
            ),
            np.asarray(self.ev.euler_integrate_zoh(self.x, u_seq, self.t, self.dt)),
            atol=1e-6,
        )

    def test_f_ivp_p_matches_f_ivp_with_frozen(self):
        np.testing.assert_allclose(
            np.asarray(self.ev.f_ivp_p(self.x, self.t, self.params)),
            np.asarray(self.ev.f_ivp(self.x, self.t)),
            atol=1e-6,
        )

    def test_euler_step_p(self):
        out = self.ev.euler_step_p(self.x, self.u, self.t, self.dt, self.params)
        self.assertEqual(out.shape, (1,))


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
        from minilink.core.compile.evaluators.jax_evaluators import JaxStepEvaluator

        np.testing.assert_allclose(
            np.asarray(ev.step_trace(x, u, 0)),
            np.asarray(ev.step(x, u, 0)),
            atol=1e-6,
        )
        self.assertIs(JaxStepEvaluator.step_jit, JaxStepEvaluator.step)


# --- merged from test_mathematical_program_evaluators.py ---

"""Unit tests for pure mathematical programs and compiled evaluators."""

import numpy as np
import pytest

from minilink.optimization.evaluators.compiler import compile_program_evaluator
from minilink.optimization.mathematical_program import MathematicalProgram


def test_mathematical_program_is_pure_description_without_z0():
    def J(z: np.ndarray):
        return z @ z

    program = MathematicalProgram(
        n_z=2,
        J=J,
        metadata={"source": "test"},
    )

    assert program.backend == "numpy"
    assert program.n_z == 2
    assert not hasattr(program, "z0")
    assert program.metadata == {"source": "test"}


def test_mathematical_program_validates_bounds_dimension():
    with pytest.raises(ValueError, match="lower bounds must have shape"):
        MathematicalProgram(
            n_z=2,
            J=lambda z: z @ z,
            lower=np.zeros(3),
        )


def test_mathematical_program_validates_bounds_order():
    with pytest.raises(ValueError, match="lower bounds must be less"):
        MathematicalProgram(
            n_z=1,
            J=lambda z: z[0] * z[0],
            lower=np.array([2.0]),
            upper=np.array([1.0]),
        )


def test_numpy_evaluator_objective_constraints_and_derivatives():
    def J(z: np.ndarray):
        return z[0] ** 2 + z[1] ** 2

    def grad_J(z: np.ndarray) -> np.ndarray:
        return 2.0 * z

    def hess_J(z: np.ndarray) -> np.ndarray:
        return 2.0 * np.eye(2)

    def h(z: np.ndarray) -> np.ndarray:
        return np.array([z[0] + z[1] - 1.0])

    def jac_h(z: np.ndarray) -> np.ndarray:
        return np.array([[1.0, 1.0]])

    def g(z: np.ndarray) -> np.ndarray:
        return np.array([z[0], z[1]])

    def jac_g(z: np.ndarray) -> np.ndarray:
        return np.eye(2)

    program = MathematicalProgram(
        n_z=2,
        J=J,
        h=h,
        g=g,
        grad_J=grad_J,
        hess_J=hess_J,
        jac_h=jac_h,
        jac_g=jac_g,
    )
    program_evaluator = compile_program_evaluator(
        program,
        sample_z=np.array([0.5, 0.5]),
    )

    assert program_evaluator.backend == "numpy"
    assert program_evaluator.n_h == 1
    assert program_evaluator.n_g == 2
    assert program_evaluator.objective(np.array([1.0, 2.0])) == pytest.approx(5.0)
    np.testing.assert_allclose(program_evaluator.equality_residual([0.25, 0.75]), [0.0])
    np.testing.assert_allclose(
        program_evaluator.inequality_margin([0.25, 0.75]),
        [0.25, 0.75],
    )
    np.testing.assert_allclose(program_evaluator.gradient([1.0, 2.0]), [2.0, 4.0])
    np.testing.assert_allclose(
        program_evaluator.hessian([1.0, 2.0]),
        2.0 * np.eye(2),
    )
    np.testing.assert_allclose(program_evaluator.jacobian_h([1.0, 2.0]), [[1.0, 1.0]])
    np.testing.assert_allclose(program_evaluator.jacobian_g([1.0, 2.0]), np.eye(2))


def test_numpy_evaluator_empty_constraints():
    program = MathematicalProgram(n_z=1, J=lambda z: z[0] ** 2)
    program_evaluator = compile_program_evaluator(program, sample_z=np.array([1.0]))

    assert program_evaluator.n_h == 0
    assert program_evaluator.n_g == 0
    np.testing.assert_allclose(program_evaluator.equality_residual([1.0]), [])
    np.testing.assert_allclose(program_evaluator.inequality_margin([1.0]), [])
    np.testing.assert_allclose(program_evaluator.jacobian_h([1.0]), np.zeros((0, 1)))
    np.testing.assert_allclose(program_evaluator.jacobian_g([1.0]), np.zeros((0, 1)))


def test_numpy_evaluator_reports_bad_constraint_shape():
    program = MathematicalProgram(
        n_z=2,
        J=lambda z: z @ z,
        h=lambda z: np.array([z[0]]),
    )
    program_evaluator = compile_program_evaluator(
        program,
        sample_z=np.array([0.0, 0.0]),
    )

    with pytest.raises(ValueError, match="decision vector must have shape"):
        program_evaluator.objective(np.array([1.0, 2.0, 3.0]))


def test_jax_evaluator_autodiff_when_available():
    jax = pytest.importorskip("jax")
    jnp = pytest.importorskip("jax.numpy")

    def J(z):
        return z @ z

    def h(z):
        return jnp.array([z[0] + z[1] - 1.0])

    def g(z):
        return jnp.array([z[0], z[1]])

    program = MathematicalProgram(n_z=2, J=J, h=h, g=g)
    program_evaluator = compile_program_evaluator(
        program,
        backend="jax",
        sample_z=jnp.array([0.5, 0.5]),
        use_hessian=True,
    )

    assert program_evaluator.backend == "jax"
    assert program_evaluator.objective([1.0, 2.0]) == pytest.approx(5.0)
    np.testing.assert_allclose(program_evaluator.gradient([1.0, 2.0]), [2.0, 4.0])
    np.testing.assert_allclose(
        program_evaluator.hessian([1.0, 2.0]),
        2.0 * np.eye(2),
    )
    np.testing.assert_allclose(
        program_evaluator.jacobian_h([1.0, 2.0]),
        [[1.0, 1.0]],
    )
    np.testing.assert_allclose(program_evaluator.jacobian_g([1.0, 2.0]), np.eye(2))

    # Keep the imported symbol used so linters do not mark it as dead in some configs.
    assert jax is not None
