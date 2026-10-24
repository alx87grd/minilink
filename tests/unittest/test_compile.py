"""Tests for the compilation pipeline.

Validates that:
- ``compile_diagram()`` produces a working ``DiagramEvaluator``
- ``f()`` matches ``diagram.f()`` (the slow recursive reference)
- ``signals()`` / ``compute_internal_signals_dict()`` expose the buffer
- ``check_algebraic_loops()`` detects loops and ``evaluation_order_of()`` returns a valid order
- ``build_execution_plan()`` produces a valid ``ExecutionPlan``
"""

import unittest
import warnings
import numpy as np
import pytest
from minilink.blocks.basic import Integrator
from minilink.control.siso import P
from minilink.core.backends import array_module
from minilink.core.compile.compiler import compile_diagram
from minilink.core.compile.plan import build_execution_plan
from minilink.core.compile.evaluators.diagram import DiagramEvaluator
from minilink.core.compile.plan import ExecutionPlan
from minilink.core.diagram import DiagramSystem
from minilink.core.system import DynamicSystem, System
from minilink.core.topology import check_algebraic_loops, evaluation_order_of


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
    diag.add_subsystem(ctl, "controller")
    diag.add_subsystem(plant, "plant")
    diag.add_input_port("r")
    diag.connect("input", "r", "controller", "r")
    diag.connect("plant", "y", "controller", "y")
    diag.connect("controller", "u", "plant", "u")
    return diag


def _build_closed_loop_with_external_output_jax():
    diag = _build_small_closed_loop_jax()
    diag.connect_new_output_port("plant", "y", "y_meas")
    return diag


def _build_small_closed_loop():
    """Step → P → Integrator → feedback to controller."""
    diag = DiagramSystem()
    diag.connection_verbose = False
    ctl = P(2.5, ports="reference")
    plant = Integrator()
    diag.add_subsystem(ctl, "controller")
    diag.add_subsystem(plant, "plant")
    diag.add_input_port("r")
    diag.connect("input", "r", "controller", "r")
    diag.connect("plant", "y", "controller", "y")
    diag.connect("controller", "u", "plant", "u")
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


class TestCheckAlgebraicLoops(unittest.TestCase):
    """Test standalone algebraic loop detection."""

    def test_no_loop_returns_order(self):
        diag = _build_small_closed_loop()
        self.assertIsNone(check_algebraic_loops(diag))
        order = evaluation_order_of(diag)
        self.assertIsInstance(order, list)
        self.assertGreaterEqual(len(order), 1)
        for sys_id, port_id in order:
            self.assertIn(sys_id, diag.subsystems)

    def test_loop_raises(self):
        diag = _build_feedthrough_loop()
        for check in (check_algebraic_loops, evaluation_order_of):
            with self.assertRaises(RuntimeError) as ctx:
                check(diag)
            self.assertIn("Algebraic loop detected", str(ctx.exception))

    def test_chain_no_loop(self):
        """A → B → C (no loop) should succeed."""
        diag = _build_feedthrough_loop()
        diag.connections["A"]["u"] = None
        order = evaluation_order_of(diag)
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
            plan.external_output_slices["y_meas"], plan.output_slices["plant", "y"]
        )


class TestCompileDiagram(unittest.TestCase):
    """Test the top-level compile_diagram entry point."""

    def test_returns_numpy_evaluator(self):
        diag = _build_small_closed_loop()
        evaluator = compile_diagram(diag, backend="numpy")
        self.assertIsInstance(evaluator, DiagramEvaluator)

    def test_invalid_backend_raises(self):
        diag = _build_small_closed_loop()
        with self.assertRaises(ValueError):
            compile_diagram(diag, backend="unknown")


class TestDiagramEvaluatorNumPy(unittest.TestCase):
    """Test the NumPy DiagramEvaluator against the reference diagram.f()."""

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
            "controller": {
                "Kp": np.array([4.0]),
                "u_min": np.array([-np.inf]),
                "u_max": np.array([np.inf]),
            },
            "plant": {"k": 3.0},
        }
        dx = diag.f(x, u, 0.0, params=params)
        np.testing.assert_allclose(dx, np.array([18.0]), atol=1e-10)

    def test_compute_internal_signals_non_empty(self):
        x = np.array([0.5])
        u = np.array([1.0])
        buf = self.evaluator.signals(x, u, 0.0)
        self.assertGreater(buf.size, 0)

    def test_compute_internal_signals_dict_plant_y(self):
        x = np.array([0.5])
        u = np.array([1.0])
        d = self.evaluator.compute_internal_signals_dict(x, u, 0.0)
        np.testing.assert_allclose(d["plant:y"], np.array([0.5]), atol=1e-10)

    def test_signals_buffer(self):
        x = np.array([0.5])
        u = np.array([1.0])
        signals = self.evaluator.signals(x, u, 0.0)
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
        self.assertGreater(np.abs(dx2 - dx1).max(), 1e-06)

    def test_f_as_callable(self):
        fn = self.evaluator.f
        x = np.array([0.2])
        u = np.array([0.7])
        t = 0.05
        np.testing.assert_allclose(fn(x, u, t), self.evaluator.f(x, u, t), atol=1e-10)

    def test_nominal_input_is_the_held_input(self):
        x = np.array([0.4])
        t = 0.15
        np.testing.assert_allclose(
            self.evaluator.f(x, self.evaluator.nominal_input, t),
            self.evaluator.f(x, self.evaluator._u_nominal, t),
            atol=1e-10,
        )

    def test_build_execution_plan_bind_params_sets_bound_params(self):
        diag = _build_small_closed_loop()
        plan_free = build_execution_plan(diag, bind_params=False)
        plan_bound = build_execution_plan(diag, bind_params=True)
        self.assertIsNone(plan_free.port_ops[0].bound_params)
        self.assertIsNotNone(plan_bound.port_ops[0].bound_params)


try:
    import jax
    import jax.numpy as jnp
    from minilink.core.compile.evaluators.diagram import DiagramEvaluator

    _JAX_AVAILABLE = True
except ImportError:
    _JAX_AVAILABLE = False


@pytest.mark.optional
@pytest.mark.jax
@unittest.skipUnless(_JAX_AVAILABLE, "JAX not installed")
class TestDiagramEvaluatorOutputsJax(unittest.TestCase):
    """The JAX DiagramEvaluator's boundary outputs are compiled and match NumPy."""

    def test_outputs_match_numpy_for_boundary_configurations(self):
        for diag in [
            _build_small_closed_loop_jax(),
            _build_closed_loop_with_external_output_jax(),
        ]:
            ev_np = compile_diagram(diag, backend="numpy")
            ev_jax = compile_diagram(diag, backend="jax")
            self.assertIsInstance(ev_jax, DiagramEvaluator)
            x_np, u_np = (np.array([0.5]), np.array([1.0]))
            x_j = jnp.array([0.5], dtype=jnp.float32)
            u_j = jnp.array([1.0], dtype=jnp.float32)
            t = 0.0
            d_np = ev_np.outputs(x_np, u_np, t)
            d_jx = ev_jax.outputs(x_j, u_j, t)
            self.assertEqual(set(d_np.keys()), set(d_jx.keys()))
            for k in d_np:
                np.testing.assert_allclose(np.asarray(d_jx[k]), d_np[k], atol=1e-05)

    def test_f_jax_matches_numpy(self):
        diag = _build_small_closed_loop_jax()
        ev_np = compile_diagram(diag, backend="numpy")
        ev_jax = compile_diagram(diag, backend="jax")
        x_np, u_np = (np.array([0.3]), np.array([1.2]))
        x_j = jnp.array(x_np, dtype=jnp.float32)
        u_j = jnp.array(u_np, dtype=jnp.float32)
        t = 0.1
        dx_np = ev_np.f(x_np, u_np, t)
        np.testing.assert_allclose(np.asarray(ev_jax.f(x_j, u_j, t)), dx_np, atol=1e-05)

    def test_internal_signals_jax_matches_numpy(self):
        diag = _build_small_closed_loop_jax()
        ev_np = compile_diagram(diag, backend="numpy")
        ev_jax = compile_diagram(diag, backend="jax")
        x_np, u_np = (np.array([0.3]), np.array([1.2]))
        x_j = jnp.array(x_np, dtype=jnp.float32)
        u_j = jnp.array(u_np, dtype=jnp.float32)
        t = 0.1
        buf_np = ev_np.signals(x_np, u_np, t)
        buf_jx = ev_jax.signals(x_j, u_j, t)
        np.testing.assert_allclose(np.asarray(buf_jx), buf_np, atol=1e-05)

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
            np.asarray(dx_jit), ev_np.f(x_np, u_np, t), atol=1e-05
        )

    def test_integer_state_gives_float_derivatives(self):
        diag = DiagramSystem()
        diag.add_subsystem(Integrator(), "plant")
        diag.add_input_port("u")
        diag.connect("input", "u", "plant", "u")
        diag.connect_new_output_port("plant", "y", "y")
        for xp in (jnp, np):
            ev = compile_diagram(diag, backend="jax")
            with warnings.catch_warnings(record=True) as caught:
                warnings.simplefilter("always")
                dx = ev.f(xp.array([0]), xp.array([0.7]), 0.0)
            np.testing.assert_allclose(np.asarray(dx), [0.7])
            self.assertEqual(
                [w for w in caught if issubclass(w.category, FutureWarning)], []
            )
        x32 = jnp.array([0.0], dtype=jnp.float32)
        u32 = jnp.array([0.7], dtype=jnp.float32)
        self.assertEqual(ev.f(x32, u32, 0.0).dtype, jnp.float32)
        x = jnp.array([0.0])
        self.assertEqual(ev.f(x, jnp.array([0.7]), 0.0).dtype, x.dtype)


class TestDiagramParamsContract(unittest.TestCase):
    """Nested params routing and the ``DiagramSystem.params`` live view."""

    def setUp(self):
        self.diag = _build_small_closed_loop()
        self.x = np.array([0.5])
        self.u = np.array([2.0])

    def test_params_property_is_nested_live_view(self):
        params = self.diag.params
        self.assertEqual(set(params), {"controller", "plant"})
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
        with self.assertRaises(ValueError):
            self.diag.f(self.x, self.u, 0.0, params={"Kp": 4.0})

    def test_non_mapping_params_raises_typeerror(self):
        message = r"Diagram params must be a mapping keyed by subsystem id .* got list"
        with self.assertRaisesRegex(TypeError, message):
            self.diag.f(self.x, self.u, 0.0, params=[1.0, 2.0])
        with self.assertRaisesRegex(TypeError, message):
            self.diag.subsystem_params([1.0, 2.0], "plant")


class TestNumpyDiagramParametricTier(unittest.TestCase):
    """DiagramEvaluator ``f`` / ``outputs`` with nested params."""

    def setUp(self):
        self.diag = _build_closed_loop_with_external_output()
        self.evaluator = compile_diagram(self.diag, backend="numpy")
        self.x = np.array([0.5])
        self.u = np.array([2.0])

    def test_f_with_params_matches_recursive_reference(self):
        params = {
            "controller": {
                "Kp": np.array([4.0]),
                "u_min": np.array([-np.inf]),
                "u_max": np.array([np.inf]),
            },
            "plant": {"k": 3.0},
        }
        for x_val, u_val, t in [(0.3, 1.2, 0.1), (0.0, 0.0, 0.0), (-5.0, 10.0, 0.0)]:
            x, u = (np.array([x_val]), np.array([u_val]))
            np.testing.assert_allclose(
                self.evaluator.f(x, u, t, params),
                self.diag.f(x, u, t, params=params),
                atol=1e-10,
            )

    def test_params_none_matches_the_compiled_parameters(self):
        np.testing.assert_allclose(
            self.evaluator.f(self.x, self.u, 0.0, None),
            self.evaluator.f(self.x, self.u, 0.0),
            atol=1e-10,
        )

    def test_partial_params(self):
        dx = self.evaluator.f(self.x, self.u, 0.0, {"plant": {"k": 3.0}})
        np.testing.assert_allclose(dx, [11.25], atol=1e-10)

    def test_unknown_sys_id_raises(self):
        with self.assertRaises(ValueError):
            self.evaluator.f(self.x, self.u, 0.0, {"plnt": {"k": 3.0}})

    def test_outputs_with_params_single_boundary(self):
        params = {
            "controller": {
                "Kp": np.array([4.0]),
                "u_min": np.array([-np.inf]),
                "u_max": np.array([np.inf]),
            }
        }
        out = self.evaluator.outputs(self.x, self.u, 0.0, params)
        self.assertEqual(set(out), {"y_meas"})
        np.testing.assert_allclose(out["y_meas"], [0.5], atol=1e-10)


@pytest.mark.optional
@pytest.mark.jax
@unittest.skipUnless(_JAX_AVAILABLE, "JAX not installed")
class TestDiagramEvaluatorParamsJax(unittest.TestCase):
    """JAX evaluator with params: ``f``, ``outputs``, gradients w.r.t. params."""

    def setUp(self):
        self.diag = _build_closed_loop_with_external_output_jax()
        self.ev = compile_diagram(self.diag, backend="jax")
        self.ev_np = compile_diagram(self.diag, backend="numpy")
        self.x_j = jnp.array([0.5], dtype=jnp.float32)
        self.u_j = jnp.array([2.0], dtype=jnp.float32)
        self.x_np = np.array([0.5])
        self.u_np = np.array([2.0])

    def test_f_with_params_matches_numpy(self):
        params = {"controller": {"Kp": 4.0}, "plant": {"k": 3.0}}
        np.testing.assert_allclose(
            np.asarray(self.ev.f(self.x_j, self.u_j, 0.0, params)),
            self.ev_np.f(self.x_np, self.u_np, 0.0, params),
            atol=1e-05,
        )

    def test_outputs_p_matches_numpy(self):
        params = {"plant": {"k": 3.0}}
        out_j = self.ev.outputs(self.x_j, self.u_j, 0.0, params)
        out_n = self.ev_np.outputs(self.x_np, self.u_np, 0.0, params)
        self.assertEqual(set(out_j), set(out_n))
        for key in out_n:
            np.testing.assert_allclose(np.asarray(out_j[key]), out_n[key], atol=1e-05)

    def test_jacobian_wrt_params_analytic(self):
        params = {"controller": {"Kp": 2.5}, "plant": {"k": 1.0}}
        jac = self.ev.jacobian("f", "params")(self.x_j, self.u_j, 0.0, params)
        np.testing.assert_allclose(
            np.asarray(jac["plant"]["k"]), [2.5 * 1.5], atol=1e-05
        )
        np.testing.assert_allclose(
            np.asarray(jac["controller"]["Kp"]), [1.5], atol=1e-05
        )

    def test_grad_flows_through_f_with_params(self):
        f = self.ev.f

        def dx0(params):
            return f(self.x_j, self.u_j, 0.0, params)[0]

        g = jax.grad(dx0)({"plant": {"k": 1.0}})
        self.assertAlmostEqual(float(g["plant"]["k"]), 2.5 * 1.5, places=4)

    def test_jacobian_wrt_params_requires_params(self):
        with self.assertRaises(ValueError):
            self.ev.jacobian("f", "params")(self.x_j, self.u_j, 0.0, None)


from minilink.blocks.routing import Gain
from minilink.core.compile.compiler import compile
from minilink.core.compile.evaluators.base import ContinuousEvaluator
from minilink.core.compile.evaluators.static import StaticEvaluator

try:
    import jax

    _JAX_AVAILABLE = True
except ImportError:
    _JAX_AVAILABLE = False


class TestCompileStatic(unittest.TestCase):
    def test_compile_gain_returns_numpy_static_evaluator(self):
        gain = Gain(K=3.0, dim=1)
        ev = compile(gain)
        self.assertIsInstance(ev, StaticEvaluator)
        self.assertFalse(isinstance(ev, ContinuousEvaluator))
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
        self.assertIsInstance(ev_jx, StaticEvaluator)
        u = np.array([2.0])
        out_np = ev_np.outputs(np.array([]), u, 0.0)
        out_jx = ev_jx.outputs(np.array([]), u, 0.0)
        np.testing.assert_allclose(np.asarray(out_jx["y"]), out_np["y"], atol=1e-05)


from minilink.core.compile.evaluators.discrete import StepEvaluator
from minilink.core.discrete.step_system import StepSystem


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
        self.assertIsInstance(ev, StepEvaluator)

    def test_frozen_params_step_parity(self):
        plant = LinearStep()
        ev = compile(plant)
        x = np.array([2.0])
        u = np.array([1.0])
        np.testing.assert_allclose(ev.step(x, u, 0), plant.step(x, u, 0))
        np.testing.assert_allclose(
            ev.step(x, u, 0, {"a": 0.25}), plant.step(x, u, 0, {"a": 0.25})
        )


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
        self.assertIsInstance(ev, StepEvaluator)
        self.assertTrue(hasattr(ev, "rollout"))
        self.assertFalse(hasattr(ev, "f"))
        self.assertFalse(hasattr(ev, "integrate"))
        self.assertFalse(hasattr(ev, "h"))
        self.assertFalse(isinstance(ev, ContinuousEvaluator))


@pytest.mark.optional
@pytest.mark.jax
@unittest.skipUnless(_JAX_AVAILABLE, "JAX not installed")
class TestEvaluatorApiJax(unittest.TestCase):
    def test_f_matches_numpy(self):
        plant = Integrator()
        ev_np = compile(plant, backend="numpy")
        ev_jx = compile(plant, backend="jax")
        x_np, u_np = (np.array([0.2]), np.array([1.0]))
        x_j, u_j = (jnp.array(x_np), jnp.array(u_np))
        np.testing.assert_allclose(
            np.asarray(ev_jx.f(x_j, u_j, 0.0)), ev_np.f(x_np, u_np, 0.0), atol=1e-05
        )


try:
    import jax
    import jax.numpy as jnp

    _JAX_AVAILABLE = True
except ImportError:
    _JAX_AVAILABLE = False


@pytest.mark.optional
@pytest.mark.jax
@unittest.skipUnless(_JAX_AVAILABLE, "JAX not installed")
class TestCompiledMethodsJax(unittest.TestCase):
    def setUp(self):
        self.plant = Integrator()
        self.ev = compile(self.plant, backend="jax")
        self.x = jnp.array([0.2])
        self.u = jnp.array([1.0])
        self.t = 0.0

    def test_params_none_reads_the_compiled_parameters(self):
        frozen = self.ev._frozen_params
        np.testing.assert_array_equal(
            np.asarray(self.ev.f(self.x, self.u, self.t)),
            np.asarray(self.ev.f(self.x, self.u, self.t, frozen)),
        )

    def test_each_method_is_compiled_once_per_evaluator(self):
        # The jitted form, built on first use and cached: the same object each time
        self.assertIs(self.ev.f, self.ev.f)
        self.assertEqual(self.ev.f.__name__, "f")
        other = compile(Integrator(), backend="jax")
        self.assertIsNot(other.f, self.ev.f)

    def test_a_compiled_method_composes_inside_jit_and_grad(self):
        x, u, t = (self.x, self.u, self.t)

        @jax.jit
        def loss(theta):
            p = {"k": theta[0]}
            return jnp.sum((self.ev.f(x, u, t, p) - jnp.array([1.0])) ** 2)

        grad = jax.jit(jax.grad(loss))(jnp.array([1.0]))
        self.assertEqual(grad.shape, (1,))

    def test_a_flag_in_a_params_dict_stays_a_python_value(self):
        from minilink import JointImpedance, Pendulum

        # The impedance law branches on params["gravity_comp"], a Python bool
        plant = Pendulum()
        loop = JointImpedance(plant) @ plant
        ev = compile(loop, backend="jax")
        x = jnp.full(ev.n, 0.1)
        u = jnp.zeros(ev.m)
        np.testing.assert_array_equal(
            np.asarray(ev.f(x, u, 0.0, loop.params)), np.asarray(ev.f(x, u, 0.0))
        )
        steps = jax.vmap(lambda xx: ev.step(xx, u, 0.0, loop.params, dt=0.01))(
            jnp.stack([x, x])
        )
        self.assertEqual(steps.shape, (2, ev.n))

    def test_the_step_model_composes_inside_vmap(self):
        x0s = jnp.stack([self.x, 2.0 * self.x])
        steps = jax.vmap(lambda x: self.ev.step(x, self.u, 0.0, dt=0.1))(x0s)
        np.testing.assert_allclose(
            np.asarray(steps[1]),
            np.asarray(self.ev.step(2.0 * self.x, self.u, 0.0, dt=0.1)),
            rtol=1e-12,
        )


class TestCompiledMethodsNumPy(unittest.TestCase):
    def test_numpy_methods_are_the_plain_bodies(self):
        ev = compile(Integrator(), backend="numpy")
        self.assertEqual(ev.f.__func__.__name__, "f")
        self.assertEqual(ev.compiled_forms, {})


@pytest.mark.optional
@pytest.mark.jax
@unittest.skipUnless(_JAX_AVAILABLE, "JAX not installed")
class TestStepEvaluatorJax(unittest.TestCase):
    def test_step_composes_in_a_trace_and_takes_params(self):

        class ScaleStep(StepSystem):
            def __init__(self):
                super().__init__(n=1)
                self.params = {"a": 2.0}

            def step(self, x, u, k=0, params=None):
                params = self.params if params is None else params
                return params["a"] * x

        ev = compile(ScaleStep(), backend="jax")
        x = jnp.array([1.0])
        u = jnp.array([0.0])
        np.testing.assert_allclose(np.asarray(ev.step(x, u, 0)), [2.0])
        np.testing.assert_allclose(np.asarray(ev.step(x, u, 0, {"a": 3.0})), [3.0])
        traced = jax.jit(lambda xx: ev.step(xx, u, 0, {"a": 3.0}))(x)
        np.testing.assert_allclose(np.asarray(traced), [3.0])


from minilink.optimization.evaluators.compiler import compile_program_evaluator
from minilink.optimization.mathematical_program import MathematicalProgram


def test_mathematical_program_is_pure_description_without_z0():

    def J(z: np.ndarray):
        return z @ z

    program = MathematicalProgram(n_z=2, J=J, metadata={"source": "test"})
    assert program.backend == "numpy"
    assert program.n_z == 2
    assert not hasattr(program, "z0")
    assert program.metadata == {"source": "test"}


def test_mathematical_program_validates_bounds_dimension():
    with pytest.raises(ValueError, match="lower bounds must have shape"):
        MathematicalProgram(n_z=2, J=lambda z: z @ z, lower=np.zeros(3))


def test_mathematical_program_validates_bounds_order():
    with pytest.raises(ValueError, match="lower bounds must be less"):
        MathematicalProgram(
            n_z=1, J=lambda z: z[0] * z[0], lower=np.array([2.0]), upper=np.array([1.0])
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
        n_z=2, J=J, h=h, g=g, grad_J=grad_J, hess_J=hess_J, jac_h=jac_h, jac_g=jac_g
    )
    program_evaluator = compile_program_evaluator(
        program, sample_z=np.array([0.5, 0.5])
    )
    assert program_evaluator.backend == "numpy"
    assert program_evaluator.n_h == 1
    assert program_evaluator.n_g == 2
    assert program_evaluator.objective(np.array([1.0, 2.0])) == pytest.approx(5.0)
    np.testing.assert_allclose(program_evaluator.equality_residual([0.25, 0.75]), [0.0])
    np.testing.assert_allclose(
        program_evaluator.inequality_margin([0.25, 0.75]), [0.25, 0.75]
    )
    np.testing.assert_allclose(program_evaluator.gradient([1.0, 2.0]), [2.0, 4.0])
    np.testing.assert_allclose(program_evaluator.hessian([1.0, 2.0]), 2.0 * np.eye(2))
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
        n_z=2, J=lambda z: z @ z, h=lambda z: np.array([z[0]])
    )
    program_evaluator = compile_program_evaluator(
        program, sample_z=np.array([0.0, 0.0])
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
        program, backend="jax", sample_z=jnp.array([0.5, 0.5]), use_hessian=True
    )
    assert program_evaluator.backend == "jax"
    assert program_evaluator.objective([1.0, 2.0]) == pytest.approx(5.0)
    np.testing.assert_allclose(program_evaluator.gradient([1.0, 2.0]), [2.0, 4.0])
    np.testing.assert_allclose(program_evaluator.hessian([1.0, 2.0]), 2.0 * np.eye(2))
    np.testing.assert_allclose(program_evaluator.jacobian_h([1.0, 2.0]), [[1.0, 1.0]])
    np.testing.assert_allclose(program_evaluator.jacobian_g([1.0, 2.0]), np.eye(2))
    assert jax is not None


class TestEquationShapeValidation(unittest.TestCase):
    """S02: wrong-shape f / h fail loudly at compile on both backends."""

    def _bad_f(self):
        class BadF(DynamicSystem):
            def __init__(self):
                super().__init__(n=2, input_dim=1, output_dim=2)

            def f(self, x, u, t=0, params=None):
                return np.array([x[1]])  # (1,) instead of (2,)

        return BadF()

    def _bad_h(self):
        class BadH(DynamicSystem):
            def __init__(self):
                super().__init__(n=2, input_dim=1, output_dim=2)

            def f(self, x, u, t=0, params=None):
                return np.array([x[1], -x[0]])

            def h(self, x, u, t=0, params=None):
                return np.array([x[0]])  # (1,) instead of (2,)

        return BadH()

    def test_wrong_f_shape_raises_on_numpy(self):
        with self.assertRaises(ValueError) as ctx:
            self._bad_f().compile(backend="numpy")
        self.assertIn("f() of", str(ctx.exception))
        self.assertIn("returned shape (1,); expected (2,)", str(ctx.exception))

    def test_scalar_f_is_accepted_for_a_first_order_plant(self):
        class FirstOrder(DynamicSystem):
            def __init__(self):
                super().__init__(n=1, input_dim=1, output_dim=1)

            def f(self, x, u, t=0, params=None):
                return -x[0] + u[0]  # a bare scalar, as students write it

        traj = FirstOrder().compute_trajectory(tf=1.0, show=False, verbose=False)
        self.assertEqual(traj.x.shape[0], 1)

    def test_wrong_x0_shape_is_reported_at_compile(self):
        sys = DynamicSystem(n=2, input_dim=1, output_dim=2)
        sys.x0 = np.array([0.1])
        with self.assertRaises(ValueError) as ctx:
            sys.compile(backend="numpy")
        self.assertIn("x0 of", str(ctx.exception))
        self.assertIn("expected (2,)", str(ctx.exception))

    def test_wrong_step_shape_inside_a_step_diagram_raises(self):
        from minilink.core.discrete.step_diagram import StepDiagramSystem
        from minilink.core.discrete.step_system import StepSystem

        class BadStep(StepSystem):
            def __init__(self):
                super().__init__(n=2, input_dim=1, output_dim=2)

            def step(self, x, u, k=0, params=None):
                return np.array([x[1]])  # (1,) instead of (2,)

        diagram = StepDiagramSystem()
        diagram.add_subsystem(BadStep(), "plant")
        diagram.add_input_port("u")
        diagram.connect("input", "u", "plant", "u")
        with self.assertRaises(ValueError) as ctx:
            diagram.compile()
        self.assertIn("step() of", str(ctx.exception))
        self.assertIn("(plant)", str(ctx.exception))

    def test_step_diagram_compile_probes_each_hook_once(self):
        from minilink.core.discrete.step_diagram import StepDiagramSystem
        from minilink.core.discrete.step_system import StepSystem

        class CountingStep(StepSystem):
            def __init__(self):
                super().__init__(n=1, input_dim=1, output_dim=1)
                self.calls = {"step": 0, "h": 0}

            def step(self, x, u, k=0, params=None):
                self.calls["step"] += 1
                return np.array([0.5 * x[0] + u[0]])

            def h(self, x, u, t=0, params=None):
                self.calls["h"] += 1
                return np.array([x[0]])

        block = CountingStep()
        diagram = StepDiagramSystem()
        diagram.add_subsystem(block, "plant")
        diagram.add_input_port("u")
        diagram.connect("input", "u", "plant", "u")
        diagram.compile()
        self.assertEqual(block.calls, {"step": 1, "h": 1})

    def test_wrong_f_shape_raises_from_compute_trajectory(self):
        with self.assertRaises(ValueError):
            self._bad_f().compute_trajectory(tf=1.0, verbose=False)

    def test_wrong_h_shape_raises(self):
        with self.assertRaises(ValueError) as ctx:
            self._bad_h().compile(backend="numpy")
        self.assertIn("h() of", str(ctx.exception))
        self.assertIn("output port 'y'", str(ctx.exception))

    def test_wrong_f_shape_inside_a_diagram(self):
        from minilink.blocks.sources import Step

        with self.assertRaises(ValueError):
            (Step() >> self._bad_f()).compile(backend="numpy")

    @pytest.mark.optional
    @pytest.mark.jax
    def test_wrong_f_shape_raises_on_jax(self):
        pytest.importorskip("jax")
        with self.assertRaises(ValueError):
            self._bad_f().compile(backend="jax")


@pytest.mark.optional
@pytest.mark.jax
@unittest.skipUnless(_JAX_AVAILABLE, "JAX not installed")
class TestRolloutBatch(unittest.TestCase):
    """rollout_batch equals a loop of single rollouts for x0, input, and params families."""

    def setUp(self):
        from minilink.dynamics.catalog.pendulum.pendulum import Pendulum

        self.sys = Pendulum()
        self.ev = self.sys.compile(backend="jax")
        self.rng = np.random.default_rng(0)
        self.x0s = self.rng.standard_normal((4, 2))
        self.dt = 0.02
        self.n_steps = 15
        self.u_nominal = np.tile(self.sys.get_u_from_input_ports(), (self.n_steps, 1))

    def test_family_of_initial_states_under_the_nominal_input(self):
        xs = np.asarray(
            self.ev.rollout_batch(self.x0s, n_steps=self.n_steps, dt=self.dt)
        )
        self.assertEqual(xs.shape, (4, self.n_steps + 1, 2))
        for i in range(4):
            single = self.ev.rollout(self.x0s[i], self.u_nominal, 0.0, self.dt)
            np.testing.assert_allclose(
                xs[i], np.asarray(single), rtol=1e-12, atol=1e-12
            )

    def test_one_input_sequence_per_member(self):
        u_seqs = 0.5 * self.rng.standard_normal((4, self.n_steps, 1))
        xs = np.asarray(self.ev.rollout_batch(self.x0s, u_seqs, dt=self.dt))
        for i in range(4):
            single = self.ev.rollout(self.x0s[i], u_seqs[i], 0.0, self.dt)
            np.testing.assert_allclose(
                xs[i], np.asarray(single), rtol=1e-12, atol=1e-12
            )

    def test_params_family_sweeps_one_leaf(self):
        lengths = np.linspace(0.5, 2.0, 4)
        params = dict(self.sys.params, l=lengths)
        xs = np.asarray(
            self.ev.rollout_batch(
                self.x0s, n_steps=self.n_steps, dt=self.dt, params=params
            )
        )
        for i in range(4):
            p_i = dict(self.sys.params, l=float(lengths[i]))
            single = self.ev.rollout(self.x0s[i], self.u_nominal, 0.0, self.dt, p_i)
            np.testing.assert_allclose(
                xs[i], np.asarray(single), rtol=1e-12, atol=1e-12
            )
        self.assertGreater(np.abs(xs[0] - xs[-1]).max(), 1e-3)

    def test_shape_errors(self):
        with self.assertRaises(ValueError):
            self.ev.rollout_batch(self.x0s, dt=self.dt)
        with self.assertRaises(ValueError):
            self.ev.rollout_batch(self.x0s[:, :1], n_steps=3, dt=self.dt)
        with self.assertRaises(ValueError):
            self.ev.rollout_batch(self.x0s, np.zeros((3, 5, 1)), dt=self.dt)


# The step model: one step body and one rollout body per backend


def step_model_plants():
    from minilink import PID, Pendulum

    return [Pendulum(), PID(Kp=5.0, Ki=1.0, Kd=1.0) @ Pendulum()]


def held_inputs_of(ev, n_steps=12):
    k = np.arange(n_steps)[:, None]
    return 0.3 * np.sin(0.4 * k + np.arange(ev.m)[None, :])


@pytest.mark.parametrize(
    "backend", ["numpy", pytest.param("jax", marks=pytest.mark.jax)]
)
@pytest.mark.parametrize("index", [0, 1], ids=["leaf", "diagram"])
def test_rk4_rollout_is_the_textbook_scheme_step_by_step(backend, index):
    """``rollout`` is ``step`` repeated at ``t_k = t₀ + k dt``, and RK4 is the four-stage formula."""
    if backend == "jax":
        pytest.importorskip("jax")
    ev = step_model_plants()[index].compile(backend)
    x0 = np.linspace(0.1, 0.3, ev.n)
    us = held_inputs_of(ev)
    dt = 0.05

    xs = np.asarray(ev.rollout(x0, us, 0.2, dt))
    x = x0
    for k, u in enumerate(us):
        t = 0.2 + k * dt
        k1 = np.asarray(ev.f(x, u, t))
        k2 = np.asarray(ev.f(x + 0.5 * dt * k1, u, t + 0.5 * dt))
        k3 = np.asarray(ev.f(x + 0.5 * dt * k2, u, t + 0.5 * dt))
        k4 = np.asarray(ev.f(x + dt * k3, u, t + dt))
        x = x + dt / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
        np.testing.assert_allclose(xs[k + 1], x, rtol=1e-12, atol=1e-14)
        np.testing.assert_allclose(
            np.asarray(ev.step(xs[k], u, t, dt=dt)), x, rtol=1e-12, atol=1e-14
        )


@pytest.mark.jax
def test_the_two_backends_agree_on_every_knob():
    pytest.importorskip("jax")
    for plant in step_model_plants():
        ev_np, ev_jax = plant.compile("numpy"), plant.compile("jax")
        x0 = np.linspace(0.1, 0.3, ev_np.n)
        us = held_inputs_of(ev_np)
        for solver in ("rk4", "euler"):
            for substeps in (1, 3):
                for hold in ("zoh", "foh"):
                    seq = us if hold == "zoh" else np.vstack([us, us[-1:]])
                    a = ev_np.rollout(
                        x0, seq, 0.0, 0.05, solver=solver, substeps=substeps, hold=hold
                    )
                    b = ev_jax.rollout(
                        x0, seq, 0.0, 0.05, solver=solver, substeps=substeps, hold=hold
                    )
                    np.testing.assert_allclose(np.asarray(b), a, rtol=1e-12, atol=1e-14)


def test_substeps_converge_to_the_fine_step():
    from minilink import Pendulum

    ev = Pendulum().compile("numpy")
    x0, u = np.array([1.0, 0.0]), np.array([0.2])
    fine = ev.step(x0, u, 0.0, dt=0.4, substeps=40)
    errors = [
        np.max(np.abs(ev.step(x0, u, 0.0, dt=0.4, substeps=s) - fine))
        for s in (1, 2, 4)
    ]
    assert errors[0] > errors[1] > errors[2]


def test_params_reach_the_step_and_none_reads_the_compiled_ones():
    from minilink import Pendulum

    plant = Pendulum()
    ev = plant.compile("numpy")
    x0, u = np.array([0.5, 0.0]), np.array([0.0])
    heavy = {**plant.params, "m": 2.0 * plant.params["m"]}
    np.testing.assert_array_equal(
        ev.step(x0, u, 0.0, dt=0.1), ev.step(x0, u, 0.0, plant.params, dt=0.1)
    )
    assert not np.allclose(
        ev.step(x0, u, 0.0, heavy, dt=0.1), ev.step(x0, u, 0.0, dt=0.1)
    )


def test_a_rollout_without_inputs_holds_the_nominal_input():
    from minilink import Pendulum

    plant = Pendulum()
    plant.inputs["u"].nominal_value = np.array([0.7])
    ev = plant.compile("numpy")
    us = np.full((5, 1), 0.7)
    np.testing.assert_array_equal(
        ev.rollout(np.ones(2), None, 0.0, 0.1, n_steps=5),
        ev.rollout(np.ones(2), us, 0.0, 0.1),
    )


@pytest.mark.jax
def test_the_compiled_forms_are_built_once():
    pytest.importorskip("jax")
    from minilink import Pendulum

    ev = Pendulum().compile("jax")
    first = ev.rollout
    ev.rollout(np.ones(2), np.zeros((3, 1)), 0.0, 0.1)
    assert ev.rollout is first and ev.step is ev.step


def test_the_step_refuses_an_unknown_solver():
    from minilink import Pendulum

    ev = Pendulum().compile("numpy")
    with pytest.raises(ValueError, match="solver"):
        ev.step(np.ones(2), np.zeros(1), 0.0, dt=0.1, solver="scipy")


def test_a_step_returning_a_bare_number_rolls_out():
    class HalfStep(StepSystem):
        def __init__(self):
            super().__init__(n=1, input_dim=1, output_dim=1)

        def step(self, x, u, k=0, params=None):
            x_next = 0.5 * x[0] + u[0]
            return x_next

    rollout = compile(HalfStep()).rollout(np.zeros(1), n_steps=3, u=np.ones(1))

    np.testing.assert_allclose(rollout.x, [[0.0, 1.0, 1.5, 1.75]])


def test_a_diagram_hands_its_blocks_float_inputs():
    class Reciprocal(DynamicSystem):
        def __init__(self):
            super().__init__(n=1, input_dim=1, output_dim=1)

        def f(self, x, u, t=0.0, params=None):
            dx = -x + u**-1
            return dx

    diagram = DiagramSystem()
    diagram.add_subsystem(Reciprocal(), "plant")
    diagram.add_input_port("u", dim=1)
    diagram.connect("input", "u", "plant", "u")
    ev = diagram.compile()

    np.testing.assert_allclose(ev.f(np.zeros(1), np.array([2])), [0.5])


@pytest.mark.jax
def test_every_jax_evaluator_applies_the_float64_policy():
    jax = pytest.importorskip("jax")
    from minilink import Pendulum

    Pendulum().compile("jax")
    jax.config.update("jax_enable_x64", False)
    try:
        ev = Pendulum().compile("jax")
        dx = ev.f(np.ones(2), np.zeros(1))
    finally:
        jax.config.update("jax_enable_x64", True)

    assert dx.dtype == np.float64
