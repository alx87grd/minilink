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


if __name__ == "__main__":
    unittest.main()
