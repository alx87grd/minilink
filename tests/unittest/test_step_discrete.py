"""StepSystem leaf contract tests."""

import unittest

import numpy as np

from minilink.blocks.step import ZOHHold
from minilink.core.system import StepSystem


class Accumulator(StepSystem):
    def __init__(self):
        super().__init__(n=1, input_dim=1, output_dim=1)
        self.x0 = np.zeros(1)

    def step(self, x, u, k=0, params=None):
        return np.array([x[0] + u[0]])

    def h(self, x, u, k=0, params=None):
        return np.array([x[0]])


class TestStepSystem(unittest.TestCase):
    def test_step_returns_next_state(self):
        plant = Accumulator()
        x1 = plant.step(np.array([1.0]), np.array([2.0]), k=3)
        np.testing.assert_allclose(x1, [3.0])

    def test_h_uses_k_slot(self):
        plant = Accumulator()
        y = plant.h(np.array([4.0]), np.array([0.0]), k=5)
        np.testing.assert_allclose(y, [4.0])

    def test_no_f_on_stepsystem(self):
        from minilink.core.compile.compiler import compile

        plant = Accumulator()
        self.assertFalse(hasattr(plant, "f"))
        ev = compile(plant)
        self.assertFalse(hasattr(ev, "f"))

    def test_rollout_attr_initialized(self):
        plant = Accumulator()
        self.assertIsNone(plant.rollout)

    def test_zoh_hold_latches_command(self):
        hold = ZOHHold()
        x1 = hold.step(np.array([0.0]), np.array([2.5]), k=0)
        np.testing.assert_allclose(x1, [2.5])
        np.testing.assert_allclose(hold.h(np.array([1.0]), np.array([0.0]), k=1), [1.0])

    def test_n_must_be_positive(self):
        with self.assertRaises(ValueError):
            StepSystem(0)


if __name__ == "__main__":
    unittest.main()


# --- merged from test_step_diagram.py ---

"""Tests for StepDiagramSystem wiring and interpreted step."""
import unittest
import numpy as np
from minilink.blocks.basic import Integrator
from minilink.blocks.routing import Gain
from minilink.core.backends import array_module
from minilink.core.compile.compiler import compile
from minilink.core.compile.evaluators.numpy_evaluators import NumpyStepDiagramEvaluator
from minilink.core.diagram import StepDiagramSystem
from minilink.core.system import StepSystem, System


class KDependentPort(System):
    """Static block whose output depends on step index ``k`` in the third slot."""

    def __init__(self):
        super().__init__()
        self.add_input_port("u", dim=1)
        self.add_output_port("y", dim=1, function=self.compute, dependencies=("u",))

    def compute(self, x, u, t=0, params=None):
        xp = array_module(u)
        k = int(t)
        return xp.array([u[0] + float(k)])


class Accumulator_step_diagram(StepSystem):
    def __init__(self):
        super().__init__(n=1, input_dim=1, output_dim=1, y_dependencies=())
        self.x0 = np.zeros(1)

    def step(self, x, u, k=0, params=None):
        xp = array_module(x, u)
        x = xp.asarray(x, dtype=float).reshape(1)
        u = xp.asarray(u, dtype=float).reshape(1)
        return xp.array([x[0] + u[0]])

    def h(self, x, u, k=0, params=None):
        xp = array_module(x)
        return xp.asarray(x, dtype=float).reshape(1).copy()


def _build_gain_plant_loop():
    diagram = StepDiagramSystem()
    diagram.add_subsystem(Gain(1.0, dim=1), "gain")
    diagram.add_subsystem(Accumulator_step_diagram(), "plant")
    diagram.add_input_port("u")
    diagram.connect("input", "u", "gain", "u")
    diagram.connect("gain", "y", "plant", "u")
    return diagram


class TestStepDiagram(unittest.TestCase):
    def test_is_step_system(self):
        diagram = StepDiagramSystem()
        self.assertIsInstance(diagram, StepSystem)

    def test_interpreted_step_matches_manual(self):
        diagram = _build_gain_plant_loop()
        x = np.zeros(1)
        u_boundary = np.array([1.0])
        x_manual = 0.0
        for k in range(5):
            u_eff = 1.0 * u_boundary[0]
            x_manual = x_manual + u_eff
            x = diagram.step(x, u_boundary, k=k)
        self.assertAlmostEqual(float(x[0]), x_manual)

    def test_k_passed_to_static_port(self):
        diagram = StepDiagramSystem()
        diagram.add_subsystem(KDependentPort(), "blk")
        diagram.add_input_port("u")
        diagram.connect("input", "u", "blk", "u")
        diagram.connect_new_output_port("blk", "y", "y")
        y = diagram.compute_subsys_output_port(
            np.array([]), np.array([2.0]), 3, "blk", "y"
        )
        self.assertAlmostEqual(float(y[0]), 5.0)

    def test_reject_dynamic_subsystem_at_compile(self):
        diagram = StepDiagramSystem()
        diagram.add_subsystem(Integrator(), "plant")
        with self.assertRaises(TypeError):
            diagram.compile()

    def test_compile_returns_step_diagram_evaluator(self):
        diagram = _build_gain_plant_loop()
        ev = compile(diagram)
        self.assertIsInstance(ev, NumpyStepDiagramEvaluator)

    def test_compiled_step_matches_interpreted(self):
        diagram = _build_gain_plant_loop()
        ev = diagram.compile()
        x = np.zeros(1)
        u = np.array([0.5])
        for k in range(8):
            x_ref = diagram.step(x, u, k=k)
            x_ev = ev.step(x, u, k=k)
            np.testing.assert_allclose(x_ev, x_ref)
            x = x_ev


# --- merged from test_step_diagram_topology.py ---

"""Tests for step-diagram topology export."""
import unittest
from minilink.core.backends import array_module
from minilink.core.diagram import StepDiagramSystem
from minilink.core.system import StepSystem
from minilink.graphical.diagrams import build_diagram_topology
from minilink.graphical.diagrams.dot import block_html


class Accumulator_step_diagram_topology(StepSystem):
    def __init__(self):
        super().__init__(n=1, input_dim=1, output_dim=1, y_dependencies=())
        self.x0 = array_module().zeros(1)

    def step(self, x, u, k=0, params=None):
        xp = array_module(x, u)
        x = xp.asarray(x, dtype=float).reshape(1)
        u = xp.asarray(u, dtype=float).reshape(1)
        return xp.array([x[0] + u[0]])

    def h(self, x, u, k=0, params=None):
        xp = array_module(x)
        return xp.asarray(x, dtype=float).reshape(1).copy()


class TestStepDiagramTopology(unittest.TestCase):
    def test_step_leaf_kind(self):
        diagram = StepDiagramSystem()
        diagram.add_subsystem(Accumulator_step_diagram_topology(), "acc")
        diagram.add_input_port("u")
        diagram.connect("input", "u", "acc", "u")
        diagram.connect_new_output_port("acc", "y", "y")
        topology = build_diagram_topology(diagram)
        kinds = {node.id: node.kind for node in topology.nodes}
        self.assertEqual(kinds["acc"], "step_system")

    def test_block_html_step_prefix(self):
        diagram = StepDiagramSystem()
        diagram.add_subsystem(Accumulator_step_diagram_topology(), "acc")
        topology = build_diagram_topology(diagram)
        acc_node = next((node for node in topology.nodes if node.id == "acc"))
        html = block_html(acc_node)
        self.assertIn("Step:", html)


if __name__ == "__main__":
    unittest.main()


# --- merged from test_step_diagram_rollout.py ---

"""Rollout parity tests for compiled step diagrams."""

import unittest

import numpy as np

from minilink.control.output import ProportionalController
from minilink.core.backends import array_module
from minilink.core.compile.compiler import compile
from minilink.core.diagram import StepDiagramSystem
from minilink.core.system import StepSystem


class DiscreteAccumulator(StepSystem):
    def __init__(self):
        super().__init__(n=1, input_dim=1, output_dim=1, y_dependencies=())
        self.name = "DiscreteAccumulator"
        self.x0 = np.zeros(1)

    def step(self, x, u, k=0, params=None):
        xp = array_module(x, u)
        x = xp.asarray(x, dtype=float).reshape(1)
        u = xp.asarray(u, dtype=float).reshape(1)
        return xp.array([x[0] + u[0]])

    def h(self, x, u, k=0, params=None):
        xp = array_module(x)
        return xp.asarray(x, dtype=float).reshape(1).copy()


def _build_unity_feedback(K=0.3):
    diagram = StepDiagramSystem()
    ctl = ProportionalController(K)
    plant = DiscreteAccumulator()
    diagram.add_subsystem(ctl, "ctl")
    diagram.add_subsystem(plant, "plant")
    diagram.add_input_port("r")
    diagram.connect("input", "r", "ctl", "r")
    diagram.connect("plant", "y", "ctl", "y")
    diagram.connect("ctl", "u", "plant", "u")
    diagram.connect_new_output_port("plant", "y", "y")
    return diagram


def _manual_unity_feedback_rollout(r_value, K, n_steps, x0=0.0):
    x = float(x0)
    xs = [x]
    for _k in range(n_steps):
        y = x
        u = float(K) * (r_value - y)
        x = x + u
        xs.append(x)
    k = np.arange(n_steps + 1, dtype=float)
    x_arr = np.array(xs, dtype=float).reshape(1, -1)
    return k, x_arr


class TestStepDiagramRollout(unittest.TestCase):
    def test_rollout_parity_unity_feedback(self):
        diagram = _build_unity_feedback(K=0.3)
        r = 1.0
        n_steps = 25
        k_ref, x_ref = _manual_unity_feedback_rollout(r, 0.3, n_steps)

        ev = compile(diagram)
        rollout = ev.rollout(diagram.x0, n_steps=n_steps, u=np.array([r]))

        np.testing.assert_allclose(rollout.k, k_ref)
        np.testing.assert_allclose(rollout.x, x_ref, rtol=1e-10, atol=1e-10)
        # ``rollout.u`` is the diagram boundary input (reference ``r``), not plant command.
        np.testing.assert_allclose(rollout.u[0, :], r)

    def test_compute_rollout_matches_compile_rollout(self):
        diagram = _build_unity_feedback(K=0.5)
        n_steps = 15
        r = 2.0

        via_facade = diagram.compute_rollout(n_steps=n_steps, u=np.array([r]))
        via_eval = diagram.compile().rollout(
            diagram.x0, n_steps=n_steps, u=np.array([r])
        )

        np.testing.assert_allclose(via_facade.x, via_eval.x)
        np.testing.assert_allclose(via_facade.u, via_eval.u)

    def test_interpreted_step_track_rollout(self):
        diagram = _build_unity_feedback(K=0.4)
        r = 1.0
        n_steps = 10
        x = diagram.x0.copy()
        u_boundary = np.array([r])
        for k in range(n_steps):
            x = diagram.step(x, u_boundary, k=k)

        rollout = diagram.compile().rollout(diagram.x0, n_steps=n_steps, u=u_boundary)
        np.testing.assert_allclose(x, rollout.x[:, -1], rtol=1e-10, atol=1e-10)


# --- merged from test_step_rollout.py ---

"""StepRollout container, gather_u, and evaluator rollout tests."""

import unittest

import numpy as np
import pytest

from minilink.core.compile.compiler import compile
from minilink.core.compile.evaluators.step_rollout import gather_u
from minilink.core.compile.execution_plan import (
    EXTERNAL_INPUT,
    INTERNAL_SIGNAL,
    NOMINAL,
)
from minilink.core.step_rollout import StepRollout
from minilink.core.system import StepSystem
from minilink.core.trajectory import Trajectory

try:
    import jax  # noqa: F401

    _JAX_AVAILABLE = True
except ImportError:
    _JAX_AVAILABLE = False


class AffineStep(StepSystem):
    def __init__(self):
        super().__init__(n=1, input_dim=1, output_dim=1)
        self.x0 = np.array([0.0])

    def step(self, x, u, k=0, params=None):
        return np.array([x[0] + u[0] + float(k)])

    def h(self, x, u, k=0, params=None):
        return np.array([x[0]])


class TestGatherU(unittest.TestCase):
    def test_nominal_source(self):
        sources = ((NOMINAL, np.array([1.0, 2.0]), 2),)
        local_u = gather_u(sources, 2, np.zeros(0), np.zeros(0))
        np.testing.assert_allclose(local_u, [1.0, 2.0])

    def test_external_input_source(self):
        sources = ((EXTERNAL_INPUT, slice(1, 2), 1), (EXTERNAL_INPUT, slice(0, 2), 2))
        boundary_u = np.array([3.0, 4.0, 5.0])
        local_u = gather_u(sources, 3, np.zeros(0), boundary_u)
        np.testing.assert_allclose(local_u, [4.0, 3.0, 4.0])

    def test_internal_signal_source(self):
        sources = ((INTERNAL_SIGNAL, slice(2, 3), 1), (INTERNAL_SIGNAL, slice(0, 2), 2))
        signals = np.array([9.0, 8.0, 7.0])
        local_u = gather_u(sources, 3, signals, np.zeros(0))
        np.testing.assert_allclose(local_u, [7.0, 9.0, 8.0])

    def test_mixed_sources(self):
        sources = (
            (NOMINAL, np.array([0.5]), 1),
            (EXTERNAL_INPUT, slice(0, 1), 1),
            (INTERNAL_SIGNAL, slice(1, 2), 1),
        )
        signals = np.array([1.0, 2.0])
        boundary_u = np.array([3.0])
        local_u = gather_u(sources, 3, signals, boundary_u)
        np.testing.assert_allclose(local_u, [0.5, 3.0, 2.0])

    def test_zero_dim_returns_empty(self):
        local_u = gather_u((), 0, np.zeros(0), np.zeros(0))
        self.assertEqual(local_u.shape, (0,))

    def test_unknown_source_raises(self):
        with self.assertRaises(RuntimeError):
            gather_u(((99, 0, 1),), 1, np.zeros(1), np.zeros(1))


class TestStepRollout(unittest.TestCase):
    def test_shapes(self):
        rollout = StepRollout(
            k=np.arange(4, dtype=float),
            x=np.zeros((1, 4)),
            u=np.zeros((1, 4)),
        )
        self.assertEqual(rollout.n_samples, 4)
        self.assertEqual(rollout.n, 1)
        self.assertEqual(rollout.m, 1)

    def test_as_trajectory(self):
        rollout = StepRollout(
            k=np.array([0.0, 1.0, 2.0]),
            x=np.array([[0.0, 1.0, 3.0]]),
            u=np.array([[0.0, 1.0, 1.0]]),
        )
        traj = rollout.as_trajectory()
        self.assertIsInstance(traj, Trajectory)
        np.testing.assert_allclose(traj.t, [0.0, 1.0, 2.0])

    def test_rollout_constant_u(self):
        plant = AffineStep()
        ev = compile(plant)
        rollout = ev.rollout(plant.x0, n_steps=3, u=np.array([1.0]))
        self.assertEqual(rollout.x.shape, (1, 4))
        self.assertEqual(rollout.u.shape, (1, 4))
        np.testing.assert_allclose(rollout.x[0], [0.0, 1.0, 3.0, 6.0])

    def test_rollout_sequence_u(self):
        plant = AffineStep()
        ev = compile(plant)
        u_seq = np.array([[1.0], [0.0], [2.0]])
        rollout = ev.rollout(plant.x0, n_steps=3, u=u_seq)
        np.testing.assert_allclose(rollout.u[0, :3], [1.0, 0.0, 2.0])
        np.testing.assert_allclose(rollout.u[0, 3], 2.0)

    def test_rollout_callable_u(self):
        plant = AffineStep()
        ev = compile(plant)

        def u_of_k(k):
            return np.array([float(k)])

        rollout = ev.rollout(plant.x0, n_steps=2, u=u_of_k)
        np.testing.assert_allclose(rollout.u[0, :2], [0.0, 1.0])

    def test_rollout_state_only_by_default(self):
        plant = AffineStep()
        ev = compile(plant)
        rollout = ev.rollout(plant.x0, n_steps=2, u=np.array([0.0]))
        self.assertEqual(len(rollout.signals), 0)


@pytest.mark.optional
@pytest.mark.jax
@unittest.skipUnless(_JAX_AVAILABLE, "JAX not installed")
class TestStepRolloutJax(unittest.TestCase):
    def test_jax_rollout_scan_smoke(self):
        from minilink.core.backends import array_module

        class JaxFriendlyStep(StepSystem):
            def __init__(self):
                super().__init__(n=1, input_dim=1)
                self.x0 = np.array([0.0])

            def step(self, x, u, k=0, params=None):
                xp = array_module(x)
                x = xp.asarray(x, dtype=float).reshape(1)
                u = xp.asarray(u, dtype=float).reshape(1)
                return xp.array([x[0] + u[0]])

        plant = JaxFriendlyStep()
        ev = compile(plant, backend="jax")
        rollout = ev.rollout(plant.x0, n_steps=4, u=np.array([1.0]))
        ev_np = compile(plant, backend="numpy")
        ref = ev_np.rollout(plant.x0, n_steps=4, u=np.array([1.0]))
        np.testing.assert_allclose(rollout.x, ref.x, atol=1e-5)


if __name__ == "__main__":
    unittest.main()


# --- merged from test_step_diagram_jax.py ---

"""JAX compile smoke for step diagrams."""
import unittest
import numpy as np
import pytest
from minilink.blocks.routing import Gain
from minilink.core.backends import array_module
from minilink.core.compile.compiler import compile
from minilink.core.compile.evaluators.jax_evaluators import JaxStepDiagramEvaluator
from minilink.core.diagram import StepDiagramSystem
from minilink.core.system import StepSystem

try:
    import jax

    _JAX_AVAILABLE = True
except ImportError:
    _JAX_AVAILABLE = False


class Accumulator_step_diagram_jax(StepSystem):
    def __init__(self):
        super().__init__(n=1, input_dim=1, output_dim=1, y_dependencies=())
        self.x0 = np.zeros(1)

    def step(self, x, u, k=0, params=None):
        xp = array_module(x, u)
        return xp.array([x[0] + u[0]])

    def h(self, x, u, k=0, params=None):
        xp = array_module(x)
        return xp.asarray(x, dtype=float).reshape(1)


@pytest.mark.skipif(not _JAX_AVAILABLE, reason="JAX not installed")
class TestStepDiagramJax(unittest.TestCase):
    def test_jax_step_diagram_rollout(self):
        diagram = StepDiagramSystem()
        diagram.add_subsystem(Gain(1.0, dim=1), "gain")
        diagram.add_subsystem(Accumulator_step_diagram_jax(), "plant")
        diagram.add_input_port("u")
        diagram.connect("input", "u", "gain", "u")
        diagram.connect("gain", "y", "plant", "u")
        ev = compile(diagram, backend="jax")
        self.assertIsInstance(ev, JaxStepDiagramEvaluator)
        rollout = ev.rollout(diagram.x0, n_steps=5, u=np.array([1.0]))
        np_ev = compile(diagram, backend="numpy")
        ref = np_ev.rollout(diagram.x0, n_steps=5, u=np.array([1.0]))
        np.testing.assert_allclose(rollout.x, ref.x, rtol=1e-10, atol=1e-10)


# --- merged from test_facades_rollout.py ---

"""Step rollout facade tests."""

import unittest

import numpy as np

from minilink.blocks.basic import Integrator
from minilink.core.system import StepSystem


class Counter(StepSystem):
    def __init__(self):
        super().__init__(n=1)
        self.x0 = np.zeros(1)

    def step(self, x, u, k=0, params=None):
        return np.array([x[0] + 1.0])


class TestFacadesRollout(unittest.TestCase):
    def test_compute_rollout_caches_on_system(self):
        plant = Counter()
        rollout = plant.compute_rollout(n_steps=5)
        self.assertIs(plant.rollout, rollout)
        self.assertEqual(rollout.n_samples, 6)
        np.testing.assert_allclose(rollout.x[0, -1], 5.0)
        self.assertEqual(len(rollout.signals), 0)

    def test_plot_rollout_headless(self):
        import matplotlib

        matplotlib.use("Agg")
        plant = Counter()
        rollout = plant.compute_rollout(n_steps=3)
        result = plant.plot_rollout(rollout, show=False)
        self.assertEqual(result.backend, "matplotlib")
        fig, axes = result.payload
        self.assertEqual(axes[-1].get_xlabel(), "Step [k]")

    def test_compute_rollout_only_on_step_system(self):
        with self.assertRaises(AttributeError):
            Integrator().compute_rollout(n_steps=3)


if __name__ == "__main__":
    unittest.main()
