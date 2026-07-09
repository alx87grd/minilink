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
