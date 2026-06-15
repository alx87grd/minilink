"""Unit tests for modal analysis."""

import os
import unittest

import numpy as np
import pytest

from minilink.analysis.linearize import linearize
from minilink.analysis.modal import animate_modal, modal_analysis
from minilink.dynamics.abstraction.state_space import LTISystem
from minilink.dynamics.catalog.mass_spring_damper.linear import TwoMass
from minilink.dynamics.catalog.pendulum.pendulum import InvertedPendulum, Pendulum


class TestModalAnalysis(unittest.TestCase):
    def test_two_mass_has_four_modes(self):
        poles, modes = modal_analysis(TwoMass(), x_bar=np.zeros(4))
        self.assertEqual(len(poles), 4)

    def test_pendulum_has_two_modes(self):
        poles, modes = modal_analysis(Pendulum(), x_bar=[0.0, 0.0])
        self.assertEqual(len(poles), 2)

    def test_oscillator_poles(self):
        w = 2.0
        A = np.array([[0.0, 1.0], [-w**2, 0.0]])
        poles, modes = modal_analysis(LTISystem(A, np.zeros((2, 1))), x_bar=[0.0, 0.0])
        np.testing.assert_allclose(np.sort(poles), np.sort([1j * w, -1j * w]), atol=1e-10)


class TestAnimateModal(unittest.TestCase):
    def test_mode_trajectory_starts_at_eigenvector(self):
        x_bar = np.array([0.0, 0.0])
        poles, modes = modal_analysis(Pendulum(), x_bar)
        time = np.linspace(0.0, 2.0, 101)
        delta_x = 0.5 * np.real(modes[:, 0:1] * np.exp(poles[0] * time))
        expected = x_bar + delta_x[:, 0]
        np.testing.assert_allclose(expected, x_bar + 0.5 * np.real(modes[:, 0]), atol=1e-10)

    @pytest.mark.optional
    def test_animate_one_mode_headless(self):
        os.environ.setdefault("MPLBACKEND", "Agg")
        poles, modes = animate_modal(Pendulum(), [0.0, 0.0], 0, show=False)
        self.assertEqual(len(poles), 2)

    @pytest.mark.optional
    def test_animate_all_modes(self):
        os.environ.setdefault("MPLBACKEND", "Agg")
        poles, modes = animate_modal(Pendulum(), [0.0, 0.0], "all", show=False)
        self.assertEqual(len(poles), 2)


class TestModalFacade(unittest.TestCase):
    def test_returns_poles_and_modes(self):
        poles, modes = Pendulum().modal_analysis(x_bar=[0.0, 0.0])
        self.assertEqual(len(poles), 2)

    @pytest.mark.optional
    def test_facade_animate(self):
        os.environ.setdefault("MPLBACKEND", "Agg")
        poles, modes = Pendulum().modal_analysis(x_bar=[0.0, 0.0], mode=0, show=False)
        self.assertEqual(len(poles), 2)


@pytest.mark.optional
@pytest.mark.jax
class TestModalJAXLinearization(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        pytest.importorskip("jax")

    def test_jax_poles_match_fd(self):
        plant = Pendulum()
        x_bar = [0.0, 0.0]
        fd_poles, _ = modal_analysis(plant, x_bar, linearization="fd")
        jax_poles, _ = modal_analysis(plant, x_bar, linearization="jax")
        np.testing.assert_allclose(np.sort(fd_poles), np.sort(jax_poles), atol=1e-4)
        fd_lti = linearize(plant, x_bar, linearization="fd")
        jax_lti = linearize(plant, x_bar, linearization="jax")
        np.testing.assert_allclose(fd_lti.A(), jax_lti.A(), atol=1e-4)


class TestInvertedPendulumModes(unittest.TestCase):
    def test_upright_has_unstable_real_mode(self):
        poles, modes = modal_analysis(InvertedPendulum(), x_bar=[0.0, 0.0])
        self.assertTrue(any(pole.real > 0.0 for pole in poles))
