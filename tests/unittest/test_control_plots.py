"""Frequency and time-response tools on one state-space channel, both plot backends."""

from __future__ import annotations

import unittest

import numpy as np
import pytest

from minilink import InvertedPendulum, Pendulum, TransferFunction
from minilink.analysis import linear
from minilink.analysis.frequency import (
    bode,
    frequency_response,
    margins,
    nyquist,
    plot_bode,
    plot_nyquist,
    plot_pzmap,
    plot_root_locus,
    pzmap,
    root_locus,
    transfer_function,
)
from minilink.analysis.time_response import (
    plot_step_response,
    step_info,
    step_response,
)
from minilink.graphical.common import PlotResult

PLOTLY = pytest.importorskip


def _matrices(tf):
    return tf.A(), tf.B(), tf.C(), tf.D()


class TestLinearCore(unittest.TestCase):
    """Everything is computed from (A, b, c, d)."""

    def setUp(self):
        # G(s) = (s + 2) / ((s + 1)(s + 3))
        self.G = TransferFunction([1.0, 2.0], [1.0, 4.0, 3.0])
        # L(s) = 1 / (s (s + 1)(s + 2)): Gm = 20 log10(6) at sqrt(2) rad/s
        self.L = TransferFunction([1.0], [1.0, 3.0, 2.0, 0.0])

    def test_poles_zeros_gain(self):
        A, B, C, D = _matrices(self.G)
        np.testing.assert_allclose(np.sort(linear.poles(A)), [-3.0, -1.0], atol=1e-9)
        np.testing.assert_allclose(linear.zeros(A, B, C, D), [-2.0], atol=1e-9)
        self.assertAlmostEqual(linear.gain(A, B, C, D), 1.0)
        self.assertEqual(linear.zeros(*_matrices(self.L)).size, 0)

    def test_frequency_response_matches_the_formula(self):
        A, B, C, D = _matrices(self.G)
        w = np.array([0.3, 1.0, 7.0])
        s = 1j * w
        np.testing.assert_allclose(
            linear.frequency_response(A, B, C, D, w), (s + 2) / ((s + 1) * (s + 3))
        )
        self.assertEqual(linear.frequency_range(A, B, C, D), (0.1, 100.0))

    def test_margins_of_a_textbook_loop(self):
        w = np.logspace(-2, 2, 4000)
        m = linear.margins(w, linear.frequency_response(*_matrices(self.L), w))
        self.assertAlmostEqual(m.gain_margin_db, 20 * np.log10(6.0), places=3)
        self.assertAlmostEqual(m.w_phase_crossover, np.sqrt(2.0), places=4)
        self.assertAlmostEqual(m.phase_margin_deg, 53.41, places=1)
        # a stable first-order lag never crosses 0 dB or -180 deg
        m = linear.margins(w, linear.frequency_response(*_matrices(self.G), w))
        self.assertTrue(np.isinf(m.gain_margin_db) and np.isinf(m.phase_margin_deg))

    def test_root_locus_branches_are_continuous_and_closed_loop_exact(self):
        A, B, C, D = _matrices(self.L)
        gains, roots = linear.root_locus(A, B, C, D)
        self.assertEqual(roots.shape[1], 3)
        self.assertEqual(gains[0], 0.0)
        np.testing.assert_allclose(np.sort(roots[0].real), [-2.0, -1.0, 0.0], atol=1e-9)
        self.assertLess(np.max(np.abs(np.diff(roots, axis=0))), 0.5)  # no branch jump
        K = gains[len(gains) // 2]
        expected = np.linalg.eigvals(A - K * B @ C)
        np.testing.assert_allclose(
            np.sort_complex(roots[len(gains) // 2]),
            np.sort_complex(expected),
            atol=1e-9,
        )
        # instability past K = 6 shows up as a branch crossing into Re > 0
        self.assertTrue(np.any(roots[gains > 6.5].real > 0.0))
        self.assertTrue(np.all(roots[(gains > 0.0) & (gains < 5.5)].real <= 1e-9))

    def test_root_locus_stops_when_every_branch_reaches_a_zero(self):
        A, B, C, D = _matrices(
            TransferFunction([1.0, 2.0, 1.0], [1.0, 4.0, 3.0])
        )  # relative degree 0
        gains, roots = linear.root_locus(A, B, C, D)
        np.testing.assert_allclose(np.sort(roots[-1].real), [-1.0, -1.0], atol=1e-2)

    def test_step_response_is_exact(self):
        A, B, C, D = _matrices(TransferFunction([1.0], [1.0, 1.0]))
        t = np.linspace(0.0, 5.0, 101)
        np.testing.assert_allclose(
            linear.step_response(A, B, C, D, t), 1.0 - np.exp(-t), atol=1e-12
        )
        self.assertAlmostEqual(linear.settling_horizon(A), 8.0)

    def test_step_info_second_order(self):
        zeta, wn = 0.3, 2.0
        tf = TransferFunction([wn**2], [1.0, 2 * zeta * wn, wn**2])
        t = np.linspace(0.0, 15.0, 3001)
        info = step_info(t, linear.step_response(*_matrices(tf), t))
        expected = 100.0 * np.exp(-np.pi * zeta / np.sqrt(1 - zeta**2))
        self.assertAlmostEqual(info.overshoot, expected, places=1)
        self.assertAlmostEqual(info.steady_state, 1.0, places=3)
        self.assertAlmostEqual(
            info.peak_time, np.pi / (wn * np.sqrt(1 - zeta**2)), places=2
        )
        self.assertGreater(info.settling_time, info.rise_time)


class TestChannelTools(unittest.TestCase):
    """The system-level verbs share the family signature and the channel selectors."""

    def test_family_signature_on_a_nonlinear_plant(self):
        plant = Pendulum()
        plant.params["d"] = 0.5
        x_bar = [0.0, 0.0]
        w, G = frequency_response(plant, x_bar, of=("y", 1), w=[1.0])
        _, mag, phase = bode(plant, x_bar, of=("y", 1), w=[1.0])
        np.testing.assert_allclose(mag, 20 * np.log10(np.abs(G)))
        z, p, k = pzmap(plant, x_bar)
        np.testing.assert_allclose(
            np.sort_complex(p),
            np.sort_complex(np.linalg.eigvals(plant.linearize(x_bar).A())),
        )
        self.assertAlmostEqual(k, 0.5)
        gains, roots = root_locus(plant, x_bar)
        self.assertEqual(roots.shape[1], 2)
        w_n, G_n = nyquist(plant, x_bar, n=50)
        self.assertEqual(G_n.shape, (50,))
        time, y = step_response(plant, x_bar, n=200)
        self.assertEqual(time.shape, y.shape)
        self.assertIsInstance(margins(plant, x_bar), linear.Margins)
        G_tf = transfer_function(plant, x_bar)
        A = plant.linearize(x_bar).A()
        np.testing.assert_allclose(
            G_tf.denominator, [1.0, -np.trace(A), np.linalg.det(A)], atol=1e-9
        )
        np.testing.assert_allclose(G_tf.numerator, [0.5], atol=1e-9)

    def test_methods_mirror_the_functions(self):
        plant = InvertedPendulum()
        x_bar = [0.0, 0.0]
        np.testing.assert_allclose(
            plant.root_locus(x_bar)[1], root_locus(plant, x_bar)[1]
        )
        np.testing.assert_allclose(
            plant.nyquist(x_bar, w=[1.0, 2.0])[1],
            nyquist(plant, x_bar, w=[1.0, 2.0])[1],
        )
        self.assertEqual(plant.margins(x_bar), margins(plant, x_bar))
        np.testing.assert_allclose(
            plant.step_response(x_bar, tf=1.0, n=20)[1],
            step_response(plant, x_bar, tf=1.0, n=20)[1],
        )

    def test_transfer_function_blocks_go_through_unchanged(self):
        L = TransferFunction([1.0], [1.0, 3.0, 2.0, 0.0])
        m = L.margins()
        self.assertAlmostEqual(m.gain_margin_db, 20 * np.log10(6.0), places=2)
        gains, roots = L.root_locus()
        self.assertEqual(roots.shape[1], 3)


class TestPlots(unittest.TestCase):
    """Every control plot renders on both backends from one figure spec."""

    def setUp(self):
        self.plant = Pendulum()
        self.plant.params["d"] = 0.5
        self.x_bar = [0.0, 0.0]
        self.loop = TransferFunction([1.0], [1.0, 3.0, 2.0, 0.0])

    def tearDown(self):
        import matplotlib.pyplot as plt

        plt.close("all")

    def test_matplotlib_titles_labels_and_axes(self):
        r = plot_bode(self.plant, self.x_bar, of=("y", 1), show=False)
        self.assertIsInstance(r, PlotResult)
        self.assertEqual(len(r.axes), 2)
        self.assertEqual(r.figure._suptitle.get_text(), "Bode Diagram")
        self.assertEqual(r.axes[0].get_title(), "From: u[0]  To: y[1]")
        self.assertEqual(r.axes[0].get_ylabel(), "Magnitude (dB)")
        self.assertEqual(r.axes[1].get_xlabel(), "Frequency (rad/s)")
        self.assertEqual(r.axes[0].get_xscale(), "log")
        r = plot_pzmap(self.plant, self.x_bar, show=False)
        self.assertEqual(r.figure._suptitle.get_text(), "Pole-Zero Map")
        self.assertEqual(len(r.axes.lines), 2 + 2)  # poles, zeros, two axis lines
        r = plot_root_locus(self.plant, self.x_bar, show=False)
        self.assertEqual(r.figure._suptitle.get_text(), "Root Locus")
        r = plot_nyquist(self.loop, show=False)
        self.assertEqual(r.figure._suptitle.get_text(), "Nyquist Diagram")
        r = plot_step_response(self.loop, show=False)
        self.assertEqual(r.figure._suptitle.get_text(), "Step Response")
        self.assertEqual(r.axes.get_xlabel(), "Time (seconds)")

    def test_margins_are_drawn_on_the_bode_plot(self):
        r = plot_bode(self.loop, show=False)
        texts = [t.get_text() for t in r.axes[0].texts]
        self.assertTrue(
            any("Gm = 15.6 dB" in t and "Pm = 53.4 deg" in t for t in texts), texts
        )
        self.assertEqual(len(r.axes[0].lines), 1 + 2 + 1)  # curve, two crossovers, 0 dB
        r = plot_bode(self.loop, margins=False, show=False)
        self.assertEqual(len(r.axes[0].lines), 1)

    def test_plotly_backend_draws_the_same_figures(self):
        pytest.importorskip("plotly")
        for plot, sys, expected in (
            (plot_bode, self.plant, 2),
            (plot_pzmap, self.plant, 2),
            (plot_root_locus, self.plant, 2 + 2),
            (plot_nyquist, self.loop, 3),
            (plot_step_response, self.loop, 2),
        ):
            r = plot(sys, backend="plotly", show=False)
            self.assertEqual(r.backend, "plotly")
            self.assertEqual(len(r.figure.data), expected, plot.__name__)
        r = plot_bode(self.plant, backend="plotly", show=False)
        self.assertEqual(r.figure.layout.xaxis.type, "log")
        self.assertIn("Bode Diagram", r.figure.layout.title.text)
        self.assertIn("From: u[0]", r.figure.layout.title.text)
        self.assertTrue(r.figure.data[0].hovertext[0].startswith("ω ="))
        r = plot_root_locus(self.plant, backend="plotly", show=False)
        self.assertIn("K =", r.figure.data[0].hovertext[1])

    def test_unknown_backend_is_refused(self):
        with self.assertRaisesRegex(ValueError, "Unknown plot backend"):
            plot_bode(self.plant, backend="bokeh", show=False)

    def test_phase_plane_on_plotly(self):
        pytest.importorskip("plotly")
        self.plant.x0 = np.array([0.5, 0.0])
        traj = self.plant.compute_trajectory(tf=1.0, verbose=False)
        r = self.plant.plot_phase_plane(traj, backend="plotly", show=False)
        self.assertEqual(r.backend, "plotly")
        names = [trace.name for trace in r.figure.data]
        self.assertTrue({"trajectory", "start", "end"} <= set(names), names)
        r = self.plant.plot_phase_plane(backend="plotly", streamplot=True, show=False)
        self.assertEqual(r.backend, "plotly")  # the cached trajectory is overlaid again


if __name__ == "__main__":
    unittest.main()
