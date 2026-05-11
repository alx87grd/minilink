import contextlib
import importlib
import io
import unittest

import matplotlib.pyplot as plt
import numpy as np
import pytest

from minilink.core.diagram import DiagramSystem
from minilink.core.trajectory import Trajectory
from minilink.core.system import DynamicSystem, StaticSystem
from minilink.graphical.plotly_style import PLOTLY_FIG_WIDTH
from minilink.graphical.plotting import (
    build_signal_plot_spec,
    open_time_signal_plot,
    plot_time_signals,
)
from minilink.simulation.simulator import Simulator


class Integrator(DynamicSystem):
    def __init__(self):
        super().__init__(1, 1, 1)
        self.add_output_port(1, "y", function=self.h, dependencies=[])

    def f(self, x, u, t=0, params=None):
        return np.array([u[0]])

    def h(self, x, u, t=0, params=None):
        return np.array([x[0]])


class PropController(StaticSystem):
    def __init__(self):
        super().__init__(2, 1)
        self.params = {"Kp": 5.0}
        self.add_input_port(1, "ref", nominal_value=np.array([1.0]))
        self.add_input_port(1, "y", nominal_value=np.array([0.0]))
        self.add_output_port(1, "u", function=self.ctl, dependencies=["ref", "y"])

    def ctl(self, x, u, t=0, params=None):
        Kp = params["Kp"] if params else self.params["Kp"]
        return np.array([Kp * (u[0] - u[1])])


class Step(StaticSystem):
    def __init__(self):
        super().__init__(0, 1)
        self.add_output_port(1, "y", function=self.compute)

    def compute(self, x, u, t=0, params=None):
        return np.array([1.0])


class TestAdvancedPlotting(unittest.TestCase):
    def setUp(self):
        self.sys = Integrator()
        self.ctl = PropController()
        self.step = Step()

        self.diagram = DiagramSystem()
        self.diagram.add_subsystem(self.step, "step")
        self.diagram.add_subsystem(self.ctl, "ctl")
        self.diagram.add_subsystem(self.sys, "plant")

        self.diagram.connect("step", "y", "ctl", "ref")
        self.diagram.connect("ctl", "u", "plant", "u")
        self.diagram.connect("plant", "y", "ctl", "y")

        self.sim = Simulator(self.diagram, t0=0, tf=2.0, dt=0.1, verbose=False)
        self.traj = self.sim.solve()

    def test_plotting_import_is_quiet(self):
        import minilink.graphical.plotting as plotting

        interactive = plt.isinteractive()
        stream = io.StringIO()
        with contextlib.redirect_stdout(stream):
            importlib.reload(plotting)
        self.assertEqual(stream.getvalue(), "")
        self.assertEqual(plt.isinteractive(), interactive)

    def test_compute_internal_signals(self):
        # By default, the base trajectory does not carry reconstructed signals
        self.assertFalse(self.traj.has_signal("step:y"))

        # Reconstruct signals
        traj_plus = self.diagram.compute_internal_signals(self.traj)

        # Test it successfully created sampled channels
        self.assertTrue(traj_plus.has_signal("step:y"))
        self.assertTrue(traj_plus.has_signal("ctl:u"))
        self.assertTrue(traj_plus.has_signal("plant:y"))

        # Check shapes (dim 1, n_pts time steps)
        n_pts = len(self.traj.t)
        self.assertEqual(traj_plus.get_signal("ctl:u").shape, (1, n_pts))

    def test_plot_time_signals_does_not_crash(self):
        from minilink.graphical.environment import override_env

        override_env("jupyter")
        self.addCleanup(override_env, None)

        result = plot_time_signals(
            self.diagram,
            self.traj,
            signals=("x", "ctl:u", "plant:y"),
            show=False,
        )
        self.assertIsNotNone(result.figure)
        plt.close(result.figure)

    def test_signal_spec_resolves_core_and_extra_signals(self):
        traj = Trajectory(
            t=np.array([0.0, 1.0]),
            x=np.array([[0.0, 1.0]]),
            u=np.array([[1.0, 1.0]]),
            signals={"extra": np.ones((2, 2))},
        )
        spec = build_signal_plot_spec(
            self.sys,
            traj,
            signals=("x", "u", "extra"),
        )
        labels = [trace.label for trace in spec.traces]
        self.assertIn("x[0]", labels)
        self.assertIn("u[0]", labels)
        self.assertIn("extra[0]", labels)
        self.assertIn("extra[1]", labels)

    def test_unknown_signal_reports_available_names(self):
        with self.assertRaisesRegex(ValueError, "ctl:u"):
            build_signal_plot_spec(self.diagram, self.traj, signals=("missing",))

    def test_live_time_signal_plot_reuses_artists(self):
        traj0 = Trajectory(
            t=np.array([0.0, 1.0]),
            x=np.array([[0.0, 1.0]]),
            u=np.array([[1.0, 1.0]]),
        )
        traj1 = Trajectory(
            t=np.array([0.0, 1.0]),
            x=np.array([[0.0, 0.5]]),
            u=np.array([[0.5, 0.5]]),
        )
        handle = open_time_signal_plot(
            self.sys,
            traj0,
            signals=("x", "u"),
            show=False,
        )
        try:
            fig_id = id(handle.fig)
            line_ids = [id(line) for line in handle.lines]
            handle.update(traj1)
            self.assertEqual(id(handle.fig), fig_id)
            self.assertEqual([id(line) for line in handle.lines], line_ids)
            np.testing.assert_allclose(
                handle.lines[0].get_ydata(),
                np.array([0.0, 0.5]),
            )
        finally:
            handle.close()


@pytest.mark.optional
class TestPlotlySignalBackend(unittest.TestCase):
    def test_plotly_static_and_live_update(self):
        pytest.importorskip("plotly")

        sys = Integrator()
        traj0 = Trajectory(
            t=np.array([0.0, 1.0]),
            x=np.array([[0.0, 1.0]]),
            u=np.array([[1.0, 1.0]]),
        )
        traj1 = Trajectory(
            t=np.array([0.0, 1.0]),
            x=np.array([[0.0, 0.25]]),
            u=np.array([[0.25, 0.25]]),
        )

        result = plot_time_signals(
            sys,
            traj0,
            signals=("x", "u"),
            backend="plotly",
            show=False,
        )
        self.assertEqual(len(result.figure.data), 2)
        self.assertEqual(result.figure.layout.width, PLOTLY_FIG_WIDTH)

        handle = open_time_signal_plot(
            sys,
            traj0,
            signals=("x", "u"),
            backend="plotly",
            show=False,
        )
        handle.update(traj1)
        np.testing.assert_allclose(handle.fig.data[0].y, np.array([0.0, 0.25]))

    def test_stacked_figsize_caps_height_for_popup_layout(self):
        from minilink.graphical.matplotlib_style import (
            SIGNAL_PLOT_MAX_FIG_HEIGHT_POPUP,
            SIGNAL_PLOT_ROW_HEIGHT,
            TRAJECTORY_MAX_FIG_HEIGHT_POPUP,
            TRAJECTORY_ROW_HEIGHT,
            signal_stack_figsize,
            trajectory_stack_figsize,
        )

        n = 20
        _, h_tall = trajectory_stack_figsize(n, allow_tall=True)
        self.assertEqual(h_tall, TRAJECTORY_ROW_HEIGHT * n)
        _, h_cap = trajectory_stack_figsize(n, allow_tall=False)
        self.assertEqual(h_cap, TRAJECTORY_MAX_FIG_HEIGHT_POPUP)

        _, h_sig = signal_stack_figsize(n, allow_tall=True)
        self.assertEqual(h_sig, SIGNAL_PLOT_ROW_HEIGHT * n)
        _, h_sig_cap = signal_stack_figsize(n, allow_tall=False)
        self.assertEqual(h_sig_cap, SIGNAL_PLOT_MAX_FIG_HEIGHT_POPUP)


if __name__ == "__main__":
    unittest.main()
