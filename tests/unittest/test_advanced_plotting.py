import unittest

import matplotlib.pyplot as plt
import numpy as np

from minilink.core.analysis import Simulator, compute_internal_signals
from minilink.core.diagram import DiagramSystem
from minilink.core.framework import DynamicSystem, StaticSystem
from minilink.graphical.plotting import plot_signals


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

    def test_compute_internal_signals(self):
        # By default, Trajectory doesn't have internal signals
        self.assertFalse(hasattr(self.traj, "internal_signals"))

        # Reconstruct signals
        traj_plus = compute_internal_signals(self.diagram, self.traj)

        # Test it successfully created the dictionary
        self.assertTrue(hasattr(traj_plus, "internal_signals"))
        self.assertIn("step:y", traj_plus.internal_signals)
        self.assertIn("ctl:u", traj_plus.internal_signals)
        self.assertIn("plant:y", traj_plus.internal_signals)

        # Check shapes (dim 1, n_pts time steps)
        n_pts = len(self.traj.t)
        self.assertEqual(traj_plus.internal_signals["ctl:u"].shape, (1, n_pts))

    def test_plot_signals_does_not_crash(self):
        import minilink.graphical.plotting as plotting

        plotting.figure_blocking = False
        traj_plus = compute_internal_signals(self.diagram, self.traj)

        # Test basic API functionality (we won't check pixel rendering, just execution)
        try:
            fig, ax = plot_signals(
                self.diagram,
                traj_plus,
                [
                    {"sys": "plant", "state": "x[0]", "label": "Plant State"},
                    {"sys": "ctl", "output": "u", "label": "Control Effort"},
                ],
            )
            self.assertIsNotNone(fig)
            plt.close(fig)
            success = True
        except Exception as e:
            success = False
            self.fail(f"plot_signals raised an exception: {e}")

        self.assertTrue(success)


if __name__ == "__main__":
    unittest.main()
