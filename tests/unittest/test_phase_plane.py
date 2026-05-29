import unittest

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from minilink.core.system import DynamicSystem  # noqa: E402
from minilink.core.trajectory import Trajectory  # noqa: E402
from minilink.graphical.plotting import (  # noqa: E402
    build_phase_plane_spec,
    plot_phase_plane,
)


class PhasePlaneTestSystem(DynamicSystem):
    def __init__(self):
        super().__init__(n=2, input_dim=1, output_dim=2, y_dependencies=())
        self.name = "PhasePlaneTestSystem"
        self.state.labels = ["position", "velocity"]
        self.state.units = ["m", "m/s"]
        self.state.lower_bound = np.array([-2.0, -3.0])
        self.state.upper_bound = np.array([2.0, 3.0])
        self.inputs["u"].set_nominal_value(np.array([0.5]))

    def f(self, x, u, t=0, params=None):
        return np.array([x[1] + u[0], -2.0 * x[0] + t])


class OneStateSystem(DynamicSystem):
    def __init__(self):
        super().__init__(n=1, output_dim=1, y_dependencies=())
        self.name = "OneStateSystem"
        self.state.lower_bound = np.array([-1.0])
        self.state.upper_bound = np.array([1.0])

    def f(self, x, u, t=0, params=None):
        return np.array([2.0 * x[0]])


class UnboundedPlanarSystem(DynamicSystem):
    def __init__(self):
        super().__init__(n=2, output_dim=2, y_dependencies=())

    def f(self, x, u, t=0, params=None):
        return np.array([x[1], -x[0]])


class TestPhasePlane(unittest.TestCase):
    def tearDown(self):
        plt.close("all")

    def test_build_phase_plane_spec_evaluates_vector_field(self):
        sys = PhasePlaneTestSystem()
        spec = build_phase_plane_spec(
            sys,
            x_axis=0,
            y_axis=1,
            t=0.25,
            bounds=((-1.0, 1.0), (-2.0, 2.0)),
            grid_shape=(3, 2),
        )

        self.assertEqual(spec.X.shape, (2, 3))
        self.assertEqual(spec.Y.shape, (2, 3))
        self.assertEqual(spec.x_label, "position")
        self.assertEqual(spec.y_label, "velocity")
        self.assertEqual(spec.x_unit, "m")
        self.assertEqual(spec.y_unit, "m/s")

        np.testing.assert_allclose(spec.X[0, 2], 1.0)
        np.testing.assert_allclose(spec.Y[0, 2], -2.0)
        np.testing.assert_allclose(spec.V[0, 2], -1.5)
        np.testing.assert_allclose(spec.W[0, 2], -1.75)

    def test_one_state_system_can_use_same_axis_twice(self):
        sys = OneStateSystem()
        spec = build_phase_plane_spec(
            sys,
            x_axis=0,
            y_axis=0,
            grid_shape=(2, 2),
        )

        self.assertEqual(spec.x_axis, 0)
        self.assertEqual(spec.y_axis, 0)
        np.testing.assert_allclose(spec.V, spec.W)
        np.testing.assert_allclose(spec.V, 2.0 * spec.X)

    def test_infinite_bounds_fall_back_to_trajectory_with_padding(self):
        sys = UnboundedPlanarSystem()
        traj = Trajectory(
            t=np.array([0.0, 1.0]),
            x=np.array([[2.0, 4.0], [-1.0, 3.0]]),
            u=np.zeros((0, 2)),
        )

        spec = build_phase_plane_spec(sys, traj)

        np.testing.assert_allclose(spec.x_bounds, (1.9, 4.1))
        np.testing.assert_allclose(spec.y_bounds, (-1.2, 3.2))
        np.testing.assert_allclose(spec.trajectory.x, np.array([2.0, 4.0]))
        np.testing.assert_allclose(spec.trajectory.y, np.array([-1.0, 3.0]))

    def test_plot_phase_plane_matplotlib_with_overlay(self):
        sys = PhasePlaneTestSystem()
        traj = Trajectory(
            t=np.array([0.0, 1.0, 2.0]),
            x=np.array([[0.0, 0.5, 1.0], [1.0, 0.0, -1.0]]),
            u=np.zeros((1, 3)),
        )

        result = plot_phase_plane(
            sys,
            traj,
            bounds=((-1.0, 1.0), (-1.5, 1.5)),
            grid_shape=(3, 3),
            show=False,
        )

        self.assertEqual(result.backend, "matplotlib")
        self.assertIsNotNone(result.figure)
        self.assertIs(result.axes.figure, result.figure)
        self.assertEqual(len(result.axes.lines), 3)

    def test_system_facade_uses_cached_trajectory(self):
        sys = PhasePlaneTestSystem()
        sys.traj = Trajectory(
            t=np.array([0.0, 1.0]),
            x=np.array([[0.0, 1.0], [0.0, -1.0]]),
            u=np.zeros((1, 2)),
        )

        result = sys.plot_phase_plane(
            bounds=((-1.0, 1.0), (-1.0, 1.0)),
            grid_shape=(3, 3),
            show=False,
        )

        self.assertEqual(result.backend, "matplotlib")
        self.assertEqual(len(result.axes.lines), 3)

    def test_unsupported_backend_reports_clear_error(self):
        sys = PhasePlaneTestSystem()
        with self.assertRaisesRegex(ValueError, "backend='plotly'.*not implemented"):
            plot_phase_plane(sys, backend="plotly", show=False)


if __name__ == "__main__":
    unittest.main()
