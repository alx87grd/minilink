"""Unit tests for the migrated signal blocks: routing, nonlinear, filters."""

import unittest

import numpy as np

from minilink.blocks.filters import LowPassFilter, NotchFilter, Washout
from minilink.blocks.nonlinear import DeadZone, Relay, Saturation
from minilink.blocks.routing import Demux, Gain, Mux, Sum
from minilink.blocks.sources import TrajectorySource
from minilink.core.trajectory import Trajectory


class TestRoutingBlocks(unittest.TestCase):
    def test_sum_default_is_tracking_error(self):
        block = Sum()  # signs (1, -1)
        y = block.outputs["y"].compute(None, np.array([5.0, 2.0]))
        np.testing.assert_allclose(y, [3.0])

    def test_sum_custom_signs_and_dim(self):
        block = Sum(signs=(1.0, 1.0, -1.0), dim=2)
        u = np.array([1.0, 2.0, 3.0, 4.0, 0.5, 0.5])  # three 2-vectors
        y = block.outputs["y"].compute(None, u)
        np.testing.assert_allclose(y, [1.0 + 3.0 - 0.5, 2.0 + 4.0 - 0.5])

    def test_gain_matrix_vector_scalar(self):
        np.testing.assert_allclose(
            Gain([[2.0, 0.0], [0.0, 3.0]])
            .outputs["y"]
            .compute(None, np.array([1.0, 1.0])),
            [2.0, 3.0],
        )
        np.testing.assert_allclose(
            Gain([2.0, 3.0]).outputs["y"].compute(None, np.array([1.0, 1.0])),
            [2.0, 3.0],
        )
        np.testing.assert_allclose(
            Gain(2.0, dim=2).outputs["y"].compute(None, np.array([1.0, 4.0])),
            [2.0, 8.0],
        )

    def test_scalar_gain_without_dim_raises(self):
        with self.assertRaises(ValueError):
            Gain(2.0)

    def test_mux_demux_round_trip(self):
        mux = Mux(dims=(2, 1))
        self.assertEqual(mux.m, 3)
        u = np.array([1.0, 2.0, 9.0])
        np.testing.assert_allclose(mux.outputs["y"].compute(None, u), u)

        demux = Demux(dims=(2, 1))
        np.testing.assert_allclose(demux.outputs["out0"].compute(None, u), [1.0, 2.0])
        np.testing.assert_allclose(demux.outputs["out1"].compute(None, u), [9.0])


class TestNonlinearBlocks(unittest.TestCase):
    def test_saturation_clips(self):
        sat = Saturation(lower=-1.0, upper=2.0)
        out = [sat.compute(None, np.array([v]))[0] for v in (-3.0, 0.5, 5.0)]
        np.testing.assert_allclose(out, [-1.0, 0.5, 2.0])

    def test_dead_zone(self):
        dz = DeadZone(width=1.0)
        out = [dz.compute(None, np.array([v]))[0] for v in (-2.0, -0.5, 0.0, 0.5, 2.0)]
        np.testing.assert_allclose(out, [-1.0, 0.0, 0.0, 0.0, 1.0])

    def test_relay_sign(self):
        relay = Relay(amplitude=2.0)
        out = [relay.compute(None, np.array([v]))[0] for v in (-3.0, 0.0, 4.0)]
        np.testing.assert_allclose(out, [-2.0, 0.0, 2.0])


class TestFilterBlocks(unittest.TestCase):
    def _dc_gain(self, lti):
        # steady-state gain  y/u = -C A^-1 B + D
        A, B, C, D = lti.A(), lti.B(), lti.C(), lti.D()
        return float((-C @ np.linalg.solve(A, B) + D)[0, 0])

    def test_low_pass_pole_and_dc_gain(self):
        lpf = LowPassFilter(cutoff_hz=0.5)
        np.testing.assert_allclose(lpf.poles, [-2.0 * np.pi * 0.5])
        self.assertAlmostEqual(self._dc_gain(lpf), 1.0, places=6)

    def test_washout_blocks_dc(self):
        self.assertAlmostEqual(self._dc_gain(Washout(cutoff_hz=1.0)), 0.0, places=6)

    def test_notch_rejects_centre_frequency(self):
        notch = NotchFilter(notch_hz=1.0, quality=10.0)
        w0 = 2.0 * np.pi * 1.0
        A, B, C, D = notch.A(), notch.B(), notch.C(), notch.D()
        n = A.shape[0]
        # frequency response magnitude at the notch centre should be ~0
        H = C @ np.linalg.solve(1j * w0 * np.eye(n) - A, B) + D
        self.assertLess(abs(H[0, 0]), 1e-6)


class TestTrajectorySource(unittest.TestCase):
    def test_interpolates_and_clamps(self):
        t = np.linspace(0.0, 10.0, 11)
        src = TrajectorySource(t, np.vstack([t, t**2]))
        self.assertEqual(src.p, 2)
        np.testing.assert_allclose(src.h(np.array([]), np.array([]), 2.5), [2.5, 6.5])
        np.testing.assert_allclose(src.h(np.array([]), np.array([]), -1.0), [0.0, 0.0])
        np.testing.assert_allclose(
            src.h(np.array([]), np.array([]), 99.0), [10.0, 100.0]
        )

    def test_from_trajectory_replays_input(self):
        t = np.linspace(0.0, 1.0, 5)
        traj = Trajectory(t=t, x=np.zeros((1, 5)), u=np.vstack([2.0 * t]))
        src = TrajectorySource.from_trajectory(traj, signal="u")
        np.testing.assert_allclose(src.h(np.array([]), np.array([]), 0.5), [1.0])


if __name__ == "__main__":
    unittest.main()
