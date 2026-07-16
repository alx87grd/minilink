import unittest
import numpy as np
import pytest
from minilink.blocks.basic import Integrator
from minilink.blocks.sources import Source, Step
from minilink.blocks.transfer_function import TransferFunction
from minilink.control.output import ProportionalController


class TestBlocks(unittest.TestCase):
    def test_source(self):
        s = Source(p=2)
        s.params["value"] = np.array([3.0, 4.0])
        y = s.h(x=[], u=[], t=0)
        np.testing.assert_array_equal(y, np.array([3.0, 4.0]))

    def test_step_source(self):
        step = Step(
            initial_value=np.array([0.0]), final_value=np.array([1.0]), step_time=2.0
        )
        y_before = step.h(x=[], u=[], t=1.0)
        y_after = step.h(x=[], u=[], t=3.0)
        np.testing.assert_array_equal(y_before, np.array([0.0]))
        np.testing.assert_array_equal(y_after, np.array([1.0]))

    def test_integrator_dynamics_and_output(self):
        plant = Integrator()
        plant.params["k"] = 2.0
        np.testing.assert_array_equal(
            plant.f(np.array([3.0]), np.array([4.0])), np.array([8.0])
        )
        np.testing.assert_array_equal(
            plant.h(np.array([3.0]), np.array([4.0])), np.array([3.0])
        )

    def test_integrator_compiled_rollout(self):
        plant = Integrator()
        evaluator = plant.compile()
        u_sequence = np.ones((3, 1))
        x = evaluator.rk4_integrate_zoh(np.array([0.0]), u_sequence, t0=0.0, dt=0.1)
        np.testing.assert_allclose(x[:, 0], np.array([0.0, 0.1, 0.2, 0.3]))

    def test_integrator_compiled_parametric_rollout(self):
        plant = Integrator()
        evaluator = plant.compile()
        u_sequence = np.ones((2, 1))
        x = evaluator.rk4_integrate_zoh_p(
            np.array([0.0]), u_sequence, t0=0.0, dt=0.1, params={"k": 2.0}
        )
        np.testing.assert_allclose(x[:, 0], np.array([0.0, 0.2, 0.4]))

    def test_transfer_function_first_order_step(self):
        plant = TransferFunction([1.0], [1.0, 1.0])
        x = np.array([0.0])
        u = np.array([1.0])
        dx = plant.f(x, u)
        np.testing.assert_allclose(dx, np.array([1.0]))
        np.testing.assert_allclose(plant.h(x, u), np.array([0.0]))
        np.testing.assert_allclose(plant.compile("numpy").f(x, u, 0.0), np.array([1.0]))

    def test_prop_controller_scales_tracking_error(self):
        controller = ProportionalController(2.5)
        np.testing.assert_array_equal(
            controller.ctl(np.array([]), np.array([3.0, 1.0])), np.array([5.0])
        )

    def test_basic_blocks_are_jax_jittable(self):
        jax = pytest.importorskip("jax")
        import jax.numpy as jnp

        plant = Integrator()
        plant.params["k"] = 2.0
        controller = ProportionalController(2.5)
        dx = jax.jit(plant.f)(jnp.asarray([3.0]), jnp.asarray([4.0]))
        y = jax.jit(plant.h)(jnp.asarray([3.0]), jnp.asarray([4.0]))
        u_cmd = jax.jit(controller.ctl)(jnp.asarray([]), jnp.asarray([3.0, 1.0]))
        np.testing.assert_allclose(np.asarray(dx), [8.0])
        np.testing.assert_allclose(np.asarray(y), [3.0])
        np.testing.assert_allclose(np.asarray(u_cmd), [5.0])

    def test_gain_jax_static_compile(self):
        pytest.importorskip("jax")
        import jax.numpy as jnp
        from minilink.blocks.routing import Gain
        from minilink.core.compile.evaluators.jax_evaluators import JaxStaticEvaluator

        gain = Gain(K=2.0, dim=1)
        ev = gain.compile(backend="jax")
        self.assertIsInstance(ev, JaxStaticEvaluator)
        out = ev.outputs(jnp.array([]), jnp.array([3.0]), 0.0)
        np.testing.assert_allclose(np.asarray(out["y"]), [6.0])


# from test_signal_blocks.py
from minilink.blocks.filters import LowPassFilter, NotchFilter, Washout
from minilink.blocks.nonlinear import DeadZone, Relay, Saturation
from minilink.blocks.routing import Demux, Gain, Mux, Sum
from minilink.blocks.sources import TrajectorySource
from minilink.core.trajectory import Trajectory


class TestRoutingBlocks(unittest.TestCase):
    def test_sum_default_is_tracking_error(self):
        block = Sum()
        y = block.outputs["y"].compute(None, np.array([5.0, 2.0]))
        np.testing.assert_allclose(y, [3.0])

    def test_sum_custom_signs_and_dim(self):
        block = Sum(signs=(1.0, 1.0, -1.0), dim=2)
        u = np.array([1.0, 2.0, 3.0, 4.0, 0.5, 0.5])
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
        A, B, C, D = (lti.A(), lti.B(), lti.C(), lti.D())
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
        A, B, C, D = (notch.A(), notch.B(), notch.C(), notch.D())
        n = A.shape[0]
        H = C @ np.linalg.solve(1j * w0 * np.eye(n) - A, B) + D
        self.assertLess(abs(H[0, 0]), 1e-06)


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


# from test_signal_colors.py
from minilink.graphical.signals.signal_colors import (
    INPUT_COLOR,
    INTERNAL_SIGNAL_COLORS,
    STATE_COLOR,
    color_for_signal,
    is_core_input,
    is_core_state,
    is_internal_signal,
    plotly_color,
)


class TestSignalColors(unittest.TestCase):
    def test_core_state_is_blue(self):
        style = color_for_signal("x")
        self.assertEqual(style.color, STATE_COLOR)
        self.assertEqual(style.linewidth, 2.0)

    def test_core_inputs_are_red(self):
        for name in ("u", "u_cmd"):
            style = color_for_signal(name)
            self.assertEqual(style.color, INPUT_COLOR)
            self.assertEqual(style.linewidth, 2.0)

    def test_internal_signals_are_not_red(self):
        for name in ("ctl:u", "y", "r", "plant:dq"):
            style = color_for_signal(name, internal_index=0)
            self.assertNotEqual(style.color, INPUT_COLOR)

    def test_internal_palette_cycles_by_index(self):
        first = color_for_signal("y", internal_index=0)
        second = color_for_signal("r", internal_index=1)
        self.assertEqual(first.color, INTERNAL_SIGNAL_COLORS[0])
        self.assertEqual(second.color, INTERNAL_SIGNAL_COLORS[1])
        self.assertNotEqual(first.color, second.color)

    def test_multi_component_shading_same_hue_different_alpha(self):
        style0 = color_for_signal("x", component=0, n_components=2)
        style1 = color_for_signal("x", component=1, n_components=2)
        self.assertEqual(style0.color, style1.color)
        self.assertLess(style0.alpha, style1.alpha)

    def test_single_component_alpha_is_one(self):
        style = color_for_signal("x", component=0, n_components=1)
        self.assertEqual(style.alpha, 1.0)

    def test_classification_helpers(self):
        self.assertTrue(is_core_state("x"))
        self.assertFalse(is_core_state("u"))
        self.assertTrue(is_core_input("u"))
        self.assertTrue(is_core_input("u_cmd"))
        self.assertFalse(is_core_input("ctl:u"))
        self.assertTrue(is_internal_signal("ctl:u"))
        self.assertFalse(is_internal_signal("x"))

    def test_plotly_color_maps_tab_names(self):
        self.assertEqual(plotly_color("tab:green"), "#2ca02c")
        self.assertEqual(plotly_color("tab:orange"), "#ff7f0e")


# from test_sources_white_noise.py
from minilink.blocks.sources import WhiteNoise


class TestWhiteNoiseSource(unittest.TestCase):
    def test_same_seed_same_refresh_same_output(self):
        times = np.linspace(0.0, 2.0, 200)
        n1 = WhiteNoise(1)
        n1.params["seed"] = 123
        n1.params["sample_period"] = 0.01
        n1.params["t0"] = -1.0
        n1.params["tf"] = 3.0
        n1.refresh()
        y1 = np.array([n1.h(np.array([]), np.array([]), t)[0] for t in times])
        n2 = WhiteNoise(1)
        n2.params["seed"] = 123
        n2.params["sample_period"] = 0.01
        n2.params["t0"] = -1.0
        n2.params["tf"] = 3.0
        n2.refresh()
        y2 = np.array([n2.h(np.array([]), np.array([]), t)[0] for t in times])
        self.assertTrue(np.allclose(y1, y2))

    def test_different_seed_changes_output(self):
        times = np.linspace(0.0, 2.0, 200)
        n1 = WhiteNoise(1)
        n1.params["seed"] = 1
        n1.refresh()
        y1 = np.array([n1.h(np.array([]), np.array([]), t)[0] for t in times])
        n2 = WhiteNoise(1)
        n2.params["seed"] = 2
        n2.refresh()
        y2 = np.array([n2.h(np.array([]), np.array([]), t)[0] for t in times])
        self.assertFalse(np.allclose(y1, y2))

    def test_continuity_with_interpolation(self):
        n = WhiteNoise(1)
        n.params["seed"] = 77
        n.params["sample_period"] = 0.05
        n.params["t0"] = 0.0
        n.params["tf"] = 1.0
        n.refresh()
        t_left = 0.5 - 1e-06
        t_right = 0.5 + 1e-06
        y_left = n.h(np.array([]), np.array([]), t_left)[0]
        y_right = n.h(np.array([]), np.array([]), t_right)[0]
        self.assertLess(abs(y_right - y_left), 0.01)

    def test_refresh_horizon_changes_edge_values(self):
        n = WhiteNoise(1)
        n.params["seed"] = 10
        n.params["t0"] = -100.0
        n.params["tf"] = 100.0
        n.refresh()
        y_at_minus_five = n.h(np.array([]), np.array([]), -5.0)[0]
        n.params["t0"] = 0.0
        n.params["tf"] = 1.0
        n.refresh()
        y_left_clamped = n.h(np.array([]), np.array([]), -5.0)[0]
        self.assertNotEqual(y_at_minus_five, y_left_clamped)


# from test_neural_blocks.py
from minilink.blocks.neural import NeuralNetwork


class TestNeuralNetwork(unittest.TestCase):
    def test_forward_equation_and_shape(self):
        net = NeuralNetwork(input_dim=2, output_dim=1, hidden_dim=3)
        params = {
            "W1": np.array([[1.0, 0.0], [0.0, 1.0], [-1.0, 1.0]]),
            "b1": np.array([0.0, 0.5, -0.5]),
            "W2": np.array([[2.0, -1.0, 0.5]]),
            "b2": np.array([0.25]),
        }
        u = np.array([0.2, -0.4])
        y = net.compute(np.array([]), u, params=params)
        expected = (
            params["W2"] @ np.tanh(params["W1"] @ u + params["b1"]) + params["b2"]
        )
        self.assertEqual(y.shape, (1,))
        np.testing.assert_allclose(y, expected)

    def test_explicit_params_override_defaults(self):
        net = NeuralNetwork(input_dim=1, output_dim=1, hidden_dim=2)
        net.params = {
            "W1": np.array([[1.0], [1.0]]),
            "b1": np.array([0.0, 0.0]),
            "W2": np.array([[1.0, 1.0]]),
            "b2": np.array([0.0]),
        }
        override = {
            "W1": np.zeros((2, 1)),
            "b1": np.zeros(2),
            "W2": np.zeros((1, 2)),
            "b2": np.array([3.0]),
        }
        y_default = net.compute(np.array([]), np.array([1.0]))
        y_override = net.compute(np.array([]), np.array([1.0]), params=override)
        np.testing.assert_allclose(y_default, [2.0 * np.tanh(1.0)])
        np.testing.assert_allclose(y_override, [3.0])

    def test_compute_is_jax_traceable(self):
        jax = pytest.importorskip("jax")
        import jax.numpy as jnp

        net = NeuralNetwork(input_dim=2, output_dim=1, hidden_dim=3)
        params = {key: jnp.asarray(value) for key, value in net.params.items()}
        y = jax.jit(lambda u, params: net.compute([], u, params=params))(
            jnp.array([1.0, -1.0]), params
        )
        self.assertEqual(np.asarray(y).shape, (1,))
