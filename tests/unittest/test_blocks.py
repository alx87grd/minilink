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
        self.assertEqual(list(plant.inputs), ["u"])
        self.assertEqual(list(plant.outputs), ["y", "x"])
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


from minilink.blocks.filters import LowPassFilter, NotchFilter, Washout
from minilink.blocks.nonlinear import DeadZone, Relay, Saturation
from minilink.blocks.routing import Demux, Error, Gain, Mux, Sum
from minilink.blocks.sources import TrajectorySource
from minilink.core.trajectory import Trajectory


class TestRoutingBlocks(unittest.TestCase):
    def test_error_is_r_minus_y(self):
        block = Error()
        e = block.outputs["e"].compute(None, np.array([5.0, 2.0]))
        np.testing.assert_allclose(e, [3.0])
        self.assertEqual(list(block.inputs), ["+", "-"])
        self.assertEqual(list(block.outputs), ["e"])

    def test_error_vector(self):
        block = Error(dim=2)
        e = block.outputs["e"].compute(None, np.array([1.0, 4.0, 0.5, 1.0]))
        np.testing.assert_allclose(e, [0.5, 3.0])

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
        self.assertEqual(list(demux.inputs), ["u"])
        self.assertEqual(list(demux.outputs), ["u[0:2]", "u[2]"])
        np.testing.assert_allclose(demux.outputs["u[0:2]"].compute(None, u), [1.0, 2.0])
        np.testing.assert_allclose(demux.outputs["u[2]"].compute(None, u), [9.0])
        named = Demux(dims=(1, 1), port="y")
        self.assertEqual(list(named.inputs), ["y"])
        self.assertEqual(list(named.outputs), ["y[0]", "y[1]"])


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


from minilink.blocks.sources import WhiteNoise


class TestWhiteNoiseSource(unittest.TestCase):
    def signal(self, noise, times, params=None):
        empty = np.array([])
        return np.array([noise.h(empty, empty, t, params) for t in times])

    def test_same_seed_same_signal_and_a_different_seed_differs(self):
        times = np.linspace(0.0, 2.0, 200)
        y1 = self.signal(WhiteNoise(1, seed=123), times)
        y2 = self.signal(WhiteNoise(1, seed=123), times)
        y3 = self.signal(WhiteNoise(1, seed=124), times)
        np.testing.assert_array_equal(y1, y2)
        self.assertFalse(np.allclose(y1, y3))

    def test_the_mean_for_a_none_seed_or_a_zero_intensity(self):
        times = np.linspace(0.0, 1.0, 50)
        np.testing.assert_array_equal(self.signal(WhiteNoise(2, seed=None), times), 0.0)
        np.testing.assert_array_equal(self.signal(WhiteNoise(2, psd=0.0), times), 0.0)
        with self.assertRaises(ValueError):
            WhiteNoise(1, seed=-1)

    def test_per_sample_covariance_is_the_intensity_over_the_period(self):
        period = 0.02
        instants = period * np.arange(20_000) + 0.5 * period
        for psd in ([1.0, 4.0], [[1.0, 0.6], [0.6, 2.0]]):
            noise = WhiteNoise(2, psd=psd, sample_period=period, seed=5)
            W = np.diag(psd) if np.ndim(psd) == 1 else np.asarray(psd)
            samples = self.signal(noise, instants)
            scale = np.max(W / period)
            np.testing.assert_allclose(
                np.cov(samples.T), W / period, rtol=0.06, atol=0.02 * scale
            )

    def test_editing_params_changes_the_next_signal_without_refresh(self):
        noise = WhiteNoise(1, seed=0)
        times = np.linspace(0.0, 1.0, 20)
        before = self.signal(noise, times)
        noise.params["seed"] = 1
        self.assertFalse(np.allclose(before, self.signal(noise, times)))
        noise.params["psd"] = 4.0
        scaled = self.signal(noise, times)
        noise.params["psd"] = 1.0
        np.testing.assert_allclose(scaled, 2.0 * self.signal(noise, times))
        edited = self.signal(WhiteNoise(1, seed=0), times, dict(noise.params, seed=1))
        self.assertFalse(np.allclose(before, edited))

    def test_zero_order_hold_is_constant_and_the_linear_hold_continuous(self):
        zoh = WhiteNoise(1, sample_period=0.1, seed=7)
        inside = self.signal(zoh, [0.30, 0.34, 0.39])
        np.testing.assert_array_equal(inside[0], inside[1])
        np.testing.assert_array_equal(inside[0], inside[2])
        self.assertNotEqual(inside[0, 0], self.signal(zoh, [0.4])[0, 0])

        linear = WhiteNoise(1, sample_period=0.1, seed=7, hold="linear")
        left, right = self.signal(linear, [0.4 - 1e-9, 0.4 + 1e-9])[:, 0]
        self.assertLess(abs(left - right), 1e-6)
        with self.assertRaises(ValueError):
            WhiteNoise(1, hold="cubic")

    @pytest.mark.optional
    @pytest.mark.jax
    def test_the_same_signal_on_numpy_and_jax_and_a_traced_params_family(self):
        pytest.importorskip("jax")
        from minilink.core.backends import require_jax, require_jax_numpy

        jax, jnp = require_jax(), require_jax_numpy()
        noise = WhiteNoise(2, psd=[1.0, 2.0], sample_period=0.05, seed=9)
        empty = np.array([])
        times = np.linspace(0.0, 1.0, 40)
        on_numpy = self.signal(noise, times)
        on_jax = np.array([noise.h(empty, empty, jnp.asarray(t)) for t in times])
        np.testing.assert_array_equal(on_numpy, on_jax)

        jitted = jax.jit(lambda t, params: noise.h(empty, empty, t, params))
        np.testing.assert_allclose(
            jitted(0.3, noise.params), noise.h(empty, empty, 0.3)
        )

        # a family of three intensities, the seed a shared integer leaf
        family = dict(
            noise.params, psd=jnp.asarray([[1.0, 2.0], [4.0, 8.0], [9.0, 18.0]])
        )
        axes = ({"seed": None, "sample_period": None, "psd": 0},)
        ws = jax.vmap(lambda params: noise.h(empty, empty, 0.3, params), in_axes=axes)(
            family
        )
        np.testing.assert_allclose(ws[1], 2.0 * ws[0])
        np.testing.assert_allclose(ws[2], 3.0 * ws[0])

    def test_a_loop_with_noise_has_a_finite_params_jacobian(self):
        from minilink import Pendulum
        from minilink.analysis import jacobian

        loop = WhiteNoise(1, seed=2) >> Pendulum()
        J = jacobian(loop, "f", "params", loop.x0, method="fd")
        for leaf in J.values():
            for value in leaf.values():
                self.assertTrue(np.all(np.isfinite(value)))


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
