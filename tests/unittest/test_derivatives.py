"""``jacobian(of, wrt)`` on evaluators and systems, and the analysis family pattern."""

from __future__ import annotations

import unittest

import numpy as np
import pytest

from minilink import (
    CartPole,
    DoublePendulum,
    ImpedanceController,
    InvertedPendulum,
    Pendulum,
    controllability,
    discretize,
    lqr_at_operating_point,
    observability,
)
from minilink.analysis.frequency import bode, pzmap, transfer_function
from minilink.analysis.linearize import linearize, linearize_matrices
from minilink.blocks.transfer_function import TransferFunction
from minilink.control.state import StateFeedbackController
from minilink.core.backends import array_module, jax_installed
from minilink.core.system import DynamicSystem, StepSystem

JAX = jax_installed()
jax_only = pytest.mark.skipif(not JAX, reason="JAX not installed")


class _Affine(DynamicSystem):
    """dx = A x + B u + c t, y = C x + D u, z = x[0]: every Jacobian is known."""

    A = np.array([[0.0, 1.0], [-2.0, -3.0]])
    B = np.array([[0.0, 1.0], [3.0, 5.0]])
    C = np.array([[1.0, 0.0]])
    D = np.array([[0.0, 2.0]])
    c = np.array([0.5, -1.0])

    def __init__(self):
        super().__init__(n=2)
        self.name = "Affine"
        self.params = {"gain": 1.0}
        self.add_input_port("force")
        self.add_input_port("bias")
        self.add_output_port("y", dim=1, function=self.h, dependencies=("bias",))
        self.add_output_port("z", dim=1, function=self.z)

    def f(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        return params["gain"] * (self.A @ x + self.B @ u) + self.c * t

    def h(self, x, u, t=0, params=None):
        return self.C @ x + self.D @ u

    def z(self, x, u, t=0, params=None):
        xp = array_module(x, u)
        return xp.array([x[0]])


class _Counter(StepSystem):
    """x_{k+1} = a x_k + b u_k."""

    def __init__(self):
        super().__init__(n=1, input_dim=1, output_dim=1)
        self.params = {"a": 0.9, "b": 0.2}

    def step(self, x, u, k=0, params=None):
        params = self.params if params is None else params
        return params["a"] * x + params["b"] * u


class _Untraceable(Pendulum):
    def f(self, x, u, t=0, params=None):
        return np.asarray(super().f(np.asarray(x), np.asarray(u), t, params), float)


def _closed_loop():
    return ImpedanceController() @ Pendulum()


class TestJacobianValues(unittest.TestCase):
    """Known Jacobians of the affine leaf, both backends."""

    def setUp(self):
        self.plant = _Affine()
        self.x = np.array([0.3, -0.2])
        self.u = np.array([0.4, 0.1])

    def check(self, method):
        plant, x, u = self.plant, self.x, self.u
        np.testing.assert_allclose(
            plant.jacobian("f", "x", x, u, method=method), _Affine.A, atol=1e-6
        )
        np.testing.assert_allclose(
            plant.jacobian("f", "u", x, u, method=method), _Affine.B, atol=1e-6
        )
        np.testing.assert_allclose(
            plant.jacobian("f", "bias", x, u, method=method),
            _Affine.B[:, [1]],
            atol=1e-6,
        )
        np.testing.assert_allclose(
            plant.jacobian("y", "x", x, u, method=method), _Affine.C, atol=1e-6
        )
        np.testing.assert_allclose(
            plant.jacobian("y", "u", x, u, method=method), _Affine.D, atol=1e-6
        )
        np.testing.assert_allclose(
            plant.jacobian("z", "x", x, u, method=method), [[1.0, 0.0]], atol=1e-6
        )
        np.testing.assert_allclose(
            plant.jacobian("f", "t", x, u, 2.0, method=method), _Affine.c, atol=1e-6
        )
        S = plant.jacobian("f", "params", x, u, method=method)
        self.assertEqual(set(S), {"gain"})
        np.testing.assert_allclose(S["gain"], _Affine.A @ x + _Affine.B @ u, atol=1e-6)

    def test_finite_differences(self):
        self.check("fd")

    @jax_only
    def test_jax(self):
        self.check("jax")
        self.assertEqual(self.plant.compiled_evaluator("jax").backend, "jax")

    def test_defaults_are_the_nominal_point(self):
        plant = self.plant
        plant.x0 = np.array([1.0, 2.0])
        plant.inputs["bias"].set_nominal_value(np.array([0.7]))
        expected = plant.jacobian(
            "f", "params", plant.x0, plant.get_u_from_input_ports()
        )
        np.testing.assert_allclose(
            plant.jacobian("f", "params")["gain"], expected["gain"]
        )

    def test_returns_numpy_float_arrays(self):
        J = self.plant.jacobian("f", "x")
        self.assertIsInstance(J, np.ndarray)
        self.assertEqual(J.dtype, np.float64)
        self.assertEqual(J.shape, (2, 2))
        self.assertEqual(self.plant.jacobian("f", "t").shape, (2,))


class TestJacobianSelectors(unittest.TestCase):
    def test_unknown_names_list_the_choices(self):
        plant = _Affine()
        with self.assertRaisesRegex(ValueError, "of must be 'f' or 'y' or 'z'"):
            plant.jacobian("h", "x")
        with self.assertRaisesRegex(ValueError, "wrt must be 'x' or 'u'"):
            plant.jacobian("f", "q")
        with self.assertRaisesRegex(ValueError, "not a diagram"):
            plant.jacobian("f", "plant:u")

    def test_static_block_x_reaches_the_port(self):
        ctl = StateFeedbackController(
            np.array([[1.0, 2.0]]), xbar=[0.0, 0.0], ubar=[0.5]
        )
        np.testing.assert_allclose(ctl.jacobian("u", "x"), [[-1.0, -2.0]], atol=1e-8)
        np.testing.assert_allclose(ctl.jacobian("u", "r"), [[1.0, 2.0]], atol=1e-8)
        with self.assertRaisesRegex(ValueError, "not a step system|of must be"):
            ctl.jacobian("f", "u")

    def test_static_block_without_x_port_has_no_state(self):
        gain = ImpedanceController()
        with self.assertRaisesRegex(ValueError, "no state"):
            gain.jacobian("u", "x")

    def test_dynamic_block_with_clashing_port_is_refused(self):
        class Clash(DynamicSystem):
            def __init__(self):
                super().__init__(n=1, output_dim=1)
                self.add_input_port("x")

            def f(self, x, u, t=0, params=None):
                return -x + u

        with self.assertRaisesRegex(ValueError, "ambiguous"):
            Clash().jacobian("f", "x")

    def test_step_system_uses_step_and_k(self):
        counter = _Counter()
        np.testing.assert_allclose(counter.jacobian("step", "x"), [[0.9]], atol=1e-8)
        np.testing.assert_allclose(
            counter.jacobian("step", "u", [1.0], [0.0], 3), [[0.2]], atol=1e-8
        )
        np.testing.assert_allclose(
            counter.jacobian("step", "params")["b"], [0.0], atol=1e-8
        )
        with self.assertRaisesRegex(ValueError, "k is an integer"):
            counter.jacobian("step", "t")
        with self.assertRaisesRegex(ValueError, "of must be 'step'"):
            counter.jacobian("f", "x")

    def test_params_without_float_leaves_is_refused(self):
        with self.assertRaisesRegex(ValueError, "float leaves"):
            _Affine().jacobian("f", "params", params={})


class TestJacobianDiagram(unittest.TestCase):
    def setUp(self):
        self.loop = _closed_loop()
        self.plant_id = [
            wire for wire in self.loop.compile()._jac_wires if wire.endswith(":y")
        ][0].split(":")[0]

    def test_closed_loop_matrices_match_finite_differences(self):
        loop = self.loop
        A_fd = loop.jacobian("f", "x", method="fd")
        self.assertEqual(A_fd.shape, (2, 2))
        self.assertLess(A_fd[1, 1], 0.0)  # damping from the controller
        if JAX:
            np.testing.assert_allclose(
                loop.jacobian("f", "x", method="jax"), A_fd, atol=1e-6
            )

    def test_wire_as_output_and_as_perturbation(self):
        loop, plant = self.loop, self.plant_id
        np.testing.assert_allclose(
            loop.jacobian(f"{plant}:y", "x", method="fd"), np.eye(2), atol=1e-6
        )
        # a disturbance added to the controller command enters f like u does
        B_dist = loop.jacobian("f", "ctl:u", method="fd")
        np.testing.assert_allclose(
            B_dist, Pendulum().jacobian("f", "u", method="fd"), atol=1e-6
        )
        np.testing.assert_allclose(
            loop.jacobian("ctl:u", "ctl:u", method="fd"), [[1.0]], atol=1e-6
        )
        if JAX:
            np.testing.assert_allclose(
                loop.jacobian("f", "ctl:u", method="jax"), B_dist, atol=1e-6
            )
        with self.assertRaisesRegex(ValueError, "Unknown wire"):
            loop.jacobian("f", "nowhere:u")

    def test_params_dict_is_nested_like_the_diagram(self):
        S = self.loop.jacobian("f", "params", method="fd")
        self.assertEqual(set(S), set(self.loop.params))
        for block, leaves in S.items():
            for key, value in leaves.items():
                self.assertEqual(value.shape[0], 2, (block, key))
        if JAX:
            S_jax = self.loop.jacobian("f", "params", method="jax")
            np.testing.assert_allclose(S_jax["ctl"]["Kp"], S["ctl"]["Kp"], atol=1e-5)


class TestCatalogAgreement(unittest.TestCase):
    """Exact and finite-difference Jacobians agree on catalog plants."""

    @jax_only
    def test_fd_matches_jax(self):
        rng = np.random.default_rng(0)
        for plant in (Pendulum(), InvertedPendulum(), CartPole(), DoublePendulum()):
            x = 0.3 * rng.standard_normal(plant.n)
            u = 0.3 * rng.standard_normal(plant.m)
            for of, wrt in (("f", "x"), ("f", "u"), ("y", "x")):
                exact = plant.jacobian(of, wrt, x, u, method="jax")
                fd = plant.jacobian(of, wrt, x, u, method="fd")
                np.testing.assert_allclose(
                    exact, fd, atol=1e-5, err_msg=f"{plant.name} {of}/{wrt}"
                )
            S_exact = plant.jacobian("f", "params", x, u, method="jax")
            S_fd = plant.jacobian("f", "params", x, u, method="fd")
            for key in S_fd:
                np.testing.assert_allclose(
                    S_exact[key], S_fd[key], atol=1e-4, err_msg=f"{plant.name} {key}"
                )


class TestMethodAndCache(unittest.TestCase):
    def test_auto_falls_back_to_finite_differences_and_jax_is_strict(self):
        plant = _Untraceable()
        A = plant.jacobian("f", "x")
        self.assertEqual(plant.compiled_evaluator("auto").backend, "numpy")
        np.testing.assert_allclose(
            A, Pendulum().jacobian("f", "x", method="fd"), atol=1e-8
        )
        if JAX:
            with self.assertRaises(RuntimeError):
                plant.jacobian("f", "x", method="jax")

    def test_unknown_method_is_refused(self):
        with self.assertRaisesRegex(ValueError, "method must be"):
            Pendulum().jacobian("f", "x", method="ad")

    def test_evaluator_is_cached_and_parameters_stay_live(self):
        plant = Pendulum()
        first = plant.compiled_evaluator("auto")
        self.assertIs(plant.compiled_evaluator("auto"), first)
        a_before = plant.jacobian("f", "x")[1, 0]
        plant.params["gravity"] = 2.0 * plant.params["gravity"]
        a_after = plant.jacobian("f", "x")[1, 0]
        self.assertIs(plant.compiled_evaluator("auto"), first)
        np.testing.assert_allclose(a_after, 2.0 * a_before, rtol=1e-6)

    def test_structural_changes_recompile(self):
        plant = Pendulum()
        first = plant.compiled_evaluator("auto")
        plant.add_output_port("angle", dim=1, function=lambda x, u, t, p=None: x[:1])
        self.assertIsNot(plant.compiled_evaluator("auto"), first)
        np.testing.assert_allclose(
            plant.jacobian("angle", "x"), [[1.0, 0.0]], atol=1e-8
        )
        # a port added to a block already inside a diagram is seen by the diagram
        loop = _closed_loop()
        first = loop.compiled_evaluator("auto")
        ctl = loop.subsystems["ctl"]
        ctl.add_output_port(
            "twice",
            dim=1,
            function=lambda x, u, t, p=None: 2.0 * ctl.ctl(x, u, t, p),
            dependencies="all",
        )
        self.assertIsNot(loop.compiled_evaluator("auto"), first)
        np.testing.assert_allclose(
            loop.jacobian("ctl:twice", "r")[0, 0],
            2.0 * loop.jacobian("ctl:u", "r")[0, 0],
        )

    def test_simulation_and_refresh_keep_the_cache(self):
        plant = Pendulum()
        first = plant.compiled_evaluator("auto")
        plant.refresh()
        plant.compute_trajectory(tf=0.1, n_steps=11, verbose=False)
        self.assertIs(plant.compiled_evaluator("auto"), first)

    def test_cache_survives_copies_without_carrying_evaluators(self):
        import copy
        import pickle

        plant = Pendulum()
        plant.jacobian("f", "x")
        twin = copy.deepcopy(plant)
        self.assertEqual(twin.compiled_evaluators, {})
        pickle.loads(pickle.dumps(twin))
        np.testing.assert_allclose(twin.jacobian("f", "x"), plant.jacobian("f", "x"))

    @jax_only
    def test_auto_uses_finite_differences_when_params_do_not_trace(self):
        from minilink.dynamics.catalog.mass_spring_damper.linear import TwoMass

        plant = TwoMass()  # A(t, params) is a NumPy matrix built from params
        self.assertEqual(plant.compiled_evaluator("auto").backend, "jax")
        S = plant.jacobian("f", "params")
        S_fd = plant.jacobian("f", "params", method="fd")
        for key in S_fd:
            np.testing.assert_allclose(S[key], S_fd[key])
        with self.assertRaises(Exception):
            plant.jacobian("f", "params", method="jax")


class TestEvaluatorFunctionForm(unittest.TestCase):
    def test_numpy_evaluator_returns_a_callable(self):
        plant = _Affine()
        ev = plant.compile("numpy")
        dfdx = ev.jacobian("f", "x")
        np.testing.assert_allclose(
            dfdx(np.zeros(2), np.zeros(2), 0.0, plant.params), _Affine.A, atol=1e-6
        )

    @jax_only
    def test_jax_callable_composes_with_jit(self):
        import jax
        import jax.numpy as jnp

        plant = _Affine()
        dfdx = jax.jit(plant.compile("jax").jacobian("f", "x"))
        J = dfdx(jnp.zeros(2), jnp.zeros(2), 0.0, plant.params)
        np.testing.assert_allclose(np.asarray(J), _Affine.A, atol=1e-6)


class TestFamilyPattern(unittest.TestCase):
    """The analysis verbs share one signature and the same selectors."""

    def test_linearize_default_channels(self):
        plant = _Affine()
        A, B, C, D = linearize_matrices(plant, [0.1, 0.2], [0.3, 0.4], method="fd")
        np.testing.assert_allclose(A, _Affine.A, atol=1e-6)
        np.testing.assert_allclose(B, _Affine.B, atol=1e-6)
        np.testing.assert_allclose(C, _Affine.C, atol=1e-6)
        np.testing.assert_allclose(D, _Affine.D, atol=1e-6)
        lti = plant.linearize([0.1, 0.2], [0.3, 0.4])
        np.testing.assert_allclose(lti.A(), A)
        self.assertEqual(lti.state.labels[0], "Delta x[0]")

    def test_linearize_selects_rows_and_columns(self):
        plant = _Affine()
        A, B, C, D = linearize_matrices(
            plant, of=["z", ("y", 0)], wrt=("bias", 0), method="fd"
        )
        np.testing.assert_allclose(B, _Affine.B[:, [1]], atol=1e-6)
        np.testing.assert_allclose(C, np.vstack([[1.0, 0.0], _Affine.C]), atol=1e-6)
        np.testing.assert_allclose(D, [[0.0], [2.0]], atol=1e-6)

    def test_linearize_without_y_port_uses_the_state(self):
        class Bare(DynamicSystem):
            def __init__(self):
                super().__init__(n=2, input_dim=1)

            def f(self, x, u, t=0, params=None):
                return np.array([x[1], -x[0] + u[0]])

        A, B, C, D = linearize_matrices(Bare(), method="fd")
        np.testing.assert_allclose(C, np.eye(2))
        np.testing.assert_allclose(D, np.zeros((2, 1)))

    def test_channel_tools_share_of_and_wrt(self):
        plant = _Affine()
        w, mag, phase = bode(plant, of="y", wrt=("bias", 0), w=[1.0], method="fd")
        # y / bias: C (sI - A)^-1 b + d with b = B[:, 1], d = 2
        G = (
            _Affine.C @ np.linalg.solve(1j * np.eye(2) - _Affine.A, _Affine.B[:, [1]])
            + 2.0
        )
        np.testing.assert_allclose(mag, [20 * np.log10(abs(G[0, 0]))], atol=1e-6)
        zeros, poles, gain = pzmap(plant, of="y", wrt="bias", method="fd")
        np.testing.assert_allclose(
            np.sort(poles), np.sort(np.linalg.eigvals(_Affine.A)), atol=1e-6
        )
        G = transfer_function(plant, of="y", wrt="bias", method="fd")
        self.assertIsInstance(G, TransferFunction)
        np.testing.assert_allclose(np.sort(G.poles), np.sort(poles), atol=1e-6)
        self.assertIn("y[0] / bias[0]", G.name)
        self.assertIsInstance(
            plant.transfer_function(of="y", wrt="bias"), TransferFunction
        )

    def test_static_channel_is_a_pure_gain(self):
        ctl = StateFeedbackController(
            np.array([[1.0, 2.0]]), xbar=[0.0, 0.0], ubar=[0.5]
        )
        G = transfer_function(ctl, of="u", wrt=("x", 1))
        np.testing.assert_allclose(G.numerator, [-2.0])
        np.testing.assert_allclose(G.denominator, [1.0])
        zeros, poles, gain = pzmap(ctl, of="u", wrt=("x", 1))
        self.assertEqual(poles.size, 0)
        np.testing.assert_allclose(gain, -2.0)

    def test_methods_on_the_system_mirror_the_functions(self):
        plant = Pendulum()
        x_bar = [0.0, 0.0]
        np.testing.assert_allclose(
            plant.linearize(x_bar).A(), linearize(plant, x_bar).A()
        )
        np.testing.assert_allclose(
            plant.bode(x_bar, w=[1.0])[1], bode(plant, x_bar, w=[1.0])[1]
        )
        np.testing.assert_allclose(plant.pzmap(x_bar)[1], pzmap(plant, x_bar)[1])
        poles, modes = plant.modal_analysis(x_bar)
        self.assertEqual(len(poles), 2)
        np.testing.assert_allclose(
            plant.find_equilibrium([0.3, 0.0]), [0.0, 0.0], atol=1e-6
        )

    def test_structural_tests_accept_an_lti_system(self):
        lti = InvertedPendulum().linearize([0.0, 0.0])
        self.assertTrue(controllability(lti).is_full_rank)
        self.assertTrue(observability(lti).is_full_rank)
        self.assertEqual(
            controllability(lti).rank, controllability(lti.A(), lti.B()).rank
        )

    def test_lqr_at_operating_point_takes_method_and_eps(self):
        plant = InvertedPendulum()
        Q, R = np.diag([10.0, 1.0]), [[1.0]]
        fd = lqr_at_operating_point(plant, [0.0, 0.0], Q, R, method="fd", eps=1e-5)
        auto = lqr_at_operating_point(plant, [0.0, 0.0], Q, R)
        np.testing.assert_allclose(fd.params["K"], auto.params["K"], atol=1e-4)

    def test_discretize_names_its_integrator(self):
        disc = discretize(Pendulum(), 0.05, integrator="euler")
        self.assertEqual(disc.integrator, "euler")
        np.testing.assert_allclose(
            disc.jacobian("step", "x")[0], [1.0, 0.05], atol=1e-8
        )
        with self.assertRaises(TypeError):
            discretize(Pendulum(), 0.05, method="euler")

    def test_old_channel_keywords_are_gone(self):
        with self.assertRaises(TypeError):
            bode(Pendulum(), input_port="u")
        with self.assertRaises(TypeError):
            linearize_matrices(Pendulum(), inputs=["u"])
        with self.assertRaises(TypeError):
            linearize_matrices(Pendulum(), epsilon=1e-6)
        loop = _closed_loop()
        with self.assertRaisesRegex(TypeError, "integer index"):
            linearize_matrices(loop, of=("ctl", "u"))  # the old (sys_id, port_id) form
        with self.assertRaisesRegex(TypeError, "one channel"):
            bode(loop, of=["y", "y"])
        with self.assertRaisesRegex(TypeError, "LTISystem"):
            controllability(np.eye(2))


if __name__ == "__main__":
    unittest.main()
