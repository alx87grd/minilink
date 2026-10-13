"""The Error-block side of ``@``: compensators, ``feedback``, port layouts."""

from __future__ import annotations

import unittest

import numpy as np

from minilink import (
    PID,
    DoubleIntegrator,
    Gain,
    ImpedanceController,
    Integrator,
    Lag,
    Lead,
    Pendulum,
    ProportionalController,
    Step,
    TransferFunction,
    feedback,
)
from minilink.core.feedback import block_roles, command_port, roles
from minilink.core.system import DynamicSystem


class _TwoInTwoOut(DynamicSystem):
    """dx = -x + u, y = x on two channels."""

    def __init__(self):
        super().__init__(n=2, input_dim=2, output_dim=2)

    def f(self, x, u, t=0, params=None):
        return -x + u


class _TwoInThreeOut(DynamicSystem):
    def __init__(self):
        super().__init__(n=3, input_dim=2, output_dim=3)

    def f(self, x, u, t=0, params=None):
        return -x + np.array([u[0], u[1], u[0] + u[1]])


def _poles(diagram):
    return np.sort_complex(np.linalg.eigvals(diagram.jacobian("f", "x")))


def _damped_pendulum():
    plant = Pendulum()
    plant.params["d"] = 0.5
    return plant


class TestJunction(unittest.TestCase):
    def test_compensator_at_plant_inserts_junction_and_selector(self):
        T = PID(Kp=20.0, Kd=2.0, tau=0.05) @ _damped_pendulum()
        self.assertEqual(list(T.inputs), ["r"])
        self.assertEqual(list(T.outputs), ["y"])
        self.assertEqual(list(T.subsystems), ["ctl", "sys", "demux", "error"])
        self.assertEqual(
            T.subsystems["demux"].dims, [1, 1]
        )  # theta, the first component
        self.assertEqual(
            T.connections["error"], {"+": ("input", "r"), "-": ("demux", "y[0]")}
        )
        self.assertEqual(T.connections["ctl"]["e"], ("error", "e"))
        self.assertEqual(T.connections["demux"]["y"], ("sys", "y"))

    def test_series_then_unity_equals_compensator_at_plant_and_leaves_the_loop_gain_alone(
        self,
    ):
        C, G = PID(Kp=20.0, Ki=5.0, Kd=2.0, tau=0.05), _damped_pendulum()
        L = C >> G
        T1 = C @ G
        T2 = feedback(L)
        np.testing.assert_allclose(_poles(T1), _poles(T2), atol=1e-9)
        self.assertEqual(list(L.inputs), ["e"])
        self.assertEqual(list(L.subsystems), ["ctl", "sys"])

    def test_series_diagram_at_plant_leaves_both_operands_alone(self):
        L = Gain(2.0, dim=1) >> Gain(3.0, dim=1)
        T = L @ Pendulum()
        self.assertEqual(list(T.subsystems), ["gain", "gain2", "sys", "demux", "error"])
        self.assertEqual(list(L.subsystems), ["gain", "gain2"])
        self.assertEqual(L.connections["output"], {"y": ("gain2", "y")})
        G = Integrator() >> TransferFunction([1.0], [0.5, 1.0])
        T = L @ G
        self.assertEqual(list(T.subsystems), ["gain", "gain2", "sys", "sys2", "error"])
        self.assertEqual(list(L.subsystems), ["gain", "gain2"])
        self.assertEqual(list(G.subsystems), ["sys", "sys2"])

    def test_closed_loop_poles_are_the_roots_of_one_plus_L(self):
        C, G = PID(Kp=20.0, Ki=5.0, Kd=2.0, tau=0.05), _damped_pendulum()
        L = C >> G
        T = C @ G
        loop_tf = L.transfer_function()  # L(s) of the wired diagram
        roots = np.roots(np.polyadd(loop_tf.denominator, loop_tf.numerator))
        np.testing.assert_allclose(_poles(T), np.sort_complex(roots), atol=1e-6)

    def test_vector_loop_uses_a_vector_junction(self):
        T = PID(Kp=[3.0, 5.0], dof=2) @ _TwoInTwoOut()
        self.assertEqual(list(T.subsystems), ["ctl", "sys", "error"])
        self.assertEqual(T.subsystems["error"].dim, 2)
        self.assertEqual(T.inputs["r"].dim, 2)
        self.assertEqual(T.jacobian("f", "x").shape, (6, 6))

    def test_component_gain_and_sensor_return_paths(self):
        L = PID(Kp=20.0, Kd=2.0) >> _damped_pendulum()
        T = feedback(L, of=("y", 1))
        self.assertEqual(T.subsystems["demux"].dims, [1, 1])
        self.assertEqual(T.connections["error"]["-"], ("demux", "y[1]"))
        T = feedback(L, 0.5)
        self.assertEqual(list(T.subsystems), ["ctl", "sys", "demux", "filter", "error"])
        np.testing.assert_allclose(T.subsystems["filter"].params["K"], [[0.5]])
        T = feedback(L, filter=TransferFunction([1.0], [0.1, 1.0]))
        self.assertIn("filter", T.subsystems)
        self.assertEqual(T.connections["error"]["-"], ("filter", "y"))
        with self.assertRaisesRegex(TypeError, "needs a plant on the right"):
            L @ 0.5

    def test_sign_and_plant_alone(self):
        T = feedback(_damped_pendulum(), sign=+1.0)
        np.testing.assert_allclose(T.subsystems["sum"].signs, [1.0, 1.0])
        T = feedback(_damped_pendulum())
        self.assertEqual(list(T.subsystems), ["sys", "demux", "error"])

    def test_mismatches_are_refused_with_guidance(self):
        with self.assertRaisesRegex(ValueError, "Cannot close the loop"):
            feedback(_TwoInThreeOut())
        with self.assertRaisesRegex(ValueError, "selects one component"):
            feedback(_TwoInTwoOut(), of=("y", 0))
        with self.assertRaisesRegex(ValueError, "of index must be in"):
            feedback(_damped_pendulum(), of=("y", 5))

    def test_a_block_after_a_two_port_controller_is_refused_not_miswired(self):
        from minilink import Saturation, StateFeedbackController

        K = np.array([[10.0, 2.0]])
        for plant in (Pendulum(), DoubleIntegrator()):
            with self.subTest(plant=plant.name):
                with self.assertRaisesRegex(
                    ValueError, "measurement on 'x'.*DiagramSystem.connect"
                ):
                    (StateFeedbackController(K) >> Saturation()) @ plant

    def test_unknown_or_unused_keywords_are_refused(self):
        from minilink import closed_loop

        with self.assertRaisesRegex(ValueError, "feedback must be one of"):
            closed_loop(PID(), Pendulum(), feedback="bogus")
        with self.assertRaisesRegex(TypeError, "needs a plant on the right"):
            closed_loop(Integrator(), 2.0)
        with self.assertRaisesRegex(ValueError, "feedback not honoured"):
            closed_loop(PID(), Pendulum(), feedback="qdq")
        with self.assertRaisesRegex(ValueError, "plant_output_port not honoured"):
            closed_loop(PID(), Pendulum(), plant_output_port="x")
        with self.assertRaisesRegex(ValueError, "output_port not honoured"):
            closed_loop(PID(), Pendulum(), output_port="z")

    def test_two_port_wiring_is_unchanged(self):
        T = ImpedanceController() @ Pendulum()
        self.assertEqual(list(T.subsystems), ["ctl", "sys"])
        T = PID(Kp=5.0, ports="reference") @ Integrator()
        self.assertEqual(list(T.subsystems), ["ctl", "sys"])
        self.assertIsNone(block_roles(PID(ports="reference")).entry)
        self.assertEqual(block_roles(PID()).entry, "e")
        self.assertEqual(block_roles(TransferFunction([1.0], [1.0, 1.0])).entry, "u")
        self.assertEqual(block_roles(Lead()).entry, "e")

    def test_step_reference_drives_the_classical_loop(self):
        plant = _damped_pendulum()
        loop = (
            Step(final_value=0.3, step_time=1.0)
            >> PID(Kp=20.0, Kd=2.0, tau=0.05) @ plant
        )
        self.assertEqual(list(loop.subsystems), ["ref", "ctl", "sys", "demux", "error"])
        traj = loop.compute_trajectory(tf=8.0, verbose=False)
        static_gain = 20.0 / (20.0 + 4.905 / 0.5)  # Kp / (Kp + wn^2 / k)
        self.assertAlmostEqual(traj.x[-2, -1], 0.3 * static_gain, delta=0.01)


class TestSeriesRule(unittest.TestCase):
    """``A >> B`` connects one output to one input by a fixed rule on the edges."""

    def test_one_output_to_one_input_whatever_the_names(self):
        L = PID() >> Pendulum()
        self.assertEqual(list(L.inputs), ["e"])
        self.assertEqual(L.connections["sys"]["u"], ("ctl", "u"))
        self.assertEqual(list(L.outputs), ["y"])

    def test_a_shared_name_wins_over_the_textbook_pair(self):
        from minilink import StateFeedbackController

        L = Pendulum() >> StateFeedbackController(np.array([[10.0, 2.0]]))
        self.assertEqual(L.connections["ctl"]["x"], ("sys", "x"))
        self.assertIsNone(L.connections["ctl"]["r"])

    def test_y_to_u_when_no_name_is_shared(self):
        L = TransferFunction([1.0], [1.0, 3.0, 2.0]) >> Integrator()
        self.assertEqual(L.connections["sys2"]["u"], ("sys", "y"))

    def test_a_leaf_puts_all_its_inputs_on_the_edge(self):
        from minilink import Saturation, StateFeedbackController

        L = StateFeedbackController(np.array([[10.0, 2.0]])) >> Saturation()
        self.assertEqual(list(L.inputs), ["x", "r"])

    def test_a_source_into_a_reference_and_measurement_block_is_refused(self):
        with self.assertRaisesRegex(ValueError, "would drive the measurement"):
            Step() >> ProportionalController()
        with self.assertRaisesRegex(ValueError, "one output to one input"):
            ProportionalController() >> ProportionalController()

    def test_feedback_closes_on_the_command_input_or_refuses(self):
        T = feedback(PID() >> Integrator())
        self.assertEqual(T.connections["ctl"]["e"], ("error", "e"))
        with self.assertRaisesRegex(ValueError, "measurement on 'y' \\(on its edge\\)"):
            feedback(ProportionalController() >> Integrator())
        from minilink import DiagramSystem

        two_outputs = DiagramSystem()
        two_outputs.add_subsystem(Pendulum(), "plant")
        two_outputs.add_input_port("u", dim=1)
        two_outputs.connect("input", "u", "plant", "u")
        two_outputs.connect_new_output_port("plant", "q", "q")
        two_outputs.connect_new_output_port("plant", "dq", "dq")
        with self.assertRaisesRegex(ValueError, "Cannot choose what comes back"):
            feedback(two_outputs)


class TestPortLayouts(unittest.TestCase):
    def test_pid_layouts_share_one_law(self):
        plant = Integrator()
        poles_error = _poles(PID(Kp=4.0, Ki=1.0, Kd=0.5, tau=0.1) @ plant)
        poles_reference = _poles(
            PID(Kp=4.0, Ki=1.0, Kd=0.5, tau=0.1, ports="reference") @ plant
        )
        np.testing.assert_allclose(poles_error, poles_reference, atol=1e-9)
        pid = PID(Kp=2.0, Ki=1.0, Kd=0.0, ports="reference")
        self.assertEqual(list(pid.inputs), ["r", "y"])
        np.testing.assert_allclose(pid.ctl(np.zeros(2), np.array([1.0, 0.25])), [1.5])
        pid = PID(Kp=2.0, Ki=1.0, Kd=0.0)
        np.testing.assert_allclose(pid.ctl(np.zeros(2), np.array([0.75])), [1.5])
        with self.assertRaises(ValueError):
            PID(ports="both")

    def test_transfer_function_and_proportional_layouts(self):
        plant = Integrator()
        C = TransferFunction([2.0, 20.0], [0.05, 1.0])
        np.testing.assert_allclose(
            _poles(C @ plant),
            _poles(
                TransferFunction([2.0, 20.0], [0.05, 1.0], ports="reference") @ plant
            ),
            atol=1e-9,
        )
        reference = TransferFunction([1.0], [1.0, 1.0], ports="reference")
        self.assertEqual(
            (list(reference.inputs), list(reference.outputs)), (["r", "y"], ["u"])
        )
        np.testing.assert_allclose(
            reference.f(np.array([0.0]), np.array([1.0, 0.25])), [0.75]
        )
        np.testing.assert_allclose(
            _poles(ProportionalController(3.0, ports="error") @ plant),
            _poles(ProportionalController(3.0) @ plant),
            atol=1e-9,
        )

    def test_unity_feedback_refuses_a_reference_layout(self):
        for block in (PID(ports="reference"), ProportionalController()):
            with self.subTest(block=block.name):
                with self.assertRaisesRegex(TypeError, "needs a plant on the right"):
                    block @ 1
                with self.assertRaisesRegex(ValueError, "measurement on 'y'"):
                    feedback(block)

    def test_lead_and_lag(self):
        lead = Lead(K=2.0, z=1.0, p=10.0)
        np.testing.assert_allclose(lead.numerator, [2.0, 2.0])
        np.testing.assert_allclose(lead.denominator, [1.0, 10.0])
        self.assertEqual(lead.name, "Lead")
        self.assertEqual(list(lead.inputs), ["e"])
        self.assertEqual(list(lead.outputs), ["u"])
        np.testing.assert_allclose(Lag(K=1.0, z=1.0, p=0.1).poles, [-0.1])
        with self.assertRaises(ValueError):
            Lead(z=10.0, p=1.0)
        with self.assertRaises(ValueError):
            Lag(z=0.1, p=1.0)
        L = Lead() >> DoubleIntegrator()
        self.assertEqual(list(L.inputs), ["e"])
        self.assertEqual(list(L.subsystems), ["ctl", "sys"])


class TestLoopInputs(unittest.TestCase):
    """``closed_loop(r=, w=, v=)``: the reference, load disturbance and noise inputs."""

    def setUp(self):
        self.H = TransferFunction([1.0], [1.0, 3.0, 2.0])

    def layouts(self):
        return (
            ("error", PID(4.0, 2.0, 0.5)),
            ("reference", PID(4.0, 2.0, 0.5, ports="reference")),
        )

    def test_defaults_keep_the_single_reference_input(self):
        from minilink.core.composition import closed_loop

        for layout, C in self.layouts():
            with self.subTest(layout=layout):
                self.assertEqual(list(closed_loop(C, self.H).inputs), ["r"])
                self.assertEqual(list((C @ self.H).inputs), ["r"])

    def test_each_flag_adds_or_drops_its_port(self):
        from minilink.core.composition import closed_loop

        for layout, C in self.layouts():
            with self.subTest(layout=layout):
                loop = closed_loop(C, self.H, w=True, v=True)
                self.assertEqual(list(loop.inputs), ["r", "w", "v"])
                loop = closed_loop(C, self.H, r=False, w=True)
                self.assertEqual(list(loop.inputs), ["w"])

    def test_w_and_v_enter_where_the_formulas_say(self):
        from minilink.analysis import (
            frequency_response,
            load_sensitivity,
            noise_sensitivity,
            sensitivity,
        )
        from minilink.core.composition import closed_loop

        w = np.logspace(-2, 3, 200)
        pieces = dict(plant=self.H, controller=PID(4.0, 2.0, 0.5))
        PS = frequency_response(load_sensitivity(**pieces), w=w)[1]
        CS = frequency_response(noise_sensitivity(**pieces), w=w)[1]
        S = frequency_response(sensitivity(**pieces), w=w)[1]
        for layout, C in self.layouts():
            with self.subTest(layout=layout):
                loop = closed_loop(C, self.H, w=True, v=True)
                np.testing.assert_allclose(
                    frequency_response(loop, of="sys:y", wrt="w", w=w)[1], PS, atol=1e-9
                )
                np.testing.assert_allclose(
                    frequency_response(loop, of="ctl:u", wrt="v", w=w)[1],
                    -CS,
                    atol=1e-9,
                )
                np.testing.assert_allclose(
                    frequency_response(loop, of="disturbance:y", wrt="w", w=w)[1],
                    S,
                    atol=1e-9,
                )

    def test_a_number_on_the_right_is_refused(self):
        from minilink.core.composition import closed_loop

        with self.assertRaisesRegex(TypeError, "needs a plant on the right"):
            closed_loop(self.H, 1, w=True)
        self.assertEqual(list(closed_loop(PID(), self.H, r=False).inputs), [])


class TestDispatchTable(unittest.TestCase):
    """Which loop ``@`` builds for each library controller, pinned as it is today.

    Error-driven blocks get an Error junction on ``e``; two-port blocks read the
    plant's ``y`` or ``x`` directly. A change to any row is a change to the loop
    a student gets, and lands with its own decision.
    """

    def rows(self):
        from minilink import (
            PD,
            PI,
            ComputedTorqueController,
            ImpedanceIntegralController,
            NeuralPolicyController,
            SingleMass,
            SlidingModeController,
            StateFeedbackController,
            TwoLinkManipulator,
        )

        K = np.array([[10.0, 2.0]])
        G = TransferFunction([1.0], [1.0, 3.0, 2.0])
        junction = {"e": ("error", "e")}
        reads_y = {"r": ("input", "r"), "y": ("sys", "y")}
        reads_x = {"x": ("sys", "x"), "r": ("input", "r")}
        return (
            ("PID", PID() @ Pendulum(), ["ctl", "sys", "demux", "error"], junction),
            ("PI", PI() @ SingleMass(), ["ctl", "sys", "error"], junction),
            ("PD", PD() @ DoubleIntegrator(), ["ctl", "sys", "error"], junction),
            ("Lead", Lead() @ G, ["ctl", "sys", "error"], junction),
            ("Lag", Lag() @ G, ["ctl", "sys", "error"], junction),
            (
                "TransferFunction error",
                TransferFunction([2.0, 1.0], [1.0, 5.0], ports="error") @ G,
                ["ctl", "sys", "error"],
                junction,
            ),
            (
                "TransferFunction reference",
                TransferFunction([2.0, 1.0], [1.0, 5.0], ports="reference") @ G,
                ["ctl", "sys"],
                reads_y,
            ),
            (
                "Proportional error",
                ProportionalController(ports="error") @ SingleMass(),
                ["ctl", "sys", "error"],
                junction,
            ),
            (
                "Proportional",
                ProportionalController() @ SingleMass(),
                ["ctl", "sys"],
                reads_y,
            ),
            (
                "PID reference",
                PID(ports="reference") @ Integrator(),
                ["ctl", "sys"],
                reads_y,
            ),
            (
                "StateFeedback",
                StateFeedbackController(K) @ Pendulum(),
                ["ctl", "sys"],
                reads_x,
            ),
            (
                "StateFeedback N",
                StateFeedbackController(K, N=np.array([[10.0]])) @ DoubleIntegrator(),
                ["ctl", "sys"],
                reads_x,
            ),
            ("Impedance", ImpedanceController() @ Pendulum(), ["ctl", "sys"], reads_y),
            (
                "ImpedanceIntegral",
                ImpedanceIntegralController() @ Pendulum(),
                ["ctl", "sys"],
                reads_y,
            ),
            (
                "NeuralPolicy",
                NeuralPolicyController(Pendulum()) @ Pendulum(),
                ["ctl", "sys"],
                {"x": ("sys", "x")},
            ),
            (
                "ComputedTorque",
                ComputedTorqueController(TwoLinkManipulator()) @ TwoLinkManipulator(),
                ["ctl", "sys"],
                reads_y,
            ),
            (
                "SlidingMode",
                SlidingModeController(TwoLinkManipulator()) @ TwoLinkManipulator(),
                ["ctl", "sys"],
                reads_y,
            ),
        )

    def test_each_controller_closes_the_loop_it_closes_today(self):
        for name, loop, ids, ctl_inputs in self.rows():
            with self.subTest(controller=name):
                self.assertEqual(list(loop.subsystems), ids)
                self.assertEqual(loop.connections["ctl"], ctl_inputs)

    def test_a_plant_alone_closes_through_the_junction(self):
        loop = feedback(TransferFunction([1.0], [1.0, 3.0, 2.0]))
        self.assertEqual(list(loop.subsystems), ["sys", "error"])
        self.assertEqual(
            loop.connections["error"], {"+": ("input", "r"), "-": ("sys", "y")}
        )


class TestResolver(unittest.TestCase):
    """The roles every tool reads: the port names of a block, the reserved names of a diagram."""

    def test_block_roles_read_the_port_names(self):
        from minilink import PendulumWithNoisePort, StateFeedbackController

        pid = block_roles(PID())
        self.assertEqual(
            (pid.measurements, pid.reference, pid.command), ((), None, "u")
        )
        self.assertEqual(pid.entry, "e")
        self.assertTrue(pid.error_driven and pid.stateful)

        p = block_roles(ProportionalController())
        self.assertEqual((p.measurement, p.reference, p.entry), ("y", "r", None))
        self.assertFalse(p.error_driven or p.stateful)

        lqr = block_roles(StateFeedbackController(np.array([[1.0, 1.0]])))
        self.assertEqual((lqr.measurements, lqr.reference), (("x",), "r"))

        compensator = block_roles(TransferFunction([1.0], [1.0, 1.0]))
        self.assertEqual((compensator.measurements, compensator.entry), ((), "u"))

        plant = block_roles(PendulumWithNoisePort())
        self.assertEqual(
            (plant.measurements, plant.command, plant.entry), ((), None, None)
        )

    def test_command_port_is_u_or_the_one_free_input(self):
        from minilink import PendulumWithNoisePort

        self.assertEqual(command_port(Pendulum()), "u")
        self.assertEqual(command_port(PendulumWithNoisePort()), "u")
        self.assertEqual(command_port(PID() @ Pendulum()), "r")
        self.assertEqual(command_port(Lead()), "e")
        with self.assertRaisesRegex(ValueError, "input port named 'u'"):
            command_port(ProportionalController())
        with self.assertRaisesRegex(ValueError, "input port named 'u'"):
            command_port(Step())

    def test_roles_read_the_reserved_names(self):
        from minilink import DiagramSystem, StateFeedbackController

        K = np.array([[10.0, 2.0]])
        plant, ctl = Pendulum(), StateFeedbackController(K)
        hand_wired = DiagramSystem()
        hand_wired.add_subsystem(ctl, "controller")
        hand_wired.add_subsystem(plant, "plant")
        who = roles(hand_wired)
        self.assertIs(who.plant, plant)
        self.assertIs(who.controller, ctl)
        self.assertIsNone(who.estimator)

        loop = ctl @ plant
        self.assertIs(roles(loop).plant, plant)
        self.assertIs(roles(loop).controller, ctl)

        leaf = roles(plant)
        self.assertIs(leaf.plant, plant)
        self.assertIsNone(leaf.controller)

        empty = roles(DiagramSystem())
        self.assertEqual(
            (empty.plant, empty.controller, empty.estimator), (None, None, None)
        )

    def test_roles_find_an_estimator_inside_the_controller(self):
        from minilink import DiagramSystem, StateFeedbackController

        gain, observer = StateFeedbackController(np.array([[1.0, 1.0]])), Integrator()
        box = DiagramSystem()
        box.add_subsystem(observer, "estimator")
        box.add_subsystem(gain, "gain")
        loop = DiagramSystem()
        loop.add_subsystem(box, "controller")
        loop.add_subsystem(Pendulum(), "plant")
        self.assertIs(roles(loop).estimator, observer)

    def test_roles_of_a_sampled_loop_come_from_its_two_sides(self):
        from minilink import StateFeedbackController

        plant, ctl = Pendulum(), StateFeedbackController(np.array([[10.0, 2.0]]))
        sampled = ctl % 0.01 @ plant
        self.assertIs(roles(sampled).plant, plant)
        self.assertIs(roles(sampled).controller, ctl)

    def test_a_second_block_under_a_reserved_name_is_refused(self):
        from minilink import DiagramSystem

        first, second = DiagramSystem(), DiagramSystem()
        first.add_subsystem(Pendulum(), "plant")
        second.add_subsystem(Pendulum(), "plant")
        with self.assertRaisesRegex(ValueError, "one plant per diagram level"):
            first.add_subsystem(Pendulum(), "plant")
        with self.assertRaisesRegex(ValueError, "one plant per diagram level"):
            first + second


if __name__ == "__main__":
    unittest.main()
