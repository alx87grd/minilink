import unittest
import numpy as np
import pytest
from minilink.core.system import DynamicSystem, System, VectorSignal
from minilink.core.discrete.step_system import StepSystem


class TestCoreComponents(unittest.TestCase):
    def test_vector_signal(self):
        v = VectorSignal("test", nominal_value=[1.0, 2.0])
        self.assertEqual(v.dim, 2)
        np.testing.assert_array_equal(v.nominal_value, np.array([1.0, 2.0]))

    def test_vector_signal_rejects_wrong_nominal_shape(self):
        with self.assertRaises(ValueError):
            VectorSignal("test", dim=2, nominal_value=[1.0, 2.0, 3.0])

    def test_vector_signal_rejects_negative_dimension(self):
        with self.assertRaises(ValueError):
            VectorSignal("test", dim=-1)

    def test_system_initialization_has_no_implicit_ports(self):
        sys = System(n=2)
        self.assertEqual(sys.n, 2)
        self.assertEqual(sys.m, 0)
        self.assertEqual(sys.p, 0)
        self.assertEqual(sys.inputs, {})
        self.assertEqual(sys.outputs, {})

    def test_system_rejects_negative_dimension(self):
        with self.assertRaises(ValueError):
            System(n=-1)

    def test_static_system(self):
        sys = System()
        self.assertEqual(sys.n, 0)
        self.assertEqual(sys.m, 0)
        self.assertEqual(sys.p, 0)

    def test_dynamic_system_standard_ports(self):
        sys = DynamicSystem(n=2, input_dim=1, output_dim=1, expose_state=True)
        self.assertEqual(sys.n, 2)
        self.assertEqual(sys.m, 1)
        self.assertEqual(sys.p, 1)
        self.assertEqual(sys.outputs["y"].dependencies, ())
        self.assertIn("x", sys.outputs)

    def test_add_input_port_dim_defaults(self):
        sys = System()
        sys.add_input_port("y", dim=2)
        self.assertEqual(sys.m, 2)
        np.testing.assert_array_equal(sys.inputs["y"].nominal_value, np.zeros(2))
        self.assertEqual(sys.inputs["y"].labels, ["y[0]", "y[1]"])
        self.assertEqual(sys.inputs["y"].units, ["", ""])
        np.testing.assert_array_equal(sys.inputs["y"].lower_bound, [-np.inf, -np.inf])
        np.testing.assert_array_equal(sys.inputs["y"].upper_bound, [np.inf, np.inf])

    def test_port_dimension_inference_and_metadata_validation(self):
        sys = System()
        sys.add_input_port("y", nominal_value=[1.0, 2.0])
        self.assertEqual(sys.inputs["y"].dim, 2)
        sys.add_input_port("z", labels=["z0", "z1"], units=["m", "m"])
        self.assertEqual(sys.inputs["z"].dim, 2)
        with self.assertRaisesRegex(ValueError, "nominal_value"):
            sys.add_input_port("bad", dim=2, nominal_value=[1.0, 2.0, 3.0])

    def test_add_output_port_updates_primary_p_only_for_y(self):
        sys = System()
        sys.add_output_port("x", dim=2, function=sys.compute_state)
        self.assertEqual(sys.p, 0)
        sys.add_output_port("y", dim=3, function=sys.h)
        self.assertEqual(sys.p, 3)

    def test_output_dependency_validation(self):
        sys = System()
        sys.add_input_port("u")
        with self.assertRaisesRegex(ValueError, "Unknown input dependencies"):
            sys.add_output_port("y", function=sys.h, dependencies=("missing",))

    def test_get_port_values_from_u_selection(self):
        sys = System()
        sys.add_input_port("r")
        sys.add_input_port("y", dim=2)
        u = np.array([1.0, 2.0, 3.0])
        signals = sys.get_port_values_from_u(u)
        np.testing.assert_array_equal(signals["r"], np.array([1.0]))
        np.testing.assert_array_equal(signals["y"], np.array([2.0, 3.0]))
        np.testing.assert_array_equal(sys.get_port_values_from_u(u, "r"), [1.0])
        r, y = sys.get_port_values_from_u(u, "r", "y")
        np.testing.assert_array_equal(r, [1.0])
        np.testing.assert_array_equal(y, [2.0, 3.0])


from minilink.blocks.basic import Integrator
from minilink.blocks.nonlinear import Saturation
from minilink.blocks.sources import Step, WhiteNoise
from minilink.control.impedance import Impedance
from minilink.control.siso import P
from minilink.core.closed_loop import closed_loop
from minilink.core.diagram import DiagramSystem
from minilink.dynamics.catalog.manipulators.arms import TwoLinkManipulator
from minilink.dynamics.catalog.pendulum.pendulum import Pendulum


class TestDiagramCompositionShortcuts(unittest.TestCase):
    def test_add_operator_adds_subsystems_without_wiring(self):
        diagram = Step(final_value=[1.0]) + P(ports="reference") + Integrator()
        self.assertIsInstance(diagram, DiagramSystem)
        self.assertEqual(list(diagram.subsystems), ["Step", "P", "Integrator"])
        self.assertEqual(diagram.connections["P"]["r"], None)
        self.assertEqual(diagram.connections["P"]["y"], None)
        self.assertEqual(diagram.connections["Integrator"]["u"], None)

    def test_add_operator_uniquifies_repeated_subsystem_ids(self):
        diagram = Integrator() + Integrator()
        self.assertEqual(list(diagram.subsystems), ["Integrator", "Integrator2"])
        self.assertEqual(diagram.connections["Integrator"]["u"], None)
        self.assertEqual(diagram.connections["Integrator2"]["u"], None)

    def test_series_operator_wires_source_to_default_input(self):
        diagram = Step(final_value=[1.0], step_time=0.0) >> Integrator()
        self.assertEqual(diagram.connections["Integrator"]["u"], ("Step", "y"))
        self.assertEqual(diagram.connections["output"]["y"], ("Integrator", "y"))

    def test_series_operator_exposes_first_input_and_final_output(self):
        diagram = Integrator() >> Integrator()
        self.assertIn("u", diagram.inputs)
        self.assertEqual(diagram.connections["Integrator"]["u"], ("input", "u"))
        self.assertEqual(diagram.connections["Integrator2"]["u"], ("Integrator", "y"))
        self.assertEqual(diagram.connections["output"]["y"], ("Integrator2", "y"))
        dx = diagram.f(np.array([1.0, 2.0]), np.array([3.0]))
        np.testing.assert_allclose(dx, np.array([3.0, 1.0]))

    def test_matmul_operator_builds_closed_loop_diagram(self):
        diagram = Impedance() @ Pendulum()
        self.assertIn("r", diagram.inputs)
        self.assertIn("y", diagram.outputs)
        self.assertEqual(diagram.connections["controller"]["r"], ("input", "r"))
        self.assertEqual(diagram.connections["controller"]["y"], ("plant", "y"))
        self.assertEqual(diagram.connections["plant"]["u"], ("controller", "u"))
        self.assertEqual(diagram.connections["output"]["y"], ("plant", "y"))
        self.assertEqual(diagram.outputs["y"].dim, 2)

    def test_matmul_operator_wires_state_feedback_when_controller_uses_x(self):
        from minilink.control.design import lqr
        from minilink.dynamics.catalog.pendulum.cartpole import CartPole

        plant = CartPole()
        controller = lqr(
            plant,
            np.diag([1.0, 10.0, 1.0, 1.0]),
            np.array([[0.1]]),
            x_bar=[0.0, np.pi, 0.0, 0.0],
        )
        diagram = controller @ plant
        self.assertEqual(diagram.connections["controller"]["x"], ("plant", "x"))
        self.assertEqual(diagram.connections["plant"]["u"], ("controller", "u"))

    def test_operators_leave_their_operands_alone(self):
        left = Step(final_value=[1.0], step_time=0.0) >> Integrator()
        left.name = "chain"
        left.x0 = np.array([2.0])

        chain = left >> Integrator()
        side_by_side = left + Pendulum()

        for result in (chain, side_by_side):
            self.assertIsNot(result, left)
            self.assertEqual(result.name, "chain")
            self.assertEqual(result.x0[0], 2.0)
            self.assertIs(
                result.subsystems["Integrator"], left.subsystems["Integrator"]
            )
        self.assertEqual(list(left.subsystems), ["Step", "Integrator"])
        self.assertEqual(left.connections["output"]["y"], ("Integrator", "y"))
        self.assertEqual(list(chain.subsystems), ["Step", "Integrator", "Integrator2"])
        self.assertEqual(chain.connections["Integrator2"]["u"], ("Integrator", "y"))
        self.assertEqual(
            list(side_by_side.subsystems), ["Step", "Integrator", "Pendulum"]
        )
        self.assertIsNone(side_by_side.connections["Pendulum"]["u"])

    def test_series_operator_flattens_source_into_closed_loop_diagram(self):
        diagram = Step(final_value=[1.0], step_time=0.0) >> Impedance() @ Pendulum()
        self.assertEqual(list(diagram.subsystems), ["Step", "controller", "plant"])
        self.assertFalse(
            any((isinstance(sys, DiagramSystem) for sys in diagram.subsystems.values()))
        )
        self.assertEqual(diagram.connections["controller"]["r"], ("Step", "y"))
        self.assertEqual(diagram.connections["controller"]["y"], ("plant", "y"))
        self.assertEqual(diagram.connections["plant"]["u"], ("controller", "u"))
        self.assertEqual(diagram.connections["output"]["y"], ("plant", "y"))
        self.assertNotIn("r", diagram.inputs)

    def test_add_operator_flattens_diagrams_without_cross_wiring(self):
        left = Step(final_value=[1.0]) + Integrator()
        right = Impedance() @ Pendulum()
        diagram = left + right
        self.assertEqual(
            list(diagram.subsystems), ["Step", "Integrator", "controller", "plant"]
        )
        self.assertFalse(
            any((isinstance(sys, DiagramSystem) for sys in diagram.subsystems.values()))
        )
        self.assertEqual(diagram.connections["Integrator"]["u"], None)
        self.assertEqual(diagram.connections["controller"]["r"], ("input", "r"))
        self.assertEqual(diagram.connections["controller"]["y"], ("plant", "y"))
        self.assertEqual(diagram.connections["plant"]["u"], ("controller", "u"))

    def test_series_operator_flattens_diagram_to_diagram_boundary(self):
        left = Step(final_value=[1.0], step_time=0.0) >> Integrator()
        right = Gain(2.0, dim=1) >> Gain(3.0, dim=1)
        diagram = left >> right
        self.assertEqual(
            list(diagram.subsystems), ["Step", "Integrator", "Gain", "Gain2"]
        )
        self.assertFalse(
            any((isinstance(sys, DiagramSystem) for sys in diagram.subsystems.values()))
        )
        self.assertEqual(diagram.connections["Integrator"]["u"], ("Step", "y"))
        self.assertEqual(diagram.connections["Gain"]["u"], ("Integrator", "y"))
        self.assertEqual(diagram.connections["Gain2"]["u"], ("Gain", "y"))
        self.assertEqual(diagram.connections["output"]["y"], ("Gain2", "y"))
        self.assertNotIn("u", diagram.inputs)

    def test_series_into_closed_loop_without_free_boundary_input_fails(self):
        diagram = Step(final_value=[1.0], step_time=0.0) >> Impedance() @ Pendulum()
        with self.assertRaisesRegex(ValueError, "no free input port"):
            WhiteNoise() >> diagram
        self.assertEqual(diagram.connections["plant"]["u"], ("controller", "u"))

    def test_series_on_a_hand_wired_diagram_reads_its_declared_output(self):
        diagram = DiagramSystem()
        diagram.add_subsystem(Integrator(), "plant")
        diagram.add_input_port("u", dim=1)
        diagram.connect("input", "u", "plant", "u")
        diagram.connect_new_output_port("plant", "y", "y")
        diagram.add_subsystem(P(ports="reference"), "monitor")  # added last
        chain = diagram >> Saturation()
        self.assertEqual(chain.connections["Saturation"]["u"], ("plant", "y"))

    def test_autowire_connects_unique_matches_without_overwrites(self):
        diagram = (
            Step(final_value=[1.0], port="r") + P(ports="reference") + Integrator()
        ).autowire(strict=True)
        self.assertEqual(diagram.connections["P"]["r"], ("Step", "r"))
        self.assertEqual(diagram.connections["P"]["y"], ("Integrator", "y"))
        self.assertEqual(diagram.connections["Integrator"]["u"], ("P", "u"))
        diagram.connect("Step", "r", "Integrator", "u")
        diagram.autowire(strict=True)
        self.assertEqual(diagram.connections["Integrator"]["u"], ("Step", "r"))

    def test_autowire_reads_names_only(self):
        with self.assertWarnsRegex(UserWarning, "no output named 'r'"):
            diagram = (Step(final_value=[1.0]) + Impedance() + Pendulum()).autowire()
        self.assertIsNone(diagram.connections["Impedance"]["r"])
        self.assertEqual(diagram.connections["Impedance"]["y"], ("Pendulum", "y"))
        with self.assertRaisesRegex(ValueError, "no output named 'r'"):
            (Step(final_value=[1.0]) + Impedance() + Pendulum()).autowire(strict=True)

    def test_autowire_strict_refuses_ambiguous_matches(self):
        diagram = (
            Step(final_value=[1.0], port="r")
            + P(ports="reference")
            + P(ports="reference")
            + Integrator()
        )
        with self.assertRaisesRegex(
            ValueError, "Ambiguous autowire target Integrator:u"
        ):
            diagram.autowire(strict=True)
        self.assertEqual(diagram.connections["P"]["y"], None)
        self.assertEqual(diagram.connections["P2"]["y"], None)
        self.assertEqual(diagram.connections["Integrator"]["u"], None)

    def test_autowire_follows_renamed_ports_through_an_actuator(self):
        ref = Step(final_value=[1.0], port="r")
        ctl = P(2.0, ports="reference").rename_ports(u="u_cmd")
        sat = Saturation(-1, 1).rename_ports(u="u_cmd", y="u")
        diagram = (ref + ctl + sat + Integrator()).autowire(strict=True)
        ctl_id = diagram.subsystem_id(ctl)
        self.assertEqual(diagram.connections["Saturation"]["u_cmd"], (ctl_id, "u_cmd"))
        self.assertEqual(diagram.connections["Integrator"]["u"], ("Saturation", "u"))
        self.assertEqual(diagram.connections[ctl_id]["y"], ("Integrator", "y"))
        self.assertEqual(list(sat.inputs), ["u_cmd"])
        self.assertEqual(list(sat.outputs), ["u"])
        measured = P(ports="reference").rename_ports(y="y_meas")
        self.assertEqual(measured.outputs["u"].dependencies, ("r", "y_meas"))

    def test_rename_ports_refuses_a_silent_mistake(self):
        with self.assertRaisesRegex(ValueError, "has no port 'z'"):
            Integrator().rename_ports(z="u")
        with self.assertRaisesRegex(ValueError, "already a port"):
            P(ports="reference").rename_ports(r="y")
        with self.assertRaisesRegex(ValueError, "same name"):
            P(ports="reference").rename_ports(r="e", y="e")
        with self.assertRaisesRegex(ValueError, "is a diagram"):
            (Integrator() >> Integrator()).rename_ports(u="w")
        swapped = P(ports="reference").rename_ports(r="y", y="r")
        self.assertEqual(list(swapped.inputs), ["y", "r"])

    def test_expose_names_the_edge_of_a_box(self):
        from minilink import StateFeedback

        K = np.array([[10.0, 2.0]])
        box = (StateFeedback(K) + Saturation()).autowire()
        box.expose("x", "r", "y")
        self.assertEqual(list(box.inputs), ["x", "r"])
        self.assertEqual(list(box.outputs), ["y"])
        self.assertEqual(box.connections["StateFeedback"]["x"], ("input", "x"))
        self.assertEqual(box.connections["StateFeedback"]["r"], ("input", "r"))
        self.assertEqual(box.connections["Saturation"]["u"], ("StateFeedback", "u"))
        self.assertEqual(box.connections["output"]["y"], ("Saturation", "y"))

        fan_in = (Integrator() + Integrator()).expose("u")
        self.assertEqual(fan_in.connections["Integrator2"]["u"], ("input", "u"))
        chain = (Integrator() >> Integrator()).expose("x")  # the box's own state
        np.testing.assert_allclose(
            chain.outputs["x"].compute(np.array([1.0, 2.0]), np.zeros(1), 0.0, None),
            [1.0, 2.0],
        )
        with self.assertRaisesRegex(ValueError, "output of several blocks"):
            (Integrator() + Integrator()).expose("y")
        with self.assertRaisesRegex(ValueError, "neither an open input nor an output"):
            Integrator().__add__(Integrator()).expose("bogus")

    def test_a_mechanical_plant_closes_on_its_state_y(self):
        plant = TwoLinkManipulator()
        self.assertFalse({"q", "dq"} & set(plant.outputs))  # y is [q; dq]
        diagram = closed_loop(Impedance(dof=2), plant)
        self.assertEqual(list(diagram.subsystems), ["controller", "plant"])
        self.assertEqual(diagram.connections["controller"]["y"], ("plant", "y"))
        with self.assertRaisesRegex(ValueError, "declares measurement 'y' dim 2"):
            closed_loop(Impedance(dof=1), plant)


from minilink.core.closed_loop import closed_loop, resolve_standard_feedback


class TestStandardFeedback(unittest.TestCase):
    def test_y_loop_ports(self):
        ctl = P(0.5, ports="reference")
        plant = Integrator()
        wiring = resolve_standard_feedback(ctl, plant)
        self.assertEqual(wiring.control_out, "u")
        self.assertEqual(wiring.measurement_in, "y")
        self.assertEqual(wiring.plant_in, "u")
        self.assertEqual(wiring.plant_out, "y")

    def test_closed_loop_uses_shared_resolver(self):
        diagram = closed_loop(P(0.4, ports="reference"), Integrator())
        self.assertIn("controller", diagram.subsystems)
        self.assertIn("plant", diagram.subsystems)

    def test_u_ff_control_out(self):

        class _MpcLike:
            name = "mpc"
            inputs = {"y": type("P", (), {"dim": 1})()}
            outputs = {
                "u_ff": type("P", (), {"dim": 1})(),
                "x_ff": type("P", (), {"dim": 1})(),
            }

        plant = Integrator()
        wiring = resolve_standard_feedback(_MpcLike(), plant)
        self.assertEqual(wiring.control_out, "u_ff")


from minilink.control.state import StateFeedback
from minilink.core.system import System


class TestPortNamesDriveTheLoop(unittest.TestCase):
    def test_the_sweep_space_comes_from_the_ports_or_plot_space(self):
        from minilink import Impedance
        from minilink.core.roles import plot_space_of

        self.assertEqual(plot_space_of(P(0.5, ports="reference")), "error")
        self.assertEqual(plot_space_of(StateFeedback(np.array([[1.0, 1.0]]))), "state")
        self.assertEqual(plot_space_of(Integrator()), "measurement")
        self.assertEqual(plot_space_of(Impedance()), "error_qdq")

        class Measured(System):
            def __init__(self):
                super().__init__()
                self.add_input_port("y", dim=4)
                self.add_input_port("r", dim=1)
                self.add_output_port("u", dim=1, dependencies=("y", "r"))

        self.assertEqual(plot_space_of(Measured()), "measurement")
        self.assertIsNone(plot_space_of(Step(final_value=[1.0])))


from minilink.blocks.routing import Gain
from minilink.core.system import System


class TestSystemEvolutionMaps(unittest.TestCase):
    def test_system_has_no_f(self):
        self.assertFalse(hasattr(System, "f") or callable(getattr(System, "f", None)))

    def test_dynamic_system_has_f(self):
        plant = Integrator()
        self.assertTrue(callable(plant.f))

    def test_static_gain_has_no_f(self):
        gain = Gain(K=2.0, dim=1)
        self.assertFalse(hasattr(gain, "f"))

    def test_static_system_import_removed(self):
        import minilink.core.system as system_mod

        self.assertFalse(hasattr(system_mod, "StaticSystem"))


class TestDefaultOutput(unittest.TestCase):
    """``output_dim == n`` gives ``y = x`` by default (pyro semantics)."""

    def test_dynamic_system_default_h_is_state(self):
        sys = DynamicSystem(n=2, input_dim=1, output_dim=2)
        x = np.array([1.5, -2.0])
        np.testing.assert_array_equal(sys.h(x, np.zeros(1)), x)
        np.testing.assert_array_equal(
            sys.outputs["y"].compute(x, np.zeros(1), 0.0, None), x
        )

    def test_dynamic_system_default_h_zero_when_dims_differ(self):
        sys = DynamicSystem(n=2, input_dim=1, output_dim=1)
        np.testing.assert_array_equal(sys.h(np.ones(2), np.zeros(1)), np.zeros(1))

    def test_step_system_default_h_is_state(self):
        sys = StepSystem(n=2, input_dim=1, output_dim=2)
        x = np.array([3.0, 4.0])
        np.testing.assert_array_equal(sys.h(x, np.zeros(1)), x)

    def test_custom_plant_composes_with_closed_loop(self):
        from minilink import Gain

        class MassSpringDamper(DynamicSystem):
            def __init__(self):
                super().__init__(n=2, input_dim=1, output_dim=2)

            def f(self, x, u, t=0, params=None):
                return np.array([x[1], u[0] - 0.3 * x[1] - 4.0 * x[0]])

        plant = MassSpringDamper()
        diagram = Gain(np.array([[1.0, 0.5]])) @ plant
        x = np.array([1.0, 0.0])
        u_ctl = diagram.compute_subsys_output_port(
            x, np.zeros(diagram.m), 0.0, "controller", "y"
        )
        # r defaults to 0, so u = K (r - y) = -K x = -1.0
        self.assertAlmostEqual(float(u_ctl[0]), -1.0)


class TestForgottenSuperInit(unittest.TestCase):
    """S06: a subclass that skips super().__init__() gets a named error."""

    def test_message_names_the_class_and_the_fix(self):
        class NoSuper(DynamicSystem):
            def __init__(self):
                self.params = {}

            def f(self, x, u, t=0, params=None):
                return np.zeros(2)

        with self.assertRaises(AttributeError) as ctx:
            NoSuper().compile()
        self.assertIn(
            "NoSuper.__init__() must call super().__init__", str(ctx.exception)
        )

    def test_unknown_attribute_is_still_a_plain_attribute_error(self):
        sys = DynamicSystem(n=1)
        with self.assertRaises(AttributeError):
            sys.no_such_attribute
        self.assertFalse(hasattr(sys, "no_such_attribute"))

    def test_message_also_reaches_the_property_paths(self):
        class NoSuper(DynamicSystem):
            def __init__(self):
                pass

        for attr in ("p", "m", "n", "camera_scale"):
            with self.assertRaises(AttributeError) as ctx:
                getattr(NoSuper(), attr)
            self.assertIn("must call super().__init__", str(ctx.exception))

    def test_property_errors_name_the_real_missing_attribute(self):
        class Typo(DynamicSystem):
            def __init__(self):
                super().__init__(n=1, input_dim=1, output_dim=1)

            @property
            def gain(self):
                return self.missing_thing

        with self.assertRaises(AttributeError) as ctx:
            Typo().gain
        self.assertIn("missing_thing", str(ctx.exception))


def test_signal_box_is_derived_from_the_bounds():
    from minilink.core.sets import BoxInputSet

    signal = VectorSignal("x", dim=2, lower_bound=[-1.0, -2.0], upper_bound=[1.0, 2.0])
    np.testing.assert_allclose(signal.box.lower, [-1.0, -2.0])
    signal.upper_bound = np.array([3.0, 4.0])
    np.testing.assert_allclose(signal.box.upper, [3.0, 4.0])  # read anew each time

    sys = DynamicSystem(n=2, input_dim=1)
    sys.add_input_port("w", dim=2, lower_bound=[-0.5, -0.5], upper_bound=[0.5, 0.5])
    sys.inputs["u"].lower_bound = np.array([-5.0])
    sys.inputs["u"].upper_bound = np.array([5.0])
    U = BoxInputSet.from_system_inputs(sys)
    np.testing.assert_allclose(U.box.lower, [-5.0, -0.5, -0.5])
    np.testing.assert_allclose(U.box.upper, [5.0, 0.5, 0.5])


def schedule_cases():
    """Sampled schedules, each with the times it is read at."""
    times = (-1.0, 0.0, 0.25, 0.5, 1.0, 1.5, 2.0, 3.0, 4.0)
    return [
        ([0.0, 1.0, 3.0], [2.0, -1.0, 4.0], times),  # strictly increasing
        ([0.0, 1.0, 1.0], [0.0, 1.0, 5.0], times),  # repeated last time
        ([0.0, 0.0, 1.0], [3.0, 4.0, 5.0], times),  # repeated first time
        ([0.0, 1.0, 1.0, 2.0], [0.0, 1.0, 5.0, 6.0], times),  # repeated inner time
    ]


def test_interpolate_schedule_holds_one_sample_and_takes_the_later_at_a_repeated_time():
    from minilink.core.trajectory import interpolate_schedule

    # One sample holds at every time
    for t in (-1.0, 0.5, 2.0):
        np.testing.assert_array_equal(
            interpolate_schedule(np, [0.5], [[1.0, 2.0]], t), [1.0, 2.0]
        )

    # Linear between samples, held outside, the later sample at a repeated time:
    # what np.interp reads
    for t_samples, values, times in schedule_cases():
        for t in times:
            value = interpolate_schedule(np, t_samples, values, t)
            assert value == np.interp(t, t_samples, values)


@pytest.mark.optional
@pytest.mark.jax
def test_interpolate_schedule_holds_one_sample_and_a_repeated_time_on_jax():
    jax = pytest.importorskip("jax")
    import jax.numpy as jnp

    from minilink.core.trajectory import interpolate_schedule

    one = jax.jit(lambda t: interpolate_schedule(jnp, [0.5], [[1.0, 2.0]], t))
    np.testing.assert_array_equal(one(2.0), [1.0, 2.0])
    for t_samples, values, times in schedule_cases():
        read = jax.jit(lambda t: interpolate_schedule(jnp, t_samples, values, t))
        for t in times:
            np.testing.assert_allclose(read(t), np.interp(t, t_samples, values))


def test_se3_takes_a_vector_translation_or_the_literal_origin():
    from minilink.core.kinematics import SE3, Rz

    R = Rz(0.3)
    np.testing.assert_array_equal(SE3(R)[:3, 3], np.zeros(3))
    np.testing.assert_array_equal(SE3(R, 0.0)[:3, 3], np.zeros(3))
    np.testing.assert_array_equal(SE3(R, 0)[:3, 3], np.zeros(3))
    np.testing.assert_array_equal(SE3(R, [1.0, 2.0, 3.0])[:3, 3], [1.0, 2.0, 3.0])

    # Any other scalar is refused by its shape, never read as a translation
    for p in (1.0, 2, True, np.array(0.0), np.float64(1.0)):
        with pytest.raises(ValueError, match="3-vector"):
            SE3(R, p)


def test_signal_dim_is_a_whole_nonnegative_number():
    from minilink.core.signals import infer_signal_dim

    assert infer_signal_dim("r", dim=2) == 2
    assert infer_signal_dim("r", dim=2.0) == 2
    assert infer_signal_dim("r", dim=np.int64(3)) == 3
    assert infer_signal_dim("r", dim=0) == 0
    for dim, message in (
        (True, "whole number"),
        (np.bool_(False), "whole number"),
        (2.5, "whole number"),
        (-1, "nonnegative"),
        (-2.0, "nonnegative"),
    ):
        with pytest.raises(ValueError, match=message):
            infer_signal_dim("r", dim=dim)


class TestDefaultNames(unittest.TestCase):
    """A block's default name is its class name; a shortcut's name is the expression."""

    def test_default_name_is_the_class_name(self):
        from minilink import DynamicSystem, Pendulum, StepSystem, System

        class MyPlant(DynamicSystem):
            def __init__(self):
                super().__init__(n=1, input_dim=1, output_dim=1)

        class MyStep(StepSystem):
            def __init__(self):
                super().__init__(n=1, input_dim=1, output_dim=1)

        self.assertEqual(MyPlant().name, "MyPlant")
        self.assertEqual(MyStep().name, "MyStep")
        self.assertEqual(System().name, "System")
        self.assertEqual(Pendulum().name, "Pendulum")

        # The diagram key follows the name, so a user's class keys itself
        diagram = MyPlant() + Integrator()
        self.assertEqual(list(diagram.subsystems), ["MyPlant", "Integrator"])

    def test_shortcut_names_read_as_the_expression(self):
        from minilink import PID, Pendulum, feedback

        source = Step(final_value=[1.0], step_time=0.0)
        self.assertEqual((source >> Integrator()).name, "Step >> Integrator")
        self.assertEqual(
            (source >> Integrator() >> Integrator()).name, "Step >> Integrator"
        )  # the second >> sees a diagram on its left and carries its name
        self.assertEqual((Integrator() + Pendulum()).name, "Integrator + Pendulum")
        self.assertEqual((PID() @ Pendulum()).name, "PID @ Pendulum")
        self.assertEqual(feedback(Integrator()).name, "feedback(Integrator)")

        # A diagram operand carries its own name instead
        chain = source >> Integrator()
        chain.name = "chain"
        self.assertEqual((chain >> Integrator()).name, "chain")


# The native trajectory


def test_a_trajectory_is_a_jax_pytree_and_its_plant_run_traces():
    jax = pytest.importorskip("jax")
    from minilink.core.backends import require_jax
    from minilink.core.trajectory import Trajectory

    require_jax()
    jnp = jax.numpy

    def run(a):
        t = jnp.linspace(0.0, 1.0, 5)
        x = jnp.stack([a * t, t])
        return Trajectory(
            t=t,
            x=x,
            u=jnp.zeros((1, 5)),
            signals={"plant.u": a * x[:1]},
            roles={"plant": {"x": (0, 1), "u": "plant.u"}},
        )

    assert isinstance(jax.jit(run)(2.0), Trajectory)
    assert jax.vmap(run)(jnp.array([1.0, 2.0])).x.shape == (2, 2, 5)
    np.testing.assert_allclose(
        jax.grad(lambda a: jnp.sum(run(a).of("plant").u))(2.0), 10.0
    )


def test_cost_integrate_is_the_trapezoid_on_both_backends():
    jax = pytest.importorskip("jax")
    from minilink import Pendulum, QuadraticCost
    from minilink.core.backends import require_jax
    from minilink.core.trajectory import Trajectory

    require_jax()
    plant = Pendulum()
    cost = QuadraticCost.from_system(plant)
    traj = plant.compute_trajectory(tf=1.0, verbose=False)
    rate, running = cost.integrate(traj)
    g = np.array([cost.g(x, u, t) for x, u, t in zip(traj.x.T, traj.u.T, traj.t)])
    np.testing.assert_allclose(rate, g)
    trapezoid = np.sum(0.5 * (g[1:] + g[:-1]) * np.diff(traj.t))
    np.testing.assert_allclose(running[-1], trapezoid, rtol=1e-12)
    native = Trajectory(
        jax.numpy.asarray(traj.t), jax.numpy.asarray(traj.x), jax.numpy.asarray(traj.u)
    )
    np.testing.assert_allclose(
        np.asarray(cost.integrate(native)[1]), running, rtol=1e-12
    )


def test_a_saved_trajectory_keeps_its_roles(tmp_path):
    from minilink.core.trajectory import Trajectory

    traj = Trajectory(
        t=np.linspace(0.0, 1.0, 3),
        x=np.zeros((2, 3)),
        u=np.zeros((1, 3)),
        signals={"plant.u": np.ones((1, 3))},
        roles={"plant": {"x": (0, 1), "u": "plant.u"}},
    )
    traj.save(tmp_path / "run.npz")
    loaded = Trajectory.load(tmp_path / "run.npz")
    assert dict(loaded.roles) == {"plant": {"x": (0, 1), "u": "plant.u"}}
    np.testing.assert_array_equal(loaded.of("plant").u, np.ones((1, 3)))
