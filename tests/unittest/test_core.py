import unittest

import numpy as np

from minilink.core.system import (
    DynamicSystem,
    StaticSystem,
    System,
    VectorSignal,
)


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
        sys = StaticSystem()
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


if __name__ == "__main__":
    unittest.main()
