"""Tests for :func:`~minilink.core.composition.resolve_standard_feedback`."""

import unittest

from minilink.blocks.basic import Integrator
from minilink.control.output import ProportionalController
from minilink.core.composition import closed_loop, resolve_standard_feedback


class TestStandardFeedback(unittest.TestCase):
    def test_y_loop_ports(self):
        ctl = ProportionalController(0.5)
        plant = Integrator()
        wiring = resolve_standard_feedback(ctl, plant)
        self.assertEqual(wiring.control_out, "u")
        self.assertEqual(wiring.measurement_in, "y")
        self.assertEqual(wiring.plant_in, "u")
        self.assertEqual(wiring.plant_out, "y")

    def test_closed_loop_uses_shared_resolver(self):
        diagram = closed_loop(ProportionalController(0.4), Integrator())
        self.assertIn("ctl", diagram.subsystems)
        self.assertIn("sys", diagram.subsystems)

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


if __name__ == "__main__":
    unittest.main()
