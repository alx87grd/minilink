import unittest

import numpy as np

from minilink.blocks.basic import Integrator, PropController
from minilink.compile.api import compile_numpy, lower_diagram_to_ir
from minilink.core.diagram import DiagramSystem


class TestCompilePipelinePrototype(unittest.TestCase):
    def _build_small_closed_loop(self):
        diag = DiagramSystem()
        diag.graphe_building_verbose = False

        ctl = PropController()
        plant = Integrator()
        ctl.params["Kp"] = 2.5

        diag.add_subsystem(ctl, "ctl")
        diag.add_subsystem(plant, "plant")
        diag.add_input_port(1, "ref")

        diag.connect("input", "ref", "ctl", "ref")
        diag.connect("plant", "y", "ctl", "y")
        diag.connect("ctl", "u", "plant", "u")
        return diag

    def test_lower_ir_contains_port_order_and_plans(self):
        diag = self._build_small_closed_loop()
        ir = lower_diagram_to_ir(diag)

        self.assertGreaterEqual(len(ir.port_execution_order), 1)
        self.assertGreaterEqual(len(ir.port_plan), 1)
        self.assertGreaterEqual(len(ir.state_plan), 1)
        self.assertEqual(ir.state_dim, diag.n)

    def test_compiled_numpy_matches_f_fast(self):
        diag = self._build_small_closed_loop()
        compiled = compile_numpy(diag)

        x = np.array([0.3])
        u = np.array([1.2])  # external "ref"
        t = 0.1

        dx_fast = diag.f_fast(x, u, t)
        dx_compiled = compiled.eval_dx(x, u, t)

        np.testing.assert_allclose(dx_compiled, dx_fast, atol=1e-10)


if __name__ == "__main__":
    unittest.main()

