"""Teaching public-import facades (DESIGN §2)."""

import unittest

from minilink.dynamics.catalog.pendulum.pendulum import Pendulum as PendulumDef


class TestPublicImports(unittest.TestCase):
    def test_root_prelude_pendulum_matches_defining_module(self):
        from minilink import Pendulum

        self.assertIs(Pendulum, PendulumDef)

    def test_catalog_band_matches_defining_module(self):
        from minilink.catalog import Pendulum

        self.assertIs(Pendulum, PendulumDef)

    def test_domain_package_matches_defining_module(self):
        from minilink.dynamics.catalog.pendulum import Pendulum

        self.assertIs(Pendulum, PendulumDef)

    def test_control_and_analysis_band_exports(self):
        from minilink.analysis import bode, modal_analysis
        from minilink.analysis.linearize import linearize
        from minilink.control import ImpedanceController
        from minilink.control.lqr import lqr

        self.assertTrue(callable(lqr))
        self.assertTrue(callable(linearize))
        self.assertTrue(callable(bode))
        self.assertTrue(callable(modal_analysis))
        self.assertTrue(callable(ImpedanceController))

    def test_root_all_is_selective(self):
        import minilink

        self.assertIn("Pendulum", minilink.__all__)
        self.assertNotIn("Boat2D", minilink.__all__)
        self.assertNotIn("ModelPredictiveController", minilink.__all__)


if __name__ == "__main__":
    unittest.main()
