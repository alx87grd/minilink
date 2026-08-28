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

    def test_simulation_band_exports_simulators(self):
        from minilink.simulation import Simulator, StaticSimulator
        from minilink.simulation.simulator import Simulator as SimulatorDef
        from minilink.simulation.static_simulator import (
            StaticSimulator as StaticSimulatorDef,
        )

        self.assertIs(Simulator, SimulatorDef)
        self.assertIs(StaticSimulator, StaticSimulatorDef)

    def test_blocks_band_exports(self):
        from minilink.blocks import Integrator, Step, Sum
        from minilink.blocks.basic import Integrator as IntegratorDef

        self.assertIs(Integrator, IntegratorDef)
        self.assertTrue(callable(Step))
        self.assertTrue(callable(Sum))

    def test_stable_band_exports_all_resolve(self):
        """Every `_EXPORTS` name on a stable band facade resolves lazily."""
        import importlib

        for band in ("minilink", "minilink.blocks", "minilink.simulation"):
            module = importlib.import_module(band)
            for name in module.__all__:
                self.assertIsNotNone(getattr(module, name), f"{band}.{name}")

    def test_catalog_exports_match_domain_all(self):
        """Catalog alias stays in full sync with the domain `__all__` lists."""
        import importlib

        import minilink.catalog as catalog

        domain_names: set[str] = set()
        for module_path in sorted(
            {module_path for module_path, _attr in catalog._EXPORTS.values()}
        ):
            domain = importlib.import_module(module_path)
            domain_names.update(domain.__all__)

        catalog_names = set(catalog._EXPORTS)
        self.assertEqual(
            catalog_names,
            domain_names,
            "minilink.catalog._EXPORTS and domain __all__ lists have drifted",
        )
        for name in sorted(catalog_names):
            self.assertIsNotNone(getattr(catalog, name), f"catalog.{name}")

    def test_catalog_exports_are_systems(self):
        """The teaching catalog exposes System classes only (plants)."""
        import minilink.catalog as catalog
        from minilink.core.system import System

        for name in sorted(catalog._EXPORTS):
            value = getattr(catalog, name)
            self.assertTrue(
                isinstance(value, type) and issubclass(value, System),
                f"catalog.{name} is not a System subclass",
            )


if __name__ == "__main__":
    unittest.main()
