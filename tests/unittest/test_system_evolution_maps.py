"""Evolution-map typing on the System hierarchy."""

import unittest

from minilink.blocks.basic import Integrator
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
