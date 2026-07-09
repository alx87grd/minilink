"""Facade routing between Simulator and StaticSimulator."""

import unittest

from minilink.blocks.basic import Integrator
from minilink.blocks.routing import Gain
from minilink.simulation.simulator import Simulator
from minilink.simulation.static_simulator import StaticSimulator


class TestFacadeRouting(unittest.TestCase):
    def test_gain_routes_to_static_simulator(self):
        gain = Gain(K=1.0, dim=1)
        sim = gain._simulate(t0=0, tf=1, n_steps=3)
        self.assertIsInstance(sim, StaticSimulator)

    def test_integrator_routes_to_simulator(self):
        plant = Integrator()
        sim = plant._simulate(t0=0, tf=1, n_steps=3)
        self.assertIsInstance(sim, Simulator)

    def test_static_leaf_rejects_simulator(self):
        gain = Gain(K=1.0, dim=1)
        with self.assertRaises(TypeError):
            Simulator(gain, t0=0, tf=1, n_steps=3)
