"""Simulate the dynamic bicycle and animate (matplotlib + meshcat)."""

from minilink.dynamics.catalog.vehicles.dynamic_bicycle import DynamicBicycleCar3D

sys = DynamicBicycleCar3D()


sys.game(renderer="meshcat")
