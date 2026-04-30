"""Simulate the dynamic bicycle with the richer 3D car body."""

from minilink.dynamics.catalog.vehicles.dynamic_bicycle import (
    DynamicBicycleCar3DRealistic,
)

sys = DynamicBicycleCar3DRealistic()
sys.game(renderer="meshcat")
