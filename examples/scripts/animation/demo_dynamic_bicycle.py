"""Simulate the dynamic bicycle and animate (matplotlib + meshcat)."""

from minilink.dynamics.catalog.vehicles.dynamic_bicycle import DynamicBicycleCar3D
import numpy as np

sys = DynamicBicycleCar3D()

sys.inputs["w_rear"].nominal_value = np.array([5.0])
sys.x0 = np.array([0.0, 2.35, 0.6, 5.0, 0.0, 0.0])

sys.compute_trajectory(tf=10)
sys.animate(renderer="meshcat")
# sys.animate(renderer="meshcat", is_3d=True)

# sys.game(renderer="meshcat")
