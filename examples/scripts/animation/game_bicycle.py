"""Drive the dynamic bicycle with the keyboard — ``sys.game()`` facade.

Run from repo root::

    python examples/scripts/animation/game_bicycle.py

UP/DOWN  -> rear wheel speed ``w_rear`` (port bounds)
LEFT/RIGHT -> steer ``delta``
"""

from minilink.dynamics.catalog.vehicles.dynamic_bicycle import DynamicBicycleCar3D

sys = DynamicBicycleCar3D()

# Finite input-port bounds are the playable keyboard range for PygameInput.
sys.inputs["w_rear"].upper_bound[0] = 200.0
sys.inputs["w_rear"].lower_bound[0] = -200.0
sys.inputs["delta"].upper_bound[0] = 0.4
sys.inputs["delta"].lower_bound[0] = -0.4

sys.game(renderer="meshcat")
# sys.game(renderer="pygame")
# sys.game(renderer="matplotlib")
