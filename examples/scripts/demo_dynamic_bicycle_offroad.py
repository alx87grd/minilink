"""Off-road UTV-style dynamic bicycle demo inspired by BRP visuals."""

from minilink.blocks.dynamic_bicycle import DynamicBicycleOffRoad3D

sys = DynamicBicycleOffRoad3D()
sys.game(renderer="meshcat")
