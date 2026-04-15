"""Simulate the dynamic bicycle and animate (matplotlib + meshcat)."""

import types

import numpy as np

from minilink.blocks.dynamic_bicycle import DynamicBicycle, DynamicBicycleCar3D
from minilink.graphical.animation import Animator

sys = DynamicBicycleCar3D()


sys.game(renderer="meshcat")
