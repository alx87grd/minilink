"""
Demo: JAX physics world block (spheres + plane contact) inside DiagramSystem.

The physics world is wrapped in a DynamicSystem so it can be simulated,
compiled, and animated in the standard minilink ecosystem.
"""

import numpy as np

from minilink.blocks.sources import Step
from minilink.core.diagram import DiagramSystem
from minilink.dynamics.engines.contact_jax import (
    PlaneModel,
    SphereModel,
    make_world_model,
)
from minilink.dynamics.engines.world import PhysicsWorldSystem

world = make_world_model(
    [
        SphereModel(mass=3.0, radius=0.5),
        SphereModel(mass=3.0, radius=0.5),
    ],
    PlaneModel(normal=(0.0, 0.0, 1.0), offset=0.0),
    gravity=(0.0, 0.0, -1.0),
    k_contact=500.0,
    c_contact=5.0,
)


sys = PhysicsWorldSystem(world, name="PhysicsSpherePlaneMVP")

# [p(3), q(4), v(3), w(3)] per body (2 bodies -> 26 states)
x0 = np.zeros(sys.n)
# body 0: high drop
x0[0:3] = [0.0, 0.0, 2.0]
x0[3:7] = [1.0, 0.0, 0.0, 0.0]
# body 1: lower drop
x0[13:16] = [1.0, 3.0, 1.5]
x0[16:20] = [1.0, 0.0, 0.0, 0.0]
sys.x0 = x0


# Input is [Fx,Fy,Fz,Tx,Ty,Tz] per body (2 bodies => 12 dims).
step = Step(
    initial_value=np.zeros(sys.m),
    final_value=np.array([0.0, 0.0, 5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
    step_time=5.0,
)

diagram = DiagramSystem()
diagram.add_subsystem(step, "force_cmd")
diagram.add_subsystem(sys, "world")
diagram.connect("force_cmd", "y", "world", "u")
diagram.plot_diagram()

traj = diagram.compute_trajectory(tf=10.0, compile_backend="auto")

diagram.animate(renderer="meshcat")

# sys.game(renderer="meshcat")
