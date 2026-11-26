"""Drive a cart-pole with the arrow keys — real-time game mode."""

import numpy as np

from minilink import CartPole
from minilink.simulation import RealtimeSimulator
from minilink.simulation.realtime import PygameInput

########################################################
### Tunables
########################################################

FORCE = 10.0  # [N]
M1 = 2.0
M2 = 0.2

########################################################
### The plant
########################################################

sys = CartPole()
sys.x0 = np.array([0.0, 0.2, 0.0, 0.0])

sys.params["m1"] = M1
sys.params["m2"] = M2

sys.inputs["u"].lower[0] = -FORCE
sys.inputs["u"].upper[0] = +FORCE

########################################################
### Real-time game
########################################################

rt_sim = RealtimeSimulator(
    sys,
    frame_dt=1 / 30,
    backend="jax",
    renderer="pygame",
    # renderer="meshcat",
    # renderer="matplotlib",
    input=PygameInput(key_axes=[("right", "left")]),
)
traj = rt_sim.run()
sys.plot_trajectory(traj)
