"""Drive a cart-pole with the arrow keys — real-time game mode."""

# RIGHT/LEFT push the cart; the force saturates at the ``u`` input-port bounds.

import numpy as np

from minilink import CartPole
from minilink.simulation import RealtimeSimulator
from minilink.simulation.realtime import PygameInput

sys = CartPole()
sys.x0 = np.array([0.0, 0.2, 0.0, 0.0])  # small pole tilt to balance away

# The playable force range is the input-port bounds used by PygameInput.
sys.inputs["u"].lower_bound[0] = -20.0
sys.inputs["u"].upper_bound[0] = +20.0

rt_sim = RealtimeSimulator(
    sys,
    frame_dt=1 / 30,
    # compile_backend="jax",
    # renderer="pygame",
    # renderer="meshcat",
    renderer="matplotlib",
    input=PygameInput(key_axes=[("right", "left")]),  # RIGHT/LEFT -> cart force
)
traj = rt_sim.run()

sys.traj = traj
sys.plot_trajectory()
