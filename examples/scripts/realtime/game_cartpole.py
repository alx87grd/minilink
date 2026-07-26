"""Drive a cart-pole with the arrow keys — real-time game mode.

Run from repo root::

    python examples/scripts/realtime/game_cartpole.py

RIGHT/LEFT push the cart; the force saturates at the ``u`` input-port bounds.
Try to keep the pole balanced. ESC or closing the window stops the session,
which returns the recorded ``Trajectory`` — the same artifact as an offline
simulation — ready for plotting or replay.
"""

import numpy as np

from minilink.dynamics.catalog.pendulum.cartpole import CartPole
from minilink.simulation.realtime import PygameInput, RealtimeSimulator

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
