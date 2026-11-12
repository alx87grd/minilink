"""Balance a cart-pole by hand: the arrow keys push the cart, in real time."""

import numpy as np

from minilink import CartPole
from minilink.simulation import RealtimeSimulator
from minilink.simulation.realtime import PygameInput

# The plant, its pole 0.2 rad off the hanging position; the keys command a force
# within the input port's bounds, here 20 N either way

plant = CartPole()
plant.x0 = np.array([0.0, 0.2, 0.0, 0.0])
plant.inputs["u"].lower[0] = -20.0
plant.inputs["u"].upper[0] = 20.0

# The game: RIGHT and LEFT push the cart; ESC ends the run, which is then plotted

game = RealtimeSimulator(
    plant,
    frame_dt=1 / 30,
    renderer="matplotlib",
    input=PygameInput(key_axes=[("right", "left")]),
)
traj = game.run()
plant.plot_trajectory(traj)
