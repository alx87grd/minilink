import numpy as np

from minilink.blocks.examples import Pendulum

# Interactive loop: today pygame keys + Euler in Animator; see ROADMAP.md §7 for planned
# integrator backends and live I/O (e.g. TCP cosimulation).

# Plant system
sys = Pendulum()
sys.params["m"] = 1.0
sys.params["l"] = 5.0
sys.x0[0] = 0.0


# sys.game()
# sys.game(renderer="pygame")
sys.game(renderer="meshcat")
# sys.game(renderer="matplotlib")
