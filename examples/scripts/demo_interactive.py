import numpy as np

from minilink.blocks.basic import Pendulum

# Plant system
sys = Pendulum()
sys.params["m"] = 1.0
sys.params["l"] = 5.0
sys.x0[0] = 0.0


# sys.game()
# sys.game(renderer="pygame")
# sys.game(renderer="meshcat")
sys.game(renderer="matplotlib")

# sys.animate()
# sys.animate(renderer="meshcat")
# sys.animate(renderer="pygame")
