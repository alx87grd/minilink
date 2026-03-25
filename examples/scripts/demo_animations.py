import numpy as np

from minilink.blocks.basic import Pendulum

# Plant system
sys = Pendulum()
sys.params["m"] = 1.0
sys.params["l"] = 5.0
sys.x0[0] = 2.0


sys.compute_trajectory(tf=10)


sys.animate()
sys.animate(renderer="meshcat")
sys.animate(renderer="pygame")
