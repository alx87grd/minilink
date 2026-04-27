import numpy as np

from minilink.dynamics.catalog.pendulum.pendulum import Pendulum

# Plant system
sys = Pendulum()
sys.params["m"] = 1.0
sys.params["l"] = 5.0
sys.x0[0] = 2.0


sys.compute_trajectory(tf=10)
# sys.compute_trajectory(tf=10, plot="xu")


# sys.animate()
# sys.animate(renderer="matplotlib")
# sys.animate(renderer="matplotlib", native=True)
# sys.animate(renderer="matplotlib", native=True, html=True)

sys.animate(renderer="meshcat")
# sys.animate(renderer="meshcat", native=True)
# sys.animate(renderer="meshcat", native=True, html=True)


# sys.animate(renderer="pygame")

# sys.x0[0] = -1.0
# sys.compute_trajectory(tf=10)
# sys.animate(renderer="meshcat")
