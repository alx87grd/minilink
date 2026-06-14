import numpy as np

from minilink.dynamics.catalog.pendulum.pendulum import Pendulum

# Plant system
sys = Pendulum()
sys.params["m"] = 1.0
sys.params["l"] = 1.0
sys.x0[0] = 2.0


# sys.compute_trajectory(tf=10, show=False)

t2u = lambda t: 3.0 * np.sin(10.0 * t)
traj = sys.compute_forced(u=t2u, tf=5.0, show=False, input_port_id="u")


# sys.plot_trajectory()
sys.plot_trajectory(backend="plotly")


# sys.animate()
# sys.animate(renderer="matplotlib")
# sys.animate(renderer="matplotlib", native=False)
# sys.animate(renderer="matplotlib", is_3d=True)
# sys.animate(renderer="pygame")
# sys.animate(renderer="meshcat")
# sys.animate(renderer="meshcat", native=False)
sys.animate(renderer="plotly")
# sys.animate(renderer="plotly", native=False)
# sys.animate(renderer="plotly", is_3d=True)
