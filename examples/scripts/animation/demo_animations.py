from minilink.dynamics.catalog.pendulum.pendulum import Pendulum

# Plant system
sys = Pendulum()
sys.params["m"] = 1.0
sys.params["l"] = 5.0
sys.x0[0] = 2.0

sys.compute_trajectory(tf=10, show=False)

# sys.animate()
# sys.animate(renderer="matplotlib")
# sys.animate(renderer="matplotlib", is_3d=True)
# sys.animate(renderer="pygame")
# sys.animate(renderer="meshcat")
sys.animate(renderer="plotly")
# sys.animate(renderer="plotly", is_3d=True)
