from minilink.dynamics.catalog.pendulum.pendulum import Pendulum

# Look-at point in world (camera matrix T[:3, 3]).
sys = Pendulum()
sys.params["m"] = 1.0
sys.params["l"] = 5.0
sys.x0[0] = 2.0

sys.compute_trajectory(tf=10.0)

sys.camera_target[:] = (2.0, 0.5, 0.0)
sys.camera_plot_axes = (0, 1)
sys.camera_scale = 8.0
sys.animate()
