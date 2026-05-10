from minilink.dynamics.catalog.pendulum.pendulum import Pendulum

# Which world axes appear as plot horizontal / vertical (when camera_R is None).
sys = Pendulum()
sys.params["m"] = 1.0
sys.params["l"] = 5.0
sys.x0[0] = 2.0

sys.compute_trajectory(tf=10.0)

sys.camera_plot_axes = (0, 2)
sys.camera_scale = 10.0
sys.animate()
