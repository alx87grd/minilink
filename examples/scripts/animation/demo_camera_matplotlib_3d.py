from minilink.dynamics.catalog.pendulum.pendulum import Pendulum

# Initial matplotlib 3D view uses these fields once at open; then orbit with the mouse.
sys = Pendulum()
sys.params["m"] = 1.0
sys.params["l"] = 5.0
sys.x0[0] = 2.0

sys.compute_trajectory(tf=10.0)

sys.camera_plot_axes = (0, 2)
sys.camera_scale = 12.0
sys.animate(is_3d=True)
