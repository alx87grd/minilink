from minilink.dynamics.catalog.pendulum.pendulum import Pendulum

# Zoom: camera_scale is orthographic half-extent (and meshcat eye distance).
sys = Pendulum()
sys.params["m"] = 1.0
sys.params["l"] = 5.0
sys.x0[0] = 2.0

sys.compute_trajectory(tf=10.0)

sys.camera_scale = 3.0
sys.animate()
