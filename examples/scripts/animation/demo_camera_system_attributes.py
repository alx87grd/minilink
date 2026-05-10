from minilink.dynamics.catalog.pendulum.pendulum import Pendulum

# Zoom via sys.camera_scale (default get_camera_transform reads camera_* fields)
sys = Pendulum()
sys.params["m"] = 1.0
sys.params["l"] = 5.0
sys.x0[0] = 2.0

sys.compute_trajectory(tf=10.0)

sys.camera_scale = 3.0
sys.animate()
