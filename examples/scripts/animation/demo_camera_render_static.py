from minilink.dynamics.catalog.pendulum.pendulum import Pendulum
from minilink.graphical.primitives import attach_standard_camera

# Single frame: render() uses the same get_camera_transform / camera_* fields
sys = Pendulum()
sys.params["m"] = 1.0
sys.params["l"] = 5.0
sys.x0[0] = 2.0

attach_standard_camera(sys, plot_axes=(0, 2), scale=8.0)
u0 = sys.get_u_from_input_ports()
sys.render(sys.x0, u0, 0.0)
