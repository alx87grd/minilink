from minilink.dynamics.catalog.pendulum.pendulum import Pendulum

# Single frame: render() reads the same camera_* attributes as animate().
sys = Pendulum()
sys.params["m"] = 1.0
sys.params["l"] = 5.0
sys.x0[0] = 2.0

sys.camera_plot_axes = (0, 2)
sys.camera_scale = 8.0
u0 = sys.get_u_from_input_ports()
sys.render(sys.x0, u0, 0.0)
