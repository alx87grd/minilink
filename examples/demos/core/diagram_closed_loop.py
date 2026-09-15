"""Wire a pendulum loop by hand: unconnected, open loop, closed loop."""

from minilink import DiagramSystem, ImpedanceController, Pendulum, Step

# Plant system
sys = Pendulum()
sys.params["m"] = 1.0
sys.params["l"] = 5.0
sys.x0[0] = -2.0

# Source input
step = Step(final_value=1.0, step_time=10.0)

# Closed loop system
ctl = ImpedanceController(Kp=1000.0, Kd=100.0)

# Diagram
diagram = DiagramSystem()

diagram.add_subsystem(step, "step")
diagram.add_subsystem(ctl, "controller")
diagram.add_subsystem(sys, "plant")
diagram.plot_diagram()


# Unconnected controller -> plant
diagram.connect("step", "y", "controller", "r")
diagram.name = "Pendulum alone"
diagram.plot_diagram()
diagram.compute_trajectory(tf=20)
diagram.plot_trajectory()

# Open loop controller -> plant
diagram.connect("controller", "u", "plant", "u")
diagram.name = "Pendulum with Open Loop Controller"
diagram.plot_diagram()
diagram.compute_trajectory(tf=20)
diagram.plot_trajectory()

# Closed loop controller -> plant
diagram.connect("plant", "y", "controller", "y")
diagram.name = "Closed Loop Pendulum "
diagram.plot_diagram()
diagram.compute_trajectory(tf=20)
diagram.plot_trajectory()
