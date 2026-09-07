"""Filtered PID step tracking with and without anti-windup."""

from minilink import DoubleIntegrator, FilteredController, Saturation, Step

step = Step(final_value=4.0, step_time=3.0)
sys = DoubleIntegrator()

# With anti-windup: the controller clips its own command at +-5 and stops integrating.
pid = FilteredController(Kp=5.0, Ki=1.0, Kd=3.0, tau=0.1, u_min=-5.0, u_max=5.0)

diagram = step >> pid @ sys
diagram.plot_diagram()
diagram.compute_trajectory(tf=20)
diagram.plot_trajectory()

# Without anti-windup: an external saturation clips u; the integrator keeps growing.
pid2 = FilteredController(Kp=5.0, Ki=1.0, Kd=3.0, tau=0.1)
sat = Saturation(-0.5, 0.5)

diagram2 = step >> pid2 >> sat >> sys
diagram2.connect("sys", "y", "ctl", "y")
diagram2.plot_diagram()
diagram2.compute_trajectory(tf=20)
diagram2.plot_trajectory()
