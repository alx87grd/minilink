"""PID step tracking with and without anti-windup, the classical loop closed with @."""

from minilink import PID, DoubleIntegrator, Saturation, Step

step = Step(final_value=4.0, step_time=3.0)
sys = DoubleIntegrator()

# With anti-windup: the PID clips its own command at +-5 and stops integrating.
pid = PID(Kp=5.0, Ki=1.0, Kd=3.0, tau=0.1, u_min=-5.0, u_max=5.0)

loop = step >> pid @ sys  # r - y -> PID -> plant, the junction is inserted by @
loop.plot_diagram()
loop.compute_trajectory(tf=20)
loop.plot_trajectory()

# Without anti-windup: an external saturation clips u; the integrator keeps growing.
pid2 = PID(Kp=5.0, Ki=1.0, Kd=3.0, tau=0.1)
sat = Saturation(-0.5, 0.5)

loop2 = step >> (pid2 >> sat >> sys) @ 1  # the same loop closed on the series diagram
loop2.plot_diagram()
loop2.compute_trajectory(tf=20)
loop2.plot_trajectory()
