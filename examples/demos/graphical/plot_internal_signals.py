"""Choose which internal signals a diagram plot shows."""

from minilink import ImpedanceController, Pendulum, Step

plant = Pendulum()
plant.params["l"] = 5.0
plant.x0[0] = 2.0

ref = Step(final_value=1.0, step_time=10.0)
ctl = ImpedanceController(Kp=1000.0, Kd=100.0)

diagram = ref >> ctl @ plant
diagram.plot_diagram()
diagram.compute_trajectory(tf=20)

diagram.plot_trajectory(signals=("ref:y", "x", "sys:y", "ctl:u"))  # chosen
diagram.plot_trajectory()  # automatic selection
