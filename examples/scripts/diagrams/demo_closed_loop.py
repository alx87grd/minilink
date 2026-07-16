import numpy as np

from minilink.blocks.sources import Step
from minilink.control.impedance import ImpedanceController
from minilink.core.diagram import DiagramSystem
from minilink.dynamics.catalog.pendulum.pendulum import Pendulum


def run_smoke(*, tf: float = 1.0, show: bool = False) -> None:
    """Headless L6 smoke: one closed-loop pendulum trajectory."""
    sys = Pendulum()
    sys.params["m"] = 1.0
    sys.params["l"] = 5.0
    sys.x0[0] = -2.0

    step = Step()
    step.params["initial_value"] = np.array([0.0])
    step.params["final_value"] = np.array([1.0])
    step.params["step_time"] = 10.0

    ctl = ImpedanceController()
    ctl.params["Kp"] = 1000.0
    ctl.params["Kd"] = 100.0

    diagram = DiagramSystem()
    diagram.add_subsystem(step, "step")
    diagram.add_subsystem(ctl, "controller")
    diagram.add_subsystem(sys, "plant")
    diagram.connect("step", "y", "controller", "r")
    diagram.connect("controller", "u", "plant", "u")
    diagram.connect("plant", "y", "controller", "y")
    diagram.name = "Closed Loop Pendulum"
    diagram.compute_trajectory(tf=tf, show=show)
    diagram.plot_trajectory(show=show)


def main() -> None:
    sys = Pendulum()
    sys.params["m"] = 1.0
    sys.params["l"] = 5.0
    sys.x0[0] = -2.0

    step = Step()
    step.params["initial_value"] = np.array([0.0])
    step.params["final_value"] = np.array([1.0])
    step.params["step_time"] = 10.0

    ctl = ImpedanceController()
    ctl.params["Kp"] = 1000.0
    ctl.params["Kd"] = 100.0

    diagram = DiagramSystem()

    diagram.add_subsystem(step, "step")
    diagram.add_subsystem(ctl, "controller")
    diagram.add_subsystem(sys, "plant")

    diagram.connect("step", "y", "controller", "r")
    diagram.name = "Pendulum alone"
    diagram.plot_diagram()
    diagram.compute_trajectory(tf=20)
    diagram.plot_trajectory()

    diagram.connect("controller", "u", "plant", "u")
    diagram.name = "Pendulum with Open Loop Controller"
    diagram.plot_diagram()
    diagram.compute_trajectory(tf=20)
    diagram.plot_trajectory()

    diagram.connect("plant", "y", "controller", "y")
    diagram.name = "Closed Loop Pendulum "
    diagram.plot_diagram()
    diagram.compute_trajectory(tf=20)
    diagram.plot_trajectory()


if __name__ == "__main__":
    main()
