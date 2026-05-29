import numpy as np

from minilink.core.blocks.sources import Step, WhiteNoise
from minilink.core.diagram import DiagramSystem
from minilink.core.system import DynamicSystem, StaticSystem
from minilink.simulation.simulator import Simulator


class Pendulum(DynamicSystem):
    def __init__(self):
        super().__init__(n=2)

        self.params = {"g": 9.81, "m": 1.0, "l": 1.0}

        self.name = "Pendulum"

        self.state.labels = ["theta", "theta_dot"]
        self.state.units = ["rad", "rad/s"]
        self.add_input_port("u", dim=1, nominal_value=np.array([0.0]))
        self.add_input_port("w", dim=1, nominal_value=np.array([0.0]))
        self.add_input_port("v", dim=1, nominal_value=np.array([0.0]))
        self.inputs["u"].labels = ["torque"]
        self.inputs["u"].units = ["Nm"]
        self.add_output_port("y", dim=2, function=self.h, dependencies=("v",))

    def f(self, x, u, t=0, params=None):

        if params is None:
            params = self.params

        g = params["g"]
        m = params["m"]
        length = params["l"]

        theta = x[0]

        signals = self.get_port_values_from_u(u)
        u = signals["u"][0]
        w = signals["w"][0]

        dx = np.zeros(2)
        dx[0] = x[1]
        dx[1] = -g / length * np.sin(theta) + 1 / (m * length**2) * (u + w)

        return dx

    def h(self, x, u, t=0, params=None):

        signals = self.get_port_values_from_u(u)
        v = signals["v"]

        y = np.zeros(self.p)

        y[0] = x[0] + v[0]
        y[1] = x[1] + v[0]

        return y


class PDController(StaticSystem):
    def __init__(self):
        super().__init__()

        self.params = {
            "Kp": 10.0,
            "Kd": 1.0,
        }

        self.name = "Controller"
        self.add_input_port("r", dim=1, nominal_value=np.array([0.0]))
        self.add_input_port("y", dim=2, nominal_value=np.array([0.0, 0.0]))
        self.inputs["y"].labels = ["theta", "theta_dot"]
        self.inputs["y"].units = ["rad", "rad/s"]
        self.add_output_port("u", dim=1, function=self.ctl, dependencies=("r", "y"))
        self.outputs["u"].labels = ["torque"]
        self.outputs["u"].units = ["Nm"]

    def ctl(self, x, u, t=0, params=None):

        if params is None:
            params = self.params

        Kp = params["Kp"]
        Kd = params["Kd"]

        ref = u[0]
        theta = u[1]
        theta_dot = u[2]

        torque = Kp * (ref - theta) - Kd * theta_dot

        u = np.array([torque])

        return u


class Integrator(DynamicSystem):
    def __init__(self):
        super().__init__(n=1, input_dim=1, output_dim=1, y_dependencies=())

        self.params = {"k": 1.0}

        self.name = "Integrator"
        self.add_output_port("y", dim=1, function=self.h, dependencies=())

    def f(self, x, u, t=0, params=None):

        if params is None:
            params = self.params
        k = params["k"]

        dx = np.zeros(self.n)
        dx[0] = k * u[0]

        return dx

    def h(self, x, u, t=0, params=None):

        y = np.zeros(self.p)
        y[0] = x[0]

        return y


class PropController(StaticSystem):
    def __init__(self):
        super().__init__()

        self.params = {
            "Kp": 10.0,
        }

        self.name = "Controller"
        self.add_input_port("r", dim=1, nominal_value=np.array([0.0]))
        self.add_input_port("y", dim=1, nominal_value=np.array([0.0]))
        self.add_output_port("u", dim=1, function=self.ctl, dependencies=("r", "y"))

    def ctl(self, x, u, t=0, params=None):

        if params is None:
            params = self.params

        Kp = params["Kp"]

        r = u[0]
        y = u[1]

        u = Kp * (r - y)

        u = np.array([u])

        return u


# Tests functions


def simulator_test():

    # Defining a test system
    sys1 = Pendulum()

    sys1.x0[0] = 2.0

    # Running the simulation

    sim = Simulator(sys1, t0=0, tf=25)

    traj = sim.solve()
    sys1.plot_trajectory(traj, signals=("x", "u"), backend="matplotlib")

    # sys1.plot_trajectory(traj, signals=("x", "u"), backend="matplotlib")

    np.set_printoptions(precision=2, suppress=True)
    print(f"Time vector:\n {traj.t}")
    print(f"Input trajectory:\n {traj.u}")
    print(f"State trajectory:\n {traj.x}")

    return sim


def system_test():

    sys1 = DynamicSystem(2, input_dim=1, output_dim=1, expose_state=True)

    sys1.add_input_port("w", dim=2, nominal_value=np.array([7.7, 2.2]))
    sys1.add_input_port("v", dim=1, nominal_value=np.array([1.1]))

    print("Sys1 u dim:", sys1.m)
    print("Sys1 x dim:", sys1.n)
    print("Sys1 y dim:", sys1.p)

    u = sys1.get_u_from_input_ports()
    input_signals = sys1.get_port_values_from_u(u)
    u2 = np.array([])
    for key, value in input_signals.items():
        u2 = np.concatenate([u2, value])
    assert np.allclose(u, u2)

    print("Default u:", u)
    print("Default u:", u2)
    print("Default input signals:", input_signals)

    sys1.print_html()

    sys1.plot_graphe()

    return sys1


def diagram_test():

    sys1 = DynamicSystem(2, input_dim=1, output_dim=1, expose_state=True)
    sys1.add_input_port("w", dim=2, nominal_value=np.array([7.7, 2.2]))
    sys1.add_input_port("v", dim=1, nominal_value=np.array([1.1]))
    sys2 = DynamicSystem(2, input_dim=1, output_dim=1, expose_state=True)
    sys3 = StaticSystem()
    sys4 = DynamicSystem(2, input_dim=1, output_dim=1, expose_state=True)
    step = Step(np.array([0.0]), np.array([1.0]), 1.0)

    gsys = DiagramSystem()

    gsys.add_subsystem(sys1, "sys1")
    gsys.add_subsystem(sys2, "sys2")
    gsys.add_subsystem(sys3, "sys3")
    gsys.add_subsystem(sys4, "sys4")
    gsys.add_subsystem(step, "step")

    print("List of subsystems:\n")
    print(gsys.subsystems)

    print("List of connections :\n")
    print(gsys.connections)

    gsys.connect("sys1", "y", "sys2", "u")
    gsys.connect("sys2", "y", "sys3", "u")
    gsys.connect("sys2", "y", "sys4", "u")
    gsys.connect("sys4", "y", "sys1", "u")
    gsys.connect("step", "y", "sys1", "v")
    gsys.connect("sys4", "y", "sys1", "w")
    # gsys.connect("sys4", "y", "sys1", "u")
    # gsys.connect("sys3", "y", "sys1", "w")

    print("List of connections :\n")
    print(gsys.connections)

    _ = gsys.plot_graphe()

    print("sys.n = ", gsys.n)
    print("sys.m = ", gsys.m)
    print("sys.p = ", gsys.p)
    print("sys.state_labels = ", gsys.state.labels)

    return gsys


def pendulum_test():

    # Plant system
    sys = Pendulum()
    sys.params["m"] = 1.0
    sys.params["l"] = 1.0
    sys.x0[0] = 2.0

    # Source input
    step = Step()
    step.params["initial_value"] = np.array([0.0])
    step.params["final_value"] = np.array([0.2])
    step.params["step_time"] = 15.0

    # Noisy input
    noise = WhiteNoise(1)
    noise.params["var"] = 10.0
    noise.params["mean"] = 0.0
    noise.params["seed"] = 1

    # Noisy measurement
    noise2 = WhiteNoise(1)
    noise2.params["var"] = 10.0
    noise2.params["mean"] = 0.0
    noise2.params["seed"] = 2

    # # Diagram
    diagram = DiagramSystem()
    diagram.add_subsystem(sys, "plant")
    diagram.add_subsystem(sys, "plant2")
    diagram.add_subsystem(step, "step")
    diagram.add_subsystem(noise, "dist")
    diagram.add_subsystem(noise2, "noise")
    diagram.connect("step", "y", "plant", "u")
    diagram.connect("step", "y", "plant2", "u")
    diagram.connect("dist", "y", "plant", "w")
    diagram.connect("noise", "y", "plant", "v")
    diagram.plot_graphe()

    sim = Simulator(diagram, t0=0, tf=20, dt=0.01, solver="euler")
    diagram.plot_trajectory(sim.solve(), signals=("x", "u"), backend="matplotlib")

    return sim


def algebraic_loop():

    # Plant system
    sys = Integrator()
    sys.x0[0] = 20.0

    # Controllers
    ctl1 = PropController()
    ctl1.params["Kp"] = 1.0
    ctl2 = PropController()
    ctl2.params["Kp"] = 1.0

    # Source input
    step = Step()
    step.params["initial_value"] = np.array([0.0])
    step.params["final_value"] = np.array([1.0])
    step.params["step_time"] = 10.0

    # # Diagram
    diagram = DiagramSystem()

    diagram.add_subsystem(step, "step")
    diagram.add_subsystem(ctl1, "controller1")
    diagram.add_subsystem(ctl2, "controller2")
    diagram.add_subsystem(sys, "integrator1")
    diagram.add_subsystem(sys, "integrator2")

    diagram.connect("controller2", "u", "integrator2", "u")
    diagram.connect("controller2", "u", "controller1", "y")
    diagram.connect("controller1", "u", "controller2", "y")
    diagram.connect("step", "y", "controller1", "r")

    diagram.plot_graphe()

    # diagram.check_algebraic_loops()

    sim = Simulator(diagram, t0=0, tf=20, n_steps=10000)
    diagram.plot_trajectory(sim.solve(), signals=("x", "u"), backend="matplotlib")

    return diagram


def solver_doing_weird_at_discontinuities():

    # Plant system
    sys1 = Integrator()
    sys1.x0[0] = 20.0

    sys2 = Integrator()
    sys2.x0[0] = 20.0

    # # Controllers
    # ctl1 = PropController()
    # ctl1.params["Kp"] = 1.0
    # ctl2 = PropController()
    # ctl2.params["Kp"] = 1.0

    # Source input
    step = Step()
    step.params["initial_value"] = np.array([0.0])
    step.params["final_value"] = np.array([1.0])
    step.params["step_time"] = 10.0

    # Baseline validation
    test = DiagramSystem()
    test.name = "test"
    test.add_subsystem(step, "step")
    test.add_subsystem(sys1, "integrator1")
    test.add_subsystem(sys2, "integrator2")
    test.connect("step", "y", "integrator2", "u")
    test.connect("integrator2", "y", "integrator1", "u")
    test.plot_graphe()
    test.name = "double integrator with auto solver"
    test.compute_trajectory()  # Doing weird at discontinuities
    test.name = "double integrator with scipy solver"
    test.compute_trajectory(solver="scipy")
    test.name = "double integrator with custom fixed step euler"
    test.compute_trajectory(solver="euler")


def diagram_in_a_diagram(debug_print=False):

    # Plant system
    sys1 = Integrator()
    sys1.x0[0] = 0.0

    sys2 = Integrator()
    sys2.x0[0] = 1.0

    # # Controllers
    ctl1 = PropController()
    ctl1.params["Kp"] = 1.0

    # Source input
    step = Step()
    step.params["initial_value"] = np.array([0.0])
    step.params["final_value"] = np.array([1.0])
    step.params["step_time"] = 5.0

    # Baseline validation
    test = DiagramSystem()
    test.debug_print = debug_print
    test.name = "test"
    test.add_subsystem(step, "step")
    test.add_subsystem(ctl1, "ctl")
    test.add_subsystem(sys1, "integrator1")
    test.add_subsystem(sys2, "integrator2")
    test.connect("step", "y", "ctl", "r")
    test.connect("ctl", "u", "integrator1", "u")
    test.connect("integrator1", "y", "integrator2", "u")
    test.connect("integrator2", "y", "ctl", "y")
    test.plot_graphe()
    test.compute_trajectory(dt=0.1, solver="euler")

    # # # Diagram
    diagram = DiagramSystem()
    diagram.debug_print = debug_print
    diagram.name = "DiagramSystem"
    diagram.add_input_port("u", dim=1, nominal_value=np.array([0.0]))
    diagram.add_subsystem(sys1, "integrator1")
    diagram.add_subsystem(sys2, "integrator2")
    diagram.connect("input", "u", "integrator1", "u")
    diagram.connect("integrator1", "y", "integrator2", "u")
    diagram.connect_new_output_port("integrator2", "y", "y", dependencies=())

    diagram.plot_graphe()

    diagram2 = DiagramSystem()
    diagram2.debug_print = debug_print

    diagram2.add_subsystem(step, "step")
    diagram2.add_subsystem(ctl1, "ctl")
    diagram2.add_subsystem(diagram, "double")

    diagram2.connect("step", "y", "ctl", "r")
    diagram2.connect("ctl", "u", "double", "u")
    diagram2.connect("double", "y", "ctl", "y")
    diagram2.plot_graphe()
    diagram2.compute_trajectory(dt=0.1, solver="euler")

    return test, diagram, diagram2


if __name__ == "__main__":
    # sys = system_test()
    # sim = simulator_test()
    # dia = diagram_test()
    # pendulum_test()
    # diagram = algebraic_loop()  # compile() (via f_fast / Simulator) runs check_algebraic_loops()
    # solver_doing_weird_at_discontinuities()  # use euler / fixed dt; Simulator picks euler when solver_info["discontinuous_behavior"]
    test, d1, d2 = diagram_in_a_diagram(debug_print=False)
