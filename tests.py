from framework import DynamicSystem, GrapheSystem, Step, StaticSystem, WhiteNoise
from analysis import Simulator, plot_trajectory
import numpy as np


######################################################################
class Pendulum(DynamicSystem):
    def __init__(self):
        super().__init__(2, 1, 2)
        
        self.params = {
            "g": 9.81,
            "m": 1.0,
            "l": 1.0
        }

        self.name = "Pendulum"

        self.state_labels = ["theta", "theta_dot"]
        self.state_units = ["rad", "rad/s"]

        self.inputs['u'].labels = ["torque"]
        self.inputs['u'].units = ["Nm"]

        self.add_input_port("w", 1, default_value=np.array([0.0]))
        self.add_input_port("v", 1, default_value=np.array([0.0]))

        self.outputs = {}
        self.add_output_port("y", self.p, function=self.h, dependencies=['v'])


    def f(self, x, u, t = 0 , params = None):

        if params is None:
            params = self.params
        g = params["g"]
        m = params["m"]
        l = params["l"]

        theta = x[0]
        theta_dot = x[1]

        signals = self.u2input_signals(u)
        w = signals['w']
        u = signals['u']

        dx = np.zeros(2)
        dx[0] = x[1]
        dx[1] = -g / l * np.sin(theta) + 1 / (m * l ** 2) * ( u[0] + w[0] )

        return dx
    
    def h(self, x, u, t = 0, params = None):

        signals = self.u2input_signals(u)
        v = signals['v']

        y = np.zeros(self.p)

        y[0] = x[0] + v[0]
        y[1] = x[1] + v[0]

        return y
    

######################################################################
class KDController( StaticSystem ):
    def __init__(self):
        super().__init__(3, 1)

        self.params = {
            "Kp": 10.0,
            "Kd": 1.0,
        }

        self.name = "Controller"

        self.inputs = {}
        self.add_input_port("ref", 1, default_value=np.array([0.0]))
        self.add_input_port("y", 2, default_value=np.array([0.0, 0.0]))
        self.inputs['y'].labels = ["theta", "theta_dot"]
        self.inputs['y'].units = ["rad", "rad/s"]

        self.outputs = {}
        self.add_output_port("u", 1, function=self.ctl, dependencies=['ref', 'y'])
        self.outputs['u'].labels = ["torque"]
        self.outputs['u'].units = ["Nm"]

    ######################################################################
    def ctl(self, x, u, t = 0, params = None):

        if params is None:
            params = self.params

        Kp = params["Kp"]
        Kd = params["Kd"]

        ref   = u[0]
        theta = u[1]
        theta_dot = u[2]

        torque = Kp * ( ref - theta)  - Kd * theta_dot

        u = np.array([torque])

        return u


######################################################################
class Integrator(DynamicSystem):
    def __init__(self):
        super().__init__(1, 1, 1)
        
        self.params = {
            "k": 1.0
        }

        self.name = "Integrator"

        self.outputs = {}
        self.add_output_port("y", 1, function=self.h, dependencies=[])

    def f(self, x, u, t = 0 , params = None):

        if params is None:
            params = self.params
        k = params["k"]

        dx = np.zeros(self.n)
        dx[0] = k * u[0]

        return dx
    
    def h(self, x, u, t = 0, params = None):
            
        y = np.zeros(self.p)
        y[0] = x[0]

        return y

######################################################################
class PropController( StaticSystem ):
    def __init__(self):
        super().__init__(2, 1)

        self.params = {
            "Kp": 10.0,
        }

        self.name = "Controller"

        self.inputs = {}
        self.add_input_port("ref", 1, default_value=np.array([0.0]))
        self.add_input_port("y", 1, default_value=np.array([0.0]))

        self.outputs = {}
        self.add_output_port("u", 1, function=self.ctl, dependencies=['ref', 'y'])

    ######################################################################
    def ctl(self, x, u, t = 0, params = None):

        if params is None:
            params = self.params

        Kp = params["Kp"]

        r  = u[0]
        y  = u[1]

        u = Kp * ( r - y ) 

        u = np.array([u])

        return u
    

######################################################################
######################################################################
######################################################################
# Tests functions
######################################################################
######################################################################
######################################################################

def simulator_test():

    # Defining a test system
    sys1 = DynamicSystem(2, 1, 1)

    sys1.add_input_port("v", 1, default_value=np.array([0.6]))
    sys1.add_input_port("w", 1, default_value=np.array([-10.0]))

    def f(x, u, t):
        a = u[0]
        v = u[1]
        w = u[2]
        dx = np.zeros(2)
        dx[0] = x[1]
        dx[1] = -x[0] - x[1] + a + v
        return dx

    def h(x, u, t):
        a = u[0]
        v = u[1]
        w = u[2]
        return x[0] + w

    sys1.f = f
    sys1.h = h
    sys1.x0 = np.array([1.0, 0.0])

    # Running the simulation

    sim = Simulator(sys1, t0=0, tf=25)

    x_traj, u_traj, t_traj, y_traj = sim.solve(show=True)

    plot_trajectory(sys1, t_traj, x_traj)
    # plot_trajectory(sys1, t_traj, u_traj=u_traj)
    plot_trajectory(sys1, t_traj, x_traj, u_traj)
    plot_trajectory(sys1, t_traj, y_traj=y_traj)

    np.set_printoptions(precision=2, suppress=True)
    print(f"Time vector:\n {t_traj}")
    print(f"Input trajectory:\n {u_traj}")
    print(f"State trajectory:\n {x_traj}")
    print(f"Output trajectory:\n {y_traj}")

    return sim


######################################################################
def system_test():

    sys1 = DynamicSystem(2, 1, 1)

    sys1.add_input_port("w", 2, default_value=np.array([7.7, 2.2]))
    sys1.add_input_port("v", 1, default_value=np.array([1.1]))

    print("Sys1 u dim:", sys1.m)
    print("Sys1 x dim:", sys1.n)
    print("Sys1 y dim:", sys1.p)

    default_input_signals = sys1.collect_input_signals()
    u = sys1.input_signals2u(default_input_signals)
    u2 = sys1.get_u_from_input_ports()
    assert np.allclose(u, u2)

    print("Default u:", u)
    # print("Default u:", u2)
    print("Default input signals:", default_input_signals)


    u = np.random.rand(sys1.m)
    u2 = sys1.input_signals2u(sys1.u2input_signals(u))
    assert np.allclose(u, u2)

    sys1.print_html()

    sys1.plot_graphe()

    return sys1


######################################################################
def diagram_test():

    sys1 = DynamicSystem(2, 1, 1)
    sys1.add_input_port("w", 2, default_value=np.array([7.7, 2.2]))
    sys1.add_input_port("v", 1, default_value=np.array([1.1]))
    sys2 = DynamicSystem(2, 1, 1)
    sys3 = StaticSystem(1, 1)
    sys4 = DynamicSystem(2, 1, 1)
    step = Step(np.array([0.0]), np.array([1.0]), 1.0)

    gsys = GrapheSystem()


    gsys.add_system(sys1, "sys1")
    gsys.add_system(sys2, "sys2")
    gsys.add_system(sys3, "sys3")
    gsys.add_system(sys4, "sys4")
    gsys.add_system(step, "step")

    print("List of subsystems:\n")
    print(gsys.subsystems)

    print("List of edges before connections:\n")
    print(gsys.edges)

    gsys.add_edge("sys1", "y", "sys2", "u")
    gsys.add_edge("sys2", "y", "sys3", "u")
    gsys.add_edge("sys2", "y", "sys4", "u")
    gsys.add_edge("sys4", "y", "sys1", "u")
    gsys.add_edge("step", "y", "sys1", "v")
    gsys.add_edge("sys4", "y", "sys1", "w")
    # gsys.add_edge("sys4", "y", "sys1", "u")
    # gsys.add_edge("sys3", "y", "sys1", "w")

    print("List of edges after connections:\n")
    print(gsys.edges)

    g = gsys.plot_graphe()

    print("sys.n = ", gsys.n)
    print("sys.m = ", gsys.m)
    print("sys.p = ", gsys.p)
    print("sys.state_label = ", gsys.state_label)

    return gsys

######################################################################
def pendulum_test():

    # Plant system
    sys = Pendulum()
    sys.params['m'] = 1.0
    sys.params['l'] = 1.0
    sys.x0[0] = 2.0

    # Source input
    step = Step()
    step.params['initial_value'] = np.array([0.0])
    step.params['final_value'] = np.array([0.2])
    step.params['step_time'] = 15.0
    
    # Noisy input
    noise = WhiteNoise(1)
    noise.params['var'] = 10.0
    noise.params['mean'] = 0.0
    noise.params['seed'] = 1

    # Noisy measurement
    noise2 = WhiteNoise(1)
    noise2.params['var'] = 10.0
    noise2.params['mean'] = 0.0
    noise2.params['seed'] = 2

    # # Diagram
    diagram = GrapheSystem()
    diagram.add_system(sys,'plant')
    diagram.add_system(sys,'plant2')
    diagram.add_system(step, 'step')
    diagram.add_system(noise, 'dist')
    diagram.add_system(noise2, 'noise')
    diagram.add_edge('step','y','plant','u')
    diagram.add_edge('step','y','plant2','u')
    diagram.add_edge('dist','y','plant','w')
    diagram.add_edge('noise','y','plant','v')
    diagram.plot_graphe()

    sim = Simulator(diagram, t0=0, tf=20, dt=0.01)
    sim.solver = 'euler' # WhiteNoise is discontinuous
    x_traj, u_traj, t_traj, y_traj = sim.solve(show=True)

    return sim

######################################################################
def closedloop_pendulum_test():

    # Plant system
    sys = Pendulum()

    sys.params['m'] = 1.0
    sys.params['l'] = 5.0
    sys.x0[0] = 2.0

    # Source input
    step = Step()
    step.params['initial_value'] = np.array([0.0])
    step.params['final_value'] = np.array([1.0])
    step.params['step_time'] = 10.0

    # Closed loop system
    ctl = KDController()
    ctl.params['Kp'] = 1000.0
    ctl.params['Kd'] = 100.0

    # Diagram
    diagram2 = GrapheSystem()

    diagram2.add_system(step, 'step')
    diagram2.add_system(ctl,'controller')
    diagram2.add_system(sys,'plant')

    diagram2.add_edge('step','y','controller','ref')
    diagram2.add_edge('controller','u','plant','u')
    diagram2.add_edge('plant','y','controller','y')

    diagram2.plot_graphe()

    sim = Simulator(diagram2, t0=0, tf=20, dt=0.01)
    x_traj, u_traj, t_traj, y_traj = sim.solve(show=True)

######################################################################
def closedloop_noisy_pendulum_test():

    # Plant system
    sys = Pendulum()

    sys.params['m'] = 1.0
    sys.params['l'] = 5.0

    sys.x0[0] = 2.0

    # Source input
    step = Step()
    step.params['initial_value'] = np.array([0.0])
    step.params['final_value'] = np.array([1.0])
    step.params['step_time'] = 10.0
    
    # Noisy input
    noise = WhiteNoise(1)
    noise.params['var'] = 1.0
    noise.params['mean'] = 0.0
    noise.params['seed'] = 1

    # Noisy measurement
    noise2 = WhiteNoise(1)
    noise2.params['var'] = 0.1
    noise2.params['mean'] = 0.0
    noise2.params['seed'] = 2

    # Closed loop system
    ctl = KDController()
    ctl.params['Kp'] = 1000.0
    ctl.params['Kd'] = 100.0

    # Diagram
    diagram2 = GrapheSystem()

    diagram2.add_system(step, 'step')
    diagram2.add_system(ctl,'controller')
    diagram2.add_system(sys,'plant')
    diagram2.add_system(noise, 'noise')
    diagram2.add_system(noise2, 'noise2')

    diagram2.add_edge('step','y','controller','ref')
    diagram2.add_edge('controller','u','plant','u')
    diagram2.add_edge('plant','y','controller','y')
    diagram2.add_edge('noise','y','plant','w')
    diagram2.add_edge('noise2','y','plant','v')

    diagram2.plot_graphe()

    sim = Simulator(diagram2, t0=0, tf=20, dt=0.01)
    sim.solver = 'euler'
    x_traj, u_traj, t_traj, y_traj = sim.solve(show=True)


######################################################################
def cascade_controllers_test():

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
    step.params['initial_value'] = np.array([0.0])
    step.params['final_value'] = np.array([1.0])
    step.params['step_time'] = 10.0

    # # Diagram
    diagram = GrapheSystem()

    diagram.add_system(step, 'step')
    diagram.add_system(ctl1, 'controller1')
    diagram.add_system(ctl2, 'controller2')
    diagram.add_system(sys,'integrator1')
    diagram.add_system(sys,'integrator2')

    diagram.add_edge('integrator1','y','integrator2','u')
    diagram.add_edge('controller2','u','integrator1','u')
    diagram.add_edge('integrator1','y','controller2','y')
    diagram.add_edge('controller1','u','controller2','ref')
    diagram.add_edge('integrator2','y','controller1','y')
    diagram.add_edge('step','y','controller1','ref')
    
    diagram.plot_graphe()

    sim = Simulator(diagram, t0=0, tf=20, n_steps=10000)
    x_traj, u_traj, t_traj, y_traj = sim.solve(show=True)

    return diagram


    

######################################################################
if __name__ == "__main__":


    # sys = system_test()
    # sim = simulator_test()
    # dia = diagram_test()
    # pendulum_test()
    # closedloop_pendulum_test()
    # closedloop_noisy_pendulum_test()
    diagram = cascade_controllers_test()




    
