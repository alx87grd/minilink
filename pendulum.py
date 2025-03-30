from framework import DynamicSystem, GrapheSystem, Step, StaticSystem, WhiteNoise
import numpy as np


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
        dx[1] = -g / l * np.sin(theta) + 1 / (m * l ** 2) * ( u + w )

        return dx
    
    def h(self, x, u, t = 0, params = None):

        signals = self.u2input_signals(u)
        v = signals['v']

        y = np.zeros(self.p)

        y[0] = x[0] + v
        y[1] = x[1] + v

        return y
    

######################################################################
class Controller( StaticSystem ):
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
if __name__ == "__main__":

    # Plant system
    sys = Pendulum()

    sys.params['m'] = 1.0
    sys.params['l'] = 5.0

    sys.x0[0] = 2.0

    # Source input
    step = Step( 
        initial_value=np.array([0.0]),
        final_value=np.array([1.0]),
        step_time=10.0,
        )
    
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

    # # Diagram
    # diagram = GrapheSystem()
    # diagram.add_system(sys,'plant')
    # diagram.add_system(sys,'plant2')
    # diagram.add_system(step, 'step')
    # diagram.add_edge('step','y','plant','u')
    # # diagram.add_edge('step','y','plant2','u')
    # diagram.render_graphe()

    from analysis import Simulator, plot_trajectory

    # # # diagram.x0[0] = 2.0
    # # # diagram.x0[2] = 2.0

    # sim = Simulator(diagram, t0=0, tf=20, n_steps=10000)
    # x_traj, u_traj, t_traj, y_traj = sim.solve(show=True)


    # Closed loop system
    ctl = Controller()

    ctl.params['Kp'] = 1000.0
    ctl.params['Kd'] = 100.0

    # Diagram
    diagram2 = GrapheSystem()
    diagram2.add_system(step, 'step')
    diagram2.add_system(ctl,'controller')
    diagram2.add_system(sys,'plant')
    diagram2.add_system(noise, 'noise')
    diagram2.add_system(noise2, 'noise2')
    # diagram2.add_system(sys,'plant2')
    # diagram2.add_system(sys,'plant3')
    # diagram2.add_system(sys,'plant4')
    # diagram2.add_system(sys,'plant5')
    diagram2.add_edge('step','y','controller','ref')
    diagram2.add_edge('controller','u','plant','u')
    diagram2.add_edge('noise','y','plant','w')
    diagram2.add_edge('noise2','y','plant','v')
    diagram2.add_edge('plant','y','controller','y')
    # diagram2.add_edge('plant','y','plant2','u')
    # diagram2.add_edge('plant2','y','plant3','u')
    # diagram2.add_edge('plant3','y','plant4','u')
    # diagram2.add_edge('plant4','y','plant5','u')
    # diagram2.add_edge('plant5','y','controller','y')
    diagram2.render_graphe()

    # diagram2.solver_info['solver'] = 'euler'

    # Algebraic loop not supported yet

    sim = Simulator(diagram2, t0=0, tf=20, dt=0.01)
    sim.solver = 'euler'
    x_traj, u_traj, t_traj, y_traj = sim.solve(show=True)


    
