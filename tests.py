from framework import DynamicSystem, GrapheSystem, Step, StaticSystem, WhiteNoise
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

    

######################################################################
if __name__ == "__main__":

    simulator_test()


    
