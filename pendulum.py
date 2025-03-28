from framework import DynamicSystem, GrapheSystem, Step
import numpy as np


class Pendulum(DynamicSystem):
    def __init__(self):
        super().__init__(2, 1, 1)
        
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

        self.outputs['y'].labels = ["noisy theta"]


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

        y = x[0] + v

        return y
    

######################################################################
if __name__ == "__main__":

    # Plant system
    sys = Pendulum()

    sys.params['m'] = 1.0
    sys.params['l'] = 5.0

    # Source input
    step = Step( 
        initial_value=np.array([0.0]),
        final_value=np.array([-20.0]),
        step_time=10.0,
        )

    # Diagram
    diagram = GrapheSystem()
    diagram.add_system(sys,'plant')
    diagram.add_system(sys,'plant2')
    diagram.add_system(step, 'step')
    diagram.add_edge('step','y','plant','u')
    # diagram.add_edge('step','y','plant2','u')
    diagram.render_graphe()

    from analysis import Simulator, plot_trajectory

    diagram.x0[0] = 2.0
    diagram.x0[2] = 2.0

    sim = Simulator(diagram, t0=0, tf=20, n_steps=10000)
    x_traj, u_traj, t_traj, y_traj = sim.solve(show=True)
