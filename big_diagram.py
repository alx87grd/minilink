import numpy as np
from framework import DynamicSystem, GrapheSystem, Step, StaticSystem
import numpy as np


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
class Controller( StaticSystem ):
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

        u = Kp * ( r- y) 

        u = np.array([u])

        return u
    

######################################################################
if __name__ == "__main__":

    # Plant system
    sys = Integrator()

    sys.x0[0] = 20.0


    ctl1 = Controller()
    ctl1.params["Kp"] = 1.0
    ctl2 = Controller()
    ctl2.params["Kp"] = 1.0

    # Source input
    step = Step( 
        initial_value=np.array([0.0]),
        final_value=np.array([1.0]),
        step_time=10.0,
        )

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
    
    diagram.render_graphe()

    from analysis import Simulator, plot_trajectory

    diagram.x0[0] = 2.0
    diagram.x0[1] = 2.0

    sim = Simulator(diagram, t0=0, tf=20, n_steps=10000)
    x_traj, u_traj, t_traj, y_traj = sim.solve(show=True)


