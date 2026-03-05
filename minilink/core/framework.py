import numpy as np
from minilink.graphical.graphe import (
    plot_graphviz,
    get_system_block_html,
    get_system_graphe,
)
from minilink.core.analysis import Simulator


######################################################################
class VectorSignal:
    """
    A class to represent a vector signal

    Attributes
    ----------
    dim : int
    the dimension of the signal
    id : str
    the identifier of the signal
    labels : list of str
    the label of each component of the signal
    units : list of str
    the units of each component of the signal
    upper_bound : np.ndarray
    the upper bound of each component of the signal
    lower_bound : np.ndarray
    the lower bound of each component of the signal
    nominal_value : np.ndarray
    the nominal value of the signal
    """

    def __init__(self, dim=1, id="x", nominal_value=None):

        self.id = id
        self.dim = dim
        self.labels = [f"{id}[{i}]" for i in range(self.dim)]
        self.units = [""] * self.dim
        self.upper_bound = np.inf * np.ones(self.dim)
        self.lower_bound = -np.inf * np.ones(self.dim)
        self.set_nominal_value(nominal_value)

    ######################################################################
    def set_nominal_value(self, nominal_value=None):

        if nominal_value is not None:
            assert len(nominal_value) == self.dim, "Nominal value has wrong dimensions"
            self.nominal_value = np.array(nominal_value)
        else:
            self.nominal_value = np.zeros(self.dim)

    ##########################################
    def __repr__(self):

        return f"VectorSignal: dim={self.dim}, nominal={self.nominal_value}"


######################################################################
class InputPort(VectorSignal):
    """
    A VectorSignal plus a default callback function to get the signal value
    """

    ##############################################
    def get_signal(self, t=0) -> np.ndarray:

        return self.nominal_value


######################################################################
class OutputPort(VectorSignal):
    """
    A VectorSignal plus a "compute" callback function to compute the signal value

    Attributes
    ----------
    compute : function
    the function to compute the signal value based on the inputs, state, time and parameters
    dependencies : list of dict key
    the list of input ports needed to compute the signal, required to avoid algebraic loops when solving diagram
    TODO: Update None vs all vs list options
    """

    ##############################################
    def __init__(self, dim=1, id="y", function=None, dependencies="all"):

        VectorSignal.__init__(self, dim=dim, id=id)

        if function is not None:
            self.compute = function
        else:
            self.compute = self.default_function

        self.dependencies = dependencies

    ##############################################
    def default_function(self, x=None, u=None, t=0, param=None) -> np.ndarray:
        return self.nominal_value


######################################################################
class System:

    def __init__(self, n=0, m=0, p=1):

        # Dimensions
        self.n = n
        self.m = m
        self.p = p

        # Name
        self.name = "System"

        # State properties
        self.state = VectorSignal(n, "x")

        # Initial state
        self.x0 = np.zeros(self.n)

        # Inputs and outputs ports
        self.inputs = {}
        self.add_input_port(self.m, "u")
        self.outputs = {}
        self.add_output_port(self.p, "y", function=self.h, dependencies="all")

        # Parameters dictionary
        self.params = {}

        # Caracteristics useful to select automatic solver parameters
        self.solver_info = {
            "continuous_time_equation": True,
            "smallest_time_constant": 0.001,
            "discontinuous_behavior": False,  # Will use a fixed time step
            "discrete_time_period": None,
            "require_building": False,  # If True, the system needs to be built before being simulated
        }

    ######################################################################
    def f(self, x, u, t=0, params=None) -> np.ndarray:
        dx = np.zeros(self.n)
        return dx

    ######################################################################
    def h(self, x, u, t=0, params=None) -> np.ndarray:
        y = np.zeros(self.p)
        return y

    ######################################################################
    def fsim(self, t, x) -> np.ndarray:
        u = self.get_u_from_input_ports(t)
        dx = self.f(x, u, t)
        return dx

    ######################################################################
    def compute_state(self, x, u, t=0, params=None):
        return x

    ######################################################################
    def add_input_port(self, dim, id, nominal_value=None):
        self.inputs[id] = InputPort(dim, id, nominal_value)
        self.recompute_input_properties()

    ######################################################################
    def add_output_port(self, dim, id, function=None, dependencies="all"):
        self.outputs[id] = OutputPort(dim, id, function, dependencies)

    ######################################################################
    def recompute_input_properties(self):

        self.m = 0
        for key, port in self.inputs.items():
            self.m += port.dim

    ######################################################################
    def get_all_input_labels_and_units(self):

        input_labels = []
        input_units = []

        for key, port in self.inputs.items():
            for i in range(port.dim):
                input_labels.append(port.labels[i])
                input_units.append(port.units[i])

        return input_labels, input_units

    ######################################################################
    def get_u_from_input_ports(self, t=0) -> np.ndarray:
        """
        Get the nominal value of all input ports

        This function is called when not part of a diagram

        Parameters
        ----------
        t : float
        the time at which the input signal is requested

        Returns
        -------
        np.ndarray
        the concatenated nominal values of all input ports

        """
        u = np.zeros(self.m)
        i = 0
        for key, port in self.inputs.items():
            # Get the signal value of the port at time t and concatenate it to u
            # By default, the signal value is the nominal value of the port, but it can be overridden by a custom function
            u[i : i + port.dim] = port.get_signal(t)
            i += port.dim
        return u

    ######################################################################
    def get_port_values_from_u(self, u):
        """
        Get the values of all input ports from a concatenated array

        Parameters
        ----------
        u : np.ndarray
        the concatenated values of all input ports

        Returns
        -------
        dict
        the values of input ports signals in a dictionary

        """
        input_signals = {}
        i = 0
        for port_id, port in self.inputs.items():
            input_signals[port_id] = u[i : i + port.dim]
            i += port.dim
        return input_signals

    ##########################

    # Shortcut functions

    ######################################################################
    def get_block_html(self, html_id="sys1"):
        """
        Get the HTML representation of the block for the label in the diagram
        """
        return get_system_block_html(self, html_id)

    ######################################################################
    def print_html(self):

        try:
            import IPython.display as display

            display.display(display.HTML(self.get_block_html()))
        except ImportError:
            print("IPython is not available")
            return

    ######################################################################
    def get_graphe(self):
        return get_system_graphe(self)

    ######################################################################
    def _repr_svg_(self):
        """Display the svg rendered graphe in the notebook"""
        g = self.get_graphe()
        return g._repr_image_svg_xml()

    ######################################################################
    def plot_graphe(self, filename=None):

        g = self.get_graphe()

        plot_graphviz(g, filename=filename)

    ######################################################################
    def compute_trajectory(
        self, t0=0, tf=10, n_steps=None, dt=None, solver="scipy", show=True
    ):

        sim = Simulator(self, t0, tf, n_steps, dt, solver)
        traj = sim.solve(show=show)

        return traj


######################################################################
class StaticSystem(System):

    def __init__(self, m, p):

        System.__init__(self, 0, m, p)
        self.name = "StaticSystem"


######################################################################
class DynamicSystem(System):

    # dx = f(x, u, t)
    # y  = g(x, u, t)

    def __init__(self, n, m, p):

        System.__init__(self, n, m, p)

        self.name = "DynamicSystem"

        # By default, a dynamic system's output 'y' only depends on its state 'x', not 'u'
        self.outputs["y"].dependencies = ()

        self.add_output_port(self.n, "x", function=self.compute_state)


######################################################################
if __name__ == "__main__":

    x = VectorSignal(2, "x")

    sys = DynamicSystem(2, 1, 1)

    func = StaticSystem(2, 2)
