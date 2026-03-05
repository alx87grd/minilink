"""
Core Framework Definitions

This module defines the foundational classes for the minilink block-diagram simulator.
It includes base signal representations (VectorSignal, InputPort, OutputPort) and the
base System class from which all static and dynamic blocks inherit.
"""

import numpy as np
from minilink.graphical.graphe import (
    plot_graphviz,
    get_system_block_html,
    get_system_graphe,
)
from minilink.graphical.primitives import Point, translation_matrix
from minilink.core.analysis import Simulator


######################################################################
class VectorSignal:
    """
    A class representing a generic vector signal.

    Attributes
    ----------
    dim : int
        The dimension of the vector signal.
    id : str
        The programmatic string identifier of the signal (e.g., 'x', 'u').
    labels : list of str
        The display string label for each component of the signal.
    units : list of str
        The physical units for each component of the signal.
    upper_bound : np.ndarray
        The upper numerical bound of each component.
    lower_bound : np.ndarray
        The lower numerical bound of each component.
    nominal_value : np.ndarray
        The default or operating point value of the signal.
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
        """
        Set the nominal (default) value of the vector signal.

        Parameters
        ----------
        nominal_value : list or np.ndarray, optional
            The nominal value to set. Must match the dimension `dim`.
            If None, defaults to an array of zeros.
        """
        if nominal_value is not None:
            assert len(nominal_value) == self.dim, "Nominal value has wrong dimensions"
            self.nominal_value = np.array(nominal_value)
        else:
            self.nominal_value = np.zeros(self.dim)

    ##########################################
    def __repr__(self):
        """
        Return a string representation of the VectorSignal.

        Returns
        -------
        str
            A string describing the dimension and nominal value.
        """
        return f"VectorSignal: dim={self.dim}, nominal={self.nominal_value}"


######################################################################
class InputPort(VectorSignal):
    """
    A VectorSignal plus a default callback function to get the signal value
    """

    ##############################################
    def get_signal(self, t=0) -> np.ndarray:
        """
        Get the value of the input signal at a given time.

        Parameters
        ----------
        t : float, optional
            The time at which to evaluate the signal (default is 0).

        Returns
        -------
        np.ndarray
            The nominal value of the input port.
        """
        return self.nominal_value


######################################################################
class OutputPort(VectorSignal):
    """
    An output signal port containing a callback function to compute its value dynamically.

    Attributes
    ----------
    compute : callable
        The function used to compute the signal value based on the inputs, state, time, and parameters.
        Signature: `compute(x, u, t, param) -> np.ndarray`
    dependencies : list of str, tuple, or "all"
        The list of `InputPort` IDs that this output directly depends on for immediate feedthrough.
        - `"all"`: depends on all inputs of the system (safest, but can create false algebraic loops in MIMO systems).
        - `list`/`tuple`: specific input IDs (e.g., `["ref", "y"]`).
        - `empty list/tuple`: depends only on the state `x` or internal time, no direct feedthrough from inputs.
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
        """
        Default compute function returning the nominal value.

        Parameters
        ----------
        x : np.ndarray, optional
            The state vector of the system.
        u : np.ndarray, optional
            The input vector of the system.
        t : float, optional
            The current time.
        param : dict, optional
            A dictionary of system parameters.

        Returns
        -------
        np.ndarray
            The nominal value of the output port.
        """
        return self.nominal_value


######################################################################
class System:
    """
    The base class for all blocks in the minilink framework.

    A System represents a mathematical block with `n` states, `m` inputs (from `InputPort`s),
    and `p` outputs (to `OutputPort`s). It provides methods for state derivatives (`f`)
    and outputs (`h`), which are meant to be overridden by subclasses.
    """

    def __init__(self, n=0, m=0, p=1):
        """
        Initialize the system with dimensions.

        Parameters
        ----------
        n : int, optional
            Number of continuous states (default is 0).
        m : int, optional
            Number of input dimensions (default is 0).
        p : int, optional
            Number of output dimensions (default is 1).
        """
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
        """
        Compute the state derivative `dx/dt`.

        Parameters
        ----------
        x : np.ndarray
            The current state vector.
        u : np.ndarray
            The current input vector.
        t : float, optional
            The current time.
        params : dict, optional
            The system parameters.

        Returns
        -------
        np.ndarray
            The state derivative vector.
        """
        dx = np.zeros(self.n)
        return dx

    ######################################################################
    def h(self, x, u, t=0, params=None) -> np.ndarray:
        """
        Compute the output `y`.

        Parameters
        ----------
        x : np.ndarray
            The current state vector.
        u : np.ndarray
            The current input vector.
        t : float, optional
            The current time.
        params : dict, optional
            The system parameters.

        Returns
        -------
        np.ndarray
            The output vector.
        """
        y = np.zeros(self.p)
        return y

    ######################################################################
    def fsim(self, t, x) -> np.ndarray:
        """
        Compute the state derivative using default input port values.

        Note: This function is used by the solver when the system is simulated alone, i.e. not part of a diagram.

        Parameters
        ----------
        t : float
            The current time.
        x : np.ndarray
            The current state vector.

        Returns
        -------
        np.ndarray
            The state derivative vector.
        """
        u = self.get_u_from_input_ports(t)
        dx = self.f(x, u, t)
        return dx

    ######################################################################
    def compute_state(self, x, u, t=0, params=None):
        """
        Helper function to output the state vector directly.

        Parameters
        ----------
        x : np.ndarray
            The current state vector.
        u : np.ndarray
            The current input vector.
        t : float, optional
            The current time.
        params : dict, optional
            The system parameters.

        Returns
        -------
        np.ndarray
            The state vector.
        """
        return x

    ######################################################################
    def add_input_port(self, dim, id, nominal_value=None):
        """
        Add a new input port to the system.

        Parameters
        ----------
        dim : int
            The dimension of the input port.
        id : str
            The programmatic identifier for the port.
        nominal_value : np.ndarray, optional
            The nominal value of the signal.
        """
        self.inputs[id] = InputPort(dim, id, nominal_value)
        self.recompute_input_properties()

    ######################################################################
    def add_output_port(self, dim, id, function=None, dependencies="all"):
        """
        Add a new output port to the system.

        Parameters
        ----------
        dim : int
            The dimension of the output port.
        id : str
            The programmatic identifier for the port.
        function : callable, optional
            The function used to compute the port's output signal.
        dependencies : list of str, tuple, or "all", optional
            The list of input port IDs this output depends on (default is "all").
        """
        self.outputs[id] = OutputPort(dim, id, function, dependencies)

    ######################################################################
    def recompute_input_properties(self):
        """
        Recalculate the total input dimension `m` based on all input ports.
        """
        self.m = 0
        for key, port in self.inputs.items():
            self.m += port.dim

    ######################################################################
    def get_all_input_labels_and_units(self):
        """
        Retrieve a flattened list of all input labels and units across all ports.

        Returns
        -------
        input_labels : list of str
            Combined list of signal labels.
        input_units : list of str
            Combined list of signal units.
        """
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
        Get the nominal value of all input ports.

        This function is called when not part of a diagram.

        Parameters
        ----------
        t : float, optional
            The time at which the input signal is requested.

        Returns
        -------
        np.ndarray
            The concatenated nominal values of all input ports.
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
        Get the values of all input ports from a concatenated array.

        Parameters
        ----------
        u : np.ndarray
            The concatenated values of all input ports.

        Returns
        -------
        dict
            A dictionary mapping port IDs to their respective signal values.
        """
        input_signals = {}
        i = 0
        for port_id, port in self.inputs.items():
            input_signals[port_id] = u[i : i + port.dim]
            i += port.dim
        return input_signals

    ######################################################################
    def u2input_signal(self, u, port_id):
        """
        Extract the signal value for a specific input port from the concatenated input array.

        Parameters
        ----------
        u : np.ndarray
            The concatenated values of all input ports.
        port_id : str
            The identifier of the input port whose signal value is to be extracted.

        Returns
        -------
        np.ndarray
            The signal value corresponding to the specified input port.
        """
        input_signals = self.get_port_values_from_u(u)
        return input_signals[port_id]

    ######################################################################
    ######################################################################

    # Shortcut functions

    ######################################################################
    ######################################################################

    ######################################################################
    def get_block_html(self, label="sys1"):
        """
        Get the HTML representation of the block for the label in the diagram.

        Parameters
        ----------
        label : str, optional
            The label identifying the block in the HTML output (default is "sys1").

        Returns
        -------
        str
            The HTML string for the block.
        """
        return get_system_block_html(self, label)

    ######################################################################
    def print_html(self):
        """
        Render and display the HTML representation of the system in IPython.
        """
        try:
            import IPython.display as display

            display.display(display.HTML(self.get_block_html()))
        except ImportError:
            print("IPython is not available")
            return

    ######################################################################
    def get_graphe(self):
        """
        Generate a Graphviz Digraph representation of the system.

        Returns
        -------
        graphviz.Digraph
            The graph representing the system.
        """
        return get_system_graphe(self)

    ######################################################################
    def _repr_svg_(self):
        """
        Display the SVG rendered graph in IPython notebooks.

        Returns
        -------
        str
            The SVG XML representation of the system graph.
        """
        g = self.get_graphe()
        return g._repr_image_svg_xml()

    ######################################################################
    def plot_graphe(self, filename=None):
        """
        Plot and optionally save the system graph using Graphviz.

        Parameters
        ----------
        filename : str, optional
            File path to save the generated graph PDF. If None, it renders without saving.
        """
        g = self.get_graphe()

        plot_graphviz(g, filename=filename)

    ######################################################################
    def compute_trajectory(
        self, t0=0, tf=10, n_steps=None, dt=None, solver="scipy", show=True
    ):
        """
        Simulate the system and return the computed trajectory.

        Parameters
        ----------
        t0 : float, optional
            Initial simulation time (default is 0).
        tf : float, optional
            Final simulation time (default is 10).
        n_steps : int, optional
            Number of steps for fixed-step solvers or output resolution.
        dt : float, optional
            Time step for fixed-step solvers.
        solver : str, optional
            The solver to use, e.g., "scipy" or "euler" (default is "scipy").
        show : bool, optional
            Whether to display plots after simulation (default is True).

        Returns
        -------
        Trajectory
            An object containing time, state, and input histories.
        """
        sim = Simulator(self, t0, tf, n_steps, dt, solver)
        traj = sim.solve(show=show)

        return traj

    ######################################################################
    # Graphical Animation Engine Baseline
    ######################################################################
    def get_kinematic_geometry(self):
        """
        Defines the rigid-body objects initialized once for drawing.
        By default, the base System generates a point for each state and each input.

        Returns
        -------
        list of minilink.graphical.primitives.GraphicPrimitive
            The list of primitive shapes describing the system.
        """
        primitives = []
        for i in range(self.n):
            primitives.append(Point(color="blue", marker="o"))
        for i in range(self.m):
            primitives.append(Point(color="red", marker="x"))
        return primitives

    def get_kinematic_transforms(self, x, u, t):
        """
        Computes the transformation matrices corresponding 1-to-1 to the static geometry.
        By default, this maps the n states and m inputs to translations along the x-axis.

        Parameters
        ----------
        x : np.ndarray
            The current state vector.
        u : np.ndarray
            The current input vector.
        t : float
            The current time.

        Returns
        -------
        list of np.ndarray
            A list of 4x4 transformation matrices (for 3D) shifting the points.
        """
        transforms = []

        for i in range(self.n):
            transforms.append(translation_matrix(dx=x[i], dy=float(i)))
        for i in range(self.m):
            transforms.append(translation_matrix(dx=u[i], dy=float(-i - 1)))

        return transforms

    def get_dynamic_geometry(self, x, u, t):
        """
        Defines temporary graphical primitives that change structure.
        Only used for dynamic drawing like paths, trails, or shapes that grow vertices over time.

        Returns
        -------
        list of minilink.graphical.primitives.GraphicPrimitive
            The dynamic primitives generated just for this frame.
        """
        return []


######################################################################
class StaticSystem(System):
    """
    A block-diagram system with no internal continuous states (`n=0`).
    Outputs depend strictly on inputs and time.
    """

    def __init__(self, m, p):
        """
        Initialize a StaticSystem.

        Parameters
        ----------
        m : int
            Number of input dimensions.
        p : int
            Number of output dimensions.
        """
        System.__init__(self, 0, m, p)
        self.name = "StaticSystem"


######################################################################
class DynamicSystem(System):
    """
    A block-diagram system with continuous states (`n > 0`).
    State derivatives (`dx/dt`) are computed from inputs and current state.
    """

    # dx = f(x, u, t)
    # y  = g(x, u, t)

    def __init__(self, n, m, p):
        """
        Initialize a DynamicSystem.

        Parameters
        ----------
        n : int
            Number of continuous states.
        m : int
            Number of input dimensions.
        p : int
            Number of output dimensions.
        """
        System.__init__(self, n, m, p)

        self.name = "DynamicSystem"

        # By default, a dynamic system's output 'y' only depends on its state 'x', not 'u'
        self.outputs["y"].dependencies = ()

        # Add a default output port 'x' that outputs the state vector directly, useful for connecting to other blocks in the diagram
        self.add_output_port(self.n, "x", function=self.compute_state)


######################################################################
if __name__ == "__main__":

    x = VectorSignal(2, "x")

    sys = DynamicSystem(2, 1, 1)

    func = StaticSystem(2, 2)
