import numpy as np
from graphical import plot_graphviz


######################################################################
class Port:

    def __init__(self, name="port", n=1, default_value=None):
        self.name = name
        self.n = n
        if default_value is not None:
            assert len(default_value) == n, "Default value has wrong dimensions"
            self.default_value = default_value
        else:
            self.default_value = np.zeros(self.n)
        self.label = [f"{name}[{i}]" for i in range(self.n)]
        self.units = [""] * self.n
        self.upper_bound = np.inf * np.ones(self.n)
        self.lower_bound = -np.inf * np.ones(self.n)


######################################################################
class InputPort(Port):

    def get_signal(self, t=0) -> np.ndarray:
        return self.default_value


######################################################################
class OutputPort(Port):

    def __init__(self, name="port", n=1, function=None):
        Port.__init__(self, name, n)

        if function is not None:
            self.compute = function
        else:
            self.compute = self.compute_default

    def compute_default(self, x=None, u=None, t=0) -> np.ndarray:
        return self.default_value


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
        self.xbar = np.zeros(self.n)
        self.state_label = [f"x[{i}]" for i in range(self.n)]
        self.state_units = [""] * self.n
        self.state_upper_bound = np.inf * np.ones(self.n)
        self.state_lower_bound = -np.inf * np.ones(self.n)

        # Initial state
        self.x0 = np.zeros(self.n)

        # Inputs and outputs ports
        self.inputs = {}
        self.outputs = {}

        # Parameters dictionary
        self.params = {}

        # Caracteristics useful to select automatic solver parameters
        self.solver_info = {
            "continuous_time_equation": True,
            "smallest_time_constant": 0.001,
            "largest_time_constant": 10,
            "discontinuous_behavior": False,  # Will use a fixed time step
            "discrete_time_period": None,
        }

        self.input_output_dependencies = {}

    ######################################################################
    def f(self, x, u, t=0, params=None) -> np.ndarray:
        dx = np.zeros(self.n)
        return dx

    ######################################################################
    def h(self, x, u, t=0, params=None) -> np.ndarray:
        y = np.zeros(self.p)
        return y

    ######################################################################
    def compute_state(self, x, u, t=0, params=None):
        return x

    ######################################################################
    def add_input_port(self, key, dim=1, default_value=None):
        self.inputs[key] = InputPort(key, dim, default_value)
        self.recompute_input_properties()

    ######################################################################
    def add_output_port(self, key, dim=1, function=None, dependencies='all'):
        self.outputs[key] = OutputPort(key, dim, function)
        self.input_output_dependencies[key] = dependencies

    ######################################################################
    def recompute_input_properties(self):
        self.m = 0
        self.input_label = []
        self.input_units = []

        for key, port in self.inputs.items():
            self.m += port.n
            for i in range(port.n):
                self.input_label.append(port.label[i])
                self.input_units.append(port.units[i])

        if self.m > 0:
            self.input_upper_bound = np.concatenate(
                [port.upper_bound for port in self.inputs.values()]
            )
            self.input_lower_bound = np.concatenate(
                [port.lower_bound for port in self.inputs.values()]
            )
            self.ubar = np.concatenate(
                [port.default_value for port in self.inputs.values()]
            )

        return self.m

    ######################################################################
    def fsim(self, t, x) -> np.ndarray:
        u = self.get_u_from_input_ports(t)
        dx = self.f(x, u, t)
        return dx

    ######################################################################
    def get_u_from_input_ports(self, t=0) -> np.ndarray:
        u = np.zeros(self.m)
        idx = 0
        for key, port in self.inputs.items():
            u[idx : idx + port.n] = port.get_signal(t)
            idx += port.n
        return u

    ######################################################################
    def u2input_signals(self, u):
        input_signals = {}
        idx = 0
        for key, port in self.inputs.items():
            input_signals[key] = u[idx : idx + port.n]
            idx += port.n
        return input_signals

    ######################################################################
    def collect_input_signals(self, t=0) -> dict:
        input_signals = {}
        for key, port in self.inputs.items():
            input_signals[key] = port.get_signal(t)
        return input_signals

    ######################################################################
    def input_signals2u(self, input_signals: dict) -> np.ndarray:
        input_list = list(input_signals.values())
        if input_list:
            return np.concatenate(input_list)
        else:
            return np.array([])  # Return empty array if sys has no inputs

    ######################################################################
    def get_block_html(self, label="sys1"):

        n_ports_out = len(self.outputs)
        n_ports_in = len(self.inputs)

        label = (
            f'<TABLE BORDER="0" CELLSPACING="0">\n'
            f"<TR>\n"
            f'<TD align="left" BORDER="1" COLSPAN="2">{self.name}::{label}</TD>\n'
            f"</TR>\n"
        )

        for j in range(np.max((n_ports_out, n_ports_in))):
            label += f"<TR>\n"

            if j < n_ports_in and j < n_ports_out:
                port_id = list(self.inputs.keys())[j]
                label += (
                    f'<TD PORT="{port_id}" align="left" BORDER="1">{port_id}</TD>\n'
                )
                port_id = list(self.outputs.keys())[j]
                label += f'<TD PORT="{port_id}" BORDER="1">{port_id}</TD>\n'

            elif j < n_ports_in:
                port_id = list(self.inputs.keys())[j]
                label += (
                    f'<TD PORT="{port_id}" align="left" BORDER="1">{port_id}</TD>\n'
                )
                label += f'<TD BORDER="1"> </TD>\n'

            elif j < n_ports_out:
                port_id = list(self.outputs.keys())[j]
                label += f'<TD BORDER="1"> </TD>\n'
                label += f'<TD PORT="{port_id}" BORDER="1">{port_id}</TD>\n'

            label += f"</TR>\n"
        label += f"</TABLE>"

        return label
    
    # ######################################################################
    # def _repr_html_(self):
    #     return self.get_block_html()

    ######################################################################
    def print_html(self):

        try:
            import IPython.display as display
        except ImportError:
            print("IPython is not available")
            return

        display.display(display.HTML(self.get_block_html()))

    ######################################################################
    def get_graphe(self):

        try:
            import graphviz
        except ImportError:
            print("graphviz is not available")
            return None

        g = graphviz.Digraph(self.name, engine="dot")
        g.attr(rankdir="LR")

        g.node(
            self.name,
            shape="none",
            label=f"<{self.get_block_html()}>",
        )

        return g
    
    ######################################################################
    def _repr_svg_(self):
        """ Display the svg rendered graphe in the notebook """
        g = self.get_graphe()
        return g._repr_image_svg_xml()
    
    ######################################################################
    def plot_graphe(self, filename=None):

        g = self.get_graphe()

        plot_graphviz(g, filename=filename)


######################################################################
class StaticSystem(System):

    def __init__(self, m, p):

        System.__init__(self, 0, m, p)
        self.name = "StaticSystem"

        self.inputs = {
            "u": InputPort("u", self.m),
            # "r": InputPort("r", self.m),
        }
        self.recompute_input_properties()

        self.outputs = {
            "y": OutputPort("y", self.p, function=self.h),
        }

    ###################################################################
    def f(self, x, u, t=0, params=None):
        raise Exception("Static system has no states")


######################################################################
class Source(System):

    def __init__(self, p):

        System.__init__(self, 0, 0, p)

        self.name = "Source"
        self.params = {"value": np.zeros(p)}

        self.inputs = {}
        self.outputs = {}
        self.add_output_port("y", self.p, function=self.h, dependencies='all')

    ###################################################################
    def f(self, x, u, t=0, params=None):
        raise Exception("Source system has no states")

    ###################################################################
    def h(self, x, u, t=0, params=None):

        #
        if params is None:
            params = self.params

        y = params["value"]
        return y


######################################################################
class Step(Source):

    def __init__(
        self, initial_value=np.zeros(1), final_value=np.zeros(1), step_time=1.0
    ):

        p = initial_value.shape[0]
        Source.__init__(self, p)

        self.name = "Step"
        self.params = {
            "initial_value": initial_value,
            "final_value": final_value,
            "step_time": step_time,
        }

    ###################################################################
    def h(self, x, u, t=0, params=None):

        #
        if params is None:
            params = self.params

        if t < params["step_time"]:
            y = params["initial_value"]
        else:
            y = params["final_value"]

        return y
    

######################################################################
class WhiteNoise(Source):
    
    def __init__(self, p=1):

        Source.__init__(self, p)

        self.random_generator = np.random.default_rng()

        self.name = "WhiteNoise"
        self.params = {
            "var": 1.0,
            "mean": 0.0,
            "seed": 0,
            }

    ###################################################################
    def h(self, x, u, t=0, params=None):

        #
        if params is None:
            params = self.params
        
        seed = params["seed"] + int(t*1000000000)
        mu = params["mean"]
        sigma = np.sqrt(params["var"])

        random_generator = np.random.default_rng( seed )
        y = random_generator.normal(mu, sigma, self.p)

        return y


######################################################################
class DynamicSystem(System):

    # dx = f(x, u, t)
    # y  = g(x, u, t)

    def __init__(self, n, m, p):

        System.__init__(self, n, m, p)

        self.name = "DynamicSystem"

        self.inputs = {
            "u": InputPort("u", self.m),
            # "r": InputPort("r", self.m),
        }
        self.recompute_input_properties()

        self.outputs = {
            "y": OutputPort("y", self.p, function=self.h),
            "x": OutputPort("x", self.n, function=self.compute_state),
        }


######################################################################
class GrapheSystem(System):

    def __init__(self):

        self.subsystems = {}
        self.edges = {}

        System.__init__(self, 0, 0, 0)

        self.name = "Diagram"

        self.inputs = {}
        self.outputs = {}

        self.recompute_input_properties()

    ######################################################################
    def add_system(self, sys, sys_id):
        self.subsystems[sys_id] = sys
        self.edges[sys_id] = {}
        for port_id, port in sys.inputs.items():
            self.edges[sys_id][port_id] = None

        self.compute_properties()

    ######################################################################
    def compute_properties(self):

        self.n_sys = len(self.subsystems)

        # Compute total number of states
        self.n = 0
        self.state_label = []
        self.state_units = []
        self.state_upper_bound = np.array([])
        self.state_lower_bound = np.array([])
        self.xbar = np.array([])
        self.x0 = np.array([])
        self.state_index = {}

        idx = 0
        for i, (key, sys) in enumerate(self.subsystems.items()):

            self.state_index[key] = (idx , idx + sys.n)

            # Update state properties
            self.n += sys.n
            self.state_label += sys.state_label
            self.state_units += sys.state_units
            self.state_upper_bound = np.concatenate(
                [self.state_upper_bound, sys.state_upper_bound]
            )
            self.state_lower_bound = np.concatenate(
                [self.state_lower_bound, sys.state_lower_bound]
            )
            self.xbar = np.concatenate([self.xbar, sys.xbar])
            self.x0 = np.concatenate([self.x0, sys.x0])

            idx += sys.n

    ######################################################################
    def add_edge(self, source_sys_id, source_port_id, target_sys_id, target_port_id):
        self.edges[target_sys_id][target_port_id] = (source_sys_id, source_port_id)

        print(
            "Added edge from "
            + source_sys_id
            + ":"
            + source_port_id
            + " to "
            + target_sys_id
            + ":"
            + target_port_id
        )

    ######################################################################
    def get_graphe(self):

        try:
            import graphviz
        except ImportError:
            print("graphviz is not available")
            return None

        g = graphviz.Digraph(self.name, engine="dot")
        g.attr(rankdir="LR")

        for i, (sys_id, sys) in enumerate(self.subsystems.items()):

            label = f"<{sys.get_block_html(sys_id)}>"

            g.node(
                sys_id,
                shape="none",
                label=label,
            )

        for sys_id, sys in self.subsystems.items():
            for port_id in sys.inputs:

                edge = self.edges[sys_id][port_id]

                if edge is not None:
                    g.edge(
                        edge[0] + ":" + edge[1] + ":e",
                        sys_id + ":" + port_id + ":w",
                    )

        return g

    ######################################################################
    def get_local_state(self, x, sys_id):
        idx = self.state_index[sys_id]
        return x[idx[0] : idx[1]]
    
    ######################################################################
    def get_local_input(self, x, u , t , sys_id, requested_input_ports = 'all'):

        sys = self.subsystems[sys_id]

        u = np.array([])

        for j, (port_id, port) in enumerate(sys.inputs.items()):

            edge = self.edges[sys_id][port_id]

            if requested_input_ports != 'all' and port_id not in requested_input_ports:
                port_u = port.get_signal(t) 

            elif edge is None:

                # Default unconnected port signal
                port_u = port.get_signal(t) 
            
            else:

                source_sys_id, source_port_id = edge
                source_sys = self.subsystems[source_sys_id]
                source_port = source_sys.outputs[source_port_id]

                req = source_sys.input_output_dependencies[source_port_id]

                # Collect signals needed to compute the source output
                source_x = self.get_local_state(x, source_sys_id)
                source_u = self.get_local_input(x, u, t, source_sys_id, req) # Recursive call, TODO check for algebraic loops
                port_u = source_port.compute(source_x, source_u, t)

            
            u = np.concatenate([u, port_u])
        
        return u
        
    
    ######################################################################
    def f(self, x, u, t=0, params=None) -> np.ndarray:

        dx = np.zeros(self.n)

        idx = 0

        for i, (sys_id, sys) in enumerate(self.subsystems.items()):

            if sys.n > 0:

                # Get input signals
                sys_u = self.get_local_input(x, u, t, sys_id)

                # Compute state derivative
                sys_x  = self.get_local_state(x, sys_id)
                sys_dx = sys.f(sys_x, sys_u, t) # Local state derivative
                dx[idx : idx + sys.n] = sys_dx
                idx += sys.n
        
        return dx


######################################################################
if __name__ == "__main__":

    sys = DynamicSystem(2, 1, 1)

    
