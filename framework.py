import numpy as np


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
        self.label = [f"{name}_{i}" for i in range(self.n)]
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

    def __init__(self, n=0, m=1, p=1, name="System", params=None):

        # Dimensions
        self.n = n
        self.m = m
        self.p = p

        # Name
        self.name = name

        # Parameters dictionary
        if params is None:
            self.params = {}
        else:
            self.params = params

        # Caracteristics useful to select automatic solver parameters
        self.solver_info = {
            "continuous_time_equation": True,
            "smallest_time_constant": 0.001,
            "largest_time_constant": 10,
            "discontinuous_behavior": False,  # Will use a fixed time step
            "discrete_time_period": None,
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
    def compute_state(self, x, u, t=0, params=None):
        return x

    ######################################################################
    def add_input_port(self, key, dim=1, default_value=None):
        self.inputs[key] = InputPort(key, dim, default_value)
        self.recompute_input_dimensions()

    ######################################################################
    def recompute_input_dimensions(self):
        self.m = 0
        for key, port in self.inputs.items():
            self.m += port.n

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

    ######################################################################
    def print_html(self):

        try:
            import IPython.display as display
        except ImportError:
            print("IPython is not available")
            return

        display.display(display.HTML(self.get_block_html()))

    ######################################################################
    def show_diagram(self):
        try:
            import graphviz
        except ImportError:
            print("graphviz is not available")
            return None

        g = graphviz.Digraph("G", filename="temp.gv", engine="dot")
        g.attr(rankdir="LR")
        g.attr(concentrate="true")
        g.node(
            self.name,
            shape="none",
            label=f"<{self.get_block_html()}>",
        )
        g.view()


######################################################################
class StaticSystem(System):

    def __init__(self, m, p):

        System.__init__(self, 0, m, p)
        self.name = "StaticSystem"

        self.inputs = {
            "u": InputPort("u", self.m),
            # "r": InputPort("r", self.m),
        }
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
        self.outputs = {
            "y": OutputPort("y", self.p, function=self.h),
        }

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
class DynamicSystem(System):

    # dx = f(x, u, t)
    # y  = g(x, u, t)

    def __init__(self, n, m, p, name="DynamicSystem", params={}):

        System.__init__(self, n, m, p, name, params)

        self.inputs = {
            "u": InputPort("u", self.m),
            # "r": InputPort("r", self.m),
        }

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

    ######################################################################
    def add_system(self, sys, sys_id):
        self.subsystems[sys_id] = sys
        self.edges[sys_id] = {}
        for port_id, port in sys.inputs.items():
            self.edges[sys_id][port_id] = None

    ######################################################################
    def compute_properties(self):

        self.n_sys = len(self.subsystems)

        # Compute total number of states
        self.n = 0
        for i, (k, sys) in enumerate(self.subsystems.items()):
            self.n += sys.n
        # TODO : get label, units and bounds for each state

    ######################################################################
    def add_edge(self, source_sys_id, source_port_id, target_sys_id, target_port_id):
        self.edges[target_sys_id][target_port_id] = (source_sys_id, source_port_id)

        print(
            "Added edge from "
            + source_sys_id
            + " port "
            + source_port_id
            + " to "
            + target_sys_id
            + " port "
            + target_port_id
        )

    ######################################################################
    def render_graphe(self):

        try:
            import graphviz
        except ImportError:
            print("graphviz is not available")
            return None

        g = graphviz.Digraph("G", filename="temp.gv", engine="dot")
        g.attr(rankdir="LR")
        g.attr(concentrate="true")

        for i, (sys_id, sys) in enumerate(self.subsystems.items()):

            print(str(i))

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

        g.view()


######################################################################
if __name__ == "__main__":

    pass
