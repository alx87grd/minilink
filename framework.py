import numpy as np
import graphviz


######################################################################
class Port:

    def __init__(self, name="port", n=1):
        self.name = name
        self.n = n
        self.default_value = np.zeros(self.n)
        self.label = [f"{name}_{i}" for i in range(self.n)]
        self.units = [""] * self.n
        self.upper_bound = np.inf * np.ones(self.n)
        self.lower_bound = -np.inf * np.ones(self.n)
        

######################################################################
class InputPort(Port):

    def get(self, t=0) -> np.ndarray:
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

    def __init__(self, n=0, m=1, p=1, name="System", params={}):

        self.n = n
        self.m = m
        self.p = p

        self.name = name
        self.latex = '$\dot{x} = f(x, u, t)$\n$y = h(x, u, t)$'

        self.inputs = {
            "u": InputPort("u", self.m),
            # "r": InputPort("r", self.m),
            }
        self.outputs = {
            "y": OutputPort("y", self.p, function=self.h),
            # "x": OutputPort("x", self.n, function=self.hx),
            # "q": OutputPort("q", self.n, function=self.hq),
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
    def fsim(self, x, t=0):
        inputs = self.collect_inputs(t)
        u = self.inputs2u(inputs)
        dx = self.f(x, u, t)
        return dx

    ######################################################################
    def add_input_port(self, key, n=1):
        self.inputs[key] = InputPort(key, n)
        self.compute_input_dimensions()

    ######################################################################
    def compute_input_dimensions(self):
        self.m = 0
        for (key, port) in self.inputs.items():
            self.m += port.n

    ######################################################################
    def fsim(self, x, t=0):
        u = self.collect_inputs(t)
        dx = self.f(x, u, t)
        return dx

    ######################################################################
    def collect_inputs(self):
        input_dict = {}
        for (key, port) in self.inputs.items():
            input_dict[key] = port.get()
        return input_dict
    
    ######################################################################
    def inputs2u(self, input_dict):
        return np.concatenate(list(input_dict.values()))

    ######################################################################
    def u2inputs(self, u):
        input_dict = {}
        idx = 0
        for key, port in self.inputs.items():
            input_dict[key] = u[idx:idx + port.n]
            idx += port.n
        return input_dict
    

    
    ######################################################################
    def print_html(self):

        try:
            import IPython.display as display
        except:
            print("IPython is not available")
            return

        display.display(display.HTML(self.get_block_html()))


    ######################################################################
    def get_block_html(self, label='sys1'):

        n_ports_out = len(self.outputs)
        n_ports_in = len(self.inputs)

        label = (
            F'<TABLE BORDER="0" CELLSPACING="0">\n'
            F'<TR>\n'
            F'<TD align="left" BORDER="1" COLSPAN="2">{self.name}::{label}</TD>\n'
            F'</TR>\n'
        )

        for j in range(np.max((n_ports_out, n_ports_in))):
            label += F'<TR>\n'

            if j < n_ports_in and j < n_ports_out:
                port_id = list(self.inputs.keys())[j]
                label += F'<TD PORT="{port_id}" align="left" BORDER="1">{port_id}</TD>\n'
                port_id = list(self.outputs.keys())[j]
                label += F'<TD PORT="{port_id}" BORDER="1">{port_id}</TD>\n'

            elif j < n_ports_in:
                port_id = list(self.inputs.keys())[j]
                label += F'<TD PORT="{port_id}" align="left" BORDER="1">{port_id}</TD>\n'
                label += F'<TD BORDER="1"> </TD>\n'

            elif j < n_ports_out:
                port_id = list(self.outputs.keys())[j]
                label += F'<TD BORDER="1"> </TD>\n'
                label += F'<TD PORT="{port_id}" BORDER="1">{port_id}</TD>\n'
            
            label += F'</TR>\n'
        label += F'</TABLE>'

        return label


######################################################################
class StaticSystem(System):

    def __init__(self, m, p):

        System.__init__(self, 0, m, p)
        self.name = "StaticSystem"

    def f(self, x, u, t):
        raise Exception("Static system has no state")


######################################################################
class DynamicSystem(System):

    # dx = f(x, u, t)
    # y  = g(x, u, t)

    def __init__(self, n, m, p, name="DynamicSystem", params={}):

        System.__init__(self, n, m, p, name, params)


######################################################################
class GrapheSystem(System):

    def __init__(self):

        self.subsystems = {}
        self.edges = {}

        System.__init__(self, 0, 0, 0)

        self.name = 'Diagram'

        self.inputs = {
            "u": InputPort("u", self.m),
            # "r": InputPort("r", self.m),
            }
        self.outputs = {
            "y": OutputPort("y", self.p, function=self.h),
            # "x": OutputPort("x", self.n, function=self.hx),
            # "q": OutputPort("q", self.n, function=self.hq),
        }

    def add_system(self, sys, sys_id):
        self.subsystems[sys_id] = sys
        self.edges[sys_id] = {}
        for j, (port_id, port) in enumerate(sys.inputs.items()):
            self.edges[sys_id][port_id] = -1
        # each susbsystem input ports connection are stored in a row:
        # [source_subsys_id, source_subsys_port_id]
        # -1 means no connection

    def compute_properties(self):

        self.n_sys = len(self.subsystems)

        # Compute total number of states
        self.n = 0
        for i, (k, sys) in enumerate(self.subsystems.items()):
            self.n += sys.n
        # TODO : get label, units and bounds for each state

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

    def render_graphe(self):
        g = graphviz.Digraph("G", filename="testgraphe.gv", engine="dot")
        g.attr(rankdir="LR")
        g.attr(concentrate="true")

        for i, (sys_id, sys) in enumerate(self.subsystems.items()):

            print(str(i))

            label = F'<{sys.get_block_html(sys_id)}>'

            g.node(
                sys_id,
                shape="none",
                label=label,
            )

        for i, (sys_id, sys) in enumerate(self.subsystems.items()):
            for j, (port_id, port) in enumerate(sys.inputs.items()):

                edge = self.edges[sys_id][port_id]

                if not edge == -1:
                    g.edge(
                        edge[0] + ":" + edge[1] + ":e",
                        sys_id + ":" + port_id + ":w",
                    )

        g.view()


######################################################################
if __name__ == "__main__":

    sys1 = DynamicSystem(2, 1, 1)
    sys2 = DynamicSystem(2, 1, 1)
    sys3 = StaticSystem(1, 1)
    sys4 = DynamicSystem(2, 1, 1)

    sys1.print_html()
    sys1.add_input_port("w", 2)
    sys1.print_html()
    sys1.add_input_port("v", 1)
    sys1.print_html()


    sys1.inputs["u"].default_value = np.array([7.7])
    sys1.inputs["w"].default_value = np.array([1.1, 2.2])

    inputs = sys1.collect_inputs()
    u = sys1.inputs2u( inputs )
    print(inputs)
    print(u)
    print(sys1.u2inputs(u))

    # gsys = GrapheSystem()
    # gsys.print_html()
    # gsys.add_system(sys1, "sys1")
    # gsys.add_system(sys2, "sys2")
    # gsys.add_system(sys3, "sys3")
    # gsys.add_system(sys4, "sys4")

    # # gsys.render_graphe()

    # gsys.add_edge("sys1", "y", "sys2", "u")
    # gsys.add_edge("sys2", "y", "sys3", "u")
    # gsys.add_edge("sys2", "y", "sys4", "u")
    # gsys.add_edge("sys4", "y", "sys1", "u")


    # gsys.render_graphe()

    # print("Done.")
