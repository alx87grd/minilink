import numpy as np
import graphviz


######################################################################
class Port:

    def __init__(self, name="port", n=1):
        self.name = name
        self.n = n
        self.default_value = np.zeros(n)


######################################################################
class InputPort(Port):

    def get(self, t=0):
        return self.default_value


######################################################################
class OutputPort(Port):

    def compute(self, x=None, u=None, t=0):
        return self.default_value


######################################################################
class System:

    def __init__(self, m, p):
        self.n = 0  # default is static system with no states
        self.m = m
        self.p = p

        self.input_ports = {"u": InputPort("u", self.m)}
        self.output_ports = {"y": OutputPort("y", self.p)}

        # Default output
        self.output_ports["y"].compute_output = self.h

        self.name = "System"

    def h(self, x, u, t):
        y = np.zeros(self.p)

    def get_default_inputs(self, t=0):
        for j, (k, port) in enumerate(self.input_ports.items()):
            uj = port.get(t)
            if j == 0:
                u = uj
            else:
                u = np.hstack((u, uj))
        return u


######################################################################
class StaticSystem(System):

    pass


######################################################################
class DynamicSystem(System):

    # dx = f(x, u, t)
    # y  = g(x, u, t)

    def __init__(self, n, m, p):

        System.__init__(self, m, p)
        self.n = n

        self.name = "DynamicSystem"

    def f(self, x, u, t):
        dx = np.zeros(self.n)

    def h(self, x, u, t):
        y = np.zeros(self.p)


######################################################################
class GrapheSystem(System):

    def __init__(self):

        self.subsystems = {}
        self.edges = {}

    def add_system(self, sys, sys_id):
        self.subsystems[sys_id] = sys
        self.edges[sys_id] = {}
        for j, (port_id, port) in enumerate(sys.input_ports.items()):
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

            g.node(
                sys_id,
                shape="none",
                label="""<
                    <TABLE BORDER="0" CELLSPACING="0">
                    <TR>
                    <TD align="left" BORDER="1" COLSPAN="2">
                    """
                + sys.name
                + ": "
                + sys_id
                + """
                    </TD>
                    </TR>
                    <TR>
                    <TD PORT="u" BORDER="1" >u</TD>
                    <TD PORT="y" BORDER="1" >y</TD>
                    </TR>
                    </TABLE>
                    >""",
            )

        for i, (sys_id, sys) in enumerate(self.subsystems.items()):
            for j, (port_id, port) in enumerate(sys.input_ports.items()):

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

    gsys = GrapheSystem()
    gsys.add_system(sys1, "sys1")
    gsys.add_system(sys2, "sys2")
    gsys.add_system(sys3, "sys3")
    gsys.add_system(sys4, "sys4")

    gsys.add_edge("sys1", "y", "sys2", "u")
    gsys.add_edge("sys2", "y", "sys3", "u")
    gsys.add_edge("sys2", "y", "sys4", "u")
    gsys.add_edge("sys4", "y", "sys1", "u")

    gsys.render_graphe()

    print("Done.")
