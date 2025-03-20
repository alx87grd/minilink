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

        self.input_ports = [InputPort("u", self.m)]
        self.output_ports = [OutputPort("y", self.p)]  # Change to dict??

        # Default output
        self.output_ports[0].compute_output = self.h

        self.name = "System"

    def h(self, x, u, t):
        y = np.zeros(self.p)

    def get_default_inputs(self, t=0):
        for j, port in enumerate(self.input_ports):
            uj = port.get(t)
            if j == 0:
                u = uj
            else:
                u = np.hstack((u, uj))
        return u

    # for (i,(k,port)) in enumerate(ports.items()):
    #     print(i,k,port)


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

        self.subsystems = []
        self.edges = []

    def add_system(self, sys):
        self.subsystems.append(sys)
        self.edges.append(-1 * np.ones((len(sys.input_ports), 2)))
        # each susbsystem input ports connection are stored in a row:
        # [source_subsys_id, source_subsys_port_id]
        # -1 means no connection

    def compute_properties(self):

        self.n_sys = len(self.subsystems)

        # Compute total number of states
        self.n = 0
        for sys in self.subsystems:
            self.n += sys.n
        # TODO : get label, units and bounds for each state

    def add_edge(self, source_sys_id, source_port_id, target_sys_id, target_port_id):
        self.edges[target_sys_id][target_port_id, 0] = source_sys_id
        self.edges[target_sys_id][target_port_id, 1] = source_port_id

        print(
            "Added edge from "
            + self.subsystems[source_sys_id].name
            + " port "
            + self.subsystems[source_sys_id].output_ports[source_port_id].name
            + " to "
            + self.subsystems[target_sys_id].name
            + " port "
            + self.subsystems[target_sys_id].input_ports[target_port_id].name,
        )

    def render_graphe(self):
        g = graphviz.Digraph("G", filename="testgraphe.gv", engine="dot")
        g.attr(rankdir="LR")
        g.attr(concentrate="true")

        for i, sys in enumerate(self.subsystems):

            print(str(i))

            g.node(
                "sys" + str(i),
                shape="none",
                label="""<
                    <TABLE BORDER="0" CELLSPACING="0">
                    <TR>
                    <TD align="left" BORDER="1" COLSPAN="2">
                    """
                + sys.name
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

        for i, sys in enumerate(self.subsystems):
            for j, port in enumerate(sys.input_ports):
                if self.edges[i][j, 0] >= 0:
                    g.edge(
                        "sys" + str(int(self.edges[i][j, 0])) + ":y:e",
                        "sys" + str(i) + ":u:w",
                    )

        g.view()


######################################################################
if __name__ == "__main__":

    sys1 = DynamicSystem(2, 1, 1)
    sys2 = DynamicSystem(2, 1, 1)
    sys3 = StaticSystem(1, 1)
    sys4 = DynamicSystem(2, 1, 1)

    gsys = GrapheSystem()
    gsys.add_system(sys1)
    gsys.add_system(sys2)
    gsys.add_system(sys3)
    gsys.add_system(sys4)

    gsys.add_edge(0, 0, 2, 0)
    gsys.add_edge(2, 0, 1, 0)
    gsys.add_edge(1, 0, 0, 0)

    gsys.add_edge(2, 0, 3, 0)

    gsys.render_graphe()

    print("Done.")
