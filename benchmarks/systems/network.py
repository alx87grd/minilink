"""Network-style synthetic systems for benchmark workloads."""

import numpy as np

from minilink.blocks.sources import Source
from minilink.core.diagram import DiagramSystem
from minilink.core.system import System


class SimpleGain(System):
    def __init__(self, id_str, gain=2.0):
        super().__init__(0)
        self.name = id_str
        self.gain = gain
        self.add_input_port("u")
        self.add_output_port("y", function=self.h, dependencies=("u",))

    def h(self, x, u, t=0, params=None):
        return u * self.gain


class SimpleIntegrator(System):
    def __init__(self, id_str):
        super().__init__(1)
        self.name = id_str
        self.add_input_port("u")
        self.add_output_port("x", function=self.compute_state, dependencies=())

    def compute_state(self, x, u, t=0, params=None):
        return x

    def f(self, x, u, t=0, params=None):
        return u


class MultiInputNode(System):
    def __init__(self, id_str, in_ports):
        super().__init__(1)
        self.name = id_str
        self.in_ports = in_ports
        for p_idx in range(in_ports):
            self.add_input_port(f"u{p_idx}")
        self.add_output_port("x", function=self.compute_state, dependencies="all")

    def compute_state(self, x, u, t=0, params=None):
        return x

    def f(self, x, u, t=0, params=None):
        return u.sum()


def build_deep_network(depth=50, initial_state0=1.0):
    """Build a long chain with non-zero dynamics by default."""
    diag = DiagramSystem()
    diag.connection_verbose = False
    for i in range(depth):
        node = SimpleIntegrator(f"Int{i}")
        if i == 0:
            node.x0[0] = initial_state0
        diag.add_subsystem(node, f"Int{i}")
        diag.add_subsystem(SimpleGain(f"Gain{i}"), f"Gain{i}")
        diag.connect(f"Int{i}", "x", f"Gain{i}", "u")
        if i > 0:
            diag.connect(f"Gain{i - 1}", "y", f"Int{i}", "u")
    return diag


def make_dense_network(
    num_nodes=50,
    connections_per_node=5,
    seed=42,
    source_value=1.0,
    *,
    initial_node0_state=1.0,
):
    """Build a dense feed-forward network with constant excitation.

    Topology is reproducible for a given ``seed``: wiring uses a local
    ``numpy.random.RandomState(seed)`` so the graph matches the historical
    ``np.random.seed(seed); np.random.choice(...)`` sequence without touching
    global RNG state.

    ``initial_node0_state`` defaults to a nonzero constant so the trajectory is
    not all zeros even if ``source_value`` is 0.
    """
    diag = DiagramSystem()
    diag.connection_verbose = False
    rng = np.random.RandomState(seed)

    for i in range(num_nodes):
        node = SimpleIntegrator(f"Node{i}")
        if i == 0:
            node.x0[0] = initial_node0_state
        diag.add_subsystem(node, f"Node{i}")

    for i in range(1, num_nodes):
        num_conn = min(i, connections_per_node)
        sources = rng.choice(np.arange(i), size=num_conn, replace=False)

        sys_id = f"MultiNode{i}"
        diag.add_subsystem(MultiInputNode(sys_id, num_conn), sys_id)

        for p_idx, src_i in enumerate(sources):
            diag.connect(f"Node{int(src_i)}", "x", sys_id, f"u{p_idx}")

        diag.connect(sys_id, "x", f"Node{i}", "u")

    source = Source(1)
    source.name = "SourceNode"
    source.params["value"] = np.array([source_value], dtype=float)
    diag.add_subsystem(source, "SourceNode")
    diag.connect("SourceNode", "y", "Node0", "u")
    diag.name = "DenseNetwork"
    return diag


if __name__ == "__main__":
    diag = build_deep_network(depth=50)
    diag = make_dense_network(num_nodes=50, connections_per_node=5)
    diag.compute_trajectory(dt=0.01)
