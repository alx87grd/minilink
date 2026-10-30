"""Nested wrappers around dense / deep chain diagram fixtures."""

from __future__ import annotations

import numpy as np

from benchmarks.systems.network import (
    MultiInputNode,
    SimpleGain,
    build_deep_network,
    make_dense_network,
)
from minilink.core.diagram import DiagramSystem


def _expose_last_node_output(
    diag: DiagramSystem, num_nodes: int, *, port_id: str = "y"
) -> None:
    last = num_nodes - 1
    diag.connect_new_output_port(f"Node{last}", "x", port_id)


def _expose_last_integrator_output(
    diag: DiagramSystem, depth: int, *, port_id: str = "y"
) -> None:
    last = depth - 1
    diag.connect_new_output_port(f"Int{last}", "x", port_id)


def make_wrapped_dense_network(
    num_nodes: int = 50,
    connections_per_node: int = 5,
    *,
    seed: int = 42,
    post_gain: float = 1.0,
) -> DiagramSystem:
    """One dense subnet nested under a post gain (boundary ``net:y`` → ``post:u``)."""
    net = make_dense_network(
        num_nodes=num_nodes,
        connections_per_node=connections_per_node,
        seed=seed,
    )
    _expose_last_node_output(net, num_nodes)

    outer = DiagramSystem()
    outer.name = f"WrappedDense(n={num_nodes},c={connections_per_node})"
    outer.add_subsystem(net, "net")
    outer.add_subsystem(SimpleGain("post", gain=post_gain), "post")
    outer.connect("net", "y", "post", "u")
    outer.connect_new_output_port("post", "y", "out")
    return outer


def make_twin_dense_network(
    num_nodes_each: int = 25,
    connections_per_node: int = 5,
    *,
    seed_a: int = 42,
    seed_b: int = 1042,
) -> DiagramSystem:
    """Two dense subnets nested, summed, then post gain."""
    net_a = make_dense_network(
        num_nodes=num_nodes_each,
        connections_per_node=connections_per_node,
        seed=seed_a,
    )
    _expose_last_node_output(net_a, num_nodes_each, port_id="y")

    net_b = make_dense_network(
        num_nodes=num_nodes_each,
        connections_per_node=connections_per_node,
        seed=seed_b,
    )
    _expose_last_node_output(net_b, num_nodes_each, port_id="y")

    outer = DiagramSystem()
    outer.name = f"TwinDense(n={num_nodes_each},c={connections_per_node})"
    outer.add_subsystem(net_a, "net_a")
    outer.add_subsystem(net_b, "net_b")
    outer.add_subsystem(MultiInputNode("sum", 2), "sum")
    outer.add_subsystem(SimpleGain("post", gain=0.5), "post")
    outer.connect("net_a", "y", "sum", "u0")
    outer.connect("net_b", "y", "sum", "u1")
    outer.connect("sum", "x", "post", "u")
    outer.connect_new_output_port("post", "y", "out")
    return outer


def make_wrapped_deep_chain(
    depth: int = 50,
    *,
    post_gain: float = 1.0,
    initial_state0: float = 1.0,
) -> DiagramSystem:
    """Long integrator/gain chain nested under a post gain."""
    chain = build_deep_network(depth=depth, initial_state0=initial_state0)
    _expose_last_integrator_output(chain, depth)

    outer = DiagramSystem()
    outer.name = f"WrappedDeepChain(depth={depth})"
    outer.add_subsystem(chain, "chain")
    outer.add_subsystem(SimpleGain("post", gain=post_gain), "post")
    outer.connect("chain", "y", "post", "u")
    outer.connect_new_output_port("post", "y", "out")
    return outer


def scenario_state(diagram: DiagramSystem) -> tuple[np.ndarray, np.ndarray]:
    """Operating point ``(x, u)`` for wrapped diagrams (no boundary inputs)."""
    x = np.asarray(diagram.x0, dtype=float).copy()
    if x.size:
        x[0] = max(abs(x[0]), 0.5) * np.sign(x[0] or 1.0)
    return x, np.zeros(diagram.m)
