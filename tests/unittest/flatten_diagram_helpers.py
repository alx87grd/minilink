"""Nested diagram fixtures that expose flatten state-stack reorder bugs.

Twin subnets plus a stateful top-level block (``sum``) mirror the benchmark
topology; smaller chains keep CI fast.
"""

from __future__ import annotations

from benchmarks.scenarios.nested_dense_network import (
    make_twin_dense_network,
    make_wrapped_dense_network,
)
from benchmarks.systems.network import (
    MultiInputNode,
    SimpleGain,
    SimpleIntegrator,
    build_deep_network,
)
from minilink.blocks.routing import Gain
from minilink.control.siso import PID
from minilink.core.diagram import DiagramSystem
from minilink.dynamics.catalog.mass_spring_damper.linear import SingleMass


def twin_dense_15():
    return make_twin_dense_network(num_nodes_each=15, connections_per_node=5)


def wrapped_dense_15():
    return make_wrapped_dense_network(num_nodes=15, connections_per_node=5)


def _small_chain_subnet(
    *, initial_state0: float, depth: int = 3, boundary_drive: bool = False
) -> DiagramSystem:
    """Integrator chain; optional boundary drive on ``Int0:u``."""
    chain = build_deep_network(depth=depth, initial_state0=initial_state0)
    if boundary_drive:
        chain.add_input_port("drive")
        chain.connect("input", "drive", "Int0", "u")
    chain.connect_new_output_port(f"Int{depth - 1}", "x", "y")
    return chain


def minimal_twin_nest_with_sum() -> DiagramSystem:
    """Two nested subnets + stateful ``sum`` (same bug class as twin dense, smaller)."""
    net_a = _small_chain_subnet(initial_state0=0.4)
    net_b = _small_chain_subnet(initial_state0=-0.25)
    outer = DiagramSystem()
    outer.add_subsystem(net_a, "net_a")
    outer.add_subsystem(net_b, "net_b")
    outer.add_subsystem(MultiInputNode("sum", 2), "sum")
    outer.add_subsystem(SimpleGain("post", gain=0.5), "post")
    outer.connect("net_a", "y", "sum", "u0")
    outer.connect("net_b", "y", "sum", "u1")
    outer.connect("sum", "x", "post", "u")
    outer.connect_new_output_port("post", "y", "out")
    return outer


def deep_pid_nest() -> DiagramSystem:
    """Three-level nested PID shell (from compile flatten regression)."""
    pid = PID(Kp=1.0, Ki=0.0, Kd=0.0, ports="reference")
    mass = SingleMass()
    mass.x0[0] = 0.2
    mass.x0[1] = 0.1
    inner = DiagramSystem()
    inner.add_subsystem(pid, "controller")
    inner.add_subsystem(mass, "plant")
    inner.add_input_port("r")
    inner.connect("input", "r", "controller", "r")
    inner.connect("plant", "y", "controller", "y")
    inner.connect("controller", "u", "plant", "u")
    inner.connect_new_output_port("plant", "y", "y")

    mid = DiagramSystem()
    mid.add_subsystem(Gain([1.0]), "gain")
    mid.add_subsystem(inner, "loop")
    mid.connect("loop", "y", "gain", "u")
    mid.connect_new_output_port("gain", "y", "y")
    mid.add_input_port("r")
    mid.connect("input", "r", "loop", "r")

    outer = DiagramSystem()
    outer.add_subsystem(Gain([0.5]), "scale")
    outer.add_subsystem(mid, "box")
    outer.connect("box", "y", "scale", "u")
    outer.connect("scale", "y", "box", "r")
    return outer


def stateful_before_nest() -> DiagramSystem:
    """Top-level integrator, then nested chain, then static post."""
    head = SimpleIntegrator("head")
    head.x0[0] = 0.6
    chain = _small_chain_subnet(initial_state0=0.15, boundary_drive=True)
    outer = DiagramSystem()
    outer.add_subsystem(head, "head")
    outer.add_subsystem(chain, "chain")
    outer.add_subsystem(SimpleGain("post", gain=1.0), "post")
    outer.connect("head", "x", "chain", "drive")
    outer.connect("chain", "y", "post", "u")
    outer.connect_new_output_port("post", "y", "out")
    return outer


def stateful_between_nests() -> DiagramSystem:
    """``net_a``, stateful ``mid``, ``net_b``, summed, then post (insertion order)."""
    net_a = _small_chain_subnet(initial_state0=0.5)
    mid = SimpleIntegrator("mid")
    mid.x0[0] = -0.2
    net_b = _small_chain_subnet(initial_state0=0.35)
    outer = DiagramSystem()
    outer.add_subsystem(net_a, "net_a")
    outer.add_subsystem(mid, "mid")
    outer.add_subsystem(net_b, "net_b")
    outer.add_subsystem(MultiInputNode("sum", 2), "sum")
    outer.add_subsystem(SimpleGain("post", gain=0.25), "post")
    outer.connect("net_a", "y", "sum", "u0")
    outer.connect("net_b", "y", "sum", "u1")
    outer.connect("sum", "x", "post", "u")
    outer.connect_new_output_port("post", "y", "out")
    return outer
