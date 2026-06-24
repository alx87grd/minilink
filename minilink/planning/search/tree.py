"""
The search tree: nodes plus nearest-neighbour queries and path extraction.

Nodes hold the state, the parent, the :class:`~minilink.planning.search.edge.Edge`
reaching them, and the cost-to-come. Nearest/near queries are brute force over a
caller-supplied metric (a KD-tree backend is a future optimisation). Path
extraction concatenates the edges along a parent chain into a single
:class:`~minilink.core.trajectory.Trajectory`.
"""

from collections.abc import Callable
from dataclasses import dataclass

import numpy as np

from minilink.core.trajectory import Trajectory
from minilink.planning.search.edge import Edge


@dataclass
class Node:
    """A tree node: a state, its parent, the reaching edge, and cost-to-come."""

    x: np.ndarray
    parent: "Node | None"
    edge: Edge | None
    cost: float


class Tree:
    """A search tree rooted at a start state."""

    def __init__(self, root: Node) -> None:
        self.root = root
        self.nodes: list[Node] = [root]

    def add(self, node: Node) -> Node:
        """Append a node and return it."""
        self.nodes.append(node)
        return node

    def nearest(self, x, metric: Callable) -> Node:
        """Return the node minimising ``metric(node.x, x)``."""
        return min(self.nodes, key=lambda node: metric(node.x, x))

    def near(self, x, radius: float, metric: Callable) -> list[Node]:
        """Return all nodes within ``radius`` of ``x`` (for RRT*)."""
        return [node for node in self.nodes if metric(node.x, x) <= radius]

    def extract_trajectory(self, node: Node) -> Trajectory:
        """Concatenate edges from the root to ``node`` into a `Trajectory`."""
        chain = []
        current = node
        while current.parent is not None:
            chain.append(current)
            current = current.parent
        chain.reverse()

        states = [np.asarray(self.root.x, dtype=float)]
        inputs: list[np.ndarray] = []
        times = [0.0]
        t0 = 0.0
        for nd in chain:
            edge = nd.edge
            for k in range(1, len(edge.states)):
                states.append(np.asarray(edge.states[k], dtype=float))
                times.append(t0 + float(edge.times[k]))
            inputs.extend(np.asarray(u, dtype=float) for u in edge.inputs)
            t0 += float(edge.times[-1])

        # control is piecewise-constant: hold the last sample at the final knot
        inputs.append(inputs[-1] if inputs else np.zeros(0))
        return Trajectory(
            t=np.asarray(times),
            x=np.asarray(states).T,
            u=np.asarray(inputs).T,
        )
