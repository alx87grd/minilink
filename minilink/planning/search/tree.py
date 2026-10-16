"""The search tree: nodes, nearest-neighbour queries and path extraction."""

from collections.abc import Callable
from dataclasses import dataclass, field

import numpy as np

from minilink.core.trajectory import Trajectory
from minilink.planning.search.edge import Edge

NEAREST_BRUTE_FORCE = "brute_force"
NEAREST_KD_TREE = "kd_tree"
NEAREST_BACKENDS = (NEAREST_BRUTE_FORCE, NEAREST_KD_TREE)


###############################################################################
# Public API
###############################################################################


@dataclass
class Node:
    """A tree node: a state, its parent, the reaching edge, and cost-to-come."""

    x: np.ndarray
    parent: "Node | None"
    edge: Edge | None
    cost: float
    children: list = field(default_factory=list, repr=False)


class Tree:
    """
    A search tree rooted at a start state.

    Nearest and near queries run by brute force over a caller-supplied metric,
    or on a SciPy :class:`~scipy.spatial.cKDTree` index (Euclidean L2 only).
    Path extraction concatenates the edges along a parent chain into a single
    :class:`~minilink.core.trajectory.Trajectory`.
    """

    def __init__(
        self, root: Node, *, nearest_backend: str = NEAREST_BRUTE_FORCE
    ) -> None:
        require_nearest_backend(nearest_backend)

        # The root, and every node in insertion order
        self.root = root
        self.nodes: list[Node] = [root]

        # The nearest-neighbour index, rebuilt at the first query after a change
        self.nearest_backend = nearest_backend
        self._kdtree = None
        self._kdtree_dirty = nearest_backend == NEAREST_KD_TREE

    def add(self, node: Node) -> Node:
        """Append a node and return it."""
        self.nodes.append(node)
        if node.parent is not None:
            node.parent.children.append(node)

        # The index no longer covers every node
        if self.nearest_backend == NEAREST_KD_TREE:
            self._kdtree_dirty = True

        return node

    def nearest(self, x, metric: Callable) -> Node:
        """Return the node minimising ``metric(node.x, x)``."""
        # Brute force: every node, under the caller's metric
        if self.nearest_backend == NEAREST_BRUTE_FORCE:
            return min(self.nodes, key=lambda node: metric(node.x, x))

        # The k-d tree: the Euclidean nearest neighbour
        self.ensure_kdtree()
        _, idx = self._kdtree.query(np.asarray(x, dtype=float))

        return self.nodes[int(idx)]

    def near(self, x, radius: float, metric: Callable) -> list[Node]:
        """Return all nodes within ``radius`` of ``x`` (for RRT*)."""
        # Brute force: every node, under the caller's metric
        if self.nearest_backend == NEAREST_BRUTE_FORCE:
            return [node for node in self.nodes if metric(node.x, x) <= radius]

        # The k-d tree: the Euclidean ball, a scalar answer read as a one-index list
        self.ensure_kdtree()
        indices = self._kdtree.query_ball_point(
            np.asarray(x, dtype=float), float(radius)
        )
        if np.isscalar(indices):
            indices = [int(indices)]

        return [self.nodes[int(idx)] for idx in indices]

    def rewire(self, node: Node, new_parent: Node, new_edge: Edge) -> None:
        """Reparent ``node`` through ``new_edge`` and refresh its cost-to-come."""
        # Detach the node from its old parent
        if node.parent is not None:
            node.parent.children = [
                child for child in node.parent.children if child is not node
            ]

        # Attach it under the new parent: c(node) = c(parent) + c(edge)
        node.parent = new_parent
        node.edge = new_edge
        node.cost = new_parent.cost + new_edge.cost
        new_parent.children.append(node)

    def propagate_cost(self, node: Node) -> None:
        """Refresh cost-to-come for ``node`` and all descendants."""
        # c(child) = c(node) + c(edge), down the whole subtree
        for child in node.children:
            child.cost = node.cost + child.edge.cost
            self.propagate_cost(child)

    def extract_trajectory(self, node: Node) -> Trajectory:
        """Concatenate edges from the root to ``node`` into a `Trajectory`."""
        # The parent chain, from the root's child down to the node
        chain = []
        current = node
        while current.parent is not None:
            chain.append(current)
            current = current.parent
        chain.reverse()

        # Each edge's knots after its first (the parent's state), on one time axis
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

        # The control is piecewise constant: hold the last sample at the final knot
        inputs.append(inputs[-1] if inputs else np.zeros(0))

        t = np.asarray(times)
        x = np.asarray(states).T
        u = np.asarray(inputs).T
        trajectory = Trajectory(t=t, x=x, u=u)

        return trajectory

    ###########################################################################
    # Internal machinery
    ###########################################################################

    def ensure_kdtree(self) -> None:
        """Build the k-d tree index over the node states, unless it is up to date."""
        if not self._kdtree_dirty and self._kdtree is not None:
            return

        # Lazy import: SciPy's k-d tree loads on first use, not with the tree
        from scipy.spatial import cKDTree

        coords = np.vstack([node.x for node in self.nodes])
        self._kdtree = cKDTree(np.asarray(coords, dtype=float))
        self._kdtree_dirty = False


###############################################################################
# Internal machinery
###############################################################################


def require_nearest_backend(nearest_backend):
    """Refuse a nearest-neighbour backend outside ``NEAREST_BACKENDS``."""
    if nearest_backend not in NEAREST_BACKENDS:
        raise ValueError(
            f"nearest_backend must be one of {NEAREST_BACKENDS}, got {nearest_backend!r}"
        )
