"""
JSON-serializable diagram topology for :class:`~minilink.core.diagram.DiagramSystem`.

Dynamics and port compute functions live in Python objects and are not serialized.
The intended workflow is:

- **Save**: export node ids, a string ``kind`` per node (chosen by the author),
  JSON-safe ``params``, and directed signal edges.
- **Load**: rebuild each subsystem from a small **catalog** mapping ``kind`` to
  a factory, then call :meth:`~minilink.core.diagram.DiagramSystem.connect`.

This module is an MVP boundary: catalogs are caller-supplied; the helpers here
stay free of web or file I/O.

TODO: User Architectural Review — expand catalogs, versioning, and diagram-level
I/O ports once a stable authoring workflow exists.
"""

from __future__ import annotations

import json
from collections.abc import Callable, Mapping, MutableMapping
from dataclasses import asdict, dataclass, field
from typing import Any

import numpy as np

from minilink.core.blocks.basic import Integrator, PropController
from minilink.core.blocks.sources import Step
from minilink.core.diagram import DiagramSystem
from minilink.core.system import System

TOPOLOGY_VERSION = 1


@dataclass(frozen=True)
class NodeSpec:
    """One subsystem instance in a diagram topology."""

    id: str
    kind: str
    params: dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class EdgeSpec:
    """Connect ``source_sys:source_port`` → ``target_sys:target_port``."""

    source_sys: str
    source_port: str
    target_sys: str
    target_port: str


@dataclass
class DiagramTopology:
    """Topology bundle: nodes, edges, optional diagram metadata."""

    nodes: tuple[NodeSpec, ...]
    edges: tuple[EdgeSpec, ...]
    name: str = "Diagram"

    def to_json_dict(self) -> dict[str, Any]:
        return {
            "version": TOPOLOGY_VERSION,
            "name": self.name,
            "nodes": [asdict(n) for n in self.nodes],
            "edges": [asdict(e) for e in self.edges],
        }

    @classmethod
    def from_json_dict(cls, data: Mapping[str, Any]) -> DiagramTopology:
        if int(data.get("version", 0)) != TOPOLOGY_VERSION:
            raise ValueError(
                f"Unsupported topology version {data.get('version')!r}; "
                f"expected {TOPOLOGY_VERSION}."
            )
        nodes = tuple(
            NodeSpec(
                id=str(n["id"]),
                kind=str(n["kind"]),
                params=dict(n.get("params") or {}),
            )
            for n in data["nodes"]
        )
        edges = tuple(
            EdgeSpec(
                source_sys=str(e["source_sys"]),
                source_port=str(e["source_port"]),
                target_sys=str(e["target_sys"]),
                target_port=str(e["target_port"]),
            )
            for e in data["edges"]
        )
        return cls(nodes=nodes, edges=edges, name=str(data.get("name") or "Diagram"))


def topology_dumps(topology: DiagramTopology) -> str:
    """Serialize :class:`DiagramTopology` to a JSON string."""
    return json.dumps(topology.to_json_dict(), indent=2, sort_keys=True)


def topology_loads(s: str) -> DiagramTopology:
    """Parse JSON string into :class:`DiagramTopology`."""
    return DiagramTopology.from_json_dict(json.loads(s))


def mvp_port_signature() -> dict[str, dict[str, tuple[str, ...]]]:
    """
    Input and output port ids for each built-in MVP ``kind`` string.

    Used by authoring tools (for example the diagram editor server) to populate
    port pickers without importing block implementations on the client.
    """
    return {
        "integrator": {"inputs": ("u",), "outputs": ("y",)},
        "prop_controller": {"inputs": ("ref", "y"), "outputs": ("u",)},
        "step": {"inputs": (), "outputs": ("y",)},
    }


def _make_integrator(params: Mapping[str, Any] | None) -> System:
    sys = Integrator()
    if not params:
        return sys
    if "k" in params:
        sys.params["k"] = float(params["k"])
    if "x0" in params:
        x0 = np.asarray(params["x0"], dtype=float).reshape(-1)
        if x0.shape != (sys.n,):
            raise ValueError(
                f"integrator x0 must have shape ({sys.n},), got {x0.shape}"
            )
        sys.x0[:] = x0
    if "state_label" in params and sys.n == 1:
        sys.state.labels[0] = str(params["state_label"])
    return sys


def _make_prop_controller(params: Mapping[str, Any] | None) -> System:
    sys = PropController()
    if params and "Kp" in params:
        sys.params["Kp"] = float(params["Kp"])
    return sys


def _list_to_vector1(value: Any) -> np.ndarray:
    arr = np.asarray(value, dtype=float).reshape(-1)
    if arr.size != 1:
        raise ValueError("expected a length-1 vector for this MVP block")
    return arr


def _make_step(params: Mapping[str, Any] | None) -> System:
    if not params:
        return Step()
    iv = _list_to_vector1(params.get("initial_value", [0.0]))
    fv = _list_to_vector1(params.get("final_value", [0.0]))
    st = float(params.get("step_time", 1.0))
    return Step(initial_value=iv, final_value=fv, step_time=st)


def default_mvp_catalog() -> dict[str, Callable[[Mapping[str, Any] | None], System]]:
    """Built-in factories for the demo blocks used in ``examples/scripts/diagrams``."""
    return {
        "integrator": _make_integrator,
        "prop_controller": _make_prop_controller,
        "step": _make_step,
    }


def build_diagram_from_topology(
    topology: DiagramTopology,
    catalog: Mapping[str, Callable[[Mapping[str, Any] | None], System]],
) -> DiagramSystem:
    """
    Instantiate a :class:`~minilink.core.diagram.DiagramSystem` from topology data.

    Parameters
    ----------
    topology
        Node list and edge list.
    catalog
        Maps each node's ``kind`` to ``factory(params) -> System``.
    """
    ids = [n.id for n in topology.nodes]
    if len(set(ids)) != len(ids):
        raise ValueError("duplicate subsystem id in topology nodes")

    diagram = DiagramSystem()
    diagram.name = topology.name

    for node in topology.nodes:
        if node.kind not in catalog:
            raise KeyError(f"unknown subsystem kind {node.kind!r} for id {node.id!r}")
        subsystem = catalog[node.kind](node.params)
        diagram.add_subsystem(subsystem, node.id)

    for edge in topology.edges:
        diagram.connect(
            edge.source_sys,
            edge.source_port,
            edge.target_sys,
            edge.target_port,
        )

    return diagram


def extract_topology_from_diagram(
    diagram: DiagramSystem,
    kinds: Mapping[str, str],
    params: Mapping[str, Mapping[str, Any]] | None = None,
) -> DiagramTopology:
    """
    Build a :class:`DiagramTopology` from an existing diagram.

    Subsystem Python types are not inspected; the caller supplies ``kind`` and
    optional ``params`` per subsystem id (matching :class:`NodeSpec`).

    Diagram-level exported outputs (connections under ``\"output\"``) are not
    included in this MVP extractor.
    """
    params = params or {}
    nodes: list[NodeSpec] = []
    for sys_id, sys in diagram.subsystems.items():
        if sys_id not in kinds:
            raise KeyError(f"missing kind for subsystem id {sys_id!r}")
        nodes.append(
            NodeSpec(id=sys_id, kind=kinds[sys_id], params=dict(params.get(sys_id, {})))
        )

    edges: list[EdgeSpec] = []
    for target_sys, port_map in diagram.connections.items():
        if target_sys in ("output",):
            continue
        if target_sys not in diagram.subsystems:
            continue
        for target_port, source in port_map.items():
            if source is None:
                continue
            source_sys, source_port = source
            edges.append(
                EdgeSpec(
                    source_sys=source_sys,
                    source_port=source_port,
                    target_sys=target_sys,
                    target_port=target_port,
                )
            )

    edges.sort(key=lambda e: (e.target_sys, e.target_port, e.source_sys, e.source_port))
    nodes.sort(key=lambda n: n.id)
    return DiagramTopology(nodes=tuple(nodes), edges=tuple(edges), name=diagram.name)


def round_trip_topology_dict(
    data: MutableMapping[str, Any],
    catalog: Mapping[str, Callable[[Mapping[str, Any] | None], System]] | None = None,
) -> dict[str, Any]:
    """
    Parse ``data``, build a diagram, and re-export topology JSON dict.

    Useful for normalizing a hand-edited file; ``catalog`` defaults to
    :func:`default_mvp_catalog`.
    """
    catalog = default_mvp_catalog() if catalog is None else catalog
    top = DiagramTopology.from_json_dict(data)
    diagram = build_diagram_from_topology(top, catalog)
    kinds = {n.id: n.kind for n in top.nodes}
    params = {n.id: n.params for n in top.nodes}
    return extract_topology_from_diagram(
        diagram, kinds=kinds, params=params
    ).to_json_dict()
