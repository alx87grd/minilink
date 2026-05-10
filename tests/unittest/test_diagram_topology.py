import numpy as np
import pytest

from minilink.core.diagram_topology import (
    DiagramTopology,
    EdgeSpec,
    NodeSpec,
    build_diagram_from_topology,
    default_mvp_catalog,
    extract_topology_from_diagram,
    round_trip_topology_dict,
    topology_dumps,
    topology_loads,
)


def _demo_topology() -> DiagramTopology:
    return DiagramTopology(
        name="Demo",
        nodes=(
            NodeSpec(
                "step",
                "step",
                {"initial_value": [0.0], "final_value": [20.0], "step_time": 10.0},
            ),
            NodeSpec("c1", "prop_controller", {"Kp": 10.0}),
            NodeSpec("c2", "prop_controller", {"Kp": 10.0}),
            NodeSpec("i1", "integrator", {"x0": [20.0], "state_label": "v"}),
            NodeSpec("i2", "integrator", {"x0": [20.0], "state_label": "x"}),
        ),
        edges=(
            EdgeSpec("i1", "y", "i2", "u"),
            EdgeSpec("c2", "u", "i1", "u"),
            EdgeSpec("i1", "y", "c2", "y"),
            EdgeSpec("c1", "u", "c2", "ref"),
            EdgeSpec("i2", "y", "c1", "y"),
            EdgeSpec("step", "y", "c1", "ref"),
        ),
    )


def test_topology_json_round_trip():
    top = _demo_topology()
    s = topology_dumps(top)
    back = topology_loads(s)
    assert back == top


def test_build_and_compile():
    diagram = build_diagram_from_topology(_demo_topology(), default_mvp_catalog())
    ev = diagram.compile(backend="numpy")
    x = np.zeros(diagram.n)
    u = np.zeros(diagram.m)
    dx = ev.f(x, u, 0.0)
    assert dx.shape == (diagram.n,)


def test_extract_matches():
    top = _demo_topology()
    diagram = build_diagram_from_topology(top, default_mvp_catalog())
    kinds = {n.id: n.kind for n in top.nodes}
    params = {n.id: n.params for n in top.nodes}
    got = extract_topology_from_diagram(diagram, kinds=kinds, params=params)
    assert set(got.edges) == set(top.edges)
    assert {n.id: (n.kind, n.params) for n in got.nodes} == {
        n.id: (n.kind, n.params) for n in top.nodes
    }


def test_round_trip_topology_dict():
    data = _demo_topology().to_json_dict()
    again = round_trip_topology_dict(data)
    t0 = DiagramTopology.from_json_dict(data)
    t1 = DiagramTopology.from_json_dict(again)
    assert {n.id: (n.kind, n.params) for n in t0.nodes} == {
        n.id: (n.kind, n.params) for n in t1.nodes
    }
    assert set(t0.edges) == set(t1.edges)
    assert t0.name == t1.name


def test_unknown_kind():
    bad = DiagramTopology(
        nodes=(NodeSpec("a", "not_a_real_kind", {}),),
        edges=(),
    )
    with pytest.raises(KeyError):
        build_diagram_from_topology(bad, default_mvp_catalog())
