"""Contract tests for :func:`~minilink.core.assembly.flatten_diagram`.

Flatten must preserve the nested global state stack ``x`` and agree with native
:math:`\\dot{x} = f(x, u, t)` on non-trivial operating points (not only rest).
"""

from __future__ import annotations

import numpy as np
import pytest

from minilink.core.assembly import flatten_diagram
from minilink.core.compile.compiler import compile_diagram
from minilink.core.diagram import DiagramSystem
from tests.unittest import flatten_diagram_helpers as fh


def _nontrivial_x(n: int) -> np.ndarray:
    if n == 0:
        return np.zeros(0, dtype=float)
    i = np.arange(n, dtype=float)
    return 0.1 + 0.07 * (i % 5) * ((-1.0) ** i)


def _assert_dynamics_nontrivial(diagram: DiagramSystem, x, u, t=0.0, *, min_max_f=1e-6):
    f = np.asarray(diagram.f(x, u, t), dtype=float)
    assert np.linalg.norm(x) > 0.1 * np.sqrt(max(x.size, 1))
    assert np.max(np.abs(f)) > min_max_f
    assert np.count_nonzero(np.abs(f) > 1e-8) >= min(3, max(f.size, 1))


def assert_flatten_dynamics_parity(diagram: DiagramSystem, *, u=None, atol=1e-10):
    """``flatten_diagram`` copy matches ``x0``, ``n``, ``f``, and ``compile(flatten=True)``."""
    if u is None:
        u = np.zeros(diagram.m)
    flat, _report = flatten_diagram(diagram)
    np.testing.assert_allclose(flat.x0, diagram.x0, atol=atol)
    assert flat.n == diagram.n

    probes = [_nontrivial_x(diagram.n)]
    if diagram.n:
        probes.append(probes[0] + 0.03 * (np.arange(diagram.n) % 7))

    for x in probes:
        _assert_dynamics_nontrivial(diagram, x, u)
        np.testing.assert_allclose(
            flat.f(x, u, 0.0),
            diagram.f(x, u, 0.0),
            atol=atol,
            err_msg="flatten_diagram.f must match nested native f",
        )

    if diagram.m:
        u_nz = np.zeros(diagram.m)
        u_nz[0] = 1.0
        x = probes[0]
        _assert_dynamics_nontrivial(diagram, x, u_nz)
        np.testing.assert_allclose(
            flat.f(x, u_nz, 0.0), diagram.f(x, u_nz, 0.0), atol=atol
        )

    ev = compile_diagram(diagram, flatten=True, backend="numpy")
    x = probes[0]
    np.testing.assert_allclose(ev.f(x, u, 0.0), diagram.f(x, u, 0.0), atol=atol)


_PARITY_BUILDERS = (
    fh.twin_dense_15,
    fh.minimal_twin_nest_with_sum,
    fh.wrapped_dense_15,
    fh.deep_pid_nest,
    fh.stateful_before_nest,
    fh.stateful_between_nests,
)


@pytest.mark.parametrize(
    "builder", _PARITY_BUILDERS, ids=[f.__name__ for f in _PARITY_BUILDERS]
)
def test_flatten_dynamics_parity(builder):
    assert_flatten_dynamics_parity(builder())


def _proportional_loop() -> DiagramSystem:
    """``r → P → integrator``, the integrator output fed back and exposed as ``y``."""
    from minilink import Integrator, P

    loop = DiagramSystem()
    loop.add_subsystem(P(2.0, ports="reference"), "controller")
    loop.add_subsystem(Integrator(), "plant")
    loop.add_input_port("r")
    loop.connect("input", "r", "controller", "r")
    loop.connect("plant", "y", "controller", "y")
    loop.connect("controller", "u", "plant", "u")
    loop.connect_new_output_port("plant", "y", "y")
    return loop


def _outer_feedback(block_id="gain") -> DiagramSystem:
    """A gain closing an outer loop around the nested ``loop`` box."""
    from minilink import Gain

    outer = DiagramSystem()
    outer.add_subsystem(Gain([-1.0]), block_id)
    outer.add_subsystem(_proportional_loop(), "loop")
    outer.connect("loop", "y", block_id, "u")
    outer.connect(block_id, "y", "loop", "r")
    return outer


def test_flatten_rewires_a_diagram_output_read_off_a_nested_box():
    outer = _outer_feedback()
    outer.connect_new_output_port("loop", "y", "out")

    flat, _report = flatten_diagram(outer)
    assert flat.connections["output"]["out"] == ("loop__plant", "y")

    x = np.array([0.7])
    u = np.zeros(0)
    nested_y = compile_diagram(outer).outputs(x, u)["out"]
    np.testing.assert_allclose(
        compile_diagram(outer, flatten=True).outputs(x, u)["out"], nested_y
    )
    np.testing.assert_allclose(flat.outputs["out"].compute(x, u, 0.0), nested_y)
    np.testing.assert_allclose(flat.f(x, u, 0.0), outer.f(x, u, 0.0))


def test_flatten_params_remap_follows_a_renamed_block():
    from minilink.core.assembly import remap_params_nested_to_flat

    # A top-level block already holds the name the nested plant would take
    outer = _outer_feedback(block_id="loop__plant")
    flat, report = flatten_diagram(outer)
    assert report.id_map["loop/plant"] == "loop__plant2"

    params = {**outer.params, "loop": {**outer.params["loop"]}}
    params["loop"]["controller"] = {
        **params["loop"]["controller"],
        "Kp": np.array([5.0]),
    }
    flat_params = remap_params_nested_to_flat(outer, params)
    assert set(flat_params) == set(flat.subsystems)

    x = np.array([0.7])
    u = np.zeros(0)
    ev = compile_diagram(outer, flatten=True)
    np.testing.assert_allclose(
        ev.f(x, u, 0.0, params=flat_params), outer.f(x, u, 0.0, params=params)
    )


def test_flatten_report_keys_every_leaf_by_its_nested_path():
    from minilink import Gain

    mid = DiagramSystem()
    mid.add_subsystem(Gain([1.0]), "gain")
    mid.add_subsystem(_proportional_loop(), "inner")
    mid.add_input_port("r")
    mid.connect("input", "r", "gain", "u")
    mid.connect("gain", "y", "inner", "r")
    mid.connect_new_output_port("inner", "y", "y")

    outer = DiagramSystem()
    outer.add_subsystem(Gain([-1.0]), "feedback")
    outer.add_subsystem(mid, "mid")
    outer.connect("mid", "y", "feedback", "u")
    outer.connect("feedback", "y", "mid", "r")

    _flat, report = flatten_diagram(outer)
    assert report.id_map == {
        "feedback": "feedback",
        "mid/gain": "mid__gain",
        "mid/inner/controller": "mid__inner__controller",
        "mid/inner/plant": "mid__inner__plant",
    }
