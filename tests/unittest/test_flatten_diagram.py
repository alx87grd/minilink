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
