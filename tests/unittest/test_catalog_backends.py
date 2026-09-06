"""Every catalog plant compiles on both backends and agrees on random points (S23).

``NUMPY_ONLY`` lists the plants still written with NumPy branches; each entry
is a strict xfail, so sweeping a module to ``xp = array_module(...)`` fails
this test until its names are removed here. The list only shrinks.
"""

from __future__ import annotations

import unittest

import numpy as np
import pytest

import minilink.catalog as catalog

pytestmark = [pytest.mark.optional, pytest.mark.jax]

# Plants that need constructor arguments.
FACTORIES = {
    "SpeedControlledManipulator": lambda: catalog.SpeedControlledManipulator(2, 2),
}

# Still NumPy-only (S22). Empty since 2026-09-06: every catalog plant traces.
NUMPY_ONLY: frozenset[str] = frozenset()


def _cases():
    for name in catalog.__all__:
        marks = ()
        if name in NUMPY_ONLY:
            marks = pytest.mark.xfail(strict=True, reason=f"{name} is NumPy-only (S22)")
        yield pytest.param(name, id=name, marks=marks)


def _random_points(sys, count=3, seed=0):
    rng = np.random.default_rng(seed)
    x0 = np.asarray(sys.x0, dtype=float)
    u0 = np.asarray(sys.get_u_from_input_ports(), dtype=float)
    for _ in range(count):
        yield (
            x0 + 0.3 * rng.standard_normal(x0.shape),
            u0 + 0.3 * rng.standard_normal(u0.shape),
            float(rng.uniform(0.0, 1.0)),
        )


@pytest.mark.parametrize("name", list(_cases()))
def test_plant_compiles_on_both_backends_and_agrees(name):
    pytest.importorskip("jax")
    sys = FACTORIES.get(name, getattr(catalog, name))()
    numpy_ev = sys.compile(backend="numpy", verbose=False)
    jax_ev = sys.compile(backend="jax", verbose=False)
    for x, u, t in _random_points(sys):
        np.testing.assert_allclose(
            np.asarray(jax_ev.f(x, u, t)), numpy_ev.f(x, u, t), rtol=1e-9, atol=1e-11
        )
        outs_np = numpy_ev.outputs(x, u, t)
        outs_jax = jax_ev.outputs(x, u, t)
        assert set(outs_np) == set(outs_jax)
        for port, value in outs_np.items():
            np.testing.assert_allclose(
                np.asarray(outs_jax[port]), value, rtol=1e-9, atol=1e-11
            )


def test_catalog_check_registry_covers_every_catalog_plant():
    from tests.demo_checks import catalog_check_registry as registry

    entries = [
        value
        for value in vars(registry).values()
        if isinstance(value, (list, tuple))
        and value
        and isinstance(value[0], registry.CatalogCheckEntry)
    ][0]
    missing = sorted(set(catalog.__all__) - {entry.id for entry in entries})
    assert missing == [], f"catalog plants without a catalog-check entry: {missing}"


def test_numpy_only_list_names_catalog_plants():
    unknown = sorted(NUMPY_ONLY - set(catalog.__all__))
    assert unknown == [], f"not catalog names: {unknown}"


if __name__ == "__main__":
    unittest.main()
