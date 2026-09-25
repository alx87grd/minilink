import importlib.util

import pytest

OPTIONAL_MARKERS = ("jax", "symbolic", "visualization", "plotting", "ipopt", "rl")


def pytest_collection_modifyitems(items):
    """Mark every optional-extra test with the aggregate ``optional`` marker.

    A ``jax`` test is skipped when jax is not installed, so a module that mixes
    NumPy and JAX tests still runs its NumPy tests without the jax extra.
    """
    optional = pytest.mark.optional
    for item in items:
        if any(item.get_closest_marker(name) for name in OPTIONAL_MARKERS):
            item.add_marker(optional)

    if importlib.util.find_spec("jax") is None:
        skip_jax = pytest.mark.skip(reason="the jax extra is not installed")
        for item in items:
            if item.get_closest_marker("jax"):
                item.add_marker(skip_jax)
