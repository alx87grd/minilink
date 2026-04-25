from __future__ import annotations

import pytest

OPTIONAL_MARKERS = ("jax", "symbolic", "visualization")


def pytest_collection_modifyitems(items):
    """Mark every optional-extra test with the aggregate ``optional`` marker."""
    optional = pytest.mark.optional
    for item in items:
        if any(item.get_closest_marker(name) for name in OPTIONAL_MARKERS):
            item.add_marker(optional)
