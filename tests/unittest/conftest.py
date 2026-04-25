"""Shared pytest helpers for separating core and optional test suites."""

from __future__ import annotations

import importlib.util

import pytest


def has_package(name: str) -> bool:
    """Return whether *name* can be imported without importing it eagerly."""
    return importlib.util.find_spec(name) is not None


def pytest_collection_modifyitems(config, items):
    optional_markers = {"jax", "symbolic", "visualization"}
    for item in items:
        item_markers = {marker.name for marker in item.iter_markers()}
        if item_markers & optional_markers:
            item.add_marker(pytest.mark.optional)
