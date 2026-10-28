"""Parameter trees: the nested ``params`` dicts of composed objects, read and written by path."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

###############################################################################
# Public API
###############################################################################


@dataclass(frozen=True)
class Batched:
    """A parameter tree whose drawn leaves carry a leading trial axis, and its axes tree (``0`` drawn, ``None`` shared)."""

    tree: dict
    axes: dict


def split_path(path):
    """A dotted path (``"controller.Kp"``; a nested box ``"plant/motor.R"``) as its keys."""
    return path.replace("/", ".").split(".")


def lookup(tree, path):
    """The leaf of ``tree`` at ``path``; refuse a path that names none."""
    node = tree
    for part in split_path(path):
        if not isinstance(node, dict) or part not in node:
            raise KeyError(f"{path!r} names no leaf of the parameters")
        node = node[part]

    return node


def with_leaf(tree, path, value):
    """A copy of ``tree`` (copied along ``path`` only) with the leaf at ``path`` replaced.

    ``path`` is a dotted string or a sequence of keys.
    """
    head, *rest = split_path(path) if isinstance(path, str) else path
    copy = dict(tree)
    copy[head] = value if not rest else with_leaf(tree[head], rest, value)

    return copy


def set_leaf(tree, path, value):
    """Insert ``value`` at the keys ``path`` of ``tree`` in place, creating nested dicts."""
    for key in path[:-1]:
        tree = tree.setdefault(key, {})
    tree[path[-1]] = value


def merge_tree(tree, override):
    """``tree`` with the entries of ``override`` replacing its own, recursively."""
    if isinstance(override, Batched):
        override = override.tree

    merged = dict(tree)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = merge_tree(merged[key], value)
        else:
            merged[key] = value

    return merged


def flatten(tree, prefix=()):
    """Yield ``(keys, leaf)`` pairs of a nested parameter dict, depth-first; ``None`` has none."""
    if tree is None:
        return
    if not isinstance(tree, dict):
        raise TypeError(
            f"params must be a dict (nested for diagrams), got {type(tree).__name__}"
        )
    for key, value in tree.items():
        if isinstance(value, dict):
            yield from flatten(value, prefix + (key,))
        else:
            yield prefix + (key,), value


def flat_leaves(tree):
    """The leaves of a nested dict, in order (a tree that is not a dict is its one leaf)."""
    if isinstance(tree, Batched):
        tree = tree.tree
    if isinstance(tree, dict):
        return [leaf for value in tree.values() for leaf in flat_leaves(value)]

    return [tree]


def float_leaves(tree):
    """``tree`` with every leaf a float NumPy array (``None``, an empty entry, kept)."""
    if isinstance(tree, dict):
        return {key: float_leaves(value) for key, value in tree.items()}
    if tree is None:
        return None

    return np.asarray(tree, dtype=float)
