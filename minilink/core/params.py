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
    """The leaf of ``tree`` at ``path`` (a dotted string or a sequence of keys); refuse a path that names none."""
    node = tree
    for part in split_path(path) if isinstance(path, str) else path:
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


def merge_known(tree, override):
    """
    ``tree`` with the leaves of ``override`` replacing its own: the one merge at a boundary.

    A partial tree is checked against the structure it merges onto: a key the
    tree lacks raises and names the known ones, a subtree given as a leaf (or a
    leaf as a subtree) raises, and a leaf of a new shape raises (a ``None``
    leaf, such as a seed at its mean, takes any value). A list or tuple leaf
    becomes an array: a tree's containers are dicts.
    """
    merged = dict(tree)
    for key, value in override.items():
        require_known_key(key, tree)
        node = tree[key]
        if isinstance(node, dict):
            require_subtree(key, value)
            merged[key] = merge_known(node, value)
        else:
            require_same_shape(key, value, node)
            merged[key] = (
                np.asarray(value) if isinstance(value, (list, tuple)) else value
            )

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


###############################################################################
# Internal machinery
###############################################################################


def require_known_key(key, tree):
    """Refuse a key the tree does not have, naming the ones it has."""
    if key not in tree:
        raise ValueError(
            f"params has no entry {key!r}; the known entries are {sorted(tree)}"
        )


def require_subtree(key, value):
    """Refuse a leaf where the tree has a subtree."""
    if not isinstance(value, dict):
        raise ValueError(
            f"params[{key!r}] is a subtree: give a dict of its entries, got "
            f"{type(value).__name__}"
        )


def require_same_shape(key, value, node):
    """Refuse a subtree where the tree has a leaf, or a leaf of another shape."""
    if isinstance(value, dict):
        raise ValueError(f"params[{key!r}] is a leaf, got a dict")
    if value is None or node is None:
        return
    shape, expected = np.shape(value), np.shape(node)
    if shape != expected:
        raise ValueError(
            f"params[{key!r}] has shape {expected}, got {shape}: a value may change, "
            "its shape is the program's structure"
        )
