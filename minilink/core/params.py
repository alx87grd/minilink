"""Parameter trees: the nested ``params`` dicts of composed objects, read and written by path."""

from __future__ import annotations

import hashlib
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


def canonical_tree(tree, reference):
    """
    ``tree`` with each leaf in the dtype and shape of the leaf at its path in ``reference``: one trace for every value.

    A Python ``9.0``, ``np.float64(9.0)``, a ``9`` or a 0-d array given for a
    leaf compiled as a ``(1,)`` float array all become that array, so a jitted
    function of the tree traces once. A leaf with no reference (or a ``None``
    reference) becomes a NumPy array of its own type; a leaf whose size differs
    from its reference (a batch axis in front) keeps its shape.
    """
    if isinstance(tree, dict):
        own = reference if isinstance(reference, dict) else {}
        return {key: canonical_tree(value, own.get(key)) for key, value in tree.items()}

    return canonical_leaf(tree, reference)


class DeviceTree:
    """
    One tree kept on the device while its values repeat: a host loop's calls then transfer nothing.

    The entry holds the leaves it was built from, so a JAX leaf's identity (what
    the digest reads of it) cannot be reused by another array while the entry
    lives. A tree holding a tracer (a call under an outer ``jit``, ``grad`` or
    ``vmap``), or put on the device inside one, is returned, never kept: a tracer
    outlives its trace.
    """

    def __init__(self):
        self.entry = None

    def get(self, tree, prepare, put):
        """``put(prepare(tree))``, reused while ``tree``'s values repeat; ``prepare(tree)`` alone for a traced tree."""
        leaves = [leaf for _, leaf in flatten_any(tree)]
        if any(is_tracer(leaf) for leaf in leaves):
            return prepare(tree)
        digest = tree_digest(tree)
        if self.entry is not None and self.entry[0] == digest:
            return self.entry[2]

        # Put inside an outer trace, the result is itself traced: returned, not kept
        on_device = put(prepare(tree))
        if not any(is_tracer(leaf) for _, leaf in flatten_any(on_device)):
            self.entry = (digest, leaves, on_device)

        return on_device


def tree_digest(tree) -> bytes:
    """A digest of a tree's structure and values: JAX arrays by identity, host values by their bytes."""
    digest = hashlib.blake2b(digest_size=16)
    for path, leaf in flatten_any(tree):
        digest.update(repr(path).encode())
        digest.update(leaf_bytes(leaf))

    return digest.digest()


###############################################################################
# Internal machinery
###############################################################################


def canonical_leaf(leaf, reference):
    """One leaf in its reference's dtype and, when the sizes agree, its shape."""
    numeric = isinstance(
        leaf, (int, float, complex, list, tuple, np.ndarray, np.generic)
    )
    if leaf is None or isinstance(leaf, bool) or not (numeric or is_jax_value(leaf)):
        return leaf
    if reference is None or isinstance(reference, bool):
        return leaf if is_jax_value(leaf) else np.asarray(leaf)
    dtype = np.result_type(reference)
    shape = np.shape(reference)

    # A value its reference's dtype cannot hold (2.5 for an integer default) promotes
    # instead: one more trace, never a truncated number
    own = leaf.dtype if hasattr(leaf, "dtype") else np.asarray(leaf).dtype
    if not np.can_cast(own, dtype, "same_kind"):
        dtype = np.result_type(own, dtype)

    # A JAX value (a tracer under an outer transform) keeps its module
    value = (
        strong_jax(leaf, dtype) if is_jax_value(leaf) else np.asarray(leaf, dtype=dtype)
    )
    if value.shape != shape and value.size == int(np.prod(shape)):
        value = value.reshape(shape)

    return value


def strong_jax(value, dtype):
    """A JAX value in ``dtype`` with a strong type (a weak scalar would trace apart)."""
    # Lazy import: only a JAX leaf reaches here
    from minilink.core.backends import require_jax

    return require_jax().lax.convert_element_type(value, dtype)


def is_tracer(value) -> bool:
    """Whether ``value`` is a JAX tracer: a value inside a trace, not an array."""
    if not is_jax_value(value):
        return False
    # Lazy import: only a JAX value reaches here
    from minilink.core.backends import require_jax

    return isinstance(value, require_jax().core.Tracer)


def is_jax_value(value) -> bool:
    """Whether ``value`` is a JAX array or tracer (told by its type's module)."""
    return type(value).__module__.startswith("jax")


def flatten_any(tree, prefix=()):
    """Yield ``(keys, leaf)`` pairs of any nested dict, a non-dict tree its one leaf."""
    if isinstance(tree, dict):
        for key, value in tree.items():
            yield from flatten_any(value, prefix + (key,))
        return
    yield prefix, tree


def leaf_bytes(leaf) -> bytes:
    """What a digest reads of one leaf: its type, shape and bytes (a JAX array's too); a traced value its identity."""
    if is_tracer(leaf):
        return b"tracer" + id(leaf).to_bytes(8, "little")
    if is_jax_value(leaf):
        leaf = np.asarray(leaf)
    if leaf is None or isinstance(leaf, (bool, str)):
        return repr(leaf).encode()
    if isinstance(leaf, (int, float, complex, np.ndarray, np.generic)):
        array = np.asarray(leaf)
        return f"{array.dtype}{array.shape}".encode() + array.tobytes()

    return b"obj" + id(leaf).to_bytes(8, "little")


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
