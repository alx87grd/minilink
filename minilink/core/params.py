"""Parameter trees: the nested ``params`` dicts of composed objects, read and written by path."""

from __future__ import annotations

import hashlib
import inspect
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


def own_params(
    doc="The object's numbers: this instance's own (``{}`` until it sets some).",
):
    """
    The ``params`` of a base class: each instance's own dict, ``{}`` until it assigns one.

    Read, it is the instance's own tree, or a new empty dict for an instance
    that holds none (nothing is written on read, and no dict is shared between
    instances). Assigned a dict, it declares the instance's numbers anew in the
    dict it already holds: that dict keeps its identity, so a composite holding
    it (``{"plant": plant.params}``) stays current. The first assignment, or
    anything but a dict, is stored as given.
    """

    def get(self):
        return self.__dict__.get("params", {})

    def set_(self, value):
        own = self.__dict__.get("params")
        if type(own) is dict and type(value) is dict and own is not value:
            # The new numbers in the held dict: every holder sees them
            fresh = dict(value)
            own.clear()
            own.update(fresh)
            return
        self.__dict__["params"] = value

    return OwnParams(get, set_, doc=doc)


def param_view(key):
    """An attribute that reads and writes ``self.params[key]``: a view of the one home of the number."""

    def read(self):
        return self.params[key]

    def write(self, value):
        self.params[key] = value

    return property(read, write, doc=f"View of ``params[{key!r}]``.")


class OwnParams(property):
    """The ``params`` property of a base class (:func:`own_params`): an assignment replaces the object's tree whole."""


def held_params(
    doc="The numbers of the parts, each at its key, and the object's own beside them.",
):
    """
    The ``params`` of a composite value: the tree it holds, each part's own dict at its key.

    The tree is built once, by the constructor's assignment, holding each
    part's ``params`` by reference beside the composite's own numbers, so a
    number has one home and an edit anywhere lands in it. Assigned again, the
    tree is routed: a part's entry goes to that part (merged onto its numbers
    and checked: an unknown key or a new shape raises), and an own number is
    written in place. The class names its parts with ``param_parts()``, a
    nested mapping of the tree's shape whose leaves are the part objects.
    """

    def get(self):
        return self.__dict__["params"]

    def set_(self, value):
        tree = self.__dict__.get("params")
        if tree is None:
            self.__dict__["params"] = value
            return
        route_tree(tree, value, self.param_parts())

    return HeldParams(get, set_, doc=doc)


class HeldParams(property):
    """The ``params`` property of a composite value (:func:`held_params`): an assignment routes to the parts."""


def delegated_params(attribute, doc="The numbers of the object it wraps."):
    """
    The ``params`` of a wrapper: the tree of the object at ``attribute``, read and assigned through it.

    An input box is its box's numbers, a uniform law its box's, a margin field
    its set's: one home, the wrapped object's.
    """

    def get(self):
        return getattr(self, attribute).params

    def set_(self, value):
        getattr(self, attribute).params = value

    return DelegatedParams(get, set_, doc=doc, attribute=attribute)


class DelegatedParams(property):
    """The ``params`` property of a wrapper (:func:`delegated_params`): its wrapped object's."""

    def __init__(self, fget, fset, doc, attribute):
        super().__init__(fget, fset, doc=doc)
        self.attribute = attribute


def member_keys(members) -> list:
    """
    The keys of a composite's members in its tree: each member's class name, a repeat numbered from 2 (``"BallSet"``, ``"BallSet_2"``).

    A member keyed by what it is, not by its position, keeps its key when the
    members are reordered, so a recorded tree still names it.
    """
    keys = []
    seen = {}
    for member in members:
        name = type(member).__name__
        seen[name] = seen.get(name, 0) + 1
        keys.append(name if seen[name] == 1 else f"{name}_{seen[name]}")

    return keys


def holds_own_tree(obj) -> bool:
    """
    Whether assigning ``obj.params`` replaces its tree whole: a leaf's own dict, not a composite's routing.

    A system's plain attribute or the base classes' :func:`own_params` holds
    the tree; a composite's own ``params`` property routes each entry to its
    parts, and keeps the entries it is not given; a wrapper is what it wraps.
    """
    descriptor = inspect.getattr_static(type(obj), "params", None)
    if isinstance(descriptor, DelegatedParams):
        return holds_own_tree(getattr(obj, descriptor.attribute))

    return descriptor is None or isinstance(descriptor, OwnParams)


def require_complete_part(tree, part, path=()):
    """Refuse a tree for ``part`` that would wipe its other numbers: checked when its assignment replaces its tree whole."""
    if holds_own_tree(part):
        require_complete(tree, part.params, path)


def route_tree(tree, value, parts, path=()):
    """
    Route ``value`` into the held ``tree``: each part's entry to the part, each own number in place.

    The tree given is complete (:func:`require_complete`, checked before the
    first write): a part that holds its own tree takes its entry whole, a
    composite part routes its entry itself, an own number is written in place.
    """
    require_complete(value, tree, path)
    for key, entry in value.items():
        part = parts.get(key)
        if isinstance(part, dict):
            route_tree(tree[key], entry, part, (*path, key))
        elif part is not None and holds_own_tree(part):
            # An empty entry moves nothing: a part with no numbers (a user's frozen
            # field) is never written
            if entry:
                part.params = entry
        elif part is not None:
            part.params = entry
        else:
            tree[key] = entry


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


def checked(tree, reference):
    """
    The tree a verb runs on: ``reference`` for ``None``, else ``tree`` checked complete against it.

    Complete trees everywhere (DESIGN §4, "Parameters"): a tree given to a verb
    names every entry of the tool's own tree, in the same shapes
    (:func:`require_complete`), so nothing is merged and nothing is left to a
    default. A list or tuple leaf becomes an array: a tree's containers are dicts.
    """
    if tree is None:
        return reference
    require_complete(tree, reference)

    return merge_known(reference, tree)


def require_complete(tree, reference, path=()):
    """
    Refuse a tree that does not name every entry of ``reference``, or names one it lacks.

    The check of a tree a composite routes to one of its parts: at every level
    the same keys, a leaf where a leaf is, a subtree where a subtree is, and
    each leaf in its reference's shape. A ``None`` leaf on either side (a seed
    at its mean) and a distribution standing where a number goes are not
    shape-checked.
    """
    where = "params" + "".join(f"[{key!r}]" for key in path)
    if isinstance(reference, dict):
        if not isinstance(tree, dict):
            raise ValueError(
                f"{where} is a subtree: give a dict of its entries, got "
                f"{type(tree).__name__}"
            )
        unknown = [key for key in tree if key not in reference]
        missing = [key for key in reference if key not in tree]
        if unknown or missing:
            raise ValueError(
                f"{where} is a complete tree: unknown {unknown}, missing {missing}; "
                f"the known entries are {list(reference)} (a tree at other numbers is "
                "the tool's own, edited: p = tool.params, a copy on a tool)"
            )
        for key, value in tree.items():
            require_complete(value, reference[key], (*path, key))
        return
    if isinstance(tree, dict):
        raise ValueError(f"{where} is a leaf, got a dict")
    if tree is None or reference is None or is_law(tree) or is_law(reference):
        return
    if np.shape(tree) != np.shape(reference):
        raise ValueError(
            f"{where} has shape {np.shape(reference)}, got {np.shape(tree)}"
        )


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


def tree_digest(tree) -> bytes:
    """A digest of a tree's structure and values: every numeric leaf by its dtype, shape and bytes (a JAX array's too), a tracer by its identity."""
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
    reference = (
        np.asarray(reference) if isinstance(reference, (list, tuple)) else reference
    )
    dtype = np.result_type(reference)
    shape = np.shape(reference)

    # A JAX value already in the reference's dtype and shape, strongly typed, passes
    # as it is: a tree held on the device is never copied (a weak scalar converts)
    if is_jax_value(leaf) and not getattr(leaf, "weak_type", True):
        if leaf.dtype == dtype and tuple(leaf.shape) == shape:
            return leaf

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
    if isinstance(leaf, (list, tuple)) and np.asarray(leaf).dtype.kind in "biufc":
        leaf = np.asarray(leaf)
    if isinstance(leaf, (int, float, complex, np.ndarray, np.generic)):
        array = np.asarray(leaf)
        return f"{array.dtype}{array.shape}".encode() + array.tobytes()

    return b"obj" + id(leaf).to_bytes(8, "little")


def is_law(value) -> bool:
    """Whether ``value`` is a distribution standing where a number goes (it draws: ``sample``)."""
    return hasattr(value, "sample") and not isinstance(value, (np.ndarray, np.generic))


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
