"""The compiled evaluators the convenience calls reuse: JAX evaluators keyed by the structure compile reads."""

from __future__ import annotations

import collections
import hashlib

import numpy as np

from minilink.core.backends import BACKEND_JAX, ensure_jax_x64, normalize_backend
from minilink.core.params import is_jax_value, is_tracer, leaf_bytes, tree_digest

# How many compiled evaluators the convenience calls keep, the least recently used dropped first
CACHE_SIZE = 16

# A block's attributes that are not structure: its numbers (read as parameters), its start,
# its run records and the containers and the edit count the fingerprint reads on their own
NOT_STRUCTURE = frozenset(
    {
        "edits",
        "params",
        "params_override",
        "x0",
        "own_x0",
        "hold_x0",
        "traj",
        "state",
        "state_index",
        "inputs",
        "outputs",
        "subsystems",
        "connections",
    }
)


###############################################################################
# Public API
###############################################################################


def compiled_of(system, backend="numpy", *, flatten=False):
    """
    ``system`` compiled on ``backend``: a JAX evaluator reused while the structure compile reads is unchanged.

    The key is the system's structure: each block by identity and its edit
    count (:attr:`~minilink.core.system.System.edits`, bumped by every
    assignment of a block's attribute but its values and run records), its
    ports, the wiring, the plain numbers and arrays a block holds by their
    bytes, a held model's own structure and numbers, and the x64 flag; never
    the parameter values: a caller passes ``evaluator.call_params()``, the
    frozen form while the values are the compiled ones and the parametric form
    after an edit, so a sweep of values compiles once. A NumPy evaluator is
    built fresh (it costs a fraction of a millisecond and reads its blocks
    live). An edit the key cannot see, a change made inside an object a block
    holds (``block.table.gain = 2``) or a class's method replaced, is seen after
    :func:`clear_compiled` or by compiling a handle of one's own
    (``system.compile(backend="jax")``).
    """
    # Lazy import: the compiler imports the systems, which import this module's callers
    from minilink.core.compile.compiler import compile as compile_system

    key = normalize_backend(backend)
    if key != BACKEND_JAX:
        return compile_system(system, backend=key, flatten=flatten)

    # The float policy the evaluator will build under, then the structure it reads
    ensure_jax_x64()
    cache_key = (key, bool(flatten), x64_enabled(), fingerprint(system))
    evaluator = COMPILED.get(cache_key)
    if evaluator is None:
        evaluator = compile_system(system, backend=key, flatten=flatten)
        COMPILED[cache_key] = evaluator
        while len(COMPILED) > CACHE_SIZE:
            COMPILED.popitem(last=False)
    COMPILED.move_to_end(cache_key)

    return evaluator


def clear_compiled():
    """Drop every evaluator the convenience calls keep: the next run compiles again."""
    COMPILED.clear()


def fingerprint(system, seen=frozenset()) -> tuple:
    """
    The structure a compile reads, by Python identity: the key of a reused evaluator.

    For a block its class and identity, its ports (size, nominal value, an
    output's dependencies and compute function), its equations when set on the
    instance, and its plain attributes outside ``params`` and ``x0`` (the values
    an equation reads at trace time: a ``Sum``'s signs, a ``PID``'s flags, a
    source's law, a clock's period); for a diagram the same of its edge ports,
    its wiring and its blocks, recursively. A cached evaluator keeps its blocks
    alive, so an identity in a key is never reused while the key exists; a
    diagram is not keyed by its own identity, so a loop rebuilt over the same
    blocks (``problem.compile``, ``closed_loop``) reuses the evaluator.
    """
    # A block reached again through its own attributes (two blocks naming each other)
    if id(system) in seen:
        return ("seen", id(system))
    seen = seen | {id(system)}
    subsystems = vars(system).get("subsystems")

    # A diagram is its wiring and its blocks: one rebuilt over the same blocks is the same
    # structure; a block is itself and its edits, since its equations read the instance
    identity = None if subsystems is not None else (id(system), system.edits)
    parts = [
        type(system),
        identity,
        ports_of(system),
        overrides_of(system),
        attributes_of(system, seen),
    ]
    if subsystems is not None:
        parts.append(
            tuple(
                (sys_id, fingerprint(sub, seen)) for sys_id, sub in subsystems.items()
            )
        )
        parts.append(wiring_of(system))

    return tuple(parts)


def structure_of(obj) -> tuple:
    """Any object's plain attributes, systems and parameterized parts, as :func:`fingerprint` reads a block's: a key for a compiled program built from it."""
    return attributes_of(obj)


###############################################################################
# Internal machinery
###############################################################################

# Above this many bytes an attribute's value is keyed by its digest, not kept in the key
KEY_BYTES = 512

# The reused evaluators, oldest first
COMPILED = collections.OrderedDict()


def x64_enabled() -> bool:
    """Whether JAX computes in float64: an evaluator's nominal input is built under it."""
    # Lazy import: only a JAX compile reads it
    import jax

    return bool(jax.config.jax_enable_x64)


def ports_of(system) -> tuple:
    """Each port's id, size and nominal value; an output's dependencies and compute function too."""
    inputs = tuple(
        (port_id, int(port.dim), leaf_bytes(np.asarray(port.nominal_value)))
        for port_id, port in system.inputs.items()
    )
    # A diagram's own outputs read its wiring (in the fingerprint), not a function of theirs
    diagram = "subsystems" in vars(system)
    outputs = tuple(
        (
            port_id,
            int(port.dim),
            str(port.dependencies),
            None if diagram else function_identity(port.compute),
        )
        for port_id, port in system.outputs.items()
    )

    return inputs, outputs


def overrides_of(system) -> tuple:
    """The equations set on the instance itself, by identity."""
    own = vars(system)

    return tuple(
        (name, function_identity(own[name]))
        for name in ("f", "h", "step", "update")
        if name in own
    )


def attributes_of(system, seen=frozenset()) -> tuple:
    """The plain attributes outside the parameters and the start: what an equation may read at trace time."""
    # Lazy import: a real cycle, the systems import the compiler
    from minilink.core.system import System

    parts = []
    for name, value in vars(system).items():
        if name in NOT_STRUCTURE:
            continue
        # A clocked view's source: its structure, its numbers read through the tree
        if name == "source" and isinstance(value, System):
            parts.append((name, fingerprint(value, seen)))
            continue
        part = structure_value(value, seen)
        if part is not None:
            parts.append((name, part))

    return tuple(parts)


def structure_value(value, seen=frozenset()):
    """
    What the fingerprint reads of one attribute: plain values by bytes, a system by its fingerprint and numbers, a held object by its own plain values.

    A JAX array is read by value; a function by its code and the object it is
    bound to (the evaluator keeps it alive, so its identity is not reused); a
    plain-data object a block holds (a helper whose attributes are numbers,
    strings and arrays: a lookup table) by its type, identity and those values;
    any other object by its type and identity.
    """
    # Lazy import: a real cycle, the systems import the compiler
    from minilink.core.system import System

    # A model held by a block (a computed-torque law's plant) is read at trace time,
    # its numbers too
    if isinstance(value, System):
        return (fingerprint(value, seen), tree_digest(getattr(value, "params", {})))
    if is_jax_value(value) and not is_tracer(value):
        value = np.asarray(value)
    if value is None or isinstance(
        value, (bool, int, float, str, np.ndarray, np.generic)
    ):
        raw = leaf_bytes(value)
        return raw if len(raw) <= KEY_BYTES else hashlib.blake2b(raw).digest()
    if isinstance(value, (list, tuple)):
        return tuple(structure_value(item, seen) for item in value)
    if isinstance(value, dict):
        return tuple((key, structure_value(item, seen)) for key, item in value.items())
    if callable(value) and hasattr(value, "__code__") or hasattr(value, "__func__"):
        return function_identity(value)
    if isinstance(getattr(value, "params", None), dict):
        return (type(value), id(value), tree_digest(value.params))
    if is_plain_data(value):
        held = tuple(
            (name, structure_value(item)) for name, item in vars(value).items()
        )
        return (type(value), id(value), held)

    return (type(value), id(value))


def is_plain_data(value) -> bool:
    """Whether an object holds numbers, strings and arrays only: a helper the key reads by value."""
    plain = (type(None), bool, int, float, str, np.ndarray, np.generic)
    attributes = getattr(value, "__dict__", None)
    if not attributes:
        return False

    return all(
        isinstance(item, plain)
        or is_jax_value(item)
        or isinstance(item, (list, tuple))
        and all(isinstance(e, plain) for e in item)
        for item in attributes.values()
    )


def wiring_of(diagram) -> tuple:
    """A diagram's connections, target by target."""
    return tuple(
        (target, tuple(sorted(sources.items(), key=lambda item: str(item[0]))))
        for target, sources in diagram.connections.items()
    )


def function_identity(function) -> tuple:
    """A function's identity: its code and, for a bound method, the object it is bound to."""
    code = getattr(function, "__func__", function)

    return (id(code), id(getattr(function, "__self__", None)))
