"""The ``@compiled`` mark: an evaluator method jitted once per JAX evaluator, plain on NumPy."""

from __future__ import annotations

import functools
import inspect

import numpy as np


def compiled(method=None, *, static=()):
    """
    Mark an evaluator method as compiled.

    On a JAX evaluator (``backend == "jax"``) the method is ``jax.jit`` of its
    body, built on first use and cached on the evaluator. The numbers and
    arrays among its arguments are traced (a new ``t``, ``dt`` or parameter
    value does not retrace); every other leaf, a Python ``bool``, a string,
    ``None`` or an object, is held static, so a block that branches on a flag in
    its ``params`` still compiles; the keyword knobs named in ``static``
    (``solver``, ``substeps``, ``hold``, …) are static too. The method is fast
    from Python and still composes inside an outer ``jit``, ``grad``, ``vmap``
    or ``scan``. On a NumPy evaluator it is the method itself. Used bare
    (``@compiled``) or with knobs (``@compiled(static=("solver",))``).
    """
    if method is None:
        return lambda body: CompiledMethod(body, tuple(static))

    return CompiledMethod(method, tuple(static))


###############################################################################
# Internal machinery
###############################################################################


class CompiledMethod:
    """The descriptor behind :func:`compiled`: one jitted form per evaluator and method."""

    def __init__(self, method, static):
        self.method = method
        self.static = static
        self.name = method.__name__
        self.params_index = positional_index(method, "params")
        functools.update_wrapper(self, method)

    def __set_name__(self, owner, name):
        self.name = name

    def __get__(self, evaluator, owner=None):
        if evaluator is None:
            return self
        bound = self.method.__get__(evaluator, owner)
        if evaluator.backend != "jax":
            return bound

        # One jitted form per evaluator, built on first use (the evaluator's
        # constructor starts compiled_forms empty)
        forms = evaluator.compiled_forms
        if self.name not in forms:
            forms[self.name] = params_filtered_jit(
                evaluator.jax, bound, self.static, self.params_index
            )

        return forms[self.name]


def params_filtered_jit(jax, body, knobs, params_index):
    """``jax.jit`` of ``body``; a ``params`` tree with leaves that are not numbers holds those static.

    ``params=None`` and a tree of numbers and arrays call the plain jitted body;
    a tree carrying a flag (a Python ``bool``, a string, an object) passes its
    numbers traced and the flags as a static argument, one compile per flag set.
    """
    plain = jax.jit(body, static_argnames=knobs or None)

    def with_flags(*args, flags, **kwargs):
        treedef, held = flags
        args, kwargs = with_params(
            args, kwargs, params_index, merged(jax, treedef, held)
        )
        return body(*args, **kwargs)

    flagged = jax.jit(with_flags, static_argnames=(*knobs, "flags"))

    def form(*args, **kwargs):
        params = (
            args[params_index] if len(args) > params_index else kwargs.get("params")
        )
        if params is None:
            return plain(*args, **kwargs)
        leaves, treedef = jax.tree_util.tree_flatten(params)
        if all(map(is_traced, leaves)):
            return plain(*args, **kwargs)

        # The flags held static, the numbers traced in their place
        held = tuple((i, leaf) for i, leaf in enumerate(leaves) if not is_traced(leaf))
        numbers = [leaf if is_traced(leaf) else None for leaf in leaves]
        args, kwargs = with_params(args, kwargs, params_index, numbers)
        return flagged(*args, flags=(treedef, held), **kwargs)

    return functools.wraps(body)(form)


def merged(jax, treedef, held):
    """A function of the traced leaves rebuilding the ``params`` tree with its static flags."""

    def rebuild(numbers):
        leaves = list(numbers)
        for i, leaf in held:
            leaves[i] = leaf
        return jax.tree_util.tree_unflatten(treedef, leaves)

    return rebuild


def with_params(args, kwargs, params_index, value):
    """``(args, kwargs)`` with the ``params`` argument replaced by ``value`` (a callable rebuilds it)."""
    if len(args) > params_index:
        current = args[params_index]
        new = value(current) if callable(value) else value
        return (*args[:params_index], new, *args[params_index + 1 :]), kwargs

    new = value(kwargs["params"]) if callable(value) else value
    return args, {**kwargs, "params": new}


def positional_index(method, name):
    """The position of argument ``name`` in a method's call, ``self`` excluded."""
    names = list(inspect.signature(method).parameters)

    return names.index(name) - 1


def is_traced(leaf):
    """Whether a leaf is traced: a number or an array, never a Python ``bool``."""
    if isinstance(leaf, bool):
        return False

    return isinstance(leaf, (int, float, complex, np.ndarray, np.generic)) or hasattr(
        leaf, "shape"
    )
