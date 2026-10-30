"""The ``@compiled`` mark: an evaluator method jitted once per JAX evaluator, plain on NumPy."""

from __future__ import annotations

import functools
import inspect

import numpy as np


def compiled(method=None, *, static=()):
    """
    Mark an evaluator method as compiled.

    On a JAX evaluator (``backend == "jax"``) the method is ``jax.jit`` of its
    body, built on first use and cached on the evaluator. Numbers and arrays
    among its arguments are traced (a new ``t``, ``dt`` or parameter value does
    not retrace); the keyword knobs named in ``static`` (``solver``,
    ``substeps``, ``hold``, …) are static. The method is fast from Python and
    still composes inside an outer ``jit``, ``grad``, ``vmap`` or ``scan``. On a
    NumPy evaluator it is the method itself. Used bare (``@compiled``) or with
    knobs (``@compiled(static=("solver",))``).
    """
    if method is None:
        return lambda body: CompiledMethod(body, tuple(static))

    return CompiledMethod(method, tuple(static))


def canonical_scalar(value, counts=False):
    """A Python number as a NumPy scalar of a strong type: ``int64`` for a count (a tick), else ``float64``; anything else as given."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return value

    return np.int64(value) if counts and isinstance(value, int) else np.float64(value)


def body_of(evaluator, name):
    """The plain body of a compiled method, bound to ``evaluator``: what the Jacobian differentiates."""
    body = getattr(type(evaluator), name)

    return body.__get__(evaluator, type(evaluator))


###############################################################################
# Internal machinery
###############################################################################


class CompiledMethod:
    """The descriptor behind :func:`compiled`: one jitted form per evaluator and method."""

    def __init__(self, method, static):
        self.method = method
        self.static = static
        self.name = method.__name__
        functools.update_wrapper(self, method)

    def __set_name__(self, owner, name):
        self.name = name

    def __get__(self, evaluator, owner=None):
        # On the class, the plain body: its signature and docstring (help, Sphinx)
        if evaluator is None:
            return self.method
        bound = self.method.__get__(evaluator, owner)
        if evaluator.backend != "jax":
            return bound

        # One jitted form per evaluator, built on first use (the evaluator's
        # constructor starts compiled_forms empty), behind the call that makes its
        # arguments canonical
        forms = evaluator.compiled_forms
        if self.name not in forms:
            jitted = evaluator.jax.jit(bound, static_argnames=self.static or None)
            forms[self.name] = CompiledCall(
                jitted, evaluator, self.arguments, self.static
            )

        return forms[self.name]

    @functools.cached_property
    def arguments(self) -> tuple:
        """The method's arguments after ``self``, in order: ``(name, counts)``, ``counts`` when its default is an integer (a tick)."""
        parameters = list(inspect.signature(self.method).parameters.values())[1:]

        return tuple(
            (p.name, isinstance(p.default, int) and not isinstance(p.default, bool))
            for p in parameters
        )


class CompiledCall:
    """
    One compiled method of one JAX evaluator, its arguments made canonical: one trace for every value.

    A Python scalar becomes a NumPy scalar of a strong type (``t=0.5`` and
    ``t=np.float64(0.5)`` share a trace); ``params=None`` is the frozen form, the
    compiled values constants of the trace, or for a large tree that tree put on
    the device once and passed as an argument; a tree given is the parametric
    form, each leaf in its compiled dtype and shape.
    """

    def __init__(self, jitted, evaluator, arguments, static):
        self.jitted = jitted
        self.evaluator = evaluator
        self.names = tuple(name for name, _ in arguments)
        self.counts = {name for name, counts in arguments if counts}
        self.static = static
        self.params_index = (
            self.names.index("params") if "params" in self.names else None
        )

    def __call__(self, *args, **kwargs):
        args = list(args)
        for i, value in enumerate(args):
            name = self.names[i]
            if name != "params":
                args[i] = canonical_scalar(value, name in self.counts)
        for name, value in kwargs.items():
            if name != "params" and name not in self.static:
                kwargs[name] = canonical_scalar(value, name in self.counts)

        # The parameters: the frozen form, or the tree in its compiled types
        index = self.params_index
        if index is not None and index < len(args):
            args[index] = self.evaluator.call_tree(args[index])
        elif index is not None:
            kwargs["params"] = self.evaluator.call_tree(kwargs.get("params"))

        return self.jitted(*args, **kwargs)

    def __getattr__(self, name):
        return getattr(self.jitted, name)
