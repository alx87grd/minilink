"""The ``@compiled`` mark: an evaluator method jitted once per JAX evaluator, plain on NumPy."""

from __future__ import annotations

import functools


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
        # constructor starts compiled_forms empty)
        forms = evaluator.compiled_forms
        if self.name not in forms:
            forms[self.name] = evaluator.jax.jit(
                bound, static_argnames=self.static or None
            )

        return forms[self.name]
