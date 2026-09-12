"""
Probability distributions for stochastic planning problems.

A distribution answers one question, ``sample(key)``, on either backend: with
a NumPy :class:`numpy.random.Generator` (or an integer seed) it returns NumPy
arrays for plain Monte Carlo; with a JAX PRNG key it returns JAX arrays and
traces inside ``jit`` / ``vmap`` / ``lax.scan``, which is where reinforcement
learning draws its episode starts. ``mean()`` gives the representative point
the deterministic bridge :meth:`StochasticPlanningProblem.nominal` uses, and
``support`` (optional) the set it lives in.

This is a duck type, not a probability library: anything with ``dim``,
``sample(key)`` and ``mean()`` works.
"""

from abc import ABC, abstractmethod

import numpy as np

from minilink.core.backends import array_module
from minilink.core.sets import BoxSet, Set

# Public API


class Distribution(ABC):
    """Vector-valued distribution sampled on NumPy or JAX."""

    support: Set | None = None

    @property
    @abstractmethod
    def dim(self) -> int: ...

    @abstractmethod
    def mean(self) -> np.ndarray:
        """Representative point (the deterministic bridge)."""
        ...

    @abstractmethod
    def sample(self, key, n=None):
        """
        Draw one sample (shape ``(dim,)``) or ``n`` samples (``(n, dim)``).

        ``key`` is a :class:`numpy.random.Generator`, an integer seed, or a
        JAX PRNG key (JAX arrays out, traceable).
        """
        ...


class Gaussian(Distribution):
    """Diagonal Gaussian ``x ~ N(mean, diag(std**2))``."""

    def __init__(self, mean, std):
        self._mean = np.asarray(mean, dtype=float).reshape(-1)
        self.std = np.broadcast_to(
            np.asarray(std, dtype=float), self._mean.shape
        ).copy()

    @property
    def dim(self) -> int:
        return int(self._mean.size)

    def mean(self) -> np.ndarray:
        return self._mean.copy()

    def sample(self, key, n=None):
        shape = (self.dim,) if n is None else (int(n), self.dim)
        if is_jax_key(key):
            import jax

            return self._mean + self.std * jax.random.normal(key, shape)
        return self._mean + self.std * generator(key).standard_normal(shape)


class Uniform(Distribution):
    """Uniform on the box ``[lb, ub]``; ``support`` is that box."""

    def __init__(self, lb, ub):
        self.lb = np.asarray(lb, dtype=float).reshape(-1)
        self.ub = np.asarray(ub, dtype=float).reshape(-1)
        if self.lb.shape != self.ub.shape or np.any(self.ub < self.lb):
            raise ValueError("Uniform needs lb <= ub of the same shape")
        self.support = BoxSet(self.lb, self.ub)

    @property
    def dim(self) -> int:
        return int(self.lb.size)

    def mean(self) -> np.ndarray:
        return 0.5 * (self.lb + self.ub)

    def sample(self, key, n=None):
        shape = (self.dim,) if n is None else (int(n), self.dim)
        if is_jax_key(key):
            import jax

            return jax.random.uniform(key, shape, minval=self.lb, maxval=self.ub)
        return generator(key).uniform(self.lb, self.ub, size=shape)


class Particles(Distribution):
    """Empirical distribution: a uniform choice among fixed points ``(N, dim)``."""

    def __init__(self, points):
        self.points = np.asarray(points, dtype=float)
        if self.points.ndim != 2:
            raise ValueError("Particles needs points of shape (N, dim)")

    @property
    def dim(self) -> int:
        return int(self.points.shape[1])

    def mean(self) -> np.ndarray:
        return self.points.mean(axis=0)

    def sample(self, key, n=None):
        count = self.points.shape[0]
        if is_jax_key(key):
            import jax

            xp = array_module(key)
            idx = jax.random.randint(key, () if n is None else (int(n),), 0, count)
            return xp.asarray(self.points)[idx]
        idx = generator(key).integers(0, count, size=None if n is None else int(n))
        return self.points[idx]


class Sampler(Distribution):
    """
    Distribution defined by a sampling function ``draw(key) -> x``.

    For task-specific starts (a random point along a track, a random pose in
    free space). ``draw`` must accept a JAX key and trace; ``mean`` is the
    representative point given explicitly.
    """

    def __init__(self, draw, mean, support: Set | None = None):
        self.draw = draw
        self._mean = np.asarray(mean, dtype=float).reshape(-1)
        self.support = support

    @property
    def dim(self) -> int:
        return int(self._mean.size)

    def mean(self) -> np.ndarray:
        return self._mean.copy()

    def sample(self, key, n=None):
        if n is None:
            return self.draw(key)
        if is_jax_key(key):
            import jax

            return jax.vmap(self.draw)(jax.random.split(key, int(n)))
        rng = generator(key)
        return np.stack([np.asarray(self.draw(rng)) for _ in range(int(n))])


# Internal machinery


def is_jax_key(key) -> bool:
    """``True`` for a JAX PRNG key (concrete or traced)."""
    return type(key).__module__.startswith("jax")


def generator(key) -> np.random.Generator:
    """A NumPy generator from a generator, an integer seed, or ``None``."""
    if isinstance(key, np.random.Generator):
        return key
    return np.random.default_rng(key)
