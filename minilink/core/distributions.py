"""Probability distributions: ``x ~ p`` with ``sample(key)`` on NumPy or JAX, ``mean()`` and an optional ``support``.

A duck type, not a probability library: anything with ``dim``, ``sample(key)`` and
``mean()`` works. ``key`` is a NumPy generator, an integer seed, or a JAX PRNG key
(JAX arrays out, traceable under ``jit`` / ``vmap`` / ``scan``).
"""

from abc import ABC, abstractmethod

import numpy as np

from minilink.core.backends import (
    array_module,
    is_jax_key,
    numpy_generator,
    require_jax,
)
from minilink.core.sets import BoxSet, Set

# Public API


class Distribution(ABC):
    """Vector-valued distribution: ``dim``, ``mean`` (an array), ``sample(key, n=None)``, optional ``support``."""

    support: Set | None = None
    mean: np.ndarray

    @property
    @abstractmethod
    def dim(self) -> int: ...

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
        self.mean = np.asarray(mean, dtype=float).reshape(-1)
        self.std = np.broadcast_to(np.asarray(std, dtype=float), self.mean.shape).copy()

    @property
    def dim(self) -> int:
        return int(self.mean.size)

    def sample(self, key, n=None):
        mean, std = self.mean, self.std
        shape = (self.dim,) if n is None else (int(n), self.dim)

        # x = mean + std * eps, eps ~ N(0, I)
        if is_jax_key(key):
            jax = require_jax()
            x = mean + std * jax.random.normal(key, shape)
        else:
            x = mean + std * numpy_generator(key).standard_normal(shape)

        return x


class Uniform(Distribution):
    """Uniform law on the box ``lower <= x <= upper``; ``support`` is that box."""

    def __init__(self, lower, upper):
        self.box = BoxSet(lower, upper)
        self.support = self.box
        self.mean = 0.5 * (self.box.lower + self.box.upper)

    @property
    def dim(self) -> int:
        return self.box.dim

    def sample(self, key, n=None):
        return self.box.sample(key, n)


class Particles(Distribution):
    """Empirical distribution: a uniform choice among fixed points ``(N, dim)``."""

    def __init__(self, points):
        self.points = np.asarray(points, dtype=float)
        if self.points.ndim != 2:
            raise ValueError("Particles needs points of shape (N, dim)")
        self.mean = self.points.mean(axis=0)

    @property
    def dim(self) -> int:
        return int(self.points.shape[1])

    def sample(self, key, n=None):
        points = self.points
        count = points.shape[0]
        if is_jax_key(key):
            jax = require_jax()
            xp = array_module(key)
            idx = jax.random.randint(key, () if n is None else (int(n),), 0, count)
            return xp.asarray(points)[idx]
        idx = numpy_generator(key).integers(
            0, count, size=None if n is None else int(n)
        )
        return points[idx]


class Sampler(Distribution):
    """
    Distribution defined by a sampling function ``draw(key) -> x``.

    For task-specific starts (a random point along a track, a random pose in
    free space). ``draw`` must accept a JAX key and trace; ``mean`` is the
    representative point given explicitly.
    """

    def __init__(self, draw, mean, support: Set | None = None):
        self.draw = draw
        self.mean = np.asarray(mean, dtype=float).reshape(-1)
        self.support = support

    @property
    def dim(self) -> int:
        return int(self.mean.size)

    def sample(self, key, n=None):
        draw = self.draw
        if n is None:
            return draw(key)
        if is_jax_key(key):
            jax = require_jax()
            return jax.vmap(draw)(jax.random.split(key, int(n)))
        rng = numpy_generator(key)
        return np.stack([np.asarray(draw(rng)) for _ in range(int(n))])


# Internal machinery


def split_keys(key, n):
    """``n`` independent keys from a JAX key, or the same NumPy generator ``n`` times."""
    if n == 0:
        return []
    if is_jax_key(key):
        jax = require_jax()
        return list(jax.random.split(key, n))
    return [numpy_generator(key)] * n
