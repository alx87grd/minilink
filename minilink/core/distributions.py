"""Probability distributions: ``x ~ p`` with ``sample(key)`` on NumPy or JAX, ``mean()`` and an optional ``support``."""

import warnings
import zlib
from abc import ABC, abstractmethod
from dataclasses import dataclass

import numpy as np

from minilink.core.backends import (
    array_module,
    is_jax_key,
    jax_x64_policy,
    numpy_generator,
    require_jax,
)
from minilink.core.inspect import inspect_text, repr_pretty
from minilink.core.sets import BoxSet, Set

###############################################################################
# Public API
###############################################################################


class Distribution(ABC):
    """Vector-valued distribution: ``dim``, ``mean``, ``cov``, ``params`` and ``sample(key, n=None, params=None)``.

    A duck type, not a probability library: anything with ``dim``, ``sample(key)``
    and ``mean`` works; ``params`` and ``cov`` are the library laws'. ``key`` is a
    NumPy generator, an integer seed, a JAX PRNG key (JAX arrays out, traceable
    under ``jit`` / ``vmap`` / ``scan``), or, on the library laws, a
    :class:`CounterKey` ``(seed, k)``: one draw by the library's cipher, the same
    on both backends (what a noise source holds over its sample ``k``).
    The law's numbers are its ``params``; ``sample(key, n, params)`` draws from
    the law they name, so a noise level is a parameter like a mass (RD-R1).
    """

    support: Set | None = None
    mean: np.ndarray

    @property
    @abstractmethod
    def dim(self) -> int: ...

    # The law's numbers, by name (``{}`` for a law that exposes none)
    params = {}

    @property
    def cov(self) -> np.ndarray:
        """The covariance of one draw, ``(dim, dim)``, for a law that states it."""
        raise NotImplementedError(f"{type(self).__name__} states no covariance")

    def __str__(self):
        return inspect_text(self)

    def _repr_pretty_(self, p, cycle):
        repr_pretty(self, p, cycle)

    @abstractmethod
    def sample(self, key, n=None, params=None):
        """
        Draw one sample (shape ``(dim,)``) or ``n`` samples (``(n, dim)``).

        ``key`` is a :class:`numpy.random.Generator`, an integer seed, or a
        JAX PRNG key (JAX arrays out, traceable). ``params=None`` reads the
        law's own :attr:`params`.
        """
        ...


class Gaussian(Distribution):
    """Gaussian ``x ~ N(mean, Σ)``: diagonal from ``std`` (``Σ = diag(std²)``), or full from ``cov``."""

    def __init__(self, mean, std=None, cov=None):
        require_one_spread(std, cov)
        self.mean = np.asarray(mean, dtype=float).reshape(-1)
        if cov is None:
            self.std = np.broadcast_to(
                np.asarray(std, dtype=float), self.mean.shape
            ).copy()
            self.full_cov = None
        else:
            self.std = None
            self.full_cov = np.asarray(cov, dtype=float).reshape(self.dim, self.dim)

    @property
    def dim(self) -> int:
        return int(self.mean.size)

    @property
    def params(self) -> dict:
        """``{"mean", "std"}`` for a diagonal law, ``{"mean", "cov"}`` for a full one."""
        if self.full_cov is None:
            return {"mean": self.mean, "std": self.std}

        return {"mean": self.mean, "cov": self.full_cov}

    @property
    def cov(self) -> np.ndarray:
        if self.full_cov is None:
            return np.diag(self.std**2)

        return self.full_cov

    def sample(self, key, n=None, params=None):
        params = self.params if params is None else params

        mean = params["mean"]
        std = params.get("std")
        cov = params.get("cov")
        dim = self.dim

        shape = (dim,) if n is None else (int(n), dim)

        # ε ~ N(0, I), drawn on the key's backend (a counter key by the cipher)
        if is_counter_key(key):
            require_one_counter_draw(n)
            xp = array_module(key.seed, key.k, mean, std, cov)
            eps = standard_normal(key.seed, key.k, dim)
        elif is_jax_key(key):
            jax = require_jax()
            xp = jax.numpy
            eps = jax.random.normal(key, shape)
        else:
            xp = np
            eps = numpy_generator(key).standard_normal(shape)

        if cov is None:
            # x = mean + std ε
            x = mean + std * eps
        else:
            # x = mean + L ε, with Σ = L Lᵀ
            L = xp.linalg.cholesky(cov)
            x = mean + eps @ L.T

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

    @property
    def params(self) -> dict:
        """``{"lower", "upper"}``, the box's bounds."""
        return self.box.params

    @property
    def cov(self) -> np.ndarray:
        lower = self.box.lower
        upper = self.box.upper

        # Σ = diag((b − a)² / 12)
        cov = np.diag((upper - lower) ** 2 / 12.0)

        return cov

    def sample(self, key, n=None, params=None):
        return self.box.sample(key, n, params)


class Particles(Distribution):
    """Empirical distribution: a uniform choice among fixed points ``(N, dim)``."""

    def __init__(self, points):
        self.points = np.asarray(points, dtype=float)
        check_points(self.points)
        self.mean = self.points.mean(axis=0)

    @property
    def dim(self) -> int:
        return int(self.points.shape[1])

    @property
    def params(self) -> dict:
        """``{"points"}``, the ``(N, dim)`` support points."""
        return {"points": self.points}

    @property
    def cov(self) -> np.ndarray:
        points = self.points

        # The law's own covariance: Σ = (1/N) Σᵢ (pᵢ − p̄)(pᵢ − p̄)ᵀ
        cov = np.atleast_2d(np.cov(points, rowvar=False, bias=True))

        return cov

    def sample(self, key, n=None, params=None):
        params = self.params if params is None else params

        points = params["points"]
        count = points.shape[0]

        # A uniform index into the points: x = points[idx]
        if is_counter_key(key):
            require_one_counter_draw(n)
            xp = array_module(key.seed, key.k, points)
            u = standard_uniform(key.seed, key.k, 1)[0]
            idx = xp.minimum(xp.floor(u * count).astype(int), count - 1)
            x = xp.asarray(points)[idx]
        elif is_jax_key(key):
            jax = require_jax()
            xp = array_module(key)
            idx = jax.random.randint(key, () if n is None else (int(n),), 0, count)
            x = xp.asarray(points)[idx]
        else:
            idx = numpy_generator(key).integers(
                0, count, size=None if n is None else int(n)
            )
            x = points[idx]

        return x


###############################################################################
# Internal machinery
###############################################################################


@dataclass(frozen=True)
class CounterKey:
    """The key of one counter-based draw: the stream's ``seed`` and the sample index ``k``.

    A law sampled with it draws by the library's cipher (:func:`standard_normal`,
    :func:`standard_uniform`): a pure function of ``(seed, k)``, identical on NumPy
    and JAX and traceable in both.
    """

    seed: object
    k: object


def is_counter_key(key) -> bool:
    """Whether ``key`` is a :class:`CounterKey`."""
    return isinstance(key, CounterKey)


def require_one_counter_draw(n):
    """Refuse ``n`` draws from one counter key: it names one sample."""
    if n is not None:
        raise ValueError("a CounterKey names one sample: draw it with n=None")


def require_one_spread(std, cov):
    """Refuse a Gaussian given both ``std`` and ``cov``, or neither."""
    if (std is None) == (cov is None):
        raise ValueError("Gaussian takes one spread: std= (diagonal) or cov= (full)")


def check_points(points):
    """Refuse particles that are not a matrix of shape ``(N, dim)``."""
    if points.ndim != 2:
        raise ValueError("Particles needs points of shape (N, dim)")


def split_keys(key, n):
    """``n`` independent keys from a JAX key, or the same NumPy generator ``n`` times."""
    if n == 0:
        return []

    if is_jax_key(key):
        jax = require_jax()
        return list(jax.random.split(key, n))

    return [numpy_generator(key)] * n


# Counter-based random numbers: every draw is a pure function of (seed, sample, channel)

_MASK_32 = 0xFFFFFFFF  # One 32-bit word: arithmetic modulo 2³²


def threefry_2x32(key, count):
    """Threefry-2x32, 20 rounds: two words of bits from a key pair and a counter pair.

    ``key`` and ``count`` are pairs of 32-bit words: Python integers, or ``uint32``
    arrays of one shape. The rounds follow the Random123 schedule, so the bits match
    the published vectors and JAX's own generator; the mask keeps Python integers to
    32 bits and is a no-op on ``uint32`` arrays.
    """
    mask = _MASK_32
    rotations = (13, 15, 26, 6, 17, 29, 16, 24)

    k0, k1 = key
    c0, c1 = count

    # The key schedule: k₀, k₁ and their parity word k₂ = k₀ ⊕ k₁ ⊕ 0x1BD11BDA
    ks = (k0, k1, k0 ^ k1 ^ 0x1BD11BDA)

    # The first key injection: x = c + k
    x0 = (c0 + k0) & mask
    x1 = (c1 + k1) & mask

    for i in range(20):
        # One round: x₀ ← x₀ + x₁, x₁ ← (x₁ ⋘ r) ⊕ x₀
        r = rotations[i % 8]
        x0 = (x0 + x1) & mask
        x1 = ((x1 << r) | (x1 >> (32 - r))) & mask
        x1 = x0 ^ x1

        # A key injection after every fourth round: x₀ += k_{j mod 3}, x₁ += k_{(j+1) mod 3} + j
        if i % 4 == 3:
            j = i // 4 + 1
            x0 = (x0 + ks[j % 3]) & mask
            x1 = (x1 + ks[(j + 1) % 3] + j) & mask

    return x0, x1


def random_bits(seed, k, p):
    """Two ``(p,)`` words of bits for sample ``k`` of a ``p``-channel stream keyed by ``seed``.

    The key words are the seed's low and high halves, the counter words the sample
    index and the channel: every sample of every channel is one cipher block, and a
    negative index wraps the same way on both backends. On NumPy the rounds run on
    Python integers, one channel at a time; on JAX on ``uint32`` arrays under the trace.
    """
    xp = array_module(seed, k)

    # NumPy: Python integers, one cipher block per channel
    if xp is np:
        seed = int(seed)
        k = int(k)
        key = (seed & _MASK_32, (seed >> 32) & _MASK_32)
        words = [threefry_2x32(key, (k & _MASK_32, channel)) for channel in range(p)]
        bits_1 = xp.array([w[0] for w in words], dtype=xp.uint32)
        bits_2 = xp.array([w[1] for w in words], dtype=xp.uint32)
        return bits_1, bits_2

    # JAX: uint32 arrays, every channel in one block of rounds
    seed = xp.asarray(seed)
    k = xp.asarray(k)
    low = xp.broadcast_to(seed.astype(xp.uint32), (p,))
    high = xp.broadcast_to((seed >> 32).astype(xp.uint32), (p,))
    sample = xp.broadcast_to(k.astype(xp.uint32), (p,))
    channel = xp.arange(p, dtype=xp.uint32)

    return threefry_2x32((low, high), (sample, channel))


def standard_normal(seed, k, p):
    """``p`` independent standard normal draws for sample ``k`` of the stream keyed by ``seed``.

    Box–Muller on the two words of the cipher block, so the draw is a pure function of
    ``(seed, k)`` and identical on NumPy and JAX.
    """
    xp = array_module(seed, k)

    bits_1, bits_2 = random_bits(seed, k, p)

    # Two uniforms in (0, 1) from the top 24 bits of each word: uᵢ = (bitsᵢ ≫ 8) 2⁻²⁴ + 2⁻²⁵
    u_1 = (bits_1 >> 8).astype(float) * 2.0**-24 + 2.0**-25
    u_2 = (bits_2 >> 8).astype(float) * 2.0**-24 + 2.0**-25

    # Box–Muller: ε = √(−2 ln u₁) cos(2π u₂)
    eps = xp.sqrt(-2.0 * xp.log(u_1)) * xp.cos(2.0 * xp.pi * u_2)

    return eps


def standard_uniform(seed, k, p):
    """``p`` independent uniforms in ``(0, 1)`` for sample ``k`` of the stream keyed by ``seed``, the same on NumPy and JAX."""
    bits_1, _ = random_bits(seed, k, p)

    # u = (bits ≫ 8) 2⁻²⁴ + 2⁻²⁵, the top 24 bits of the first word
    u = (bits_1 >> 8).astype(float) * 2.0**-24 + 2.0**-25

    return u


def sample_index(t, period):
    """The index ``k`` of the sample held at time ``t``, the floor of ``t / period``.

    A relative tolerance absorbs a ``t`` accumulated step by step that lands a rounding
    error before a boundary. Returned as a 0-d integer array; the 32-bit JAX mode cannot
    hold the index past a few boundaries and is announced once.
    """
    xp = array_module(t, period)

    # The 32-bit JAX mode is announced, not refused
    if xp is not np and not jax_x64_policy():
        warnings.warn(
            "JAX runs in 32-bit floats (MINILINK_JAX_X64=0): the sample index of a held "
            "signal is wrong from the first boundaries; noise blocks need the 64-bit default",
            stacklevel=2,
        )

    # k = ⌊t / period⌋, with a relative tolerance of 10⁻¹⁰ against rounding
    s = xp.asarray(t) / period
    k = xp.floor(s + 1e-10 * (1.0 + xp.abs(s)))
    index = xp.asarray(k).astype(int)

    return index


def child_seed(seed, name):
    """The seed of the stream named ``name`` under ``seed``: one cipher word, 31 bits, both backends alike."""
    # The stream's sample index is the CRC-32 of its name
    counter = np.asarray(zlib.crc32(name.encode()))
    word, _ = random_bits(seed, counter, 1)

    # The top 31 bits of the first word
    child = int(word[0] >> 1)

    return child
