"""Linear-in-the-weights function approximation ``f_hat(x | w) = wᵀ φ(x)``.

The features ``φ(x)`` are fixed basis functions of the state — a quadratic
form about a point, Gaussian bumps on a mesh — and the weights ``w`` are the
only parameters. Fitting is then a least-squares problem on a batch, or one
stochastic-gradient step per sample: the two updates every approximate
dynamic-programming and value-learning method is built on.
"""

import numpy as np


class Features:
    """Basis functions ``φ(x)``; ``+`` stacks the bases of two feature sets."""

    n_features: int

    def phi(self, x):
        """Return the feature vector ``φ(x)`` of one state, shape ``(n_features,)``."""
        raise NotImplementedError

    def matrix(self, X):
        """Return the regression matrix ``Φ``, one row ``φ(x_i)ᵀ`` per state of ``X``."""
        X = np.atleast_2d(np.asarray(X, dtype=float))
        return np.array([self.phi(x) for x in X])

    def __add__(self, other):
        return ConcatenatedFeatures(self, other)


class QuadraticFeatures(Features):
    """Quadratic form about ``xbar``: ``φ(x) = [1, δx, δx_i δx_j (i ≤ j)]``, ``δx = x - xbar``.

    ``wᵀφ(x)`` spans every function ``c + bᵀδx + δxᵀ S δx``, the shape of an
    LQR cost-to-go; :meth:`quadratic_form` reads ``(c, b, S)`` back from ``w``.
    """

    def __init__(self, xbar):
        self.xbar = np.asarray(xbar, dtype=float).reshape(-1)
        n = self.xbar.size
        self.pairs = [(i, j) for i in range(n) for j in range(i, n)]
        self.n_features = 1 + n + len(self.pairs)

    def phi(self, x):
        xbar = self.xbar
        dx = np.asarray(x, dtype=float).reshape(-1) - xbar
        cross = [dx[i] * dx[j] for i, j in self.pairs]
        return np.concatenate(([1.0], dx, cross))

    def quadratic_form(self, w):
        """Return ``(c, b, S)`` such that ``wᵀφ(x) = c + bᵀδx + δxᵀ S δx``."""
        n = self.xbar.size
        w = np.asarray(w, dtype=float)
        c = float(w[0])
        b = w[1 : 1 + n]

        # one weight per pair (i, j): split the cross terms so S is symmetric
        S = np.zeros((n, n))
        for weight, (i, j) in zip(w[1 + n :], self.pairs):
            if i == j:
                S[i, i] = weight
            else:
                S[i, j] = S[j, i] = weight / 2.0
        return c, b, S


class RadialBasisFeatures(Features):
    """Gaussian bumps ``φ_i(x) = exp(-‖x - μ_i‖² / (2σ²))`` centred on ``centers``."""

    def __init__(self, centers, sigma):
        self.centers = np.atleast_2d(np.asarray(centers, dtype=float))
        self.sigma = float(sigma)
        self.n_features = self.centers.shape[0]

    @classmethod
    def on_grid(cls, lower, upper, shape, *, sigma=None):
        """Centres on a regular mesh of ``shape`` over the box ``[lower, upper]``.

        ``sigma`` defaults to the largest mesh spacing, so neighbouring bumps
        overlap and the sum can represent a smooth field.
        """
        lower = np.asarray(lower, dtype=float)
        upper = np.asarray(upper, dtype=float)
        shape = np.asarray(shape, dtype=int)
        levels = [np.linspace(lo, hi, k) for lo, hi, k in zip(lower, upper, shape)]
        mesh = np.meshgrid(*levels, indexing="ij")
        centers = np.stack(mesh, axis=-1).reshape(-1, lower.size)
        spacing = (upper - lower) / np.maximum(shape - 1, 1)
        return cls(centers, float(np.max(spacing)) if sigma is None else sigma)

    def phi(self, x):
        mu = self.centers
        sigma = self.sigma
        r2 = np.sum((np.asarray(x, dtype=float) - mu) ** 2, axis=1)
        return np.exp(-r2 / (2.0 * sigma**2))


class ConcatenatedFeatures(Features):
    """The bases of two feature sets stacked: ``φ(x) = [φ_1(x); φ_2(x)]``."""

    def __init__(self, first, second):
        self.first = first
        self.second = second
        self.n_features = first.n_features + second.n_features

    def phi(self, x):
        return np.concatenate((self.first.phi(x), self.second.phi(x)))


class LinearApproximator:
    """``f_hat(x | w) = wᵀ φ(x)``: fixed features, learned weights.

    :meth:`fit` solves the least-squares problem on a batch of samples and
    :meth:`sgd_step` applies one stochastic-gradient update on a single one;
    calling the approximator evaluates ``f_hat`` at a state or a batch.
    """

    def __init__(self, features, w=None):
        self.features = features
        self.w = (
            np.zeros(features.n_features)
            if w is None
            else np.asarray(w, dtype=float).copy()
        )

    def __call__(self, x):
        """Return ``f_hat(x)`` at one state ``x``, or at each row of a batch ``X``."""
        x = np.asarray(x, dtype=float)
        if x.ndim == 1:
            return float(self.w @ self.features.phi(x))
        return self.features.matrix(x) @ self.w

    def fit(self, X, y):
        """Least squares: set ``w`` to minimize ``Σ_i (y_i - wᵀφ(x_i))²`` and return it."""
        Phi = self.features.matrix(X)
        y = np.asarray(y, dtype=float)
        self.w = np.linalg.lstsq(Phi, y, rcond=None)[0]
        return self.w

    def sgd_step(self, x, y, eta):
        """One stochastic-gradient step ``w ← w + η (y - wᵀφ(x)) φ(x)``; returns ``w``."""
        w = self.w
        phi = self.features.phi(x)

        error = float(y) - w @ phi
        self.w = w + eta * error * phi
        return self.w
