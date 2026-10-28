"""The seeded stochastic policy: a law plus its own counter-based noise, a pure block."""

import math

import numpy as np

from minilink.core.backends import array_module
from minilink.core.distributions import child_seed, sample_index, standard_normal
from minilink.core.system import System

###############################################################################
# Public API
###############################################################################


class GaussianPolicy(System):
    """
    A seeded Gaussian law, ``u = μ_θ(x) + σ_θ(x) · ε(t)``: its noise internal and deterministic given its seed.

    ``ε(t) = F(seed, ⌊t/Δ⌋)`` is the standard normal signal a
    :class:`~minilink.blocks.sources.WhiteNoise` draws, the library's cipher at
    the sample index of the block's own ``sample_period`` ``Δ``, so the block
    is pure, holds each draw over its period, traces, and draws the same on
    NumPy and JAX. ``mean`` is the deterministic law, a static block (a
    :class:`~minilink.control.neural.NeuralPolicy`, a gain); ``log_std`` is the
    log standard deviation per channel, an array (a leaf) or a static block of
    the same inputs (``σ_θ(x)``). ``seed=None`` is the deterministic face,
    ``u = μ(x)``; ``seed`` is an integer leaf of :attr:`params`
    (``{"mean", "log_std", "seed", "sample_period"}``) that :meth:`realize` and
    a problem's realization draw per trial. :meth:`log_prob` and
    :meth:`entropy` are the law's, beside ``h``.
    """

    def __init__(self, mean, log_std, *, sample_period, seed=None):
        require_static(mean, "mean")
        require_positive_period(sample_period)
        require_seed(seed)

        System.__init__(self, 0)
        self.name = f"GaussianPolicy({mean.name})"

        # The laws: the mean, and the spread as numbers or as a law of the same inputs
        self.mean = mean
        self.command = command_of(mean)
        m = mean.outputs[self.command].dim
        self.spread = log_std if isinstance(log_std, System) else None
        if self.spread is not None:
            require_static(self.spread, "log_std")

        # Parameters: the two laws' numbers, the realization's seed, the period Δ
        self.params = {
            "mean": mean.params,
            "log_std": log_std.params
            if self.spread is not None
            else np.broadcast_to(np.asarray(log_std, dtype=float), (m,)).copy(),
            "seed": seed,
            "sample_period": float(sample_period),
        }

        # Ports: the mean's inputs, the command out
        self.inputs = mean.inputs
        self.add_output_port(
            "u",
            dim=m,
            function=self.ctl,
            dependencies=mean.outputs[self.command].dependencies,
        )

    def h(self, x, u, t=0, params=None):
        """The command, the block's one output: :meth:`ctl`."""
        return self.ctl(x, u, t, params)

    def ctl(self, x, u, t=0, params=None):
        """The command at ``t``: the mean plus the held draw scaled by the spread (the mean when ``seed`` is ``None``)."""
        params = self.params if params is None else params

        seed = params["seed"]
        period = params["sample_period"]
        m = self.outputs["u"].dim

        mu, sigma = self.mean_and_std(x, u, t, params)

        # The deterministic face: no seed, the mean
        if seed is None:
            return mu

        # ε_k = F(seed, k), k = ⌊t/Δ⌋: the draw held over the period
        k = sample_index(t, period)
        eps = standard_normal(seed, k, m)

        # u = μ + σ ε
        u_cmd = mu + sigma * eps

        return u_cmd

    def log_prob(self, u, action, t=0, params=None):
        """``ln π(action | u)``: the Gaussian log density of an action at the policy's input ``u``."""
        params = self.params if params is None else params
        xp = array_module(u, action)

        mu, sigma = self.mean_and_std(xp.zeros(0), u, t, params)

        # ln π = −½ Σ ((a − μ)/σ)² − Σ ln σ − (m/2) ln 2π
        z = (action - mu) / sigma
        log_pi = (
            -0.5 * xp.sum(z**2)
            - xp.sum(xp.log(sigma))
            - 0.5 * mu.size * math.log(2.0 * math.pi)
        )

        return log_pi

    def entropy(self, u, t=0, params=None):
        """The law's entropy at the policy's input ``u``: ``Σ ln σ + (m/2) ln 2πe``."""
        params = self.params if params is None else params
        xp = array_module(u)

        mu, sigma = self.mean_and_std(xp.zeros(0), u, t, params)

        # H = Σ ln σᵢ + (m/2) ln(2π e)
        entropy = xp.sum(xp.log(sigma)) + 0.5 * mu.size * math.log(
            2.0 * math.pi * math.e
        )

        return entropy

    def realize(self, key):
        """The params of one realization: a fresh seed under ``key``, or the mean for ``None``."""
        seed = None if key is None else child_seed(key, "seed")
        return dict(self.params, seed=seed)

    @property
    def is_random(self):
        """A seed draws; ``seed=None`` is the deterministic face ``u = μ(x)``, which no evaluation reseeds."""
        return self.params["seed"] is not None

    @property
    def solver_info(self) -> dict:
        """The solver hints, the sample period among them, read from the current ``params``."""
        period = float(self.params["sample_period"])

        info = self.own_solver_info
        info["sample_period"] = period
        info["smallest_time_constant"] = period

        return info

    @solver_info.setter
    def solver_info(self, value: dict):
        self.own_solver_info = dict(value)

    ###########################################################################
    # Internal machinery
    ###########################################################################

    def mean_and_std(self, x, u, t, params):
        """``(μ, σ)`` at the policy's input ``u``: the mean law's command and the spread's exponential."""
        xp = array_module(u)
        nothing = xp.zeros(0)

        mean = self.mean
        spread = self.spread

        mu = mean.outputs[self.command].compute(nothing, u, t, params["mean"])
        if spread is None:
            log_std = params["log_std"]
        else:
            log_std = spread.outputs[command_of(spread)].compute(
                nothing, u, t, params["log_std"]
            )

        # σ = exp(ln σ)
        sigma = xp.exp(xp.asarray(log_std))

        return mu, sigma


class SquashedGaussianPolicy(GaussianPolicy):
    """
    A seeded squashed Gaussian law, ``u = c + r · tanh(μ_θ(x) + σ_θ(x) · ε(t))``, inside the box ``[lower, upper]``.

    ``c`` and ``r`` are the box's center and half-width; the draw is the
    :class:`GaussianPolicy`'s, and :meth:`log_prob` includes the change of
    variables ``−Σ ln(r (1 − tanh²))``. Its entropy has no closed form.
    """

    def __init__(self, mean, log_std, *, lower, upper, sample_period, seed=None):
        super().__init__(mean, log_std, sample_period=sample_period, seed=seed)
        self.name = f"SquashedGaussianPolicy({mean.name})"

        # The action box: its center and half-width, structure of the squash
        lower = np.asarray(lower, dtype=float).reshape(-1)
        upper = np.asarray(upper, dtype=float).reshape(-1)
        self.center = 0.5 * (lower + upper)
        self.half_width = 0.5 * (upper - lower)

    def ctl(self, x, u, t=0, params=None):
        """The squashed command: ``c + r tanh(z)``, ``z`` the Gaussian law's command."""
        xp = array_module(u)

        c = self.center
        r = self.half_width

        z = super().ctl(x, u, t, params)

        # u = c + r tanh(z)
        u_cmd = c + r * xp.tanh(z)

        return u_cmd

    def log_prob(self, u, action, t=0, params=None):
        """``ln π(action | u)``: the Gaussian's at ``z = artanh((a − c)/r)``, minus ``Σ ln(r (1 − tanh² z))``."""
        xp = array_module(u, action)

        c = self.center
        r = self.half_width

        # z = artanh((a − c)/r), the action clipped inside the open box
        s = xp.clip((action - c) / r, -1.0 + 1e-6, 1.0 - 1e-6)
        z = xp.arctanh(s)

        # ln π(a) = ln π_z(z) − Σ ln(r (1 − s²))
        log_pi = super().log_prob(u, z, t, params) - xp.sum(xp.log(r * (1.0 - s**2)))

        return log_pi

    def entropy(self, u, t=0, params=None):
        """Refuse: a squashed Gaussian's entropy has no closed form."""
        raise NotImplementedError(
            "a squashed Gaussian has no closed-form entropy; estimate it as -ln pi of samples"
        )


###############################################################################
# Internal machinery
###############################################################################


def command_of(law):
    """The law's command port: ``u``, else its one output."""
    if "u" in law.outputs:
        return "u"
    require_one_output(law)

    return next(iter(law.outputs))


def require_one_output(law):
    """Refuse a law with no ``u`` and several outputs: its command is ambiguous."""
    if len(law.outputs) != 1:
        raise ValueError(
            f"{law.name} has no output u and {len(law.outputs)} outputs: name its command u"
        )


def require_static(law, label):
    """Refuse a law with a state: the policy's ``h`` is pure in its input."""
    if law.n != 0:
        raise TypeError(f"the {label} law {law.name} has a state; give a static law")


def require_positive_period(sample_period):
    """Refuse a sample period that is not positive."""
    if float(sample_period) <= 0.0:
        raise ValueError("sample_period must be > 0")


def require_seed(seed):
    """Refuse a negative seed (``None`` is the mean)."""
    if seed is not None and int(seed) < 0:
        raise ValueError(f"seed must be a non-negative integer or None, got {seed!r}")
