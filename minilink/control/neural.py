"""
Neural state-feedback controllers.

:class:`NeuralPolicyController` is the law ``u = pi(x)`` that reinforcement
learning (or any policy search) trains: features of the state, a multilayer
perceptron, and a map from the network's normalized output to the input-port
bounds. It is a :class:`~minilink.core.feedback.Controller`, so
``ctl @ plant`` closes the loop, ``plot_control_law`` draws the law, and the
weights are ordinary ``params`` entries that a planner updates in place.
"""

import numpy as np

from minilink.blocks.neural import MLP
from minilink.core.backends import array_module
from minilink.core.feedback import Controller

# Public API


class NeuralPolicyController(Controller):
    """
    State feedback ``u = u_mid + u_half * clip(MLP(z(x)), -1, 1)``.

    Parameters
    ----------
    sys : System
        Plant whose state bounds and ``u`` port bounds define the feature
        normalization and the action range.
    features : callable, optional
        Observation map ``z = features(x)`` (must trace under JAX). Use it to
        make angles periodic or to give the policy task-relative sensing.
        Default: the state scaled by its bounds when ``normalize`` is true.
    hidden : tuple of int
        Hidden widths of the policy network.
    activation : {"tanh", "relu"}
        Hidden nonlinearity.
    normalize : bool
        Scale the default features by the state box (needs finite bounds).
    squash : {"clip", "tanh"}
        How the network output is kept in ``[-1, 1]``: clipped (PPO family)
        or squashed by ``tanh`` (SAC family).
    seed : int
        Weight initialization seed.

    The trainable weights are ``params["mlp"]``; the normalized-action range
    ``[-1, 1]`` maps onto the ``u`` port bounds, which is what makes
    exploration cover the whole input range from the first step.
    """

    # Explicit port roles: state measurement, no boundary reference.
    measurement_port = "x"
    ref_port = None
    control_port = "u"
    plot_space = "state"

    def __init__(
        self,
        sys,
        *,
        features=None,
        hidden=(64, 64),
        activation="tanh",
        normalize=True,
        squash="clip",
        seed=0,
        name="Neural Policy Controller",
    ):
        super().__init__()
        self.name = name
        if squash not in ("clip", "tanh"):
            raise ValueError(f"squash must be 'clip' or 'tanh', got {squash!r}")
        self.squash = squash
        n, m = int(sys.n), int(sys.inputs["u"].dim)
        x_lb = np.asarray(sys.state.lower_bound, dtype=float)
        x_ub = np.asarray(sys.state.upper_bound, dtype=float)
        u_lb = np.asarray(sys.inputs["u"].lower_bound, dtype=float)
        u_ub = np.asarray(sys.inputs["u"].upper_bound, dtype=float)
        if not (np.all(np.isfinite(u_lb)) and np.all(np.isfinite(u_ub))):
            raise ValueError(
                "NeuralPolicyController needs finite bounds on sys.inputs['u']"
            )

        # Features: user map, or the state scaled to the box [-1, 1]
        self.features = features
        finite_box = bool(np.all(np.isfinite(x_lb)) and np.all(np.isfinite(x_ub)))
        self.normalize = bool(normalize) and features is None and finite_box
        width = x_ub - x_lb
        self.x_mid = 0.5 * (x_ub + x_lb)
        self.x_half = np.where(width > 0, 0.5 * width, 1.0)

        # Normalized action a in [-1, 1] spans the input-port bounds
        self.u_mid = 0.5 * (u_ub + u_lb)
        self.u_half = 0.5 * (u_ub - u_lb)

        n_features = int(
            np.asarray(self.observe(np.asarray(sys.x0, dtype=float))).shape[0]
        )
        self.mlp = MLP(n_features, m, hidden, activation, seed=seed, output_gain=0.01)
        self.params = {"mlp": self.mlp.params}

        self.add_input_port(
            "x",
            dim=n,
            labels=list(sys.state.labels),
            units=list(sys.state.units),
            lower_bound=x_lb,
            upper_bound=x_ub,
        )
        self.add_output_port(
            "u",
            dim=m,
            function=self.ctl,
            dependencies=("x",),
            labels=list(sys.inputs["u"].labels),
            units=list(sys.inputs["u"].units),
            lower_bound=u_lb,
            upper_bound=u_ub,
        )

    def observe(self, x):
        """Features ``z(x)`` fed to the network."""
        if self.features is not None:
            return self.features(x)
        if self.normalize:
            return (x - self.x_mid) / self.x_half
        return x

    def mean_action(self, x, params=None):
        """Normalized network output ``a = MLP(z(x))`` (unclipped)."""
        params = self.params if params is None else params
        return self.mlp.compute(None, self.observe(x), params=params["mlp"])

    def action(self, x, params=None):
        """Plant input ``u = u_mid + u_half * squash(a)`` with ``squash`` clip or tanh."""
        xp = array_module(x)
        a = self.mean_action(x, params)
        a = xp.tanh(a) if self.squash == "tanh" else xp.clip(a, -1.0, 1.0)
        return self.u_mid + self.u_half * a

    def ctl(self, x, u, t=0, params=None):
        """State feedback; the ``x`` input port carries the plant state in ``u``."""
        return self.action(u, params)


def angle_features(angles, scales=None):
    """
    Feature map making the listed state angles periodic: ``cos, sin`` per angle.

    ``angles`` are state indices; every other component is passed through,
    multiplied by ``scales`` (a dict ``{index: gain}``, e.g. ``{1: 0.1}`` to
    bring a rate to order one). Works on NumPy and traces under JAX.
    """
    angles = tuple(int(i) for i in angles)
    scales = {} if scales is None else {int(k): float(v) for k, v in scales.items()}

    def features(x):
        xp = array_module(x)
        parts = []
        for i in range(int(x.shape[0])):
            if i in angles:
                parts += [xp.cos(x[i]), xp.sin(x[i])]
            else:
                parts.append(scales.get(i, 1.0) * x[i])
        return xp.stack(parts)

    return features
