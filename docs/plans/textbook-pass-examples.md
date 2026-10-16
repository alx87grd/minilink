# The textbook pass: four target examples

Four modules as they are on `dev` today and as the pass would write them, under the style
sheet of [textbook-pass.md](textbook-pass.md) §3. Every target computes the same numbers in
the same order (byte-identical), with the same backend types.

---

## 1. A plant: `TwoLinkManipulator` (`dynamics/catalog/manipulators/arms.py`)

**On `dev` today** (abridged): one parameter per line, `array_module(...)` inside the math
line, and the gravity and damping vectors computed inside the array constructor.

```python
    def g(self, q, params=None):
        params = self.params if params is None else params
        _, s1, _, _, _, s12 = self.trig(q)

        m1 = params["m1"]
        m2 = params["m2"]
        l1 = params["l1"]
        lc1 = params["lc1"]
        lc2 = params["lc2"]
        gravity = params["gravity"]

        # gravity torque on each joint
        g1 = (m1 * lc1 + m2 * l1) * gravity
        g2 = m2 * lc2 * gravity
        g = array_module(q).array([-g1 * s1 - g2 * s12, -g2 * s12])

        return g
```

**Target:** the unpack beat groups related names and binds `xp` last. Every entry is a named
math line, and the last beat assembles them: a vector on the `return`, a matrix row by row.

```python
    def H(self, q, params=None):
        params = self.params if params is None else params
        m1, m2 = params["m1"], params["m2"]
        l1, lc1, lc2 = params["l1"], params["lc1"], params["lc2"]
        I1, I2 = params["I1"], params["I2"]
        _, _, c2, _, _, _ = self.trig(q)
        xp = array_module(q)

        # The coupled inertia of the two planar links
        H11 = m1 * lc1**2 + I1 + m2 * (l1**2 + lc2**2 + 2.0 * l1 * lc2 * c2) + I2
        H12 = m2 * lc2**2 + m2 * l1 * lc2 * c2 + I2
        H22 = m2 * lc2**2 + I2

        # fmt: off
        H = xp.array([
            [H11, H12],
            [H12, H22],
        ])
        # fmt: on

        return H

    def C(self, q, dq, params=None):
        params = self.params if params is None else params
        m2, l1, lc2 = params["m2"], params["l1"], params["lc2"]
        _, _, _, s2, _, _ = self.trig(q)
        xp = array_module(q, dq)

        # Coriolis and centrifugal coupling, driven by the second joint's rate
        h = m2 * l1 * lc2 * s2
        C11 = -h * dq[1]
        C12 = -h * (dq[0] + dq[1])
        C21 = h * dq[0]

        # fmt: off
        C = xp.array([
            [C11, C12],
            [C21, 0.0],
        ])
        # fmt: on

        return C

    def g(self, q, params=None):
        params = self.params if params is None else params
        m1, m2 = params["m1"], params["m2"]
        l1, lc1, lc2 = params["l1"], params["lc1"], params["lc2"]
        gravity = params["gravity"]
        _, s1, _, _, _, s12 = self.trig(q)
        xp = array_module(q)

        # The gravity torque on each joint
        g1 = (m1 * lc1 + m2 * l1) * gravity
        g2 = m2 * lc2 * gravity
        tau_g1 = -g1 * s1 - g2 * s12
        tau_g2 = -g2 * s12

        return xp.array([tau_g1, tau_g2])

    def d(self, q, dq, u=None, t=0.0, params=None):
        params = self.params if params is None else params
        d1, d2 = params["d1"], params["d2"]
        xp = array_module(dq)

        # Linear viscous joint damping, τ_d = diag(d₁, d₂) q̇
        tau_d1 = d1 * dq[0]
        tau_d2 = d2 * dq[1]

        return xp.array([tau_d1, tau_d2])

    def J(self, q, params=None):
        l1, l2 = self.params["l1"], self.params["l2"]
        c1, s1, _, _, c12, s12 = self.trig(q)
        xp = array_module(q)

        J11 = l1 * c1 + l2 * c12
        J12 = l2 * c12
        J21 = -l1 * s1 - l2 * s12
        J22 = -l2 * s12

        # fmt: off
        J = xp.array([
            [J11, J12],
            [J21, J22],
        ])
        # fmt: on

        return J
```

`J` and `forward_kinematics` read `self.params` rather than their `params` argument. That
is a bug (recorded, not fixed), so the target keeps reading `self.params` exactly as today.

---

## 2. An algorithm: `PPO.loss` (`planning/reinforcement_learning/algorithms/ppo.py`)

**On `dev` today:** `self.` in the math (`self.clip_range`, `self.vf_coef`, `self.ent_coef`),
minibatch fields read inside the math lines, and ASCII comments that do not match the code
(`rho` names the discount rate elsewhere).

```python
        # Clipped surrogate: L_clip = E[min(rho A, clip(rho, 1 - eps, 1 + eps) A)]
        eps = self.clip_range
        clipped = jnp.clip(ratio, 1.0 - eps, 1.0 + eps)
        policy_loss = -jnp.mean(jnp.minimum(ratio * advantage, clipped * advantage))

        # Value regression E[(R - V_w(x))^2], and the entropy bonus H(pi)
        value_loss = jnp.mean((minibatch["return"] - V.value(w, x)) ** 2)
        entropy = jnp.mean(pi.entropy(theta, x))

        total = policy_loss + self.vf_coef * value_loss - self.ent_coef * entropy
```

**Target:**

```python
    def loss(self, params, minibatch):
        """Clipped surrogate, plus the weighted value loss, minus the entropy bonus, on one minibatch."""
        jnp = require_jax_numpy()
        pi, V = self.policy, self.critic
        eps, c_v, c_e = self.clip_range, self.vf_coef, self.ent_coef
        theta, w = params["policy"], params["critic"]
        x, a, logp_old = minibatch["x"], minibatch["a"], minibatch["logp"]
        advantage, returns = minibatch["advantage"], minibatch["return"]

        # ratio = π_θ(a|x) / π_old(a|x), on the stored actions
        ratio = jnp.exp(pi.log_prob(theta, x, a) - logp_old)

        # Â = (A − mean A) / (std A + 10⁻⁸)
        advantage = (advantage - advantage.mean()) / (advantage.std() + 1e-8)

        # L_clip = −E[min(ratio Â, clip(ratio, 1 − ε, 1 + ε) Â)]
        clipped = jnp.clip(ratio, 1.0 - eps, 1.0 + eps)
        L_clip = -jnp.mean(jnp.minimum(ratio * advantage, clipped * advantage))

        # L_V = E[(R − V_w(x))²], and the entropy bonus H(π_θ)
        L_V = jnp.mean((returns - V.value(w, x)) ** 2)
        entropy = jnp.mean(pi.entropy(theta, x))

        # L = L_clip + c_v L_V − c_e H
        L = L_clip + c_v * L_V - c_e * entropy

        # Monitoring only: an estimate of how far the update moved the policy
        approx_kl = jnp.mean((ratio - 1.0) - jnp.log(ratio))
        stats = {
            "policy_loss": L_clip,
            "value_loss": L_V,
            "entropy": entropy,
            "approx_kl": approx_kl,
        }

        return L, stats
```

---

## 3. A law: `PID.ctl` (`control/siso.py`)

**On `dev` today:** the law `u = Kp e + Ki e_int − Kd ṁ_filt` lives in a helper method,
`PID.command`, which `f` and `ctl` both call. The module ends with a banner. Its three
shared pieces (`filtered_rate`, `held_integral`, `clipped`) are module functions by the
S73 ruling, and they stay as they are.

```python
    def ctl(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        xp = array_module(x)
        n_int = self.n_int
        e = self.error(u)
        e_int, m_filt = x[:n_int], x[n_int:]
        dm_filt = None
        if self.has_filter:
            dm_filt = filtered_rate(self.derivative_signal(u), m_filt, params, xp)

        # u = Kp e + Ki e_int − Kd ṁ_filt, then the actuator limits
        u_cmd = self.command(e, e_int, dm_filt, params, xp)
        u = clipped(u_cmd, params, xp)

        return u
```

**Target**, with the law in the body (RULES 5.3: "if a helper contains an equation from the
textbook page, it is in the wrong place"):

```python
    def ctl(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        n_int, has_integrator, has_filter = self.n_int, self.has_integrator, self.has_filter
        e = self.error(u)
        e_int, m_filt = x[:n_int], x[n_int:]
        xp = array_module(x)
        Kp = xp.asarray(params["Kp"])
        Ki = xp.asarray(params["Ki"]) if has_integrator else None
        Kd = xp.asarray(params["Kd"]) if has_filter else None

        # τ ṁ_filt = m − m_filt: the filter whose state approximates ṁ
        dm_filt = None
        if has_filter:
            dm_filt = filtered_rate(self.derivative_signal(u), m_filt, params, xp)

        # u = Kp e + Ki e_int − Kd ṁ_filt, each term only when the block carries its state
        u_cmd = Kp * e
        if has_integrator:
            u_cmd = u_cmd + Ki * e_int
        if has_filter:
            u_cmd = u_cmd - Kd * dm_filt

        # u_min ≤ u ≤ u_max
        u = clipped(u_cmd, params, xp)

        return u
```

`f` shows the same law, since its anti-windup reads the unsaturated command. The law is
then written twice, in `f` and `ctl`, instead of once in `command`. This is the one place
where "math in the body" and "one owner" pull against each other (see the question at the
end).

The module's last section loses its banner:

```python
# =============================================================================
# Internal machinery: the pieces every member of the family shares
# =============================================================================
```

becomes

```python
# Internal machinery


# The pieces every member of the family shares
```

---

## 4. A module's shape: `planning/search/dubins.py`

**On `dev` today:** a twelve-line module docstring, the underscore helpers `_mod2pi` and
`_words` above the public function, and the public function far below the first screen.

**Target:** a one-line title, the module's one note moved above the constants it explains,
`# Public API` with the primary function first, and plain-named helpers below in the order
the body calls them.

```python
"""Dubins paths: the shortest path of a forward-only car with a minimum turning radius."""

import math

# The geometry is scalar, so it runs on ``math`` rather than NumPy: it is the
# nearest-neighbour metric of a steering RRT, evaluated against every node.
_2PI = 2.0 * math.pi
_SIGN = {"L": 1.0, "S": 0.0, "R": -1.0}

# Public API


def shortest_path(q0, q1, radius):
    """
    Return ``(word, segments, length)`` for the shortest of the six Dubins words.

    ``segments`` lists ``(curvature_sign, arc_length)``, the sign in ``{+1, 0, -1}``
    (left, straight, right); ``length`` is the total arc length. ``None`` if degenerate.
    """
    # The normalised frame: the start-to-goal line as the x axis, a unit radius
    dx, dy = q1[0] - q0[0], q1[1] - q0[1]
    d = math.hypot(dx, dy) / radius
    theta = mod2pi(math.atan2(dy, dx))
    a = mod2pi(q0[2] - theta)
    b = mod2pi(q1[2] - theta)

    # The shortest of the feasible words
    ...

    return best_word, segments, length


# Internal machinery


def word_segments(a, b, d):
    """Segment triples ``(t, p, q)``, normalised, of each feasible word."""
    ...


def mod2pi(angle):
    """``angle`` wrapped to ``[0, 2π)``."""
    return angle % _2PI
```

The private module constants keep their underscore (RULES 5.8).

---

## Question raised by example 3

When a textbook law is needed by two methods (the PID's `f` and `ctl`, the impedance pair),
which rule wins?

- **Math in the body:** the law is written in each method's body, as above, at the cost of a
  second copy.
- **One owner:** the law stays in one helper (`PID.command`), and the bodies call it.
