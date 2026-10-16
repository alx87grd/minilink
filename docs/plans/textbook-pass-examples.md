# The textbook pass: target examples

Real modules as they are on `dev` today and as the pass would write them, under the style
sheet of [textbook-pass.md](textbook-pass.md) §3 (revised with the maintainer's
preferences of 2026-10-16). Sections 1–5 are equation paths; 6–9 are constructors, public
verbs, iterative algorithms and class layout. Every target computes the same numbers in the same order, with
the same backend types.

**The fixed pattern of an equation path.** Each block is separated by a blank line, and a
block with nothing to say is left out:

1. the `params = …` line, then `xp = array_module(...)`;
2. the parameters, one per line;
3. the pre-computation (state unpack, shorthands, intermediates);
4. the core equation(s);
5. the matrix assembly, for a complex array only;
6. `return` of a name.

---

## 1. `Pendulum` (`dynamics/catalog/pendulum/pendulum.py`)

**Before (`dev`):**

```python
    def H(self, q, params=None):
        params = self.params if params is None else params
        m = params["m"]
        l = params["l"]
        I = params["I"]
        xp = array_module(q)

        H = xp.array([[m * l**2 + I]])

        return H

    def g(self, q, params=None):
        params = self.params if params is None else params
        m = params["m"]
        l = params["l"]
        gravity = params["gravity"]
        xp = array_module(q)
        theta = q[0]

        g = xp.array([m * gravity * l * xp.sin(theta)])

        return g

    def d(self, q, dq, u=None, t=0.0, params=None):
        params = self.params if params is None else params
        d = params["d"]
        xp = array_module(q)
        omega = dq[0]

        tau_d = xp.array([d * omega])

        return tau_d
```

**After:** `xp` moves up to the second line, the parameters form their own block, and the
state unpack gets its own line. Each array is simple, so it is built on its equation line.

```python
    def H(self, q, params=None):
        params = self.params if params is None else params
        xp = array_module(q)

        m = params["m"]
        l = params["l"]
        I = params["I"]

        H = xp.array([[m * l**2 + I]])

        return H

    def g(self, q, params=None):
        params = self.params if params is None else params
        xp = array_module(q)

        m = params["m"]
        l = params["l"]
        gravity = params["gravity"]

        theta = q[0]

        g = xp.array([m * gravity * l * xp.sin(theta)])

        return g

    def d(self, q, dq, u=None, t=0.0, params=None):
        params = self.params if params is None else params
        xp = array_module(q)

        d = params["d"]

        omega = dq[0]

        tau_d = xp.array([d * omega])

        return tau_d
```

---

## 2. `TwoLinkManipulator` (`dynamics/catalog/manipulators/arms.py`)

**Before (`dev`):**

```python
    def H(self, q, params=None):
        params = self.params if params is None else params
        _, _, c2, _, _, _ = self.trig(q)

        l1 = params["l1"]
        lc1 = params["lc1"]
        lc2 = params["lc2"]
        m1 = params["m1"]
        m2 = params["m2"]
        I1 = params["I1"]
        I2 = params["I2"]

        # coupled inertia of the two planar links
        h11 = m1 * lc1**2 + I1 + m2 * (l1**2 + lc2**2 + 2.0 * l1 * lc2 * c2) + I2
        h12 = m2 * lc2**2 + m2 * l1 * lc2 * c2 + I2
        h22 = m2 * lc2**2 + I2
        # fmt: off
        H = array_module(q).array([
            [h11, h12],
            [h12, h22],
        ])
        # fmt: on

        return H

    def C(self, q, dq, params=None):
        params = self.params if params is None else params
        _, _, _, s2, _, _ = self.trig(q)

        m2 = params["m2"]
        l1 = params["l1"]
        lc2 = params["lc2"]

        # Coriolis/centrifugal coupling driven by the second joint rate
        h = m2 * l1 * lc2 * s2
        # fmt: off
        C = array_module(q, dq).array([
            [-h * dq[1], -h * (dq[0] + dq[1])],
            [ h * dq[0],                  0.0],
        ])
        # fmt: on

        return C

    def J(self, q, params=None):
        l1 = self.params["l1"]
        l2 = self.params["l2"]
        c1, s1, _, _, c12, s12 = self.trig(q)
        # fmt: off
        J = array_module(q).array([
            [ l1 * c1 + l2 * c12,  l2 * c12],
            [-l1 * s1 - l2 * s12, -l2 * s12],
        ])
        # fmt: on

        return J
```

**After:** the same blocks in the fixed order. The complex matrices get named entries
(`H11`, `C12`, `J21`, …) and their own assembly block. The simple vectors (`g`, `d`) stay
on their equation line.

```python
    def H(self, q, params=None):
        params = self.params if params is None else params
        xp = array_module(q)

        l1 = params["l1"]
        lc1 = params["lc1"]
        lc2 = params["lc2"]
        m1 = params["m1"]
        m2 = params["m2"]
        I1 = params["I1"]
        I2 = params["I2"]

        _, _, c2, _, _, _ = self.trig(q)

        # Coupled inertia of the two planar links
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
        xp = array_module(q, dq)

        m2 = params["m2"]
        l1 = params["l1"]
        lc2 = params["lc2"]

        _, _, _, s2, _, _ = self.trig(q)
        h = m2 * l1 * lc2 * s2

        # Coriolis and centrifugal coupling, driven by the second joint's rate
        C11 = -h * dq[1]
        C12 = -h * (dq[0] + dq[1])
        C21 = h * dq[0]
        C22 = 0.0

        # fmt: off
        C = xp.array([
            [C11, C12],
            [C21, C22],
        ])
        # fmt: on

        return C

    def g(self, q, params=None):
        params = self.params if params is None else params
        xp = array_module(q)

        m1 = params["m1"]
        m2 = params["m2"]
        l1 = params["l1"]
        lc1 = params["lc1"]
        lc2 = params["lc2"]
        gravity = params["gravity"]

        _, s1, _, _, _, s12 = self.trig(q)
        g1 = (m1 * lc1 + m2 * l1) * gravity
        g2 = m2 * lc2 * gravity

        # Gravity torque on each joint
        g = xp.array([-g1 * s1 - g2 * s12, -g2 * s12])

        return g

    def d(self, q, dq, u=None, t=0.0, params=None):
        params = self.params if params is None else params
        xp = array_module(dq)

        d1 = params["d1"]
        d2 = params["d2"]

        # Linear viscous joint damping
        tau_d = xp.array([d1 * dq[0], d2 * dq[1]])

        return tau_d

    def J(self, q, params=None):
        xp = array_module(q)

        l1 = self.params["l1"]
        l2 = self.params["l2"]

        c1, s1, _, _, c12, s12 = self.trig(q)

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

`J` (and `forward_kinematics`) reads `self.params` rather than its `params` argument. That
is a recorded bug; the pass keeps it as it is, and the bug pass fixes it with a test.

---

## 3. `PPO.loss` (`planning/reinforcement_learning/algorithms/ppo.py`)

**Before (`dev`):**

```python
    def loss(self, params, minibatch):
        """Clipped surrogate, plus the weighted value loss, minus the entropy bonus, on one minibatch."""
        jnp = require_jax_numpy()
        pi, V = self.policy, self.critic
        theta, w = params["policy"], params["critic"]
        x, a = minibatch["x"], minibatch["a"]

        # Probability ratio rho = pi_theta(a|x) / pi_old(a|x) of the stored actions
        ratio = jnp.exp(pi.log_prob(theta, x, a) - minibatch["logp"])

        # Normalized advantage
        advantage = minibatch["advantage"]
        advantage = (advantage - advantage.mean()) / (advantage.std() + 1e-8)

        # Clipped surrogate: L_clip = E[min(rho A, clip(rho, 1 - eps, 1 + eps) A)]
        eps = self.clip_range
        clipped = jnp.clip(ratio, 1.0 - eps, 1.0 + eps)
        policy_loss = -jnp.mean(jnp.minimum(ratio * advantage, clipped * advantage))

        # Value regression E[(R - V_w(x))^2], and the entropy bonus H(pi)
        value_loss = jnp.mean((minibatch["return"] - V.value(w, x)) ** 2)
        entropy = jnp.mean(pi.entropy(theta, x))

        total = policy_loss + self.vf_coef * value_loss - self.ent_coef * entropy

        # Monitoring only: an estimate of how far the update moved the policy
        stats = {
            "policy_loss": policy_loss,
            "value_loss": value_loss,
            "entropy": entropy,
            "approx_kl": jnp.mean((ratio - 1.0) - jnp.log(ratio)),
        }
        return total, stats
```

**After:** the backend line first, then one name per line: the algorithm's fields, the
parameters, the minibatch. No `self.` and no minibatch read in the math. Unicode equation
comments, and `rho` dropped for the ratio, since `rho` is the discount rate.

```python
    def loss(self, params, minibatch):
        """Clipped surrogate, plus the weighted value loss, minus the entropy bonus, on one minibatch."""
        jnp = require_jax_numpy()

        pi = self.policy
        V = self.critic
        eps = self.clip_range
        c_v = self.vf_coef
        c_e = self.ent_coef
        theta = params["policy"]
        w = params["critic"]

        x = minibatch["x"]
        a = minibatch["a"]
        logp_old = minibatch["logp"]
        advantage = minibatch["advantage"]
        returns = minibatch["return"]

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

## 4. `PID` (`control/siso.py`)

**Before (`dev`):** the law sits in a helper method, `PID.command`, called by `f` and
`ctl`; the module ends with a `# ====` banner.

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

    def command(self, e, e_int, dm_filt, params, xp):
        """The unsaturated command, dropping the terms the block does not carry."""
        has_integrator, has_filter = self.has_integrator, self.has_filter
        Kp = xp.asarray(params["Kp"])

        # u = Kp e + Ki e_int − Kd ṁ_filt
        u_cmd = Kp * e
        if has_integrator:
            Ki = xp.asarray(params["Ki"])
            u_cmd = u_cmd + Ki * e_int
        if has_filter:
            Kd = xp.asarray(params["Kd"])
            u_cmd = u_cmd - Kd * dm_filt

        return u_cmd
```

**After:** the law is a textbook equation, not a generic building block, so it is written
in the body of `ctl` and again in `f` (whose anti-windup reads the unsaturated command).
`PID.command` goes, because it was a private-purpose helper. The family's three shared
building blocks (`filtered_rate`, `held_integral`, `clipped`) stay helpers.

```python
    def ctl(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        xp = array_module(x)

        n_int = self.n_int
        has_integrator = self.has_integrator
        has_filter = self.has_filter
        Kp = xp.asarray(params["Kp"])
        Ki = xp.asarray(params["Ki"]) if has_integrator else None
        Kd = xp.asarray(params["Kd"]) if has_filter else None

        e = self.error(u)
        e_int = x[:n_int]
        m_filt = x[n_int:]
        dm_filt = None
        if has_filter:
            dm_filt = filtered_rate(self.derivative_signal(u), m_filt, params, xp)

        # u = Kp e + Ki e_int − Kd ṁ_filt, each term only when the block carries its state
        u_cmd = Kp * e
        if has_integrator:
            u_cmd = u_cmd + Ki * e_int
        if has_filter:
            u_cmd = u_cmd - Kd * dm_filt

        # Actuator limits: u_min ≤ u ≤ u_max
        u = clipped(u_cmd, params, xp)

        return u
```

`f` follows the same blocks, with the same law, then `held_integral` and the state
derivative `dx = [ė_int; ṁ_filt]`. The bottom of the module:

```python
###############################################################################
# Internal machinery
###############################################################################


# The building blocks every member of the classical family shares


def filtered_rate(m, m_filt, params, xp):
    ...
```

`PID.command` is a public method on `dev`. Its removal falls under §1 ("redundant": the
law now in the bodies), counted on `dev`, and listed in the PR.

---

## 5. A module's shape: `planning/search/dubins.py`

**Before (`dev`):** a twelve-line module docstring, then `_2PI`, `_SIGN`, `_mod2pi` and
`_words`, with the public `shortest_path` at line 67.

**After:** a one-line title, the note moved above the constants it explains, the two
section titles in banners, and plain-named helpers below in the order the body calls them.

```python
"""Dubins paths: the shortest path of a forward-only car with a minimum turning radius."""

import math

# The geometry is scalar, so it runs on ``math`` rather than NumPy: it is the
# nearest-neighbour metric of a steering RRT, evaluated against every node.
_2PI = 2.0 * math.pi
_SIGN = {"L": 1.0, "S": 0.0, "R": -1.0}


###############################################################################
# Public API
###############################################################################


def shortest_path(q0, q1, radius):
    """
    Return ``(word, segments, length)`` for the shortest of the six Dubins words.

    ``segments`` lists ``(curvature_sign, arc_length)``, the sign in ``{+1, 0, -1}``
    (left, straight, right); ``length`` is the total arc length. ``None`` if degenerate.
    """
    # The normalised frame: the start-to-goal line as the x axis, a unit radius
    dx = q1[0] - q0[0]
    dy = q1[1] - q0[1]
    d = math.hypot(dx, dy) / radius
    theta = mod2pi(math.atan2(dy, dx))
    a = mod2pi(q0[2] - theta)
    b = mod2pi(q1[2] - theta)

    # The shortest of the feasible words
    ...

    return best_word, segments, length


###############################################################################
# Internal machinery
###############################################################################


def word_segments(a, b, d):
    """Segment triples ``(t, p, q)``, normalised, of each feasible word."""
    ...


def mod2pi(angle):
    """``angle`` wrapped to ``[0, 2π)``."""
    return angle % _2PI
```

The private module constants keep their underscore (RULES 5.8).

---

## 6. A constructor: `TwoLinkManipulator.__init__`

**Before (`dev`):** a single block, and a module helper that sets two attributes on the
object from outside `__init__` (RULES 5.7, shadow state).

```python
    def __init__(self):
        super().__init__(dof=2, actuators=2, task_dim=2)
        self.name = "Two Link Manipulator"
        self.params = {
            "l1": 0.5,
            "l2": 0.3,
            ...
        }
        self.state.labels = ["theta1", "theta2", "dtheta1", "dtheta2"]
        self.state.units = ["rad", "rad", "rad/s", "rad/s"]
        self.inputs["u"].labels = ["tau1", "tau2"]
        self.inputs["u"].units = ["Nm", "Nm"]
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)
        _set_planar_reach_camera(self, self.params["l1"] + self.params["l2"])


def _set_planar_reach_camera(manipulator, reach):
    """Match orthographic zoom to planar arm reach (``System.camera_scale``)."""
    reach = float(reach)
    manipulator.camera_scale = max(1.5 * reach, 0.35)
    manipulator.camera_target = np.array([0.0, 0.5 * reach, 0.0])
```

**After:** named blocks, each opened by a short comment. The helper returns the camera, and
`__init__` assigns it, so every attribute is visibly set in the constructor.

```python
    def __init__(self):
        super().__init__(dof=2, actuators=2, task_dim=2)
        self.name = "Two Link Manipulator"

        # Parameters
        self.params = {
            "l1": 0.5,
            "l2": 0.3,
            ...
        }

        # Signal labels and units
        self.state.labels = ["theta1", "theta2", "dtheta1", "dtheta2"]
        self.state.units = ["rad", "rad", "rad/s", "rad/s"]
        self.inputs["u"].labels = ["tau1", "tau2"]
        self.inputs["u"].units = ["Nm", "Nm"]
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)

        # Graphics: the camera frames the arm's full reach
        reach = self.params["l1"] + self.params["l2"]
        self.camera_scale, self.camera_target = planar_reach_camera(reach)


def planar_reach_camera(reach):
    """The orthographic zoom and target that frame a planar arm of the given reach."""
    reach = float(reach)
    camera_scale = max(1.5 * reach, 0.35)
    camera_target = np.array([0.0, 0.5 * reach, 0.0])
    return camera_scale, camera_target
```

---

## 7. A public verb: `linearize_matrices` (`analysis/linearization.py`)

**Before (`dev`):** close to the target already. The input checks and the setup are mixed
together, an inline `raise` sits between them, and the module ends with a `# ====` banner.

```python
    require_own_point(sys, x_bar, u_bar)
    at = dict(method=method, eps=eps)
    point = (x_bar, u_bar, t, params)
    n = sys.n
    inputs = input_selectors(sys, wrt)
    outputs = output_selectors(sys, of)
    if n == 0 and outputs is None:
        raise ValueError(f"{sys.name!r} has neither a state nor an output port")

    # A = ∂f/∂x and B = ∂f/∂u at the operating point; a static block has no state
    A = jacobian(sys, "f", "x", *point, **at) if n > 0 else np.zeros((0, 0))
    ...
```

**After:** the checks first, then the setup, then the math blocks. The inline `raise`
becomes a `require_*` helper with the same type and message.

```python
    n = sys.n
    inputs = input_selectors(sys, wrt)
    outputs = output_selectors(sys, of)
    require_own_point(sys, x_bar, u_bar)
    require_state_or_output(sys, outputs)

    point = (x_bar, u_bar, t, params)
    at = dict(method=method, eps=eps)

    # A = ∂f/∂x and B = ∂f/∂u at the operating point; a static block has no state
    A = jacobian(sys, "f", "x", *point, **at) if n > 0 else np.zeros((0, 0))
    B = jacobian_block(sys, [("f", None)], inputs, point, at) if n > 0 else None

    # C = ∂h/∂x and D = ∂h/∂u; with no output port the output is the state: C = I, D = 0
    if outputs is None:
        C = np.eye(n)
        D = np.zeros((n, B.shape[1]))
    else:
        C = jacobian_block(sys, outputs, [("x", None)], point, at) if n > 0 else None
        D = jacobian_block(sys, outputs, inputs, point, at)

    # A static block's channel is its feedthrough D alone
    if n == 0:
        B = np.zeros((0, D.shape[1]))
        C = np.zeros((D.shape[0], 0))

    return A, B, C, D
```

The order change is safe only because each step is pure (the selectors raise the same
errors either way). The writer proves it with the probes, and keeps dev's order otherwise.

---

## 8. An iterative algorithm: `RRTPlanner.solve_trajectory` (`planning/search/rrt.py`)

**Before (`dev`):** the loop's steps carry no comments; four `_` helpers; `_search_callback`
is first set here rather than in `__init__`.

```python
    def solve_trajectory(self, *, evaluate=False, n_trials=50) -> PlanningSolution:
        """Grow the tree until the goal region is reached or the budget is spent."""
        options = self.options
        self._validate_nearest_backend()
        rng = np.random.default_rng(options.seed)
        goal_region = self._goal_region()

        x_start = np.asarray(self.problem.x_start, dtype=float)
        self.tree = Tree(
            Node(x=x_start, parent=None, edge=None, cost=0.0),
            nearest_backend=options.nearest_backend,
        )
        self.reached_goal = False
        self.solution_node = None
        self.iterations = 0
        self._search_callback = self._resolve_search_callback()

        for _ in range(options.max_nodes):
            x_rand = self._sample(rng)
            near = self.tree.nearest(x_rand, self.metric)
            edge = self._try_extend(near, x_rand, rng)
            if edge is None:
                continue

            node = self.tree.add(
                Node(x=edge.x_end, parent=near, edge=edge, cost=near.cost + edge.cost)
            )
            self.iterations += 1
            if goal_region.contains(node.x):
                ...
```

**After:** the setup in blocks, the algorithm's steps as commented blocks inside the loop,
the bookkeeping as one helper call per iteration, and plain helper names.

```python
    def solve_trajectory(self, *, evaluate=False, n_trials=50) -> PlanningSolution:
        """Grow the tree until the goal region is reached or the budget is spent."""
        options = self.options
        metric = self.metric
        self.require_nearest_backend()

        rng = np.random.default_rng(options.seed)
        goal_region = self.goal_region()
        x_start = np.asarray(self.problem.x_start, dtype=float)

        # The tree, rooted at the start
        self.tree = Tree(
            Node(x=x_start, parent=None, edge=None, cost=0.0),
            nearest_backend=options.nearest_backend,
        )
        self.reached_goal = False
        self.solution_node = None
        self.iterations = 0
        self.search_callback = self.resolve_search_callback()

        for _ in range(options.max_nodes):
            # Sample a state, and extend the nearest node toward it
            x_rand = self.sample_state(rng)
            near = self.tree.nearest(x_rand, metric)
            edge = self.try_extend(near, x_rand, rng)
            if edge is None:
                continue

            # The new node, with its cost-to-come
            cost = near.cost + edge.cost
            node = self.tree.add(Node(x=edge.x_end, parent=near, edge=edge, cost=cost))
            self.iterations += 1

            # Stop at the first node inside the goal region
            if goal_region.contains(node.x):
                self.reached_goal = True
                self.solution_node = node
                self.on_search_step(phase="explore")
                return self.finish_trajectory(
                    self.tree.extract_trajectory(node), evaluate, n_trials
                )

            self.on_search_step(phase="explore")

        # The budget is spent: the node closest to the goal, when best effort is allowed
        goal_point = self.goal_point(goal_region)
        closest = self.tree.nearest(goal_point, metric)
        self.solution_node = closest
        if not options.return_best_effort:
            raise RuntimeError("RRT failed to reach goal within max_nodes")

        return self.finish_trajectory(
            self.tree.extract_trajectory(closest), evaluate, n_trials
        )
```

`__init__` gains `self.search_callback = None` (RULES 5.7). The renames `_sample` →
`sample_state`, `_validate_nearest_backend` → `require_nearest_backend` and the others go
into the run's rename table, and the tests that call them follow.

---

## 9. A class's layout: `RRTPlanner`

**Before (`dev`):** the public verbs, then ten `_` methods in no particular order.

**After:** the docstring, `__init__`, the public methods with the primary verb first, then
the indented banner and the helpers in the order the public methods call them.

```python
class RRTPlanner(Planner):
    """
    Rapidly-exploring random tree over a ``PlanningProblem``.

    Parameters
    ----------
    ...
    """

    def __init__(self, problem, extender=None, *, metric=euclidean, options=None, ...):
        ...

    def solve(self, *, evaluate=False, n_trials=50) -> PlanningSolution:
        ...

    def solve_trajectory(self, *, evaluate=False, n_trials=50) -> PlanningSolution:
        ...

    def search_record(self) -> TreeSearchRecord:
        ...

    def plot_tree(self, **kwargs):
        ...

    def animate_search(self, **kwargs):
        ...

    def show_live_search(self, **kwargs):
        ...

    ###########################################################################
    # Internal machinery
    ###########################################################################

    def require_nearest_backend(self) -> None:
        ...

    def goal_region(self):
        ...

    def resolve_search_callback(self):
        ...

    def sample_state(self, rng):
        ...

    def try_extend(self, from_node, toward, rng):
        ...

    def on_search_step(self, *, phase: str) -> None:
        ...

    def finish_trajectory(self, trajectory, evaluate, n_trials) -> PlanningSolution:
        ...
```

