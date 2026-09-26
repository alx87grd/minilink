# One convention for randomness

Status: design draft 2026-09-26, brainstorm with the maintainer; rulings RN-a to RN-g open.
Rung: v0.2 wave B, the decision ROADMAP §6 lists before B3 (`estimation/`, P4); it closes
finding F9 of [2026-09-15-foundations-review.md](../reviews/2026-09-15-foundations-review.md)
and absorbs the `NoiseSource` and `Distribution.sample(key=None)` rows of TODO A5.
Consumers: the `WhiteNoise` block, a colored-noise recipe, random starts, LQG, RL training,
Monte Carlo evaluation. One question underneath all six: **what a number on a noise means in
time**, and **where a random draw happens** in a library whose equation paths are pure.

## 1. The code as it stands

| Site | What "random" means there | Time semantics |
| --- | --- | --- |
| `Distribution` (`core/distributions.py`) | law of one draw: `sample(key, n)`, `mean`, `support` | none, correct |
| `WhiteNoise` (`blocks/sources.py`) | Gaussian table drawn at `refresh()`, linear interpolation, `params["seed"]`, NumPy only | per-sample `var` at its own `sample_period` |
| `StochasticPlanningProblem.disturbances` | `{port: Distribution}`, a fresh draw per step, held | per-sample at whatever `dt` the tool picks |
| `RolloutEnvironment`, `MonteCarloEvaluator` | draw from the problem at the tool's `dt` | inherit the ambiguity above |
| `Sys2Gym(sys)` | `x0_lb / x0_ub / x0_std` of its own | per trial, a second start-state dialect |
| Tools (`RRT`, tabular RL, RL planner, evaluator, `lyapunov`) | `seed: int`, own generator or key inside | fine |

Two consequences. `Gaussian(0, 0.1)` on port `w` is a different physical disturbance for a
planner at `dt = 0.05` and an evaluator at `dt = 0.01`; `WhiteNoise` has the same problem with
`var` plus `sample_period`. A Kalman filter cannot be designed against either without a rule.
Two smaller inconsistencies: `Set.sample` accepts `key=None` and `Distribution.sample` does
not; `split_keys` gives independent keys on JAX but the same generator repeated on NumPy, so
the two backends do not have the same tree of streams. The nine modules that construct a
generator or a key (`backends`, `neural`, `sources`, `lyapunov`, `evaluation`, `tabular`, the RL
`planner`, `rrt`, `rrt_star`) are all constructors or tool boundaries, which is the right list.

## 2. The core math

**A distribution has no time.** `p` is the law of one vector: `x ~ p`, with a mean and a
covariance `Σ`. A start `x(0) ~ p`, a parameter `m ~ p`, one held sample of a disturbance:
the same object.

**A continuous white noise has no variance, only an intensity.** `w(t)` with

    E[w(t)] = 0,      E[w(t) w(τ)ᵀ] = W δ(t − τ)

`W` is the spectral density (intensity), in units of the signal squared times seconds. The
value at one instant has infinite variance; only its integral is finite:

    ∫₀^Δ w(t) dt ~ N(0, W Δ)                      (the Wiener increment: dβ with E[dβ dβᵀ] = W dt)

The plant integrates `w`, so the state's variance is finite and does not depend on how the
noise is sampled.

**A held sample train is the band-limited approximation.** Take `w(t) = w_k` on
`[kΔ, (k+1)Δ)`, with `w_k` independent, `w_k ~ N(0, Σ)`. Its integral over one period is
`Δ w_k`, of covariance `Δ² Σ`. Matching the Wiener increment `W Δ` gives the one rule:

    Σ = W / Δ          (per-sample covariance of a white noise held over Δ)

Equivalently, the held train has spectral density `Σ Δ`, flat up to about the Nyquist
frequency `π / Δ`. As `Δ → 0` at fixed `W`, `Σ → ∞`: infinite variance, seen concretely.

**Why per-sample alone is the wrong invariant for a continuous plant.** For `ẋ = −a x + w`
with intensity `W`, the stationary variance solves the Lyapunov equation `0 = −2 a P + W`, so
`P = W / (2a)`. Under the held train with `Σ = W / Δ` the simulation converges to that value
as `Δ → 0`. With `Σ` fixed instead, `P ≈ Σ Δ / (2a)`: halve the sample period and the state
noise halves. That is F9 in one line.

**The bridge to discrete time, once.** With the disturbance entering through `B_w = ∂f/∂w`
and a measurement `y = h(x) + v`, `v` white of intensity `V`:

| Quantity | Continuous (Kalman-Bucy, dual of `lqr`) | Held at Δ (discrete Kalman) |
| --- | --- | --- |
| process noise | `Q = B_w W B_wᵀ` | `Q_d = B_w W B_wᵀ Δ` (first order; exactly `∫₀^Δ e^{Aτ} B_w W B_wᵀ e^{Aᵀτ} dτ`) |
| measurement noise | `R = V` | `R_d = V / Δ` |

`Q_d` is the covariance of the noise *added to `x_{k+1}`*: the held sample `Δ B_w w_k` has
covariance `Δ² B_w (W/Δ) B_wᵀ = B_w W B_wᵀ Δ`. `R_d` is the covariance of *one measurement
sample*, a sensor averaging over `Δ`. Times Δ on the state, over Δ on the measurement: the
asymmetry students get wrong, and why the rule lives in one place (DESIGN §3 and one equation
comment beside the filter Riccati solve; never a docstring).

**Colored noise is white noise through a filter.** A first-order shaping filter
`τ ż = −z + w`, `w` white of intensity `W`, gives `z` with variance `W / (2τ)` and correlation
time `τ`. The textbook handles it by augmenting the state with `z`; so does a diagram.

## 3. The convention

Six rules. Each keeps the constitution's nouns and adds only the time semantics.

- **R1. A `Distribution` is always the law of one draw.** `Gaussian(0, σ)` samples have
  standard deviation `σ` everywhere: a start, a parameter, a particle, one held sample. It
  never hides a `dt`. It grows only as A5 plans: `Gaussian(mean, std=None, cov=None)`, a
  `cov` property on every law (`Uniform`: `diag((b − a)² / 12)`; `Particles`: the sample
  covariance), `log_prob`, and `sample(key, n=None, params=None)` like the sets, so a noise
  level can be a parameter and a family of noise levels vmaps like a family of masses.

- **R2. A random signal is a law held over a period.** `w(t) = w_k` on `[kΔ, (k+1)Δ)`,
  `w_k ~ p`. The object that carries time is the signal, not the distribution:
  `NoiseSource(distribution, sample_period)`, a `Source` block. `WhiteNoise` is its Gaussian
  shortcut. Colored noise is not a class: `WhiteNoise(...) >> LowPassFilter(...)` is a diagram
  like any other, and inside a plant it is the augmented state of §2.

- **R3. The dt-free number of a white noise is its spectral density.** `WhiteNoise` takes
  exactly one of `psd=` or `cov=` (`std=` as the scalar convenience of `cov`), and exposes
  both as properties, related by `cov = psd / sample_period`. A design tool reads `.psd`; a
  stepping tool reads `.cov`; both agree because `Δ` is on the source.

  `WhiteNoise(psd=W, sample_period=Δ)` **is** `NoiseSource(Gaussian(0, cov=W / Δ), Δ)`.
  `WhiteNoise(cov=Σ, sample_period=Δ)` **is** `NoiseSource(Gaussian(0, cov=Σ), Δ)`.
  Two names for one object; `psd` is the name that survives a change of `Δ`.

- **R4. On a problem, a disturbance port takes a signal or a law.**
  `disturbances={"w": WhiteNoise(psd=W)}` with no `sample_period` means the tool's `dt` is
  `Δ`, and the physics does not change between a planner at `0.05 s` and an evaluator at
  `0.01 s`. A bare `Distribution` stays legal and means per-sample at the tool's `dt`, the
  discrete textbook's `w_k ~ N(0, Q)`. The tool converts at its boundary (RULES 4.3) and warns
  once (RULES 4.12), stating the implied intensity in units.

- **R5. A random start is a `Distribution` over `x`, owned by the problem; `sys.x0` is its
  mean.** Already so for `StochasticPlanningProblem`. `Sys2Gym` drops `x0_lb / x0_ub /
  x0_std` and takes a `Distribution`, `from_problem` being the route (TODO row exists).

- **R6. Seeds enter at a tool's boundary as an integer, and nothing else.** Inside the
  library one generator or one JAX key is threaded and split per consumer; `split_keys`
  splits on NumPy too (`Generator.spawn`, NumPy ≥ 1.25), so both backends have the same tree
  of streams. No global RNG anywhere (Constitution §4.4; JAX has none). Stated plainly: the
  same seed gives different numbers on NumPy and JAX unless the draws are tables (§4).

## 4. How a stochastic simulation runs

Equation paths are pure (Constitution §5.1), so **a draw never happens inside `f` or `h`**.
Randomness is frozen into data before the equation path runs, and the simulation of that data
is deterministic. A *realization* is the triple

    (x0, params, noise tables)

and a stochastic simulation is the deterministic simulation of one realization. Monte Carlo is
`N` realizations: `N` deterministic simulations, looped on NumPy or vmapped on JAX. This is
the textbook's own scheme (a sample path; Euler–Maruyama draws the increments, then steps).

Two mechanisms produce the held train `w_k`, and the convention makes them the same process:

| Where | Mechanism | Why |
| --- | --- | --- |
| continuous simulation (`Simulator`, any solver) | a table drawn once, `h(t)` looks up `table[⌊t / Δ⌋]` | an adaptive solver evaluates `h` many times at one `t`; a draw inside `h` is impure and breaks it |
| discrete stepping (RL environment, JAX Monte Carlo) | one key per step, `w_k = p.sample(k_step)` inside the step map | the step is a discrete map; the key is an input, so the map stays pure and scans |

Both draw `w_k ~ N(0, W / Δ)` at their own `Δ`; they agree in distribution, not sample by
sample.

**The table lives in `params`.** `NoiseSource` draws its realization at construction from
`seed` into `params["samples"]` (shape `(p, N)` over its window), the way the neural blocks
keep their weights in `params`. Then `h` is a zero-order-hold lookup on `xp`, so it traces;
`refresh()` and `params["seed"]` go away; a new realization is a new params dict, not a
mutation; and a family of realizations is a params family, which the compiled evaluators
already vmap. The block's `seed` is its *nominal* realization, as `x0` is the nominal start:
`sys.compute_trajectory()` on a diagram with a noise block runs with no seed plumbing.

**One key per experiment.** A diagram gathers params from its subsystems; it gathers
realizations the same way. `sys.realize(key)` splits the key once per random subsystem, in
subsystem order, and returns the params dict with fresh tables everywhere (nested, like
params). A problem does the same one level up:

    params = sys.realize(key)                       # every noise block, one key
    traj = sys.compute_trajectory(tf, params=params)

    x0, params = problem.realize(key)               # start, parameters, disturbances
    trials = vmap(problem.realize)(split(key, N))   # Monte Carlo = a params family

Block seeds are not re-plumbed one by one; the key tree is the diagram structure, which is
explicit and stable. This is `jax.random.split` and NumPy's `SeedSequence.spawn` discipline,
applied along the object tree instead of the call tree.

**Tables first for evaluation.** `MonteCarloEvaluator` draws `(x0, params, tables)` for all
trials once, on NumPy, and both backends consume the arrays: an `Evaluation` is then identical
across backends, the "one seed, two streams" warning disappears, and a realization can be
saved and replayed. Cost: one array of shape `(trials, steps, dim)` per disturbance port.
Training stays key-threaded in JAX, where cross-backend identity buys nothing.

## 5. The six uses, one page

```python
plant = PendulumWithNoisePort()                        # ports u, w [N m], v [rad/s]

# white noise: a law held over a period; the intensity is the dt-free number
w = WhiteNoise(psd=1e-2, sample_period=0.01, seed=1)   # w_k ~ N(0, psd / Δ)
v = WhiteNoise(psd=1e-4, sample_period=0.01, seed=2)

# colored noise: white noise through a shaping filter
wind = WhiteNoise(psd=1.0, sample_period=0.01) >> LowPassFilter(tau=0.5)

# random start, random parameters, disturbances: one problem owns every law
problem = StochasticPlanningProblem(
    plant, cost, tf=10.0,
    x0_distribution=Gaussian(x_bar, std=[0.2, 0.1]),
    params_distribution={"m": Uniform(0.8, 1.2)},
    disturbances={"w": WhiteNoise(psd=1e-2), "v": WhiteNoise(psd=1e-4)},
)

# LQG: the design reads the same two numbers the simulation draws from
lin = plant.linearize()
B_w = jacobian(plant, "f", "w")
K = lqr(lin.A, lin.B, Q_x, R_u)
L = kalman(lin.A, lin.C, B_w @ w.psd @ B_w.T, v.psd)   # Q = B_w W B_wᵀ,  R = V

# RL training and Monte Carlo: one integer seed per tool, draws at the tool's dt
ctl = ReinforcementLearningPlanner(problem, dt=0.05, seed=0).solve().ctl
report = MonteCarloEvaluator(problem, dt=0.05, n_trials=100, seed=1).evaluate(ctl)

# one continuous closed-loop run of a fresh realization
loop = (w, v) >> ...                                    # noise blocks wired to w, v
traj = loop.compute_trajectory(tf=10.0, params=loop.realize(key=3))
```

## 6. Rulings (maintainer)

- **RN-a. Per-sample or intensity as the meaning of a bare number.** Recommended: intensity is
  the dt-free number; a bare `Distribution` on `disturbances` stays legal, per-sample at the
  tool's `dt`, with a one-time warning. Stricter alternative: signals only, which breaks the
  current notebooks.
- **RN-b. The keyword.** `psd=` is the word students meet in the frequency chapters;
  `intensity=` is Åström's. One name, the same on the property.
- **RN-c. Hold shape in the block.** Zero-order hold is what every discrete tool does and
  traces as an index; linear interpolation (today) is smoother for adaptive solvers but
  changes the spectrum. Recommended: ZOH, with a note that adaptive solvers pay for the steps.
- **RN-d. Table in `params` or attribute plus `refresh()`.** Recommended: `params` (§4). A
  behavior change to the block: its own step, seeded baseline.
- **RN-e. `Set.sample(key=None)`.** Recommended: drop `None`, so every draw in the library is
  reproducible; tools may still take `seed=None` for fresh entropy and say so in their report.
- **RN-f. `split_keys` on NumPy.** `Generator.spawn` changes every NumPy draw that goes
  through it: a byte-changing step with its own test, not a refactor.
- **RN-g. Tables-first evaluation.** Recommended: yes (§4).

## 7. Steps, once ruled

- [ ] **RN-1** `Gaussian(cov=)`, `cov` on every law, `sample(key, n, params)` on
  `Distribution`; `Set.sample` key required (RN-e). Baseline: every `sample` call site.
- [ ] **RN-2** `NoiseSource(distribution, sample_period, seed, window)`, table in `params`,
  ZOH lookup on `xp`; `WhiteNoise(psd= | cov= | std=)` as its shortcut with both properties;
  `refresh()` and `params["seed"]` removed (a release note). Demos `blocks_sources.py`,
  `diagram_noise_ports.py`, `diagram_shortcuts.py` updated; the guard test of TODO A5.
- [ ] **RN-3** `System.realize(key)` gathered over subsystems; `StochasticPlanningProblem.realize(key)`
  and `disturbances` accepting a signal (RN-a); the RL environment and the evaluator read
  `.cov` at their `dt`; `split_keys` spawns on NumPy (RN-f).
- [ ] **RN-4** `MonteCarloEvaluator` tables first (RN-g); `Sys2Gym` takes a `Distribution` (R5).
- [ ] **RN-5** DESIGN §3: the convention and the bridge table; ROADMAP §6 closes F9; TODO A5
  rows retired; `kalman()` (P4) reads intensities, the discrete filter later reads `Q_d`, `R_d`.

Out of scope on purpose: time-varying laws `p(t)` (a gust is `Step` times a noise source),
a `ColoredNoise` class (a filter), a stochastic system wrapper (randomness enters only through
named ports, RULES 4.9).
