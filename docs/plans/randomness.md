# One convention for randomness

Status: decided (§8: RD-D1–RD-D12 on 2026-09-26, RD-D13–RD-D24 on 2026-09-30), and brought in
line with the 2026-10-27 choices (below), which hold where they differ.
- **Landed:**
  - RN-1 on 2026-09-30: `WhiteNoise` on the counter-based cipher, `System.realize`, the solver
    hint (DESIGN §4, "Randomness").
  - RN-5's first part on 2026-10-24 (R44, V1's step 4): `evaluate(problem, arg, *, dt,
    n_trials, key)` over the start and parameter draws of `problem.realize(key, n)`, with
    `compare` and `pe.program` on the same draws and `MonteCarloEvaluator` removed (DESIGN §6,
    "The problem evaluator").

  The code before RN-1, RN-1's build plan, its measurements and the options weighed for
  RD-D1–RD-D24 are in git history.
- **Open:**
  - RN-2: the distributions read `params`;
  - RN-3: `NoiseSource`, with its first consumer;
  - RN-4: `realize` as one tree, the sources' seeds, disturbances as source blocks;
  - the rest of RN-5: the disturbance sources in the evaluated loop, `Sys2Gym`'s start law, the
    LQG gate;
  - RN-6: the docs, with each step.

  RN-4 and RN-5 change public behaviour and the Monte Carlo numbers once.
- **The 2026-10-27 choices** (the architecture of record since 2026-10-28, R166):
  - **disturbances are source blocks** (R128, [record](../reviews/2026-10-27-design-questions-rulings.md)):
    no `Distribution` on a port; a source block wired into the problem's system, with
    `disturbances={"d": block}` as the shortcut that wires it; RD-R4 taken to its end;
  - **a problem is stochastic by its parts** (R143, [record](../reviews/2026-10-27-core-vision-decisions.md)):
    no `StochasticPlanningProblem`; `x_start=` takes a point, a `Set` or a `Distribution`;
    `params=` takes laws in the tree's own shape (`{"plant": {"m": Uniform(0.9, 1.1)}}`);
    `x0_distribution=` and `params_distribution=` go; `criterion=` and `nominal()` on the class;
    the same on `StepPlanningProblem` (R141);
  - **`realize` returns one tree** (R144): `problem.realize(key, n_trials) -> params`,
    `x_start` a leaf with a trial axis; a law in a leaf is drawn to a value from a sub-key by its
    path; a source block gets an integer seed leaf; one key, streams by name (§5);
  - **periods on blocks** (R146): no `Computer`; a sampled law is a `SampledSystem` and the loop
    `computer @ plant` a `SampledDiagramSystem`.
- **Ids:** `RD-` marks this doc's own: the rules RD-R1–RD-R6 (§3), the rulings RD-D1–RD-D24
  (§8) and the review's amendments RD-A1–RD-A10 (§10). `FD-F9` is finding F9 of the
  [foundations review](../reviews/2026-09-15-foundations-review.md); a bare `A5`, `P4` or `RN-2`
  is a workboard step.

**Where it sits:**
- RN-2 is in the core's first invariant: [core-architecture.md](core-architecture.md) §1,
  [TODO.md](TODO.md) §1.1.
- RN-3 to RN-6 are the draws: core-architecture §3, TODO §1.3.
- Step 7 of [differentiable-problem.md](differentiable-problem.md) is RN-4's evaluator side.

Consumers: the `WhiteNoise` block, a colored-noise recipe, random starts, LQG, RL training,
Monte Carlo evaluation. One question underneath all six: **what a number on a noise means in
time**, and **where a random draw happens** in a library whose equation paths are pure.

**Port names (2026-10-11).** The loop's disturbance port is `d` and its measurement-noise
port `n` (DESIGN §4, "Control laws"). Where §10 says port `w` or `v`, it describes the code of
that time; the math symbols (`w_k`, `W`, `V`, `B_w`) are unchanged.
`closed_loop(disturbance=, noise=)` accepts a noise block (landed 2026-10-14).

## The design in one screen

- **A `Distribution` has no time.** It is the law of one vector: a start, a parameter value,
  one sample of a noise.
- **A noise signal is a source block that holds draws over a sample period Δ.**
  `NoiseSource(law, Δ)` holds independent draws `w_k ~ law`: sampled noise.
  `WhiteNoise(psd=W, Δ)` is continuous white noise of two-sided spectral density `W`, drawn as
  `w_k ~ N(0, W / Δ)`, so its physics does not change when Δ does. Colored noise is white noise
  through a filter block.
- **The block is pure.** Its seed, Δ and magnitude are params. `h` computes `w(t)` from
  `(t, params)` with a counter-based generator, `ε_k = F(seed, ⌊t/Δ⌋)`: no drawn table, no time
  window, no `refresh()`, the same signal for every solver and on both backends, and it traces
  under JAX.
- **A problem is stochastic by its parts** (R143). A law goes where a number goes: the start
  (`x_start=Gaussian(...)`), a parameter leaf (`params={"plant": {"m": Uniform(...)}}`).
  Disturbances are source blocks wired onto the plant's ports (R128), explicitly or by
  `disturbances={"d": WhiteNoise(psd=W)}`.
- **One key reproduces an experiment.** `problem.realize(key, n_trials)` returns one parameter
  tree (R144): each law drawn to a value, each source block given a seed, every stream derived
  from the key by its name.
- **Monte Carlo is the closed loop simulated on draws realized once from one key.**
  `evaluate(problem, ctl, dt=, n_trials=, key=)` realizes the tree once and runs the compiled
  fixed-step rollout on every trial, batched (R10, R15). Built in v0.3 for the starts and the
  parameter values; the sources in the loop and the one tree are RN-4 and the rest of RN-5.
- **A noisy diagram picks its own solver.** The noise block publishes its period as the
  solver hint, so a plain `compute_trajectory()` runs fixed-step RK4 at `dt = Δ`.
- **LQG reads the same numbers the simulation draws from**: `Q = B_w W B_wᵀ`, `R = V` for the
  continuous filter; `Q_d = B_w W B_wᵀ Δ`, `R_d = V / Δ` for the discrete one.

## 1. What the code still lacks

- `StochasticPlanningProblem` carries `x0_distribution=`, `params_distribution=` (flat names)
  and `disturbances` as `{port: Distribution}` (`planning/problems.py`); R143 and R128 replace
  all three.
- The RL environment and `Sys2Gym` draw a fresh value per step and hold it on the port, at their
  own `dt`; `evaluate` holds the declared ports at their nominal value and warns
  (`warn_if_disturbed`, `planning/evaluator.py`).
- `problem.realize(key, n)` returns `(x0s, {"params": ..., "disturbances": ...})`: the starts,
  the parameter values and one held value per disturbance port, drawn from positional
  `split_keys`, with no seeds for the sources. `evaluate` realizes random blocks inside the loop
  per trial through `trial_seeds` (R92), while `RolloutEnvironment` keeps a random block's seed
  across episodes. `System.realize(key)` and `child_seed` exist since RN-1.
- One integer key draws different starts and parameter values on NumPy and JAX (`trial_key`;
  KNOWN_ISSUES).
- `Gaussian(mean, std)` has no `cov`, and `Distribution.sample(key, n=None)` takes no `params`
  (RN-2); `Set.sample` still accepts `key=None` (`core/sets.py`; RN-2, RD-D9).
- There is no `NoiseSource` (RN-3).
- `Sys2Gym` keeps a start dialect of its own, `x0_lb / x0_ub / x0_std`
  (`interfaces/gymnasium.py`; RD-R5).

## 2. The core math

**A distribution has no time.** `p` is the law of one vector: `x ~ p`, with a mean and a
covariance `Σ`. A start `x(0) ~ p`, a parameter `m ~ p`, one held sample of a disturbance:
the same object.

**A continuous white noise has no variance, only an intensity.** `w(t)` with

    E[w(t)] = 0,      E[w(t) w(τ)ᵀ] = W δ(t − τ)

`W` is the spectral density (intensity), in units of the signal squared times seconds. The
value at one instant has infinite variance; only its integral is finite:

    ∫₀^Δ w(t) dt ~ N(0, W Δ)                      (the Wiener increment: dβ with E[dβ dβᵀ] = W dt)

**A held sample train is the band-limited approximation.** Take `w(t) = w_k` on
`[kΔ, (k+1)Δ)`, with `w_k` independent, `w_k ~ N(0, Σ)`. Its integral over one period is
`Δ w_k`, of covariance `Δ² Σ`. Matching the Wiener increment `W Δ` gives the one rule:

    Σ = W / Δ          (per-sample covariance of a white noise held over Δ)

As `Δ → 0` at fixed `W`, `Σ → ∞`: infinite variance, seen concretely.

**Why per-sample alone is the wrong invariant for a continuous plant.** For `ẋ = −a x + w`
with intensity `W`, the stationary variance solves the Lyapunov equation `0 = −2 a P + W`, so
`P = W / (2a)`. Under the held train with `Σ = W / Δ` the simulation converges to that value
as `Δ → 0`. With `Σ` fixed instead, `P ≈ Σ Δ / (2a)`: halve the sample period and the state
noise halves. That is FD-F9 in one line.

**Δ is a resolution for white noise, and physical for sampled noise.** With `W` fixed, the
plant stops seeing Δ once Δ is small against its fastest time constant. Exact variance of the
held-sample model for `ẋ = −x + w` (time constant 1 s), over the theory. The process is
cyclostationary, so the variance is read at the sample instants `t = kΔ`,
`(2 / aΔ) tanh(aΔ / 2)`, or averaged over the period, `≈ 1 − aΔ / 3`, which is what the
variance of a trajectory logged finer than Δ shows:

| Δ | 0.001 s | 0.01 s | 0.1 s | 0.3 s | 1 s | 3 s |
| --- | --- | --- | --- | --- | --- | --- |
| Var[x] / (W / 2a), at the sample instants | 1.000 | 1.000 | 0.999 | 0.993 | 0.924 | 0.603 |
| Var[x] / (W / 2a), averaged over the period | 1.000 | 0.997 | 0.968 | 0.907 | 0.736 | 0.456 |

Rule of thumb: Δ at most a tenth of the fastest time constant (3% low on the averaged
variance, 0.1% at the sample instants). For noise that really is sampled (a fresh sensor
reading, a disturbance drawn per control step), Δ is the physical period and the per-sample law
is the natural number.

**When the physics is Δ-free (RD-A2).** Three conditions (§10, finding 1): the port enters `f`
affinely with a gain that does not depend on the state; the consumer integrates the noise (a
sampler that point-reads `y = x + v` sees `V / Δ_v`); Δ is small against the loop and the
observer, not only the plant. Wind and anything entering through drag is white noise through a
filter; a one-sided density `n` per √Hz is `W = n² / 2`.

**The hold shape.** A held train is `w(t) = Σ_k w_k φ(t − kΔ)`. The zero-order hold uses a box
kernel of width Δ; linear interpolation a triangle of width 2Δ. Both kernels have area Δ, so
both signals have the same low-frequency spectral density `Σ Δ = W`, and the plant sees the same
intensity. They differ elsewhere: the zero-order hold has pointwise variance `Σ` everywhere;
linear interpolation dips to `Σ / 2` at mid-period (`(1 − s)² + s²`), rolls off sooner (`sinc⁴`
against `sinc²`), and no longer matches the discrete map sample by sample.

**The bridge to discrete time, once.** With the disturbance entering through `B_w = ∂f/∂w`
and a measurement `y = h(x) + v`, `v` white of intensity `V`:

| Quantity | Continuous (Kalman-Bucy, dual of `lqr`) | Held at Δ (discrete Kalman) |
| --- | --- | --- |
| process noise | `Q = B_w W B_wᵀ` | `Q_d = B_w W B_wᵀ Δ` (first order; exactly `∫₀^Δ e^{Aτ} B_w W B_wᵀ e^{Aᵀτ} dτ`) |
| measurement noise | `R = D_v V D_vᵀ` | `R_d = D_v V D_vᵀ / Δ_v`, at the measurement noise's own period |

`D_v = ∂h/∂v` maps the noise onto the outputs it corrupts (`R = V` when `y = h(x) + v`); an
output with no noise on it makes `R` singular, and the continuous filter needs `R > 0`.
`Q_d` is the covariance of the noise *added to `x_{k+1}`*: the held sample `Δ B_w w_k` has
covariance `Δ² B_w (W/Δ) B_wᵀ = B_w W B_wᵀ Δ`. `R_d` is the covariance of *one measurement
sample*, a sensor averaging over `Δ_v`; a sampler that point-reads the held noise sees the same
covariance only when the noise block's period is the filter's. A datasheet sensor is
`NoiseSource(Gaussian(0, cov=R_d), sample_period=Tₛ)`, typed as it reads. Times Δ on the state,
over Δ on the measurement: the asymmetry students get wrong, and why the rule lives in one place
(DESIGN §4 and one equation comment beside the filter Riccati solve; never a docstring).

**Colored noise is white noise through a filter.** A first-order shaping filter
`τ ż = −z + w`, `w` white of intensity `W`, gives `z` with variance `W / (2τ)` and correlation
time `τ`. The textbook handles it by augmenting the state with `z`; so does a diagram. Its state
starts from `N(0, W / 2τ)` to be stationary from `t = 0` (RD-D23).

## 3. The convention

- **RD-R1. A `Distribution` is always the law of one draw.** `Gaussian(0, σ)` samples have
  standard deviation `σ` everywhere: a start, a parameter, a particle, one held sample. It
  never hides a `dt`. It grows only as A5 plans: `Gaussian(mean, std=None, cov=None)`, a
  `cov` property on every law (`Uniform`: `diag((b − a)² / 12)`; `Particles`: the sample
  covariance), `log_prob`, and `sample(key, n=None, params=None)` like the sets, so a noise
  level can be a parameter and a family of noise levels vmaps like a family of masses.

- **RD-R2. A random signal is a law held over a period, one class per model.** The object that
  carries time is the signal block, never the distribution. Two textbook objects, two classes
  (§4):
  - `NoiseSource(distribution, sample_period=Δ)`: independent draws `w_k ~ p`, held over Δ.
    Sampled noise; any `Distribution`.
  - `WhiteNoise(psd=W, sample_period=Δ)`: continuous white noise of intensity `W`, approximated
    by held samples `w_k ~ N(0, W / Δ)`. A subclass of `NoiseSource` whose per-sample law is
    derived in `h`. Its physics does not change with Δ; a filter design reads its `psd`.

  `WhiteNoise` landed first on its own (RN-1) and becomes the subclass when `NoiseSource` lands
  (RN-3). Colored noise is not a class: `WhiteNoise(...) >> LowPassFilter(...)` is a diagram
  like any other, and inside a plant it is the augmented state of §2.

- **RD-R3. The dt-free number of a white noise is its spectral density.** `psd` is the only
  magnitude `WhiteNoise` takes. A per-sample covariance is a `NoiseSource(Gaussian(0, cov=Σ), Δ)`:
  it keeps each sample when Δ is edited, and so changes the physics, which is what sampled
  noise means.

- **RD-R4. A disturbance is a source block, never a bare law** (taken to its end by R128).
  `disturbances={"d": WhiteNoise(psd=W)}` or `{"d": NoiseSource(Gaussian(0, cov=Σ))}` is the
  shortcut that wires the block onto its port in the problem's system; the same diagram may be
  built explicitly. The object says what its number means in time. With no `sample_period`, the
  tool's `dt` is Δ. A tool refuses a `sample_period` shorter than its `dt`: it holds the input
  over its whole step, so it would read one sample in several and inflate the intensity by
  `dt / Δ`. A bare `Distribution` on a port is refused, naming the fix, on every problem.

- **RD-R4b. Noise reaches a problem by the shortcut or inside `sys`, one mechanism
  underneath.** The shortcut is the wiring itself (R128), so the two routes are one diagram.
  Planners plan with each source at its mean (`nominal()`, RD-D17, RD-D22); the evaluator and
  the RL environment draw on it; a Kalman design reads the noise model from the source. Either
  way every source gets a fresh seed in each trial (§5); a block's own seed is only its default
  realization, used by a plain `compute_trajectory()` and never by an evaluator.

- **RD-R5. A random start is a `Distribution` over `x`, owned by the problem.** The problem's
  `x_start=` takes a point, a `Set` or a `Distribution` (R124, R143); the plant block's `x0` is
  only the fallback when the problem states none. `Sys2Gym` drops `x0_lb / x0_ub / x0_std` and
  takes a `Distribution`, `from_problem` being the route (TODO §1.3).

- **RD-R6. Seeds enter at a tool's boundary as an integer, or live in a block's params.**
  No global RNG anywhere (Constitution §4.4; JAX has none). Every `sample` takes a key: there
  is no unseeded draw in the library. A tool may take `seed=None` for fresh entropy and then
  reports the seed it drew, so the run can be replayed. Inside a tool one key is derived into one
  independent stream per consumer, each from a stable name (a block's id path, a leaf's path)
  with the library's own cipher, `child = F(key, crc32(name))`, the same on both backends
  (RD-D21): adding a law or a random block never reshuffles the others. A `seed` of `None` is
  the mean: no draw (RD-D17). A source's `seed` is an integer leaf, so a raw gradient over the
  tree is `jax.grad(f, allow_int=True)` (R152) and the seed never enters `wrt` (R131).

## 4. The noise block

**Everything `h` reads is in `params`.** The params dict plus the system reproduce a run
exactly, as for any block. Structure stays on the constructor, as the channel count `p` does.

| In `params` | On the constructor (structure) |
| --- | --- |
| `seed` (an integer), `sample_period` Δ, and the magnitude: `psd` for `WhiteNoise` (the two-sided density, `S(ω) = W`, stated in the docstring: a one-sided estimate such as `scipy.signal.welch`'s default is `2W`), the distribution's own numbers under `params["law"]` for `NoiseSource` | `p`, `hold="zoh"` or `"linear"`, the distribution's family for `NoiseSource` |

```python
noise = NoiseSource(Uniform(-0.005, 0.005), sample_period=0.02, seed=3)
noise.params
# {"seed": 3, "sample_period": 0.02, "law": {"lower": [-0.005], "upper": [0.005]}}
```

`NoiseSource` needs the distributions to read `params` (RD-R1, step RN-2); a law that cannot
expose its numbers keeps them as structure on its `NoiseSource`, stated in the docstring.

No `t0` / `tf` window, no drawn table, no `refresh()`. The block's `seed` is its default
realization, as `x0` is the default start: a plain `compute_trajectory()` on a noisy diagram
needs no plumbing.

**The counter-based draw.** An ordinary generator is a state machine: each call advances a
hidden state, so the value depends on how many calls came before. A counter-based generator is
a pure function of its key and a counter:

    bits = F(key, counter)

`F` scrambles its inputs through rounds of add, rotate and xor, like a block cipher: distinct
counters give unrelated outputs, the same inputs the same bits. The noise block uses the seed
as the key and the sample index as the counter:

    k = ⌊t / Δ⌋,      ε_k = F(seed, k) ~ N(0, I)

so `w(t)` is a function of `(t, params)` alone. It is pure, it traces, and it is the same
signal for any solver, any step and any number or order of calls.

**One generator, both backends (RD-D13).** `F` is Threefry-2x32, the cipher JAX's own generator
is built on, with the library's layout: key = seed, counter = `(k mod 2³², channel)`. On NumPy it
is about 25 lines of Python integers under `# Internal machinery`; on JAX the same function. The
bits are identical on both, so the same seed is the same signal whichever backend compiles the
diagram. Two tests pin it: the published Random123 vectors, which need no JAX, and equality with
the JAX primitive when JAX is installed. One named library transform turns the bits into normals.
The layout is the library's own, not `jax.random.fold_in` and `bits`, whose output changes with
the `jax_threefry_partitionable` flag. `NoiseSource` draws on the same cipher.

The white-noise body, three beats (the draw and the index are plumbing, in helpers):

```python
class WhiteNoise(NoiseSource):
    """White noise of intensity W, E[w(t) w(τ)ᵀ] = W δ(t − τ), held over each sample period Δ."""

    def h(self, x, u, t=0.0, params=None):
        seed, W, Δ, hold = params["seed"], params["psd"], params["sample_period"], self.hold

        # the sample index of the held train: w(t) = w_k on [kΔ, (k+1)Δ)
        k = sample_index(t, Δ)
        s = t / Δ - k

        # per-sample covariance of a white noise of intensity W held over Δ: Σ = W / Δ
        Σ = W / Δ
        L = cholesky(Σ)

        # w_k = L ε_k, with ε_k = F(seed, k) ~ N(0, I)
        w_k = L @ standard_normal(seed, k, self.p)
        w_next = L @ standard_normal(seed, k + 1, self.p)

        # zero-order hold, or linear interpolation between w_k and w_k+1
        w = w_k if hold == "zoh" else (1 - s) * w_k + s * w_next

        return w
```

`NoiseSource.h` has the same beats, with `w_k` drawn from its distribution at the counter key
`(seed, k)`. For a diagonal `W` the Cholesky factor is an element-wise square root.

**Three periods, one of them the block's.**

| Period | Owner | What it sets |
| --- | --- | --- |
| Δ, `sample_period` | the noise block | how long each `w_k` is held: `k` is constant on `[kΔ, (k+1)Δ)` |
| Tₛ, the control period | the sampled controller (`ctl % Tₛ`, a `SampledSystem`, R146), or the `dt` of the RL environment and the evaluator | how often the controller acts |
| h, the integration step | the solver (in a sampled loop, the plant's `substeps`, R147) | where `f` and `h` are evaluated; must satisfy `h ≤ Δ` |

The RL environment already works with Δ = Tₛ: it draws a disturbance per control period and
holds it while the plant integrates inside the period.

**The solver (landed with RN-1: RD-D16, RD-D18).** A noisy diagram runs fixed-step RK4 on a
divisor of its period by itself, one step per sample by default; a user-named solver or `dt`
wins, and a fixed `dt` that does not divide Δ warns. A fixed step coarser than Δ holds one sample
over its whole step and so inflates the intensity by `dt / Δ`; RK4's last stage reads the next
sample at a boundary, an effect first order in `dt` and accepted (RD-D18). The traps of the
sample index (floating-point time at a boundary, the 32-bit counter, negative time, 32-bit JAX,
the integer seed in `params`, shared seeds) each have a test. DESIGN §4 states the rule.

## 5. How a stochastic simulation runs

Equation paths are pure (Constitution §5.1), so **a draw never happens inside `f` or `h` from
hidden state**: randomness is a function of data. A *realization* is one parameter tree (R144):

    params        (x_start a value, every law drawn to a value, every source block's seed)

and a stochastic simulation is the deterministic simulation of one realization. Monte Carlo is
`n` realizations, one tree with a trial axis: `n` deterministic simulations, looped on NumPy or
vmapped on JAX. This is the textbook's own scheme (a sample path; Euler–Maruyama draws the
increments, then steps). The judge reads the start from the tree like any leaf, so
`pe.evaluate(params={"x_start": starts})` and a realized tree mean the same thing.

**What `realize` writes, by kind (R144):**
- **a law in a leaf** (`x_start`, a plant parameter): drawn **to a value**, once per trial,
  from a sub-key derived by the leaf's path (`child(key, i, "x_start")`); the realized tree holds
  numbers, never the law or a seed, so the start stays differentiable and inspectable;
- **a source block** (a noise block, the seeded policy of R129): given **an integer seed** leaf
  (`p["d"]["seed"][i]`); the block draws during the run from its seed and the tick;
- the leaves with no law are unchanged.

```python
sys.params = sys.realize(key)                   # every noise block in a diagram, one key
traj = sys.compute_trajectory(tf)

params = problem.realize(key, n_trials=100)     # one tree: params["x_start"] of shape (100, nx)
```

A tool's key is realized once per call: `evaluate(..., key=1)` hands it to
`problem.realize(key, n_trials)`, and every entry `compare` scores reads the same tree (common
random numbers).

**One independent stream per consumer.** Inside a trial there is one stream for the start, one
per law and one per source, each derived by its name, on both backends. A shared stream couples
consumers through the order they draw in. Seed 1, three trials, NumPy, before and after also
randomizing `l`:

| Trial | One shared stream: `m` only | One shared stream: `m` and `l` | Spawned: `m` only | Spawned: `m` and `l` |
| --- | --- | --- | --- | --- |
| 0 | x0 = +0.346, m = +0.822 | x0 = +0.346, m = +0.822 | x0 = −0.054, m = −1.097 | x0 = −0.054, m = −1.097 |
| 1 | x0 = +0.330, m = −1.303 | x0 = −1.303, m = +0.905 | x0 = −1.336, m = +0.851 | x0 = −1.336, m = +0.851 |
| 2 | x0 = +0.905, m = +0.446 | x0 = −0.537, m = +0.581 | x0 = −1.031, m = −0.893 | x0 = −1.031, m = −0.893 |

With streams by name, the only difference between two runs is what was changed. Today's
positional `split_keys` changes with RN-4, so a given seed's numbers change once.

**The same trials on both backends.** Once RN-4 draws the laws with the library's cipher, the
same key gives the same tree on any backend, and the same `J` to rounding (the two backends'
arithmetic already differs by 3e-16 from the second step of a noise-free rollout, so a test
compares with a tolerance); the noise comes from the blocks, already backend-independent
(RD-D13). Any trial replays in the continuous `Simulator`. Cost: a few numbers per trial (80 kB
of seeds for 10,000 trials, against 320 MB for the noise values of 10,000 × 1,000 × 4). RL
training stays key-threaded in JAX, where cross-backend identity buys nothing.

**The evaluator simulates the closed loop (RD-D24, as amended in §8's note).** `evaluate`
builds the loop (`closed_loop_of(problem, arg)`: `ctl @ sys` for a feedback block, `source >>
sys` for an open-loop law) and runs the compiled fixed-step rollout of `problem.compile(ctl, dt)`
on every trial, `vmap`ped on JAX and looped on NumPy (R15; an adaptive solver label is the
escape, one run per trial, R42). The cost is scored on the plant's own run. A controller with
state and the observer-based compensator of P4 go through it; a sampled law is refused until the
sampled face ([differentiable-problem.md](differentiable-problem.md) step 7b), where it closes on
the plant as a `SampledDiagramSystem` (R146). The held-input rollout of `RolloutEnvironment`
stays the learner's stepped view and is not the evaluator's.

## 6. The six uses, one page

The target API (R128, R143, R144); today's problem is `StochasticPlanningProblem(...,
x0_distribution=, params_distribution=)`, swept once with R143.

```python
plant = NoisyPendulum()                        # ports u, d [N m], n [rad/s]

# white noise: the intensity is the dt-free number; w_k ~ N(0, psd / Δ)
w = WhiteNoise(psd=1e-2, sample_period=0.01, seed=1)
v = WhiteNoise(psd=1e-4, sample_period=0.01, seed=2)

# sampled noise: one draw per sensor reading, any law
quantization = NoiseSource(Uniform(-0.005, 0.005), sample_period=0.02, seed=3)

# colored noise: white noise through a shaping filter; τ = 0.5 s, Var[z] = W / 2τ
wind = WhiteNoise(psd=1.0, sample_period=0.01) >> LowPassFilter(cutoff_hz=1 / (2 * np.pi * 0.5))

# random start, random parameters, disturbances: a problem stochastic by its parts
problem = PlanningProblem(
    plant, cost, tf=10.0,
    x_start=Gaussian(x_bar, std=[0.2, 0.1]),
    params={"plant": {"m": Uniform(0.8, 1.2)}},
    disturbances={"d": WhiteNoise(psd=1e-2), "n": WhiteNoise(psd=1e-4)},   # wired onto the ports
)   # n reaches only h: inert in the stepped tools until they read y (§10, RD-A5 and RD-A6)

# LQG: the design reads the same two numbers the simulation draws from
rate = ("y", 1)                                                  # the output the noise corrupts
A, B, C, _ = linearize_matrices(plant, of=rate, wrt=("u", 0))    # the control port alone
_, B_w, _, _ = linearize_matrices(plant, wrt="d")                # B_w = ∂f/∂d
_, _, _, D_v = linearize_matrices(plant, of=rate, wrt="n")       # D_v = ∂h/∂n
W, V = w.psd, v.psd                                              # (1, 1) arrays
K = lqr_gain(A, B, Q_x, R_u)
observer = kalman(LTISystem(A, B, C), B_w @ W @ B_w.T, D_v @ V @ D_v.T)     # Q_n = B_w W B_wᵀ,  R_n = D_v V D_vᵀ

# RL training and Monte Carlo: one integer seed per tool, one key per evaluation
ctl = ReinforcementLearningPlanner(problem, dt=0.05, seed=0).solve().policy
report = evaluate(problem, ctl, dt=0.05, n_trials=100, key=1)   # an Evaluation; report.objective, report.success_rate

# one continuous closed-loop run of a fresh realization, on a fixed step at Δ
loop = ...                                              # a diagram: d and n wired to the plant, ctl @ plant
loop.params = loop.realize(key=3)
traj = loop.compute_trajectory(tf=10.0, solver="rk4_fixedsteps", dt=0.01)
```

## 7. Options weighed

The options weighed for RD-D1–RD-D24 (where a block's randomness lives, what the evaluator fixes
at creation, how noise reaches a problem, the name of `W`, classes or keywords) are in git
history; §8 holds the outcomes. The 2026-10-27 options are in their records (R128, R143, R144).

## 8. Rulings

The record of this plan. Where a later ruling conflicts, it holds (the note at the end).

Decided 2026-09-26 (maintainer):

- **RD-D1. The seed is a parameter, read by a counter-based draw.** `w(t)` is a function of
  `(t, params)`; no drawn table, no `refresh()`.
- **RD-D2. `sample_period` is a parameter.** Everything `h` reads is in `params`, so the params
  dict reproduces a run.
- **RD-D3. Two classes.** `NoiseSource(distribution, sample_period)` for sampled noise;
  `WhiteNoise(psd, sample_period)` its subclass for continuous white noise.
- **RD-D4. The hold is a constructor keyword.** `hold="zoh"` by default; `hold="linear"` kept as
  the option, its `Σ / 2` mid-period dip stated in the docstring.
- **RD-D5. No time window.** `t0` and `tf` leave the block's params; `show_signal(t0=, tf=)` keeps
  its own arguments and evaluates `h` on a grid.
- **RD-D6. A disturbance port takes a signal.** `NoiseSource` or `WhiteNoise`, never a bare
  `Distribution`; no `sample_period` means the tool's `dt`; a shorter one is refused (RD-R4).
- **RD-D7. Both noise routes, one mechanism.** The port route is the documented one for problems; a
  block inside `sys` also works; the evaluator realizes every random block per trial (RD-R4b).
- **RD-D8. The magnitude is named `psd`:** keyword, params key and property; the docstring states
  the two-sided density.
- **RD-D9. Every `sample` takes a key.** `key=None` leaves `Set.sample` and `InputSet.sample`.
- **RD-D10. One independent stream per consumer,** written once inside `realize`.
- **RD-D11. The evaluator holds a test set:** first ruled with the noise values in it; amended by
  RD-D13 (no noise values), then superseded (the note).
- **RD-D12. `NoiseSource` waits for the distributions to read `params`:** RD-R1 lands first,
  `NoiseSource` nests the law's numbers under `params["law"]`; `WhiteNoise` lands before both.

Decided 2026-09-30 (maintainer), from the review of §10:

- **RD-D13. One counter generator on both backends** (was RD-A1): Threefry-2x32 in the library's
  own layout (key = seed, counter = sample index and channel), the same bits on NumPy and JAX
  (§4). It replaced "Philox on NumPy; threefry on JAX" and amended RD-D11.
- **RD-D14. RN-1 lands in v0.2 wave B**, with every cell it touches rewritten in the same commit.
  Landed.
- **RD-D15. The constructor keeps `p` first; the rest is keyword-only.**
  `WhiteNoise(p=1, *, psd=1.0, sample_period=0.01, seed=0, hold="zoh")`, a scalar `psd` meaning
  `W = psd·I`; params `{seed, sample_period, psd}`. `mean` is dropped (a bias is a `Sum` with a
  `Source`). A retired key (`var`, `mean`, `t0`, `tf`) is ignored like any unknown key; the
  docstring carries `psd = var × sample_period`. `psd = 0` and a semidefinite `psd` are legal
  (element-wise root for a diagonal); a negative seed is refused.
- **RD-D16. A noisy diagram picks fixed-step RK4 on a divisor of its period by itself** (one step
  per sample by default). The block publishes `Δ` as `solver_info["smallest_time_constant"]` and
  `solver_info["sample_period"]` (a float gathered as a minimum); `select_solver` maps a held
  signal to `rk4_fixedsteps` when no solver is named, the automatic step being the largest
  `Δ / n` at or under `0.1 × τ_min`. A named solver or `dt` wins. Any block whose output is
  piecewise constant on a known grid may set the flag.
- **RD-D17. `seed = None` is the mean.** `sys.realize(None)` returns the nominal params, every
  random block's seed `None` (a leaf-free pytree node, so it traces), and `h` returns zero there.
  `linearize`, `find_equilibrium`, `transfer_function`, the LQR shortcuts and the deterministic
  transcriptions apply it themselves: a noisy loop linearizes at `E[w] = 0`, and `psd` stays
  readable by the Kalman design.
- **RD-D18. The hold across RK4 stages is accepted as first order in `dt`.** The effect and the
  rule `dt = Δ / n` are stated in the docstring and DESIGN; a fixed step that does not divide Δ
  warns. No integrator change; a solver that samples held sources once per step is a Later row,
  taken up only if a twin test shows the bias matters (RD-A8).
- **RD-D19. No silent noise in the stepped tools.** A disturbance that reaches only `h` is refused
  until a tool reads `y`; the evaluator and the RL environment warn once when the system holds a
  random block they do not realize. With RN-1.
- **RD-D20. Monte Carlo of an observer-based loop is in scope** for RN-4 and RN-5: the
  disturbance sources wired onto the plant ports inside the closed loop, the realization applied
  per trial, the loop stepped at the noise grid, a controller with state accepted.
- **RD-D21. One key per episode; streams derived by name.** The episode's realization (start,
  parameter values, block seeds) is drawn at `reset` and carried, so two episodes never replay
  one noise (§10, finding 8). Each stream comes from a stable name with the library's cipher,
  `child = F(key, crc32(name))`, identical on both backends. `ProblemEnv` realizes at reset; its
  action space is the action port alone.
- **RD-D22. A problem pins its disturbances at their nominal value** for every planner. The bare
  shortcuts on a multi-input system stay as they are: the user passes `wrt=("u", 0)`.
- **RD-D23. The review's text amendments are accepted as written:** RD-A2 (the three conditions,
  the bridge table's two periods, the datasheet sensor, `W = n² / 2`) and RD-A9 into §2; RD-A10
  (the deterministic twin as the acceptance test, the shaping filter's state started from
  `N(0, W / 2τ)`) into §9. `LowPassFilter(tau=)` is a Later row. The fixed-step loops keep
  accumulating `t + dt`; `sample_index` absorbs the drift with a relative tolerance, and 64-bit
  is required on JAX.
- **RD-D24. The evaluator is a batched simulation of the closed-loop diagram,** one path for any
  controller, any noise block, any parameter draw; `dt` the simulation step, so the evaluator's
  numbers change once, with RN-5.

**Superseded in part by later rulings.** The rulings above stand as the history of this plan;
where they conflict, the later rulings hold:

- **R10, R15, R19, R27, R44, R45 (2026-10-22/24;
  [the rulings](../reviews/2026-10-22-rulings-session-1.md)):**
  - RD-D11's test set is not an evaluator object: the draws are `problem.realize`, realized once
    per `evaluate` or `program` call and shared by every entry `compare` scores;
  - RD-D24's run is the compiled fixed-step rollout of `problem.compile(ctl, dt)` on both
    backends, not a loop of `Simulator` runs; `solver="scipy"` the escape (R42); `dt` required
    (R36);
  - `MonteCarloEvaluator` folded into `evaluate` and was removed in v0.3; its record is the one
    `Evaluation`;
  - the score is the integral to `tf` with no cut at exit (R19); the price a switch (R32); a run
    that is not finite is failed (R31);
  - the RL environment keeps its own episode (R43), a training setting reading the problem's
    fields as hints.
- **R128, R143, R144, R146 (2026-10-27):**
  - RD-D6, RD-D7 and RD-D22: the port route is the shortcut that wires a source block into the
    problem's system, one diagram; a bare law refused on every problem;
  - RD-R5: the start law is `x_start=` on either problem class; `StochasticPlanningProblem`,
    `x0_distribution=` and `params_distribution=` go;
  - RD-D10 and RD-D21: a realization is one tree, `x_start` a leaf; laws drawn to values by
    path, sources given seed leaves (§5); "`(x0, params)`" in older text means that tree;
  - the control period's owner is a `SampledSystem`, not a `Computer` (§4).

## 9. Steps: the implementation plan

The rungs and the order are TODO's (§1.1, §1.3). Each step that changes draws records a seeded
baseline before and pins the new numbers with a test after (RULES 7.7, AGENTS refactor recipe);
each step's math is read next to `dp.py` before it lands. The acceptance test of every step is
its deterministic twin (RD-D23). RN-3 comes with its first consumer.

- [x] **RN-1 `WhiteNoise`** (RD-D1–RD-D5, RD-D8, RD-D13–RD-D17, RD-D19). Landed 2026-09-30: the
  cipher (`threefry_2x32`, `standard_normal`, `sample_index`, `child_seed` in
  `core/distributions.py`), the block in `blocks/sources.py`, the solver hints,
  `System.realize(key)` and the stepped-tools warnings. DESIGN §4 ("Randomness") states the
  contract.
- [x] **RN-2 Distributions read `params`** (landed in the v0.3 run, the objects lane; the sets
  with them, a set's numbers its `params` and `with_params` its moved copy) (RD-R1, RD-D9, RD-D12): `Gaussian(cov=)`, `cov` on
  every law, `sample(key, n=None, params=None)`, a `params` dict on `Gaussian`, `Uniform`,
  `Particles`; `Set.sample` and `InputSet.sample` take a required key. Files:
  `core/distributions.py`, `core/sets.py`, every `sample` call site. Baseline: every `sample`
  call site, byte-identical. Done when a family of noise levels vmaps like a family of masses
  (one test).
- [x] **RN-3 `NoiseSource`** (landed in the v0.3 run; its first consumer the `lqg_monte_carlo`
  demo's sensor; a law draws at a `CounterKey(seed, k)` by the cipher) (RD-D3, RD-D12, RD-D23): the law's numbers under `params["law"]`;
  `WhiteNoise` becomes its subclass; the datasheet-sensor recipe
  `NoiseSource(Gaussian(0, cov=R_d), sample_period=Tₛ)` as the first demo. Done when the twin
  test of a sampled sensor passes (the sample covariance against `R_d`) and `WhiteNoise`'s tests
  pass untouched.
- [x] **RN-4 `realize` and the problem** (landed in the v0.3 run, the objects lane: items 1–3; the
  hold rule (4) stands as written; colored noise (5) waits for its demo and twin test) (RD-R4, RD-R4b, RD-D6, RD-D7, RD-D10, RD-D18,
  RD-D20–RD-D22; R128, R143, R144).
  1. `problem.realize(key, n_trials) -> params`, one tree: each law in a leaf drawn to a value
     from a sub-key by its path, each source block (on the shortcut or inside `sys`) given a seed
     leaf, `child_seed(key, path)` as `System.realize(key)` already derives it; the
     `"disturbances"` key and the positional `split_keys` go. Files: `core/system.py`,
     `core/wiring.py`, `planning/problems.py`, `core/distributions.py`, `planning/evaluator.py`.
  2. Laws in the tree's shape on both problem classes (R143): `x_start=` takes a law,
     `params=` takes laws by path, `criterion=` and `nominal()` on the class;
     `StochasticPlanningProblem`, `x0_distribution=` and `params_distribution=` go, every caller
     in the same commit (R82). `disturbances` takes source blocks only and wires them (R128);
     a planner plans at the mean (`nominal()`, RD-D22).
  3. `RolloutEnvironment.reset(key)` returns the episode's realization and `step` reads each
     source's `h` at `t_k` with the episode's seeds (RD-D21); `ProblemEnv` realizes at reset, its
     action space the action port alone. Files: `learning/reinforcement_learning/environment.py`,
     `interfaces/gymnasium.py`.
  4. The hold across stages: the RD-D18 rule in DESIGN and the block's docstring. No integrator
     change.
  5. Colored noise: the recipe of §6 with the shaping filter's state started from
     `N(0, W / 2τ)` (RD-D23), as a demo and its twin test (`W = 2 τ σ²`).

  A seeded baseline records the one change of draws. Done when `problem.realize(key)` is
  bit-identical on NumPy and JAX, two episodes from one training key draw distinct noise, and
  adding a law or a source leaves every other stream unchanged (the table of §5, by name).
- [ ] **RN-5 Monte Carlo is the closed loop on the draws** (RD-D20, RD-D24 as amended in §8's
  note; step 7 of [differentiable-problem.md](differentiable-problem.md), the same row). Its v0.3
  part landed 2026-10-24 (`evaluate` on `closed_loop_of(problem, arg)`, the starts and parameter
  laws realized once, the compiled fixed-step rollout, `compare` and `pe.program` on the same
  draws, `MonteCarloEvaluator` and the `simulator` backend removed). What is left:
  1. the disturbance sources in the evaluated loop with RN-4's seeds (today `evaluate` holds the
     declared ports at nominal and warns, `warn_if_disturbed`);
  2. `Sys2Gym` takes a `Distribution` for the start (RD-R5); `x0_lb / x0_ub / x0_std` go.

  Done when an LQG loop (P4's observer and gain, noise on `d` and `n`) scores a mean cost within
  the sampling error of the covariance prediction (1.013 ± 0.023 measured by hand, §10 RD-A6);
  the same `evaluate` call on JAX and on NumPy agrees to a tolerance on one key; the
  deterministic twin (`n_trials=1` on the nominal problem) equals the deterministic `evaluate`;
  the release note records the one change of the evaluator's numbers.
- [ ] **RN-6 Docs**, with each step: DESIGN §4 (the distributions entry: the convention, the
  three periods, the bridge table with its two periods, the solver hint) and §6 (the problem's
  laws and sources, `realize`, `evaluate` on the draws). Done so far: DESIGN §4 ("Randomness")
  for RN-1 and §6 ("The problem evaluator") for `evaluate` and `problem.realize(key, n)`. Then
  TODO A5 rows retired; `kalman()` (P4) reads intensities, the discrete filter later reads `Q_d`,
  `R_d`; the release notes of RN-1, RN-4 and RN-5; CONSTITUTION §2's disturbance sentence
  amended with the landing change (R128).

Out of scope on purpose: time-varying laws `p(t)` (a gust is `Step` times a noise source),
a `ColoredNoise` class (a filter), a stochastic system wrapper (randomness enters only through
named ports and seeded sources, RULES 4.9), a Brownian-bridge refinement that keeps one path
across changes of Δ, RL with observation noise until a learner reads `y`.

## 10. Review 2026-09-30: findings and amendments

A second opinion on RD-D1–RD-D12, measured on the code of that time and on a prototype of the §4
block. The convention held; the findings are around it, kept as the evidence. All ten amendments
are ruled (RD-A1 as RD-D13, RD-A2–RD-A10 as RD-D14–RD-D23, §8).

### Findings

1. **"Its physics does not change with Δ" holds under three conditions** (now in §2):
   - the port enters `f` affinely, with a gain that does not depend on the state: for
     `v̇ = F − c (v − w)|v − w|` with `WhiteNoise(psd=1)` on the wind port, the mean speed went
     0.80, 0.44, 0.24, 0.066 at Δ = 0.1, 0.03, 0.01, 0.001 (2.0 without wind); the same psd
     through a 0.5 s filter gave 1.79 at every Δ; with a state-dependent gain the held sample
     integrated finely is the Stratonovich solution and Euler at dt = Δ the Itô one;
   - the consumer integrates the noise: a sampler point-reading `y = x + v` sees `V / Δ_v`; a
     discrete Kalman filter tuned with `R_d = V / Tₛ` at `Tₛ / Δ_v = 10` had an error variance
     7.1 times its prediction;
   - Δ is small against the loop and the observer: an observer pole near 10 rad/s and Δ = 0.1 s
     gave a discrete filter covariance 1.53 times the continuous one.
2. **The port route and the block route were two numerics:** a port disturbance held over the
   four RK4 stages, a block read at each stage time (indices `(k, k, k, k+1)`); 8.7e-4 apart after
   50 pendulum steps. With R128 both are a block (RD-D18 for the stage effect).
3. **RN-1 removed an accidental guard:** with the pure block, `linearize`, `transfer_function`
   and the LQR shortcuts ran at `t = 0` on the draw `w(0)`; hence RD-D17's mean at `seed=None`.
   Planners then treated a declared disturbance port as an actuator (fixed 2026-10-14).
4. **RD-D11's noise array reached ports only,** and Philox on NumPy against threefry on JAX gave
   J = 4.664 and 4.579 on one trial seed. A Threefry-2x32 cipher written once is bit-identical on
   both, passes the Random123 vectors, and costs 4.35 µs a call; hence RD-D13.
5. **Measurement noise was inert in the stepped tools:** they fed the true state to a static
   law; J with `v ~ N(0, 5²)` equalled J with no noise to 16 digits.
6. **The default run degraded between RN-1 and RN-4:** RK45 at `rtol = 1e-4` gave 12% rms path
   error and ran 4 to 7 times slower; RK4 at dt = Δ ten times faster; hence RD-D16.
7. **`cholesky(W / Δ)` failed when the noise was off** (`psd = 0`, one quiet channel); hence
   RD-D15's element-wise root.
8. **RL episodes restart at `t = 0`**, so with a fixed block seed the second episode replayed the
   first one's noise exactly; hence RD-D21.
9. **The colored-noise recipe started quiet:** the filter state at zero gave an ensemble standard
   deviation of 43% of stationary at 0.1 τ, and a 10 s Monte Carlo cost 11% low at τ = 2 s; hence
   the stationary start of RD-D23.
10. **RN-1 was not a technical prerequisite of P4:** `kalman_gain(A, C, Q_n, R_n)` takes arrays;
    the GRO501 notebook was the first consumer of `psd` with `kalman()`.

### Amendments (all ruled, §8)

- **RD-A1:** one counter generator on both backends (RD-D13).
- **RD-A2:** the three conditions of finding 1 and the bridge table's two periods (RD-D23, §2).
- **RD-A3:** random blocks enter analysis at their mean (RD-D17).
- **RD-A4:** RN-1 safe on its own: the solver hint, the constructor, the zero `psd` (RD-D15,
  RD-D16).
- **RD-A5:** no silent noise in the stepped tools (RD-D19).
- **RD-A6:** Monte Carlo of an observer-based loop in scope (RD-D20). Checked by hand on an LQG
  compensator and `NoisyPendulum`, a noise block on each port with a fresh seed per trial (RK4 at
  dt = Δ): a mean cost of 1.013 ± 0.023 times the covariance prediction over 240 trials, RN-5's
  gate.
- **RD-A7:** one key in the RL environment, streams by name (RD-D21).
- **RD-A8:** the hold across integrator stages stated, not changed (RD-D18); evaluating a held
  source once per step stays a Later option.
- **RD-A9:** what an H2 or H∞ tool reads (RD-D23, the table below). `psd` is an LQG and H2
  number (under `W = I` the output variance is `‖G‖₂²`, checked 1.989062 against 1.989063); an H∞
  tool reads the ports and weighting filters, no distribution.
- **RD-A10:** the deterministic twin as the acceptance test (RD-D23, §9).

### What each tool reads

| Tool | Reads from the noise model |
| --- | --- |
| Simulation | the block's `h(t; params)` |
| Monte Carlo, RL | a realization (one tree) per trial or episode; the per-sample law at the tool's dt |
| Kalman–Bucy, LQG | `B_w`, `D_v`, the intensities `W`, `V` |
| Discrete Kalman, EKF, particle filter | the per-sample covariance or law at the filter's period |
| H2 | the ports and `W` |
| H∞ | the ports and weighting filters; no distribution |
| Identification | the per-sample output covariance at the log rate |
| Stochastic DP (not planned) | the per-step law as points and weights at the grid's dt |
