# One convention for randomness

Status: design agreed 2026-09-26 (§8, D1–D12), amended 2026-09-30 (D13); not started.
Reviewed 2026-09-30: factual corrections applied in place; ten amendments proposed in §10.
A1 is ruled (D13: one counter generator on both backends, which amends D11); A2–A10 await
the maintainer's ruling.
Rungs (decided 2026-09-26): RN-1 (`WhiteNoise`) in v0.2 wave B, the prerequisite of B3
(`estimation/`, P4); RN-2 in v0.2 wave A, with A5; RN-3 with its first consumer (P11 if a GRO501
notebook shows sampled sensor noise, else v0.3); RN-4 and RN-5 together in v0.3, after the fall
term, because they change public behaviour and the Monte Carlo numbers once, and before the v0.9
freeze; RN-6 with each step. It settles the disturbance convention ROADMAP §6
held open before B3, closes finding F9 of
[2026-09-15-foundations-review.md](../reviews/2026-09-15-foundations-review.md), and absorbs the
`Gaussian(cov=)`, `NoiseSource`, `Distribution.sample(key=None)` and `WhiteNoise` guard-test
rows of TODO A5.
Consumers: the `WhiteNoise` block, a colored-noise recipe, random starts, LQG, RL training,
Monte Carlo evaluation. One question underneath all six: **what a number on a noise means in
time**, and **where a random draw happens** in a library whose equation paths are pure.

## The design in one screen

- **A `Distribution` has no time.** It is the law of one vector: a start, a parameter value,
  one sample of a noise.
- **A noise signal is a block that holds draws over a sample period Δ.** `NoiseSource(law, Δ)`
  holds independent draws `w_k ~ law`: sampled noise. `WhiteNoise(psd=W, Δ)` is continuous
  white noise of two-sided spectral density `W`, drawn as `w_k ~ N(0, W / Δ)`, so its physics
  does not change when Δ does. Colored noise is white noise through a filter block.
- **The block is pure.** Its seed, Δ and magnitude are params. `h` computes `w(t)` from
  `(t, params)` with a counter-based generator, `ε_k = F(seed, ⌊t/Δ⌋)`: no drawn table, no time
  window, no `refresh()`, the same signal for every solver and on both backends, and it traces
  under JAX. Zero-order hold by default, linear interpolation as an option.
- **One key reproduces an experiment.** `realize(key)` gives every random block and every random
  parameter its own independent stream, following the diagram and the problem.
- **A problem declares noise on ports**, `disturbances={"w": WhiteNoise(psd=W)}`; a noise block
  already inside the system also works. Planners see the nominal port; the RL environment and
  the evaluator draw on it.
- **The Monte Carlo evaluator holds a fixed test set.** Starts, parameter values and seeds are
  drawn once, on NumPy; the noise follows from the seeds, the same on every backend. Every law
  is scored on the same trials, whatever backend rolls it out.
- **LQG reads the same numbers the simulation draws from**: `Q = B_w W B_wᵀ`, `R = V` for the
  continuous filter; `Q_d = B_w W B_wᵀ Δ`, `R_d = V / Δ` for the discrete one.

## 1. The code as it stands

| Site | What "random" means there | Time semantics |
| --- | --- | --- |
| `Distribution` (`core/distributions.py`) | law of one draw: `sample(key, n)`, `mean`, `support` | none, correct |
| `WhiteNoise` (`blocks/sources.py`) | Gaussian table drawn at `refresh()`, linear interpolation, `params["seed"]`, NumPy only | per-sample `var` at its own `sample_period` |
| `StochasticPlanningProblem.disturbances` | `{port: Distribution}`, a fresh draw per step, held | per-sample at whatever `dt` the tool picks |
| `RolloutEnvironment`, `MonteCarloEvaluator` | draw from the problem at the tool's `dt` | inherit the ambiguity above |
| `Sys2Gym(sys)` | `x0_lb / x0_ub / x0_std` of its own | per trial, a second start-state dialect |
| Tools (`RRT`, tabular RL, RL planner, evaluator, `lyapunov`) | `seed: int`, own generator or key inside | fine |

Consequences:

- `Gaussian(0, 0.1)` on port `w` is a different physical disturbance for a planner at
  `dt = 0.05` and an evaluator at `dt = 0.01`; `WhiteNoise` has the same problem with `var`
  plus `sample_period`. A Kalman filter cannot be designed against either without a rule.
- `WhiteNoise` returns its mean outside `[t0, tf]` (default `[−100, 100]` s): a simulation past
  100 s runs noise-free without a warning (`h(150) = 0`, checked 2026-09-26).
- `WhiteNoise.h` refuses a `params` argument and does not trace (`interp1d`); editing its params
  needs `refresh()`, hidden state.
- Two `WhiteNoise()` blocks share the default `seed = 0`: in `diagram_noise_ports.py` the process
  noise and the measurement noise are the same signal whenever both are on.
- A noise block wired inside a problem's `sys` is frozen across Monte Carlo trials: its seed
  never changes, so every trial sees the same noise signal (pendulum, `WhiteNoise` on `w`,
  state feedback, three trials from one start: J = 3.96004 in all three, checked 2026-09-26).
  The evaluator also falls back to NumPy, because the block does not trace, with a warning
  about draws that does not name the cause; the `simulator` backend fails on that system with a
  nested-diagram error (a wiring gap, tracked apart from this plan).
- `Set.sample` accepts `key=None` and `Distribution.sample` does not; `split_keys` gives
  independent keys on JAX but the same generator repeated on NumPy, so the two backends do not
  have the same tree of streams. On NumPy, adding one random parameter shifts every later draw
  of the run: with seed 1, randomizing `l` beside `m` turned trial 1's start into the value
  that was trial 1's mass (§5).
- Only one test passes `disturbances=` (`test_rl_planner.py`); no demo and no notebook does.

The nine modules that construct a generator or a key (`backends`, `neural`, `sources`,
`lyapunov`, `evaluation`, `tabular`, the RL `planner`, `rrt`, `rrt_star`) are all constructors
or tool boundaries, which is the right list.

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
noise halves. That is F9 in one line.

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
variance, 0.1% at the sample instants). For noise that really is
sampled (a fresh sensor reading, a disturbance drawn per control step), Δ is the physical
period and the per-sample law is the natural number.

**The hold shape.** A held train is `w(t) = Σ_k w_k φ(t − kΔ)`. The zero-order hold uses a box
kernel of width Δ; linear interpolation a triangle of width 2Δ. Both kernels have area Δ, so
both signals have the same low-frequency spectral density `Σ Δ = W`, and the plant sees the same
intensity (Var[x] matched within one realization's sampling error for both, 2026-09-26). They
differ elsewhere: the zero-order hold has pointwise variance `Σ` everywhere; linear
interpolation dips to `Σ / 2` at mid-period (`(1 − s)² + s²`), rolls off sooner (`sinc⁴`
against `sinc²`), and no longer matches the discrete map sample by sample.

**The bridge to discrete time, once.** With the disturbance entering through `B_w = ∂f/∂w`
and a measurement `y = h(x) + v`, `v` white of intensity `V`:

| Quantity | Continuous (Kalman-Bucy, dual of `lqr`) | Held at Δ (discrete Kalman) |
| --- | --- | --- |
| process noise | `Q = B_w W B_wᵀ` | `Q_d = B_w W B_wᵀ Δ` (first order; exactly `∫₀^Δ e^{Aτ} B_w W B_wᵀ e^{Aᵀτ} dτ`) |
| measurement noise | `R = D_v V D_vᵀ` | `R_d = D_v V D_vᵀ / Δ` |

`D_v = ∂h/∂v` maps the noise onto the outputs it corrupts (`R = V` when `y = h(x) + v`); an
output with no noise on it makes `R` singular, and the continuous filter needs `R > 0`.
`Q_d` is the covariance of the noise *added to `x_{k+1}`*: the held sample `Δ B_w w_k` has
covariance `Δ² B_w (W/Δ) B_wᵀ = B_w W B_wᵀ Δ`. `R_d` is the covariance of *one measurement
sample*, a sensor averaging over `Δ`; a sampler that point-reads the held noise sees the same
covariance only when the noise block's period is the filter's (§10, A2). Times Δ on the state,
over Δ on the measurement: the
asymmetry students get wrong, and why the rule lives in one place (DESIGN §4 and one equation
comment beside the filter Riccati solve; never a docstring).

**Colored noise is white noise through a filter.** A first-order shaping filter
`τ ż = −z + w`, `w` white of intensity `W`, gives `z` with variance `W / (2τ)` and correlation
time `τ`. The textbook handles it by augmenting the state with `z`; so does a diagram.

## 3. The convention

- **R1. A `Distribution` is always the law of one draw.** `Gaussian(0, σ)` samples have
  standard deviation `σ` everywhere: a start, a parameter, a particle, one held sample. It
  never hides a `dt`. It grows only as A5 plans: `Gaussian(mean, std=None, cov=None)`, a
  `cov` property on every law (`Uniform`: `diag((b − a)² / 12)`; `Particles`: the sample
  covariance), `log_prob`, and `sample(key, n=None, params=None)` like the sets, so a noise
  level can be a parameter and a family of noise levels vmaps like a family of masses.

- **R2. A random signal is a law held over a period, one class per model.** The object that
  carries time is the signal block, never the distribution. Two textbook objects, two classes
  (§4):
  - `NoiseSource(distribution, sample_period=Δ)`: independent draws `w_k ~ p`, held over Δ.
    Sampled noise; any `Distribution`.
  - `WhiteNoise(psd=W, sample_period=Δ)`: continuous white noise of intensity `W`, approximated
    by held samples `w_k ~ N(0, W / Δ)`. A subclass of `NoiseSource` whose per-sample law is
    derived in `h`. Its physics does not change with Δ; a filter design reads its `psd`.

  That is the final shape; `WhiteNoise` lands first on its own (RN-1), and becomes the subclass
  when `NoiseSource` lands (RN-3).

  Colored noise is not a class: `WhiteNoise(...) >> LowPassFilter(...)` is a diagram like any
  other, and inside a plant it is the augmented state of §2.

- **R3. The dt-free number of a white noise is its spectral density.** `psd` is the only
  magnitude `WhiteNoise` takes. A per-sample covariance is a `NoiseSource(Gaussian(0, cov=Σ), Δ)`:
  it keeps each sample when Δ is edited, and so changes the physics, which is what sampled
  noise means.

- **R4. On a problem, a disturbance port takes a signal, never a bare law.**
  `disturbances={"w": WhiteNoise(psd=W)}` or `{"w": NoiseSource(Gaussian(0, cov=Σ))}`: the
  object says what its number means in time. With no `sample_period`, the tool's `dt` is Δ,
  and a `WhiteNoise`'s physics does not change between a planner at `0.05 s` and an evaluator at
  `0.01 s`. A tool refuses a `sample_period` shorter than its `dt`: it holds the input over its
  whole step, so it would read one sample in several and inflate the intensity by `dt / Δ`.
  A bare `Distribution` on a port is refused with a message naming the two signals.

- **R4b. Noise reaches a problem by a port or inside `sys`, one mechanism underneath.** The
  port route (`disturbances=`) is shorthand for wiring that block onto the port in every trial,
  and it is the documented route for problems: planners plan on `sys` with the port at its
  nominal value, the evaluator and the RL environment draw on it, and a Kalman design reads the
  noise model from the problem. A noise block already wired inside `sys` also works (the natural
  way to build a simulation diagram). Either way the evaluator realizes every random block with
  fresh seeds in each trial (§5); a block's own seed is only its default realization, used by a
  plain `compute_trajectory()` and never by an evaluator. ("Nominal" is kept for the mean value
  of a port.)

- **R5. A random start is a `Distribution` over `x`, owned by the problem; `sys.x0` is its
  mean.** Already so for `StochasticPlanningProblem`. `Sys2Gym` drops `x0_lb / x0_ub /
  x0_std` and takes a `Distribution`, `from_problem` being the route (TODO row exists).

- **R6. Seeds enter at a tool's boundary as an integer, or live in a random block's params.**
  No global RNG anywhere (Constitution §4.4; JAX has none). Every `sample` takes a key: there
  is no unseeded draw in the library. A tool may take `seed=None` for fresh entropy and then
  reports the seed it drew, so the run can be replayed. Inside a tool one generator or one JAX
  key is split into one independent stream per consumer, on both backends (`Generator.spawn` on
  NumPy, written once in `realize`). A `Distribution.sample` gives different numbers on NumPy
  and JAX for the same seed, with the same law; the evaluator removes that difference by drawing
  its starts and parameter values once on NumPy (§5). A noise block gives the same signal on
  both (D13).

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

`NoiseSource` needs the distributions to read `params` (R1); a `Sampler` exposes only what its
caller hands it, so a `NoiseSource` on one keeps its numbers as structure, stated in the
docstring. `WhiteNoise` needs none of this and lands first.

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

**One generator, both backends (D13).** `F` is Threefry-2x32, the cipher JAX's own generator is
built on, with the library's layout: key = seed, counter = `(k mod 2³², channel)`. On NumPy it
is about 25 lines of Python integers under `# Internal machinery`; on JAX the same function
(`jax.extend.random.threefry_2x32`, or the same code on `jax.numpy`). The bits are identical on
both, so the same seed is the same signal whichever backend compiles the diagram. Two tests
pin it: the published Random123 vectors, which need no JAX, and equality with the JAX primitive
when JAX is installed. One named library transform turns the bits into normals. The layout is
the library's own, not `jax.random.fold_in` and `bits`, whose output changes with the
`jax_threefry_partitionable` flag. Checked 2026-09-26: every
value `h` returned under RK45 (6,422 calls), LSODA (4,237) and RK45 with the step capped at Δ/7
(6,314) equalled `F(seed, ⌊t/Δ⌋)`, with no disagreement.

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
| Tₛ, the control period | a `Computer`, or the `dt` of the RL environment and the evaluator | how often the controller acts |
| h, the integration step | the solver | where `f` and `h` are evaluated; must satisfy `h ≤ Δ` |

The RL environment already works with Δ = Tₛ: it draws a disturbance per control period and
holds it while the plant integrates inside the period.

**The solver.** No minilink solver breaks with a zero-order hold. Prototype 2026-09-26
(scratchpad, not in the library): `PendulumWithNoisePort` driven on `w`, Δ = 10 ms, 20 s.

| Solver | ZOH: `h` calls per sample | ZOH: time | Linear: calls per sample | Linear: time |
| --- | --- | --- | --- | --- |
| `scipy` (default) | 21.6 | 1.2 s | 8.1 | 0.6 s |
| `scipy_lsoda` | 87.2 | 4.4 s | 46.9 | 3.0 s |
| `scipy_stiff` | 67.2 | 5.0 s | 24.3 | 2.2 s |
| `rk4_fixedsteps`, dt = Δ | 4.0 | 0.19 s | 4.0 | 0.25 s |
| `rk4_fixedsteps`, dt = Δ/5 | 20.0 | 1.0 s | 20.0 | 1.3 s |
| `euler_fixedsteps`, dt = Δ/10 | 10.0 | 0.5 s | 10.0 | 0.6 s |

- The adaptive solvers are not accurate on noise with either hold: their final angles scatter
  by a few percent around the fine fixed-step result, and the stiff solver by about 20%. A
  rough signal defeats step-size control however it is smoothed.
- A fixed step aligned to Δ is the fastest and the most accurate: the recommended setup for a
  noisy simulation, and what the warning points to.
- A fixed step coarser than Δ reads the right signal and gets the wrong physics: it holds one
  sample for its whole step, so the intensity it feels is multiplied by `dt / Δ` (Euler on
  `ẋ = −x + w`, theory 0.0100: Var[x] = 0.0098, 0.0099, 0.0205, 0.0523 at dt = Δ/2, Δ, 2Δ, 5Δ).
- With RK4 at dt = Δ, the last stage of a step lands on a jump and reads the next sample,
  shifting the zero-order-hold result by about 0.5% against a finer step. Linear interpolation
  gave the same final angle at both steps. The fix is the left limit at that stage. Measured
  2026-09-30 on a faster plant (`ẋ = −10x + w`, dt = Δ = 0.01): 7.5% rms path error and −2.6%
  on the variance, first order in dt; the same samples held on an input port give 6e-7. A
  nudge of the stage time does not give the left limit (§10, A8).

**Traps, each with a test.**

- *Floating-point time at a boundary.* `floor(0.3 / 0.1) = 2`. With time accumulated by a fixed
  step `dt = Δ`, a plain `floor` picked the previous sample at 398 of 999 boundaries.
  `sample_index` uses a tolerance, `⌊t/Δ + ε⌋`; stepped tools pass the integer `k` directly.
  Measured 2026-09-30: 798 of 999 at Δ = 0.01, and no fixed ε survives a long run, because the
  fixed-step rollouts accumulate `t = t + dt` (ε = 1e-9 misreads 83,000 of 100,000 boundaries
  at 1000 s). `t_k = t0 + k·dt` with ε = 1e-9, or a relative tolerance of 1e-10, held over
  2·10⁶ steps (§10, A4).
- *The counter.* One cipher block per sample and channel, so no stream runs into the next. The
  counter word is 32 bits: the signal repeats after 2³² samples (49.7 days at Δ = 1 ms), stated
  in the docstring.
- *Negative time.* A negative `k` maps to `k mod 2³²`, the same on both backends. A float cast
  to uint32 saturates at zero, so `sample_index` casts once, float to int64 then modulo. A
  negative seed is refused at construction (corrected 2026-09-30).
- *32-bit JAX.* In float32 the index is wrong from the first boundaries: at Δ = 1 ms, `k = 4`
  already, and 11,564 repeats over 1000 s (corrected 2026-09-30). The library's 64-bit default
  (`MINILINK_JAX_X64`) is applied when the first JAX evaluator is built, not at import; the
  block warns when it is off.
- *An integer in params.* The params Jacobian differentiates float leaves only
  (`core/compile/evaluators/jacobian.py`), so it skips the seed. A raw `jax.grad` over the whole
  params dict raises (`allow_int=True` is the escape). `params_distribution` over a seed is
  refused with a message pointing to `realize`: a distribution over seeds is Monte Carlo.
  A test checks the seed stays an integer through a parameter-family vmap.
- *Shared seeds.* Compiling a diagram warns when two random blocks hold the same seed.

**Cost against today's block.** On NumPy a Threefry draw in Python integers costs 4.35 µs per
call at one channel and 12 µs at three, against 2.8 µs for today's interpolation lookup
(measured 2026-09-30; Philox 5.45 µs, `default_rng([seed, k])` 4.8 µs). A 20 s noisy pendulum
at Δ = 10 ms runs in the same time with either generator. Under JAX jit a draw at each RK4
stage costs 114 to 170 ns per trial-step against 35 ns for the same diagram with a constant
source. Existing realizations change: `seed = 1` gives a different signal (a release note;
demos and tests that pin noise values get new baselines).

## 5. How a stochastic simulation runs

Equation paths are pure (Constitution §5.1), so **a draw never happens inside `f` or `h` from
hidden state**: randomness is a function of data. A *realization* is

    (x0, params)            (params holding the drawn parameter values and every random block's seed)

and a stochastic simulation is the deterministic simulation of one realization. Monte Carlo is
`N` realizations: `N` deterministic simulations, looped on NumPy or vmapped on JAX. This is the
textbook's own scheme (a sample path; Euler–Maruyama draws the increments, then steps).

The continuous simulation and the stepped tools read the same process. A `Simulator` evaluates
the block's `h` at the solver's times; the RL environment and the evaluator can evaluate the
same `h` at `t_k` with the episode's seed. With the same seed and Δ = Tₛ, both see the same
`w_k`, sample by sample.

**One key per experiment.** A diagram gathers params from its subsystems; it gathers
realizations the same way. `sys.realize(key)` splits the key once per random subsystem, in
subsystem order, and returns the params dict with a fresh seed in every random block (nested,
like params). A problem does the same one level up:

    sys.params = sys.realize(key)                   # every noise block, one key
    traj = sys.compute_trajectory(tf)

    x0, params = problem.realize(key)               # start, parameters, disturbances
    trials = vmap(problem.realize)(split(key, N))   # Monte Carlo = a params family

Block seeds are not re-plumbed one by one; the key tree is the diagram structure. A tool's
`seed` (`MonteCarloEvaluator(seed=1)`) calls `realize` once per trial.

A problem realizes its port route the same way: each `disturbances` entry is a noise block the
tools wire onto its port, so `problem.realize(key)` returns a start, the drawn parameter values,
and a fresh seed for each disturbance block and for each random block already inside `sys`.

**One independent stream per consumer.** `realize` spawns a child stream for each trial and,
inside a trial, one for the start, one per random parameter and one per random block, on both
backends. A shared stream couples consumers through the order they draw in. Seed 1, three
trials, NumPy, before and after also randomizing `l`:

| Trial | One shared stream: `m` only | One shared stream: `m` and `l` | Spawned: `m` only | Spawned: `m` and `l` |
| --- | --- | --- | --- | --- |
| 0 | x0 = +0.346, m = +0.822 | x0 = +0.346, m = +0.822 | x0 = −0.054, m = −1.097 | x0 = −0.054, m = −1.097 |
| 1 | x0 = +0.330, m = −1.303 | x0 = −1.303, m = +0.905 | x0 = −1.336, m = +0.851 | x0 = −1.336, m = +0.851 |
| 2 | x0 = +0.905, m = +0.446 | x0 = −0.537, m = +0.581 | x0 = −1.031, m = −0.893 | x0 = −1.031, m = −0.893 |

With spawned streams, the only difference between two runs is what was changed. The switch
changes the numbers a given seed produces once, in the `realize` step, and adds a
`numpy>=1.25` floor (`Generator.spawn`).

**The evaluator holds a fixed test set.** A trial needs a start, parameter values and noise
signals. Starts and parameter values come from `Distribution.sample`, whose numbers differ
between NumPy and JAX; the noise comes from the block, whose numbers do not (D13). So
`MonteCarloEvaluator` draws its test set once, on NumPy: the starts and the params (values and
block seeds), `ev.trials`. A trial is a realization `(x0, params)` and nothing else. Every law
is rolled out on it, whatever backend runs the rollout, on a port or inside `sys`. Hence

```python
ev = MonteCarloEvaluator(problem, n_trials=100, seed=1)   # draws ev.trials once
ev.evaluate(lqr)   # rolled out on JAX, on ev.trials
ev.evaluate(vi)    # rolled out on NumPy, on the same ev.trials: J comparable
```

The same seed gives the same trials on any backend, and the same `J` to rounding (the two
backends' arithmetic already differs by 3e-16 from the second step of a noise-free rollout, so
a test compares with a tolerance); the "one seed, two streams" warning and `compare`'s NumPy
forcing go; any trial replays in the continuous `Simulator`. Cost: a few numbers per trial
(80 kB of seeds for 10,000 trials, against 320 MB for the noise values of 10,000 × 1,000 × 4).
The options weighed are in §7. RL training stays key-threaded in JAX, where cross-backend identity buys nothing.

## 6. The six uses, one page

```python
plant = PendulumWithNoisePort()                        # ports u, w [N m], v [rad/s]

# white noise: the intensity is the dt-free number; w_k ~ N(0, psd / Δ)
w = WhiteNoise(psd=1e-2, sample_period=0.01, seed=1)
v = WhiteNoise(psd=1e-4, sample_period=0.01, seed=2)

# sampled noise: one draw per sensor reading, any law
quantization = NoiseSource(Uniform(-0.005, 0.005), sample_period=0.02, seed=3)

# colored noise: white noise through a shaping filter; τ = 0.5 s, Var[z] = W / 2τ
wind = WhiteNoise(psd=1.0, sample_period=0.01) >> LowPassFilter(cutoff_hz=1 / (2 * np.pi * 0.5))

# random start, random parameters, disturbances: one problem owns every law
problem = StochasticPlanningProblem(
    plant, cost, tf=10.0,
    x0_distribution=Gaussian(x_bar, std=[0.2, 0.1]),
    params_distribution={"m": Uniform(0.8, 1.2)},
    disturbances={"w": WhiteNoise(psd=1e-2), "v": WhiteNoise(psd=1e-4)},
)   # v reaches only h: inert in the stepped tools until they read y (§10, A5 and A6)

# LQG: the design reads the same two numbers the simulation draws from
rate = ("y", 1)                                                  # the output the noise corrupts
A, B, C, _ = linearize_matrices(plant, of=rate, wrt=("u", 0))    # the control port alone
_, B_w, _, _ = linearize_matrices(plant, wrt="w")                # B_w = ∂f/∂w
_, _, _, D_v = linearize_matrices(plant, of=rate, wrt="v")       # D_v = ∂h/∂v
W, V = w.psd, v.psd                                              # (1, 1) arrays
K = lqr_gain(A, B, Q_x, R_u)
observer = kalman(A, B, C, B_w @ W @ B_w.T, D_v @ V @ D_v.T)     # Q = B_w W B_wᵀ,  R = D_v V D_vᵀ

# RL training and Monte Carlo: one integer seed per tool, draws at the tool's dt
ctl = ReinforcementLearningPlanner(problem, dt=0.05, seed=0).solve().ctl
report = MonteCarloEvaluator(problem, dt=0.05, n_trials=100, seed=1).evaluate(ctl)

# one continuous closed-loop run of a fresh realization, on a fixed step at Δ
loop = ...                                              # a diagram: w and v wired to the plant, ctl @ plant
loop.params = loop.realize(key=3)
traj = loop.compute_trajectory(tf=10.0, solver="rk4_fixedsteps", dt=0.01)
```

## 7. Options weighed

**Where a block's randomness lives.**

| Option | Pure `h` | Edit without `refresh()` | Traces, vmaps | Memory | Window |
| --- | --- | --- | --- | --- | --- |
| Seed on the constructor, table drawn once | yes | no | no | grows with the span | yes |
| Seed in params, table rebuilt on `refresh()` (today) | until params change | no; passing `params` raises | no | grows with the span | yes, ends at 100 s |
| The drawn table in params | yes | yes | yes | trials × steps × channels | yes |
| **Seed in params, counter-based draw (D1)** | yes | yes | yes | one integer | none |
| No seed; the tool passes a key to `h` | yes | yes | yes | none | none; breaks the one signature |

**Who sets it for an experiment.** The block's own seed (its nominal realization), `realize(key)`
along the diagram, and a tool's integer seed layer rather than compete (§5).

**What the evaluator fixes at creation.** The scenario: one evaluator scores an LQR law, which
rolls out on JAX, and a DP lookup table, which rolls out on NumPy, with the same seed.

| Option | Same starts | Same parameters | Same noise | Extra memory | Can it fail? |
| --- | --- | --- | --- | --- | --- |
| Each backend draws its own trials (today) | no | no | no | none | no; it warns, and `compare` forces NumPy |
| The backend fixed at creation | yes | yes | yes | none | yes: a DP law on an evaluator resolved to JAX raises; two evaluators still differ |
| The test set without the noise values | yes | yes | no | a few numbers per trial | no |
| The test set with the noise values (D11 as first ruled) | yes | yes | on a port only: an array cannot feed a block inside `sys` | trials × steps × channels | no |
| **One generator on both backends, the test set without the noise values (D13)** | yes | yes | yes, to 4e-16, below the 3e-16 the backends already differ by | a few numbers per trial | no; 25 lines pinned by published test vectors |

D11 chose the noise array on 2026-09-26, against a ported generator thought costly to maintain
and only "nearly" identical. Measured 2026-09-30 (§10, finding 4), the port is small and exact,
the last-bit difference exists without any noise, and the array cannot serve a noise block
inside a diagram, which Monte Carlo of an observer-based loop needs. D13 reverses the choice.

**How noise reaches a problem.** Port route only (a system with a random block inside refused),
block route only (`disturbances=` removed, and planners then plan against a frozen noise signal
as a known input), or both on one mechanism (D7).

**Where a problem's disturbance meaning comes from.** A bare law legal with a one-time warning,
legal and silent (today, the F9 ambiguity), or signals only (D6): with one test as the only user,
the strict option costs one line and every disturbance then carries its own meaning in time.

**Splitting streams on NumPy.** Keep one shared stream (adding a random quantity reshuffles every
later draw), switch `split_keys` now (the draws change twice, now and with `realize`), or spawn
inside `realize` (D10: the draws change once).

**Sampling without a key.** Allowed everywhere (a draw nobody can replay), `None` meaning seed 0
(two bare calls return the same point), or a key required (D9: no call site breaks).

**The name of `W`.** `psd` (D8: the word of the frequency chapters, beside `std`, `var`, `cov`;
the two-sided convention stated), `intensity` (the Kalman-Bucy textbook word, no one-sided
reading), `spectral_density` (long, the same ambiguity as `psd`).

**`NoiseSource`'s numbers.** The distributions read `params` first (D12), a Gaussian-only start
(its constructor would change shape later), or the distribution kept as structure (breaks D2).

**Classes or keywords.** Covariance against intensity is a difference of model: two classes
(D3). Zero-order hold against linear is a difference of reconstruction, same model, same
params, same role: a keyword (D4). Four classes for two models times two holds would be twin
classes that drift (RULES 7.1).

## 8. Rulings

All decided 2026-09-26 (maintainer):

- **D1. The seed is a parameter, read by a counter-based draw.** `w(t)` is a function of
  `(t, params)`; no drawn table, no `refresh()`.
- **D2. `sample_period` is a parameter.** Everything `h` reads is in `params`, so the params
  dict reproduces a run.
- **D3. Two classes.** `NoiseSource(distribution, sample_period)` for sampled noise;
  `WhiteNoise(psd, sample_period)` its subclass for continuous white noise.
- **D4. The hold is a constructor keyword.** `hold="zoh"` by default; `hold="linear"` kept as
  the option (today's behaviour), its `Σ / 2` mid-period dip stated in the docstring.
- **D5. No time window.** `t0` and `tf` leave the block's params; `show_signal(t0=, tf=)` keeps
  its own arguments and evaluates `h` on a grid.
- **D6. A disturbance port takes a signal** (was RN-a). `NoiseSource` or `WhiteNoise`, never a
  bare `Distribution`; no `sample_period` means the tool's `dt`; a shorter one is refused (R4).
  Costs one test line.
- **D7. Both noise routes, one mechanism.** The port route is the documented one for problems; a
  block inside `sys` also works; the evaluator realizes every random block per trial (R4b).
- **D8. The magnitude is named `psd`** (was RN-b): keyword, params key and property; the
  docstring states the two-sided density.
- **D9. Every `sample` takes a key** (was RN-e). `key=None` leaves `Set.sample` and
  `InputSet.sample` (no call site passes none); the TODO A5 row asking for `None` on
  `Distribution.sample` is retired the other way.
- **D10. One independent stream per consumer on NumPy** (was RN-f), written once inside `realize`.
- **D11. The evaluator holds a test set** (was RN-g): `ev.trials` drawn once on NumPy, consumed
  by every backend. First ruled with the noise values in it; amended by D13, it holds the
  starts, the parameter values and the seeds.
- **D12. `NoiseSource` waits for the distributions to read `params`** (was RN-h): R1 lands first,
  `NoiseSource` nests the law's numbers under `params["law"]`; `WhiteNoise` lands before both.

Decided 2026-09-30 (maintainer), from the review of §10:

- **D13. One counter generator on both backends** (was A1). The noise block draws with
  Threefry-2x32 in the library's own layout (key = seed, counter = sample index and channel),
  the same bits on NumPy and JAX (§4). It replaces "Philox on NumPy; threefry on JAX" and amends
  D11: the test set holds no noise values. It lands with RN-1, so seeded realizations change
  once.

## 9. Steps

Order: RN-1 first (it unblocks P4); RN-2 before RN-3; RN-4 before RN-5; RN-6 closes each step's
docs as it lands. Rungs as in the header: RN-1 and RN-2 in v0.2, RN-3 with its first consumer,
RN-4 and RN-5 in v0.3. Each step that changes draws records a seeded baseline before and pins the new
numbers with a test after (RULES 7.7, AGENTS refactor recipe).

- [ ] **RN-1 `WhiteNoise`** (D1–D5, D8, D13): `sample_index` and `standard_normal` helpers under
  `# Internal machinery` with the trap tests; the Threefry cipher with its two pin tests (the
  Random123 vectors, equality with the JAX primitive) and a NumPy against JAX test of the block's
  signal; `hold`; the shared-seed and 32-bit warnings;
  `refresh()`, `var`, `t0`, `tf` removed (a release note). Demos `blocks_sources.py`,
  `diagram_noise_ports.py`, `diagram_shortcuts.py` updated (distinct seeds); the guard test of
  TODO A5 (editing params changes the next simulation). Unblocks P4.
  Also in its blast radius (found 2026-09-30): `tutorial/00_core.ipynb` (cell 27),
  `tutorial/01_blocks.ipynb` (cell 3), the live course notebook
  `udes_gro501/cartpole_dynamic_controller.ipynb` (cell 11), `tests/unittest/test_blocks.py`
  (four tests on `refresh()`, `t0`, `tf`), and the block's own `__main__`. The `mean` param's
  fate and the constructor's first argument are unruled (§10, A4). **[ask — student-facing]**
- [ ] **RN-2 Distributions read `params`** (R1, D9, D12): `Gaussian(cov=)`, `cov` on every law,
  `sample(key, n=None, params=None)`, a `params` dict on `Gaussian`, `Uniform`, `Particles`;
  `Set.sample` and `InputSet.sample` take a required key. Baseline: every `sample` call site.
- [ ] **RN-3 `NoiseSource`** (D3, D12): the law's numbers under `params["law"]`; `WhiteNoise`
  becomes its subclass.
- [ ] **RN-4 `realize` and the problem** (R4, R4b, D6, D7, D10): `System.realize(key)` gathered
  over subsystems, one spawned stream per consumer (`numpy>=1.25`); `StochasticPlanningProblem.realize(key)`;
  `disturbances` takes signals only, the one test line updated; the RL environment reads the
  block's `h` at its `dt`; the simulator warns once when an adaptive solver meets a noise
  source, or a fixed step exceeds Δ; RK4's last stage at the left limit of a jump. A seeded
  baseline records the one change of draws.
- [ ] **RN-5 The evaluator's test set** (D11, D13): `ev.trials` (starts, params with their seeds)
  drawn once on NumPy; every backend rolls out the same realizations, the noise computed by the
  block; the fallback warning and `compare`'s NumPy forcing removed with their tests; every
  random block inside `sys` realized per trial. `Sys2Gym` takes a `Distribution` (R5).
- [ ] **RN-6 Docs**: DESIGN §4 (the distributions entry: the convention, the three periods, the
  bridge table) and §6 (the stochastic problem's `disturbances`, the evaluator's test set);
  ROADMAP §6 closes F9; TODO A5 rows retired; `kalman()` (P4) reads intensities, the discrete
  filter later reads `Q_d`, `R_d`.

Out of scope on purpose: time-varying laws `p(t)` (a gust is `Step` times a noise source),
a `ColoredNoise` class (a filter), a stochastic system wrapper (randomness enters only through
named ports, RULES 4.9), a Brownian-bridge refinement that keeps one path across changes of Δ.

## 10. Review 2026-09-30: findings and proposed amendments

A second opinion on D1–D12, measured on the current code and on an in-memory prototype of the
§4 block (NumPy 2.5, JAX 0.10.2; nothing landed). The convention holds: `Σ = W / Δ`, the two
classes, `psd` as the dt-free number, the counter-based pure block. The findings are around it.
A1 was ruled on 2026-09-30 (D13). A2–A10 are proposals; their remedies were not stress-tested
by a second reviewer.

### Findings

1. **"Its physics does not change with Δ" holds under three conditions the doc does not state.**
   - *The port enters `f` affinely, with a gain that does not depend on the state.* For
     `v̇ = F − c (v − w)|v − w|` with `WhiteNoise(psd=1)` on the wind port, the mean speed goes
     0.80, 0.44, 0.24, 0.066 at Δ = 0.1, 0.03, 0.01, 0.001 (2.0 without wind), because
     `E[w²] = W / Δ`. The same psd through a 0.5 s filter gives 1.79 at every Δ. With a gain
     that depends on the state (`ẋ = (−a + w) x`), the held sample integrated finely is the
     Stratonovich solution and Euler at dt = Δ the Itô one (`E[x(1)]` = 0.472 against 0.366).
   - *The consumer integrates the noise.* A sampler that point-reads `y = x + v` sees variance
     `V / Δ_v`, whatever its own period: 0.0020, 0.0102, 0.1011 at Δ_v = 0.05, 0.01, 0.001 for
     `Tₛ = 0.05`. A discrete Kalman filter tuned with `R_d = V / Tₛ` at `Tₛ / Δ_v = 10` has an
     actual error variance 7.1 times its prediction. `HybridSimulator` point-samples plant
     outputs. A cost on `u` under `u = −k (x + v)` scales as `k² V / dt`.
   - *Δ is small against the loop and the observer, not only the plant.* With an observer pole
     near 10 rad/s, Δ = 0.1 s gives a discrete filter covariance 1.53 times the continuous one.
2. **The port route and the block route are two numerics** (R4b and D7 say one mechanism). A
   port disturbance is held over the four RK4 stages; a block inside `sys` is read at each stage
   time, indices `(k, k, k, k+1)`. After 50 pendulum steps the two differ by 8.7e-4; on
   `ẋ = −10x + w` the block route is 7.5% off the exact held-train path. Moving the last stage's
   time does not give the left limit under any tolerance, and the stage is written at 22 sites.
3. **RN-1 removes an accidental guard.** Today `linearize`, `transfer_function` and
   `lqr_at_operating_point` raise on a diagram holding `WhiteNoise` (`h` refuses `params`). With
   the pure block they run at `t = 0` and read the draw `w(0)`, of standard deviation `√(W / Δ)`:
   on the drag plant `A` went from −0.40 to −1.6 (Δ = 0.01) and −6.1 (Δ = 0.001), equilibria
   moved with the seed, and direct collocation planned against the frozen realization with
   `success=True`. Separately, planners treat a declared disturbance port as an actuator today:
   `lqr_at_operating_point(PendulumWithNoisePort())` returns a 3 × 2 gain, and `wrt="u"` names
   every input stacked, so the control port is `("u", 0)`.
4. **D11's noise array reaches ports only.** The evaluator steps `plant_step(x, u, t, params)`;
   a block inside `sys` is a compiled port operation with no override. With Philox on NumPy and
   threefry on JAX the same trial seed gave J = 4.664 and 4.579. A Threefry-2x32 cipher written
   once (25 lines of Python integers on NumPy, the JAX primitive on JAX; key = seed, counter =
   `(k, channel)`) is bit-identical on both, passes the Random123 vectors, costs 4.35 µs a call
   (Philox 5.45 µs) and the same end to end. NumPy and JAX rollouts of one *deterministic* trial
   already differ by 3e-16 from the second step, so identical noise values buy nothing over
   noise equal to 4e-16.
5. **Measurement noise is inert in the stepped tools.** The evaluator and the RL environment
   feed the true state to a static law and never call `h`: J with `v ~ N(0, 5²)` equals J with
   no noise to 16 digits, on both backends. A controller with state is refused outside
   `backend="simulator"`, which ignores the draws and fails on a diagram plant. Monte Carlo of an
   observer-based loop and RL with observation noise are therefore on no step.
6. **The default run degrades between RN-1 and RN-4.** With the zero-order hold, a plain
   `compute_trajectory()` runs RK45 at `rtol = 1e-4`: 12% rms path error (the variance stays
   within 0.4%) and 4 to 7 times slower than today's block. RK4 at dt = Δ is ten times faster.
   The tight adaptive modes (`scipy_lsoda`, `scipy_max`) are accurate and slow, so a warning
   against every adaptive solver would be wrong.
7. **`cholesky(W / Δ)` fails when the noise is off.** `psd = 0`, or one quiet channel, gives
   `LinAlgError` on NumPy and NaN on every channel on JAX; `diagram_noise_ports.py` uses
   `var = 0` as "off". `params` is a plain dict, so a stale `var` key is kept and ignored.
8. **RL episodes restart at `t = 0`**, so `k` restarts too: with a fixed block seed the second
   episode replayed the first one's noise exactly. `ProblemEnv` / `Sys2Gym` applies no parameter
   or disturbance draw, and its action space stacks the `w` and `v` ports.
9. **The colored-noise recipe starts quiet and cannot ride a port.** The filter state starts at
   zero: the ensemble standard deviation is 43% of its stationary value at 0.1 τ and 92% at τ,
   and a 10 s Monte Carlo cost is 11% low at τ = 2 s. A filter has state, and the stepped tools
   integrate `problem.sys` only. `LowPassFilter(cutoff_hz, order)` has no `tau` and no params.
10. **RN-1 is not a technical prerequisite of P4.** `kalman(A, B, C, Q, R)` takes arrays. On
    today's block, reading `W = var · Δ`, a Kalman–Bucy filter in the diagram had error
    variances `[9.69e-5, 4.35e-4]` against `[9.23e-5, 4.52e-4]` predicted. The live GRO501
    notebook designs with `Qn = I`, `Rn = 1e-3 I` and simulates `var` 0.01 and 0.001: F9 is in
    course material, and that notebook is the first consumer of `psd` with `kalman()`.

### Proposed amendments

- **A1. One counter generator on both backends** (ruled 2026-09-30: D13, written into §4, §5,
  §7 and RN-1, RN-5). It replaced "Philox on NumPy; threefry on JAX" and the noise values of
  D11: Threefry-2x32 with the library's own layout, pinned by the Random123 vectors without JAX
  and against the JAX primitive with it; one named normal transform. `ev.trials` holds starts,
  parameter values and seeds, drawn once on NumPy; a realization is exactly the `(x0, params)`
  of §5, on a port or in a block, and any trial replays in the `Simulator`.
- **A2. State the three conditions of finding 1 in §2**, and their consequences: the bridge
  table carries two periods (`Q_d` with the filter's step, `R_d = D_v V D_vᵀ / Δ_v` with the
  measurement noise's own period, equal to the sampler's); the noise of a sampled sensor is
  `NoiseSource(Gaussian(0, cov=R_d), sample_period=Tₛ)`, so a datasheet's per-sample value is
  typed as it reads; wind and anything entering through drag is white noise through a filter;
  a one-sided density `n` per √Hz is `W = n² / 2`. Every noise signal answers both questions,
  its intensity (`W`, or `Σ Δ`) and its per-sample covariance at a period (`W / Δ`, or `Σ`), so
  a continuous and a discrete design read one object.
- **A3. Random blocks enter analysis at their mean.** `linearize`, `find_equilibrium`,
  `transfer_function`, the LQR shortcuts and the deterministic transcriptions evaluate a
  zero-noise params transform, gathered over subsystems like `realize`. One form of it:
  `sys.realize(None)`, no key meaning no draw, as `RolloutEnvironment.step(key=None)` already
  means nominal; a reserved seed value makes each block return its mean and leaves `psd`
  readable by a design. The smaller option: a marker on random sources and the verbs refuse.
  With RN-1, which removes today's guard.
- **A4. RN-1 is safe on its own.** The block publishes Δ and the simulator, when no solver is
  named, takes a fixed step that divides it (from RN-4). Writing `var`, `mean`, `t0` or `tf`
  raises, with `psd = var × sample_period`. The notebooks and tests listed in §9 join the step.
  `psd = 0` and a semidefinite `psd` are legal (element-wise root for a diagonal). To rule:
  whether the first positional argument stays `p`, the default `psd` and Δ, the fate of `mean`
  (dropped: a bias is a `Step` summed in). Fixed-step rollouts compute `t_k = t0 + k·dt`;
  64-bit is required on JAX.
- **A5. No silent noise in the stepped tools.** A disturbance on a port that reaches only `h`
  is refused with a message, and the evaluator warns when `problem.sys` holds a random block
  (frozen across trials until RN-4): one line each, with RN-1.
- **A6. Monte Carlo of an observer-based loop: the `simulator` backend realizes its trials.**
  No new rollout is needed. Checked 2026-09-30 on an LQG compensator (a `DynamicController`
  with ports `r`, `y` → `u`) and `PendulumWithNoisePort`: `compensator @ plant` builds the
  four-state loop; `backend="simulator"` runs it, projects it on the plant with `trajectory_of`
  and scores it with the problem's cost, but returns J = 0 in every trial because it applies no
  draw; the NumPy and JAX backends refuse a controller with state. The same loop simulated by
  hand, a noise block wired on `w` and on `v` with a fresh seed per trial (RK4 at dt = Δ),
  gave a mean cost of 1.013 ± 0.023 times the covariance prediction over 240 trials. So RN-4
  and RN-5 gain three lines of scope: the simulator backend wires each `disturbances` signal
  onto its plant port inside the closed loop (R4b, literally), applies `realize(key)` and the
  parameter draws per trial, and steps at the noise period; `backend="auto"` sends a controller
  with state there. It needs D13 (a trial is `(x0, params)`; a noise-value array cannot feed a
  diagram) and the compensator block of P4's composition ruling. A JAX batch of the same loop
  is `rollout_batch` over a family of seeds, later. RL with observation noise stays out of
  scope until a learner reads `y`. Planners pin a declared disturbance port at its nominal
  value.
- **A7. One key in the RL environment.** The episode's realization (start, parameter values,
  block seeds) is drawn at reset and carried; "RL training stays key-threaded" (§5) and "the
  environment reads the block's `h`" (RN-4) become that one sentence. `realize` derives each
  stream from a stable name (the subsystem id path, the parameter name), not from subsystem
  order, which would bring back the coupling D10 removes. `ProblemEnv` realizes at reset or
  refuses a problem with disturbances.
- **A8. The hold across integrator stages.** Drop "the left limit at RK4's last stage" from
  RN-4. State the effect (first order in dt, statistically small) and the rule dt = Δ / n.
  Evaluating a held source once per step, as a port disturbance already is, would make the two
  routes one mechanism and leave deterministic baselines untouched, but it is a change to
  `compile`: an open question for RN-4, the maintainer's.
- **A9. What an H2 or H∞ tool reads.** `psd` is an LQG and H2 number: with
  `E[w(t) w(τ)ᵀ] = W δ(t − τ)`, the output variance under `W = I` is `‖G‖₂²` (checked:
  1.989062 by the Lyapunov equation, 1.989063 by the frequency integral). An H∞ tool reads no
  distribution: it reads the disturbance ports (`B_w = ∂f/∂w`), the performance outputs and
  weighting filters, and a weighting filter is the shaping filter of §2. The object every tool
  shares is a named disturbance port, with an optional filter inside `sys`; P5's loop inputs
  `w` and `v` are those ports. Nothing in D1–D12 stands in its way.
- **A10. The deterministic twin as the acceptance test.** RN-1: the simulated variance at the
  sample instants against `A P + P Aᵀ + B_w W B_wᵀ = 0`. P4: the filter Riccati `P` against the
  empirical error covariance. Colored noise: `W = 2 τ σ²` for a gust of standard deviation `σ`
  and correlation time `τ`, the filter inside `sys` with its white input as the disturbance
  port, its state started from `N(0, W / 2τ)`.

### What each tool reads

| Tool | Reads from the noise model |
| --- | --- |
| Simulation | the block's `h(t; params)` |
| Monte Carlo, RL | a realization `(x0, params)` per trial or episode; the per-sample law at the tool's dt |
| Kalman–Bucy, LQG | `B_w`, `D_v`, the intensities `W`, `V` |
| Discrete Kalman, EKF, particle filter | the per-sample covariance or law at the filter's period |
| H2 | the ports and `W` |
| H∞ | the ports and weighting filters; no distribution |
| Identification | the per-sample output covariance at the log rate |
| Stochastic DP (not planned) | the per-step law as points and weights at the grid's dt |
