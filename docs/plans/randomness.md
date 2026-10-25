# One convention for randomness

Status: decided (§8: RD-D1–RD-D12 on 2026-09-26, RD-D13–RD-D24 on 2026-09-30; the evaluator
rulings superseded in part by R10, R15, R19, R27 and R44, as §8's note says). Landed: RN-1 on
2026-09-30 (`WhiteNoise` on the counter-based cipher, `System.realize`, the solver hint; DESIGN
§4, "Randomness") and RN-5's v0.3 part on 2026-10-24 (`evaluate(problem, arg, *, dt, n_trials,
key)` over the start and parameter draws of `problem.realize(key, n)`, `compare` and
`pe.program` on the same draws, `MonteCarloEvaluator` removed; DESIGN §6, "The problem
evaluator"). This doc holds what is open, all in v0.4: RN-2 (the distributions read `params`,
before V1's step 7), RN-3 (`NoiseSource`, with its first consumer), RN-4 (per-block seeds in
`problem.realize`, signals on `disturbances`), the rest of RN-5 (the disturbance signals wired on
their ports in `evaluate`, today held at nominal with a warning; `Sys2Gym`'s start law; the LQG
gate) and RN-6 (the docs, with each step). RN-4 and RN-5 change public behaviour and the Monte
Carlo numbers once, before the v0.9 freeze. The description of the code before RN-1, RN-1's
build plan and its measurements were cut once they landed (the history is in git).
Reviewed 2026-09-30: factual corrections applied in place; the ten amendments of §10 are ruled
(RD-A1 as RD-D13, RD-A2–RD-A10 as RD-D14–RD-D23) and the maintainer added RD-D24 (the evaluator
simulates the closed-loop diagram). §10 stays as the record of the findings.
Ids: `RD-` marks this doc's own, the rules RD-R1–RD-R6 (§3), the rulings RD-D1–RD-D24 (§8) and
the review's amendments RD-A1–RD-A10 (§10); `FD-F9` is finding F9 of the foundations review;
a bare `A5`, `P4`, `RN-2` is a workboard step.
Rungs (decided 2026-09-26, reorganized 2026-10-21 and 2026-10-23): RN-1 (`WhiteNoise`) in v0.2
wave B, the prerequisite of B3 (`estimation/`, P4), landed; RN-5's fold into `evaluate` in v0.3
(R44: V1's step 4, the draws over the start and parameter laws), landed; RN-2 in v0.4, before
V1's step 7 (TODO §1); RN-3 with its first consumer, in the v0.4 row (unless a teaching notebook
needs sampled sensor noise first); RN-4's per-block noise seeds and the rest of RN-5 in v0.4,
joining under the same verb, before the v0.9 freeze; RN-6 with each step. The course runs on
`main` while v0.3 stays on `dev` (R47).
**Aligned 2026-10-23** to the continuous core (DESIGN §5) and the rulings R10, R15,
R27 of [2026-10-22-rulings-session-1.md](../reviews/2026-10-22-rulings-session-1.md), and to
its session 3 (R36 `dt` required, R44 the fold in v0.3, R45 the record's names): the evaluator
is `evaluate(problem, arg, *, dt, n_trials, key)` on the compiled problem, the draws
`problem.realize(key, n)`, the record `Evaluation`; §8's note says which rulings here that
supersedes. It settles the disturbance convention ROADMAP §6
held open before B3, closes finding FD-F9 of
[2026-09-15-foundations-review.md](../reviews/2026-09-15-foundations-review.md), and absorbs the
`Gaussian(cov=)`, `NoiseSource`, `Distribution.sample(key=None)` and `WhiteNoise` guard-test
rows of TODO A5.
Consumers: the `WhiteNoise` block, a colored-noise recipe, random starts, LQG, RL training,
Monte Carlo evaluation. One question underneath all six: **what a number on a noise means in
time**, and **where a random draw happens** in a library whose equation paths are pure.

**Port names (2026-10-11).** The loop's disturbance port is `d` and its measurement-noise
port `n` (DESIGN §4, "Control laws"; renamed in P10's step AC-2). The examples below use
them; where §10 says port `w` or `v`, it describes the code and the review of that time. The
math symbols (`w_k`, `W`, `V`, `B_w`) are unchanged. A plant that declares `d` and `n` owns
them, and `closed_loop(disturbance=, noise=)` accepts a noise block (P10's AC-4, landed
2026-10-14), which is how the loop inputs meet RD-A9 and RD-D24.

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
- **A problem declares noise on ports**, `disturbances={"d": WhiteNoise(psd=W)}`; a noise block
  already inside the system also works. Planners see the nominal port; the RL environment and
  the evaluator draw on it (the evaluator once RN-5 is complete: today it holds the port at
  nominal and warns).
- **Monte Carlo is the closed-loop diagram simulated on draws realized once from one key.**
  `evaluate(problem, ctl, dt=, n_trials=, key=)` builds the loop with the disturbance blocks
  wired on their ports, realizes the draws once (`problem.realize(key, n)`: starts, parameter
  values, seeds), and runs the compiled fixed-step rollout on every draw, batched (R10, R15).
  Any controller, any noise block, any parameter draw goes through the one path. Built in v0.3
  for the starts and the parameter values; the signals on their ports and the per-block seeds
  are RN-4 and the rest of RN-5.
- **A noisy diagram picks its own solver.** The noise block publishes its period as the
  solver hint, so a plain `compute_trajectory()` runs fixed-step RK4 at `dt = Δ`.
- **LQG reads the same numbers the simulation draws from**: `Q = B_w W B_wᵀ`, `R = V` for the
  continuous filter; `Q_d = B_w W B_wᵀ Δ`, `R_d = V / Δ` for the discrete one.

## 1. What the code still lacks

RN-1 and RN-5's v0.3 part replaced the code this section described in September (the
drawn-table `WhiteNoise`, `MonteCarloEvaluator` and its three held-input rollouts). What the open
steps still change:

- `StochasticPlanningProblem.disturbances` is `{port: Distribution}` (`planning/problems.py`).
  The RL environment and `Sys2Gym` draw a fresh value per step and hold it on the port, at their
  own `dt`; `evaluate` holds the declared ports at their nominal value and warns
  (`warn_if_disturbed`, `planning/evaluator.py`). RN-4 makes them signals; the rest of RN-5 wires
  them on their ports.
- `problem.realize(key, n)` draws the starts, the parameter values and one held value per
  disturbance port, with no per-block seeds (`evaluate` realizes random blocks inside the
  evaluated loop per trial via `trial_seeds`, R92, while `RolloutEnvironment` still keeps a
  random block's seed across episodes until RN-4). `System.realize(key)` and
  `child_seed` exist since RN-1.
- `Gaussian(mean, std)` has no `cov`, and `Distribution.sample(key, n=None)` takes no `params`
  (RN-2).
- `Set.sample` still accepts `key=None` (`core/sets.py`; RN-2, RD-D9).
- There is no `NoiseSource` (RN-3).
- `Sys2Gym` keeps a start dialect of its own, `x0_lb / x0_ub / x0_std`
  (`interfaces/gymnasium.py`; RD-R5, the rest of RN-5).

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
covariance only when the noise block's period is the filter's (§10, RD-A2). Times Δ on the state,
over Δ on the measurement: the
asymmetry students get wrong, and why the rule lives in one place (DESIGN §4 and one equation
comment beside the filter Riccati solve; never a docstring).

**Colored noise is white noise through a filter.** A first-order shaping filter
`τ ż = −z + w`, `w` white of intensity `W`, gives `z` with variance `W / (2τ)` and correlation
time `τ`. The textbook handles it by augmenting the state with `z`; so does a diagram.

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

  That is the final shape; `WhiteNoise` landed first on its own (RN-1, 2026-09-30), and becomes
  the subclass when `NoiseSource` lands (RN-3).

  Colored noise is not a class: `WhiteNoise(...) >> LowPassFilter(...)` is a diagram like any
  other, and inside a plant it is the augmented state of §2.

- **RD-R3. The dt-free number of a white noise is its spectral density.** `psd` is the only
  magnitude `WhiteNoise` takes. A per-sample covariance is a `NoiseSource(Gaussian(0, cov=Σ), Δ)`:
  it keeps each sample when Δ is edited, and so changes the physics, which is what sampled
  noise means.

- **RD-R4. On a problem, a disturbance port takes a signal, never a bare law.**
  `disturbances={"d": WhiteNoise(psd=W)}` or `{"d": NoiseSource(Gaussian(0, cov=Σ))}`: the
  object says what its number means in time. With no `sample_period`, the tool's `dt` is Δ,
  and a `WhiteNoise`'s physics does not change between a planner at `0.05 s` and an evaluator at
  `0.01 s`. A tool refuses a `sample_period` shorter than its `dt`: it holds the input over its
  whole step, so it would read one sample in several and inflate the intensity by `dt / Δ`.
  A bare `Distribution` on a port is refused with a message naming the two signals.

- **RD-R4b. Noise reaches a problem by a port or inside `sys`, one mechanism underneath.** The
  port route (`disturbances=`) is shorthand for wiring that block onto the port in every trial,
  and it is the documented route for problems: planners plan on `sys` with the port at its
  nominal value, the evaluator and the RL environment draw on it, and a Kalman design reads the
  noise model from the problem. A noise block already wired inside `sys` also works (the natural
  way to build a simulation diagram). Either way the evaluator realizes every random block with
  fresh seeds in each trial (§5); a block's own seed is only its default realization, used by a
  plain `compute_trajectory()` and never by an evaluator. ("Nominal" is kept for the mean value
  of a port.)

- **RD-R5. A random start is a `Distribution` over `x`, owned by the problem; `sys.x0` is its
  mean.** Already so for `StochasticPlanningProblem`. `Sys2Gym` drops `x0_lb / x0_ub /
  x0_std` and takes a `Distribution`, `from_problem` being the route (TODO row exists).

- **RD-R6. Seeds enter at a tool's boundary as an integer, or live in a random block's params.**
  No global RNG anywhere (Constitution §4.4; JAX has none). Every `sample` takes a key: there
  is no unseeded draw in the library. A tool may take `seed=None` for fresh entropy and then
  reports the seed it drew, so the run can be replayed. Inside a tool one key is derived into one
  independent stream per consumer, each from a stable name (a subsystem's id path, a parameter's
  name) with the library's own cipher, `child = F(key, crc32(name))`, the same on both backends
  (RD-D21): adding a random block never reshuffles the others. A `Distribution.sample` gives
  different numbers on NumPy and JAX for the same seed, with the same law; the evaluator realizes
  its starts and parameter values once per call, in `problem.realize(key, n)`, and RN-4 makes
  that realization the same on both backends (§5, §8's note). A noise block
  gives the same signal on both (RD-D13). A `seed` of `None` is the mean: no draw (RD-D17).

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
`WhiteNoise` needed none of this and landed first.

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

**One generator, both backends (RD-D13).** `F` is Threefry-2x32, the cipher JAX's own generator is
built on, with the library's layout: key = seed, counter = `(k mod 2³², channel)`. On NumPy it
is about 25 lines of Python integers under `# Internal machinery`; on JAX the same function
(`jax.extend.random.threefry_2x32`, or the same code on `jax.numpy`). The bits are identical on
both, so the same seed is the same signal whichever backend compiles the diagram. Two tests
pin it: the published Random123 vectors, which need no JAX, and equality with the JAX primitive
when JAX is installed. One named library transform turns the bits into normals. The layout is
the library's own, not `jax.random.fold_in` and `bits`, whose output changes with the
`jax_threefry_partitionable` flag. `NoiseSource` draws on the same cipher.

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

**The solver (landed with RN-1: RD-D16, RD-D18).** A noisy diagram runs fixed-step RK4 on a
divisor of its period by itself, one step per sample by default; a user-named solver or `dt`
wins, and a fixed `dt` that does not divide Δ warns. A fixed step coarser than Δ holds one sample
over its whole step and so inflates the intensity by `dt / Δ`; RK4's last stage reads the next
sample at a boundary, an effect first order in `dt` and accepted (RD-D18). The traps of the
sample index (floating-point time at a boundary, the 32-bit counter, negative time, 32-bit JAX,
the integer seed in `params`, shared seeds) each have a test. DESIGN §4 states the rule; the
measurements RD-D16 and RD-D23 cite are in this doc's git history.

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
key is realized once per call: `evaluate(..., key=1)` hands it to `problem.realize(key, n)`
through `ProblemEvaluator.realize` (`planning/evaluator.py`).

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
changes the numbers a given seed produces once, in the `realize` step; the streams are derived
by name with the library's cipher (RD-D21), so no `numpy` floor is needed.

**One realization, shared by every law.** A trial needs a start, parameter values and noise
signals. The draws are `problem.realize(key, n)`, one key realized once per `evaluate` or
`program` call and shared by every entry `compare` scores (common random numbers; §8's note);
the noise comes from the blocks, whose numbers do not depend on the backend (RD-D13). A trial is
a realization `(x0, params)` and nothing else: every law is rolled out on it, whatever backend
runs the rollout, on a port or inside `sys`.

```python
cmp = compare(problem, LQR=lqr, VI=vi, dt=0.01, n_trials=100, key=1)   # one realization, shared: common random numbers
cmp.evaluations["LQR"].objective, cmp.difference("LQR", "VI")
```

Once RN-4 realizes the starts and parameter values with the library's cipher, the same key gives
the same trials on any backend, and the same `J` to rounding (the two backends' arithmetic
already differs by 3e-16 from the second step of a noise-free rollout, so a test compares with a
tolerance); today a start or parameter law is drawn by `Distribution.sample` on the evaluator's
backend. Any trial replays in the continuous `Simulator`. Cost: a few numbers per trial (80 kB of
seeds for 10,000 trials, against 320 MB for the noise values of 10,000 × 1,000 × 4). The options
weighed are in §7. RL training stays key-threaded in JAX, where cross-backend identity buys
nothing.

**The evaluator simulates the closed-loop diagram (RD-D24, as amended in §8's note).** A trial
is a realization, and a realization is simulated by the one path every diagram uses. `evaluate`
builds the loop as a diagram (`closed_loop_of(problem, arg)`: `ctl @ sys` for a feedback block,
`source >> sys` for an open-loop law) and runs the compiled fixed-step rollout of
`problem.compile(ctl, dt)` on every draw, `vmap`ped on JAX and looped on NumPy (R15; an adaptive
solver label is the escape, one run per trial, R42). The cost is scored on the plant's own run
(`traj.of("plant")`). There is no static-law assumption left: a controller with state and the
observer-based compensator of P4 go through it; a sampled law (a `Computer @ plant` loop) is
refused until V1's step 7b.   Still open (RN-4, the rest of RN-5): each `disturbances` signal wired
  onto its port (today held at nominal, with a warning), and per-block seeds moved into
  `problem.realize` (`evaluate` already realizes random blocks inside the loop per trial, R92). A continuous law acts continuously; a sampled law is a `Computer`. The
held-input rollout of `RolloutEnvironment` stays the learner's stepped view and is not the
evaluator's.

## 6. The six uses, one page

```python
plant = NoisyPendulum()                        # ports u, d [N m], n [rad/s]

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
    disturbances={"d": WhiteNoise(psd=1e-2), "n": WhiteNoise(psd=1e-4)},
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

**Where a block's randomness lives.**

| Option | Pure `h` | Edit without `refresh()` | Traces, vmaps | Memory | Window |
| --- | --- | --- | --- | --- | --- |
| Seed on the constructor, table drawn once | yes | no | no | grows with the span | yes |
| Seed in params, table rebuilt on `refresh()` (the block before RN-1) | until params change | no; passing `params` raises | no | grows with the span | yes, ends at 100 s |
| The drawn table in params | yes | yes | yes | trials × steps × channels | yes |
| **Seed in params, counter-based draw (RD-D1)** | yes | yes | yes | one integer | none |
| No seed; the tool passes a key to `h` | yes | yes | yes | none | none; breaks the one signature |

**Who sets it for an experiment.** The block's own seed (its nominal realization), `realize(key)`
along the diagram, and a tool's integer seed layer rather than compete (§5).

**What the evaluator fixes at creation.** The scenario: one evaluator scores an LQR law, which
rolls out on JAX, and a DP lookup table, which rolls out on NumPy, with the same seed.

| Option | Same starts | Same parameters | Same noise | Extra memory | Can it fail? |
| --- | --- | --- | --- | --- | --- |
| Each backend draws its own trials (the evaluator before RN-1) | no | no | no | none | no; it warns, and `compare` forces NumPy |
| The backend fixed at creation | yes | yes | yes | none | yes: a DP law on an evaluator resolved to JAX raises; two evaluators still differ |
| The test set without the noise values | yes | yes | no | a few numbers per trial | no |
| The test set with the noise values (RD-D11 as first ruled) | yes | yes | on a port only: an array cannot feed a block inside `sys` | trials × steps × channels | no |
| **One generator on both backends, the test set without the noise values (RD-D13)** | yes | yes | yes, to 4e-16, below the 3e-16 the backends already differ by | a few numbers per trial | no; 25 lines pinned by published test vectors |

RD-D11 chose the noise array on 2026-09-26, against a ported generator thought costly to maintain
and only "nearly" identical. Measured 2026-09-30 (§10, finding 4), the port is small and exact,
the last-bit difference exists without any noise, and the array cannot serve a noise block
inside a diagram, which Monte Carlo of an observer-based loop needs. RD-D13 reverses the choice.

**How noise reaches a problem.** Port route only (a system with a random block inside refused),
block route only (`disturbances=` removed, and planners then plan against a frozen noise signal
as a known input), or both on one mechanism (RD-D7).

**Where a problem's disturbance meaning comes from.** A bare law legal with a one-time warning,
legal and silent (today, the FD-F9 ambiguity), or signals only (RD-D6): with one test as the only user,
the strict option costs one line and every disturbance then carries its own meaning in time.

**Splitting streams on NumPy.** Keep one shared stream (adding a random quantity reshuffles every
later draw), switch `split_keys` now (the draws change twice, now and with `realize`), or spawn
inside `realize` (RD-D10: the draws change once).

**Sampling without a key.** Allowed everywhere (a draw nobody can replay), `None` meaning seed 0
(two bare calls return the same point), or a key required (RD-D9: no call site breaks).

**The name of `W`.** `psd` (RD-D8: the word of the frequency chapters, beside `std`, `var`, `cov`;
the two-sided convention stated), `intensity` (the Kalman-Bucy textbook word, no one-sided
reading), `spectral_density` (long, the same ambiguity as `psd`).

**`NoiseSource`'s numbers.** The distributions read `params` first (RD-D12), a Gaussian-only start
(its constructor would change shape later), or the distribution kept as structure (breaks RD-D2).

**Classes or keywords.** Covariance against intensity is a difference of model: two classes
(RD-D3). Zero-order hold against linear is a difference of reconstruction, same model, same
params, same role: a keyword (RD-D4). Four classes for two models times two holds would be twin
classes that drift (RULES 7.1).

## 8. Rulings

All decided 2026-09-26 (maintainer):

- **RD-D1. The seed is a parameter, read by a counter-based draw.** `w(t)` is a function of
  `(t, params)`; no drawn table, no `refresh()`.
- **RD-D2. `sample_period` is a parameter.** Everything `h` reads is in `params`, so the params
  dict reproduces a run.
- **RD-D3. Two classes.** `NoiseSource(distribution, sample_period)` for sampled noise;
  `WhiteNoise(psd, sample_period)` its subclass for continuous white noise.
- **RD-D4. The hold is a constructor keyword.** `hold="zoh"` by default; `hold="linear"` kept as
  the option (today's behaviour), its `Σ / 2` mid-period dip stated in the docstring.
- **RD-D5. No time window.** `t0` and `tf` leave the block's params; `show_signal(t0=, tf=)` keeps
  its own arguments and evaluates `h` on a grid.
- **RD-D6. A disturbance port takes a signal** (was RN-a). `NoiseSource` or `WhiteNoise`, never a
  bare `Distribution`; no `sample_period` means the tool's `dt`; a shorter one is refused (RD-R4).
  Costs one test line.
- **RD-D7. Both noise routes, one mechanism.** The port route is the documented one for problems; a
  block inside `sys` also works; the evaluator realizes every random block per trial (RD-R4b).
- **RD-D8. The magnitude is named `psd`** (was RN-b): keyword, params key and property; the
  docstring states the two-sided density.
- **RD-D9. Every `sample` takes a key** (was RN-e). `key=None` leaves `Set.sample` and
  `InputSet.sample` (no call site passes none); the TODO A5 row asking for `None` on
  `Distribution.sample` is retired the other way.
- **RD-D10. One independent stream per consumer on NumPy** (was RN-f), written once inside `realize`.
- **RD-D11. The evaluator holds a test set** (was RN-g): `ev.trials` drawn once on NumPy, consumed
  by every backend. First ruled with the noise values in it; amended by RD-D13, it holds the
  starts, the parameter values and the seeds.
- **RD-D12. `NoiseSource` waits for the distributions to read `params`** (was RN-h): RD-R1 lands first,
  `NoiseSource` nests the law's numbers under `params["law"]`; `WhiteNoise` lands before both.

Decided 2026-09-30 (maintainer), from the review of §10:

- **RD-D13. One counter generator on both backends** (was RD-A1). The noise block draws with
  Threefry-2x32 in the library's own layout (key = seed, counter = sample index and channel),
  the same bits on NumPy and JAX (§4). It replaces "Philox on NumPy; threefry on JAX" and amends
  RD-D11: the test set holds no noise values. It lands with RN-1, so seeded realizations change
  once.

Decided 2026-09-30 (maintainer), from the decision batch on RD-A2–RD-A10 of §10:

- **RD-D14. RN-1 lands now, in v0.2 wave B**, with every cell it touches rewritten in the same
  commit (`tutorial/00_core.ipynb` 27, `tutorial/01_blocks.ipynb` 3,
  `udes_gro501/cartpole_dynamic_controller.ipynb` 11, the three demos, the four tests), so
  nothing a student opens from `main` breaks. Finding 10 stands: P4's array API does not need
  it; its Kalman demo and the GRO501 notebook, the first consumers of `psd`, do.
- **RD-D15. The constructor keeps `p` first; the rest is keyword-only.**
  `WhiteNoise(p=1, *, psd=1.0, sample_period=0.01, seed=0, hold="zoh")`, a scalar `psd` meaning
  `W = psd·I`; params `{seed, sample_period, psd}`. `mean` is dropped (white noise is zero-mean;
  a bias is a `Sum` with a `Source`). A retired key (`var`, `mean`, `t0`, `tf`) is ignored like
  any unknown key, as in every block (decided 2026-09-30: no leaf validates its keys, and the
  block adds no check); the docstring and the release note carry `psd = var × sample_period`. A
  library-wide key check on leaf params is a Later row. `psd = 0` and a semidefinite `psd` are
  legal (element-wise root for a diagonal); a negative seed is refused.
- **RD-D16. A noisy diagram picks fixed-step RK4 on a divisor of its period by itself** (one step
  per sample by default, measured 2026-09-30, §4). The block publishes
  `Δ` as `solver_info["smallest_time_constant"]` and as `solver_info["sample_period"]` (a
  float gathered as a minimum, since a boolean would lose Δ under a faster plant; landed so);
  `select_solver` maps a held signal to `rk4_fixedsteps` when no solver is named, and the
  automatic grid's existing `0.1 × τ_min` policy sets the step (§4): with a held signal the
  automatic step is the largest `Δ / n`, `n` an integer, at or under `0.1 × τ_min`, so a plant
  faster than the noise still steps on a grid that divides Δ. Moved from RN-4 into RN-1 so
  the default run never degrades. A named solver or `dt` wins. The block's hint follows
  `params["sample_period"]`. The flag says what the signal is, not what to do: any block whose
  output is piecewise constant on a known grid may set it.
- **RD-D17. `seed = None` is the mean.** `sys.realize(None)` returns the nominal params, every
  random block's seed `None` (a leaf-free pytree node, so it traces), and `h` returns zero there.
  `linearize`, `find_equilibrium`, `transfer_function`, the LQR shortcuts and the deterministic
  transcriptions apply it themselves: a noisy loop linearizes at `E[w] = 0`, and `psd` stays
  readable by the Kalman design. It mirrors `RolloutEnvironment.step(key=None)`.
- **RD-D18. The hold across RK4 stages is accepted as first order in `dt`.** The effect and the
  rule `dt = Δ / n` are stated in the docstring and DESIGN; a fixed step that does not divide
  Δ warns; "the left limit at RK4's last stage" leaves RN-4. The integrator is the vanilla
  fixed-step RK4 as it exists: the block's `h` is evaluated at every stage like any source's, and
  neither `compile` nor a solver changes (decided 2026-09-30: the randomness API lands first; a
  solver that samples held sources once per step is a Later row, taken up only if a twin test
  shows the bias matters).
- **RD-D19. No silent noise in the stepped tools.** A `disturbances` port that reaches only `h`
  is refused with a message until a tool reads `y`; the evaluator and the RL environment warn
  once when `problem.sys` holds a random block, until RN-4 realizes it per trial. With RN-1.
- **RD-D20. Monte Carlo of an observer-based loop is in scope** for RN-4 and RN-5: the
  disturbance signals wired onto the plant ports inside the closed loop, `realize(key)` and the
  parameter draws applied per trial, the loop stepped at the noise grid, a controller with
  state accepted. RD-D24 makes this the general path rather than a route of the `simulator`
  backend. Needs P4's compensator ruling for its demo. A JAX batch of the same loop is RD-D24's
  `rollout_batch`.
- **RD-D21. One key per episode; streams derived by name.** The episode's realization (start,
  parameter values, block seeds) is drawn at `reset` and carried, so two episodes never replay
  one noise (finding 8). `realize` derives each stream from a stable name with the library's
  own cipher, `child = F(key, crc32(name))`, identical on both backends; `Generator.spawn`
  and the `numpy>=1.25` floor go. RD-D10 keeps its intent (one independent stream per consumer)
  with this derivation. `ProblemEnv` realizes at reset; its action space is the action port
  alone.
- **RD-D22. A problem pins its declared disturbance ports at their nominal value** for every
  planner. The bare shortcuts on a multi-input system stay as they are: a two-input plant with
  no problem is an ordinary MIMO plant, and the user passes `wrt=("u", 0)`.
- **RD-D23. The review's text amendments are accepted as written**: RD-A2 (the three conditions
  under which the physics is Δ-free, the bridge table with two periods, a datasheet sensor as
  `NoiseSource(Gaussian(0, cov=R_d), sample_period=Tₛ)`, `W = n² / 2` for a one-sided density)
  into §2; RD-A9 (H2 reads `psd`, H∞ reads ports and filters) into §2; RD-A10 (the deterministic
  twin as the acceptance test of RN-1, P4 and colored noise, the shaping filter's state started
  from `N(0, W / 2τ)`) into §9. `LowPassFilter(tau=)` is a Later row, not an RN step. The
  fixed-step loops keep accumulating `t + dt`; the block's `sample_index` absorbs the drift with
  a relative tolerance (§4, held over 2·10⁶ steps), and 64-bit is required on JAX (RD-A4).
- **RD-D24. The evaluator is a batched simulation of the closed-loop diagram.** `MonteCarloEvaluator`
  builds `ctl @ sys` (or `source >> sys`) with the disturbance blocks on their ports, draws
  `ev.trials` once, and simulates that diagram over the trials: `rollout_batch` on JAX, a loop
  of `Simulator` runs on NumPy, RK4 at the automatic grid of RD-D16 (§5). It replaces the three
  hand-written held-input rollouts (`evaluate_jax`, `evaluate_numpy`, `evaluate_simulator`)
  with one path; `backend` chooses the arrays only. Any controller, any noise block, any
  parameter draw is a regular diagram simulation. `dt` becomes the simulation step rather than
  a control period, so the evaluator's numbers change once, in v0.4 with RN-5.

**Superseded in part by R10, R15, R19, R27 (2026-10-22/23;
[the rulings](../reviews/2026-10-22-rulings-session-1.md)).** The rulings above stand as the
history of this plan; where they conflict, the later rulings hold:

- **RD-D11's test set** is no longer `ev.trials` drawn on NumPy and held by an evaluator object:
  the draws are `problem.realize(key, n)`, a function of the problem and one key, realized once
  per `evaluate` or `program` call and shared by every entry `compare` scores (common random
  numbers). Cross-backend identity still holds for the noise blocks (counter-based, RD-D13) and
  for the realization itself (RN-4's done-when); a start or parameter law drawn by
  `Distribution.sample` is realized on one backend inside `realize`.
- **RD-D24's run** is the compiled fixed-step rollout of `problem.compile(ctl, dt)` on both
  backends (R15: the judge and the engine), not a loop of `Simulator` runs at the automatic
  grid; `evaluate(..., solver="scipy")` is the adaptive escape, one run per trial (R42). `dt`
  stays the simulation step and is required (R36).
- **`MonteCarloEvaluator`** folded into `evaluate(problem, arg, *, dt, n_trials, key)` (R10)
  and was removed **in v0.3** (R44, landed 2026-10-24; RD-D24's "in v0.4" no longer holds); its
  record is the one `Evaluation` (`cost` the plain integral, `J` the
  run's score, the two margins, `failed`, `reached`, `objective` the problem's criterion over
  `J`, the three rates `feasibility_rate` / `reachability_rate` / `success_rate`; no
  `failure_rate`; R27 as renamed by R45).
- **The score** is the integral to `tf` with no cut at exit (R19); the price is a switch
  (R32): with `infeasible_cost=None`, the default, `J = cost`; with `True` or a number, `J = cost
  + price · (failed or not reached)`, the price `True` reads from the cost's upper-bound hint
  (`problem.cost_bound()`). A run that is not finite is failed, its cost `+inf` (R31).
  RD-D24's "the numbers change once" now covers both changes (D21).
- **The RL environment** keeps its own episode (R43): a training setting reading the problem's
  fields as hints, no longer the Monte Carlo trials' plant.

## 9. Steps: the implementation plan

Order: RN-1 first (landed); RN-2 before RN-3; RN-5 before RN-4 since R44 (the fold landed in
v0.3 on the start and parameter draws; RN-4's per-block seeds join under the same verb); RN-6
closes each step's docs as it lands. Rungs as in the header: RN-1 landed in v0.2 and RN-5's
first part in v0.3; RN-2, RN-3, RN-4, the rest of RN-5 and RN-6 in v0.4. Each
step that changes draws records a seeded baseline before and pins the new numbers with a test
after (RULES 7.7, AGENTS refactor recipe); each step's math is read next to `dp.py` before it
lands. The acceptance test of every step is its deterministic twin (RD-D23).

- [x] **RN-1 `WhiteNoise`** (RD-D1–RD-D5, RD-D8, RD-D13–RD-D17, RD-D19). Landed 2026-09-30, five
  commits on `dev-random`: the cipher (`threefry_2x32`, `standard_normal`, `sample_index`,
  `child_seed` in `core/distributions.py`), the block in `blocks/sources.py`, the solver hints,
  `System.realize(key)` (leaf, block, diagram; the `None` and the keyed path) and the
  stepped-tools warnings. DESIGN §4 ("Randomness") states the contract; the step's files, notes
  and done-when are in git history.
- [ ] **RN-2 Distributions read `params`** (RD-R1, RD-D9, RD-D12): `Gaussian(cov=)`, `cov` on every law,
  `sample(key, n=None, params=None)`, a `params` dict on `Gaussian`, `Uniform`, `Particles`;
  `Set.sample` and `InputSet.sample` take a required key. Files: `core/distributions.py`,
  `core/sets.py`, every `sample` call site. Baseline: every `sample` call site, byte-identical.
  Done when a family of noise levels vmaps like a family of masses (one test).
- [ ] **RN-3 `NoiseSource`** (RD-D3, RD-D12, RD-D23): the law's numbers under `params["law"]`;
  `WhiteNoise` becomes its subclass; the datasheet-sensor recipe
  `NoiseSource(Gaussian(0, cov=R_d), sample_period=Tₛ)` as the first demo. Done when the twin
  test of a sampled sensor passes (the sample covariance against `R_d`) and `WhiteNoise`'s tests
  pass untouched.
- [ ] **RN-4 `realize` and the problem** (RD-R4, RD-R4b, RD-D6, RD-D7, RD-D10, RD-D18, RD-D20–RD-D22).
  1. `StochasticPlanningProblem.realize(key, n)` (since RN-5's v0.3 part: the start, the
     parameter values and a held value per disturbance port) gains a seed for each
     `disturbances` block and for each random block inside `sys`, under the same verb, each
     block's seed `child_seed(key, id_path)` as `System.realize(key)` already derives it (both
     landed with RN-1, RD-D21).
     Files: `core/system.py`, `core/wiring.py`, `planning/problems.py`, `core/distributions.py`.
  2. `disturbances` takes signals only (RD-D6), the one line of `test_rl_planner.py` updated; a
     planner pins the declared disturbance ports at their nominal value (RD-D22), in
     `planning/problems.py` (`nominal()`) and the transcriptions that read a problem.
  3. `RolloutEnvironment.reset(key)` returns the episode's realization and `step` reads each
     block's `h` at `t_k` with the episode's seeds (RD-D21); `ProblemEnv` realizes at reset, its
     action space the action port alone. Files: `learning/reinforcement_learning/environment.py`,
     `interfaces/gymnasium.py`.
  4. The hold across stages: the RD-D18 rule in DESIGN and the block's docstring. No integrator
     change (RD-D18).
  5. Colored noise: the recipe of §6 with the shaping filter's state started from
     `N(0, W / 2τ)` (RD-D23), as a demo and its twin test (`W = 2 τ σ²`); `LowPassFilter(tau=)` a
     Later row in TODO.
  A seeded baseline records the one change of draws. Done when `problem.realize(key)` is
  bit-identical on NumPy and JAX, two episodes from one training key draw distinct noise, and
  adding a random parameter leaves every other stream unchanged (the table of §5, by name).
- [ ] **RN-5 Monte Carlo is the closed-loop diagram on the draws** (RD-D20, RD-D24 as amended
  in §8's note; R10, R15, R27, R44, R45; V1's step 7 in
  [differentiable-problem.md](differentiable-problem.md), the same row). Its v0.3 part landed
  2026-10-24: `evaluate(problem, arg, *, dt, n_trials, key)` (`dt` required, R36) on the loop
  `closed_loop_of(problem, arg)`, the draws `problem.realize(key, n)` realized once (the start
  and the parameter laws), the compiled fixed-step rollout `vmap`ped on JAX and looped on
  NumPy, the cost scored on the plant's run, the record one `Evaluation`; `compare` and
  `pe.program(wrt, key=, n_trials=)` on the same draws; `Planner.evaluate` calling it;
  `MonteCarloEvaluator`, `evaluate_jax`, `evaluate_numpy`, `evaluate_simulator` and the
  `simulator` backend removed; a controller with state accepted, a sampled controller refused
  until V1's step 7b. What is left, in v0.4:
  1. each `disturbances` signal wired on its port in the evaluated loop (today `evaluate` holds
     the declared ports at nominal and warns, `warn_if_disturbed`), and a noise block inside
     `sys` realized per trial with RN-4's seeds.
  2. `Sys2Gym` takes a `Distribution` for the start (RD-R5); `x0_lb / x0_ub / x0_std` go.
  Done when an LQG loop (P4's observer and gain, noise on `d` and `n`) scores a mean cost within
  the sampling error of the covariance prediction (1.013 ± 0.023 measured by hand, §10 RD-A6);
  the same `evaluate` call on JAX and on NumPy agrees to a tolerance on one key; the
  deterministic twin (`n_trials=1` on the nominal problem) equals the deterministic `evaluate`;
  the release note records the one change of the evaluator's numbers. The workboard row
  (RN-3 / RN-4 / RN-5 with V1's step 7, v0.4) carries the gate.
- [ ] **RN-6 Docs**, with each step: DESIGN §4 (the distributions entry: the convention, the
  three periods, the bridge table with its two periods, the solver hint) and §6 (the stochastic
  problem's `disturbances`, `realize`, `evaluate` on the draws). Done so far: DESIGN §4
  ("Randomness") for RN-1 and §6 ("The problem evaluator") for `evaluate`,
  `problem.realize(key, n)` and the disturbances `evaluate` holds at nominal. Then the open
  question is closed (DESIGN §4 carries the convention); TODO A5 rows retired; `kalman()` (P4)
  reads intensities, the discrete filter later reads `Q_d`, `R_d`; the release notes of RN-1,
  RN-4 and RN-5.

Out of scope on purpose: time-varying laws `p(t)` (a gust is `Step` times a noise source),
a `ColoredNoise` class (a filter), a stochastic system wrapper (randomness enters only through
named ports, RULES 4.9), a Brownian-bridge refinement that keeps one path across changes of Δ,
RL with observation noise until a learner reads `y`.

## 10. Review 2026-09-30: findings and proposed amendments

A second opinion on RD-D1–RD-D12, measured on the current code and on an in-memory prototype of
the §4 block (NumPy 2.5, JAX 0.10.2; nothing landed). The convention holds: `Σ = W / Δ`, the two
classes, `psd` as the dt-free number, the counter-based pure block. The findings are around it.
All ten amendments are ruled (RD-A1 as RD-D13, RD-A2–RD-A10 as RD-D14–RD-D23, §8); the findings
stay as the evidence. Their remedies were not stress-tested by a second reviewer.

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
2. **The port route and the block route are two numerics** (RD-R4b and RD-D7 say one mechanism). A
   port disturbance is held over the four RK4 stages; a block inside `sys` is read at each stage
   time, indices `(k, k, k, k+1)`. After 50 pendulum steps the two differ by 8.7e-4; on
   `ẋ = −10x + w` the block route is 7.5% off the exact held-train path. Moving the last stage's
   time does not give the left limit under any tolerance, and the stage is written at 22 sites.
3. **RN-1 removes an accidental guard.** Today `linearize`, `transfer_function` and
   `lqr(plant, …)` (then `lqr_at_operating_point`) raise on a diagram holding `WhiteNoise` (`h` refuses `params`). With
   the pure block they run at `t = 0` and read the draw `w(0)`, of standard deviation `√(W / Δ)`:
   on the drag plant `A` went from −0.40 to −1.6 (Δ = 0.01) and −6.1 (Δ = 0.001), equilibria
   moved with the seed, and direct collocation planned against the frozen realization with
   `success=True`. Separately, planners treat a declared disturbance port as an actuator today:
   the then `lqr_at_operating_point(NoisyPendulum())` returned a 3 × 2 gain (fixed 2026-10-14: `lqr(plant, …)` linearizes about the command port alone), and `wrt="u"` names
   every input stacked, so the control port is `("u", 0)`.
4. **RD-D11's noise array reaches ports only.** The evaluator steps `plant_step(x, u, t, params)`;
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
10. **RN-1 is not a technical prerequisite of P4.** `kalman_gain(A, C, Q_n, R_n)` takes arrays. On
    today's block, reading `W = var · Δ`, a Kalman–Bucy filter in the diagram had error
    variances `[9.69e-5, 4.35e-4]` against `[9.23e-5, 4.52e-4]` predicted. The live GRO501
    notebook designs with `Qn = I`, `Rn = 1e-3 I` and simulates `var` 0.01 and 0.001: FD-F9 is in
    course material, and that notebook is the first consumer of `psd` with `kalman()`.

### Proposed amendments

- **RD-A1. One counter generator on both backends** (ruled 2026-09-30: RD-D13, written into §4, §5,
  §7 and RN-1, RN-5). It replaced "Philox on NumPy; threefry on JAX" and the noise values of
  RD-D11: Threefry-2x32 with the library's own layout, pinned by the Random123 vectors without JAX
  and against the JAX primitive with it; one named normal transform. `ev.trials` holds starts,
  parameter values and seeds, drawn once on NumPy; a realization is exactly the `(x0, params)`
  of §5, on a port or in a block, and any trial replays in the `Simulator`.
- **RD-A2. State the three conditions of finding 1 in §2**, and their consequences: the bridge
  table carries two periods (`Q_d` with the filter's step, `R_d = D_v V D_vᵀ / Δ_v` with the
  measurement noise's own period, equal to the sampler's); the noise of a sampled sensor is
  `NoiseSource(Gaussian(0, cov=R_d), sample_period=Tₛ)`, so a datasheet's per-sample value is
  typed as it reads; wind and anything entering through drag is white noise through a filter;
  a one-sided density `n` per √Hz is `W = n² / 2`. Every noise signal answers both questions,
  its intensity (`W`, or `Σ Δ`) and its per-sample covariance at a period (`W / Δ`, or `Σ`), so
  a continuous and a discrete design read one object.
- **RD-A3. Random blocks enter analysis at their mean.** `linearize`, `find_equilibrium`,
  `transfer_function`, the LQR shortcuts and the deterministic transcriptions evaluate a
  zero-noise params transform, gathered over subsystems like `realize`. One form of it:
  `sys.realize(None)`, no key meaning no draw, as `RolloutEnvironment.step(key=None)` already
  means nominal; a reserved seed value makes each block return its mean and leaves `psd`
  readable by a design. The smaller option: a marker on random sources and the verbs refuse.
  With RN-1, which removes today's guard.
- **RD-A4. RN-1 is safe on its own.** The block publishes Δ and the simulator, when no solver is
  named, takes a fixed step that divides it (from RN-4). Writing `var`, `mean`, `t0` or `tf`
  raises, with `psd = var × sample_period`. The notebooks and tests listed in §9 join the step.
  `psd = 0` and a semidefinite `psd` are legal (element-wise root for a diagonal). To rule:
  whether the first positional argument stays `p`, the default `psd` and Δ, the fate of `mean`
  (dropped: a bias is a `Step` summed in). Fixed-step rollouts compute `t_k = t0 + k·dt`;
  64-bit is required on JAX.
- **RD-A5. No silent noise in the stepped tools.** A disturbance on a port that reaches only `h`
  is refused with a message, and the evaluator warns when `problem.sys` holds a random block
  (frozen across trials until RN-4): one line each, with RN-1.
- **RD-A6. Monte Carlo of an observer-based loop: the `simulator` backend realizes its trials.**
  No new rollout is needed. Checked 2026-09-30 on an LQG compensator (a `DynamicController`
  with ports `r`, `y` → `u`) and `NoisyPendulum`: `compensator @ plant` builds the
  four-state loop; `backend="simulator"` runs it, projects it on the plant with `trajectory_of`
  and scores it with the problem's cost, but returns J = 0 in every trial because it applies no
  draw; the NumPy and JAX backends refuse a controller with state. The same loop simulated by
  hand, a noise block wired on `w` and on `v` with a fresh seed per trial (RK4 at dt = Δ),
  gave a mean cost of 1.013 ± 0.023 times the covariance prediction over 240 trials. So RN-4
  and RN-5 gain three lines of scope: the simulator backend wires each `disturbances` signal
  onto its plant port inside the closed loop (RD-R4b, literally), applies `realize(key)` and the
  parameter draws per trial, and steps at the noise period; `backend="auto"` sends a controller
  with state there. It needs RD-D13 (a trial is `(x0, params)`; a noise-value array cannot feed a
  diagram) and the compensator block of P4's composition ruling. A JAX batch of the same loop
  is `rollout_batch` over a family of seeds, later. RL with observation noise stays out of
  scope until a learner reads `y`. Planners pin a declared disturbance port at its nominal
  value.
- **RD-A7. One key in the RL environment.** The episode's realization (start, parameter values,
  block seeds) is drawn at reset and carried; "RL training stays key-threaded" (§5) and "the
  environment reads the block's `h`" (RN-4) become that one sentence. `realize` derives each
  stream from a stable name (the subsystem id path, the parameter name), not from subsystem
  order, which would bring back the coupling RD-D10 removes. `ProblemEnv` realizes at reset or
  refuses a problem with disturbances.
- **RD-A8. The hold across integrator stages.** Drop "the left limit at RK4's last stage" from
  RN-4. State the effect (first order in dt, statistically small) and the rule dt = Δ / n.
  Evaluating a held source once per step, as a port disturbance already is, would make the two
  routes one mechanism and leave deterministic baselines untouched, but it is a change to
  `compile`: an open question for RN-4, the maintainer's.
- **RD-A9. What an H2 or H∞ tool reads.** `psd` is an LQG and H2 number: with
  `E[w(t) w(τ)ᵀ] = W δ(t − τ)`, the output variance under `W = I` is `‖G‖₂²` (checked:
  1.989062 by the Lyapunov equation, 1.989063 by the frequency integral). An H∞ tool reads no
  distribution: it reads the disturbance ports (`B_w = ∂f/∂w`), the performance outputs and
  weighting filters, and a weighting filter is the shaping filter of §2. The object every tool
  shares is a named disturbance port, with an optional filter inside `sys`; P5's loop inputs
  `w` and `v` are those ports. Nothing in RD-D1–RD-D12 stands in its way.
- **RD-A10. The deterministic twin as the acceptance test.** RN-1: the simulated variance at the
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
