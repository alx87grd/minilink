# What `t` means in the core signature: reviewer C, the future and the adversary

**Date:** 2026-10-28
**Audience:** Prof. Alexandre Girard (Maintainer)
**Status:** an independent review of the session's converged proposal on time in the core
(the brief of 2026-10-28), not a ruling. The other reviewers' reports were not read.
**Commit reviewed:** `d8d9321cc646b57707a0dd94425be55ee68f6e09` (`dev-v0.3`).
**Lens:** what comes later (MPPI, the seeded policy, the Kalman filters, gain schedules,
action chunks, event and wall clocks, Diffrax, a shrinking horizon, nested sampled loops, the
step problems, `grad` and `vmap` over the loop), and the adversary: the scenario where a rule
gives a wrong number silently, two ways to say one thing, or a block nobody can write.
**Read:** the brief's list in its order, then ROADMAP §5–§7, mppi.md §6, future-policies.md
§2–§5, estimation-and-state-feedback.md §5–§6, native-optimizers.md §4.6–§4.6b, randomness.md
§4, automation-by-convention.md §4.5–§4.6, the two QUESTIONS entries on the MPC's time and the
clock port, and the code the brief names (`WhiteNoise.h`, `sample_index`, the gain schedule in
`control/state.py`, the realtime loop and the MPC's `t0 + k·dt_mpc` included).

---

## 0. One screen

**Verdict: keep the converged proposal's skeleton, change one bullet.** The skeleton is right
and future-proof: time stays one channel (the `t` argument; the `t` port goes), the pure tick
world never sees seconds, `SampledSystem` is the sampled-data kind with a seconds face
(`update`, `output`) that is its equation and a tick face (`step`, `h`) for the step algebra,
native sampled laws (the MPC, a time-varying Kalman filter, MPPI, an action chunk) subclass it,
multi-rate is divisors in one pure map on the base tick, and the record is `t = t_start + k·Δ`.

**The bullet that is wrong:** "a sampled diagram passes indices; a bare static block in it reads
the index." A sampled diagram *has* seconds (`Δ` and `t_start` are known at every tick), so
handing the count to a block whose equation says `sin(ωt)` is choosing a wrong number when the
right one is one multiplication away. Concretely, under that bullet:

- `Sine(omega=1.0) >> (pid % 0.01) @ plant` runs the reference 100× too fast, silently; the
  user must know to write `Sine() % 0.01`. The same block on the other side of `@`
  (`(pid % 0.01) @ (Sine() >> plant)`) reads seconds. One block, two numbers, by wiring.
- `closed_loop(pid % 0.01, plant, disturbance=WhiteNoise(psd=W, sample_period=0.1))` draws a
  fresh sample every base tick instead of every 0.1 s: the noise intensity is off by the ratio
  of the two periods (§3.3, with the formula). `GaussianPolicy`'s draws `ε(seed, ⌊t/Δ⌋)`
  become `ε(seed, ⌊k/Δ⌋)`: common random numbers between the continuous evaluator and the
  sampled loop break, and the sentence of R129 ("in a sampled diagram, the tick index") is
  contradicted.
- `jax.grad` of the loop's cost with respect to `t_start` misses every bare reference: the
  number is wrong, not merely inconvenient.
- the two runners disagree: the host clock runner (realtime, hardware in the loop) can only hand
  a bare block wall seconds, the step diagram hands it a count, so "the step diagram is the
  runner's test oracle" (sampled-path §8.6) is false for any loop with a bare reference.
- point 6 ("blocks and the record agree on what `t` is") is false by point 1's own words: the
  record says `k·Δ`, the static blocks saw `k`.
- point 5 has no clean home for `t_start`: an attribute read inside `step` is a hidden input
  (no `grad`, a retrace per origin); a leaf in every clocked block's `params` is one number in
  N homes; a diagram leaf injected into each block's `params` is a key the block never
  declares. The only clean home is the run's argument, the time twin of `x0` — and then the
  only object that can turn `k` into seconds is the diagram's scan, which must hand seconds
  down. The bullet is not just a bad choice; it is hard to implement purely.

**My design (§1), in one sentence:** `t` is the clock of the diagram a block sits in — seconds
wherever the diagram has a period (continuous, sampled), the count where it has none (pure
step); a sampled diagram's scan carries `k` and the origin, computes `t_k = t_start + k·Δ` once
per tick, hands seconds to every block that reads the clock and its own count to a tick law;
the origin is a run argument like `x0`, never a parameter and never an attribute an equation
reads; a sampled loop compiles to one flat plan on one clock. It is the converged proposal with
its third bullet inverted, the origin's home fixed, and nesting flattened. Every scenario in the
brief then has one answer (§2), the step-diagram plan for pure step systems is untouched, no
static block is doubled, no block declares a unit, and the maintainer's "a step that knows the
clock and converts internally" is kept as the clocked objects' two faces.

**Regret risks beyond the bullet (§4):** the discretized problem's origin typed as an integer
tick (R148 says a step problem's `t_start` is a tick; `discretize(problem, dt)` returns a step
problem, so its seconds origin would be lost); a tick law designed at one rate silently running
at the base rate when left bare (R146); nested sampled boxes as clocked leaves (two clocks, the
origin view problem) instead of one flat plan; the words "time is a port" and "the MPC never
knows `Δ`" left in three plan docs.

---

## 1. My own design, derived before judging the proposal

### 1.1 The principle

**A block reads the clock of the diagram it sits in, through the one argument `t`; a diagram
with a period hands seconds, a diagram without one hands its count; the objects that own two
clocks (`SampledSystem`, `SampledDiagramSystem`) show both faces and convert at one line; the
origin of time is the run's, as the start of state is.**

Why this and not "the sampled diagram passes indices": a sampled diagram is a step diagram, but
it is a step diagram *with a period*. Its record is in seconds (R147). Its blocks were written
in seconds: every catalog source, the gain schedule (`interpolate_schedule(t_schedule, K, t)`),
the noise block (`⌊t/Δ⌋`), the seeded policy (R129), the MPC's `t_start`, the time-varying
plant `f(x, u, t)`, the cost `g(x, u, t)`, the sets `Z(t)`. The count is an implementation
detail of the scan (where `k mod d` is evaluated), and the only blocks whose equation is written
on the count are the pure tick laws. So the count goes to them, and to nobody else.

### 1.2 The rule per kind of system

| Kind | Equation | What `t` is | Who converts |
| --- | --- | --- | --- |
| static `System` (n = 0) | `h(x, u, t)` | the enclosing diagram's clock: seconds in a continuous or sampled diagram, the count `k` in a pure step diagram; a time-free block (`Gain`, `Sum`) never notices | nobody: one catalog, no unit, no mode |
| `DynamicSystem` | `f`, `h` in seconds | seconds, always; it lives in a continuous diagram or behind `discretize` / `%` | the integrator, or its `SampledSystem` |
| `StepSystem` (pure tick law) | `step(x, u, k)`, `h(x, u, k)` | its own count, always; it never sees seconds (chess, a grid world, a z-domain filter) | nobody |
| `SampledSystem(StepSystem)` | **the seconds face is the equation:** `update(x, u, t)` (Åström–Wittenmark's `x(kh + h) = F(x(kh), u(kh), kh)`) and `output(x, u, t)`; **the tick face serves the step algebra:** `step(x, u, k) = update(x, u, k·sample_period)`, `h(x, u, k) = output(x, u, k·sample_period)` | seconds inside; the count outside | the base class, one line, `t = k·sample_period`; a run with another origin passes it (§1.4) |

The implementations of `SampledSystem`, as the converged proposal lists them, with the face each
writes:

- `discretize(sys, dt)` of a continuous source: `update` is the step model of DESIGN §5
  (`step_body` from `t` over one period, the stages at `t + c_i·Δ`), `output` is `source.h` at
  `t`; its seconds face **is** today's `ev.step(x, u, t, dt=Δ)`, nothing new;
- a static source under `%`: `output(x, u, t) = source.h(x, u, t)`, no state; needed only to
  slow or hold the block (a divisor above one), never to give it seconds (§1.3);
- a tick source under `%`: `step(x, u, k) = source.step(x, u, k)` and `h` likewise, the count
  delegated; its `update(x, u, t)` is `source.step(x, u, ⌊t/period⌋)` with `sample_index`'s
  tolerance, used only by a runner on a wall clock; it declares that it *counts*;
- a native sampled law subclasses it and writes `update` and `output` in seconds: the delayed
  MPC (`update(z, y, t)`: bind `t_start = t`, shift by `t − t_last`, solve; `output`:
  `pv.policy(z, y, t)`), the discrete Kalman filter with `A(t)`, `C(t)`, MPPI, a diffusion
  policy's chunk; its per-tick seed is `child_seed(seed, ⌊t/period⌋)`, the noise block's own
  convention, so the same seed gives the same draws in the scan and in the host runner;
  `%` on a native law sets its period.

### 1.3 The rule per kind of diagram

- **Continuous `DiagramSystem`:** the integrator's `t`, seconds, to every block. Unchanged.
- **Pure `StepDiagramSystem`:** the scan's `k`, to every block; a static block reads `t = k`
  (what the code does today; R147's record `t = k`). Unchanged, and its plan is untouched by
  anything below.
- **`SampledDiagramSystem`:** a step diagram with a period. Its compiled plan is one pure map on
  the base tick, `X_{k+1} = F(X_k, U_k, k; t_start, P)`:
  - at wiring, `Δ` = the gcd of the clocked blocks' periods (a rational tolerance), each
    clocked block its divisor `d = period/Δ`, a period that is not a multiple refused;
  - on tick `k` the scan computes **`t_k = t_start + k·Δ` once**, then sweeps the ops in
    evaluation order: a block of divisor `d` fires when `k mod d == 0` (a select on the traced
    face), its outputs held in state slots between firings; every block fires at `k = 0`;
    signals propagate within the tick (R125);
  - **each op receives the time its equation reads:** a static block `t_k`; a clocked block
    `t_k` through its seconds face (`update`, `output`); a clocked tick source its own count
    `k // d` through its tick face (exact integer division of the scan's `k`); a bare tick law
    the base count `k` (R146: it runs at `Δ`);
  - **a nested sampled or step box is inlined into the flat plan** (ids joined as the flatten
    code already does, params remapped), so a loop has one clock and one origin, and the gcd is
    over every leaf; the box still exists as an object for `@`'s roles, `plot_diagram` and
    `trajectory_of`.

  The one rule in words: **a sampled diagram hands seconds to everything that reads the
  clock and the count to everything that counts.** Clocking a static block at the base period
  (`Sine() % Δ`) is therefore the identity: a test pins `Sine() >> (pid % Δ)` and
  `(Sine() % Δ) >> (pid % Δ)` to the same numbers.

  Why this is not "per-kind dispatch in the plan" in the sense the session rejected: the pure
  step diagram's plan passes one integer to every op, as today; only the sampled diagram's plan
  — a different class, which already has firing, divisors and hold slots — adds one line,
  `t_k = t_start + k·Δ`, and one `isinstance` it already performs (step op or port op) decides
  whether an op counts. The tick world stays pure: no `StepSystem` ever receives seconds.

### 1.4 The origin of time is the run's, the twin of `x0`

DESIGN §4 already separates the nominal from the start: "`x0` is the start of a simulation and
nothing else". Time has the same pair: the clock (structural, `sample_period`) and the start of
the run, `t_start`. The continuous evaluator already takes it as an argument (`rollout(x0,
u_seq, t0, dt)`). So:

- `t_start` is an argument of every run on a clocked object: `compute_rollout(n_steps,
  t_start=0.0)`, `compute_trajectory(..., t_start=0.0)`, `ev.rollout(x0, *, n_steps, t_start=0.0,
  params=None)`, `ev.step(X, U, k, params=None, *, t_start=0.0)` on a sampled diagram;
- under `problem.compile`, the problem's `t_start` leaf binds it, as `x_start` binds `x0`;
  `jax.grad` with respect to `t_start` flows through `t_k` into every clock reader (the
  references, the plant's `f(t)`, the MPC's bound `t_start`, the transition's cost and sets);
- it is never a key of `params` (the rule `require_dt_free` already states for the sample
  time) and never an attribute an equation reads; a clocked leaf never holds it, because the
  scan hands it seconds;
- the tick face of a clocked object, called by hand or by `jacobian(…, k)`, counts from the
  origin 0 (`t = k·sample_period`); a run with another origin names it, as it names `x0`.

### 1.5 What the design refuses or limits, on purpose

- A static block that *counts* (a table indexed by the tick) has no way to receive the count in
  a sampled diagram: there, time is seconds, and a thing indexed by the tick is a tick law.
  The library's own tables are in seconds (`TrajectorySource`, `pv.policy(z, x, t)`, the gain
  schedules), so nothing in the catalog is lost; it is the one deliberate limit, stated.
- `problem.compile(controller, dt=)` on a sampled controller reads `Δ` off the loop; a `dt`
  that differs is refused (D15's question, answered the only way `t_k` can be consistent).
- A continuous block and a step block still never mix except `sampled @ continuous` (R126).

---

## 2. The scenario table

`t` is what the block's equation receives; "scan" is the sampled diagram's compiled plan.

| Scenario | What `t` is | Who supplies it | Note |
| --- | --- | --- | --- |
| continuous diagram with a `Sine` reference | seconds | `Simulator` / the continuous evaluator | unchanged |
| time-varying plant `f(x,u,t)` discretized by RK4 | seconds at the stages `t_k + c_i·Δ`, `t_k = t_start + k·Δ` | `SampledSystem.update` (the step model) from the scan's `t_k` | the stages never see a count |
| chess / grid world, pure step diagram, with a `Gain` and a `Step` source | the count `k`; `Step(t_step=3)` fires at the third move | the pure step plan | there are no seconds to give; `t = k` as today |
| z-domain filter (tick law) alone | its count | `compute_rollout` | record `t = k` |
| the same filter bare inside a sampled loop | the base count `k` | the scan | runs at `Δ` (R146); a filter designed at a rate should name it, one sentence in the docs |
| the same filter under `%` (`filt % 0.02` in a `Δ = 0.01` loop) | its own count `k // 2` | the scan, through the tick face of its wrapper | exact integer division |
| `pid % 0.01 @ plant` | seconds `t_k` to the PID's `update` (its Euler difference equation; `f` ignores `t`) and to the plant's step model | the scan | `Δ = 0.01`, both divisors 1 |
| `(filt % 0.01) >> (ctl % 0.1) @ plant` | `filt`: `t_k` every tick; `ctl`: `t_k` on `k mod 10 == 0`, its output held in a slot between | the scan | all three fire at `k = 0` |
| `(mpc % 0.05) @ plant`, the MPC binding `t_start` | seconds `t_k` to `update(z, y, t)`, which binds the planner's `t_start = t`; `shift(z, t − t_last)` | the scan; the MPC's own state holds `t_last` | the MPC knows its period (harmless, it never uses it); the `t` port is gone |
| dual rate `(mpc % 0.05) >> (follower % 0.01)` | MPC: `t_k` on `k mod 5 == 0`; follower: `t_k` every tick, reading the held `z` | the scan | one slow period of delay, as R148 says |
| `TrajectorySource(traj)` reference in a sampled loop, bare | seconds `t_k`: the trajectory read at the right time | the scan | **converged: the count, 100× too fast** |
| gain schedule `K(t)` (`ScheduledStateFeedback`) sampled, clocked or bare | seconds; `interpolate_schedule(t_schedule, K, t_k)` | the scan | **converged, bare: `K(k)`** |
| discrete Kalman filter with a time-varying model | a native `SampledSystem`: `update(x̂, [u; y], t)` with `A(t)`, `C(t)` in seconds; a time-invariant one is a tick law `kf % dt` | the scan | the EKF stays continuous (`f` on `model.f`) |
| sampled loop started at `t_start = 100` with a time-varying cost (`problem.compile`) | `t_k = 100 + k·Δ` to every clock reader; the transition's `c_k`, sets and references at `t_k` | the problem's `t_start` leaf → the loop's run argument | **converged: a bare reference reads `k` and never sees 100** |
| `discretize(problem, dt)` and `StepPlanningProblem` on a tick law | tick law: `transition(x, u, k)` calls `step(x, u, k₀ + k)`, `c(x, u, k₀ + k)`, `k₀` an integer origin; sampled: `update(x, u, t_start + k·dt)`, `t_start` in seconds | the compiled problem | the sampled problem must keep its clock in seconds (§4, risk 2) |
| the RL environment on `transition` | as the row above; a seeded policy inside draws from `⌊t_k/Δ⌋` | the environment's episode counter → the transition | common random numbers with the evaluator hold |
| nested: a continuous `DiagramSystem` as the source of `discretize()` | seconds at the stages to every inner block | `SampledSystem.update` | unchanged |
| a `SampledDiagramSystem` as a block of a bigger sampled diagram | one clock: the inner box is inlined; its blocks keep their periods; `Δ` = gcd over every leaf; each leaf fires on its own divisor and reads `t_k` | the outer scan | "two base clocks" never exist; a non-multiple period is refused at wiring as for a leaf |
| the realtime runner / hardware in the loop | wall seconds to every clock reader through the same faces (`h`, `update`, `output`); a tick law counts its firings | the host clock runner | both runners hand the same block the same kind of time: the step diagram is the runner's oracle at the ticks |
| delay compensation (`t + Δ`) | a keyword on the MPC (`lookahead=Δ`, added to the bound `t_start`) or a predictor block on `x_start` | the MPC | the port rewiring of R148 goes with the port |
| event-triggered replanning (out of scope) | a native law's `update` may select on a signal; a future event runner calls the seconds faces at event times and counts a tick law's firings | — | not precluded: the seconds face is the hook; the `k mod d` select never leaks into a block's contract |
| asynchronous or incommensurate clocks (out of scope) | refused by the scan's gcd rule; a future host runner over the seconds faces | — | not precluded for the same reason |
| Diffrax as an optional solver | `update(x, u, t)` integrates `[t, t + Δ]` with any integrator | `SampledSystem.update` | the seconds face is the solver's natural call |
| a shrinking-horizon MPC (later) | `update(z, y, t)` binds `t_start = t` and `tf = T_end − t` once `tf` is a leaf | the MPC | needs seconds; works |
| `jax.grad` through the sampled loop w.r.t. a controller gain | the scan over `k`; `k mod d`, `k // d` on the integer path | — | as today |
| `jax.grad` w.r.t. `t_start` | an argument of `rollout`; flows through `t_k` to every clock reader, zero through `⌊t/Δ⌋` (piecewise constant, as expected) | — | **converged: bare references contribute zero** |
| a Monte Carlo (`vmap`) over starts and seeds on the loop | `x0`, `t_start` and integer seed leaves batched; draws from `(seed, ⌊t_k/Δ⌋)` | — | as the noise block |
| `compute_rollout` / `compute_trajectory` on each kind | pure step: `t = k`; sampled: `t = t_start + k·Δ`; dense: the substep times | the facades | what the blocks read is what the record says |

---

## 3. Verdict on the converged proposal

### 3.1 Keep

- **Point 1, first two bullets and the principle:** time belongs to the loop, never to a block;
  one channel; a continuous diagram hands seconds; a pure step diagram hands the count and a
  static block in it reads `t = k`. Right, and the end of the `t` port (R148) is right: a port
  would be a second channel for time (CONSTITUTION §2, one signature), would need a clock
  source in every continuous diagram, and would carry "seconds or ticks, whatever was wired",
  which is the unit declaration moved onto the wire.
- **Point 2 entire:** `SampledSystem` as the sampled-data kind with the seconds face as its
  equations and the tick face derived; the four implementations; native laws subclassing it.
  This is what makes hardware in the loop, Diffrax, the shrinking horizon and event clocks
  possible: a runner on a wall clock has a face to call. "Nobody subclasses it" (R146) rightly
  becomes "a native sampled law subclasses it".
- **Point 4:** multi-rate in the compiled plan as one pure map, divisors, hold slots as state,
  every block firing at `k = 0`, same-tick propagation; all divisors 1 reproducing today's
  step plan exactly.
- **Point 6:** the record `t = k` or `t = t_start + k·Δ`.
- The maintainer's rejections stand: no unit declaration, no seconds into a pure
  `step(x, u, k)`, no third static kind, no doubled catalog. My design needs none of them.

### 3.2 Wrong: the third bullet of point 1, and point 3 with it

"A sampled diagram IS a step diagram: it passes indices too. Seconds exist only inside a clocked
block. A bare static block in a sampled diagram reads the index." Four independent reasons it
should be inverted, each a concrete number:

**(a) The silent rate error.** The canonical teaching line of the conventions plan (AC-8b,
automation-by-convention.md §4.5: "`Step() >> ctl % dt @ plant` (the source on the continuous
side, sampled by the computer)") is wrong under the bullet for any source with a time scale:

```python
loop = Sine(omega=1.0) >> (pid % 0.01) @ plant     # Sine is a bare static block of the sampled diagram
loop.compute_rollout(n_steps=1000)                  # the reference is sin(k): one radian per tick, 100 rad/s
loop = (pid % 0.01) @ (Sine(omega=1.0) >> plant)    # the same Sine inside the discretized plant: sin(t)
```

The same block gives two numbers depending on which side of `@` it sits; nothing raises. The
remedy the proposal offers (`Sine() % 0.01`) is a rule a student must remember for every
reference, every schedule, every noise source, and forgetting it produces a plausible-looking
plot. Three years on, every tutorial carries `% dt` on its references "because you have to".

**(b) A physical quantity wrong by the period ratio.** The noise block (randomness.md §4) holds
`w_k = L·ε(seed, ⌊t/Δ_n⌋)` with per-sample covariance `W/Δ_n`, so the intensity is `W` whatever
`Δ_n` is. With `t = k` (the base count of a `Δ = 0.01` loop) and `Δ_n = 0.1`:

```text
sample index = ⌊k / 0.1⌋ = 10 k      a fresh sample every base tick, held 0.01 s, covariance W / 0.1
effective intensity = (W / 0.1) · 0.01 = 0.1 W      wrong by Δ / Δ_n
```

and `closed_loop(pid % 0.01, plant, disturbance=WhiteNoise(psd=W, sample_period=0.1))` puts
the noise exactly there, a bare block on the computer's side of the sampled loop. The seeded
policy (R129) is the same block form: `ε(seed, ⌊k/Δ⌋)` is not "the tick index of R104" as
R129 promises, and common random numbers between the continuous evaluator's run and the
sampled run of the same policy and seed, the point of §5.4 of future-policies.md, are gone.

**(c) The gradient with respect to the origin.** With `t_start` a leaf of the problem and the
loop started at `t_start = 100`, `jax.grad(J)(t_start)` on a tracking cost against a bare
`Sine` is zero through the reference (it read `k`), while the cost, the sets and the MPC saw
100. The number is wrong, not the convenience.

**(d) The two runners disagree.** The host clock runner (sampled-path §8.6: realtime, hardware
in the loop, host-only blocks) has only wall seconds to hand a bare static block; the scan
hands it a count. For any loop with a bare reference, "the pure step diagram is the runner's
test oracle" cannot hold, and the realtime re-base on the pure blocks (TODO §1.5) inherits a
disagreement by design.

And an internal one: point 6 says "blocks and the record agree on what `t` is"; under point 1
the record says `k·Δ` and the static blocks saw `k`. They do not agree.

### 3.3 Hard to implement purely: point 5's home for `t_start`

Under the bullet, each clocked block converts `k // d` to seconds inside its tick face, so each
needs `t_start`. The proposal says "one leaf of the sampled diagram's tree, written through to
every clocked block (like `x0` views, R124)". Every reading of that sentence breaks a rule:

| Home | What breaks |
| --- | --- |
| an attribute `self.t_start` read inside `step` | an equation reading a per-run number from an attribute: not pure (CONSTITUTION §5.1), invisible to `grad`, a retrace per origin, and `problem.compile` cannot bind it per call |
| a leaf in every clocked block's `params` | one number in N homes (RULES 5.6); N gradient leaves to tie by hand; `loop.params["plant"]["t_start"]` beside `["m"]` for a view whose `params` are "the source's own dict" (R146) |
| a diagram-level leaf injected into each op's `params` by the plan | a key the block's own `params` never declare (a hidden input; standalone `sampled.step(x, u, k)` reads a key that is not there), and a run quantity in `params` against the `x0` precedent and the `dt` rule |
| a keyword on the tick face, `step(x, u, k, params, t_start=)` for clocked ops only | the dispatch the proposal wanted to avoid, with a worse signature |

The clean home is the fourth one DESIGN already has: a run argument, like `x0` and like the
continuous evaluator's `t0`. And once the origin is the scan's argument, the scan is the only
object that can form `t_k`, so it must hand seconds down. The bullet is not independent of
point 5; fixing point 5 inverts the bullet.

### 3.4 Smaller risks in the proposal

- **Ports of a view.** Under the bullet, every port function of every clocked block must be
  wrapped to convert `k` to seconds (today's `Discretization` wraps `y` and `x` only, so a
  custom port such as a manipulator's `p` is lost). When the scan hands seconds, a view's port
  functions can be the source's own, unwrapped, and "a view, not a container" is literally
  true.
- **A nested sampled box as a clocked leaf** ("two base clocks?"): with `k // d` handed in and
  `t_start` needed inside, the box has the §3.3 problem one level down. Inlining nested step
  and sampled boxes into one flat plan removes the question: one clock, one origin, the gcd
  over every leaf.
- **"The MPC never knows `Δ`"** (R148, planner-and-mpc.md §6.1, §10 point 4, automation §4.6):
  under both the proposal and my design the MPC is a `SampledSystem` child and knows its
  period. Harmless (it never uses it: `shift` takes `t − t_last`), but three documents say the
  opposite and still say "time is a port".

### 3.5 Relation of my design to the proposal

A refinement, not a different design: points 1 (first two bullets), 2, 4 and 6 kept as written;
point 1's third bullet inverted (a sampled diagram hands seconds to clock readers, the count to
tick laws); point 3 reduced to "a clock on a static block slows or holds it" (never needed to
give it seconds); point 5 made precise (the origin is a run argument, the scan forms `t_k`
once); nesting flattened. It is the session's third alternative ("the evaluator handing seconds
to static and sampled blocks and `k` to tick laws") made exact and justified: the objection
"then the tick world is not pure" does not hold, since the pure step plan is untouched and no
`StepSystem` ever receives seconds; the objection "then there are two converters" is answered by
one formula in one base class and one evaluation of it per tick in the scan.

### 3.6 The alternatives in the brief, stressed the same way

- **The `t` port (R148 as ruled):** a second channel for time; every time-reading block in the
  catalog would need the port (or the sources read the argument in continuous diagrams and
  the port in sampled ones, a doubled catalog by the back door); the port holds "seconds or the
  tick" by wiring, so the unit question moves to the wire and the trap of §3.2(a) becomes a
  wiring trap; a clock source block in continuous diagrams. Its one merit, delay compensation
  by rewiring, is a keyword. Dropping it is right.
- **A unit declaration on the block:** a mode in the core; a block would have to be re-declared
  to move between a chess diagram and a sampled loop. Rejected rightly.
- **Three static kinds or a doubled catalog:** the same block exists twice; every operator and
  tool learns the difference. Rejected rightly.
- **Per-kind dispatch in the plan (the session's third alternative):** right in substance;
  what it lacked was the origin's home (§1.4), the flattening of nested boxes, and the
  statement that the pure step plan is untouched. My design is this one, completed.

---

## 4. Regret risks: three years on

1. **The silent rate error (§3.2).** If the bullet stands, the maintainer will be explaining to
   a cohort why `Sine() >> (pid % 0.01) @ plant` is 100× too fast and `% 0.01` on the Sine is
   not optional, and reviewing demos for a missing `%`. Inverting the bullet now costs one line
   in the sampled plan; inverting it after v0.4 moves every sampled demo's numbers.

2. **The discretized problem's origin typed as a tick.** R148: "on a step problem `t_start` is
   an integer tick". `discretize(problem, dt)` returns a `StepPlanningProblem` (R141), so by
   class its origin would be a tick and `t_start = 100.005` would be rounded or refused, and
   `grad` with respect to it lost. Three kinds of system, two kinds of problem: the sampled
   problem is where the missing third kind shows. The fix is small and should be ruled with
   this question: a problem on a clock keeps `(dt, t_start)` in seconds (a `clock` on the step
   problem, or a `SampledPlanningProblem` sibling as `SampledSystem` is to `StepSystem`); its
   transition forms `t_k = t_start + k·dt` and calls the plant's seconds face.

3. **A bare tick law at the base rate (R146).** A z-domain filter designed at 100 Hz, left bare
   in a loop whose base became 1 kHz when an MPC at `0.001` joined, runs at 1 kHz with no
   error. Unlike the Sine, its equation says nothing about seconds, so the base count is its
   only possible meaning; I do not propose a refusal, but the docs should carry one sentence
   ("a tick law designed for a rate names it: `filt % 0.01`") and the names sweep should keep
   `%` the natural spelling.

4. **Nested sampled boxes as clocked leaves.** Two clocks and an origin view inside every
   `controller` box of `computer @ plant` (the common case, since `@` nests). Inline at plan
   build; keep the box as an object. Deciding this late means rewriting the sampled plan once
   the multi-rate demos exist.

5. **Two counters for draws, stated once.** Seconds readers draw from `(seed, ⌊t/Δ⌋)` (the
   noise block, the seeded policy, the MPC's per-tick seed), pure tick laws from `(seed, k)`
   (D14). Both are already in the plans; DESIGN should say it in one sentence so MPPI, the
   particle filter and the diffusion policy do not each choose.

6. **The origin's name.** The continuous evaluator says `t0`, the problem says `t_start`, the
   MPC today says `t0`. One word for the run's origin: `t_start` on the facades and the problem;
   whether the evaluator's `t0` is renamed belongs to the names sweep (QUESTIONS, the `dt` /
   `sample_period` entry is the same kind of decision).

7. **The words left behind.** "Time is a port" (planner-and-mpc.md §6.1, §6.4, §10 point 4;
   automation-by-convention.md §4.6; the QUESTIONS entry "The clock port `t`"), "nobody
   subclasses it" (sampled-path.md §8.1, R146), "the MPC never knows `Δ`" (R148). An agent
   reading them next week builds the port. They go in the landing change of this ruling.

8. **What stays open on purpose, and is not precluded:** event-triggered and asynchronous
   clocks (ROADMAP §7) keep their hook, the seconds face and a future host runner over it;
   the scan's `k mod d` select never appears in a block's contract, so a block written today
   runs under a future event runner unchanged.

---

## 5. Naming, and the contract sentences for DESIGN §4

**Names.** `t` stays the one argument; `k` is what the docstrings call it on a pure
`StepSystem` and on the tick face. `SampledSystem` (R146), `SampledDiagramSystem`. The seconds
faces `update(x, u, t, params)` and `output(x, u, t, params)` (the converged names; `advance`
would also do for the first; I would not spend a sweep on it). `sample_period` on the object,
`dt` on the calls (QUESTIONS' option (a)). `t_start` for the run's origin everywhere a user
sees it.

**Sentences** (for the `System` entry and a new "Time" paragraph of DESIGN §4; the step model
paragraph of §5 gains the one on `update`):

1. **Time is one channel.** Every equation reads time through its third argument, `t` in
   `(x, u, t; params)`: no time port, no unit on a block, no second channel. What `t` holds is
   decided by the diagram the block sits in, never by the block, so one static catalog serves
   every kind of diagram.
2. **Three clocks.** A continuous diagram hands seconds, the integrator's time. A pure step
   diagram has no period and hands its count: `step(x, u, k)`, `h(x, u, k)`, and a static block
   in it reads `t = k`. A sampled diagram has a period `Δ` and hands seconds, `t_k = t_start +
   k·Δ`, to every block that reads the clock; a tick law in it receives its own count.
3. **Two faces on a clock.** A `SampledSystem` writes its equations in seconds, `update(x, u,
   t)` and `output(x, u, t)` (the step model for a continuous source, `h` sampled for a static
   one, a native sampled law for the MPC and its kin), and presents the step algebra's faces
   `step(x, u, k)` and `h(x, u, k)` as `t = k·sample_period`, written once. A `SampledSystem`
   of a tick source delegates the count and never converts. A pure `StepSystem` never sees
   seconds.
4. **The origin is the run's.** `t_start` is to time what `x0` is to state: an argument of
   `compute_rollout`, `compute_trajectory` and the evaluator's `rollout` and `step` (default
   `0.0`), bound from the problem's `t_start` leaf by `problem.compile`; never a key of
   `params`, never read by an equation from an attribute. The tick face counts from `0`.
5. **Multi-rate.** Periods sit on blocks; `Δ` is their gcd; a block of divisor `d` fires on
   `k mod d == 0` and holds its outputs in state slots between firings; every block fires at
   `k = 0`; signals propagate within the tick. A nested box is inlined at compile, so a loop has
   one clock. Clocking a static block at the base period changes nothing; a clock on a static
   block slows or holds it.
6. **The record agrees.** `compute_rollout` returns `t = k` on a pure step system and
   `t = t_start + k·Δ` on a sampled one; `compute_trajectory` keeps the substeps. What a block
   read is what the record says.
7. **Draws.** A block that reads seconds draws from `(seed, ⌊t/Δ⌋)` of its own period, the
   noise block's convention; a pure tick law from `(seed, k)`.

**The tests that pin it** (for the row, not for DESIGN): `Sine() >> (pid % Δ) @ plant` equals
`(Sine() % Δ) >> (pid % Δ) @ plant` bit for bit; the loop at `t_start = 100` with a
time-varying cost gives the same `J` as the continuous closed loop sampled at the ticks
within the step model's tolerance; `jax.grad` with respect to `t_start` matches a central
difference on a loop with a bare `Sine` reference; the host clock runner matches the scan at
the ticks on every hybrid demo, a bare reference included; a `WhiteNoise(sample_period=10Δ)`
bare in a `Δ` loop reproduces its continuous-diagram realization at the ticks with the same
seed.

---

## 6. What I checked against the code

- `StepDiagramEvaluator.port_signals` hands the same `k` to every op, static ones included
  (`core/compile/evaluators/discrete.py`): the sampled plan is the one place where
  `t_k = t_start + k·Δ` enters, and the pure step plan keeps this line untouched.
- `Discretization.step` and `h` already form `t_k = k·dt` from the count and call the source
  in seconds (`analysis/discretization.py`): the seconds face exists today as `ev.step(x, u,
  t, dt=)`; naming it `update` and letting the scan call it with `t_k` is a rename, not a
  rewrite.
- `WhiteNoise.h` and `sample_index` (`blocks/sources.py`, `core/distributions.py`) define the
  library's counter as `⌊t/Δ⌋` of absolute seconds with a tolerance; §3.2(b)'s number follows
  from `Σ = W/Δ`.
- `ScheduledStateFeedback.ctl` reads `t` in seconds through `interpolate_schedule`
  (`control/state.py`): a gain schedule is a static block that reads the clock, the §3.2(a)
  trap verbatim.
- `Computer.tick` fires on `k_tick % divisor` and reads the previous tick's buffer
  (`simulation/discrete/computer.py`): the divisor logic carries over; the one-tick latency
  goes with R125, unrelated to this question.
- The realtime loop advances `t += dt_hold` in wall-quantized seconds and calls
  `evaluator.step(x, u, t, dt=dt_hold)` (`simulation/realtime/simulator.py`): a runner that
  can only hand seconds, the §3.2(d) point.
- The MPC's `t_solve = t0 + k·dt_mpc` (`planning/trajectory_optimization/mpc/controller.py`)
  is the conversion the two faces replace; its `t0` is the origin that becomes the run's.
