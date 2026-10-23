# The sampled path: the step model and the sampled loop

**Status:** ruled 2026-10-22/23 (R6–R8, R12–R14, R17, R18 of
[the rulings](../reviews/2026-10-22-rulings-session-1.md), amended by session 3: R40 the hold on
sequences only, R42 the keyword `solver=`, R43 the RL episode a training setting); aligned to
[continuous-core.md](continuous-core.md), the design of record for the phase, on 2026-10-23.
First written 2026-10-21 as an evaluation of the code and a recommendation.  
**Lane:** core (the compiled evaluator, `discretize`, every sampled consumer) — every step is
**[ask — core]**.  
**Rung:** v0.3, the continuous-core phase of [TODO.md](TODO.md) §4: step S75 (the step
model, §1–§7, steps SM-0 to SM-7) first; V1's steps 1–2 may run beside it, and its step 3 (the
whole run out of the scan) needs SM-1. S31 (the sampled loop, §8) is ruled in principle and
its code comes after the continuous core: in v0.3 if the calendar allows, else v0.4 (the slot
is raised in ROADMAP §6).  
**Gathers:** S31, S37 whole (its one scannable step as SM-1, its integrators' move and the
evaluator-grid trim as SM-6; ruled 2026-10-22, S27 Diffrax stays in v0.9), the `discretize`
follow-ups, S65's input-hold item and the Later row on a sample stride. SM-6 is the one home
of the grid trim; no K row carries it.

Many features turn a continuous plant into a step map: reinforcement learning, dynamic
programming, the gym bridge, trajectory optimization, the hybrid and realtime loops, RRT,
the Lyapunov sampler, MPPI, and soon identification and behavior cloning. Each does it its
own way today. This plan proposes one contract, one public verb and one implementation for
that conversion (§1–§7), then the closed loop that runs a sampled controller on it (§8).

## 1. What the code does today: eight discretizations of the same plant

Every row computes "the state one sample period later, the input held", or close to it.

| Consumer | How it steps the plant | Substeps | Hold | Body |
| --- | --- | --- | --- | --- |
| `discretize()` ([`analysis/discretization.py`](../../minilink/analysis/discretization.py); GRO501 digital implementation, tutorial 04) | RK4 or Euler on `source.f` | 1 per `dt` | zero-order | hand-written |
| `RolloutEnvironment` ([`reinforcement_learning/environment.py`](../../minilink/planning/reinforcement_learning/environment.py); also the Monte Carlo evaluator's and the tabular learners' plant) | compiled `rk4_step_trace(_p)` | 1 | zero-order | compiled evaluator |
| `Sys2Gym` ([`interfaces/gymnasium.py`](../../minilink/interfaces/gymnasium.py)) | compiled `rk4_step` / `euler_step`, jitted by its own `_compiled_step` | 1 | zero-order | compiled evaluator |
| the DP grid ([`policy_synthesis/discretizer.py`](../../minilink/planning/policy_synthesis/discretizer.py), two places) | inline `x + f(x, u, t) dt` | 1 | zero-order | hand-written Euler |
| shooting and multiple shooting (`rk4_step_between_knots` in [`transcription.py`](../../minilink/planning/trajectory_optimization/transcription.py)) | RK4 | 1 per knot | **first-order** (linear between knots) | hand-written |
| direct collocation | trapezoidal defects | — | linear | implicit: a transcription, not a step map; correct as is |
| `HybridSimulator`, `RealtimeSimulator` | compiled `integrate_zoh(..., dt_inner=)` | **n** (the only ones) | zero-order | compiled evaluator |
| the RRT extender, the Lyapunov sampler, the MPPI scripts | compiled `rk4_step` in a Python loop, or `rk4_integrate_zoh_trace` | n, by hand | zero-order | compiled evaluator |

Underneath sits a grid of about 36 integrator methods on the compiled evaluator (`rk4_step`,
its `_p`, `_trace`, `_trace_p` twins, the same for Euler, `rk4_integrate_zoh*`,
`rk4_integrate_linear*`, `integrate_zoh*`, `rollout_batch`), 1763 lines in
`jax_evaluators.py` and 646 in `numpy_evaluators.py`, which the K-wave row already flags.

What it costs:

- **The same plant is a different model in each pillar.** A pendulum at `dt = 0.05` is Euler
  in value iteration, one RK4 step in PPO, RK4 with a linear input in trajectory
  optimization, trapezoids in collocation MPC, and adaptive RK45 when `compare()` and the
  Monte Carlo evaluator replay the laws. The "value iteration vs LQR vs PPO on one problem"
  lesson compares laws trained on three discretizations and scored on a fourth.
- **No accuracy knob where it matters.** RL, the gym bridge, DP and `discretize` take exactly
  one integration step per control period. For the racecar or a stiff arm at a realistic
  control rate that is the whole error budget; only the hybrid and realtime simulators can
  sub-step.
- **Four hand-written integrators** beside the compiled one, each with its own handling of
  `params`, backend and tracing; `discretize` steps `source.f`, so a diagram is discretized
  through the Python wiring rather than the compiled graph.
- **The planned pillars would add more.** Identification and behavior cloning both "roll the
  plant at the logging rate"; without a standard each brings its own.

## 2. What the roadmap already plans

| Row | What it says | Its part of this |
| --- | --- | --- |
| S31 (ROADMAP §6, recommended 2026-10-20) | one sampled rollout kernel in a NumPy and a JAX face, called by the RL environment, `HybridSimulator` and the MPC export | the closed-loop half; it presupposes the plant step this plan standardizes |
| S37 (absorbed here, 2026-10-22) | evaluators keep pure maps and one scannable step; integrators move to `simulation/solvers/`; first dedupe the zero-order-hold sugar and the JAX step rollout | SM-1 and SM-6 |
| the `discretize` follow-ups (TODO v0.9) | an exact zero-order hold (the matrix exponential) shared with `step_response`; the `params["dt"]` fallback; the ports kept; `PID % dt` needs a discretization method | the public verb |
| S65 (wave D) | the forced-input hold as one `input_interp` option every solver obeys | the hold rule |
| Later (TODO §7) | a sample stride on the fixed-step solvers (`substeps=k`) | the substep knob |
| the evaluator-grid trim (the core-objects review's finding; its home is SM-6) | the evaluator grid trimmed to `_p` bodies plus generated bindings | the trim this enables |
| V1 ([differentiable-problem.md](differentiable-problem.md)) | the continuous closed loop as a diagram (RD-D24); `rollout_batch` with a params family (S53) | stays continuous; reads the same compiled RK4 |

The pieces are planned; the contract that ties them is not.

## 3. The proposal

### 3.1 One contract: the step model

$$x_{k+1} = \Phi(x_k, u_k, t_k; p)$$

the plant integrated over one sample period with the input held: pure in arrays, traceable
in every argument (`params` included), the same on NumPy and JAX. Three knobs, no more (R40:
a step map holds its input by definition, so the hold is not a knob of the step):

| Knob | Values | Default |
| --- | --- | --- |
| `dt` | the sample period, owned by the step model and never a key of `params` (as `discretize` already rules) | — |
| `solver` | `"rk4"`, `"euler"`, `"exact"` (the matrix exponential, linear models only); the time-integration keyword of every tool that integrates (R42) | `"rk4"` |
| `substeps` | integration steps per sample | `1` (today's numbers) |

**The hold belongs to a sequence (R40).** A first-order hold interpolates from `u_k` to
`u_{k+1}`, which a one-step map `step(x, u, k)` never receives; so `hold="zoh" | "foh"` (`foh`
being what shooting uses between knots) is a keyword of `rollout` and `compute_forced`, which
hold a sequence, never of `step` or `discretize`. A callable `u(t)` given to `compute_forced` is
evaluated at the RK4 stage times (no hold); a sampled input matrix is held, `zoh` by default.

**Parameters are an argument, always.** The step is `step_p(x, u, t, params)`, and there is
no frozen twin. The parameterless call students use stays, as a binding: the parameters
bound once (`partial(step_p, params=sys.params)`) and the result jitted, which JAX treats as
constants. That is today's frozen behaviour (the evaluator deep-copies `params` at compile
time and closes over them), obtained from the one body (§3.5).

### 3.2 Constant-step explicit integration on this path, and only on it

A fixed step is what `scan`, `vmap` and `grad` want, it is deterministic, and its error is
set by `substeps` rather than tolerances. Adaptive RK45 stays the default of `Simulator`,
the tool that answers "what does the plant really do". The step model is the plant every
sampled consumer sees: RL, DP, MPC replay, the gym bridge, the digital controller, and
later identification and imitation.

### 3.3 One public verb: `discretize`

`discretize(plant, dt, *, solver, substeps) -> StepSystem` becomes the standard path,
as it already is in the GRO501 digital-implementation lesson. The result is a `StepSystem`:
it composes in a `StepDiagramSystem`, runs `compute_rollout`, and its `compile()` gives the
scannable step. Its body stops being hand-written and calls the compiled step.

**Two clocks, two signatures (R14).** The compiled evaluator's `ev.step(x, u, t, params, *, dt,
…)` takes the time in seconds, as `f` does; a `StepSystem`'s `step(x, u, k, params)` takes the
integer tick. `discretize` binds `t_k = t0 + k·dt` between the two, so the tick is the step
contract of every sampled consumer and seconds are the continuous plant's business. `integrator=`
becomes `solver=` (at every caller in one sweep, R82; R42, which replaces R6's
`method=`: `method=` stays the verb's own numerical method, the derivative or the optimizer's
algorithm, and `integrator=` would clash with the `Integrator` block).

### 3.4 One implementation

In `core/compile`, one step per backend, `step(x, u, t, params; dt, solver, substeps)`, and one
rollout over it, `rollout(x0, u_seq, t0, dt, params; solver, substeps, hold)`,
an `xp` body where possible; the 36-method grid becomes bindings over it (the grid trim,
SM-6), and `integrate_zoh(dt_inner=)` is `substeps` under another name. SM-6 then moves the
integrator to `simulation/solvers/`, the move S37 planned. The differentiable problem's
evaluator (V1) is this same step applied to the whole closed-loop diagram with the controller
inside `x`: a continuous controller is never held, so it is not the sampled path, but it reads
no other integrator.

### 3.5 Why one params path

Measured on the current JAX evaluator (`benchmarks/params_binding.py`; 2026-10-22, CPU, JAX
0.12, float64), the frozen RK4 step against the one taking `params`, and against the frozen
name defined as a binding (`jit(partial(step_p, params=sys.params))`); identical results in
every case:

| Case | Frozen | `params` dict each call | `params` bound |
| --- | --- | --- | --- |
| pendulum, one jitted step called from Python | 10.1 µs | 13.6 µs | 10.0 µs |
| cart-pole, one jitted step called from Python | 14.4 µs | 17.5 µs | 13.6 µs |
| pendulum, 1000 RK4 steps in one scan | 2.59 ms | 2.33 ms | 2.52 ms |
| cart-pole, the same scan | 4.55 ms | 4.52 ms | 4.42 ms |
| pendulum, 1000 rollouts × 1000 steps, `vmap` | 139 ms | 127 ms | — |
| cart-pole, the same batch | 1307 ms | 1307 ms | — |
| pendulum, the same batch with one `m` per rollout (a family) | — | 673 ms | — |
| cart-pole, the same batch with one `lcg` per rollout | — | 1276 ms | — |
| `jax.grad` of a rollout cost w.r.t. every param (pendulum / cart-pole) | impossible | 5.5 / 10.1 ms (value 2.3 / 4.6 ms) | — |

- **Inside a compiled scan or `vmap` there is no difference**: RL training, batch rollouts,
  trajectory optimization, MPPI and the differentiable tuning all run there.
- **Step by step from Python, the dict costs about 3 µs per call**, constant whatever the
  plant: a trivial jitted function dispatches in 5.2 µs, 7.9 µs with a dict argument. On a
  10 to 15 µs step that is 25 to 35 %, and it matters only where a Python loop calls the step
  one at a time (the gym bridge, the RRT extender, the realtime loop at 30 Hz). **The binding
  removes it**: the bound form equals the frozen call within noise. (An earlier measurement on
  another machine read 13 vs 34 µs; the ratio here is 1.3, not 2.5.)
- **A gradient through the rollout costs about 2.2× the value**, and only the `params` form
  can take it.
- **The parameter family is the one real cost** (S53, a separate row): 5× on the pendulum,
  none on the cart-pole.
- **The grid halves.** Every integrator exists frozen and with `params` today, before the
  trace and jit tiers double it again; one `_p` body plus a binding is the trim SM-6
  delivers.
- **A bug class goes.** A frozen evaluator silently ignores parameters changed after
  `compile()`; with one path, freezing is an explicit act at a visible place.
- **The stable names hold.** DESIGN §5's frozen evaluator subset (`f`, `rk4_step`,
  `integrate_zoh`, their trace twins) keeps every name; each parameterless one becomes the
  binding over its `_p` body.

It does not fix S53: a batch with a *different* parameter set per rollout (a family under
`vmap`) runs about 10× slower than the plain batch, a separate problem.

### 3.6 The evaluator's surface after the trim (discussed 2026-10-22, the shape proposed for D10)

Today the JAX evaluator has 61 method names and the NumPy one 30; they are four families, two
adapters and one convenience. After the trim, **four traced bodies**, all taking `params`:

```python
ev = plant.compile(backend="jax")            # or "numpy": the same four names, no jit tier

ev.f(x, u, t, params)                        # the law; u=None is the nominal input (the _ivp family goes)
ev.h(x, u, t, params)                        # the outputs and, for a diagram, the internal signals
ev.step(x, u, t, params, *, dt, solver="rk4", substeps=1)              # x_{k+1}: the one step model (SM-1), u held
ev.rollout(x0, u_seq, t0, dt, params, *, solver="rk4", substeps=1, hold="zoh")   # scan of step: (t, xs, signals); hold "foh" interpolates u_seq
ev.rollout_batch(x0s, u_seqs, dt=, params=family)                       # the one convenience: vmap(rollout)
```

| Today | After |
| --- | --- |
| `f`, `f_p`, `f_trace`, `f_trace_p`, `f_ivp*` | `f`; `u=None` nominal |
| `outputs*`, `compute_internal_signals*` | `h` |
| `rk4_step*`, `euler_step*`, `step*` (20 names) | `step` with `solver`, `substeps`; for a `StepSystem` evaluator `step` is the system's own map `step(x, u, k, params)` on the integer tick, no knobs |
| `rk4_integrate_zoh*`, `rk4_integrate_linear*` (`hold="foh"`), `*_integrate_ivp*`, `euler_integrate_*`, `integrate_zoh*`, `rollout`, `integrate_zoh_rollout` (22 names) | `rollout`, the one place `hold` lives |
| `f_scipy`, `f_ivp_scipy`, `as_scipy_jac` | not evaluator methods: the `Simulator` swaps the arguments of `frozen.f` for `solve_ivp`, in one place |
| `rollout_batch` | stays |

Jacobians stay in `analysis.derivatives` (`jax.jacobian` of `f`).

**Three access forms of one body**, the two derived ones built lazily and cached by one helper,
never hand-written:

| Form | What it is | Who calls it | Pendulum, one step |
| --- | --- | --- | --- |
| traced: `ev.step(x, u, t, params, dt=)` | the plain function; eager if called from Python | composed inside `scan`, `vmap`, `grad`: RL, batches, the compiled problem and its program, MPPI | 5 700 µs eager (never meant to be called so) |
| jitted: `ev.jit.step(x, u, t, params, dt=)` | `jax.jit(ev.step)`, compiled once, cached | one step at a time from Python with changing params (an estimator feeding `mu`, a sweep) | 14 µs |
| frozen: `ev.frozen.step(x, u, t, dt=)` | `jit(partial(ev.step, params=plant.params))`, bound at first use, cached, rebound when `plant.params` changes | the student's `discretize(plant, dt).step`, the realtime loop, the gym bridge, RRT | 10 µs (today's hand-written frozen method) |

Rebuilding the wrapper per call is the one thing to forbid: `jax.jit(body)` at every call costs
87 µs, `jit(partial(...))` at every call 93 000 µs (a new object, a cache miss, a recompile).
DESIGN §5's stable names (`rk4_step`, `integrate_zoh`, their trace twins) go, every caller moved
to `frozen.step` / `step` in the same change (R82), with a ledger line.

**The knobs are static, never traced.** `solver`, `substeps` and `hold` are keyword-only
Python values (strings, ints), resolved at trace time (`static_argnames` on the generated
`jit`), so a `hold` branch or a `substeps` loop leaves no branch in the compiled program and
each distinct combination compiles once. `dt` and `params` are the only array arguments. A knob
is never a pytree leaf (a runtime array choosing the integrator would need `lax.switch` and has
no meaning under `vmap`). The `solver="scipy"` door of `evaluate` and `compute_forced` swaps the
run function in Python before anything is traced; `problem.compile` has no such door.

**Batches: shared or per-member, any mix.** In `rollout_batch` a leaf with a leading axis of
length `B` is mapped and a leaf without one is shared: `x0s (B, n)` with one shared `u_seq`,
`u_seqs (B, N, m)`, a `params` family with one `m` per member and every other parameter
shared, or one `x0` with `B` parameter sets. `problem.realize(key, n)` returns exactly such a
triple (starts, a parameter family, seeds), which is what `evaluate` hands to the batch.

**The speed trade-off of a parameter family.** Shared parameters are compile-time constants:
XLA folds `g/l`, `m·l²`, `1/I` once and each step is a few fused multiply-adds on the `(B, n)`
state. A per-member parameter is a `(B,)` operand: nothing folds, the derived quantities are
recomputed at every step for every member, and each step reads one more operand per
parameter. On a small, memory-bound plant that is the whole cost (pendulum: 673 ms vs 127 ms,
5×); on a plant whose step is already compute-bound it is hidden (cart-pole: 1276 vs 1307 ms,
none). Results never change; it is the price of differentiating or sampling with respect to a
plant parameter at all. Two mitigations for the profiling row (S53), neither a design change:
hoist the plant's pre-computation block (the `m·l²` lines RULES 5.3 already separates) out of
the scan when parameters are batched, computing it once per member; or carry the drawn
parameters as constant columns of the batched state so they ride the same memory stream.

## 4. The consumers after the change

| Consumer | After | Numbers |
| --- | --- | --- |
| `discretize` | calls the compiled step; gains `substeps=`, `solver="exact"`; `integrator=` replaced by `solver=` at every caller (R82) | identical for RK4 and Euler (checked by the baseline) |
| `RolloutEnvironment` | takes the step of `discretize(plant, dt, ...)`; gains `substeps=`, `solver=` replaces `integrator=`; the RL episode is a training setting it owns, reading the problem's fields as hints (R43), and it no longer serves the Monte Carlo trials | identical at `substeps=1` |
| `Sys2Gym` | the same | identical |
| the DP grid | the step model with `solver="euler"` | identical; moving DP to RK4 is a separate ask |
| the RRT extender, the Lyapunov sampler, the MPPI scripts | `substeps` instead of hand loops | identical |
| `HybridSimulator`, `RealtimeSimulator` | the step model (`dt_inner` becomes `substeps`) | identical |
| shooting, multiple shooting | the rollout with `hold="foh"` | identical |
| direct collocation | unchanged | — |
| `compare()`, `MonteCarloEvaluator` | the compiled fixed-step engine is the judge (R15): `evaluate` and `compare` run `problem.compile(arg, dt)` (`dt` required, R36), the continuous loop on SM-1's step; `solver="scipy"` is the adaptive escape; the Monte Carlo fold into `evaluate` lands in v0.3, right after `evaluate` exists (R44) | change once with V1 (the integral to `tf`, R19) and with the fold (the draws, D21); a sampled replay on the training model waits for v0.4 |
| `compute_forced` (R17, R18, R40) | the one `ev.rollout`, `hold` on its sequence; `solver="rk4" \| "scipy"`; `input_interp` becomes `hold`; a callable `u(t)` evaluated at the RK4 stage times; `euler_fixedsteps` on the same rollout, `euler` kept for discontinuous systems and non-uniform grids (R83) | a sampled input matrix held `zoh` by default: those `rk4_fixedsteps` forced numbers change once (a baseline and a ledger line); a callable `u(t)` becomes exact at the stages |
| S31's sampled kernel | scans the step model with the controller's `step` between samples | lands on this |
| identification, behavior cloning (planned) | roll the step model at the logging rate | born on the standard |

## 5. The steps, bottom-up (one commit each; byte-identical is the bar)

What each step builds; the order and each step's "done when" are the S75 rows of
[TODO.md](TODO.md) §4. The AGENTS recipe for behaviour-preserving refactors applies to every step: read the tests
first, capture a seeded baseline of every consumer to JSON, run it twice and `cmp`, and
compare after each step. A number that changes on purpose gets its own test and its own ask.

- **SM-0** The baseline: every consumer of §1 on a pendulum and a cart-pole, both backends
  where they exist, `params` on and off, repeated calls.
- **SM-1** The one compiled step, taking `params`, with its knobs (`dt`, `solver`, `substeps`)
  and the rollout with `hold` on both backends; the
  existing methods become bindings over it, the parameterless ones a `partial` over `params`
  then `jit`, byte-identical and within the measured speed of §3.5.
- **SM-2** `discretize` on the compiled step, gaining `substeps=` (no `hold`: R40);
  `StepSystem.step(x, u, k, params)` on the tick, `t_k = t0 + k·dt` bound by `discretize`;
  `integrator=` replaced by `solver=` at every caller (R42, R82); `solver="exact"` (the follow-up) checked
  against `scipy.signal.cont2discrete`; the `params["dt"]` fallback and the source-port
  follow-ups (TODO §4).
- **SM-3** `RolloutEnvironment` and `Sys2Gym` on `discretize`, `substeps` exposed, `solver=`
  replacing `integrator=` (also on `ReinforcementLearningPlanner` and `TabularLearningPlanner`,
  in one sweep, R82).
- **SM-4** The RRT extender, the Lyapunov sampler, the MPPI scripts, the hybrid and realtime
  simulators on the step model.
- **SM-5** Shooting on the rollout's `hold="foh"`; the DP grid on `solver="euler"` (byte-identical;
  `DynamicProgrammingPlanner(solver="rk4")` available, R8, R42).
- **SM-7** `compute_forced` on the one rollout (R17, R18, R40): `solver="rk4" | "scipy"`,
  `input_interp` replaced by `hold`; a callable `u(t)` evaluated at the RK4 stage times; a
  sampled input matrix held `zoh` by default; the `euler` loop backend removed; the one
  intended change, the `rk4_fixedsteps` forced numbers (held matrices; exact callables), gets a
  baseline, its own test and a ledger line. Lands after SM-2 and before SM-6 (the number keeps its id; it was
  added on 2026-10-23).
- **SM-6** The grid trim (one `_p` body per integrator, the bound and jitted variants
  generated; the evaluator internals deduplicated: ZOH sugar, the JAX step rollout, the four
  Jacobian probes, the undeclared batch cache), then the integrators' move to
  `simulation/solvers/`. The one home of both (S37 absorbed, 2026-10-22).

## 6. The asks, ruled

1. **The contract and its name**: the step model, produced by `discretize`, a `StepSystem`
   (R6).
2. **The knobs and their defaults**: `dt` required; `solver` `rk4` / `euler` / `exact`,
   default `rk4` (R42, the word replacing R7's `method`); `substeps=1`; `substeps` exposed on
   every consumer that takes `dt` (R7). The hold is a keyword of the sequences only, `rollout`
   and `compute_forced`, `zoh` / `foh`, default `zoh` for a sampled matrix, a callable `u(t)`
   exact at the stages (R40, which supersedes R7's hold on the step and R18 for callables).
3. **DP's integrator**: Euler through the step model, byte-identical; RK4 available; whether
   the default flips is V3's question (R8).
4. **A sampled replay** in `compare()` and the Monte Carlo evaluator: superseded by R15 (the
   compiled continuous loop is the judge); a sampled replay waits for v0.4, the one item still
   open here.
5. **The order**: this first, beside wave K; V1's step 3 builds on SM-1; S31 after the
   continuous core (R13, the maintainer's order of 2026-10-22).
6. **Params always an argument**: one traced body, the jitted and frozen forms generated and
   cached once (R12).

## 7. Fit with the other plans

- **S31**: the sampled kernel is "scan the step model, call the controller between samples";
  this plan is its plant half.
- **S37**: absorbed; SM-1 is its "one scannable step", defined by its consumers before the
  files move, and SM-6 its move.
- **V1** ([differentiable-problem.md](differentiable-problem.md), rewritten to
  [continuous-core.md](continuous-core.md)): `problem.compile(arg, dt)` integrates the whole
  continuous closed loop on SM-1's compiled step, the controller's state inside `x`; it is the
  judge and the engine (R15), and it exports the program (`pe.program(wrt)` for today's
  `Optimizer`, R23). In this phase it takes continuous controllers only; its step 7b (after
  S31) dispatches a sampled controller to the kernel of §8, so a digital PID or a held neural
  policy gets the same `evaluate` and program export through that kernel.
- **S65**: the hold rule here (`zoh` / `foh`, on sequences, R40) replaces its `input_interp`
  option (SM-7, R17); one vocabulary for both.
- **The Later row on a sample stride**: `substeps` on the step model is the same knob for the
  sampled path; `Simulator`'s fixed-step solvers can read it too.

## 8. The sampled loop (S31)

The plant half above makes one step model; the loop half runs a sampled controller on it.

**Ruled 2026-10-22 (R13): the loop is a tool, not a `System`; the name `HybridDiagram` stays for
now. Ruled the same day (R14): ticks** (`step(x, u, k, params)`; the loop converts to seconds
where it calls the plant). The rest (D13–D15 of [end-state-api.md](end-state-api.md)) is parked
to the hybrid phase, after the continuous core.

- **Two model algebras, both closed.** The continuous one (`System`, diagrams, `@`, `>>`,
  `+`) and the discrete-time one (`StepSystem`, `StepDiagramSystem`). The loop that joins the
  two clocks is neither: an object with `compute_trajectory` and `compile`, built by `%`, and
  not a `System`. Nesting a sampled loop in a diagram has no consumer, `linearize` and the
  frequency tools have no meaning on it, and multi-clock scheduling as a framework feature is
  out of scope (ROADMAP §7).
- **One sampled rollout kernel, two faces.** The controller held over its period, the plant
  stepped by the step model between samples, a `StepSchedule` for multi-rate; a NumPy face
  and a JAX face, as `Simulator` and `compile` have. Its two copies today
  (`HybridSimulator`, the MPC computer export) become its callers; the RL environment is a
  consumer of the step model only, the learner calling the policy itself (ruled 2026-10-22).
- **Two loops on purpose.** The compiled problem (`problem.compile(arg, dt)`, R15) keeps the
  continuous closed loop as a diagram, for evaluation and the program export of a continuous
  law; this kernel is the loop of every sampled controller. A sampled controller optimized by
  gradient (a digital PID, a held neural policy) goes through this kernel, once V1's step 7b
  dispatches it there (D15, the hybrid phase).
- **The kernel never breaks tracing; the computer decides.** A gain, a neural policy or a
  digital PID traces end to end (the RL planner, gain tuning, a digital PID tuned by
  gradient). MPC with SciPy or Ipopt inside runs on the NumPy face. A traceable inner solver
  later makes MPC a differentiable policy too, so the MPC block's `step` stays a pure
  function of arrays with the solver inside.
- **Parked to the hybrid phase (D13–D15):** whether the run-state caches
  (`StepSystem.rollout`, `HybridDiagram.last_result`) join `self.traj` as allowed caches (RULES 2.4); the `core` → `simulation` import; whether
  the tick carries controller memory (MPPI's warm start, a diffusion policy's action chunk;
  random draws come from the controller's seed and the tick, as the noise block's;
  [future-policies.md](future-policies.md)); how the compiled problem dispatches a sampled
  controller to this kernel (V1's step 7b).

**Order:** after the continuous core (S75, then V1's steps 1–6), with P10's AC-8b; in v0.3 if
the calendar allows, else v0.4 (the slot is raised in ROADMAP §6); the
hand-copied `HybridDiagram` facades and the Mermaid twin of the hybrid drawing go with it;
the hybrid topology drawing stays for teaching.
