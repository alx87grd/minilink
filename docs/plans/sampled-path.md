# The sampled path: the step model and the sampled loop

**Status:** ideas to review, 2026-10-21 — an evaluation of the code and a recommendation;
nothing decided.  
**Lane:** core (the compiled evaluator, `discretize`, every sampled consumer) — every step is
**[ask — core]**.  
**Rung:** v0.3, wave S of [TODO.md](TODO.md): step S75 (the step model, §1–§7), then S31
(the sampled loop, §8); both open decisions in ROADMAP §6, both ruled before the
differentiable problem's step 0. Their code runs beside V1's steps 0–2 and lands before its
step 3, where the evaluator meets the sampled seam (v0.3, wave V).  
**Gathers:** S31, S37's one scannable step, the `discretize` follow-ups, S65's input-hold
item, the Later row on a sample stride, and the K-wave evaluator-grid trim.

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
| S37 (v0.9) | evaluators keep pure maps and one scannable step; integrators move to `simulation/solvers/`; first dedupe the zero-order-hold sugar and the JAX step rollout | the implementation home |
| the `discretize` follow-ups (TODO v0.9) | an exact zero-order hold (the matrix exponential) shared with `step_response`; the `params["dt"]` fallback; the ports kept; `PID % dt` needs a discretization method | the public verb |
| S65 (wave D) | the forced-input hold as one `input_interp` option every solver obeys | the hold rule |
| Later (TODO §7) | a sample stride on the fixed-step solvers (`substeps=k`) | the substep knob |
| the K-wave row | the evaluator grid trimmed to `_p` bodies plus generated bindings | the trim this enables |
| V1 ([differentiable-problem.md](differentiable-problem.md)) | the continuous closed loop as a diagram (RD-D24); `rollout_batch` with a params family (S53) | stays continuous; reads the same compiled RK4 |

The pieces are planned; the contract that ties them is not.

## 3. The proposal

### 3.1 One contract: the step model

$$x_{k+1} = \Phi(x_k, u_k, t_k; p)$$

the plant integrated over one sample period with the input held: pure in arrays, traceable
in every argument (`params` included), the same on NumPy and JAX. Four knobs, no more:

| Knob | Values | Default |
| --- | --- | --- |
| `dt` | the sample period, owned by the step model and never a key of `params` (as `discretize` already rules) | — |
| `method` | `"rk4"`, `"euler"`, `"exact"` (the matrix exponential, linear models only) | `"rk4"` |
| `substeps` | integration steps per sample | `1` (today's numbers) |
| `hold` | `"zoh"`, `"foh"` (what shooting uses between knots) | `"zoh"` |

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

`discretize(plant, dt, *, method, substeps, hold) -> StepSystem` becomes the standard path,
as it already is in the GRO501 digital-implementation lesson. The result is a `StepSystem`:
it composes in a `StepDiagramSystem`, runs `compute_rollout`, and its `compile()` gives the
scannable step. Its body stops being hand-written and calls the compiled step.

### 3.4 One implementation

In `core/compile`, one step per backend, `step(x, u, t, params; dt, method, substeps, hold)`,
an `xp` body where possible; the 36-method grid becomes bindings over it (the K-wave trim),
and `integrate_zoh(dt_inner=)` is `substeps` under another name. S37 then moves the
integrator to `simulation/solvers/` as planned.

### 3.5 Why one params path

Measured on the current JAX evaluator, the frozen RK4 step against the one taking `params`
(identical results in every case):

| Case | Frozen | `params` as argument |
| --- | --- | --- |
| pendulum, 1000 rollouts × 1000 steps in one compiled scan | 156 ms | 170 ms |
| cart-pole, the same batch | 1503 ms | 1514 ms |
| pendulum, one jitted step called from Python | 13 µs | 34 µs |
| cart-pole, one jitted step called from Python | 24 µs | 54 µs |

- **Inside a compiled loop there is no real difference** (1 to 9 %): RL training, batch
  rollouts, trajectory optimization, MPPI and the differentiable tuning all run there.
- **Step by step from Python, the argument costs about 2.5×**, all of it the per-call
  flattening of the parameter dictionary, not arithmetic. The binding of §3.1 removes it: the
  gym bridge, the realtime loop, the RRT extender and the hybrid ticks get the frozen speed
  from the one body.
- **The grid halves.** Every integrator exists frozen and with `params` today, before the
  trace and jit tiers double it again; one `_p` body plus a binding is the trim the K-wave
  row asks for.
- **A bug class goes.** A frozen evaluator silently ignores parameters changed after
  `compile()`; with one path, freezing is an explicit act at a visible place.
- **The stable names hold.** DESIGN §5's frozen evaluator subset (`f`, `rk4_step`,
  `integrate_zoh`, their trace twins) keeps every name; each parameterless one becomes the
  binding over its `_p` body.

It does not fix S53: a batch with a *different* parameter set per rollout (a family under
`vmap`) runs about 10× slower than the plain batch, a separate problem.

## 4. The consumers after the change

| Consumer | After | Numbers |
| --- | --- | --- |
| `discretize` | calls the compiled step; gains `substeps=`, `hold=`, `method="exact"` | identical for RK4 and Euler (checked by the baseline) |
| `RolloutEnvironment` | takes the step of `discretize(plant, dt, ...)`; gains `substeps=` | identical at `substeps=1` |
| `Sys2Gym` | the same | identical |
| the DP grid | the step model with `method="euler"` | identical; moving DP to RK4 is a separate ask |
| the RRT extender, the Lyapunov sampler, the MPPI scripts | `substeps` instead of hand loops | identical |
| `HybridSimulator`, `RealtimeSimulator` | the step model (`dt_inner` becomes `substeps`) | identical |
| shooting, multiple shooting | the step model with `hold="foh"` | identical |
| direct collocation | unchanged | — |
| `compare()`, `MonteCarloEvaluator` | unchanged (the continuous replay, RD-D24); optionally a sampled replay on the training model | a separate ask |
| S31's sampled kernel | scans the step model with the controller's `step` between samples | lands on this |
| identification, behavior cloning (planned) | roll the step model at the logging rate | born on the standard |

## 5. The steps, bottom-up (one commit each; byte-identical is the bar)

The AGENTS recipe for behaviour-preserving refactors applies to every step: read the tests
first, capture a seeded baseline of every consumer to JSON, run it twice and `cmp`, and
compare after each step. A number that changes on purpose gets its own test and its own ask.

- **SM-0** The baseline: every consumer of §1 on a pendulum and a cart-pole, both backends
  where they exist, `params` on and off, repeated calls.
- **SM-1** The one compiled step, taking `params`, with its four knobs on both backends; the
  existing methods become bindings over it, the parameterless ones a `partial` over `params`
  then `jit`, byte-identical and within the measured speed of §3.5.
- **SM-2** `discretize` on the compiled step, gaining `substeps=` and `hold=`;
  `method="exact"` (the follow-up) checked against `scipy.signal.cont2discrete`.
- **SM-3** `RolloutEnvironment` and `Sys2Gym` on `discretize`, `substeps` exposed.
- **SM-4** The RRT extender, the Lyapunov sampler, the MPPI scripts, the hybrid and realtime
  simulators on the step model.
- **SM-5** Shooting on `hold="foh"`; the DP grid on `method="euler"`.
- **SM-6** The grid trim (the K-wave row), then S37's move of the integrators to
  `simulation/solvers/`.

## 6. The asks

1. **The contract and its name**: the step model, produced by `discretize`, a `StepSystem`.
2. **The four knobs and their defaults** (`rk4`, `substeps=1`, `zoh`), and whether the RL
   planner and the gym bridge expose `substeps`.
3. **DP's integrator**: keep Euler (byte-identical) or move to RK4 (every value-iteration
   lesson's numbers change once).
4. **A sampled replay** in `compare()` and `MonteCarloEvaluator`, so a law is scored on the
   model it was trained on; as an option, or as the default (the comparison lessons' numbers
   change).
5. **The order**: this first, then S31's kernel, then V1's step 0.
6. **Params always an argument**: one `_p` body per method, the frozen call a binding at the
   boundary (§3.5); the parameterless call kept as the student convenience, under its
   current stable name.

## 7. Fit with the other plans

- **S31**: the sampled kernel is "scan the step model, call the controller between samples";
  this plan is its plant half.
- **S37**: this plan is S37's "one scannable step", defined by its consumers before the files
  move.
- **V1** ([differentiable-problem.md](differentiable-problem.md)): its evaluator keeps the
  continuous closed loop (RD-D24); a sampled controller tuned by gradient goes through the
  step model and S31's kernel.
- **S65**: the hold rule here (`zoh` / `foh`) is the sampled side of its `input_interp`
  option; one vocabulary for both.
- **The Later row on a sample stride**: `substeps` on the step model is the same knob for the
  sampled path; `Simulator`'s fixed-step solvers can read it too.

## 8. The sampled loop (S31)

The plant half above makes one step model; the loop half runs a sampled controller on it.
The open decision (ROADMAP §6): `HybridDiagram` as a `System` (a continuous and a
piecewise-constant state, periodic update events, a simulator and a compiler that know
sample times), or an honest `HybridLoop`.

**Recommended (2026-10-20): the loop is a tool, not a diagram.**

- **Two model algebras, both closed.** The continuous one (`System`, diagrams, `@`, `>>`,
  `+`) and the discrete-time one (`StepSystem`, `StepDiagramSystem`). The loop that joins the
  two clocks is neither: an object with `compute_trajectory` and `compile`, built by `%`, and
  not a `System`. Nesting a sampled loop in a diagram has no consumer, `linearize` and the
  frequency tools have no meaning on it, and multi-clock scheduling as a framework feature is
  out of scope (ROADMAP §7).
- **One sampled rollout kernel, two faces.** The controller held over its period, the plant
  stepped by the step model between samples, a `StepSchedule` for multi-rate; a NumPy face
  and a JAX face, as `Simulator` and `compile` have. Its three copies today
  (`RolloutEnvironment`, `HybridSimulator`, the MPC computer export) become its callers.
- **Two loops on purpose.** The differentiable problem's evaluator keeps the continuous
  closed loop as a diagram (RD-D24), for analysis and gradient tuning of a continuous law;
  this kernel is the loop of every sampled controller. A sampled controller tuned by
  gradient (a digital PID, a held neural policy) goes through this kernel.
- **The kernel never breaks tracing; the computer decides.** A gain, a neural policy or a
  digital PID traces end to end (the RL planner, gain tuning, a digital PID tuned by
  gradient). MPC with SciPy or Ipopt inside runs on the NumPy face. A traceable inner solver
  later makes MPC a differentiable policy too, so the MPC block's `step` stays a pure
  function of arrays with the solver inside.
- **Still to rule with it:** the sampled seam's time argument, integer ticks (today) or
  seconds; whether the run-state caches (`StepSystem.rollout`, `HybridDiagram.last_result`)
  join `self.traj` as allowed caches (RULES 2.4); the `core` → `simulation` import; whether
  the tick carries controller memory (MPPI's warm start, a diffusion policy's action chunk;
  random draws come from the controller's seed and the tick, as the noise block's;
  [future-policies.md](future-policies.md)).

**Order:** after S75 in wave S (the kernel scans the step model), with P10's AC-8b; the
hand-copied `HybridDiagram` facades and the Mermaid twin of the hybrid drawing go with it;
the hybrid topology drawing stays for teaching.
