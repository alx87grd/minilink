# The step model: one path from a continuous plant to a step map

**Status:** ideas to review, 2026-10-21 — an evaluation of the code and a recommendation;
nothing decided.  
**Lane:** core (the compiled evaluator, `discretize`, every sampled consumer) — every step is
**[ask — core]**.  
**Rung:** step S75 of [TODO.md](TODO.md), an open decision in ROADMAP §6; proposed before
the sampled-loop kernel (S31) and before the differentiable problem's step 0 (V1).  
**Gathers:** S31, S37, the `discretize` follow-ups, S65's input-hold item, the Later row on
a sample stride, and the K-wave evaluator-grid trim.

Many features turn a continuous plant into a step map: reinforcement learning, dynamic
programming, the gym bridge, trajectory optimization, the hybrid and realtime loops, RRT,
the Lyapunov sampler, MPPI, and soon identification and behavior cloning. Each does it its
own way today. This plan proposes one contract, one public verb and one implementation for
that conversion.

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
| Later (TODO §6) | a sample stride on the fixed-step solvers (`substeps=k`) | the substep knob |
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
- **SM-1** The one compiled step with its four knobs on both backends; the existing methods
  become bindings over it.
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
