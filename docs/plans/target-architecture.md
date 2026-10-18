# The target architecture: one traceable path from any parameter to any metric

Status: **ideas to explore and adjust** (2026-10-18), consolidated from one morning's
discussion with the maintainer; nothing here is decided unless it says so. What is decided
lives elsewhere: the compiled evaluator and its steps in
[differentiable-cost.md](differentiable-cost.md), the cost parameters in
[cost-params.md](cost-params.md), the fields in [fields.md](fields.md), the open steps on
the workboard ([TODO.md](TODO.md) §4). The counts behind the verdict are in
[the core-objects review](../reviews/2026-10-18-core-objects-review.md).
Rung: v0.3 and after. Lane: research first.

## 1. The target, in one sentence and one table

**Any performance metric that is a traceable function of a run or of a linearization can be
optimized through the native `Optimizer`, with respect to any parameter set the problem
holds.** The library provides the metric as a verb, JAX provides the gradient, the optimizer
provides the descent loop; no demo writes a scan, a cost formula or a loop of its own.

The parameter sets are the columns; the metrics are the rows; each cell says where that pair
stands: traced today by hand in a demo, the step of the evaluator plan that makes it one
verb, the workboard row or the section below that proposes it, `—` where the pair has no
meaning.

| Metric (a verb on the compiled problem or the linearization) | controller params | open-loop input sequence | plant params | cost, set, disturbance params | `x0` |
| --- | --- | --- | --- | --- | --- |
| Trajectory cost `J` of one run | hand scan today → step 3 | step 3 with the open-loop `compile()` (§4.6) | hand scan today → step 3 | A2 (cost), A5 (sets) → step 3 | step 3 |
| Whole trajectory, final state | step 2 | step 3 | step 2 | step 3 | step 2 |
| Mean and worst cost over the problem's draws | step 4 | step 4 | step 4 | step 4; disturbance draws per the plan's ask 6 | step 4 |
| Failure rate, its smooth stand-in `feasibility` | step 4 (the plan's ask 5) | step 4 | step 4 | step 4 | step 4 |
| Closed-loop poles, damping ratios, natural frequencies | §4.7 (S73) | — | §4.7 (S73) | — | — (linearized at `x̄`) |
| Gain and phase margins, Bode magnitude and phase at a frequency, the sensitivity peaks | §4.7 (S73) | — | §4.7 (S73) | — | — |

The consumer of every cell is the same: the program bridge builds a `MathematicalProgram`
whose objective is one metric and whose constraints `g ≥ 0` can be other metrics (maximize
the phase margin over the PID gains subject to a gain margin above 6 dB and the gains in a
box), and `Optimizer` solves it with a method the program's class admits (§4.8). The worked
examples the table implies, each one demo once it lands:

- gain tuning on the closed-loop cost (`pid_autotuning_jax`, today's hand loop);
- gain tuning on a stability margin (§4.7 plus the bridge);
- an open-loop input sequence by single shooting (`cartpole_rollout_gradients`, today's
  hand loop);
- robust tuning on the mean cost over a plant-parameter family;
- plant-parameter sensitivity of poles and margins (the Buckingham-π question in a linear
  setting).

## 2. The pipeline: the nouns are right, the verbs are duplicated

![The planned pipeline](differentiable-cost-pipeline.png)

*The statement (green) holds the system, the cost, the sets and the distributions with one
params pytree at its root. The traced path (purple) is one compiled object per problem and
controller: one integrator kernel that emits the internal signals, a native `Trajectory`
pytree, one scoring kernel, an `Evaluation` of scalars. The NumPy consumers (grey) sit below
the kernels and call them; none keeps a twin. Sources: `differentiable-cost-pipeline.svg`
(the PNG) and `differentiable-cost-pipeline.mmd` (the same figure in Mermaid, rendered by GitHub).*

The review counted the state of each noun. Every law a user writes (`f`, `h`, `g`, `margin`)
exists once and traces through the array-module idiom; the duplication is one layer up, in
the code that runs the laws: integrating, scoring, cutting at exit, drawing samples and
reconstructing signals each exist in two to five copies, about half NumPy-only. So the
target needs no new noun beyond the compiled evaluator and its `Evaluation`; it needs one
traced kernel per verb with the NumPy consumers calling it. Two nouns become surplus
(`MonteCarloEvaluator` absorbed, `StepRollout` folded into `Trajectory`), none else appears
or disappears.

## 3. Every workflow on the same objects

![Ten workflows as lanes](differentiable-cost-workflows.png)

*One lane per workflow, left to right: the source object, the intermediate representations,
the result, the final consumer. Purple is traced and array in / array out today, green
becomes traced when the evaluator plan lands, grey is NumPy by contract (reporting, solver
loops, random search). Source: `differentiable-cost-workflows.svg`.*

What the lanes showed, beyond the single pipeline:

- the middle column is a family of solver-ready forms (§4.4);
- `Trajectory` is the result of six of the ten lanes, so its native form matters more than
  one box suggests, and the scorer should take any of them (§4.5);
- the open-loop form already exists inside the closed-loop one (§4.6);
- the gain-tuning lane ends in a hand-written step while trajectory optimization ends in the
  `Optimizer`; after the bridge they are the same consumer (§4.8);
- two lanes stay honestly grey in the middle: model predictive control (no implicit gradient
  of the inner solve) and the sampling planner (random search). The target does not promise
  to trace either.

## 4. The ideas, one by one

Each entry: what it is, why, what it changes, what it risks, where it sits. "Decided" marks
the two that already are.

### 4.1 Read the parameters work and the kernels as one job

Ten cost methods, twelve set methods and six shape methods take `params` and none reads it.
Making them read it (cost-params.md, the sets in the later-nouns row) is the first half of
the differentiable problem; the kernels are the second half. Schedule them back to back and
let the kernels' tests be the "done when" of the parameters work. Changes the order of the
workboard's v0.3 section only.

### 4.2 One kernel per verb; the NumPy consumers call it

Five scoring rules with four quadratures (the cost's NumPy evaluator, the planning scorer,
the transcriptions without discount, the Monte Carlo evaluator in JAX, the RL per-step
reward) become one traced `score_rollout`, called by the NumPy consumers with NumPy arrays.
The RL reward stays the documented exception, derived from the same `g`. Two signal
reconstructions become one (`rollout_p` emits the signals). `Set.contains` is derived from
`margin ≥ 0`. The quadrature (trapezoid, `h` at the final sample, discount `e^{−ρt}`, cut at
the first exit) is stated once in DESIGN and cited by the transcriptions. Risk: the
transcriptions' discount is a behaviour change on every discounted problem's plan (the
plan's ask 4). Sits in the evaluator plan, steps 1–3.

### 4.3 A native `Trajectory` (decided 2026-10-18)

The object keeps the arrays it is given through the array module, no `float` cast, no copy
of a traced value, registered as a JAX pytree; the reporting verbs stay NumPy by contract.
Step 2b of the evaluator plan. The follow-on, not decided: `StepRollout` becomes a
constructor of the same noun with `t = k·dt` (a November consolidation candidate).

### 4.4 The verb family on the problem

Every lane first turns the problem into a solver-ready form: the transcription (trajectory
optimization), the grid (dynamic programming), the environment (reinforcement learning), the
linear model (analysis), and now the compiled loop. Each was designed on its own, with its own
constructor and its own copy of the scoring rule. The idea: a small set of verbs on the
problem, each returning one frozen traced form (`compile`, `transcribe`, a grid verb,
`environment`), named together in the brainstorm of the evaluator plan's step 0 rather than
naming the new member alone. Changes names and the entry points, not the math. Risk: a
rename sweep across demos and notebooks; keep the old constructors one release.

### 4.5 `Evaluation` as a verb of the problem

Six lanes end in a `Trajectory`: the reference simulator, the MPC loop, the RRT tree and the
trajectory optimizer as much as the compiled rollout. The scoring kernel should be reachable
as a verb on the problem that takes any trajectory and returns the same `Evaluation`, the
compiled evaluator calling it on its own traced rollout. One scorer for every lane, and
`PlanningSolution.evaluation` reproducible for every planner. Changes where the kernel is
exposed, not what it computes. Sits with 4.2.

### 4.6 The open-loop `compile()`

`rollout_p` takes an input sequence, so compiling the problem without a controller is the
open-loop evaluator. Two lanes then collapse onto it: single-shooting trajectory optimization
is the program bridge with the inputs as decision leaves, and the RL environment is the same
object exposed one step at a time (its reward is one step of the scoring rule). Collocation
stays its own transcription: it decides the states and adds defect constraints. If accepted,
`problem.compile()` with no controller is a valid case from the first prototype, not a
closed-loop-only design opened later. Caution: merging the RL environment touches the
collectors, which carry a bit-identical baseline; that merge comes after the kernels exist
and are tested.

### 4.7 Traceable linear analysis (workboard row S73)

`poles`, `frequency_response` and `margins` through the array module, so the sensitivity of
closed-loop poles, damping ratios, Bode values and margins to any plant or controller param is
`jax.grad` of the existing verbs on the JAX jacobian. The math lines stay the textbook ones:
`C (jωI − A)⁻¹ B + D` as one batched solve instead of the Python loop; eigenvalues through
the array module, whose derivative is the classical `vᴴ (∂A/∂p) u / (vᴴ u)` sensitivity;
Brent's method on the crossovers replaced by a fixed number of Newton steps from the grid
bracket, so the crossover frequency carries the implicit-function derivative. Zeros stay NumPy
(a generalized eigenproblem JAX lacks); root-locus branch matching stays NumPy (plotting); the
root locus generalizes to any parameter. Non-differentiable spots to document: repeated poles,
the switch of the minimum margin between crossovers; the nonsymmetric eigensolver is
CPU-only. Independent of the evaluator (needs only the jacobian); self-contained for a cloud
agent. The done-when is on the workboard row.

### 4.8 The optimizer as the one descent loop (workboard row S74)

Today `Optimizer` offers three constrained-NLP presets on a flat `z`, and every gradient-based
tuning writes its own loop: four compile demos, three notebooks, while the RL algorithms share
one pytree `Adam` in the RL package. Three parts: (1) the program knows its class from its
fields (`unconstrained`, `bounded`, `constrained`, plus a `stochastic` flag the bridge sets
when the objective resamples keys); `Optimizer` refuses a method the class does not admit and
`method=None` picks by class (quasi-Newton for a smooth unconstrained or bounded program,
SLSQP otherwise, Adam when stochastic). (2) One first-order backend, gradient descent and
Adam on the flat `z`, the box by projection, the global clip; the RL package imports that Adam.
Two SciPy presets (`scipy_lbfgsb`, `scipy_bfgs`) come free. (3) Pytree decisions stay outside
`Optimizer`: the bridge ravels the chosen leaves to `z` and unravels the result. Then the demos
and notebooks end in one optimizer call. For the maintainer: whether one tutorial keeps a
hand-written loop on purpose, as the GRO501 from-scratch notebooks do. The done-when is on the
workboard row.

### 4.9 Metrics as constraints in the bridge

The bridge of the evaluator plan's step 5 takes one metric as `J`; the target of §1 wants
other metrics as `g ≥ 0` (a margin floor while tuning on cost). Design the bridge's signature
for that from the start, `program(objective=, constraints=(), wrt=)`, or keep `J` only in the
prototype (the plan's ask 9).

### 4.10 Consolidation rows the review found

Not part of the differentiable path, but they remove the code it would otherwise sit beside:

- the JAX evaluator's method grid (five families times up to six suffixes, 36 integrator
  methods, 1763 lines): keep the `_p` bodies and generate the bound and jitted variants;
- `StepRollout` folded into `Trajectory` once it is a pytree (4.3);
- the planned core `Field` absorbs the six field-like classes in `planning/spatial`, and a
  grid field with a JAX interpolation makes the lookup-table policy traceable;
- a test pinning that the simulator's fixed-step RK4 and the compiled rollout agree to
  round-off on one diagram;
- the compiled evaluator refuses the MPC controller (no implicit gradient) with a message
  saying so, as the Monte Carlo evaluator refuses stateful controllers today.

## 5. How this fits the existing plans

| Document | Owns |
| --- | --- |
| [differentiable-cost.md](differentiable-cost.md) | the compiled evaluator: its design (§3), the verified facts, steps 0–6 with their done-whens, the asks |
| [differentiable-cost-options.md](differentiable-cost-options.md) | the architectures and naming systems weighed for the evaluator; feeds the brainstorm of step 0 |
| [cost-params.md](cost-params.md) | every cost reads `params`, through to the program's objective (4.1's first half) |
| [fields.md](fields.md) | the `Field` object; gains the "absorbs `planning/spatial`" done-when if 4.10 is accepted |
| [optimizer-parametric-wiring.md](optimizer-parametric-wiring.md) | the shared optimizer backend factory for parametric MPC, which 4.8's backend joins |
| [TODO.md](TODO.md) §4 | the rows: V1 (the evaluator), A2, A5, S73 (4.7), S74 (4.8), the November consolidation rows |
| [the core-objects review](../reviews/2026-10-18-core-objects-review.md) | the dated record: the per-noun table and the counts |

## 6. An order, if the ideas are accepted (not scheduled)

1. The evaluator plan's steps 0–3 (names, kernel, trajectory, compile) with 4.5 and 4.6
   folded into step 0 so the names and the no-controller case are settled before code.
2. The cost parameters (A2), then the evaluator's steps 4–6, with the bridge signature of 4.9
   decided at step 5.
3. 4.7 (traceable analysis) any time, beside the above; 4.8 (the optimizer) at or after the
   bridge, where its first customers appear.
4. The consolidation rows of 4.10 in the November pass, the RL-environment merge of 4.6 last.

## 7. What needs the maintainer

- The names (the brainstorm of step 0), now including the verb family of 4.4.
- Which of 4.4–4.10 are accepted, folded into a plan, or dropped; the workboard rows S73 and
  S74 wait on that.
- Whether the one sentence of §1 goes into the constitution's vision text.
- Whether one tutorial keeps a hand-written gradient loop on purpose (4.8).
