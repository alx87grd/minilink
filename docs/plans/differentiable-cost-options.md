# The differentiable planning problem: architecture and naming options

Status: an analysis for the maintainer (2026-10-17), feeding step 0 of
[differentiable-cost.md](differentiable-cost.md). Six architectures and four naming systems
are laid out and criticized; §5 converges on three candidate packages and a recommendation.
Deleted once the ruling is recorded.

## 1. What the thing must do, and what it must respect

Requirements, from the design conversation:

- **R1** one compiled object per `(problem, controller)`, so the loop is built and compiled once;
- **R2** pure functions of one params pytree (`system` nested as the diagram's, `cost`, `sets`),
  so `jit`, `grad` and `vmap` all reach every parameter;
- **R3** the outputs a user wants a gradient of: the whole trajectory (the plant's `x` and `u`,
  `y`), the end state, the scalar cost, its mean over the problem's draws, and a feasibility
  number beside the hard success rate;
- **R4** the draws are the `StochasticPlanningProblem`'s, not a second mechanism;
- **R5** a way to pick parameters as decision variables and get a `MathematicalProgram` the
  `Optimizer` solves;
- **R6** a NumPy twin for laws that do not trace and for the Basic tier, as the evaluators have;
- **R7** controllers with state (a PID with an integrator, an observer) from the first commit;
- **R8** the Monte Carlo evaluator, the RL environment and the transcriptions agree on one
  scoring rule.

Constraints from the law: tools are verbs on a `System` and return domain objects or arrays
(CONSTITUTION §4.2, §4.6, no DTO layers); the continuous core is primary; one signature per
object; names by role (RULES 2.1), one verb family per role (RULES 5.25); the existing pairs
`compile` → `Evaluator`, `solve` → `Solution`, `evaluate` → `Evaluation`, `compute_trajectory`
→ `Trajectory`.

What the material already says (files mentioning the word, tutorials, teaching and demos):
`compile` 35, `simulation` 31, `program` 24, `rollout` 23, `closed loop` 23, `evaluate` 21,
`simulate` 20, `margin` 13, `terminal` 12, `cost-to-go` 11, `feasib*` 9, `score` 9,
`evaluation` 7, `Monte Carlo` 4, `failure rate` 3, `final state` 2, `expected cost` 1,
`success rate` 0, `average cost` 0.

## 2. Architectures

### A. The compiled problem (the plan's §3)

`problem.compile(controller, dt=, backend=)` → an evaluator object; verbs for the trajectory,
the end state, the cost, the mean, the feasibility; `program(...)` as the bridge; absorbs
`MonteCarloEvaluator`.

- For: one object per R1; the problem owns the cost, the sets, the horizon and the draws, so
  the object has everything (R3, R4); mirrors `System.compile` → `DynamicsEvaluator`, a pair
  students already know; the NumPy twin is one `backend=` away (R6); the loop is a diagram so
  controller state is inside `x` (R7).
- Against: a new noun on the surface; a method on a frozen dataclass that returns a stateful
  evaluator (acceptable: `System.compile` does the same on a mutable one); the object is
  "the loop" and "the scorer" at once.

### B. The loop is the object, the cost is a block

No new object: `loop = controller @ plant`; a `CostIntegrator(cost)` block with state `J`,
`dJ/dt = e^{−ρt} g(x, u, t)`, wired into the diagram; `loop.compile("jax")` then gives
`jax.grad` of `J` through the existing evaluator; the draws are `rollout_batch` over `x0s` and
a params family. Prototyped in cost-params.md ("the cost as a block", `J` within 0.02 % of the
post-hoc score).

- For: nothing new to learn; everything is a `System`; the gradient comes from code that exists.
- Against: the cost becomes part of the plant's diagram (the closed loop now has a state `J`
  that is not physics, and `plot_trajectory` shows it); the integration scheme differs from
  the trapezoid rule every other tool uses, so R8 fails unless the block is the new owner of
  the rule, which the transcriptions cannot share; the exit rule and the terminal cost do not
  fit a block (a set is not a signal); `tf` and the draws belong to the problem, not the
  diagram; the mean over draws needs the problem's distributions anyway. Verdict: the right
  trick for a demo, the wrong owner of the contract.

### C. Verbs only, no object

Functions in `planning/evaluation.py`: `rollout(problem, controller, params, *, dt)`,
`cost(problem, controller, params, ...)`, `mean_cost(...)`, each compiling and caching
internally.

- For: tools-as-verbs in its purest form; no noun.
- Against: every call repeats `(problem, controller, dt, backend)` and relies on a hidden cache
  keyed on object identity, the kind of invisible global state CONSTITUTION §4.4 forbids; the
  params pytree has no home (`problem.params` is a `ProblemParameters` of the plant alone, not
  of the loop); the program bridge needs an object to hold the fixed leaves and the keys.
  Verdict: the verbs are right, the missing noun is the compiled loop.

### D. The `Simulator` does it

`Simulator(loop, backend="jax").solve_p(params)` returns a traced trajectory; `problem.score(traj)`
gives `J`; the draws as a batch of simulators.

- For: one orchestrator for simulation; no new concept.
- Against: `Simulator` is the NumPy-first orchestrator of solver presets, adaptive grids, a
  10 001-point reporting grid and verbose panels; the differentiable path is a fixed-step
  `lax.scan` and nothing else; `Trajectory` is a NumPy reporting object (`float()` casts) and
  would need a traced twin; the exit rule, the terminal cost and the draws are the problem's,
  so `Simulator` would grow a dependency on `planning`. Verdict: wrong layer; the scan under
  option A is what `Simulator` would have to become.

### E. A planner

`GradientPlanner(problem, controller)` whose `solve()` tunes the parameters and returns a
`PlanningSolution`; the evaluation inside `Planner.evaluate`.

- For: fits `solve` → `Solution` and `compare()`; a student meets one shape for every method.
- Against: it consumes the differentiable path, it does not provide it: the trajectory, the
  end state and the cost as functions of params are needed by the RL planner, the Monte Carlo
  evaluator and a hand-written loss alike, so the primitive must exist below any planner.
  Verdict: a consumer to add later (a `ParameterTuningPlanner` over the program bridge),
  not the architecture.

### F. Program first

`problem.program(controller, wrt=...)` returns a `MathematicalProgram` directly; the trajectory
and the cost are functions on its metadata.

- For: the `Optimizer` is the one solver; gain tuning and trajectory optimization look alike.
- Against: a student who wants `jax.grad` of a run, or the trajectory for a plot, does not
  think in NLPs; the program hides the evaluator it needs; the draws and the keys have to be
  fixed before there is anything to look at. Verdict: the bridge, not the object.

**Outcome.** A is the architecture; C's verbs are its methods; F is one of them; E is a later
consumer; B stays a research idea; D is ruled out.

## 3. Sub-decisions inside A

| Question | Options | Recommendation |
| --- | --- | --- |
| Are the draws part of the object? | the object knows them because the problem does; or `nominal()` problem plus an explicit `draws=` argument | the former: every planner takes the stochastic problem, and `nominal()` is the deterministic view for free |
| The params pytree | a plain nested dict shaped like `ProblemParameters`; or the frozen `ProblemParameters` itself | the dict: a pytree with string keys the way `diagram.params` is; `ProblemParameters` stays the problem's record of overrides |
| Where the scoring kernel lives | `core/costs.py` (the cost owns its integral); `planning/evaluation.py` (the problem owns exit, horizon, price) | `planning/evaluation.py`: the rule needs `X`, `tf` and `infeasible_cost`, which are the problem's |
| The record of a Monte Carlo run | `Evaluation` stays NumPy; or becomes a pytree so `evaluate(params, keys).mean` differentiates | a pytree: one verb, one record, the mean and the worst both differentiable, the failure rate reported |
| `MonteCarloEvaluator` | deleted; alias one release; or kept as the NumPy and simulator backends' home | alias one release, then gone; its backends are the new object's `backend=` |
| A traced `Trajectory` | `Trajectory` stays NumPy and the verb returns arrays; or `Trajectory` becomes array-module native | arrays now (`t, x, u, y`), the native `Trajectory` with A5 |
| The whole run out of the scan | a new method returning `(xs, signals)`; or `rollout_batch` gaining a `signals` output | both, same change; the single-run method is what the object calls |

## 4. Naming systems

Each row is one coherent set; mixing rows is what to avoid.

| Thing | N1 compile → evaluator | N2 closed-loop vocabulary | N3 evaluate → Evaluation | N4 rollout vocabulary |
| --- | --- | --- | --- | --- |
| facade | `problem.compile(controller)` | `problem.closed_loop(controller)` | `problem.evaluator(controller)` | `problem.rollout(controller)` |
| object | `ProblemEvaluator` | `ClosedLoop` | `Evaluator` (clashes with `core/compile/evaluators`) | `Rollout` (clashes with `StepRollout`, `rollout_batch`) |
| the run | `trajectory(params)` | `simulate(params)` | `simulate(params)` | `run(params)` |
| end state | `final_state` | `x_final` | `final_state` | `x_final` |
| the scalar | `cost` | `J` | `evaluate(...).mean` on one key | `cost` |
| over draws | `mean_cost`, `worst_cost` | `expected_J`, `worst_J` | `evaluate(params, keys)` → `Evaluation(J, failed, mean, worst, failure_rate)` | `average_cost` |
| feasibility | `failure_rate`, `margin` | `failure_rate`, `margin` | on the `Evaluation` | `failure_rate`, `margin` |
| system-evaluator method | `rollout_p` → `(xs, signals)` | `simulate_p` | `rollout_p` | `rollout_p` |
| kernel | `score_rollout` | `score` | `score_rollout` | `score_rollout` |
| bridge | `program(wrt=)` | `as_program(free=)` | `program(wrt=)` | `program(wrt=)` |

Critique:

- **N1** reads as the pair the library already teaches (`sys.compile()` → an evaluator with
  `f`, `rollout_batch`; `problem.compile(ctl)` → an evaluator with `trajectory`, `cost`). `cost`
  and `mean_cost` are plain English; `failure_rate` and `margin` are the words the material
  uses (3 and 13 files; `success rate` and `feasibility` 0 and 9). One new noun.
- **N2** uses the course's own phrase (`closed loop`, 23 files) and `J`, the symbol every
  lecture writes; `simulate` matches `simulation` (31). But `J` as a method name breaks RULES
  5.4 (a symbol is a local in math, not an API verb), `expected_J` is clumsy, and `ClosedLoop`
  names a diagram that already has a name (`ctl @ plant` *is* the closed loop); the object is
  the loop *compiled against a problem*, which N2 hides.
- **N3** adds the fewest names: `evaluate` → `Evaluation` exists on the surface, and making the
  record a pytree gives `jax.grad(lambda p: ev.evaluate(p, keys).mean)` with no new scalar verb.
  Cost: `Evaluator` as a bare noun collides with the compile band's evaluators, so it has to be
  `ProblemEvaluator` anyway; a single deterministic run is `evaluate(p, keys=one)` which is
  honest but roundabout; `.mean` of one trial reads oddly in a demo.
- **N4** matches the RL side (`rollout`, 23 files) but `Rollout` already names the step
  rollout record and `rollout_batch` the batch, so the word would mean three things.

The selection argument: `wrt=` is the word `jacobian(of, wrt)` already uses for "differentiate
with respect to", and the decision leaves are exactly what the program's gradient is taken
with respect to; `decision=` is the textbook word but new to the library; `free=` is short
and vague. `wrt=` wins on precedent.

## 5. Three candidate packages

| | **P1 Evaluator** | **P2 Evaluation** | **P3 Closed loop** |
| --- | --- | --- | --- |
| architecture | A | A | A |
| facade | `problem.compile(ctl, dt=, backend=)` | `problem.compile(ctl, dt=, backend=)` | `problem.closed_loop(ctl, dt=, backend=)` |
| object | `ProblemEvaluator` | `ProblemEvaluator` | `CompiledLoop` |
| run | `trajectory(params, x0=, key=)` → `t, x, u, y` | `simulate(params, x0=, key=)` | `simulate(params, x0=, key=)` |
| end state | `final_state(...)` | `final_state(...)` | `final_state(...)` |
| scalar | `cost(params, x0=, key=)` | `evaluate(params, key).mean` | `cost(...)` |
| over draws | `mean_cost(params, keys)`, `worst_cost` | `evaluate(params, keys)` → `Evaluation` pytree (`J`, `failed`, `mean`, `worst`, `failure_rate`) | `expected_cost(params, keys)`, `worst_cost` |
| feasibility | `failure_rate(params, keys)` reported; `margin(params, keys)` differentiable | on the `Evaluation`; `margin(...)` differentiable | same as P1 |
| bridge | `program(wrt=[...], lower=, upper=, constraints=, keys=)` | same | `program(wrt=...)` |
| system-evaluator method | `rollout_p(x0, u_seq, t0, dt, params)` → `(xs, signals)`; `rollout_batch` gains `signals` | same | same |
| kernel | `score_rollout` (JAX), `score_trajectory` (NumPy twin) | same | same |
| `MonteCarloEvaluator` | alias of `ProblemEvaluator` one release | its `evaluate` *is* the new `evaluate`; the class becomes the alias | alias |
| new public names | 1 noun, 6 verbs | 1 noun, 3 verbs (+ `Evaluation` as pytree) | 1 noun, 6 verbs |
| what a student types | `jax.grad(ev.cost)(ev.params)` | `jax.grad(lambda p: ev.evaluate(p, keys).mean)(ev.params)` | `jax.grad(loop.cost)(loop.params)` |

Where they differ:

- **P1 vs P2** is "one scalar verb per quantity" against "one verb, one record". P1 reads best
  in a demo and in `jax.grad`; P2 adds the fewest names and keeps `Evaluation` as the one
  record of a Monte Carlo run, which `PlanningSolution.evaluation` and `compare()` already
  consume. They are not exclusive: P2's `evaluate` can sit beside P1's `mean_cost` as the
  record-returning form, at the cost of two ways to get the mean.
- **P3** is P1 with the course's phrase on the facade. It names what the student built (the
  closed loop) rather than what the library did (compiled it against a problem); it reads
  well until an open-loop plan is "closed" with a `TrajectorySource`, where `closed_loop`
  misleads and `compile` does not.

**Recommendation: P1, with P2's `evaluate` → `Evaluation` kept as the record-returning verb
for the Monte Carlo use.** Reasons: it is the pair students already know (`compile` → an
evaluator), the verbs are the plain words of the quantities, the two feasibility words are the
material's own (`failure_rate`, `margin`), and `wrt=` carries the `jacobian` precedent into
the program bridge. `MonteCarloEvaluator` keeps working one release as an alias whose
`evaluate` is the new one.

The two choices this leaves for the maintainer: the object's noun (`ProblemEvaluator` is
proposed; `PlanningProblemEvaluator` is longer and exact; `CompiledProblem` says what it is
but not what it does), and whether both `mean_cost` and `evaluate(...).mean` exist, or only
one.
