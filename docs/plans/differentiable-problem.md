# The differentiable planning problem: one path from any parameter to any metric

Status: consolidated proposal for the maintainer (2026-10-18). One document replaces the
plan of 2026-10-17, its architecture-and-naming analysis and the target-architecture ideas of
2026-10-18, after a third, independent review probed each against the code; what each of the
three said and why the consolidation went the way it did is in
[the consolidation record](../reviews/2026-10-18-differentiable-problem-consolidation.md).
What is marked **decided** was ruled before this document; everything else is proposed and
waits on the asks of §9. §10 records where a seven-design panel (2026-10-20) overturned or
split on the proposals above; where §10 and an earlier section disagree, §10 is the proposal. Nothing is built; §8 is the build order, step V1 and rows S73 and S74
of [TODO.md](TODO.md) §4.
Rung: v0.3 (the evaluator and the optimizer), v0.3 and after (traceable analysis).
Lane: research first; the teaching surface once a course has run it.

## 1. The target

**Any performance metric that is a traceable function of a run or of a linearization can be
optimized through the native `Optimizer`, with respect to any parameter set the problem
holds.** The library provides the metric as a verb, JAX provides the gradient, the optimizer
provides the descent loop; no demo writes a scan, a cost formula or a loop of its own.

The parameter sets are the columns, the metrics the rows; each cell names the step of §8
that makes the pair one verb (`—` where the pair has no meaning):

| Metric (a verb on the compiled problem or the linearization) | controller params | open-loop input sequence | plant params | cost and set params | `x0` |
| --- | --- | --- | --- | --- | --- |
| Trajectory cost `J` of one run | step 4 (a hand scan today) | step 8 (`wrt="u"`) | step 4 (a hand scan today) | step 6 | step 4 |
| The whole trajectory, its final state | step 4 | step 8 | step 4 | step 6 | step 4 |
| Mean and worst cost over the problem's draws | step 7 | step 8 | step 7 | step 7 | step 7 |
| Failure rate; the smallest margin to `X` and `U` along the run | step 7; step 4 | step 8 | step 7; step 4 | step 7 | step 7; step 4 |
| Closed-loop poles, damping ratios, natural frequencies | S73 | — | S73 | — | — (linearized at `x̄`) |
| Gain and phase margins, Bode values, the sensitivity peaks `M_s`, `M_t` | S73 | — | S73 | — | — |

Every cell has the same consumer: the program bridge (§4.6) builds a `MathematicalProgram`
whose objective is one metric and whose constraints `g ≥ 0` can be other metrics (maximize the
phase margin over the PID gains subject to a gain margin above 6 dB and the gains in a box),
and `Optimizer` solves it with a method the program's class admits (§4.7). The worked
examples the table implies, each one demo once it lands: gain tuning on the closed-loop cost
(today's `pid_autotuning_jax` hand loop); gain tuning on a stability margin; an open-loop
input sequence by single shooting (today's `cartpole_rollout_gradients` and the showcase cell);
robust tuning on the mean cost over a plant-parameter family; parameter identification from
logged data (today's `params_gradient` and the showcase cell); plant-parameter sensitivity of
poles and margins.

## 2. The thesis: three nouns carry the path

```python
problem = PlanningProblem(Pendulum(), cost=QuadraticCost(...), X=BoxSet(...), tf=10.0)
evaluator = problem.compile(ImpedanceIntegralController(Kp=1.0, Ki=0.0, Kd=1.0), dt=0.05)

params = evaluator.params                        # {"system": {"controller": …, "plant": …}, "cost": …, "sets": …}
traj = evaluator.trajectory(params)              # the run, a Trajectory (of=plant: the plant's own x and u)
J = evaluator.cost(params)                       # the one scoring contract
dJ = jax.grad(evaluator.cost)(params)            # dJ / d every parameter: gains, mass, weights
m = evaluator.margin(params)                     # the smallest margin to X and U along the run, ≥ 0 iff feasible
report = evaluator.evaluate(params, key, n_trials=200)     # the draws: J per trial, failed, mean, worst, failure_rate
J_bar = evaluator.mean_cost(params, key, n_trials=200)     # the same mean as one scalar, for jax.grad

program = evaluator.program(wrt=["controller.Kp", "controller.Ki", "controller.Kd"], lower=0.0)
result = Optimizer(program, method="scipy_slsqp").solve()  # gain tuning as an NLP; "adam", "gd" after S74
```

Three nouns do the work, and two of them exist:

- **`Trajectory`**, the run, becomes array-native and a JAX pytree (**decided 2026-10-18**),
  so the compiled rollout, the cost integral and the plots speak one noun.
- **`ProblemEvaluator`** (working name), the problem compiled against a controller: pure
  functions of one params pytree, from the parameters of anything in the loop, the cost or
  the sets to the trajectory, the cost, its mean over the problem's draws and the feasibility
  margin. It absorbs `MonteCarloEvaluator`.
- **`MathematicalProgram`**, unchanged in shape, gains one more producer (the bridge) and, on
  `Optimizer`, the program classes that pick a method (S74).

Everything else folds in. The core-objects review counted the duplication one layer above the
laws: five scoring rules with four quadratures, two signal reconstructions, two trajectory
nouns, hand-written scans in four demos and three notebook cells
([the record](../reviews/2026-10-18-core-objects-review.md)). The nouns are right; the verbs
are duplicated; this plan gives each verb one traced kernel and lets the NumPy consumers call
it.

![The pipeline: the problem and its params, the compile step, the rollout and scoring kernels around a native Trajectory, the Evaluation, and the NumPy consumers](differentiable-cost-pipeline.png)

*The statement (green) holds the system, the cost, the sets and the distributions with one
params pytree at its root. The traced path (purple) is one compiled object per problem and
controller: one integrator kernel that emits the internal signals, a native `Trajectory`, one
scoring kernel, an `Evaluation` of scalars. The NumPy consumers (grey) call the kernels; none
keeps a twin. `differentiable-cost-pipeline.svg` and `.mmd` are the editable sources.*

![Ten workflows as lanes](differentiable-cost-workflows.png)

*One lane per workflow: the source object, the solver-ready form, the result, the consumer.
Purple traces today, green traces once this plan lands, grey is NumPy by contract (reporting,
solver loops, random search). Two lanes stay grey in the middle on purpose: model predictive
control (no implicit gradient of the inner solve) and the sampling planner (random search).*

**What exists, and what the prototype of 2026-10-18 showed** (the record has the numbers):

- The loop `ctl @ plant` compiles to a `lax.scan` with `params` threaded
  (`rk4_integrate_zoh_trace_p`, `rollout_batch` with a params family); the controller's state
  is inside `x`, so a PID with an integrator traces from the first commit.
- The plant's command along the traced run is a slice of the port-signal buffer the diagram
  evaluator already fills at every step and throws away.
- With a masked trapezoid kernel on that run, `jax.grad` with respect to a gain, a damping
  gain and the plant's mass matched central differences to better than 10⁻⁹ relative, through
  the real `Pendulum`, `ImpedanceIntegralController` and `QuadraticCost`.
- The chosen leaves ravelled to `z`, a `MathematicalProgram` with `backend="jax"` and the
  existing `Optimizer` reached a stationary point (`‖∇J‖ = 2·10⁻⁸`) in 49 SLSQP iterations and
  two seconds, where 400 steps of the demo's hand-written gradient loop stopped at twice the cost.
- `ProblemParameters(system, cost, sets, scene)` is read by the transcriptions, DP, the
  evaluators and the RL environment; the library costs and sets receive `params` and ignore
  it (cost-params.md, A5).
- `sample_x0(key)`, `sample_params(key)` (dotted names, nested dict, `merge_params`) and
  `sample_disturbances(key)` trace; `System.realize(key)` casts to `int` and does not.
- Every library set's `margin` traces; `contains` is NumPy. `Trajectory` casts to `float`
  and copies, so nothing traced survives it today.

## 3. The objects, today and after

| Object | Today | After |
| --- | --- | --- |
| `PlanningProblem` (frozen) | `sys`, `cost`, `X`, `U`, `X0`, `Xf`, `tf`, `infeasible_cost`, `params: ProblemParameters`; computes nothing | unchanged in shape; gains `compile(controller=None, *, dt, backend="auto", episode_length=None)`, a two-line facade |
| `StochasticPlanningProblem` | adds the three laws, `criterion`, the three `sample_*(key)`; every planner takes it through `nominal()` | unchanged; its draws are the one source the evaluator vmaps over |
| `Trajectory` (frozen) | a NumPy reporting object: `float` casts, copies | **decided:** array-native, a JAX pytree; reporting verbs NumPy by contract (§4.2) |
| `StepRollout` | the step twin of `Trajectory` | a November consolidation candidate: one noun with `t = k·dt`, after §4.2 |
| `CostFunction` | `g`, `h` trace; `evaluate_trajectory` is a NumPy `for` loop with `float()` casts | `evaluate_trajectory` / `total_cost` on `xp` (vectorized, no cast); reads `params` with A2 |
| `Set` | `margin` traces; `contains` is NumPy | `contains` derived from `margin` on `xp`, so the kernel and the RL environment share the one exit test; reads `params` with A5 |
| `MonteCarloEvaluator` | a verb pretending to be a noun: composes `policy @ plant` again, recompiles, refuses controllers with state, scores with an inline copy of the rule | absorbed: `evaluator.evaluate` (§4.5); the name an alias for one release |
| `Evaluation` | a NumPy record: `J`, `failed`, `x0`, `trajectories`, `mean`, `worst`, `failure_rate`, `value(criterion)` | a pytree with `xp` statistics, so `jax.grad(lambda p: ev.evaluate(p, key).mean)` runs; the trials' trajectories on demand through `evaluator.trajectory(params, x0=, key=)`, not stored |
| `ProblemEvaluator` | does not exist | **new** (§4.1), in `planning/evaluation.py` |
| `RolloutEnvironment` | the problem as a step function for the RL collectors | unchanged role; its exit test is the one `Set.contains`, its reward the same `g` |
| `PlanningSolution` (frozen) | `problem`, `policy`, `solver`, `trajectory`, `evaluation`, `cost_to_go` | unchanged; `trajectory` and `evaluation` of a feedback planner come from the evaluator, reproducible from problem, policy and key |
| `MathematicalProgram` (frozen) | `J(z)`, `h`, `g`, bounds, `backend`, `metadata`; reached through a trajopt transcription | unchanged in shape; one more producer, `evaluator.program(wrt=)` (§4.6); its class read from its fields (§4.7) |
| `Optimizer` | three constrained-NLP presets on a flat `z` | the program classes, a first-order backend (`gd`, `adam`), two free SciPy presets, one global derivative-free preset (§4.7) |
| `ParametricMathematicalProgram` | `h(z, x0)`, `bind(x0)`, the MPC block's | unchanged now; later the case "no decision leaves, `x0` bound" of the same bridge |

The `System` evaluators gain one method (§4.4); nothing else appears or disappears.

## 4. The design

### 4.1 The compiled problem

`problem.compile(controller=None, *, dt, backend="auto", episode_length=None)` builds the loop
as `closed_loop` does (`controller @ problem.sys`; a `TrajectorySource` policy gives
`source >> plant`, the open-loop plan replayed; no controller gives the plant alone, driven by
the input sequence the verbs take), compiles it once on the backend, and returns the
evaluator. `dt` is the fixed RK4 step and the sampling grid; `tf` is the problem's, or
`episode_length` on an infinite horizon (10 s by default, as the Monte Carlo evaluator has).

Its verbs are pure functions of `(params, x0=None, key=None)`: `params` the pytree below,
`x0` the plant's start (the problem's `x_start`, or the draw), `key` one episode's draws (a
stochastic problem's parameters and disturbances; `None` is the nominal episode):

| Verb | Returns | Differentiable |
| --- | --- | --- |
| `trajectory(params, x0, key, *, u=None, of=None)` | the loop's `Trajectory`: `t`, `x`, its boundary `u` (`u=` a held input sequence, else the nominal), every internal port as a signal (`"controller:u"`, `"plant:y"`); `of=plant` the plant's own `(x, u)`, the traced twin of `trajectory_of` | yes |
| `cost(params, x0, key)` | `J` under the one scoring contract (§4.3), the plant's run scored | yes; zero after an exit (§5) |
| `margin(params, x0, key)` | the smallest margin along the run, `min_k min(X.margin(x_k, t_k), U.margin(u_k, x_k, t_k))`: `≥ 0` iff the run is feasible | yes (piecewise) |
| `evaluate(params, key, *, n_trials)` | the `Evaluation` over the draws of §4.5 | its statistics |
| `mean_cost(params, key, *, n_trials)` | `evaluate(...).mean`, one scalar for `jax.grad` | yes |
| `program(wrt, ...)` | a `MathematicalProgram` (§4.6) | through the `Optimizer` |

`evaluator.params` is the nominal pytree, a plain nested dict in the shape of
`ProblemParameters`: `"system"` nests the loop's blocks exactly as `diagram.params` does
(`problem.params.system` is its `"plant"` entry), `"cost"` is the cost's dictionary (A2),
`"sets"` the sets' (A5); a group with nothing to read is `None`, an empty pytree. Partial
dictionaries follow the diagram rule: a missing block reads its own params, an unknown key
raises. `jax.grad(evaluator.cost)(evaluator.params)` is the first line a student writes.

`backend="auto"` is JAX when the loop, `X` and the cost trace on it, NumPy otherwise, with the
fallback announced when it changes the draws; an explicit `"jax"` on a NumPy-only law raises
and says so; `"numpy"` is the same object on the NumPy evaluator (`rk4_integrate_zoh_p`, the
internal signals per sample), one trial at a time, for a lookup-table law or a laptop without
JAX. The integration is the loop's own RK4 on the diagram, so a controller with state (an
integrator, an observer) is inside `x` from the first commit; the Monte Carlo evaluator's
held-input static law goes with it (§4.5). Refused with a message saying so: a model
predictive controller (its inner solve has no implicit gradient; `backend="numpy"` evaluates
it), and a `StepSystem` controller or a `Computer @ plant` loop (the sampled seam; the hybrid
simulator runs it).

The AC-7 rule holds by construction: the cost reads the plant's `(x, u)` inside the loop,
never the loop's boundary signals.

### 4.2 The native `Trajectory` (decided 2026-10-18; the rules below are the implementation)

`Trajectory` keeps the arrays it is given through the array module: no `float` cast, no copy of
a traced value; `t`, `x`, `u` and the signal arrays are pytree leaves; the shape checks stay
(shapes are static under tracing), the monotonic-time check runs on NumPy only (a tracer has
no concrete values); the reporting verbs (`t0`, `tf`, `resample`, `save`, the plots) stay
NumPy by contract and are never called on the traced path (`traj.t[-1]`, not `traj.tf`, inside
a kernel). Two rules the probe of 2026-10-18 found necessary:

- **Unflatten bypasses validation.** JAX rebuilds a pytree with batched leaves (`vmap` adds an
  axis: `x` becomes `(B, n, N)`) and with `None` or sentinel leaves (`tree_map`, the zeros of a
  gradient). A registration whose unflatten goes through `__post_init__` fails on both
  (`vmap` of a function returning a `Trajectory` raised in the probe); the registered
  unflatten builds the object from its leaves without checks (`object.__new__` and the
  frozen setattr), and the public constructor keeps every check.
- **`signals` flatten in a fixed key order** (sorted; a `MappingProxyType` is not a pytree
  node), so two trajectories with the same signal names share one treedef and `vmap`,
  `lax.scan` and `jit` accept them.

The follow-on, not decided: `StepRollout` becomes a constructor of the same noun with
`t = k·dt` (a November consolidation candidate, §6).

### 4.3 One scoring kernel: `score_trajectory`, the contract, on `xp`

DESIGN §6 states one scoring contract and the code writes it five times with four quadratures:
`CostFunction.evaluate_trajectory` (a NumPy loop), `score_trajectory` (NumPy, the reference),
the transcriptions' `trajectory_cost` (both backends, no discount), `MonteCarloEvaluator.evaluate_jax`
(JAX, inline, an `alive` mask) and the RL reward (`−g dt` per step). After this plan,
`score_trajectory(problem, traj, *, params=None, infeasible_cost=inf)` keeps its name and its
signature and becomes the one body, on `xp = array_module(traj.x)`, layered on
`CostFunction.evaluate_trajectory` / `total_cost` upgraded the same way:

```
inside_k = X.contains(x_k, t_k; p_X) and x_k finite              every sample k
alive_k  = Π_{j ≤ k} inside_j                                     1 until the first exit, 0 after
dJ_k     = e^{−ρ t_k} g(x_k, u_k, t_k; p_g)
J        = [Σ_k alive_k · ½ (dJ_k + dJ_{k+1}) (t_{k+1} − t_k)]  as a running sum, its last entry
         + [an exit] · price(x_exit, t_exit)
         + [no exit and a finite t_f] · h(x_N, t_N; p_g)
```

the exit price being the problem's `infeasible_cost` (`infeasible_penalty(x, t)`), else the
`infeasible_cost` passed in (an evaluator passes the bound its environment derived; `+inf`
otherwise); `failed` is `not all(inside)`. Three facts about this form, from the probe:

- **The running sum keeps the NumPy numbers.** Written with `xp.cumsum`, the masked kernel
  reproduced today's sliced `score_trajectory` bit for bit on 300 random trajectories with
  and without an exit; written with a masked `xp.sum` it did not (70 of 300), because NumPy's
  `sum` is pairwise. So the NumPy consumers see a byte-identical refactor (RULES 7.7) and the
  JAX path, which agreed to `3·10⁻¹⁵` relative, gets the tolerance test the backends already
  share (`rtol = 1e-6`).
- **The transcriptions take the cost's discount.** `trajectory_cost` applies none today while
  the contract does; it reads `cost.discount_rate` like every other caller. No
  trajectory-optimization test, demo or benchmark baseline declares a rate, so every baseline
  stays bit-identical (`e⁰ = 1`), and a discounted problem's plan changes once, as DESIGN §6
  says it should. No `discount=` keyword: the rate has one owner, the cost (RULES 5.6).
- **No scoring modes.** The kernel is the problem's contract and nothing else; a tool that
  wants the uncut integral scores the problem without the constraint it lowers elsewhere
  (`dataclasses.replace(problem, X=unconstrained)`, which is what the bridge does in §4.6),
  and a tool that wants a penalty adds `Set.as_cost` to the cost. One number per trajectory,
  in every report.

The callers after the change: `Evaluation.of_trajectory`, `evaluator.cost` (on its traced
run), the transcriptions' decision-variable objective and reporting (the same quadrature, the
same `h` and discount rule, stated once in DESIGN and cited), `PlanningSolution.cost` and the
`compute_cost` facade. The RL reward stays the documented exception (`r_k = −g_k dt`, the
learner's left-Riemann contract, the discount through `γ` once), derived from the same `g`, and
its exit test is the same `Set.contains` on `xp` (today's `~all(margin ≥ 0) | ~finite`
written in two places). `MonteCarloEvaluator.evaluate_jax`'s inline copy goes with the class.

### 4.4 The whole run out of the scan

`rk4_integrate_zoh_trace_p` and `rollout_batch` return the state history `(N+1, n)` and drop
the rest: the boundary `y` and the internal signals, above all the plant's command. The
diagram evaluator fills its port-signal buffer at every step (`_compute_port_signals_p`) and
throws it away. One method on the JAX and NumPy diagram evaluators carries it out:

```python
t, xs, signals = evaluator.rollout_p(x0, u_seq, t0, dt, params)   # signals (N+1, n_signals), at the sample times
```

the scan emitting the buffer of the step's first stage beside `x_next`; `rollout_batch` gains
the same `signals` output. The plant's `(x, u)` are slices of it (`plan.state_index`,
`plan.output_slices`), the traced twin of `DiagramSystem.trajectory_of`, whose NumPy loop
becomes a call to the same method with NumPy arrays. One signal per sample, read at the grid
state as `reconstruct_internal_signals` reads it today, so the plots and the kernel see the
same `u`. The method is written in the trimmed style the evaluator-grid consolidation wants
(one `_p` body, the bound and jitted variants generated), so it does not add a seventh column.

A test pins that the simulator's `rk4_fixedsteps` on one diagram and the compiled rollout
agree to round-off on the same grid, so the traced path cannot drift from what the plots show.

### 4.5 Draws, as the problem states them

`evaluate(params, key, n_trials=)` splits one key per trial (one key per experiment, the
randomness plan's convention) and `vmap`s one `cost` over the trials: `x0 = problem.sample_x0(k0)`;
`system = merge(params["system"], problem.sample_params(k1))`, the draw a pytree override so a
drawn mass and a differentiated gain coexist; the disturbances as held draws per step from
`sample_disturbances(k2)`, as the RL environment draws them, until RN-4 puts a seed per random
block in the pytree (`realize(key)` traced: `uint32` seeds instead of Python ints, which the
blocks' counter-based draws already accept); then the mechanism changes under the same verb.
A deterministic problem gives `evaluate(params).J` of one trial, equal to `cost(params)`, the
twin test of randomness.md's RD-D23. `Evaluation` holds `J (B,)`, `failed (B,)`, `x0 (B, n)`
as leaves and `mean`, `std`, `worst`, `failure_rate`, `value(criterion)` as `xp` scalars
(`"worst_case"` is the worst draw, a CVaR when the criterion asks for one), so the mean and the
worst differentiate and the failure rate is reported; `print(report)` formats them. The
recorded trials of today's `record=True` are not stored: `evaluator.trajectory(params,
x0=report.x0[i], key=keys[i])` replays any one of them.

This is RN-5 seen from the other side (randomness.md: "the Monte Carlo evaluator as a batched
simulation of the closed-loop diagram on its test set", ruled RD-D24): the loop is a diagram,
compiled once, simulated on every realization, the cost scored on the plant. Two consequences
the maintainer already accepted there and this plan restates: the evaluator's numbers change
once (the held-input static law becomes the continuous loop's own RK4, O(dt²) apart on a smooth
law), re-pinned with a release note; and `backend` chooses the arrays only, the `"simulator"`
backend being this path. `Planner.evaluate`, `solve(evaluate=True)`, `nominal_trajectory` and
`compare(...).evaluate(dt=, key=, n_trials=)` read the evaluator; `MonteCarloEvaluator` is an
alias whose `evaluate` is the new one, gone the release after.

### 4.6 The bridge to `MathematicalProgram`

Tuning parameters is an NLP whose objective is a metric of the run. The bridge is designed
for the whole table of §1 from the start:

```python
program = evaluator.program(
    wrt=["controller.Kp", "controller.Kd"],   # dotted leaves of evaluator.params; "u" the input sequence; "x0" the start
    objective=None,                           # None: cost (one episode) or mean_cost (key given); else a callable of params
    constraints=(),                           # callables of params, each lowered to g(z) ≥ 0 (a margin floor, a phase margin)
    sets="hard",                              # the problem's X and U: "hard" (margin ≥ 0 as g), "soft" (a penalty on J), "ignore"
    key=None, n_trials=None,                  # the draws of mean_cost, drawn once here: a sample average two solves share
    lower=None, upper=None,                   # a box on the chosen leaves, scalars or their shape
)
```

- `wrt` names leaves by dotted path from the root of `evaluator.params` (`system.controller.Kp`,
  `cost.Q`, `sets.…`); a path whose first segment is a block of the loop is read under
  `system`, the convention `params_distribution` already uses; a pytree mask is accepted too.
  `"u"` ravels the held input sequence of the open-loop evaluator (single shooting with a held
  input, §5), `"x0"` the start. The chosen leaves are ravelled to `z`
  (`jax.flatten_util.ravel_pytree`); every other leaf is fixed at `evaluator.params`.
- `J(z)` is the objective at `unravel(z)`; `g(z)` stacks the constraints and the lowered sets;
  `lower` and `upper` are bounds on `z`. `backend="jax"`: the `Optimizer` differentiates it
  as it does any program.
- **The sets lower at the solver boundary**, as the transcriptions lower theirs (RULES 4.3):
  `"hard"` scores the problem with the lowered set removed from the cut and adds
  `margin(params) ≥ 0` as `g` (the exact minimum along the run; a smooth minimum,
  `−τ·logsumexp(−m/τ)`, is an option named `temperature=` only if a solver stalls on the
  kink); `"soft"` scores the problem with `X.as_cost(weight, shaping)` and `U.as_cost(...)`
  added to the cost (step 8); `"ignore"` scores the problem without them and reports
  `failed` beside. The default is `"hard"`: a problem that declares `X` means it.
- `program.metadata` records `wrt`, the keys and `unravel`, so the callable `program.metadata["unravel"]` applied to `result.z`
  gives the tuned leaves back as a pytree; `MathematicalProgram` stays flat on `z` and
  `Optimizer` never sees a pytree (the constitution: not an optimization modeling language).
  A facade, `evaluator.tune(wrt, *, method="scipy_slsqp", **program_kwargs) -> params`, builds
  the program, runs the `Optimizer` and unravels, for the demo that wants one line (an ask).
- The same bridge is **identification**: `objective=lambda p: mismatch(evaluator.trajectory(p), data)`
  with `wrt` the plant's physical parameters or a network's weights (the C4 row's `fit`, one
  verb for both). `ParametricMathematicalProgram.bind(x0)` is the case "no decision leaves,
  `x0` bound" of this bridge, and the scene bind `J(z, p)` the case "bound params"; both can
  follow once the program and the evaluator share the pytree.

### 4.7 The optimizer as the one descent loop (S74)

Today `Optimizer` offers three constrained-NLP presets (`scipy_slsqp`, `scipy_trust_constr`,
`ipopt`) and every gradient-based tuning writes its own loop: four compile demos, three
notebook cells, while the RL algorithms share one pytree `Adam` in
`reinforcement_learning/optim.py`. Three parts, the maintainer's idea of 2026-10-18 made
concrete:

1. **The program knows its class, read from its fields.** `unconstrained` (no `h`, no `g`, no
   bounds), `bounded` (a box only), `constrained` (any `h` or `g`); `stochastic` is a flag a
   resampling objective sets, added with its first consumer (the bridge draws once by default,
   so its programs are deterministic sample averages and every preset applies). `Optimizer`
   refuses a method the class does not admit, naming the admissible ones; `method="auto"` picks
   by class. The default stays `method="scipy_slsqp"` (behaviour-preserving); whether `"auto"`
   becomes the default is the V3 freeze's question.

   | Class | Admits | `"auto"` |
   | --- | --- | --- |
   | `unconstrained` | `gd`, `adam`, `scipy_bfgs`, `scipy_lbfgsb`, `scipy_slsqp`, `scipy_trust_constr`, `ipopt` | `scipy_lbfgsb` |
   | `bounded` | `gd`, `adam` (projected), `scipy_lbfgsb`, `scipy_slsqp`, `scipy_trust_constr`, `ipopt`, `scipy_differential_evolution` (a finite box) | `scipy_lbfgsb` |
   | `constrained` | `scipy_slsqp`, `scipy_trust_constr`, `ipopt` | `scipy_slsqp` (Ipopt when installed is the Later idea it already is) |
   | `stochastic` | `gd`, `adam`; `scipy_differential_evolution` when bounded and gradient-free | `adam` |

   A NumPy program without `grad_J` keeps SciPy's own finite differences (today's path); `gd`
   and `adam` refuse it ("needs a gradient: a JAX program or `grad_J`").
2. **One first-order backend**, `optimization/optimizers/first_order.py`: gradient descent and
   Adam on the flat `z`, a fixed step count or a gradient tolerance, the box by projection
   (the cart-pole demo's clip), the global gradient clip, `callback(z)` per iterate. The RL
   package's pytree `Adam` moves there unchanged (a flat vector is a pytree) and the RL
   package imports it, so the RL baselines stay bit-identical; the Optax-style
   `init` / `update` protocol is kept. Two SciPy presets (`scipy_lbfgsb`, `scipy_bfgs`) come free
   from the existing backend; **one global, derivative-free preset**,
   `scipy_differential_evolution` (`scipy.optimize.differential_evolution` on the box, seeded),
   is the stochastic global search for a bounded program whose landscape has several basins or
   whose objective is noisy and gradient-free (a Monte Carlo score of a lookup-table law);
   `"auto"` never picks it, the user does, knowing its cost in evaluations. The sampling
   planner (MPPI, S72) stays the global search *over trajectories*; the optimizer's global
   preset is the one over *parameters*. Presets join one registry, the shared factory of
   OPW-1 (optimizer-parametric-wiring.md), where the trajopt planner's duplicate registry dies.
3. **Pytree decisions stay outside `Optimizer`**: the bridge ravels and unravels (§4.6).

Then the four demos and the notebook cells end in `Optimizer(program, method=...)` or
`evaluator.tune(...)`, and the library holds one descent loop; the maintainer decides whether
one tutorial keeps a hand-written `value_and_grad` loop on purpose (§5), as the GRO501
from-scratch notebooks do.

### 4.8 Traceable linear analysis (S73, independent of the evaluator)

`poles`, `frequency_response` and `margins` through the array module, so the sensitivity of
closed-loop poles, damping ratios, Bode values and margins to any plant or controller param is
`jax.grad` of the existing verbs on `jacobian("f", "params")`. The math lines stay the
textbook ones: `C (jωI − A)⁻¹ B + D` as one batched solve; eigenvalues through
`xp.linalg.eigvals`, whose derivative is the classical `vᴴ (∂A/∂p) u / (vᴴ u)`; Brent's method
on the crossovers replaced by a fixed number of Newton steps from the grid bracket, so the
crossover frequency carries the implicit-function derivative. Beside the crossover margins,
the sensitivity peaks `M_s = max_ω |S(jω)|` and `M_t = max_ω |T(jω)|` (and a `logsumexp`
surrogate over the grid) as traceable metrics on the sensitivity functions: a bound
`M_s ≤ 1.5` guarantees a gain margin above `M_s / (M_s − 1)` and a phase margin above
`2 asin(1 / 2M_s)` with no crossover to jump, the smooth constraint for loop shaping through
the bridge. Zeros stay NumPy (a generalized eigenproblem JAX lacks); root-locus branch
matching stays NumPy (plotting); the root locus generalizes to any parameter. Documented
non-differentiable spots: repeated poles, the switch of the minimum margin between crossovers;
the nonsymmetric eigensolver is CPU-only. The done-when is on the workboard row.

## 5. Guardrails: the sharp edges and the rule each takes

- **A cut has zero gradient after the exit.** The contract's `J` of a run that leaves `X` is
  the price, a constant, so `jax.grad(evaluator.cost)` says nothing about how to pull the
  run back inside. The rule: declare `X` only where it is meant (it is unconstrained unless
  declared; `X = sys.state.box` is a safety constraint, not a plotting domain), tune through
  the bridge, whose `"hard"` and `"soft"` lowerings carry the margin's gradient, and read
  `margin(params)` beside `cost(params)` in a hand-written loop. Not a mode on the kernel.
- **`Xf` is never read by the kernel.** `PlanningProblem` builds `Xf = SingletonSet(x_goal)`
  whenever `x_goal` is given; a closed loop approaches it asymptotically, so it is a reference,
  not a terminal equality. The kernel charges `h(x_N, t_N)` at a finite horizon and nothing
  else; a region `Xf` lowers to `Xf.margin(x_N) ≥ 0` and a singleton to the residual `= 0` only
  in the open-loop transcriptions, as today.
- **`U` is information, not saturation.** No backend clips a law (DESIGN §6); a linear law
  without a `Saturation` block can command outside `U`. The `margin` verb includes
  `U.margin` on the plant's command, so the bridge enforces or penalizes it; a `Saturation`
  block in the loop is the modelling answer.
- **Single shooting is not collocation.** The open-loop evaluator with `wrt="u"` is single
  shooting with a held input, the showcase's hand loop as one line; through an unstable plant
  over a long horizon its gradients explode or vanish and the landscape is nonconvex, where
  collocation's defect constraints break the chain and expose sparsity. `ShootingTranscription`
  (a linear input between knots, so `policy >> plant` replays the plan) and `direct_collocation`
  stay the trajectory-optimization path; the bridge's shooting is an evaluation and tuning tool.
- **Backpropagation through the dynamics and policy gradients are complementary.** The
  evaluator differentiates *through* `f` and the reparameterized draws; `REINFORCE`, `PPO`,
  `SAC` differentiate the log-policy score and never `f`. Short smooth horizons favour the
  former by orders of magnitude; long horizons, saturations and stiff dynamics favour the
  latter. One demo puts `problem.compile(policy)` and `ReinforcementLearningPlanner` side by
  side on one `StochasticPlanningProblem` and one `NeuralPolicyController`.
- **The program stays flat.** `MathematicalProgram` is `J(z)`, `h(z) = 0`, `g(z) ≥ 0` and a
  box on `z ∈ ℝⁿ`; ravelling lives at the bridge, so SLSQP, Ipopt, L-BFGS-B and Adam solve one
  object interchangeably. One tutorial may keep a six-line `jax.value_and_grad` loop so the
  student sees there is no magic, followed by the one-line `Optimizer` call every other demo
  uses (an ask).
- **Crossover margins jump; sensitivity peaks do not.** A gain or phase margin is defined at a
  crossover that can appear, vanish or switch during a descent, and `eig` has no derivative at
  a repeated pole; `M_s`, `M_t` are the smooth robustness bounds for an NLP, the margins the
  numbers a course reports (§4.8).

## 6. What this plan changes in the other plans

| Plan | Change |
| --- | --- |
| cost-params.md (A2) | read as one job with this plan (the core-objects review's first consequence): its `xp` upgrade of `evaluate_trajectory` / `total_cost` is step 1 here; its `params` work is step 6, after the deterministic evaluator exists; its migration step 5 (the `CostIntegrator` block) closes: the trapezoid kernel on samples is the contract, the block stays a demo idea |
| TODO A5 (the later nouns) | `Set.as_cost(weight=, shaping=)` lands in step 8 with its first consumer (the bridge's `"soft"`), after `shaping.py` moves to `core/`; sets reading `params` fills `params["sets"]` in step 6 |
| randomness.md (RN-4, RN-5) | RN-5's step 1 is step 7 here, one path; RN-4's seeds slot under the same verb when they land; RN-5's done-when (the LQG loop's mean cost within the covariance prediction's sampling error) is step 7's |
| automation-by-convention.md (AC-7) | the evaluator scores the plant by construction; AC-7 extends the rule to the design tools |
| mppi.md (S72) | `rollout_batch` with `signals` and the realization batch are what MP-1 and MP-3 roll out on; `score_trajectory` is their scoring rule |
| fields.md (A1) | the planned `Field` absorbs the six field-like classes of `planning/spatial` (the review's §2.5), and `shaping` moves to core with it or earlier (step 8) |
| optimizer-parametric-wiring.md (OPW-1) | the shared backend factory is where the new presets register |
| TODO C4 (identification) | `fit` is the bridge with a data-mismatch objective (§4.6) |
| TODO K (November) | two rows the review found: the evaluator grid trimmed to `_p` bodies plus generated bindings (36 integrator methods, 1763 lines); `StepRollout` folded into `Trajectory` once it is a pytree |

## 7. Decided and proposed

| Item | State |
| --- | --- |
| The architecture: one compiled object per `(problem, controller)`, verbs as its methods, the program as one of them | agreed in conversation 2026-10-17 (the options doc's A, C, F); confirmed by the prototype |
| `Trajectory` array-native and a pytree, reporting verbs NumPy | **decided 2026-10-18** |
| The evaluator simulates the loop as a diagram; its numbers change once | **decided** with RN-5 (RD-D24) |
| Costs read `params`, nested like a diagram's | **decided 2026-09-13** (cost-params.md) |
| Every name in §4 (`compile`, `ProblemEvaluator`, the six verbs, `rollout_p`, `wrt`, `tune`) | proposed; step 0 |
| One scoring kernel without modes; the transcriptions take the cost's rate | proposed |
| The bridge's signature and `sets="hard"` default | proposed |
| The program classes, the first-order backend, the global preset, the default method kept | proposed (S74) |
| The traceable analysis band and the sensitivity peaks | proposed (S73) |

## 8. The steps, bottom-up (one commit each; the AGENTS recipe where behaviour must hold)

Each step records a seeded baseline of every branch it touches before any edit, runs it twice
and `cmp`s, and ends byte-identical unless the step names its one intended change, which gets
its own test and a release note. Every math line reads next to `planning/policy_synthesis/dp.py`
before it lands.

0. **The names, in one conversation before any code.** The package of §4 on the table: the
   facade (`compile` / `closed_loop` / `evaluator`), the noun (`ProblemEvaluator` /
   `PlanningProblemEvaluator` / `CompiledProblem`), the run (`trajectory` / `simulate` /
   `rollout`), the scalar (`cost` / `J` / `score`), the mean (`mean_cost` / `expected_cost` /
   `evaluate(...).mean` alone), the feasibility number (`margin` / `feasibility`), the
   system-evaluator method (`rollout_p` / `simulate_p` / `trajectory_p`), the selection keyword
   (`wrt`, the `jacobian` verb's word / `decision` / `free`), the facade on the program
   (`tune`, or none). The constraints: RULES 5.25's verb families, the existing pairs `compile`
   → `Evaluator` and `solve` → `Solution`, the words the course notes use (`closed loop` 23
   files, `simulate` 20, `failure rate` 3, `margin` 13, `success rate` 0). The decision goes to
   the dated record and to §4.
1. **The native `Trajectory` and the `xp` cost integral.** `core/trajectory.py` (and
   `step_rollout.py`) per §4.2, the pytree registration with the validation-free unflatten and
   the fixed signal order; `CostFunction.evaluate_trajectory` / `total_cost` on `xp`, the
   integrand vectorized (`vmap` on JAX, one call per sample on NumPy), no `float` cast;
   `Set.contains` and `InputSet.contains` derived from `margin` on `xp`. Baseline: every
   `Trajectory` producer and every `compute_cost` / `plot_cost` call in the tests and demos.
   Done when `jax.jit`, `jax.grad` and `jax.vmap` of a function returning a `Trajectory` run,
   `test_core.py`, `test_costs_optimizer.py` and `test_planning.py` pass byte-identical, and
   `evaluate_trajectory` agrees NumPy against JAX to `1e-12` on a recorded trajectory.
2. **The one scoring kernel.** `score_trajectory` on `xp` with the running-sum form of §4.3;
   `Evaluation.of_trajectory`, the transcriptions' `trajectory_cost` and reporting, and
   `MonteCarloEvaluator.evaluate_jax` call it (the last until step 7 removes the class). The
   transcriptions take the cost's discount. Baseline: the trajopt, DP, Monte Carlo and RL
   baselines (`benchmarks/run_regression_check.py --suite all --tiny`) and
   `test_planning_stochastic.py`. Done when the NumPy numbers are byte-identical, JAX agrees to
   `rtol 1e-6` with and without an exit, and DESIGN §6 states the quadrature once with the
   transcriptions citing it.
3. **The whole run out of the scan.** `rollout_p` on `JaxDiagramEvaluator` and
   `NumpyDiagramEvaluator` returning `(t, xs, signals)`; `rollout_batch` gains `signals`;
   `trajectory_of` and `reconstruct_internal_signals` call it. Done when the traced `u` matches
   today's `reconstruct_internal_signals` on a PID loop to `1e-12`, the simulator's
   `rk4_fixedsteps` and the compiled rollout agree to round-off on one diagram (the pinned
   test), and `jax.grad` of a function of the emitted signals runs.
4. **The object, deterministic.** `ProblemEvaluator` with `params`, `trajectory` (`of=`),
   `cost`, `margin`; `problem.compile`; the NumPy backend; the refusals of §4.1; the
   `pid_autotuning_jax` demo rewritten on it with the gradient loop it has today. Done when the
   demo's gradient matches the hand-written scan's on the same gains to `1e-9` relative, the
   pathwise gradient with respect to a controller gain, the plant's mass and `x0` matches a
   central difference (the prototype's three checks, as tests), and `margin` is negative on a
   run that leaves `X` and equals the minimum of the margins on one that does not.
5. **The bridge and the optimizer.** `evaluator.program(wrt=, lower=, upper=)` on the system
   leaves with `program.metadata["unravel"]`, and `tune` if accepted; S74 parts 1 and 2: the
   program classes with the refusals, `optimizers/first_order.py` (`gd`, `adam`, the RL `Adam`
   moved and imported back), the `scipy_lbfgsb`, `scipy_bfgs` and `scipy_differential_evolution`
   presets on the shared factory of OPW-1; the autotuning demo ends in the `Optimizer`.
   Baseline: the RL baselines (the moved `Adam`), `test_costs_optimizer.py`, `test_mpc.py`.
   Done when SLSQP and `adam` reach the gradient loop's gains, `method="auto"` picks by class
   and a refused method names the admissible ones, SLSQP refuses nothing it accepted before,
   the RL baselines stay bit-identical, and the parametric MPC path solves with `ipopt`
   (OPW-4's smoke).
6. **Cost and set parameters.** cost-params.md's dual pattern and nesting (A2); sets reading
   `params` (A5's row) so `params["cost"]` and `params["sets"]` have leaves. Done when every
   planner baseline `cmp`s at nominal parameters, `jax.grad(evaluator.cost)` reaches `Q`, and a
   trajopt objective differentiates with respect to `Q` (A2's done-when).
7. **The draws.** `evaluate`, `mean_cost`, the `Evaluation` pytree; `MonteCarloEvaluator` an
   alias of the new `evaluate` on both backends; `Planner.evaluate`, `solve(evaluate=True)`,
   `nominal_trajectory` and `compare(...).evaluate` rewired; the deterministic-twin test; the
   one intended change of the evaluator's numbers recorded. Done when the pathwise gradient of
   `mean_cost` with respect to the controller's params matches a central difference on the
   same key (V1's done-when), the deterministic twin is byte-identical, and RN-5's LQG loop
   scores within the covariance prediction's sampling error.
8. **Sets as costs, the lowerings, the metrics as constraints.** `shaping.py` to `core/`,
   `Set.as_cost(weight=, shaping=)` (`quadratic_hinge(0.0)` by default, the same scale as a
   quadratic cost); the bridge's `sets="hard" | "soft" | "ignore"`, `constraints=`, `objective=`,
   `wrt="u"` and `"x0"`; the robust-tuning demo (the mean cost over a mass family) and the
   identification demo on the bridge; the margin-floor example once S73 lands. Done when a
   tuning that leaves `X` under `"ignore"` stays inside under `"hard"` and `"soft"`, the
   showcase's shooting cell is `program(wrt="u")`, and `params_gradient`'s identification is
   `program(objective=mismatch, wrt=[...])`.
9. **Material and contracts.** Tutorial 11's last cell and the showcase cells on the evaluator
   (one hand loop kept if the maintainer says so); the compile demos merged by K4 (one per
   feature); DESIGN §4 (`Trajectory`, parameters), §5 (`rollout_p`), §6 (the evaluator, the
   bridge, the program classes); ROADMAP §3's ledger; the alias removed the release after.

**In parallel, any time:** S73 (§4.8), self-contained for a cloud agent. **November
consolidation (K rows):** the evaluator grid trimmed; `StepRollout` into `Trajectory`.

## 9. Asks for the maintainer

Each with the proposed answer; a bare option list is not the ask.

1. **The names** (step 0): the package of §4 as proposed, `wrt=` from `jacobian(of, wrt)`,
   `tune` as the one facade on the bridge.
2. **One scoring contract, no modes.** `score_trajectory` keeps its signature; no
   `constraints=` or `discount=` keywords; the transcriptions take the cost's rate (free of
   baseline risk, §4.3); the uncut integral is the problem without `X`. Confirm.
3. **The evaluator's numbers change once** (the loop as a diagram, RD-D24): the feedback
   planners' `trajectory` and `evaluation` re-pinned in step 7 with a release note. Confirm.
4. **The bridge**: `sets="hard"` by default; `objective=` and `constraints=` as callables of
   `params`; `unravel` on `program.metadata` plus the `tune` facade, or the metadata alone.
5. **The optimizer**: the class table of §4.7; `method="scipy_slsqp"` stays the default and
   `"auto"` is explicit until V3; `scipy_differential_evolution` as the one global preset; the
   RL `Adam` moves to `optimization/` (the RL package imports it).
6. **The verbs dropped**: no `final_state` (`trajectory(params).x[:, -1]`), no `success_rate`
   (`1 − report.failure_rate`), `margin` the exact minimum with the smooth form an option.
7. **Disturbances under `grad`**: held per-step draws from `sample_disturbances` now; seeds in
   the pytree with RN-4, the verb unchanged.
8. **The lane**: research until GMC714 has run it; `problem.compile` is a method, so the root
   prelude gains no name; `ProblemEvaluator` and `Evaluation` on the `minilink.planning`
   facade as provisional names.
9. **The open-loop evaluator** is the plant alone with `u=` (and `wrt="u"`), an evaluation and
   tuning tool beside the transcriptions; the RL environment is not merged into it (its step
   keeps the policy outside the plant, the learner's contract). Confirm.
10. **Which tutorial keeps a hand-written loop**, if any (§5), and whether the one sentence of
    §1 enters the constitution's vision text.
11. **S73**: scheduled beside the above or left as the row it is.
12. **The `CostIntegrator` block** (cost-params.md's research step): closed as a demo idea; the
    kernel on samples is the contract.

## 10. The panel's amendments (2026-10-20)

Seven independent designs and six critiques (the record's last section has the counts) amend
the proposal above. Each item below replaces the earlier text where they conflict.

1. **The pytree is flat; `x0` and the boundary inputs are leaves.** `evaluator.params` is
   `{"controller": …, "plant": …, "cost": …, "sets": …, "x0": …, "r": …}`: the loop's own
   block ids at the root, one leaf per boundary input port holding its held sequence, the
   start as a leaf. Every verb is `(params, key)`; `trajectory` keeps `of=`. A draw is a
   partial override merged over `params`; `wrt` needs no special case.
2. **`compile` wires the reference.** The loop's reference comes from `problem.x_goal` (the
   cost's target as the fallback) and is a leaf; a loop whose reference cannot be set is
   refused with a message. Without this every tuning regulates to zero.
3. **The open-loop case is a `TrajectorySource` policy**, its table in `params["values"]`
   (traceable through `interpolate_schedule`), under `controller`; `wrt="controller.values"`
   is single shooting with the source's input model (held or linear, so it can match
   `ShootingTranscription`); no bare-plant mode, no `u=`. `Xf` lowers at the bridge for a
   source policy only.
4. **`wrt` names leaves, never a whole block**: an explicit list of dotted paths, or the
   leaves a block declares tunable; a library controller's ±inf limits are never ravelled.
5. **The hard lowering stacks every sample's margins** of `X` (and `U` where its box has a
   finite face) as `g(z) ≥ 0`, per trial; `margin` stays the reported minimum; no smooth-min
   option. No `sets=` keyword, no `objective=` or `constraints=` callables in the first cut:
   "soft" is `cost + w * X.as_cost()` on the problem, "ignore" is `replace(problem, X=None)`;
   `bounds` by dotted path. `pack(params, wrt)` and `unpack(z, wrt)` are the evaluator's
   verbs; `program.metadata` stays data.
6. **No `mean_cost`.** `evaluate(params, key, n_trials)` returns the record; the traced
   scalar of the problem's criterion is `value(params, key, n_trials)` or `evaluate(...).mean`
   (which of the two is an ask). The per-trial keys are stored on the `Evaluation` for replay;
   `record=True` keeps one batched `Trajectory`.
7. **Draws depend on the problem and the key only, never on the policy**, so one test set
   scores many laws with common random numbers; `problem.realize(key, n)` draws it once.
8. **`tune(problem, policy, wrt, *, dt, ...)` returns a `PlanningSolution`** (the tuned
   policy, a record, its run, its evaluation), a module verb on the planning facade, on the
   teaching surface by the demo rule. `MonteCarloEvaluator` stays as a thin facade, not an
   alias to retire.
9. **`dt` is explicit** on `compile` and `tune` (or one documented default), never the
   simulator's automatic grid. `rollout_batch` keeps its documented return; the signals come
   from `rollout_p`; `trajectory_of` keeps its recompute path for a trajectory without
   recorded signals. The transcriptions share the discounted integrand, not the kernel's
   running-sum body, so their bits stay. A `Computer @ plant` loop runs on the NumPy path
   (RN-5's scope), it is not refused.
10. **Open for the maintainer** (the panel split): a new noun returned by `problem.compile`
    versus `MonteCarloEvaluator` gaining `objective` and `program` verbs that take the law;
    `Evaluation` as a pytree versus a NumPy report with a traced `value`; the draws before or
    after the bridge; method keys by algorithm or with the `scipy_` prefix.
