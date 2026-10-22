# The differentiable planning problem: one path from any parameter to any metric

Status: consolidated proposal for the maintainer (2026-10-18, revised 2026-10-20). One
document replaces the plan of 2026-10-17, its architecture-and-naming analysis and the
target-architecture ideas of 2026-10-18, after an independent review probed each against the
code and a panel of seven independent designs with six critiques amended the result; what each
said and how the splits were judged is in
[the consolidation record](../reviews/2026-10-18-differentiable-problem-consolidation.md).
What is marked **decided** was ruled before this document; everything else is proposed and
waits on the asks of §9. Nothing is built; §8 is the build order, step V1 and row S74
of [TODO.md](TODO.md) §4, row S73 in §5.
Rung: v0.3 wave V, steps 0–6 (the deterministic path to the cost, the API first, beside the
sampled spine of [sampled-path.md](sampled-path.md)); v0.4, steps 7–9 (the draws, with
RN-4 / RN-5); traceable analysis (S73) any time.
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

params = evaluator.params                        # {"controller": …, "plant": …, "cost": …, "sets": …, "x0": …, "r": …}
traj = evaluator.trajectory(params)              # the run, a Trajectory (of=plant: the plant's own x and u)
J = evaluator.cost(params)                       # the one scoring contract
dJ = jax.grad(evaluator.cost)(params)            # dJ / d every parameter: gains, mass, weights, the start
m = evaluator.margin(params)                     # the smallest margin to X and U along the run, ≥ 0 iff feasible
report = evaluator.evaluate(params, key, n_trials=200)     # the draws: J per trial, failed, keys; mean, worst, failure_rate
J_bar = evaluator.value(params, key, n_trials=200)         # the problem's criterion over the draws, one traced scalar

program = evaluator.program(wrt=["controller.Kp", "controller.Ki", "controller.Kd"], bounds={"controller.Kp": (0.0, None)})
result = Optimizer(program, method="scipy_slsqp").solve()  # gain tuning as an NLP; "adam", "gd" after S74
solution = tune(problem, controller, wrt=[...], dt=0.05)  # the same, as a PlanningSolution: print, compare, plot
```

Three nouns do the work, and two of them exist:

- **`Trajectory`**, the run, becomes array-native and a JAX pytree (**decided 2026-10-18**),
  so the compiled rollout, the cost integral and the plots speak one noun.
- **`ProblemEvaluator`** (working name), the problem compiled against a policy: pure
  functions of one params pytree and a key, from the parameters of anything in the loop, the
  cost, the sets or the start to the trajectory, the cost, the problem's criterion over its
  draws and the feasibility margin. `MonteCarloEvaluator` becomes a thin facade on it.
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
| `PlanningProblem` (frozen) | `sys`, `cost`, `X`, `U`, `X0`, `Xf`, `tf`, `infeasible_cost`, `params: ProblemParameters`; computes nothing | unchanged in shape; gains `compile(policy, *, dt, backend="auto", episode_length=None)`, a two-line facade, and `realize(key, n=None)` on the stochastic class (§4.5) |
| `StochasticPlanningProblem` | adds the three laws, `criterion`, the three `sample_*(key)`; every planner takes it through `nominal()` | unchanged; its draws are the one source the evaluator vmaps over |
| `Trajectory` (frozen) | a NumPy reporting object: `float` casts, copies | **decided:** array-native, a JAX pytree; reporting verbs NumPy by contract (§4.2) |
| `StepRollout` | the step twin of `Trajectory` | a November consolidation candidate: one noun with `t = k·dt`, after §4.2 |
| `CostFunction` | `g`, `h` trace; `evaluate_trajectory` is a NumPy `for` loop with `float()` casts | `evaluate_trajectory` / `total_cost` on `xp` (vectorized, no cast); reads `params` with A2 |
| `Set` | `margin` traces; `contains` is NumPy | `contains` derived from `margin` on `xp`, so the kernel and the RL environment share the one exit test; reads `params` with A5 |
| `MonteCarloEvaluator` | a verb pretending to be a noun: composes `policy @ plant` again, recompiles, refuses controllers with state, scores with an inline copy of the rule | a thin facade holding `dt` and a test set drawn once, `evaluate(policy)` delegating to the compiled problem (§4.5); its three backends go |
| `Evaluation` | a NumPy record: `J`, `failed`, `x0`, `trajectories`, `mean`, `worst`, `failure_rate`, `value(criterion)` | the same NumPy report, plus the per-trial `keys` for replay and `trajectories` as one batched `Trajectory` when `record=True`; the traced scalar is the evaluator's `value` verb |
| `TrajectorySource` | an open-loop plan as a block; its table an attribute, `h` a NumPy interpolation | the table in `params["values"]`, `h` on `interpolate_schedule` (traceable), so an open-loop plan is a policy like any other and its samples are leaves (§4.1) |
| `ProblemEvaluator` | does not exist | **new** (§4.1), in `planning/evaluation.py` |
| `RolloutEnvironment` | the problem as a step function for the RL collectors | unchanged role; its exit test is the one `Set.contains`, its reward the same `g` |
| `PlanningSolution` (frozen) | `problem`, `policy`, `solver`, `trajectory`, `evaluation`, `cost_to_go` | unchanged; `trajectory` and `evaluation` of a feedback planner come from the evaluator, reproducible from problem, policy and key |
| `MathematicalProgram` (frozen) | `J(z)`, `h`, `g`, bounds, `backend`, `metadata`; reached through a trajopt transcription | unchanged in shape; one more producer, `evaluator.program(wrt=)` (§4.6); its class read from its fields (§4.7) |
| `Optimizer` | three constrained-NLP presets on a flat `z` | the program classes, a first-order backend (`gd`, `adam`), two free SciPy presets, one global derivative-free preset (§4.7) |
| `tune` | does not exist | **new** facade, `planning/tuning.py`: `tune(problem, policy, wrt, *, dt, ...) -> PlanningSolution` (§4.6) |
| `ParametricMathematicalProgram` | `h(z, x0)`, `bind(x0)`, the MPC block's | binds every problem parameter, `J(z, p)`, `h(z, p)`, `g(z, p)`, the frozen `MathematicalProgram` its binding (S76, [parametric-program.md](parametric-program.md)): the case "no decision leaves, parameters bound" of the same bridge |

The `System` evaluators gain one method (§4.4); nothing else appears or disappears.

## 4. The design

### 4.1 The compiled problem

`problem.compile(policy, *, dt, backend="auto", episode_length=None)` builds the loop as
`closed_loop` does (`policy @ problem.sys` for a feedback block, `source >> plant` for a
`TrajectorySource`: the open-loop plan is a policy like any other, placed under `controller`),
wires the loop's reference from the problem (`x_goal`, else the cost's target, as a leaf; a
loop whose reference cannot be set is refused with a message, since a loop left at its nominal
reference regulates to zero while the cost asks for the goal), compiles it once on the backend,
and returns the evaluator. `dt` is the fixed RK4 step and the sampling grid, always explicit
(the simulator's automatic grid is 1e-4 s on an undeclared loop and is not this tool's); `tf`
is the problem's, or `episode_length` on an infinite horizon (10 s by default, as the Monte
Carlo evaluator has).

Its verbs are pure functions of `(params, key)`: `params` the pytree below, `key` one
episode's draws (a stochastic problem's start, parameters and disturbances; `None` is the
nominal episode). `key` is positional on every verb, so `vmap` maps it.

| Verb | Returns | Differentiable |
| --- | --- | --- |
| `trajectory(params=None, key=None, *, of=None)` | the loop's `Trajectory`: `t`, `x`, its boundary `u`, every internal port as a signal (`"controller:u"`, `"plant:y"`); `of=plant` the plant's own `(x, u)`, the traced twin of `trajectory_of` | yes |
| `cost(params=None, key=None)` | `J` under the one scoring contract (§4.3), the plant's run scored | yes; zero after an exit (§5) |
| `margin(params=None, key=None)` | the smallest margin along the run, `min_k min(X.margin(x_k, t_k), U.margin(u_k, x_k, t_k))`: `≥ 0` iff the run is feasible | yes (piecewise) |
| `evaluate(params=None, key=0, *, n_trials=100, record=False)` | the `Evaluation` over the draws of §4.5 | its statistics on the host |
| `value(params=None, key=0, *, n_trials=100)` | the problem's criterion over the draws (the mean, the worst draw, a CVaR) as one traced scalar; one trial on a deterministic problem | yes |
| `program(wrt, ...)`, `pack(params, wrt)`, `unpack(z, wrt)` | a `MathematicalProgram` and the two sides of its ravel (§4.6) | through the `Optimizer` |

`evaluator.params` is the nominal pytree, flat at the root: the loop's blocks by id exactly as
`diagram.params` has them (`"controller"`, `"plant"`, an observer, a noise block), then
`"cost"` (the cost's dictionary, A2), `"sets"` (the sets', A5), `"x0"` (the plant's start) and
one leaf per boundary input port of the loop holding its held sequence (`"r"` the reference,
`"d"` a disturbance, `"values"` under `controller` for a source policy). A group with nothing
to read is `None`, an empty pytree; a block id that collides with a reserved leaf name is
refused at compile. One namespace, so a drawn start, a disturbance sequence, an identification
input, a tuned reference and a tuned gain are one merge mechanism, and `wrt` needs no special
case. Partial dictionaries follow the diagram rule: a missing block reads its own params, an
unknown key raises. `jax.grad(evaluator.cost)(evaluator.params)` is the first line a
researcher writes; the student writes `tune` (§4.6).

`backend="auto"` is JAX when the loop, `X` and the cost trace on it, NumPy otherwise, with the
fallback announced when it changes the draws; an explicit `"jax"` on a NumPy-only law raises
and says so; `"numpy"` is the same object on the NumPy evaluator (`rk4_integrate_zoh_p`, the
internal signals per sample), one trial at a time, for a lookup-table law or a laptop without
JAX. The integration is the loop's own RK4 on the diagram, so a controller with state (an
integrator, an observer) is inside `x` from the first commit; the Monte Carlo evaluator's
held-input static law goes with it (§4.5). A model predictive controller is refused on JAX
(its inner solve has no implicit gradient) and evaluated on NumPy; a `Computer @ plant` loop
runs on the NumPy path (RN-5's scope, RD-D24) until the hybrid seam batches.

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
- **The transcriptions take the cost's discount and keep their quadrature body.**
  `trajectory_cost` applies no discount today while the contract does; it reads the cost's
  discounted integrand (`e^{−ρ t_k} g_k`, one owner on `CostFunction`) and keeps its own
  `xp.sum` trapezoid, because the kernel's running sum would change its bits (a pairwise sum
  and a sequential one differ in the last place on about half of random grids). No
  trajectory-optimization baseline declares a rate, so every baseline stays bit-identical
  (`e⁰ = 1`), and a discounted problem's plan changes once, as DESIGN §6 says it should. No
  `discount=` keyword: the rate has one owner, the cost (RULES 5.6).
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

the scan emitting the buffer of the step's first stage beside `x_next`; `rollout_batch`
keeps its documented single-array return (tested, used by the README and the showcases) and
a batched twin of `rollout_p` serves the evaluator. The plant's `(x, u)` are slices of the
signals (`plan.state_index`, `plan.output_slices`), the traced twin of
`DiagramSystem.trajectory_of`, which slices when a trajectory carries recorded signals and
keeps its recompute path when it does not (every `Simulator` run). One signal per sample, read at the grid
state as `reconstruct_internal_signals` reads it today, so the plots and the kernel see the
same `u`. The method is written in the trimmed style the evaluator-grid consolidation wants
(one `_p` body, the bound and jitted variants generated), so it does not add a seventh column.

A test pins that the simulator's `rk4_fixedsteps` on one diagram and the compiled rollout
agree to round-off on the same grid, so the traced path cannot drift from what the plots show.

### 4.5 Draws, as the problem states them

The draws are a function of the problem and one key only, never of the policy, so one test
set scores many laws with common random numbers: `problem.realize(key, n=None)` on the
stochastic class draws `n` realizations once (`x0` from `sample_x0`, the parameter draws as a
pytree override, the disturbance sequences per boundary port until RN-4 puts a seed per random
block in the pytree, after which the mechanism changes under the same verb), one key per
experiment split inside, the randomness plan's convention; a deterministic problem realizes
its single start. `evaluate(params, key, n_trials=, record=)` `vmap`s one `cost` over the
realizations and returns the `Evaluation`, a NumPy report converted once at the boundary (lower
late, convert once): `J (B,)`, `failed (B,)`, `x0 (B, n)`, the per-trial `keys` (so
`evaluator.trajectory(params, report.keys[i])` replays any trial with no bookkeeping), and
`trajectories` as one batched `Trajectory` when `record=True` (the fan-of-runs plot without a
second simulation); `mean`, `std`, `worst`, `failure_rate`, `value(criterion)` as today. The
traced scalar is the evaluator's `value(params, key, n_trials)`: the problem's criterion over
the draws (`expectation` the mean, `worst_case` the worst draw, a CVaR at a `risk_level` when
the criterion asks), computed by one shared `criterion_value` the bridge and the sampling
planner read too. A deterministic problem gives `value(params) == cost(params)`, the twin test
of randomness.md's RD-D23.

This is RN-5 seen from the other side (randomness.md: "the Monte Carlo evaluator as a batched
simulation of the closed-loop diagram on its test set", ruled RD-D24): the loop is a diagram,
compiled once, simulated on every realization, the cost scored on the plant. Two consequences
the maintainer already accepted there and this plan restates: the evaluator's numbers change
once (the held-input static law becomes the continuous loop's own RK4, O(dt²) apart on a smooth
law), re-pinned with a release note; and `backend` chooses the arrays only, the `"simulator"`
backend being this path. `MonteCarloEvaluator(problem, dt=, n_trials=, seed=)` stays as the
thin facade that holds `dt` and a test set and whose `evaluate(policy)` compiles the problem
against the policy and calls the verb above, so tutorial 11, `compare(...).evaluate(evaluator)`
and `Planner.evaluate` keep their lines; `solve(evaluate=True)` and `nominal_trajectory` read
the compiled problem.

### 4.6 The bridge to `MathematicalProgram`

Tuning parameters is an NLP whose objective is a metric of the run:

```python
program = evaluator.program(
    wrt=["controller.Kp", "controller.Kd"],   # leaves of evaluator.params by dotted path, or a pytree mask; never a whole block
    bounds={"controller.Kp": (0.0, None)},    # a box on the chosen leaves, by path, lowered to the bounds on z
    key=None, n_trials=None,                  # the draws, realized once here: a sample average two solves share
)
z0 = evaluator.pack(evaluator.params, wrt)     # the two sides of the ravel are the evaluator's verbs
params = evaluator.unpack(result.z, wrt)       # the full pytree with the chosen leaves replaced
```

- `wrt` names leaves, never a block: every library controller keeps its saturation and
  integral limits at ±inf in its params, so a whole-block selection would put infinities in
  `z`; a block may declare its tunable leaves for the facade's default. `"x0"`, a boundary
  port's sequence and a source policy's `"controller.values"` are ordinary leaves, so the
  start, a reference and an open-loop input sequence (single shooting, with the source's held
  or linear input model, the latter matching `ShootingTranscription`) are tuned the same way.
- `J(z) = value(unpack(z), key, n_trials)` on the problem scored with the lowered sets removed
  from the cut: the uncut discounted trapezoid plus `h`. `g(z)` stacks every sample's margins
  of `X` (and of `U` on the plant's command where `U` has a finite face) at every trial, as
  the transcriptions lower theirs (RULES 4.3): smooth per entry, the natural form for SQP,
  cheap with forward-mode Jacobians when `n_z` is small; `margin` stays the reported minimum
  and no smooth-min option exists. `Xf` lowers (a singleton to `h(z) = 0`, a region to its
  terminal margin) for a source policy only; a feedback law approaches the goal
  asymptotically and `Xf` is its reference. No `sets=` keyword and no `objective=` or
  `constraints=` callables in the first cut: a penalty is `problem.cost + w * X.as_cost()`
  composed on the problem, where `print(problem)` shows it; no constraint is
  `replace(problem, X=None)`; a metric objective or a metric constraint (a phase-margin floor)
  comes with S73 and its first consumer. `z` is scaled by the nominal magnitude of each leaf
  (`|p0|` or 1), since SLSQP's tolerances and a quasi-Newton's initial Hessian are absolute and
  gains, masses and weights span decades; the gradient is probed once at `z0` and a vanishing
  one warns.
- `MathematicalProgram` stays flat on `z` and its `metadata` data-only (`wrt`, the keys);
  `Optimizer` never sees a pytree (the constitution: not an optimization modeling language).
- **The facade**, `tune(problem, policy, wrt, *, dt, bounds=None, method="auto", n_trials=None,
  key=0, backend="auto", verbose=False) -> PlanningSolution` in `planning/tuning.py`,
  compiles, builds the program, runs the `Optimizer` and returns the planner family's one
  result: the policy with the tuned leaves written in (a copy; the plant, cost or start leaves
  in the solution's problem), a `TuningRecord` (the method, iterations, `J` before and after,
  the solver record), the tuned run and its evaluation, so `print(solution)`,
  `compare(LQR=..., tuned=...)` and `solution.plot_trajectory()` work with nothing new. It
  passes `method="auto"` while `Optimizer`'s default stays `scipy_slsqp`.
- The same bridge is **identification**: a tracking cost with the logged run as its scheduled
  target, the logged input as a source leaf, `tune(wrt="plant...")` (the C4 row's `fit` is a
  facade on it); equation-error identification stays a direct program on `vmap(f_p)`.
  `ParametricMathematicalProgram.bind(x0)` is the case "no decision leaves, `x0` bound" of
  this bridge, widened by S76 to every problem parameter bound at solve time
  ([parametric-program.md](parametric-program.md)), the scene among them once A3 lands.

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
   and `adam` refuse it ("needs a gradient: a JAX program or `grad_J`"); a gradient-free
   handful of gains (a lookup-table law) may take `nelder-mead`. The new presets are named by
   their algorithm (`l-bfgs`, `bfgs`, `gd`, `adam`, `nelder-mead`, `differential-evolution`),
   the backend being implementation technology (RULES 2.1); the three existing `scipy_*` keys
   stay until V3 rules on their rename (an ask).
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
   OPW-1 (parametric-program.md §6), where the trajopt planner's duplicate registry dies.
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
- **The reference is the problem's.** A loop built by `@` reads its reference at the port's
  nominal value, zero; a cost whose target is the goal then tunes a loop that regulates to
  zero. `compile` wires the reference from the problem (§4.1) and refuses a loop it cannot
  set. Every design on the panel missed it until a critic probed the code.
- **A block is not a leaf list.** `wrt="controller"` would ravel ±inf limits into `z`; the
  bridge takes leaves by path, and a block may declare which of its params are tunable.
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
| parametric-program.md (OPW-1, S76) | the shared backend factory is where the new presets register |
| TODO C4 (identification) | `fit` is the bridge with a data-mismatch objective (§4.6) |
| TODO K (November) | two rows the review found: the evaluator grid trimmed to `_p` bodies plus generated bindings (36 integrator methods, 1763 lines); `StepRollout` folded into `Trajectory` once it is a pytree |
| the step model (S75, [sampled-path.md](sampled-path.md)) | the plant side of every sampled tool: `discretize` on one compiled step; this plan's evaluator keeps the continuous loop and reads the same compiled RK4 |
| the sampled loop (S31, ROADMAP §6) | this plan's evaluator is the continuous closed loop as a diagram (RD-D24) and stays so; the sampled rollout kernel S31 recommends (the controller held over its period, one kernel in two faces) is the sibling path the RL environment, the hybrid simulator and MPC share, and the two meet where §4.2 says "until the hybrid seam batches"; a sampled controller tuned by gradient (a digital PID, a held neural policy) goes through that kernel, not through this evaluator |

## 7. Decided and proposed

| Item | State |
| --- | --- |
| The architecture: one compiled object per `(problem, controller)`, verbs as its methods, the program as one of them | agreed in conversation 2026-10-17 (the options doc's A, C, F); confirmed by the prototype |
| `Trajectory` array-native and a pytree, reporting verbs NumPy | **decided 2026-10-18** |
| The evaluator simulates the loop as a diagram; its numbers change once | **decided** with RN-5 (RD-D24) |
| Costs read `params`, nested like a diagram's | **decided 2026-09-13** (cost-params.md) |
| Every name in §4 (`compile`, `ProblemEvaluator`, the verbs, `rollout_p`, `wrt`, `tune`) | proposed; step 0 |
| One scoring kernel without modes; the transcriptions take the cost's rate and keep their sum | proposed; unanimous on the panel |
| The flat pytree with `x0` and the boundary inputs as leaves; the reference wired from the problem; the open-loop plan a source policy; per-sample margins at the bridge with no mode keyword; `tune` returning a `PlanningSolution` | proposed; a panel majority (the record's counts) |
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
   `rollout`), the scalar (`cost` / `J` / `score`), the criterion over the draws (`value` /
   `expected_cost` / `evaluate(...).mean` alone), the feasibility number (`margin` /
   `feasibility`), the
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
4. **The object, deterministic.** `ProblemEvaluator` with the flat `params`, `trajectory`
   (`of=`), `cost`, `margin`; `problem.compile` wiring the reference; `TrajectorySource` made
   traceable with its table in `params`; the NumPy backend; the refusals of §4.1; the
   `pid_autotuning_jax` demo rewritten on it with the gradient loop it has today. Done when the
   demo's gradient matches the hand-written scan's on the same gains to `1e-9` relative, the
   pathwise gradient with respect to a controller gain, the plant's mass and `x0` matches a
   central difference (the prototype's three checks, as tests), and `margin` is negative on a
   run that leaves `X` and equals the minimum of the margins on one that does not.
5. **The bridge and the optimizer.** `evaluator.program(wrt, bounds=)`, `pack` / `unpack`,
   the per-sample lowering on one trial, `tune` returning a `PlanningSolution`; S74 parts 1 and 2: the
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
7. **The draws** (between cohorts, with RN-5). `problem.realize`, `evaluate` with keys and
   `record=`, `value` and `criterion_value`; `MonteCarloEvaluator` reduced to the facade of
   §4.5; `Planner.evaluate`, `solve(evaluate=True)` and `nominal_trajectory` read the compiled
   problem; the bridge over `n_trials`; the deterministic-twin test; the one intended change of
   the evaluator's numbers recorded. Done when the pathwise gradient of `value` with respect to
   the controller's params matches a central difference on the same key (V1's done-when), the
   deterministic twin is byte-identical, and RN-5's LQG loop scores within the covariance
   prediction's sampling error.
8. **Sets as costs, the lowerings, the metrics as constraints.** `shaping.py` to `core/`,
   `Set.as_cost(weight=, shaping=)` (`quadratic_hinge(0.0)` by default, the same scale as a
   quadratic cost); `wrt="x0"` and the source policy's `"controller.values"` as decision
   leaves with `Xf` lowered; the robust-tuning demo (the mean cost over a mass family) and the
   identification demo on the bridge; the metric objective and the margin-floor example once
   S73 lands. Done when a tuning that leaves `X` on `replace(problem, X=None)` stays inside on
   the problem and with `X.as_cost()`, the showcase's shooting cell is
   `tune(wrt="controller.values")`, and the identification demo is `tune(wrt="plant...")` on a
   tracking cost.
9. **Material and contracts.** Tutorial 11's last cell and the showcase cells on the evaluator
   (one hand loop kept if the maintainer says so); the compile demos merged by K4 (one per
   feature); DESIGN §4 (`Trajectory`, parameters), §5 (`rollout_p`), §6 (the evaluator, the
   bridge, the program classes); ROADMAP §3's ledger; the alias removed the release after.

**In parallel, any time:** S73 (§4.8), self-contained for a cloud agent. **November
consolidation (K rows):** the evaluator grid trimmed; `StepRollout` into `Trajectory`.

## 9. Asks for the maintainer

Each with the proposed answer; the panel's splits come first. **Before any code** (they shape
step 0 and the first commits): 1, 2, 5, 6, 7 and 8. **Before the step they name**: 3 (step 7),
4 (step 5), 9 (step 9). **Any time**: 10, 11, 12, 13. Taking an ask: the ruling goes to §4 or §8
here and to a dated record under `docs/reviews/`, and the ask leaves this list and ROADMAP §6.

1. **The noun.** `problem.compile(policy) -> ProblemEvaluator` (five designs of seven; the
   `System.compile` precedent), with `MonteCarloEvaluator` kept as the thin facade that holds
   a test set, rather than no new noun and the facade gaining `objective` and `program` verbs
   that take the law (two designs). The draws are shared either way (§4.5).
2. **`Evaluation`**: a NumPy report with keys and a traced `value` verb (proposed: lower late,
   convert once; one spelling of the traced scalar), rather than a pytree with traced
   statistics (three designs each).
3. **The order**: the deterministic steps and the bridge on one trial now; the draws and the
   facade's Monte Carlo form between cohorts with RN-5 (proposed, the ruled order), rather
   than the draws before the bridge (three designs).
4. **Method keys** by algorithm, the existing `scipy_*` keys kept as aliases until V3
   (proposed), or the prefix kept throughout.
5. **The names** (step 0): the package of §4 as proposed, `wrt=` from `jacobian(of, wrt)`;
   `tune` the one facade; `value` the traced criterion verb.
6. **One scoring contract, no modes**: `score_trajectory` keeps its signature; the
   transcriptions take the cost's rate and keep their own sum. Confirm.
7. **The evaluator's numbers change once** (the loop as a diagram, RD-D24): re-pinned in step 7
   with a release note. Confirm.
8. **The verbs dropped**: no `final_state`, no `success_rate`, no `mean_cost`; `margin` the
   exact minimum, the per-sample margins the bridge's rows.
9. **The lane**: `tune` on the teaching surface by the demo rule; `compile`, the class,
   `program`, `pack` / `unpack`, `margin`, `value` provisional on the planning facade until
   GMC714 has run them.
10. **Which tutorial keeps a hand-written loop**, if any (§5), and whether the one sentence of
    §1 enters the constitution's vision text.
11. **S73**: scheduled beside the above or left as the row it is.
12. **The `CostIntegrator` block** (cost-params.md's research step): closed as a demo idea; the
    kernel on samples is the contract.
13. **S31 before step 0**: the sampled-loop recommendation (ROADMAP §6: the loop as a tool,
    one sampled rollout kernel in two faces, tracing decided by the computer) taken before
    the first commit, so that kernel and this evaluator are designed once at the seam (§6).
