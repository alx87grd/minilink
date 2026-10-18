# The differentiable planning problem: `problem.compile(controller)`

Status: draft for the maintainer (2026-10-17, the object map agreed in conversation the same
day; every name provisional until the brainstorm of step 0), step V1 of [TODO.md](TODO.md) §4:
steps 0–3 now, beside K1; steps 4–6 after A2; all before the GMC714 release. Nothing here is
built; §7 lists the asks.
Rung: v0.3.
Lane: research first, the teaching surface once a course has run it.

## 1. Goal

One compiled, differentiable path from the parameters of anything in a planning problem to
the quantities a student or a researcher wants a gradient of:

```python
evaluator = problem.compile(controller, dt=0.01)        # the closed loop, compiled once
params = evaluator.params                               # {"system": {"controller": ..., "plant": ...},
                                                        #  "cost": {...}, "sets": {...}}

x_f  = evaluator.final_state(params)                    # the end state
traj = evaluator.trajectory(params)                     # t, x and u on the plant, y
J    = evaluator.cost(params)                           # the one scoring contract
J̄    = evaluator.mean_cost(params, keys)                # over the problem's draws
p    = evaluator.success_rate(params, keys)             # reported, not differentiable
m    = evaluator.feasibility(params, keys)              # its smooth stand-in

jax.grad(evaluator.mean_cost)(params, keys)             # d J̄ / d everything

program = evaluator.program(["controller.Kp", "controller.Kd"], lower=0.0)
Optimizer(program).solve()                              # gain tuning as an NLP
```

The same `params` pytree feeds `jit`, `grad` and `vmap`, so a parameter family is one call.
Everything above is what `examples/demos/compile/pid_autotuning_jax.py` and the last cell of
tutorial 11 write by hand today, each with its own scan, its own cost formula and its own
params dictionary.

The target this evaluator serves (any traceable metric optimized through the native
optimizer, with respect to any parameter set the problem holds), the table of metrics by
parameter set and the ideas around it: [target-architecture.md](target-architecture.md).

## 2. The objects, today and after

![The planned pipeline: the problem and its params, the compile step, the rollout and scoring kernels around a native Trajectory, the Evaluation, and the NumPy consumers](differentiable-cost-pipeline.png)

*Figure: the end state, from the params pytree to the consumers (`differentiable-cost-pipeline.svg` is the editable source).*

| Object | Today | After |
| --- | --- | --- |
| `PlanningProblem` (frozen) | `sys`, `cost`, `X`, `U`, `X0`, `Xf`, `tf`, `infeasible_cost`, `params: ProblemParameters(system, cost, sets, scene)`; computes nothing | unchanged in shape; `params` is the one pytree every path reads (A2 fills `cost`, A5 `sets`); gains `compile(controller, *, dt, backend="jax")`, a two-line facade |
| `StochasticPlanningProblem` (subclass) | adds `x0_distribution`, `params_distribution`, `disturbances`, `criterion` and the three `sample_*(key)`; every planner takes it, the deterministic ones through `nominal()` | unchanged; its draws are the one source the evaluator vmaps over, and a `params_distribution` draw is merged into `params["system"]` by the evaluator, not by the RL environment |
| `MonteCarloEvaluator` | a verb pretending to be a noun: takes a policy and a plant, composes `policy @ plant` again, compiles again with the controller's params baked into the frozen tier, refuses controllers with state, scores with its own inline copy of the rule | absorbed: `PlanningProblemEvaluator.mean_cost` plus the `Evaluation` record; the name stays one release as a thin alias |
| `PlanningProblemEvaluator` | does not exist | **new**: the closed loop compiled once, pure functions of `(params, x0, key)` (§3); built by `problem.compile`, lives in `planning/evaluation.py` |
| `RolloutEnvironment` | the problem as a step function for the RL collectors: reward `−g dt`, the exit rule, the horizon | unchanged role; its reward and exit call the scoring kernel's pieces, so RL, the evaluator and the planners agree on what an episode means |
| `PlanningSolution` (frozen) | `problem`, `policy`, `solver`, `trajectory`, `evaluation`, `cost_to_go` | unchanged; `evaluation` is what `mean_cost` recorded, reproducible from the problem, the policy and the seed |
| `MathematicalProgram` (frozen) | `J(z)`, `h(z)`, `g(z) ≥ 0`, bounds, `backend`; reached only through a trajopt transcription | unchanged; one more producer, `evaluator.program(decision)` (§3.4) |
| `ParametricMathematicalProgram` | `h(z, x0)`, `bind(x0)`, the MPC block's | unchanged now; later the case "no decision leaves, `x0` bound" of the same bridge |

The `System` evaluators underneath gain one method (§3.2); the costs and sets gain nothing
beyond A2 and A5.

## 3. The design

### 3.1 `PlanningProblemEvaluator`

`problem.compile(controller, *, dt, backend="jax")` builds the loop `controller @ problem.sys`
as `closed_loop` does (a source `>>` for an open-loop plan), compiles it once on the backend,
and returns the evaluator. Its methods are pure functions of `(params, x0=None, key=None)`:

| Method | Returns | Differentiable |
| --- | --- | --- |
| `trajectory(params, x0, key)` | `t (N+1,)`, the plant's `x (n, N+1)` and the command `u (m, N+1)` it received, `y` | yes |
| `final_state(params, x0, key)` | the plant's `x` at `tf` | yes |
| `cost(params, x0, key)` | `J`, the one scoring contract (§3.3) | yes |
| `mean_cost(params, keys)` | `E[J]` over the draws of §3.5, the CVaR under `criterion="worst_case"` | yes |
| `success_rate(params, keys)` | the fraction of draws that never left `X` | no (an indicator) |
| `feasibility(params, keys)` | the mean of `softmin(X.margin)` over the run and the draws, `≥ 0` iff every draw stayed inside | yes |
| `program(decision, *, lower, upper, constraints)` | a `MathematicalProgram` (§3.4) | through the Optimizer |

`params` is the `ProblemParameters` shape as a plain nested dict: `"system"` nests the loop's
blocks exactly as `diagram.params` does, `"cost"` is the cost dictionary of A2, `"sets"` the
sets'. `evaluator.params` is the nominal pytree, so `jax.grad(evaluator.cost)(evaluator.params)`
is the first line a student writes. Partial dictionaries follow the diagram rule (a missing
block reads its own params; an unknown key raises).

`backend="jax"` is the differentiable object; `backend="numpy"` is the same object with the
same methods for a law that does not trace (a lookup table) or a laptop without JAX, the way
`MonteCarloEvaluator` offers both today. The integration is the loop's
`rk4_integrate_zoh_trace_p`: the controller's state is inside `x`, so a PID with an
integrator or an observer-based loop is supported from the first commit, which the present
`static_law` refuses.

### 3.2 The one addition to the `System` evaluators: the whole trajectory

`rk4_integrate_zoh_trace_p` and `rollout_batch` already return the full state history,
`(N+1, n)` or `(B, N+1, n)`. What they drop is the rest of the trajectory: the boundary `y`
and the internal signals, above all `controller:u`, the command the cost scores on the plant.
The parametric port buffer is filled at every step by the private `_compute_port_signals_p`
and thrown away. One method carries it out:

```python
t, xs, signals = evaluator.rollout_p(x0, u_seq, t0, dt, params)   # signals: (N+1, n_signals)
```

the scan emitting the buffer beside `x_next`. The plant's `(x, u)` are slices of it
(`plan.state_index`, `plan.output_slices`): the traced twin of `DiagramSystem.trajectory_of`,
which today is a NumPy loop after the fact. `rollout_batch` gains the same `signals` output.

### 3.3 One scoring kernel

The rule that turns a run into `J` is written three times today and the copies disagree:
`score_trajectory` (NumPy, the reference: trapezoid, `exp(−ρt)`, cut at the first sample
outside `X` with the problem's `infeasible_penalty`, `h(x_f, t_f)` at a finite `tf`),
`MonteCarloEvaluator.evaluate_jax` (JAX, the same rule inline with an `alive` mask), and the
transcriptions' `trajectory_cost` (trapezoid and `h`, **no discount**).

`score_rollout(problem, t, xs, us, params)` in JAX becomes the single owner, with
`score_trajectory` kept as its NumPy twin and a test pinning both to the same number on the
same trajectory, with and without an exit. The exit is a mask, so the kernel traces; a
non-finite state counts as an exit. `evaluate_jax`, `RolloutEnvironment.step` and
`trajectory_cost` call it (the trajopt discount is ask 4).

### 3.4 The bridge to `MathematicalProgram`

Tuning gains is an NLP whose objective is the loop's cost. `evaluator.program(decision, *,
lower=None, upper=None, constraints=(), keys=None)`:

- `decision` names leaves of `evaluator.params` by dotted path, the convention
  `params_distribution` already uses (`"controller.Kp"`, `"plant.m"`, `"cost.Q"`), resolved by
  the same `lookup_param`; a pytree mask is accepted too.
- the chosen leaves are ravelled into `z` (`jax.flatten_util.ravel_pytree`), every other leaf
  is fixed at `evaluator.params`;
- `J(z) = cost(unravel(z))`, or `mean_cost(unravel(z), keys)` on a stochastic problem, the
  keys drawn once at `program()` time and recorded on `program.metadata`, so two solves of one
  program see one sample average;
- `g(z) ≥ 0` from `feasibility`, or a user callable on `trajectory(unravel(z))`; `lower` and
  `upper` as bounds on `z`;
- `backend="jax"`: the `Optimizer` differentiates it as it does any program.

The PID autotuning demo's second half becomes
`Optimizer(evaluator.program(["controller.Kp", "controller.Ki", "controller.Kd"])).solve()`.
`ParametricMathematicalProgram`'s `bind(x0)` is the case "no decision leaves, `x0` bound" of
this bridge, and the "scene bind `J(z, p)`" Later idea is "bound params" on it; both can
follow once the program and the evaluator share the pytree.

### 3.5 Draws, as the problem states them

`mean_cost(params, keys)` is `vmap` over `jax.random.split(key, n)` of one `cost`: `x0 =
problem.sample_x0(k0)`; `system = merge(params["system"], problem.sample_params(k1))`, the
draw a pytree override so a drawn mass and a differentiated gain coexist; the disturbances as
held draws per step from `sample_disturbances`, as the RL environment does. A random block
inside the loop (a `WhiteNoise`) is realized by a seed leaf in its params: `realize(key)` gains
a traced twin that puts `uint32` seeds in the pytree instead of Python ints, which the block's
`random_bits` JAX path already accepts. A deterministic problem gives `mean_cost == cost` with
one key, the twin test of randomness.md's RD-D23. This is RN-5, "the Monte Carlo evaluator as
a batched simulation of the closed loop on its test set", seen from the other side.

### 3.6 The worked example (the autotuning demo, rewritten)

```python
problem = PlanningProblem(sys=Pendulum(), cost=QuadraticCost.from_system(plant, Q, R, xbar=x_up), tf=10.0)
evaluator = problem.compile(ImpedanceIntegralController(Kp=1.0, Ki=0.0, Kd=1.0), dt=0.05)

params = evaluator.params
loss_and_grad = jax.jit(jax.value_and_grad(evaluator.cost))
for _ in range(200):
    J, dJ = loss_and_grad(params)
    params = tree_map(lambda p, g: p - 0.5 * g, params, dJ)

evaluator.trajectory(params)                        # the tuned run, a Trajectory, traceable
```

Twelve lines against today's eighty, the cost from the problem instead of an inline
`err**2`, and the same call differentiates with respect to `Q`, the mass, or a cost weight.

## 4. What exists, verified on `dev` (2026-10-17)

- The loop compiles to a `lax.scan` with `params` threaded (`rk4_step_trace_p`,
  `rk4_integrate_zoh_trace_p`, `rollout_batch` with a params family); no stop-gradient, no
  data-dependent Python branch anywhere on the path.
- `ProblemParameters(system, cost, sets, scene)` is read by the transcriptions, DP, the
  evaluators and the RL environment; the library costs and sets receive `params` and ignore it.
- `x0_distribution.sample(key)`, `sample_params(key)` (dotted names, nested dict, `merge_params`)
  and `sample_disturbances(key)` trace; `System.realize(key)` casts to `int` and does not.
- Every library set's `margin` is native-array and traces; `contains` is NumPy.
- `MonteCarloEvaluator.evaluate_jax` is `jit(vmap(trial))` with the scoring rule inline and the
  law from the frozen tier; its NumPy and simulator backends score with `score_trajectory`.
- `JaxMathematicalProgramEvaluator` takes `jax.grad(J)` and `jacfwd` of `h`, `g` itself.

## 5. What the prototype needs, and what it can skip

| Needs first | Why |
| --- | --- |
| A2, cost parameters | `params["cost"]` is empty without it; the prototype can ship with the system group alone, so A2 first is the cheaper order |
| `rollout_p` on the `System` evaluators (§3.2) | the plant's `u` is what the cost scores |
| the scoring kernel (§3.3) | one contract; also RN-5's core |

| Can wait | Why |
| --- | --- |
| A5, sets reading `params` | `BoxSet` and `BallSet` have no parameters; `params["sets"]` matters for field sets only |
| RN-4 in full | the evaluator draws disturbances as the environment does today |
| AC-7 | the evaluator scores the plant by construction; AC-7 extends that rule to the other tools |
| the teaching surface | research lane until the course has run it; the facade name is the V3 freeze's question |

## 6. Steps (one commit each, the AGENTS recipe where behaviour must hold)

0. **The names, in one conversation before any code.** Every name below is provisional and
   several will be public; the maintainer picks them together, once, so the demos, the
   docstrings and DESIGN say the same thing. What is on the table:

   | Thing | Candidates |
   | --- | --- |
   | the facade on the problem | `problem.compile(controller)`, `problem.closed_loop(controller)`, `problem.evaluator(controller)`, `problem.simulate(controller)` |
   | the object | `PlanningProblemEvaluator`, `CompiledProblem`, `ClosedLoop`, `Rollout`, `ProblemEvaluator` |
   | the run | `trajectory`, `rollout`, `simulate` |
   | the end state | `final_state`, `x_final`, `terminal_state` |
   | the scalar | `cost`, `total_cost`, `J`, `score` |
   | the average over draws | `mean_cost`, `expected_cost`, `evaluate`, `value` (the criterion's) |
   | the two feasibility numbers | `success_rate` / `feasibility`, `failure_rate` / `margin`, `p_success` / `soft_margin` |
   | the method on the system evaluators | `rollout_p`, `simulate_p`, `trajectory_p`, `integrate_zoh_rollout_p` |
   | the scoring kernel | `score_rollout`, `score`, `trajectory_cost` (the transcriptions' name today) |
   | the bridge | `program`, `as_program`, `to_program`, `mathematical_program` |
   | the selection argument | `decision=`, `variables=`, `free=`, `wrt=` (the `jacobian` verb's word) |
   | the nominal pytree | `evaluator.params`, `evaluator.nominal_params`, `problem.params` reused |

   The constraints on the choice: RULES 5.25's verb families (`build_*` constructs,
   `*_of` resolves, `require_*` refuses, `plot_*` draws), the existing pairs `compile` /
   `Evaluator` and `solve` / `Solution`, and the words the course notes already use for these
   quantities. The decision goes to the dated record and to this file's §3.

1. **Kernel.** `score_rollout` in `planning/evaluation.py`; `evaluate_jax` rewired to it; the
   twin test against `score_trajectory` on recorded trajectories, with and without an exit.
   Done when the Monte Carlo baselines `cmp`.
2. **The whole trajectory.** `rollout_p` on `JaxDiagramEvaluator` returning `(t, xs,
   signals)`; the plant slices as a traced `trajectory_of`. Done when the traced `u` matches
   `reconstruct_internal_signals` on a PID loop to float64 precision.
   - **2b. A native `Trajectory`** (decided 2026-10-18). The object keeps the arrays it is
     given, through `array_module`, with no `float` cast and no copy of a traced value, and
     the shape checks unchanged; it is registered as a JAX pytree (`t`, `x`, `u` and the
     signal arrays as leaves, the signal names static), so a trajectory can be a scan output,
     a `vmap` result (a batch is one object with a leading axis) and a `grad` argument;
     `Trajectory.from_rollout(x0, xs, us, dt)` builds one from a scan's time-major arrays with
     the transpose and the grid inside, so the evaluator's `trajectory(params)` returns the
     noun. The reporting verbs stay NumPy by contract and are documented as such (`t0`, `tf`,
     `n_samples`, `nearest_index`, `resample`, `save`, `load`, the plots): called after the
     computation, never inside a traced function. `StepRollout` gets the same treatment.
     Done when `jax.grad` of a function of `evaluator.trajectory(params).x` runs, a NumPy
     caller sees no change (the trajectory baselines `cmp`), and the autotuning demo builds
     no trajectory by hand.
3. **The object.** `PlanningProblemEvaluator` with `trajectory`, `final_state`, `cost`; the
   `problem.compile(controller, dt=)` facade; the autotuning demo rewritten on it. Done when the
   demo's gradient matches the hand-written scan's on the same gains.
4. **Draws.** `mean_cost`, `success_rate`, `feasibility`; the traced `realize`; the
   deterministic-twin test. Done when the pathwise gradient of `mean_cost` matches a central
   difference on the same keys (the V1 "done when"), and `MonteCarloEvaluator(backend="jax")`
   is a call to `mean_cost` plus the `Evaluation` record.
5. **The program bridge.** `evaluator.program(decision, ...)`; the autotuning demo's second
   half solved by the `Optimizer`. Done when SLSQP reaches the gradient loop's gains.
   If S74 (the optimizer as the one descent loop, an idea to review) is accepted, the bridge
   ravels the chosen leaves to `z` and marks the program `stochastic` when keys are resampled,
   and the demo ends in `method="adam"` or the class default rather than SLSQP alone.
6. **Material.** Tutorial 11's last cell on the evaluator; a robust-tuning demo (mean cost over a
   mass family); DESIGN §6 states the contract; `MonteCarloEvaluator` an alias.

## 7. Asks for the maintainer

1. **The facade.** `problem.compile(controller, dt=)` with `backend=` as on `System.compile`
   (proposed), or a standalone verb only?
2. **The names.** The brainstorm of step 0; `PlanningProblemEvaluator` and the method names
   of §3 are placeholders until then.
3. **The integral.** The trapezoid kernel on the samples, shared with the evaluators and the
   NumPy reference (proposed), or the `CostIntegrator` block of cost-params.md (`dJ/dt =
   e^{−ρt} g` integrated with the ODE), whose `J` differs from the post-hoc rule by the
   integration scheme?
4. **The discount in trajopt.** `trajectory_cost` applies none; `score_trajectory` does. Align
   (a behaviour change on every discounted problem's plan) or document the open-loop exception?
5. **Success.** Report the hard rate and offer `feasibility` (a soft minimum of the margins) as
   the differentiable stand-in, or a probabilistic surrogate (a sigmoid of the margin with a
   width the user sets)?
6. **Disturbances under `grad`.** Held per-step draws from `sample_disturbances`, as the
   environment does (proposed for the prototype), or every disturbance a `WhiteNoise` block in
   the loop with a seed leaf, so the diagram is the one place randomness lives?
7. **Teaching surface.** `problem.compile` on the root prelude in v0.3, or research lane until
   GMC714 has run it?
8. **The lane ideas (2026-10-18), to review, not decided.** The verb family on the problem,
   `Evaluation` as a verb of the problem, the open-loop `compile()` covering single shooting
   and the RL environment, the `Optimizer` as the one consumer:
   [target-architecture.md](target-architecture.md) §4. Accept, drop, or fold any of them into
   step 0.
9. **Metrics as constraints.** The bridge of step 5 takes one metric as `J`; the target
   (target-architecture.md §1) wants other
   metrics as `g ≥ 0` (a margin floor while tuning on cost). Design the bridge's signature for
   that from the start (`program(objective=, constraints=(), wrt=)`), or keep `J` only in the
   prototype?

## 8. Architectural recommendations for the bridge and scoring (2026-10-18)

From the big-picture review of the differentiable path ([target-architecture.md](target-architecture.md) §5):

- **Facade and object naming (Step 0):** Prefer `problem.compile(controller=None, *, dt, backend="jax")`
  returning `ProblemEvaluator` (or `PlanningProblemEvaluator`), with `wrt=` as the selection
  keyword on `.program(wrt=...)` to match `sys.jacobian(of, wrt)`.
- **Uncut rollout objective + smooth feasibility margin in `.program()` (Step 5):** Cutting a
  trajectory at the first exit from $X$ (`alive` mask) and charging `infeasible_cost` is exact
  for reporting and `MonteCarloEvaluator`, but produces zero gradient after the exit time and
  no boundary gradient telling the solver how to pull the trajectory back inside $X$. When
  lowering a state-constrained problem in `evaluator.program(wrt=..., constraints=...)`,
  integrate the **uncut** rollout in `J(z)` and lower nontrivial state sets $X$ to the smooth
  inequality constraint `g(z) = evaluator.feasibility(...) >= 0` (`softmin` of `X.margin(x_k)`
  over the rollout, or per-knot margins) so `scipy_slsqp` and `ipopt` receive informative
  boundary derivatives $\frac{\partial\,\text{margin}}{\partial \theta}$.
- **Flat `MathematicalProgram` boundary (Step 5):** Keep `MathematicalProgram` strictly on flat
  vectors $z \in \mathbb{R}^{n_z}$; `evaluator.program(wrt=...)` owns `ravel_pytree` and
  provides `program.unravel(result.z)` (or `evaluator.update_params(result.z)`), keeping pytree
  machinery out of `Optimizer`.
