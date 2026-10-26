# Independent review B: the v0.3 core, its draft API and its names

**Date:** 2026-10-26 · **Reviewer B** · **Emphasis:** the draft API and naming (the names a
user writes in the final core, their signatures, the shape of a script) and the id/label
system of the docs; the vision, the plan and the draft architecture are covered through that lens.
**Status:** an independent review, not a ruling. Nothing here changes a decision; the
maintainer rules.
**Commit reviewed:** `ce082e4474f42f8433bd62e994ea8a7e977f43a4` (`dev-v0.3`, level with `origin`).

## Top summary

- **The spine is right, and the landed half reads well.** Compile, roll out, score, export a
  program, with one `params=None` rule and one parameter tree that `jax.grad` reaches, in a
  five-line script. Make that the bar for every pillar.
- **The parametric program should not be a second noun.** The maintainer's own words ("a
  `MathematicalProgram` compiled with an explicit, optional `params` argument") give the
  cleanest design: `J(z, params=None)`, as on a `System`. No `ParametricProgram`, no `bind`.
  `bind` already means three things in the code.
- **"Evaluator" names 14 classes doing four jobs.** The discrete judge must not add a
  fifteenth; keep it inside `pe.evaluate`.
- **Keep the hybrid loop's "two runners" out of the user's script:** one verb,
  `loop.compute_trajectory(solver=...)`, on the R15 precedent.
- **The discrete problem can be the same `PlanningProblem` on a `StepSystem`**, with dispatch
  on the plant's kind. No new class, and grid worlds become stateable.
- **Three keyword families are overloaded:** `solver` / `method`, `backend` /
  `compile_backend`, and the sample period (`sample_period`, `dt`, `dt_mpc`, `dt_base`). Settle
  one rule before phase 1 writes new signatures.
- **The id system costs more than it saves:** about 280 ids in 23 families, one id per 35 words
  in core-architecture.md, and three collisions (`R1`, `D3`, `V9`/`V10`). Use slugs for open
  rows, keep R-ids in `docs/reviews/`, and give each pillar one number.
- **Plan docs keep stale rungs behind disclaimers.** Remove rungs from plan docs; TODO is their
  one home.
- **The plan risks a long-lived branch.** Merge the landed continuous core now (the merge into
  `dev` after review, R79), then merge after each phase.

## 1. Strengths worth keeping

1. **One rule for parameters.** `params=None` reads the snapshot, and a tree overrides it.
   `pe.params` is one tree of gains, plant, cost and start, and
   `jax.grad(lambda p: pe.evaluate(p).objective)(pe.params)` is a textbook line. Judge every
   pillar by whether it keeps this.
2. **Teaching verbs named by what they return** (the R27 principle): `evaluate`, `compare`, the
   `Evaluation` record, `problem.replace`. The RL demos read
   `evaluate(problem, ppo_ctl, dt=DT, n_trials=100, key=1)`.
3. **The randomness convention.** A pure noise block with a seed as an integer leaf, a
   counter-based draw that is the same on both backends, and `realize(key)` by name. It is what
   makes "the stochastic program is the deterministic one over seeds" true and cheap.
4. **`Discretization` as the precedent.** A step object that keeps its continuous source is
   the right pattern for `computer @ plant` and for the discrete problem.
5. **The decision trail.** Options per question, dated records, byte-identical refactors.

## 2. Problems

**P1. "Evaluator" is one word for four jobs.** The code has 14 `*Evaluator` classes
(`grep "class .*Evaluator"`):
- a compiled model (`DynamicEvaluator`, `StepDiagramEvaluator`, …: `f`, `step`, `rollout`);
- a closed loop scored on a task (`ProblemEvaluator`);
- a fixed policy's cost-to-go on a grid (`PolicyEvaluator`, from dynamic programming);
- a compiled program (`JaxParametricProgramEvaluator`, `MathematicalProgramEvaluator`).

There is also the `evaluate` function, `pe.evaluate`, the `Evaluation` record and
`evaluate_trajectory`. The discrete-judge pillar then plans "one discrete evaluator"
(core-architecture §3).
*Why it matters:* a student reading `ev = plant.compile()` beside `pe = problem.compile(pid,
dt)` gets two "evaluators" with no shared method.
*Suggestion:* reserve "evaluate" for scoring against a problem. The judge is the body of
`pe.evaluate`, with no public class. The compiled-model classes are research-lane; consider
`CompiledSystem` at the v0.9 name pass. `PolicyEvaluator` keeps its textbook name ("policy
evaluation" in dynamic programming).
*Cost:* zero now (a rule about what not to add). The rename is a v0.9 decision (the frozen
surface, V3).
*Touches:* the discrete judge (TODO §1.3); the v0.9 names question.

**P2. `bind` is already overloaded, and the plan adds a fourth meaning.**
- `minilink.planning.bind(sys, geometry, frame="body")` mounts collision geometry on a plant;
  it is public, on the teaching facade.
- `ParametricProgramEvaluator.bind(x0)` binds the measured start.
- An RL algorithm's `bind(policy, critic, gamma)`.
- The plan adds `ParametricProgram.bind(p)` and `evaluator.bind(p)` (parametric-program §2).

*Suggestion:* drop `bind` from the program API entirely (see O1). Rename the spatial one to
the mechanical verb it is: `mount(car_outline(), on=car, frame="body")`.
*Cost:* `mount` is student-facing (the track demos), so it needs the maintainer's yes. Dropping
`bind` from the program design costs nothing, since nothing has landed.
*Touches:* the parametric program (S76, PP-1–PP-4); QUESTIONS "one family".

**P3. Keyword overloads the new signatures would copy.**
- `solver=` is the integrator in `problem.compile(…, solver="rk4")`, `ev.step` and
  `discretize`. It is the NLP solver in prose, and `PlanningSolution.solver` is a record.
- The plan docs write `evaluate(..., method="scipy")` (sampled-path §8.4,
  differentiable-problem §9.1, TODO §1.3, core-architecture §3), but the code's keyword is
  `solver=`, while `method=` is the `Optimizer`'s algorithm.
- `backend=` means the array module on `problem.compile` but `compile_backend=` on
  `Optimizer` and the planners. `Optimizer.backend` is the solver adapter.
- The sample period is spelled `sample_period` (`WhiteNoise`), `dt` (`%`, and D29's
  `GaussianPolicy`), `dt_mpc` (the MPC) and `dt_base` (`StepSchedule`). DESIGN §4 insists
  these are three different periods.

*Suggestion:* one rule in DESIGN §4 before phase 1: `backend=` is the array module, `solver=`
the integrator, `method=` the optimization algorithm, `sample_period=` a block's period and
`dt=` a tool's grid. Fix the plan docs' `method="scipy"` now (docs only). *Cost:*
`compile_backend` → `backend` and `dt_mpc` → `sample_period` rename frozen names (G3 froze the
MPC constructor, R50): the maintainer's call, one sweep (R82).
*Touches:* every pillar; the one-owner-per-rule row (S65).

**P4. Three `realize`s with three shapes.**
- `System.realize(key)` returns a params tree.
- `PlanningProblem.realize(key=None, n=1)` and the stochastic one, `realize(key, n)`, return
  `(x0s, {"params", "disturbances"})`.
- `ProblemEvaluator.realize(params, key, n_trials)` returns `(x0s, Batched)`.
- CONSTITUTION-level text says "a realization is `(x0, params)` and nothing else"
  (core-architecture §2.1).

*Suggestion:* the draws pillar's target is `problem.realize(key, n) -> (x0s, params)`, with
`params` the full batched tree. The `"disturbances"` key disappears once disturbances are
seeded blocks (RN-4). Use `n` or `n_trials` everywhere, not both: `n_trials` matches
`evaluate`.
*Cost:* a small change, and it lands with the noise seeds anyway.
*Touches:* the draws (TODO §1.4, V1 step 7).

**P5. Two verbs on the hybrid loop with different numbers.** sampled-path §8.5 keeps
`compute_rollout` (the pure step semantics) and adds `compute_trajectory` (dense, possibly
adaptive) on the same object. "Differ by the integration error only" is a trap for a student
comparing plots to `evaluate`'s `J`. The `StepRollout` fold also makes `compute_rollout`'s
return type a `Trajectory`, which leaves the two verbs nearly synonyms.
*Suggestion:* one verb for every kind once the fold lands, with the run chosen by argument
(O3).
*Touches:* the hybrid loop (§1.5), the fold (§1.2), QUESTIONS "which runner by default".

**P6. Plan docs carry rungs and disclaimers instead of facts.** Rungs that are wrong but
"explained": randomness.md ("all in v0.4", then "the rungs written in this doc are the ones
before that direction"), parametric-program.md lines 3–21, future-policies §5.1 and §6,
native-optimizers §3.1, §4.6 and §5, and differentiable-problem §9.4 point 6 ("both v0.4").
The `ParametricProgram` name is credited to R26 in some places and to D19 in others, and
R26's text does not contain it. This breaks "one fact, one home".
*Suggestion:* a plan doc states no rung; it links its TODO row. A docs-only sweep, which
agents can do.

**P7. The plan's scope against a long-lived branch.** v0.3 now has 9 phases (11 counting
6a–6c). Three rows are design-open with "Done when: written with the ruling". Phase 0 is 23
open questions, and there is no date, while class material stays on `main`. The landed
continuous core still waits for its review (the merge into `dev`, R79).
*Suggestion:* review and merge the landed core into `dev` first (accept the overnight run's
decisions as a block, §5 below). Then merge after every phase that ends green. Start phase 1
now: of its rows, only the `as_cost` home is open.
*Touches:* the order (core-architecture §4), the release question.

**P8. Counting layers.** Eight items, seven pillars, two foundations, nine TODO subsections,
phases 0–8 with 6a–6c, and ROADMAP §5's "six pillars" (the application ones). Each doc
counts differently; §4 has the fix.

## 3. Opportunities and improvements to the ideas

**O1. One program noun, with `params` like a system.** The maintainer's direction, quoted in
the consolidation record §1, says the program is "compiled with an explicit, optional `params`
argument". Take it literally:

```python
# before (parametric-program.md §2, option C)
parametric = transcription.transcribe_parametric(problem)
evaluator.bind({"x0": y, "plant": {"mu": mu_hat}})
program = parametric.bind(p)                       # a second noun, a fourth `bind`

# after: one noun, the CONSTITUTION's "one signature"
program = planner.program()                        # MathematicalProgram with .params
Optimizer(program).solve(params={"x0": y, "plant": {"mu": mu_hat}})
MathematicalProgram(n_z=3, J=J)                    # J(z): params=None, unchanged for students
```

Option B (parametric-program §2) was rejected because "a student writes `J(z, p)`". With
`params` optional, the student does not: the program wraps a one-argument `J`. *Cost:* it
retires a name whose ruling cannot be found, so a fresh ruling is needed either way. *Risk:*
low; host solvers close over `params` per solve.
*Touches:* the generic parametric program (S76); QUESTIONS "one family" and "how do programs
take parameters".

**O2. The discrete problem is a `PlanningProblem` on a `StepSystem`.**
`problem.discretize(dt, solver="rk4", substeps=1)` returns the same class, with `sys` a
`Discretization` (which already exists), the cost per step `c(x_k, u_k, k)` and horizon `N`,
and `compile` and `evaluate` dispatching on the plant's kind. This is the R105 dispatch
applied to the plant. A grid world or a natively discrete plant is then stated directly. Today
`problem.compile` refuses a discrete-time plant. If a separate class is preferred, name it
`StepPlanningProblem` for the `Step*` family, not "discrete" or "judge". *Risk:* `tf`, `ρ`
and `g` read differently by kind. That has to be stated in DESIGN §6, the way `h` is already
stated twice.
*Touches:* the discrete judge (§1.3), QUESTIONS §9.4 point 3.

**O3. The runners are an argument, not a user concept.** R15's continuous split already gives
the shape:

```python
loop = pid % 0.01 @ plant                  # a SampledDataSystem (a StepDiagramSystem)
traj = loop.compute_trajectory()           # the step-diagram run, substeps recorded
traj = loop.compute_trajectory(solver="scipy")   # the orchestrator, adaptive plant
pe = problem.compile(pid % 0.01, dt=0.01)  # the same pure tick, scanned
```

The clock runner stays behind the realtime verbs.

"Pure step diagram" and "clock orchestrator" stay internal names.
*Touches:* the hybrid loop (§1.5).

**O4. The exported program inverts itself.** The PID closing example writes `wrt` three
times and threads `start` through `pack`/`unpack`:
`pe.unpack(best.z, wrt, start)` (`pid_autotuning_jax.py`, lines 181–213). The program already
stores `metadata["wrt"]`, so:

```python
program = pe.program(wrt=["controller.Kp", "controller.Ki", "controller.Kd"],
                     params={"controller": {"Kp": 1.0, "Ki": 0.1, "Kd": 0.2}})
best = Optimizer(program).solve()
print(pe.evaluate(program.params_at(best.z)))      # the tuned tree, wrt read from the program
```

*Cost:* one method. It is additive.
*Touches:* the landed program export (the rest of V1); the parametric program.

**O5. Name the conversions by what they return.** `Set.as_cost` (planned) and `Field.as_cost`
return a `CostFunction`. `Field.as_constraint` returns a `Set`, and `as_input_constraint` an
`InputSet`. Symmetric names, `as_set(upper=level)`, `as_input_set(lower=0)`, `as_cost(...)`,
make the CONSTITUTION's nouns the method names: "a sublevel set is `V.as_set(upper=c)`".
*Cost:* `as_constraint` is in the track demos, so it is student-facing; ask.
*Touches:* Fields (§1.8, A1); QUESTIONS "where `as_cost` lives".

**O6. The discrete judge's refinement is the right one; keep it internal.** Integrating the
cost with the state (`ċ = e^{−ρt} g`) gives `c_k` at the scheme's accuracy, and it is exactly
the textbook augmented state. Implement it inside the step the judge scans, not as a change to
the `StepSystem` contract.

## 4. Naming suggestions

**Public names** (keep / change; the cost column marks the asks):

| Today or planned | Suggestion | Why | Cost |
| --- | --- | --- | --- |
| `problem.compile(ctl, dt)` → `ProblemEvaluator` | keep both | `compile` matches `System.compile`; still provisional (R28), so cheap to revisit | — |
| `ParametricProgram`, `bind(p)` | none: `MathematicalProgram(..., params=)` | O1; `bind` collides | ruling |
| `minilink.planning.bind` | `mount(geometry, on=sys, frame=)` | a mechanical verb; frees `bind` | student-facing |
| `HybridDiagram` (to become a `StepDiagramSystem` subclass) | `SampledDataSystem` | the textbook's term (Åström and Wittenmark); the docs already say "the sampled-data loop's stability" | with the amendment |
| `Computer`, `%` | keep | *Computer-Controlled Systems* is the textbook; `%` is ruled and reads "sampled at" | — |
| `StepPlanningProblem` (if a class) | the same `PlanningProblem` (O2) | no new noun | — |
| `NoiseSource(law, Δ)`, `GaussianPolicy(…, dt, …)` | keep the names; `sample_period=` on both, as on `WhiteNoise` | one keyword per concept (P3) | — |
| `NeuralPolicyController`, `LookupPolicy`, RL `StochasticPolicy` | suffix rule: a law `u = π(x)` is a `…Policy`; `NeuralPolicy` at the v1.0 rename window; rename the RL-internal `StochasticPolicy` so the public `GaussianPolicy` owns the idea | three suffixes for one role | v1.0 list (V4) |
| `Field` | keep; docstring "scalar field" | `f` is the textbook's vector field | — |
| `as_constraint` / `as_input_constraint` | `as_set` / `as_input_set` | O5 | student-facing |
| `compute_rollout` / `plot_rollout` | fold into `compute_trajectory` / `plot_trajectory` with the `StepRollout` fold | one verb per job | sweep, R82 |
| `Optimizer.solve_traced` (sketched, native-optimizers §4.5) | `solve(params=...)`, traced when the program is | CONSTITUTION §4.5: name by role, not technology | — |
| `live=["plant.mu"]` (MPC, D25) | `parameter_ports=["plant.mu"]` | says what appears: ports | — |

**The id and label scheme.** The ids were built to be precise, and they are. But the reader
pays for that on every line:
- about 280 distinct ids across ROADMAP, QUESTIONS and the plan docs: R ×419 mentions, S ×312,
  RN ×189, D ×143, RD-D ×122;
- ids on ids: "V1's step 7b", "the rest of RN-5", "NS-2's fixed-`K` form", "K8-3", "D1.3";
- three collisions:
  - the plans README calls `R1` a step id, but R1–R123 are rulings;
  - `D1`–`D3` (wave-D rows) share a namespace with `D13`–`D31` (decisions), as in TODO line 249,
    "absorbs that D3 row";
  - ROADMAP's mermaid uses `V4`, `V9` and `V10` as nodes for v0.4, v0.9 and v1.0, while `V4`,
    `V9` and `V10` are also the deprecation, labs and public-face rows.

A simpler scheme, each step docs-only and agent-managed except where noted:
1. **Open workboard rows get slugs**, not numbers: `noise-seeds`, `sampled-face`,
   `step-operators`. A slug builds the plain-words-first rule into the id and cannot collide
   with a release. Old ids stay in the records, with one mapping table in TODO.
2. **One number per pillar:** phase *N* = core-architecture §*N* = TODO §1.*N*. Then "the
   draws (§1.4)" is the same pointer everywhere, and the "eight items / seven pillars" count
   goes. Fold the seeded policy into the draws or give it its own number, but only one of the
   two.
3. **Rulings live in `docs/reviews/`.** Live docs cite them by a short name and a link ("the
   complete-tree rule, 2026-10-25"). The R-id stays as the anchor inside the record.
4. **Plan-local prefixes** (parametric-program's PP and OPW, native-optimizers' NS,
   randomness' RD, MPPI's MP) stay inside their own doc. TODO and QUESTIONS cite "steps 1–5 of
   parametric-program.md".
5. **Retire the `D` decision series** now that end-state-api.md is gone. Each entry has a home
   section and a slug.

*Cost:* one mechanical sweep, docs only. *Risk:* external links to anchors; check the course
documents before the sweep.

## 5. Recommended answers to "Blocks v0.3"

A reviewer's opinion, one line each, in QUESTIONS order.

| Question | Recommendation |
| --- | --- |
| Which release for the borderline rows | The derived start (S29) joins phase 1: a controller's initial state read from `loop.x0` is not a parameter. A minimal error family (S70) lands before the hardening that raises it. The realtime re-base waits for v1.0's review. The RL heads go to v0.4; the core ships the block and one test. |
| Same-tick semantic | Propagation within the tick (the textbook block diagram, algebraic loops refused). Latency is an explicit unit-delay block, and the `Computer`'s hidden one-tick lag is removed. That matches the computational-delay ruling (R123): delay lives in state. |
| Operators on step systems | Yes, the same `@`, `>>`, `+`. It is the same grammar on the sibling algebra, not a second dialect; mixing kinds raises. Needed for `computer @ plant` as a step diagram. |
| Unify from the rollout on | Yes. Per-step cost by the augmented state (O6); margins at samples for v0.3; the object as in O2. |
| Failure rule, hold, scope | `score_run`'s rule (to the horizon, `X` and `U` at every sample, the price once); the RL early stop is a training choice, like the training zone. Zero-order hold. Shooting's score lowered to the judge. The planners' fixed-`dt` runs stay evidence. S63 in v0.4. |
| Standard stochastic policy | `GaussianPolicy(mean, log_std, *, sample_period, seed=None)`, with `log_prob(x, u, t, params=None)` and `entropy(x, t, params=None)` on the one signature; squashed as a subclass. |
| Draw convention | One: the tick, `k = ⌊(t − t₀)/Δ⌋`, stated once. A continuous block computes `k`; a step block receives it. |
| `computer @ plant` a step diagram | Yes, with the amendment as drafted: the coupling is a tool, and its result is a `System` of the step algebra. Name it `SampledDataSystem`. |
| `compute_trajectory`'s default runner | The step-diagram run with its substeps recorded: the same numbers as `evaluate`, and today's density. `solver="scipy"` selects the adaptive orchestrator (O3). |
| Run caches | They go (RULES 2.4). |
| `problem.compile` and the stateless MPC | Refuse it now, naming the hybrid loop. A latch running at the RK4 stages is a silent wrong answer. |
| One program family | One noun, `MathematicalProgram` with `params` (O1). A hand-written program may declare `params`. The MPC takes `parameter_ports=`. |
| How programs take parameters and randomness | Pass, do not bind. Merge partial trees at the program boundary (like `ProblemEvaluator.merged`), complete inside (R121). The seed is the `seed` key; integers never enter `wrt`. Shapes on the constructor, values in `params`; warm start on `solve`; sensitivities in v0.4. Drop `J(z, key)`. |
| Kernel shape | B, the pure kernel with a step-system view. The view arrives with the teaching face (NS-5). |
| Which tick first | The gradient MPC, as directed. The sampled kernel's first consumer is the digital PID. Update native-optimizers §4.6b, which still says MPPI comes first. |
| `as_cost` and shaping | One home, `core/shaping.py`. `Set.as_cost` lands first in phase 1 as "the margin's field, shaped"; `Field.as_cost` reuses it in phase 7. |
| Compile-time flatten default | Wait for the benchmark. It does not block v0.3; move it to "Standing". |
| The overnight run's 16 decisions | Accept them as a block after the three number changes the log names. This unblocks the merge (P7). |
| Three hand-loop demos | Convert two; keep the schedule demo's hand loop until a source's samples are parameters. Your teaching call. |
| LQR and a declared `U` | No warning (amend R21). Write "U ignored" in the Riccati record instead: a warning that fires on 90 correct lines teaches students to ignore warnings. |
| `DynamicBicycle`'s base | Rename only; the base moves with the catalog clean-up (K8). |
| Price follows cost overrides | Yes, with the cache keyed on the cost's parameters. Otherwise `pe.program(wrt=["cost…"])` prices a different cost from the one it tunes. |
| Three demos' style | Your demo pass (K4). Fix the `NoisyPendulum()` shown as `Pendulum()` now: it is a wrong label, not style. |
