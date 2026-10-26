# Independent review A: the vision and the architecture of the v0.3 core

**Date:** 2026-10-26 · **Reviewer:** Reviewer A · **Emphasis:** the vision and the architecture
(coherence with the CONSTITUTION, simplicity, the pillars and foundations, what is missing,
over-built or premature, the risk of a framework) · **Status:** an independent review, not a
ruling · **Commit reviewed:** `ce082e4474f42f8433bd62e994ea8a7e977f43a4` (`dev-v0.3`)

Read: AGENTS, CONSTITUTION, core-architecture.md (whole), ROADMAP §1 and §5, TODO §1–§2,
QUESTIONS "Blocks v0.3", DESIGN §4–§6, the plan docs core-architecture cites, the 2026-10-26
consolidation record; code checked in `planning/`, `core/`, `simulation/discrete/`.

## Top summary

1. **Keep the bar you already have.** The CONSTITUTION and the landed continuous core are the
   project's best assets: one step model, one score, one small differentiable path
   (`problem.compile` → `evaluate` → `program` → `Optimizer`).
2. **v0.3 has become one large release with no intermediate merge.** It has two foundations,
   seven pillars and eight phases, nearly every row is maintainer-owned, several rows are "done
   when: written with the ruling", and the merge into `dev` sits in the last phase. Merge the
   landed core now. Treat three pillars as *acceptance consumers* of the core, not parts of it.
   Build one thin end-to-end slice before the layers.
3. **The plan is less simple than the code.** About 14,000 lines of plans and 5,000 of records
   steer about 68,000 lines of library. One pillar's status is restated in up to seven places,
   and the text uses about twenty id prefixes. Apply CONSTITUTION §1.3 to the plan itself.
4. **The framework risk sits in the hybrid loop.** The drafts carry three layouts, two runners
   plus an adaptive path, three resolution options and a deployment generator. v0.3 needs one
   pure kernel.
5. **The single-score pillar should not add a new noun.** Integrate the running cost with the
   state in the step model. Under forward Euler this reproduces value iteration's and RL's
   `g·dt` exactly, so one rule replaces three. The "discrete problem" is then a
   `PlanningProblem` whose system is a `StepSystem`.
6. **The parametric program already exists, as `pe.program(wrt)`.** Make it the one family.
   Move the parameter-tree helpers, copied in four modules today, into one core module.
7. **Two inconsistencies in the CONSTITUTION should be fixed before more code:** disturbances
   as bare laws, and "foundation" and "pillar" each naming two different things.

## 1. Strengths worth keeping

- **The CONSTITUTION is unusually precise.** `f(x, u, t; p)` is at the centre, and "One
  signature", "Lower late" and §1.4's boundaries can be applied mechanically. Most findings
  below are the CONSTITUTION applied to the plan.
- **One step model under every sampled consumer** (`stepping.py`, DESIGN §5). This is what
  makes the single-score idea cheap.
- **The problem evaluator's API is small and textbook.** `pe.params` is one tree, `pe.evaluate`
  returns a pytree `Evaluation`, and `pe.program(wrt)` exports a plain `MathematicalProgram`
  (DESIGN §6). Numeric-only `params` (R118) and complete-tree overrides (R121) keep tracing
  honest.
- **Randomness is designed from first principles.** Draws come from a counter-based cipher, the
  same bits on NumPy and JAX. The noise block is pure, and one key reproduces a run
  (randomness.md, "The design in one screen"). "The seed is a parameter" then unifies
  simulation, policies and programs.
- **The analysis behind the two hybrid runners** (sampled-path.md §8.2) is excellent. It frames
  the choice as *semantics versus execution* and makes the step diagram the test oracle for the
  orchestrator. The MPC's one-period computational delay (R123) is the honest textbook model.
- **The native-optimizer split is right** (native-optimizers.md §3.3): an Optax-shaped pure
  kernel, with the step-system loop as a teaching view and not a second implementation.

## 2. Problems

**2.1 The v0.3 scope ships nothing until everything is done.**
*Seen:* ROADMAP §1 and §5.3 and TODO §1 put fifteen-odd rows under "[ask — core]". The
discrete-problem row, the two-runner row and the stochastic-program row have no "done when"
beyond the ruling (TODO §1.3, §1.5, §1.6). The merge of `dev-v0.3` into `dev` is in phase 8
(TODO §1.9). The continuous core has waited for review since 2026-10-24, with 16 overnight
decisions still open.
*Why it matters:* a long-lived branch without a date, while the course material stays on
`main`, is the classic way a rewrite stalls, and every phase moves the numbers again.
*Suggest:* (a) review and merge the landed continuous core into `dev` now; R79 does not tie
that merge to the later phases. (b) Apply the maintainer's own test, "intertwined or not", to
the pillars. Fields is "additive to the path" (core-architecture §4, phase 7). The seeded
policy is a control block. The native solver with its MPC "demonstrates the pipeline". Make
these three the core's acceptance tests: the core is done when each can be written with no
change to `core/`. (c) Before the phases, build one thin slice end to end: a digital PID,
`PID % dt @ plant`, through `problem.compile`, with `jax.grad` of `J` (the "done when" of the
sampled-face row, V1's step 7b). That slice crosses the step algebra, the single score and the
hybrid kernel at their thinnest.
*Cost/risk:* low. It reorders work and adds none. *Touches:* core-architecture §4, TODO §1.9,
the release question.

**2.2 The planning layer has outgrown the "one fact, one home" rule.**
*Seen:* the seeded policy's status is written in core-architecture §5.3, TODO §1.4, ROADMAP
§5.3, QUESTIONS (two entries), future-policies.md §5.1 and §6, and the plans index. The last
8 commits changed about 2,600 lines of docs and 7 lines of code; 35 of the last 70 commits are
docs-only. Sentences like "V1's step 8 (`Set.as_cost`, the open-loop table in `params`), T1,
T2" ask the reader to hold the workboard in their head, which AGENTS forbids.
*Why it matters:* every restatement must be edited on each change, the maintenance cost
ROADMAP §5 names as the enemy.
*Suggest:* make core-architecture.md the only design home for the v0.3 pillars. Reduce the
other plan docs' v0.3 sections to a link, let TODO hold checkboxes that point into it, and
edit in place instead of running new consolidation passes.
*Cost/risk:* a docs pass of a day. The risk is losing history, but git and docs/reviews already
hold it. *Touches:* plans README, AGENTS doc map.

**2.3 The CONSTITUTION contradicts a ruled convention.**
*Seen:* CONSTITUTION §2 says "the stochastic sibling adds distributions over … disturbances".
DESIGN §6 and the code (`StochasticPlanningProblem.sample_disturbances`,
`planning/problems.py:631`) take a bare `Distribution` per port and hold one draw. RD-R4
(randomness.md) rules "a disturbance port takes a signal, never a bare law".
*Why it matters:* the top law and the ruled design disagree, and the draws phase (RN-4) will
have to pick one.
*Suggest:* amend the sentence to "…adds distributions over starts and parameters, and noise
signals on its disturbance ports". Make RN-4's first commit the code change, with a one-time
refusal message for a bare law.
*Cost/risk:* small. It moves the numbers once for RL and `Sys2Gym`. *Touches:* the draws
pillar, RN-4.

**2.4 The hybrid loop is drifting toward a scheduling framework.**
*Seen:* the draft vision carries nested-scan, base-tick-with-selects and hyperperiod layouts
(sampled-path.md §8.1, Draft 3), the clock orchestrator, an adaptive restart path (§8.4),
three ways to record the plant trajectory, and "deployment generates its runner from the
schedule" (§8.2 point 4; core-architecture §6.2). CONSTITUTION §1.4 and ROADMAP §7 put
arbitrary multi-clock scheduling out of scope.
*Suggest:* v0.3 builds **one** pure kernel: the base tick with selects. At a single rate it is
the textbook sampled-data map, and with the plant's substeps inside `discretize` it covers the
nested-scan case. Leave today's `Computer` and `HybridSimulator` as the realtime runner,
re-based onto the pure blocks later. Remove the deployment generator from the vision text;
hyperperiod unrolling and the adaptive restart come later, as optimizations, only when a
profile asks for them.
*Cost/risk:* it cuts work. Multi-rate loops with an expensive slow block compute and discard
on idle ticks, which is acceptable for teaching and research. *Touches:* the hybrid pillar,
S31, the two-runner row.

**2.5 The step algebra's closure is only claimed.** Invariant 1 calls it "closed", but `@`,
`>>` and `+` refuse step systems, a step diagram has two same-tick semantics, and the step
`rollout` returns host NumPy (core-architecture §2.2). The single score, the hybrid loop and the
solve-as-rollout all rest on it. *Suggest:* close it in the step-algebra phase: one semantic,
the same three operators, native arrays out (§5).

**2.6 The parameter-tree logic is duplicated, and a third mechanism is planned.**
*Seen:* path lookup and merge are written in `planning/evaluator.py:986–1045` (`lookup`,
`with_leaf`, `merge_tree`, `flat_leaves`), in `planning/problems.py:896–914` (`lookup_param`,
`merge_params`), in `core/compile/evaluators/jacobian.py:317` (`flatten_params`,
`replace_leaf`) and in `core/assembly.py:160`. parametric-program.md §3.1 plans partial-tree
`bind`, against R121's complete trees, and §3.5 plans live ports addressed by string paths.
*Suggest:* one module, `core/params.py`, holding `leaf(tree, path)`, `with_leaf`, `merge`,
`leaves` and the dotted-path rule. Use it from every caller. Internal machinery is exempt from
the textbook test (CONSTITUTION §6), so this is not new API.
*Cost/risk:* a behaviour-preserving refactor with a byte-identical baseline. *Touches:* phase
1, S76, the question on programs and randomness.

**2.7 The probability and optimization surfaces are creeping.**
*Seen:* `log_prob` on the distributions, distributions over parameter dictionaries and
`UnionSet` (A5); `Gaussian(cov=)` (RN-2); program categories "least squares, stochastic,
parametric" (native-optimizers.md §4.1), while §7.4 concludes that "the stochastic category is
a property of how a solver uses the program".
*Suggest:* `log_prob` and `entropy` live on the policy that needs them, not on `Distribution`,
which keeps its two-method surface. Drop "stochastic" as a program category; §7.4's own
conclusion is better. Add a category only with its first consumer in the same rung.
*Touches:* A5, RN-2, the native-optimizer plan.

**2.8 The stochastic program is designed past its consumers.** It plans stochastic
approximation with four soundness conditions (native-optimizers.md §7.3–§7.4), but its only
v0.3 consumer is `pe.program(wrt, key, n_trials)`, a sample average. *Suggest:* ship the sample
average with the seed as an integer leaf of `p` (a `vmap` over seeds is then a Monte Carlo over
solves); stochastic approximation waits for its first stochastic solver (NS-7, v0.4).

## 3. Opportunities and improvements to the ideas

**3.1 One rule for the per-step cost: integrate it with the state.** differentiable-problem.md
§9.3 proposes to append `ċ = e^{−ρt} g(x, u, t)` to the state. It is better than §9.4 admits.
With `solver="euler", substeps=1` it gives exactly `c_k = e^{−ρ t_k} g(x_k, u_k, t_k) dt`, the
cost that value iteration uses today (`dp.py:203`: `g dt + alpha J(x + f dt)`) and the reward
the RL environment uses (`environment.py:162`). With RK4 it gives the accurate integral. Value
iteration's and RL's numbers therefore do not move at their defaults. Only the evaluator's
trapezoid moves, once, toward the more accurate value.

**3.2 No new problem noun.** `problem.discretize(dt, solver=, substeps=)` can return a
`PlanningProblem` whose `sys` is `discretize(plant, dt, ...)` (already a `StepSystem`) and whose
cost is the per-step cost on `(x_k, u_k, k)`. This is closure, the CONSTITUTION's own
instrument: a discretized plant is a system, so a discretized problem is a problem. Then
`problem.compile` accepts a step plant, and the "one discrete evaluator" is the existing
evaluator on a step loop.

```python
# Before (QUESTIONS, open): a new class
dp = problem.discretize(dt=0.05)            # StepPlanningProblem?
# After: the same noun, a step system inside
dp = problem.discretize(dt=0.05)            # a PlanningProblem; dp.sys is a StepSystem
pe = dp.compile(controller)                 # the one evaluator, a scan over k
```

**3.3 The problem evaluator is the parametric program.** `pe.program(wrt)` already builds
`J(z) = evaluate(merge(p, unpack(z)))`. Write the family once as `J(z, p)` with
`p = pe.params`. Tuning binds the leaves outside `wrt`, the MPC tick binds `x0` and the plant,
and the stochastic program binds a seed. Transcriptions return the same shape. One family then
has three consumers instead of three parallel mechanisms.

**3.4 A stochastic policy as a deterministic law with a noise input.** As an alternative to
D29's own draw: write `u = μ_θ(x) + σ_θ(x) ε`, with `ε` an input port fed by a `NoiseSource` or
`WhiteNoise` block. The reparameterization becomes a diagram, the noise block's convention is
reused exactly, and the draw-convention question disappears. A factory
(`gaussian_policy(mean, log_std, sample_period)`) can return the pair. *Tradeoff:* `log_prob`
on a two-block diagram is less direct than a method on one block. Weigh it against D29 before
ruling.

**3.5 Missing: one page that shows the end state.** Since `end-state-api.md` was retired,
nothing shows what a user writes at the end of v0.3. A 30-line script would serve as the
definition of done and as a test: a stochastic problem, a continuous loop and a digital loop,
`evaluate`, `program`, `Optimizer`, the fixed-K MPC tuned by gradient, and a `QuadraticField`
as a terminal cost.

**3.6 Missing: two decisions to take early.** How a hybrid run's two time grids (sparse
computer ticks, dense plant substeps) live in one `Trajectory`: decide it before the
`StepRollout` fold, or the fold lands twice. And the error family (S70): it types the errors
that hardening (S66) raises, so land its three classes in the first phase.

## 4. Naming suggestions

**Public names.**
- **The hybrid loop:** if `computer @ plant` becomes a `System`, name it by the textbook,
  `SampledDataSystem` (Åström and Wittenmark's term). If it stays a tool, `SampledLoop`.
  `HybridDiagram` names the implementation, not the role (CONSTITUTION §4.5).
- **The program family:** one public noun. Prefer `MathematicalProgram(..., params=None)`,
  whose functions take `(z, p)` when `params` is given, over a second class
  `ParametricProgram`, unless the two cannot share an evaluator. This revisits R26's name and
  is your call.
- **The seeded policy:** if D29 stands, name its period `sample_period`, as `WhiteNoise` does,
  and not `dt`. One word, one meaning across blocks.
- **"Judge":** not textbook vocabulary. Say "one score" in prose. The function stays
  `score_run`.

**Vision words.**
- **"Foundation"** names the three objects in CONSTITUTION §1.1 and also the two properties in
  core-architecture §2. Call the latter **"the two invariants to finish"**.
- **"Pillar"** names the six application families in ROADMAP §5 and also the seven core
  workstreams. Call the first **"the toolbox chapters"** and the second **"the core
  workstreams"**.

**The id and label scheme.**
- Phases are numbered 0–8 with 6a–6c, beside the workboard's `P4`, `P8` and `P10`. Name each
  phase by its content ("the parameters phase", "the hybrid phase") and drop the numbers.
- `V1` is overloaded: "V1's step 7b", "V1.1's rest" and "V1.2" are three different things.
  Give each open step its own workboard id.
- Ruling ids (R1–R123), decision ids (D13–D30, OD-7–OD-36) and plan prefixes (RD-, FD-, NS-,
  PP-, OPW-, RN-, AC-, MP-, SM-) together make about twenty prefixes. In plan prose, cite a
  ruling by its words plus a link, and leave the bare id to the records. Retire the D and OD
  numbers once each one is ruled. The workboard keeps one id family.
- Enforce the AGENTS rule in the docs themselves: a plain gloss before every id, every time.

## 5. Recommended answers to "Blocks v0.3" (reviewer's opinion, not a ruling)

| Question | Recommendation |
| --- | --- |
| Release of the borderline rows | The diagram's derived start state (S29) and the error family (S70) join v0.3's first phase: both are parameter or error groundwork. The realtime re-base, the adaptive orchestrator and the RL heads' rebuild stay out (v1.0 review, later, v0.4). |
| Same-tick semantic | Propagation within the tick, the step diagram's semantic and the textbook one. A computer's one-tick wire latency becomes an explicit `UnitDelay` where it is wanted. One semantic; the numbers move once. |
| Operators on step systems | Yes: the same `>>`, `+` and `@` with the same meaning. It is one grammar on the sibling type, not a second dialect. Mixing continuous and step operands refuses, naming `%` or `discretize`. Build both on `assembly.py`, not as a copy. |
| Unify the evaluation from the rollout on | Yes, by §3.1 (the cost integrated with the state) and §3.2 (no new noun). |
| Failure rule, hold, scope | One rule: any margin below zero at a sample is a failure, and the score runs to `N`. The RL environment may stop early as a training choice, but it reports the score's `J`. Zero-order hold only. Shooting keeps its NLP objective, but its reported evaluation goes through the score. S63 stays in v0.4. |
| The seeded policy's contract | D29's shape, with `sample_period` in place of `dt` (§4); `log_prob` and `entropy` on the policy, not on `Distribution`. Weigh §3.4 first. |
| Draw convention | One convention: the block's own `⌊t/Δ⌋` from its own `sample_period`. `%` checks that `Δ` equals the computer period, so it equals the tick index there. |
| `computer @ plant` a step diagram | Amend invariant 1 narrowly: "the coupling is a tool; at its base clock the loop is a `StepDiagramSystem`". A subclass is acceptable only if it adds verbs and overrides none, with a test that every step tool answers as on a plain step diagram. |
| `compute_trajectory` default on a hybrid loop | The step-diagram runner with the substeps recorded: the same numbers as `evaluate`. The adaptive path only behind `solver="scipy"`, later. |
| Run caches | They go, as recommended. |
| `problem.compile` and the stateless MPC | Refuse it now. A latch firing at the RK4 stages gives wrong numbers silently; a refusal costs one line. |
| One parametric family | Yes, per §3.3. A hand-written program may declare `p`. The MPC's live ports wait for a consumer (v0.4). `bind` on the program only. |
| Programs and randomness | Pass `p` (`J(z, p)`), with `bind` as the host-solver convenience. Complete trees inside, one merge at the boundary (R121 kept). The seed is found by the `seed` key, as on blocks; integer leaves take no gradient. Shapes are structural. The warm start belongs to the solve call. Sample average only in v0.3 (§2.8). |
| Kernel shape | B, the pure kernel; the step-system view comes with the teaching-face step (NS-5) in v0.4, not in v0.3. |
| Which fixed-iteration tick first | The gradient MPC, as directed. The sampled kernel's *first* consumer should be the digital PID (§2.1c). MPPI stays in v0.4. |
| Where `as_cost` and the shaping live | One function in `core/fields.py`, the shaping beside it, moved in the first phase. `Set.as_cost` and `Field.as_cost` delegate to it. |
| Flatten by default | Wait for the benchmark, then prefer dropping the flag (one path) if parity holds. |
| The 16 overnight decisions | Review the number changes and the input-box sweep; accept the rest as a block. Then merge (§2.1a). |
| Three hand gradient-loop demos | Your teaching call. My view: keep one hand loop (show the mechanism once, RULES 6.15) and move the other two. |
| LQR warning when `U` is declared | Do not warn. State "U ignored" in the LQR record's one-line `str` (this amends R21). |
| `DynamicBicycle`'s base | Rename only; the base waits for the catalog clean-up (K8). |
| Failure price under cost overrides | Keep the problem's price, computed once at compile. That removes the 4096 calls of `g` per evaluation without a cache. |
| Three demos' style | Your teaching call. |
