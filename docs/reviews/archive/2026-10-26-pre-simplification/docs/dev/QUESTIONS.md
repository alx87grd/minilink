# Open questions for the maintainer

One entry per question that waits for you, ordered by the release it blocks. Answer on the
`Answer:` line or in a session; the agent then records the ruling (the plan doc it shapes and
a dated record in docs/reviews/), applies it, and deletes the entry. Rulings already taken:
[docs/reviews/](../reviews/). Known defects: [KNOWN_ISSUES.md](KNOWN_ISSUES.md).

## Blocks v0.3 (the complete core)

By the core's pillars, in the proposed order of [TODO](../plans/TODO.md#1-v03--the-complete-core-by-pillar)
§1 ([core-architecture.md](../plans/core-architecture.md)): the scope and the order first, then
each pillar, then the landed continuous core and the clean-up lane. The entries marked "an open
point the 2026-10-26 consolidation recorded" are new; the others are kept as written, moved from
the bucket they had.

**The scope and the order**

### Which release does each borderline row take, now that v0.3 is the complete core?
- **Blocks:** the order of the core's rows ([TODO](../plans/TODO.md#1-v03--the-complete-core-by-pillar) §1).
- **Today:** rows the core reads but that sit elsewhere: S29, the diagram's derived start state (v0.9;
  only the plant slice of the start is a leaf of `pe.params` today); S70, the error family
  (ruled R114, v0.9), whose types S66 (in the core's last phase) raises; the realtime
  simulator re-based on the pure blocks and the hybrid orchestrator's adaptive plant
  integration (sampled-path.md §8.4); the RL heads rebuilt on the seeded policy (C5, v0.4).
- **Options:** each joins v0.3's core; or each keeps its rung outside v0.3 (S29 and S70 in v0.9; the realtime re-base
  with the realtime band, whose architectural review is a v1.0 row; the RL heads with C5 in v0.4).
- **Recommendation:** none (an open point the 2026-10-26 consolidation recorded).
- **Detail:** [core-architecture.md](../plans/core-architecture.md) §4
- **Answer:**

**Foundations: every object pure; the step algebra (phases 1, 2)**

### Which same-tick semantic does a step diagram have?
- **Blocks:** the step algebra (TODO §1.2) and the hybrid loop's pure tick (§1.5); numbers move once either way.
- **Today:** two semantics: the step-diagram evaluator propagates a signal within its tick, from a fresh
  buffer; `Computer.tick` reads the previous tick's committed buffer, one tick of latency per
  wire. S65 lists "the one-tick lag inside a `Computer` tested and stated, or removed".
- **Options:** propagation within the tick; one tick per wire (the committed buffer); either, stated per
  connection.
- **Recommendation:** none (an open point the 2026-10-26 consolidation recorded).
- **Detail:** [core-architecture.md](../plans/core-architecture.md) §2.2; [sampled-path.md](../plans/sampled-path.md) §8.1 (Draft 2); TODO S65
- **Answer:**

### Do step systems get the composition operators?
- **Blocks:** the teaching face of the solver loop (NS-5) and `computer @ plant` as a step diagram (§8.5).
- **Today:** `@`, `>>` and `+` refuse step systems; a step diagram is wired with `connect` only. CONSTITUTION
  §6 freezes the grammar and gives the hybrid algebra one operator, `%`.
- **Options:** operators on the step algebra (a core-grammar ask); a helper that wires a loop
  (`gradient_loop`, native-optimizers.md §4.5); `connect` only.
- **Recommendation:** none (an open point the 2026-10-26 consolidation recorded).
- **Detail:** [native-optimizers.md](../plans/native-optimizers.md) §3.1, §4.7
- **Answer:**

**One discrete judge (phase 3)**

### Does the evaluation unify from the rollout on, with a discretized planning problem? (needs refinement)
- **Blocks:** one discrete judge (TODO §1.3); it shapes step 7b (the sampled face) and the sampled
  loop (S31), both v0.3 (TODO §1.5).
- **Today:** the evaluator's run is already the step model and its score already reads the
  samples; the same continuous cost becomes a per-step cost three ways (value iteration and RL
  a left Riemann sum `g dt`, the evaluator a trapezoid), so one policy has three close `J`s.
- **Your idea (2026-10-26, draft):** once `dt` and the scheme are set, discretize the problem
  (time becomes an index in the cost and the sets) and score every stepping loop with one
  discrete evaluator.
- **Options:** the per-step cost by integrating the cost with the state (exact, the agent's
  recommendation), or by a quadrature on the samples; the object's name and contents
  (`problem.discretize(dt, ...)` returning a discrete problem).
- **Recommendation:** the direction, yes; refine the six points of the detail before a plan.
- **Your direction (2026-10-26):** a core pillar of v0.3, "one discrete judge"; the refinement stays open.
- **Detail:** [differentiable-problem.md](../plans/differentiable-problem.md) §9
- **Answer:**

### What failure rule, hold and scope does the discrete problem take?
- **Blocks:** one discrete judge (TODO §1.3); the numbers of value iteration, RL and the evaluator move once.
- **Today:** `score_run` integrates to `tf` and checks `X` and `U` at every sample, under the price switch;
  the RL environment stops at the first exit from `X` and checks no `U`; value iteration prices
  the cells that leave its grid and ignores a finite `tf` (S63, a v0.4 row); shooting scores with
  its own trapezoid and a first-order hold; the feedback planners report their held-input
  rollout (OD-27).
- **Options:** as the docs state them: one rule, absorbing or not; the zero-order hold; shooting's score
  lowered to the discrete problem, or kept; planner policies acting at a fixed `dt` scored
  through the sampled face; S63 with the judge, or in v0.4.
- **Recommendation:** none (an open point the 2026-10-26 consolidation recorded).
- **Detail:** [differentiable-problem.md](../plans/differentiable-problem.md) §9.4; [core-architecture.md](../plans/core-architecture.md) §3
- **Answer:**

**The draws and the seeded policy (phase 4)**

### What does the standard stochastic policy look like as a seeded block?
- **Blocks:** the seeded stochastic policy (TODO §1.4, D29); the RL heads rebuilt on it, the RL
  numbers changing once (core or the RL follow-ups C5: open, the release entry above).
- **Today:** your direction (2026-10-22), `u = μ_θ(x) + σ_θ(x) · ε(seed, ⌊t/Δ⌋)`; its methods
  and first heads are open.
- **Recommendation:** `GaussianPolicy(mean, log_std, *, dt, seed=None)` in `control/neural.py`,
  `seed=None` the deterministic face, `log_prob` and `entropy` beside `h`, the squashed variant
  a subclass.
- **Your direction (2026-10-26):** the seeded stochastic policy is a core pillar of v0.3; its contract stays open.
- **Detail:** [future-policies.md §5.1, D29](../plans/future-policies.md#51-d29-the-standard-stochastic-policy-as-a-seeded-block-moved-from-end-state-apimd-2026-10-26)
- **Answer:**

### Which draw convention does a seeded policy use: the held sample or the tick?
- **Blocks:** the seeded stochastic policy (TODO §1.4) and every sampled controller with draws (§1.5).
- **Today:** D29 writes `ε(seed, ⌊t/Δ⌋)`, the noise block's held draw on continuous time; R104 draws a
  sampled controller's noise from its seed and the tick `k`.
- **Options:** one convention, stated (with `Δ = d·dt_base`, `⌊t/Δ⌋` is the tick index); two, by the block's
  kind.
- **Recommendation:** none (an open point the 2026-10-26 consolidation recorded).
- **Detail:** [future-policies.md](../plans/future-policies.md) §5 point 4, §5.1; [sampled-path.md](../plans/sampled-path.md) §8.6 (D14)
- **Answer:**

**The hybrid loop: one model, two runners (phase 5)**

### Does `computer @ plant` become a step diagram, with a CONSTITUTION amendment?
- **Blocks:** the sampled loop (S31, v0.3, TODO §1.5); CONSTITUTION invariant 1.
- **Today:** `computer @ plant` builds a `HybridDiagram`, a plain class beside the step algebra;
  R13 and R120 rule the sampled loop a tool, not a `System`.
- **Your direction (2026-10-26):** a tentative draft vision, promising, not decided: the result is
  a `StepDiagramSystem` subclass on the base clock (the computer's pure step and the plant
  discretized), regular for the algebra and the analysis, with hybrid-aware
  `compute_trajectory`, `plot_diagram` and the clock runner. Later the same day, a core pillar of
  v0.3, "one model, two runners": the pure step diagram for analysis, evaluation and gradients,
  the clock orchestrator for realtime and compute efficiency, over the same pure blocks; the
  amendment itself stays open.
- **Options:** amend invariant 1's second sentence to "the coupling is a tool; its result, at the
  base clock, is a `System` of the step algebra"; or keep R120 and expose a read-only step map.
- **Recommendation:** the amendment, when S31 lands; the precedent is `Discretization`.
- **Detail:** [sampled-path.md](../plans/sampled-path.md) §8.5
- **Answer:**

### On a hybrid loop, which runner does `compute_trajectory` use by default, and how dense is the plant's trajectory?
- **Blocks:** the hybrid loop (TODO §1.5); the hybrid demos' numbers and the regression gates `f.hybrid_zoh`,
  `f.dual_rate`; the shape of the `StepRollout` fold (§1.2).
- **Today:** `HybridDiagram.compute_trajectory` integrates the plant by RK4 substeps inside each base tick
  and records every substep; no adaptive option exists.
- **Options:** sampled-path.md §8.4's two drafts: the step-diagram runner by default (the substeps recorded,
  `solver="scipy"` the adaptive path), or the orchestrator with an adaptive plant by default
  (the step diagram for analysis, `evaluate` and gradients); and the resolution: a finer base
  clock, the substeps recorded, or reconstructed after the run.
- **Recommendation:** none (an open point the 2026-10-26 consolidation recorded).
- **Detail:** [sampled-path.md](../plans/sampled-path.md) §8.4
- **Answer:**

### Do the run caches on the sampled loop go?
- **Blocks:** the sampled loop (S31, D13); RULES 2.4, no run state on a System.
- **Today:** `StepSystem.rollout` and `HybridDiagram.last_result` cache a run beside `self.traj`.
- **Recommendation:** they go: `loop.compute_trajectory()` returns the `Trajectory` and stores
  `loop.traj`; the facades read `traj`.
- **Detail:** [sampled-path.md §8.6, D13](../plans/sampled-path.md#d13-the-run-state-caches-s31-rules-24)
- **Answer:**

### Does `problem.compile` refuse the stateless MPC until the sampled face lands?
- **Blocks:** the hybrid loop (TODO §1.5); R123 lands with it.
- **Today:** `require_continuous` (`planning/evaluator.py`) refuses a `StepSystem` controller, but
  `MPCStatelessController` is a plain `System` (`planning/trajectory_optimization/mpc/controller.py`),
  so `problem.compile` accepts it and its tick latch then runs at the RK4 stages.
- **Options:** refuse it now, naming the hybrid loop; leave it until V1's step 7b.
- **Recommendation:** none (an open point the 2026-10-26 consolidation recorded).
- **Detail:** [core-architecture.md](../plans/core-architecture.md) §6.3
- **Answer:**

**The generic parametric program; the stochastic program its case (phases 6a, 6b)**

### Does the parametric program become one family, with hand-written parameters and live MPC ports?
- **Blocks:** the compiled MPC program binding every parameter (S76, D25); the parametric
  optimizer table (OPW-1–OPW-5).
- **Today:** it binds the measured `x0` only; collocation freezes the plant, cost and set
  parameters in closures. Names ruled (R26: `ParametricProgram`, the key `plant`).
- **Options:** two parallel types; one class always taking `p`; one family, the plain
  `MathematicalProgram` its binding.
- **Recommendation:** the family; a hand-written program may declare its own `p`; the MPC block
  takes optional live-parameter ports, unwired at nominal (`live=["plant.mu"]`). Also open:
  `bind` on both the evaluator and the program.
- **Your direction (2026-10-26):** the generic parametric program, a `MathematicalProgram` compiled once with an
  explicit, optional `params` argument (`x0`, the plant, the cost and the sets), finished all at
  once in v0.3's core; open: a hand-written program's own `p`, the MPC's live ports, `bind` on
  both.
- **Detail:** [parametric-program.md §2.1, D25](../plans/parametric-program.md#21-d25-the-two-remaining-asks-moved-from-end-state-apimd-2026-10-26);
  [parametric-program.md §2](../plans/parametric-program.md#2-the-recommendation-one-family-the-frozen-program-a-binding)
- **Answer:**

### How do programs and solvers take parameters and randomness? (needs analysis and discussion)
- **Blocks:** the stochastic-program row (TODO §1.6); it shapes S76's `ParametricProgram` (v0.3),
  the native solvers and the stochastic category of native-optimizers.md.
- **Today:** only `x0` varies in the compiled MPC program; `pe.program(wrt, key, n_trials)`
  freezes its draws at build (a sample average); seeds live in `params` as integers (RD-R6, R118);
  overrides are complete trees (R121) while `parametric-program.md` §3.1 binds partial ones.
- **Your leaning (2026-10-25, draft):** seeds are params, as on a system, so every program is
  deterministic given its params; a stochastic solver may change the seed internally to work on
  the sampled, noisy function.
- **Your direction (2026-10-26):** every program is deterministic given its params, the seed one of them; solving a
  stochastic program is solving over several seeds; the stochastic program is the parametric
  program's case, a core pillar of v0.3. Open: §7.5's seven points.
- **Options:** not yet; the draft is one signature `J(z, p)` with the seed an integer leaf of `p`,
  kept fixed by deterministic solvers (a sample average), advanced per iteration by stochastic
  ones (`child(seed, k)`), with four soundness conditions and seven open points to settle.
- **Recommendation:** none yet; a discussion session first.
- **Detail:** [native-optimizers.md](../plans/native-optimizers.md) §7
- **Answer:**

**The solve as a rollout, a proof of concept (phase 6c)**

### Native optimizers: is the solver a pure kernel with a step-system view, or a `StepSystem` itself?
- **Blocks:** the native-optimizer plan: the proof of concept (NS-1, NS-2's fixed-`K` form; TODO
  §1.7, v0.3) and the teaching face (NS-5, v0.4).
- **Your direction (2026-10-26):** a proof of concept in v0.3, a simple fixed-step solver and an
  MPC on it; better solvers in v0.4. The kernel's shape stays open.
- **Today:** `gd` and `adam` are a Python loop in `optimizers/first_order.py`; a hand-wired
  `StepDiagramSystem` of a law `z⁺ = z − η u` and a gradient oracle already rolls out gradient
  descent.
- **Options:** (A) every native solver a `StepSystem` by construction; (B) a pure
  `init` / `step` kernel run by `Optimizer`, with the step-diagram loop as a teaching view of it;
  (C) kernel only.
- **Recommendation:** B: A forces Adam's pytree state into a flat `x`, C loses the
  step-size-as-stability lesson.
- **Detail:** [native-optimizers.md](../plans/native-optimizers.md) §3
- **Answer:**

### Which fixed-iteration tick comes first: the gradient MPC or MPPI's?
- **Blocks:** the solve as a rollout (TODO §1.7) and MPPI (S72, TODO §4).
- **Today:** native-optimizers.md §4.6b and §5 say "MPPI is the first instance, ahead of NS-8": its
  tick is already pure, needing no native solver. Your direction of 2026-10-26 puts the gradient
  MPC's proof of concept (NS-1, NS-2's fixed-`K` form, NS-8, NS-9) in v0.3 and MPPI in v0.4.
- **Options:** the gradient MPC first, as the direction places it; MPPI's tick first as the
  sampled loop's first pure controller, MPPI's planner still v0.4; both in the same phase.
- **Recommendation:** none (an open point the 2026-10-26 consolidation recorded).
- **Detail:** [native-optimizers.md](../plans/native-optimizers.md) §4.6, §4.6b, §5;
  [core-architecture.md](../plans/core-architecture.md) §8
- **Answer:**

**Fields (phase 7)**

### Where do `as_cost` and the shaping live?
- **Blocks:** Fields (TODO §1.8), V1's step 8 (§1.1) and the workspace geometry (A3, v0.4).
- **Today:** three plans: `Set.as_cost(weight=, shaping=)` (A5; differentiable-problem.md step 8, after
  `shaping.py` moves to `core/`); `Field.as_cost` with the shaping next to it in `core/fields.py`
  (fields.md 4.1); the shaping helpers moved to `core/fields.py` (geometry-module.md).
- **Options:** one home and one signature for sets and fields; `Set.as_cost` first, fields following.
- **Recommendation:** none (an open point the 2026-10-26 consolidation recorded).
- **Detail:** [fields.md](../plans/fields.md) 4.1; [geometry-module.md](../plans/geometry-module.md); [differentiable-problem.md](../plans/differentiable-problem.md) §8
- **Answer:**

**The landed continuous core and the clean-up lane**

### Should compile-time diagram flatten default to on after the parity and speed study?
- **Blocks:** nothing until benchmarks and tests are stable; default stays `False` until you decide.
- **Today:** `compile_diagram(..., flatten=False)` inlines nested boxes with ``__`` ids
  (`inner__plant`); parity tests and `benchmarks/scenarios/nested_compile_flatten.py`
  compare nested compile, `flatten=True`, and hand-flat wiring.
- **Options:** keep default off; turn default on if flatten wins on compile and/or `f()`;
  drop the flag and always flatten at compile.
- **Recommendation:** wait for one informational benchmark run on your reference machine,
  then choose.
- **Detail:** [DESIGN.md](DESIGN.md) (compile-time flatten paragraph)
- **Answer:**

### Do you review the overnight run's 16 remaining decisions, or accept them as a block?
- **Blocks:** your merge of the v0.3 branch into `dev` (R79).
- **Today:** the run of 2026-10-24 decided 36 points without you, each with its reason, the
  alternative and the commit; its §3 measures the intended number changes. Settled since: OD-9
  confirmed, OD-10 superseded, sixteen accepted from chat (R103). Open: OD-7, OD-8, OD-12,
  OD-14, OD-17, OD-18, OD-20, OD-21, OD-23, OD-24, OD-25, OD-26, OD-27, OD-29, OD-30, OD-35;
  OD-32 and OD-33 are the two questions below.
- **Options:** accept the rest as a block; review each; accept all but the ones you name.
- **Recommendation:** the log's "please look first at": the number changes (value iteration's
  default price is now the cost bound, so the GRO860 cost-to-go colour scale tops there;
  scores moved from a held law to the continuous loop, up to 8% for a stiff PD), the input-box
  sweep (OD-18), the planners' reported trajectory (OD-27); then the rest as a block.
- **Detail:** [the run's log, §1–§3](../reviews/2026-10-24-v03-run-decisions.md#2-decisions)
- **Answer:**

### Do the three demos that teach a hand-written gradient loop move onto the `Optimizer`?
- **Blocks:** the rest of the optimizer work (S74's rest, OD-32).
- **Today:** `cartpole_rollout_gradients`, `neural_controller_jax`, `params_gradient`
  (`examples/demos/compile/`) descend by hand; the first tunes an open-loop schedule, which the
  evaluator cannot tune until a source's samples are parameters. R90 kept the hand loop in two
  places on purpose.
- **Options:** convert all three; convert two, the schedule demo later; keep all three.
- **Recommendation:** none yet (converting changes what they teach).
- **Detail:** [the run's log, OD-32](../reviews/2026-10-24-v03-run-decisions.md#2-decisions)
- **Answer:**

### Does LQR warn when a problem declares `U`, now that the LQR lessons declare it?
- **Blocks:** the rest of the problem-fields work (V1.2's rest, OD-33).
- **Today:** R21 says LQR ignores `U` and warns once when it is declared. The run wrote
  `U=plant.input_box` into about 90 constructions to keep their numbers (OD-18), the GRO860 LQR
  notebooks among them, so the warning would fire where nothing changed. Not implemented.
- **Options:** drop the line from the LQR lessons, then warn; keep it and warn; keep it and
  never warn (amends R21).
- **Recommendation:** none yet.
- **Detail:** [the run's log, OD-18, OD-33](../reviews/2026-10-24-v03-run-decisions.md#2-decisions)
- **Answer:**

### One mechanical family: does `DynamicBicycle` move onto the generalized mechanical base now?
- **Blocks:** one mechanical family (S32); the car family clean-up (K8).
- **Today:** the row moves its base from `DynamicSystem` to the generalized child, but its named
  ports and its subclasses' extra states do not fit `[q; v]`.
- **Options:** move it now; rename `M` to `H` only and move it when K8 reworks the cars.
- **Recommendation:** rename only; the base waits for K8.
- **Detail:** [TODO, S32](../plans/TODO.md#2-v03--the-clean-up-lane-beside-the-core)
- **Answer:**

### Should the failure price follow a run's cost overrides?
- **Blocks:** programs over cost weights (`pe.program(wrt=["cost…"])`; the parametric program,
  S76, TODO §1.6).
- **Today:** the price keeps the problem's own weights; with no exact bound it costs 4096 calls
  of `g` at every `evaluate` and program iteration. A cache keyed on the cost's parameters
  would remove that.
- **Options:** price from the overridden weights, with the cache; keep the problem's price.
- **Recommendation:** none yet.
- **Detail:** [the run's log, §6 item 6](../reviews/2026-10-24-v03-run-decisions.md#6-the-review-pass-after-the-run)
- **Answer:**

### Do three demos change style?
- **Blocks:** your demo pass (K4); the closing example (S77) landed.
- **Today:** `pid_autotuning_jax.py` keeps two helpers, prints that carry arithmetic and a
  hand-made last figure; `diagram_shortcuts.py` labels `NoisyPendulum()` as `Pendulum()`;
  `custom_one_link_arm.py` reads module constants, not `params`, and a comment restates a line.
- **Recommendation:** none yet.
- **Detail:** [the run's log, §7](../reviews/2026-10-24-v03-run-decisions.md#7-the-second-review-pass)
- **Answer:**

## Blocks v0.4 (features on the core)

### Does the MPC block ask its planner for three verbs instead of probing it?
- **Blocks:** the MPC block's contract (T6, D27) and MPPI inside it (MP-4).
- **Today:** `validate_mpc_planner` and the dual-rate export probe with `hasattr`.
- **Recommendation:** `decision_dimension()`, `prepare_online()`,
  `warm_start_guess(z_prev, y, *, dt_mpc, k)` and `z` on the tick's record; then
  `ModelPredictiveController(planner, dt_mpc=)` wraps trajectory optimization and MPPI alike.
- **Detail:** [mppi.md §10, D27](../plans/mppi.md#d27-the-mpc-blocks-planner-contract-mppimd-ask-2-t6)
- **Answer:**

### mppi.md's design predates R104, R107 and R123: is it rewritten to them before MP-1?
- **Blocks:** the MPPI planner (S72, v0.4).
- **Today:** its body splits a stateful `self.key` (R104: a sampled controller draws from its seed and the
  tick); §4 proposes `M = 4` plant draws (R107: one by default); §6 keeps the tick latch (R123:
  it goes); its opening lists the planner verbs otherwise than §6 and D27.
- **Options:** rewrite the doc to the rulings when MP-1 starts; now.
- **Recommendation:** none (an open point the 2026-10-26 consolidation recorded).
- **Detail:** [mppi.md](../plans/mppi.md) §1, §4–§6, §10
- **Answer:**

### Native optimizers: which program categories and which native algorithms are in scope?
- **Blocks:** NS-4, NS-6, NS-7; CONSTITUTION §1.4 ("not a solver framework").
- **Today:** three classes read from the fields (`unconstrained`, `bounded`, `constrained`);
  native `gd` and `adam` only.
- **Options:** add least squares, stochastic and parametric categories; native GD, momentum,
  Adam, Newton, Gauss–Newton, Levenberg–Marquardt, augmented Lagrangian; or a smaller set.
- **Recommendation:** the categories each with a named consumer (identification, inverse
  kinematics, the Monte Carlo objective, the MPC); the textbook algorithms only; Ipopt stays the
  constrained workhorse.
- **Detail:** [native-optimizers.md](../plans/native-optimizers.md) §4.1–§4.2
- **Answer:**

### Do you adopt or drop the optional `KinematicModel` delegate?
- **Blocks:** the general serial chain (C6), decided with its mechanism description.
- **Today:** C6 is design first; its July 2026 draft is in git history.
- **Recommendation:** none yet.
- **Detail:** [TODO, C6](../plans/TODO.md#4-v04--features-on-the-core)
- **Answer:**

### Which pyro demos do the courses still need?
- **Blocks:** the pyro parity rows and the migration guide (C1).
- **Today:** of pyro's 195 example scripts, 29 done, 47 partial, 83 to do, 36 dropped.
- **Recommendation:** none yet.
- **Detail:** [pyro-port-remaining.md](../plans/pyro-port-remaining.md)
- **Answer:**

### Do the planning records get solver-specific views?
- **Blocks:** the planning records before the freeze.
- **Today:** no plan doc says more than this line.
- **Recommendation:** none yet.
- **Detail:** none beyond this entry (once a one-line item of ROADMAP §6).
- **Answer:**

## Blocks v0.9 / v1.0

### Where do the course material and the research projects live?
- **Blocks:** the freeze candidate (K5, then the move K6), between terms after GMC714.
- **Your direction (2026-10-26):** no course calendar shapes a step; "between terms" above is the
  plan's wording before it. The question stays open.
- **Today:** the GRO860 notes and webpage and GRO501's `\colab{}` cite the course notebooks, which
  clone `main` unpinned; on 2026-10-22 you put pinning off to the pinned cell (V6).
- **Options:** one repo; a course repo pinned on a release; one repo with pinned course cells and
  a nightly drift run. And `projects/`: research material that stays, or course material.
- **Recommendation:** pinned cells now, a course repository at v1.0.
- **Detail:** [repository-structure.md §4](../plans/repository-structure.md#43-recommendation), [§6](../plans/repository-structure.md#6-the-asks)
- **Answer:**

### How is the examples tree laid out?
- **Blocks:** the examples moves, between terms (with K5).
- **Your direction (2026-10-26):** no course calendar shapes a step; "between terms" above is the
  plan's wording before it. The question stays open.
- **Today:** five kinds of folder on four axes; the README table is the only map.
- **Options:** tidy today's names; two axes, chapter and course; notebook-first with paired
  scripts.
- **Recommendation:** two axes, staged between terms; to rule with it, the chapter list and a home
  for `hybrid`, `mpc`, `realtime`, `robotic` and the racecar plant folder.
- **Detail:** [repository-structure.md §3](../plans/repository-structure.md#33-recommendation)
- **Answer:**

### Does a diagram's start state become a derived property, like its live parameters?
- **Blocks:** the foundations that could break a name (S29).
- **Today:** `DiagramSystem.x0`, `n` and `state` are stored; `Simulator` reads them after a
  `refresh()`.
- **Recommendation:** the row's plan: derived properties, `x0` and `state.nominal_value` with
  one owner each.
- **Detail:** [TODO, S29](../plans/TODO.md#v09--what-could-still-break-a-name)
- **Answer:**

### Does Diffrax join as an optional JAX solver?
- **Blocks:** the foundations (S27); a new optional dependency.
- **Today:** the integrators are in `core/compile/evaluators/stepping.py` (OD-11).
- **Recommendation:** none yet.
- **Detail:** [TODO, S27](../plans/TODO.md#v09--what-could-still-break-a-name)
- **Answer:**

### Does one posed-geometry hook replace `tf` plus a skin, and are the drawing `Sphere` and `Box` renamed?
- **Blocks:** the foundations (S44, S30).
- **Today:** `graphical/animation/primitives.py` and `core/geometry.py` each export a `Sphere`
  and a `Box`.
- **Recommendation:** none yet.
- **Detail:** [TODO, S44 and S30](../plans/TODO.md#v09--what-could-still-break-a-name)
- **Answer:**

### Which names does the v1.0 freeze cover?
- **Blocks:** the frozen surface (V3).
- **Today:** `StepSystem`, `StepDiagramSystem`, `UnitDelay`, `ModelPredictiveController` and its
  sub-facade, and the spatial names the geometry work moves, are provisional.
- **Options:** each joins the freeze, or sits behind a visibly provisional name.
- **Recommendation:** none yet.
- **Detail:** [TODO, V3](../plans/TODO.md#v09--what-could-still-break-a-name)
- **Answer:**

### From v1.0, does a rename ship one minor release behind a deprecation shim?
- **Blocks:** the trust contract (V4); amends RULES "no deprecated aliases" from v1.0.
- **Today:** no alias or shim (R82); `xbar` / `ubar` and `plot_cost2go` wait on this.
- **Recommendation:** the row's proposal: a `DeprecationWarning` shim for one minor release, and
  a snapshot test of the export list.
- **Detail:** [TODO, V4](../plans/TODO.md#v09--the-trust-contract)
- **Answer:**

## Standing (no release depends on it)

### Is there one separator rule for wires, parameter paths, block labels and duplicate state labels?
- **Blocks:** the larger naming alignment.
- **Today:** block outputs are recorded `id:port`, the plant's input `plant.u` (OD-13), parameter
  paths `motor.v2.A`; `add_subsystem` refuses an id holding `.`, `/` or `:`.
- **Recommendation:** none yet.
- **Detail:** [the run's log, §7](../reviews/2026-10-24-v03-run-decisions.md#7-the-second-review-pass)
- **Answer:**

### How do behavior cloning and diffusion policy enter the API?
- **Blocks:** nothing (for discussion).
- **Today:** three questions: does `fit` join `solve` and the program export as the third way to
  train; is diffusion policy in scope, research lane only; is distilling MPC or MPPI into a
  neural policy the first imitation demo?
- **Recommendation:** none yet.
- **Detail:** [future-policies.md §7](../plans/future-policies.md#7-questions-for-the-maintainer-for-discussion-not-blocking)
- **Answer:**

### Does the safety filter with control barrier functions get a rung?
- **Blocks:** nothing.
- **Today:** a design draft, after the field object (A1) and the geometry (A3).
- **Recommendation:** research lane until a course asks.
- **Detail:** [cbf-safety-filter.md](../plans/cbf-safety-filter.md)
- **Answer:**

### Does the RL planner scale rewards automatically, and by default?
- **Blocks:** nothing scheduled; a new default moves every RL demo's numbers once.
- **Today:** raw costs throttle PPO through the gradient clip; scaled, it reaches LQR on every
  seed tested.
- **Recommendation:** `reward_scale="auto"`, `c = 1 / (g_max dt)` from `problem.cost_bound()`;
  the default is yours.
- **Detail:** [rl-reward-scaling.md §6](../plans/rl-reward-scaling.md#6-proposal)
- **Answer:**

## Steps waiting for your go

Workboard rows marked **[ask]** whose question is not an entry above, by release; each waits for
your go (or your review) before an agent starts.

| Step, in plain words | Id | Release | What the ask is |
| --- | --- | --- | --- |
| The default output stub builds a NumPy array under JAX | T1 | v0.3 (the core, first foundation) | the returned type of the default `h` (`np.zeros(p)`) |
| The demo sweep to the flatness rule | D1.3 | standing (wave D) | your review of each diff; `showcase_jax` waits |
| The planning demos on the solution verbs | D1.4 | standing (wave D) | `solution.plot_*` and `compare(...)` in place of hand-rolled comparisons |
| `Source.show_signal` | D2 | standing (wave D) | remove it (the same picture is `plot_trajectory`) |
| Two dead modules | D2 | standing (wave D) | remove `planning/spatial/overlays.py` (with A3) and `experimental/symbolic/mechanics/utils.py` |
| A connected input rewired silently | S66 | v0.3 (the core, hardening) | `connect` refuses to rewire it |
| Your demo pass, and a tabular Q-learning demo | K4 | v0.3 (the clean-up lane) | your pass; the demo beside value iteration (R65) |
| The noise seeds on the draws | V1 step 7 | v0.3 (the core, the draws) | one seed per random block in `problem.realize` (RN-4, RN-5, AC-7) |
| LQG as a planner | LQGPlanner | v0.4 | `LQGPlanner(problem)` beside `LQRPlanner` |
| The field object | A1 | v0.3 (the core, Fields) | `StateField` → `Field` in `core/fields.py`; the certificate's `V` a field |
| GMC714's names pinned | G3 | v0.3 (the clean-up lane) | `test_teaching_surface.py` pins the teaching names, each with a demo |
| The unsupported plants to the experimental tier | K8-3 | v0.3 (the clean-up lane) | per plant, from K1's table |
| Workspace geometry | A3 | v0.4 | the `core/geometry/` package; `planning.spatial` moved, then deleted |
| SAC's actor step | S50 | v0.4 | the critic before the update (today) or after it |
| Policy iteration on the grid | S51 | v0.4 | a mode or a sibling of `DynamicProgrammingPlanner` |
| Approximate dynamic programming | C5 | v0.4 | design first; it touches the planner API |
| The general serial chain | C6 | v0.4 | one mechanism description (DH table or URDF) |
| The pyro → minilink migration guide | C1 | v0.4 | its section in README |
| Value iteration on a finite horizon | S63 | v0.4 | backward sweeps for a finite `tf`, or a warning |
| Catalog hygiene | S64 | v0.4 | the per-plant clean-ups |
| Traceable linear analysis | S73 | v0.4 (any time) | `poles`, `frequency_response`, `margins` through the array module, and the sensitivity peaks |
| The move of the course material | K6 | v0.9 | only if K5 says so |
| The pinned Colab cell | V6 | v0.9 | a pinned release in every notebook; `showcase_jax`'s `c_export` import |
| A lighter git history | V11 | v0.9 (after the v0.3 tag) | a force-push without notebook outputs (176 MB clone to under 25 MB) |
| The convergence leftovers | TODO §6 | before V3 | the racecar 3-D skin moved; `Sys2Gym` / `SB3Controller` to the experimental tier; `planning/search/` cut; three facade verbs |
| The public face | V10 | v1.0 | a teaching quickstart first in README; `CONTRIBUTING.md` and templates |
| The Stable-Baselines3 notebooks | S49 | v1.0 | retire two, decide two |
| The realtime band's review | — | v1.0 | frozen surface or provisional |
| Behavior cloning | — | after v1.0 | the game mode and the fitting planner |
| A plant that owns its noise ports | — | later | the example (R57), then the two noisy plants revisited |
| Simon's curve speed scaler | — | later | into the path-tracking project, or the cap `v ≤ √(a_y,max / κ)` |
