# Open questions for the maintainer

One entry per question that waits for you, ordered by what it blocks. Answer on the `Answer:`
line or in a session. The agent then records the ruling (in the plan doc it shapes and a dated
record in docs/reviews/), applies it, and deletes the entry. Rulings already taken are in
[docs/reviews/](../reviews/); known defects are in [KNOWN_ISSUES.md](KNOWN_ISSUES.md).

**Reviewers** lines quote the three independent reviews of 2026-10-26
([A](../reviews/2026-10-26-independent-review-a-vision-architecture.md),
[B](../reviews/2026-10-26-independent-review-b-api-naming.md),
[C](../reviews/2026-10-26-independent-review-c-plan-roadmap.md)): opinions, not rulings.
Where all three agree, the entry says so, so you can take those as a block. `§N` names the
workstream of [core-architecture.md](../plans/core-architecture.md) and TODO §1.N.

**Where all three reviewers agree.** You could take these as one block, each still your call;
the entries below hold the context:
1. **The 16 overnight decisions:** look at the number changes and the input-box sweep, accept
   the rest, then merge the landed core into `dev`, and each workstream as it lands (A).
2. **The stateless MPC:** refuse it in `problem.compile` now (B).
6. **The run caches:** they go (B).
9. **Programs:** pass `p`, complete trees inside, the seed by the `seed` key, the warm start on
   the solve call (B).
10. **`as_cost` and the shaping:** one function, moved to `core/` first (B).
13. **The native kernel:** a pure kernel with a step-system view (C).
14. **The first fixed-iteration tick:** the gradient MPC; MPPI in v0.4 (C).
15. **The borderline rows:** S70 lands before S66, the realtime re-base
    waits for v1.0, and the RL heads go to v0.4 (B).
16. **LQR and `U`:** no warning (D).
17. **`DynamicBicycle`:** rename only (D).

## v0.3 — A. Gates the merge into `dev`

### Do you review the overnight run's 16 remaining decisions, or accept them as a block?
- **Blocks:** your merge of the v0.3 branch into `dev` (R79).
- **Today:**
  - The run of 2026-10-24 decided 36 points without you, each with its reason, the alternative
    and the commit; its §3 measures the intended number changes.
  - Settled since: OD-9 confirmed, OD-10 superseded, sixteen accepted from chat (R103).
  - Open: OD-7, OD-8, OD-12, OD-14, OD-17, OD-18, OD-20, OD-21, OD-23, OD-24, OD-25, OD-26,
    OD-27, OD-29, OD-30, OD-35. OD-32 and OD-33 are entries in D below.
- **Options:** accept the rest as a block; review each; accept all but the ones you name.
- **Recommendation:** first the log's "please look first at":
  - the number changes: value iteration's default price is now the cost bound, so the GRO860
    cost-to-go colour scale tops there; scores moved from a held law to the continuous loop, by
    up to 8% for a stiff PD;
  - the input-box sweep (OD-18);
  - the planners' reported trajectory (OD-27).

  Then the rest as a block.
- **Reviewers:** all three: that, then merge.
- **Detail:** [the run's log, §1–§3](../reviews/2026-10-24-v03-run-decisions.md#2-decisions)
- **Answer:**

### Does the landed continuous core merge into `dev` now, and each workstream as it lands?
- **Blocks:** every workstream's base. Today the merge is the last row of TODO §1.10.
- **Today:**
  - The continuous core (the step model, the path to the cost, the optimizer table, the PID
    example) landed on `dev-v0.3` on 2026-10-24.
  - `origin/dev` last moved on 2026-10-23.
  - R79 says "merged into `dev` after the maintainer's review"; it does not tie the merge to the
    later workstreams.
  - R47 (nothing to `main` before the notebooks move) is untouched either way.
- **Options:** merge now, then per workstream; merge once, at the end.
- **Recommendation:** none.
- **Reviewers:** all three: merge now, then per workstream (A §2.1, B P7, C §2.2).
- **Answer:**

### What is the `origin/v0.3.0` branch, and does v0.3 ship as one tag or as v0.3.0 then v0.3.x?
- **Blocks:** the release gate (ROADMAP §1 and §5.3).
- **Today:**
  - A branch `origin/v0.3.0` exists at `70f93f3`, the commit before the 2026-10-26
    consolidation; no doc names it.
  - v0.3 has one gate: "when the core is complete".
- **Options:**
  - one tag;
  - v0.3.0: the continuous core, the objects-and-draws lane, the loop lane through the sampled
    face, the program lane through the online call, and the wiring-time failures. Then v0.3.1,
    the one score; v0.3.2, the native proof of concept; v0.3.3, Fields; all before any v0.4
    feature.
- **Recommendation:** none.
- **Reviewers:** C: the split (§3.3), or one tag with per-workstream merges.
- **Answer:**

### Is v0.3's definition of done the acceptance scripts of core-architecture §11?
- **Blocks:** the release gate. The rows whose "done when" is "the rulings are taken" (§5, §6,
  §7).
- **Today:** no definition of done. Seven proposed scripts:
  - an LQG Monte Carlo on one key;
  - a digital PID's gradient;
  - an MPC program rebound;
  - one `J` from `compute_rollout` and `evaluate`;
  - the fixed-iteration MPC tuned;
  - a `QuadraticField` as terminal cost and `V`;
  - every plant and notebook green.
- **Options:** adopt them; amend the list; keep "complete" as the gate.
- **Recommendation:** none.
- **Reviewers:** A (§3.5, one end-state script) and C (§2.1, §3.3) ask for one.
- **Detail:** [core-architecture.md](../plans/core-architecture.md) §11
- **Answer:**

### Are regression baselines re-pinned per intended change, or once at the end of the hybrid loop?
- **Blocks:** CI's regression job through the loop lane (R80–R82).
- **Today:** the order of 2026-10-26 proposed one re-pin of demos, notebooks and baselines after
  the hybrid loop. AGENTS' recipe and RULES 7.7 want byte-identical steps, an intended change
  carrying its own test, and CI reads `benchmarks/baselines/`.
- **Options:**
  - each intended change re-pins the regression baselines in its own commit, with its
    closeness report, and only the student-facing material is batched at the merge to `main`;
  - one re-pin, with the gates off meanwhile.
- **Recommendation:** none.
- **Reviewers:** C §2.5: per change.
- **Answer:**

## v0.3 — B. Gates code on the critical path

In the order to decide (reviewer C §5).

### Which release does each borderline row take, now that v0.3 is the complete core?
- **Blocks:** the order of the core's rows (§1, §5, §1.10).
- **Today:** these are rows the core reads that sit elsewhere:
  - S70, the error family (R114, v0.9), whose types S66 raises;
  - the realtime simulator re-based on the pure blocks;
  - the orchestrator's adaptive plant integration (sampled-path.md §8.4);
  - the RL heads rebuilt on the seeded policy (C5, v0.4).
- **Options:** each joins v0.3's core, or keeps its rung: S70 in v0.9, the realtime
  re-base with the realtime review (v1.0), the RL heads with C5 (v0.4).
- **Recommendation:** none.
- **Reviewers:** all three agree:
  - S70 lands before or with S66;
  - the realtime re-base waits for v1.0's review;
  - the adaptive orchestrator goes to v0.4 or later;
  - the RL heads go to v0.4.
- **Answer:**

### Does `problem.compile` refuse the stateless MPC until the sampled face lands?
- **Blocks:** the hybrid loop (§5); R123 lands with it.
- **Today:** `require_continuous` (`planning/evaluator.py`) refuses a `StepSystem` controller.
  But `MPCStatelessController` is a plain `System`
  (`planning/trajectory_optimization/mpc/controller.py`), so `problem.compile` accepts it, and
  its tick latch then runs at the RK4 stages: a silent continuous-feedback MPC.
- **Options:** refuse it now, naming the hybrid loop; leave it until the sampled face.
- **Recommendation:** none.
- **Reviewers:** all three: refuse it now (one line).
- **Answer:**

### Do the run caches on the sampled loop go?
- **Blocks:** the sampled loop (§5, D13); RULES 2.4, no run state on a System.
- **Today:** `StepSystem.rollout` and `HybridDiagram.last_result` cache a run beside `self.traj`.
- **Recommendation:** they go. `loop.compute_trajectory()` returns the `Trajectory` and stores
  `loop.traj`, and the facades read `traj`.
- **Reviewers:** all three: they go.
- **Detail:** [sampled-path.md §8.6, D13](../plans/sampled-path.md#d13-the-run-state-caches-s31-rules-24)
- **Answer:**

### Is there one keyword per concept: `backend`, `solver`, `method`, `sample_period`, `dt`?
- **Blocks:** every new signature the workstreams write (§1–§8); S65's one-owner rule.
- **Today:**
  - `solver=` is the integrator (`problem.compile`, `discretize`).
  - `method=` is the `Optimizer`'s algorithm.
  - The plan docs wrote `evaluate(..., method="scipy")`, now corrected to `solver=`.
  - `backend=` is the array module on `problem.compile`, but `compile_backend=` on `Optimizer`
    and the planners, where `Optimizer.backend` is the solver adapter.
  - The sample period is `sample_period` (`WhiteNoise`), `dt` (`%`, D29's `GaussianPolicy`),
    `dt_mpc` (the MPC) and `dt_base` (`StepSchedule`).
- **Options:** one rule in DESIGN §4: `backend=` the array module, `solver=` the integrator,
  `method=` the optimization algorithm, `sample_period=` a block's period, `dt=` a tool's grid.
  Renaming the frozen names `compile_backend` and `dt_mpc` (G3, R50) is a separate sweep (R82).
- **Recommendation:** none.
- **Reviewers:** B P3 proposes the rule. A and C: `sample_period` on the seeded policy.
- **Answer:**

### How do programs and solvers take parameters and randomness, and what ships in v0.3? (needs a discussion)
- **Blocks:**
  - the stochastic-program row (§7);
  - the parametric program;
  - the native solvers and the stochastic category of native-optimizers.md.
- **Today:**
  - Only `x0` varies in the compiled MPC program.
  - `pe.program(wrt, key, n_trials)` freezes its draws at build, a sample average.
  - Seeds live in `params` as integers (RD-R6, R118).
  - Overrides are complete trees (R121), while parametric-program.md §3.1 binds partial ones.
- **Your direction (2026-10-26):**
  - every program is deterministic given its params, the seed one of them;
  - solving a stochastic program is solving over several seeds;
  - the stochastic program is the parametric program's case, a core pillar of v0.3.

  Open: §7.5's seven points (bind or pass `p`, partial or complete trees, non-differentiable
  leaves, which leaf is the seed, structural against value parameters, the warm start,
  sensitivities), and §4.1's `J(z, key)`.
- **Options:**
  - **The signature:** not settled. The draft is one signature `J(z, p)`, with the seed an
    integer leaf of `p`:
    - deterministic solvers keep it fixed (a sample average);
    - stochastic solvers advance it per iteration (`child(seed, k)`), under four soundness
      conditions.
  - **The v0.3 scope:** all three ways, or the sample average only, with stochastic
    approximation waiting for the stochastic category (NS-7, v0.4).
- **Recommendation:** none yet; a discussion session first.
- **Reviewers:** all three:
  - pass `p`, binding only at the user boundary;
  - complete trees inside, one merge at the boundary;
  - the seed found by the `seed` key, with integers never in `wrt`;
  - shapes are structural;
  - the warm start goes on the solve call.

  Also:
  - B and C: sensitivities in v0.4;
  - B: drop `J(z, key)`;
  - A and C: only the sample average in v0.3.
- **Detail:** [native-optimizers.md](../plans/native-optimizers.md) §7
- **Answer:**

### Where do `as_cost` and the shaping live, and are the conversions named by what they return?
- **Blocks:**
  - Fields (§9);
  - the sets as costs (§1, was V1 step 8);
  - the workspace geometry (A3, v0.4).
- **Today:** three plans:
  - `Set.as_cost(weight=, shaping=)` (A5; differentiable-problem.md step 8, after `shaping.py`
    moves to `core/`);
  - `Field.as_cost`, with the shaping next to it in `core/fields.py` (fields.md 4.1);
  - the shaping helpers moved to `core/fields.py` (geometry-module.md).
- **Options:**
  - one home and one signature for sets and fields, `Set.as_cost` first and fields following;
  - for the names, `as_constraint` / `as_input_constraint` (student-facing in the track demos)
    or `as_set` / `as_input_set`.
- **Recommendation:** none.
- **Reviewers:**
  - all three: one function, moved first;
  - A: in `core/fields.py`;
  - B: in `core/shaping.py`;
  - C: the shaping to `core/` as a pure move now;
  - B: `as_set(upper=c)` / `as_input_set` / `as_cost`.
- **Detail:** [fields.md](../plans/fields.md) 4.1; [geometry-module.md](../plans/geometry-module.md); [differentiable-problem.md](../plans/differentiable-problem.md) §8
- **Answer:**

## v0.3 — C. Gates later core work

### Does the evaluation unify from the rollout on, with a discretized planning problem? (an investigation for the final design)
- **Your note (2026-10-27): an investigation for the final design, not settled tonight.** Sets and
  costs take time as an argument: does a discrete problem need discrete versions of them, or
  do they bind `t_k = t₀ + k·dt`, and how does a natively discrete cost `c(x_k, u_k, k)` fit?
  Should the evaluation of a continuous system be done exactly as a `StepSystem` on its `dt`, so
  that the trapezoid becomes a plain sum? This is a core part of the final design; investigate
  the cleanest option, with its pros and cons, before a choice.
- **Blocks:** one score (§6); it shapes the sampled face and the sampled loop (§5).
- **Today:** the evaluator's run is already the step model, and its score already reads the
  samples. The same continuous cost becomes a per-step cost three ways: value iteration and RL
  use a left Riemann sum `g dt`, the evaluator a trapezoid. So one policy has three close `J`s.
- **Your idea (2026-10-26, draft):** once `dt` and the scheme are set, discretize the problem
  (time becomes an index in the cost and the sets) and score every stepping loop with one
  discrete evaluator.
- **Your direction (2026-10-26):** a core pillar of v0.3; the refinement stays open.
- **Options:**
  - the per-step cost by integrating the cost with the state (exact, the agent's
    recommendation), or by a quadrature on the samples;
  - the object: `problem.discretize(dt, ...)` returning a new noun (`StepPlanningProblem`);
    the same `PlanningProblem` whose `sys` is a `StepSystem`; or an internal lowering held by
    the evaluator, published when a native discrete consumer needs it.
- **Recommendation:** the direction, yes; refine the six points of the detail before a plan.
- **Reviewers:**
  - all three: integrate the cost with the state; A: under forward Euler it is today's `g·dt`,
    so value iteration's and RL's numbers do not move;
  - A and B: no new noun, the same `PlanningProblem` on a `StepSystem`;
  - C: internal first;
  - C: after the loop lane, not before it.
- **Detail:** [differentiable-problem.md](../plans/differentiable-problem.md) §9
- **Answer:**

### What failure rule, hold and scope does the discrete problem take?
- **Status (2026-10-27):** part of the one-score investigation above; settled with it.
- **Blocks:** one score (§6). The numbers of value iteration, RL and the evaluator move once.
- **Today:**
  - `score_run` integrates to `tf` and checks `X` and `U` at every sample, under the price
    switch.
  - The RL environment stops at the first exit from `X` and checks no `U`.
  - Value iteration prices the cells that leave its grid and ignores a finite `tf` (S63, v0.4).
  - Shooting scores with its own trapezoid and a first-order hold.
  - The feedback planners report their held-input rollout (OD-27).
- **Options:**
  - one rule, absorbing or not;
  - the zero-order hold;
  - shooting's score lowered to the discrete problem, or kept;
  - planner policies acting at a fixed `dt` scored through the sampled face;
  - S63 with the score, or in v0.4.
- **Recommendation:** none.
- **Reviewers:** all three:
  - `score_run`'s rule, to the horizon, with the RL early stop a training choice that reports
    the score's `J`;
  - the zero-order hold;
  - shooting's reported score lowered;
  - S63 in v0.4.
- **Detail:** [differentiable-problem.md](../plans/differentiable-problem.md) §9.4
- **Answer:**

### Native optimizers: is the solver a pure kernel with a step-system view, or a `StepSystem` itself?
- **Blocks:** the native proof of concept (NS-1, NS-2's fixed-`K` form; §8) and the teaching
  face (NS-5, v0.4).
- **Your direction (2026-10-26):** a proof of concept in v0.3, a simple fixed-step solver and an
  MPC on it; better solvers in v0.4. The kernel's shape stays open.
- **Today:** `gd` and `adam` are a Python loop in `optimizers/first_order.py`. A hand-wired
  `StepDiagramSystem` of a law `z⁺ = z − η u` and a gradient oracle already rolls out gradient
  descent.
- **Options:**
  - (A) every native solver a `StepSystem` by construction;
  - (B) a pure `init` / `step` kernel run by `Optimizer`, with the step-diagram loop as a
    teaching view of it;
  - (C) kernel only.
- **Recommendation:** B. A forces Adam's pytree state into a flat `x`; C loses the
  step-size-as-stability lesson.
- **Reviewers:** all three: B, with the view arriving with NS-5 in v0.4.
- **Detail:** [native-optimizers.md](../plans/native-optimizers.md) §3
- **Answer:**

### Which fixed-iteration tick comes first: the gradient MPC or MPPI's?
- **Blocks:** the native proof of concept (§8) and MPPI (S72, TODO §4).
- **Today:** native-optimizers.md §4.6b and §5 say "MPPI is the first instance, ahead of NS-8",
  because its tick is already pure and needs no native solver. Your direction of 2026-10-26 puts
  the gradient MPC's proof of concept in v0.3 and MPPI in v0.4.
- **Options:**
  - the gradient MPC first, as the direction places it;
  - MPPI's tick first, as the sampled loop's first pure controller, MPPI's planner still v0.4;
  - both together.
- **Recommendation:** none.
- **Reviewers:** all three: the gradient MPC as directed; the sampled kernel's first consumer is
  the digital PID; MPPI stays in v0.4; fix native-optimizers' wording.
- **Detail:** [native-optimizers.md](../plans/native-optimizers.md) §4.6, §4.6b, §5
- **Answer:**

### Do all of Fields' steps land in v0.3?
- **Blocks:** Fields (§9).
- **Today:** fields.md's steps 4.1–4.8 are one row, A1. 4.1–4.3 fix a public signature and the
  Lyapunov `V`; 4.4–4.6 (`GridField`, the approximator, `CallableField`) are the tables and the
  critics.
- **Options:** all in v0.3; 4.1–4.3 in v0.3 and 4.4–4.6 in v0.4.
- **Recommendation:** none.
- **Reviewers:** C §3.2: the split.
- **Answer:**

### Should the failure price follow a run's cost overrides?
- **Blocks:** programs over cost weights (`pe.program(wrt=["cost…"])`; the parametric program,
  §7).
- **Today:** the price keeps the problem's own weights. With no exact bound, it costs 4096 calls
  of `g` at every `evaluate` and program iteration; a cache keyed on the cost's parameters would
  remove that.
- **Options:** price from the overridden weights, with the cache; keep the problem's price.
- **Recommendation:** none yet.
- **Reviewers:** split:
  - A: keep the problem's price, computed once at compile;
  - B and C: follow the overrides with the cache, or a tuned weight can price a failed run
    below a feasible one.
- **Detail:** [the run's log, §6 item 6](../reviews/2026-10-24-v03-run-decisions.md#6-the-review-pass-after-the-run)
- **Answer:**

### Does the geometry `bind` keep its name, now that programs may bind parameters?
- **Blocks:** the program API (§7) and the workspace geometry (A3, v0.4). It is a public name.
- **Today:** `bind` means three things:
  - `minilink.planning.bind(sys, geometry, frame="body")` mounts collision geometry on a plant,
    on the teaching facade;
  - `ParametricProgramEvaluator.bind(x0)` binds the measured start;
  - an RL algorithm's `bind(policy, critic, gamma)`.

  The program plan adds a fourth.
- **Options:** rename the geometry verb (`mount(geometry, on=sys, frame=)` or `attach`); keep
  `bind` and drop it from the program API; keep both.
- **Recommendation:** none.
- **Reviewers:**
  - B: drop `bind` from programs and rename the geometry verb `mount`;
  - C: rename it `attach` and keep `bind` for parameters.
- **Answer:**

## v0.3 — D. The clean-up lane; blocks no core code

### Should compile-time diagram flatten default to on after the parity and speed study?
- **Blocks:** nothing until benchmarks and tests are stable. The default stays `False` until
  you decide.
- **Today:** `compile_diagram(..., flatten=False)` inlines nested boxes with `__` ids
  (`inner__plant`). Parity tests and `benchmarks/scenarios/nested_compile_flatten.py` compare
  the nested compile, `flatten=True` and hand-flat wiring.
- **Options:** keep the default off; turn it on if flatten wins on compile and/or `f()`; drop
  the flag and always flatten at compile.
- **Recommendation:** wait for one informational benchmark run on your reference machine, then
  choose.
- **Reviewers:** A: then prefer dropping the flag if parity holds. B and C: it blocks nothing,
  standing.
- **Detail:** [DESIGN.md](DESIGN.md) (compile-time flatten paragraph)
- **Answer:**

### Do the three demos that teach a hand-written gradient loop move onto the `Optimizer`?
- **Blocks:** the rest of the optimizer work (S74's rest, OD-32).
- **Today:** `cartpole_rollout_gradients`, `neural_controller_jax` and `params_gradient`
  (`examples/demos/compile/`) descend by hand. The first tunes an open-loop schedule, which the
  evaluator cannot tune until a source's samples are parameters. R90 kept the hand loop in two
  places on purpose.
- **Options:** convert all three; convert two, the schedule demo later; keep all three.
- **Recommendation:** none yet (converting changes what they teach).
- **Reviewers:** your teaching call. A and B: convert two, keep one hand loop. C: keep all three
  (R90).
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
- **Reviewers:** all three: no warning. Say "U ignored" in the LQR record's `str` or docstring
  (this amends R21).
- **Detail:** [the run's log, OD-18, OD-33](../reviews/2026-10-24-v03-run-decisions.md#2-decisions)
- **Answer:**

### One mechanical family: does `DynamicBicycle` move onto the generalized mechanical base now?
- **Blocks:** one mechanical family (S32); the car family clean-up (K8).
- **Today:** the row moves its base from `DynamicSystem` to the generalized child, but its named
  ports and its subclasses' extra states do not fit `[q; v]`.
- **Options:** move it now; rename `M` to `H` only and move it when K8 reworks the cars.
- **Recommendation:** rename only; the base waits for K8.
- **Reviewers:** all three: rename only.
- **Detail:** [TODO, S32](../plans/TODO.md#2-v03--the-clean-up-lane-beside-the-core)
- **Answer:**

### Do three demos change style?
- **Blocks:** your demo pass (K4). The closing example (S77) landed.
- **Today:**
  - `pid_autotuning_jax.py` keeps two helpers, prints that carry arithmetic, and a hand-made
    last figure.
  - `diagram_shortcuts.py` labels `NoisyPendulum()` as `Pendulum()`.
  - `custom_one_link_arm.py` reads module constants, not `params`, and a comment restates a
    line.
- **Recommendation:** none yet.
- **Reviewers:** your demo pass. B: fix the `NoisyPendulum()` label now, since it is a wrong
  label, not style.
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
| The default output stub builds a NumPy array under JAX | T1 | v0.3 (the core, §1) | the returned type of the default `h` (`np.zeros(p)`) |
| The demo sweep to the flatness rule | D1.3 | standing (wave D) | your review of each diff; `showcase_jax` waits |
| The planning demos on the solution verbs | D1.4 | standing (wave D) | `solution.plot_*` and `compare(...)` in place of hand-rolled comparisons |
| `Source.show_signal` | D2 | standing (wave D) | remove it (the same picture is `plot_trajectory`) |
| Two dead modules | D2 | standing (wave D) | remove `planning/spatial/overlays.py` (with A3) and `experimental/symbolic/mechanics/utils.py` |
| A connected input rewired silently | S66 | v0.3 (the core, hardening) | `connect` refuses to rewire it |
| Your demo pass, and a tabular Q-learning demo | K4 | v0.3 (the clean-up lane) | your pass; the demo beside value iteration (R65) |
| The noise seeds on the draws | V1 step 7 | v0.3 (the core, §3) | one seed per random block in `problem.realize` (RN-4, RN-5, AC-7) |
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
