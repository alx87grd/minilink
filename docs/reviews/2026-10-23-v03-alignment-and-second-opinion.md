# The v0.3 alignment pass and a second opinion (2026-10-23)

**Audience:** Prof. Alexandre Girard (Maintainer)
**Context:** after the rulings of 2026-10-22/23 (R1–R29,
[record](2026-10-22-rulings-session-1.md)), the plan docs that
[continuous-core.md](../plans/continuous-core.md) supersedes were aligned to it and its steps
written as workboard rows (TODO §4, §5). In parallel an independent reviewer, given the
documents and the code but not this pass's conclusions, read the design afresh. This record
holds what both found: the inconsistencies fixed in place, and the points raised for the
maintainer (listed in ROADMAP §6, "Raised 2026-10-23"). Nothing here is ruled.

## 1. What the alignment pass did

- **The plan docs.** differentiable-problem.md rewritten to the design (the detail of V1: the
  target, the implementation rules, the guardrails, the steps); sampled-path.md, cost-params.md
  and randomness.md's evaluator step aligned; the `tune`, `score_trajectory`, `failure_rate`
  and engine-verb leftovers replaced in the other plans.
- **The workboard.** TODO §4: the step model as SM-0 to SM-7 (SM-7, `compute_forced` on the one
  rollout, is new: R17 and R18 had no row), V1's steps 1–6 with gates, A2, S31 after the core,
  S74 deferred, one "Raised" row; §5: V1's step 7 (the draws) had no row and has one.
- **The roadmap.** §1, §3, the gantt, §5.3 (the continuous core replaces waves S and V), §5.4,
  §6 (the two sessions' blocks replaced by what they ruled; the raised points added).

## 2. Inconsistencies fixed in place

| Where | What it said | Now |
| --- | --- | --- |
| continuous-core.md §2 | "gone: … `evaluate_trajectory`" (the ruled scoring verb) | gone are the cost's four members; `score_trajectory` becomes the problem-level `evaluate_trajectory` |
| continuous-core.md §3.1, sampled-path.md §3.6 | a `StepSystem`'s `step` has "the same signature" as the compiled `step(x, u, t, params)` | the `StepSystem` takes the tick `k` (R14); `discretize` binds `t_k = t0 + k·dt` |
| continuous-core.md §3.4 vs R9 | `tf=None`: no `h` | no `h`, and a nonzero `h` warns once (R9) |
| continuous-core.md §7 vs R19 | "the cut at exit becomes the integral to `tf`" listed outside the phase | the single-run scorer changes in v0.3 (V1's step 2); the Monte Carlo re-pin is step 7 |
| continuous-core.md, end-state-api.md D9 | `dt=None`: "0.01 fallback" | the code's fallback (1e-4 s); raised (§3) |
| ROADMAP §5.3, §6, TRL table; TODO C4 | `tune`, "the engine under `tune`", S74 as V1's step 5, session 2 open | the program export (R23); S74 deferred (R25); the sessions ruled |
| TODO S76 row | builds on "S74's backend factory (landed in v0.3)" | S74 is deferred, so it must land before S76 |
| end-state-api.md header | "deleted once the sessions have ruled" | kept as the ledger and the home of D13–D15 and D25–D30 until those are ruled |
| continuous-core.md §3.3, cost-params.md vs end-state-api.md D16b | `integrate` returns `J(t_k)` / returns `(cost_rate, cost)` | both arrays, one body (what `with_cost` attaches) |
| automation-by-convention.md, TODO AC-7 | AC-7 whole in v0.4 | its deterministic half with V1's steps 2–4 (v0.3) |
| TODO V1.4 | the key `plant` (D19) with no step renaming `ProblemParameters.system` | the rename in V1's step 4 |
| randomness.md §6 | `.solve().ctl` | `.policy` (the field since 2026-09-15) |
| randomness.md RD-D11, RD-D24, RN-5 | the evaluator as a loop of `Simulator` runs on a NumPy test set | superseded in part by R10, R15, R27: the compiled run is the judge, one key realized once |

## 3. The points raised, merged and ranked

> **Ruled the same evening** as R30–R47 ([the rulings record](2026-10-22-rulings-session-1.md),
> "Session 3"): items 1–7, 10–17, 19, 20 and 22, some with a different answer than recommended
> below (the price became one switch read by every pipeline, R32; the problem never fills `r`,
> R38; `solver=` for time, R42; the names `cost`, `J`, `objective`, R45; no pin, R47). Open:
> items 8, 9, 18, 21; items 23 and 24 are agent-managed.

From both sources (marked *pass* and *2nd*), checked against the code where they make a claim
about it. ROADMAP §6 "Raised 2026-10-23" lists them in one line each; nothing is ruled.

### Major: the scoring formula, before V1's step 2

The rulings changed direction several times in two days (R10 → R15, R9 → R19, R26 → R27, R19's
goal reconciliation → R29), and the one formula that scores a run has not been run against the
code. The second opinion's advice, which this pass shares: **a one-day spike of
continuous-core.md §4.3 and §4.6 on the real pendulum before V1's step 2**, which would have
caught items 1 to 4.

1. **A diverging run scores `NaN` and counts as feasible** (*2nd* M1, *pass*; checked). Today's
   JAX evaluator ends a run whose state is not finite (`planning/evaluation.py`, the `diverged`
   line); "no mask, no stop" removes that guard. A NaN state gives `margin < 0` false, so
   `failed` is false, and one such trial poisons the mean, the CVaR, `criterion_value` and
   every gradient. *Recommended:* keep the integral to `tf` and add one rule: a run that is not
   finite is failed and its cost is replaced by the price (with a safe `where`, so the gradient
   stays finite). The program export then never sees an infinite objective.
2. **The default price is `+inf` under the default sets** (*2nd* M2; checked). With `X` and `U`
   unbounded by default (R21), `g_max(X, U) · tf` is `+inf` (today's bound returns `inf` with no
   finite box, `planning/reinforcement_learning/environment.py`); `inf · 0` is NaN, so the
   literal formula makes successful runs NaN too. The bound also ignores `h`, so a failed run
   can be cheaper than a successful one with a heavy terminal weight. *Recommended:* default
   `g_max · tf + h_max`; when that is not finite and a set can fail, refuse at construction with
   the line to write; price with `where`, never a product with a boolean.
3. **`x_goal` still builds a singleton `Xf`, so nothing is ever reached** (*2nd* M3; checked:
   `planning/problems.py`, `Xf = SingletonSet(x_goal)` when `Xf` is not given). A closed loop
   never lands exactly on a point, so `reached` is false on every run and every run is priced.
   R29 says `x_goal` is a point of `Xf` but no ruling removes this default. *Recommended:*
   `x_goal` never creates `Xf`; a terminal set is written (`Xf=BallSet(x_goal, r)`); trajectory
   optimization's terminal equality then needs an explicit `Xf=SingletonSet(...)` (a ledger line,
   its numbers change). Maintainer-owned.
4. **The program counts infeasibility twice** (*2nd* M5). The objective is the priced
   `criterion_value` and the margins are also `g ≥ 0` rows (TODO V1.5): the objective jumps by
   `infeasible_cost` exactly at the constraint boundary. *Recommended:* the program's objective
   is the criterion of `cost` (the plain integral) with the margins of the declared sets as rows;
   the priced number stays the report's and `jax.grad`'s target.
5. **`evaluate`'s automatic `dt` is 1e-4 s on almost every plant** (*pass*, *2nd* M4; checked:
   `DEFAULT_SMALLEST_TIME_CONSTANT = 0.001` in `core/system.py`, a tenth of it in the fixed-step
   rule). 100 000 RK4 steps per 10 s run, per trial; today's Monte Carlo evaluator uses 0.05. With
   a fixed step as the judge (R15), the step is part of the answer. *Recommended:* `dt` required
   on `evaluate` and `compare`, as on `problem.compile`.

### Major: the calendar

6. **The course audit (G1) has not started** (*pass*, *2nd*). It is the one input with a hard
   date, and it decides whether item 9's collisions are real. *Recommended:* three days this
   week, before wave K, one output: "what each GMC714 lesson needs by January 5".
7. **The Colab cell clones `main` with no tag** (*2nd*; checked: all 13 GRO860 course notebooks
   run `git clone https://github.com/alx87grd/minilink`). The calendar rule ("a step that moves
   a running course's numbers ships behind the pinned release the course runs on") assumes a pin
   that does not exist; the first number change merged to `main` reaches the running course.
   *Recommended:* pin the clone on the `0.2.x` tag (V6, pulled forward) before `dev` merges to
   `main`. Student-facing, maintainer-owned.
8. **The scope of v0.3** (*2nd*). 8.5 weeks; the bottleneck is the maintainer's attention (K1's
   table, per-plant and per-item rulings, five intended number changes to sign off, two weeks of
   lessons in exam period), not agent hours. *Proposed cuts to v0.4:* SM-7 (the forced-hold
   change, no consumer asks for it), SM-6's move of the integrators (relocation), A2 (research
   lane), K4 and K8-3 (between terms anyway), S31 now rather than "if room". *Keep:* SM-0 to
   SM-5, V1.1 to V1.5, K1, K8-1, K8-2, K9. *And:* freeze the GMC714-facing API on December 1.
9. **The v0.3 / v0.4 line against GMC714** (*2nd* C6, *pass*). MPC is a sampled controller,
   refused by `evaluate` in v0.3, with S31 "if room" and `HybridSimulator` moving under it in
   November (SM-4); robust control is the draws (v0.4, during the GMC714 term). G1 answers
   whether those lessons need `evaluate`, `compare` or the draws; if yes the split moves.

### Moderate

10. **The Monte Carlo evaluator keeps the old rule for one release** (*pass*). V1.2 changes the
    record and the formula; the class keeps its cut-at-exit copy until step 7 (v0.4): two
    scorers for one run. *Recommended:* pull the fold (step 7 without RN-4's per-block seeds)
    into v0.3 if item 8's cuts make room; otherwise accept it for one release.
11. **`U` unbounded by default changes trajectory optimization** (*2nd* C1; checked:
    `plant.input_box` does not exist in `minilink/`). Today `U` defaults to the input ports' box,
    and V1.2's gate wants the trajopt baselines byte-identical. The ±5 mechanical default the box
    would read is S64's open question. *Recommended:* R21 for `X` only, `U` keeps the input box,
    S64 ruled before V1.2; or R21 as ruled with the trajopt numbers re-pinned and `input_box`
    added.
12. **`hold` on a one-step map** (*2nd* C2). A first-order hold needs `u_{k+1}`, which
    `step(x, u, k)` never receives; today's linear input lives only in the rollout. And R18's
    forced `zoh` makes RK4 first order in a callable `u(t)`. *Recommended:* `hold` on `rollout`
    and `compute_forced` only (a step map is a zero-order hold by definition); a callable `u(t)`
    evaluated at the stage times.
13. **The reference from `x_goal`** (*pass*, *2nd* C4). `r` has the measured output's size (1
    for a PID on the pendulum), `x_goal` the state's (2). *Recommended:* `r = h(x_goal)` on the
    measured port, refused otherwise with the built-loop way out.
14. **Scoring a role-less trajectory by its dimension** (*2nd* C5). It contradicts "roles,
    never inference": a static-controller loop has the plant's `x` size and `u = r`, so a
    role-less copy is scored on the reference. *Recommended:* the trajectory carries its static
    layout (pytree aux data, persisted by save/load); a role-less one is accepted only without
    signals and with `u` of the plant's size.
15. **One home for the parameters** (*2nd* C3). The cost's live in `CostFunction.params`,
    `problem.params.cost` (`ProblemParameters`) and `pe.params["cost"]`; the sets' have none.
    *Recommended:* the object's own `params`, as on `System`; `ProblemParameters` retired;
    `pe.params["sets"]` when A5 lands.
16. **`method=` against the Simulator's `solver=`** (*2nd* C7). The Simulator's keyword is
    `solver=` (`"scipy"`, `"scipy_stiff"`, `"rk4_fixedsteps"`, …); `"scipy"` is RK45, not the
    stiff escape. *Recommended:* rule the one word and its values across `Simulator`,
    `compute_forced`, `evaluate` and `discretize` before SM-2 adds the third.
17. **An exit priced two ways** (*pass*). The RL environment ends an episode at the first exit;
    `evaluate` integrates to `tf`. *Recommended:* RL's termination the documented exception (its
    bootstrap needs the episode end), with a test that the two agree on a run that never exits.
18. **The slots of S31 and S74** (*pass*). S31 "if room" (item 8 says v0.4 now); S74 deferred
    with no slot while S76 needs its backend factory: the first v0.4 row before S76.

### Minor

19. `criterion_value` to confirm; and `value` carries three meanings (`Field.value`, the priced
    run cost, the criterion) where the course notes and `dp.py` say `J` (*2nd*). Consider
    `report.J` per run and `report.objective` for the scalar, `report.cost` the plain integral.
20. `evaluate` on a stochastic problem in v0.3 with `n_trials = 1`: refused (the message names
    `MonteCarloEvaluator`) unless item 10 pulls the draws in (*pass*).
21. The `params["dt"]` fallback of `discretize`: R7 says `dt` is required; drop it with a ledger
    line (*pass*).
22. The autotuning demo's gain regularizer (`1e-4 ‖K‖²`) is not a `g(x, u, t)`: the program needs
    a documented term on `z`, or the demo's numbers change (*2nd*).
23. V1.1, the native `Trajectory`, is a core rewrite (72 constructor calls, the `(n, N)` layout,
    `check_samples` refusing a run axis) and V1's critical path; split it when it starts (*2nd*).
24. The rulings record lets R29 reverse R19's goal reconciliation and R15 reverse R11's
    "`evaluate` runs the Simulator" without saying "supersedes" (*2nd*); the record is history,
    so this note is the marker.

## 4. Fixed after the second opinion

- The cascade example (continuous-core.md §4.7, end-state-api.md D16c) used a wiring dialect
  that does not exist (`"a:b"` strings, `slice=`, the name first); rewritten in today's calls
  (`add_subsystem(sys, sys_id)`, `connect(src, port, dst, port)`, `expose_input`, a `Demux` for
  the velocity). `DCMotor`, `Arm` and `Quantizer` stay illustrative names.
- `record=False` added to the two `evaluate` signatures that the examples call with it.
- end-state-api.md D1: the problem statement as the third foundation (R1 amended).
- Who binds `t_k`: `discretize`, stated once (the code today computes `k · dt` with no `t0`;
  SM-2 adds it).

## 5. Appendix: the second opinion as delivered

Read on `dev` at `d29507aa` plus the working tree mid-pass; line numbers are that tree's. Kept
verbatim as the evidence for §3.

Read on `dev` at `d29507aa`, plus the uncommitted alignment pass in the working tree (ROADMAP,
TODO, continuous-core.md and five plan docs were being edited while I read). That pass already
fixes several stale points: `tune` is gone from ROADMAP §1, the S74 slot, the stochastic half
moved to v0.4, the `dt` fallback stated as 1e-4. Line numbers are the working tree's unless
marked HEAD. No repo file was edited.

**Headline.** The architecture is sound and I would keep most of it. But the one evaluation of a
run (the scoring formula that rulings R19 and R21 fixed) has four defaults that, together, give
`inf` or `NaN` on the most ordinary student problem. Nobody has run the formula against the code
yet, and the rulings changed direction several times in two days: R10 to R15, R9 to R19, R26
to R27, R19's goal reconciliation to R29. Before more plan prose, a one-day spike of
continuous-core §4.3 and §4.6 on the real pendulum would catch all four. G1, the GMC714 course
audit, has not started. It is the only item with a hard external date.

---

#### Major

##### M1. A diverging run scores `NaN` and counts as feasible
- **Claim.** "Integrated to `tf`; no event, no mask, no stop" and "a NaN from the model is a model
  bug" (continuous-core.md:222–257; end-state-api.md D16).
- **Evidence.** Today's JAX evaluator guards against exactly this: `diverged =
  ~jnp.all(jnp.isfinite(x_next))` ends the run (planning/evaluation.py:191–196). An unstable gain
  set, or an RK4 step too coarse for a high-gain loop, overflows to `inf`, and then `inf − inf` is
  `NaN`. That is not a model bug. With `NaN` states, `margin < 0` is `False`, so `failed = False`;
  `reached = False`; and `value = NaN`.
- **Why it matters.** One diverging trial poisons `mean`, `cvar`, `criterion_value` and every
  gradient. It also inflates `feasibility_rate`, since the `NaN` run counts as feasible. An
  optimizer that explores gains will hit this, and so will a robust Monte Carlo.
- **Recommendation.** Keep the integral to `tf`, and add one rule to the formula: a run that is
  not finite is failed, and its cost is replaced by the price.

  ```python
  # ruled
  failed = feasibility_margin < 0
  value  = cost + infeasible_cost * (failed | ~reached)
  # proposed (the finite guard; implement with the safe-where so grad stays finite)
  finite = xp.all(xp.isfinite(x))
  failed = ~finite | (feasibility_margin < 0)
  value  = xp.where(finite, cost, 0.0) + infeasible_cost * (failed | ~reached)
  ```

##### M2. The default price is `+inf` under the default sets
- **Claim.** `X` and `U` default to unbounded (R21). `infeasible_cost` defaults to
  `cost.g_max(X, U)·tf` (R10, continuous-core.md:216).
- **Evidence.** `g_max` over an unbounded box is `+inf`; today's bound returns `np.inf` when no
  finite box exists (planning/reinforcement_learning/environment.py:320–323). With `tf=None` and
  `ρ=0`, the `/ρ` form is `inf` as well. In IEEE arithmetic `inf·0 = NaN`, so the literal formula
  makes every *successful* run `NaN` too.
- **Also.** `g_max·tf` does not bound a feasible run's cost when `h ≠ 0`. A run that fails can
  then be cheaper than one that succeeds with a heavy terminal weight. That breaks the
  penalty-method promise the docstring makes (environment.py:311–313).
- **Recommendation.** The default should be `g_max·tf + h_max` over `Xf` or `X`. When that is not
  finite and the problem declares a set that can fail, refuse at construction:
  `"X, U are unbounded: no finite price; pass infeasible_cost=… or X=plant.state.box"`. Never
  multiply by a boolean; use `where`.

##### M3. `x_goal` still builds a singleton goal set, so nothing is ever "reached"
- **Claim.** `x_goal` is "one point of `Xf`"; `Xf=None` means no terminal constraint (R29,
  continuous-core.md:212–213).
- **Evidence.** The code sets `Xf = SingletonSet(x_goal)` whenever `x_goal` is given and `Xf` is
  not (planning/problems.py:119). A singleton's margin is `−|x − p|`, which is ≥ 0 only at
  equality (core/sets.py:261–270). No design doc rules on this default, and the R29 loop rule
  *needs* `x_goal` to fill `r`.
- **Why it matters.** `PlanningProblem(plant, cost=cost, x_goal=[π, 0], tf=5)` gives
  `reached = False` on every closed-loop run. So `value = cost + infeasible_cost` on every run,
  which is `inf` with M2's default. The same default is what makes trajectory optimization treat
  `x_goal` as a hard terminal equality (field inventory, `Xf` row).
- **Recommendation.** Decouple the two: `x_goal` never creates `Xf`. A student who wants a
  terminal set writes `Xf=BallSet(x_goal, r)`. Trajectory optimization's terminal equality then
  needs an explicit `Xf=SingletonSet(...)` and a ledger line. This is a maintainer ask; it
  changes TO's numbers.

##### M4. With `dt=None`, `evaluate` steps at 1e-4 s on almost every plant
- **Claim.** `dt=None` means "the Simulator's automatic rule" (R15). HEAD said "0.01 fallback";
  the working tree now says 1e-4 and raises it as open (continuous-core.md:271–272).
- **Evidence.** `DEFAULT_SMALLEST_TIME_CONSTANT = 0.001` (core/system.py:19) and
  `AUTO_DT_SCALE = 0.1` (simulation/time_grid.py:16). Any dynamic subsystem contributes that
  default (core/wiring.py:449–458). Only the racecar, the oscillators and three-body declare their
  own. So `evaluate(problem, PID(...))` runs 100 000 RK4 steps over 10 s. On NumPy that is a
  per-step Python loop. On JAX, `jax.grad` over 200 trials stores residuals for 2·10⁷ steps.
  Today's `MonteCarloEvaluator` uses `dt=0.05`.
- **Why it matters.** It hurts the teaching surface's "laptop, standard Python" guarantee
  (CONSTITUTION §3). The reported number also depends on `dt` for the switching laws GMC714
  teaches (sliding mode sets `discontinuous_behavior`, control/modelbased.py:167). Making a fixed
  step the judge (R15) is defensible, but then the step is part of the problem's answer and must
  be explicit.
- **Recommendation.** Make `dt` required on `evaluate` and `compare`, as it already is on
  `problem.compile(arg, dt)`: one fact, explicit data flow (CONSTITUTION §4.4). Failing that, use
  a closed-loop rule, `dt = 0.1 / max|λ(A_cl)|` from `linearize`, which is a textbook rule that
  sees the controller's own dynamics.

##### M5. The program counts infeasibility twice: penalty in the objective and margin rows
- **Claim.** The objective is `criterion_value`, that is the *priced* `value`
  (continuous-core.md:267). The margins are also the program's constraint rows (continuous-core.md:256;
  TODO V1.5: "the per-sample margins of `X` and `U` as `g ≥ 0` rows").
- **Why it matters.** The objective jumps by `infeasible_cost` exactly at the constraint
  boundary. SLSQP started from an infeasible point sees a cliff and gets no gradient from it,
  while the rows ask it to cross. With an unbounded `X` the rows are `+inf`.
- **Recommendation.** Pick one form per program:

  ```python
  pe.program(wrt)                      # ruled: J = criterion(value), g = margins
  pe.program(wrt)                      # proposed: J = criterion(cost), g = per-sample margins of the declared sets only
  pe.evaluate(p).criterion_value       # the priced number stays the report's and jax.grad's target
  ```

---

#### Moderate

##### C1. Flipping `U` to unbounded silently changes trajectory optimization
- **Evidence.** Today `U` defaults to the input-port box, "an actuator limit is part of the plant
  model" (problems.py:57–59, 117). Mechanical plants carry an invented ±5 (dynamics/abstraction/mechanical.py:66).
  R21 makes trajectory optimization unbounded in `u` "as the problem says". Yet V1.2's gate
  requires "trajopt baselines byte-identical" (TODO V1.2).
- **Coupling.** `plant.input_box`, the line R21 tells users to write, does not exist (no
  definition in `minilink/`). The ±5 default it would read is itself an open decision (S64,
  catalog hygiene, ROADMAP §6).
- **Recommendation.** Leave `U`'s default as the input box (R21 for `X` only). Rule S64 before V1.2.

##### C2. `hold` belongs to an input sequence, not to a one-step map
- **Evidence.** `ev.step(x, u, t, params, *, hold=…)` and `discretize(..., hold="foh")`
  (continuous-core.md:149, 166). A first-order hold needs `u_{k+1}`, which `step(x, u, k)` never
  receives; today's FOH exists only in the rollout (`rk4_integrate_linear`).
- **R18.** Forcing `zoh` on `compute_forced` with a callable `u(t)` also degrades RK4 to first
  order in the input. The ruled change makes forced responses *less* accurate.
- **Recommendation.**

  ```python
  plant_k = discretize(plant, dt, method="rk4", substeps=1)    # a step map holds u: ZOH by definition
  ev.rollout(x0, u_seq, t0, dt, params, hold="zoh" | "foh")    # the hold is the sequence's
  plant.compute_forced(lambda t: np.sin(t), method="rk4")      # a callable: evaluated at the stage times, no hold
  ```

##### C3. Three homes for the cost's parameters; no home for the sets'
- **Evidence.** `CostFunction.params` (D16b), `problem.params.cost` through `ProblemParameters`
  (problems.py:90; TODO A2 "pass `problem.params.cost` through"), and `pe.params["cost"]`.
  `pe.params` has no `sets` entry, although V1 promises gradients with respect to "the sets"
  (TODO V1 HEAD). No design doc says whether `ProblemParameters` survives.
- **Recommendation.** One home: the object's own `params`, as on `System`. `ProblemParameters`
  is then retired (a maintainer ask), and `pe.params` gains `"sets"` when A5 lands.

##### C4. "`r` filled from `x_goal`" has a dimension gap
- **Evidence.** `PID(...) @ Pendulum()` exposes `r` of dim 1, against a state of dim 2 (checked in
  this environment), and the demo tracks θ only.
- **Recommendation.** State the map: `r = h(x_goal)` on the measured port, or the measured state
  slice. Refuse anything else with the line to write.

##### C5. Scoring by the `plant` role still infers when the role is missing
- **Evidence.** "A trajectory whose `x` has the plant's dimension and no roles is taken as the
  plant's" (continuous-core.md:246) contradicts "Roles, never inference" (:426). A loop with a
  static controller (`StateFeedbackController`) has the plant's `x` dimension, and its boundary
  `u` is `r`. A role-less copy of it, from `Trajectory.load` or a rebuilt `Trajectory(t, x, u)`,
  is then scored on the reference instead of the command, silently.
- **Also.** `traj.of("plant")` needs the diagram's slices, so a `Trajectory` is no longer "arrays
  and nothing else" (:422).
- **Recommendation.** `Trajectory` carries a static layout (pytree aux data), and save/load
  persist it. A role-less trajectory is accepted only when it has no signals and `u` has the
  plant's `m`; otherwise refuse.

##### C6. The v0.3/v0.4 split collides with what GMC714 will teach
- **MPC.** It is a sampled controller, so `evaluate` and `compare` refuse it in v0.3, and the
  sampled loop (S31) is "if room". HybridSimulator moves onto the step model in November (SM-4),
  right under December's MPC lesson.
- **Robust control.** That is the draws (`params_distribution`), which are v0.4. v0.4's "numbers
  change once" lands in winter 2027: that is the GMC714 term itself. The calendar rule names only
  GRO860 (ROADMAP §5).
- **Recommendation.** G1 must answer whether the MPC and robust lessons need
  `evaluate`/`compare`/draws. If yes, the split is wrong and S31 plus the draws must be scoped
  now, not discovered in December.

##### C7. Two words for one knob
- **Evidence.** R6 says `method=` is "the word `Simulator` … uses". The Simulator's public
  keyword is `solver=` with `"scipy"`, `"scipy_stiff"`, `"rk4_fixedsteps"` and others
  (simulation/simulator.py:28–71; core/facades.py:138).
- **Also.** `method="scipy"` is RK45, so it is not "the escape for stiff or switching loops"
  (end-state-api.md:281); that escape is `"scipy_stiff"`.
- **Recommendation.** Rule the one word, and its value list, across `Simulator`,
  `compute_forced`, `evaluate` and `discretize` before SM-2 adds the third.

---

#### Minor and inconsistencies

- **The §4.7 cascade is written in a wiring dialect that does not exist.** It uses
  `add_subsystem("motor", DCMotor())` with the name first, `connect("a:b", "c:d")`, `slice=`,
  `add_input_port("r", "pos_ctl:r")` (continuous-core.md:360–371; end-state-api.md:507–516).
  The code has `add_subsystem(sys, sys_id)`, `connect(src, port, dst, port)` and no slice
  (core/wiring.py:79, 96, 122); `DCMotor` and `Quantizer` do not exist. Either fix the example
  or rule a new dialect, which CONSTITUTION §6 forbids.
- **Who binds `t_k`.** continuous-core.md:179 says it is "the loop's business"; :167–168 and
  sampled-path SM-2 say `discretize` binds it. The code computes `t_k = k·dt` with no `t0`
  (analysis/discretization.py, `step`).
- **`record=True`** is used (continuous-core.md:233, 338) but appears in neither signature
  (:266, :270).
- **Two foundations or three.** end-state-api.md:141–142 and ROADMAP.md:547 still say two;
  continuous-core.md:25 and amended R1 say three.
- **Stale ROADMAP §6.** ROADMAP.md:550 still reads "ODE solver as the judge … engine under
  `tune`", which R15 and R23 superseded. "Blocking code now, session 2" (ROADMAP.md:556) is
  listed as open although R13–R29 ruled it. The "Raised 2026-10-23" list it points to is not
  written yet.
- **Reversals not marked.** The rulings record lets R29 reverse R19's "x_goal and cost.xbar are
  reconciled at construction" (record:152 against :227) without saying "supersedes". R11's
  "`evaluate` runs the Simulator" (:79–80) is likewise left unmarked under R15.
- **`criterion_value`** is used everywhere as ruled but is still "to confirm"
  (end-state-api.md §6).
- **`value` means three things:** `Field.value(x)` (CONSTITUTION §2), the per-run priced cost,
  and `criterion_value`. The courses and `dp.py` call the priced run cost `J`. Consider
  `report.J` (per run) and `report.objective` (the scalar), keeping `report.cost` as the plain
  integral.
- **The demo's gain regularizer has no place in the program.** `pid_autotuning_jax.py`'s
  objective includes `1e-4‖pid‖²` (line 79), which is not a `g(x, u, t)`. The program export
  needs a documented way to add a term on `z`, or the demo's numbers change.
- **The native `Trajectory` is a core rewrite.** `(n, N)` layout, `float` copies and
  `MappingProxyType` (core/trajectory.py:80–95; 72 constructor calls), and `check_samples`
  refuses a leading run axis (:289–296). V1.1 is bigger than "one step" and is the critical path
  of V1.

---

#### Plan and schedule risk

- **Time and dependencies.** 8.5 weeks to 2026-12-21. Wave K needs per-item rulings (K1's table,
  K8-3 per plant, K8-4 per item, K4 per group). The continuous core is 15 gated steps, each
  byte-identical across every consumer, with five intended number changes that each need
  sign-off: R18, V1.2, R21's lesson lines, `failure_rate` renamed to `feasibility_rate`, and TO
  if M3 is taken. Wave G is two weeks in exam period, and course notebooks are the maintainer's
  own work. The bottleneck is the maintainer's attention, not agent hours.
- **G1 is under-weighted.** It has not started, and it was due in parallel from 2026-10-22. It is
  the one input that decides whether C6's collisions are real. Recommendation: three days this
  week, before wave K, with one output only: a table of "the GMC714 lesson needs X by Jan 5",
  for each topic.
- **Cut or defer from v0.3** (none of these is needed by GMC714):
  - SM-7 / R18, the forced-hold change: a number change with no consumer asking for it;
  - SM-6's move of the integrators to `simulation/solvers/`: pure relocation;
  - A2, gradients with respect to `Q` and `R`: research lane;
  - K4 and K8-3: they belong to the between-terms window anyway;
  - S31: make it v0.4 now; "if room" only invites churn under the MPC lesson.
- **Keep in v0.3:** SM-0 to SM-5, V1.1 to V1.5, K1, K8-1, K8-2 and K9.
- **Freeze the GMC714-facing API by Dec 1.** That covers trajopt, MPC, arms and vehicles, so
  wave G does not write lessons on a moving surface.
- **Pin the Colab cell (V6) before dev merges to main.** All 13 GRO860 course notebooks run
  `git clone https://github.com/alx87grd/minilink` with no tag. The calendar rule's
  "ships behind the pinned release" (ROADMAP §5) assumes a pin that does not exist yet.

#### What is right (keep)

- `discretize` returning the existing `StepSystem`: the GRO501 word, the integer tick, `params`
  always an argument, the frozen form a generated binding, with the measurement behind it.
- The cost as a pure integrand. The problem owns the horizon, the sets, the price and the
  discount. There is one `evaluate_trajectory` formula, with no scoring modes.
- Margins as the differentiable carriers and booleans as reported facts; the three positive
  rates.
- The `plant` reserved key and refusals that name the fix, rather than guessing from the wiring
  (with C5's exception closed).
- Exporting the program only, with no `tune` facade, and deferring the optimizer upgrade (R25).
  Continuous controllers first.
- The field inventory: the best evidence in the set. It should drive more of the rulings,
  including M3.
- The byte-identical refactor discipline and the step ordering SM-0 → SM-1 → V1.3.
