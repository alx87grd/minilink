# Independent review C: the plan and the roadmap of the v0.3 core

**Date:** 2026-10-26 · **Reviewer:** Reviewer C · **Emphasis:** the plan and the roadmap (the
phase order and its dependencies, the scope split between releases, the risks, feasibility for one
maintainer with agents, the open decisions, the smallest usable v0.3) · **Status:** an
independent review, not a ruling · **Commit reviewed:** `ce082e4474f42f8433bd62e994ea8a7e977f43a4`
(`dev-v0.3`, level with `origin/dev-v0.3`)

Read: AGENTS, CONSTITUTION, core-architecture.md (whole), ROADMAP §1 and §5, TODO §1–§2,
QUESTIONS "Blocks v0.3", DESIGN §5–§6, the core-consolidation record, and the plan docs it cites;
code claims spot-checked under `minilink/`.

## Top summary

1. **The vision holds; the plan's shape is the risk.** v0.3 grew from "the continuous core"
   (landed 2026-10-24) to two foundations and seven pillars, with no date, no definition of done,
   and three rows whose only gate is "done when: written with the ruling".
2. **The landed work merges last.** The continuous core waits on the end of the whole plan to reach
   `dev`. Review it now, merge it, then merge each pillar into `dev` as it lands.
3. **The order is a dependency graph drawn as a line, and two of its edges contradict the plan
   docs.** The one-cost evaluation plan says it comes *with* the sampled loop, not before it; the
   step-rollout fold placed in the second phase waits on the sampled loop in the fifth. The
   step-algebra phase has no row that delivers its own vision (a step rollout that traces).
4. **Three lanes, two joins** fit the code better: objects and draws; the step and sampled loop;
   the program and the native solver. The sampled loop, the least converged and the largest, is
   the critical path and should start first.
5. **The maintainer's decisions are the bottleneck, not the agents' code.** Twenty-one "Blocks
   v0.3" entries (seven with no recommendation), sixteen overnight decisions unreviewed, 123
   rulings in five days. Re-bucket, decide in the order of §5, and prototype the three draft
   pillars before ruling them.
6. **"Re-pin once, at the end of phase 5"** conflicts with the per-step byte-identical gate and CI's
   regression job: re-pin regression baselines per intended change; batch only the student-facing
   material.
7. **Shrink v0.3 without breaking the direction:** ship the smallest usable core as v0.3.0, the
   remaining pillars as v0.3.x, all before any v0.4 feature.
8. **Two fixes should not wait for any phase:** refuse the stateless MPC in `problem.compile`, and
   the cross-backend draw defect.

## 1. Strengths worth keeping

- **The spine and "One signature"** (CONSTITUTION §1.1, §2). Every object a pure method on
  `(x, u, t; p)` that traces in `params` is what makes `grad`, `vmap` and parameter families
  uniform; the landed evaluator already proves it (DESIGN §6: `jax.grad` of
  `pe.evaluate(p).objective` reaches gains, plant parameters, cost weights and the start).
- **Honest "Today in the code" sections.** I checked several and they hold: `Set.params = None`
  (`core/sets.py:38`, `:100`), the MPC tick latch (`mpc/controller.py:567`), the refusal of a
  sampled controller (`planning/evaluator.py:789`), the step rollout returning host NumPy
  (`core/compile/evaluators/discrete.py:57`).
- **The number-change discipline** (closeness on the same problem, `rtol 1e-6` for a restructure,
  no alias: R80–R82) and the seeded baseline recipe.
- **Class material stays on `main` until the core is ready.** This removes the biggest risk of the
  earlier plans: number changes no longer collide with a running course.
- **The sampled-loop analysis** (sampled-path.md §8.2–§8.3). The MPC's one-period computational
  delay (R123) is the right call: honest to hardware, and pure.
- **The core/feature test** (core-architecture §10): a v0.4 feature reads a pillar, never changes it.

## 2. Problems

### 2.1 v0.3 has no definition of done and no size

**Saw.** ROADMAP §1: "When the core is complete (no date; the order is the commitment)". The rows
for the one-cost evaluation, the two runners and the stochastic program end "Done when: written
with the ruling" (TODO §1.3, §1.5, §1.6). The only sizing in the docs is the retired Gantt
(consolidation record §5): the sampled loop, the draws, the parametric program and the fields were
about 17 weeks *before* the judge, the two runners, the step algebra, the seeded policy and the
native solver joined. At TODO's own grain (one agent-hour to one agent-day per step), I count
100–120 steps in TODO §1, plus the 44 notebooks the merge must move (R47).
**Why it matters.** "Complete" can always grow; a branch without a gate does not merge.
**Suggest.** Six acceptance scripts as v0.3's definition of done (§3.3); size each pillar in steps
before it starts. **Cost:** an afternoon. **Touches:** ROADMAP §1, §5.3; TODO §1.9.

### 2.2 The landed continuous core merges last

**Saw.** TODO §1: `dev-v0.3` merges into `dev` after the maintainer's review (R79), and §1.9 puts
that merge after every pillar. `origin/dev` last moved 2026-10-23; the overnight run sits on
`dev-v0.3`; the sixteen open overnight decisions are QUESTIONS' merge blocker. A branch
`origin/v0.3.0` exists at `70f93f3` (the commit before the consolidation) and no doc names it.
**Why.** AGENTS opens with work lost on stale branches; months of pillars on an unreviewed base
multiply that risk.
**Suggest.** Review the continuous core and the sixteen decisions now, merge into `dev`, then merge
each pillar as it lands (R79 applied per pillar). The rule that nothing reaches `main` before the
notebooks move (R47) is untouched. Say what `v0.3.0` is. **Cost:** one review session. **Risk:**
low. **Touches:** R79; QUESTIONS "the 16 remaining decisions".

### 2.3 The phase order contradicts its own plan docs

**Saw.**
- The one-cost evaluation (phase 3) comes before the hybrid loop (phase 5), but
  differentiable-problem.md §9.4 point 6 orders it "after the `StepRollout` fold and S31's kernel
  ... together with step 7b, since the sampled face is one of the loops this kernel scores". Its
  selling point (core-architecture §3) needs the sampled loop.
- The step-rollout fold (phase 2) waits on the sampled loop by its own row (TODO §1.2, OD-36).
- The step-algebra vision (core-architecture §2.2: step rollouts that trace end to end, a batch
  rollout for step systems) has **no row**. Phase 2's rows are the fold (really phase 5), a
  profiling task (S53) and a fifteen-item grab-bag (S65). The real prerequisite of the pure
  computer tick, the fixed-iteration solver and the closed loop in one scan is unscheduled.
- The draws (phase 4) after the judge, "one change site for the realization": the draws touch
  `problem.realize`, the distributions and the RL environment, and do not need the judge.
- The parametric program, the optimizer table and the native kernel depend on nothing in phases
  2–5 (parametric-program.md §4: "No dependency on S75 or S31").
**Why.** A line serializes parallel work and puts the least converged design on the critical path
before the code that would inform it. **Suggest.** The lanes of §3.1, and a new row: "step
evaluators trace and batch". **Touches:** core-architecture §4, ROADMAP §5, TODO §1.2–§1.3.

### 2.4 Rows that are one piece of work

TODO §1.5's two-runner row and the sampled-loop row (S31) describe one kernel with two faces; the
caches (D13) and the mainstream loop (AC-8b) sit inside them. TODO §1.4's noise seeds (V1 step 7),
the per-block draws (RN-4, RN-5) and the cost's draws (AC-7) are "the same row" by
randomness.md §9. **Suggest:** one row per deliverable, gates merged. **Cost:** low.

### 2.5 "Re-pin once" against the gates

**Saw.** core-architecture §4 and TODO §1.9: demos, notebooks and baselines re-pinned once at the
end of phase 5. AGENTS' recipe and RULES 7.7 require byte-identical steps unless an intended change
carries its own test; CI's regression job reads `benchmarks/baselines/` (`f_mpc_parity.json`,
`dp_speed.json`, …).
**Why.** Either CI is red from phase 3 to 5, or the gates are switched off while five intended
changes pile up and R80's before/after comparison is lost.
**Suggest.** Each intended change re-pins the regression baselines in its own commit with its
closeness report; only the student-facing re-pin (notebook outputs, course numbers) is batched, at
the merge to `main`. Sequence so each number moves once: the same-tick semantic, then the MPC
delay, then the hybrid defaults; the per-step cost before any RL re-pin. **Touches:** R80–R82.

### 2.6 The decision load is the critical path

**Saw.** "Blocks v0.3" has 21 entries, seven "Recommendation: none". Several block nothing in the
core: the flatten default ("Blocks: nothing until benchmarks…"), the hand-loop demos, the LQR
warning, three demos' style, `DynamicBicycle`'s base. The plan docs are about 4,800 lines and
ROADMAP, TODO and QUESTIONS another 2,100, for a 71k-line library.
**Why.** Agents landed the continuous core overnight; decisions arrive at human speed.
**Suggest.** Split "Blocks v0.3" into *gates code on the critical path* (about eight), *gates the
merge*, and *clean-up lane*, each with a recommendation (§5). For the three draft pillars, *spike
then rule*: an agent prototypes under `experimental/` and reports numbers, as native-optimizers.md
§3.1's hand-wired gradient loop did. Pause doc restructuring until the first merge.

### 2.7 Stale rung text an agent will execute against

differentiable-problem.md §8 and §9.4, native-optimizers.md §2 and §4.6, parametric-program.md §4,
randomness.md §9 and future-policies.md §5.1 still write these steps as "v0.4". The banners say the
new rung wins, but an agent executing a step reads the body. **Suggest:** one agent-managed sweep.

### 2.8 A silent wrong-semantics path, open today

The stateless MPC is a plain `System` (`mpc/controller.py:418`) whose latch keys on `(k, y)`; inside
`problem.compile` its measurement changes at every RK4 stage, so it re-solves the NLP up to four
times per step: a continuous-feedback MPC the user did not build. QUESTIONS asks whether to refuse
it "until the sampled face lands". Refuse it now (one line beside `require_continuous`). Fix the
cross-backend draw defect (KNOWN_ISSUES) first in lane A.

## 3. Opportunities and improvements

### 3.1 Proposed order: three lanes, two joins

| Lane | Steps, in order | Waits on |
| --- | --- | --- |
| **A. Objects and draws** | the textbook leftovers (T1, T2) → distributions read `params` (RN-2) → sets read `params`, the shaping to `core/`, `Set.as_cost` (A5, V1 step 8) → the diagram's derived start state (S29, moved in) → per-block seeds and disturbance signals, one draw on both backends (RN-4, the rest of RN-5, AC-7, the Gym start law) → `NoiseSource` with its consumer (RN-3) → the seeded policy block | the draws' rulings only |
| **B. Step and sampled loop** | step evaluators trace and batch (new) → the same-tick semantic → the pure computer tick, `Computer.tick` as its oracle → the MPC delay and the caches gone → the step-diagram runner for `computer @ plant` → the sampled face with a digital PID (V1 step 7b) → the step-rollout fold → the orchestrator over pure blocks | nothing |
| **C. Program and solver** | one optimizer backend table (OPW-1–OPW-5) → the program with parameters, collocation reading the plant, the online call (PP-1, PP-2, PP-4) → the native kernel, byte-identical (NS-1) → the cost and set leaves (PP-3, after lane A) → the fixed-`K` traced form (NS-2) → the fixed-iteration MPC tick (NS-8) | nothing |
| **Join 1** | one per-step cost and one failure rule, read by the evaluator, then value iteration and RL | A, B |
| **Join 2** | the MPC's closed loop in one scan, tuned by gradient (NS-9); the stochastic program as a stated convention and a test | A, B, C |
| **Last** | the `Field` noun and `QuadraticField`; wiring failures and the error family (S66 with S70); the notebooks; the merge | all |

Lanes A and C are mostly ruled, overnight agent work; lane B, the big unknown, starts on day one.

### 3.2 Cut, defer or merge

- **Fields:** keep the noun, the one `as_cost` and `QuadraticField` (fields.md 4.1–4.3, they fix a
  public signature and the Lyapunov `V`); move `GridField`, the approximator and `CallableField`
  (4.4–4.6) to v0.4. core-architecture §4 itself calls Fields "additive to the path".
- **Stochastic program:** in v0.3, a ruling and a test (the seed a leaf of `p`; the sample average,
  which `pe.program(key=, n_trials=)` already is; `vmap` over seeds after NS-2). A solver advancing
  the seed needs the stochastic category (NS-7), already v0.4.
- **Seeded policy:** the block in v0.3; the RL heads' rebuild, which moves every RL number, with the
  RL follow-ups in v0.4.
- **Profiling (S53)** and the S65 items no pillar reads (verbose rules, print text, Monte Carlo
  defaults) back to standing work; keep the automatic `dt`, the input hold, `n_steps` and the
  one-tick lag in lane B.
- **Adaptive orchestrator** to v0.4; **realtime re-base** with the realtime review (v1.0). Each cut
  also removes decisions from the critical path.

### 3.3 The smallest usable v0.3

v0.3.0 = the landed continuous core + lane A + lane B through the sampled face + lane C through the
online call + the wiring-time failures. Its definition of done, as scripts:

1. An LQG loop with noise on `d` and `n`: the Monte Carlo mean within the covariance prediction's
   sampling error, one key giving the same draws on NumPy and JAX.
2. A digital PID (`pid % 0.01 @ plant`): `problem.compile(loop).evaluate(p)` runs and `jax.grad`
   matches central differences.
3. One compiled MPC program rebound with two plant `mu` and a moved target, matching rebuilds;
   `optimizer_method="ipopt"` on the compiled path.
4. The same digital PID run by `compute_trajectory` and scored by `evaluate`: one `J` to `rtol`.
5. Every catalog plant and notebook green on the merge gate.

Then v0.3.1 the one-cost evaluation, v0.3.2 the native proof of concept, v0.3.3 Fields, all before
any v0.4 feature, so the direction holds. With one tag instead, keep the order and the per-pillar
merges: the risk drops with the merges, not the tag.

### 3.4 Keep the discrete problem internal at first

CONSTITUTION §2 lists eight nouns; a public discrete problem would be the ninth, with a name, a
teaching-surface call and docs. "Lower late" suggests a lowering held by the problem evaluator,
like `ExecutionPlan`:

```python
pe = problem.compile(controller, dt=0.01)                  # unchanged
c = per_step_cost(problem, dt=0.01, solver="rk4")          # (x_k, u_k, k, p) -> c_k, internal
# value iteration and the RL environment call the same function
```

Publish it when a native discrete consumer (a grid world) needs the noun.

## 4. Naming suggestions

**Public names.**
- `GaussianPolicy(mean, log_std, *, dt, seed=None)` → `sample_period=`, as
  `WhiteNoise(p, *, psd, sample_period, seed, hold)`: it is a hold period, and `dt` already means
  the evaluation step of `problem.compile(controller, dt)`. One name per concept with the RL
  `StochasticPolicy` and `GaussianHead` once the heads move.
- `bind`: the planning facade already exports the geometry `bind(sys, spec, *, frame)`
  (`planning/spatial/collision.py:98`), so a student meets two meanings beside
  `ParametricProgram.bind(p)`. The geometry name is provisional; rename it (`attach`) and keep
  `bind` for parameters.
- If `computer @ plant` becomes a step diagram: `SampledDataSystem`, the textbook's term
  (Åström–Wittenmark, Franklin–Powell), rather than `HybridDiagram`; the docs already say "the
  sampled loop".
- If the discrete problem goes public: `StepPlanningProblem` from `problem.discretize(dt, *,
  solver, substeps)`, matching `StepSystem`, `StepRollout`, `StepSchedule` and the existing
  `discretize(plant, dt, *, solver, substeps)`.
- "One discrete judge": "judge" is neither a textbook nor a library word; "one per-step cost" or
  "one discrete evaluation" (the noun is `Evaluation`).

**Ids and labels.**
- **Phase numbers:** the order is a graph; name lanes and pillars instead. It also ends the "phase
  4" / P4 near-collision the consolidation already had to fix.
- **`D13`, `D25`, `D27`, `D29`** are plan-local decision ids with a bare workboard-style prefix while
  wave D's rows `D1`–`D3` exist: plans/README forbids exactly that. Retire each when ruled and cite
  QUESTIONS headings meanwhile, or prefix them.
- **The path to the cost (V1)** carries steps 0–9, 7a/7b and "V1.1, the rest", "V1.2, the rest":
  three notations for one row. Close it; its open steps become rows of their pillars.
- **Twenty-three prefixes** appear across ROADMAP, TODO, QUESTIONS and core-architecture. A row
  should lead with its plain title, the id an anchor; a plan-local prefix retires with its plan.

## 5. Recommended answers to "Blocks v0.3" (opinion, not a ruling), in the order to decide

1. **The sixteen overnight decisions:** accept as a block after the three number changes the log
   flags. Unblocks the merge into `dev`.
2. **Borderline rows:** the derived start state (S29) into lane A, since a controller's or an
   observer's start not being a leaf is exactly "every number a parameter"; the error family (S70)
   with the wiring-time failures that raise it; the realtime re-base v1.0; the adaptive orchestrator
   v0.4; the RL heads v0.4.
3. **Refuse the stateless MPC:** yes, now (§2.8).
4. **Same-tick semantic:** propagation within the tick, like a continuous diagram's feedthrough and
   a textbook discrete block diagram; latency only where a block states it (`UnitDelay`, the MPC's
   delay state, a held slow output). Remove the computer's implicit one-tick lag. Numbers move once,
   at lane B's start.
5. **`computer @ plant` a step diagram:** direction yes; amend invariant 1 when the pure tick
   passes the `Computer.tick` oracle.
6. **`compute_trajectory` on a hybrid loop:** the step-diagram runner, substeps recorded (§8.4
   option B): today's numbers and density, and `compute_trajectory` agrees with `evaluate`.
7. **Run caches:** they go.
8. **Parametric family:** one family; hand-written programs may declare `p`; live ports
   `live=[...]`; `bind` on the program only, the evaluator taking `p` as an argument (which answers
   §7.5 point 1: pass inside, bind at the user boundary).
9. **Parameters and randomness** (one session with 8): the seed an integer leaf found by the `seed`
   key, as on blocks; complete trees at the API (R121), partial merging in one helper; gradients by
   `wrt` paths; `N`, `K` and batch sizes structural; the warm start on the solve call;
   sensitivities v0.4.
10. **Native kernel shape:** a pure `init` / `step` kernel with a step-diagram teaching view.
11. **Which fixed-iteration tick first:** the digital PID proves the sampled face first; then the
    gradient MPC as directed; MPPI stays v0.4. Fix native-optimizers.md §4.6b/§5's wording.
12. **Operators on step systems:** none in v0.3; extend the existing `feedback()` entry point if a
    lesson needs a one-liner (CONSTITUTION §6 forbids a second dialect, not a wider entry point).
13. **The one-cost evaluation:** yes, after lane B; per-step cost by integrating the cost with the
    state (exact for the scheme); margins at samples in v0.3; `t_k` bound; `γ = e^{−ρ dt}`;
    internal first (§3.4).
14. **Failure rule, hold, scope:** one rule, `score_run`'s (the RL environment may still end an
    episode at its training zone, a training choice per DESIGN §6); zero-order hold; shooting's
    score lowered to it; fixed-`dt` planner policies through the sampled face; DP's finite horizon
    v0.4.
15. **Seeded policy:** the recommended `GaussianPolicy` with `sample_period=`; heads v0.4.
16. **Draw convention:** one, the sample index: `⌊t/Δ⌋` in continuous time, `k` on the tick with
    `Δ = d·dt_base`, stated once in DESIGN §4.
17. **`as_cost` and shaping:** the shaping to `core/` as a pure move now; `Set.as_cost` and
    `Field.as_cost` call one function with one signature.
18. **Failure price after cost overrides:** follow the overrides, with the cache; otherwise a tuned
    weight can price a failed run below a feasible one, breaking DESIGN §6's ordering.
19. **Move out of "Blocks v0.3":** the flatten default, the hand-loop demos (keep, R90), the LQR
    warning (my lean: no warning, said in the docstring), demo style (the maintainer's demo pass),
    `DynamicBicycle` (rename only; its base with the catalog clean-up).
