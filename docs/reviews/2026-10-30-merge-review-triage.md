# Before the v0.3 merge: what the four runs left for your review

**Audience:** Prof. Alexandre Girard (Maintainer), for one review sitting.
**Context:** four agent runs on `dev-v0.3` took about 180 decisions without you, as you
asked, each written in its log with the context and the alternative:

| Run | Log | Decision ids |
| --- | --- | --- |
| The overnight run, 2026-10-24 (the step model, the path to the cost) | [2026-10-24-v03-run-decisions.md](2026-10-24-v03-run-decisions.md) | OD-1 … OD-36, plus the open points of its §6 and §7 |
| The v0.3 core run, 2026-10-28 (the lanes and the two joins) | [2026-10-29-v03-run-log.md](2026-10-29-v03-run-log.md) | RD-1 … RD-69 |
| The night run, 2026-10-29 (MPPI, recording, the MPC's speed) | [2026-10-29-night-run-log.md](2026-10-29-night-run-log.md) | N-1 … N-22 |
| The consolidation, 2026-10-29 (your rulings R168–R179) | [2026-10-29-consolidation-run-log.md](2026-10-29-consolidation-run-log.md) | C-1 … C-43 |
| After the night run, 2026-10-29 and -30 (MPPI's fans and verbose line, `1df28c76`; the consolidation's two speed regressions fixed, `a648534c`) | [the consolidation log's §8](2026-10-29-consolidation-run-log.md#8-after-the-consolidation-the-speed-recheck-2026-10-30) | P-1 … P-3 (§2.3 items 27–28, §4) |

This sheet sorts them by what your answer changes, so the review row of TODO §1.10 ("The
merge") closes in one sitting. Many early decisions were overtaken by later rulings; §5 lists
them so you can skip them. The recommendations are the agent's (2026-10-30), checked against
the code at `9e98b74a`; nothing here rules anything. **Updated the same day** against the merged
head (`01a6289f`, which adds `a648534c`) and every ruling of record: the items a ruling already
decides (R53, R125, R142, R144, R148, R159, R160, R171–R174, R176–R178) left §1–§3 for §5 under
their original numbers, so the numbers below have gaps on purpose; the work after the night run
joined §2.3 and §4.

**In session, 2026-10-30:** the answers are recorded in [the rulings of the merge review](2026-10-30-merge-review-rulings.md) (R180 onward); each answered item below carries its ruling in brackets.

**How to answer.** Per line: **yes** (keep it as landed), **no** (take the log's alternative),
or **later** (it becomes a TODO row). §5 needs no answer.

| § | What | Lines | What you give |
| --- | --- | --- | --- |
| 1 | Numbers that moved on purpose | 3 | one yes for the block |
| 2 | Behaviour and names a user meets | 24 | one answer each |
| 3 | Student-facing material (yours by AGENTS) | 11 | one answer each |
| 4 | Internal choices | about 115 | one yes for the block |
| 5 | Already settled by a ruling | about 45 | nothing |

## 1. Numbers that moved on purpose

Each change ran the same problem and cost before and after (R80) and re-pinned the baselines it
moved in its own commit. The size is the relative difference from its log's ledger.

| # | What changed | Size | Where a student sees it | Log |
| --- | --- | --- | --- | --- |
| 1.1 | A sampled input to the fixed-step RK4 run is held over each period (it was interpolated); a callable input is read at the RK4 stage times | 4.4e-3 (sampled), 9.5e-5 (callable) | nowhere pinned | OD §3, SM-7 |
| 1.2 | `evaluate` runs the continuous loop to `tf` (the old Monte Carlo held the law over each period and cut at the first exit) | median 7.7 % for a stiff PD at `dt = 0.05`, 2.8 % for a weak one | the Monte Carlo scores of the RL and evaluation lessons | OD §3 |
| 1.14 | The Lyapunov certificate's `P` is symmetrized after the solve | 1.8e-16 | nowhere | N §3 |

**Recommendation:** yes to the three left. The eleven others follow a ruling (§5). **[Ruled R184: yes]**

## 2. Behaviour and names a user meets

### 2.1 Marked "to raise first" by the consolidation

1. **A planner's evidence on an undeclared plant** (C-30). **[Ruled R181]** The judge integrates a plant that
   declares no time constant at the simulator's 0.1 ms step, so a planner's `evaluate` no
   longer equals its own step face to the bit (7e-5); `substeps=1` gives the face's number back.
   *Question:* should a planner's evidence score on its own step model instead? *Recommendation:*
   keep the one rule (R174): the record already carries the face's own `J` (C-42).
2. **The continuous simulator's held-signal step** (C-23). **[Ruled R180: 2c]** Your rule removed "one step per
   sample" from the sampled loop; the continuous `Simulator` still uses it for a held source in
   a continuous diagram (a `WhiteNoise`, a sampled policy). Removing it there too would move the
   noise demos' numbers (`lqg_cartpole`, `lqg_monte_carlo`, the state-space notebooks) and make
   them about 100× slower. *Recommendation:* keep it in v0.3, named in DESIGN §5 as the one
   exception; give those demos an explicit `dt` in a later row if you want one rule everywhere.
4. **The dual-rate plan one slow period late** (C-33). **[Ruled R183: by hand]** The MPC's `z` port is now the plan
   solved at this tick, so the late form needs
   `(mpc % slow) >> (UnitDelay(dim=n_z).rename_ports(u="z") % slow) >> (PlanFollower(planner) % fast)`.
   *Recommendation:* a small `PlanDelay(planner)` block reads better; it is a new public name,
   so yours.
5. **Hand-wiring a continuous plant into a sampled diagram** (C-22). **[Ruled R186: no new method]** Only `@` and `closed_loop`
   put a continuous plant in a `SampledDiagramSystem`; `add_subsystem` on a `discretize`d model
   gives a loop of ticks only. *Recommendation:* no new method until a caller needs one.
6. The `experimental/` scripts (C-31): see 3.7.

### 2.2 The consolidation's other points to raise

7. **A model-based law copies the plant's numbers** (C-7). **[Ruled R182: copy]** `ComputedTorque`, `SlidingMode` and
   the impedance and kinematic laws hold `params["model"]`, a deep copy taken at construction;
   an edit of `plant.params` after the law is built no longer reaches the law. A GMC714 lab that
   tunes the plant after building its controller now sees a model mismatch.
   *Recommendation:* yes (the model and the plant are two things, and the mismatch is written
   `law.params["model"]["m"] = …`); say it in the lab text. *Ruled in substance by R169* (a
   law's model is its own leaves); what is left is the lab text.
8. **The RL planner's tree is a report** (C-9). **[Ruled R182: report]** `planner.params` shows the training numbers and
   the seed, but `solve(params=)` does not rebind them. *Recommendation:* yes for v0.3; a later
   RL row if a training run should take its numbers as an argument.
9. **Value iteration's bound** (C-11) **[Ruled R184]** is taken over its sweeps to convergence, not over the
   problem's finite horizon. *Recommendation:* yes (it is the sum value iteration computes);
   R173 (a tool's own discount and its sums are internal) already covers it.
### 2.3 New or changed public names

11. `disconnect(sys_id, port_id)` **[Ruled R185]**, a new verb; `connect` refuses to rewire a connected input
    (RD-60). *Recommendation:* yes.
12. `certificate.V` is an attribute **[Ruled R185]**, a `QuadraticField` (`V(x)` still works) (RD-57).
    *Recommendation:* yes.
14. `compute_trajectory(record=(…))` **[Ruled R184: hold]** records signals during the run; the last sample holds the
    last tick's value (N-1, N-11, N-12). The tool is your decision 6 of 2026-10-28 and its form
    your words ("an argument to pass to compute_trajectory"); what is left is the end-sample rule
    (N-11). *Recommendation:* yes.
15. `StepPlanningProblem` on the root prelude **[Ruled R185: bands]**, `StepCost` and `SampledCost` on the root and
    `minilink.core` (RD-42). The root is the course set (R64, K8-1) and no course teaches these
    yet. *Recommendation:* **no**: keep them on `minilink.planning` and `minilink.core` only.
16. `NoiseSource` on `minilink.blocks` **[Ruled R185]**, not on the root (RD-26). *Recommendation:* yes.
17. `Sys2Gym(x_start=law)`; **[Ruled R184]** with no start it starts at `sys.x0` (the old default, a uniform
    tenth of the state bounds, was a law nobody wrote and refused an unbounded plant). The
    GRO860 SB3 notebooks moved to `x_start=Uniform(…)` (RD-27). R177 (`x_start` a point or a
    law) decides the argument; what is left is the default at `sys.x0`. *Recommendation:* yes.
18. `transcription="single_shooting"` **[Ruled R183]** beside the two other strings (RD-50). *Recommendation:* yes.
19. The MPC refuses a planner whose problem has a point start **[Ruled R183: refuse]** beside an explicit `X0` set
    (N-4). *Recommendation:* yes.
20. `compare` keeps two forms **[Ruled R185]** by its first argument: solutions only, or
    `compare(problem, …, dt=)` that also scores each entry (OD-28). *Recommendation:* yes.
22. `pe.program(wrt, …, params=)` **[Ruled R182]** overrides the base point (OD-14); `problem.replace(**fields)`
    builds a new problem (OD-23). *Recommendation:* yes.

27. **MPPI's fan of sampled futures** **[Ruled R183]** (P-1, `1df28c76`): `pv.samples(params, z0, n_shown)` on
    MPPI's face redraws a solve's own samples on its key (the seed and the tick);
    `mpc_sampled_futures(run, mpc, loop=)` rebuilds every tick's fan after a run,
    `mpc_animation_overlays(..., samples=n)` draws it, `plot_sampled_futures(run, mpc, loop=,
    ticks=, axes=)` gives the static figure (all on `minilink.planning.trajectory_optimization.mpc`,
    the provisional band of R178). *Recommendation:* yes, provisional with MPPI.
28. **Each face reads its plan as a trajectory** **[Ruled R183]** (P-2): `pv.planned_trajectory(z, params)` on
    the collocation face (its knots) and on MPPI's (its plan rolled out on the model); the MPC's
    `verbose=True` line appends a planner's own account (MPPI: `S_min`, `S_weighted`,
    `effective_samples=…/K`, `exit_rate`). *Recommendation:* yes.

### 2.4 Small leftovers

23. `lqr_finite_horizon` does not hand out **[Ruled R185: later]** its `QuadraticField` (RD-58). *Recommendation:*
    later, with the fields' consumers (TODO §4).
24. A drawn mass runs 3.6× the plain batch **[Ruled R184: reported; the solve unchanged]** (each trial factors its mass matrix); the budget
    reports it and does not judge it (RD-62, line 5b). *Recommendation:* not judged in v0.3; it
    joins the batch-price row (S53).
25. The same cost added twice (`q + 10 * q`) **[Done: now refused with the line to write]** shares its numbers between the two terms
    (OD §6, point 5). Not re-checked on the current code. *Recommendation:* later.
26. `A >> B >> C` is named `A >> B` (OD §7). **[Done: named `Gain >> Gain >> Gain` on the current code]** Not re-checked. *Recommendation:* an agent fixes it
    if it still holds.

## 3. Student-facing material

These are yours by AGENTS ("Ask first": demos, notebooks, lessons).

1. **The cart-pole RL swing-up** (N-7, N-22; **[Ruled R186: the maintainer's]** the consolidation's §1 item 7). At 2M steps:
   9 of 12 seeds swung up before the runs, 4 of 12 on the integrated reward `−c_k` (3 of 8 at
   3M). R171 put the training reward back to `−g·dt`, the variant the night run measured at
   7 of 12. Nothing has been re-run since, and the demo now trains 3M steps.
   *Recommendation:* one 12-seed re-measure at 2M on a machine with JAX (about ten minutes)
   before you set the demo's budget back.
2. **About 90 constructions write `U=plant.input_box`** **[Ruled R186: (c)]** so their numbers stay (OD-18), the
   GRO860 LQR lessons among them; the LQR warning waits on them (OD-33). The options are in
   TODO §2, "V1.2, the rest".
3. **The three hand gradient-loop demos** (OD-32): **[Ruled R186: convert, one hand loop kept]** the options are in TODO §2, "S74, the rest".
4. **The closing PID example's loss** **[Ruled R186: as simple as possible]** is the problem's cost, tracking plus `ρ = 0.01` effort
   measured from the torque that holds the setpoint (OD-20). *Recommendation:* one read.
5. **The MPC notebooks' prose** (RD-63): **[Ruled R186: behaviour prose deleted]** `udes_gro501/racecar_toward_mpc` §7–§8, the showcase
   §8, `mpc_spatial_stack`. Since your call of 2026-10-29 they run `delay=False`, so the
   zero-delay story should hold again, but no one has re-read the prose against the new runs
   (1.11 moved the bicycle's guess). *Recommendation:* one read. The demos' retuning for the
   delay (RD-52) can close: every MPC demo runs `delay=False` except the two compile demos that
   show the delay on purpose.
6. **The showcase notebook's sampled loop** (1.12, C-26). **[Follows R180]** *Recommendation:* give
   `(ctl % 0.1) @ plant` an explicit `dt=` so it runs as before.
7. **Three scripts in the top-level `experimental/`** **[Done in session]** (`boat/parking_compare.py`,
   `boat/parking_mpc.py`, `mppi/racecar_mppi.py`) close loops on `discretize(...)`, so they now
   run ticks only (C-31). They are your scratch. *Recommendation:* move them to `@ plant` with
   `dt=`.
8. **The `x_goal` lines removed from the demos** (C-18), each where the cost's `xbar` or the
   given `Xf` is the same point; RRT's turned into `Xf=BallSet(goal, tol)`. *Recommendation:*
   a look at the diff.
9. **ROADMAP §4.1's RL row** **[Ruled R186: yes]** still names `StochasticPlanningProblem`, which is gone (§4 is
   yours). *Recommendation:* "`PlanningProblem` with laws".
11. **Three demos' style** (OD §7): **[Ruled R186: fix all]** in K4's row (TODO §2), for your demo pass.
12. **The pipeline figures** **[Ruled R186: the map only, with the System's origins]** (`docs/pipeline_figures.py`) still show the old labels; they are
    regenerated with the student-facing numbers at the merge. Nothing to decide.

## 4. Internal choices: accept as a block **[Ruled R187: accepted]**

Structure, placement and internal names behind the public contract. Each row in its log has the
alternative.

- **The overnight run:** the step formula written once in `core/compile` (OD-7, OD-11);
  `rollout` returning states, the signals after the scan (OD-8); `ev.jit` on NumPy (OD-10);
  the presets and the first-order backend (OD-15, OD-16, OD-17); the closing example's two
  sections on one cost (OD-20's mechanics); the evaluator's tree (OD-26); the adaptive solver
  and the `auto` fallback (OD-29); `TrajectorySource` tracing (OD-30); the mechanical bases
  inventing no bounds (OD-35, already in ROADMAP §2's breaking changes); OD-1 to OD-6, OD-9,
  OD-19.
- **The v0.3 core run:** the names sweep and `core/params.py` (RD-2 to RD-7); the sampled
  loop's internals (RD-1, RD-8 to RD-14, RD-18); the objects lane (RD-19 to RD-25, RD-28,
  RD-29); the program lane (RD-30 to RD-35); join 1 (RD-36 to RD-40, RD-43, RD-44, RD-46 to
  RD-48); the MPC's failure rule (RD-55); the fields (RD-59, RD-61); the read-back of the
  plant's input (RD-64); where `jit` sits and the NumPy loops' speed (RD-65, RD-67 to RD-69).
- **The night run:** the exact-hold refusal (N-6); the nested tick law refused (N-8); the
  read-only start (N-9); `box % dt` sharing the blocks (N-10); MPPI's deterministic scope, its
  tick key and its receding horizon (N-15 to N-17); the two Adams (N-18); the `_jac_*`
  protocol (N-20); `readme_examples` 6 % slower, explained (N-21).
- **The consolidation:** C-1 to C-6, C-8, C-10, C-12 to C-16, C-20, C-21, C-24 to C-29, C-32,
  C-34 to C-43.
- **After the night run** (the consolidation log's §8): the fans rebuilt from the run on the
  solve's own key instead of recorded in the loop (P-1); `SampledDiagramEvaluator.block_input`,
  a signal pass over a block's ancestors alone, and one `solve_inputs` per MPC block shared by
  its solve and the read-back (P-1); a table's stage cost built once per sweep
  (`stage_cost_law`) and an undiscounted rate `g` itself (P-3); the frozen value objects'
  `params` a `cached_property`, built once per object under R169 (P-3).

## 5. Already settled by a ruling: no answer needed

| Old decision | What settled it |
| --- | --- |
| 1.3 Value iteration's failure cost: `1e6` became the cost bound | R172: one cost bound, `problem.cost_bound(tf, dt=)`; the value-iteration notebooks' colour scale now tops at the bound |
| 1.4 The sampled loop's same-tick semantic | R125: the same-tick semantic |
| 1.5 One integer key draws the same trials on NumPy and JAX | R144: one tree drawn by named streams from one key |
| 1.6 Value iteration's stage cost the integral of `g` over the period | R142: the stage cost integrated in the step |
| 1.7 The judge's running cost integrated with the state | R142, R159: the judge's running cost is the stage cost |
| 1.8 A planner's score holds its feedback law over each period | R159: the run priced to the horizon with the input held |
| 1.9 RL: the exploration at the episode's seed; the training reward | R160 (the seeded heads), R171 (the training reward `−g·dt` beside the score); the re-measure is 3.1 |
| 1.10 The MPC's default the warm-started box | R176 and its amendment (one MPC box, `delay=False, warm_start=True` the default), your call of 2026-10-29 for the demos and notebooks |
| 1.11, 2.1 item 3 The default first guess aims at the cost's target | R177: a guess aims at `Xf`'s point, else the cost's target |
| 1.12 A sampled loop's undeclared plant at the automatic step | R174: the plant a loop closes on is integrated at the continuous simulator's step; the showcase's explicit `dt=` is 3.6 |
| 1.13 The judge on such a plant | R174 (the judge follows the same rule); the follow-up question is 2.1's item 1 |
| 2.2 item 10 RRT's `goal_tolerance` removed | R177 and its amendment, which names it (RRT's goal is an `Xf` region) |
| 2.3 item 13 `planner.solve(params=None, *, guess=None)` | R148 (the planner's pure face; `planner.solve` with no mutation) |
| 2.3 item 21 A cost term's id is its class name | R53 (a part's id is its class name, as a block's) |
| 3.10 ROADMAP §2's drafts: MPPI's provisional names | R178 (named provisional in ROADMAP §2); the breaking-changes entry is yours to paste |
| The engine first, in its own module (OD-12) | history: the order of a finished run |
| The recorded plant input `plant.u` (OD-13) | `plant:u`, R134 (RD-4) |
| The price switch `infeasible_cost` (OD-21), and the failure cost's value at a finite horizon (OD §6, §7) | `penalize_failure` and `failure_cost`, one `cost_bound` (R172, the consolidation's amendments) |
| A frozen set holding no parameters (OD-24) | every set has `params` and `with_params` (RD-19, R169) |
| `nominal_plant_params` keeping the part's number (OD-25) | the design point is the laws' means (RD-43, the no-key ruling) |
| The planners' reported trajectory (OD-27) | the evidence on the step face (R159, RD-41, C-42) |
| The program's leaves left unscaled (OD-31) | decided against; no row |
| The README snippet's `U` line (OD-34) | landed, then `x_goal` gone (R177) |
| `StepRollout` kept (OD-36) | gone with the legacy runtime at join 2 |
| The disturbance draws dropped by `evaluate` (OD §6, point 1) | disturbances are source blocks in the evaluated loop (R128, RD-28) |
| A diverged run scoring better than a failing one (OD §6, point 2) | a non-finite run is charged the cost bound (DESIGN §6) |
| The tabular learner on RK4 beside an Euler grid (OD §6, point 3) | its step defaults to the grid's scheme |
| The Gym bridge's extra step (OD §6, point 4) | the horizon test has a half-period tolerance |
| The price ignoring the cost's overrides (OD §6, point 6) | R165 |
| `closed_loop` and `feedback` on no band facade (OD §7) | on `minilink.control` |
| Which place writes the gradient loop out (OD §7) | RULES 6.15 (R75, amended by R91) |
| ROADMAP §4.2's `identification/` sentence (OD §7) | gone |
| `@` and `>>` precedence in the definition of done (RD-15) | your amendment 1: `>>` into a loop is legal, the docs warn that `@` binds first (C-29) |
| Which block is the loop's plant (RD-16) | the view's `kind` (C-22) |
| A bare tick law in a nested box (RD-17) | refused at compile (N-8) |
| `lookahead`, the delayed and pure MPC classes, the MPC's shape (RD-49, RD-51, RD-56) | one MPC architecture, `lookahead` gone (R176) |
| The MPC demos driving worse under the delay (RD-52) | they run `delay=False` (the zero-delay record, your call of 2026-10-29) |
| The `"method"` and `seed` leaves deferred (RD-54) | one tree per planner (N-3, R170) |
| The compile cache and its frozen form (RD-66, RD-68, N-2, N-13) | no cache for the convenience verbs (R168) |
| The MPC notebooks on the delayed MPC (RD-63, its MPC part) | they run `delay=False`; the prose read is 3.5 |
| The cart-pole's reward (N-7's choice) | the training reward is `−g·dt` (R171); the re-measure is 3.1 |
| MPPI's names (N-3, N-14) | R170, R178 |
| `alpha=` beside a discount rate (N-5) | a tool's own discount is internal (R173, C-14) |
| The fields' wrappers dropping their tree (N-19) | every field's number is a leaf (R169) |

## 6. One sitting, in order

**Done on 2026-10-30:** the sitting was held and ruled R180–R187
([the record](2026-10-30-merge-review-rulings.md)); [the plan](../plans/merge-review-run.md)
implements them, and [the run's log](2026-10-31-merge-review-run-log.md) records how. The
order below is kept as it was.

1. §1 as a block: three numbers (about 5 minutes).
2. §2.1 and §2.2, the consolidation's points: eight answers.
3. §2.3 and §2.4: sixteen answers, most of them yes (items 27–28 new).
4. §3: your material calls; 3.1's re-measure can run while you read the rest.
5. §4 as a block.

Then the merge row of TODO §1.10: `dev-v0.3` into `dev` (R79), the student-facing numbers
re-pinned, the v0.3 tag; `main` when you choose (R78).
