# Minilink workboard

Every open step to v1.0, by release, in order. Each row carries a **bold plain title**, then its
id in parentheses. The id is an anchor, not a vocabulary: prose leads with the title. Each step
is sized at roughly one agent-hour to one agent-day and ends with a "done when" gate.
**[ask]** marks maintainer-owned territory (AGENTS "Ask first"); unmarked rows are
agent-managed. Landed rows leave this file; their history is git and
[docs/reviews/](../reviews/). The pieces live elsewhere:
- **Why each v0.3 row exists:** [core-architecture.md](core-architecture.md) §N, the same N as
  §1.N here.
- **Its open design questions:** [QUESTIONS.md](../dev/QUESTIONS.md); the choices
  are [R124–R140](../reviews/2026-10-27-design-questions-rulings.md) and
  [R141–R153](../reviews/2026-10-27-core-vision-decisions.md), the latter taking the decisions of
  [the consolidated core vision](../reviews/2026-10-27-consolidated-core-vision.md).
- **Milestones:** [ROADMAP](../dev/ROADMAP.md#5-the-path-to-v10).
- **Pyro parity:** [pyro-port-remaining.md](pyro-port-remaining.md).
- `scan: band#n` cites the 2026-09-22 improvement scan, which is in git history before
  2026-10-17.

| Section | Release |
| --- | --- |
| [§1](#1-v03--the-complete-core) | v0.3 — the complete core, one subsection per workstream |
| [§2](#2-v03--the-clean-up-lane-beside-the-core) | v0.3 — the clean-up lane, beside the core |
| [§3](#3-standing-work--textbook-code-minimal-demos-consolidation-the-plotting-lane) | Standing work |
| [§4](#4-v04--features-on-the-core) | v0.4 — features on the core |
| [§5](#5-v09-freeze-candidate-and-v10) | v0.9 freeze candidate and v1.0 |
| [§6](#6-later-ideas) | Later ideas |

## Rows waiting for the maintainer's go

Workboard rows marked **[ask]**: each waits for the maintainer's go, pick or review before an
agent starts. They are details, versions and demos, not design questions (those are
[QUESTIONS.md](../dev/QUESTIONS.md)); each row holds its options.

**The core's rows have the go** (2026-10-28, R166): every row of §1 is cleared to start under its
"done when"; the internal choices an agent meets (the default output stub's type under JAX, the
open-loop face as the evaluator or a sibling, the hold slots' labels) are decided as the
maintainer would and logged in the run's record.

| Step, in plain words | Id | Release | What the ask is |
| --- | --- | --- | --- |
| Your review of the overnight run's open decisions | OD-7 … OD-35 | v0.3 (the merge, §1.10) | review the number changes first, then the rest as a block |
| Where the course material lives; the examples tree | K5 | v0.9 | repository-structure.md's options |
| The demo sweep to the flatness rule | D1.3 | standing (wave D) | your review of each diff; `showcase_jax` waits |
| The planning demos on the solution verbs | D1.4 | standing (wave D) | `solution.plot_*` and `compare(...)` in place of hand-rolled comparisons |
| `Source.show_signal` | D2 | standing (wave D) | remove it (the same picture is `plot_trajectory`) |
| Two dead modules | D2 | standing (wave D) | remove `planning/spatial/overlays.py` (with A3) and `experimental/symbolic/mechanics/utils.py` |
| Your demo pass, and a tabular Q-learning demo | K4 | v0.3 (the clean-up lane) | your pass; the demo beside value iteration (R65) |
| LQG as a planner | LQGPlanner | v0.4 | `LQGPlanner(problem)` beside `LQRPlanner` |
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
| The LQR warning on a declared `U` | V1.2 rest | v0.3 (the clean-up lane) | warn, or "U ignored" in the record (amends R21) |
| Three hand gradient-loop demos | S74 rest | v0.3 (the clean-up lane) | convert all, two, or none |
| `DynamicBicycle`'s base | S32 | v0.3 (the clean-up lane) | rename `M` → `H` now, the base with K8 |
| The compile-time flatten default | — | standing | after one benchmark: off, on, or always |
| Which pyro demos the courses need | C1 | v0.4 | your pick |
| Reward scaling by default | — | later | `reward_scale="auto"` the default or opt-in |

---

## 1. v0.3 — the complete core

**How the work flows.**
- **Order:** three lanes and two joins, a proposal ([core-architecture.md](core-architecture.md)
  §10).
- **Where the work lands:** `dev-v0.3`, a temporary branch, merged into `dev` after the
  maintainer's review (R79).
- **Reaching `main`:** nothing merges to `main` until every notebook has moved (R47), and when
  it does is the maintainer's call (R78). The course material stays on `main` until then.
- **What landed overnight on 2026-10-24** (pending review; every decision taken without the
  maintainer is in [the run's log](../reviews/2026-10-24-v03-run-decisions.md)):
  - the clean-up's renames and moves (K2, K3, K10, A6, G3's moves);
  - the step model (SM-0 to SM-8);
  - the path to the cost (V1.1 to V1.5, with A2);
  - the optimizer's method table (S74);
  - the closing PID example (S77);
  - R68: every plant states its own bounds;
  - K3b: every public name has a test.

**The gates** (R80–R82):
- **A restructure** agrees to `rtol 1e-6`, and a pure move or rename stays identical.
- **An intended number change** runs the same problem and cost before and after, with the
  relative differences in the step's report.
- **A rename or keyword change** lands with all its callers, with no alias.
- **A step that restructures a computation** records a seeded baseline first (AGENTS' recipe,
  the JSONs and scripts under `benchmarks/`), and its math is read next to
  `planning/dynamic_programming/dp.py`.
- **Re-pinning:** an intended change re-pins the regression baselines it moves
  (`benchmarks/baselines/`) in its own commit, with its closeness report, so CI stays green;
  the student-facing numbers (notebook outputs, course figures) are re-pinned together at the
  merge to `main`.

**The path to the cost (V1)** is closed as a row. Its open steps are rows below:
- step 7 is in §1.3;
- step 7b is in §1.5;
- step 8 is in §1.1;
- step 9 is in §1.10;
- what V1.1 left is in §1.2.

### 1.1 Every object pure, every number a parameter

Why: [core-architecture.md](core-architecture.md) §1.

- [ ] **Names across the API, before the new signatures** (R134, R151,
  choices of 2026-10-27), one sweep per line, every caller in the same change, no
  alias (R82):
  - one keyword per concept, written in DESIGN §4: `compile_backend=` becomes `backend=`, the
    `Optimizer.backend` attribute becomes `adapter`, a planner's `optimizer_method=` becomes
    `method=`;
  - the geometry `bind(sys, geometry, frame=)` becomes `mount(geometry, on=sys, frame=)`;
  - one separator rule: `.` walks a tree, `:` names a signal (`plant.u` in records becomes
    `plant:u`);
  - the RL package's internal `StochasticPolicy` renamed, so `GaussianPolicy` owns the idea;
  - `NeuralPolicyController` becomes `NeuralPolicy` (R151; thirteen files, the RL notebooks
    swept once with R143's cells) **[student-facing, given]**;
  - the names the day's rulings left to this sweep: `sys.discretize(dt)` as the facade of
    `discretize` (R141), the dense-run evaluator method (R147), and the planner and MPC names of
    [planner-and-mpc.md](planner-and-mpc.md) §11, all ruled on 2026-10-28 (R164): `update` (the
    seconds state face), `rollout_dense` (the dense evaluator method), `pv.command`, the letter
    `z`, `PlanFollower`, `inputs=`, `delay=`, `lookahead=`, `t_start` for the run's origin on
    every runner (today's `t0`), `program.unpack(z)` (was `params_at`); and `dt` at the calls,
    `sample_period` on the blocks (R157).

  Done when DESIGN §4 states the rule and no public signature uses `compile_backend`,
  `optimizer_method` or the geometry `bind`. Later, not in this row: the compiled-model classes →
  `Compiled…` (the v0.9 name pass, V3).
- [ ] **Later nouns, each with its first consumer** (A5):
  - `log_prob` on the distributions;
  - parameter-dictionary support on sets and distributions (one flatten rule, with
    identification or the robust problem);
  - `Set.as_cost(weight=, shaping=)` as the set-to-cost dual of `Field.as_cost()`: its first
    consumer is the program bridge's `"soft"` lowering (the next row), after `shaping.py`
    moves to `core/`, and MPPI reads it next;
  - `UnionSet` via `|`, when reachability or multi-goal work needs it;
  - vector bounds on `Saturation`;
  - the library shapes and fields reading `params` (the same rule as A2; the sets read theirs);
  - `PlanningProblem.hamiltonian()`, only if the course teaches Pontryagin;
  - the RL critic as a `Field` once training is over;
  - the double-integrator homework's two verbs, entry time into a set and the Bellman residual
    (scan: examples#10).

  `log_prob` lives on the seeded policy, not on `Distribution` (R129).
- [ ] **Sets as costs, the lowerings, the metrics as constraints** (was V1
  step 8; [differentiable-problem.md](differentiable-problem.md) §8):
  - the robust-tuning and identification demos run on the export;
  - the metric objective and the margin-floor example wait for S73, a v0.4 row.

- [ ] **One parameter-tree module** (new, from reviewer A §2.6; internal, agent-managed): the
  path lookup, merge and flatten of parameter trees, written today in `planning/evaluator.py`
  (`lookup`, `with_leaf`, `merge_tree`, `flat_leaves`), `planning/problems.py` (`lookup_param`,
  `merge_params`), `core/compile/evaluators/jacobian.py` (`flatten_params`, `replace_leaf`)
  and `core/assembly.py`, become one `core/params.py` used by every caller. Done when the four
  copies are gone and a seeded baseline is byte-identical.
- [ ] **Blocks and control leftovers** (T2; its first two items and T1 landed in the v0.3 run):
  - one time-interpolation for trajectories and gain schedules;
  - one warm-start helper on the plan `Trajectory`;
  - the impedance and robotic law bodies consolidated (scan: control#2, #5, #7).

  The row's old note that `TrajectorySource.h` cannot trace is stale: it traces since OD-30.

### 1.2 The step algebra first-class

Why: [core-architecture.md](core-architecture.md) §2.

- [ ] **The step algebra's rest, with the MPC's rebuild** (landed on `dev-v0.3-loop` 2026-10-28:
  the same-tick semantic in the sampled loop, the operators on step systems, the step evaluators
  that trace and batch, `compute_rollout` returning a `Trajectory`, one signal pass per tick;
  [the run's log](../reviews/2026-10-29-v03-run-log.md) §7): `Computer.tick`'s one-tick lag and
  `StepRollout`, the legacy hybrid computer's record, leave with `Computer` when the MPC is
  rebuilt on the sampled loop (§1.8, join 2; the run's log RD-8). Done when no `Computer` or
  `StepRollout` remains.
- [ ] **The batch price of a parameter family** (S53): profile `rollout_batch` with a `params`
  family, which takes 278 ms against 27 ms for the plain batch (pendulum, 1000 × 1000 RK4
  steps). Done when the family path is within 2× of the plain batch. Also: gate the quoted
  batch-rollout claim, and batch the static-leaf time grid (scan: compile-sim#11, #12).
- [ ] **One owner per rule** (S65; scan: compile-sim#0, #1, #2, #3, #6, #7, #8, #10, #13,
  planning#0, #5, #8, #11, core#8, #10). One item per step:
  - **Read by the loop lane:**
    - the automatic `dt` as `auto_dt(sys)` in `time_grid.py`, with the discontinuous scale made
      finer or DESIGN's promise dropped (the duplicated constant goes with it);
    - the forced-input hold as one `input_interp` option every solver obeys (its sampled side
      is S75; today the automatic solver choice changes the hold model);
    - what `n_steps` counts on each verb;
    - the one-tick lag inside a `Computer` removed (R125).
  - **The rest, which no pillar reads and which reviewer C §3.2 would return to standing
    work:**
    - the try-JAX-then-NumPy policy in `compile_auto`, with a typed `NotTraceableError`;
    - `simulation` stops importing `optimization` for display constants;
    - `scipy_stiff` with explicit tolerances and a Jacobian;
    - `compile(verbose=True)` in order;
    - one set of Monte Carlo defaults for `MonteCarloEvaluator` and `Planner.evaluate`;
    - flat-kwarg keys derived from the option dataclasses;
    - one owner of the sets between `StateSpaceGrid` and its planner;
    - one print-text rule for the textbook objects;
    - one verbose and empty-cache rule across the `compute_*` facades.

### 1.3 The draws

Why: [core-architecture.md](core-architecture.md) §3. Design and the per-step files and "done
when": [randomness.md](randomness.md) §9.

- [ ] **Per-block seeds and disturbance signals, one draw on both backends** (merges V1 step 7,
  RN-4, the rest of RN-5 and the rest of P10's AC-7). Landed in the v0.3 run: `problem.realize`
  one tree with a stream per consumer by name and a seed per random source; disturbances are
  source blocks only, wired into the evaluated loop; one integer key draws the same trials on
  NumPy and JAX (the KNOWN_ISSUES entry gone); the shortcut is the loop `closed_loop` builds
  with the same sources, one Monte Carlo score for both forms (AC-7's done-when); RN-5's LQG
  gate is acceptance script 1. Rest: RN-6, the contract in DESIGN, with each step.
- [ ] **A problem stochastic by its parts** (R143): landed in the v0.3 run (no
  `StochasticPlanningProblem`; `x_start=` a law or a set, `params=` laws, `disturbances=` source
  blocks, `criterion=` and `nominal()` on `PlanningProblem`), every Python caller swept; the
  ten notebooks move at step 7 (the run log's §4). Rest: the RL baselines re-pin with the
  seeded policy (§1.4).

### 1.4 The seeded stochastic policy

Why: [core-architecture.md](core-architecture.md) §4. Design:
[future-policies.md](future-policies.md) §5 point 4 and §5.1 (D29).

- [ ] **The standard stochastic policy as a seeded block** (the maintainer's
  direction, 2026-10-22; its shape a choice of 2026-10-27, R129,
  [record](../reviews/2026-10-27-design-questions-rulings.md)):
  - one block, `GaussianPolicy(mean, log_std, *, sample_period, seed=None)`:
    `u = μ_θ(x) + σ_θ(x) · ε(t)`, `ε(t)` the noise block's counter-based signal at `⌊t/Δ⌋`,
    deterministic given the seed;
  - the mean at `seed=None`, and a seed per experiment from `realize(key)`;
  - the log-probability and the entropy as methods beside `h`;
  - the RL Gaussian and squashed-Gaussian heads rebuilt on it;
  - the collectors moved from their own keys to per-episode seeds. The RL numbers change once,
    with a baseline and one re-pin.

  The block landed in the v0.3 run (`control/stochastic.py`: `GaussianPolicy` and
  `SquashedGaussianPolicy`, `log_prob` and `entropy`, `vmap` over seeds on both backends); the
  heads' rebuild, the per-episode seeds of the collectors and the RL re-pin wait for join 1
  (the run log's RD-29: the core plan's change 4 follows change 2, the one score). Its
  deterministic face is `NeuralPolicy` (R151). The heads are rebuilt in v0.3 with the block
  (R160), with one gate: the training performance is unchanged after the move (the learning
  curve and the final score within the Monte Carlo spread on the same demo and seeds, the training
  time within the budget's 2× bar). Done when the RL demos re-pin and a policy block vmaps over
  seeds on both backends.

### 1.5 The sampled loop: a step diagram, two runners

Why: [core-architecture.md](core-architecture.md) §5. Design: [sampled-path.md](sampled-path.md).
Ruled: R13, R14, R104, R105, R123, R125, R145, R146, R147 (choices of 2026-10-27,
[record](../reviews/2026-10-27-core-vision-decisions.md)); R158, time in the core and
`SampledSystem` the sampled-data kind (2026-10-28, [record](../reviews/2026-10-28-session-rulings.md)).

- [ ] **The sampled loop's rest** (landed on `dev-v0.3-loop` 2026-10-28: `SampledSystem`, `%`
  on any system, `SampledDiagramSystem` and its plan, the two run verbs and `rollout_dense`, the
  host clock runner, the sampled face of `problem.compile`, the stateless MPC refused, the hybrid
  demos re-pinned once; [the run's log](../reviews/2026-10-29-v03-run-log.md) §7):
  - `Computer`, `StepSchedule`, `HybridDiagram`, `hybrid_closed_loop`, `HybridSimulator`'s
    computer path, `x0_plant=` and `x0_computer=`, `HybridDiagram.last_result` (D13) and the
    `core` → `simulation` lazy import they carry: they are the MPC's runtime and go with its
    rebuild (§1.8, join 2);
  - the dual-rate MPC projects rewritten as composition, with the MPC (join 2);
  - tutorial 06 on the sampled loop, with the notebook sweep (§1.10, R167).

  Done when every hybrid notebook and project runs as a `SampledDiagramSystem`.

The conventions' first sampled-loop step (AC-8a: standard port names only, the MPC gives `u`,
no port guessing) is folded into this section's rows (R163): it is what the rebuilt loop does by
construction.

### 1.6 One score

Why: [core-architecture.md](core-architecture.md) §6. Design:
[differentiable-problem.md](differentiable-problem.md) §9. Ruled: R141, R142, R150, R154, and the
three points (R159, 2026-10-28): the sampled check accepted, the run priced to the horizon with
the input held, a solver's own objective allowed but the score always the problem's cost.

- [ ] **`StepPlanningProblem` and `StepCost`** (R141): a problem on a
  `StepSystem` with a horizon `N`, a per-step `gamma` and a `StepCost` (`c(x, u, k)`, `h(x, k)`);
  `SampledCost(g, h, plant, dt, *, solver=, substeps=)` the `StepCost` of a discretized problem,
  its `c` integrating one period when called directly with the plant's solver and substeps
  (R154); sharing the sets, the start, the tree and `realize` with `PlanningProblem` by composition. Done
  when a grid world solves by value iteration as a `StepPlanningProblem`.
- [ ] **The compiled step face** (R141, R142): `problem.compile(dt=, solver=,
  substeps=)` with no controller gives `transition(x, u, k, params) -> (x_next, c_k)`, `N`,
  `gamma = e^{−ρ dt}`, the sets at `t_k` and the price; `c_k` integrated with the state in the
  stationary form; `discretize(problem, dt)` (facade `problem.discretize`) returns the explicit
  `StepPlanningProblem` compiling to the same face. Done when the augmented step's state rows are
  byte-identical to `step_body`, its `c_k` equals `g·dt` under Euler to the last bit, and the
  discretized problem's face equals `problem.compile(dt=)`'s.
- [ ] **The judge, value iteration and the RL environment on `transition`**
  (after the open points): `problem.compile(controller)` scans `transition`; value iteration's
  backup and the RL environment's step read it; the environment's step and `score_run` share one
  failure, price and horizon rule (R108's one owner); the judge's `J` re-pinned once (R80). Done
  when one policy gets one `J` from the judge, value iteration and the RL environment.
- [ ] **The RL discount derived** (R150): the RL planner's `γ` read from the problem's step face
  by default, `planner.gamma` readable; `gamma=` a training hyperparameter under
  `planner.params["method"]`, the record saying when it differs from the scored discount; the
  notebooks and demos swept with R143's cells **[student-facing, given]**. Done when the training
  return and the judge use the same `γ` unless an override is stated.

### 1.7 The parametric program; the stochastic program its seed case

Why: [core-architecture.md](core-architecture.md) §7. Design:
[parametric-program.md](parametric-program.md), [native-optimizers.md](native-optimizers.md) §7.

- [ ] **The parametric program binds every problem parameter** (S76, steps
  PP-0–PP-5; PP-0, the names, in V1's step 0).
  - **Today:** the compiled MPC program binds the measured `x0` only: direct collocation freezes
    the plant, cost and set parameters in closures; the online call refuses every other key; the
    MPC block's port functions drop `params`. This row absorbs the D3 row on the parametric
    evaluator's placement and the scene binding of the Later list.
  - **The shape** (R130): one noun, `MathematicalProgram(..., params=None)`; `J(z, params=None)`,
    `h(z, params=None)`, `g(z, params=None)` on the problem's tree (`x_start`, `t_start`, plant,
    cost, sets; the scene with A3), the frozen default when `None`; the values passed at the solve
    (`Optimizer(program).solve(params=...)`); hand-written programs stay `J(z)`; one evaluator
    body serves both, and the optimizer sees a flat `z`; `ParametricMathematicalProgram` folds in.
  - **The failure price under cost overrides** (R165): the price follows the overridden weights,
    with a cache keyed on the cost's parameters (today it keeps the problem's own weights, 4096
    calls of `g` per evaluation when no exact bound exists).
  - **The performance gate, first:** benchmark compile time and per-solve time of the compiled
    MPC program carrying `params` against the same program with them frozen; if carrying them
    costs speed, an export option compiles a parameter-free program.
  - **First users:** direct collocation and the rebuilt MPC, whose inputs bind leaves of the tree
    (R148, `inputs={"plant.mu": "mu_hat"}`); the export gains `program.unpack(z)`, the full
    tuned tree.
  - **Order:** PP-1, PP-2 and PP-4 on S74's backend factory (OPW-1); then PP-3, the cost and set
    leaves, on A2.
  - **Landed in the v0.3 run:** PP-1, the noun with its optional `params`, `Optimizer.solve(params=,
    z0=)` with the one checked merge (`core.params.merge_known`), one evaluator body;
    `ParametricMathematicalProgram` and `JaxParametricProgramEvaluator` folded in.

  Done when one compiled MPC program, bound with two values of a plant parameter and of a cost
  target, gives what a rebuild gives; the MPC baselines are byte-identical with only `x_start`
  bound; the friction-aware MPC example runs from a port.
- [ ] **`t_start` and the start from `x_start`** (R148): a problem's horizon is
  `[t_start, t_start + tf]` (`t_start = 0` reproduces today); the discount and the horizon count
  from `t_start`, costs, sets and references read absolute time; on a native step problem
  `t_start` is the tick offset, and a discretized problem keeps `(dt, t_start)` in seconds (R158);
  every planner pins the start from the `x_start` leaf and `X0` is derived from it
  (today shooting and collocation read `X0` first). Done when a time-varying reference is tracked
  from `t_start = 100` with the same cost as from 0, and every baseline is byte-identical at
  `t_start = 0`.
- [ ] **The stochastic program as the parametric program's seed case**
  (native-optimizers.md §7; R131): every program is deterministic given its parameters, the seed
  one of them; integers never in `wrt`; complete trees inside, one merge at the boundary; sizes
  structural; the warm start on the solve call; the core keeps the sample-average export,
  `pe.program(wrt, key=, n_trials=)`; a Monte Carlo over solves comes free once a solve traces;
  how a solver uses the seed is an `Optimizer` strategy (§4). Done when the convention is stated
  in DESIGN §6 and a test solves the sample-average program of a stochastic problem (robust gain
  tuning) on both backends.

### 1.8 The native proof of concept: the planner's pure face and an MPC on it

Why: [core-architecture.md](core-architecture.md) §8. Design:
[native-optimizers.md](native-optimizers.md), [planner-and-mpc.md](planner-and-mpc.md). Better
solvers and MPPI are v0.4 (§4).

- [ ] **The native kernel and the fixed-count solve** (NS-1, NS-2's fixed-`K`
  form; R132, R149): pure `init` / `step` laws for `gd` and `adam` on the array module, run by
  `Optimizer`, with `as_step_system`; `Optimizer(program, method=, iterations=K)` exactly `K`
  steps in one `lax.scan`, `iterations=None` to a tolerance (`maxiter` its cap). Done when the
  NumPy path is byte-identical and `jax.grad` through `K` iterations matches a central difference.
- [ ] **The planner's compiled face** (R148): `planner.compile()` gives `n_z`,
  `initial`, `solve(params, z0) -> (z, status)`, `shift(z, t)`, `command(z, x, t)`,
  `prediction`, `solution`, `pack`, `program`; `planner.params` is the problem's tree plus
  `"method"` and `seed`; `planner.solve(params=None, *, guess=None, ...)` rebuilt on it, with no
  mutation (`last_solution` and the caches leave the pure path; the plotting shortcuts move to
  the solution, with D1.4); trajectory optimization first, then LQR and value iteration, the
  others as they are wrapped. Done when `pv.solution(pv.solve(p)[0])` reproduces today's
  `planner.solve()` byte-identically and `jax.vmap` over starts runs on the native method.
- [ ] **The MPC rebuilt on the planner's face** (NS-8; R148, R123):
  `ModelPredictiveController(planner, *, inputs={"x_start": "y", "t_start": "t"}, delay=True)`;
  the delayed form a block whose state is `z` alone, its origin inside (R155; its kind under the
  R158: a native `SampledSystem`, `update` solving and the `u` port applying), `x0` from
  `pv.initial`, shifting and solving once per tick and keeping the old plan on a failed solve; the
  pure form a static block; inputs as bindings (`{"x_start": "y"}` by default), `t_start` bound
  to the time `update` receives, `lookahead=` for delay compensation; ports `u`, `z`, `x_ref`,
  `u_ref`;
  `PlanFollower` for dual rate; `dt_mpc`, `t0`, `warm_start`, `MPCTickLatch`, the stateless and
  stateful pair, `export_to_computer`, `dt_broadcast`, `Command` and `replan_divisor` gone; the
  MPC demos and the racecar demos swept **[student-facing, given]**, `racecar_toward_mpc.ipynb`
  with the notebook sweep (R167); the MPC
  numbers re-pinned once with R123's delay. Done when one tick jits on the native planner and
  matches the host SLSQP plan within a tolerance on the pendulum and the cart-pole, and the Ipopt
  MPC runs on the host runner.
- [ ] **Its closed loop in one scan, tuned by gradient** (NS-9): `problem.compile(mpc % Δ)`
  `.evaluate(p)` runs, `vmap`s over starts, and `jax.grad` with respect to `controller.cost.Q`
  tunes a demo by `Optimizer`. The gradient MPC comes first, as the direction places it; MPPI is
  v0.4.

### 1.9 Fields

Why: [core-architecture.md](core-architecture.md) §9. Design: [fields.md](fields.md). Baselines
first.

- [ ] **A minimal field object, `core/fields.py`** (A1, steps 4.1–4.3 of
  fields.md; R133, R153):
  - landed in the v0.3 run (step 2): `Field` in `core/fields.py` with `gradient` and the four
    conversions, `as_set` / `as_input_set` renamed with every caller, `core/shaping.py`, and
    `Set.as_cost` reusing `Field.as_cost` (the log's RD-20, RD-21);
  - `QuadraticField` (the Lyapunov `V`, the LQR value, the `S(t)` schedule), with the ellipsoid
    as its sublevel set;
  - the DESIGN §4 fields bullet.

  Done when the RoA baseline `cmp`s, `test_analysis_lyapunov.py` and the corridor and RRT demos
  pass, and an LQR value serves as an MPC terminal cost through
  `QuadraticField.as_terminal_cost`.

The rest of the field plan (steps 4.4–4.7: `GridField`, `CallableField`, `LinearApproximator`
and `LyapunovCertificate.V` as fields, `PolicyEvaluator`'s field-shaped result) is a row of §4,
after the core (R153). Step 4.7, the CBF plan's, is done.

### 1.10 Hardening and the merge

- [ ] **Wiring mistakes fail at wiring time** (S66; RULES 4.10; scan: core#2,
  #13):
  - `connect` refuses to rewire a connected subsystem input;
  - the compile probe detects an output that moves with an input it does not declare and
    raises, naming the block, the port and the fix (R116); the library's under-declared blocks
    are fixed first;
  - after P10's AC-4 (landed 2026-10-14), the loop-input junctions still rewire the connected
    input they sum into (`composition.sum_into`), so S66 changes them first.

  The refusals use plain `ValueError` and `TypeError` with clear messages (R165); the typed
  error family (S70) stays where it is planned, §5.
- [ ] **Material and contracts** (was V1 step 9;
  [differentiable-problem.md](differentiable-problem.md) §8):
  - each step updates DESIGN when it lands (§4 `Trajectory` and parameters, §5 the rollout, §6
    the evaluation and the export), and ROADMAP §3's ledger; DESIGN §6's gradient one-liner reads
    `jax.grad(f, allow_int=True)` (R152);
  - the showcase cells are rewritten on the engine;
  - K4 merges the compile demos, one per feature;
  - the three hand-gradient-loop demos are the maintainer's call (OD-32; §2, "S74, the rest").
- [ ] **The merge**:
  - every notebook moved to the new interface (R47);
  - the student-facing numbers re-pinned (the regression baselines move with each change, §1's
    gates);
  - DESIGN brought up to each landed step;
  - the maintainer's review of the overnight run's open decisions
    ([the run's log](../reviews/2026-10-24-v03-run-decisions.md) §1–§3: OD-7, OD-8, OD-12,
    OD-14, OD-17, OD-18, OD-20, OD-21, OD-23, OD-24, OD-25, OD-26, OD-27, OD-29, OD-30, OD-35;
    the log says to look first at the number changes, the input-box sweep OD-18 and the
    planners' reported trajectory OD-27, then accept the rest as a block);
  - `dev-v0.3` into `dev` after that review (R79);
  - the v0.3 tag.

---

## 2. v0.3 — the clean-up lane, beside the core

Not on the core's path; it runs beside it. The GMC714 date of G3 is lifted (the maintainer,
2026-10-26: the course material stays on `main` until the new core is ready).

**Wave K — convergence** (planned for November 2026, before the continuous core, which has since landed; K7's
analysis is the ruling table K1 asks for). The goal: a
smaller library that is easier to read and keep — fewer names, fewer demos, prototypes
labelled as such. It extends D2 (whose open picks join K1's table) and runs before D1.3, so
the demo sweep touches fewer files. Nothing leaves without the maintainer's ruling (RULES 7.1).

- [ ] **One demo per feature** (K4) **[maintainer]** (R54): the maintainer's own pass on the demos,
  with the K1 inventory's §3 (twins and proposals,
  [the record](../reviews/2026-10-23-k1-inventory.md)) as the input; agents update `examples/README.md`'s
  map and the demo checks after each of his changes. Also, an orphan ruling (R65): one demo of
  tabular Q-learning on the pendulum grid beside value iteration, converging toward the DP
  cost-to-go (GRO860's RL row) **[ask — student-facing]**. Also for this pass, three demos' style
  (the run's log §7): `pid_autotuning_jax.py` keeps two helpers, prints that carry arithmetic
  and a hand-made last figure; `diagram_shortcuts.py` labels `NoisyPendulum()` as `Pendulum()`
  (a wrong label, fixable now); `custom_one_link_arm.py` reads module constants, not `params`.
- [ ] **The catalog clean-up** (K8) (ruled 2026-10-22, R2;
  [record](../reviews/2026-10-22-rulings-session-1.md)): two import layers and support by
  test. Steps, one commit each: **K8-0** the car family to three models (R55: `KinematicBicycle`,
  `DynamicBicycle`, `UdeSRacecarDyn` on the root; `KinematicCar` and `DynamicBicycleCar3D` merged as
  skins, `UdeSRacecar` a parameter preset, `UdeSRacecarDyn3D` a `wheel_angles=True` option, the
  notebooks and demos migrated); **K8-1** the root prelude trimmed to the course set (the
  plants of R55–R60, 16 by R60, no cap; all 50 catalog plants today; the other names per R64; ledger line in ROADMAP §2);
  **K8-2** `minilink.catalog` the one flat index of every supported plant, each passing the
  both-backends contract test, with a docs entry; **K8-3** the unsupported plants the
  maintainer names from K1's table ([the record](../reviews/2026-10-23-k1-inventory.md)) moved to `minilink/experimental/catalog/` with their skins
  and demos (the top-level `experimental/`), their tests and docs entries removed, out of the
  wheel **[ask — per plant]**; **K8-4** twins merged (R55, R59) and the dead helpers deleted
  (R54, R67: the Mermaid exporters, `camera.py`, `ProblemEnv` / `to_gymnasium`, the
  `identification/` stub; the pygame renderer stays, the real-time default); **K8-6** the README's catalog line by domain and the API
  site's plant page. Done when the root prelude lists exactly R60's plants, `minilink.catalog`
  lists only tested plants, no two supported plants differ by parameters alone, and
  `test_teaching_surface.py` pins both lists.
- [ ] **The experimental tier, untested** (K9) (ruled 2026-10-22, R3; [record](../reviews/2026-10-22-rulings-session-1.md)). Landed 2026-10-23: the prototype
  scripts moved from `examples/experimental/` to a top-level `experimental/` (K9-1); the tier's
  tests (`test_symbolic.py`, `test_engine_jax.py`) moved to `tests/experimental/`, off the merge
  gate, run by the nightly job (`pytest tests/experimental`), `c_export`'s flagship smoke kept in
  the JAX regression job (K9-2); the status line on every `minilink/experimental/` module
  docstring (K9-3). Ruled 2026-10-23: `manipulator_eom.ipynb` deleted (it imported the symbolic
  tier); `showcase_jax.ipynb` keeps its quick C-export demo, the one documented exception beside the
  flagship smoke (both on `minilink.experimental.c_export`). Left: the unsupported plants' test rows
  leave with K8-3. Done when CI's gate imports nothing under the tier but those two exceptions,
  and `pip wheel` ships nothing under it.
- [ ] **V1.2, the rest** (the evaluation of a run, the problem's fields, the price switch and the cost
  bound landed 2026-10-24): LQR warns once when `U` is declared (the U sweep wrote
  `U=plant.input_box` on lessons that never needed it, so the warning waits for the maintainer's
  pass on those lines; the options: drop the line from the LQR lessons then warn, keep it and warn,
  or never warn and say "U ignored" in the Riccati record's `str`, which amends R21, the
  reviewers' choice) **[ask — student-facing]**; `ReinforcementLearningPlanner` sizes its default `NeuralPolicy`
  from `problem.U.bounding_box()` and refuses an unbounded `problem.U` (R21), and
  `StateSpaceGrid.state_bounds` aligns with `input_bounds`.
- [ ] **S74, the rest** (the method table and the first-order backend landed 2026-10-24): the hand
  descent loops of `examples/demos/compile/` other than the closing example's first section
  (`cartpole_rollout_gradients`, `neural_controller_jax`, `params_gradient`) moved onto
  `Optimizer` **[ask — student-facing]**: each teaches the hand loop today, and R90 kept that
  loop in two places on purpose; the open-loop schedule needs a source whose samples are
  parameters first. The options: convert all three; convert two and the schedule demo later
  (reviewers A and B); keep all three (reviewer C, R90).

**GMC714** (R48, 2026-10-23): no new feature; the teaching surface covers every lab (ROADMAP
§4.3). The labs move from pyro to minilink as the maintainer's teaching work, outside this
workboard; the former wave G rows are §4 rows.

- [ ] **GMC714's names stable by January** (G3) **[ask — public names]** (R49; the January date
  lifted 2026-10-26: pinned when v0.3 merges; the moves and renames landed 2026-10-24, the gate
  is left): every tool a lab
  uses has one public teaching name on the teaching surface, frozen for the term (week 1 on
  January 9, as planned then), after A6's renames (R52: `ComputedTorque`, `SlidingMode`, `Impedance`,
  `ThreeLinkManipulator`, `LongitudinalTorqueCar`). Moves: `RRTPlanner` from provisional to the teaching lane; `JointImpedance` (R63: the class,
  was `ModelJointImpedance`; the factory deleted), `TaskKinematic`, `TaskKinematicNullspace` onto the
  root prelude; a new band facade,
  `minilink.dynamics` (R50, R61): `MechanicalSystem`, `Manipulator`, `GeneralizedMechanicalSystem`,
  `StateSpaceSystem`, `LTISystem`, with `JaxMechanicalSystem` deleted (`MechanicalSystem`'s default
  builders on `xp`); MPC as one teaching name, `ModelPredictiveController` (R50), over today's
  `MPCStatelessController` / `MPCStatefulController`, its constructor frozen until the rebuild on the planner's
  compiled face (R148, TODO §1.8) changes it once, and the loop it runs in (`mpc @ plant`,
  as every demo and lesson writes it) kept until that rebuild, which makes it `(mpc % Δ) @ plant`
  (R146, R148). `ModelPredictiveController`
  already exists as a factory over the two classes, in a research-lane module the teaching-surface
  test rejects: it moves to a teaching-lane module. A6's catalog renames land before the term
  (R50), so G3's list is pinned after them. Each name gets a docstring, a demo and the teaching-surface test
  (ROADMAP §2's soft rule). Done when `test_teaching_surface.py` pins the list, each name has a
  demo under `examples/demos/`, and no GMC714 lab needs a deep import.

- [ ] **One mechanical family** (S32) **[ask — core]**, ruled 2026-10-23 (R62), v0.3: `GeneralizedMechanicalSystem` a child of `MechanicalSystem` beside `Manipulator`, `H` the
  one mass-matrix name (`M` renamed in the class, the boat and the plane), `qv2x` / `x2qv` aligned, the
  model-based controllers accepting every mechanical plant; `Boat2D` (and `Plane3D` in the
  experimental tier) with `y` and `x` like every mechanical plant (already so today);
  `DynamicBicycle` on the generalized child (scan: dynamics#5); also `DynamicBicycle.M` (read by the racecar and
  `BicycleDynRate`) and `examples/projects/car_trajopt`'s ladder, and `DynamicBicycle`'s base
  moves from `DynamicSystem` to the generalized child. Done when the boat and the plane pass the
  catalog contract test as children of `MechanicalSystem`, `ComputedTorque` runs on one of them in a
  test, and no class names the mass matrix `M`.
  Postponed by the maintainer (2026-10-24). Ruled 2026-10-25: computed torque on a body-frame
  plant is `v̇_des = Kp N(q)⁻¹ (q_d − q) + Kd (v_d − v)`, a non-square `N` refused (R101); a
  model-based law refuses an underactuated plant at construction (R102), so the done-when runs
  `ComputedTorque` on a fully actuated body-frame test body, not on the boat or the plane.
  Still open: `DynamicBicycle`'s base. Its named ports and its subclasses' extra states do not
  fit `[q; v]`; the recommendation (and the reviewers') is to rename `M` to `H` only now and
  move the base when K8 reworks the cars.

---

## 3. Standing work — textbook code, minimal demos, consolidation, the plotting lane

Work that runs across the releases, behaviour-preserving where it touches code. Its core rows
(A5, RN-2, T1, T2, S53, S65, S66) moved to §1.

- [ ] **Naming, the rest** (A4) (any time, agent; the naming quick wins landed 2026-10-17:
  the class name as the default `name`, shortcuts named as the expression they build, `id`
  documented): one colour keyword, `jmax`, across DP, tabular RL, `PolicyEvaluator` and
  `Comparison`, with S54's clipping (scan: planning#3, examples#5); `plot_cost2go` keeps its
  name to v1.0 and its rename to `plot_cost_to_go` (the name the solution and `Comparison`
  already use) is on V4's list; the operators' named forms exported together (scan: core#9).

### Wave B — GRO501, the carry-over (0.2.0 shipped 2026-10-17)

Release contract:
[ROADMAP §4.2](../dev/ROADMAP.md#42-v02--gro501-end-to-end).

**Order:** P10's remaining steps (AC-5, AC-6 in the plotting lane; AC-7 and AC-8 in the v0.3
core, §1.3 and §1.5),
then the P4 follow-ups. P11 and P6 were dropped (2026-10-15, 2026-10-17); TB-b landed with the
textbook pass.

- [ ] **Automation by convention** (P10) ([automation-by-convention.md](automation-by-convention.md),
  decided 2026-10-11, amended 2026-10-13 by
  [the second-opinion record](../reviews/2026-10-13-automation-by-convention-second-opinion.md)):
  standard port names drive the wiring, three reserved names (`plant`, `controller`,
  `observer`) are the roles every tool reads, and the cost is scored on the plant. Its
  steps, in build order, are the plan's §7, each with its "done when":
  - AC-0 no controller classes, AC-2a the resolver, AC-OP the operators, AC-3
    composition on standard names and AC-2b/c the one-time rename and the ports `d` / `n`
    (all landed 2026-10-13; the MPC's `u_ff` → `u` trails with AC-8a, plan §10). Wave B
    of P10 is done.
  - AC-4 the loop inputs and `observer=` (landed 2026-10-14; rulings R18, R19); AC-5
    plots and AC-6 animation, in the plotting lane; AC-7 the cost on the plant, the v0.3 core
    (§1.3); AC-8a the sampled loop keeps in step, alongside; AC-8b with S31 (§1.5; it was v0.4
    by R86).

  Related rows: S66 lands after
  AC-4; RN-4 and RN-5 build on AC-2, AC-4 and AC-7; S70 turns P10's refusals into
  `WiringError`; S31 brings AC-8b. The automatic `dt` and input-hold findings stay with S65.
  Also, later, not v0.2: `sensitivity(T)` reading the roles of one loop diagram instead of
  `plant=, controller=` ([the 2026-10-15 analysis record](../reviews/2026-10-15-linear-analysis-rulings.md),
  last note).
- [ ] **follow-ups** (P4) (design: [estimation-and-state-feedback.md](estimation-and-state-feedback.md)):
  named input ports on the state-space base so observers and sensitivity blocks share one
  `f` (scan: analysis#13); the review of the `observer` role against the real blocks when
  the EKF lands ([automation-by-convention.md](automation-by-convention.md) §10).

### Wave D — textbook code, minimal demos, consolidation

**Order:** the S69 follow-up now; the T rows' leftovers as picked; D1.1, D1.2, then D1.3
file by file; D2, D3, S67 and S68 as picked (S65 and S66 moved to §1).

**T — the textbook pass.** Landed: the textbook layout of TB-b and T1–T7 in the second-run
pass (PR #125, 2026-10-16), behaviour- and
backend-preserving; its style sheet lives in RULES §5 (5.3's six blocks, 5.8, 5.10, 5.22,
5.25–5.27) and §7.8. The pass's recorded bugs and open questions were ruled the next day and
fixed one commit each with a regression test (PR #126, 2026-10-17,
[rulings](../reviews/2026-10-17-textbook-pass-follow-up-rulings.md)): the plants' `tf` and
geometry read their `params`, the MPC blocks take plain hook names and one mixin
constructor, the wiring core keeps no shadow state, every helper name says its role. The
rows below hold what neither PR touched, each a behaviour, backend or API item verified open
on `dev` at 638215e; the recipe and the reference are AGENTS'. (T1 and T2 moved to §1.1.)

- [ ] **dynamics catalog** (T3): `manipulators/arms.py` (`from_manipulator` sets `_kin_arm`,
  `name`, `camera_scale` and `camera_target` on the instance from outside `__init__`;
  `link_lengths` probes with `hasattr`), `mass_spring_damper/linear.py` (`A` / `B` / `C` / `D`
  on `np`, so they do not trace — a JAX-safety fix; extend the both-backends test),
  `aerial/plane.py` (the 55-line pygame `__main__`, a demo in disguise, RULES 5.18).
  Also: `Manipulator` kinematics defaults (`forward_kinematic_effector`, `J`) return zeros
  instead of raising, with `link_lengths` the base contract (scan: dynamics#7).
- [ ] **analysis** (T4): `analysis/lyapunov.py`'s `EVEN` / `ODD` slices sit under Internal
  machinery, apart from `METHODS` at the top.
- [ ] **planning** (T5): `trajectory_optimization/planner.py` probes
  `getattr(transcription, "supports_parametric", False)`; `direct_collocation.py` /
  `shooting.py` / `multiple_shooting.py` read `self.options.*` inside their bodies;
  `search/extenders.py` asks `getattr(U, "box", None)` instead of `U.bounding_box()` (also a
  D3 row); `spatial/paths.py`'s `PolylinePath` sets `_seg_lengths`, `_s_knots` and
  `_total_length` through `object.__setattr__` without declaring them as fields.
  `ReinforcementLearningPlanner(verbose=True)` keeps printing by default (ruled). (The planner's
  method table went with OPW-1, in the v0.3 run.)
- [ ] **the MPC block's contract** (T6): answered by R148 ([planner-and-mpc.md](planner-and-mpc.md));
  the work is TODO §1.8's "The MPC rebuilt on the planner's face", which removes the `hasattr`
  probing (`validate_mpc_planner`, `export_mpc_dual_rate_computer`'s `replan_divisor` and
  `set_after_solve` checks), `Command` and `MPCTickSolve`. The rebuilt MPC's ports are `u`, `z`, `x_ref` and
  `u_ref` (R148). This row
  closes when that §1.8 row lands.

**D1 — demos to the rule** (RULES 6.1 / 6.10 / 6.11; the census is comment FD-C14 of the
foundations review, docs/reviews/2026-09-15-foundations-review.md).

- [ ] **Native reporting the demos hand-roll** (D1.1) (agent lane, plotting): an `Optimizer`
  convergence plot (`optim_plot.py`); two planners' trees or solutions side by side
  (`rrt_holonomic_obstacles.py`); a `rollout_batch` family plot (`rollout_param_family.py`);
  a 3-D phase plot (`lorenz_attractor.py`). Each one `plot_*` or `__str__` with a test;
  DESIGN §7 lists them.
  Also: `print` on `StepRollout` and `HybridSimResult`, one validator shared with
  `Trajectory` (scan: core#7); `print(controller)` shows the gains (scan: control#10);
  several trajectories on one phase plane and two fields on one grid surface (scan:
  examples#6). Also, planned with the evaluation record (2026-10-23) and not built: the
  `Evaluation` record's `plot_cost` / `plot_trajectories`, `Comparison.plot_cost`, and the
  per-trial `keys` for a replay.
- [ ] **The flatness ratchet** (D1.2) in `test_teaching_imports.py`: no top-level `def` or
  utility class in `examples/demos/`, no notebook cell mixing an API call with matplotlib
  code; today's offenders allowlisted, the list shrinking with D1.3.
  Also: teaching code stops rebinding `sys` (scan: examples#7); one owner for the
  teaching-facade list the two teaching tests share (scan: tests-ci#7).
- [ ] **The sweep** (D1.3), one file per step, maintainer reviews each diff **[ask per file]**:
  the tutorial chapters (`11_reinforcement_learning.ipynb` and `showcase_jax.ipynb` first;
  `showcase_jax` rewritten, awaiting review), then the non-clean demos, then the teaching
  notebooks; the three `compile/` demos wait for V1 (their helpers *are* that feature). Done when
  the D1.2 allowlist is empty and the notebook checks pass.
  Also: the placeholder sections of tutorials 01, 02, 03, 05, 08 and 09 filled (scan:
  examples#3); the facade gaps the import allowlist records closed (scan: examples#13).
- [ ] **The planning demos and notebooks on the solution verbs** (D1.4) **[ask]**: the
  `planner.solve().trajectory` one-liners, the hand-rolled comparisons and the per-file
  `@dataclass` rows become `solution.plot_*` and `compare(...)` (a runnable draft of
  `vi_pendulum_lqr.py` exists).
  Also: `Planner.plot_solution` is the solution's own `plot_trajectory` (scan: planning#7).

**D2 — consolidation picks** still open from
the 2026-09-05 consolidation inventory, in git history (nothing removed without the maintainer's
pick):

- [ ] `Source.show_signal` (80 lines of bespoke matplotlib; the same picture is
  `source.plot_trajectory(tf=…)`), with the sources demo and `__main__` that exist only for
  it (scan: graphics#14) **[ask — user-callable]**.
- [ ] Dead modules `planning/spatial/overlays.py` (retires with A3) and
  `experimental/symbolic/mechanics/utils.py` **[ask — importable names]**.
- [ ] MPC debug figure → `planning/trajectory_optimization/mpc/viz.py`.
- [ ] Plotting in eight homes: write the placement rule in DESIGN first, then move
  `graphical/port_map.py` next to `control/` if agreed.
- [ ] The DESIGN research-lane trim (scan: docs-gov#10).
- [ ] The three catalog rows the dynamics scan names (scan: dynamics#13).
- [ ] One `ctl` for the model-based laws; `LookupPolicy` beside the other laws
  (scan: control#8, control#13).

**D3 — hardening rows** (research lane, small; S53 and the `HybridSimulator` drift row moved to
§1):

- [ ] RRT `KinodynamicExtender` ignores `problem.params.system`; `extenders.py` probes
  `getattr(U, "box", None)` instead of `U.bounding_box()`.
- [ ] RRT checks `X.contains(sample)` and the goal region without the time: a time-varying `X` or
  `Xf` is read at `t = 0` (`rrt.py`, `rrt_star.py`); the tree nodes need a time (the sum of the
  edge durations) and the checks pass it (found 2026-10-23).
- [ ] `ShootingTranscription` orphaned from the string presets.
- [ ] `CostDensityField` / `WorkspaceField` export decision (with A3).
- [ ] **S43** Constructor-derived `camera_scale` hints (boat, steering, propulsion, plane,
  arms, rotating cart-poles) → auto-fit or params-derived; raw ground `CustomLine`s in catalog
  skins → `ground_line()`. Also: a renderer capability table with honest fallbacks (scan:
  graphics#7); the physics owns the drawn lengths (scan: dynamics#9).
- [ ] RRT* tracks goal nodes incrementally; the transcriptions lower the state set
  member-wise (scan: planning#6, planning#12).
- [ ] **Third-round notes from the 2026-09-23 fix reviews** (small, or design questions;
  details in the 2026-09-22 improvement suggestions §2.4, in git history): `step_diagram % dt`
  adds boundary ports to a user `StepDiagramSystem`; the sampled `@` copy of a one-block
  plant diagram drops a diagram-level function output port; a `%` override that ends in
  `as_computer(self, schedule)` would recurse, and `System.__mod__` calls a private helper;
  `RealtimeSimulator`'s offline automatic `dt` reads the hint without `refresh()`; an
  explicit `backend='jax'` on a NumPy-only set or cost raises a raw trace error;
  `discretize(diagram, params={'dt': v})` is refused although a diagram would keep its
  blocks' params, and a raising params setter can leave a diagram half-updated;
  assigning into `q.labels[i]` in place no longer sticks; the gravity hook's signature is
  read on every call; the collocation demo's SLSQP fallback is not run in CI (its manifest
  entry still requires Ipopt).
- [ ] Bare `import jax` in `simulation/compile_backend.py` and `from jax.errors` in
  `analysis/derivatives.py` → `require_jax()` (RULES 5.12); the two rows the textbook-style
  allowlist still carries.

**S67–S69 — one plot vocabulary, contracts as tests, students' laptops** (S65 and S66 moved to
§1):

- [ ] **The compile-time flatten default**: `compile_diagram(..., flatten=False)` inlines nested
  boxes with `__` ids; the parity tests and `benchmarks/scenarios/nested_compile_flatten.py`
  compare the nested compile, `flatten=True` and hand-flat wiring. Run the benchmark once on the
  maintainer's reference machine, then keep the default off, turn it on, or drop the flag and
  always flatten if parity holds (reviewer A's lean) **[ask — core]**.
- [ ] **One plot vocabulary** (S67) (scan: graphics#0, graphics#4, graphics#5, graphics#6,
  graphics#8, graphics#9, analysis#9, analysis#10): `title`, `ax`, `backend` and `show`
  honoured by every plot verb, one `PlotResult.axes` shape; `plot_trajectory` selects a
  leaf's named output ports; one axis-label formatter, one unit convention, one backend
  resolver; the renderers silent (RULES 4.6); a warning on the ±10 phase-plane window; one
  keyword set for the five control plots; the region plot overlays trajectories without
  touching `sys.x0`.
- [ ] **Contracts as tests** (S68) (RULES 6.12; scan: tests-ci#3, tests-ci#5, tests-ci#9,
  tests-ci#10, tests-ci#11, tests-ci#14, docs-gov#1, docs-gov#2, examples#8, control#11,
  planning#10, dynamics#11; the RULES 5.8 ratchet landed as
  `tests/unittest/textbook_style.py` with its allowlist): ROADMAP §5 step ids and TODO rows agree, and no plan doc numbers its
  steps with a bare workboard-style id (scan: docs-gov#3); the CI regression flags agree
  across `test.yml`, `tests/run/_common.py` and the tests/README Agent table (scan:
  tests-ci#4); the subprocess demo-check tests behind a marker; wall-clock claims out of
  unit tests; unit tests stop importing
  `examples/projects` and `benchmarks/systems`; `Animator.resolve_frame` public for the
  graphics harness; course pins byte-identical to their topic twins; one both-backends test
  over every control law, the catalog with perturbed `params`, and the `params.sets`
  scoring parity.
- [ ] **follow-up** (S69): the `laptops` job (Windows and macOS legs) and the Linux coverage
  report landed; confirm the first remote CI run is green before the GRO501 cohort installs.

---

## 4. v0.4 — features on the core

The maintainer's direction (2026-10-26): v0.4 adds features built on the finished core, not
intertwined with it; the rows are independent and run in any order. A row that reads a core
pillar names it.

- [ ] **The realtime runner on the pure blocks** (R161, 2026-10-28): game mode and hardware in
  the loop re-based on the sampled loop's blocks (`update`, the port functions, `step`), pacing the
  ticks on the wall clock and reading the keyboard or the device between them; the step diagram its
  test oracle at the ticks; the GRO501 game demos re-pinned once. Reads core §5. Done when every
  realtime demo runs the same blocks the judge scores and matches the scan at the ticks.

- [ ] **Identification and generation** (C4) (its design states which fitting route it takes and
  flags it for the behavior-cloning decision, R136): identification is V1's program export
  (`pe.program(wrt=["plant.…"])` and the `Optimizer`) with a data-mismatch cost (`identification/fitting.py` on `rollout_batch`; physical params and NN
  weights are one verb), its first data consumer; `trajectory_generation/` port (polynomial,
  min-snap); SMC trajectory-following demo; trajectory post-filter; experiment-design
  helpers (the PRBS / chirp inputs live in `blocks/sources`).
  Also: the simulated `Trajectory` logs `dx` and `y`, so an equation-error fit has its data
  (scan: analysis#14).
- [ ] **LQGPlanner** **[ask — planner API]**: `LQGPlanner(problem)` on a
  `PlanningProblem` stochastic by its parts (R143), with a `QuadraticCost` and noise source blocks
  on `d` / `n` (R128), beside
  `LQRPlanner`: the policy is the `lqg` box, the record holds `K`, `L`, both Riccati matrices
  and the poles, the cost-to-go the certainty-equivalent form; `Q_n = B_w W B_wᵀ`,
  `R_n = D_v V D_vᵀ` from `linearize_matrices(sys, wrt="d")` / `wrt="n"` and the blocks'
  `psd`. With RN-4. Design: [estimation-and-state-feedback.md](estimation-and-state-feedback.md) §5.

- [ ] **The rest of the field object** **[ask — core]** (A1's steps 4.4–4.8 of
  [fields.md](fields.md), after the minimal `Field` of §1.9, R133): `GridField` (DP and tabular
  tables, a per-sweep time axis); `CallableField` (RL critics); `LinearApproximator` as a field;
  `LyapunovCertificate.V` a field; `PolicyEvaluator` takes the evaluators' verb and returns a
  field-shaped result (scan: planning#4). Done when the DP baselines `cmp` and
  `test_planning_solution.py`, `test_rl_tabular.py` and `test_rl_planner.py` pass.

**Workspace geometry.** Design: [geometry-module.md](geometry-module.md) (the field object is
§1.9). Baselines first, as each plan states.

- [ ] **Workspace geometry** (A3) **[ask — core]** (S57 was its earlier id).
  [geometry-module.md](geometry-module.md) implementation steps 1–6: `core/geometry/`
  package (shapes, paths, track, scene, probes, `mount`, spatial fields, grid, catalog
  `oval_circuit` / `racecar_circuit` / `holonomic_forest`); shaping next to `Field.as_cost`;
  plots and overlays under `graphical/` with lazy methods on `Scene` / `Track`;
  `planning.spatial` re-exports then deletes in the same change as the call sites;
  `PurePursuit` takes a Path / Track. Done when the two UdeS racecar demos and the RRT forest demo
  use the catalog, `control` imports geometry with no planning import, and the suite is
  green. Glyph rename stays S30.
  Also: `Shape` unions flatten like `IntersectionSet` and `SumCost` (scan: core#6);
  `PurePursuit` sizes its measurement from the vehicle, not `state_dim=9` (scan: control#9);
  retarget `planning/trajectory_optimization/mpc/viz.py`'s `TrackCorridorOverlay` import in the change that deletes
  `planning.spatial.overlays` (scan: control#12).

**On the spine.**

- [ ] **The path-integral planner (MPPI)** (S72) (a planner under the compiled-face contract of
  [planner-and-mpc.md](planner-and-mpc.md), R148; the seed and the tick, R104; one plant draw by
  default, R107), after the core's sampled loop and draws (§1.3, §1.5), its plant the step model (S75), on the evaluator's batched rollout over realizations. Design:
  [mppi.md](mppi.md); the standalone prototypes `experimental/mppi/pendulum_mppi.py`
  and `racecar_mppi.py` run today. MP-1 `planning/trajectory_optimization/path_integral.py`:
  `MPPIPlanner` / `MPPIRecord` (R106), its compiled face (mppi.md §6) on
  `RolloutEnvironment` (`jit(vmap(scan))` over `K` samples; MP-0 first moves it to
  `planning/environment.py`, the RL import kept, R108), the six-beat body of the plan's
  §5; MP-2 the hand-loop demo
  `experimental/mppi/pendulum_mppi.py`; MP-3 the stochastic branch
  (`n_plant_samples` realizations per control sample, mean or CVaR by the problem's
  criterion) and the fan of sampled futures as `solution.plot_samples()`. Done when the
  seeded-determinism, LQR-agreement and swing-up tests pass, the deterministic twin of the
  stochastic branch is byte-identical, and the body reads next to `dp.py`. MP-4 (the rebuilt MPC on MPPI) and MP-5 (demos, `compare(MPC=, MPPI=)`) before v0.9;
  MP-6 (the `loop` backend, facade names, DESIGN §6) with V3.
- [ ] **RL follow-ups** (C5):
  - [ ] **SAC actor step** (S50) **[ask]**: pre-update critic (today, kept for bit-identity) or
    updated critic (SB3, CleanRL). One line in `algorithms/sac.py`; re-measure
    `pendulum_swing_up_rl.py` if changed.
  - [ ] **Policy iteration on the grid** (S51) (course ch. 12) as a `DynamicProgrammingPlanner`
    mode or sibling. **[ask — planner API]**
  - [ ] **Deep Q-learning** (S52) (course ch. 17): a discrete-action head over the grid's input
    levels, `QFunction` with replay and a target network. Research lane first.
  - [ ] **Approximate dynamic programming** on `dynamic_programming/approximation.py` (fitted
    value iteration); design first, it touches the planner API. **[ask]**
- [ ] **General serial chain** (C6) **[ask — core]**: one mechanism description (DH table or
  URDF: geometry, inertia, topology) feeding RNEA / ABA beyond the UR5, the shared serial-chain
  stack, ABA on the other RNEA arms, the symbolic path reading the same description; the GMC714
  arm topic's modelling rung. Design first (the July 2026 articulated-mechanism draft is in git
  history; it is re-planned here, no separate `KinematicModel` delegate: its one mechanism description is the robot's home for its
  frames, feeding the dynamics, the kinematics and the drawing's poses, R137, a preliminary
  choice of 2026-10-27). Moved here from wave G on 2026-10-22; the GMC714 audit (2026-10-23) found the labs need no
  second arm description (`ThreeLinkManipulator` and the UR5 cover them). Done when a second arm runs on the shared
  stack with the catalog both-backends test.
- **The former wave G rows** (moved from v0.3 by R48; none is needed by GMC714):
  - [ ] **Pyro parity** (C1) open rows — [pyro-port-remaining.md](pyro-port-remaining.md); then
    the pyro → minilink migration guide in README from the name map there. **[ask]** for the
    README. Re-audit the table against the code first (scan: docs-gov#11). Which pyro demos the
    courses still need is the maintainer's pick (of pyro's 195 example scripts: 29 done, 47
    partial, 83 to do, 36 dropped).
  - [ ] **Modelling ladder** (C2): manipulators + the four-rung vehicle ladder as a
    `02_dynamics` tutorial lesson (not a GMC714 lab); robotic PID wrappers (`JointPD`, `EndEffectorPD` twins).
    Also: one constructor contract for the robotic laws (scan: control#6).
  - [ ] **Blocks** (C3): `Ramp` / `Chirp` / `Delay` / `Switch`; `Integrator` and `UnitDelay` take
    a `dim` like every static block (scan: graphics#11).
  - [ ] **DP reads the horizon** (S63) **[ask — planner API]** (scan: planning#1): a finite
    `problem.tf` runs `round(tf / dt)` backward sweeps as `solve_steps` does, the way
    `LQRPlanner` picks the Riccati ODE, or warns once. Done when LQR and DP agree on the
    finite-horizon double integrator.
  - [ ] **Catalog hygiene** (S64) **[ask — catalog]** (scan: dynamics#0, dynamics#1,
    dynamics#2, dynamics#3, dynamics#4, dynamics#6, dynamics#8; the mechanical default
    bounds landed, R68: each plant states its own): output ports that are
    the state or a slice of it read labels and units from the state (the 56 copy lines go);
    one port layout across the bicycle rungs; one owner for the wheelbase; the hidden `0.01`
    damping in `Drone2D.d` and `Rocket.d` named; `VanderPol` with a real input or none;
    constructor hygiene in the pendulum and mass-spring-damper families.
- [ ] **Traceable linear analysis** (S73) **[idea to review — ask — core]**, proposed by the
  maintainer 2026-10-18, not scheduled: `poles`, `frequency_response` and `margins` in
  `analysis/linear.py` through the array module, so the sensitivity of closed-loop poles,
  damping ratios, Bode magnitude and phase, and gain and phase margins to any plant or
  controller param is `jax.grad` of the existing verbs on `jacobian("f", "params")`. The
  math lines stay the textbook ones (`C (jωI − A)⁻¹ B + D` as one batched solve instead of
  the Python loop; eigenvalues through `xp.linalg.eigvals`, whose derivative is the classical
  `vᴴ (∂A/∂p) u / (vᴴ u)` sensitivity). Brent's method on the crossovers becomes a fixed number
  of Newton steps from the grid bracket, so the crossover frequency carries the
  implicit-function derivative. Zeros stay NumPy (a generalized eigenproblem JAX lacks; the
  companion-matrix route for `D = 0` is a later step); root-locus branch matching stays NumPy
  (a plotting concern); the root locus generalizes to any parameter (`root_locus(wrt="m")`).
  Documented non-differentiable spots: repeated poles, the switch of the minimum margin
  between crossovers; JAX's nonsymmetric eigensolver is CPU-only. Recommendation from the
  2026-10-18 architectural review: alongside crossover `margins`, expose the maximum
  sensitivity peaks $M_s = \max_\omega |S(j\omega)|$ and $M_t = \max_\omega |T(j\omega)|$
  (and their smooth `logsumexp` surrogates over $\omega$) as traceable metrics on
  `sensitivity` / `complementary_sensitivity`, providing a smooth, crossover-free robustness
  bound for NLP gain tuning. Independent of V1 (needs
  only the jacobian), self-contained for a cloud agent. Done when `jax.grad` of the phase
  margin and of each closed-loop pole of `PID() @ Pendulum()` with respect to a gain and to
  the mass matches a central difference, and the NumPy callers see no change. Design:
  [differentiable-problem.md](differentiable-problem.md) §4.6.
- [ ] **Native solvers beyond the proof of concept** **[ask — core]** (moved from §6, where it
  was a Later idea; the proof of concept, NS-1, NS-2's fixed-`K` form, NS-8 and NS-9, is §1.8):
  native optimizers, the solve as a rollout (the maintainer's vision, 2026-10-25): native
  solvers as pure laws beside the wrapped ones, program categories, fully traced JAX solves
  (NS-2's tolerance form, NS-3 implicit differentiation, NS-4 least squares, NS-6 the augmented
  Lagrangian, NS-7 the stochastic category); MPPI's tick a pure instance of the
  fixed-iteration tick (native-optimizers.md §4.6b; the proof of concept puts the gradient MPC
  first); long term, convergence analysed as stability
  (NS-5, with the discrete step analysis ruled in, R122) —
  [native-optimizers.md](native-optimizers.md) (its shape a choice, R132; v0.4 by the direction
  of 2026-10-26).

---

## 5. v0.9 freeze candidate and v1.0

ROADMAP §5.5 and §5.6: v1.0 means a colleague builds a course on minilink and trusts it for
three years. The foundation rows are design conversations before code.

**Order:** the foundations (S27, S44, S30, V3; S31 and S75 moved to v0.3, S37 absorbed by S75) before the trust contract; V11 once,
after the v0.3 tag (planned for December 2026 when v0.3 had that date); the v1.0 rows after two cohorts.

### v0.9 — what could still break a name

- [ ] **Where the course material and the projects live** (K5) **[ask — student-facing]**
  `examples/teaching/` and/or `examples/projects/` in their own repositories, or here, and the
  examples tree's layout (repository-structure.md §3: tidy today's names, two axes chapter and
  course, or notebook-first; recommended two axes). The recommendation for the material: pinned
  cells now, a course repository at v1.0. Options, a recommendation and the asks:
  [repository-structure.md](repository-structure.md). Done when the ruling is recorded.
- [ ] **The move** (K6), if K5 says so **[ask — student-facing]**: the history kept
  (`git filter-repo` per folder), every citing document updated (the GRO860 notes and webpage
  links, GRO501's `\colab{}`), the new repositories' CI pinned on a minilink release, the
  notebook smoke check moved with them, this repo's README and `examples/README.md` pointing
  there. Done when every course link resolves and both CIs pass.
- [ ] **S27** Diffrax as an optional JAX solver backend, beside the step model's integrators,
  only if a benchmark asks for it, an optional extra (`solver="diffrax"`), never required (R139,
  a choice of 2026-10-27)
  (`core/compile/evaluators/stepping.py`, kept there by OD-11). S37 (evaluators keep pure maps and one scannable step; the integrators'
  move; the evaluator internals deduplicated: ZOH sugar, the JAX step rollout, the four
  Jacobian probes, the undeclared batch cache; scan: compile-sim#9) is absorbed by S75's SM-1
  and SM-6 (ruled 2026-10-22).
- [ ] **S44** A single posed-geometry hook so "two functions" (`f` + drawing) is literal;
  today `tf` + skin. Designed with C6's mechanism description, so a robot's drawing reads the
  same frames (R137).
- [ ] **S30** Rename the graphical `Sphere` / `Box` glyphs to `SphereGlyph` / `BoxGlyph`, so no
  two importable public types share a name with the geometry solids (R139). **[ask — public
  names]**
- [ ] **The frozen surface** (V3) **[ask — public names]**: list the names the v1.0 freeze
  covers, drawn from one rule (R139, a choice of 2026-10-27): a name joins the freeze
  when it has its teaching demo, its test, and has gone through one release unchanged. `StepSystem`, `StepDiagramSystem`, `UnitDelay` (blocks facade), `ModelPredictiveController`
  (control facade) and the spatial names A3 moves to `core/geometry/` (provisional teaching
  names, ROADMAP §2) join it or move behind a visibly provisional name. Done when the
  teaching-surface registry (`tests/unittest/test_teaching_surface.py`) marks each name
  frozen or provisional and the API site shows the mark.

### v0.9 — the trust contract

- [ ] **Deprecation policy** (V4) **[ask]** (open: the shim below was not adopted on 2026-10-27,
  R139; R82 holds meanwhile). The proposal was: from v1.0, a teaching-surface removal or rename
  ships one minor release as a `DeprecationWarning` shim (RULES "no deprecated aliases" is
  amended for v1.0 on). A snapshot of the export list is a test; changing it fails until the
  snapshot and the CHANGELOG line move together. Done when RULES, ROADMAP §2 and the
  snapshot test agree.
  The rename list, applied with shims under the policy (the names hold until then, with no
  alias before): `xbar` / `ubar` → `x_bar` / `u_bar`, the name everywhere else, on the costs
  (`QuadraticCost.from_system(xbar=, ubar=)`, `cost.xbar`, `cost.ubar`, `TimeCost`'s
  `xbar`); `plot_cost2go` → `plot_cost_to_go` (A4). (`NeuralPolicy` is renamed in v0.3, R151.) The policy
  itself is open (QUESTIONS; R139).
- [ ] **`CHANGELOG.md`** (V5): user-facing, one entry per release from v0.9 (back-filled for
  0.1.0 / 0.1.1 from ROADMAP §2's breaking changes); `pyproject.toml` `Changelog` URL points
  there. Done when `publish.yml` refuses a tag without its CHANGELOG heading.
- [ ] **The pinned Colab cell** (V6) **[ask — student-facing]**: conda from `environment.yml`
  is the recommended install and README leads with it; `pip install minilink` is the light
  alternative, one `full` extra beside it (one install story, ruled 2026-10-17,
  [record](../reviews/2026-10-17-one-install-story.md)). What remains: the Colab setup cell
  clones a pinned release (`git clone --depth 1 -b vX.Y`, R113: the notebooks' relative
  imports of the examples tree keep working) instead of `git clone main`;
  `COLAB_CLONE` in `tests/unittest/test_repo_contract.py` follows. `showcase_jax.ipynb`
  imports `minilink.experimental.c_export`, which the wheel does not ship: settle it here.
  The "pin the course notebooks on `0.2.0` now" option of
  [repository-structure.md](repository-structure.md) §4 waits here (ruled 2026-10-22: course
  sync is secondary to the order of the waves; the course notebooks keep tracking `main` and
  migrating in the same commit until then).
  Done when every notebook carries the pinned cell and runs in Colab.
- [ ] **`MinilinkError` family** (S70) (ruled 2026-10-25, R114): `WiringError(ValueError)`,
  `ShapeError(ValueError)`, `SolverError(RuntimeError)`, subclassing what is raised today (no caller breaks);
  S66's wiring-time checks and P10's refusals raise them. Done when the wiring and compile
  paths of `core/` raise only the family.
- [ ] **Rewrite the history without notebook outputs** (V11) **[ask — force-push]**, in
  December 2026 after the v0.3 tag, between terms (as planned then; now after the v0.3 tag, which
  has no date). A fresh clone is 176 MB for 19 MB of
  files: 169 MB of the history is `.ipynb` outputs committed before the `nbstripout` hook
  worked, mostly five notebooks deleted by May (`examples/notebooks/demo*.ipynb`,
  `animation_colab.ipynb`). `main` carries it all, so deleting branches or `archive/*` tags
  does not help. Steps: keep a bare mirror clone as the backup; `git filter-repo` stripping
  outputs from every `.ipynb` blob (or dropping the deleted notebooks' paths); re-point the
  release tags `0.0.1`–`0.1.1` and the `archive/*` tags; force-push branches and tags; the
  re-clone notice to students and collaborators, and the line in the CHANGELOG (V5). PyPI
  releases are untouched. GitHub keeps the old objects behind `refs/pull/*` until its support
  purges them; clones shrink regardless. Done when `git clone` of the repo is under 25 MB and
  CI is green on the rewritten `dev`.

### v0.9 — the syllabus holes

- [ ] **Estimation follow-ups** after P4, each in the slot
  [estimation-and-state-feedback.md](estimation-and-state-feedback.md) §6 names: the
  finite-horizon filter (`kalman_finite_horizon` → a time-varying observer, the dual of
  `lqr_finite_horizon`, the forward sweep on the dual pair with `riccati_step`); the
  linearized Kalman filter along a trajectory (`trajectory_kalman` → `TrajectoryObserver`,
  the dual of `trajectory_lqr`); the extended Kalman filter (`ExtendedKalmanFilter(model, Q_n,
  R_n, x0, P0)`, `P` a state, Jacobians taken on the model's `f`, `h` inside its own `f`);
  the sampled filters (unscented, discrete Kalman, particle) after AC-8a / S31; the
  `EstimationProblem` / `EstimationSolution` and the `*Estimator` tools (§5 of the plan);
  online parameter estimators (recursive least squares, gradient laws) under a role of their
  own. Each revisits the observer conventions of
  [automation-by-convention.md](automation-by-convention.md) §10.
- [ ] **The classic first plants** (S71): DC motor, tank / thermal first-order-plus-delay
  process, ball and beam, differential drive, 3-D quadrotor, each with `tf`, a skin and the
  catalog both-backends contract test; LQI beside the `N` matrix of P8. Done when each has a
  one-line demo in `examples/demos/dynamics/`.
- C4 (identification, trajectory generation) is a v0.4 row (§4), as V1's program export with
  a data cost; it moves here only if v0.4 does not land it.

### v0.9 — the reference site

- [ ] **The docs site as a guide** (V7): myst-nb (or nbsphinx) renders `examples/tutorial/`
  from stored outputs; a plant gallery (one image per catalog plant from
  `docs/make_assets.py`); a concept page (System, composition, tools as verbs). Done when
  the Sphinx `-W` build on `docs.yml` publishes all three.

### v1.0 — a colleague can build a course on it

- [ ] **Textbook-validation suite** (V8): `tests/unittest/test_textbook_examples.py`, worked
  examples from Dorf & Bishop, Ogata, Franklin–Powell asserted numerically (margins, step
  specifications, root locus, LQR and Kalman gains), each citing its example number; GRO501
  gate 3 is the seed. Done when the README links it as the correctness evidence.
- [ ] **Course-neutral labs** (V9) **[maintainer]**: `examples/teaching/topics/` grows from 7
  to 10–15 labs (objectives, prerequisites, time, starter and solution), classical control
  and robotics first; a "For instructors" page (pin a version, Colab, adapt a lab, report a
  bug).
- [ ] **The public face** (V10) **[ask]**: the README opens on a teaching quickstart; TRL,
  lanes, wave and step ids leave README and `examples/README.md`; `CONTRIBUTING.md`, issue
  templates, a code of conduct (the governance docs moved under `docs/dev/` on 2026-10-17).
- [ ] **V2** Zenodo archive and a citable DOI (`CITATION.cff`), now (R115: the maintainer links
  the repository on Zenodo once; each release then gets a DOI); the JOSS entry and the RULES 6.9
  carve-out for its state-of-the-field section, decided after v1.0.
- [ ] **The Stable-Baselines3 teaching notebooks** (S49) **[ask]**, at the end of the term.
  Four, in `teaching/courses/udes_gro860/`: retire `drone_ppo_sb3.ipynb` and
  `pendulum_value_iteration_vs_lqr_vs_ppo_sb3.ipynb` (delete them, their notebook overrides
  and allowlist entries, and the README rows marked Stable-Baselines3) once the course notes
  point only at the native twins; decide `particle_gymnasium_sb3.ipynb` and
  `gymnasium_interface.ipynb` then.
- [ ] **Realtime architectural review** **[ask — core]**: `RealtimeSimulator` and the pygame
  I/O (`simulation/realtime/simulator.py`, `io.py`, each marked `TODO: User Architectural
  Review`) reviewed before the freeze; the band joins the V3 surface or stays provisional.
  Done when both markers are gone.

### After v1.0 — 1.x features, additive

- [ ] **S36** iLQR planner from parts (`jacfwd` of `ev.f`; idea, research lane).
- [ ] **Behavior-cloning pipeline** **[ask]**: a game mode on `RealtimeSimulator` that records
  human demonstrations (keyboard or joystick driving the plant live) as trajectory datasets; a
  planner that fits a policy to them (imitation beside the RL planner, `NeuralPolicy`
  as its output); the cloned controller closed on the plant and compared against the
  demonstrator. Its first scenes are realtime games written for it (pyro's are not ported, R109).
  How fitting enters the API is decided here (R136, a choice of 2026-10-27): the
  program export with a data-mismatch cost (a dataset as `Trajectory`s, no new verb), or a `fit`
  verb beside `solve` with a dataset noun ([future-policies.md](future-policies.md) §7).

---

## 6. Later ideas

One line each; open a plan doc only when a design needs a writeup.

- **[ask]** From the 2026-10-20 convergence analysis, not ruled when K7 closed (R69): the
  racecar 3-D skin, `meshes.py` and `assets/` to the racecar plant folder (between terms; a
  GRO501 notebook imports it by path); `Sys2Gym` / `SB3Controller` to the experimental tier
  when the SB3 notebooks retire (S49); `planning/search/`'s `dubins.py`, `steering.py`,
  `live_plot.py` and `plotting.py` cut to what the two RRT demos use; the facade verbs no
  pillar needs (`get_diagram`, `plot_input_output_map`, `render`) decided before V3.
- A `ClippedCost`, when a saturating cost is needed (planned with the cost, 2026-10-23).
- A tutorial on gradient descent and the `Optimizer`, once the generic optimizer and the
  mathematical-program system are extended (R91); until then the closing PID example teaches it.
- **A plant that owns its noise ports, as a real example** (R57): a disturbance the loop cannot
  add (unmatched, `B_w ≠ B`: a push on the cart-pole's tip, a lateral wind on `Drone2D`) and noise
  on some sensors only, with a demo and its LQG twin test. When it lands, `NoisyPendulum` and
  `NoisyCartPole` are revisited: one becomes that example, the other folds into
  `closed_loop(disturbance=, noise=)` (the tutorial cells and GRO501 `cartpole_dynamic_controller`
  migrate). **[ask — student-facing]**
- Order reduction by neglecting fast modes (out of v0.2 by decision): keep the slow modes and
  the DC gain, a band function in `analysis/modal.py`. Separately, swap the guide's §9.13
  realization into `TestMinreal` and its §1.4.2 parking model into `TestPolePlacement` once
  the matrices are in hand (stand-ins today).
- Automatic reward scaling for the RL planner (`reward_scale="auto"` = `1 / (g_max dt)`): raw
  costs throttle PPO through the joint gradient clip; the scaled planner reaches LQR on every
  seed tested — [rl-reward-scaling.md](rl-reward-scaling.md) (RS-1–RS-4; whether it is the default,
  which moves every RL demo's numbers once, is the maintainer's call) **[ask — planner API]**.
- The CBF safety filter ([cbf-safety-filter.md](cbf-safety-filter.md)): research lane until a
  course asks; after the minimal `Field` and the workspace geometry (A3).
- The automatic `d` / `n` plant wrapper of P10, deferred until someone needs it
  ([automation-by-convention.md](automation-by-convention.md) §10).
- Vehicle view ports (`pose` / `bodyvel` on `DynamicBicycle` for impedance / PID).
- **[ask]** Simon's curve speed scaler in the path-tracking project: `PathTracking` in
  `examples/projects/pathtracking/bicycle_los_v2/path_tracking.py` holds `vx_ref` constant;
  Simon's LOS caps it by the heading change ahead along the path (the weighted sum of
  `|Δψ|` over a look-ahead window, mapped to a speed limit through lookup tables). Source:
  PR #99, `minilink/control/guidance_los_coupled.py` (`compute_curve_demand_norm`,
  `speed_limit_from_curve_demand`). Weigh the tables against the one-parameter textbook cap
  `v ≤ sqrt(a_y_max / κ)` on the sharpest curvature ahead.
- `SimulationOptions`, one record for the `Simulator` solver presets (DESIGN §5), weighed
  against RULES 2.5 (no option bags) before it is built.
- A sample stride on the fixed-step solvers (`compute_trajectory(..., substeps=k)`: `k` RK4
  steps per stored sample, the scan keeping only the samples): a stiff model stepped at
  0.5 ms over 40 s stores 80k states today, which is what pushed jaxnet to keep its own
  decimated `lax.scan` rollout beside `Simulator`.
- Cross-fidelity `lift` / `project` maps on the car ladder (a July 2026 draft, in git history).
- `RobustPlanningProblem` (set-bounded uncertainty, minimax criterion) only when a minimax
  consumer exists; the path-integral planner's `"worst_case"` branch (CVaR over plant draws,
  [mppi.md](mppi.md) §4) would be that consumer; the two decisions go together.
- Diffusion policy (research lane only, JAX-only, after behavior cloning, R136): a sampled
  controller with memory and a seeded draw; see [future-policies.md](future-policies.md).
- `MjxPlant` (`interfaces/mjx.py`) and `torch` / `flax` model wrappers in `interfaces/`;
  Pacejka tire; stochastic forcing; ROS2 / FMI; sparse long-horizon trajopt; RRT-Connect.
- Ipopt given the Lagrangian Hessian, not the objective's alone (scan: planning#13);
  `optimizer_method="auto"` picking Ipopt when installed (scan: examples#11).
- `LowPassFilter(tau=)`: the shaping filter of the colored-noise recipe parameterized by its
  time constant, beside `cutoff_hz` (randomness.md RD-D23, RN-4).
- A fixed-step RK4 that samples held sources once per step, so the four stages read one
  sample and the block route equals the port route (randomness.md RD-A8, RD-D18): fourth order
  with no boundary error at `dt = Δ`, one draw per step instead of four. Worth it only if a
  twin test shows the first-order bias.
- A key check on leaf params (a `validate_params` hook on `System`, no-op by default, called
  where diagram params are validated): today a typo or a retired key in any block's params is
  ignored silently; a question for the whole library.
- A 3-D still of any animation: `save_frame(t)` on the meshcat renderer writes the static
  HTML of one frame. The headless screenshot side exists: `docs/asset_export.py::html_to_gif`
  seeks an exported page frame by frame, so a still is one `seek` and one screenshot. With it,
  two camera notes for S43: the default meshcat eye sits low so the viewer's grid reads as
  horizon lines, and an overlay (a track corridor) drives the auto-fit camera, so a follow
  camera cannot zoom on the car; overlays should opt out of the fit as the ground lines
  already do.
- Declined (do not re-propose): scalar / list signal bounds and a coercing `x0`; scalar `Q` /
  `R` / `S` in `QuadraticCost.from_system`.
