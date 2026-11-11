# Minilink workboard

Every open step to v1.0, by release, in order. Each row carries a **bold plain title**, then its
id in parentheses. The id is an anchor, not a vocabulary: prose leads with the title. Each step
is sized at roughly one agent-hour to one agent-day and ends with a "done when" gate.
**[ask]** marks maintainer-owned territory (AGENTS "Ask first"); unmarked rows are
agent-managed. Landed rows leave this file; their history is git. The pieces live elsewhere:
- **Why each v0.4 row exists:** [ROADMAP §5.4](../dev/ROADMAP.md#54-v04--the-clean-surface), its leftovers table, the same N as §1.N here.
- **Its open design questions:** [QUESTIONS.md](../dev/QUESTIONS.md).
- **Milestones:** [ROADMAP](../dev/ROADMAP.md#5-the-path-to-v10).
- **Pyro parity:** [pyro-port-remaining.md](pyro-port-remaining.md).
- `scan: band#n` cites the 2026-09-22 improvement scan, which is in git history before
  2026-10-17.

| Section | Release |
| --- | --- |
| [§1](#1-v04--the-clean-surface) | v0.4 — the clean surface: the v0.3 core's leftovers by workstream, the core API proposals, the minimal surface, the root exports |
| [§2](#2-v04--the-clean-up-lane) | v0.4 — the clean-up lane |
| [§3](#3-standing-work--textbook-code-minimal-demos-consolidation-the-plotting-lane) | Standing work |
| [§4](#4-v05--features-on-the-core) | v0.5 — features on the core |
| [§5](#5-v09-freeze-candidate-and-v10) | v0.9 freeze candidate and v1.0 |
| [§6](#6-later-ideas) | Later ideas |

## Rows waiting for the maintainer's go

Workboard rows marked **[ask]**: each waits for the maintainer's go, pick or review before an
agent starts. They are details, versions and demos, not design questions (those are
[QUESTIONS.md](../dev/QUESTIONS.md)); each row holds its options. The core's leftover rows
(§1.1 to §1.10) without the mark have the go: the internal choices an agent meets are taken as
the maintainer would and logged in the run's record.

| Step, in plain words | Id | Release | What the ask is |
| --- | --- | --- | --- |
| The API settled before the demo rebuild | — | v0.4 (before the rebuild) | your go on [api-stabilization.md](api-stabilization.md): its defects section landed (2026-11-10); your go on the two other sections that need no decision, then its 24 decisions one at a time (the "API sweep's judgment calls" row of §1.11 is folded into them) |
| Your demo rebuild, then the tests' clean slate | MS-D1 | v0.4 (end) | your new minimal demo set; decided with it: the test layers, the tabular Q-learning demo, the MPC-tuning demo, the cart-pole RL demo's budget, the supported plants, GMC714's pins (§1.12) |
| The car cut's two skins and the bicycles' port layout | K8-0 | v0.4 | `KinematicCar` and `DynamicBicycleCar3D` as skins: a name for the four-wheel 2-D skin, the corner arrows (a skin is static geometry), the pathtracking project's import ([vehicle-models-run.md](vehicle-models-run.md) §6, the pull request); the one port layout of the bicycle rungs ([vehicle-models.md](vehicle-models.md) §9) |
| The mechanical hierarchy and the car models' base | S32 | v0.4 end or later | your review: which class is the child of which, the cars' base, the mass matrix's one name |
| One owner per rule, its two public items | S65 | v0.4 | one set of Monte Carlo defaults for `evaluate` and the planners' evidence; one empty-cache rule for the `compute_*` facades |
| Where the course material lives; the examples tree | K5 | v0.9 | repository-structure.md's options |
| The demo sweep to the flatness rule | D1.3 | standing (wave D) | your review of each diff; `showcase_jax` waits |
| The planning demos on the solution verbs | D1.4 | standing (wave D) | `solution.plot_*` and `compare(...)` in place of hand-rolled comparisons |
| `Source.plot_signal` | D2 | standing (wave D) | remove it (the same picture is `plot_trajectory`) |
| Two dead modules | D2 | standing (wave D) | remove `planning/spatial/overlays.py` (with A3) and `experimental/symbolic/mechanics/utils.py` |
| LQG as a planner | LQGPlanner | v0.5 | `LQGPlanner(problem)` beside `LQRPlanner` |
| Workspace geometry | A3 | v0.5 | the `core/geometry/` package; `planning.spatial` moved, then deleted |
| SAC's actor step | S50 | v0.5 | the critic before the update (today) or after it |
| Policy iteration on the grid | S51 | v0.5 | a mode or a sibling of `DynamicProgrammingPlanner` |
| Approximate dynamic programming | C5 | v0.5 | design first; it touches the planner API |
| The general serial chain | C6 | v0.5 | one mechanism description (DH table or URDF) |
| The pyro → minilink migration guide | C1 | v0.5 | its section in README |
| Which pyro demos the courses need | C1 | v0.5 | your pick |
| Catalog hygiene | S64 | v0.5 | the per-plant clean-ups |
| Traceable linear analysis | S73 | v0.5 (any time) | `poles`, `frequency_response`, `margins` through the array module, and the sensitivity peaks |
| The move of the course material | K6 | v0.9 | only if K5 says so |
| The pinned Colab cell | V6 | v0.9 | a pinned release in every notebook; `showcase_jax`'s `c_export` import |
| A lighter git history | V11 | v0.9 | a force-push without notebook outputs (176 MB clone to under 25 MB) |
| The convergence leftovers | TODO §6 | before V3 | the racecar 3-D skin moved; `Sys2Gym` / `SB3Controller` to the experimental tier; `planning/search/` cut; three facade verbs |
| The README's two headline examples | — | v0.4 (with the demo rebuild) | the planner comparison's numbers so each planner reaches the goal, and bounds on the co-design example (KNOWN_ISSUES, "The README's examples") |
| A linear model's matrices | — | before the v0.9 freeze | `lin.A()` kept as a call, or `lin.A` an attribute at the block's own `params` (KNOWN_ISSUES) |
| The public face | V10 | v1.0 | a teaching quickstart first in README; `CONTRIBUTING.md` and templates |
| The Stable-Baselines3 notebooks | S49 | v1.0 | retire two, decide two |
| The realtime band's review | — | v1.0 | frozen surface or provisional |
| Behavior cloning | — | after v1.0 | the game mode and the fitting planner |
| A plant that owns its noise ports | — | later | the example, then the two noisy plants revisited |
| Simon's curve speed scaler | — | later | into the path-tracking project, or the cap `v ≤ √(a_y,max / κ)` |
| The compile-time flatten default | — | standing | after one benchmark: off, on, or always |
| Reward scaling by default | — | later | `reward_scale="auto"` the default or opt-in |

---

## 1. v0.4 — the clean surface

**Next, in order** (as of 2026-11-10). One rule orders v0.4: what changes a public name or a
behaviour lands first, so the demos, notebooks and tests are redrafted once, on the settled
surface, and the student-facing numbers are re-pinned once, at the release. Additive work that
breaks nothing goes last and may follow the tag.

Phases 1 to 4 landed on `dev` (2026-11-08): the planners' math and fields and value
iteration's finite horizon (plans as policies, its first two changes), the core API proposals
and the later round of API sweeps as ruled, the core's small leftovers, and the maintainer's
rulings of 2026-11-07; then the API naming sweep (PR #138) and the API review's defects
(2026-11-10). Their contracts are DESIGN's; the release's status is ROADMAP §5.4's. Every
feature branch is merged into `dev`. What is left:

4b. **The API settled before the demo rebuild** (§1.11, [api-stabilization.md](api-stabilization.md)
   §2 to §5) **[ask]**: on the maintainer's go, the names a rule already decides and the
   additions that break nothing, then its 24 decisions one at a time, then the planned renames
   pulled forward, so the rebuilt demos are written once.
5. **The surface redrafted on the settled names** (§1.12): the maintainer's demo rebuild, then
   the tests' clean slate and the course notebooks on the new names; beside it the clean-up lane
   (§2: the catalog clean-up K8, the experimental tier K9, the car family's time constant), and
   last GMC714's names pinned (G3). The small rows of §1.1, §1.2, §1.7 and §1.11 are
   independent and run beside it; one that renames a public name lands before the redraft.
6. **The release** (§1.10): the student-facing numbers re-pinned once, the course notebooks on
   the new names, the PR from `dev` to `main`, `0.4.0`.
7. **The faces and the MPC protocol** (§1.8, plans as policies, change 3; ruled): additive (no
   name breaks, the open-loop loops byte-identical), so it lands before the tag if it is ready,
   else in a 0.4.x or with the RL and RRT faces in v0.5 (§4).

**How the work flows.**
- **Status:** [ROADMAP §5.4](../dev/ROADMAP.md#54-v04--the-clean-surface). v0.3 shipped as
  `0.3.0` (2026-10-30); §1.1 to §1.10 are its core's leftovers, by workstream, §1.11 to §1.13
  the clean surface's own rows.
- **Where the work lands:** feature branches, merged into `dev` on the maintainer's go.
- **Reaching `main`:** nothing merges to `main` until every notebook has moved to the new
  names, and when it does is the maintainer's call. The course material on `main` runs on
  `0.3.0` until then.

**The gates**:
- **A restructure** agrees to `rtol 1e-6`, and a pure move or rename stays identical.
- **An intended number change** runs the same problem and cost before and after, with the
  relative differences in the step's report.
- **A rename or keyword change** lands with all its callers, with no alias.
- **A step that restructures a computation** records a seeded baseline first (AGENTS' recipe,
  a scratch script and its JSON, run twice and `cmp`'d), and its math is read next to
  `planning/dynamic_programming/dp.py`.
- **Re-pinning:** an intended change re-pins the regression baselines it moves
  (`benchmarks/baselines/`) in its own commit, with its closeness report, so CI stays green;
  the student-facing numbers (notebook outputs, course figures) are re-pinned together at the
  merge to `main`.

**Each workstream's main rows landed in v0.3**; the rows below are what is left, each small.

### 1.1 Every object pure, every number a parameter

- [ ] **Later nouns, each with its first consumer** (A5; `Set.as_cost`, the library shapes and
  fields reading `params`, and `log_prob` on the seeded policy, landed in the v0.3 runs):
  - parameter-dictionary support on sets and distributions (one flatten rule, with
    identification or the robust problem);
  - `UnionSet` via `|`, when reachability or multi-goal work needs it;
  - vector bounds on `Saturation`;
  - `PlanningProblem.hamiltonian()`, only if the course teaches Pontryagin;
  - the double-integrator homework's two verbs, entry time into a set and the Bellman residual
    (scan: examples#10).
- [ ] **Sets as costs, the lowerings, the metrics as constraints** (the path to the cost's last
  open step, V1 step 8):
  - the robust-tuning and identification demos run on the export;
  - the metric objective and the margin-floor example wait for S73, a v0.5 row.

### 1.2 The step algebra first-class

- [ ] **One owner per rule, its two public items** (S65; the rest landed in v0.4) **[ask]**,
  each a public change:
  - one set of Monte Carlo defaults for `evaluate` and the planners' evidence: `evaluate` takes
    `backend="auto"` and needs `n_trials` on a stochastic problem, the evidence runs
    `backend="numpy"`, `n_trials=50`; evidence on `"auto"` moves the stochastic scores by one
    ulp and adds a JAX compile to every `solve(evaluate=True)`, and `evaluate` defaulting to 50
    relaxes its documented refusal;
  - one empty-cache rule across the `compute_*` facades: `plot_rollout()` with no rollout raises,
    because `compute_rollout(n_steps)` has no default length to compute one with.

  Done when each is ruled and built, or dropped.

### 1.3 The draws

No open row: the contract is DESIGN §4 ("Randomness": the convention, the three periods,
the Kalman bridge, the solver hint) and §6 (the problem's laws and sources, `realize`,
`evaluate` on the draws); the discrete Kalman filter is an estimation follow-up (v0.9).

### 1.4 The seeded stochastic policy

No open row: the block (`GaussianPolicy`, `control/stochastic.py`) and the RL heads on it
landed in the v0.3 run; the contract is DESIGN §3.

### 1.5 The sampled loop: a step diagram, two runners

No open row: the sampled loop, its two runners and the legacy runtime's removal landed in the
v0.3 run (contract: DESIGN §3 and §5). The realtime runner on the pure blocks is v0.5 (§4).

### 1.6 One score

No open row: the contract is DESIGN §6 (the problem evaluator; the step problem's own equations
and its own tree, `realize` and the tick offset `t0`, landed in v0.4). Ruled besides it: the
sampled check accepted, the run priced to the horizon with the input held, a solver's own
objective allowed but the score always the problem's cost.

### 1.7 The parametric program; the stochastic program its seed case

Design: [native-optimizers.md](native-optimizers.md) §7.

- [ ] **The friction-aware MPC example from a port** (S76's rest; the program with parameters,
  its online binding of every leaf, the performance gate, the failure cost under overrides, the
  stochastic program as the seed case and the MPC's `inputs=` bindings landed in the v0.3 run):
  an MPC that plans with the latest friction estimate of a port,
  `ModelPredictiveController(planner, inputs={"x0": "y", "plant.mu": "mu_hat"})`. Done when the
  example runs as a demo and its plan follows a moved `mu_hat` without a recompile.
- [ ] **A demo: an MPC's hyperparameter tuned by minilink's native methods** (ruled 2026-11-07:
  an input to the maintainer's demo rebuild, §1.12, which decides whether the new set has it and
  on which planner and plant): a script that tunes an MPC (or MPPI) hyperparameter on a closed
  loop richer than the pendulum of `examples/demos/compile/native_mpc_tuning.py` (which tunes a
  native planner's cost weights by gradient), through the evaluator's program
  (`problem.compile(mpc % dt)`, `evaluator.program(wrt, bounds=, n_trials=)`) and minilink's own
  optimizers, no external solver needed. The first attempt (the MPPI temperature on the dynamic
  racecar, removed 2026-11-06) shows the points to design: the solver (the SLSQP fallback
  stopped on "Inequality constraints incompatible" and the tuned temperature scored worse than
  its start), the cost of each evaluation (a JAX MPPI lap took about 76 s of solve for 8
  iterations), a hyperparameter with a clear effect on the score, and a gradient through the
  loop or a derivative-free search, depending on the planner (MPPI's sampling). Options: the
  planner (native trajectory optimization or MPPI), the plant (racecar lap, cart-pole), the
  hyperparameter (temperature, horizon, weights). Done when the script runs headless in its
  time budget, the tuned score beats the start, and its README row says what it teaches.

### 1.8 The native proof of concept: the planner's pure face and an MPC on it

Design: [native-optimizers.md](native-optimizers.md); the face and the MPC's contract are
DESIGN §6. Better solvers are v0.5 (§4); MPPI runs on the face (its rest is S72).

**Plans as policies** (the design of what is left and the rulings that shape it:
[plans-as-policies.md](plans-as-policies.md)). Its first two changes landed in v0.4: the pure
cores, `GridField`, the cost-to-go of LQR and value iteration as fields, discounted LQR; value
iteration's finite horizon and the law past the horizon for both planners (contracts: DESIGN §4
"Fields" and §6). Left, the third change:

- [ ] **The faces and the MPC protocol** (`PP-7` to `PP-9`): done when the MPC and the
  follower read the state for a feedback face (`pv.feedback`), the open-loop loops
  byte-identical (`mpc_dual_rate.py`, `test_mpc.py`, `test_v03_core.py`, the MPC parity
  benchmark); `pv.solution(pv.solve(p)[0])` equals `planner.solve()` on the fields of the plan's
  §6 for LQR and value iteration, both modes, NumPy and JAX; `test_no_recompile.py` covers the
  two faces; LQR and value iteration close in the dual-rate loop (and the MPC loop where `z` fits
  a tick); DESIGN §6 carries the contract and the plan doc is deleted.
- **Later, the maintainer's own pass:** the GRO860 notebooks' wording on `solve_steps`
  (`grid_world_dynamic_programming.ipynb`, `double_integrator_minimum_time.ipynb` §4) when the
  course is next taught: the time-varying finite-horizon law is `solve()` on `horizon = N dt`;
  their cells and plots are unchanged (`PP-D17`).

### 1.9 Fields

No open row: `Field`, `QuadraticField` and the four conversions landed in the v0.3 run,
`GridField` and the cost-to-go of LQR and value iteration as fields in v0.4 (contract: DESIGN
§4, "Fields"). The rest of the field object (`CallableField`, `LinearApproximator`, tabular
RL's `Q`) is a row of §4; value iteration's policy evaluation returns a `GridField` (the API
naming sweep, §1.11).

### 1.10 Hardening and the release

- [ ] **The release** (every notebook on the v0.4 names, the course notebooks included):
  - the student-facing numbers re-pinned (notebook outputs, course figures, the pipeline figures
    of `docs/pipeline_figures.py`; the regression baselines moved with each change, §1's gates),
    after the maintainer checks the demos on `dev`;
  - the PR from `dev` to `main`;
  - the `0.4.0` tag.

### 1.11 The core API proposals

- [ ] **The API settled before the demo rebuild** **[ask]** (design and status:
  [api-stabilization.md](api-stabilization.md), the review of 2026-11-10): its defects section
  (AS-1 to AS-15) landed on `dev`; left, on the maintainer's go, the names a rule already
  decides (§2, AS-20 to AS-32), the additions that break nothing (§3, AS-40 to AS-49), the 24
  decisions one at a time (§4; the API naming sweep's judgment calls of 2026-11-08 are folded
  in there), and the planned rows pulled forward (§5: K8-0, K8-1, S64's and S32's names). Done
  when every row has landed or been dropped and the plan doc is deleted.

The proposals, the later round of API sweeps and the API naming sweep (PR #138) landed in
v0.4 as ruled (contracts: DESIGN §4 to §6, RULES 2.7). Left:

- [ ] **The parameters model's last follow-up** (the model's contract: DESIGN §4,
  "Parameters"): the problem's internal helpers (`parts_view`, `sample_params`,
  `nominal_plant_params`, `drawn_paths`) folded into its held tree; deferred in the rulings run
  (it spans twelve modules and moves no number). Done when it has landed or been dropped.

### 1.12 The minimal surface

Design: [minimal-surface.md](minimal-surface.md), what is left of it: the test layers' options
and a starting demo list, the inputs of the rebuild below. Landed in v0.4: the CI shape
(MS-D3), the short-term test restructure (the `slow` marker and `--runslow`), the notebooks
(MS-D5: the two duplicate topic notebooks gone) and the benchmarks (MS-D6: the regression gate
alone).

- [ ] **The maintainer's demo rebuild, then the tests' clean slate** (MS-D1, ruled 2026-11-07)
  **[maintainer]**, the last steps of v0.4: the maintainer deletes all demos and rebuilds a
  minimalist set (the demo set, MS-D4, is his); once the new demos are what he wants, the test
  infrastructure is rebuilt from scratch around the new library (the layers' details, MS-D2a–e,
  decided then). Agents update `examples/README.md`'s map and the demo checks after each of his
  changes. Decided with the new set:
  - a demo of tabular Q-learning on the pendulum grid beside value iteration, converging toward
    the DP cost-to-go (GRO860's RL row, ruled with K4) **[ask — student-facing]**;
  - the MPC hyperparameter-tuning demo (§1.7);
  - the cart-pole RL demo's budget **[ask]**: the training reward moved to `−g·dt`; measured
    2026-11-07 (PPO, 3 M steps, `dt = 0.05`, seeds 0–7, the demo's own
    `evaluate(n_trials=100, key=1)` and its 10 s run from hanging; success = upright
    `cos θ < −0.95` and `|cart| < 5 m` over the last 2 s): **1 seed of 8** swings up and
    balances (seed 2, `J = 1.23 ± 0.84`); the other seven swing up once and spin (eval `J` 21 to
    1756, median about 32), and the training's final episode cost (1.17–1.85) does not tell them
    apart. Options: a larger budget, another reward or exploration setting, or the demo shown as
    one seed's result. Also seen: `evaluate` reports every seed feasible and reached at
    `J ≈ 1756` (seed 5), to check against its `X` (`|θ| ≤ 20`);
  - the supported plants (K8-3) and GMC714's pinned names (G3), both in §2, drawn after it.

  Done when the new demo set and the new suite replace the old ones.

### 1.13 The root exports and the tutorials' quirks

No open row: the root exports (`QuadraticField` and `GridField` on the root, the core's
mathematical types on `minilink.core`), a linear model's matrices read as `lin.A()`, and the
four linear-analysis plots on continuous and step systems alike landed in v0.4.

---

## 2. v0.4 — the clean-up lane

Not on the core's path; it runs beside it. The course material stays on `main` until the new core is ready.

**Wave K — convergence** (K7's analysis is the ruling table K1 asks for). The goal: a smaller
library that is easier to read and keep — fewer names, fewer demos, prototypes labelled as
such. It extends D2 (whose open picks join K1's table) and runs before D1.3, so the demo sweep
touches fewer files. Nothing leaves without the maintainer's ruling (RULES 7.1). One demo per
feature (K4) became the maintainer's demo rebuild (§1.12).

- [ ] **The catalog clean-up** (K8): two import layers and support by
  test. Steps, one commit each: **K8-0** the car family to three models (widened by [vehicle-models.md](vehicle-models.md), its steps and their "done when" in [vehicle-models-run.md](vehicle-models-run.md); `KinematicBicycle`,
  `DynamicBicycle`, `UdeSRacecarDyn` on the root; `KinematicCar` and `DynamicBicycleCar3D` merged as
  skins, `UdeSRacecar` a parameter preset, `UdeSRacecarDyn3D` a `wheel_angles=True` option, the
  notebooks and demos migrated; in review on `vehicle-models-phase1`: the tire and terrain
  modules, `DrivetrainBicycle` with `UdeSRacecarDyn` its preset, `UdeSRacecarDyn3D` as
  `UdeSRacecarDyn(wheel_angles=True)`, `psi`, the wheelbase owned by `a` and `b`; **[ask]** the
  two skins, waiting on three questions, the brief's §6 and the pull request; **[ask]** the
  one port layout of the bicycle rungs, the plan's §9); **K8-1** the root prelude trimmed to the course set (the
  course plants, 16, no cap; all 50 catalog plants today; ledger line in ROADMAP §2);
  **K8-2** `minilink.catalog` the one flat index of every supported plant, each passing the
  both-backends contract test, with a docs entry; **K8-3** the unsupported plants moved to `minilink/experimental/catalog/` with their skins
  and demos (the top-level `experimental/`), their tests and docs entries removed, out of the
  wheel (ruled 2026-11-07, by rule after the maintainer's demo rebuild: a plant stays supported
  if his new demos, a course notebook or the tutorial uses it, the rest move, the list put to
  him for review; 24 of 50 are used today by their tests alone); **K8-4** twins merged and the dead helpers deleted
  (`camera.py`, `ProblemEnv` / `to_gymnasium`; the pygame renderer stays, the real-time
  default); **K8-6** the README's catalog line by domain and the API
  site's plant page. Done when the root prelude lists exactly the course plants, `minilink.catalog`
  lists only tested plants, no two supported plants differ by parameters alone (except the UdeS Racecar's thin preset classes, ruled 2026-11-11: vehicle-models.md), and
  `test_teaching_surface.py` pins both lists.
- [ ] **The kinematic car family declares its time constant** (v0.4, with K8-0): `KinematicBicycle` and `UdeSRacecar` get default input bounds and declare `τ = L / (v_max tan δ_max)`, so their loops leave the 1 ms default (1000 substeps a 0.1 s MPC period today). Done when the car family's automatic step reads its declared `τ` and the moved numbers sit in a ledger.
- [ ] **The experimental tier, untested** (K9). Landed: the prototype
  scripts moved from `examples/experimental/` to a top-level `experimental/` (K9-1); the tier's
  tests (`test_symbolic.py`, `test_engine_jax.py`) moved to `tests/experimental/`, off the merge
  gate, run by the nightly job (`pytest tests/experimental`), `c_export`'s flagship smoke kept in
  the JAX regression job (K9-2); the status line on every `minilink/experimental/` module
  docstring (K9-3); `manipulator_eom.ipynb` deleted (it imported the symbolic
  tier); `showcase_jax.ipynb` keeps its quick C-export demo, the one documented exception beside the
  flagship smoke (both on `minilink.experimental.c_export`). Left: the unsupported plants' test rows
  leave with K8-3. Done when CI's gate imports nothing under the tier but those two exceptions,
  and `pip wheel` ships nothing under it.
- [ ] **The rest of the catalog declares its time constant** (the automatic step's first scope landed:
  the linear blocks, the pendulum, the cart-pole, the dynamic bicycle, the racecar,
  `SlidingMode`): every other catalog plant (the arms and manipulators, the drones, the boats,
  the mass-spring-damper's nonlinear kin, the oscillators and the three-body set that declare a
  constant today) a `fastest_mode()` read from its `params`, from its linearization at the
  nominal point by a closed formula. The kinematic bicycle declares none (a zero spectrum): an input box would give it one. Done when a sweep of the catalog shows no
  plant on the 1 ms default but the undeclared custom ones.

**GMC714**: no new feature; the teaching surface covers every lab (ROADMAP
§4.3). The labs move from pyro to minilink as the maintainer's teaching work, outside this
workboard; the former wave G rows are §4 rows.

- [ ] **GMC714's names pinned** (G3; ruled 2026-11-07: pinned once, at the very end of v0.4,
  after the maintainer's demo rebuild; the agent drafts the list from the GMC714 labs, every
  name they import or call, for the maintainer's review). The moves and renames landed (A6's
  names, `RRTPlanner` in the teaching lane, `JointImpedance`, `TaskKinematic` and
  `TaskKinematicNullspace` on the root prelude, the `minilink.dynamics` band facade,
  `ModelPredictiveController(planner, *, inputs=, delay=False, warm_start=True, backend=,
  verbose=)` in a teaching-lane module, run as `(mpc % Δ) @ plant`). Left, the gate: every tool
  a lab uses has one public teaching name, frozen for the term, each with a docstring, a demo
  and the teaching-surface test (ROADMAP §2's soft rule). Done when `test_teaching_surface.py`
  pins the list, each name has a demo under `examples/demos/`, and no GMC714 lab needs a deep
  import.

- [ ] **The mechanical hierarchy and the car models' base, reviewed** **[maintainer]**
  (postponed 2026-11-07 to the end-of-v0.4 demo rebuild or later), then **one mechanical
  family** (S32) **[ask — core]** rebuilt on the ruling. The review: which mechanical class is
  the child of which (`MechanicalSystem`, `GeneralizedMechanicalSystem`, the manipulators, the
  body-frame vehicles), where the car family sits (`DynamicBicycle` and its children, the
  racecar, `BicycleDynRate`), and the one mass-matrix name. The proposal on the table:
  `GeneralizedMechanicalSystem` a child of `MechanicalSystem` beside `Manipulator`; `H` the one
  mass-matrix name (`M` renamed in the class, the boat, the plane, `DynamicBicycle.M` read by
  the racecar and `BicycleDynRate`, and `examples/projects/car_trajopt`'s ladder); `qv2x` /
  `x2qv` aligned; the model-based controllers accepting every mechanical plant; `DynamicBicycle`
  on the generalized child (scan: dynamics#5), although its named ports and its subclasses'
  extra states do not fit `[q; v]` (the reviewers' recommendation: the rename only, the base
  moved when K8 reworks the cars). Already ruled: computed torque on a body-frame plant is
  `v̇_des = Kp N(q)⁻¹ (q_d − q) + Kd (v_d − v)`, a non-square `N` refused; a model-based law
  refuses an underactuated plant at construction. Done when the maintainer rules, the boat and
  the plane pass the catalog contract test as children of `MechanicalSystem`, `ComputedTorque`
  runs on a fully actuated body-frame test body in a test, and no class names the mass matrix
  `M`.

---

## 3. Standing work — textbook code, minimal demos, consolidation, the plotting lane

Work that runs across the releases, behaviour-preserving where it touches code. Its core rows
(A5, S53, S65) moved to §1.

- [ ] **Naming, the rest** (A4) (any time, agent; the naming quick wins landed:
  the class name as the default `name`, shortcuts named as the expression they build, `id`
  documented): one colour keyword, `jmax`, across DP, tabular RL, the policy evaluation and
  `Comparison`, with S54's clipping (scan: planning#3, examples#5); the operators' named forms
  exported together (scan: core#9).

### Wave B — GRO501, the carry-over (0.2.0 shipped)

Release contract:
[ROADMAP §4.2](../dev/ROADMAP.md#42-v02--gro501-end-to-end).

**Order:** P10's remaining steps (AC-5 and AC-6, in the plotting lane), then the P4
follow-ups.

- [ ] **Automation by convention** (P10):
  standard port names drive the wiring, three reserved names (`plant`, `controller`,
  `observer`) are the roles every tool reads, and the cost is scored on the plant. Its
  steps AC-0 to AC-4, AC-7 and AC-8 landed (the roles, the resolver, the operators, the
  standard names, the loop inputs and `observer=`, the cost on the plant, the sampled loop).
  Left, in the plotting lane:
  - **AC-5, the plots' rules:** `signals="plant"`, `"controller"` or `"observer"` selects
    every port of that block, as `"plant:y"` selects one; the default auto mode shows every
    port of the plant and the controller, each wire once (the reference, the error, `d` and
    `n` come in through their inputs); signal names take paths (`plant/plant:y`), and a block
    inside a box is found by object too; labels and units are copied onto edge ports, colours
    follow roles; with no roles the plain default, a leaf keeping `("x", "u")`;
    `compute_trajectory(show=True)` and the planning and comparison plots use the same
    default. Open while building: how the auto mode drops duplicate wires, and whether a
    many-output plant (the racecar) needs a name filter.
  - **AC-6, the animation's rules:** resolved when animating; the camera comes from the one
    drawable block anywhere in the tree, else the drawable block inside `plant`, else an
    auto-fit view, and `scene_grid` and the follow frame from the same block; a compensator
    draws no skin inside a loop; the camera copied at composition time and `camera_priority`
    (never read) go.

  Done when `controller @ plant`, the same loop wired by hand with the role names, and
  `closed_loop` with sources give the same plots and camera, and a census of the loops (ids,
  edge ports with labels and units, roles) is byte-identical apart from the intended change.

  Related: S70 turns P10's refusals into `WiringError`. Also, later: `sensitivity(T)` reading
  the roles of one loop diagram instead of `plant=, controller=`.
- [ ] **follow-ups** (P4) (design: [estimation-and-state-feedback.md](estimation-and-state-feedback.md)):
  named input ports on the state-space base so observers and sensitivity blocks share one
  `f` (scan: analysis#13); the review of the `observer` role against the real blocks when
  the EKF lands (what else a filter gives, a covariance or an innovation; how plots compare
  the estimate with the plant's state; how Monte Carlo starts x̂0).

### Wave D — textbook code, minimal demos, consolidation

**Order:** the S69 follow-up now; the T rows' leftovers as picked; D1.1, D1.2, then D1.3
file by file; D2, D3, S67 and S68 as picked.

**T — the textbook pass.** Landed (PR #125, PR #126): the textbook layout of TB-b and T1–T7,
behaviour- and backend-preserving; its style sheet lives in RULES §5 (5.3's six blocks, 5.8,
5.10, 5.22, 5.25–5.27) and §7.8. The rows below hold what neither PR touched, each a
behaviour, backend or API item, rechecked open on `dev` on 2026-11-08; the recipe and the
reference are AGENTS'.

- [ ] **dynamics catalog** (T3): `manipulators/arms.py` (`from_manipulator` sets `_kin_arm`,
  `name`, `camera_scale` and `camera_target` on the instance from outside `__init__`;
  `link_lengths` probes with `hasattr`), `mass_spring_damper/linear.py` (`A` / `B` / `C` / `D`
  on `np`, so they do not trace — a JAX-safety fix; extend the both-backends test),
  `aerial/plane.py` (the 55-line pygame `__main__`, a demo in disguise, RULES 5.18).
  Also: `Manipulator` kinematics defaults (`forward_kinematic_effector`, `J`) return zeros
  instead of raising, with `link_lengths` the base contract (scan: dynamics#7).
- [ ] **analysis** (T4): `analysis/lyapunov.py`'s `EVEN` / `ODD` slices sit under Internal
  machinery, apart from `METHODS` at the top.
- [ ] **planning** (T5): `direct_collocation.py` / `shooting.py` / `multiple_shooting.py`
  read `self.options.*` inside their bodies.

**D1 — demos to the rule** (RULES 6.1 / 6.10 / 6.11; the census is comment FD-C14 of the
2026-09-15 foundations review, in git history). The demo rows apply to the demos the
maintainer's rebuild keeps (§1.12); the notebooks' are unchanged.

- [ ] **Native reporting the demos hand-roll** (D1.1) (agent lane, plotting): an `Optimizer`
  convergence plot (`optim_plot.py`); two planners' trees or solutions side by side
  (`rrt_holonomic_obstacles.py`); a `rollout_batch` family plot (`rollout_param_family.py`);
  a 3-D phase plot (`lorenz_attractor.py`). Each one `plot_*` or `__str__` with a test;
  DESIGN §7 lists them.
  Also: `print(controller)` shows the gains (scan: control#10);
  several trajectories on one phase plane and two fields on one grid surface (scan:
  examples#6). Also, planned with the evaluation record and not built: the
  `Evaluation` record's `plot_cost` / `plot_trajectories`, `Comparison.plot_cost`, and the
  per-trial `keys` for a replay.
- [ ] **The flatness ratchet** (D1.2) in `test_teaching_imports.py`: no top-level `def` or
  utility class in `examples/demos/`, no notebook cell mixing an API call with matplotlib
  code; today's offenders allowlisted, the list shrinking with D1.3.
  Also: teaching code stops rebinding `sys` (scan: examples#7); one owner for the
  teaching-facade list the two teaching tests share (scan: tests-ci#7).
- [ ] **The sweep** (D1.3), one file per step, maintainer reviews each diff **[ask per file]**:
  the showcases (`showcase_jax.ipynb` first;
  `showcase_jax` rewritten, awaiting review), then the non-clean demos, then the teaching
  notebooks. Done when
  the D1.2 allowlist is empty and the notebook checks pass.
  Also: the placeholder sections of tutorials 01, 02, 03, 05, 08 and 09 filled (scan:
  examples#3); the facade gaps the import allowlist records closed (scan: examples#13).
- [ ] **The planning demos and notebooks on the solution verbs** (D1.4) **[ask]**: the
  hand-rolled comparisons and the per-file `@dataclass` rows become `solution.plot_*` and
  `compare(...)` (a runnable draft of `vi_pendulum_lqr.py` exists).
  Also: the Simulator under RULES 2.4, which keeps `last_traj` and `last_solve_time_s`
  besides the `sys.traj` shortcut (the maintainer held it, 2026-11-04); its `last_debug`
  stays a diagnostic.

**D2 — consolidation picks** still open from
the 2026-09-05 consolidation inventory, in git history (nothing removed without the maintainer's
pick):

- [ ] `Source.plot_signal` (`show_signal` before the API naming sweep; 80 lines of bespoke matplotlib; the same picture is
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

**D3 — hardening rows** (research lane, small):

- [ ] RRT checks `X.contains(sample)` and the goal region without the time: a time-varying `X` or
  `Xf` is read at `t = 0` (`rrt.py`, `rrt_star.py`); the tree nodes need a time (the sum of the
  edge durations) and the checks pass it.
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
  plant diagram drops a diagram-level function output port; an explicit `backend='jax'` on a
  NumPy-only set or cost raises a raw trace error; a raising params setter can leave a diagram
  half-updated;
  assigning into `q.labels[i]` in place no longer sticks; the gravity hook's signature is
  read on every call; the collocation demo's SLSQP fallback is not run in CI (its manifest
  entry still requires Ipopt).

**S67–S69 — one plot vocabulary, contracts as tests, students' laptops** (S65 and S66 moved to
§1):

- [ ] **The compile-time flatten default**: `compile_diagram(..., flatten=False)` inlines nested
  boxes with `__` ids; the parity tests compare the nested compile, `flatten=True` and
  hand-flat wiring (the timing scenario, `benchmarks/scenarios/nested_compile_flatten.py`, left
  with the studies in v0.4: in history at `b5b9892`). Run it once on the
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

## 4. v0.5 — features on the core

v0.5 adds features built on the finished core, not
intertwined with it; the rows are independent and run in any order. A row that reads a core
pillar names it.

- [ ] **The realtime runner on the pure blocks**: game mode and hardware in
  the loop re-based on the sampled loop's blocks (`update`, the port functions, `step`), pacing the
  ticks on the wall clock and reading the keyboard or the device between them; the step diagram its
  test oracle at the ticks; the GRO501 game demos re-pinned once. Reads core §5. Done when every
  realtime demo runs the same blocks the judge scores and matches the scan at the ticks.

- [ ] **Identification and generation** (C4) (its design states which fitting route it takes and
  flags it for the behavior-cloning decision): identification is V1's program export
  (`pe.program(wrt=["plant.…"])` and the `Optimizer`) with a data-mismatch cost (`identification/fitting.py` on `rollout_batch`; physical params and NN
  weights are one verb), its first data consumer; `trajectory_generation/` port (polynomial,
  min-snap); SMC trajectory-following demo; trajectory post-filter; experiment-design
  helpers (the PRBS / chirp inputs live in `blocks/sources`).
  Also: the simulated `Trajectory` logs `dx` and `y`, so an equation-error fit has its data
  (scan: analysis#14).
- [ ] **LQGPlanner** **[ask — planner API]**: `LQGPlanner(problem)` on a
  `PlanningProblem` stochastic by its parts, with a `QuadraticCost` and noise source blocks
  on `d` / `n`, beside
  `LQRPlanner`: the controller is the `lqg` box, the record holds `K`, `L`, both Riccati matrices
  and the poles, the cost-to-go the certainty-equivalent form; `Q_n = B_w W B_wᵀ`,
  `R_n = D_v V D_vᵀ` from `linearize_matrices(sys, wrt="d")` / `wrt="n"` and the blocks'
  `psd`. With RN-4. Design: [estimation-and-state-feedback.md](estimation-and-state-feedback.md) §5.

- [ ] **The rest of the field object** **[ask — core]** (A1; `GridField` and the cost-to-go
  of LQR and value iteration as fields landed in v0.4): tabular RL's `Q` as a `GridField` over
  the joint levels of `(x, u)`, with `min_over_u()` the `J` field and `greedy()` the
  `LookupPolicy` (ruled 2026-11-06, [plans-as-policies.md](plans-as-policies.md) §7), and
  tabular RL's `cost_to_go` on it; `CallableField` (RL critics); `LinearApproximator`
  as a field; `LyapunovCertificate.contains`, `slice_extent`, `V_dot` and `search_level` on the
  field (its `V`, `region`, `extent` and `verify`'s draw landed in v0.3); `lqr_finite_horizon`
  handing out `QuadraticField(x̄, S, t)` beside its controller (scan: planning#4; the policy
  evaluation's `GridField` landed with the API naming sweep, §1.11). Done when the DP
  baselines `cmp` and `test_planning_solution.py`, `test_rl_tabular.py` and
  `test_rl_planner.py` pass.

**Workspace geometry.** Design: [geometry-module.md](geometry-module.md). Baselines first, as
each plan states.

- [ ] **Workspace geometry** (A3) **[ask — core]**: the rest of geometry-module.md's steps.
  Landed: the `core/geometry/` package with the shapes and the paths (a polyline open or
  `closed`; the path names left `planning.spatial`, ruled 2026-11-07), and the path-tracking
  laws reading a path. Left: the track, the scene, the probes, `mount`, the spatial fields, the
  grid, and the catalog (`oval_circuit`, `racecar_circuit`, `holonomic_forest`); the plots and
  overlays under `graphical/` with lazy methods on `Scene` / `Track`; `planning.spatial`
  deleted in the same change as its call sites; the path-tracking laws take a `Track` once it
  moves. Done when the two UdeS racecar demos and the RRT forest demo
  use the catalog, `control` imports geometry with no planning import, and the suite is
  green. Glyph rename stays S30.
  Also: `Shape` unions flatten like `IntersectionSet` and `SumCost` (scan: core#6);
  the path-tracking laws size their measurement from the vehicle, not `state_dim=9` (scan: control#9);
  retarget `planning/trajectory_optimization/mpc/viz.py`'s `TrackCorridorOverlay` import in the change that deletes
  `planning.spatial.overlays` (scan: control#12).

**On the spine.**

- [ ] **The path-integral planner (MPPI), the rest** (S72) (contract: DESIGN §6, "Path integral control"). Landed:
  `MPPIPlanner` / `MPPIRecord` on the `minilink.planning` facade
  (`planning/trajectory_optimization/path_integral.py`), its compiled face on
  `RolloutEnvironment` (moved to `planning/environment.py`), the `method` and `seed`
  leaves, the MPC in its three forms on it, the seeded-determinism, LQR-direction and
  swing-up tests (`test_mppi.py`), the demos `examples/demos/mpc/mppi_pendulum.py` and
  `mppi_racecar.py`. Rest: MP-3 the stochastic branch: each control sample scored over
  `M = n_plant_samples` plant draws (the tree's laws and the sources' seeds, by path from one
  key; the start stays the bound `x0`), its score the mean of the `M` for `"expectation"`
  or their CVaR for `"worst_case"` (`problem.criterion`), one `vmap` over the `K × M` axis;
  `M = 1` by default, the docstring stating its bias (the softmin rewards a lucky plant
  draw as much as a good input); a deterministic problem ignores `M`, byte-identical. Today a
  stochastic problem plans on its nominal with the deterministic planner's warning; the fan of an offline solve as
  `solution.plot_samples()` (the closed loop's fans landed: `mpc_sampled_futures`,
  `mpc_animation_overlays(samples=)`, `plot_sampled_futures`); MP-6 (the `loop` backend,
  DESIGN §6) with V3.
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
- [ ] **The Gymnasium view on the step problem's equations** (from the v0.3 run): `Sys2Gym`
  steps the step problem's own equations (`transition`, its failure rule and cost bound) in
  place of its own compiled step, its reward the RL environment's. Moves the GRO860 Gymnasium notebooks' numbers
  once **[ask — student-facing]**. Done when one policy's return from `Sys2Gym` and from the RL
  environment agree.
- [ ] **General serial chain** (C6) **[ask — core]**: one mechanism description (DH table or
  URDF: geometry, inertia, topology) feeding RNEA / ABA beyond the UR5, the shared serial-chain
  stack, ABA on the other RNEA arms, the symbolic path reading the same description; the GMC714
  arm topic's modelling rung. Design first (the July 2026 articulated-mechanism draft is in git
  history; it is re-planned here, no separate `KinematicModel` delegate: its one mechanism description is the robot's home for its
  frames, feeding the dynamics, the kinematics and the drawing's poses, a preliminary
  choice). The GMC714 audit found the labs need no
  second arm description (`ThreeLinkManipulator` and the UR5 cover them). Done when a second arm runs on the shared
  stack with the catalog both-backends test.
- **The former wave G rows** (moved from v0.3; none is needed by GMC714):
  - [ ] **Pyro parity** (C1) open rows — [pyro-port-remaining.md](pyro-port-remaining.md); then
    the pyro → minilink migration guide in README from the name map there. **[ask]** for the
    README. Re-audit the table against the code first (scan: docs-gov#11). Which pyro demos the
    courses still need is the maintainer's pick (of pyro's 195 example scripts: 29 done, 47
    partial, 83 to do, 36 dropped).
  - [ ] **Modelling ladder** (C2): manipulators + the four-rung vehicle ladder as a
    tutorial lesson (not a GMC714 lab); robotic PID wrappers (`JointPD`, `EndEffectorPD` twins).
    Also: one constructor contract for the robotic laws (scan: control#6).
  - [ ] **Blocks** (C3): `Ramp` / `Chirp` / `Delay` / `Switch`; `Integrator` and `UnitDelay` take
    a `dim` like every static block (scan: graphics#11).
  - [ ] **Catalog hygiene** (S64) **[ask — catalog]** (scan: dynamics#0, dynamics#1,
    dynamics#2, dynamics#3, dynamics#4, dynamics#6, dynamics#8; the mechanical default
    bounds landed, each plant states its own): output ports that are
    the state or a slice of it read labels and units from the state (the 56 copy lines go);
    one port layout across the bicycle rungs; one owner for the wheelbase; the hidden `0.01`
    damping in `Drone2D.d` and `Rocket.d` named; `VanderPol` with a real input or none;
    constructor hygiene in the pendulum and mass-spring-damper families.
- [ ] **Traceable linear analysis** (S73) **[idea to review — ask — core]**, proposed by the
  maintainer, not scheduled: `poles`, `frequency_response` and `margins` in
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
  between crossovers; JAX's nonsymmetric eigensolver is CPU-only. Recommendation from an
  architectural review: alongside crossover `margins`, expose the maximum
  sensitivity peaks $M_s = \max_\omega |S(j\omega)|$ and $M_t = \max_\omega |T(j\omega)|$
  (and their smooth `logsumexp` surrogates over $\omega$) as traceable metrics on
  `sensitivity` / `complementary_sensitivity`, providing a smooth, crossover-free robustness
  bound for NLP gain tuning. Independent of V1 (needs
  only the jacobian), self-contained for a cloud agent. Done when `jax.grad` of the phase
  margin and of each closed-loop pole of `PID() @ Pendulum()` with respect to a gain and to
  the mass matches a central difference, and the NumPy callers see no change.
- [ ] **Native solvers beyond the proof of concept** **[ask — core]** (the proof of concept,
  NS-1, NS-2's fixed-`K` form, NS-8 and NS-9, landed in v0.3): native solvers as pure laws
  beside the wrapped ones, program categories, fully traced JAX solves (NS-2's tolerance form,
  NS-3 implicit differentiation, NS-4 least squares, NS-6 the augmented Lagrangian, NS-7 the
  stochastic category); long term, convergence analysed as stability (NS-5's lesson on
  `optimizer.as_step_system()`) — [native-optimizers.md](native-optimizers.md) (its shape a
  choice; v0.5).

---

## 5. v0.9 freeze candidate and v1.0

ROADMAP §5.5 and §5.6: v1.0 means a colleague builds a course on minilink and trusts it for
three years. The foundation rows are design conversations before code.

**Order:** the foundations (S27, S44, S30, V3) before the trust contract; V11 once, between
terms; the v1.0 rows after two cohorts.

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
  only if a benchmark asks for it, an optional extra (`solver="diffrax"`), never required (a choice)
  (`core/compile/evaluators/stepping.py`). S37 (evaluators keep pure maps and one scannable step; the integrators'
  move; the evaluator internals deduplicated: ZOH sugar, the JAX step rollout, the four
  Jacobian probes, the undeclared batch cache; scan: compile-sim#9) is absorbed by S75's SM-1
  and SM-6.
- [ ] **S44** A single posed-geometry hook so "two functions" (`f` + drawing) is literal;
  today `tf` + skin. Designed with C6's mechanism description, so a robot's drawing reads the
  same frames.
- [ ] **S30** Rename the graphical `Sphere` / `Box` glyphs to `SphereGlyph` / `BoxGlyph`, so no
  two importable public types share a name with the geometry solids. **[ask — public
  names]**
- [ ] **The frozen surface** (V3) **[ask — public names]**: list the names the v1.0 freeze
  covers, drawn from one rule (a choice): a name joins the freeze
  when it has its teaching demo, its test, and has gone through one release unchanged. `StepSystem`, `StepDiagramSystem`, `UnitDelay` (blocks facade), `ModelPredictiveController`
  (control facade) and the spatial names A3 moves to `core/geometry/` (provisional teaching
  names, ROADMAP §2) join it or move behind a visibly provisional name. Done when the
  teaching-surface registry (`tests/unittest/test_teaching_surface.py`) marks each name
  frozen or provisional and the API site shows the mark. With it, the v0.9 name pass: the
  compiled-model classes renamed `Compiled…` (deferred by v0.3's names sweep).

### v0.9 — the trust contract

- [ ] **Deprecation policy** (V4) **[ask]** (open: the shim below was not adopted; no alias meanwhile). The proposal was: from v1.0, a teaching-surface removal or rename
  ships one minor release as a `DeprecationWarning` shim (RULES "no deprecated aliases" is
  amended for v1.0 on). A snapshot of the export list is a test; changing it fails until the
  snapshot and the CHANGELOG line move together. Done when RULES, ROADMAP §2 and the
  snapshot test agree.
  The pending renames landed before the freeze (the vocabulary sweeps, 2026-11-04); the policy is for
  the names changed after v1.0, and is open (QUESTIONS).
- [ ] **`CHANGELOG.md`** (V5): user-facing, one entry per release from v0.9 (back-filled for
  0.1.0 / 0.1.1 from ROADMAP §2's breaking changes); `pyproject.toml` `Changelog` URL points
  there. Done when `publish.yml` refuses a tag without its CHANGELOG heading.
- [ ] **The pinned Colab cell** (V6) **[ask — student-facing]**: conda from `environment.yml`
  is the recommended install and README leads with it; `pip install minilink` is the light
  alternative, one `full` extra beside it (one install story). What remains: the Colab setup cell
  clones a pinned release (`git clone --depth 1 -b vX.Y`, the notebooks' relative
  imports of the examples tree keep working) instead of `git clone main`;
  `COLAB_CLONE` in `tests/unittest/test_repo_contract.py` follows. `showcase_jax.ipynb`
  imports `minilink.experimental.c_export`, which the wheel does not ship: settle it here.
  The "pin the course notebooks on `0.2.0` now" option of
  [repository-structure.md](repository-structure.md) §4 waits here (course
  sync is secondary to the order of the waves; the course notebooks keep tracking `main` and
  migrating in the same commit until then).
  Done when every notebook carries the pinned cell and runs in Colab.
- [ ] **`MinilinkError` family** (S70): `WiringError(ValueError)`,
  `ShapeError(ValueError)`, `SolverError(RuntimeError)`, subclassing what is raised today (no caller breaks);
  S66's wiring-time checks and P10's refusals raise them. Done when the wiring and compile
  paths of `core/` raise only the family.
- [ ] **Rewrite the history without notebook outputs** (V11) **[ask — force-push]**, after
  the v0.3 tag, between terms. A fresh clone is 176 MB for 19 MB of
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
  the sampled filters (unscented, discrete Kalman, particle) on the sampled loop; the
  `EstimationProblem` / `EstimationSolution` and the `*Estimator` tools (§5 of the plan);
  online parameter estimators (recursive least squares, gradient laws) under a role of their
  own. Each revisits the observer conventions of
  the P4 row (§3).
- [ ] **The classic first plants** (S71): DC motor, tank / thermal first-order-plus-delay
  process, ball and beam, differential drive, 3-D quadrotor, each with `tf`, a skin and the
  catalog both-backends contract test; LQI beside the `N` matrix of P8. Done when each has a
  one-line demo in `examples/demos/dynamics/`.
- C4 (identification, trajectory generation) is a v0.5 row (§4), as V1's program export with
  a data cost; it moves here only if v0.5 does not land it.

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
- [ ] **Course-neutral labs** (V9) **[maintainer]**: `examples/teaching/topics/` grows from 5
  to 10–15 labs (objectives, prerequisites, time, starter and solution), classical control
  and robotics first; a "For instructors" page (pin a version, Colab, adapt a lab, report a
  bug).
- [ ] **The public face** (V10) **[ask]**: the README opens on a teaching quickstart; TRL,
  lanes, wave and step ids leave README and `examples/README.md`; `CONTRIBUTING.md`, issue
  templates, a code of conduct (the governance docs live under `docs/dev/`).
- [ ] **V2** Zenodo archive and a citable DOI (`CITATION.cff`), now (the maintainer links
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
  demonstrator. Its first scenes are realtime games written for it (pyro's are not ported).
  How fitting enters the API is decided here (a choice): the
  program export with a data-mismatch cost (a dataset as `Trajectory`s, no new verb), or a `fit`
  verb beside `solve` with a dataset noun. Why it is a choice: the differentiable path is built
  on a rollout cost, and behavior cloning's loss (like equation-error identification's) is
  supervised on recorded states, with no rollout; the dataset is a batch of `Trajectory`s with
  padding masks (the scoring kernel's alive mask almost provides it), recorded at the step
  model's rate with its input hold, and logging `y` (real demonstrations observe outputs).
  Identification (C4) flags its route for it. A first demo free of hardware: distil MPC or MPPI
  into a neural policy, a library controller as the teacher (a pointer).

---

## 6. Later ideas

One line each; open a plan doc only when a design needs a writeup.

- **[ask]** From the 2026-10-20 convergence analysis, not ruled when K7 closed: the
  racecar 3-D skin, `meshes.py` and `assets/` to the racecar plant folder (between terms; a
  GRO501 notebook imports it by path); `Sys2Gym` / `SB3Controller` to the experimental tier
  when the SB3 notebooks retire (S49); `planning/search/`'s `dubins.py`, `steering.py`,
  `live_plot.py` and `plotting.py` cut to what the two RRT demos use; the facade verbs no
  pillar needs (`get_diagram`, `plot_input_output_map`, `render`) decided before V3.
- A `ClippedCost`, when a saturating cost is needed (planned with the cost).
- A tutorial on gradient descent and the `Optimizer`, once the generic optimizer and the
  mathematical-program system are extended; until then the closing PID example teaches it.
- **A plant that owns its noise ports, as a real example**: a disturbance the loop cannot
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
  (`closed_loop`'s `disturbance=` and `noise=` add summing junctions today, so the cost sees
  `u + d` as the plant's command, a known limit).
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
  S72) would be that consumer; the two decisions go together.
- Diffusion policy (research lane only, JAX-only, after behavior cloning): a sampled
  controller with memory (the observation history, the current action chunk) and a seeded
  draw, replanning slower than it acts: the MPPI shape on the sampled loop; trained by the
  fitting route behavior cloning chose, with a denoising loss. Scope line: minilink owns the
  loop, the data path, the training bridge and the evaluation; an external JAX network library
  owns the architecture (no deep-learning library grows here).
- `MjxPlant` (`interfaces/mjx.py`) and `torch` / `flax` model wrappers in `interfaces/`;
  Pacejka tire; stochastic forcing; ROS2 / FMI; sparse long-horizon trajopt; RRT-Connect.
- Ipopt given the Lagrangian Hessian, not the objective's alone (scan: planning#13);
  `optimizer_method="auto"` picking Ipopt when installed (scan: examples#11).
- Colored noise, as a demo and its twin test: white noise of intensity `W` through the
  shaping filter `τ ż = −z + w`, its state started from `N(0, W / 2τ)` so it is stationary
  from `t = 0`, the twin test checking the variance `W / 2τ` (`W = 2 τ σ²`); with it
  `LowPassFilter(tau=)`, the filter parameterized by its time constant beside `cutoff_hz`.
- A training run that rebinds its numbers: the RL planner's `params` is a report
  today, with no `solve(params=)` (DESIGN §6); a run that trains on a moved plant or reward, if research needs
  one.
- A 3-D still of any animation: `save_frame(t)` on the meshcat renderer writes the static
  HTML of one frame. The headless screenshot side exists: `docs/asset_export.py::html_to_gif`
  seeks an exported page frame by frame, so a still is one `seek` and one screenshot. With it,
  two camera notes for S43: the default meshcat eye sits low so the viewer's grid reads as
  horizon lines, and an overlay (a track corridor) drives the auto-fit camera, so a follow
  camera cannot zoom on the car; overlays should opt out of the fit as the ground lines
  already do.
- Declined (do not re-propose): scalar / list signal bounds and a coercing `x0`; scalar `Q` /
  `R` / `S` in `QuadraticCost.from_system`.
