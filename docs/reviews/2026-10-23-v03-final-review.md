# The v0.3 plan: a final independent review before code (2026-10-23)

**Audience:** Prof. Alexandre Girard (Maintainer)
**Context:** every v0.3 decision is ruled (R1–R77,
[the rulings record](2026-10-22-rulings-session-1.md)). Before the first line of code, a fresh
reviewer read CONSTITUTION, RULES, ROADMAP, TODO §4, continuous-core.md, the rulings record and
[the earlier second opinion](2026-10-23-v03-alignment-and-second-opinion.md), then checked the
plan's statements of fact against the code on `dev-v0.3` at `e9f69dff` (three read-only passes:
the step model, the path to the cost, the clean-up and the names; the key claims re-checked by
hand). Nothing here is ruled: §1 needs the maintainer, §2 and §3 are agent-managed fixes.

**Verdict.** The architecture holds and the rulings are coherent with each other; the earlier
second opinion's four scoring defects are all closed by R31–R33 and R72. What remains is of a
different kind: the *order* the documents give is contradictory, a handful of gates cannot be met
as written, and the plan's picture of today's code is wrong in about thirty places (most of them
small, a few that change a row's size or its gate). None of it reopens a ruling.

---

## 1. For the maintainer: seven points to settle before code

### 1.1 The order: names first or step model first

- R73 rules the names and structure first (A6, K2, K8, K10, S32, G3, R68), then the step model.
- TODO §4 "Start here" and ROADMAP §5.3 "Next" still say the first code is SM-0.
- TODO §1 and ROADMAP §5.2 put A6 "between terms" (late December); G3 must land "by
  mid-December" yet its names are pinned *after* A6.
- Inside the names phase the code imposes an order: K2 and K8 before A6 (A6's 68 display names
  include classes K2 and K8 delete, so those files would be edited twice), K10 before G3 (G3 moves
  `ModelPredictiveController` to a teaching module and K10 moves `control/mpc/` into
  `planning/trajectory_optimization/`: one file moved twice otherwise), A6 before G3.

*Recommended:* R73's order, made concrete: K2 → K3 → K8 → K10 → A6 → S32 → G3 → R68. The new
`dev-v0.3` branch removes the reason A6 waited for the break: nothing reaches the running course
until the branch merges, so A6 can land in November. The step model's SM-0 and SM-1 touch only
`core/compile/` and may run beside the names phase.

### 1.2 The DP and RL numbers move in V1.2

V1.2's gate says "DP and RL baselines byte-identical", but three of its own changes move them:

- DP's price today is `out_of_bound_cost = 1e6` (`planning/policy_synthesis/dp.py`), tabular RL
  reuses it; RL's is a Halton-sampled bound summed over steps (`reinforcement_learning/environment.py`).
  R32/R71 make both read `problem.cost_bound()` (`g_max · tf + h_max`, exact on a box): new numbers.
- DP and RL never see an unbounded `X` today: both fall back to `sys.state.box`
  (`discretizer.py`, `environment.py`). "Refuse an unbounded set" breaks every DP and RL lesson
  that relies on that fallback, not only the ones missing `U`.
- `X` is already unbounded by default (`planning/problems.py`); the real default change is `U`
  (today the box over *all* input ports, not the command port).

*Recommended:* name these as V1.2's intended changes (one baseline, one re-pin, one ledger line
each, in their own commit), or rule that DP keeps its `1e6` and RL its bound until a later row.

### 1.3 Where `ProblemParameters` goes, and when A2 lands

`ProblemParameters` has 46 readers: 14 read `params.cost`, 13 `params.system`, 19 `params.sets`.
A2 moves the cost's; nothing in v0.3 gives the sets a home (sets read `params` with A5, and
continuous-core.md says `pe.params["sets"]` "once A5 lands"), yet K2, V1.2 and A2 each say the
bundle retires ("here or with A2").

*Recommended:* (a) one owner: the bundle retires at the end of A2, not in K2 or V1.2; (b) A5's
"sets read `params`" lands with A2, as TODO §1 already says; (c) **A2 moves before V1.4**: it
needs nothing from the engine, and landing it first lets `problem.compile` assemble `pe.params`
from the objects on day one instead of reading the bundle and being rewritten in V1.6. A2's
gate line on `pe.evaluate` moves to V1.4.

### 1.4 "Byte-identical" on JAX when the traced body changes

RULES 7.7 and the refactor recipe ask for byte-identical baselines. On NumPy that is achievable.
On JAX, SM-1 (every integrator a binding over one body), SM-5 (shooting's three paths onto one
`rollout`) and V1.3 (the scan returning signals) change the jaxpr itself; XLA may fuse and
reorder differently, and an ulp-level drift is then not a bug. An agent held to `cmp` will burn
hours chasing it.

*Recommended:* byte-identical on NumPy and wherever the jaxpr is unchanged; on JAX, where a step
restructures the traced body, agreement to `rtol 1e-13` (stated per entry in the step's report).

### 1.5 Shims: one policy

Classes are renamed with no alias (A6, RULES 3.4), but `integrator=`, `plant_dt_inner` and DESIGN
§5's evaluator names get one-release shims (R42, SM-4, SM-6). On a branch where every caller
migrates in the same commit and nothing is released in between, the shims only add code to delete
later. *Recommended:* no shim in v0.3 except DESIGN §5's published evaluator names (the one surface
a user outside the repo may call); `integrator=` and `plant_dt_inner` go in one step with a ledger
line.

### 1.6 Decisions no row covers

Each needs one line; the recommendation is in parentheses.

1. `infeasible_cost` accepts a callable today (DP reads it, a test pins it). The ruled values are
   `None | True | number` (drop the callable, a ledger line).
2. `("cvar", α)` does not exist today (`CRITERIA = ("expectation", "worst_case")`): new work with
   no row (part of V1.4b).
3. `PlanningProblem.X0` and `.metadata` are absent from continuous-core.md §3.4 (`X0` stays as the
   start set; `metadata` is ROADMAP §6's open item, untouched in v0.3).
4. `Comparison.evaluate(evaluator)`, used by two notebooks, has no fate in the plan (removed with
   `MonteCarloEvaluator` in V1.4b; the notebooks call `compare(..., n_trials, key)`).
5. `discretize(params=)` disappears from the new signature with no replacement stated (the
   `StepSystem` takes `params` per call; the keyword goes).
6. SM-7 removes the `euler` forced backend, but `select_solver` returns `"euler"` for
   `discontinuous_behavior` systems and it is the only solver taking a non-uniform grid;
   `euler_fixedsteps` is not addressed (keep `euler` as the discontinuous and non-uniform
   solver, on the one rollout; `euler_fixedsteps` becomes `solver="euler"`).
7. The step evaluators already have `rollout(x0, *, n_steps, u)` and `rollout_p`, which clash with
   the planned `rollout(x0, u_seq, t0, dt, params)`; `outputs` becomes `h` though DESIGN §5 lists
   `outputs` as stable (the new signature wins; `outputs` gets the DESIGN §5 alias of 1.5).
8. The RRT extender keeps every intermediate state as an edge knot and holds `t` fixed; the
   Lyapunov sampler holds `t` too. On a step that advances `t` they stay byte-identical only for
   time-invariant plants (the extender moves onto `rollout`, not `step`; both pass `t` honestly,
   an intended change for time-varying plants only).

### 1.7 The new branch

R47 says "v0.3 stays on `dev`". The maintainer opened `dev-v0.3` (2026-10-23) so `dev` stays the
line the running course's fixes ride. *To confirm:* `dev` takes fixes only; `dev` merges into
`dev-v0.3` (never the reverse) until v0.3 is done; `dev-v0.3` joins the long-lived branches
AGENTS.md protects. CI does not run on it today (`test.yml` triggers on `main`, `dev`, `dev-alex`).

---

## 2. Rows whose gate cannot be met as written (agent fixes)

- **K10** "done when the tree matches R66", but `spatial/` leaves with A3 (v0.4): the v0.3 gate
  excludes `spatial/`.
- **K3b** gates on "the inventory's script", which lived in a previous session's scratchpad and
  is gone: the script lands in the repo first.
- **V1.4** wants the autotuning gradient to match "its hand scan" to `1e-9`; the demo's loss is a
  rectangle-rule mean over 1000 steps at LR 1 (the plan says 400) with no `QuadraticCost`: the
  reference is rewritten on the trapezoid first, or the gate compares against a central difference.
- **K8-1** "16 today": the root re-exports all 50 catalog plants today; 16 is R60's target.
- **S32** sits in TODO §5 (v0.4) though R62/R73 put it in v0.3, first; its scope misses
  `DynamicBicycle.M` (used by the racecar and `BicycleDynRate`) and `examples/projects/car_trajopt`'s
  ladder, and `DynamicBicycle` moving onto the generalized child is a base-class change.

## 3. Statements about today's code that are stale (agent fixes)

- **Step model** (sampled-path.md §1, §3.6; TODO SM rows): the DP grid has four inline Euler steps,
  not two (`discretizer.py` ×2, `dp.py`, `policy_eval.py`); shooting has three paths (a JAX scan, the
  compiled `rk4_integrate_linear`, a Python loop) and multiple shooting its own; about ten hand
  steppers are missing from §1 (two MPC projects and benchmarks, `pid_autotuning_jax`,
  `neural_controller_jax`, `cartpole_rollout_gradients` and its notebook, tutorial 11,
  `showcase_minilink`, the Simulator's fixed-step solvers); the evaluator exposes 69 distinct public
  names (NumPy's 35 are a subset of JAX's), not 91; `HybridSimulator` calls `integrate_zoh_rollout`
  and its knob is `plant_dt_inner` (`RealtimeSimulator`: `sim_dt`), not `dt_inner`; `input_interp` is a
  SciPy solver option, not a `compute_forced` keyword; the `PendulumWithNoisePort` bullet of SM-2 is
  already done (`y_dependencies_of`, tested); `PID % dt` raises `TypeError`, not
  `NotImplementedError`; `benchmarks/params_binding.py` prints and gates nothing, and its "frozen"
  case is a user-side `jit`, not the evaluator's.
- **Path to the cost** (differentiable-problem.md §4.2): `score_trajectory`'s callers are
  `Evaluation.of_trajectory` and two `MonteCarloEvaluator` paths (`PlanningSolution.cost` does not
  exist; `compute_cost` calls `cost.total_cost`); `compare` takes no positional solutions and replays
  nothing; `Trajectory` has no roles today, so V1.1's "role layout kept by save and load" is a new
  field and a save-format change; `Set.contains` is already derived from `margin` (NumPy only) and
  `CallableSet.contains_fn` overrides it; `discount_rate` has 18 readers, `tabular.py` and
  `core/inspect.py` among them.
- **Clean-up and names**: K2 and K8-4 both list `ProblemEnv`/`to_gymnasium` and `camera.py`; K8-0 and
  K8-4 both list R55's twins; K3's plants are K8-3's. `MPCBroadcastController` is not on the
  `control.mpc` facade (K3 has nothing to do there). A6's "68 display names" counts `self.name = …`
  only; ten `name=` defaults (the filters, `TransferFunction`, `StateSpaceSystem`, `LTISystem`,
  `NeuralPolicyController`, the three mechanical bases' f-strings, `lqg`'s box) fall under its gate
  too. R63's rename is an eighteenth beside R52's seventeen, and two tests (not one) call
  `JointImpedance(dof=2)`. G3 is partly done (`JointImpedance` and `TaskImpedance` are on the root;
  `RRTPlanner` already lives in a teaching module: a ledger edit). R61's symbolic export must read
  `MechanicalSystem`, but the symbolic tier defines its own class of that name.
- **Elsewhere:** "S74 deferred (R25)" survives in the plans index, the S76 row and the
  parametric-program row (R74 put S74 in v0.3); ROADMAP's gantt has no names phase, S32, G3 or S74;
  ROADMAP §3's compile row puts "the draws" in v0.4 though the Monte Carlo fold is v0.3 (R44).

## 4. Size, for planning the work

| Area | Library and test files | Notebooks |
| --- | --- | --- |
| K2 | ~50 | 0 |
| K8 (cars, experimental plants, twins) | ~65 | 3 |
| K10 (without `spatial/`) | ~75 | 5 |
| A6 (renames, display names, ids) | ~90 | 9 |
| G3 | ~50 | 4 |
| S32 | ~25 | 0 |
| The step model (evaluator call sites outside `evaluators/`) | ~45 | 5 |
| V1 (`PlanningProblem`, `MonteCarloEvaluator`, the cost's members) | ~90 | 22–23 |

About thirty of the forty-four notebooks under `examples/` change at least once, and five
intended number changes need the maintainer's sign-off (the forced hold, the integral to `tf`,
trajopt's `U`/`Xf` defaults, the DP/RL price, the Monte Carlo re-pin).
