# Open design questions for the maintainer

The design questions that wait for the maintainer: the architecture, the API and its naming, the
vision, and what affects performance. Details, versions, process, tests and demos are not here:
they sit in their workboard rows, and the rows that wait for the maintainer's go are listed at
the head of [TODO.md](../plans/TODO.md#rows-waiting-for-the-maintainers-go). Known defects are in
[KNOWN_ISSUES.md](KNOWN_ISSUES.md).

Answers are recorded in a dated record under [docs/reviews/](../reviews/) (the latest:
[R154–R166](../reviews/2026-10-28-session-rulings.md), 2026-10-28; before them
[R124–R140](../reviews/2026-10-27-design-questions-rulings.md) and
[R141–R153](../reviews/2026-10-27-core-vision-decisions.md)), in the plan doc each shapes, and in
TODO; the entry then leaves this file. The v0.3 core's rulings are the architecture of record
(R166). `§N` names the workstream of
[core-architecture.md](../plans/core-architecture.md).

## The core

The v0.3 core's architecture is of record (2026-10-28, R166). The questions below came from
the night run's review (2026-10-29) and from the independent review of both runs the same day
([the record](../reviews/2026-10-29-independent-review-of-the-runs.md)); the first is the
night run's own.

### Should a field's numbers be leaves of the set's and the cost's tree?
- **Today:** a field's wrappers read the field's own parameters, never the tree they are given
  (`FieldSet.margin`, `FieldInputSet.margin`, `FieldCost.g`, `FieldTerminalCost.h` call
  `field.value(...)` without `params`; their docstrings say the field's numbers are not yet
  leaves of the tree). A box used both as `X` and as `X.as_cost()` then has two homes: moving
  `X` through the problem's tree moves the constraint and leaves the soft cost on the frozen
  box; an MPC ended by `QuadraticField(x̄, S).as_terminal_cost()` has `S` outside its tree.
  A field's numbers (a track's waypoints, an ellipsoid's `P`, a barrier's center) are then
  outside the problem's tree: `jit`, `grad` and a parameter family do not reach them, against
  the constitution's "one signature: an object that does not yet read `params` is a gap".
- **Evidence:** the night run's textbook review (finding 4). `QuadraticField.value` itself
  reads its `params`; only the wrappers drop it.
- **Options:**
  - (a) the wrappers take `{"weight": w, "field": field.params}` and pass `params["field"]`
    through: every problem holding a field cost gains those leaves (the racecar's corridor, the
    circuit's track), traced into the collocation programs;
  - (b) keep the field's numbers out of the tree, stated as a design (a field is structure,
    like a set's shape), and close the gap in the constitution's words.
- **Detail:** [the night run's log](../reviews/2026-10-29-night-run-log.md), N-19; the review's
  worked snippet of option (a) in [its record](../reviews/2026-10-29-independent-review-of-the-runs.md).
- **Answer:**

### Does a module-level cache of compiled evaluators meet CONSTITUTION §4.4?
- **Today:** `compiled_of` (`core/compile/cache.py`) keeps a 16-entry LRU of JAX evaluators,
  reached by every convenience call (`compute_trajectory`, `Simulator`, `problem.compile`), keyed
  by each block's identity and edit count, its ports, wiring and numbers; `clear_compiled()` is
  the escape. §4.4 says "no hidden registries, no invisible global switches, no implicit backend
  state"; the night brief itself wrote that the cache "sits uneasily with §4.4" and N-2 kept it
  without a ruling. Two blind spots stay in KNOWN_ISSUES (a class's method replaced, a change deep
  inside a held object); a module-global read by `f` is a third (the review, B8).
- **Options:**
  - (a) keep it and amend §4.4 with one sentence naming the one registry allowed: a cache keyed by
    structure, never by values, emptied by `clear_compiled`;
  - (b) an instance-owned cache with the edit counters (a loop rebuilt over the same blocks then
    compiles again, 0.35 s each);
  - (c) handles only (`sys.compile()`), the convenience calls compiling fresh.
- **Detail:** [the night run's log](../reviews/2026-10-29-night-run-log.md), N-2, N-13.
- **Answer:**

### The RL training reward: the one score, or a training reward beside it?
- **Today:** `RolloutEnvironment` rewards `r_k = −c_k`, the face's `∫₀^dt e^{−ρτ} g dτ` under
  its solver (RK4 by default), the one score (R142, R159). On the cart-pole demo it learns less
  reliably: 9/12 seeds swung up at 2M steps before the run with `−g(x_k, u_k)·dt`, 4/12 with
  `−c_k`, 7/12 with `−g·dt` put back alone; 3/8 at 3M and 2/8 at 4M (N-7, N-22). The Gym bridge
  still trains Stable-Baselines3 on `−g·dt` (RD-40), so the library has two training rewards.
- **Options:**
  - (a) one reward, the score's `−c_k` (today), the demo tuned or its budget raised;
  - (b) a training reward `−g(x_k, u_k, t_k)·dt` recorded beside the scored one, as `gamma` is
    (R150's pattern: a solver may optimize its own objective, the score stays the problem's and
    the record says so, R159);
  - (c) `−c_k` under one Euler step for training (`−g·dt` to the bit) while the score runs RK4.
- **Detail:** [the night run's log](../reviews/2026-10-29-night-run-log.md), N-7, N-22.
- **Answer:**

### Value iteration's `alpha=` on a problem that already discounts
- **Today:** the face keeps the problem's `ρ` inside each period (`c_k = ∫₀^dt e^{−ρτ} g dτ`)
  while `alpha` replaces `e^{−ρ dt}` between periods, so the backed-up quantity is
  `Σ αᵏ ∫₀^dt e^{−ρτ} g dτ`, two unrelated discounts. Under one Euler step the two coincide (the
  inner factor is 1), which is why N-5 called the inner discount second order; under RK4 the
  review measured `J = 11.19` at `ρ = 2` against `12.28` at `ρ = 0` for the same `alpha = 0.9`.
  The test that pins "an explicit `alpha` wins" asserts only `options.alpha`.
- **Options:**
  - (a) `alpha` sets the face's rate too, `ρ = −ln α / dt`, so the face, the price and the backup
    share one discount (the price already does this in `price_at`), the record naming the override
    as R150's `gamma=` does;
  - (b) refuse `alpha=` when `discount_rate > 0`, naming the one discount to give (the night run
    tried and reverted it, N-5);
  - (c) keep the mix and document it.
- **Detail:** [the review's record](../reviews/2026-10-29-independent-review-of-the-runs.md), C.
- **Answer:**

### The MPC: its default form, what `lookahead` means without a delay, and the print in `update`
- **Today:** `ModelPredictiveController(planner)` defaults to the delayed form (R123, a real
  computer), while every demo, project and course notebook passes `delay=False` (14 of 16
  constructions since your call of 2026-10-28/29); `lookahead=Δ` predicts the start under "the
  command the block applies now" only in the delayed form: the zero-delay form predicts under the
  previous plan's command and applies the new plan's, the pure form under the default guess's
  zero; `verbose=True` prints one line per solve inside `update`, which RD-51 had rejected as "a
  print inside a pure `update`" (CONSTITUTION invariant 2 forbids state and memory, not a print:
  a tension, not a breach; the review fixed its crash under a trace).
- **Options:**
  - the default: (a) `delay=True` and the teaching surface writes `delay=False` (today);
    (b) `delay=False` the default, the research lane asking for the delay; (c) one architecture
    (your decision 7 of 2026-10-28) where the delay is a visible `z⁻¹` block and the keyword goes;
  - `lookahead` without a delay: refuse it; or define it per form in the docstring; or a predictor
    block on `x_start` with the keyword shifting the time only (planner-and-mpc §6.4 as first
    written);
  - the print: rule it allowed in an algorithm body that is also a state equation (RULES 5.26,
    `dp.py`'s `SweepLog` precedent), recording it against RD-51; or move it to the run's recorder.
- **Detail:** [the review's record](../reviews/2026-10-29-independent-review-of-the-runs.md), D.
- **Answer:**

### One tree shape for every planner?
- **Today:** R148 says a planner's tree is its problem's plus `"method"` and `seed`. MPPI's tree
  has `method` and `seed` but not the problem's `cost`, `discount_rate` or sets, so
  `controller.cost.Q` tunes nothing on `ModelPredictiveController(MPPIPlanner(…))` where DESIGN
  §4 promises it; trajectory optimization's tree has the problem's leaves and no `method` (its
  learning rate a constructor option); the RL planner has no `params` tree at all, `gamma` an
  attribute.
- **Options:**
  - (a) every planner's tree is the problem's parts plus `method` and `seed` (an empty `method`
    where nothing binds), MPPI's environment reading the cost's numbers through its `params`;
  - (b) each planner names the leaves it reads (today), and DESIGN's "the MPC's tree is
    `planner.params`, `controller.cost.Q` tunes" is qualified per planner.
- **Detail:** [the review's record](../reviews/2026-10-29-independent-review-of-the-runs.md), D.
- **Answer:**

### What marks the plant of a sampled loop?
- **Today:** the words (RD-13, RD-16) say the plant is "the block integrated inside every base
  period" and "a law under `%` is never integrated adaptively"; the code marks it by
  `resolves`, a continuous source not held exactly, which a law under `%` (Euler) satisfies. So
  `(pid % Δ) >> discretize(plant, Δ)` is refused ("name it `plant`"), and a lone `pid % Δ` or
  `Pendulum() % 0.1` runs `solver="scipy"` as an adaptively integrated plant, 0.12–0.16 away
  from its own scan.
- **Options:**
  - (a) a mark set by construction: `discretize` makes a plant, `%` a law (one boolean on the
    view);
  - (b) the role alone, `@`'s `plant` (a chain then has no plant and one point per period);
  - (c) `solver != "euler"` as the mark (`discretize(plant, dt, solver="euler")` then loses its
    substeps).
- **Detail:** [the review's record](../reviews/2026-10-29-independent-review-of-the-runs.md), A.
- **Answer:**

### One failure price, or one per tool?
- **Today:** the judge, the RL environment and value iteration each hold a bound of their own
  (`g_max ∫₀ᵀ e^{−ρt}dt + e^{−ρT} h_max`; `g_max Σ_{k<N} γᵏ dt + e^{−ρT} h_max`;
  `g_max dt/(1 − γ)` with no `h`: 193.8, 198.7 and 314.3 on the same pendulum), and a
  discretized problem answers a fourth from its own method (179.6) while its face answers the
  continuous one (RD-44). DESIGN documents the three; R159 says a solution's score is always the
  problem's cost through the common step function.
- **Options:**
  - (a) one price on the face, `face.price()`, read by the judge, the environment and value
    iteration, so a failed run has one `J` wherever it is scored;
  - (b) keep the three as the tools' own, said in one DESIGN line, and route the step problem's
    own bound through its origin as the face does.
- **Detail:** [the review's record](../reviews/2026-10-29-independent-review-of-the-runs.md), C.
- **Answer:**

### `%` on a native law copies; on a view it shares
- **Today:** `sys % dt` and `box % dt` return views that share the blocks (their numbers and
  starts move together, N-10), `law % dt` on a native sampled law (the MPC) returns a copy with
  its numbers and start deep-copied, and that copy shares the law's port tables with functions
  still bound to the unclocked original (a bug the review found: a standard `y` port on such a
  law fails under `%`). One operator, two sharing rules.
- **Options:** (a) the view rule everywhere (a clocked native law shares its numbers with its
  source, as `pid % dt` does); (b) the copy rule for native laws, stated in DESIGN, with its own
  ports rebound; (c) refuse `%` on a native law that already has a period (today) and require
  the period at construction.
- **Detail:** [the review's record](../reviews/2026-10-29-independent-review-of-the-runs.md), A.
- **Answer:**

### Who owns the start of a sampled loop's held outputs?
- **Today:** R124 names three owners of a start (the block, the diagram's view, the problem's
  leaf) and gives every stateful block an `x0` leaf of the evaluator's tree; a sampled loop's
  held outputs start from `hold_x0`, an array on the loop that is neither a block's `x0` nor a
  leaf (the evaluator reads it as a constant), silently reset to zeros when the hold layout
  changes.
- **Options:** (a) `hold_x0` becomes a leaf of the loop's entry in the evaluator's tree;
  (b) declare it structure, always zero (every slow block fires at `k = 0`), and have the `x0`
  setter refuse a nonzero hold slice.
- **Detail:** [the review's record](../reviews/2026-10-29-independent-review-of-the-runs.md), E.
- **Answer:**

### Which lane is `minilink.planning` for MPPI?
- **Today:** ROADMAP §2 makes every band facade the teaching surface, with an entry rule and a
  deprecation promise, the spatial names its one provisional exception; N-14 exported
  `MPPIPlanner` and `MPPIRecord` on `minilink.planning` as "the provisional band", research
  lane. The teaching-surface test pins an explicit list, so the facade is not checked.
- **Options:** (a) ROADMAP §2 names them provisional beside the spatial names until the teaching
  decision (MP-6, V3); (b) they leave the facade until then (deep import only, as `StepFace`);
  (c) they enter the teaching surface now (a demo and a docstring exist; a both-backends test
  cannot, MPPI is JAX-only).
- **Detail:** [the night run's log](../reviews/2026-10-29-night-run-log.md), N-14.
- **Answer:**

### Should the compile probe cover port functions?
- **Today:** the probe that refuses an output moving with an input it does not declare (S66)
  runs only on an output computed by the leaf's own `h` (the `DynamicSystem` shortcut); a port
  added by `add_output_port` with its own function, or on a bare `System`, still schedules
  before its source and reads `0` silently (KNOWN_ISSUES).
- **Options:** (a) probe every port function the same way (one extra call per undeclared input
  at compile; a custom compute may be costly); (b) keep `h` only and let KNOWN_ISSUES say so.
- **Detail:** [the review's record](../reviews/2026-10-29-independent-review-of-the-runs.md), F.
- **Answer:**

## Toward v1.0

### How does a public name change once v1.0 is frozen?
- **Your answer (2026-10-27):** not a `DeprecationWarning` shim for one minor release, as
  proposed; the policy stays open (R139). R82 (no alias, one sweep) holds meanwhile.
- **Blocks:** the trust contract (V4); the renames that wait for the freeze (`xbar` / `ubar` →
  `x_bar` / `u_bar`, `plot_cost2go` → `plot_cost_to_go`; `NeuralPolicy` moved to v0.3, R151).
- **Today:** no alias or shim ever (R82). The frozen names are drawn at v0.9 from one rule (R139).
- **Options still open:** do the pending renames before the freeze, so none is needed after it;
  a shim of another length; a major version per rename; a snapshot test of the export list
  either way.
- **Detail:** [TODO, V4](../plans/TODO.md#v09--the-trust-contract)
- **Answer:**
