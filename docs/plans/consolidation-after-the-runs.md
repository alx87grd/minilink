# The consolidation after the two runs: the plan (2026-10-29)

**Read first.** This is the one plan for the work the maintainer ruled on 2026-10-29 after the
independent review of the v0.3 core run and the night run
([the review](../reviews/2026-10-29-independent-review-of-the-runs.md); the rulings R168–R179 in
[the record](../reviews/2026-10-29-consolidation-rulings.md)). An agent picks it up and implements
it; nothing here is open except where a step says "**[ask]**". Each section gives the problem as
found, the decision with its reason, the design in the ruled names, what to delete, what to
write, and its "done when". The order of §9 is the order to work in.

**The working rules** are AGENTS.md's: one step, one branch off `dev-v0.3`, one commit per row
pushed as it lands; the seeded baseline `benchmarks/v03_baseline.py` captured before the first
edit and `cmp`'d after every row (byte-identical unless a row's ledger line says what moved and
why); every math line read beside `planning/dynamic_programming/dp.py` and said so in the report;
the gates of tests/README.md's Agent table ("Always before push", "Cross-cutting or handoff",
"Regression gates, CI flags", "Notebook change" when a notebook moves, "Docs build" when a
docstring moves). Tuning constants in demos and notebooks are never changed; a demo whose loop
line is rewritten keeps its numbers. The report of the run goes to a dated log under
`docs/reviews/`, in the format of the night run's log (decisions, ledger, textbook check,
progress).

**The spirit, in the maintainer's words.** The planning problem defines the problem; a tool may
plan or train on a modified copy of it (another discount, another reward) and its record names
the modification, but the score is always the problem's. Every core object carries `params`, and
every number that is not structure lives there; speed comes from the frozen form, never from a
number hidden in code. A tool is built on a system once and run many times; the convenience
verbs compile every call. `%` means one thing: this block runs on the computer at this period.
A sampled loop is a computer closed on one continuous system.

---

## 1. Compile once, without the cache (R168)

**The problem.** The convenience calls (`compute_trajectory`, `Simulator`, `problem.compile`)
reused a compiled JAX evaluator from a module-level 16-entry LRU (`core/compile/cache.py`) keyed
by each block's identity and an edit counter bumped by `System.__setattr__`. It met the two asks
of 2026-10-28 (a frozen run as fast as hard-coded; no recompile when nothing structural changed)
but it is a global registry (CONSTITUTION §4.4), it has blind spots (a class method replaced, an
edit deep in a held object, a module global read by `f`), and it serves only three usages: the
facade called repeatedly, `problem.compile(ctl)` or `closed_loop(...)` called repeatedly, and
the planner compiling the plant once per planner. The audit found every tool that calls a
compiled function many times (`Simulator`, `ProblemEvaluator`, `Optimizer`, the planners' program
evaluators, the MPC's and MPPI's faces, the RL planner, the Lyapunov tool, value iteration on
JAX, the RRT extender, the live plot) already builds its object once and holds it.

**The decision (c).** No cache for the high-level verbs. A convenience call compiles every time
(about 0.35 s on JAX; a fraction of a millisecond on NumPy). Speed is the explicit tool held by
the user (`Simulator(sys, backend="jax")`, `pe = problem.compile(ctl)`, `planner.compile()`), as
today. The two forms inside each evaluator stay exactly as built: `params=None` runs the frozen
executable (the compiled values constants of the trace), a tree runs the parametric one traced
once, and `call_params()`'s digest chooses between them, so a frozen run never pays for the
argument form and a value sweep on one tool compiles once. The edit counter goes with the cache.

**Delete:** `core/compile/cache.py` whole (`compiled_of`, `fingerprint`, `clear_compiled`,
`NOT_STRUCTURE`, `COMPILED`); `System.__setattr__`, `System.edits` and `NOT_EDITS` in
`core/system.py`; every key that reads `edits` (the RL planner's `rollout_forms` key, the
trajectory-optimization planner's offline key: each becomes "built at construction, rebuilt by
the user", or a key on the parts' identity where a tool genuinely rebinds); the KNOWN_ISSUES
entry "A class's method replaced after a JAX run…"; the stale-case tests of
`tests/unittest/test_compile_once.py` that test the cache (eviction, the remedy, the stale
cases) while keeping those that test the two forms and the digest switch; `docs/plans/compile-once.md`
(its contract moves to DESIGN §5 in three sentences: the two forms, the digest, "a convenience
call compiles each time; hold a tool").

**Change:** the seven call sites of `compiled_of` (`facades/simulation.py`'s `compiled_run_of`,
`simulation/simulator.py`'s `build_evaluator`, `planning/evaluator.py`'s `compile_loop`,
`transcription.py`, `shooting.py`, `planner.py`, `compiler.py`'s `compile_auto(reuse=True)`)
call `compile(system, backend)`; DESIGN §4–§5's compile-once paragraphs; the night log's N-2 and
N-13 rows are history and stay.

**Done when:** `grep -rn "compiled_of\|clear_compiled\|\.edits\b\|NOT_EDITS\|NOT_STRUCTURE" minilink tests docs` is empty;
`test_compile_once.py` still pins the frozen form against the parametric one and the digest switch;
the baseline is byte-identical; the night sweep's flagship demos run (a few seconds slower where
a demo repeats a facade call: say which in the log).

## 2. Every number a parameter, every core object a `params` (R169)

**The problem.** The fields' wrappers (`FieldSet.margin`, `FieldInputSet.margin`, `FieldCost.g`,
`FieldTerminalCost.h` in `core/fields.py`) call `field.value(...)` without `params`, so a field's
numbers (an LQR terminal `S`, a corridor's geometry, a barrier's center) are outside every tree
they sit in. A box used as `X` and as `X.as_cost()` has two homes: moving `X` through the tree
moves the constraint and leaves the soft cost on the old box. The review's probes also found
numbers held outside `params` on other objects (the list below is the starting point, not the
whole audit).

**The decision (a), generalized.** Every core object (system, cost, set, distribution, field, and
any composition of them into a problem or a solver) has a `params` tree, and every number that is
not structure (a scalar, a vector, a matrix, a tensor) is a leaf of it. A number may stay out of
the tree only for a written structural reason (a size, a count, a choice of method, a boolean
switch). Speed comes from the frozen form (§1), so nothing is lost by moving a number into the
tree. A composed object's tree is the dict of its parts' trees.

**Design.**
- `FieldCost.params = {"weight": w, "field": field.params}`, `FieldTerminalCost` the same,
  `FieldSet.params = {"field": field.params}` (and bounds `lower`/`upper` when they are numbers),
  `FieldInputSet` the same; each wrapper passes `params["field"]` to `field.value`. `MarginField`
  (the set seen as a field) forwards the set's tree, so `Set.as_cost` reaches the box through
  `params["field"]`.
- **The audit**, object by object, under `minilink/core/` and the planning objects: every
  attribute an equation path reads at call time that is a number goes to `params`, read by name
  in the parameters block (RULES 5.3). Known candidates: `LookupPolicy`'s table (core-architecture
  §1 "Today"); `TrajectorySource` (already `params["values"]`, check its times); the step costs
  (`StepCost.params` is a property that writes on read, RULES 5.7: an attribute set in `__init__`);
  `QuadraticField`'s schedule times `t` (structural? they are a grid: a number array, so a leaf);
  the shaping functions' constants in `core/shaping.py`; the catalog plants that keep a number on
  `self` (grep for `self.<name>` in `f`/`h` bodies outside the parameters block); the RL planner,
  which has no `params` tree at all (its tree is §3's); a model-based law holding its plant
  (`ComputedTorque(plant)`), whose model numbers are the TODO §1.1 row "a model-based law's
  internal model numbers in its own params".
- The contract test `tests/unittest/test_contract.py` walks the new leaves: `params=None` the
  defaults, a moved tree moving the math, a finite `jax.grad` through each new float leaf.

**Delete:** the docstrings that state the gap as a design (already reworded to "not yet"; now
delete the clause); the QUESTIONS.md entry (ruled).

**Done when:** every object in the contract test's table has `params`, the audit's list in the
log names each number moved and each kept with its structural reason; the racecar corridor and
the terminal-field demos run with their numbers unchanged (the leaves default to the same values);
the baseline is byte-identical; DESIGN §4 "One signature" gains the sentence "a field's numbers are
leaves of the set's and the cost's tree".

## 3. One tree per planner (R170)

**The problem.** R148 says a planner's tree is its problem's plus `"method"` and `seed`. MPPI has
`method` and `seed` but not the problem's `cost`, `discount_rate` or sets, so
`controller.cost.Q` tunes nothing on `ModelPredictiveController(MPPIPlanner(…))` where DESIGN
promises it; trajectory optimization has the problem's leaves but no `method` (Adam's learning
rate is a constructor option); the RL planner has no tree.

**The decision.** One rule for every planner: `planner.params = {the problem's parts…,
"method": {the method's numbers}, "seed": s}`. The method's *name* (`"adam"`, which transcription)
is structure and stays a constructor option (CONSTITUTION: `params` never holds a string switch).
The method's *numbers* are leaves under `"method"`: a learning rate, MPPI's `temperature`, `noise`,
`control_cost`, the RL planner's `gamma` (its training discount, §4.3), an iteration count as an
integer leaf never in `wrt`. A planner that draws nothing has `"seed": None`. An empty `"method": {}`
is allowed where a method has no numbers.

**Design.** `TrajectoryOptimizationPlanner.params` gains `"method"` (the native laws read their
numbers from the tree: `GradientDescentLaw` and `AdamLaw` take their hyperparameters from
`params["method"]` through the optimizer's `solve(params=)`); `MPPIPlanner.params` gains the
problem's parts (`cost`, `discount_rate`, the sets that hold numbers) and its environment reads
the cost through `params` (`RolloutEnvironment.episode_tree` merges the cost's subtree as it merges
the plant's); `ReinforcementLearningPlanner.params` is created (`"method": {"gamma": …, the
algorithm's numbers}`, `"seed"`). The MPC's tree is the planner's, under the solve block (§6).

**Done when:** `planner.params` has the same shape on the three planners (a test walks it);
`pe.params["controller"]["solve"]["cost"]["Q"]` tunes an MPPI MPC (a test moves it and the plan
moves); `jax.grad` through MPPI's `temperature` is finite; the baseline is byte-identical (the
defaults are the constructor's values).

## 4. The one score's edges (R171, R172, R173)

### 4.1 The RL training reward beside the score (R171)

**The problem.** The RL environment rewards `−c_k`, the face's stage cost integrated under RK4,
and the cart-pole demo learns less reliably (9/12 seeds at 2M steps before the run, 4/12 after,
7/12 with `−g·dt` put back; 3/8 at 3M). The Gym bridge still trains on `−g·dt`.

**The decision (b).** The environment's training reward is `r_k = −g(x_k, u_k, t_k)·dt`, one call
of the cost's own `g` the environment already holds through the face, traceable, no new pipeline;
the terminal and the failure price stay as they are; the planner's `evaluate` keeps reporting the
judge's `J` on the problem's cost; the record says "trained on −g·dt, scored on the problem's cost".
A `reward=` option on the environment selects `"stage"` (`−c_k`, the score's) for whoever wants
it. The condition the maintainer set: if this needs more than that one call, come back.

**Done when:** the RL baselines re-pin once with a ledger line (the reward moved); acceptance
script 13's gate (R160) holds; the cart-pole demo's reliability is re-measured on 8 seeds at its
budget and reported (no constant changed).

### 4.2 One cost bound on the face (R172)

**The problem.** The judge, the RL environment and value iteration each held a bound of their own
(193.8, 198.7, 314.3 on one pendulum) and a discretized problem answered a fourth from its own
method; the noun was `price`.

**The decision (a).** One bound on the compiled face, the face's own sum with the maxima:
`cost_bound = g_max·dt·Σ_{k<N} γᵏ + γᴺ·h_max`, the infinite horizon its geometric limit
`g_max·dt/(1 − γ)` with no `h`; it bounds every stage since `c_k ≤ g_max·dt`. The judge, the
environment and value iteration read `face.cost_bound`; `StepPlanningProblem.cost_bound` is the
same number (one owner through the face). The noun is `cost_bound` everywhere (`price`, `price_at`,
`infeasibility_price` renamed; the user's switch keeps its name `infeasible_cost`: `True` the
bound, a number that number).

**Done when:** the three tools give one number on one problem (a test); the judge's and value
iteration's baselines re-pin once with a ledger line; no `price` left in `minilink/` (`grep`).

### 4.3 A tool's own discount is internal (R173)

**The problem.** Value iteration's `alpha=` on a problem with `discount_rate > 0` backed up
`Σ αᵏ ∫₀^dt e^{−ρτ} g dτ`, two discounts; invisible under Euler, measured under RK4.

**The decision.** `alpha` (value iteration) and `gamma` (RL) are tool-internal discounts: the tool
builds its face with that discount consistently inside and between periods (the stage integral and
the step both at `ρ_tool = −ln α / dt`), plans or trains on it, and its record names the override;
the score of the resulting policy stays the problem's (the judge at the problem's `ρ`). DESIGN §6
states the rule once: "the planning problem defines the problem; a tool may plan or train on a
modified copy and says so in its record". `test_dp_reads_the_problem_discount_rate` asserts the
backed-up `J`, not only the option.

**Done when:** the mixed-discount `J` is gone (the review's measurement, `repro_b.py`, gives one
value whatever `ρ` when `alpha` is given); the RL `gamma` path is unchanged in numbers.

## 5. The loop's grammar: the plant continuous, the computer clocked (R174, R175)

**The problem.** One `SampledSystem` served three meanings: a law implemented on the computer
(`pid % dt`, Euler), a plant simulated between the computer's samples
(`discretize(sys, Δ, substeps=round(Δ / SIM_DT))` in every MPC demo: the period redundant, the
substeps an awkward ratio), and a discrete-time model (`discretize(plant, dt, solver="exact")`,
the step face). The loop marked its plant by `resolves`, a solver kind, so `pid % Δ` alone was
integrated adaptively by the host runner and a chain of two Euler blocks was refused.

**The decision (B), the five rules.**
1. `%` means one thing: this block runs on the computer at period Δ. A continuous law becomes its
   Euler difference equation; a static block is sampled and held; a tick law gets its clock; a box
   each block. The block's *kind* (its base class) decides what its third argument is, never the
   operator: a `System` (static or continuous law) receives seconds `t_k = t_start + kΔ`; a
   `StepSystem` (a tick law) receives `k`, its own count of firings, never seconds, never Δ; a
   `NativeSampledSystem` (a sampled-data law written in seconds whose equation needs its period)
   receives seconds and takes `sample_period=` at construction, as the noise block does, and `%`
   on it is refused ("already on a clock"). The one convention a student must know: a static
   `System` in a *pure* step diagram (no clock anywhere) receives `t = k`.
2. A sampled diagram holds at most one continuous system: it is the plant, found by kind, never by
   id. Two are refused: "compose them into one continuous system first" (actuator, plant and
   sensor dynamics close under the continuous algebra).
3. `discretize(sys, Δ, solver=, substeps=)` is the textbook verb: a discrete-time model for the
   discrete tools (the step face, value iteration, the RL environment, z-domain analysis). It is
   never needed to close a loop; a loop built on one is a loop of step systems (rule 4).
4. The dense run, the substeps and the host runner exist exactly when the loop has a continuous
   system; a loop of step systems only has ticks (`compute_trajectory` on it is the ticks, or
   refuses naming `compute_rollout`).
5. Time: seconds wherever there is a clock, the count where there is none (R158, unchanged).

**The plant's integration step.** One rule, the continuous simulator's: a fixed-step solver steps
at `Δt = AUTO_DT_SCALE·τ_min` (the plant's `solver_info["smallest_time_constant"]`, the catalog
default 1 ms), so `substeps = ⌈Δ / Δt⌉`; the loop's exception (`HELD_SIGNAL_STEPS_PER_SAMPLE`,
one step per period for an undeclared time constant) goes. The knob: `dt=` at the run verbs
(`loop.compute_trajectory(tf, dt=0.01)`, `loop.compute_rollout(N, dt=0.01)`, the plant's
integration step, the continuous facade's word) and `substeps=` at the compiles
(`problem.compile(mpc % Δ, substeps=20)`, `loop.compile(substeps=20)`, where `dt=` already names
the loop's period for an unclocked controller). `solver=` beside it (`"rk4"` default, `"euler"`,
`"scipy"` the adaptive host runner).

**The pipelines, as the user writes them** (the acceptance list; each a test):
```python
pid @ plant                                       # continuous; compute_trajectory(dt=), linearize, bode
UnitDelay() >> DiscreteLowPass(alpha)             # pure step, t = k; compute_rollout(N); jacobian; linearize → z-poles (R122)
(Sine() % 0.01) >> (pid % 0.01)                   # a computer alone, seconds; linearize → a discrete LTI with its period
(pid % 0.01) @ plant                              # the loop: compute_rollout, compute_trajectory(tf, dt=), solver="scipy", record=
((filt % 0.01) >> (ctl % 0.1)) @ plant            # multi-rate; closed_loop(ctl % 0.1, plant, filter=filt % 0.01) by roles
(DiscreteLowPass(a) % 0.01) >> (pid % 0.01)       # a tick law on its rate inside the computer; bare, it ticks on the base clock
(ctl % 0.01) @ (actuator >> plant >> sensor)      # continuous dynamics the computer does not own are the plant's
Pendulum() % 0.01                                 # a plant's Euler model as a computer block (a predictor), never adaptive
discretize(plant, 0.1, solver="exact"); problem.compile(dt=0.1); problem.discretize(0.1)   # models for the discrete tools
(pid_o % 0.01) @ ((pid_i % 0.001) @ motor)        # a cascade: one flat plan, one base clock, the one plant inside
(mpc % 0.2) @ car; ((mpc % 0.5) >> (PlanFollower(planner) % 0.01)) @ car   # the MPC family (§6)
WhiteNoise(psd, sample_period=0.01); GaussianPolicy(..., sample_period=0.1)  # the period in the equation: the one exception
```
**Refusals, each with its message named in a test:** `pid @ (plant % Δ)` (a clocked block as a
continuous law's plant: write `(pid % Δ) @ plant`); two continuous systems in a sampled diagram;
`UnitDelay() @ plant` with no clock anywhere ("a tick law needs a clock to meet a continuous
plant: `% Δ`"); a sampled loop that already has a plant as an operand of `>>` without parentheses
(`(filt % 0.01) >> (ctl % 0.1) @ plant` binds `@` first: refuse, naming the parentheses or
`closed_loop(...)`); a bare tick law in a nested box slower than the loop (N-8, kept).

**Change:** `closed_loop`'s `on_the_loop_clock` stays as the plan's internal plant op (the wrap is
implementation, its kind "plant" set there, never by the user); the plan's `plant_op` reads the
one continuous system; `require_one_plant` becomes rule 2's refusal; `sampled_substeps` becomes
the shared rule; `rollout_dense` and the host runner take `dt=`/`substeps=`; the facades' and
`problem.compile`'s signatures; `SampledSystem` keeps `solver`/`substeps` for the model verb;
every demo, project and notebook loop line `@ discretize(sys, MPC_DT, substeps=round(MPC_DT /
SIM_DT))` becomes `@ sys` with `dt=SIM_DT` at the run (byte-identical for the same substeps: say
so per demo in the log); DESIGN §3/§4's time table gains the three-kinds table of rule 1 and the
`%` docstring states it; sampled-path.md §8.4 is rewritten to the five rules; the A4, A10, A12,
A15 findings of the review close with it (`compute_rollout(record=)` lands here too, so a ticks run
records `plant:u` and its role).

**z-domain analysis (R175).** R122's `linearize` on a `StepSystem` and a `SampledSystem` lands
with this grammar: a discrete LTI (`A_d, B_d, C_d, D_d`, the period when there is one), `poles`
against the unit circle, `pzmap` in `z`, the discrete Bode at the period; `jacobian` already
exists. Its demo: the digital PID's poles moving with Δ.

**Done when:** the pipeline list runs as tests; the refusals fire with their messages; the flagship
MPC demos give their numbers (ledger: none moved); the hybrid demos the same; the notebooks' loop
lines moved and the smoke check green; `grep -rn "discretize(" examples` finds only the model
uses (analysis, the step face).

## 6. One MPC architecture (R176)

**The problem.** Three forms behind keywords (`delay=`, `warm_start=`, `lookahead=`); the default
(the one-period delayed form) used by no student-facing artifact; `lookahead` meaning delay
compensation only in the delayed form; a `NativeSampledSystem` whose `%` copied where every other
`%` shares; the MPC reading inside a collocation `z` (`pinned_shift`, `predicted_start`).

**The decision (c), the maintainer's decision 7 of 2026-10-28.** Every MPC is one architecture:
three pure blocks on the computer, the planner's solve (static: `z*_k = pv.solve(P_k, z0=warm)`,
warm-started from the memory), the command (`u_k = pv.command(z*_k, t_k)`), and the plan's memory
(`UnitDelay(dim=n_z)` from the default guess). The computational delay is a visible block the user
adds (`UnitDelay()` on `u`, or on `z` before the command, the textbook's `z⁻¹`); `delay=`,
`warm_start=` and `lookahead=` go; a user who wants delay compensation writes the predictor block
on `x_start`'s binding (planner-and-mpc §6.4 as first written). `ModelPredictiveController(planner,
*, inputs=, backend=, verbose=)` returns that box; `mpc % Δ` clocks its three blocks (rule 1).

**Design.**
- The planner's tree sits under the solve block: the one tuning path is `controller.solve.cost.R`
  (`pe.params["controller"]["solve"]["cost"]["R"]`), stated in DESIGN.
- The dual-rate form reads the memory's `z` port: `((mpc % 0.5) >> (PlanFollower(planner) % 0.01))
  @ plant`, the follower applying the plan at its own rate; the one-period-late plan is the user's
  `UnitDelay(dim=n_z)` between them when wanted.
- `verbose` stays a host-only report on the solve block, one line per solve (RULES 5.26), silent
  under a trace (guarded on every printed value, landed in the review).
- The shift and the prediction are face verbs, so the block never reads inside `z`:
  `pv.shift(z, t, x_start=None)` (the planner pins its own start knot) and
  `pv.predict_start(x, u, t, Δ, params)` (the planner's model stepped), MPPI's face included;
  viz's `require_state_knots` is `compiled.py`'s (one owner).
- The traced first guess: `pv.initial(params)` traceable (the default guess a function of
  `x_start`), so a traced loop's first tick equals the host's (the review's D-4).
- `NativeSampledSystem` stays for a sampled-data law written in seconds (a discrete Kalman filter
  with a time-varying model, later); it takes `sample_period=` at construction (§5, rule 1), its
  ports its own (the review's A6 closes with it).

**Delete:** `DelayedPredictiveController`, `PurePredictiveController`, the `delay`/`warm_start`/
`lookahead` keywords and their docs (DESIGN §4's MPC paragraph, planner-and-mpc §6.2–§6.4, the
zero-delay record is history), `predicted_tree`/`predicted_start`/`pinned_shift` in
`mpc/predictive.py`, the `lookahead` tests; the N-4 refusal stays (a planner whose problem starts
from a set).

**Done when:** every MPC demo, project and notebook runs the one architecture with its numbers
(ledger: the delayed-form demos of `examples/demos/compile/` move once, said in the log; the
zero-delay ones are byte-identical); acceptance script 5 (the closed loop in one scan, `vmap`,
`grad`) and the dual-rate script pass on the box; `test_mpc.py` has one form; the traced loop's
first plan equals the host's.

## 7. The problem's boundary dials (R177)

**The problem.** The start had three dials (`x_start` a point, a set or a law; `X0` a set;
`sys.x0`) and two homes in the tree (`params["x_start"]` and `params["X0"]["point"]`, which
`problem.x_start = v` desyncs); the goal had `x_goal` (a hint that never created `Xf`) and `Xf`:
the start's point was a singleton shortcut, the goal's was not.

**The decision.** Two concepts, two dials, each with a vector shortcut:

| Dial | Meaning | Accepts | Default |
| --- | --- | --- | --- |
| `X0=` | the start constraint `x(0) ∈ X0`, what planners enforce | a set; a vector → the singleton | from `x_start`: a point → its singleton, a law → its support; nothing → the singleton at `sys.x0` |
| `Xf=` | the terminal constraint `x(tf) ∈ Xf` | a set; a vector → the singleton; `None` → free end | `None` |
| `x_start=` | the evaluation start: where runs begin, what `realize` draws | a point, or a law (a set given here is the uniform law on it) | from `X0`: a singleton's point, else the region's center (a nominal run; no randomness unless a law is given) |
| the cost's `xbar` | the objective's target, the cost's own number | in the cost's `params` | the cost's |

Both may be given (`x_start=Gaussian(x̄, σ)` with `X0=box`): evaluations draw from the law,
planners constrain to the set, and the law's mean (or the point) must belong to `X0`, refused
otherwise. `X0` and `Xf` are the sets, period. `x_start` is the one leaf `params["x_start"]`,
written through to a singleton `X0` (one owner; `problem.x_start` a property whose setter is
`move_start`); a singleton `X0` is not a second entry of the tree (a region's numbers stay under
`X0`). `x_goal` disappears: a goal has no evaluation role; a planner's heuristic endpoint reads
`Xf`'s point, else the cost's `xbar`, else the start (`initial_guess.py`, the DP grid's anchor);
RRT's goal region is `Xf`, a ball written as one (`Xf=BallSet(goal, tol)`); the `start_law`
attribute becomes the law held in `x_start`.

**Demos and notebooks:** `x_goal=` becomes `Xf=` where the arrival is required and is deleted
where the cost does the work (a point `Xf` is a hard terminal equality for collocation and
shooting: an unreachable goal in `tf` is then reported infeasible rather than ignored; each such
demo gets a ledger line). 19 `x_goal=` lines, 12 `Xf=`, 67 `x_start=` (these keep working).

**Done when:** the constructor has no `x_goal`; `problem.params` has one home per number; the
desync test (`problem.x_start = v` then `X0.point`) passes; every demo and notebook runs; the
docstring of `PlanningProblem` carries the table above.

## 8. The details the review owns (R178, R179 and the agent's)

- **`hold_x0`** (the start of a sampled loop's held outputs): a leaf of the loop's entry in the
  evaluator's tree (`params["<loop>"]["hold_x0"]`), differentiable and `vmap`pable like every
  block's `x0`; the silent reset to zeros on a layout change becomes a refusal.
- **The compile probe** covers every output port function, not only the shortcut's `h`: one extra
  evaluation per undeclared input at compile; the KNOWN_ISSUES entry leaves when it lands.
- **The hold slot's label** in `traj.x` reads as the memory it is (`controller:u (memory)`), and
  DESIGN says it is one period behind the recorded signal.
- **MPPI's lane:** `MPPIPlanner` and `MPPIRecord` stay on `minilink.planning`, named provisional in
  ROADMAP §2 beside the spatial names until the teaching decision (MP-6, V3); ROADMAP §2 is the
  maintainer's, so the sentence is drafted in the log for him.
- **The review's open findings** that no ruling above covers, each a row: `RolloutEnvironment` on a
  `StepPlanningProblem` (a refusal naming `dt`, or reading the face's `N`, `γ`, `cost_bound`); the
  planners' evidence carrying the face's running cost so `evaluate_trajectory` gives the score, not
  the trapezoid; the three copies of the Euler stage-cost rule folded into `stepping.py`'s one;
  `StepCost.params` set in `__init__`; the shared class-level `params = {}` on `Set`, `InputSet`,
  `Field`, `Distribution` (an instance dict, or a read-only mapping that refuses a write); a law's
  `params` as a value with `with_params` (the set pattern); `BoxSet.sample` reading `params`;
  MPPI's tick key on the solve's time rather than the planner's `dt`; the cipher's 24-bit normal
  tails and crc32 stream names stated in randomness.md; `function_identity` goes with the cache.

## 9. The order

| Step | Section | Why here |
| --- | --- | --- |
| 1 | §1 the cache out | touches every compile path; everything after builds on the clean one |
| 2 | §2 params everywhere | the fields first, then the audit; §3 and §6 need the trees |
| 3 | §3 one tree per planner | needs §2's audit of the planners |
| 4 | §4 the one score's edges | independent of the grammar; small, three ledger lines |
| 5 | §7 the boundary dials | before §5 and §6 rewrite the demos, so each demo line moves once |
| 6 | §5 the loop's grammar | the largest; the demos' loop lines move here |
| 7 | §6 one MPC architecture | on the new grammar; the MPC demos move here, once |
| 8 | §8 the details | wherever they fit, each its own row |
| 9 | the records | the dated log; DESIGN, ROADMAP §3, TODO, core-architecture updated in place; QUESTIONS.md emptied of the ruled items; ROADMAP §2's breaking-changes entry drafted for the maintainer |

**Number changes, each a ledger line:** the RL reward (§4.1), the cost bound (§4.2), value
iteration's `alpha` on a discounting problem (§4.3), the delayed-form MPC demos (§6), the demos
whose `x_goal` becomes a hard `Xf` (§7). Everything else byte-identical.

**[ask] left for the maintainer, in the log:** the sentence for ROADMAP §2 (MPPI provisional; the
breaking-changes ledger entry for the two runs and this consolidation); which demos drop `x_goal`
and which keep it as `Xf` (the agent proposes per demo, the maintainer confirms); the cart-pole
demo's budget once the reward is re-measured.
