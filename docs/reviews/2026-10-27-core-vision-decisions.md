# The consolidated core vision, decision by decision (2026-10-27)

**Audience:** Prof. Alexandre Girard (Maintainer)
**Context:** the one-by-one review of the fourteen decisions of
[the consolidated core vision](2026-10-27-consolidated-core-vision.md) §8. Each entry records the
maintainer's choice and how it was reached.
**Status: preliminary choices,** with the same caveat as
[R124–R140](2026-10-27-design-questions-rulings.md): open to readjustment when the final
architecture is drafted.

## R141 (2026-10-27, preliminary, amended the same day) Two kinds of problem, sibling classes; the step face compiled, the discretization explicit

Decision 1 of the vision. The vision recommended one `PlanningProblem` on either kind of system,
a natively discrete task's cost "reading `t` as the tick". The maintainer asked for one global
picture that holds when a problem is natively discrete (a chess game, a cost of the index, no
`dt`), as systems already have three kinds: continuous, step, and sampled.

- **Two kinds, mirroring the systems.** `DynamicSystem` and `StepSystem` are sibling classes and
  `Discretization(StepSystem)` is a step system that keeps its continuous `source` and `dt`. The
  problems follow the same shape:
  - `PlanningProblem`: the continuous problem, unchanged (`tf`, `discount_rate`, a
    `CostFunction` whose `g` is a rate);
  - `StepPlanningProblem`: a problem on a `StepSystem`, with a horizon `N`, a discount per step
    `gamma`, and a `StepCost` whose stage cost `c(x, u, k)` is per step and `h(x, k)` terminal;
  - "sampled" is not a third kind: it is a step object derived from a continuous one,
    keeping its source and `dt`.
- **The problem stays a description; the step face is compiled** (amended the same day, the
  maintainer: "the problem should be kept clean"). As a `System` holds `f` and its compiled
  evaluator holds the step model (`ev.step(x, u, t, dt=, solver=, substeps=)`), a problem holds
  `g`, `h`, its sets and horizon, and compiling it gives the step face:
  - `problem.compile(dt=, solver=, substeps=)`, with no controller (new, open loop):
    `transition(x, u, k, params) -> (x_next, c_k)`, `N = tf/dt`, `gamma = exp(-discount_rate *
    dt)`, the sets at `t_k = k·dt`, the price;
  - `problem.compile(controller, dt=, ...)` (today's `ProblemEvaluator`, closed loop): its run is
    one scan of the same `transition`, `u_k` from the controller, so `evaluate`, `program` and
    the Monte Carlo score through the same `c_k`;
  - a `StepPlanningProblem` compiles to the same face, with no `dt`;
  - value iteration, the RL environment, the judge, shooting and the native MPC read only this
    face and cannot tell a native problem from a sampled one. Collocation, the continuous LQR
    and the linear analysis read the continuous description.
- **The explicit discretization, as for systems** (the maintainer: "useful, and it matches the
  system pattern"): `discretize(problem, dt, *, solver=, substeps=)` beside `discretize(sys, dt)`
  returns a `StepPlanningProblem` on `Discretization(plant, dt)` with `N`, `gamma` and the sets
  at `t_k`; `problem.discretize(dt)` is its facade (CONSTITUTION §4.3). Compiling the
  discretization gives the same face as `problem.compile(dt=...)`, to the bit: one test pins
  it. (`System` has no `discretize` facade today; adding `sys.discretize(dt)` for symmetry is a
  naming detail for the names sweep.)
- **A sampled problem has no cost object of its own:** its `c_k` needs the plant's flow over the
  period, so `transition` produces it (R142); the discretized problem's cost is that transition's
  cost row, not a free-standing `StepCost`. `StepCost` is written for native step problems.
- **What students meet is unchanged:** the course notebooks keep building a `PlanningProblem`;
  a stepping planner compiles it at its `dt` itself.
- **Open, internal:** whether the open-loop face is `ProblemEvaluator` with no controller or a
  smaller sibling the closed loop builds on; settled when the code lands.
- **Shared parts by composition:** the sets, the start, the parameter tree and `realize` are
  composed the same way in both classes; a private mixin only if the duplication shows in code.
- **Independent of decision 3** (stochastic by its parts): together, two problem classes.
- **Why not one class:** the step world is a sibling algebra (CONSTITUTION invariant 1), and one
  class with `tf` or `N`, `discount_rate` or `gamma`, `CostFunction` or `StepCost` would
  complicate the continuous problem for the discrete case (precedence 3).

## R142 (2026-10-27, preliminary) The stage cost computed in the transition, in the stationary form

Decision 2 of the vision, taken as recommended.

- `c_k` is integrated with the state inside the step (one plant evaluation per step), not by a
  post-pass over a recorded run.
- The stationary form: `c_k = ∫₀^dt e^{−ρτ} g dτ`, `J = Σ γ^k c_k + γ^N h`, so `c_k` does not
  depend on `k` and value iteration's backup `c + γ J'` reads it as is.
- Under forward Euler this is today's `g·dt` of value iteration and RL, to the last bit; the
  judge's trapezoid `J` changes once (an intended number change, R80). A logged run scored by
  `evaluate_trajectory` keeps the trapezoid and says so.

## R143 (2026-10-27, preliminary) A problem is stochastic by its parts; `StochasticPlanningProblem` goes

Decision 3 of the vision, taken as recommended.

- **No stochastic class.** Where a number goes, a law may go:
  - `x_start=` takes a point, a `Set` or a `Distribution` (R124's three kinds of start);
  - `params=` takes laws in the tree's own shape (`{"plant": {"m": Uniform(0.9, 1.1)}}`), not
    flat names;
  - disturbances are source blocks (R128).
- `x0_distribution=` and `params_distribution=` go; `criterion=` and `nominal()` (each law
  replaced by its mean) move onto the class; `problem.is_stochastic` is read off its parts.
- The same rule holds on `StepPlanningProblem` (R141): two problem classes in all, not four.
- **Why:** R141's two classes are justified by different fields (horizon, discount, cost kind);
  a stochastic problem has the same fields, some of them random, so a class would add nothing.
- **Student-facing [ask — student-facing, given]:** the four RL demos and ten notebooks that
  write `StochasticPlanningProblem(..., x0_distribution=...)` (the GRO860 RL notebooks,
  tutorial 11, the showcases) are swept once, every caller in the same commit, no alias (R82;
  the name freeze is paused between terms).

## R144 (2026-10-27, preliminary) `realize` returns one tree; the start is a leaf of the problem's parameters

Decision 4 of the vision, taken as recommended. The maintainer: "x0 should be part of params of
a planning problem".

- **The start is a parameter of the problem.** `problem.params` already holds it as the
  `"x_start"` leaf beside `"plant"`, `"cost"`, the sets and `"discount_rate"`
  (`planning/problems.py`, the `params` view); with R143 that leaf may hold a law.
- **`problem.realize(key, n_trials) -> params`:** one tree, `params["x_start"]` of shape
  `(n, nx)`, every drawn leaf and every seed with a trial axis, the shared leaves unchanged. The
  judge reads the start from the tree like any leaf, so `pe.evaluate(params={"x_start": starts})`
  and a realized tree mean the same thing.
- **Revises R128's wording only** (its pair `(x0s, params)`); its substance stays: disturbances
  are source blocks, their seeds leaves of the tree. Today's internal `realize` returns
  `(x0s, {"params": ..., "disturbances": ...})` and changes with the draws lane.
- **The name stays `x_start`** (R124): `x0` is a block's own default start, `x_start` the
  problem's start of the plant.
- **What `realize` writes, by kind** (a clarification the maintainer asked for and accepted):
  - **a law in a leaf** (`x_start`, a plant parameter): drawn **to a value**, once per trial, from
    a sub-key derived by the leaf's path (`child(key, i, "x_start")`); the realized tree holds
    numbers, never the law or a seed, so the start stays differentiable and inspectable;
  - **a source block** (a noise block, the seeded policy): given **an integer seed** leaf
    (`p["d"]["seed"][i]`); the block draws during the run from its seed and the tick;
  - both from the one key, by name (randomness.md, one stream per consumer): adding a law leaves
    the others' draws unchanged. Today's positional `split_keys` and the held disturbance value
    change with the draws lane (RN-4, RN-5).

## R145 (2026-10-27, preliminary) `%` is the shortcut of `discretize`, generic for any system, Euler by default

Decision 5 of the vision. The maintainer: "`%` should just be a shortcut to the discretize
method, generic for any system; Euler by default is fine, this shortcut being mostly for the
controller side".

- **`sys % dt` is `discretize(sys, dt, solver="euler")`**, for every kind of system:
  - a `DynamicSystem` with states (a PID, a lead-lag, an observer) becomes its difference
    equation, `z_{k+1} = z_k + dt·f(z_k, e_k)`, the one a GRO501 student writes for the Arduino;
  - a static block and a `StepSystem` are tagged with their period.
- **`discretize(sys, dt)` keeps `rk4`** as its own default: the shortcut is for implementing a
  law, the function for simulating a plant. Another scheme is the function's `solver=`
  (`discretize(pid, 0.01, solver="rk4")`); `%` takes none.
- `sys.discretize(dt)` is the method facade of the function (R141's note), so `%` reads as its
  operator form.
- No number moves: `%` refuses a law with continuous states today
  (`core/discrete/step_diagram.py`, `require_step_subsystem`). Tustin is not part of it; a later
  feature row if wanted.
- **Depends on decision 6**, the multi-clock computer: what `%` returns inside a computer (a
  block carrying its period, or a one-block `Computer`) is settled there.

## R146 (2026-10-27, preliminary) Periods on blocks; one `SampledSystem`, a system on a clock; no `Computer`, `StepSchedule` or `HybridDiagram`

Decisions 6 and 7 of the vision, with the maintainer's refinements: a pure step system carries no
`dt` ("it would force chess to have `dt = None`"), a sampled block is a child of `StepSystem`,
and `discretize` returns it directly, with no intermediate class.

- **The tree:**

  ```text
  StepSystem ─── StepDiagramSystem         pure tick: step(x, u, k); chess, grid worlds; no clock
    └─ SampledSystem                       any system on a clock; discretize and % return it
        └─ SampledDiagramSystem            a diagram with a clocked block: the computer, and computer @ plant
  ```

- **A pure `StepSystem` stays clean:** no `dt`, no `source`. A native z-domain design is one.
- **`SampledSystem(source, sample_period, *, solver="rk4", substeps=1)`, "this system on this
  clock":**
  - `source` is always set: a continuous system (its `step` integrates `source.f` over the
    period, the input held), a static block (its `h` sampled) or a pure `StepSystem` (its `step`
    delegated);
  - `sample_period` is a structural attribute, not a parameter: it sets the divisors, the base
    period and the tick count, so it is fixed at compile (a sweep, not a gradient);
  - `solver` and `substeps` apply to a continuous source; a step source refuses them;
  - **a view, not a container:** its `params` are the source's own dict, live and not nested,
    its ports, state and labels the source's; a parent's tree holds it as one block;
  - nobody subclasses it: physics is written as a `DynamicSystem`, a native law as a
    `StepSystem`, and the clock is always applied from outside;
  - `discretize(sys, dt)` (RK4) and `sys % dt` (Euler, R145) return it and leave the source
    unchanged.
- **`Discretization` is renamed `SampledSystem`** (no alias, R82; its callers `dp.py`,
  `policy_eval.py`, the trajectory-optimization planner and one test file, none student-facing);
  its `dt` becomes `sample_period`; `ExactDiscretization` folds in as `solver="exact"` or a
  private subclass.
- **Periods on blocks; the schedule derived** (decision 6, option a): a diagram with any clocked
  block is a `SampledDiagramSystem`, its base period the greatest common divisor of its blocks';
  a block of divisor `d > 1` fires on `k mod d = 0` and its held outputs are state slots; a period
  not an integer multiple of the base is refused at wiring; a pure step block inside runs at the
  base tick. A multi-rate computer is ordinary composition, `(filt % 0.01) >> (ctl % 0.1)`. The
  MPC's two rates are `mpc % 0.05` beside a `% 0.01` block, so `replan_every` goes.
- **`Computer` and `StepSchedule` go** as core classes; "computer" stays the word of the runner
  (realtime, hardware in the loop) and of the drawing's box.
- **The operators** (R126): `%`, then `>>`, `+`, `@` between step systems; `sampled @ continuous`
  is the one place the two worlds meet, the plant becoming `discretize(plant, base, substeps=n)`
  under `plant`; every other mix refused, naming `%`.
- **The loop needs no class of its own** (decision 7): `computer @ plant` is a
  `SampledDiagramSystem` whose plant block is a `SampledSystem` of the continuous plant; the dense
  and adaptive runs reach the plant through its `source`. `HybridDiagram` (not a `System` today)
  goes. Revises R127's "named `HybridDiagram`".
- CONSTITUTION invariant 1 is amended when it lands (the sampled loop a step diagram of the
  sibling algebra, as R127 already names).

## R147 (2026-10-27, preliminary) Two run verbs that agree: the ticks and the same scan with its substeps kept

Decision 8 of the vision, option (a), with the maintainer's question "how would the substeps be
recorded?" answered by one evaluator method.

- **The verbs, by what an object has underneath:**
  - `compute_rollout` on every `StepSystem`: the ticks, a `Trajectory` (`StepRollout` folded, §2),
    `t = k` on a pure step system, `t = k·Δ` on a sampled one; what `evaluate` scores and
    `jax.grad` differentiates;
  - `compute_trajectory` wherever continuous time is underneath: a `DynamicSystem` (unchanged), a
    `SampledSystem` or `SampledDiagramSystem` with a continuous source (the loop,
    `discretize(plant)`); refused on a pure step system and on a clocked step source.
- **The dense run is the same scan, its substeps kept.** `step_body`
  (`core/compile/evaluators/stepping.py`) already computes the plant at `t_k + j·Δ/substeps` and
  keeps only `x(t_k + Δ)`; the dense path keeps every substep end, so its value at each tick is
  the tick value, bit for bit. The input is the held staircase; the outputs are evaluated at
  every substep point; the RK4 stage slopes are not recorded.
- **One new evaluator method beside `rollout`:** `rollout` keeps its one shape `(N+1, n)`; the
  new method (name for the names sweep: `rollout_path`, `rollout_dense`, …) returns
  `(N·substeps+1, n)` from one shared period body, on the system and the sampled-diagram
  evaluators alike. `compute_rollout` calls `rollout`, `compute_trajectory` the new method; user
  code never calls them directly. No flag changing `rollout`'s return shape.
- **The substep count is structural, on the loop's plant `SampledSystem`**, defaulting from the
  automatic-`dt` rule (`AUTO_DT_SCALE × τ_min` against the base period): `compute_rollout`,
  `compute_trajectory` and `evaluate` integrate with the same count, one run.
- **The cross-check:** `compute_trajectory(solver="scipy")` is the clock runner with an adaptive
  plant between the held inputs (NumPy), agreeing within its tolerance. The loop re-clocked at
  `Δ/substeps` (the computer's divisor times `substeps`) is the dense method's test oracle, to
  `rtol 1e-12`.
- `x0_plant=` and `x0_computer=` go (R124's views: one `x0`, written through to each block).
- The hybrid demos and tutorial 06 re-pin once, with the same-tick semantic (R125) and the MPC
  delay (R123). Revises R127's default runner of `compute_trajectory`.

## The one-score question

R141 and R142 answer its object (the compiled step face) and its per-step cost. Still open
there, for the review of the remaining decisions: time-varying sets and the margins between samples, the one failure rule and
the hold, shooting's reported score.
