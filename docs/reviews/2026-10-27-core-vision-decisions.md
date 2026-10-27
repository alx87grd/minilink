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

## The one-score question

R141 and R142 answer its object (the compiled step face) and its per-step cost. Still open
there, for the review of the remaining decisions: time-varying sets and the margins between samples, the one failure rule and
the hold, shooting's reported score.
