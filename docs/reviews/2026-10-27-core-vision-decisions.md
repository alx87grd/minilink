# The consolidated core vision, decision by decision (2026-10-27)

**Audience:** Prof. Alexandre Girard (Maintainer)
**Context:** the one-by-one review of the fourteen decisions of
[the consolidated core vision](2026-10-27-consolidated-core-vision.md) §8. Each entry records the
maintainer's choice and how it was reached.
**Status: preliminary choices,** with the same caveat as
[R124–R140](2026-10-27-design-questions-rulings.md): open to readjustment when the final
architecture is drafted.

## R141 (2026-10-27, preliminary) Two kinds of problem, sibling classes; "sampled" is a step problem that keeps its source

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
  - "sampled" is not a third class: `problem.discretize(dt, solver=, substeps=)` returns a
    `StepPlanningProblem` on `Discretization(plant, dt)`, with `N = tf/dt`,
    `gamma = exp(-discount_rate * dt)` and the sets at `t_k = k·dt`.
- **The one face every stepping tool reads:** `transition(x, u, k, params) -> (x_next, c_k)`,
  `N`, `gamma`, the sets at the index, the price. Value iteration, the RL environment, the judge,
  shooting and the native MPC read only this face and cannot tell a native problem from a sampled
  one. Collocation, the continuous LQR and the linear analysis take the continuous class.
- **A sampled problem has no cost object of its own:** its `c_k` needs the plant's flow over the
  period, so `transition` produces it (R142). `StepCost` exists for native step problems.
- **What students meet is unchanged:** the course notebooks keep building a `PlanningProblem`;
  a stepping planner calls `discretize(dt)` itself.
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

## The one-score question

R141 and R142 answer its object and its per-step cost. Still open there, for the review of the
remaining decisions: time-varying sets and the margins between samples, the one failure rule and
the hold, shooting's reported score.
