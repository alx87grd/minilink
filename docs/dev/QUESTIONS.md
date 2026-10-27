# Open design questions for the maintainer

The design questions that wait for the maintainer: the architecture, the API and its naming, the
vision, and what affects performance. Details, versions, process, tests and demos are not here:
they sit in their workboard rows, and the rows that wait for the maintainer's go are listed at
the head of [TODO.md](../plans/TODO.md#rows-waiting-for-the-maintainers-go). Known defects are in
[KNOWN_ISSUES.md](KNOWN_ISSUES.md).

Answers are recorded as **preliminary choices**, open to revision when the final architecture is
drafted, in a dated record under [docs/reviews/](../reviews/) (the latest:
[2026-10-27](../reviews/2026-10-27-design-questions-rulings.md), R124–R139), in the plan doc each
shapes, and in TODO; the entry then leaves this file. `§N` names the workstream of
[core-architecture.md](../plans/core-architecture.md).

## The core

### One score: how is a stepping loop evaluated? (an investigation for the final design)
- **Your note (2026-10-27): a core part of the final design, to investigate before a choice.**
  Sets and costs take time as an argument: does a discrete problem need discrete versions of
  them, or do they bind `t_k = t₀ + k·dt`, and how does a natively discrete cost `c(x_k, u_k, k)`
  fit? Should the evaluation of a continuous system be done exactly as a `StepSystem` on its
  `dt`, so that the trapezoid becomes a plain sum? Investigate the cleanest option, with its pros
  and cons.
- **Your idea (2026-10-26, draft; a core pillar):** once `dt` and the scheme are set, discretize
  the problem (time becomes an index in the cost and the sets) and score every stepping loop with
  one discrete evaluator, so a policy has one `J`.
- **Today:**
  - one policy gets three or four close `J`s: a left Riemann sum `g dt` in value iteration and
    RL, a trapezoid in `score_run`, shooting's own trapezoid with a first-order hold;
  - the failure rules differ: `score_run` integrates to `tf` and checks `X` and `U` at every
    sample under the price switch; the RL environment stops at the first exit from `X` and
    checks no `U`; value iteration prices the cells that leave its grid and ignores a finite `tf`
    (S63, v0.4); the feedback planners report their held-input rollout (OD-27).
- **The points to settle together:**
  - the per-step cost: integrate the cost with the state (exact for the scheme; under forward
    Euler it is today's `g·dt`, so value iteration's and RL's numbers do not move), or a
    quadrature on the samples;
  - the object: a new noun (`StepPlanningProblem` from `problem.discretize(dt, ...)`), the same
    `PlanningProblem` whose `sys` is a `StepSystem`, or an internal lowering held by the
    evaluator;
  - time-varying sets and costs, and the margins between samples;
  - one failure rule (absorbing or not) and the zero-order hold; shooting's reported score
    lowered to it or kept; planner policies acting at a fixed `dt` scored through the sampled
    face;
  - the discount, `γ = e^{−ρ dt}`.
- **Reviewers:** all three: integrate the cost with the state; `score_run`'s rule, to the
  horizon, the RL early stop a training choice that reports the score's `J`; the zero-order hold;
  shooting's reported score lowered; S63 in v0.4. A and B: no new noun, the same
  `PlanningProblem` on a `StepSystem`; C: internal first, and after the loop lane.
- **Detail:** [differentiable-problem.md](../plans/differentiable-problem.md) §9 (its six points
  in §9.4); [core-architecture.md](../plans/core-architecture.md) §6
- **Answer:**

## Toward v1.0

### How does a public name change once v1.0 is frozen?
- **Your answer (2026-10-27):** not a `DeprecationWarning` shim for one minor release, as
  proposed; the policy stays open (R139). R82 (no alias, one sweep) holds meanwhile.
- **Blocks:** the trust contract (V4); the renames that wait for the freeze (`xbar` / `ubar` →
  `x_bar` / `u_bar`, `plot_cost2go` → `plot_cost_to_go`, `NeuralPolicyController` →
  `NeuralPolicy`).
- **Today:** no alias or shim ever (R82). The frozen names are drawn at v0.9 from one rule (R139).
- **Options still open:** do the pending renames before the freeze, so none is needed after it;
  a shim of another length; a major version per rename; a snapshot test of the export list
  either way.
- **Detail:** [TODO, V4](../plans/TODO.md#v09--the-trust-contract)
- **Answer:**
