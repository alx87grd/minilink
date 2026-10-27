# Open design questions for the maintainer

The design questions that wait for the maintainer: the architecture, the API and its naming, the
vision, and what affects performance. Details, versions, process, tests and demos are not here:
they sit in their workboard rows, and the rows that wait for the maintainer's go are listed at
the head of [TODO.md](../plans/TODO.md#rows-waiting-for-the-maintainers-go). Known defects are in
[KNOWN_ISSUES.md](KNOWN_ISSUES.md).

Answers are recorded as **preliminary choices**, open to revision when the final architecture is
drafted, in a dated record under [docs/reviews/](../reviews/) (the latest:
[R124–R140](../reviews/2026-10-27-design-questions-rulings.md) and
[R141–R153](../reviews/2026-10-27-core-vision-decisions.md), 2026-10-27), in the plan doc each
shapes, and in TODO; the entry then leaves this file. `§N` names the workstream of
[core-architecture.md](../plans/core-architecture.md).

## The core

### One score: the three points left (an investigation for the final design)
- **Decided (2026-10-27, preliminary; [R141, R142, R150](../reviews/2026-10-27-core-vision-decisions.md)):**
  two problem classes, `PlanningProblem` and `StepPlanningProblem`; a problem's step face is
  compiled, `problem.compile(dt=)` giving `transition(x, u, k) -> (x_next, c_k)`, `N` and
  `γ = e^{−ρ dt}`; the stage cost integrated with the state in the stationary form; the RL
  discount derived from the problem. Every stepping tool reads that one face.
- **Still open, the points to settle together:**
  - **time-varying sets and costs, and the margins between samples:** they read `t`; the step face
    evaluates them at `t_k`, so a violation between two samples is not seen. Options: accept
    the sampled check (the score is of the sampled loop); check at the substeps the dense run
    already computes (R147); or a margin integrated over the period like the cost;
  - **one failure rule and the input hold:** `score_run` integrates to the horizon and checks `X`
    and `U` at every sample; the RL environment stops at the first exit and checks no `U`; value
    iteration prices the cells that leave its grid. Options: absorbing (the first exit ends the
    run and pays the price, the RL rule) or to the horizon (today's judge, every violation
    priced); the hold is the zero-order hold of `transition` (the reviewers' view) unless a
    planner states otherwise;
  - **shooting's reported score:** shooting reports its own trapezoid with a first-order hold.
    Options: lower it onto `transition` (one `J`), or keep it as the transcription's own
    objective, reported beside the judge's.
- **Reviewers (2026-10-27):** to the horizon, the RL early stop a training choice reporting the
  judge's `J`; the zero-order hold; shooting's score lowered; value iteration on a finite horizon
  (S63) in v0.4.
- **Detail:** [differentiable-problem.md](../plans/differentiable-problem.md) §9;
  [core-architecture.md](../plans/core-architecture.md) §6
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
