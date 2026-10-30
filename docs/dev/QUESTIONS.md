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

The v0.3 core's architecture is of record (2026-10-28, R166). The questions the independent
review of the two runs raised were ruled on 2026-10-29 (R168–R179,
[the record](../reviews/2026-10-29-consolidation-rulings.md)); their design is
[the consolidation plan](../plans/consolidation-after-the-runs.md), landed on `dev-v0.3` the same
day. Its run's decisions taken without you, and those of the three runs before it, are sorted
for one sitting in [the triage](../reviews/2026-10-30-merge-review-triage.md); its "To raise"
points come first there. Nothing else is open here.

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
