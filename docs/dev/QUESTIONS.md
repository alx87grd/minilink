# Open design questions for the maintainer

The design questions that wait for the maintainer: the architecture, the API and its naming, the
vision, and what affects performance. Details, versions, process, tests and demos are not here:
they sit in their workboard rows, and the rows that wait for the maintainer's go are listed at
the head of [TODO.md](../plans/TODO.md#rows-waiting-for-the-maintainers-go). Known defects are in
[KNOWN_ISSUES.md](KNOWN_ISSUES.md).

A ruling the maintainer takes updates the doc it changes (DESIGN, ROADMAP, TODO or the plan
doc it shapes) in the same change; the commit message carries its why. It then leaves this
file. The core's architecture is of record: the v0.3 rulings and the v0.4 rulings of
2026-11-04 to 2026-11-08 live in DESIGN and RULES. Nothing on the core is open here; one question on the vision is.

## The vision

### Where is the fair comparison stated?
- **Context:** the maintainer's direction of 2026-11-09 (ROADMAP §5): one problem handed to
  every planner and one judge for every answer is the toolbox's distinctive contribution, to be
  promoted in the vision. Today CONSTITUTION §1.1 names the three foundations and calls every
  approach "a tool on that path", without saying the approaches are compared on it; the README
  shows it in §7 of 9, after the model, the wiring, the graphics, JAX and the optimizer.
- **Proposed wording, CONSTITUTION §1** (a new item after "One toolbox, one model"):
  "**One problem, one judge.** Every approach takes the same `PlanningProblem` and returns a
  `PlanningSolution`; every answer is scored by the same evaluation, on the same model, cost,
  sets, start law and disturbances. A method is never scored by a judge of its own."
- **Options:**
  1. *CONSTITUTION §1 and the README's lead.* The item above, and the README's first example
     the comparison table. Costs: the README leads with the advanced tool, before a student's
     first simulation.
  2. *CONSTITUTION §1 only, the README's order kept.* The item above; README §1's backbone
     paragraph gains one sentence and §7 is fixed. The teaching order stays.
  3. *A governing principle (CONSTITUTION §4) instead of the vision.* "One judge for every
     planner" beside "One set of tools for every System": a rule for reviews, not an identity.
- **Recommendation:** option 2 with the principle of option 3 as well: the identity in §1, the
  review rule in §4, and the README's teaching order kept until its example wins.
- **Blocks:** nothing in the code; the README's wording waits on it.
- **Answer:**

## Toward v1.0

### How does a public name change once v1.0 is frozen?
- **Your answer (2026-10-27):** not a `DeprecationWarning` shim for one minor release, as
  proposed; the policy stays open. No alias, one sweep, holds meanwhile.
- **Your answer (2026-11-03):** the pending renames land before the freeze, one sweep per
  group, under the vocabulary of RULES 2.7; so the freeze itself needs no shim.
- **Blocks:** the trust contract (V4), for the names changed after v1.0.
- **Today:** no alias or shim ever. The frozen names are drawn at v0.9 from RULES 2.7.
- **Options still open (after v1.0 only):** a shim of some length; a major version per rename;
  a snapshot test of the export list either way.
- **Detail:** [TODO, V4](../plans/TODO.md#v09--the-trust-contract)
- **Answer:**
