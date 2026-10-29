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

The v0.3 core's architecture is of record (2026-10-28, R166); one question came from the
night run's review (2026-10-29).

### Should a field's numbers be leaves of the set's and the cost's tree?
- **Today:** a field's wrappers read the field's own parameters, never the tree they are given
  (`FieldSet.margin`, `FieldInputSet.margin`, `FieldCost.g`, `FieldTerminalCost.h` call
  `field.value(...)` without `params`; fields.md says "the field reads its own parameters").
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
- **Detail:** [the night run's log](../reviews/2026-10-29-night-run-log.md), N-19.
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
