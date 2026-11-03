# Open design questions for the maintainer

The design questions that wait for the maintainer: the architecture, the API and its naming, the
vision, and what affects performance. Details, versions, process, tests and demos are not here:
they sit in their workboard rows, and the rows that wait for the maintainer's go are listed at
the head of [TODO.md](../plans/TODO.md#rows-waiting-for-the-maintainers-go). Known defects are in
[KNOWN_ISSUES.md](KNOWN_ISSUES.md).

A ruling the maintainer takes updates the doc it changes (DESIGN, ROADMAP, TODO or the plan
doc it shapes) in the same change; the commit message carries its why. It then leaves this
file. The v0.3 core's rulings are the architecture of record. `§N` names the workstream of
[ROADMAP §5.3](ROADMAP.md#53-v03--the-complete-core), the same N as TODO §1.N.

## The core

The v0.3 core's architecture is of record. The questions the independent
review of the two runs raised were ruled on 2026-10-29; they landed on
`dev-v0.3` the same day. Its run's decisions taken without you, and those of the three runs before it, are sorted
for one sitting in the merge-review triage. Nothing else is open here.

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


### Does a planner keep its last solution?
- **Why it is asked:** the solution is the answer (CONSTITUTION §2: a policy and its evidence),
  yet a planner also keeps the last one it returned. The removal was planned with the planning
  demos' move to the solution's verbs (TODO, D1.4); you held it on 2026-11-03 for more analysis.
- **Today:** `Planner.solve()` stores `self.last_solution` (`planning/planner.py`).
  `get_controller()` returns `last_solution.policy`, `plot_solution()` plots it, and RRT's
  `plot_tree()` draws its path. The planner's own internals are a separate state the plots
  need: the RRT tree, the value-iteration tables (`plot_cost2go`, `plot_policy`), the RL
  learning curve.
- **Evidence (counted on `dev`):** `planner.get_controller()` in 22 example and 22 test lines,
  `solution.policy` in 18 and 33; planner-side plots (`planner.plot_*`) in 55 example lines,
  solution-side plots in 5. `last_solution` itself is read in 1 example and 5 tests.
- **What it costs:** a second owner of the answer. A stored solution goes stale when the
  problem is changed after the solve, and a shared mutable object is how the RL planner gave
  every solution one live law (fixed 2026-11-03 by copying the law into each solution). The
  compiled face (`planner.compile()`) and the MPC keep no last solution.
- **What it buys:** the one-object student flow, `planner.solve()` then
  `planner.plot_cost2go()` and `planner.get_controller()`, which the course notebooks use.
- **Options:**
  1. Keep it as it is.
  2. Keep it as a read-only convenience: every plot and the policy live on the solution, and
     the planner's methods call `self.last_solution.<verb>`; the planner keeps only what is
     its own (the tree, the tables, the learning history).
  3. Remove it: `solution = planner.solve()` is the one way; the plots of the planner's own
     internals stay on the planner.
- **Detail:** [TODO, the compiled face beyond trajectory optimization](../plans/TODO.md)
- **Answer:**
