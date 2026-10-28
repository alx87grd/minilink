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

The v0.3 core's architecture is of record (2026-10-28, R166), with one question your ask of
2026-10-28 opened (the demos keep the old MPC's performance):

### The zero-delay MPC: a stored solve per tick, and which warm start?
- **Blocks:** the MPC demos' numbers on `dev-v0.3-mpc-nodelay` (`delay=False`), and whether
  R123 is amended.
- **Today (on that branch):** `ModelPredictiveController(planner, delay=False)` is the old
  MPC: zero delay, warm-started from the last plan shifted one period with its first state
  knot at the measurement. Its command and its update both need the tick's solve, and the
  run's read-backs (the plant's input, the overlays) need it again, so the block keeps its
  solves by `(t_k, z_k, P_k)` on the host: one solve per tick, as the old latch did. R123 had
  removed that latch ("no hidden cache"). The warm start is the correct one-period shift; the
  old code's shift had a timing slip (its guess re-timed by `k·Δ` on a plan timed from 0, so
  after `tf/Δ` ticks it was the last plan's next-to-last knot held constant). That guess,
  replayed, gives the old runs to the bit (the integrator demo exactly, the kinematic racecar
  to 1.8e-14); the correct shift drives 5–8 % less distance on the racecars and the slalom
  at their loose SLSQP `ftol`, and is faster, and better on the circuit (no stall).
- **Options:** (a) keep the stored solve and amend R123 for the zero-delay form; (b) R123's
  option B: a combined `update → (x_next, y)` face and runs that record their signals in the
  scan, a core evaluator change, then no store. For the warm start: the correct shift (with
  the demos' `ftol` retuned by you where the distance matters), or the old guess replayed for
  bit-equal demo numbers.
- **Detail:** [planner-and-mpc.md §6.3](../plans/planner-and-mpc.md), the evidence in
  [the record](../reviews/2026-10-28-zero-delay-mpc.md).
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
