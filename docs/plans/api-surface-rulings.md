# The v0.4 API surface: nine rulings to build

The maintainer ruled on nine proposals from a scan of the public API (2026-11-05), each
taken one by one with its context, before and after, and options. Each removes or merges
names a user must learn: about twenty fewer in all. The maintainer's review pass of 2026-11-05 went
through the nine one by one and amended AR-4 to AR-9 in place (each says so). None is built yet.
This doc is the starting point for the branch that builds them; the order of work and "done when" sit in
[TODO.md](TODO.md) §1.11 ("The API surface rulings"). Once a ruling lands, its contract moves
into [DESIGN.md](../dev/DESIGN.md) and its section here is deleted; the doc goes with the last.

The rulings are numbered `AR-1`–`AR-9` (local to this doc). Two rulings taken the same day
already landed and frame them: the one parameters model (DESIGN §4 "Parameters") and "Minilink
keeps no cache" (DESIGN §4 rule 6): `planner.solve()` and the other convenience verbs build every
call, and the fast route for repeated calls is a held tool (`pv = planner.compile()`), which never
compiles for new numbers.

## How to build them

- One branch off `dev` (never on `dev`), one commit per ruling; each commit carries its ruling
  into DESIGN, its callers (library, tests, examples, notebooks, benchmarks) and, when a user
  could still hit the old name, a deprecation-free removal: Minilink has no aliases (the
  maintainer ruled "no alias for now").
- Before each commit: `benchmarks/params_baseline.py` byte-identical against `dev` (an intended
  numeric change gets its own test instead), ruff, the touched modules' tests and
  `tests/unittest/test_vocabulary.py` (add each retired name to its list).
- Each commit sweeps the docs for its retired names (`grep` over `docs/` and `README.md`): the
  README is clean today, but RULES 2.7's `solver=` row names `compute_forced`, KNOWN_ISSUES's
  reproducer `BoxInputSet.from_bounds`, TODO's CA-3 row `trajectory_of`, and DESIGN about
  fourteen lines.
- On the finished branch, once: the full `pytest -n auto`, the demo-check bridge on its own, the
  regression gates (CI flags), the notebook smoke check, and the docs build when docstrings moved
  (AGENTS.md, "Before push or PR").
- Student-facing edits are covered by these rulings and limited to the renamed calls: the core
  tutorials 01, 06 and 08, the showcase notebooks, and one call each in three course notebooks
  (GRO501 racecar for AR-7, GRO860 pendulum for AR-7, GRO860 drone PPO for AR-8). Nothing else in
  a course notebook changes; tuned constants and commented-out cells stay as they are.
- Hand the branch to the maintainer for review before any merge.

Call-site counts below are files per tree at 2026-11-05 (`grep`), a guide for the size of each
commit.

## AR-7 One block selector: `traj.of("plant")` on both loop kinds

`traj.of("plant")` and `compute_cost(cost, of="plant")` work on a continuous loop as on a
sampled one (today the continuous one refuses both). A continuous run always records every
block's state slice under its role; a block's input is recorded only when asked,
`record=("plant:u",)`. `diagram.trajectory_of` goes (it replaces part of CA-3 in
[core-api-improvements.md](core-api-improvements.md): update that section). Callers: 8 notebook
lines, including one call each in the GRO501 racecar and GRO860 pendulum notebooks.

Its companion, **a continuous run records its signals from its own pass**, is its own row and
branch (split 2026-11-05, the maintainer's review): a behaviour-preserving restructure of the
simulator's pass with its own baseline and a measurement-gated default, not an API trim, so it does
not hold this branch. Its design and benchmark plan are in TODO §1.11 ("A continuous run records
its signals from its own pass"); this ruling lands without it, on today's recording path.

## What the branch also carries (the maintainer's picks, 2026-11-05)

Small rows on the same lines as the rulings, bundled so one run lands them; each keeps its own
commit and its own "done when":

- **Agent lane, no ask:** the planner's `supports_parametric` probe (the textbook pass's planning
  row, T5) goes with AR-9's plumbing; `analysis/derivatives.py`'s bare `from jax.errors` becomes
  `require_jax()` (a D3 row) in AR-5's commit; the RL planner sizes its default `NeuralPolicy` from
  `problem.U.bounding_box()` and refuses an unbounded `problem.U` (KNOWN_ISSUES; the V1.2 rest row
  of TODO §2) beside AR-3.
- **Ruled to bundle:** `PlanningProblem.start_point`, a second name for the `x0` leaf, goes (9
  call sites; the start is a number of the tree, AR-9's spirit); `diagram.of("plant")` and string
  ids in `subsystem_id` (CA-3's remainder once `trajectory_of` goes with AR-7); the program
  evaluator's derivatives take the record's own textbook names, `grad_J`, `hess_J`, `jac_h`,
  `jac_g`, and the long names `gradient`, `hessian`, `jacobian_h`, `jacobian_g` go (CA-6, ruled:
  one name per derivative, matching `J`, `h`, `g`). `evaluator.h` (CA-2) waits for the core-API
  round.
- **Two defects, each re-pinning what it moves with its closeness report:** a value-iteration
  policy's interpolated command is clipped to the grid's input box (KNOWN_ISSUES, the `u_max +
  1e-15` overshoot); the forced-input hold is one rule every solver obeys (the one-owner row S65's
  item), built with AR-8 since it rewrites the forced-input path.
- **After the nine, on the same branch:** TODO §1.11's "No cache left" (the problem evaluator's
  re-jit on a replaced part and its digest-keyed failure cost, value iteration's option-keyed
  tables, the Lyapunov digest: each rebuilt in its verb or snapshotted at build) and the
  parameters model's four follow-ups (each byte-identical on `benchmarks/params_baseline.py`).

The branch is `api-surface-rulings-review`, off `dev` at 8029a36; nothing merges to `dev`
before the maintainer's review.

## Not ruled yet (a later round)

From the same scan, each needs its own context and options before a ruling: the planner's
shortcuts taking the solution's names (`plot_solution` → `plot_trajectory`; `get_controller` and
`value_at`; student-facing); training budgets `n_episodes` and `n_timesteps` (RULES 2.7);
workflow keywords on constructors; `problem.compile()` always returning the evaluator, the step
face from `discretize(dt).compile()`; about 80 public helper methods on the diagram, the problem,
the evaluator and the planners moved to module functions (`PlanningProblem.start_point` is ruled,
above). Not recommended by the scan: `%` against `discretize`, dropping the analysis
method facades, one array layout for every plot and scan, a result object for `pv.solve`.
