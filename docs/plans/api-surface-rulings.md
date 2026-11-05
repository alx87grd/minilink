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

## AR-1 A distribution's mean has one spelling

`Distribution.mean` (a property, at the law's own numbers) and `sample(None, ...)` stay;
`mean_at(params)`, `mean_draw(n, params)` and `Gaussian.full_cov` go (`core/distributions.py`;
callers in 8 library files, 2 examples, 1 test). A mean at other numbers is the law built at
those numbers, or `sample` with its key `None`. `full_cov` is read as `cov` (a Gaussian holds
`{mean, std}` or `{mean, cov}`; `cov` is the view that always answers).

```python
# before                         # after
law.mean_at(p)                   law.sample(None, params=p)   # or law.mean at its own numbers
law.mean_draw(n, p)              law.sample(None, n, params=p)
g.full_cov                       g.cov
```

`sample(key, n=None, params=None)` exists on every law today; `key=None` returns the mean, so no
signature changes.

## AR-4 `Trajectory` reads a value at a time with `traj.at(t, name="x")`

One reader, **linearly interpolated** between samples (today's readers return the nearest
sample), `name` defaulting to the state: `traj.at(2.5)`, `traj.at(2.5, "u")`,
`traj.at(2.5, "controller:u")`. The interpolation already exists: `interpolate_schedule` in
`core/trajectory.py` (linear between samples, held outside), which the gain-scheduled controllers
(`control/state.py`) and the finite-horizon LQR already read; `traj.at` is that function on the
record's samples, one transpose at the boundary. Landing it closes TODO's "one time-interpolation
for trajectories and gain schedules" row outright (amended 2026-11-05, the maintainer's go).

Gone (`core/trajectory.py`): `sample(name, t)` (it clashed with `sample(key)` on sets and laws,
which draws), `t2x`, `t2u`, `with_signal` (use `with_signals({name: value})`), `time_duration`
(`traj.tf - traj.t0`), `copy` (the record is frozen; no caller). User calls: three lines in the
systems tutorial (01), `get_signal` stays.

`at` here reads a value at a time on a record; it is unrelated to the planner copy verb `at` the
parameters model removed.

## AR-5 `jacobian` returns an array everywhere, and there is no function form

`ev.jacobian(of, wrt, x, u, t, params)` returns the array, as `sys.jacobian` does (today
`ev.jacobian(of, wrt)` returns a function: `core/compile/evaluators/jacobian.py`). There is no
second name: the textbook's derivative is a matrix at a point, `A = ∂f/∂x |₍x̄, ū₎`, and the
function form is the method itself, unevaluated, as `f` is the function form of a rate (ruled
2026-11-05, option A of three; a `jacobian_fn` sibling and "no point means the function" were
declined: nothing else in the core returns a function, and `plant.jacobian("f", "x")` already
means the matrix at the nominal point).

```python
A = plant.jacobian("f", "x", x_bar, u_bar)       # NumPy array; the nominal point if omitted
A = ev.jacobian("f", "x", x, u, t, params)       # the array on the evaluator's backend

# A loop or a filter that wants the function takes the method, unevaluated
A_of = partial(ev.jacobian, "f", "x")            # A_of(x, u, t, params) -> matrix
A_jit = jax.jit(partial(ev.jacobian, "f", "x"))  # a hot loop on JAX
```

The evaluator's point arguments are `x, u, t, params`, the slots of `ev.f` (an EKF evaluates at a
running estimate); the system's shortcut keeps `x_bar, u_bar`, the operating-point words of
RULES 2.7. The evaluator builds each `(of, wrt)` derivative once, at its first call, and keeps
it: compiled machinery of a held tool keyed on structure, not a cache (DESIGN §4 rule 6), so the
partial costs a lookup per call; the built derivative stays unjitted, as today, so it traces under
the caller's `jit` with concrete `params`. A specialized tool (the EKF of
[estimation-and-state-feedback.md](estimation-and-state-feedback.md)) takes its derivatives on
`model.f` directly, as that plan already says. The two library callers (`analysis/derivatives.py`)
evaluate at a point; about 17 lines change.

## AR-6 A default start on the constructor, this solve's on `solve`; the solve time at the user's verb

Amended 2026-11-05 (the maintainer's review): the constructors keep their default start, and the
timing flag goes.

**The start.** `Optimizer(program, z0=)` and `TrajectoryOptimizationPlanner(..., guess=)` set the
default start, as `method=` sets the default method; `solve(z0=)` and `solve(guess=)` are this
solve's, overriding it once. The same word on both, as `method=` already is; no `default_` prefix.
The optimizer's default is the constructor's `z0`, else the program's own (`metadata["z0"]`, set by
an exported program), else a refusal, as today; it is also the sample the build traces the program
at. In a loop, the natural route is the compiled face, `pv = planner.compile()` then
`pv.solve(P, z0)`, which takes its start explicitly every call (the MPC's memory holds the last
plan, `pv.initial()` the default guess while there is none).

**The solve time.** `record_solve_time` goes. A user's verb (`Optimizer.solve`, `planner.solve`)
always times the backend solve it wraps and fills `solve_time_s`; the timer costs nothing. The
machine-side solves never time: the compiled face's `pv.solve(P, z0)` and the fixed-iteration pure
solve (`n_iterations=K`, `solve_count`) are traced under `jit`, with no wall clock inside; their
results leave `solve_time_s` empty, and the verb that calls them on the host times around them.
Callers of the flag: 11 library files, 16 examples, 12 tests, 5 benchmarks.

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

## AR-8 One simulate verb for the ODE run; the rollout stays beside it

Amended 2026-11-05 (the maintainer's review). Two run verbs, each with its plot, because they mean
two things on a hybrid system: `compute_trajectory` integrates the ODE (the continuous side, dense
between the ticks) and `compute_rollout` steps the sampled loop as a pure stepping system, tick
by tick. `plot_trajectory` and `plot_rollout` mirror them; the fold of `plot_rollout` into
`plot_trajectory` is declined.

**The forced run is the same verb.** `compute_trajectory(tf, u=None, ...)` on every system kind
and `Simulator.solve(u=)`: `u` decides, `None` the nominal inputs, else the prescribed input as
today's `compute_forced` took it (a full array, a function `u(t)`, a constant vector, a scalar on a
one-input system; `input_port_id=` forces one port and keeps the others nominal). `compute_forced`
and `solve_forced` go. On a rollout, `u` stays the tick's: a constant, a sequence, or `u(k)`.

**An input array is `(m, N)` everywhere**, the textbook layout (`U = [u_0 … u_N]`, a column per
sample, RULES §5), the layout of `traj.u`, so `compute_trajectory(tf, u=old.u)` replays a run.
`compute_rollout(n_steps, u)` switches from `(n_steps, m)` to `(m, n_steps)` (the systems
tutorial cell, two demos, one test), at the facade: the compiled evaluators keep their
time-leading `(N, m)` scan layout, and the facade transposes once, as it already does (the RL
and MPC rollouts are untouched). A vector of length `m` stays a constant input.
Callers of the removed names: 12 examples, 14 tests, 5 benchmarks, plus one call in the GRO860
drone PPO notebook.

## AR-9 Two routes to replan from a new start

Offline: `planner.solve(params=p, guess=prev)` with the new `p["x0"]` (it builds each call).
Real time: `pv = planner.compile()`, then `pv.solve(p, z)`, which MPC already uses. The start state
is a number of the tree like a mass or a cost weight. `solve_trajectory_from`,
`solve_trajectory_from_parametric`, `solve_trajectory_from_rebuild`, `step`,
`compile_parametric_program`, `has_parametric_program`, `transcribe_parametric` and the
parametric optimizer plumbing go (`planning/trajectory_optimization/planner.py`; 24 test lines,
1 example, 5 benchmark lines), and with them the base `Planner.solve_trajectory_from` hook. The RL
planner's `solve_trajectory_from(x0, tf)`, which rolled its learned policy out from `x0` with
draws, goes too (amended 2026-11-05, the maintainer's review): that rollout is
`evaluate(problem, solution.policy, record=True)`, the one judge, with no second verb to learn
(its callers: the planners tutorial 08, `test_rl_planner.py`).

Measured before the ruling (pendulum, 20 knots, a new `x0` and mass each call): the compiled
planner `pv.solve` 8 ms a solve after a 0.54 s build, the old start-only route 11 ms after
0.89 s. The old route was never specialized: `transcribe_parametric` returns the same program and
binds the whole tree. Freezing every number but `x0` is 0–35 % faster per program evaluation and
within noise per tick (SLSQP's own loop dominates), so no partial freezing is built; revisit only if
a GPU or Ipopt case shows program evaluation dominating (API then: `planner.compile(free=("x0",))`).
Removing the route also clears the last item of TODO's "No cache left" row that it names.

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
