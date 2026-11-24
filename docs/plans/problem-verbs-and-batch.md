# The problem's verbs, the compiled objective `J`, and the batch rollout

The review of the README pass and its new public methods (`PlanningProblem.evaluate`,
`.compare`, `.program`; `System.rollout_batch`) took nine rulings. This document holds them,
the work they ask for, in steps `PV-1` to `PV-7`, and the run's record. It is deleted when the
work lands; the contracts move into DESIGN.

## Rulings

1. **Where the batch lives.** The compiled continuous evaluator batches on both backends:
   one `vmap` on JAX, a loop on NumPy, with `solver=`. `system.rollout_batch(...)` is a thin
   shortcut: the time grid, the compile, one call. The step evaluator stays JAX-only, with its
   `TypeError` naming the backend.
2. **What a batch returns.** `t, xs = system.rollout_batch(...)`, `xs[member, sample, state]`,
   shape `(B, N + 1, n)`. A batched `Trajectory` (one object, `x` of shape `(B, n, N)`) is an
   open question, reopened when a second tool needs batched runs.
3. **One route per scoring verb.** `problem.evaluate(controller, ...)` and
   `problem.compare(Name=entry, ...)` are the only shortcuts. The free functions `evaluate` and
   `compare` leave the public surface now, without a deprecation step.
4. **Comparing is scoring.** The comparison by names only is deleted: a `Comparison` always
   holds its problem and its evaluations. Every `PlanningSolution` carries its problem, so the
   no-problem case never arose; the names-only mode only skipped the simulation (measured:
   two controllers on the pendulum, 5 s horizon, `dt=0.01`, scored in 1.6 s, cloud CPU, warm,
   median of 5).
5. **`problem.compile` keeps its name.** `compile` has one meaning across the system, the
   planner and the problem: it fixes the structure, snapshots the parameters and returns pure
   functions of them.
6. **The compiled object is `J`.** In code, docs and examples its instance is named `J`, the
   performance index of the closed loop as a function of the parameter tree. `J(p)` is the
   objective (`J.evaluate(p).objective`), so `jax.grad(J)(J.params)` reads as the math.
   `problem.compile(controller, ..., n_trials=None, key=0)` fixes the draws `J()` reads on a
   stochastic problem; a per-call `n_trials=` overrides it. `ProblemEvaluator` keeps its class
   name.
7. **`problem.program` is removed.** Tuning goes through `J.program(wrt, ...)`: reading the
   answer back (`J.unpack`) and running the tuned loop (`J.trajectory`) need `J` anyway.
8. **README §5.** `t, swings = plant.rollout_batch(starts, tf=5.0, dt=0.005, backend="jax")`,
   in the README and the showcase.
9. **`array_module` on the core facade.** `from minilink.core import array_module`.

Rejected during the review, kept here so it is not proposed again: a `problem.tune(...)` verb
returning a `PlanningSolution` (it hides the explicit program and optimizer step, and tuning
is not planning: `wrt` may name the plant's parameters); `params=` on the problem's verbs plus a
read-back on `MathematicalProgram` (too much new surface to avoid holding `J`); renaming
`compile` (`with_controller`, `objective`, `evaluator`).

## Steps

Each step is one commit on the working branch. After each: ruff and format, the touched test
files, `tests/unittest/test_vocabulary.py`.

**`PV-1`, the batch rollout (rulings 1, 2).**
- `minilink/core/compile/evaluators/base.py`: `ContinuousEvaluator.rollout_batch(...,
  solver="rk4")` on both backends; the NumPy loop and the batch-member slicing move here from
  `minilink/core/facades/simulation.py` (`numpy_rollout_batch`, `take_batch_member`).
- `minilink/core/facades/simulation.py`: `DynamicSimulation.rollout_batch` keeps the time grid
  (`batch_time_grid`) and one call; the docstring names `xs[member, sample, state]`.
- `docs/dev/KNOWN_ISSUES.md`: the entry "`rollout_batch` on a NumPy evaluator fails with an
  `AttributeError`" leaves.
- `tests/unittest/test_problem_and_batch_shortcuts.py`: the `TypeError` test on the continuous
  evaluator becomes NumPy-against-JAX agreement; the step evaluator's refusal stays tested.
- Done when: the new test file, `test_compile.py`, `test_simulation.py`,
  `test_step_discrete.py` pass with JAX installed.

**`PV-2`, the free `evaluate` and `compare` and the names-only comparison leave (rulings 3, 4).**
- `minilink/planning/comparison.py`: `compare` becomes the module's internal scoring function
  behind `problem.compare`; the `problem is None` branches, `require_no_scoring`,
  `require_solutions` and the names-only fallback simulation in the simulated-trajectory
  figure leave; `Comparison.problem` and `.evaluations` are required.
- `minilink/planning/evaluator.py`: `evaluate` stays the module function `problem.evaluate`
  calls, off the public surface.
- `minilink/__init__.py`, `minilink/planning/__init__.py`: the two exports leave.
- Call sites: about 70 in `tests/` and `examples/`, among them
  `examples/tutorial/advanced/02_optimal_control.ipynb` (the feedback-law plot:
  `problem.compare(VI=vi, LQR=lqr, dt=0.01).plot_control_law()`) and
  `examples/demos/rrt/rrt_holonomic_obstacles.py` (`problem.compare({"RRT": ..., "RRT*": ...})`).
- Done when: no `evaluate(problem` or bare `compare(` call is left outside the two modules;
  the planning tests pass.

**`PV-3`, `J` (ruling 6).**
- `minilink/planning/evaluator.py`: `ProblemEvaluator.__call__(params=None, key=None,
  n_trials=None)`, the objective; `n_trials` and `key` held from the compile.
- `minilink/planning/problems.py`: `compile(..., n_trials=None, key=0)`.
- Docstrings and the class's example use `J`.
- Tests: `J()` on a deterministic and a stochastic problem, `jax.grad(J)(J.params)` against
  `jax.grad(lambda p: J.evaluate(p, ...).objective)`, the per-call override.
- Done when: those tests pass on both backends where they apply.

**`PV-4`, `problem.program` leaves (ruling 7).**
- `minilink/planning/problems.py`: the method leaves.
- `examples/tutorial/advanced/03_mpc_and_mppi.ipynb`, `advanced/05_differentiable.ipynb`,
  `core/05_problem.ipynb`, `showcase_minilink.ipynb`: back to `J.program(...)`.
- Tests: the program test moves to `J.program`; signature tests that `problem.evaluate` and
  `problem.compare` forward every keyword of `compile` and of `J.evaluate`.
- Done when: no `problem.program(` is left; the touched notebooks pass the smoke check.

**`PV-5`, the README and the showcase on one story (ruling 8 and the rulings above).**
- §3: `uncertain.compare(VI=vi, RRT=rrt, TrajOpt=opt, MPC=mpc, n_trials=20)`.
- §4: `J = task.compile(Impedance(Kp=1.0, Kd=1.0), n_trials=20, backend="jax")`,
  `jax.grad(J)(J.params)`, `J.program(gains, bounds=bounds)`, the showcase's tuned loop through
  `J.unpack` and `J.trajectory`.
- §5: `plant.rollout_batch(starts, tf=5.0, dt=0.005, backend="jax")`.
- The instances named `pe` or `evaluator` in notebooks and demos that hold a problem's
  compiled loop become `J`.
- The README is the maintainer's: its prose diff is shown in the run's report before the push.
- Done when: README and showcase read cell for cell; the showcase passes the smoke check; the
  math touched is read against `planning/dynamic_programming/dp.py`.

**`PV-6`, `array_module` on `minilink.core` (ruling 9).**
- `minilink/core/__init__.py` registry; README §5's sentence gives the import.
- Done when: `from minilink.core import array_module` works; `test_vocabulary.py` passes.

**`PV-7`, the docs.**
- DESIGN: the problem's verbs, `J` and its call, `rollout_batch` and `(t, xs)`, `compare` only
  on a problem, the `Comparison`'s required fields; the paragraph "The problem evaluator".
- ROADMAP: the row "Planning / evaluation".
- QUESTIONS: two open items, the batched `Trajectory` (ruling 2) and the tension the review
  left: tuning passes through the compiled object `J` while the rest of the main story is
  verbs on nouns (the ideas weighed: a read-back on the program, `params=` on the verbs, a
  domain name for `compile`).
- `examples/README.md` and `docs/plans/readme-and-showcase-story.md` where they cite the
  removed names.
- Done when: `git grep` finds no removed name in the docs.

## Gates on the finished branch

The full `pytest -n auto`; the demo-check bridge (`tests/unittest/test_demo_check_runners.py`)
on its own; the regression gates with the CI flags (the simulator and the evaluator are
touched); the notebook smoke check on every touched notebook; the "Always before push" row.
Every run headless (`MINILINK_HEADLESS=1`).

## Run record

The run may stop at a context limit. To resume: `git fetch --all --prune`, `git status -sb`,
`git log --oneline` on the working branch (one commit per step, its id in the subject's body),
then this table.

| Step | State |
| --- | --- |
| `PV-1` the batch rollout | done |
| `PV-2` the free functions and the names-only comparison | done |
| `PV-3` `J` | done |
| `PV-4` `problem.program` | done (the showcase with `PV-5`) |
| `PV-5` README and showcase | done (notebook sweep pending) |
| `PV-6` `array_module` | done |
| `PV-7` the docs | open |

**Decisions taken during the run:**
- `PV-1`: the jitted batch is cached per solver as well as per layout, so an Euler batch
  never reuses an RK4 one; the NumPy loop lives on the base evaluator (`looped_rollout`)
  beside the `vmap` (`batched_rollout`), the family split by the same rule.
- `PV-2`: the free `evaluate` is deleted, its body and docstring on `problem.evaluate`
  (`self.compile(...)` then `J.evaluate`); `comparison.compare(problem, ...)` stays the
  module function `problem.compare` calls, off `__all__`. The RRT demo scores the two plans
  over the longer plan's duration (`episode_length=`), the problem having no horizon; its
  table shows the RRT* plan, simulated by the judge, missing the goal its nominal trajectory
  reaches (its steps are not uniform): reported to the maintainer, not changed.
- README §3 still calls the free `compare` until `PV-5`.
- `PV-5`: `core/07_program.ipynb` keeps its hand-written `def J(z)` objective (a later
  cell, the program's own `J`), after the cells that use the problem's `J`. The tutorials and
  demos that imported `array_module` from `minilink.core.backends` import it from
  `minilink.core`; `examples/projects/` is left as it is. `system.rollout_batch` compiles at
  each call (0.43 s for README §5's 1000 × 1000 batch on JAX, cloud CPU, median of 5, against
  0.11 s for a held compiled evaluator): README §5's timing sentence waits for the maintainer.
- `PV-3`: `J.evaluate`, `J.program` and `J(p)` take `key=None`, read as the compile's
  `key` (0 by default), so a call that names no key runs as before.

**For the maintainer after the merge:** `alx87grd/teaching` and `alx87grd/workbench` on `dev`
call the removed names; search them for `evaluate(`, `compare(`, `.program(` on a problem,
`pe.` and `evaluator.` on a compiled problem, and `rollout_batch(` on a NumPy evaluator.
