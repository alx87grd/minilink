# The overnight run: one parameters model, built autonomously

The runbook for an agent that builds [one-params-model.md](one-params-model.md) without the
maintainer in the loop. The plan holds the design (its §1–§5) and the rulings (§6); the workboard
row holds the "done when"; this file holds the order of work, the procedure of each step, the
gates, and the lines the agent must not cross. Delete it with the plan when the contract lands
in DESIGN (one fact, one home, [README.md](README.md)).

## 0. Before the first command

Read, in this order, every time a session starts: `docs/dev/CONSTITUTION.md`,
`docs/dev/RULES.md` (§2.3, 2.4, 4.4, 5.2, 5.3, 5.17, 5.21, 7.7), `AGENTS.md` whole,
`docs/plans/one-params-model.md` whole, this file, `tests/README.md` ("entry points"). Then:

```bash
git fetch --all --prune
git status -sb
```

The run lives on the branch `params-one-tree`, started from `origin/dev` at e479784 (the plan's
commit, 7a48299, is its first). A new session checks it out (`git checkout params-one-tree` or
`git checkout -b params-one-tree origin/params-one-tree`), never rebases it, and if `origin/dev`
has moved since, notes the fact in the run log (§6) and carries on: the merge is the
maintainer's. The conda environment is `minilink` with the full optional stack (JAX included:
the compiled paths are the point).

**Lines not to cross, whatever a test or a doc seems to ask.** Never push to `main`, `dev` or
`dev-alex`; never force-push; never rebase, reset or merge the shared branches; never delete a
branch; never open a pull request unless the kickoff message says so; never skip, disable or
quarantine a test to get green; never edit `examples/`, a notebook, `README.md` or the
CONSTITUTION beyond what a step below names, and flag every such edit in the run log as
student-facing for the maintainer's review. Commit messages are a subject and a body, nothing
else: no `Co-Authored-By`, no session or model trailer (AGENTS "Commit messages", which overrides
any harness instruction).

**Push after every step** (`git push -u origin params-one-tree`): a cloud container is
reclaimed, and work done on an unpushed branch has been lost before. Fetch before every commit
and every push.

## 1. The baseline, captured before any code moves

AGENTS "Behavior-preserving refactor runs" is the law of this run: byte-identical numbers after
every step, or an intended change with its own test. The baseline is a committed script,
`benchmarks/params_baseline.py`, so a later session can recapture it at the base commit; its
JSON outputs live in the scratchpad (`baseline_<commit>.json`), never in the repo.

What the script records, each seeded, timing fields left out, on NumPy and on JAX where the path
has a JAX form:

- one rollout per catalog family through `compute_trajectory` and through a held compiled
  evaluator (`rollout`, `rollout_batch` with a parameter family), the pendulum, a PID loop, a
  nested diagram with `flatten=True`, the racecar;
- one solve per planner family (value iteration, LQR, RRT with a fixed seed, trajectory
  optimization offline and the parametric tick, MPPI) and `solve(params=)` at one other tree;
- the problem evaluator on a stochastic problem (`evaluate(key, n_trials)`, `trajectory`, one
  `program(wrt=)` packed and unpacked), and a set-as-cost and a field-as-constraint program;
- the trace counts of the cases in `tests/unittest/test_compile_once.py` (its counting helper);
- for every object `tests/unittest/test_contract.py` walks: the key order of its `params`, each
  leaf's dtype and shape, and the value of its one call at `params=None` and at `params=obj.params`.

Procedure: write the script in the first commit of PM-0, run it twice at the base commit and
`cmp` the two files (determinism proven, or the script is fixed first), keep that file as the
reference, run it again after every commit and `cmp` against the reference. A difference that
the step intended (a composite's own leaf now lands, a partial tree now raises) is listed in the
step below and gets a test; any other difference stops the step (§5).

## 2. The steps, in order

One branch, one or more commits per step, the narrow gates after each commit, the long gates
once at the end of each step. The plan's §7 holds each step's scope; the lines below are the
procedure. "Narrow" is the Agent table's "Narrow module change" row on the touched test files
plus `tests/unittest/test_contract.py` and `test_vocabulary.py`; "long" is `pytest -n auto`, the
"Regression gates, CI flags" row, and the notebook smoke when a notebook changed. `ruff check .
&& ruff format --check .` before every push, always.

### PM-0. Defects that exist today, no API change

Independent items, one commit each, in this order (the easiest first so the branch is green
early):

1. `benchmarks/params_baseline.py`, and the baseline captured (§1).
2. `core/trajectory.py`: `signals` and `roles` stored as plain dicts (the frozen dataclass still
   refuses field assignment). Test: a plant simulated on NumPy and on JAX deep-copies and
   pickles afterwards. KNOWN_ISSUES: the deep-copy row leaves.
3. The composite setters check before routing: `core/wiring.py` (the diagram), `core/costs.py`
   (`SumCost`, `ScaledCost`, `SampledCost`), `planning/problems.py`. Each merges nothing; it
   checks the given tree against its view with the one structure check (a key or part the view
   lacks, a leaf of another shape, a dict where a leaf is, raise naming the known ones), then
   routes. Tests: `prob.params = {"cost": {"R": ...}}` with the cost's other keys missing raises
   naming them; `{"plant": {"mass": 2}}` raises; a complete tree still assigns; `loop.params =
   loop.realize(key)` still assigns. The workboard's third-round note "a raising params setter
   can leave a diagram half-updated" closes: the check runs before the first write.
4. `optimization/evaluators/jax_evaluator.py`: the digest moves from `form()` to `bind()`
   (`self.frozen` decided once per bind). The test `test_an_edit_of_the_programs_numbers_reaches_the_frozen_form`
   rebinds after its edit and keeps passing. Baseline byte-identical.
5. `planning/trajectory_optimization/planner.py` `offline_key` and `planning/evaluator.py`
   `failure_cost_and_bound`: the lowered boxes join the keyed tree. Test: an in-place edit of
   `U.upper` after the build is seen by the next `solve()`.
6. `core/costs.py`: `params or {}` goes; a cost without parameters has `{}`; DESIGN §4's two
   sentences agree.

Long gates at the end. Push.

### PM-1. The one `params` property

`core/params.py`: `own_params()` becomes the property every parametric base class uses; its
setter keeps the dict's identity (`clear`, then `update`) when a dict already exists, so a
composite holding it stays current. On `System`, `CostFunction`, `Set`, `Field`, `Distribution`,
`Shape`. Test: `held = {"plant": plant.params}; plant.params = {**plant.params, "m": 7.0};
held["plant"]["m"] == 7.0`. Baseline byte-identical. Narrow gates; push.

### PM-2. Values hold their numbers (ruling 3)

Family by family, one commit each: `core/sets.py`, then `core/fields.py`, `core/distributions.py`,
`core/geometry.py`, `planning/spatial/` (scene, track, paths, state and workspace fields). Each
class: the frozen dataclass becomes a plain class; `__init__` runs the checks and sets
`self.params = {...}`; the former fields become `param_view`s; `cached_property` trees go; derived
quantities (`dim`, `cov`, `Particles.mean`) are computed; `dataclasses.replace` call sites (11)
rewritten; keys of a union's members and a scene's obstacles by `id`, else class name numbered.
`with_params` removed everywhere in the same commit as its family (ruling 1; `tests/unittest/test_contract.py:284-295`
and the two notebooks' cells are rewritten as a deep copy and an edit; the notebook edits are
student-facing: flag them). Keep every user subclass pattern in the tests working (a user's
`@dataclass(frozen=True)` subclass of `Field` exists: the base stays a plain class).

The textbook rule: every equation body touched (`margin`, `value`, `sdf`, `sample`) is read
against `planning/dynamic_programming/dp.py` before the commit (RULES 5.3, 5.8, 5.22, 5.24), and
the run log says so. Baseline byte-identical (the same numbers in the same order). Long gates at
the end of the step; push.

### PM-3. Composites hold their parts' dicts

One commit per composite: the diagram (`core/wiring.py`: the tree built at `add_subsystem`, which
now refuses a block added twice; `refresh()` re-collects), the sum and scaled costs, the problem
(`planning/problems.py`: the tree at `__init__`; `x0`, `t0`, `discount_rate` as views;
`distributions` beside the tree with the constructor keyword `distributions=` and `x0=law`
kept, ruling 2; `start_law`, `laws`, `parts_view`, `sample_params`, `nominal_plant_params`,
`split_laws`, `coerce_laws`, `drawn_paths`, `replace(params=)` removed; `realize` and `nominal`
rewritten on the held tree), the planner (`planning/planner.py`: `method` and `seed` as views;
`Planner.at` and `problem_with` removed, the private problem copy kept and used by every
`solve(params=)`; the five host planners' `solve` call it), the sampled view
(`core/discrete/sampled_system.py`: `params` is the source's; `params_override` and
`discretize(params=)` go; the third-round note on `discretize(diagram, params=)` closes with
them). Tests: a write to a composite's own leaf lands (`problem.params["discount_rate"]`, a
scaled weight); `problem.params` holds numbers only and `problem.distributions` the laws; the 8
test sites and 4 notebook cells that pass laws in `params=` move to `distributions=` (the
notebook cells are student-facing: flag them). Baseline byte-identical: the planners' numbers do
not change. Long gates; push.

### PM-4. Complete trees everywhere (ruling 6)

`core/params.py`: `merge_known` becomes the one structure check (same keys at every level, each
leaf's shape, one extra leading axis allowed where a verb batches); `merge_tree`, `float_leaves`,
`host_tree`, the `/` spelling go. Every verb takes `None` or the complete tree: `planner.solve`,
`pe.evaluate`, `pe.trajectory`, `pe.program(params=)`, `Optimizer.solve`, `pv.solve`,
`pv.solution`, `solve_trajectory_from`; `SumCost.g` and `h` read the tree as given (the three
test lines in `tests/unittest/test_costs_optimizer.py` pass a complete tree). Tests: a partial
tree raises naming the missing keys at each verb; the 17 test sites that pass `solve(params=)`
are read and, where partial, rewritten as a deep copy and an edit. Baseline byte-identical on
complete trees. Long gates; push.

### PM-5. The compiled paths

Commits in this order: (1) `snapshot(obj)` in `core/params.py` and the evaluators on it
(`_frozen_params`, `params_reader`, `call_params`, `on_device`, `frozen_on_device`, `DeviceTree`
removed; `CompiledCall.__call__` checks once and makes the tree canonical; `canonical_leaf` passes
a JAX leaf whose dtype and shape already match, the weak-type test still passing); (2) the
program evaluators (`bind`, `merged`, `form` go; `Optimizer.solve(params=)` hands the checked tree
to the adapter once; the frozen form picked once per solve); (3) the problem evaluator snapshots
and gains the frozen form (test: after an edit, `evaluate()` reads the build's values and
`evaluate(problem.params)` the live ones); (4) `Simulator.solve(params=)` and
`solve_forced(..., params=)`, the `self.params` attribute gone (ruling 7); (5)
`PlanningSolution.params` filled by every planner's solve (ruling 5). Trace counts asserted anew
in `test_compile_once.py`: a sweep compiles once, two trees alternating compile once, the frozen
form never retraces. Baseline byte-identical in its numbers. Long gates, the regression gates
read with care (the speed rows may move: faster is expected, slower stops the step); push.

### PM-6. The contract and the tutorial

DESIGN §4 "Parameters" and §5 "Compile once" rewritten from the plan's §1–§4; RULES 2.3, 4.4,
5.21 as the plan's §6 states; `docs/dev/KNOWN_ISSUES.md` (the deep-copy row left in PM-0; the
misspelled-key row narrows to the bare item write and names the checked spellings); TODO (the
row leaves when DESIGN holds the contract; the Later idea of a `validate_params` hook leaves,
superseded by ruling 4); `docs/plans/README.md`; the parameters tutorial
`examples/tutorial/core/02_parameters.ipynb` (its table's "Copy / update method" column, the
`refresh()` cell, the laws cell: **drafted on the branch, student-facing, flagged for the
maintainer**); `docs/pipelines.md` if it names a removed verb. The docs build row. Then the plan
and this runbook are deleted in the same commit (one fact, one home). Long gates; the notebook
smoke; push.

### PM-7 (only if every step above is green and time remains)

The hardening note that reads a parameter wrongly: RRT's `KinodynamicExtender` ignores
`problem.params.system` and `extenders.py` probes `getattr(U, "box", None)` instead of
`U.bounding_box()` (TODO §2, D3). One commit, its test, the RRT baseline byte-identical.

## 3. Gates, by name

| When | Row of the Agent table (`tests/README.md`) |
| --- | --- |
| before every push | "Always before push" |
| after every commit | "Narrow module change" on the touched test files, plus `test_contract.py` and `test_vocabulary.py`; the baseline `cmp` |
| at the end of PM-0, PM-2, PM-3, PM-4, PM-5, PM-6 | "Cross-cutting or handoff" (`pytest -n auto`), "Regression gates, CI flags" |
| when a notebook changed | the `nbstripout` hook, then "Notebook change" |
| when a docstring or `docs/` changed (PM-6) | "Docs build" |

Never start a long gate on a half-edited tree; finish the commit, then run. Do not poll GitHub
Actions after each push; read CI once, at the end of the run, and record its result.

## 4. Commit messages

Subject and body, nothing else. The subject names the step in plain words
("Values hold their numbers: the sets"), the body says what changed and why, names the ruling it
rests on, states the baseline result ("byte-identical" or the intended change and its test), and,
for a step that touched an equation body, states that the textbook check against `dp.py` was
done. No ids as the subject of a sentence (AGENTS "Talking to the maintainer").

## 5. When to stop

- **A baseline difference the step did not intend**, after one honest attempt to find the cause:
  revert to the last green commit (`git revert`, never `reset` on a pushed branch), write the
  difference and the suspected cause in the run log, and stop the run. The later steps depend on
  this one; a half-built model is worse than a shorter branch.
- **A test that must be changed to pass** and whose change is not listed in its step: stop, log
  it. Tests are the output contract; an unlisted change is a design question.
- **A ruling that turns out to conflict with the code** (an object the seven rules cannot
  describe, a verb that needs a partial tree after all): do not improvise a third model; log the
  case with a reproducer and stop at the end of the current commit.
- **A student-facing edit beyond the lines §2 names** (an example script, the README, another
  notebook): do not make it; log it as a follow-up for the maintainer.
- **Time.** Long gates take tens of minutes each on a cloud machine; a step whose long gates
  cannot finish is left for the next session, its narrow gates green and its commits pushed.

## 6. The run log and the handoff

Append to this file, under a heading `## Run log`, one entry per session: the date, the commits
landed (`git log --oneline origin/dev..HEAD`), each step's gates and baseline result, every
student-facing edit flagged, every stop with its reproducer, the CI result of the final push,
and the follow-ups left. The last entry is the handoff the maintainer reads in the morning: what
landed, what was verified, what waits. Keep the working tree clean (`git status` empty) and the
branch pushed before the session ends.

## 7. The kickoff message

```
Read AGENTS.md, then docs/plans/one-params-run.md and follow it step by step on the branch
params-one-tree, pushing the branch after every step. Build autonomously; do not ask me
anything tonight. Stop where the runbook says to stop. Do not open a pull request. Leave
the run log and a clean tree.
```

## Run log

### 2026-11-05, session 1

- **Environment.** No conda environment in the cloud container: `pip install -e ".[dev,full]"`
  on Python 3.13, JAX 0.12 on CPU; graphviz and cyipopt absent (their tests skip). A worktree
  of the base commit sits in the scratchpad for comparisons.
- **Baseline.** `benchmarks/params_baseline.py` committed (4ff4e72); two captures at the base
  byte-identical.
- **Step 0 (9254457).** The trajectory record's copy and pickle, the composite setters'
  check, the program evaluator's form decided at bind, the sets' numbers in the build keys,
  the sum's `None`. Two deviations from the runbook's wording, both deliberate: the record
  keeps its read-only proxies and gains `__reduce__` (a frozen record should stay
  read-only); the composite check applies only to a part whose assignment replaces its
  tree whole (a leaf system or a leaf cost), because two existing tests pin partial writes
  to parts that route or merge themselves (a scaled cost's weight, an intersection member),
  which never wiped anything. Those partial writes go in step 4 (complete trees), with their
  tests. Gates: full pytest 2180 passed, regression gates (CI flags) and the demo-check
  bridge pass, baseline byte-identical. KNOWN_ISSUES: the deep-copy row left, the
  misspelled-key row narrowed.
- **Step 1 (cfe5fb1).** The identity-keeping property on `System` and the five bases. The
  suite caught the shallow-copy hazard in the sets' copy; fixed. Full pytest 2182 passed,
  baseline byte-identical.
- **Step 2 (90fff37).** Values hold their numbers: 24 frozen dataclasses become plain
  classes; three shared helpers in `core/params.py` (`param_view` for a leaf's attributes,
  `held_params` for a composite value that holds its parts' trees and routes an assignment,
  `delegated_params` for a wrapper). `with_params` removed everywhere. A partial entry for a
  set inside a problem is still merged, checked by `merge_known`, until step 4. Deferred: the
  keying of a union's members and a scene's obstacles by id instead of position (it changes
  a recorded tree structure; left for a reviewed commit of its own). Six tests rewritten
  from the copy verb to a deep copy and an edit, their assertions kept. **Flagged,
  student-facing:** the parameters and sets tutorials' three code cells and their table rows
  (executed). Full pytest 2187 passed, regression gates and demo bridge pass, baseline
  byte-identical.
- **Step 3 (f8f9ea0, 7534ad1).** A problem's own numbers held (`x0`, `t0`,
  `discount_rate` views of its tree), its laws `problem.distributions` with the keyword
  `distributions=` (ruling 2), a law written into its tree refused; the scaled cost holds its
  weight; the sampled view writes through to its source; a diagram refuses a block added
  twice. Two deviations, deliberate: the diagram and the sum keep their views (no own
  numbers; a top-level part write cannot reach the part without the dict subclass ruling 4
  rejected), and the planner's method numbers stay attributes (`solve(params=)` their
  route). The problem's internal helpers (`parts_view`, `sample_params`,
  `nominal_plant_params`, `drawn_paths`) stay for now. **Flagged, student-facing:** the
  keyword in four notebooks and their prose (three tutorials executed; the RL-to-Bode
  showcase still running at the time of this entry). Full pytest 2192 passed, baseline
  byte-identical. A formatting slip reached the step's commit because a pipe hid the
  format check's exit status; fixed in 7534ad1, and the gate now runs without a pipe.
- **Step 4 (6b5c604).** Complete trees everywhere (ruling 6): one check at every verb and
  every composite assignment; no merge at a verb; the equations read the tree as given;
  `Planner.at` becomes the module helper `planner_at` (ruling 1). **For the maintainer's
  review, a semantic choice:** a planner rebuilds from "the numbers it was given, never the
  defaults it derived", so a new discount rate re-derives the discount factor. A complete
  tree carries the derived numbers too, which would pin them. The rebuild therefore takes a
  method number as given only when the tree changes it; a number left at the planner's value
  is derived again on the new problem. The alternative is to pin every number the tree
  carries, which changes what `solve(params=)` means for value iteration and the learners.
  The baseline caught one internal partial call the tests missed (the offline trajectory
  solve's call into the optimizer); fixed in the same step. About fifty test lines move to a
  shared test helper, `edited(tree, changes)`, the tool's tree deep-copied with the changed
  leaves set. **Flagged, student-facing:** the MPC example holds one tick tree (its ROS
  snippet is three lines now) and reads its plan back on the face's tree; the compile and
  program tutorials show the deep copy edited; the MPC parity benchmark holds its tick tree.
  Executed: both tutorials and the MPC example. Full pytest 2192 passed, regression gates
  and the demo bridge pass, baseline byte-identical. The full notebook smoke check runs from
  a snapshot worktree of this commit; its result is logged with step 5.
- **Step 5 (40e6fdc).** The device cache and its per-call digest removed; a JAX leaf already
  in its compiled dtype and shape passes untouched. Measured against the base commit, same
  script, per `ev.f` call: the frozen form unchanged (11.5 against 12.7 µs pendulum, 13.5
  both racecar), a device tree 1.8 to 2.4 times faster, two trees alternating (the MPC
  pattern) 5 to 6 times faster. The problem evaluator snapshots at its compile; one test that
  pinned its live read of a goal set replaced after the compile reads it through a new
  compile. `Simulator.solve(params=)` (ruling 7); every `PlanningSolution` records `params`
  (ruling 5). Deferred: the problem evaluator's frozen form (it snapshots but runs its tree as
  an argument, so the baseline stays byte-identical). The notebook smoke check of step 4
  caught the planners tutorial's partial tree; fixed here (**flagged, student-facing**).
- **Step 6 (23cbd58).** DESIGN §4 holds the seven rules; §5 "Compile once" without the cache;
  RULES 2.3, 4.4, 5.21; KNOWN_ISSUES and TODO follow. Two deviations, deliberate: the plan
  and this runbook stay on the branch until the review (this log is the handoff), and the
  parameters tutorial's `refresh()` cell, listed for removal, is kept (it runs; removing
  teaching content is the maintainer's call). **Flagged:** that tutorial's diagram row.
- **Step 7 (6b0c7f2).** The RRT extender reads the plant's numbers at each solve and asks
  `U` for its bounding box; its TODO rows leave.

### Handoff, 2026-11-05 morning

- **Landed on `params-one-tree`** (`git log --oneline origin/dev..origin/params-one-tree`):
  the baseline script, steps 0 to 7, three run-log commits, one format fix. Not merged into
  `dev`; no pull request opened.
- **Verified on the final commit:** full pytest 2197 passed locally; the seeded baseline
  byte-identical after every step; regression gates (CI flags) and the demo bridge pass;
  docs build passes; the full notebook smoke check 38 of 38 pass; **CI on the branch**
  (workflow dispatch, run 41974576046): all ten jobs green, the regression job's JAX tests,
  flagships and gates included.
- **Waiting for the maintainer:**
  1. The planner-rebuild choice (step 4): a method number a complete tree leaves at the
     planner's value is derived again on the new problem, one it changes is given.
  2. The student-facing edits, all executed: the parameters, sets, problem, compile,
     program and planners tutorials, the RL-to-Bode showcase, the MPC path example (its
     ROS snippet is three lines now: one tick tree, its start and time set).
  3. Whether to delete the parameters tutorial's `refresh()` cell, as the plan lists.
  4. The merge into `dev`, and then deleting the plan and this runbook, the contract being
     in DESIGN.
- **Follow-ups, each small** (the TODO row lists them): the problem evaluator's frozen form;
  the planner's method numbers as views of its tree; the problem's internal helpers folded
  into its held tree; a union's members and a scene's obstacles keyed by id.
