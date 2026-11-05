# One parameters model for every object

**Status:** ruled by the maintainer on 2026-11-05 (§6, seven rulings); ready to run, in the
order of §7. **Release:** v0.4, the clean surface (ROADMAP §5.4, workstream 11). Its steps are
prefixed `PM-`.

How it was reached: a proposal and its first review (2026-11-04), a second independent review
with two reviews of its own, then three designs written from a blank page without reading any of
the above (a student's-notebook lens, a compiled-path lens, a composition lens) and one
consolidation of all seven documents, every contested fact run again on `dev` at e479784. The
three blank-page designs converged on the model below on their own; the maintainer's rulings
settled what they did not.

## 1. The model, in seven rules

1. Every object has `params`, one dict of its numbers, `{}` when it has none; its mathematics
   reads that dict when a call passes `params=None`.
2. A composite's `params` holds its parts' own dicts at the parts' keys, beside its own leaves,
   so every number has one home and a write anywhere lands in it.
3. Structure is the constructor's: sizes, ports, a method, a clock, a part's key, a leaf's
   name; a value is written by item, `plant.params["m"] = 2.0`.
4. A `params` argument is `None` or the whole tree, the same shape as `.params`; nothing merges
   anywhere; a one-off tree is a copy edited like any dict.
5. A tree that crosses a boundary (a tool's `params=`, a composite's assignment) is checked
   against the structure it enters: an unknown key or a changed shape raises, naming the known.
6. A tool snapshots at build; `tool.params` is what `params=None` runs on; nothing reads the
   objects or compares trees per call.
7. A problem's uncertain numbers are `Distribution`s in `problem.distributions`, a sparse tree
   of the same shape; `realize(key, n)` turns them into numbers with a trial axis.

A copy at other numbers is Python's own: `heavy = copy.deepcopy(plant); heavy.params["m"] = 2.0`.
There is no copying verb (ruling 1).

## 2. The API surface

| Name | On | Takes / gives | Semantics |
| --- | --- | --- | --- |
| `params` (read) | every object and tool | a dict, nested by part key, `{}` for none | An object's: live. A tool's: the snapshot taken at build, read-only. |
| `params[k] = v` | every object | one leaf, nested on a composite (`p["plant"]["m"]`) | A dict write into the dict that owns the leaf; the composites hold that very dict (rule 2). |
| `params = tree` | leaves | the complete dict | The declaration, the constructor's gesture. The dict keeps its identity (its contents are replaced), so a composite holding it stays current. |
| `params = tree` | composites | the complete tree, every part named | Routed part by part, each part's complete dict to that part; a missing or unknown part, key or shape raises naming the known ones (today's diagram law, `core/wiring.py:670-703`, on every composite). `loop.params = loop.realize(key)` is such a tree. |
| `params=` | equation methods `f h tf g margin value sdf sample step`, and every tool verb (`compute_trajectory`, `Simulator.solve`, `ev.f/step/rollout/rollout_batch`, `pe.trajectory/evaluate/program`, `planner.solve`, `pv.solve/solution`, `Optimizer.solve`) | `None` or the complete tree | An equation reads the tree as given (`params = self.params if params is None else params`, RULES 5.2). A tool: `None` is the frozen form; a tree is checked (rule 5), made canonical (each leaf the compiled dtype and shape; one extra leading axis on the batched verbs) and passed to the parametric form. |
| `realize(key)` / `realize(key, n)` | random blocks, diagrams, problems, planners | a tree of numbers | A fresh seed per random block under its own stream (`blocks/sources.py:277-280`, `core/wiring.py:334-338`); `None`: every seed `None`, every distribution its mean; `n` stacks the drawn leaves on a leading axis. |
| `distributions` | `PlanningProblem` | a sparse tree of `Distribution`s, `{}` for none | Given at construction as `x0=law` for the start and `distributions={"plant": {"m": Uniform(…)}}` for a number; `nominal()` is the deep copy at `realize(None)` without distributions. |
| `solution.params` | `PlanningSolution` (new field) | the tree the solve ran on | The evidence's numbers; the live problem may move after the solve. |
| `wrt=` | `pe.program`, `jacobian` | dotted paths (`"plant.m"`, `"inner.plant.m"`) | The leaves a verb differentiates or tunes; `lookup` and `with_leaf` in `core/params.py` spell them, internals. |

**Removed.** `with_params` (41 library definitions, 11 test lines, 2 notebooks), `Planner.at`
(5 internal callers, 1 test), `problem_with` (5 callers), `PlanningProblem.replace(params=)`,
`parts_view`, `sample_params`, `nominal_plant_params`, `split_laws`, `coerce_laws`, `start_law`
and `laws` (into `distributions`), `drawn_paths`, `host_tree`, `float_leaves`, `merge_tree`,
`merge_known` (the merge; one structure check replaces it), `own_params` (into the one `params`
property), `call_params`, `params_reader`, `_frozen_params`, `DeviceTree`, `tree_digest` in the
call path (it stays in the offline planner's build key only), `bind`/`merged`/`form` on the
program evaluators, `SampledSystem.params_override` and `discretize(params=)`,
`check_diagram_params`/`validate_cost_params`/`require_known_keys` (into the one structure
check), the `Simulator.params` attribute (into `solve(params=)`), the `/` path spelling, `None`
as "no parameters" on costs and the sampled view (`{}` everywhere; the one `None` is
`MathematicalProgram.params`, a `J(z)` signature on a record), `SumCost.g`'s partial per-call
tree, and the parameters tutorial's `refresh()` cell. `refresh()` itself stays what DESIGN §5
says: a diagram-internal relayout after a block changes size, with no role in parameters.
Kept as internals: `param_view` (`core/costs.py:13-22`), `canonical_tree`, `flatten`, `Batched`.

**What a student types, in full.**

```python
plant.params["m"] = 2.0                              # one number, in place
loop.params["controller"]["Kp"] = 20.0               # nested by block id, the controller's own dict
problem.params["X"]["upper"] = np.array([3., 3.])    # a set's number through the problem

heavy = copy.deepcopy(plant); heavy.params["m"] = 2.0          # a copy at other numbers

p = copy.deepcopy(planner.params); p["plant"]["m"] = 1.3       # one solve at other numbers
sol = planner.solve(params=p)                                  # the user's problem untouched

sim = Simulator(plant, tf=10, backend="jax")                   # a sweep on one compiled tool
trajs = []
for m in masses:
    p = copy.deepcopy(sim.params); p["m"] = m
    trajs.append(sim.solve(params=p))
```

## 3. Composition

**Keys.** A part is keyed by what names it in the algebra: a block by its diagram id
(`controller`, `plant`, else `block.id`, else its class name with `2`, `3`, … for a repeat); a
cost term by `cost.id`, else the scaled cost's cost's id, else its class name numbered the same
way (`core/costs.py:689-696`); a problem's parts by role (`plant cost X U X0 Xf`, each
disturbance source by its port's block id), its own leaves `x0 t0 discount_rate` beside them; a
planner is the problem's keys plus `method` (its numbers) and `seed`; a program's tree is the
subset of the problem's leaves it reads under the same keys (`direct_collocation.py:300-321`), so
`"plant.m"` names one number from problem to program to MPC tick. A scene's obstacles and a
union's members take the same rule (`id`, else class name numbered) instead of today's
positional `"0"`, `"1"` (`core/geometry.py:179-182`, `planning/spatial/scene.py:46-56`; nothing
addresses a positional key today). A scaled term is `{"weight": w, "cost": cost.params}`; a noise
source `{"seed", "sample_period", "law": law.params}`.

**One home, held, not viewed.** A composite builds its dict when its structure is set (wiring,
`__init__`, `replace()`): `{key: part.params}` by reference, its own leaves by value, with
`param_view`s making `problem.x0`, `problem.t0`, `problem.discount_rate`, `ScaledCost.weight`,
`Planner.seed` and the method's numbers attributes of that dict. Today's views, rebuilt on each
access (`core/wiring.py:109-119`, `core/costs.py:372-374, 439-441`, `planning/problems.py:271-290`,
`planning/planner.py:82-92`), lose every write to a composite's own leaf
(`problem.params["discount_rate"] = 0.5` and `total.params[term]["weight"] = 20` both change
nothing, reproduced) and every write to a top-level key. Holding the dicts closes both with no
setter code per composite. It needs one thing: a leaf's assignment keeps its dict's identity
(`clear`, `update`; 0.08 µs), or a composite holding the old dict goes stale (reproduced). That
is two lines in the one `params` property on `System`, `CostFunction`, `Set`, `Field`,
`Distribution`, `Shape`. A block added twice to a diagram, or a cost twice to a sum, would reach
one home by two paths: the sum refuses it today (`core/costs.py:699`); `add_subsystem` will.

**Nested and flattened.** Nesting mirrors the object: `outer.params["inner"]["plant"]["m"]`,
dotted `"inner.plant.m"`. Anonymous composition inlines (`ctl @ plant`, `a + b`); a named part
nests. The flattened compile joins ids with `__` (`core/assembly.py:131-165`) but shares the
block objects and keeps the evaluator's tree nested; `__` never reaches a user.

**Distributions and seeds.** `params` is numeric everywhere, so the whole tree traces
(CONSTITUTION §2, "One signature") and `jax.grad(f)(obj.params)` needs no stripping. A problem
keeps its distributions beside the tree; where one stands, the part's own number is the mean and
the tools plan on `realize(None)`, draw on `realize(key, n)`. Today the distribution sits inside
`problem.params` (`planning/problems.py:285-288`), which makes the view a copy whenever any
plant leaf has one, and `problem.params["plant"]["l"] = 2.0` is then silently lost (reproduced).
A seed is an integer leaf (`None` the mean), never in `wrt`.

**Copies and part replacement.** A copy is `copy.deepcopy`; a system's copy drops the kept
trajectory (a copy carries no last answer, RULES 2.4). `problem.replace(cost=other)` and
`add_subsystem` swap that entry of the held tree. A sampled view (`plant.discretize(dt)`) has no
numbers of its own: `sampled.params is plant.params`.

## 4. The compiled paths

A held tool (`Evaluator`, `Simulator`, `ProblemEvaluator`, the program evaluators under
`Optimizer`, the compiled planner face `pv`) has one tree and two executables behind each method
name, as DESIGN §5 says today. What changes: nothing is read from the objects after build,
nothing is compared per call.

- **Build snapshots.** `tool.params` is a deep copy of the object's tree, leaves canonical. The
  system evaluators already do this (`core/compile/evaluators/dynamic.py:32`); the `Simulator`
  does by accident (`simulation/simulator.py:176`, verified); the problem evaluator does not
  (`planning/evaluator.py:158-184`, rebuilt per access and always passed as an argument) and
  becomes one.
- **Frozen form, `params=None`.** The snapshot's leaves are constants of the trace; the only
  per-call Python is one `is None` test. It earns its keep: when the tree is an argument instead,
  a 1000-step rollout is 13 % slower on a PID loop (13 leaves), 23 % on a three-link arm (20), 42 %
  on the racecar (29), because XLA folds the parameter-only algebra out of the scan body. The
  large-leaf rule stays (`core/compile/evaluators/base.py:44`).
- **Live form, `params=tree`.** The structure check, `canonical_tree`, then the jitted call with
  the tree as argument: one trace per structure, new values never retrace. No device cache and no
  digest: the digest cost more than the transfer it saved at 5 and at 29 leaves and lost badly
  when two trees alternate (an MPC estimate, a sweep). A caller who repeats a value holds device
  leaves (`jax.device_put(tree)` once); `canonical_leaf` passes a JAX leaf whose dtype and shape
  already match untouched (today it converts every one, 13.6 µs per leaf).
- **Once per solve.** `Optimizer.solve(params=)` checks the tree once and hands it to the
  adapter: the JAX forms are `(z, tree)` functions compiled once, the host loop closes over the
  tree for that solve; `params=None` picks the frozen form once per solve, never per `J(z)`. Today
  `form()` digests the bound tree on every `J`, `grad`, `h` call
  (`optimization/evaluators/jax_evaluator.py:240-244`): 23 % of a real offline collocation solve.
- **Edit after build.** Never seen by a held tool; `tool.params` says what it runs on; the
  convenience calls (`compute_trajectory`, `problem.compile`, `planner.solve`) build per call and
  see every edit, as today. The one tool this changes observably is the problem evaluator (no
  test relies on its live read); it gets its own test and gains the frozen path it lacks.
- **Structure change.** After a part is replaced a held tool is stale; its next `params=` refuses
  the new keys or shapes; the frozen form keeps its snapshot.
- **Draws.** `pe.evaluate(params, key, n_trials)` builds the trial tree from
  `problem.distributions` and the sources' seeds and `vmap`s over the drawn axis, the same two
  forms with the drawn leaves batched.

## 5. The twenty use cases, as the user types them

| # | Use case | Code |
| --- | --- | --- |
| 1 | one number everywhere | `plant.params["m"] = 2.0`; `X.params["upper"] = np.array([2., 2.])`; `law.params["std"] = np.array([.5, .5])`; `cost.params["R"] = 0.1 * np.eye(1)` (or `cost.R = …`); `loop.params["controller"]["Kp"] = 20.0`; `problem.params["plant"]["m"] = 2.0` |
| 2 | sweep on a held simulator | `sim = Simulator(plant, tf=10, backend="jax")`; per value `p = copy.deepcopy(sim.params); p["m"] = m; sim.solve(params=p)` |
| 3 | `grad`, whole tree and one leaf | `pe = problem.compile(ctl, backend="jax")`; `dJ = jax.grad(lambda p: pe.evaluate(p).objective)(pe.params)`; `dJ["controller"]["Kp"]` |
| 4 | `vmap` over a family | `p = copy.deepcopy(ev.params); p["plant"]["m"] = jnp.linspace(.5, 2, 10)`; `ev.rollout_batch(x0s, None, dt=0.01, n_steps=1000, params=p)` |
| 5 | a sum of costs, retuned, same tree in the program | `J = SumCost.of(track=base, avoid=10 * obstacle)`; `J.params["avoid"]["weight"] = 20.0`; `J.params["track"]["R"] = 0.1 * np.eye(1)`; `planner.compile().params["cost"]` is that tree |
| 6 | a problem; a solve at other numbers; an uncertain mass; trials | `problem = PlanningProblem(plant, cost=J, X=X, U=U, Xf=Xf, x0=x0, horizon=3., discount_rate=.1, distributions={"plant": {"m": Uniform(.8, 1.2)}})`; `p = copy.deepcopy(planner.params); p["plant"]["d"] = .3; planner.solve(params=p)`; `trees = problem.realize(key=0, n=100)` |
| 7 | a planner's numbers and seed | `planner.params["method"]["step_size"] = .05`; `planner.params["seed"] = 3`; `planner.solve()` |
| 8 | MPC, no retrace | `pv = planner.compile()`; `tick = copy.deepcopy(pv.params)`; per tick `tick["x0"], tick["t0"], tick["plant"]["d"] = x, t, d_hat`; `z, status = pv.solve(params=tick, z0=pv.shift(z, t))`; `plan = pv.solution(z, params=tick)` |
| 9 | held frozen tool after an edit | `ev = plant.compile(backend="jax")`; `plant.params["m"] = 3.0`; `ev.f(x, u)` runs `m = 1` (`ev.params` says so); `ev.f(x, u, params=plant.params)` or `plant.compile()` reads the edit |
| 10 | identification over two leaves | `program = pe.program(wrt=["plant.m", "plant.d"])`; `z = Optimizer(program).solve().z`; `plant.params.update(pe.unpack(z, wrt)["plant"])` |
| 11 | random blocks | `noise.params["seed"]`; `noise.params = noise.realize(key)`; `noise.params["seed"] = None` (the mean signal) |
| 12 | a copy at other numbers | `heavy = copy.deepcopy(plant); heavy.params["m"] = 2.0`; the same on a box, a loop, a problem; a planner is built anew on the copied problem |
| 13 | a sampled view | `sd = plant.discretize(.01)`; `sd.params is plant.params`; other numbers: copy the plant first |
| 14 | nested diagram, flattened compile | `outer.params["inner"]["plant"]["m"]`; `wrt=["inner.plant.m"]`; `outer.compile().params` nested the same |
| 15 | trees given to a call | `None` or complete, everywhere; a typo or a new shape raises at the tool's `params=` and at a composite's assignment, naming the known keys |
| 16 | structure vs value | constructor arguments; a shape is free on an object, fixed on a tool |
| 17 | no parameters | `{}` everywhere; `MathematicalProgram.params=None` the one written exception (`J(z)` alone) |
| 18 | records | frozen; `PlanningSolution.params` the tree solved on; `Trajectory` has none |
| 19 | pickle and deep copy | every object, after a run too; tools are rebuilt by `compile()` |
| 20 | a typo in a key | `sim.solve(params={…, "mass": 2})` and `problem.params = {…, "plant": {…, "mass": 2}}` raise naming `['I', 'd', 'gravity', 'l', 'm']`; the bare `plant.params["mass"] = 2` stays a dict write (ruling 4) |

## 6. The rulings (2026-11-05)

1. **No public copying verb.** `with_params` and `Planner.at` go; a copy is a new instance, or
   `copy.deepcopy` then the usual edit. The only internal need, a problem at other numbers for one
   solve, is a private helper in `planning/planner.py`. No alias, ever (the 2026-10-27 policy).
2. **`distributions=` and `problem.distributions`**, not `laws`: "law" already means four things
   in the library (a probability law, a control law, an optimizer's update law, a rule citation)
   and the object is a `Distribution`; the keyword sits beside `disturbances=`. `x0=law` stays the
   start's spelling. Today's `params=` keyword, which refuses a plain number, goes.
3. **Sets, fields, distributions and shapes are plain classes holding `params`**, attributes as
   views (the `QuadraticCost` pattern); checks run in `__init__` and at every boundary. RULES 5.21
   becomes: frozen dataclasses for records; plain classes with `params` for every parametric object.
4. **A misspelled key is caught at the boundaries only.** `plant.params["mass"] = 2.0` stays a
   dict write; every tree that enters a tool or a composite is checked. No dict subclass (verified
   to mismatch plain-dict pytrees under JAX and to cost six times the write).
5. **`PlanningSolution.params`** records the tree the solve ran on.
6. **Complete trees everywhere**; no partial tree, no merge. CONSTITUTION §6's sentence ("the full
   nested tree, or `None` for the snapshot") stands as written.
7. **`Simulator.solve(params=)`** is the sweep gesture, the same keyword as every tool.

## 7. The steps

Each step is one branch, behaviour-preserving where it can be. Before it, a seeded baseline to
JSON in the scratchpad of every path it touches (each planner family, the problem evaluator on a
stochastic problem, a set-as-cost and a field-as-constraint program, the compiled rollouts, the
trace counts of `test_compile_once`, each tree's key order and leaf dtypes, the packing of `z` and
`wrt`), run twice and `cmp`-ed; after each commit, ruff, the touched tests, the baseline again,
byte-identical (AGENTS "Behavior-preserving refactor runs"); an intended change gets its own
test. The long gates run once, on the finished branch of each step.

- **PM-0. Defects that exist today, no API change.** `Trajectory` stores plain dicts (deep copy
  and pickle of a system after a run; its KNOWN_ISSUES row closes). Every composite setter
  checks the tree against its view before routing (`core/wiring.py:121-128`,
  `core/costs.py:376-380, 443-451`, `planning/problems.py:292-317`): a partial dict that would wipe
  a leaf's other keys raises naming them, a typo through a composite raises. The program
  evaluator's digest moves from `form()` to `bind()`. The offline planner's build key and the
  failure-bound cache include the lowered boxes. The sum cost's `None` becomes `{}`. One test per
  defect. Done when each reproducer raises or passes as stated and the baseline is byte-identical.
- **PM-1. The one `params` property.** `own_params` becomes the identity-keeping property on the
  six base classes (`System`, `CostFunction`, `Set`, `Field`, `Distribution`, `Shape`); a held
  `{"plant": plant.params}` stays current after `plant.params = {...}` (test). Byte-identical.
- **PM-2. Values hold their numbers (ruling 3).** Sets, fields, distributions, shapes, scene,
  track, paths: plain classes, `self.params = {...}` in `__init__`, `param_view` attributes,
  derived quantities (`dim`, `cov`, `Particles.mean`) computed, keys by id or class name,
  `dataclasses.replace` sites rewritten (11), `with_params` removed (ruling 1; 11 test lines, 2
  notebooks). Family by family, the sets' diff shown first. Byte-identical.
- **PM-3. Composites hold their parts' dicts.** Diagram at wiring (`add_subsystem` refuses a
  block added twice), sum and scaled cost at build, problem at `__init__` with `x0`, `t0`,
  `discount_rate` as views, planner with `method` and `seed` views; `distributions` beside the tree
  with `distributions=` (ruling 2; 8 test sites, 4 notebook cells); `parts_view`, `sample_params`,
  `nominal_plant_params`, `drawn_paths`, `start_law`, `problem_with`, `Planner.at` removed, the
  private problem copy kept; the sampled view reads its source. Tests: a write to an own leaf
  lands; `problem.params` is numeric. The planners' numbers are unchanged: byte-identical.
- **PM-4. Complete trees everywhere (ruling 6).** `merge_known` becomes the one structure check
  (`core/params.py`); `solve(params=)`, `evaluate(params=)`, `Optimizer.solve(params=)`,
  `pv.solve(params=)` take `None` or the complete tree; `SumCost.g` too (3 test lines). Tests: a
  partial tree raises naming the missing keys. Byte-identical on complete trees.
- **PM-5. The compiled paths.** One `snapshot` in `core/params.py`; `DeviceTree`, `call_params`,
  `params_reader`, `_frozen_params`, `bind`/`merged`/`form` removed; `CompiledCall` checks once;
  `canonical_leaf` passes a matching JAX leaf; the problem evaluator snapshots and gains the
  frozen form (test: after an edit, `evaluate()` reads the build's values and
  `evaluate(problem.params)` the live ones); `Simulator.solve(params=)` (ruling 7);
  `PlanningSolution.params` (ruling 5). The numbers stay byte-identical; the trace counts are
  asserted anew (two trees alternating: one trace).
- **PM-6. The contract and the tutorial.** DESIGN §4 "Parameters" and §5 "Compile once"
  rewritten as §1–§4 of this plan; RULES 2.3, 4.4, 5.21; the parameters tutorial's table and
  cells (maintainer's review: student-facing); KNOWN_ISSUES (two rows close, the typo row narrows
  to the bare item write); this plan deleted, its contract in DESIGN (one fact, one home). Gates
  at the end: full `pytest -n auto`, the regression gates, the notebook sweep, the docs build; CI
  on the branch.

## 8. What breaks for users today

Counts: `examples/*.py` / notebooks / tests. `with_params` → a deep copy and an edit (0 / 2 / 11);
`PlanningProblem(params={laws})` → `distributions=` (0 / 4 / 8); `problem.params` shows numbers
only (`drawn_paths` 0 / 2 / 3, `sample_params` 0 / 1 / 4); sets, laws and shapes stop being
frozen dataclasses (`dataclasses.replace` on them → a copy, value equality, which raises on
arrays today, goes); `Planner.at` → `copy.deepcopy(planner.problem)`, edit, a new planner (0 / 0 / 1);
`SumCost.g(x, u, t, partial)` → the complete tree (0 / 0 / 3); `solve(params=partial)` → the
complete tree (0 / 0 / 17 to read); a held `ProblemEvaluator` snapshots (no example edits an
object between `compile` and `evaluate()`); `discretize(sys, dt, params=)` and the sampled view's
override go (0 / 0 / 9); `bind` → `solve(params=)` (0 / 0 / 5); imports of `tree_digest`,
`DeviceTree`, `merge_tree`, `own_params` (0 outside the library). Unchanged: every
`f(x, u, t, params=None)` body, the in-place `params["m"] = 2.0` (56 example and 47 notebook
sites), `realize`, `pe.program(wrt=)`, `planner.compile()`.

## 9. Evidence and residual risks

Measured on this machine (CPU, JAX 0.12), each figure the median of alternated blocks:

| What | Frozen | Live tree |
| --- | --- | --- |
| `ev.rollout`, 1000 RK4 steps, PID loop (13 leaves) | 2.45 ms | 2.78 ms |
| the same, racecar (29 leaves) | 4.34 ms | 6.17 ms |
| `ev.f`, pendulum (5 leaves) | 7.5 µs | 25 µs (host leaves), 8 µs (device leaves) |
| program `J(z)`, digest per call today | 68 µs | 45 µs |
| real collocation solve, 178 calls, digest per call | 76 ms | 58 ms without the digest |
| `tree_digest`, 5 / 29 leaves | 14 / 77 µs | |
| two trees alternating through today's device cache, per pair | 325 / 1426 µs | |

Not verified: GPU; a tree large enough to bloat an executable with constants (the large-leaf
rule covers it); the notebooks were read, not run. Risks: the identity-keeping setter means
`a.params = b.params` copies contents rather than aliasing two objects to one dict (no library or
test site does this); `canonical_leaf`'s pass-through must keep the weak-type rule
(`test_a_leaf_given_as_any_number_type_shares_one_trace`); the failure-bound cache keeps one
digest per `evaluate`, at the verb, never per call; the RL planner's `params` (a report, DESIGN
§6's exception) is untouched.
