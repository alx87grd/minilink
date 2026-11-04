# One parameters model for every object

**Status:** proposal for the maintainer's ruling, with an independent review to come.
**Release:** v0.4, the clean surface (ROADMAP §5.4). **Owner of the ruling:** the maintainer
(core architecture, AGENTS "Ask first"). Its steps are prefixed `PM-`.

## 1. The problem

Every number in minilink is a parameter (CONSTITUTION, "every number a parameter"), but two
kinds of objects hold their parameters in two different ways today.

**Things you build are mutable** (`System`, diagrams, costs, `PlanningProblem`): `.params` is
the object's live tree, edited in place, and a composite nests its parts by id.

**Values are frozen** (sets, fields, distributions, geometry shapes, the spatial scene and
track): frozen dataclasses whose `.params` is a dictionary computed from their fields on each
access; a change is a new object made by `with_params(tree)`.

What a user meets today, run on `dev` (2026-11-04):

```python
p = Pendulum();  p.params["m"] = 2.0              # ✓ edits the plant
loop = PID(Kp=10, Ki=1, Kd=2) @ Pendulum()
loop.params["plant"]["m"] = 2.0                   # ✓ edits the plant inside the loop

X = BoxSet([-1, -1], [1, 1])
X.params["upper"] = [3, 3]                        # no error, and X.upper is still [1, 1]
X.upper = [3, 3]                                  # FrozenInstanceError
X2 = X.with_params({"upper": [3, 3]})             # ✓ a new set; X unchanged

g = Gaussian([0, 0], std=[0.1, 0.2])
g.params["mean"] = [1, 0]                         # no error, and g.mean is still [0, 0]
g.params = {"mean": [1, 0]}                       # AttributeError: no setter
g.with_params({"mean": [1, 0]})                   # ValueError: Gaussian takes one spread
                                                  #   (a partial tree loses the std: a bug)
```

The composites disagree too:

| Composite | Nested by | `obj.params = {partial tree}` | Unknown id |
| --- | --- | --- | --- |
| `PlanningProblem` | role | ✓ the named parts change | refused |
| `SumCost` | term id | ✓ | refused |
| `DiagramSystem` | block id | ✗ "must name every subsystem" | refused |
| `IntersectionSet` (frozen; `FieldSet`, `Scene` alike by reading, not run) | index or name | through `with_params`, partial ✓ | refused |

So the same student gesture, "change one number", works on a plant, does nothing silently on a
set or a law, fails on a diagram with a partial tree, and fails on a Gaussian with a partial
`with_params`.

## 2. What being frozen buys today: checked, nothing load-bearing

| Claimed reason | What the code shows |
| --- | --- |
| Tracing (`jit`, `grad`, `vmap`) needs frozen values | Every object already takes its numbers per call: `Pendulum.f(x, u, t, params=None)`, `QuadraticCost.g(x, u, t, params=None)`, `BoxSet.contains(z, t, params=None)`, `Gaussian.sample(key, n, params=None)`, `QuadraticField.value(x, u, t, params=None)`. The mutable plant traces through that argument; a mutable set would trace the same way. The problem evaluator reads its sets through the tree it is passed (`Xf.margin(x, t, Xf_params)`, `planning/evaluation.py:208`). |
| Tools must not write into the user's objects | They do not, and do not rely on freezing for it: a solve at another tree is `problem_with` (`planning/planner.py:329`): `deepcopy(problem)`, then assign the tree. The mutable plant goes through the same copy. |
| A value shared by several holders must not change under them | The library hands out fresh sets: `plant.input_box` and `plant.state.box` are new objects on each access (checked). The things are shared anyway: a solution holds the live problem and its live plant (`sol.problem is prob`, `sol.problem.sys is plant`, both True). |
| Equality and hashing by value | Neither works: `BoxSet(a, b) == BoxSet(a, b)` raises "truth value of an array is ambiguous", and `hash(BoxSet(...))` raises "unhashable type: numpy.ndarray". Nothing relies on either. |

The frozen values predate the per-call `params=` contract; `with_params` exists because their
fields cannot be assigned.

## 3. The fast frozen path under `jit`: unaffected

The fast path is the compiled evaluator's **frozen form**: a compiled method called with
`params=None` uses the values taken at compile as constants of the trace, the fastest run of one
fixed model (DESIGN §5, "Compile once, in the form the call needs").

How it decides today (`core/compile/evaluators/base.py`, `call_params`): it keeps the tree it
compiled with, and on each call compares a digest of the live tree with it. Equal: `None`, the
frozen form. Different: the live tree, the parametric form (traced leaves, no retrace).

- **Systems are already mutable and use this path**: an edit is detected by the digest, so the
  frozen form stays exact. Sets, fields and laws do not enter a system's compiled evaluator, so
  the change does not touch it.
- **The problem evaluator** (`problem.compile(controller, dt)`) passes the problem's tree as an
  argument of its jitted run on every call (`run_compiled(params, x0)`), sets included: no set
  value is baked as a constant there, so mutable sets change nothing on that path.
- **Planners that bake a set at build** read its bounds once (`problem.X.bounding_box()`: the
  value-iteration grid, RRT's sampling box, the transcriptions' bounds, MPPI's input clip). An
  edit after the planner is built is not seen, **exactly as today** when `problem.params =
  {"X": ...}` replaces the set after the build. The offline trajectory-optimization cache is keyed
  on `tree_digest(params_of(problem))` (`trajectory_optimization/planner.py:485`), which reads the
  sets' params, so an in-place edit invalidates it as a replacement does. `solve(params=)` goes
  through `Planner.at` (a rebuild), unchanged.
- **Cost of the check:** the digest per call already runs for systems; no new check is added on
  any hot path. A set's method called with `params=None` inside an outer `jit` reads `self.params`
  as constants, as a frozen field reads today.

Nothing in the change moves a number from "constant of the trace" to "argument" or back.

## 4. The options

**A. Keep the two kinds, fix the edges.** The Gaussian merge fix; a read-only `.params` view on
frozen values (an in-place edit raises and names `with_params`); a partial tree on the diagram.
Small (three changes), no migration. The two models stay, and the student still meets two ways
to change a number.

**B. One model: every object holds a live `params` tree** (recommended). Sets, fields,
distributions, shapes, the scene and the track hold their numbers as systems do: `.params` is
the live tree, edited in place or assigned a partial tree; their named fields (`X.upper`,
`g.mean`) read and write through it. `with_params(tree)` stays, on **every** object (systems
included), as the one copying verb: `deepcopy` then assign. Every function keeps `params=` per
call. Composites nest by id, take a partial tree, and refuse an unknown id; the diagram's setter
joins. **Records stay frozen**: `Trajectory`, `OptimizationResult`, `PlanningSolution`,
`MathematicalProgram` (built once, compiled; its numbers come per call) — outputs of a run or a
build, which nobody tunes (RULES 2.4).

**C. Freeze everything.** Systems become frozen values too (a functional API: `p2 =
p.with_params(...)` for every change). Consistent and pure, but it breaks the student syntax
`plant.params["m"] = 2` across every course notebook; ruled out by CONSTITUTION §3 ("the student
syntax never breaks").

## 5. The rules under option B

1. **`.params` is the object's live tree.** Edit it in place (`X.params["upper"] = ...`) or
   assign a tree (`X.params = {...}`); a partial tree changes the named entries and keeps the rest;
   an unknown key or id is refused with the known ones.
2. **Named fields are views of the tree.** `X.upper`, `g.mean`, `field.center` read the tree; a
   write through one goes into the tree (one home for each number).
3. **Every function takes `params=` per call** and changes nothing. Tools and JAX use this path.
4. **`with_params(tree)` is the copy**, on every object: a deep copy holding the tree; the
   original is unchanged.
5. **Composites nest their parts by id.** Assigning a partial tree routes each entry to its part.
6. **Records are frozen.** A run's or a build's output never changes after it is returned.

## 6. The plan (PM steps)

Behaviour-preserving run (AGENTS, "Behavior-preserving refactor runs"): a seeded baseline of
every path the change touches (value iteration, LQR, RRT, trajectory optimization, the problem
evaluator on a stochastic problem, a set-as-cost and a field-as-constraint program), captured
twice and `cmp`-ed; after each step ruff, the touched tests, the baseline `cmp` byte-identical.

- **PM-1. The base pattern.** One mixin in `core/params.py` beside `tree_digest`: a live
  `params` dict, a setter merging a partial tree and refusing unknown keys, `with_params` as
  `deepcopy` + assign, named-field properties. Tests for the rules of §5 on one class (`BoxSet`).
- **PM-2. Sets** (`core/sets.py`, 6 frozen classes, the input sets, `IntersectionSet` by index):
  the frozen dataclass shell replaced; `dataclasses.replace(self, ...)` call sites rewritten.
- **PM-3. Fields** (`core/fields.py`, 5 classes; `FieldSet`, `MarginField` composites).
- **PM-4. Distributions** (`core/distributions.py`): the Gaussian's spread kept on a partial tree
  (the bug of §1 gone by construction), `std` or `cov` replacing the spread.
- **PM-5. Shapes and spatial** (`core/geometry.py`, 4 classes; `planning/spatial/` scene, track,
  paths, state and workspace fields).
- **PM-6. Systems get `with_params`** (the copy, rule 4); the diagram's setter takes a partial
  tree (rule 5).
- **PM-7. Docs and tutorials.** DESIGN §4 ("Parameters": the six rules replace the two kinds),
  RULES 2.3, the tutorial `core/02_parameters` and `core/03_sets_fields_distributions` tables
  (their `with_params` examples keep working; the in-place form is added beside them, the
  maintainer's review: student-facing). `tests/unittest/test_contract.py` walks every core
  object: it gains the rules of §5.
- **Gates at the end:** full `pytest -n auto`, the regression gates, the notebook sweep, the
  docs build; CI on the branch.

**Scope:** 25 `with_params` definitions collapse into the mixin; their 14 library callers, 7
test callers and 8 example and tutorial lines keep working unchanged (the verb stays). About
twenty classes lose `@dataclass(frozen=True)`.

## 7. Risks and open points

- **Value equality.** Today it raises; after the change a set compares by identity unless it
  defines `__eq__`. Nothing compares sets (checked): no `__eq__` is added.
- **An edit after a planner's build** is not seen by a baked grid or box, as today with a
  replacement; the planners' docstrings say so, and `solve(params=)` is the per-solve route.
- **A solution holds the live problem** (`sol.problem is prob`). Not this change's question: a
  separate entry for QUESTIONS (should a solution snapshot its problem at solve time, RULES 2.4).
- **Records stay frozen**; `MathematicalProgram` is listed as a record: its numbers are passed
  per call (`Optimizer(program).solve(params=)`), and its callables are compiled once.
- **Pickling and `deepcopy`**: the live tree is a plain `dict`, never a `MappingProxyType` (the
  known deepcopy defect, KNOWN_ISSUES).
