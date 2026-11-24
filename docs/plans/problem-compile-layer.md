# The layer between a problem and its score: a proposal to review

**Status: a proposal, nothing ruled.** It follows the maintainer's second-opinion request of
2026-11-24 on the README pass and its public methods
([problem-verbs-and-batch.md](problem-verbs-and-batch.md)). The maintainer's diagnosis: the
compiled object `J = problem.compile(controller)` is the wrong shape, not the wrong name. It
is an orchestrator, a function, a factory of results and a tuning workshop at once. This
document redesigns that layer from scratch, with every current capability kept and no name
protected. The open decisions are at the end (`PC-1` to `PC-6`); the document is deleted when
they are ruled and the contracts move into DESIGN.

## 1. The workflows, with no design attached

Every workflow reads one map at a different point: problem × candidate × parameters → runs →
scores → one number.

| What the user wants | The mathematics |
| --- | --- |
| Score one controller, deterministic | `J(p)` |
| Score it over uncertainty | a criterion over `J(p, wᵢ)`, `wᵢ` drawn |
| Compare several candidates fairly | the same `wᵢ` for each |
| See the runs (plot, animate) | `p, w → Trajectory` |
| A function to differentiate or batch | `p ↦ J(p)`, `∇ₚJ` |
| Tune gains, co-design the plant, recover cost weights | `min_z J(p(z))`, `z` a subset of the tree's leaves |
| Identify a model from data | `min_z ‖x(·; p(z)) − data‖`, on the run, not the cost |
| Sweep a parameter, sensitivity | `J(p, wᵢ)`, `wᵢ` on a grid |
| A robust or risk-averse score | another reduction (worst case, CVaR) over the same rows |
| Score inside a training loop | the same compiled map, new controller numbers at each update |
| Judge on another problem than the one planned on | the candidate is an argument, not a part of the judge |
| Sampled controllers, open-loop plans, hand-built loops | the same map; only how the loop is closed differs |

**The objects that appear:** the task (the problem); the candidate (a controller, a solution,
a held plan); the closed loop, itself a `System`; the parameter tree `p`; a selection of its
leaves (the decision vector `z` and its map back to the tree); the draws `w`; a run and a
batch of runs; the per-run scores and their statistics; the objective as a function; the
program, the optimizer and its result.

**The verbs:** close the loop, compile, draw, run, score, evaluate (run, score, reduce),
compare, differentiate, pack and unpack, minimize, sweep.

The compiled object of today holds six of those objects: the loop, the tree, the draws, the
selection map, the function and the factory of reports. Counted across the library, the
examples and the tests, it is read as `.evaluate` 86 times, `.params` 68, `.program` 30,
`.trajectory` 28, `.unpack` 22 and `.loop` 18: the closed loop, a plain `System`, is fetched
out of the evaluator because nothing else hands it out.

## 2. The proposal: the problem gets the two layers a `System` has

| | Student layer: verbs on the noun | Research layer: `compile` gives pure functions |
| --- | --- | --- |
| `System` (today) | `plant.compute_trajectory()` | `ev = plant.compile()` → `ev.f`, `ev.h`, `ev.rollout` |
| `PlanningProblem` (proposed) | `problem.evaluate(c)`, `problem.compare(...)`, `problem.program(c, wrt)` | `cp = problem.compile(c)` → `cp.rollout`, `cp.evaluate`, `cp.objective` |

A compiled form is a set of pure functions of the parameter tree. It is never itself a
function, a report or a workshop.

### 2.1 The objects, one job each

| Object | Its one job | Change from today |
| --- | --- | --- |
| `PlanningProblem` | the task and the judge's verbs | gains `closed_loop`, `score`, `program` |
| Candidate | a controller block, a solution, or a held plan | none |
| Parameter tree | every number of the run, one dict | `problem.params` plus one branch per loop block |
| Compiled problem | three pure functions of the tree | loses the call, the program, pack and unpack, the live plot, the held draws |
| `Evaluation`, `Trajectory`, `Comparison` | records | none |
| `MathematicalProgram` | the program, and the map from `z` back to the tree | gains `variables`, `unpack`, `pack` |

Two facts of the code make this smaller than it reads: `problem.params` already is the task's
tree (plant, cost, sets, `x0`, `t0`, discount rate), and `problem.realize(key, n)` already
returns that tree with the drawn leaves batched; `MathematicalProgram` already stores the
tuned paths (`metadata["wrt"]`) and already takes a parameter tree it is solved again on.

### 2.2 The research layer

```python
cp = problem.compile(controller, dt=None, *, solver="rk4", substeps=None,
                     backend="auto", episode_length=None)

cp.params                    # the run's tree, a copy: problem.params + {"controller": ...}
cp.realize(n_trials, key=0)  # the same tree, the drawn leaves batched, n_trials deep
cp.rollout(p)                # p → Trajectory, the running cost integrated with the state
cp.evaluate(p)               # p → Evaluation: one row per member of p
cp.objective(p)              # p → one number, the problem's criterion over the rows
cp.loop, cp.dt, cp.horizon, cp.backend      # the layout, read-only
```

One rule replaces `key` and `n_trials` on this layer: **the tree passed is what runs.** A
plain tree is one run; a tree with batched leaves is one run per member. No draw is hidden,
so the two meanings of `key=None` (QUESTIONS, item 7 of the core notebooks' list) disappear.

### 2.3 The student layer

```python
problem.closed_loop(controller)                       # the System the judge runs
problem.evaluate(controller, n_trials=20, key=0)      # Evaluation
problem.compare(PID=pid, VI=vi, n_trials=20)          # Comparison, the same draws for all
problem.score(traj)                                   # Evaluation of a run already in hand
problem.program(controller, wrt, bounds=None, n_trials=20)    # MathematicalProgram
```

- `n_trials=N, key=k` means exactly `params=cp.realize(N, key=k)`. Each verb also takes
  `params=`; both given is refused.
- Where a controller is accepted, a held `cp` is accepted too, so research code compiles once.

### 2.4 The draws are a batched tree

The idea the maintainer asked to keep in mind (2026-11-24, not ruled): three levels of one
fact. The problem says **what is random** (a law where a number goes,
`distributions={"plant": {"m": Uniform(0.8, 1.2)}}`); `n_trials` says **how many**; `realize`
gives **the numbers themselves**, as the full tree with the drawn leaves batched.

- A sweep is the same thing made by hand: `p["plant"]["m"] = np.linspace(0.5, 2.0, 100)`.
- Two candidates scored on the same tree share their draws by construction, not by a matching
  key.
- A batched leaf is recognised by its shape, one more leading axis than the nominal leaf, the
  rule `rollout_batch(params=)` already follows; a printed tree says it (`plant.m: 20 draws`).
- A leaf is not both drawn and tuned: refused, as a random seed already is.

### 2.5 The workflows

```python
# Score one controller
print(problem.evaluate(pid))

# Monte Carlo, with the runs
report = problem.evaluate(pid, n_trials=200, trajectories=True)

# Compare on the same draws
table = uncertain.compare(VI=vi, RRT=rrt, MPC=mpc, n_trials=20)

# Tune two gains
program = task.program(pid, ["controller.Kp", "controller.Kd"], bounds=bounds, n_trials=20)
best = Optimizer(program).solve()
tuned = program.unpack(best.z)             # the whole tree at the optimum
pid.params = tuned["controller"]           # the tuned controller
print(task.evaluate(pid, n_trials=20))

# The function and its gradient
cp = task.compile(pid, backend="jax")
P = cp.realize(n_trials=20)
jax.grad(cp.objective)(P)["controller"]["Kp"]

# Sweep a plant parameter, one batched call
p = cp.params
p["plant"]["m"] = np.linspace(0.5, 2.0, 100)
cp.evaluate(p).J                           # one cost per mass

# Identify a model from a recorded run
loss = lambda p: ((cp.rollout(p).x - data.x) ** 2).sum()
```

### 2.6 Where today's class goes

| Today on the compiled object | Lands |
| --- | --- |
| closing the loop, compiling, the grid | `problem.compile`, unchanged |
| `n_trials=`, `key=` held at compile | gone; `cp.realize` returns a tree |
| `J(p)` | `cp.objective(p)` |
| `J.evaluate`, `J.trajectory` | `cp.evaluate(p)`, `cp.rollout(p)` |
| `J.program` | `problem.program(candidate, wrt, ...)` |
| `J.pack`, `J.unpack` | `program.pack`, `program.unpack` |
| `J.live_plot` | a plotting callback built from `(cp, program)` |
| `J.loop` | `problem.closed_loop(c)`, or `cp.loop` |

### 2.7 What it costs

- **A core object changes.** `MathematicalProgram` gains three members (`PC-1`).
- **`jax.grad(J)(J.params)` is lost** for `jax.grad(cp.objective)(P)`; the exception RULES 2.7
  records for the compiled `J` leaves with it. A user who wants the letter writes
  `J = cp.objective`.
- **Three rulings of [problem-verbs-and-batch.md](problem-verbs-and-batch.md) are
  superseded** if this is taken: the compiled object named `J` and callable (ruling 6, with
  its `n_trials` and `key` held at compile), and `problem.program` removed (ruling 7; its
  reason, that the answer could only be read back through `J`, falls once the program decodes
  its own `z`). The others stand: one route per scoring verb, comparing is scoring,
  `compile` keeps its name and its meaning, the batch rollout.
- **A student verb compiles at each call** unless it is handed a `cp`. Not measured.
- **Call sites move:** about 86 `.evaluate`, 30 `.program` and 22 `.unpack` across the
  library, the examples and the tests, and the teaching notebooks. Mechanical, but wide.

### 2.8 Not designed here

- The exact keys where the run's tree and `problem.params` differ today (the start set, `t0`,
  the free inputs, a stateful block's `x0`).
- What `objective` means on a hand-made sweep (the criterion over the members, or refused in
  favour of `.J`).
- The integer seeds of random blocks, which sit in the tree and ask `jax.grad` for
  `allow_int=True`.
- The class's name and its instance's name in the docs.

## 3. The alternatives explored

Two independent designers were briefed with the code and the constraints, one under a
function-first lens and one noun-first, without the agent's opinion; a third design was the
agent's. All three made the same five moves, which the proposal keeps: the draws become data;
the compiled thing shrinks to pure functions; the map from `z` to the tree leaves the
evaluator; the closed loop becomes a public `System`; `problem.evaluate` and
`problem.compare` stay the student's verbs.

**The function and nothing else.** `J = problem.compile(c)` stays callable, with the draws a
second argument, `J(p, W)`, and `pack` / `unpack` free functions of the tree. It keeps the
spelling `jax.grad(J)`; it stays one object with a tuning method beside the call, and the
user carries the paths and the base tree to read an answer back. The proposal takes its
draws-as-data and folds `W` into the tree.

**The judge as a block diagram.** The problem wires its cost onto the closed loop as a block
(`dJ/dt = e^{−ρ(t − t0)} g`, output `J = c + e^{−ρ(t − t0)} h`), so the scored loop is an
ordinary `System` and no problem-level compile exists. A throwaway prototype on the pendulum
(an impedance law, a quadratic cost with a terminal term, 5 s at `dt = 0.01`) reproduced the
library exactly:

| Check | Scored loop | Library |
| --- | --- | --- |
| Deterministic score | 22.1428065571 | 22.1428065571 |
| Gradient with respect to `Kp`, `Kd`, the plant's `m`, the cost's `Q` and `R` | equal to every printed digit | |
| Expected cost, 20 draws of the mass, one `rollout_batch` on a batched `m` | 21.6580827255 | 21.6580827255 |
| One warm `J(p)` on JAX | 1.15 ms | 1.00 ms |

(Timing: a macOS laptop, median of 30, the two sides alternated twice; the prototype not
optimized.) Dropped all the same:

- only the running cost is a differential equation. The feasibility margin is a minimum over
  the run, which has no rate; the terminal terms, the failure cost and the criterion are
  functions of a finished run. One line of the score became a block and the others still
  needed a function;
- a sampled loop takes one continuous block, the plant, and refuses the cost block; a plant
  carrying its own cost state runs (the score within 1.5e-6 of the library's), under a block
  named `plant` with one state more;
- the start becomes the stacked `[plant; cost]` state, and the score is read from an output
  port at the last sample.

What it confirmed and the proposal uses: the cost's weights sit in the same tree as the gains
with no merging code, and a batched tree runs through the existing batch path. Not tested:
held plans, the set margins, the failure cost, criteria other than the mean, adaptive solvers,
disturbances.

## 4. The open decisions

Each waits for the maintainer; none is taken. They are put one at a time, in this order.

**`PC-1`, where the map from `z` to the tree lives.**
- *On the program:* `tuned = program.unpack(best.z)`. Changes `MathematicalProgram`, a core
  object; the user never repeats the list of paths.
- *Free functions:* `tuned = unpack(best.z, gains, cp.params)`. The program untouched; the
  user carries the paths and the base tree, and `problem.program` has less reason to exist.
- The two independent designers split on this one.
- **Recommendation:** on the program: it is the object that says what `z` means, and it
  already stores the paths.

**`PC-2`, `problem.program(candidate, wrt, ...)` comes back.** It supersedes ruling 7 of the
README pass. With `PC-1` on the program, tuning needs no compiled object in student code.
**Recommendation:** yes, with `PC-1`.

**`PC-3`, the compiled form is a bundle, not a callable.** `cp.objective(p)` in place of
`J(p)`; ruling 6 and the exception in RULES 2.7 leave. **Recommendation:** yes: one role, the
shape `plant.compile()` already has.

**`PC-4`, the draws as a batched tree** (§2.4): `n_trials=N` means `params=cp.realize(N)`, and
the pure layer takes the tree only. Kept in mind by the maintainer, not ruled.
**Recommendation:** yes.

**`PC-5`, the closed loop and the score as verbs of the problem:** `problem.closed_loop(c)`
and `problem.score(traj)`, the second the existing `evaluate_trajectory`.
**Recommendation:** yes to both.

**`PC-6`, the names:** the class (`ProblemEvaluator` today), its instance in the docs (`cp`
here), `rollout` or `trajectory` for one run, `realize`'s signature (`(key=None, n=1)` today).
No recommendation before the shape is ruled.
