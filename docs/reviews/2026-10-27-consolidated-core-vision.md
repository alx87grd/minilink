# The consolidated core vision: one API, one architecture, one naming

**Date:** 2026-10-27
**Audience:** Prof. Alexandre Girard (Maintainer)
**Status:** a consolidated draft, not a ruling; **its fourteen decisions (§8) were taken the
same day as [R141–R153](2026-10-27-core-vision-decisions.md)**, several revised from the
recommendations here (two problem classes, `SampledSystem`, the planner's compiled face, a
separate `as_terminal_cost`). Where they differ, the rulings and
[core-architecture.md](../plans/core-architecture.md) hold; this file stays as the dated record
of how the vision was assembled.
**Built from:** the six independent proposals of the same day
(`2026-10-27-final-architecture-proposal-{a..f}.md`), read in full, and the preliminary
choices of the maintainer (R124–R140, `2026-10-27-design-questions-rulings.md`). Where the
six agree, the vision takes their answer. Where they diverge, §3 names the choice made here
and §8 puts it to the maintainer. Ideas that come from none of the six are marked **(own)**.

Plain words first, then the workboard id in parentheses, as AGENTS asks. "A, B, …, F" name
the proposals; "D4" means proposal D's fourth adjustment, not a decision of this document;
the decisions of this document are "decision 1 … 14" (§8).

---

## 1. The vision in one screen

Minilink is one toolbox around one model. A student writes `ẋ = f(x, u, t; p)` once and never
changes framework to simulate it, draw its Bode plot, close a PID or an LQR on it, run it on a
computer at 100 Hz, plan a trajectory, run an MPC, train a policy, or ask for a Monte Carlo
over draws and the gradient of a closed-loop cost with respect to a gain or a mass. The same
object serves the first lesson and the research run, and the student syntax never breaks on
the way up (CONSTITUTION §1.1, §3).

What holds it together is **one differentiable path**: a `System` with its parameter tree,
compiled once; rolled out by one step model to a `Trajectory`; scored by a `PlanningProblem`'s
cost and sets into an `Evaluation`; lowered, when asked, to a `MathematicalProgram` an
`Optimizer` solves. The v0.3 core makes that path **complete**. Three laws decide every
question below, and the six proposals converged on all three:

1. **One contract, every number a parameter** (R140). A core math object is a function of its
   arguments and `params`; `params=None` reads its frozen defaults; composites nest by name;
   leaves are floats, arrays and the integer `seed`; every function is deterministic given its
   tree; `jit`, `grad`, `vmap` and a parameter family reach every leaf the way they reach `f`.
2. **The continuous model is primary; the sampled world is its sibling algebra.** `f` is a
   rate, never a next-step map. Step systems form a closed algebra on a tick with the same
   grammar (`@`, `>>`, `+`); a continuous object enters it through one adapter, `discretize`
   (and its operator form `%`). The loop a real computer runs, a digital law at its period
   around a continuous plant, is a step diagram at its base clock.
3. **One kernel, one score.** Once `dt`, the scheme and the substeps are set, every loop that
   steps is one discrete map `Φ(x_k, u_k, k; p) = (x_{k+1}, c_k)`, the Bellman transition,
   run by one scan. The evaluator, value iteration, the learners' environment, shooting, the
   native MPC's tick and the Monte Carlo all read that one map, so a policy has one `J`
   whatever tool produced it and whatever loop runs it.

What v0.3 adds to the landed continuous path, in the order of the code: the contract and the
tree (`core/params.py`); the three owners of a start; the step algebra first-class; the
sampled loop as a step diagram; the draws as data (one key, integer seeds, source blocks on
ports, a seeded policy that is a plain block); the one score; the parametric program and the
native kernel; the MPC as a pure step block whose closed loop compiles and differentiates;
the `Field`.

---

## 2. Where the six proposals agree (taken as the vision)

Each line is in all six, or in five with the sixth silent. These are not put to the review;
the maintainer may still veto any of them.

| Point | The agreed answer |
| --- | --- |
| The stage cost | integrated **with the state** by the same scheme and substeps: the augmented state `[x; c]`, `ċ = e^{−ρ(τ−t_k)} g(x, u_k, τ)`; under Euler `c_k = g(x_k, u_k, t_k)·dt` **bitwise** (value iteration's and RL's numbers do not move; verified by C, D, E, F); under RK4 fourth-order exact |
| Sets and costs | keep their continuous signature and read `t`; a stepping tool binds `t_k = t₀ + k·dt`; no discrete twins of `Set` or `CostFunction`; a natively discrete cost reads `t` as the tick |
| The failure rule | today's `score_run`, once: the run goes to the horizon, the least margin over the samples decides `failed`, `Xf` at `x_N` decides `reached`, the price switch; RL's early stop is a training choice that reports the judge's `J`; margins between samples are not checked in v0.3 (documented) |
| A continuous loop is scored | exactly as a `StepSystem` on its `dt`: the trapezoid on samples becomes the plain sum `Σ γ^k c_k + γ^N h(x_N)`; the evaluator's `J` moves once (0.04 % under RK4, about 1 % under Euler on the test loops) |
| The sampled loop's model | one pure base tick: every block fires on `k mod d = 0` as a select, the held outputs of slow blocks are state (a hold is a memory, R104), the plant is `discretize(plant, dt_base, substeps)`; propagation within the tick (R125); idle blocks computed and discarded in v0.3, the hyperperiod layout a later optimization with the same numbers |
| The clock orchestrator | a **runner** over the same pure blocks (realtime, hardware, host-solver blocks, an adaptive plant between samples), never a second model; its test is byte-identity with the pure tick at the samples (verified on NumPy by C, D, E, F) |
| `%` on a stateful law | `pid % dt` **discretizes** the law (`discretize(pid, dt)`), the textbook digital implementation GRO501 teaches; a static law is tagged with its period; a `StepSystem` is tagged |
| `computer @ plant` | a `StepDiagramSystem` subclass at the base clock, keeping the continuous plant as its `source`; `@` resolves ports as `ctl @ plant` does; the substep default from the plant's time constant, one owner (`AUTO_DT_SCALE`) |
| Mixed operands | refused, naming `%` or `discretize`, **except** a step left operand carrying a period on a continuous plant, which is the sampled loop ("the left side sets the clock") |
| Step rollouts | native arrays, traced end to end, `rollout_batch`; `StepRollout` folds into `Trajectory` with `t = k·dt` (`t = k` on a tickless system) |
| The draws | integer keys everywhere, the library's counter-based cipher on both backends (closes the first KNOWN_ISSUES entry); `realize(key, n_trials)` derives one stream per consumer **by path** (`child_seed(key, "trial i/d.seed")`), so adding a random block never moves another's draw; a seed is an integer leaf with a trial axis |
| Disturbances | source blocks wired onto the plant's ports, on both problem classes; the shortcut `disturbances={"d": WhiteNoise(...)}` builds `sources >> plant` once; a bare law refused naming the fix (R128) |
| The seeded policy | `GaussianPolicy(mean, log_std, *, sample_period, seed=None)` where **`mean` is any `System`** (a `StateFeedback`, a `NeuralPolicy`); `u = μ(x) + exp(log_std)·ε(seed, ⌊t/Δ⌋)`; `seed=None` is the mean law; `log_prob`, `entropy` on the block; the RL heads rebuilt on it (a precision of R129) |
| The program | one noun `MathematicalProgram(..., params=None)`; a transcription's `params` **is** `problem.params` word for word (`x_start` the one word for the start); `Optimizer.solve(z0=, params=)` merges a partial tree once at the boundary; `params_at(z)`; `ParametricMathematicalProgram` folds in; the performance gate stays (a baked export if carrying `params` ever costs speed; D and E measured no retrace and no per-call cost) |
| The native kernel | `init(z0, params) → state`, `step(state, params) → state`, pure on the array module, projected on the box; a fixed count `K` runs as one `lax.scan`, traceable and differentiable; `as_step_system` the lesson's view (R132) |
| The MPC block | `ModelPredictiveController(planner, *, sample_period, parameter_ports=())`, a `StepSystem` whose **state is the plan `z`**; `step` solves from `shift(z)` with the measurement bound as `x_start`; `h` reads the first input of the state (one period of delay, R123); the latch, the stateless block, `Command`, the dual-rate export go; a host-solver planner keeps the loop on the NumPy face, a native fixed-`K` planner lets the whole closed loop trace |
| The planner never loop-aware | the MPC owns tick, memory, delay and replanning (R135) |
| Fields | `Field`, `QuadraticField` in `core/fields.py` with `value`, `gradient`, `as_set`, `as_input_set`, `as_cost(weight, shaping, *, terminal=False)`; `Set.as_cost` reuses `Field.as_cost`; `as_constraint → as_set`; `cost_to_go: Field` on every solution |
| The tree module | one `core/params.py` (`lookup`, `with_leaf`, `merge`, `leaves`, `pack`, `unpack`, `realize`, `batch_axes`) replacing the four copies; `params = {}` for an object with none; the contract test walks every public core math object on both backends |
| The homes | `discretize`, the sampled block and the sampled loop in `core/discrete/`; R100's lazy `core → simulation` import deleted; the runner in `simulation/discrete/`; `GaussianPolicy` in `control/stochastic.py`; the kernels in `optimization/kernels/` |
| Names | R134 as ruled: `backend=`, `adapter`, `sample_period=`, `mount`, `.` walks a tree and `:` names a signal, `…Policy`, `parameter_ports=`, `params_at` |

---

## 3. Where they diverged, and the choice made here

Each row is a decision of §8; this table is the map.

| Divergence | A | B | C | D | E | F | Chosen here | Decision |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| The discrete problem | internal `DiscreteProblem` | the same `PlanningProblem` on a `StepSystem` | public `StepProblem` | internal lowering | public `StepPlanningProblem` + `StepCost` | the same `PlanningProblem` + `transition` | **the same `PlanningProblem` on a `StepSystem`, with `transition`** | 1 |
| Where `c_k` is computed | in the step | post-pass `vmap` | in the step | in the step | post-pass `vmap` | in the step | **in the step** | 2 |
| The stochastic problem's keywords | one noun, `x_start=law`, `params=laws` | — | two nouns, `x_start=law`, `params=` | `x_start=law`, `params_distribution=` | `x0_distribution=`, `params=` | `x0=law`, `params=` | **`x_start=` a point, a `Set` or a law; `params=` the tree of laws; one noun (own, with A)** | 3 |
| `realize`'s shape | `(x0s, params)` | **one tree** | `(x0s, params)` | `(x0s, params)` | `(x0s, params)` | `(x0s, params)` | **one tree** | 4 |
| `%`'s default scheme on a stateful law | — | rk4 | rk4 (via `discretize`) | euler | euler | euler (`StepSchedule.solver`) | **euler for a law, rk4 for a plant (own rule: a law is implemented, a plant is simulated)** | 5 |
| The computer object | `Computer(StepDiagramSystem)` | `Computer` + `StepSchedule` in core | `Computer` + `StepSchedule` in core | **periods on blocks, no class** | periods on blocks, `Held`; `Computer` the runner | `Computer` + `StepSchedule` in core | **periods on blocks; no `Computer` or `StepSchedule` class; "computer" the runner's and the drawing's word** | 6 |
| The loop's name | `SampledDataSystem` now | `HybridDiagram` | `HybridDiagram` till v0.9 | `SampledDataSystem` now | `HybridDiagram` till v0.9 | `SampledDataSystem` now | **`SampledDataSystem` now** | 7 |
| The verbs on the sampled loop | two (R127) | one | two | one, `solver=` | `compute_trajectory` on the step runner by default | one | **two verbs that agree: `compute_rollout` the step verb; `compute_trajectory(solver=)` dense, the pure scan with substeps by default, `"scipy"` the orchestrator** | 8 |
| The planner interface | `compile/plan/shift/unpack` | `solve_from/warm_start/plan_of` | `compile/solve/shift` | `program/solve/shift/plan` | `prepare/plan/shift` | `solve/shift/program` | **`solve(params=, z0=)`, `shift(z)`, `program`** | 9 |
| Where `iterations=K` lives | planner | — | planner | MPC block | MPC block | planner (`n_iter`) | **the planner (it owns its solver), keyword `iterations`** | 9 |
| The MPC's inner tree key | `plant` | — | `plant` | `plant` | `plant` | `model` | **`plant`, word for word** | 9 |
| The fixed-`K` solve on `Optimizer` | `iterations=` | — | `iterations=` | `iterate(n_steps=)` verb | `solve(iterations=)` | `n_iter=` | **`Optimizer(..., iterations=K)`, one `solve`** | 10 |
| RL's `gamma` | — | — | **derived from `discount_rate`** | — | — | — | **derived** | 11 |
| `GridField` | later | later | later | **v0.3** | later | later | **v0.3, with its consumer (value iteration's `cost_to_go`)** | 12 |
| `NeuralPolicy` rename | v1.0 | v1.0 | v1.0 | v1.0 | v1.0 | **now** | **now, in the v0.3 sweep** | 13 |
| `jax.grad` over a tree with an integer seed | — | — | — | — | `pe.float_params` | **`allow_int=True`, documented** | **`allow_int=True`, documented** | 14 |

---

## 4. The user's view: the main objects and their verbs

| Object | In the textbook | Verbs | `params` |
| --- | --- | --- | --- |
| `System` / `DynamicSystem` | `ẋ = f(x, u, t; p)`, `y = h(x, u, t; p)` | `f`, `h`, `tf`; `compute_trajectory`, `linearize`, `plot_*`, `animate`; `+`, `>>`, `@`, `%`; `compile(backend=)`, `realize(key)` | its numbers; `x0` its own field (R124) |
| `DiagramSystem` | the block diagram | the same; `connect`, `expose`, `trajectory_of`; `x0` a write-through view of its blocks | `{block_id: block.params}`, live |
| `StepSystem` / `StepDiagramSystem` | `x_{k+1} = F(x_k, u_k, k; p)` on a tick; `sample_period` seconds per tick or `None` | `step`, `h`; `compute_rollout(n_steps)` → `Trajectory` with `t = k·Δ`; `jacobian`, `poles`; the same operators | its numbers |
| `discretize(sys, dt, *, solver="rk4", substeps=1)` | `x_{k+1} = Φ(x_k, u_k)`, `u` held | the one adapter into the step algebra; every port of the source kept, read at `t_k` | the source's, live |
| `block % dt` | the law as a computer runs it, every `dt` | a `StepSystem` tagged with its period; a stateful law discretized (`solver="euler"`) | the block's |
| `SampledDataSystem` (`computer @ plant`) | the sampled-data loop: computer, ZOH, plant, sampler, at the base clock | a `StepDiagramSystem`: `compute_rollout` (the ticks), `compute_trajectory(solver=)` (dense), `plot_diagram` (two sides), `animate`, the realtime runner | `{"controller": …, "plant": …}`, the holds as state |
| `Trajectory` | `(t, x, u)` sampled | `of(role)`, `with_cost`, `resample`, `plot`; a pytree | none |
| `Set` / `InputSet` | `z ∈ Z(t)`: `margin(z, t; p) ≥ 0` | `margin`, `contains`, `sample(key, n, params)`, `bounding_box`, `&`, `as_cost` | the set's numbers; `{}` when none |
| `Distribution` | a law over a vector | `sample(key, n, params)`, `mean`, `support` | `mean`, `std` / `cov`, `lower`, `upper` |
| `CostFunction` | `g(x, u, t; p)`, `h(x, t; p)` | `g`, `h`, `+`, `*`, `g_max`, `h_max`; `integrate` stays for a logged run | `Q`, `R`, `S`, `xbar`, `ubar`; a sum by term |
| `Field` | `V(x, u, t; p)`: a value, a Lyapunov function, a barrier | `value`, `gradient`, `as_set`, `as_input_set`, `as_cost(terminal=)`, `plot` | `center`, `P`; a table |
| sources (`Step`, `Sine`, `WhiteNoise`, `TrajectorySource`) | a signal of `t` | `h`; a random one `realize(key)` | levels, `psd`, `sample_period`, `seed`, `values` |
| `GaussianPolicy` | `u = μ(x) + σ ε(seed, ⌊t/Δ⌋)` | `h`, `log_prob`, `entropy`, `realize(key)` | `{"mean": μ's tree, "log_std", "seed"}` |
| `PlanningProblem` | the task: plant, sets, cost, horizon, discount, price, start (a point, a `Set` or a law), laws over parameters, sources on ports | `compile(controller, dt)`, `discretize(dt)`, `transition(x, u, k)` on a step problem, `realize(key, n_trials)`, `replace`, `nominal`, `cost_bound` | `{"plant", "cost", "X", "U", "X0", "Xf", "x_start", "discount_rate", <source ids>}`, a live view |
| the problem evaluator (`pe = problem.compile(...)`) | the closed loop run to the horizon and scored | `params`, `trajectory(params, key)`, `evaluate(params, key, n_trials)`, `program(wrt, bounds)`, `pack`, `unpack` | one tree: every block by id (each stateful block with its `x0`), the problem's leaves, each free input |
| `Evaluation` | the scores, one row per run | `objective`, `mean`, `std`, `worst`, `cvar`, the rates; `print`, `plot_trajectories` | none |
| `MathematicalProgram` | `min J(z; p)` s.t. `h = 0`, `g ≥ 0`, bounds | `J`, `h`, `g` of `(z, params=None)`; `params_at(z)` | the problem's tree; `{}` for a hand-written one |
| `Optimizer` | the solver | `solve(z0=, params=)`; `iterations=K` the fixed-count traced form; `as_step_system()` | the method's numbers (`eta`, `b1`, …) |
| planners | a method on a problem | `solve(params=, z0=)` → `PlanningSolution`; `shift(z)`; `program` where the planner has one; the method's own plots | — |
| `PlanningSolution` | the policy and its evidence | `policy`, `trajectory`, `evaluation`, `cost_to_go: Field`, `solver` (the record, `z` inside); the generic plots | — |
| `ModelPredictiveController` | the receding-horizon law, one period late | a `StepSystem`: `step`, `h`; `mpc @ plant` | its planner's problem tree (`plant`, `cost`, the sets), the kernel's `eta`, `x0` the first plan |
| `evaluate`, `compare` | the facades | `evaluate(problem, ctl, dt=, n_trials=, key=)`, `compare(problem, A=, B=, dt=)` | — |

The rule under the table: **an object's math is `method(args, params=None)`; `params` is a
dict whose leaves are numbers (`seed` the one integer); a composite nests its parts by name;
a dotted path walks it (`controller.Kp`, `cost.Q`), a colon names a signal (`plant:u`); the
same call gives the same numbers on NumPy and JAX for the same seed.**

---

## 5. Four scripts in the final API

Lesson 1 (a plant, `pid @ plant`, `compute_trajectory`, `plot_*`) and the classical analysis
(`linearize`, `bode`, `margins`, `step_info`) are today's API unchanged; the six proposals
agree. The four scripts below are what v0.3 adds, on one plant.

### 5.1 The digital implementation, its score and its gradient

```python
import jax, numpy as np
from minilink import Pendulum, PID, PlanningProblem, QuadraticCost, Optimizer

plant = Pendulum()
x_up = np.array([np.pi, 0.0])
pid = PID(Kp=20.0, Ki=5.0, Kd=4.0)

loop = pid % 0.01 @ plant                 # the PID discretized at 100 Hz, the plant held between samples
ticks = loop.compute_rollout(tf=3.0)      # the step verb: the samples, pure, what evaluate scores
dense = loop.compute_trajectory(tf=3.0)   # the same scan with the plant's substeps recorded (dense)
check = loop.compute_trajectory(tf=3.0, solver="scipy")   # the orchestrator, the adaptive plant: the cross-check
loop.plot_trajectory(signals=("plant:y", "controller:u"))
print(loop.poles())                       # the sampled-data loop against the unit circle

problem = PlanningProblem(plant, cost=QuadraticCost.from_system(plant, xbar=x_up), tf=3.0, x_start=x_up - [0.4, 0])
pe = problem.compile(loop)                # dt is the loop's base clock; a different dt refused
print(pe.evaluate())                      # J = Σ γ^k c_k + γ^N h: the ticks of compute_rollout, scored
dJ = jax.grad(lambda p: pe.evaluate(p).objective, allow_int=True)(pe.params)
print(dJ["controller"]["Kp"], dJ["plant"]["m"])

program = pe.program(wrt=["controller.Kp", "controller.Ki", "controller.Kd"])
best = Optimizer(program, method="l-bfgs").solve()
print(pe.evaluate(program.params_at(best.z)))
```

The tree: `pe.params = {"controller": {"Kp", "Ki", "Kd", "tau", …, "x0": (2,)}, "plant": {"m", "l", "I", "gravity", "d"}, "cost": {…}, "x_start", "discount_rate", "r"}`.
Wrong calls: `pid @ discretize(plant, 0.01)` → refused, naming `pid % 0.01 @ plant`;
`problem.compile(loop, dt=0.02)` → refused, the loop runs at 0.01 s.

### 5.2 An observer, a Monte Carlo over starts, parameters and noise, from one key

```python
from minilink import Pendulum, QuadraticCost, PlanningProblem, Gaussian, Uniform, WhiteNoise, lqg, evaluate

plant = Pendulum()
x_up = np.array([np.pi, 0.0])
ctl = lqg(plant, Q=np.diag([10.0, 1.0]), R=0.1 * np.eye(1), Q_n=0.05, R_n=1e-3, x_bar=x_up)   # observer + gain

problem = PlanningProblem(
    plant, cost=QuadraticCost.from_system(plant, xbar=x_up), tf=4.0,
    x_start=Gaussian(x_up + [0.3, 0.0], std=[0.1, 0.1]),         # the start is a law: the problem is stochastic
    params={"plant": {"m": Uniform(0.8, 1.2)}},                   # a law where a number goes, in the tree's own shape
    disturbances={"d": WhiteNoise(1, psd=0.05, sample_period=0.001)},   # a source block on the port d
    X=plant.state.box, infeasible_cost=True, criterion="expectation",
)
report = evaluate(problem, ctl, dt=0.001, n_trials=200, key=1)   # 200 draws of (x0, m, the noise seed), one vmap
print(report)
params = problem.realize(key=1, n_trials=3)                       # one tree: x_start (3, 2), plant.m (3,), d.seed (3,)
print(params["x_start"], params["plant"]["m"], params["d"]["seed"])
print(problem.compile(ctl, dt=0.001).params["controller"]["observer"]["x0"])   # the observer's own start, a leaf
```

### 5.3 Trajectory optimization, the program rebound, the MPC, and the native MPC tuned by gradient

```python
from minilink import (Pendulum, PlanningProblem, QuadraticCost, SingletonSet,
                      TrajectoryOptimizationPlanner, ModelPredictiveController, Optimizer)

plant = Pendulum()
x_goal = np.array([np.pi, 0.0])
task = PlanningProblem(plant, cost=QuadraticCost.from_system(plant, Q=np.diag([1.0, 0.1]), R=0.01 * np.eye(1), xbar=x_goal),
                       tf=3.0, x_start=np.zeros(2), Xf=SingletonSet(x_goal), U=plant.input_box)

planner = TrajectoryOptimizationPlanner(task, transcription="direct_collocation", n_steps=40, backend="jax", method="ipopt")
sol = planner.solve()                                            # the offline plan and its evidence
heavy = planner.solve(params={"plant": {"m": 1.3}}, z0=sol.solver.z)   # the program built once, rebound: no retrace
print(planner.program)                                           # n_z, the rows, params = task.params

mpc = ModelPredictiveController(planner, sample_period=0.05)     # a StepSystem; its state is the plan; Ipopt is host code
loop = mpc @ plant                                               # the MPC's period sets the clock
loop.compute_trajectory(tf=4.0)                                  # the NumPy face: the orchestrator ticks the host solver
loop.animate(overlays=mpc.horizons(loop.traj))

# The native fixed-iteration MPC: K Adam steps on single shooting, its closed loop one scan
fast = TrajectoryOptimizationPlanner(task.replace(tf=0.75), transcription="single_shooting", n_steps=15,
                                     method="adam", iterations=20, backend="jax")
mpc = ModelPredictiveController(fast, sample_period=0.05)
pe = task.compile(mpc @ plant)                                   # 60 ticks, K = 20 inside each, one lax.scan
print(pe.evaluate())
print(pe.evaluate(params={"x_start": x_goal + np.array([[-0.8, 0], [-0.3, 0.5]])}))   # a batched leaf: two runs, one vmap
program = pe.program(wrt=["controller.cost.Q", "controller.eta"])   # the MPC's own weight and step size, by the closed-loop J
best = Optimizer(program, method="adam", iterations=200, eta=0.05).solve()
print(program.params_at(best.z)["controller"]["cost"]["Q"])
```

The tree: `loop.params = {"controller": {"plant": {…the MPC's model…}, "cost": {…}, "U": {}, "Xf": {…}, "eta", "x0": z0}, "plant": {…the real plant…}}`;
model mismatch is `loop.params["plant"]["m"] = 1.3` with the controller's `plant.m` left at 1.

### 5.4 Value iteration, LQR and reinforcement learning on one `J`, a field as terminal cost

```python
from minilink import (Pendulum, PlanningProblem, QuadraticCost, Uniform, LQRPlanner,
                      DynamicProgrammingPlanner, ReinforcementLearningPlanner, compare)

plant = Pendulum()
x_up = np.array([np.pi, 0.0])
running = QuadraticCost.from_system(plant, Q=np.diag([1.0, 0.1]), R=0.1 * np.eye(1), xbar=x_up)
problem = PlanningProblem(plant, cost=running, X=plant.state.box, U=plant.input_box, tf=np.inf,
                          discount_rate=0.5, x_start=Uniform([-np.pi, -6.0], [np.pi, 6.0]), infeasible_cost=True)
DT = 0.05

lqr = LQRPlanner(problem.nominal()).solve()
V = lqr.cost_to_go                                       # a QuadraticField: (x − x̄)ᵀ P (x − x̄)
region = V.as_set(upper=20.0)                            # the ellipsoid {V ≤ 20}, a Set

step = problem.discretize(DT, solver="euler")            # the same PlanningProblem on discretize(plant, DT)
x_next, c_k = step.transition(np.zeros(2), np.ones(1), k=0)   # the Bellman transition every stepping tool reads
print(step)                                              # PlanningProblem on Discretized(Pendulum), dt 0.05, γ 0.9753, episode 10 s

vi = DynamicProgrammingPlanner(problem, x_grid=(101, 101), u_grid=(11,), dt=DT).solve()   # reads step.transition
rl = ReinforcementLearningPlanner(problem, dt=DT, algorithm="ppo", seed=0).solve(timesteps=400_000)   # γ = e^{−ρ dt}, derived

race = compare(problem, VI=vi, LQR=lqr, PPO=rl, dt=DT, solver="euler", n_trials=100, key=1)   # one judge, the same draws
print(race)                                              # VI's table at x0 equals its J row to the interpolation error
finite = problem.replace(tf=2.0, cost=running + vi.cost_to_go.as_cost(terminal=True))   # the DP table ends the horizon
```

---

## 6. The API, object by object

Signatures are given once; "today" cites the code as it stands. The one contract applies to
each: `params=None` reads the frozen defaults; inside the library a tree is complete, a user
boundary merges a partial tree once (R121, R131); sizes (`n_steps`, `iterations`, `n_trials`,
`substeps`) are structural keywords, never leaves.

### 6.1 Systems, the step algebra, the sampled loop

```python
class System:                                   # core/system.py:26
    params: dict                                # {} when the block has none (never None)
    x0: ndarray                                 # the block owns its start (R124)
    def h(self, x, u, t=0.0, params=None); def tf(self, x, u, t=0.0, params=None)
    def realize(self, key) -> dict              # this block's params with a fresh seed, by id
    def __add__ / __rshift__ / __matmul__       # +, >>, @ (unchanged; @ gains the sampled branch)
    def __mod__(self, sample_period) -> StepSystem   # block % dt (today a Computer, simulation/discrete/computer.py:34)

class DynamicSystem(System):  def f(self, x, u, t=0.0, params=None)
class DiagramSystem(DynamicSystem):             # core/diagram.py:19
    x0, n, state: views of the blocks; the x0 setter writes through (derived_x0 and refresh()'s state part go)
    params: property                            # {block_id: block.params}, live (wiring.py:54, unchanged)

class StepSystem(System):                       # core/discrete/step_system.py:16
    sample_period: float | None                 # seconds per tick; None for a pure tick system
    def step(self, x, u, k=0, params=None); def h(self, x, u, k=0, params=None)
    def compute_rollout(self, n_steps=None, u=None, *, tf=None, x0=None, backend="numpy") -> Trajectory   # t = k·Δ
    def jacobian(...); def poles(self, x_bar=None, u_bar=None, k=0, params=None)

class StepDiagramSystem(StepSystem):            # core/discrete/step_diagram.py:13
    # +, >>, @ return a StepDiagramSystem (R126); a continuous operand beside a step one refused naming % / discretize
    # multi-rate: each block's sample_period a multiple of the diagram's base period (their gcd); a block with
    # divisor d > 1 fires on k mod d == 0 (a select) and its held outputs are state slots of the diagram;
    # propagation within the tick (R125); an algebraic loop refused at wiring
    sample_period: float | None                 # the base period, derived from the blocks

def discretize(sys, dt, *, solver="rk4", substeps=1) -> Discretization   # moves to core/discrete/ (today analysis/discretization.py:21)
class Discretization(StepSystem):               # every port of the source kept, read at t_k = k·dt (today only y, :148)

def sampled(block, dt, *, solver="euler", substeps=1) -> StepSystem      # the body of %:
    # a static block: tagged with its period; a DynamicSystem: discretize(block, dt, solver, substeps); a StepSystem: tagged

class SampledDataSystem(StepDiagramSystem):     # replaces HybridDiagram (core/discrete/hybrid_diagram.py:30, today not a System)
    source: DynamicSystem                       # the continuous plant it was built from
    sample_period: float                        # the base clock
    def compute_rollout(...) -> Trajectory      # the ticks: the pure scan, evaluate's semantics
    def compute_trajectory(self, tf, *, solver=None, substeps=None, x0=None, backend="numpy") -> Trajectory
        # solver=None: the same pure scan with the plant's substeps recorded (dense, t in seconds)
        # solver="scipy": the clock runner, the adaptive plant between held-input changes (NumPy)
    def plot_diagram(...)                       # the two-sided drawing, kept; the computer side labelled "computer"
    def animate(...); def run(self, runner)     # realtime, hardware in the loop, over the same pure blocks
```

`(ctl % Δ) @ plant` is the one place a step operand meets a continuous one: the plant becomes
`discretize(plant, base_period, substeps=n)` under `plant`, the computer's blocks under
`controller` (a multi-block computer nested), the held commands as state; `n` defaults from the
continuous rule (`AUTO_DT_SCALE × τ_min` against the base period, `simulation/time_grid.py`).
An MPC or a seeded policy carries its period already, so `mpc @ plant` works without `%`.

### 6.2 Trajectories, sets, costs, distributions, sources, fields

```python
@dataclass(frozen=True)
class Trajectory:                               # core/trajectory.py:52, a pytree
    t; x; u; signals; roles; dt: float | None   # dt: the step when t is a uniform grid (a step run)
    def of(self, role); def with_cost(self, cost, *, discount_rate=0.0, params=None); def plot(...)

class Set:                                      # core/sets.py:22
    params: dict = {}
    def margin(self, z, t=0.0, params=None); def contains(...); def sample(self, key, n=None, params=None); def bounding_box(self)
    def as_cost(self, weight=1.0, shaping=None) -> CostFunction   # Field.as_cost on the margin (R133)
class InputSet:  def margin(self, u, x=None, t=0.0, params=None)

class CostFunction:                             # core/costs.py:29
    params: dict = {}
    def g(self, x, u, t=0.0, params=None); def h(self, x, t=0.0, params=None); g_max; h_max; +; *
    def integrate(self, traj, *, discount_rate=0.0, params=None)   # stays: the trapezoid on a logged run (with_cost, evaluate_trajectory)

class Distribution:                             # core/distributions.py:24
    params: dict                                # Gaussian {"mean", "std" | "cov"}, Uniform {"lower", "upper"}, Particles {"points"}
    def sample(self, key: int, n=None, params=None)   # the cipher on both backends; the generator and JAX-key paths go
    mean: property; support: Set | None

class WhiteNoise(Source)                        # blocks/sources.py:211, unchanged: h = ε(seed, ⌊t/Δ⌋), realize(key)
class TrajectorySource(Source)                  # its table a leaf params["values"] (tunable as an open-loop plan)

class Field:                                    # core/fields.py (today planning/spatial/state_fields.py:26, StateField)
    params: dict = {}
    def value(self, x, u=None, t=0.0, params=None); def gradient(...)       # jax.grad on JAX, central differences on NumPy
    def as_set(self, lower=None, upper=None) -> Set; def as_input_set(...) -> InputSet
    def as_cost(self, weight=1.0, shaping=None, *, terminal=False) -> CostFunction   # g (or h) = weight·shaping(V)
    def plot(self, X, ...)
class QuadraticField(Field):   params {"center", "P"}; as_set(upper=level) -> EllipsoidSet (sample, bounding_box)
class GridField(Field):        a table on a StateSpaceGrid, value by interpolation (value iteration's cost_to_go)
```

### 6.3 The problem, its draws, its discrete face

```python
class PlanningProblem:                          # planning/problems.py:29
    def __init__(self, sys, x_start=None, x_goal=None, cost=None, X=None, U=None, X0=None, Xf=None,
                 tf=None, infeasible_cost=None, discount_rate=0.0, *, params=None, disturbances=None,
                 criterion="expectation"):
        # x_start: a point, a Set (a set of starts) or a Distribution (a stochastic problem)   (decision 3)
        # params: laws over the tree, in the tree's own shape: {"plant": {"m": Uniform(0.8, 1.2)}}
        # disturbances: {"d": source block}, wired onto the plant's port; a bare law refused (R128)
        # sys may be a StepSystem: a natively discrete task (tf in seconds when it has a period, else in ticks)
    is_stochastic: property                     # any law in x_start or params, any random block
    params: property                            # {plant, cost, X, U, X0, Xf, x_start, discount_rate, <source ids>}, live
    def realize(self, key=None, n_trials=1) -> dict        # ONE tree: x_start and every drawn leaf with a trial axis,
                                                           # every random block's seed a child seed by path (decision 4)
    def discretize(self, dt, *, solver="rk4", substeps=1) -> PlanningProblem   # the same class on discretize(sys, dt, …)
    def transition(self, x, u, k=0, params=None) -> (x_next, c_k)   # the Bellman transition, on a step problem only;
                                                                     # a continuous problem refuses it naming discretize
    def compile(self, controller, dt=None, *, solver="rk4", substeps=1, backend="auto", episode_length=None) -> ProblemEvaluator
    def replace(self, **fields); def nominal(self); def cost_bound(...); def discount_factor(self, dt)
```

`transition` on a `Discretization` is the augmented step: `[x; c]` with `ċ = e^{−ρ(τ−t_k)} g(x, u_k, τ)`
stepped by the discretization's scheme and substeps, `u` held; on a native `StepSystem` it is
`(sys.step(x, u, k), g(x, u, t_k)·Δ)` with `t_k = k·Δ` (`Δ = 1` for a tickless system). The sets
are read at `t_k` by the judge, never by `transition`. Then `J = Σ_k γ^k c_k + γ^N h(x_N, t_N)`,
`γ = e^{−ρ dt}`. `StochasticPlanningProblem` goes (decision 3): a problem is stochastic by its
parts.

### 6.4 Compile, the judge, the evaluation

```python
class ProblemEvaluator:                         # planning/evaluator.py:40 (pe in scripts)
    loop: StepDiagramSystem                     # the lowered loop; step_problem: problem.discretize(dt, …)
    params: property                            # the run's tree: every block (+ "x0" per stateful block), the problem's leaves, the free inputs
    def trajectory(self, params=None, key=None) -> Trajectory
    def evaluate(self, params=None, key=0, n_trials=None, *, record=False) -> Evaluation
    def program(self, wrt, bounds=None, key=0, n_trials=None, *, params=None) -> MathematicalProgram
    def pack(self, params, wrt); def unpack(self, z, wrt, params=None)      # core/params.py

def evaluate(problem, controller, *, dt=None, solver="rk4", substeps=1, n_trials=None, key=0, record=False, backend="auto", episode_length=None)
def compare(problem, *, dt=None, n_trials=None, key=0, **named) -> Comparison
def evaluate_trajectory(problem, traj) -> Evaluation   # a logged run: the trapezoid, and says so
```

`compile` dispatches on the controller's kind (R105): a continuous law or a built continuous
loop → `discretize(ctl @ plant, dt, solver, substeps)` whole (today's numbers for the states);
a step controller with a period, or a built `SampledDataSystem` → taken as the step loop at
its base clock (a given `dt` must agree; the stateless MPC refused, naming the block); a
`StepSystem` plant → the controller a step or static block. In every case the run is **one
scan** carrying `(x_k, J_k, m_k)` over `problem.transition` and the margins at the samples;
`score_run` keeps its failure and price rules and reads `J` from the scan; `require_continuous`
goes. `backend="auto"` keeps JAX when the whole loop traces; a host-solver block makes the loop
NumPy, announced once.

### 6.5 Programs, the optimizer, the kernels

```python
@dataclass(frozen=True)
class MathematicalProgram:                      # optimization/mathematical_program.py:26
    n_z; J; h; g; lower; upper; grad_J; hess_J; jac_h; jac_g; backend; metadata
    params: dict = {}                           # the frozen defaults (R130): an exported program's is the problem's tree
    def params_at(self, z) -> dict              # the full tuned tree of an export

class Optimizer:                                # optimization/optimizer.py:75
    def __init__(self, program, z0=None, *, method="auto", backend=None, iterations=None, options=None, **method_kwargs)
        # compile_backend → backend; the adapter attribute is .adapter; iterations=K: a native kernel for exactly K steps,
        # one lax.scan on JAX (traceable, differentiable); None: to the method's tolerance (a host loop)
    def solve(self, *, z0=None, params=None, callback=None, verbose=False) -> OptimizationResult   # params merged once
    def as_step_system(self) -> StepSystem      # z_{k+1} = step(z_k; p): the solve as a rollout, its poles (R132)

class GradientDescent:  params {"eta"};        init(z0, params) -> state; step(program, state, params) -> state   # projected on the box
class Adam:             params {"eta", "b1", "b2", "eps"}; the shape of today's first_order.py:135
```

The program evaluator jits `J(z, p)` and its derivatives in `z` with `p` a pytree argument:
rebinding is a call, not a trace (D and E: one trace for six rebinds). The wrapped methods
(SciPy, Ipopt) close over `p` per `solve`. The stochastic program is R131's convention: `seed`
an integer leaf, never in `wrt`; `pe.program(wrt, key, n_trials)` the sample average realized
once; a Monte Carlo over solves is `vmap` of `solve` over the seed leaf.

### 6.6 The policies, the planners, the MPC

```python
class NeuralPolicy(System)                      # control/neural.py:15, renamed now (decision 13)
class GaussianPolicy(System):                   # control/stochastic.py (new)
    def __init__(self, mean: System, log_std, *, sample_period, seed=None)
    params = {"mean": mean.params, "log_std": (m,), "seed": int | None}; is_random = True
    def h(self, x, u, t=0.0, params=None)       # μ(x) + exp(log_std)·ε(seed, ⌊t/Δ⌋); seed None: μ
    def log_prob(self, x, u, t=0.0, params=None); def entropy(self, x, t=0.0, params=None); def realize(self, key)
class SquashedGaussianPolicy(GaussianPolicy)    # the SAC head

class Planner:                                  # planning/planner.py:21
    def solve(self, *, params=None, z0=None, **options) -> PlanningSolution   # the one verb; the record carries z
    def shift(self, z) -> z0                    # the warm start one period later (planners with a decision)
    program: MathematicalProgram | None         # built once, where the planner has one

class TrajectoryOptimizationPlanner(Planner):
    def __init__(self, problem, *, transcription="direct_collocation", n_steps, backend="auto", method="auto",
                 iterations=None, options=None, ...)   # iterations=K with a native method: solve is a fixed-K scan

@dataclass(frozen=True)
class PlanningSolution:  problem, policy, trajectory, evaluation, cost_to_go: Field | None, solver, success; the generic plots (R138)

class ModelPredictiveController(StepSystem):    # mpc/controller.py (rebuilt; re-exported by control)
    def __init__(self, planner, *, sample_period, parameter_ports=(), name=None)
    # state: the plan z (x0 the default guess); ports: y and one per parameter path in, u out
    params = {**planner.program.params without x_start, "eta": …}      # the planner's problem tree under the block's own keys
    def step(self, x, u, k=0, params=None)      # z⁺ = planner.solve(params | {"x_start": y_k}, z0=shift(z)).z
    def h(self, x, u, k=0, params=None)         # u_k = the first input of z_k (one period late, R123)
    def horizons(self, traj)                    # the plans read back from the recorded z, for animate
```

---

## 7. The architecture under it

### 7.1 One kernel

```text
Φ(x_k, u_k, k; p) = (x_{k+1}, c_k)       problem.transition: the augmented step on a Discretization,
                                          the block's own step and c_k = g·Δ on a native StepSystem
carry_k = (X_k, J_k, m_k)                 the loop's state, the running score, the least margin
u_k     = π(X_k, k; p)                    the law: a step diagram's wiring, propagated within the tick
m_{k+1} = min(m_k, X.margin(x_k, t_k), U.margin(u_k, x_k, t_k))
J_{k+1} = J_k + γ^k c_k
(X_N, J_N, m_N) = scan(body, carry_0, k < N);   J = J_N + γ^N h(x_N, t_N)
```

Who runs it: the step evaluator's `rollout` (`c_k` dropped), the judge's `run` (the full
carry), the RL environment (one body call, reward `−c_k`), the grid's successors (one body call
per node and action), shooting (the scan as the objective, `x_start` from `params`), the MPC's
inner loop (the kernel's scan over `K`, nested in the tick's scan), the Monte Carlo (`vmap` of
the scan over the trial axis of the tree).

### 7.2 Layers

```text
 ┌──────────────────────────────────────────────────────────────────────────────────────────┐
 │ 5  TOOLS        analysis · simulation (Simulator; the clock runner, realtime) · planning   │
 │                 (planners, MPC, compare) · learning (RL) · optimization (adapters, kernels) │
 ├──────────────────────────────────────────────────────────────────────────────────────────┤
 │ 4  THE PATH     PlanningProblem ─discretize→ the step problem ─compile(ctl)→ the judge      │
 │                 ─run (one scan)→ Evaluation ─program(wrt)→ MathematicalProgram ─→ Optimizer │
 ├──────────────────────────────────────────────────────────────────────────────────────────┤
 │ 3  LIBRARIES    blocks/ (sources, routing, UnitDelay) · dynamics/ · control/ (laws,         │
 │                 NeuralPolicy, GaussianPolicy) · estimation/                                 │
 ├──────────────────────────────────────────────────────────────────────────────────────────┤
 │ 2  COMPILE      compile(sys) → evaluator: f · step · rollout · rollout_batch · signals ·    │
 │                 jacobian; stepping.py: step_body, the augmented transition, iterate (scan)  │
 ├──────────────────────────────────────────────────────────────────────────────────────────┤
 │ 1  ALGEBRAS     continuous: System, DynamicSystem, DiagramSystem (@ >> + connect)           │
 │                 step: StepSystem, StepDiagramSystem, SampledDataSystem (the same grammar)   │
 │                 the one adapter: discretize / %                                             │
 ├──────────────────────────────────────────────────────────────────────────────────────────┤
 │ 0  NOUNS AND THE CONTRACT   Trajectory · Set · Distribution · CostFunction · Field ·        │
 │                 the cipher · core/params.py (the tree) · array_module                       │
 └──────────────────────────────────────────────────────────────────────────────────────────┘
```

Layers import downward only. The dependency law of DESIGN §3 holds with three edits:
`core/discrete/` no longer imports `simulation` (R100's exception retired); `planning` imports
`blocks.sources` for the `disturbances=` shortcut and `control.stochastic` for the RL heads
(the existing `learning → control.neural` exception widens by one module); `optimization`
imports `core/discrete` for `as_step_system`.

### 7.3 Module moves

| Where | What |
| --- | --- |
| `core/params.py` (new) | the tree verbs, from the four copies (`planning/evaluator.py:991`, `planning/problems.py:896`, `core/compile/evaluators/jacobian.py`, `core/assembly.py`) |
| `core/fields.py` (new) | `Field`, `QuadraticField`, `GridField`, `EllipsoidSet`, the shaping (from `planning/spatial/`) |
| `core/discrete/` | `discretization.py` (from `analysis/`, every port kept), `sampled.py` (the `%` body, the hold slots), `sampled_data.py` (`SampledDataSystem`, replacing `hybrid_diagram.py` and `hybrid_composition.py`); `step_rollout.py` goes |
| `core/compile/` | `stepping.py` gains the augmented transition; `plan.py` gains the divisors and hold slots; `evaluators/discrete.py`'s `rollout` native, plus `rollout_batch` |
| `simulation/discrete/` | `runner.py`: the clock runner over the compiled blocks (from `computer.py` and `hybrid_simulator.py`, both gone) |
| `planning/` | `problems.py` (one noun, `discretize`, `transition`, `realize` one tree); `evaluator.py` on the scan; `evaluation.py`'s `score_run` reads the scan; `environment.py` from `learning/` (R108) |
| `planning/trajectory_optimization/` | the transcriptions on `params`; `parametric_program.py` and its evaluator deleted; `mpc/controller.py` the one pure block |
| `optimization/` | `params` on the noun; `kernels/` (`gd.py`, `adam.py` from `first_order.py`); `adapters/` |
| `control/stochastic.py` (new) | `GaussianPolicy`, `SquashedGaussianPolicy`; the RL heads deleted from `learning/.../policy.py` |

### 7.4 The runtime model

| Compiled once | Varies per call, no retrace | Host code |
| --- | --- | --- |
| a system's evaluator: the plan, `f`, `step`, `rollout`, jitted on first use | `x`, `u`, `t` or `k`, the complete tree | the NumPy bodies |
| a step diagram's plan: port order, slices, divisors, hold slots, substeps | the same, the free inputs | — |
| the judge: `jit(vmap(scan))` per batch layout (which leaves carry a trial axis) | the tree, the key, the starts, `n_trials` as an axis | `solver="scipy"` (one run per trial) |
| a program: `J(z, p)` and its derivatives in `z`, `p` a pytree argument | `z`, `p` (the MPC's `x_start`, a mass, a weight, the `seed`) | SciPy, Ipopt |
| a native solve: `scan(step, length=K)` | `z0`, `p`, `eta` | `iterations=None`: the loop to a tolerance |
| the MPC tick inside the loop's plan | `y_k`, `z_k`, the ports' values | a host planner (the NumPy face, the runner) |

What retraces: a new batch layout, a new size (`N`, `K`, `n_trials`), a replaced problem part.
What never retraces: a new value of any leaf, a new key, a new start.

### 7.5 Traceability and cost, with the numbers measured (CPU, float64, this container)

| Path | Why it traces | Measured |
| --- | --- | --- |
| the one score on a continuous loop | one `lax.scan`; the augmented step is `substeps` static RK4 stages; margins are `minimum`s in the carry; the price a `where` | 1 000 starts × 200 RK4 steps in 44 ms (D); `grad` w.r.t. a gain and a weight = central differences to 10 digits (D), 6 digits (E) |
| the sampled loop | the firing a `where` (or a `cond`, a `select` under `vmap`) on `k mod d`; the hold a carry entry; the plant's substeps static | 100 starts × 3 000 base ticks in 93 ms (D); 300 ticks in 3.2 ms (F); NumPy scan = clock runner byte for byte (C, D, E, F); the real `compile_step_diagram` ran `Sampled`, `Held`, a native MPC and a seeded policy as ordinary blocks (E) |
| the native MPC's closed loop | the tick a `scan` over `K` with `jax.grad` of the shooting program inside, nested in the loop's scan | 60 ticks × 15 iterations × a 20-step horizon: 134 ms a call, 32 starts in 477 ms, `grad` w.r.t. the terminal weight and the step size to 8 digits (D); 80 ticks × 15 Adam in 112 ms (E); 105 ms (F) |
| the draws and the seeded policy | the cipher is `uint32` arithmetic, `vmap`pable over seeds; the gradient pathwise through the reparameterized noise | 256 seeded trials in 14 ms, the mean score from 13.4 to 0.22 in 30 gradient steps (D); the same bits on both backends (D, E, F) |
| the parametric rebind | `p` a pytree argument of the jitted `J` | one trace for six rebinds (E), three starts (D) |
| an integer seed leaf under `jax.grad` | `allow_int=True`: `float0` for the seed, no cost | F |
| memory under `grad` | one carry per tick; `jax.checkpoint` per tick (or per controller period) a knob on `compile` | not needed at these sizes (D, F) |

By design, not traced: a host-solver MPC (`iterations=None`), an adaptive plant solve
(`solver="scipy"`), the realtime runner, a `while` to tolerance under reverse mode; each behind
one keyword, never inside a traced body.

---

## 8. The decisions for the one-by-one review

Each decision: the concept in plain words, the code today, the options with their
consequences, where the six proposals stood, and a recommendation. They are ordered by the
code, the problem first, and numbered for the review.

### Decision 1. The discrete face of a problem: no new noun

**Concept.** Every stepping tool (the judge, value iteration, the RL environment, shooting, the
native MPC, MPPI later) needs the same five things from a problem at a given `dt`: the Bellman
transition `(x_{k+1}, c_k)`, the sets at `t_k`, `N`, `γ`, the price. Today each writes its own
(`dp.py:308`, `environment.py:130`, `evaluation.py:125`, `shooting.py:293`), and they disagree.
The question is what object holds the lowering.

**Options.**
- (a) **The same `PlanningProblem` on a `StepSystem`** (B, F): `problem.discretize(dt, solver=,
  substeps=)` returns a `PlanningProblem` whose `sys` is `discretize(plant, dt, …)`; it gains
  one verb, `transition(x, u, k, params) → (x_next, c_k)`, refused on a continuous problem. A
  natively discrete task (a grid world, inventory control) is the same class on a `StepSystem`
  plant, its cost reading `t` as the tick. One noun, one precedent (`discretize(plant, dt)`
  returns a `StepSystem`), and `print(problem.discretize(dt))` shows `dt`, `N`, `γ`.
- (b) **A small public noun** (`StepProblem`, C; `StepPlanningProblem` + `StepCost`, E): a
  frozen record with `transition`, `margin`, `terminal`, `N`, `gamma`, `price`. One more class
  and one more cost type on the research lane; a researcher can print it; a natively discrete
  task writes it directly.
- (c) **Internal only** (A, D): the lowering is a private object of the tools; `transition`
  stays in `stepping.py`. Nothing public; a natively discrete task has no home.

**Recommendation.** (a). It is the "simplicity over machinery" law: the discrete problem is
the continuous one on a discrete system, exactly as a `StepSystem` is to a `DynamicSystem`,
and E's `StepCost` is not needed once the cost binds `t = k`. Cost: `PlanningProblem` has two
faces by its system's kind, which `System` already has.

### Decision 2. The stage cost: computed in the transition, in the discount's stationary form

**Concept.** All six integrate the running cost with the state. Two details remain: where
`c_k` is computed, and which discount form.

**Options.**
- Where: (a) **inside the transition** (A, C, D, F): the augmented step returns `(x_{k+1}, c_k)`
  in the scan, one plant evaluation; (b) **a post-pass** (B, E): the rollout records the
  samples, then one `vmap` over `(x_k, u_k, k)` computes `c_k`, which must re-run the plant over
  each period to integrate the cost (a second plant evaluation), but can score any held
  trajectory.
- The discount: (a) **stationary** (D): `c_k = ∫₀^{dt} e^{−ρτ} g dτ`, `J = Σ γ^k c_k + γ^N h`,
  so `c_k` does not depend on `k` and value iteration's backup `c + γ J'` reads it as is;
  (b) **absolute** (F): `ċ = e^{−ρt} g`, `J = Σ c_k + e^{−ρ tf} h`; equivalent numbers, but a
  stage cost that depends on `k`.

**Recommendation.** Inside the transition, stationary form. A logged run (`evaluate_trajectory`)
keeps the trapezoid and says so (F).

### Decision 3. One problem noun; the start and the laws under the tree's own keywords

**Concept.** Today `StochasticPlanningProblem(sys, x0_distribution=, params_distribution=
{"m": law}, disturbances={port: law})` is a second class with its own keywords. R124 says the
problem's start is "a point, a `SingletonSet` or a `Distribution`" and R128 makes disturbances
blocks on both classes. What remains is the keywords, and whether a second class is needed at
all.

**Options.**
- Keywords: (a) **`x_start=` takes a point, a `Set` or a law; `params=` the laws in the tree's
  own shape** (`{"plant": {"m": Uniform(0.8, 1.2)}}`) (A, C; D for `x_start`; E, F for
  `params`); "where a number goes, a law may go", one keyword per concept (R134);
  (b) keep `x0_distribution=` and `params_distribution=` with dotted paths (E, D).
- The class: (a) **one noun** (A, and this document): `PlanningProblem` with `criterion=`;
  `problem.is_stochastic` is read off its parts (a law in `x_start` or `params`, a random
  block); `StochasticPlanningProblem` goes; (b) **two nouns** with aligned keywords (C, D, E,
  F): the subclass only adds `criterion` and `nominal`.

**Where it touches.** Student-facing: two GRO860 notebooks and the RL demos write
`StochasticPlanningProblem(..., x0_distribution=Uniform(...))`; both options sweep them once
(the keyword changes anyway). **[ask — student-facing]**

**Recommendation.** (a) and (a): one noun, `x_start=` and `params=`. A problem is stochastic
because its parts are, not because of its class name; `nominal()` and `criterion` sit on the
one class; the notebook sweep is done once with the keyword.

### Decision 4. `realize` returns one tree

**Concept.** R128 chose one shape, `(x0s, params)`. B observed that `x_start` is already a leaf
of the problem's tree, so the pair repeats a leaf the tree holds.

**Options.** (a) **one tree** (B): `realize(key, n_trials) → params` with `params["x_start"]`
of shape `(n, nx)`, every drawn leaf and every seed with a trial axis, shared leaves unchanged;
the judge reads the start from the tree like any leaf; (b) the pair (A, C, D, E, F; R128).

**Recommendation.** (a). It is "every number a parameter" applied once more, and
`pe.evaluate(params={"x_start": starts})` (a batched leaf) already means the same thing.
Revises R128's shape wording only.

### Decision 5. `%` on a stateful law: discretize it, with Euler; a plant: RK4

**Concept.** `pid % 0.01` today refuses a law with continuous states. Five proposals discretize
it (the textbook digital implementation). The remaining choice is the default scheme.

**Options.** (a) **`%` discretizes a law with `solver="euler"`; `discretize(plant, dt)` keeps
`rk4`** (D, E, F; the rule **(own)**: a law is *implemented* on a computer, the difference
equation GRO501 writes; a plant is *simulated*, so the scheme that converges is its default);
both take `solver=` explicitly; (b) one default for both, `rk4`, through `discretize` (B, C):
one owner of the default, but `pid % 0.01` is then not the difference equation a student
derives by hand.

**Recommendation.** (a), with the rule written in DESIGN: `%` is implementation, `discretize`
is simulation.

### Decision 6. Periods live on blocks; no `Computer` or `StepSchedule` class

**Concept.** Today a multi-rate computer is `Computer(diagram, StepSchedule(dt_base, {id:
divisor}))`, a runtime object with buffers. R127 makes the loop a step diagram; what carries
the rates is the question.

**Options.**
- (a) **Periods on blocks** (D, E): `%` sets a block's `sample_period`; a step diagram's base
  period is the greatest common divisor of its blocks'; a block with divisor `d > 1` fires on
  `k mod d = 0` and its outputs are held as state slots, wrapped at compile; a multi-rate
  computer is ordinary composition, `(filter % 0.01) >> (pid % 0.05)`; no `Computer` class, no
  `StepSchedule`; "computer" stays the word of the runner (`Computer(loop)` for realtime, E)
  and of the drawing's box.
- (b) **`Computer(StepDiagramSystem)` with a `StepSchedule` in core** (A, C, F): one noun for
  the lesson ("the controller as it runs on a computer"), the schedule a data object, `%` a
  one-block computer. One more class and one more data object; two ways to say a rate (a
  block's period, a schedule's divisor).

**Recommendation.** (a). The step algebra already composes rates when each block knows its
own; a schedule object would be a second place for the same fact. The MPC's dual rate is then
`mpc % 0.05` beside a `% 0.01` block, so `replan_every` is not needed either.

### Decision 7. The name of `computer @ plant`: `SampledDataSystem` now

**Concept.** R127 keeps `HybridDiagram` "for now". The class becomes a `StepDiagramSystem`
subclass that knows its base clock and its continuous plant: the sampled-data system of
Åström and Wittenmark.

**Options.** (a) **`SampledDataSystem` now** (A, D, F): the hybrid demos and tutorial 06 are
rewritten by the loop lane anyway (R125's semantic, R123's delay), so the name changes once; no
alias (R82); (b) **`HybridDiagram` until the v0.9 name pass** (B, C, E; R127): no rename now,
a second sweep later. `SampledLoop` is the shorter variant (F).

**Recommendation.** (a). The code moves once; the name should move with it.

### Decision 8. The verbs on the sampled loop: two verbs that agree

**Concept.** R127 chose `compute_rollout()` (the ticks, pure) and `compute_trajectory()` (the
orchestrator with an adaptive plant, dense). D, E, F showed that the pure scan can record the
plant's substeps as its per-tick output (verified cheap), so a dense run needs no second
model; three of them then want one verb.

**Options.**
- (a) **Two verbs, one default runner**: `compute_rollout()` is the step algebra's verb on every
  `StepSystem` (the samples; `evaluate` scores exactly these); `compute_trajectory(solver=None)`
  on the sampled loop is the **same pure scan with the substeps recorded** (dense, in seconds),
  and `solver="scipy"` is the clock runner with the adaptive plant between samples, the
  cross-check (R15's split applied literally). The plot, the score and the gradient agree by
  default.
- (b) **Two verbs, the orchestrator by default** (R127 as written; A, C): `compute_trajectory`
  is the adaptive runner; its samples differ from `evaluate`'s by the plant's integration error.
- (c) **One verb**, `compute_trajectory(solver=)`, `compute_rollout` dropped from the loop (D,
  F): one number per object; the step verb then means two things on two kinds.

**Recommendation.** (a). It keeps the maintainer's two verbs and his point that the step
rollout is the standard one, and answers his technical doubt ("extracting substeps with the
standard rollout would not work?"): the substeps are the scan's output, not a change to the
rollout. **[ask — the hybrid demos re-pin on it]**

### Decision 9. The planner interface and the MPC block

**Concept.** R135 asked for a generic planner interface that an MPC can drive without the
planner knowing it is in a loop. Six proposals offered three to four verbs each.

**Options.**
- The verbs: (a) **`solve(params=None, z0=None) → PlanningSolution`** (the one verb every
  planner has; the record carries the decision `z`), **`shift(z) → z0`** (what one period later
  means for the decision), **`program`** (the parametric program, built once, where the planner
  has one) (C, F; D with `plan`); (b) `prepare()` / `plan(x0, params, z0)` / `shift` (E):
  `x0` a positional argument, a verb the student does not know; (c) B's four
  (`decision_dimension`, `solve_from`, `warm_start`, `plan_of`).
- Where `iterations=K` lives: (a) **on the planner** (A, C, F): the planner owns its solver
  (`method=`, `options=`), and a planner with a native method and `iterations=K` has a pure
  `solve`; the MPC inherits traceability; (b) on the MPC block (D, E): the block decides how
  its planner solves.
- The MPC's constructor: `ModelPredictiveController(planner, *, sample_period,
  parameter_ports=())`; `warm_start` goes (the state is always the plan; D).
- The inner tree's key for the model: (a) **`plant`** (A, C, D, E): `program.params ==
  problem.params` word for word, so `pe.params["controller"]["plant"]["m"]` beside
  `pe.params["plant"]["m"]` is the model-mismatch study as it reads; (b) `model` (F): a loop's
  tree never holds two `plant` keys, the textbook word for the MPC's internal model, at the
  cost of a rename inside the block.

**Recommendation.** (a) throughout: `solve`, `shift`, `program`; `iterations` on the planner;
the key `plant`.

### Decision 10. The fixed-count solve on `Optimizer`

**Concept.** R132 makes the native solver a kernel; the fixed-`K` traced form needs a surface.

**Options.** (a) **`Optimizer(program, method="adam", iterations=K)`** with the one `solve`
(A, C, F; `n_iter` in F): a structural keyword where `method` is; (b) a second verb
`iterate(z0, params, n_steps=K) → (z_K, Trajectory)` (D); (c) `solve(iterations=K)` (E).

**Recommendation.** (a); the iterates as a `Trajectory` come from
`optimizer.as_step_system().compute_rollout(K)`, the lesson's view, not a second solve verb.

### Decision 11. The RL planner's `gamma` derived from the problem's `discount_rate`

**Concept.** Value iteration and the judge read `γ = e^{−ρ dt}` from the problem; the RL
planner takes `gamma=0.97` as its own argument (`pendulum_value_iteration_vs_lqr_vs_rl.ipynb`,
`drone_ppo.ipynb`). Only C raised it; it is "one owner per quantity" (RULES 5.6) and the one
score needs the training return and the judge to agree.

**Options.** (a) **derive it**: the notebooks set `discount_rate=-np.log(0.97)/DT` on the
problem, `gamma=` leaves the planner; (b) keep `gamma=` as the RL planner's own; the training
return and the judge's `J` may then disagree on the discount; (c) derive it, and keep `gamma=`
one release as an override that warns and sets the problem's rate (C's recommendation, a
stated exception to R82).

**Recommendation.** (a); the notebook cell changes once, with the RL re-pin that v0.3 does
anyway. **[ask — student-facing, the lesson's vocabulary "γ = 0.97"]**

### Decision 12. Fields: `terminal=` and `GridField` in v0.3

**Concept.** R133 keeps the `Field` minimal (`Field`, `QuadraticField`, three conversions).
Five proposals add `as_cost(terminal=True)`; D adds `GridField`.

**Options.** (a) **`as_cost(weight, shaping, *, terminal=False)`** (A, C, D, E, F): one
conversion, one keyword, the LQR value as a terminal cost in one call; (b) a fourth verb
`as_terminal_cost`. And: (a) **`GridField` in v0.3** (D), about sixty lines, so value
iteration's `cost_to_go` is a `Field` and the DP table ends an RL horizon (the one-score
pillar's own demonstration, §5.4); (b) `GridField` in v0.4 (R133).

**Recommendation.** `terminal=` and `GridField` now, with its consumer.

### Decision 13. `NeuralPolicy` renamed in the v0.3 sweep

**Concept.** R134 places `NeuralPolicyController → NeuralPolicy` at the v1.0 window. F notes
v0.3 already moves every RL notebook (the seeded policy re-pins their numbers).

**Options.** (a) **now**, in the v0.3 names sweep, between terms (the freeze is paused, ROADMAP
§4.1 gate 7); (b) at v1.0 (R134), a second migration of the same notebooks.

**Recommendation.** (a). **[ask — student-facing]**

### Decision 14. The gradient one-liner over a tree with an integer seed

**Concept.** F found that `jax.grad(lambda p: pe.evaluate(p).objective)(pe.params)` as DESIGN
§6 writes it refuses an integer leaf. `pe.program(wrt)` never sees it (R131).

**Options.** (a) **document `jax.grad(f, allow_int=True)`** (the seed's gradient a `float0`, no
cost) (F); (b) a helper `pe.float_params` (E); (c) a float seed (bits reinterpreted): not
recommended, an integer leaf says what it is.

**Recommendation.** (a); one line in DESIGN.

---

## 9. What this vision revises among the preliminary choices

| Choice | Kept, refined or revised |
| --- | --- |
| R124 three owners of a start | kept; `x_start` the one keyword for the problem's start, three kinds (decision 3) |
| R125 one same-tick semantic | kept |
| R126 operators on step systems | kept; the mixed pair allowed exactly for a step left side with a period (§2) |
| R127 `computer @ plant` a step diagram, two verbs, `HybridDiagram` for now | kept in substance; **revised** on the name (decision 7), the default runner of `compute_trajectory` (decision 8), and the computer object (decision 6) |
| R128 disturbances are blocks, one `realize` shape | kept; **revised** on the shape, one tree (decision 4) |
| R129 the seeded policy | kept; refined: `mean` is any `System` (§2) |
| R130, R131 the program and its convention | kept; the key `x_start` |
| R132 the kernel with a step-system view | kept; `iterations=` the fixed-`K` form (decision 10) |
| R133 a minimal `Field` | kept; refined with `terminal=` and `GridField` (decision 12) |
| R134 names | kept; line 4 moved to v0.3 (decision 13) |
| R135 the planner interface | filled: `solve`, `shift`, `program` (decision 9) |
| R136–R139 | untouched |
| R140 the one contract | kept, the first row of the core |
| The one-score investigation (QUESTIONS) | answered: §2's first four rows, decisions 1 and 2 |
| CONSTITUTION | to amend at landing: invariant 1 (the sampled loop a step diagram of the sibling algebra), §6 (the operators on step systems), the disturbance sentence (sources, not laws), as R126–R128 already name |

---

## 10. The order of work, and what is verified

**The order** (the three lanes and two joins of core-architecture §10, with the six proposals'
agreement on where each step sits):

1. **The contract and the tree** (byte-identical): `core/params.py`; `params = {}`; the laws'
   leaves; the cipher-only draws (one intended change: the seeded draws); the diagram views
   of `x0`; `TrajectorySource.values`; the names sweep (R134, decision 13) before any new
   signature.
2. **The loop lane** (the critical path): `Discretization` with every port; the periods on
   blocks, the hold slots and the divisors in the plan; the operators on step systems; native
   step rollouts and `rollout_batch`; `SampledDataSystem` with its two verbs; the runner; the
   hybrid demos re-pinned once (R125, R123 and decision 8 together).
3. **Join 1, the one score**: `problem.discretize`, `transition`; the judge on the scan;
   value iteration and the RL environment reading `transition`; the judge's `J` re-pinned once.
4. **The draws and the policy**: `realize` one tree by path; `disturbances=` as blocks on the
   one noun; `GaussianPolicy`; the RL heads on it (the RL re-pin once, with the seeds).
5. **The program lane**: `MathematicalProgram(params=)` with one evaluator body; the
   transcriptions on the tree; `kernels/`, `iterations=`, `as_step_system`; the planner verbs;
   the MPC block pure; the MPC demos re-pinned once (R123).
6. **Join 2**: the native MPC's closed loop under `problem.compile`, tuned by gradient (§5.3's
   second half as the acceptance test).
7. **Fields**, beside the others from step 1 on. **Hardening and the merge** last.

Four intended number changes, each its own commit with a closeness report (R80): the sampled
loop (the same-tick semantic, the MPC delay, the default runner), the one-score `J`, the
cross-backend draws with live disturbances, the RL re-pin with the seeded policy.

**Verified by the proposals, on NumPy and JAX in this container** (the scripts are under the
scratchpad folders named in each proposal's header): the augmented step's `x` rows
byte-identical to `step_body` and its `c_k` equal to `g·dt` under Euler to the last bit (C, D,
E, F); the pure base tick byte-identical to a clock orchestrator at the samples (C, D, E, F);
`jit`, `grad`, `vmap` and `lax.scan` on every main path, every gradient against a central
difference (D, E, F); the real `compile_step_diagram` running the sampled block, the hold, a
native MPC block and a seeded policy unchanged (E); the native MPC's closed loop in one scan,
differentiated and tuned (D, E, F); one trace for several rebinds (D, E); the same bits on both
backends from one key (D, E, F); the `allow_int` finding (F).

**Not verified** (the spikes before their lane starts): the hold slots and divisors inside the
compiled step plan (`plan.py`; the prototypes carry the hold by hand); the clock runner with an
adaptive plant between samples (`solver="scipy"`); the program's constraint rows on the
compiled path with Ipopt inside a tick; `jax.checkpoint` on long loops under `grad`; the DP
sweep's wiring on `transition` under JAX.

---

## 11. Ideas beyond the six proposals **(own)**

- **One problem noun** (decision 3, with A): a problem is stochastic by its parts;
  `is_stochastic` is a property, not a class.
- **"A law is implemented, a plant is simulated"** (decision 5): the rule that gives `%` Euler
  and `discretize` RK4 without two owners of one default.
- **Two verbs that agree** (decision 8): the maintainer's `compute_rollout` / `compute_trajectory`
  split kept, with the dense run read off the same scan, so the two numbers are one.
- **`iterations` on the planner, not the block** (decision 9): the block inherits traceability
  from the planner it drives, and the MPC's constructor stays three keywords.
