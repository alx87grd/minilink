# Final architecture proposal F: the complete core of minilink (2026-10-27)

**Audience:** Prof. Alexandre Girard (Maintainer)
**Status:** Proposal F, an independent proposal, not a ruling. Written without reading the
other proposals of the same night.
**Commit reviewed:** `f634e47eb04db70d0089b46c4e043f334eb8bdc7` (`dev-v0.3`, level with `origin`).
**Read:** AGENTS, CONSTITUTION, RULES, ROADMAP, core-architecture.md, TODO §1, QUESTIONS, the
2026-10-27 record (R124–R140), the 2026-10-26 direction and the three independent reviews,
DESIGN §1–§7, the plan docs sampled-path §8, differentiable-problem §9, native-optimizers
§3–§7, parametric-program, randomness, fields; the code under `core/`, `planning/`,
`optimization/`, `simulation/discrete/`, `learning/` and the compile demos.
**Prototypes:** `/tmp/claude-0/-home-user-minilink/36350854-ab7e-51d8-8262-d68717b27e02/scratchpad/arch-F/`
(NumPy and JAX; JAX 0.12 was installed mid-session, so the JAX paths were run, not argued; §10).

Plain words first, then the workboard id in parentheses, as AGENTS asks.

## 2. The aim in one screen

Minilink is one toolbox around one model. A student writes `dx = f(x, u, t; p)` once and never
changes framework to simulate it, draw its Bode plot, close a PID or an LQR on it, plan a
trajectory, run an MPC, or train a neural policy on it. The same object serves the first lesson
on a laptop and the research run on a GPU; the student syntax never breaks on the way up
(CONSTITUTION §1.1, §3).

What holds the toolbox together is **one differentiable path**: a `System` with its parameter
tree, compiled once; rolled out by one step model to a `Trajectory`; scored by a
`PlanningProblem`'s cost and sets into an `Evaluation`; lowered, when the user asks, to a
`MathematicalProgram` an `Optimizer` solves. Every approach to controlling a vehicle or a
robot is a tool on that path, and v0.3 is the release where the path is **complete**: it
takes a stochastic problem (draws over starts, parameters and noise, each a leaf of the
tree), it takes the sampled loop a real computer runs (a digital law at its period on a
continuous plant), it gives a policy one score whatever tool produced it, and it lets a
program be compiled once and solved many times, by a host solver or by a native one that
traces end to end, so an MPC's own closed loop can be differentiated and tuned.

Three design laws decide every question below:

1. **One signature, every number a parameter** (R140). A core object's mathematics is a
   function of its arguments and `params`, deterministic given them, the seed an integer
   leaf; `params=None` reads its defaults; composites nest by name; `jit`, `grad` and `vmap`
   reach every leaf.
2. **The continuous contract is primary; sampled and discrete loops are the sibling algebra**
   that sits beside it and never complicates `f`, `DiagramSystem` or `Simulator`.
3. **Simplicity over machinery.** One kernel, one score, one program noun, one word per
   concept; a new noun only when it is a textbook noun.

The user I design for is the maintainer's student who, in one semester, goes from
`Pendulum().compute_trajectory()` to
`Optimizer(problem.compile(mpc % 0.05, dt=0.01).program(wrt=["controller.cost.Q"])).solve()`
without learning a second vocabulary.

## 3. Adjustments to the vision

Each adjustment names what it revises, why, and what it costs. Everything not listed here is
taken as ruled or as the preliminary choice of 2026-10-27.

### 3.1 One score: the running cost is one more state (revises nothing; answers the open investigation)

**The choice.** Once `dt`, the scheme and the substeps are set, the running cost is appended
to the state and integrated by the plant's own step, `ċ = e^{−ρt} g(x, u, t)`. The step then
returns the Bellman transition `(x_{k+1}, c_k) = T(x_k, u_k, k; p)`, exact for the scheme,
and one scan over `k` scores every loop that steps: `J = Σ_k c_k + e^{−ρ tf} h(x_N)`.
- Under forward Euler, `c_k = e^{−ρ t_k} g(x_k, u_k, t_k) dt` exactly: value iteration's and
  RL's numbers do not move (verified, §10.1).
- Under RK4 the cost converges with the state and the substeps count for it; the trapezoid
  on the samples (today's `score_run`) lags behind by 1e-2 at `dt = 0.1` (verified, §10.1).
- The `x` rows of the augmented step are byte-identical to the plant's own step (the cost
  row does not feed back into `f`), so the run's states do not move either.

**Sets and costs keep time; the step binds it.** A cost `g(x, u, t)` and a set `X.margin(x,
t)` keep their continuous signature. The discretized problem evaluates them at `t_k = k·dt`
(a `Discretization` knows its `dt`); a natively discrete system has `dt = 1`, so `t_k = k`
and `c_k = γ^k g(x_k, u_k, k)` is the textbook discrete cost with `γ = e^{−ρ}`. No discrete
twin of `Set` or `CostFunction` is needed, and a natively discrete cost `c(x_k, u_k, k)` is
a `CostFunction` whose `g` reads `t` as the index.

**No new noun.** `problem.discretize(dt, solver=, substeps=)` returns the same
`PlanningProblem` class on `discretize(plant, dt, ...)`; `problem.compile(controller, dt)`
does it internally. The one new verb is `problem.transition(x, u, k, params=None)`, defined
on a problem whose system is a `StepSystem`: it is the Bellman transition that value
iteration, the RL environment, the evaluator and the sampled loop all read. A continuous
problem refuses it, naming `discretize`.

**One failure rule**, today's `score_run`: the run goes to `N`, the least margin over the
samples decides `failed`, `Xf` at `x_N` decides `reached`, the price switch prices them. The
RL environment's early stop is a training choice that reports the judge's `J`. Zero-order
hold everywhere a loop steps. Shooting's own trapezoid stays its NLP objective; its
reported `evaluation` goes through the judge.

**Cost.** The evaluator's `J` moves once (trapezoid → the integrated cost), with a closeness
report; `score_run` on a held `Trajectory` (`evaluate_trajectory`) keeps the trapezoid,
since a logged run has no step to integrate with, and says so.

### 3.2 One verb on the sampled loop, the runner an argument (revises R127's two verbs)

R127 keeps `compute_rollout()` (ticks, traced) beside `compute_trajectory()` (the
orchestrator, dense) on `computer @ plant`. I propose **one verb**,
`loop.compute_trajectory(solver=...)`, on every system kind once the step rollout is a
`Trajectory` with `t = k·dt`: the default runs the pure step diagram and records the plant's
substeps (option B of sampled-path §8.4), so the plot is as dense as today and the numbers
are `evaluate`'s; `solver="scipy"` runs the clock orchestrator with an adaptive plant between
samples, the cross-check, as `evaluate(solver="scipy")` is for a continuous loop (R15).
- Reason: two verbs with two numbers on one object is the trap reviewer B named; the step
  rollout and the hybrid run differ only in what is recorded, which is an argument.
- Cost: `compute_rollout` leaves the step facade (six hybrid demos and tutorial 06 change
  one verb); `StepRollout` folds into `Trajectory` first (planned).
- Fallback: if the maintainer prefers R127 as written, everything else here stands.

### 3.3 `%` discretizes a continuous law (extends R127; no ruling revised)

`block % dt` means: a static block or a `StepSystem` scheduled at `dt`; a continuous law
with states (a `PID`, an observer) **discretized at `dt`** by `discretize(block, dt,
solver="euler")`, the textbook digital implementation of GRO501, then scheduled. Today `%`
refuses a controller with continuous states (automation-by-convention §4.5). The user writes
`pid % 0.01 @ plant` for the digital PID and gets the sampled-data loop; the discretization
scheme is a keyword (`pid % StepSchedule(0.01, solver="rk4")`). Cost: one line in `%`.

### 3.4 The sampled loop is one pure base tick with selects (takes reviewer A's cut of the draft)

One kernel in v0.3: the base tick, every block's firing a `where` on `k mod d`, the held
outputs of slow blocks in the carry (a hold is a memory), the plant `discretize(plant,
dt_base, substeps)`. At a single rate it is the textbook sampled-data loop with no hold
state at all. The nested scan and the hyperperiod layouts are later optimizations; the
deployment generator leaves the vision text. The clock orchestrator stays as the execution
shell (realtime, hardware, host solvers) over the same pure blocks, and the pure tick is its
oracle (verified byte-identical on NumPy, §10.2).

### 3.5 The name: `SampledDataSystem` (revises R127's "`HybridDiagram` for now")

The result of `computer @ plant` is a `StepDiagramSystem` subclass that knows its base clock
and its continuous plant. The final name is the textbook's, `SampledDataSystem` (Åström and
Wittenmark); `HybridDiagram` names the implementation (RULES 2.1). Cost: one sweep of the
six hybrid demos, tutorial 06 and DESIGN; no alias (R82). Done with the loop's landing, so the
name changes once.

### 3.6 `jax.grad` over `pe.params` needs `allow_int=True` (a finding, not a revision)

With the seed an integer leaf (R118, R129), `jax.grad(lambda p: pe.evaluate(p).objective)(pe.params)`
as DESIGN §6 writes it **refuses** ("grad requires real- or complex-valued inputs … got
int64"; verified, §10.4). The contract keeps the integer seed and documents the one-liner as
`jax.grad(f, allow_int=True)`, whose seed entry is a `float0` (no gradient); `pe.program(wrt)`
is unaffected, since `wrt` never names an integer leaf (R131). A library helper is not
needed; the docs line changes.

### 3.7 The planner interface: `solve(params=, z0=)`, `shift(z)` and nothing else (makes R135 concrete)

A planner builds its program once and is solved many times with `params` (R130); the MPC
owns the loop. The interface is two verbs: `planner.solve(params=None, z0=None) ->
PlanningSolution` (the solution carries the planner's decision `z` in its record and the
input sequence in its `trajectory`), and `planner.shift(z) -> z0` (what "one period later"
means for its decision). The native fixed-iteration planner is the same class with
`n_iter=K`; its `solve` traces. No `plan`, `tick` or `compile_parametric_program` verb.

### 3.8 The native solver is a kernel the `Optimizer` runs; `n_iter` is the fixed-K form (as R132)

`Optimizer(program, method="adam", n_iter=K).solve(params=, z0=)` runs the pure
`init` / `step` kernel as one scan, traceable in every mode; without `n_iter`, the method's
stopping rule (a host loop, or a `while_loop` on JAX without reverse-mode gradient). The
step-system view `as_step_system(optimizer)` is a teaching face added with its lesson.

### 3.9 `NeuralPolicy` now, not at v1.0 (revises the timing of R134 line 4)

The rename `NeuralPolicyController` → `NeuralPolicy` lands in the v0.3 name sweep, between
terms, with every notebook: v0.3 already moves every RL notebook (the seeded policy re-pins
their numbers), so a second migration at v1.0 costs more than doing it once now. Cost: the
GRO860 notebooks' import line (the freeze is paused between terms, ROADMAP §4.1 gate 7).

### 3.10 What I kept as ruled, deliberately

R124 (three owners of a start), R125 (within-tick propagation), R126 (the operators on the
step algebra), R128 (disturbances are source blocks; one `realize` shape), R129 (the seeded
Gaussian policy), R130–R131 (one program noun, the seed a leaf), R133 (a minimal `Field`,
one `as_cost`), R134 lines 1–3, 5, 6, R136–R140. Each is where my own reasoning landed too;
§6 gives their full signatures.

## 4. The user's view: the main objects and verbs

| Object | In the textbook | Its verbs (the user's) | Its `params` |
| --- | --- | --- | --- |
| `System` / `DynamicSystem` | `y = h(x, u, t; p)`, `ẋ = f(x, u, t; p)` | `f`, `h`, `tf`; `compute_trajectory`, `compile`, `linearize`, `plot_*`, `animate`; `+`, `>>`, `@`, `%` | the plant's numbers (`m`, `l`, …); `x0` beside them, its own leaf in a loop's tree |
| `StepSystem` / `StepDiagramSystem` | `x_{k+1} = step(x_k, u_k, k; p)`, on a tick, `dt` the tick's length or `None` | `step`, `h`; `compute_trajectory` (`t = k·dt`), `compile`, `jacobian`, `linearize` (poles against the unit circle); the same three operators | the block's numbers |
| `DiagramSystem` | the block diagram, `Ẋ = F(X, U, t; P)` | `connect`, `expose`, `trajectory_of`, `compute_cost(cost, of=plant)` | `{block_id: block params}`, a live view; `diagram.x0` a view that writes through |
| `Computer` (`block % dt`) | a digital law on its clock, firing `k mod d = 0` | built by `%`; `@ plant` | the scheduled blocks' trees |
| `SampledDataSystem` (`computer @ plant`) | the sampled-data loop at its base clock, the plant held between samples | every step verb; `compute_trajectory(solver=)` dense; `plot_diagram` two-sided; the realtime runner | `{"controller": …, "plant": …}`, as a continuous loop |
| `Trajectory` | `(t, x, u)` sampled, `t = k·dt` for a step run | `of("plant")`, `with_cost`, `resample`, `plot`, `save` | none (a value) |
| `Set` / `InputSet` | `z ∈ Z(t)`: `margin(z, t; p) ≥ 0` | `margin`, `contains`, `sample(key, n)`, `bounding_box`, `&`, `as_cost` | the set's numbers (a box's bounds, a ball's center and radius); `{}` when none |
| `Distribution` | a law over a vector: `sample(key, n; p)`, `mean` | `sample`, `mean`, `support` | `mean`, `std` / `cov`, the bounds |
| `Field` | `V(x, u, t; p)`: a Lyapunov function, a cost-to-go, a barrier | `value`, `gradient`, `as_set(lower=, upper=)`, `as_input_set`, `as_cost(weight=, shaping=, terminal=)` | the field's numbers (`P`, `center`) |
| `CostFunction` | `g(x, u, t; p)`, `h(x, t; p)` | `g`, `h`, `+`, `*`, `g_max` / `h_max` | `Q`, `R`, `S`, `xbar`, `ubar`; terms by id in a sum |
| source blocks (`Step`, `WhiteNoise`, `TrajectorySource`) | `w(t; p)`, a noise its seed and `psd` | `h`; `realize(key)` | the signal's numbers, `seed` an integer leaf, `values` a table |
| `GaussianPolicy` | `u = μ_θ(x) + σ_θ ε(seed, ⌊t/Δ⌋)` | `h`, `log_prob(x, u)`, `entropy(x)` | `{"mean": θ, "log_std": …, "seed": int}` |
| `PlanningProblem` | the task: `sys`, `X`, `U`, `X0`, `Xf`, `cost`, `tf`, `ρ`, the price | `compile(controller, dt)`, `discretize(dt)`, `transition` (step problems), `realize(key, n_trials)`, `replace`, `cost_bound` | `{"plant", "cost", "X", "U", "X0", "Xf", "x_start", "discount_rate"}`, a view of the parts |
| `StochasticPlanningProblem` | the same with laws on the start and the parameters; noise as blocks on ports | the same; `realize` draws; `nominal()` | the same tree; the laws' numbers under `"x0"` and `"laws"` |
| the problem evaluator (`pe = problem.compile(ctl, dt)`) | the closed loop run to the horizon and scored | `params`, `trajectory(params, key)`, `evaluate(params, key, n_trials)`, `program(wrt, bounds)` | one tree: every block by id, the problem's leaves, each free input |
| `Evaluation` | the scores of the runs | `objective`, `mean`, `std`, `worst`, `cvar`, the rates; `print` | none (a pytree record) |
| `MathematicalProgram` | `min J(z; p)` s.t. `h = 0`, `g ≥ 0`, bounds | `J`, `h`, `g` with `params=None`; `params_at(z)` on an export | the problem's tree (`x0`, `plant`, `cost`, the sets, `seed`) |
| `Optimizer` | the solver boundary: a method on a program | `solve(params=, z0=)`; with `n_iter=K` the fixed-K traced solve; `rollout(z0, params, n_steps)` the iterates as a `Trajectory` | the method's numbers (`eta`, `b1`, `b2`) |
| planners (`LQRPlanner`, `DynamicProgrammingPlanner`, `TrajectoryOptimizationPlanner`, `RRTPlanner`, `ReinforcementLearningPlanner`, `TabularLearningPlanner`) | a method on a problem | `solve(params=, z0=)` → `PlanningSolution`; `shift(z)`; the method's own plots | the method's numbers |
| `PlanningSolution` | a policy, its trajectory and its evidence | `policy`, `trajectory`, `evaluation`, `cost_to_go` (a `Field`), `solver` (the record, `z` inside), `plot_*` | none |
| `ModelPredictiveController` | a receding-horizon law: one solve per period, applied one period later | a `StepSystem`: `step`, `h`; `% dt @ plant` | `{"cost": …, "model": …, "eta": …}`: its planner's problem tree, the native method's numbers |

## 5. The pipelines, as user scripts

Every script imports through the root prelude. Each ends with what prints, what plots, and
the `params` tree the user sees. The numbers in the "prints" blocks are the shape of the
output, not pinned values.

### 5.1 Modelling and simulation (week 1)

```python
"""A pendulum, a custom plant, a source, a loop: the first lesson."""
import numpy as np
from minilink import DynamicSystem, Pendulum, Step, PD, linearize

plant = Pendulum()                              # ẋ = f(x, u, t; p): m, l, I, g, d in plant.params
plant.x0 = np.array([0.5, 0.0])
traj = plant.compute_trajectory(tf=5.0)         # a Trajectory (t, x, u); plant.traj keeps it
plant.plot_trajectory()
plant.animate()


class Tank(DynamicSystem):                       # a student's own plant: the equations, nothing else
    def __init__(self):
        super().__init__(n=1, input_dim=1, output_dim=1)
        self.params = {"A": 1.0, "a": 0.1, "g": 9.81}

    def f(self, x, u, t=0.0, params=None):
        params = self.params if params is None else params
        A, a, g = params["A"], params["a"], params["g"]
        # A ḣ = q − a √(2 g h)
        dh = (u[0] - a * np.sqrt(2.0 * g * x[0])) / A
        return np.array([dh])


tank = Tank()
tank.x0 = np.array([1.0])
(Step(final_value=[0.5]) >> tank).compute_trajectory(tf=20.0, show=True)   # a source into a plant
loop = PD(Kp=20.0, Kd=4.0) @ plant               # the loop closed through an Error block
loop.plot_diagram()
loop.compute_trajectory(tf=5.0, show=True)
print(loop)
print(loop.params)
```

- **Prints:** `print(loop)` → `PD @ Pendulum, DiagramSystem, n = 3 (controller: d_filt0; plant: theta, dtheta), inputs r (1), outputs y (2)`; `print(loop.params)` → `{"controller": {"Kp": [20.], "Kd": [4.], "tau": [0.1], "u_min": …, "u_max": …}, "plant": {"m": 1.0, "l": 1.0, "I": 1.0, "gravity": 9.81, "d": 0.0}}`.
- **Plots:** the time signals of the pendulum; the animation; the tank's step; the loop's block diagram and its run.
- **The tree:** a diagram's `params` is `{block_id: block.params}`, a live view (today's contract, `core/wiring.py:54`); `loop.x0` is a view of the blocks' `x0` that writes through (R124).

### 5.2 Classical analysis and the digital implementation (GRO501)

```python
"""Model, loop shape, design to spec, then the difference equation on the computer."""
from minilink import Pendulum, PID, Step, linearize, bode, margins, step_info, transfer_function

plant = Pendulum()
G = linearize(plant, x_bar=[0.0, 0.0])          # an LTISystem at the operating point
print(transfer_function(G))
plot_bode = bode(G, show=True)

pid = PID(Kp=30.0, Ki=10.0, Kd=5.0, tau=0.02)
L = pid >> G                                    # the loop gain
print(margins(L))
loop = pid @ plant                              # the nonlinear loop
print(step_info(loop, tf=3.0))
loop.plot_bode()                                 # of the linearized loop

# The digital implementation: the same law discretized at its period, held between samples
sampled = pid % 0.01 @ plant                    # a SampledDataSystem on the 100 Hz clock
sampled.plot_diagram()                           # the computer and the plant, two sides
traj_c = loop.compute_trajectory(tf=3.0)
traj_d = sampled.compute_trajectory(tf=3.0)      # the pure base tick, substeps recorded
sampled.plot_trajectory(signals=("plant:y", "controller:u"))
print(sampled)
print(sampled.linearize().poles())               # the sampled-data loop, against the unit circle
```

- **Prints:** `transfer_function(G)` → `G(s) = 1 / (2 s² + 9.81)` (the textbook form); `margins(L)` → `gain margin 12.3 dB at 9.8 rad/s, phase margin 48.1° at 3.2 rad/s`; `step_info` → `rise time 0.21 s, overshoot 8.4 %, settling time 0.9 s, final error 0`; `print(sampled)` → `PID % 0.01 @ Pendulum, SampledDataSystem, dt = 0.01, n = 3 (controller: e_int0, d_filt0; plant: theta, dtheta)`; the poles as a list of complex numbers inside the unit circle.
- **Plots:** Bode of `G`, Bode of the loop, the two-sided diagram, the sampled run with the staircase command.
- **The tree:** `sampled.params` is `{"controller": pid.params, "plant": plant.params}`: the same keys as the continuous loop, so a notebook moving from `pid @ plant` to `pid % 0.01 @ plant` changes one line.

### 5.3 State-space design with an observer, Monte Carlo over draws

```python
"""LQG on a noisy plant: the design, the loop by role, the draws scored."""
import numpy as np
from minilink import (CartPole, QuadraticCost, StochasticPlanningProblem, Gaussian, Uniform,
                      WhiteNoise, lqg, closed_loop, evaluate, compare, lqr, kalman)

plant = CartPole()
Q, R = np.diag([10.0, 100.0, 1.0, 1.0]), np.eye(1)
W, V = 0.05, 1e-4                                # process and measurement noise intensities

box = lqg(plant, Q, R, Q_n=W, R_n=V)              # the observer and the gain in one box
loop = box @ plant                                # roles: controller (observer inside), plant

problem = StochasticPlanningProblem(
    plant, cost=QuadraticCost.from_system(plant, Q, R), tf=5.0,
    x0=Gaussian([0.0, 0.2, 0.0, 0.0], std=[0.1, 0.1, 0.0, 0.0]),
    params={"plant": {"m_pole": Uniform(0.08, 0.12)}},
    disturbances={"d": WhiteNoise(1, psd=W, sample_period=0.01),
                  "n": WhiteNoise(1, psd=V, sample_period=0.01)},   # source blocks on the ports
    X=plant.state.box, U=plant.input_box, infeasible_cost=True,
)
report = evaluate(problem, loop, dt=0.01, n_trials=200, key=1)   # one key, 200 realizations
print(report)
print(problem.params)
x0s, params = problem.realize(key=1, n_trials=3)                  # the draws themselves
print(params["plant"]["m_pole"], params["d"]["seed"])

K_only = lqr(plant, Q, R)                          # state feedback on the true state, for comparison
cmp = compare(problem, LQG=loop, LQR=K_only, dt=0.01, n_trials=200, key=1)
print(cmp)
cmp.plot_trajectory()
problem.compile(loop, dt=0.01).trajectory(key=1).of("plant").plot()   # one realization, the plant's run
```

- **Prints:** `report` → `J over 200 runs: objective 3.12, mean 3.12, std 0.41, worst 4.6; feasible 100%, reached 100%`; `problem.params` → `{"plant": {...}, "cost": {"Q": …, "R": …, "S": …, "xbar": …, "ubar": …}, "X": {"lower": …, "upper": …}, "U": {...}, "x_start": [0, 0.2, 0, 0], "discount_rate": 0.0, "x0": {"mean": …, "std": …}, "laws": {"plant.m_pole": {"lower": 0.08, "upper": 0.12}}, "d": {"psd": 0.05, "sample_period": 0.01, "seed": None}, "n": {...}}`; the three drawn masses and three seeds; the comparison table, one row per entry with the same draws (common random numbers).
- **Plots:** the overlaid trajectories of the two loops; one realization's plant run.
- **The tree:** the problem's system is the diagram `WhiteNoise(d) + WhiteNoise(n) → plant` (R128), so `pe.params` is `{"controller": {"observer": {...}, "gain": {"K": …}}, "plant": {...}, "d": {"seed": …}, "n": {"seed": …}, "cost": …, "x_start": …, …}`; `realize` fills every `seed` and every drawn leaf with a trial axis.

### 5.4 The digital loop with its gradient (the sampled face)

```python
"""A digital PID tuned by gradient through the sampled loop; the continuous law for comparison."""
import jax
import numpy as np
from minilink import Pendulum, PID, PlanningProblem, QuadraticCost, Optimizer

plant = Pendulum()
cost = QuadraticCost.from_system(plant, Q=np.diag([1.0, 0.0]), R=0.01 * np.eye(1),
                                 xbar=[1.0, 0.0], ubar=[plant.params["m"] * 9.81 * np.sin(1.0)])
problem = PlanningProblem(plant, cost=cost, tf=5.0)
pid = PID(Kp=1.0, Ki=0.0, Kd=1.0, ports="reference")

pe = problem.compile(pid % 0.05, dt=0.01)          # the sampled loop: PID at 20 Hz, plant stepped at 100 Hz
print(pe.params)
print(pe.evaluate())                                # one J: the cost integrated with the state
dJ = jax.grad(lambda p: pe.evaluate(p).objective, allow_int=True)(pe.params)
print(dJ["controller"]["Kp"], dJ["plant"]["m"])

program = pe.program(wrt=["controller.Kp", "controller.Ki", "controller.Kd"])
best = Optimizer(program, method="l-bfgs").solve()
print(best)
tuned = program.params_at(best.z)                   # the full tree with the tuned leaves
print(pe.evaluate(tuned))
pe.trajectory(tuned).plot(signals=("plant:y", "controller:u"))

pe_c = problem.compile(pid, dt=0.01)                # the same law acting continuously
print(pe_c.evaluate(tuned))                         # the two loops, one score
```

- **Prints:** the tree below; `J = 2.289, cost = 2.289, feasibility margin = inf`; two numbers; the optimizer's result line `l-bfgs: success, 23 iterations, J = 0.412`; the tuned evaluation; the continuous law's evaluation of the same gains (close, not equal: it acts between samples).
- **Plots:** the tuned run, the angle and the staircase torque.
- **The tree (`pe.params`):** `{"controller": {"Kp": [1.], "Ki": [0.], "Kd": [1.], "tau": [.1], "u_min": …, "u_max": …, "e_int_min": …, "e_int_max": …, "x0": [0., 0.]}, "plant": {"m": 1.0, "l": 1.0, "I": 1.0, "gravity": 9.81, "d": 0.0}, "cost": {"Q": …, "R": …, "S": …, "xbar": …, "ubar": …}, "x_start": [0., 0.], "discount_rate": 0.0, "r": [1.0]}` (the free input `r` at its nominal; the controller's `x0` a leaf, R124).

### 5.5 Trajectory optimization and MPC, the program rebound without recompiling

```python
"""A swing-up planned offline, then the same program inside an MPC, the plant's mass live."""
import numpy as np
from minilink import (Pendulum, PlanningProblem, QuadraticCost, SingletonSet, BoxInputSet,
                      TrajectoryOptimizationPlanner, ModelPredictiveController, Optimizer, Step)

plant = Pendulum()
problem = PlanningProblem(
    plant, cost=QuadraticCost.from_system(plant, Q=np.diag([1.0, 0.1]), R=0.01 * np.eye(1),
                                          xbar=[np.pi, 0.0]),
    x_start=[0.0, 0.0], Xf=SingletonSet([np.pi, 0.0]), U=plant.input_box, tf=3.0,
)
planner = TrajectoryOptimizationPlanner(problem, transcription="direct_collocation", n_steps=40,
                                        backend="jax", method="ipopt")
solution = planner.solve()                            # offline: the plan and its evidence
print(solution)
solution.plot_trajectory()
(solution.policy >> plant).animate()                  # the open-loop plan replayed

program = planner.program()                           # the MathematicalProgram, compiled once
print(program)                                        # n_z, the rows, and its params tree
heavy = Optimizer(program, method="ipopt").solve(params={"plant": {"m": 1.3}, "x0": [0.2, 0.0]})
print(heavy)                                          # the same program, a heavier pendulum, no rebuild
print(program.params_at(heavy.z)["plant"]["m"])

mpc = ModelPredictiveController(planner, sample_period=0.1, parameter_ports=["plant.m"])
loop = mpc % 0.1 @ plant                              # the receding horizon on the 10 Hz clock
loop.params["plant"]["m"] = 1.3                       # the true plant is heavier; the port m stays at nominal
loop.plot_diagram()
traj = loop.compute_trajectory(tf=4.0)                # the orchestrator ticks the host solver
loop.plot_trajectory()
loop.animate()                                        # the horizon overlay drawn by the MPC's own hook

aware = (Step(final_value=[1.3], port="m") + mpc % 0.1) @ plant   # the estimate fed to the port m
print(aware.compute_trajectory(tf=4.0).of("plant").with_cost(problem.cost).signals["J"][-1])
```

- **Prints:** `solution` → `TrajectoryOptimizationPlanner: success (defects ≤ 1e-6), 40 knots, J = 7.84, ipopt 31 iterations`; `program` → `MathematicalProgram, n_z = 123, h: 120 rows, bounds on 40, params: x0, plant (m, l, I, gravity, d), cost (Q, R, S, xbar, ubar), Xf (point)`; `heavy` → the optimizer's result line; `1.3`; the closed-loop cost of the model-aware loop.
- **Plots:** the plan; the animation of the replay; the two-sided diagram with the `m` port unwired (at nominal) then wired; the closed-loop run; the animation with the horizon overlay.
- **The tree:** `program.params` is the problem's tree plus `"x0"`; `mpc.params` is `{"cost": …, "model": plant tree, "sets": …}` (the planner's problem tree, R26's key `plant` renamed `model` inside the controller so a loop's tree never has two `plant` keys: `loop.params = {"controller": {"model": …, "cost": …}, "plant": …}`).

### 5.6 The native fixed-iteration MPC, tuned by gradient (join 2)

```python
"""An MPC whose tick is K Adam steps on single shooting: compiled, batched, differentiated."""
import jax
import numpy as np
from minilink import (Pendulum, PlanningProblem, StochasticPlanningProblem, QuadraticCost, Gaussian,
                      TrajectoryOptimizationPlanner, ModelPredictiveController, Optimizer)

plant = Pendulum()
cost = QuadraticCost.from_system(plant, Q=np.diag([1.0, 0.1]), R=0.01 * np.eye(1), xbar=[0.4, 0.0])
task = PlanningProblem(plant, cost=cost, U=plant.input_box, tf=3.0)
planner = TrajectoryOptimizationPlanner(task.replace(tf=1.0), transcription="shooting", n_steps=20,
                                        method="adam", n_iter=10, options={"eta": 0.3})
mpc = ModelPredictiveController(planner, sample_period=0.05)       # a StepSystem: state z, one period late

pe = task.compile(mpc % 0.05, dt=0.05)                              # the closed loop in one scan
print(pe.evaluate())
pe.trajectory().plot(signals=("plant:y", "controller:u"))

dJ = jax.grad(lambda p: pe.evaluate(p).objective, allow_int=True)(pe.params)
print(dJ["controller"]["cost"]["Q"])                                 # the loop's J against the MPC's own weights

program = pe.program(wrt=["controller.cost.Q", "controller.cost.P"],
                     bounds={"controller.cost.Q": (0.0, None), "controller.cost.P": (0.0, None)})
tuned = Optimizer(program, method="gd", options={"eta": 0.3, "maxiter": 5}).solve()
print(pe.evaluate(program.params_at(tuned.z)))

robust = StochasticPlanningProblem.from_problem(task, x0=Gaussian([0.0, 0.0], std=[0.3, 0.0]))
pe_r = robust.compile(mpc % 0.05, dt=0.05)
print(pe_r.evaluate(n_trials=64, key=0))                             # 64 closed-loop MPC runs, one vmap
```

- **Prints:** `J = 0.438, cost = 0.438, feasibility margin = 2.1`; the gradient matrix; the tuned evaluation (`J = 0.410`); the Monte Carlo line over 64 runs.
- **Plots:** the angle settling at 0.4 rad with the torque the inner solver converged to across ticks.
- **The tree (`pe.params`):** `{"controller": {"cost": {"Q", "R", "S", "xbar", "ubar"}, "model": {"m", "l", "I", "gravity", "d"}, "eta": 0.3, "x0": zeros(20)}, "plant": {...}, "cost": {...}, "x_start": …, "discount_rate": 0.0}`. `n_steps`, `n_iter` and `sample_period` are structural (R131); `eta` is a leaf, so the step size is tunable too.

### 5.7 Value iteration and reinforcement learning on one `J`, a field as terminal cost

```python
"""Three solvers, one score: LQR's value as a field, value iteration, a neural policy."""
import numpy as np
from minilink import (Pendulum, PlanningProblem, QuadraticCost, LQRPlanner, DynamicProgrammingPlanner,
                      ReinforcementLearningPlanner, compare, evaluate)

plant = Pendulum()
running = QuadraticCost.from_system(plant, Q=np.diag([1.0, 0.1]), R=0.01 * np.eye(1), xbar=[np.pi, 0.0])
problem = PlanningProblem(plant, cost=running, X=plant.state.box, U=plant.input_box, tf=10.0,
                          infeasible_cost=True)

lqr = LQRPlanner(problem).solve()
V = lqr.cost_to_go                                     # a QuadraticField: V(x) = (x − x̄)ᵀ P (x − x̄)
print(V)
V.plot(problem.X)                                      # the field over the state box
problem = problem.replace(cost=running + V.as_cost(terminal=True))   # the LQR value ends the horizon

step = problem.discretize(dt=0.05)                     # the same problem on discretize(plant, 0.05)
print(step)
x_next, c = step.transition([0.0, 0.0], [1.0], k=0)   # the Bellman transition every solver reads
print(x_next, c)

vi = DynamicProgrammingPlanner(problem, x_grid=(101, 101), u_grid=(11,), dt=0.05).solve()
rl = ReinforcementLearningPlanner(problem, dt=0.05, algorithm="ppo").solve(timesteps=200_000)
print(vi.cost_to_go)                                   # a GridField
vi.plot_cost_to_go()
rl.plot_control_law()

cmp = compare(problem, LQR=lqr, VI=vi, PPO=rl, dt=0.05, n_trials=1)   # one score for three policies
print(cmp)
cmp.plot_control_law()
```

- **Prints:** `V` → `QuadraticField, center [π, 0], P 2×2`; `step` → `PlanningProblem on Discretized(Pendulum), dt = 0.05, N = 200, γ = 1.0`; the transition pair; `vi.cost_to_go` → `GridField on StateSpaceGrid 101×101`; the comparison table: `J` per entry from the one judge (value iteration's own `J(x₀)` and PPO's critic beside it as the solvers' claims).
- **Plots:** the LQR value, the DP cost-to-go, the learned law, the three laws on one scale.
- **The tree:** `problem.params["cost"]` is `{"QuadraticCost": {...}, "FieldCost": {"weight": 1.0, "field": {"center": …, "P": …}}}` (a sum keyed by term id, today's rule); the DP and RL planners read `problem.discretize(dt).transition`, so their `c_k` is the judge's.

### 5.8 A stochastic sampled loop with a seeded policy, vmapped over seeds, tuned by gradient

```python
"""A Gaussian policy at its period on the plant: realizations, the mean score, its gradient."""
import jax
import numpy as np
from minilink import (Pendulum, StochasticPlanningProblem, QuadraticCost, Gaussian, Uniform,
                      NeuralPolicy, GaussianPolicy, WhiteNoise, Optimizer)

plant = Pendulum()
problem = StochasticPlanningProblem(
    plant, cost=QuadraticCost.from_system(plant, Q=np.diag([1.0, 0.1]), R=0.01 * np.eye(1), xbar=[1.0, 0.0]),
    tf=3.0, x0=Gaussian([0.0, 0.0], std=[0.3, 0.1]), params={"plant": {"m": Gaussian(1.0, 0.1)}},
    disturbances={"d": WhiteNoise(1, psd=0.01, sample_period=0.02)},
)
policy = GaussianPolicy(NeuralPolicy(plant, hidden=(32, 32)), log_std=np.log(0.3), sample_period=0.1)
print(policy.params["seed"], policy.log_prob(np.zeros(2), np.array([0.1])))

pe = problem.compile(policy % 0.1, dt=0.02)            # the policy at 10 Hz, the plant and the noise at 50 Hz
x0s, params = problem.realize(key=3, n_trials=4)       # the draws: starts, masses, every seed by path
print(params["controller"]["seed"], params["d"]["seed"], params["plant"]["m"])
report = pe.evaluate(n_trials=64, key=3)               # 64 realizations in one vmap
print(report)

grad = jax.grad(lambda p: pe.evaluate(p, n_trials=64, key=3).objective, allow_int=True)(pe.params)
print(grad["controller"]["log_std"], grad["controller"]["seed"].dtype)

program = pe.program(wrt=["controller.mean.mlp", "controller.log_std"], n_trials=64, key=3)
best = Optimizer(program, method="adam", n_iter=300).solve()         # the sample-average program
print(pe.evaluate(program.params_at(best.z), n_trials=64, key=4))    # scored on fresh draws
pe.trajectory(key=3).of("plant").plot()
```

- **Prints:** `None` and a log-density; four seeds, four seeds, four masses; `J over 64 runs: objective 3.55, mean 3.55, std 0.28, …`; the gradient of `log_std` and `float0` for the seed; the tuned evaluation on key 4.
- **Plots:** one realization's plant run.
- **The tree (`pe.params`):** `{"controller": {"mean": {"mlp": {...}}, "log_std": …, "seed": None}, "plant": {...}, "d": {"psd": 0.01, "sample_period": 0.02, "seed": None}, "cost": …, "x_start": …, "x0": {"mean", "std"}, "laws": {"plant.m": {"mean", "std"}}, "discount_rate": 0.0}`. `evaluate(key=3)` realizes it: the `seed` leaves become integers with a trial axis, `plant.m` a drawn vector, `x_start` the drawn starts. `seed=None` is the mean policy, the deterministic face.

## 6. The final API, object by object

Signatures are given in full; `today:` cites the code as it stands, `changes:` what moves.
Equation paths keep bare signatures (RULES 5.1).

### 6.1 Systems

```python
class System:                                    # core/system.py:26
    def __init__(self, n=0)
    params: dict                                 # numbers and integer seeds only (R118)
    x0: ndarray                                  # the block owns its start (R124)
    def h(self, x, u, t=0, params=None)
    def tf(self, x, u, t=0, params=None)
    def realize(self, key) -> dict               # every random block reseeded, by id
    def __add__ / __rshift__ / __matmul__        # +, >>, @ (core/operators.py, core/closed_loop.py)
    def __mod__(self, schedule) -> Computer      # % : a digital law on its clock (§6.4)

class DynamicSystem(System):                     # core/system.py:516
    def f(self, x, u, t=0, params=None)
    def compute_trajectory(self, t0=0, tf=10, n_steps=None, dt=None, solver=None, show=False,
                           x0=None, backend="numpy", verbose=True) -> Trajectory
    def compile(self, backend="numpy") -> CompiledSystem   # DynamicEvaluator today
    def linearize(self, x_bar=None, u_bar=None, t=0.0, params=None) -> LTISystem

class StepSystem(System):                        # core/discrete/step_system.py:16
    dt: float | None                             # NEW: the tick's length in seconds, None for a pure tick
    def step(self, x, u, k=0, params=None)
    def h(self, x, u, k=0, params=None)
    def compute_trajectory(self, n_steps=None, tf=None, u=None, *, x0=None, backend="numpy",
                           show=False) -> Trajectory       # t = k·dt (k when dt is None); replaces compute_rollout
    def linearize(self, x_bar=None, u_bar=None, k=0, params=None) -> DiscreteLTISystem  # NEW (R122's slice)

class DiagramSystem(DynamicSystem):              # core/diagram.py:19
    params: dict                                 # {block_id: block.params}, live (core/wiring.py:54)
    x0: ndarray                                  # NEW: a view that writes through (R124)
    def add_subsystem(self, sys, sys_id) / connect(...) / expose(*names) / autowire()
    def trajectory_of(self, subsystem, traj=None) -> Trajectory

class StepDiagramSystem(StepSystem):             # core/discrete/step_diagram.py:13
    # the same operators +, >>, @ (R126), built on core/assembly.py; mixing kinds refused
```

- `changes:` `compile_backend=` → `backend=` on the facades (`core/facades/simulation.py:42`);
  `StepSystem.rollout` and `HybridDiagram.last_result` caches go, `self.traj` stays (RULES 2.4);
  `StepRollout` (`core/discrete/step_rollout.py`) folds into `Trajectory` with `t = k·dt`;
  `compute_rollout` (`core/facades/simulation.py:368`) folds into `compute_trajectory`.
- Params trees: a leaf's own dict; a diagram's `{id: tree}`; a reserved id (`plant`,
  `controller`, `observer`) never suffixed.

### 6.2 The compiled system (research lane; `Compiled…` names at v0.9, R134)

```python
class CompiledSystem:                            # core/compile/evaluators/base.py:39 (Evaluator)
    params: dict                                 # the snapshot; params=None reads it
    def f(self, x, u, t=0.0, params=None)
    def outputs(self, x, u, t=0.0, params=None) -> dict
    def signals(self, x, u, t=0.0, params=None)
    def step(self, x, u, t, params=None, *, dt, solver="rk4", substeps=1)       # base.py:96
    def rollout(self, x0, u_seq, t0, dt, params=None, *, solver="rk4", substeps=1, hold="zoh", n_steps=None)
    def rollout_batch(self, x0s, u_sequences=None, *, t0=0.0, dt, n_steps=None, params=None)
    def jacobian(self, of, wrt) -> callable

class CompiledStepSystem:                        # core/compile/evaluators/discrete.py:32
    def step(self, x, u, k=0, params=None)
    def rollout(self, x0, *, n_steps, u=None, params=None)   # changes: native arrays out, traced, batched
    def rollout_batch(self, x0s, *, n_steps, u=None, params=None)   # NEW: a vmap over starts, seeds, families
```

### 6.3 The step model and the Bellman transition (the one kernel, `core/compile/evaluators/stepping.py`)

```python
def rk4(f, x, ua, ub, t, h)                      # stepping.py:17
def euler(f, x, ua, ub, t, h)                    # stepping.py:32
def step_body(f, x, u0, u1, t, dt, solver, substeps)    # stepping.py:40, unchanged
def iterate(evaluator, transition, x0, sequences)       # stepping.py:62: a loop on NumPy, lax.scan on JAX

def augmented_rhs(f, g, rho, p_plant, p_cost)    # NEW: F([x; c], u, t) = [f(x,u,t;p); e^{−ρt} g(x,u,t;p)]
def transition_body(f, g, rho, x, u, t, dt, solver, substeps, p_plant, p_cost) -> (x_next, c)   # NEW
```

The Bellman transition is the augmented step. Verified: its `x` rows equal `step_body`'s
byte for byte, its `c` under Euler equals `e^{−ρ t_k} g dt` to the last bit (§10.1).

### 6.4 The sampled loop: `Computer`, `SampledDataSystem`, the two runners

```python
class StepSchedule:                              # simulation/discrete/computer.py:227 → core/discrete/schedule.py
    dt_base: float
    fire: dict[str, int]                         # block id → divisor d; a block fires on k mod d = 0
    solver: str = "euler"                        # NEW: how % discretizes a continuous law
    @classmethod from_rates(cls, dt_base, rates_hz) -> StepSchedule

class Computer(StepDiagramSystem):               # NEW home core/discrete/computer.py; today a runtime object
    """A digital law on its clock: the pure base tick of its blocks."""
    def __init__(self, diagram: StepDiagramSystem, schedule: StepSchedule)
    dt: float                                    # = schedule.dt_base
    def step(self, x, u, k=0, params=None)       # every block: x⁺ = where(k mod d == 0, step_i, x); held outputs in x
    def __matmul__(self, plant) -> SampledDataSystem

System.__mod__(self, schedule) -> Computer       # a float is StepSchedule(dt, {id: 1}); a continuous law
                                                 # with states is discretize(self, dt, solver=schedule.solver) first

class SampledDataSystem(StepDiagramSystem):      # NEW, replaces HybridDiagram (core/discrete/hybrid_diagram.py:30)
    """computer @ plant at the base clock: {controller: the computer's blocks, plant: discretize(plant, dt, substeps)}."""
    dt: float
    plant: DynamicSystem                         # the continuous source, kept (the Discretization precedent)
    def compute_trajectory(self, t0=0, tf=10, *, solver=None, substeps=None, x0=None, backend="numpy",
                           show=False) -> Trajectory
        # default: the pure step diagram scanned, the plant's substeps recorded (dense);
        # solver="scipy*": the clock orchestrator, the plant adaptive between samples (simulation/discrete/)
    def plot_diagram(self, abstract_boundary=True)         # the two-sided drawing, kept
    def game(self, ...)                          # the realtime runner, over the same pure blocks
```

- The hold state: a block with `d > 1` gets one state slot per output port (its last fired
  value); a single-rate computer adds none, so `pid % 0.01 @ plant` is literally the two
  blocks `discretize(pid, 0.01)` and `discretize(plant, 0.01, substeps)` wired as `pid @ plant`
  is (`core/closed_loop.py:76`, the same `closed_loop` on the step kinds, R126).
- Propagation within the tick (R125): the step diagram's gather order, `topology.py`'s.
- `today:` `Computer.tick` (`computer.py:114`) mutates `k`, `x` and two buffers and reads the
  previous tick; `HybridSimulator.run` (`hybrid_simulator.py:145`) is a Python loop that
  re-implements the hold. `changes:` both become runners of `SampledDataSystem.step`; the
  orchestrator keeps only the plant's adaptive integration (`solve_ivp` restarted at each
  change of the held plant input) and the realtime I/O.

### 6.5 Trajectories, sets, distributions, costs, fields, sources

```python
@dataclass(frozen=True)
class Trajectory:                                # core/trajectory.py:52, a pytree
    t: ndarray; x: ndarray; u: ndarray; signals: dict; roles: dict
    def of(self, role) -> Trajectory; def with_cost(self, cost, *, discount_rate=0.0, params=None)
    dt: float | None                             # NEW: the step when t is a uniform grid (a step run)

class Set:                                       # core/sets.py:22
    params: dict                                 # changes: {} when none, never None (R140)
    def margin(self, z, t=0.0, params=None)
    def contains(self, z, t=0.0, params=None)
    def sample(self, key, n=None, params=None)   # changes: key required (RN-2)
    def bounding_box(self) -> BoxSet | None
    def as_cost(self, weight=1.0, shaping=None) -> CostFunction   # NEW: Field.as_cost on the margin (R133)

class Distribution:                              # core/distributions.py:24
    params: dict                                 # changes: mean, std / cov, bounds become leaves (R140)
    def sample(self, key, n=None, params=None)
    mean: ndarray; support: Set | None

class CostFunction:                              # core/costs.py:29
    def g(self, x, u, t=0.0, params=None); def h(self, x, t=0.0, params=None)
    def g_max(self, X, U, params=None); def h_max(self, X, params=None)
    # changes: integrate(traj) stays for a held trajectory (the trapezoid); the judge no longer calls it

class Field:                                     # NEW core/fields.py (StateField, planning/spatial/state_fields.py:26)
    params: dict
    def value(self, x, u=None, t=0.0, params=None)
    def gradient(self, x, u=None, t=0.0, params=None)
    def as_set(self, lower=None, upper=None) -> Set; def as_input_set(self, lower=None, upper=None) -> InputSet
    def as_cost(self, weight=1.0, shaping=None, *, terminal=False) -> CostFunction
    def plot(self, X, ...)
class QuadraticField(Field):                     # NEW: (x − c)ᵀ P (x − c); a schedule (t, S) interpolated in t
class GridField(Field): ...                      # with its consumers (DP, tabular), after the core
class CallableField(Field): ...                  # the RL critics, after the core

class WhiteNoise(Source):                        # blocks/sources.py:211, unchanged: params psd, sample_period, seed
class NoiseSource(Source):                       # RN-3, with its first consumer
class TrajectorySource(Source):                  # blocks/sources.py:302; changes: the table a leaf params["values"]
```

### 6.6 The policies

```python
class NeuralPolicy(System):                      # control/neural.py:15 (NeuralPolicyController), renamed (§3.9)
    def __init__(self, sys, *, features=None, hidden=(64, 64), activation="tanh", normalize=True,
                 squash="clip", seed=0, name=None, ...)
    params: {"mlp": {...}}
    def ctl(self, x, u, t=0, params=None)        # u = u_mid + u_half · squash(MLP(z(x)))

class GaussianPolicy(System):                    # NEW control/stochastic.py (R129)
    """u = μ(x; θ) + σ(θ) ε(seed, ⌊t/Δ⌋): the seeded stochastic policy, a plain block."""
    def __init__(self, mean: System, log_std, *, sample_period, seed=None)
    params: {"mean": mean.params, "log_std": ..., "seed": int | None}
    def ctl(self, x, u, t=0, params=None)        # seed None: the mean; else the counter-based draw (distributions.py:230)
    def log_prob(self, x, u, t=0.0, params=None)
    def entropy(self, x, t=0.0, params=None)
class SquashedGaussianPolicy(GaussianPolicy)     # SAC's head, a subclass
```

- `today:` `learning/reinforcement_learning/policy.py:38` (`StochasticPolicy`, `GaussianHead`,
  `SquashedGaussianHead`) draw from their own keys. `changes:` the heads become the two
  blocks; the collectors (`collect.py:12`, `jax.random.split` per step) take per-episode seeds
  from `realize` (a written exception until then, R140 point 4).

### 6.7 The problem, its draws, the discretized problem

```python
class PlanningProblem:                           # planning/problems.py:29
    def __init__(self, sys, x_start=None, x_goal=None, cost=None, X=None, U=None, X0=None, Xf=None,
                 tf=None, infeasible_cost=None, discount_rate=0.0, *, disturbances=None)
        # NEW disturbances={"d": block}: source blocks wired onto the plant's ports (R128); a bare law refused
    params: dict                                 # the live view by role (problems.py:198), plus the sources by id
    def compile(self, controller, dt, *, solver="rk4", substeps=1, backend="auto", episode_length=None)
        # today problems.py:358; changes: a Computer, a SampledDataSystem or a StepSystem controller accepted
    def discretize(self, dt, *, solver="rk4", substeps=1) -> PlanningProblem   # NEW: the same class on discretize(sys, dt)
    def transition(self, x, u, k=0, params=None) -> (x_next, c_k)              # NEW: on a step problem only
    def realize(self, key=None, n_trials=1) -> (x0s, params)                  # changes: one shape (R128)
    def replace(self, **fields) -> PlanningProblem
    def cost_bound(self, tf=None, *, discount_rate=None, dt=None); def discount_factor(self, dt)
    def horizon_kind(self) -> str

class StochasticPlanningProblem(PlanningProblem):   # problems.py:506
    def __init__(self, sys, ..., x0: Distribution, params: dict[str, Distribution] = None,
                 criterion="expectation", disturbances=None)
        # changes: x0_distribution → x0, params_distribution → params (dotted paths, the tree's own words);
        # disturbances are blocks, as on the base class; the "disturbances" key of realize goes
    params: dict                                 # adds "x0": the start law's leaves, "laws": {path: law.params}
    def realize(self, key, n_trials) -> (x0s, params)   # every drawn leaf and every seed with a trial axis
    @classmethod def from_problem(cls, problem, **laws)   # NEW: the stochastic sibling of a task (RULES 4.3's as_stochastic)
    def nominal(self) -> PlanningProblem
```

- `transition` on a step problem: `sys` a `Discretization` → the augmented step with the
  source's `f`, the problem's `cost.g`, `ρ`, and the discretization's `dt`, `solver`,
  `substeps`; `sys` a native `StepSystem` → `(sys.step(x, u, k, p), γ^k g(x, u, t_k, p) · dt)`
  with `t_k = k·dt` (`dt = 1` when the system has none). The sets are read at `t_k` by the
  judge, never by `transition`.
- `problem.params` on a stochastic problem: `{"plant", "cost", "X", "U", "X0", "Xf",
  "x_start", "discount_rate", "x0": {...}, "laws": {"plant.m": {...}}, "d": {"psd", "sample_period", "seed"}}`.

### 6.8 The judge: `problem.compile`, the evaluation and the program

```python
class ProblemEvaluator:                          # planning/evaluator.py:40 (the name stays internal; pe in scripts)
    def __init__(self, problem, controller, dt, *, solver="rk4", substeps=1, backend="auto", episode_length=None)
    params: dict                                 # one tree: every block by id, the problem's leaves, each free input
    def trajectory(self, params=None, key=None) -> Trajectory          # evaluator.py:146
    def evaluate(self, params=None, key=0, n_trials=None, *, record=False) -> Evaluation   # evaluator.py:166
    def program(self, wrt, bounds=None, key=0, n_trials=None, *, params=None) -> MathematicalProgram  # :195
    def pack(self, params, wrt) -> z; def unpack(self, z, wrt, params=None) -> dict

def evaluate(problem, controller, *, dt, solver="rk4", substeps=1, backend="auto", n_trials=None, key=0, episode_length=None) -> Evaluation
def compare(problem, *solutions, dt, n_trials=None, key=0, **named) -> Comparison      # planning/comparison.py:25
def evaluate_trajectory(problem, traj) -> Evaluation                                    # planning/evaluation.py:223, the trapezoid on a held run

@dataclass class Evaluation:                     # planning/evaluation.py:27, unchanged
    cost, J, feasibility_margin, failed, reachability_margin, reached, x0, trajectories, criterion
    objective, mean, std, worst, cvar(alpha), feasibility_rate, reachability_rate, success_rate
```

**What changes inside `compile` (the dispatch on the controller's kind, R105):**
1. a continuous law, a source or a built continuous loop → the diagram `controller @ plant`
   as today (`closed_loop_of`, `evaluator.py:586`), its rhs augmented with the cost state;
   `run` (`evaluator.py:355`) rolls out `[X; c]` with `ev.rollout`; `score_run`
   (`evaluation.py:125`) takes `c_k` from the run instead of integrating the trapezoid;
2. a `Computer` → `computer @ plant` is built (`SampledDataSystem`), the plant block's step
   augmented; the loop's compiled step rollout is the run, one scan at `dt_base` (the
   evaluator's `dt` must be the base clock or divide it; refused otherwise, naming the fix);
3. a `StepSystem` controller on a continuous plant → refused, naming `%`;
4. a problem whose `sys` is a `StepSystem` (a native step plant, or `problem.discretize`) → the
   controller must be a step or static block; the run is the step diagram's rollout and
   `c_k` is `transition`'s.

In every case the score is `J = Σ_k c_k + e^{−ρ tf} h(x_N)`, the margins at the samples, the
price switch: `score_run`'s rule with one quadrature.

**The program export** (R130 applied): `pe.program(wrt)` returns
`MathematicalProgram(n_z, J, g, lower, upper, params=pe.params)` with
`J(z, params=None) = pe.evaluate(with_leaves(params, wrt, z), key, n_trials).objective`;
so the tuning program is parametric in every leaf it does not tune (a gain tuned under a
family of plant masses is `vmap` over `params["plant"]["m"]` of the same program). The seed of
a stochastic export is `params["seed"]` (the key), `n_trials` structural.

### 6.9 The program and the optimizer

```python
@dataclass(frozen=True)
class MathematicalProgram:                       # optimization/mathematical_program.py:26
    n_z: int
    J: callable                                  # J(z) or J(z, params): the evaluator reads the arity once
    h: callable | None = None; g: callable | None = None
    lower, upper: ndarray | None
    grad_J, hess_J, jac_h, jac_g: callable | None
    backend: str = "numpy"
    params: dict | None = None                   # NEW (R130): the frozen defaults; params=None reads them
    metadata: dict
    def params_at(self, z) -> dict               # NEW: the full tree with z written back (an export's)

class Optimizer:                                 # optimization/optimizer.py:75
    def __init__(self, program, z0=None, *, method="auto", options=None, backend=None, n_iter=None, verbose=False)
        # changes: compile_backend → backend; the adapter attribute is `adapter`; n_iter the fixed-K form (R132)
    def solve(self, *, z0=None, params=None, callback=None, record_solve_time=False, verbose=False) -> OptimizationResult
        # changes: params= merged once onto program.params at the boundary (R131), complete inside
    def rollout(self, z0=None, params=None, *, n_steps) -> Trajectory      # NEW: the iterates on k (the lesson)
    kernel: Kernel                               # the native method's pure laws, for a tick that embeds a solve

class Kernel:                                    # NEW optimization/kernels/: gd.py, adam.py (first_order.py:135 today)
    def init(self, z0, params) -> state
    def step(self, state, params) -> state       # one iteration, projected on the box; pure, on the array module
    def z_of(self, state) -> z
```

- `today:` the first-order backend (`first_order.py:22`) runs a Python loop; `Adam`
  (`first_order.py:135`) is already the kernel shape on a pytree. `changes:` the kernels are
  the one implementation; `FirstOrderOptimizer.solve` runs them (a loop on NumPy, a `scan`
  with `n_iter`, a `while_loop` on a tolerance); `ParametricMathematicalProgram`
  (`trajectory_optimization/parametric_program.py`) and `JaxParametricProgramEvaluator`
  (`parametric_evaluator.py`) fold into the one program evaluator with `(z, params)`; one
  backend factory for the offline and compiled paths (OPW-1).

### 6.10 The planners, the solution, the planner interface, the MPC

```python
class Planner:                                   # planning/planner.py:21
    def __init__(self, problem)
    def solve(self, *, params=None, z0=None, **options) -> PlanningSolution   # changes: params and z0 (R135)
    def shift(self, z) -> z0                     # NEW: the warm start one period later (planners with a decision)
    def program(self) -> MathematicalProgram     # NEW: the parametric program, built once (the trajopt family)

class TrajectoryOptimizationPlanner(Planner):    # trajectory_optimization/planner.py:64
    def __init__(self, problem, *, transcription="direct_collocation", n_steps=None, backend="auto",
                 method="auto", n_iter=None, options=None, feasibility_tol=1e-6, verbose=False, ...)
        # changes: compile_backend → backend, optimizer_method → method; n_iter makes the solve a fixed-K scan
    def solve(self, *, params=None, z0=None) -> PlanningSolution   # params: x0, plant, cost, sets; no rebuild

@dataclass class PlanningSolution:               # planning/results.py:21, basic (R138)
    problem, policy, trajectory, evaluation, cost_to_go: Field | None, solver (the record; z inside), success
    def plot_trajectory / plot_cost_to_go / plot_control_law / plot_cost

class ModelPredictiveController(StepSystem):     # trajectory_optimization/mpc/controller.py:471 (MPCStatefulController)
    def __init__(self, planner, *, sample_period, warm_start=True, parameter_ports=(), name=None)
    dt = sample_period
    params: {"cost": ..., "model": ..., "sets": ..., "eta": ...}   # the planner's problem tree, renamed inside
    def step(self, x, u, k=0, params=None)       # z⁺ = solve(params ∪ {x0: y}, z0=shift(z)); pure when the planner traces
    def h(self, x, u, k=0, params=None)          # u_k = the plan's first input, read from the state (R123)
    # no latch, no compute_command / Command, no dual_rate_computer; a Computer with two rates replaces it
```

- The MPC's `step` is a pure function of `(x, u, k, params)`; with a host solver it runs on
  the NumPy face only (the orchestrator); with `n_iter` and a native kernel it traces, so
  `problem.compile(mpc % dt, dt)` is one scan (verified end to end, §10.3).
- The dynamic programming and learning planners keep their constructors; they read
  `problem.discretize(dt).transition` for the step and the per-step cost, and the RL
  environment (`learning/reinforcement_learning/environment.py:130`) becomes its consumer:
  `reward = −c_k`.

## 7. The architecture under it

### 7.1 Layers

```text
 ┌──────────────────────────────────────────────────────────────────────────────────────┐
 │ TOOLS (verbs on a System or a problem; return objects, numbers, figures)              │
 │  simulation/  Simulator (adaptive, continuous)   the clock orchestrator, realtime      │
 │  analysis/    linearize, bode, margins, …, step linearize (unit circle)                │
 │  planning/    PlanningProblem · discretize · transition · compile → the judge          │
 │               (trajectory, evaluate, program) · planners · PlanningSolution · MPC      │
 │  learning/    the RL and tabular planners, consumers of transition and GaussianPolicy  │
 │  optimization/ MathematicalProgram(params) · Optimizer · kernels (gd, adam) · adapters │
 ├──────────────────────────────────────────────────────────────────────────────────────┤
 │ SYSTEM LIBRARIES (System subclasses by diagram role)                                   │
 │  blocks/ (sources, routing, UnitDelay) · dynamics/ (plants) · control/ (laws, policies)│
 │  estimation/ (observers)                                                               │
 ├──────────────────────────────────────────────────────────────────────────────────────┤
 │ FRAMEWORK (core/)                                                                      │
 │  system · diagram · operators/closed_loop/assembly · roles · signals · params.py       │
 │  discrete/  StepSystem · StepDiagramSystem · StepSchedule · Computer · SampledDataSystem│
 │  compile/   the compiled systems, the step model (stepping.py: step_body, the           │
 │             augmented transition, iterate = scan)                                       │
 │  trajectory · sets · distributions (the cipher) · costs · fields · geometry             │
 └──────────────────────────────────────────────────────────────────────────────────────┘
```

### 7.2 The one contract

Every core math object is a function of its arguments and `params`; `params=None` reads the
defaults; a composite's tree nests by id; leaves are floats, arrays or an integer `seed`;
the function is deterministic given the tree. The object families and their signatures:

| Family | Signature | Params tree |
| --- | --- | --- |
| system | `f(x, u, t, p)`, `h(x, u, t, p)`, `step(x, u, k, p)` | own leaves; diagrams by id |
| set, field, cost | `margin(z, t, p)`, `value(x, u, t, p)`, `g(x, u, t, p)`, `h(x, t, p)` | own leaves, `{}` when none |
| distribution | `sample(key, n, p)` | the law's numbers |
| source, policy | `h(x, u, t, p)` with `p["seed"]` | the signal's numbers, `seed` |
| problem | `transition(x, u, k, p)` on a step problem | the parts by role |
| program | `J(z, p)`, `h(z, p)`, `g(z, p)` | the problem's tree, `seed` |
| kernel | `init(z0, p)`, `step(state, p)` | the method's numbers |

`core/params.py` (new) owns the tree algebra every caller uses: `leaf(tree, path)`,
`with_leaf`, `merge(defaults, partial)`, `float_leaves`, `pack(tree, wrt)`, `unpack`; the
four copies (`planning/evaluator.py:991–1045`, `planning/problems.py:896–914`,
`core/compile/evaluators/jacobian.py`, `core/assembly.py`) go. The one contract test walks
every public core math object on both backends: its `params`, `params=None` equal to the
defaults, `jax.grad` through a float leaf, the same values twice from one seed.

### 7.3 Module layout and the dependency law

DESIGN §3's four bands stand. The moves:
- `simulation/discrete/computer.py`'s `Computer` and `StepSchedule` become
  `core/discrete/computer.py` and `schedule.py` (pure objects of the step algebra); the
  runners stay in `simulation/discrete/` (`orchestrator.py`: the clock runner with the
  adaptive plant; `realtime/` unchanged). The lazy `core → simulation.discrete` exception
  (R100) is deleted: `core` no longer needs the simulator to build `%` or `@`.
- `core/discrete/hybrid_diagram.py` and `hybrid_composition.py` become
  `core/discrete/sampled_data.py` (the class and its `@` wiring on `core/closed_loop.py`).
- `core/fields.py` (from `planning/spatial/state_fields.py` and `shaping.py`);
  `core/params.py` (new).
- `optimization/kernels/` (`gd.py`, `adam.py`, from `optimizers/first_order.py`),
  `optimization/adapters/` (`scipy_minimize.py`, `ipopt.py`); `parametric_program.py` and
  `parametric_evaluator.py` deleted.
- `control/stochastic.py` (`GaussianPolicy`, `SquashedGaussianPolicy`); the RL heads deleted
  from `learning/reinforcement_learning/policy.py`.

The law: libraries import `core` (and `dynamics/abstraction`); tools import `core` and may
consume libraries; `planning/` imports `control.stochastic` for the same reason it imports
`control.neural` (a learned law is a control block). No new cross-library import.

### 7.4 The runtime model

| Compiled once | Varies per call | Runner |
| --- | --- | --- |
| a `System.compile(backend)`: the execution plan, `f`, `step`, `rollout`, each jitted on first use | `x`, `u`, `t`, the complete `params` tree, the sequences | `rollout`: a loop on NumPy, `lax.scan` on JAX |
| `problem.compile(ctl, dt)`: the loop (continuous or sampled-data), its compiled step, the judge's scan, the batched trial (`vmap` + `jit`) per batch layout | `pe.params` (every leaf), the key, `n_trials` | one scan over `k`; the trials one `vmap` on JAX, a loop on NumPy |
| `planner.program()`: the transcription with `params` (x0, plant, cost, sets) | `params` at the solve, `z0` | the optimizer: a host adapter, or a kernel scan |
| `mpc % dt @ plant` as a `SampledDataSystem` | its tree | `compute_trajectory`: the scan (traced when every block traces) or the orchestrator (`solver="scipy"`, host blocks) |

Structural (fixed at compile; never in `params`): the topology, the port dimensions, the
schedule's divisors and `dt_base`, the substep count, `n_steps`, `n_iter`, `n_trials`, the
solver and hold names.

**Where host code stays:** SciPy and Ipopt solves (the NumPy face of the MPC, the offline
planners), the adaptive `solve_ivp` plant of the orchestrator, realtime I/O, plotting, the
DP sweep's `while` to tolerance (`lax.while_loop` on JAX, no reverse gradient), the RL
training loop's outer Python loop over updates (each update one scan, as today).

### 7.5 JAX traceability and efficiency, path by path

| Path | Why it traces | Cost and shape | Verified |
| --- | --- | --- | --- |
| the continuous loop's rollout with the cost state | `step_body` on an augmented vector; `iterate` is `lax.scan`; `params` a pytree of numeric leaves | one scan of `N` RK4 steps on `n + 1` states; memory `O(N)` under `grad` (`jax.checkpoint` per period if needed) | §10.1: `jit`, `grad` = central difference to 6 digits, `vmap` over masses and starts |
| the sampled loop's base tick | the firing is `where(k % d == 0, …)`; the holds are carry; the plant's substeps inside `step_body` | one scan over `N_base` ticks; idle blocks computed and discarded (the slow block's cost × its divisor); 300 ticks × 2 substeps in 3.2 ms jitted | §10.2: `grad` through the selects = central difference; `vmap` over starts; byte-identical to the orchestrator on NumPy |
| the judge over trials | `vmap` of the scored run over the batched tree (drawn leaves carry an axis, shared leaves `None`), as today (`evaluator.py:462`) | 32 trials × 150 RK4 steps in 8 ms | §10.4 |
| the seeded policy | `standard_normal(seed, k, p)` is Threefry on `uint32` (`distributions.py:199`), `k = ⌊t/Δ⌋` | no table, no refresh; the same bits on both backends | §10.4: same values twice; held over Δ |
| the parametric program | `J(z, params)` closes over nothing; the shooting rollout is a scan | one jit per program; a new `params` is a new call, no retrace while shapes hold | §10.3: three starts, no rebuild; L-BFGS-B on `jax.grad` |
| the native kernel's fixed-K solve | `init` / `step` pure on the array module; `scan` over `K` | `K` gradient evaluations per solve; the gradient of the solve by unrolling (memory `O(K)`) | §10.3 |
| the MPC's closed loop | the tick is `K` kernel steps on the program bound with `x0 = y`; the loop a scan over ticks | 60 ticks × 10 Adam steps × 20-sample shooting: 105 ms jitted, 0.9 s compile; `grad` through all of it | §10.3: `dJ/dQ₁₁` = central difference to 3 digits; `vmap` over 4 starts |
| integer seed leaves | they do not trace as inputs of `grad` | `jax.grad(f, allow_int=True)`: `float0` for the seed, no cost | §10.4 |

Three things do **not** trace by design, and are host code: a host solver inside a block
(the orchestrator runs it), an adaptive plant integration (Diffrax later, R139a), a `while`
to tolerance under reverse-mode `grad` (the fixed-K form is the differentiable one).

## 8. The v0.3 coverage table

| Workstream / choice | Where this design satisfies or revises it |
| --- | --- |
| §1 every object pure, every number a parameter | §6.5–§6.6 (`params = {}`, laws' leaves, the table a leaf, the seed), §7.2 (`core/params.py`, the contract test); R124 kept (§6.1 `diagram.x0` writes through; `controller.x0` a leaf, §5.4's tree) |
| §2 the step algebra first-class | §6.1 (`dt` on `StepSystem`, `compute_trajectory` with `t = k·dt`, `linearize` against the unit circle), §6.2 (native rollouts, `rollout_batch`), R125–R126 kept |
| §3 the draws | §6.7 (`disturbances=` blocks on both problems, one `realize` shape, laws' leaves), §5.3, §5.8; verified §10.4 (streams independent by name) |
| §4 the seeded stochastic policy | §6.6 `GaussianPolicy` as R129; verified §10.4 (held over Δ, deterministic, `log_prob`, gradient of the mean `J` over seeds) |
| §5 the hybrid loop: one model, two runners | §6.4: one pure base tick (§3.4), the orchestrator its runner; **revises** R127's two verbs to one (§3.2) and the name to `SampledDataSystem` (§3.5); `%` discretizes a continuous law (§3.3); R123's delay kept in the MPC's `h` reading the state |
| §6 one score | §3.1 (the answer to the open investigation), §6.3, §6.7 `transition`, §6.8 the dispatch; verified §10.1 |
| §7 the parametric program; the stochastic program | §6.9 (`params=None` on the noun, `params_at`, `solve(params=)`), §6.8 (the export parametric in its untuned leaves; `seed` the key); R130–R131 kept; verified §10.3 |
| §8 the native proof of concept | §6.9 kernels and `n_iter`, §6.10 the MPC's pure `step`; verified §10.3 (the whole closed loop traced and tuned) |
| §9 Fields | §6.5 `Field`, `QuadraticField`, the three conversions, `terminal=` for the terminal cost (§5.7); R133 kept |
| §10 hardening and the merge | §9's order; the contract test (§7.2) and the acceptance scripts of core-architecture §11, each a pipeline of §5 |
| R124 | kept (§6.1, §5.4) |
| R125, R126 | kept (§6.1, §6.4) |
| R127 | revised: one verb, `SampledDataSystem` (§3.2, §3.5) |
| R128, R129, R130, R131, R132, R133 | kept (§6.7, §6.6, §6.9, §6.5) |
| R134 | kept; line 4's rename moved to v0.3 (§3.9) |
| R135 | made concrete: `solve(params=, z0=)`, `shift`, `program()` (§3.7, §6.10) |
| R136–R139 | kept; nothing here depends on them |
| R140 | kept and written as the one contract (§7.2) |
| the one-score investigation (QUESTIONS) | answered in §3.1 |

## 9. The migration from today's code

Module by module: what changes, which numbers move, the risk, and the order.

| Step | Modules | What changes | Numbers | Risk |
| --- | --- | --- | --- | --- |
| 1. the names sweep (R134, §3.9) | facades, `Optimizer`, the planners, the MPC, the spatial `bind`, `NeuralPolicyController`, every demo and notebook | `compile_backend` → `backend`, `optimizer_method` → `method`, `dt_mpc` → `sample_period`, `bind` → `mount`, `NeuralPolicyController` → `NeuralPolicy`, `plant.u` → `plant:u` in records | none (byte-identical) | low; one sweep, no alias |
| 2. `core/params.py` and the contract pass (R140) | `planning/evaluator.py`, `problems.py`, `compile/evaluators/jacobian.py`, `assembly.py`; `sets.py`, `distributions.py`, `sources.py` (the table), `costs.py` | one tree algebra; `params = {}` for empty; laws' leaves; `sample(key, n, params)` | none, except distributions drawn by the cipher instead of `numpy.random` / `jax.random` (one intended change, R128's "one draw on both backends") | low |
| 3. three owners of a start (R124) | `core/wiring.py` (`state_layout`, `derived_x0`, `refresh`), `simulation/simulator.py`, `planning/evaluator.py` | `diagram.x0` a view; `controller.x0` a leaf of `pe.params` | byte-identical | medium: `refresh` has three callers |
| 4. the step algebra (R125, R126) and the fold | `core/discrete/`, `core/assembly.py`, `closed_loop.py`, `compile/evaluators/discrete.py`, `facades/simulation.py`, `core/trajectory.py` | the operators on step kinds; native step rollouts and `rollout_batch`; `StepRollout` → `Trajectory(dt=)`; `compute_rollout` → `compute_trajectory`; `StepSystem.dt`, `linearize` | the hybrid demos move once (within-tick propagation, R125) | medium |
| 5. the one score (§3.1) | `compile/evaluators/stepping.py` (the augmented transition), `planning/problems.py` (`discretize`, `transition`), `planning/evaluator.py` (`run` on `[X; c]`), `evaluation.py` (`score_run` takes `c_k`), `dp.py`, `discretizer.py`, `learning/.../environment.py` | one transition, one quadrature | the evaluator's `J` moves once (trapezoid → integrated); DP and RL numbers unchanged under Euler (verified); shooting's reported evaluation moves | low: the change is local and verified |
| 6. the sampled loop (§3.2–§3.5) | `core/discrete/computer.py` (new, pure), `schedule.py`, `sampled_data.py`; `simulation/discrete/orchestrator.py` (from `hybrid_simulator.py`); `System.__mod__`; `plot_diagram`'s hybrid drawing; `mpc/controller.py` (the latch gone) | the pure tick, the hold state for slow blocks, `%` discretizing, one verb; the orchestrator a runner | the hybrid demos and the MPC baselines move once (R123's one-period delay; propagation within the tick) | high: the largest step; its oracle test (the orchestrator byte-identical to the tick) is the gate |
| 7. the sampled face of the judge | `planning/evaluator.py` (`compile` dispatch, `require_continuous` gone) | a `Computer` or a `SampledDataSystem` accepted; the stateless MPC refused | none for continuous laws | low once 6 lands |
| 8. the draws (R128) | `planning/problems.py`, `evaluator.py` (`realize` one shape, seeds by path), `interfaces/gymnasium.py` | blocks on ports; `"disturbances"` key gone; the frozen-noise defect fixed | the stochastic evaluations move once (live noise instead of a held draw) | low |
| 9. the seeded policy (R129) | `control/stochastic.py`, `learning/.../policy.py`, `collect.py`, the RL algorithms | the heads become the block; per-episode seeds | the RL numbers move once, re-pinned | medium: four algorithms |
| 10. the program (R130–R131) and the optimizer kernels (R132) | `optimization/` (the noun's `params`, `params_at`, the kernels, one adapter table, `n_iter`), `trajectory_optimization/` (collocation and shooting reading `params`; the parametric twins deleted), `mpc/controller.py` (`solve(params=)`, `shift`, `parameter_ports`) | one program noun; `solve(params=)`; the fixed-K scan | the MPC baselines byte-identical with `x0` only bound (the gate of PP-1) | medium |
| 11. the native MPC (join 2) | `trajectory_optimization/shooting.py` (with `params`), the MPC on a kernel | `n_iter` on the planner; the closed loop in one scan | new numbers (a new demo) | low: prototyped end to end |
| 12. Fields (R133) | `core/fields.py`, `analysis/lyapunov.py`, `planning/dynamic_programming/lqr.py` (`cost_to_go` a `QuadraticField`), the spatial fields | the noun and the three conversions | the RoA and DP baselines byte-identical | low |
| 13. hardening and the merge | `core/wiring.py` (`connect` refuses a rewire), the compile probe (R116), DESIGN §4–§6, the notebooks | — | the student-facing numbers re-pinned once | — |

**The order.** 1 → 2 → 3 (the objects lane, each byte-identical); 4 → 5 → 6 → 7 (the loop
lane: the algebra, then the score on the continuous loop, then the sampled loop, then its
face); 8 → 9 (the draws lane, after 2); 10 → 11 (the program lane, 10 can start after 2);
12 beside, after 2; 13 last. Each intended number change is its own commit with a closeness
report (R80). The thin end-to-end slice reviewer A asked for is step 7's gate: a digital PID,
`pid % 0.01 @ plant`, through `problem.compile`, with `jax.grad` of `J`; §10.2 shows it
already works in the pure-function form.

**The risks that matter.**
1. The hybrid step (6) is where a second implementation could creep back in. The rule: the
   orchestrator calls `SampledDataSystem.step`'s pure block functions and owns only the
   plant's adaptive integration and the I/O; its test is byte-identity with the tick at the
   samples on NumPy.
2. The slow-block cost on the base tick (an MPC computed and discarded on idle ticks). For
   the fixed-iteration MPC at 20 Hz on a 100 Hz base clock this is ×5. Acceptable for v0.3;
   the hyperperiod unrolling is the later optimization, with the same numbers.
3. `allow_int=True` in every gradient one-liner: documented once in DESIGN; `pe.program`
   never sees it.
4. The `Discretization`'s `params` property reads the source live
   (`analysis/discretization.py:119`); inside a `SampledDataSystem` the plant block's tree
   must be the source's, so the loop's `params["plant"]` edits the pendulum, as in a
   continuous loop. Keep the live view; the compile snapshot copies it as for any block.

## 10. What I verified

Five prototypes under `scratchpad/arch-F/`, run with `PYTHONPATH=/home/user/minilink
python3 <file> numpy|jax` (Python 3.13, NumPy 2.5.3, JAX 0.12.0, SciPy 1.18.1). Each uses
minilink's own `Pendulum.f`, `step_body`, `rk4`, `euler`, `child_seed` and
`standard_normal`; the loop and judge logic is the proposal's, written as pure functions
in the shape the library would carry (`xp` idiom; `scan` a Python loop on NumPy and
`lax.scan` on JAX; selects by `where`).

### 10.1 One score (`proto_judge.py`)

```text
[numpy] 1. Under forward Euler c_k = e^(−ρ t_k) g(x_k, u_k, t_k) dt exactly (VI and RL today)
   dt=0.05: max |c_k − e^(−ρ t_k) g dt| = 0.00e+00
   dt=0.01: max |c_k − e^(−ρ t_k) g dt| = 0.00e+00
[numpy] 2. Under RK4 the cost converges with the state; the trapezoid on samples lags
   dt=0.2   : J (cost as a state) = 2.08754528   J (trapezoid) = 2.03451425
   dt=0.1   : J (cost as a state) = 2.22499710   J (trapezoid) = 2.19927832
   dt=0.05  : J (cost as a state) = 2.28891655   J (trapezoid) = 2.27624448
   dt=0.025 : J (cost as a state) = 2.31922678   J (trapezoid) = 2.31293388
[numpy] 3. The substeps count for the cost as for the state; x_1 identical to the plant's own step
   substeps=1: J = 2.22499710   x_1 byte-identical: True
   substeps=2: J = 2.22494863   x_1 byte-identical: True
   substeps=4: J = 2.22494574   x_1 byte-identical: True
[jax]  (1–3 identical to the digits shown; Euler's difference 1.4e-17)
[jax] 4. jit, grad and vmap through the scan
   J = 2.28891374
   dJ/dKp = -0.206905, dJ/dKd = 0.177035, dJ/dm = 1.692704
   central difference dJ/dKp = -0.206905
   vmap over m ∈ [0.8, 1.2]: J = [1.93057 2.11475 2.28891 2.45345 2.60891]
   vmap over 4 starts: J = [2.28891 2.21709 2.16871 2.14219]
```

Reading: the change of `J` with `dt` in item 2 is the zero-order hold acting on the loop
(a coarser `dt` holds the law longer), not the quadrature; item 3 isolates the quadrature
(same hold, finer substeps): the integrated cost converges to 3e-6 at two substeps while
the trapezoid at `dt = 0.1` is 2.6e-2 away.

### 10.2 The sampled loop (`proto_sampled_loop.py`): a 100 Hz filter, a 20 Hz digital PID, the plant at 2 RK4 substeps per base tick

```text
[numpy] 1. Pure tick vs clock orchestrator: x identical True, u identical True; max |Δx| = 0.0e+00
   final angle 0.5476 rad (reference 1.0); the command changes every 5 ticks: u[0:12] = [6. 6. 6. 6. 6. 6. 6. 6. 6. 6. 5.819 5.819]
[numpy] 2. J = 2.735766
[jax] 1. Pure tick vs clock orchestrator: max |Δx| = 8.3e-16   (the orchestrator ran on NumPy)
[jax] 3. grad through the selects: dJ/dKp = -0.344804 (central difference -0.344804); dJ/dalpha = -0.176375; dJ/dm = 1.874799
[jax] 4. vmap over 5 starts: J = [2.7358 2.8254 3.0789 3.4692 3.9632]
[jax] 5. one jitted closed-loop run of 300 base ticks (2 RK4 substeps each): 3.24 ms (first call 5 ms)
```

### 10.3 The parametric program, the kernel, the fixed-iteration MPC, its closed loop (`proto_mpc_kernel.py`, JAX)

```text
1. One program, several starts, no rebuild: J(z=0; x0) for three x0
   x0=[0.0, 0.0]: J = 4.8000 · x0=[1.0, 0.0]: J = 20.7214 · x0=[3.0, 0.0]: J = 168.3849
2. The solve as a rollout: K iterations of two kernels on the program at x0 = 0
   projected gd : J(z_60) = 4.4263, u in [0.01, 0.23]
   adam         : J(z_60) = 2.5109, u in [-0.51, 5.00]
3. The closed loop, 60 ticks × 10 Adam iterations: J = 0.4381, final angle 0.288 rad (target 0.400), 0.81 s
4. jitted closed loop: first call 0.96 s, then 104.7 ms
5. grad of the closed-loop J through 60 ticks × 10 Adam steps × 20-sample shooting:
   dJ/dQ11 = -0.09039 (central difference -0.09026); dJ/dP11 = -0.00527; dJ/d(model m) = -0.0812
6. vmap over 4 starts: J = [0.438 0.391 0.48 0.689] (1.08 s with compile)
7. tuning the MPC's own Q and P by the closed-loop J, projected gd(η=0.3): start J = 0.4381
   step 0: J = 0.4306 · step 1: J = 0.4262 · step 2: J = 0.4221 · step 3: J = 0.4208 · step 4: J = 0.4102
   step 5: J = 0.4371 (a too-long step; a line search's job, the optimizer's, not the pipeline's)
8. Optimizer(program, method='l-bfgs').solve(params={x0: [0.0, 0.0]}): J* = 2.5064, nit = 6, u_0 = 3.663
   Optimizer(program, method='l-bfgs').solve(params={x0: [0.5, 0.0]}): J* = 2.5011, nit = 6, u_0 = 2.228
```

The NumPy face of the same script gives the same `J = 0.4381` in 69 s (the kernel's
gradient by central differences there): the pure functions are one body on both backends.
Two honest notes: (a) the first task I tried, holding the pendulum at 1 rad, is infeasible
under the 5 N·m bound (τ_ss = 8.3 N·m), which the fixed-iteration MPC showed as a sluggish
loop, so the set point moved to 0.4 rad; (b) a raw gradient step or bare Adam on the three
weights overshoots (`r` went negative in one step), which is why `pe.program(wrt, bounds=)`
must carry the weights' box and why a tuning demo wants a line search.

### 10.4 The draws and the seeded policy (`proto_draws_policy.py`)

```text
1. realize(key=7, n_trials=4): m draws [1.1687 1.0109 1.0219 0.9181], controller seeds [258273906 1448622918 9156651 329912624]
   adding a second random parameter and a second block leaves every other stream unchanged: True
2. the policy's draw is held over Δ = 0.1: u(t) at t = 0, .05, .1, .15, .2 = [6.0473 6.0473 5.6952 5.6952 5.8485]
   same seed twice: True; log_prob(u(0.1)) = -0.2310; entropy = 0.2150
3. Monte Carlo over 32 trials (starts, masses, policy seeds from key 3): mean J = 3.54857, std = 0.2844   (numpy and jax alike)
[jax] 4a. jax.grad over a tree with an integer seed leaf: refused (grad requires real- or complex-valued inputs…)
[jax] 4. grad of the mean J through the vmap over seeds: dJ/dKp = -0.46545, dJ/dlog_std = -0.00526 (central difference -0.00526);
         the integer seed leaf's gradient dtype: float0
[jax] 5. 32 seeded trials of 150 RK4 steps in one vmap: 8.17 ms
```

### 10.5 Facts checked in the code

- `score_run` integrates the trapezoid on the samples (`planning/evaluation.py:187–195`,
  `core/costs.py:95–124`); value iteration charges `g·dt` (`dp.py:286–315`); the RL reward is
  `−g dt` (`environment.py:159`): the three quadratures of core-architecture §6.
- `Computer.tick` reads the committed buffer of the previous tick and mutates `k`, `x` and
  the buffers (`simulation/discrete/computer.py:114–171`); `HybridSimulator.run` re-implements
  the hold and the substeps (`hybrid_simulator.py:145–300`).
- `Discretization.step` is `step_body` on the source's `f` with `t_k = k·dt`
  (`analysis/discretization.py:130–146`), the precedent for `SampledDataSystem`'s plant block.
- `MPCStatefulController.step` solves through the latch and `del params`
  (`mpc/controller.py:504–523`); its ports call `warm_tick_solve` (the hidden cache R123 removes).
- `ProblemEvaluator.realize` draws the plant's laws on the backend's RNG and the block
  seeds by `child_seed` (`evaluator.py:324–353`); `trial_key` makes the two backends disagree
  (KNOWN_ISSUES).
- `StochasticPlanningProblem.realize` returns `(x0s, {"params", "disturbances"})`
  (`problems.py:638–681`), the second shape R128 retires.
- `Adam.init / update` (`optimizers/first_order.py:155–193`) is already the kernel shape.
- `ParametricMathematicalProgram` varies `x0` only (`parametric_program.py`), and collocation
  closes over the plant's nominal parameters (`direct_collocation.py`).

### 10.6 What is still untested

- The hold state of a slow block **inside a `StepDiagramSystem`'s compiled plan** (the
  prototype carries it by hand in a dict): the compiler must add one state slot per output
  of a `d > 1` block. Mechanical, not verified.
- The orchestrator with an adaptive plant between samples (`solver="scipy"`): not prototyped
  (SciPy arrived mid-session); the restart-per-input-change rule is the same as today's
  `HybridSimulator` loop with `solve_ivp` in place of the RK4 segment.
- The DP sweep reading `transition` on JAX (`dp_jax.py`): the backup's `c_k` is a table built
  by `vmap` of `transition`; under Euler it is today's table (verified in value), the wiring
  is not.
- `jax.checkpoint` per period for long horizons under `grad`: not needed at 300 ticks; the
  memory argument is standard.
- A `while_loop` to tolerance on the kernel (the non-fixed-K solve): not run; it is `jit`-
  and `vmap`-safe by construction and refuses reverse-mode `grad`, as native-optimizers §3.2 says.

## 11. Open questions I could not settle

1. **`compute_rollout` or one verb** (§3.2). I recommend one verb; R127's two verbs cost a
   second number on one object. Your call, since both the hybrid demos and tutorial 06 change.
2. **The judge's `dt` against the sampled loop's base clock.** I require the evaluator's `dt`
   to equal `dt_base` (or divide it, refining the plant's substeps). The alternative, scoring
   only every `d`th tick at a controller's period, would hide the plant between samples; I
   recommend refusing it.
3. **The slow block on idle ticks.** Base tick with selects computes an MPC on every base
   tick and discards it (×d). I accept it for v0.3 and name the hyperperiod unrolling as the
   v0.4 optimization. If a benchmark on the racecar MPC says otherwise, the nested-scan
   layout for the two-clock case is the fallback, same numbers.
4. **`terminal=True` on `Field.as_cost`** versus a fourth conversion `as_terminal_cost`. I
   prefer one function with a keyword (RULES 2.7); the maintainer names the surface.
5. **The MPC's inner tree keys.** I renamed the planner's `plant` to `model` inside the
   controller's tree so a loop never holds two `plant` keys (`loop.params["controller"]["model"]`).
   R26 ruled `plant` as the key of a problem's tree; this is the same key seen from inside a
   block, and I recommend `model` there, with the rule written in DESIGN §4.
6. **`allow_int=True` in the gradient one-liner** (§3.6). The alternative is a float seed
   (bits reinterpreted), which I do not recommend: an integer leaf says what it is.
7. **When the RL heads move to `GaussianPolicy`** (with the block or in v0.4): with the block,
   so the RL numbers move once, provided the four algorithms' tests re-pin in the same
   change; otherwise v0.4.
8. **The name `SampledDataSystem`** (§3.5): textbook and role-named; `SampledLoop` is the
   shorter alternative if a `System` suffix reads too formal in `print(loop)`.
