# The final core architecture and API of minilink: proposal A, the textbook purist lens

**Date:** 2026-10-27.
**Status:** an independent proposal, not a ruling. Written by one of three agents working in
parallel; the maintainer compares and decides.
**Commit reviewed:** `082103cbf4c62f627ffd416f9b66eaa4aae5998e` (`dev-v0.3`).
**Lens:** the fewest nouns and verbs; every object reads like the textbook (Åström–Murray,
Franklin–Powell, Bertsekas, Nocedal–Wright); reuse before a new class; every public name earns
its place.

How to read this: §2 is the aim in one screen, §3 the six places where I depart from the
preliminary choices of 2026-10-27 (R124–R140), §4 the user's view on one page, §5 the eight
pipelines as complete user scripts, §6 the API object by object, §7 the architecture under it
with the JAX argument, §8 the coverage table, §9 the migration, §10 what I ran, §11 what I could
not settle. Step ids from the workboard appear in parentheses as pointers only.

---

## 2. The aim in one screen

The maintainer wants one toolbox where a student's first script (a pendulum, a PID, a plot) and
a researcher's last one (a stochastic sampled loop with a seeded policy, vmapped over seeds and
tuned by gradient) are written in the same words on the same objects, and where every number
between the two is reachable by `jax.grad`. The spine is fixed (CONSTITUTION §1.1, R119): a
`System`, compiled, rolled out to a `Trajectory`, scored by a `PlanningProblem`'s cost into an
`Evaluation`, lowered to a `MathematicalProgram`, solved by an `Optimizer`. v0.3 is "the complete
core": the spine made true for every object (pure, parametric, deterministic given a seed), for
the sampled loop (a computer on a continuous plant), for stochastic problems (starts,
parameters, disturbances, policy noise), and for the two ends of the solver boundary (a program
that rebinds without recompiling; a native solver that is itself a rollout).

The design question under all of it is one of *closure*: which lowerings happen where, so that
one `J` is reported by every tool, one scan runs every stepping loop, and one tree of parameters
reaches every number. My answer in one sentence: **the continuous objects stay the user's
vocabulary; one internal lowering, at `problem.compile`, turns a problem and a policy into a
single step loop whose state carries the running cost; every stepping tool (the evaluator, value
iteration, the RL environment, the MPC's internal rollout) is that scan read by a different
reader.** Everything else in this proposal follows from making that lowering pure, traceable
and the only one.

What the user ends up writing (the eight scripts of §5 are the proof): `plant`, `pid`,
`pid @ plant`, `pid % 0.01 @ plant`, `PlanningProblem(plant, cost=..., tf=..., x0=...)`,
`problem.compile(loop).evaluate()`, `pe.program(wrt=[...])`, `Optimizer(program).solve()`,
`ModelPredictiveController(planner, sample_period=...)`, `QuadraticField(P)`,
`GaussianPolicy(mean, log_std, sample_period=...)`. Eleven names carry the whole course.

---

## 3. Adjustments to the vision

Each adjustment names the preliminary choice it revises, its reason and its cost. Everything
not listed here is taken as ruled.

### A1. The start is a leaf of the block, in the run's tree: `plant.x0`, not `x_start` (revises R124's last clause)

R124 gives every stateful block an `x0` leaf beside its parameters in the evaluator's tree
(`controller.x0`, `observer.x0`) but keeps the plant's start apart, as the top-level leaf
`x_start`. I apply the one rule to the plant too: in the run's tree `pe.params`, the plant's
start is `pe.params["plant"]["x0"]`, written by the problem's start (`problem.x0`, a point, a
set's representative point or a law's mean). Then "the start of block *b* is `b.x0`" holds with
no exception, trajectory optimization's free start is `wrt=["plant.x0"]`, and the MPC rebinds
the measured state as `{"plant": {"x0": y}}`, the same path as everywhere else.

- **Reason:** one rule instead of a rule and an exception; every path in the tree is
  `<block>.<leaf>` (R134's separator rule) with no top-level leaf that belongs to a block.
- **Cost:** the `"x_start"` key disappears from `pe.params`, `wrt` lists and the parametric
  program (the robust-tuning and MPC demos, their tests). It is one sweep, in the same change
  as R124, which touches those files anyway. The block's own `params` (what `f` reads) does not
  gain `x0`: the evaluator strips the `x0` leaves before handing a block its tree (§7.4).

### A2. One problem noun: `PlanningProblem` takes a point, a set or a law as its start; `StochasticPlanningProblem` goes (revises R128's "both problems" wording, DESIGN §6)

R124 already makes the problem's start polymorphic: a point or a `SingletonSet` (deterministic),
a `Distribution` (stochastic), a `Set` (trajectory optimization). Once the start can be a law,
the two classes differ only by two keywords (`params=` for the parameter laws and `criterion=`)
and one flag (`is_stochastic`). Bertsekas has one "problem" whose disturbance may be
degenerate; so does this proposal: `PlanningProblem(plant, cost=, tf=, x0=Gaussian(...),
params={"plant.m": Uniform(0.9, 1.1)}, disturbances={"d": WhiteNoise(...)})`. `is_stochastic` is
true when any law is declared; `realize(key, n_trials)` of a deterministic problem returns the
one start and the nominal tree, repeated; `criterion` defaults to the expectation and is
irrelevant when nothing is random.

- **Reason:** fewer nouns, and the two classes already share every verb (`compile`, `realize`,
  `evaluate`, `params`); every planner "takes the stochastic one" (DESIGN §6), so the type never
  carried information a tool read. The RL notebooks then state their problem with the same
  class as the LQR notebooks, which is the course's own lesson.
- **Cost:** a student-facing rename (the RL demos and the GRO860 RL notebooks write
  `StochasticPlanningProblem`), one sweep with every caller, no alias (R82), timed with the
  notebook move the merge already requires (R47). CONSTITUTION §2's sentence "the stochastic
  sibling adds distributions over starts and parameters" becomes "a problem may state a law on
  its start and its parameters; disturbances are source blocks". If the maintainer keeps the
  sibling, every script in §5 still reads with `StochasticPlanningProblem` in place of
  `PlanningProblem` where a law appears; nothing else moves.

### A3. The one score is an internal lowering with one rule: the cost is a state of the step (settles the open investigation; no new noun)

Answering QUESTIONS' investigation (core-architecture §6, differentiable-problem §9.4):

1. **No discrete versions of sets and costs.** They keep `t`; the lowering binds
   `t_k = t₀ + k·dt`. A natively discrete cost is written as `g(x, u, t)` with `t = k·dt`.
2. **The per-step cost is integrated with the state, by the same scheme, reset each period:**
   `ĉ_k = ∫_{t_k}^{t_{k+1}} e^{−ρ(τ−t_k)} g(x(τ), u_k, τ) dτ`, computed by stepping the augmented
   plant `x̃ = [x; c]`, `f̃ = [f; e^{−ρ(τ−t_k)} g]`, with the cost slot zeroed at the start of
   each period. Under forward Euler this is exactly `g(x_k, u_k, t_k)·dt`, today's value
   iteration and RL reward, so their numbers do not move; the evaluator's trapezoid and
   shooting's trapezoid move once (an intended change, R80).
3. **The score is `J = Σ_{k<N} γᵏ ĉ_k + γᴺ h(x_N, t_N)`, `γ = e^{−ρ dt}`**, the discount
   outside the period so the RL reward is `−ĉ_k` with the per-step factor `γ` and nothing is
   discounted twice.
4. **One failure rule, `score_run`'s:** run to the horizon, `failed = not finite or
   min margin < 0`, `reached = Xf.margin(x_N) ≥ 0`, the price added when the switch is on. The
   RL collector's early stop is a training choice inside the collector; it reports `evaluate`.
   Value iteration prices leaving its grid with the same price.
5. **The hold is zero order,** the computer's own semantics; a continuous policy under
   `problem.compile(policy, dt)` is sampled at `dt` with its command held, which is what
   `evaluate` already does.
6. **The object is internal:** a `DiscreteProblem` record built by `problem.compile`, holding
   the step loop, `ĉ`, the margins at the index, `N`, `γ` and the start. Not exported in v0.3.
   A `PlanningProblem` whose plant is already a `StepSystem` lowers trivially (`dt` from the
   block's `sample_period`, else one tick is one second).
7. **A continuous system's evaluation is done exactly as a `StepSystem` on its `dt`:** the
   evaluator compiles the loop as a step diagram (`discretize(plant_with_cost, dt, substeps)`
   closed on the policy) and scans it; the trapezoid becomes the plain sum of point 3.
8. **Margins between samples:** at the samples in v0.3; a `substeps` margin is a later
   refinement that touches only the lowering.

- **Reason:** the textbook's own device (Bryson–Ho augment the state with the cost); exact for
  the scheme; one owner for the rule; VI and RL numbers preserved.
- **Cost:** `CostFunction.integrate` (the trapezoid) becomes a reporting verb on a
  `Trajectory`, no longer the evaluator's rule; shooting's reported score is lowered to the
  same rule. Two intended number changes, re-pinned once each.

### A4. `computer @ plant` is named `SampledDataSystem` now, not at the freeze (revises R127's "the name stays `HybridDiagram` for now")

- **Reason:** the class is rebuilt from scratch (a `StepDiagramSystem` subclass in place of a
  plain record holding a `Computer` and a `DiagramSystem`); every caller of `HybridDiagram`
  is touched by that rebuild. The textbook word is "sampled-data system" (Åström–Wittenmark;
  Franklin–Powell–Workman), and "hybrid" promises event-driven dynamics the class does not
  have.
- **Cost:** none beyond the rebuild's own sweep. The drawing's two-cluster figure keeps its
  file (`graphical/diagrams/hybrid_topology.py` is internal).

### A5. `%` returns a `Computer` that is a step diagram (`core/`), and the clock runner is a function of the simulation band (refines R127; closes the R100 exception)

Today `block % dt` builds a mutable, NumPy-only runtime in `simulation/discrete/computer.py`,
reached from `core` through a lazy import written up as an exception (R100). In this design
`Computer(StepDiagramSystem)` lives in `core/discrete/computer.py`: its blocks, its firing
divisors and its held outputs (as state) are the pure model; `%` builds it with no import
across the dependency law. The runner that ticks it on a wall clock
(`simulation/discrete/clock_runner.py`, today's `HybridSimulator` and `Computer.tick` folded
into one function over pure blocks) is a tool.

- **Reason:** the dependency law holds with no written exception; the computer is a `System`
  (CONSTITUTION invariant 1's first sentence) and composes with `@` (R126).
- **Cost:** `Computer.tick`, `reset`, `compile` as mutable methods go; the realtime simulator
  calls the runner function instead.

### A6. The planner interface is four verbs, two of them pure (gives R135 its verbs)

`compile()` (build the program once), `plan(z0, params) -> z` (pure arrays: the solve the MPC's
step calls, traceable when the planner's method is native), `shift(z) -> z` (the warm start one
control period later, the planner's knowledge), and `solve(*, params=None, z0=None) ->
PlanningSolution` (the user's verb, a host wrapper around `plan` that unpacks the plan into a
`Trajectory` and a record). `ModelPredictiveController` reads `compile`, `plan`, `shift` and
`planner.unpack(z) -> (u_seq, x_seq)`; it never probes attributes.

- **Reason:** the smallest set that serves trajectory optimization on a host solver, the native
  fixed-iteration solver and MPPI; the pure verb is what makes the closed loop one scan.
- **Cost:** `solve_trajectory_from`, `compile_parametric_program` and the `Command` record
  of the MPC go; `TrajectoryOptimizationRecord` gains `z` (as mppi.md §6 asked).

### Taken as ruled, with one note each

- **R125** (within-tick propagation; delay is a block): taken. The `Computer`'s pure step
  computes the tick's signals in the compile-time topological order; a slow block's held output
  is a state the computer adds (§6.2).
- **R126** (the operators on step systems): taken; `@` on a `Computer` and a continuous plant
  is the one mixed case, and it returns the `SampledDataSystem`.
- **R129** (`GaussianPolicy`): taken; §6.11 fixes what `mean` and `log_std` are.
- **R130–R132**: taken; §6.9–§6.10 give the signatures, and the kernel is bound to its program
  at construction so `step(state, params)` has the R132 shape.
- **R133–R134, R138**: taken. `as_cost` gains one structural flag, `terminal=`, so a value
  function serves as the terminal cost (R133's second conversion) without a second verb.

---

## 4. The user's view: the main objects and verbs on one page

Every object below obeys one contract (§7.2): its mathematics is a method of its specific
arguments and `params=None`; its `params` is a dict of float or array leaves, a composite's a
dict of its parts' dicts by name, the seed the one integer leaf; it is deterministic given its
parameters.

| Object | In the textbook | Verbs (the math first, then the tools) | Its `params` |
| --- | --- | --- | --- |
| `System` | a block of the diagram, `y = h(x, u, t)` | `h`; `+`, `>>`, `@`, `%`; `compile`, `compute_trajectory`, `plot_*`, `animate`, `realize(key)` | its own numbers |
| `DynamicSystem(System)` | `ẋ = f(x, u, t)` | `f`; `linearize`, `discretize` | idem |
| `StepSystem(System)` | `x_{k+1} = Φ(x_k, u_k, k)`, with `sample_period` giving `t_k = k·Δ` | `step`, `h`; `compute_rollout`, `jacobian` | idem |
| `DiagramSystem`, `StepDiagramSystem` | a block diagram, continuous or sampled | the same as a leaf; `connect`, `expose`, `trajectory_of(block)` | `{block_id: block.params}`; `x0` a view of its blocks |
| `discretize(sys, dt, solver=, substeps=)` | the plant one period later, `u` held | returns a `StepSystem` keeping its continuous source | the source's (a view) |
| `Computer(StepDiagramSystem)`, from `block % dt` or `block % Computer.schedule(dt, fire=)` | the digital controller: blocks on a base clock, slow ones firing every `d` ticks, held outputs | the step diagram's; `@ plant` | one block: that block's; several: by id |
| `SampledDataSystem(StepDiagramSystem)`, from `computer @ plant` | the sampled-data loop at the base clock, ZOH into the plant | `compute_rollout` (the ticks, pure), `compute_trajectory` (dense, adaptive plant), `plot_diagram` (two clusters) | `{"controller": …, "plant": …}` |
| `Trajectory` | the sampled `(t, x, u)` | `of(role)`, `resample`, `with_cost`, `plot`, `save`; a pytree | — |
| `Set`, `InputSet` | `z ∈ Z(t)` as `margin(z, t) ≥ 0` | `margin`, `contains`, `sample(key)`, `bounding_box`, `&`, `as_cost` | the set's numbers (`{}` when none) |
| `CostFunction` | `g(x, u, t)`, `h(x, t)` | `g`, `h`, `+`, `*`; `integrate(traj)` for reporting | `Q`, `R`, … by term |
| `Distribution` | a law over a vector | `sample(key, n, params=None)`, `mean`, `support` | `mean`, `std`/`cov`, … |
| `Field` | `V(x, u, t)`: a Lyapunov function, a cost-to-go, a barrier | `value`, `gradient`, `as_set`, `as_input_set`, `as_cost(terminal=)` | `P`, `center`, … |
| sources: `Step`, `Sine`, `WhiteNoise`, `TrajectorySource` | the signal generators of the diagram | `h`; a random one draws by `(seed, ⌊t/Δ⌋)` | amplitudes, `seed`, `psd`, `sample_period`, a table |
| `GaussianPolicy(mean, log_std, *, sample_period, seed=None)` | `u = μ(x) + σ(x) ε_k` | `h`, `log_prob(x, u)`, `entropy(x)` | `{"mean": …, "log_std": …, "seed": s}` |
| `PlanningProblem(sys, *, cost, tf, x0, X, U, Xf, params=, disturbances=, discount_rate, criterion)` | the optimal-control problem; its start a point, a set or a law; parameter laws; sources on the plant's ports | `compile(policy, dt)`, `realize(key, n_trials)`, `replace`, `cost_bound` | `{"plant", "cost", "X", "U", "Xf", "d", …, "discount_rate"}` (nominal values) |
| `ProblemEvaluator`, from `problem.compile` | the closed loop lowered to one step loop with the cost in its state | `trajectory(params, key)`, `evaluate(params, key, n_trials) -> Evaluation`, `program(wrt, …)`, `pack`/`unpack` | the run's tree: each block's leaves and `x0`, the problem's parts |
| `Evaluation` | the score and the facts | `objective`, `mean`, `std`, `worst`, `cvar`, the rates; a pytree | — |
| `MathematicalProgram(n_z, J, h, g, lower, upper, params=None)` | `min J(z; p)` s.t. `h = 0`, `g ≥ 0`, bounds | `J`, `h`, `g` each `(z, params=None)`; `params_at(z)` on an export | the problem's tree minus `wrt` |
| `Optimizer(program, z0, *, method, backend)` | the solver boundary | `solve(params=None, z0=None) -> OptimizationResult` | — |
| kernels: `GradientDescent`, `Adam` (`optimization/kernels.py`) | the update law `z_{k+1} = S(z_k)` | `init(z0, params)`, `step(state, params)`; `as_step_system(kernel)` | `step_size`, moments' rates |
| `Planner(problem)` | a method that returns a law or a plan | `compile()`, `plan(z0, params)`, `shift(z)`, `solve(**) -> PlanningSolution` | — |
| `PlanningSolution` | the law and the evidence | `policy`, `trajectory`, `evaluation`, `cost_to_go` (a `Field`), `solver`, `success`; the generic plots | — |
| `ModelPredictiveController(planner, *, sample_period, warm_start=True, parameter_ports=())` | the receding-horizon law, a `StepSystem` whose state is the last plan | `step`, `h`; `@ plant` | the planner's tree (`plant`, `cost`, sets), the kernel's, `x0` (= `z0`) |

Eleven names carry a course: `Pendulum`, `PID`, `@`, `%`, `PlanningProblem`, `compile`,
`evaluate`, `program`, `Optimizer`, `ModelPredictiveController`, `QuadraticField`.

---

## 5. The pipelines, as user scripts

Every script is complete in the final API. "Prints" shows the shape of the output; a number
marked † was produced by the prototype of §10, the others are illustrative. The `params` tree
shown is what `print(obj.params)` gives, keys only where the values are long.

### 5.1 Lesson 1: a pendulum, a PID, simulate, plot

```python
from minilink import PID, Pendulum, Step

plant = Pendulum()                                  # (m l²) θ̈ = τ − m g l sin θ − d θ̇
pid = PID(Kp=20.0, Ki=5.0, Kd=2.0, tau=0.05)        # u = Kp e + Ki ∫e − Kd d/dt(y filtered), e = r − y
loop = Step(final_value=[0.5], step_time=1.0) >> pid @ plant

traj = loop.compute_trajectory(tf=8.0)             # adaptive SciPy by default, 10001 points
loop.plot_diagram()
loop.plot_trajectory()
loop.animate()

print(plant.params)
print(loop.params.keys())
print(loop.x0)
```

Prints:
```text
{'m': 1.0, 'l': 1.0, 'gravity': 9.81, 'd': 0.0}
dict_keys(['Step', 'controller', 'plant'])
[0. 0. 0. 0.]      ← a view: the PID's two states, then the plant's; loop.x0 = v writes through
```
Plots: the diagram (three blocks, `Step >> PID @ Pendulum`), the time signals (`controller:u`,
`plant:x`, every block output recorded), the animation. What changes from today: nothing in
this script (`loop.x0` becomes a view, same numbers). `params`:
`{"Step": {"final_value", "step_time"}, "controller": {"Kp", "Ki", "Kd", "tau", "u_min", "u_max", …}, "plant": {"m", "l", "gravity", "d"}}`.

### 5.2 Classical analysis: linearize, poles, Bode, margins, the sampled loop's poles

```python
import numpy as np
from minilink import PID, Pendulum, bode, linearize, margins, pzmap, discretize

plant = Pendulum()
C = PID(Kp=20.0, Ki=5.0, Kd=2.0, tau=0.05)
x_bar = np.array([0.5, 0.0])                        # hold the arm at 0.5 rad: u_bar = m g l sin(0.5)

G = linearize(plant, x_bar)                         # LTISystem(A, B, C, D) about (x_bar, u_bar)
print(G)
bode(C >> G)                                        # the loop gain C(s) G(s)
print(margins(C >> G))
_, poles, _ = pzmap(C @ plant, x_bar)               # closed-loop poles of the linearized loop
print(poles)

# The same loop on a computer: a sampled-data system at 100 Hz, its poles against the unit circle
loop_d = C % 0.01 @ plant                           # SampledDataSystem; the PID discretized exactly (it is LTI), the plant by RK4
Phi = loop_d.jacobian("step", "x", x_bar=loop_d.x0) # ∂Φ/∂x at the fixed point
print(np.abs(np.linalg.eigvals(Phi)).max() < 1.0)   # stable iff every |λ| < 1
Gz = discretize(G, 0.01, solver="exact")            # G(z): the matrix exponential
```

Prints: the state-space matrices at the operating point, the gain and phase margins with their
crossover frequencies, four closed-loop poles in the left half-plane, `True`. Plots: the Bode
magnitude and phase of `C(s)G(s)`, the pole-zero map. What changes from today: `C % 0.01 @ plant`
returns a step diagram so `jacobian("step", "x")` applies (today `%` returns a runtime that
has no Jacobian); everything else is today's API (`analysis/frequency.py:93`, `:123`, `:182`).

### 5.3 State space: LQR and an observer (LQG), Monte Carlo over noise and parameter draws with one key

```python
import numpy as np
from minilink import (Gaussian, NoisyCartPole, PlanningProblem, QuadraticCost, Uniform,
                      WhiteNoise, closed_loop, kalman, lqr)

plant = NoisyCartPole()                             # ports u, d (force disturbance), n (measurement noise)
X_UP = np.array([0.0, np.pi, 0.0, 0.0])
Q, R = np.diag([1.0, 10.0, 1.0, 1.0]), np.array([[0.1]])
W, V = 1e-2, 1e-4                                   # intensities: the same numbers the simulation draws from

B_w = plant.jacobian("f", "d", X_UP)
gain = lqr(plant, Q, R, x_bar=X_UP)                 # u = u_bar − K (x̂ − x_bar)
observer = kalman(plant, W * B_w @ B_w.T, V * np.eye(4), x_bar=X_UP)
loop = closed_loop(gain, plant, observer=observer)  # d and n left open: the problem drives them

problem = PlanningProblem(
    plant, cost=QuadraticCost.from_system(plant, Q, R, xbar=X_UP), tf=8.0,
    x0=Gaussian(X_UP + [-0.5, 0.2, 0.0, 0.0], std=[0.1, 0.05, 0.0, 0.0]),   # a law on the start
    params={"plant.m_pole": Uniform(0.8, 1.2)},                            # a law on a parameter
    disturbances={"d": WhiteNoise(1, psd=W, sample_period=0.01),           # sources on the ports
                  "n": WhiteNoise(4, psd=V, sample_period=0.01)},
)
pe = problem.compile(loop, dt=0.01)                 # the loop, its sources wired on d and n, lowered once
report = pe.evaluate(n_trials=200, key=1)           # one key → 200 starts, 200 masses, 200 seeds per source
print(report)
print(pe.params.keys(), pe.params["d"]["seed"], pe.params["observer"]["x0"])

for i in range(3):                                  # three realizations replayed, the plant's run
    pe.trajectory(key=i).of("plant").plot()
x0s, draws = problem.realize(key=1, n_trials=200)   # the same draws on NumPy and JAX
```

Prints:
```text
Evaluation: 200 trials   objective (mean J) 12.41   std 1.9   worst 19.0   feasible 100%   reached —
dict_keys(['controller', 'observer', 'plant', 'd', 'n', 'cost', 'X', 'U', 'Xf', 'discount_rate'])
0                                                 ← the nominal seed; each trial realizes its own
[0. 0. 0. 0.]                                     ← the observer's start x̂₀, a tunable leaf
```
Plots: three Monte Carlo plant runs. The check of the definition of done (core-architecture
§11.1): `report.std` within the covariance prediction's sampling error, and
`problem.realize(1, 200)` byte-identical across backends (the cipher draws, §7.5). The `params`
tree: `{"controller": {"K", "xbar", "ubar"}, "observer": {"L", "A", "B", "C", "xbar", "ubar", "x0"},
"plant": {"m_cart", "m_pole", "l", "gravity", "x0"}, "d": {"seed", "psd", "sample_period"},
"n": {…}, "cost": {"Q", "R", "S", "xbar", "ubar"}, "X": {}, "U": {}, "Xf": {}, "discount_rate": 0.0}`.
What changes from today: `x0=Gaussian(...)` and `params=` on the one problem class (A2);
sources on ports instead of held laws (R128); `plant.x0` in the tree (A1); the observer's `x0`
leaf (R124).

### 5.4 The digital loop: `pid % dt @ plant`, two runners, one `J`, the gradient of `J` with respect to the gains

```python
import jax
import numpy as np
from minilink import PID, Pendulum, PlanningProblem, QuadraticCost

plant = Pendulum()
pid = PID(Kp=20.0, Ki=5.0, Kd=2.0, tau=0.05, ports="reference")
loop = pid % 0.01 @ plant                            # SampledDataSystem at 100 Hz: the PID stepped, the plant held
loop.inputs["r"].nominal_value = np.array([0.5])

run = loop.compute_rollout(tf=3.0)                   # the ticks: Trajectory with t = k·0.01, pure, traceable
dense = loop.compute_trajectory(tf=3.0)              # the clock runner, adaptive plant between ticks, dense
print(np.abs(run.of("plant").x - dense.resample(run.t).of("plant").x).max())   # integration error only

cost = QuadraticCost.from_system(plant, Q=np.eye(2), R=0.1 * np.eye(1), xbar=[0.5, 0.0])
problem = PlanningProblem(plant, cost=cost, tf=3.0, x0=[0.0, 0.0], X=plant.state.box)
pe = problem.compile(loop)                           # dt = the computer's 0.01; the plant block carries the cost state
report = pe.evaluate()                               # one scan over 300 ticks
print(report.objective, report.feasibility_margin)

dJ = jax.grad(lambda p: pe.evaluate(p).objective)(pe.params)
print(dJ["controller"]["Kp"], dJ["controller"]["Kd"], dJ["plant"]["m"])
print(np.isclose(pe.evaluate().objective, cost.integrate(run.of("plant"))[0], rtol=1e-3))  # the trapezoid, for the student
```

Prints (†: the prototype's numbers for this loop, §10):
```text
3.1e-06                 ← the two runners agree to the plant integrator's tolerance
6.2378†  8.871†
[[-0.31]] [[0.07]] 1.9  ← the gradient reaches the gains and the plant's mass alike
True
```
Plots: none; `run.plot()` would show `controller:u` as a staircase held over each tick. The
`params` tree is `{"controller": {"Kp", "Ki", "Kd", "tau", …, "x0"}, "plant": {"m", "l",
"gravity", "d", "x0"}, "r": [0.5], "cost": {…}, "X": {}, "U": {}, "discount_rate"}`: a
one-block computer is transparent, so the PID's leaves sit directly under `controller`.
`pe.program(wrt=["controller.Kp", "controller.Kd"])` then exports the tuning program exactly as
in `examples/demos/compile/pid_autotuning_jax.py` §2 today. What changes: `%` and `@` return a
step diagram (R126, R127, A4, A5); `problem.compile` accepts it (the sampled face); the score
is the augmented-state sum (A3).

### 5.5 Trajectory optimization and MPC: one program compiled once, rebound with a new start, plant and cost; the MPC in the loop

```python
import numpy as np
from minilink import (BallSet, ModelPredictiveController, Optimizer, Pendulum, PlanningProblem,
                      QuadraticCost, TrajectoryOptimizationPlanner)

plant = Pendulum()
goal = np.array([np.pi, 0.0])
cost = QuadraticCost.from_system(plant, Q=np.diag([1.0, 0.1]), R=0.01 * np.eye(1), xbar=goal)
problem = PlanningProblem(plant, cost=cost, tf=3.0, x0=[0.0, 0.0], Xf=BallSet(goal, 0.05), U=plant.input_box)

planner = TrajectoryOptimizationPlanner(problem, n_steps=40, transcription="direct_collocation",
                                        method="ipopt", backend="jax")
program = planner.compile()                          # the MathematicalProgram, once: J(z, params), h(z, params), g(z, params)
print(program)                                       # n_z = 123, 80 equality rows, params: plant, cost, U, Xf, discount_rate
sol = planner.solve()                                # Optimizer(program).solve(params=problem.params) inside
print(sol.solver)                                    # TrajectoryOptimizationRecord: success, iterations, residuals, z
sol.plot_trajectory()
(sol.policy >> plant).compute_trajectory(tf=3.0)     # the plan replayed open loop

# Rebind without recompiling: a new start, a heavier plant, a new weight; one solve each
sol_b = planner.solve(params={"plant": {"x0": [0.3, 0.0], "m": 1.3}, "cost": {"Q": np.diag([2.0, 0.1])}})
assert sol_b.solver.success
print(program.params_at(sol_b.solver.z)["plant"]["m"])          # 1.3: the full tree of that solve

# The receding-horizon law on the same planner; the plant's mass fed live from a port
mpc = ModelPredictiveController(planner, sample_period=0.05, parameter_ports=["plant.m"])
loop = mpc @ plant                                   # SampledDataSystem at 20 Hz: the MPC's state is its last plan
traj = loop.compute_trajectory(tf=5.0)               # host Ipopt per tick: the clock runner, NumPy face
loop.plot_trajectory()
loop.animate()
```

Prints: the program's size, the record (`success=True`, the worst residuals), the rebound
solve's `1.3`. Plots: the plan, the open-loop replay, the closed-loop run with the MPC's
horizons overlaid. The `params` tree of the MPC block (`loop.params["controller"]`):
`{"plant": {"m", "l", "gravity", "d", "x0"}, "cost": {"Q", "R", "S", "xbar", "ubar"}, "U": {},
"Xf": {"center", "radius"}, "discount_rate": 0.0, "x0": z_warm}`, the planner's model of the
problem, distinct from the outer problem's own `cost` when the loop is evaluated (§5.6). What
changes: `compile()` returns the program with `params` (R130), `solve(params=)` rebinds
(`ParametricMathematicalProgram`, `compile_parametric_program` and `solve_trajectory_from` go),
`dt_mpc` → `sample_period`, `parameter_ports=` (R134), the one-period delay (R123).

### 5.6 The native fixed-iteration MPC: its closed loop in one scan, tuned by gradient on a cost weight

```python
import jax
import numpy as np
from minilink import (ModelPredictiveController, Optimizer, Pendulum, PlanningProblem, QuadraticCost,
                      TrajectoryOptimizationPlanner)

plant = Pendulum()
problem = PlanningProblem(plant, cost=QuadraticCost.from_system(plant, Q=np.eye(2), R=0.1 * np.eye(1), xbar=[0.5, 0.0]),
                          tf=3.0, x0=[0.0, 0.0], U=plant.input_box)

# The MPC's own model: a short horizon with a terminal weight, single shooting, K = 20 projected gradient steps
inner = problem.replace(tf=0.5, cost=QuadraticCost.from_system(plant, Q=np.diag([10.0, 1.0]), R=0.1 * np.eye(1),
                                                              S=np.diag([200.0, 10.0]), xbar=[0.5, 0.0]))
planner = TrajectoryOptimizationPlanner(inner, n_steps=10, transcription="single_shooting",
                                        method="gd", iterations=20, step_size=1.0, backend="jax")
mpc = ModelPredictiveController(planner, sample_period=0.05)     # a StepSystem: step = K iterations from the shifted plan
loop = mpc @ plant

pe = problem.compile(loop)                           # the closed loop is one lax.scan; each tick's K steps an inner scan
report = pe.evaluate()
print(report.objective)
batch = pe.evaluate(params=pe.params, key=0, n_trials=1)         # one start; vmap over starts: problem with x0=Uniform(...)

# Tune the MPC's own terminal weight by gradient of the problem's cost through the closed loop
program = pe.program(wrt=["controller.cost.S"])      # J(z): the closed-loop score; ∇J by reverse mode through both scans
tuned = Optimizer(program, method="adam", options={"learning_rate": 0.05, "maxiter": 200}).solve()
print(tuned.cost, program.params_at(tuned.z)["controller"]["cost"]["S"])
pe.trajectory(program.params_at(tuned.z)).of("plant").plot()
```

Prints (†: the prototype's closed loop of §10, NumPy, with a finite-difference gradient in
place of `jax.grad`): `8.767†`; then the tuned cost and the terminal weight. Plots: the tuned
closed-loop run. The tree: `pe.params["controller"] = {"plant": {…, "x0"}, "cost": {"Q", "R", "S",
"xbar", "ubar"}, "U": {}, "discount_rate", "step_size": 1.0, "x0": z0}`; `iterations` and
`n_steps` are structural, not leaves (R131). The definition of done §11.5 is this script with
`x0=Uniform(...)` and `n_trials=64`. What changes: the native kernel (`optimization/kernels.py`),
`method="gd"` with `iterations=` on the planner, the pure `plan` verb (A6).

### 5.7 Value iteration and RL read the same `J`; a `QuadraticField` as terminal cost and as the Lyapunov `V`

```python
import numpy as np
from minilink import (CostFunction, DynamicProgrammingPlanner, LQRPlanner, Pendulum, PlanningProblem,
                      QuadraticCost, QuadraticField, ReinforcementLearningPlanner, Uniform, compare, evaluate)
from minilink.core.backends import array_module

plant = Pendulum()
plant.inputs["u"].lower_bound, plant.inputs["u"].upper_bound = [-4.0], [4.0]
plant.state.lower_bound, plant.state.upper_bound = [-4 * np.pi, -20.0], [4 * np.pi, 20.0]
UP = np.array([np.pi, 0.0])

class SwingUp(CostFunction):
    def g(self, x, u, t=0.0, params=None):
        xp = array_module(x)
        return (1.0 + xp.cos(x[0])) + 0.01 * x[1] ** 2 + 0.01 * u[0] ** 2
    def h(self, x, t=0.0, params=None):
        return 0.0

problem = PlanningProblem(plant, cost=SwingUp(), tf=np.inf, discount_rate=0.5,
                          x0=Uniform([-np.pi, -1.0], [np.pi, 1.0]), X=plant.state.box, U=plant.input_box)

vi = DynamicProgrammingPlanner(problem, x_grid=(201, 201), u_grid=(21,), dt=0.05).solve()   # Euler successors, ĉ_k = g dt
rl = ReinforcementLearningPlanner(problem, dt=0.05, algorithm="ppo", hidden=(32, 32)).solve(timesteps=120_000)

cmp = compare(problem, VI=vi, PPO=rl, dt=0.05, n_trials=100, key=1, episode_length=10.0)
print(cmp)                                           # the same scan scores both: starts and seeds from one key
x0 = np.array([0.0, 0.0])
print(vi.cost_to_go.value(x0), evaluate(problem, vi.policy, dt=0.05, episode_length=10.0).objective)  # equal to the grid's resolution
vi.plot_cost_to_go(); rl.plot_control_law()

# The LQR value as a field: a terminal cost for a short-horizon problem, and the certificate's V
lq = LQRPlanner(problem.replace(cost=QuadraticCost.from_system(plant, Q=np.eye(2), R=np.eye(1), xbar=UP))).solve()
V = lq.cost_to_go                                    # QuadraticField(P, center=UP): V(x) = (x − x̄)ᵀ P (x − x̄)
short = problem.replace(tf=1.0, cost=problem.cost + V.as_cost(terminal=True))   # h(x_N) = V(x_N)
region = V.as_set(upper=2.0)                         # the ellipsoid {V ≤ c}, a Set with sample and bounding_box (c from the certificate's search)
print(region.contains(UP + [0.1, 0.0]))
```

Prints: the comparison table (two rows: the record and the evaluation of each law, mean `J`,
feasible and reached rates), the two values of `J(x0)` within the grid's interpolation error,
`True`. Plots: the cost-to-go surface, the learned law over the box. What changes: value
iteration's per-step cost and the RL reward come from the one lowering (`ĉ_k`, under Euler
`g·dt`: their numbers hold); `cost_to_go` is a `Field` (`GridField` for the table is a v0.4
row; in v0.3 the interpolated callable keeps `.value`); `QuadraticField`, `as_cost(terminal=)`,
`as_set` (R133).

### 5.8 The research frontier: a stochastic sampled loop with a seeded policy, vmapped over seeds, tuned by gradient

```python
import jax
import jax.numpy as jnp
import numpy as np
from minilink import (GaussianPolicy, MLP, Optimizer, Pendulum, PlanningProblem, Uniform, WhiteNoise)

plant = Pendulum()
features = lambda x: jnp.array([jnp.cos(x[0]), jnp.sin(x[0]), 0.1 * x[1]])
policy = GaussianPolicy(MLP(3, (32, 32), 1, features=features), log_std=-1.0, sample_period=0.05)  # seed=None: the mean
loop = policy % 0.05 @ plant                         # a seeded computer on the plant

problem = PlanningProblem(plant, cost=SwingUp(), tf=5.0, x0=Uniform([-np.pi, -1.0], [np.pi, 1.0]),
                          disturbances={"d": WhiteNoise(1, psd=0.05, sample_period=0.05)}, U=plant.input_box)
pe = problem.compile(loop)                           # 100 ticks; the policy's ε_k and the source's w_k from their seeds
report = pe.evaluate(n_trials=256, key=7)            # 256 × (start, policy seed, source seed), one vmap
print(report.mean, report.std)

# Policy gradient by hand: the exact gradient of the sample average through the dynamics
J = lambda p: pe.evaluate(p, key=7, n_trials=256).objective
dJ = jax.grad(J)(pe.params)
print(jax.tree_util.tree_map(jnp.shape, dJ["controller"]["mean"]))   # the MLP's weight shapes

# The same through the program: Adam on the policy weights, the seeds fixed (a sample average, R131)
program = pe.program(wrt=["controller.mean"], key=7, n_trials=256)
best = Optimizer(program, method="adam", options={"learning_rate": 1e-3, "maxiter": 500}).solve()
print(best.cost, pe.evaluate(program.params_at(best.z), key=8, n_trials=256).mean)   # scored on fresh draws

x, u = jnp.array([0.3, 0.0]), jnp.array([0.2])
print(policy.log_prob(x, u), policy.entropy(x))      # what PPO reads; the RL planner trains this block
```

Prints: the mean and spread over 256 seeds; the weight shapes `{'W0': (32, 3), …}`; the tuned
cost and its held-out score; a log-density and an entropy. Plots: none (add
`pe.trajectory(key=7).of("plant").plot()`). The tree: `pe.params = {"controller": {"mean":
{"W0", "b0", …}, "log_std": [-1.0], "seed": 0, "sample_period": 0.05}, "plant": {…, "x0"}, "d":
{"seed", "psd", "sample_period"}, "cost": {}, "X": {}, "U": {}, "discount_rate"}`; `evaluate`
realizes `controller.seed` and `d.seed` per trial from `key`. What changes: `GaussianPolicy`
(R129), the seeded block inside a computer (R104, R128), the sampled face of `problem.compile`.

---

## 6. The final API, object by object

Signatures are given as code; "today" cites `file:line` at the reviewed commit; "changes" says
what moves. Equation paths keep RULES 5.1's bare signatures.

### 6.1 `System`, `DynamicSystem`, `StepSystem` (`core/system.py`, `core/discrete/step_system.py`)

```python
class System:
    def __init__(self, n=0): ...
    # fields: n, m, p, name, id, state, inputs, outputs, x0, params, solver_info, traj (the one cache)
    def h(self, x, u, t=0, params=None): ...                    # y; the port computes likewise
    def tf(self, x, u, t=0, params=None): ...                   # frames, unchanged
    def realize(self, key): ...                                 # params with a fresh seed under key; None → the mean
    def __add__(self, other) / __rshift__ / __matmul__ / __mod__(self, schedule)

class DynamicSystem(System):
    def f(self, x, u, t=0, params=None): ...                    # dx

class StepSystem(System):
    sample_period: float | None                                 # seconds per tick; None: a tick is one second
    def step(self, x, u, k=0, params=None): ...                 # x_{k+1}
    def h(self, x, u, k=0, params=None): ...
```

- Today: `core/system.py:77–131` (the fields), `:190` (`realize`), `:409–459` (the operators,
  `%` importing `simulation.discrete.computer`); `core/discrete/step_system.py:16–96`.
- Changes: `StepSystem` gains the structural attribute `sample_period` (set by `discretize`, by
  `Computer`, by `ModelPredictiveController`); `%` builds a `Computer` from `core/discrete/`
  (A5); `StepSystem.rollout` and `HybridDiagram.last_result` go (RULES 2.4), `self.traj`
  stays the one cache; `compute_rollout` returns a `Trajectory` with `t = k·sample_period`.
- `params` contract: every leaf float, array or the integer `seed`; `x0` is a field, not a
  leaf of the block's own `params` (it joins the run's tree at compile, A1).

### 6.2 The diagrams, the operators, `discretize`, `%`, `Computer`, `SampledDataSystem`

```python
class DiagramSystem(WiredDiagramMixin, DynamicSystem):          # core/diagram.py
    # x0, n, state: views of the blocks (R124); setting x0 writes through
    def add_subsystem(self, sys, sys_id) / connect(...) / expose(*names) / autowire(...)
    def f(self, x, u, t=0, params=None) / compile(backend="numpy", *, flatten=False)
    def trajectory_of(self, block, traj=None) -> Trajectory

class StepDiagramSystem(WiredDiagramMixin, StepSystem):         # core/discrete/step_diagram.py
    def step(self, x, u, k=0, params=None) / compile(backend="numpy")

# core/operators.py, core/closed_loop.py: the same three operators on both algebras (R126)
a + b        # parallel: both continuous, or both step (sample periods equal or None)
a >> b       # series, one edge rule (DESIGN §4)
c @ plant    # feedback by role: controller / plant; a Computer on a DynamicSystem → SampledDataSystem
feedback(L, filter=1, *, of=None, sign=-1.0)
closed_loop(controller, plant, *, reference=, disturbance=, noise=, filter=, observer=)

def discretize(sys: DynamicSystem, dt, *, solver="rk4", substeps=1) -> StepSystem   # analysis/discretization.py
    # "exact" for an LTI block; the result keeps `source`, `sample_period = dt`, params a view of the source's

class Computer(StepDiagramSystem):                              # core/discrete/computer.py (new home)
    """Blocks on a base clock: block i fires on k mod d_i == 0; a slow block's held outputs are states."""
    def __init__(self, blocks: dict[str, System] | System, *, sample_period, fire=None, solver="auto", substeps=1): ...
    sample_period: float; fire: dict[str, int]
    @staticmethod
    def schedule(sample_period, fire=None) -> Schedule           # for `block % Computer.schedule(0.01, fire={"mpc": 5})`

block % dt                     # Computer(block, sample_period=dt): a continuous block is discretize(block, dt) (exact for LTI,
                               # rk4 otherwise); a static block is sampled and held; a StepSystem keeps its own step
(a >> b) % dt                  # several blocks on one clock, ids kept; the diagram's params nested by id
block % Computer.schedule(dt, fire={"b": 4})   # multi-rate

class SampledDataSystem(StepDiagramSystem):                     # core/discrete/sampled_data.py (today hybrid_diagram.py)
    """computer @ plant at the computer's base clock: two blocks, `controller` (the Computer) and
    `plant` (discretize(plant, sample_period, substeps)), the ZOH in the plant's step; keeps `computer`
    and the continuous `plant` as sources."""
    computer: Computer; plant: DynamicSystem; sample_period: float
    def compute_rollout(self, *, tf=None, n_steps=None, u=None, params=None) -> Trajectory      # the ticks, pure
    def compute_trajectory(self, tf, *, solver="scipy", params=None, x0=None) -> Trajectory     # the clock runner (simulation/)
    def plot_diagram(self, ...)                                 # the two-cluster drawing
```

- Today: `core/diagram.py:19–206`, `core/discrete/step_diagram.py:13–105`,
  `simulation/discrete/computer.py:34–258` (`Computer`, mutable, NumPy only, one tick of lag),
  `core/discrete/hybrid_diagram.py:30–420` (a plain class), `core/discrete/hybrid_composition.py:26`
  (`hybrid_closed_loop`), `analysis/discretization.py:21–80` (imports SciPy at module level for
  `"exact"`: to make lazy, since the step model itself is NumPy).
- The pure tick of a `Computer` (its `step`), in the order a `lax.scan` body needs:
  1. gather the tick's signals in the compile-time topological order (R125): for each block,
     its outputs at tick `k` from its state and its inputs; a slow block's output on a
     non-firing tick is its held value `y_held`, on a firing tick its fresh value;
  2. each block's state: `x_i⁺ = where(k mod d_i == 0, step_i(x_i, u_i, k, p_i), x_i)`;
     each held output: `y_held⁺ = where(fired, y_i, y_held)`;
  3. return the stacked state. The held outputs are part of `n` and of `x0` (zeros, or the
     block's nominal output).
  An algebraic loop (a cycle with no state between) is refused at wiring time (`topology.py`).
- Multi-rate layout: the base tick with selects (sampled-path §8.1 draft 3, row 2), one layout
  only in v0.3; the hyperperiod unrolling is an optimization for later with the same numbers.
- `SampledDataSystem.compute_trajectory`: the runner in `simulation/discrete/clock_runner.py`
  ticks the computer's pure `step` and integrates the plant's `f` between the instants where
  the plant's input changes (`solve_ivp`, restarted there; `solver="rk4"` keeps today's
  substeps), records the dense plant path and every tick's signals, and returns a `Trajectory`
  with roles. It is the realtime simulator's offline twin. The step diagram is its oracle at
  the ticks (prototype §10.2: identical to the bit on NumPy).

### 6.3 `Trajectory` (`core/trajectory.py:52`)

```python
@dataclass(frozen=True)
class Trajectory(SampledSignals):
    t: (N,); x: (n, N); u: (m, N); signals: dict[str, (d, N)]; roles: dict
    def of(self, role) -> Trajectory / resample(t) / with_cost(cost, *, discount_rate, params) / sample(name, t)
    def plot(self, *, signals=None, backend="matplotlib")        # the one plotting verb a run owns
    t0, tf, n_samples; save / load                                 # a JAX pytree
```

Changes: `StepRollout` folds in (`t = k·sample_period`, a `k` property reads `t / Δ`); a
`plot` shortcut (today the plots sit on the system facade only).

### 6.4 `Set`, `InputSet`, `CostFunction`, `Distribution` (`core/sets.py`, `core/costs.py`, `core/distributions.py`)

```python
class Set:       params: dict = {}                               # frozen dataclasses keep their numbers as fields and in params
    def margin(self, x, t=0.0, params=None) / contains(x, t, params) / sample(key, n=None, params=None) / bounding_box()
    def as_cost(self, *, weight=1.0, shaping=None) -> CostFunction    # Field.as_cost on the margin seen as a field (R133)
    __and__ → IntersectionSet
class InputSet:  def margin(self, u, x=None, t=0.0, params=None) ...
BoxSet(lower, upper), BallSet(center, radius), SingletonSet(point), CallableSet(fn), IntersectionSet, FieldSet (from Field.as_set)

class CostFunction:  params: dict
    def g(self, x, u, t=0.0, params=None) / h(self, x, t=0.0, params=None)
    def integrate(self, traj, *, discount_rate=0.0, params=None)   # a reporting verb (the trapezoid on samples), not the score
    __add__ → SumCost (terms by id), __mul__ → ScaledCost ({"weight", "cost"})
QuadraticCost(Q, R, S, xbar, ubar), TimeCost, FieldCost (from Field.as_cost)

class Distribution:  params: dict; dim; mean; support: Set | None
    def sample(self, key, n=None, params=None)                   # key: an int, a NumPy generator or a JAX key
Gaussian(mean, std=None, *, cov=None), Uniform(lower, upper), Particles(points)
```

- Today: `core/sets.py:22–140` (`params=None` on sets, `Set.params = None`), `core/costs.py:29–150`
  (`integrate` is the evaluator's rule), `core/distributions.py:24–120` (`sample(key, n)`, no
  `params`).
- Changes (R140's five resolutions): `params = {}` on an object with no numbers; a
  distribution's law numbers in `params` and `sample(key, n, params=None)`; `Set.as_cost`;
  `integrate` demoted to reporting.

### 6.5 `Field` (`core/fields.py`, new)

```python
class Field:     params: dict
    def value(self, x, u=None, t=0.0, params=None)                # a scalar
    def gradient(self, x, u=None, t=0.0, params=None)             # jax.grad under JAX, central differences on NumPy
    def as_set(self, *, lower=0.0, upper=None) -> Set             # {x : lower ≤ V(x) ≤ upper}
    def as_input_set(self, *, lower=0.0, upper=None) -> InputSet  # over u at a given x
    def as_cost(self, *, weight=1.0, shaping=None, terminal=False) -> CostFunction
    __call__ = value

class QuadraticField(Field):  # V = (x − c)ᵀ P (x − c); params {"P", "center"}; as_set gives an EllipsoidSet with sample and bounding_box
    def __init__(self, P, center=None): ...
CallableField(fn), GridField(grid, values)        # v0.4, with their consumers (R133)
quadratic_hinge, inverse_barrier, …               # the shaping functions, moved from planning/spatial/shaping.py
```

- Today: `planning/spatial/state_fields.py:26–56` (`StateField`, `as_constraint`, `as_cost`),
  `planning/spatial/shaping.py`.
- Changes: the move and the names (R133); `terminal=` is a structural flag on the conversion
  (never in `params`), so `cost + V.as_cost(terminal=True)` charges `V(x_N)` at the horizon.
  `LQRPlanner`'s `cost_to_go` and the Lyapunov certificate's `V` become `QuadraticField`s.

### 6.6 The sources (`blocks/sources.py`) and the seeded policy (`control/neural.py`)

```python
Step(final_value, step_time, *, port="r"), Sine(...), TrajectorySource(traj, *, interpolation)   # the table in params ("values")
WhiteNoise(p=1, *, psd=1.0, sample_period=0.01, seed=0, hold="zoh", port="y")
    # params {"seed", "sample_period", "psd"}; h(x, u, t, params): w_k = (W/Δ)^{1/2} ε(seed, ⌊t/Δ⌋), ε by Threefry-2x32
NoiseSource(law: Distribution, *, sample_period, seed=0)          # held draws of any law (RN-3), the same cipher

class GaussianPolicy(System):
    """u = μ(x) + exp(log_std) · ε(seed, ⌊t/Δ⌋); seed=None is the mean."""
    def __init__(self, mean: System, log_std, *, sample_period, seed=None): ...
    # mean: a static block u = μ(x) (an MLP, a StateFeedback, a LookupPolicy); log_std: an (m,) array, or a static block σ(x)
    # params {"mean": mean.params, "log_std": …, "seed": seed, "sample_period": Δ}
    def h(self, x, u, t=0, params=None): ...
    def log_prob(self, x, u, params=None) / entropy(self, x, params=None)
SquashedGaussianPolicy(GaussianPolicy)                              # SAC's head
NeuralPolicy = GaussianPolicy with seed=None (the v1.0 rename of NeuralPolicyController)
```

- Today: `blocks/sources.py:232–300` (`WhiteNoise`, already pure and seeded),
  `learning/reinforcement_learning/policy.py:129–160` (`GaussianHead` with its own keys).
- Changes: `GaussianPolicy` is the block (R129); the RL heads are rebuilt on it (with the block
  or in v0.4, a placement detail); the collectors take per-episode seeds.

### 6.7 `PlanningProblem` (`planning/problems.py`)

```python
class PlanningProblem:
    def __init__(self, sys, *, cost=None, tf=None, x0=None, x_goal=None, X=None, U=None, Xf=None,
                 params=None, disturbances=None, discount_rate=0.0, infeasible_cost=None, criterion="expectation"):
        # x0: a point (X0 = its singleton), a Set (a free start; its representative point is the nominal),
        #     or a Distribution (a law; X0 = its support, nominal = its mean); None: the plant's x0
        # params: {dotted path: Distribution}, laws on leaves of sys.params ("plant.m")
        # disturbances: {port: source block}, wired onto sys's ports by the shortcut; a bare law is refused naming NoiseSource
    sys, cost, tf, X, U, X0, Xf, x_goal, discount_rate, infeasible_cost, criterion
    @property params -> dict          # {"plant": sys.params + {"x0": nominal start}, "cost", "X", "U", "Xf", "<port>": source.params, "discount_rate"}
    @property is_stochastic -> bool   # any law declared, or a random block in sys
    def realize(self, key, n_trials=1) -> (x0s, params)   # (n, n_x) starts; the tree with a leading axis on drawn leaves and seeds
    def compile(self, policy, dt=None, *, solver="rk4", substeps=1, backend="auto", episode_length=None) -> ProblemEvaluator
    def replace(self, **changes) / cost_bound(tf=None, *, discount_rate=None, dt=None) / discount_factor(dt) / horizon_kind()
```

- Today: `planning/problems.py:111–195` (`x_start`, `X0`), `:551–600` (the stochastic sibling),
  `:246` (`realize(key, n)` → three shapes).
- Changes: A1, A2, R128's shortcut and one `realize` shape; `x_start`/`X0` → `x0`.

### 6.8 `problem.compile` → `ProblemEvaluator`, `evaluate`, `Evaluation`, the one score (`planning/evaluator.py`, `planning/evaluation.py`)

```python
class ProblemEvaluator:
    def __init__(self, problem, policy, dt=None, *, solver="rk4", substeps=1, backend="auto", episode_length=None): ...
    # policy: a continuous controller (closed as policy @ plant, sampled at dt with its command held),
    #         a Computer (closed as computer @ plant; dt = its sample_period), a SampledDataSystem or a
    #         DiagramSystem whose `plant` block is the problem's system, a source (open loop), a PlanningSolution (its policy)
    loop: StepDiagramSystem         # the lowering: the policy's step side closed on discretize(PlantWithCost(plant, cost, rho), dt, substeps)
    N: int; gamma: float; dt: float
    @property params -> dict        # the run's tree: {block_id: block.params + {"x0"}} for every block, the problem's parts, "discount_rate"
    def trajectory(self, params=None, key=None) -> Trajectory
    def evaluate(self, params=None, key=0, n_trials=None, *, record=False) -> Evaluation
    def program(self, wrt, *, bounds=None, key=0, n_trials=None, params=None) -> MathematicalProgram
    def pack(self, params, wrt) -> z / unpack(self, z, wrt, params=None) -> dict

def evaluate(problem, policy, *, dt=None, n_trials=None, key=0, **compile_kwargs) -> Evaluation
def compare(problem, *, dt, n_trials, key, **entries) -> Comparison

@dataclass(frozen=True)
class Evaluation:   # a pytree, one row per trial
    cost, J, feasibility_margin, failed, reachability_margin, reached, x0, trajectories, criterion
    objective, mean, std, worst, cvar(alpha), feasibility_rate, reachability_rate, success_rate
```

The one score, as the scan `evaluate` runs (the prototype of §10.3 is this body in NumPy):

```text
carry = (X_0, J = 0, w = 1, m = +inf)                   X: the loop's stacked state, the plant's cost slot included
for k in 0 … N−1 (lax.scan):
    x_k, u_k      = the plant's state and input at tick k (from the loop's signals)
    m             = min(m, X.margin(x_k, t_k), U.margin(u_k, x_k, t_k))
    X_{k+1}       = loop.step(X_k, u_ext, k)            the plant's step integrates [x; c] with c reset to 0 → ĉ_k
    J             = J + w · ĉ_k ;  w = w · γ
J   = J + w · h(x_N, t_N)                 (finite horizon)
reachability = Xf.margin(x_N, t_N);  failed = ¬finite ∨ m < 0;  J += price · (failed ∨ ¬reached) when the switch is on
```

- Today: `planning/evaluator.py:40–125` (the constructor), `:146–195` (`trajectory`,
  `evaluate`, `program`), `:355–410` (`run`: the continuous rollout on the step model, then
  `score_run`), `:789` (`require_continuous` refuses a `StepSystem`); `planning/evaluation.py:125–222`
  (`score_run`, the trapezoid).
- Changes: the lowering (A3) replaces `run` + `score_run`'s quadrature; the sampled face
  accepts a `Computer` and a `SampledDataSystem` (R105); `score_run` keeps the failure rule
  and is called on the scan's results; `evaluate_trajectory(problem, traj)` stays for a held
  run (the trapezoid, reporting). `DynamicProgrammingPlanner`, `RolloutEnvironment` and the
  MPC's single-shooting transcription read `ĉ_k` from the same lowering (`planning/stepping.py`,
  the internal `DiscreteProblem`).

### 6.9 `MathematicalProgram`, `Optimizer`, the kernels (`optimization/`)

```python
@dataclass(frozen=True)
class MathematicalProgram:
    n_z: int; J; h=None; g=None; lower=None; upper=None; params: dict | None = None
    # J(z, params=None) -> scalar; h(z, params=None) -> (n_h,); g(z, params=None) -> (n_g,); params=None reads the frozen default.
    # A one-argument J(z) is accepted and wrapped (the student's form).
    grad_J, hess_J, jac_h, jac_g; backend; metadata
    def params_at(self, z) -> dict                        # on an export: the full tuned tree (R134.6)

class Optimizer:
    def __init__(self, program, z0=None, *, method="auto", backend=None, iterations=None, options=None, **method_kwargs): ...
    # method: "scipy_slsqp", "scipy_trust_constr", "ipopt", "l-bfgs" (wrapped, host), "gd", "adam" (native kernels)
    # iterations: the fixed K of a native kernel (then the solve is one scan and traces); None: run to options' gtol
    def solve(self, *, params=None, z0=None, callback=None, verbose=False) -> OptimizationResult
    # params: a partial tree merged once onto program.params at this boundary (R131); the compiled J(z, p) is not rebuilt
    adapter: OptimizerBackend                               # was .backend (R134.1)

# optimization/kernels.py: pure update laws, the Optax shape (R132)
class GradientDescent:
    def __init__(self, program, *, step_size): ...
    def init(self, z0, params=None) -> state                # {"z": z}
    def step(self, state, params=None) -> state             # z ← Π_box(z − η ∇J(z; p))
class Adam:  # {"z", "m", "v", "k"}
def as_step_system(kernel) -> StepSystem                    # x = the packed state, u = the program's parameter ports, step = kernel.step
```

- Today: `optimization/mathematical_program.py:26–123` (`J(z)` only),
  `optimization/optimizer.py:118–227` (`compile_backend`, `.backend`),
  `optimization/optimizers/first_order.py:51–196` (a Python loop; `Adam.init/update` already the kernel shape),
  `planning/trajectory_optimization/parametric_program.py:25` (`x0` only).
- Changes: `params=None` on the program (R130) with the performance gate (§7.6); the kernels;
  `iterations=` for the fixed-`K` traced solve; `backend=`/`adapter`.

### 6.10 The planners, `PlanningSolution`, the planner interface, `ModelPredictiveController` (`planning/`)

```python
class Planner:
    def __init__(self, problem): ...
    def compile(self) -> MathematicalProgram | None          # build once; None for a planner with no program (DP, RRT)
    def plan(self, z0, params) -> z                          # pure: the solve from a warm start at these parameters (the MPC's verb)
    def shift(self, z) -> z                                  # z one control period later (the warm start's knowledge)
    def unpack(self, z, params=None) -> (u_seq, x_seq)       # the plan the MPC reads: u_seq (N, m); x_seq (N+1, n) or None
    def solve(self, *, params=None, z0=None, evaluate=False, n_trials=None, **kw) -> PlanningSolution   # the user's verb
    def get_controller(self) -> System

@dataclass(frozen=True)
class PlanningSolution:   # unchanged in spirit (R138)
    problem; policy: System; solver: record (with z where there is one); trajectory; evaluation; cost_to_go: Field | None
    success, open_loop, method; plot_trajectory / plot_cost_to_go / plot_control_law / plot_cost

TrajectoryOptimizationPlanner(problem, *, n_steps, transcription="direct_collocation" | "single_shooting" | "multiple_shooting",
                              method="auto", iterations=None, step_size=None, backend="auto", options=None)
DynamicProgrammingPlanner(problem, *, x_grid, u_grid, dt, solver="euler", ...)      # reads ĉ_k from the lowering
LQRPlanner(problem)                                                                  # cost_to_go: a QuadraticField
ReinforcementLearningPlanner(problem, *, dt, algorithm, hidden, features, ...)      # trains a GaussianPolicy; the environment is the lowering
RRTPlanner, RRTStarPlanner, TabularLearningPlanner, MPPIPlanner (v0.4)

class ModelPredictiveController(StepSystem):
    """step(z, y, k; p) = plan(shift(z); p with plant.x0 = y);  u_k = unpack(z_k).u_seq[0]  (one-period delay, R123)."""
    def __init__(self, planner, *, sample_period, warm_start=True, parameter_ports=(), verbose=False): ...
    # params: the planner's program params (plant, cost, sets, discount_rate) + the kernel's (step_size) + "x0" (z0)
    # parameter_ports: ["plant.m"] declares an input port `m` whose value replaces that leaf at the tick (nominal when unwired)
    def step(self, x, u, k=0, params=None) / h(self, x, u, k=0, params=None)
```

- Today: `planning/planner.py:21–200` (`solve`, `solve_trajectory_from`),
  `planning/results.py:21–170`, `planning/trajectory_optimization/planner.py:64–790`
  (`compile_parametric_program`, `solve_trajectory_from_parametric`),
  `planning/trajectory_optimization/mpc/controller.py:59–420` (the mixin, the latch,
  `Command`), `mpc/predictive.py:14–67` (`dt_mpc`).
- Changes: A6's four verbs; the MPC as one `StepSystem` class (the stateless variant goes:
  `warm_start=False` is `shift = lambda z: z0`); the latch and `Command` go; a host-solver
  planner's `plan` runs on NumPy (the clock runner), a native one traces (the scan).

---

## 7. The architecture under it

### 7.1 Layers

```text
 user scripts, notebooks, demos
 ───────────────────────────────────────────────────────────────────────────────────────────
 learning/      ReinforcementLearningPlanner, TabularLearningPlanner   (read the lowering, train a GaussianPolicy)
 planning/      PlanningProblem · ProblemEvaluator · the one lowering (stepping.py: DiscreteProblem, PlantWithCost)
                Planner · PlanningSolution · trajopt (transcriptions) · MPC · DP · LQRPlanner · RRT · spatial
 optimization/  MathematicalProgram(params) · Optimizer · kernels (gd, adam) · adapters (scipy, ipopt)
 ───────────────────────────────────────────────────────────────────────────────────────────
 analysis/      linearize · discretize · bode · margins · pzmap · lyapunov · riccati        (verbs on a System)
 simulation/    Simulator (continuous, adaptive) · clock_runner (the sampled-data orchestrator) · realtime
 control/ estimation/ blocks/ dynamics/                                                      (the catalog of Systems)
 ───────────────────────────────────────────────────────────────────────────────────────────
 core/          System · DynamicSystem · StepSystem · DiagramSystem · StepDiagramSystem · Computer · SampledDataSystem
                Trajectory · Set · CostFunction · Distribution · Field · geometry · params (the one tree module)
                compile/ (evaluators: f, step, rollout, the scan; jacobian) · backends (array_module, require_jax)
```

### 7.2 The one contract (R140), as the test states it

For every public core math object `obj` and each of its equation methods `m`:
1. `obj.params` is a dict; its leaves are floats, numeric arrays, or the integer under the key
   `seed`; a composite's `params` is `{name: part.params}` plus its own leaves; an object with
   no numbers has `{}`.
2. `m(*args)` equals `m(*args, params=obj.params)` on both backends.
3. `jax.grad` of a scalar of `m` with respect to every float leaf of `params` is finite.
4. `m` called twice with the same arguments and the same `params` returns the same arrays; a
   random object called with two `seed`s returns different arrays; `realize(key)` is a pure
   function of `key`.
5. `jax.vmap` over a leaf with one more axis (a parameter family, a batch of seeds) works.
The exceptions list in DESIGN §4: RL training's collectors until they take per-episode seeds;
student-written models (RULES 2.3's tolerance).

### 7.3 Module layout and the dependency law

```text
core            imports nothing of minilink but itself
blocks, dynamics, control, estimation       → core
analysis, simulation                         → core (the compiler directly), blocks
optimization                                 → core (array_module, require_jax) only; never planning
planning                                     → core, analysis, simulation, control, estimation, optimization
learning                                     → planning (and below)
graphical                                    → core (+ lazily from the facades)
```

Facades on `System` delegate lazily to `simulation/`, `analysis/`, `graphical/` (the pattern
of `core/facades/`). The one exception written today (R100: `core` → `simulation.discrete`
for `%`) disappears with `Computer` in `core/discrete/`. `SampledDataSystem.compute_trajectory`
reaches the clock runner the way `DynamicSystem.compute_trajectory` reaches `Simulator`.
`planning/stepping.py` (the lowering) imports `analysis.discretization` and
`core.compile`; `learning` imports it from `planning`, never a copy. Parameter trees have one
module, `core/params.py` (`lookup`, `with_leaf`, `merge`, `flatten`, `unflatten`, `strip`), used
by the diagrams, the evaluator, the program export and the Jacobian.

### 7.4 The runtime model: what compiles once, what varies, the two runners, where host code stays

| Call | Built once (structural) | Varies per call (values) | Runs |
| --- | --- | --- | --- |
| `sys.compile(backend)` | the plan: topology, slices, port order, `n`, `m` | `x, u, t, params` | `f`, `step`, `rollout` jitted on first use |
| `problem.compile(policy, dt)` | the lowering: the step loop (the policy's step side closed on `discretize(PlantWithCost)`), `N`, `γ`, `dt`, `substeps`, `solver`, the recorded slices, the batched trial function | `pe.params` (every leaf), `key`, `n_trials` (a shape: one compile per `n_trials`) | one `lax.scan` per trial, `vmap` over trials, `jit` of the whole |
| `pe.program(wrt)` | the pack/unpack of `wrt`, `J(z, p)`, `∇J` by reverse mode, the rows | `z`, `p` (the rest of the tree) | the `Optimizer`'s adapter calls the jitted `J`, `∇J` |
| `planner.compile()` | the transcription: `n_z`, the rows, `J(z, p)`, `h(z, p)`, `g(z, p)` | `p = {plant (x0 included), cost, sets, discount_rate}` | host (Ipopt, SLSQP) or native (`K` steps of a kernel, one scan) |
| `ModelPredictiveController.step` | the planner's program, `K`, the shift | `z`, `y`, the live ports' leaves | inside the loop's scan (native) or the clock runner (host) |
| `Optimizer(program)` | the program evaluator: `J`, gradients, Jacobians compiled with `p` as an argument | `z0`, `params` | the adapter (host) or the kernel (scan) |

The two runners of a sampled-data system (R127): `compute_rollout` and everything that
compiles, differentiates or batches run the step diagram's scan; `compute_trajectory` and the
realtime simulator run the clock runner over the same pure blocks, with the adaptive plant
solver on the NumPy face. Host-only blocks (an Ipopt MPC) live in the clock runner; a loop
that holds one is refused by `problem.compile(backend="jax")` with the native method named.

Where the `x0` leaves go (A1, R124): `pe.params[b]["x0"]` for each stateful block `b` is read
by the evaluator to assemble the loop's start `X_0`; `core/params.strip(tree, "x0")` hands
each block its equation tree without it.

### 7.5 The JAX-traceability and efficiency argument, path by path

1. **The compiled rollout (today, kept):** `iterate` is a `lax.scan` on JAX
   (`core/compile/evaluators/stepping.py:66`), the knobs static, `params` a pytree argument:
   values vary without retrace. 1000 rollouts × 1000 RK4 steps in tens of milliseconds
   (DESIGN §5). Unchanged.
2. **The computer's tick:** `where` for firing (no Python branch on `k`), the held outputs in
   the carry, the port order fixed at compile. Cost: a slow block is computed and discarded on
   the ticks it skips (the base-tick layout); for the common two-clock loop (`d ≤ 10`) this is
   under a factor of two on the controller side, and the plant dominates. Verified on NumPy
   (§10.2) to be the orchestrator's oracle to the bit.
3. **The one score:** one scan with carry `(X, J, w, m)`; `vmap` over trials maps the batched
   leaves of `realize`'s tree (starts, drawn parameters, seeds) and broadcasts the rest, the
   shape of today's `run_trials_jax` (`planning/evaluator.py:462`); `jax.grad` of `objective`
   flows through the scan in reverse mode (memory `N × |X|` per trial, as today). The cost
   slot adds one state to the plant: the RK4 stages integrate it for free. The failure price
   is a `where` on the result (no gradient, as today).
4. **The draws:** `child_seed` and `standard_normal` are integer arithmetic on `seed` and `k`
   (Threefry-2x32, `core/distributions.py:162–250`), the same bits on both backends; `vmap`
   over a seed axis traces since seeds are integer leaves (§10.3 shows two trials from one key
   and a repeatable `ε(seed, k)`). The seed never enters `wrt` (R131).
5. **The native solve:** `Optimizer(program, method="gd", iterations=K)` is
   `lax.scan(kernel.step, kernel.init(z0, p), None, length=K)`; `∇J(z; p)` inside the body is
   `jax.grad` of the jitted `J`; the box projection is `clip`. A program with equality rows is
   refused for a native method (the class table of DESIGN §6). The prototype §10.4 runs the
   kernel as a rollout and reads the convergence rate off the step's Jacobian.
6. **The MPC's closed loop:** the outer scan (the loop) carries `(z, x_plant, …)`; each tick's
   `plan` is the inner scan of point 5 with `p["plant"]["x0"] = y_k`; `jax.grad` of the
   closed-loop score with respect to `controller.cost.S` is reverse mode over a scan whose
   body contains a `grad`: supported (higher-order differentiation through `scan`), with
   memory `N_ticks × K × H` stage states; `jax.checkpoint` on the tick body bounds it. Single
   shooting keeps the program box-bounded so no constraint rows enter the kernel. The prototype
   §10.5 runs the closed loop and the tuning gradient (finite differences standing in for
   `grad`) on NumPy.
7. **The program with parameters (R130's gate):** `J(z, p)` jitted with `p` a pytree argument
   costs one extra argument transfer per call and no retrace; the benchmark of the gate
   compares `jit(J)(z, p)` against a closure `jit(lambda z: J(z, p0))` on the MPC and PID
   programs (compile and per-solve time). Expected: within noise, because XLA sees the same
   graph with `p` as inputs instead of constants. If the gate fails, `Optimizer(program,
   params=p0)` compiles the closure (parameters baked in) and refuses `solve(params=)`.
8. **What does not trace, by design:** `compute_trajectory` on a sampled-data system (the
   adaptive plant), `solver="scipy"` on the evaluator (the cross-check), host solvers (Ipopt,
   SLSQP) inside a tick, the realtime runner. Each is a NumPy face of the same pure blocks.

### 7.6 The naming and the separator rule (R134), applied

`backend=` the array module everywhere (`compile_backend=` goes), `solver=` the integrator,
`method=` the optimization algorithm, `sample_period=` a block's period, `dt=` a tool's grid,
`iterations=` the fixed `K`, `n_steps=` a transcription's knots. `.` walks a tree
(`controller.cost.S`, `plant.x0`), `:` names a signal (`plant:u`, `controller:u`). `mount(geometry,
on=sys, frame=)` replaces the geometry `bind`. `…Policy` for a learned or tabulated law.

---

## 8. The v0.3 coverage table

| Workstream / choice | Where this design satisfies or revises it |
| --- | --- |
| §1 every object pure, every number a parameter (R140) | §7.2 the contract and its test; §6.4 `params = {}`, distributions with `params`; one `core/params.py` (§7.3); A1 for `x0` |
| §1 three owners of a start (R124) | §6.2 diagram `x0` a view with write-through; §6.7 the problem's `x0`; §6.8 the `x0` leaves; **revised** by A1 (`plant.x0`, no `x_start`) |
| §2 the step algebra first-class (R125, R126) | §6.2 the operators on step systems; the `Computer`'s pure tick with within-tick propagation; `Trajectory` with `t = k·Δ`; step evaluators trace and batch through the same `iterate` |
| §3 the draws (R128) | §6.7 `disturbances=` sources, `params=` laws, one `realize(key, n_trials) -> (x0s, params)`; §7.5.4 the cipher on both backends; §5.3 |
| §4 the seeded policy (R129) | §6.6 `GaussianPolicy`; §5.8 |
| §5 the hybrid loop (R127, R123, R104, R105) | §6.2 `Computer`, `SampledDataSystem`, `compute_rollout` / `compute_trajectory`; §7.4 the two runners; **revised** by A4 (the name) and A5 (the home of `Computer`) |
| §6 one score (the investigation) | A3 and §6.8: no discrete sets or costs, `t_k` bound; the cost a state of the step; one failure rule; the ZOH; an internal `DiscreteProblem`; VI and RL read it (§5.7) |
| §7 the parametric program (R130, R131) | §6.9 `MathematicalProgram(params=None)`, `solve(params=)`, the convention; §7.5.7 the gate; the stochastic program as the sample-average export (§5.8) |
| §8 the native proof of concept (R132) | §6.9 the kernels and `as_step_system`; §6.10 `method="gd", iterations=K` on single shooting; §5.6 the closed loop in one scan tuned by gradient |
| §9 Fields (R133) | §6.5 `Field`, `QuadraticField`, `as_set` / `as_input_set` / `as_cost(terminal=)`, the shaping in `core/`; §5.7 |
| §10 hardening and the merge | §9 the order; the contract test; S66's wiring-time refusals (the algebraic loop of a `Computer` among them) |
| R134 names | §7.6 |
| R135 the planner interface | A6 and §6.10: `compile`, `plan`, `shift`, `unpack`; the MPC owns the tick, the delay and the warm-start decision |
| R136, R137, R139 | untouched (later rows); R139.a's Diffrax would be the clock runner's JAX plant solver if ever needed |
| R138 `PlanningSolution` | §6.10, basic; `cost_to_go` typed `Field` |
| CONSTITUTION amendments owed | invariant 1 (the coupling is a tool, its result a `System` of the step algebra); §2's disturbance sentence (A2's wording); §6's grammar sentence (the operators on both algebras) |

---

## 9. The migration from today's code

Module by module: what changes, which numbers move (R80–R82: a move is byte-identical, a
restructure `rtol 1e-6`, an intended change re-pinned once with a closeness report), the risk,
and the order.

| Module | What changes | Numbers | Risk |
| --- | --- | --- | --- |
| `core/params.py` (new) | the four copies of path lookup, merge, flatten (`planning/evaluator.py:976–1046`, `planning/problems.py:896–914`, `core/compile/evaluators/jacobian.py`, `core/assembly.py`) become one module, plus `strip` | identical | low |
| `core/sets.py`, `core/costs.py`, `core/distributions.py` | `params = {}`; `sample(key, n, params=None)`; `Set.as_cost`; `integrate` kept as reporting | identical | low |
| `core/fields.py` (new) | `StateField` → `Field` moved, `QuadraticField`, the shaping, `as_set` / `as_input_set` / `as_cost(terminal=)`; callers renamed (track demos) | identical (RoA, DP baselines `cmp`) | low |
| `core/diagram.py`, `core/wiring.py` | `x0`, `n`, `state` as views; write-through; `refresh()` out of `Simulator`; `derived_x0` goes | identical | medium: `WhiteNoise` and the racecar read `refresh()` hints |
| `core/discrete/step_system.py`, `step_rollout.py`, `core/compile/evaluators/discrete.py` | `sample_period`; `StepRollout` → `Trajectory`; `rollout` returns native arrays | identical | low |
| `core/operators.py`, `core/closed_loop.py`, `core/assembly.py` | `+`, `>>`, `@` on step systems; `Computer @ DynamicSystem` → `SampledDataSystem` | n/a (new) | low |
| `core/discrete/computer.py` (new home), `simulation/discrete/computer.py` (goes) | `Computer(StepDiagramSystem)`: the pure tick, the held outputs as state, `%` from `core` | **intended change 1**: the one-tick lag goes (R125); the hybrid demos re-pinned with a closeness report | medium: the multi-rate demo's held-output states change `n` |
| `core/discrete/sampled_data.py` (today `hybrid_diagram.py`, `hybrid_composition.py`) | `SampledDataSystem(StepDiagramSystem)`; `compute_rollout`; `compute_trajectory` on the runner; the caches go | at the ticks: identical to the runner (its oracle) | low |
| `simulation/discrete/clock_runner.py` (today `hybrid_simulator.py`) | one function over pure blocks; the adaptive plant between input changes; today's RK4 substeps as `solver="rk4"` | **intended change 5**: the adaptive default moves the dense numbers by the integration error | low |
| `planning/stepping.py` (new) | the lowering: `PlantWithCost`, `DiscreteProblem`, `ĉ_k`, the margins at the index, `N`, `γ` | n/a | the core of §6 |
| `planning/evaluator.py`, `planning/evaluation.py` | `run` → the scan over the lowering; `score_run` keeps the rules; the sampled face; `require_continuous` goes; `x_start` → `plant.x0` | **intended change 2**: the augmented-state `J` replaces the trapezoid (`4.7e-5` per step on the prototype's pendulum, §10.1); PID-tuning and robust demos re-pinned | medium: every evaluator test |
| `planning/problems.py` | one class; `x0=` point / set / law; `params=` laws; `disturbances=` sources; one `realize` | **intended change 3**: live disturbances replace held draws (the frozen-noise defect, KNOWN_ISSUES) | medium: student-facing rename (A2) |
| `planning/dynamic_programming/discretizer.py`, `dp.py` | successors and `ĉ_k` from the lowering (Euler default) | identical by construction (`g·dt` under Euler, §10.1) | low |
| `learning/reinforcement_learning/environment.py`, `collect.py`, `policy.py` | the environment reads the lowering; `GaussianPolicy` the head; per-episode seeds | **intended change 4**: the RL numbers move once (seeds) | medium: training-time regressions |
| `blocks/sources.py`, `control/neural.py` | `NoiseSource`; `GaussianPolicy`; `TrajectorySource`'s table in `params` | identical | low |
| `optimization/mathematical_program.py`, `optimizer.py`, `evaluators/` | `params=None`; one evaluator body for `(z, p)`; `backend=`, `adapter`; the gate's benchmark | identical for one-argument programs | low |
| `optimization/kernels.py` (new), `optimizers/first_order.py` | `GradientDescent`, `Adam` as `init` / `step`; the Python loop runs the kernel; `iterations=K` the scan | identical for `gd` (same arithmetic), `adam` `rtol 1e-6` | low |
| `planning/trajectory_optimization/planner.py`, `direct_collocation.py`, `shooting.py`, `parametric_*.py` | `compile()` → the program with `params`; `plan`, `shift`, `unpack`; the parametric twin goes; shooting's score lowered to `ĉ_k`; `method="gd", iterations=` | the MPC baselines identical with only `x0` rebound; shooting's reported score an intended change (with 2) | medium: the transcriptions' closures over `params` |
| `planning/trajectory_optimization/mpc/` | one `ModelPredictiveController(StepSystem)`; the latch, `Command`, `export_to_computer`, the dual-rate computer go (a slow block is `fire=`); `sample_period`; `parameter_ports` | **R123's one-period delay**: the MPC demos re-pinned with a closeness report | medium |
| `analysis/discretization.py` | SciPy import made lazy (`"exact"` only); `sample_period` set | identical | low |
| `simulation/realtime/` | calls the clock runner's tick | identical | low (the realtime review stays v1.0) |
| CONSTITUTION, DESIGN §4–§6, RULES 2.3 | the amendments of §8's last row; the contract and its exceptions | — | maintainer-owned |

**A proposed order of work** (three lanes as core-architecture §10, with my joins):

1. **Objects:** `core/params.py`; the contract test and the five resolutions (sets, costs,
   distributions with `params`); `Field` and the shaping; `x0` views (R124) with `plant.x0`.
   Every step byte-identical.
2. **The step side:** `sample_period`; `StepRollout` → `Trajectory`; the operators on step
   systems; `Computer` in `core/` with the pure tick (change 1, re-pinned); `SampledDataSystem`
   and the clock runner (change 5).
3. **The lowering:** `planning/stepping.py`; the evaluator on it (change 2) with the sampled
   face; DP and RL reading it (identical, then change 4 with the seeds); one problem class
   with sources (change 3).
4. **The program:** `MathematicalProgram(params)` with the gate; the planner's four verbs; the
   kernels; collocation and shooting on `params`; the MPC as one block with the delay.
5. **The join:** `method="gd", iterations=K` on single shooting; the native MPC's closed loop
   under `problem.compile`; the §11 scripts as tests; the notebooks (R47); the merge.

Each intended change is one commit with its ledger line, in the sequence reviewer C gave so a
number moves once: the same-tick semantic, then the MPC delay, then the hybrid default; the
per-step cost before any RL re-pin.

---

## 10. What I verified

**Prototype** (`/tmp/claude-0/-home-user-minilink/36350854-ab7e-51d8-8262-d68717b27e02/scratchpad/fable-arch-A/proto.py`,
run with `PYTHONPATH=/home/user/minilink python3`, NumPy 2.5.3, minilink at the reviewed commit;
SciPy and JAX are not installed, so `discretize` was replaced by the step model's own
`step_body`, since `analysis/discretization.py:6` imports SciPy at module level). Every function
is written as a `lax.scan` body: a carry, a row, `np.where` for the selects, fixed shapes. Run
twice; the two outputs `cmp` byte-identical. Its output
(`.../fable-arch-A/proto_output.txt`):

1. **The cost-augmented step** (A3): on the pendulum with `QuadraticCost`, `ρ = 0.2`,
   `dt = 0.05`: RK4 gives `ĉ_k = 0.02571576`; Euler gives `0.02575000`, equal to `g·dt =
   0.02575000` (value iteration's and RL's rule, asserted); the trapezoid on the samples
   (today's `score_run`) gives `0.02576254`, `4.7e-5` from the RK4 integral: the size of
   intended change 2 per step.
2. **The pure multi-rate computer tick** (R125, §6.2): a `PID(ports="reference")` discretized
   by RK4 at 10 ms feeding the plant (2 substeps), with a setpoint filter firing every 4 ticks
   whose output is held as a state and read within the tick; 300 ticks: `θ = 0.372654` at
   3 s toward 0.5. The Python orchestrator over the same pure blocks (a branch instead of a
   select) ends at a state with `max |diff| = 0.0`: the step diagram is the runner's oracle to
   the bit (asserted with `array_equal`).
3. **One scan scores the loop** (§6.8): carry `(X, J, γᵏ, m)` over the same 300 ticks, the
   cost integrated with the plant's substeps, the margin to a `BoxSet` at every sample:
   `J = 6.237830`, least margin `8.8711`, not failed. The draw machinery of today:
   `child_seed(child_seed(1, "trial 0"), "noise") = 529177350` and trial 1 `696411125`
   (distinct), and `standard_normal(seed, k=3, 1)` twice `[-0.608192]` (repeatable).
4. **The native kernel** (R132): `GradientDescent` as `init` / `step` on a parametric quadratic
   `J(z, p)`, 30 steps: `|z_30 − z*| = 4.95e-3`; the step's Jacobian eigenvalues `0.8, 0.2`
   inside the unit circle (convergence as stability); the same kernel rebound with a new `q`
   converges to the new `z*` with no rebuild.
5. **The fixed-iteration MPC** (§5.6): single shooting, `H = 10`, `K = 20` projected gradient
   steps, `η = 1.0`, the command read from the state (one-period delay), the closed loop as a
   scan of 60 ticks: `θ = 0.511, 0.505, 0.504` at 1, 2, 3 s toward 0.5; the closed-loop score
   under the problem's cost `8.7671`; `dJ/d(controller.cost.Q[0,0]) = +0.0029` by central
   differences, the tuning gradient `jax.grad` would return in one call.

**Facts checked in the code** (all at `082103c`): `System.__init__` fields and `realize`
(`core/system.py:77–131, 190`); `%` importing the simulation band (`:452–459`);
`StepSystem.step` (`core/discrete/step_system.py:62`); `Computer.tick` reading the committed
buffer (`simulation/discrete/computer.py:114–171`), NumPy only (`:68–75`);
`HybridDiagram.compute_trajectory` returning a `HybridSimResult` (`core/discrete/hybrid_diagram.py:103`);
`discretize` (`analysis/discretization.py:21`); `step_body` and `iterate`'s `lax.scan`
(`core/compile/evaluators/stepping.py:38, 66`); the continuous `rollout` and `rollout_batch`
(`core/compile/evaluators/base.py:110, 146`); the discrete `rollout` returning a NumPy
`StepRollout` (`core/compile/evaluators/discrete.py:39`); `ProblemEvaluator` (`planning/evaluator.py:77–125,
146–195, 355–410`), `require_continuous` (`:789`), the four tree helpers (`:976–1046`);
`score_run`'s trapezoid (`planning/evaluation.py:125–222`); `PlanningProblem.__init__` with
`x_start` / `X0` (`planning/problems.py:111`) and the stochastic sibling (`:551`);
`MathematicalProgram` with `J(z)` (`optimization/mathematical_program.py:26`); `Optimizer`
with `compile_backend` and `.backend` (`optimization/optimizer.py:118–156`); the first-order
Python loop and `Adam.init` / `update` (`optimization/optimizers/first_order.py:56, 155, 167`);
`StateField.as_constraint` / `as_cost` (`planning/spatial/state_fields.py:33, 39`); the MPC's
mixin, latch and `dt_mpc` (`planning/trajectory_optimization/mpc/controller.py:59, 567`,
`mpc/predictive.py:14`); `WhiteNoise.h` from `(seed, ⌊t/Δ⌋)` (`blocks/sources.py:257–290`);
the RL reward `−g·dt` (`learning/reinforcement_learning/environment.py:162`); DP's `g·dt`
(`planning/dynamic_programming/dp.py:270, 390`); `lqr`, `lqg`, `bode`, `margins`
(`control/design.py:78, 135`, `analysis/frequency.py:93, 123`).

**Untested on JAX, and why I believe it traces:** every body above uses only array
arithmetic, `where`, `clip`, `concatenate` and integer arithmetic on `k` and `seed`; the
loops are `for` over a fixed range with a carry (a `scan` by `iterate`'s own rule, which is
already a `lax.scan` on JAX at `stepping.py:66`); no shape depends on a value; `params` is a
dict of arrays (a pytree). The two places that need care are named in §7.5: reverse mode
through a scan whose body calls `grad` (the native MPC under `jax.grad`; supported, with
`checkpoint` for memory) and the `n_trials` axis (a static shape, one compile per value, as
today).

---

## 11. Open questions I could not settle

1. **The margins between samples** (A3, point 8): check the sets at the samples only (my
   proposal for v0.3) or let the plant's step also return the least margin over its substeps.
   Recommendation: samples only now; the substep margin is a one-line extension of
   `PlantWithCost` later, and a user who needs it halves `dt`.
2. **One problem class** (A2): the maintainer's call, student-facing. Recommendation: merge,
   timed with the notebook move; otherwise keep the sibling and everything else stands.
3. **The transparent one-block computer:** `pid % dt` exposing the PID's leaves directly under
   `controller` (the `Discretization` precedent) keeps `wrt=["controller.Kp"]`; the alternative
   nests one level (`controller.PID.Kp`) for uniformity with multi-block computers.
   Recommendation: transparent, as `discretize` already is.
4. **The native MPC under `jax.grad`:** whether the higher-order path (grad over a scan of
   grads) meets the memory budget on the cart-pole without `checkpoint`. Recommendation: land
   the tick with `jax.checkpoint` on its body from the start; measure on the pendulum and the
   cart-pole (core-architecture §11.5).
5. **`as_cost(terminal=)`:** a flag on one verb, or a second verb `as_terminal_cost()`.
   Recommendation: the flag (fewest verbs; it is structural, never in `params`).
6. **Diffrax for the clock runner's plant solver on JAX** (R139.a): not needed for v0.3 since
   `compute_trajectory` is a NumPy face by design; a benchmark decides later.
7. **The RL heads' rebuild on `GaussianPolicy`:** with the block (one re-pin) or in v0.4
   (two). Recommendation: with the block, since the collectors' seeds move the numbers anyway.
8. **The `Trajectory.plot` shortcut** (§6.3): a small addition outside the core's path;
   drop it if the plotting lane prefers the system facades only.
