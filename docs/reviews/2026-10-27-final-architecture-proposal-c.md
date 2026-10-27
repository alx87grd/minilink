# The final core architecture and API of minilink: proposal C, the user-experience lens

**Date:** 2026-10-27
**Audience:** Prof. Alexandre Girard (Maintainer)
**Status:** an independent proposal, not a ruling. One of three written in parallel; the
others were not read.
**Commit reviewed:** `082103cbf4c62f627ffd416f9b66eaa4aae5998e` (`dev-v0.3`).
**Lens:** what a student writes in lesson 1 and what a researcher writes at the frontier,
with no second dialect between them. Everything is still shown to trace under JAX on the
main math path and to run efficiently.
**Prototypes:** `/tmp/claude-0/-home-user-minilink/36350854-ab7e-51d8-8262-d68717b27e02/scratchpad/fable-arch-C/`
(NumPy only; SciPy and JAX are not installed on this machine; §10 says what ran and what did not).

---

## 1. The aim in one screen

One toolbox, one model. A student writes a plant as `f(x, u, t; p)`, closes a loop with
`@`, simulates, plots, linearizes, designs a PID or an LQR. A researcher takes the *same*
objects and asks for a Monte Carlo over noise and parameter draws, the gradient of a
closed-loop cost with respect to a gain or a plant parameter, a digital controller on a
computer clock, an MPC whose closed loop compiles and differentiates, a stochastic policy
trained by RL and scored by the same number as everything else. The v0.3 core is the set
of objects and verbs that makes the second sentence true without breaking the first.

The spine, as the Constitution has it: a `System`, compiled, rolled out to a `Trajectory`,
scored by a problem's cost into an `Evaluation`, lowered to a `MathematicalProgram`, solved
by the `Optimizer`. v0.3 completes the spine with:

- **one contract** on every math object: a function of its own arguments and `params`,
  pure and deterministic given its seed, so `jit`, `grad`, `vmap` and a parameter family
  reach every object the way they reach `f`;
- **one score**: every loop that steps, once `dt` and the scheme are set, is one discrete
  system scored by one scan, so a policy has one `J` whether value iteration, RL, the
  evaluator or a digital loop computed it;
- **one step algebra** that holds the digital loop: `pid % 0.01 @ plant` is a step system at
  its base clock; `compute_rollout` is its pure run, `compute_trajectory` its dense run;
- **the draws** as data: starts, parameters and block seeds realized once from one key,
  disturbances as source blocks, a seeded Gaussian policy that is a plain block;
- **one program noun** with optional parameters, rebound without recompiling, and a native
  fixed-iteration solver so an MPC's closed loop is one scan, tuned by gradient;
- **a minimal `Field`**, the last textbook noun, so a value function is a terminal cost and a
  Lyapunov `V`.

The maintainer's preliminary choices of tonight (R124–R140) already draw most of this. This
proposal keeps them where they serve the user, adjusts a few (§3), and spells out the one open
investigation (one score) as a concrete design with a prototype behind it.

## 2. Reading the vision critically: what the user feels today

Three frictions a user hits today, each with the design that removes it:

1. **Three or four `J`s for one policy.** Value iteration and RL sum `g·dt`, the evaluator
   takes a trapezoid on the samples, shooting its own trapezoid with a first-order hold
   (`planning/dynamic_programming/dp.py:308`, `learning/reinforcement_learning/environment.py:130`,
   `planning/evaluation.py:125`). A student comparing VI, LQR and PPO sees numbers that do not
   agree and cannot say why. → **One lowering, `problem.discretize(dt)`, one per-step cost,
   one scan** (§6.9).
2. **Two dialects for the digital loop.** A continuous controller closes with `@`; a sampled
   one needs `hybrid_closed_loop(...)` or `% dt @`, returns a `HybridDiagram` that is not a
   `System`, cannot be evaluated by `problem.compile` (`planning/evaluator.py:789`), runs on
   NumPy only (`simulation/discrete/computer.py:34`) and has a one-tick lag the continuous
   diagram does not (`computer.py:114`). → **`%` gives a step system, `@` closes it, the
   result is a `System` of the step algebra, and the same `problem.compile` scores it** (§6.1–§6.3).
3. **Randomness that is not data.** The RL heads draw from their own keys, `realize` returns
   three shapes, a disturbance on a port is one held draw, and one key gives different trials
   on NumPy and JAX (KNOWN_ISSUES). → **Every draw a function of a seed in `params`; one
   `realize(key, n_trials) -> (x0s, params)`; a `GaussianPolicy` block** (§6.5–§6.7).

## 3. Adjustments to the vision

Each with its reason, its cost, and the preliminary choice it touches. Everything not listed
here is taken as the maintainer chose it.

| # | Adjustment | Reason | Cost | Touches |
| --- | --- | --- | --- | --- |
| A1 | **`block % dt` returns a step system** (a `Computer`, which *is* a `StepDiagramSystem` carrying its schedule), and **a step system that knows its period closes on a continuous plant directly**: `mpc @ plant`, `policy @ plant` with `sample_period` set, `pid % 0.01 @ plant` otherwise. `@` on a continuous plant with a step left operand means `left @ discretize(plant, dt_base, substeps=auto)`. | An MPC and a seeded policy carry their period already; writing it twice is the trap `export_to_computer` refuses today ("a `dt` other than `dt_mpc`"). One rule, read by `@`: "the left side sets the clock". | `@` gains one branch; `%` stays the explicit act for blocks with no period and for multi-rate computers. | R127 (confirms), DESIGN §4 "meets a continuous plant only through a Computer" (relaxes) |
| A2 | **The stochastic problem's keywords:** `x_start=` takes a point *or a law*; `params=` is the tree of laws over the problem's own tree (`{"plant": {"m": Uniform(0.9, 1.1)}}`). `x0_distribution` and `params_distribution` go. | R124 says the problem's start is "a point, a `SingletonSet` or a `Distribution`": one keyword, three kinds. "Where a number goes, a law may go" is the same rule the one contract gives `params`. | A sweep of two GRO860 notebooks (`pendulum_value_iteration_vs_lqr_vs_rl`, `drone_ppo`) and the RL demos; no alias (R82). **[ask — student-facing]** | DESIGN §6 (not an R) |
| A3 | **One score has a small public noun, `StepProblem`,** returned by `problem.discretize(dt, solver=, substeps=, episode_length=)`. Value iteration, the RL environment and the problem evaluator all read it; a researcher can print it. The `PlanningProblem` stays continuous and primary. | Three tools need the same five things (`transition`, `N`, `gamma`, the sets at `t_k`, the price); a noun with one constructor is simpler than three private lowerings that must agree. Reviewers A and B wanted no new noun; C wanted it internal. A frozen record a student never writes costs one docstring. | One more class on the research lane (not on the prelude). | The one-score investigation (QUESTIONS) |
| A4 | **`Field.as_cost(weight=, shaping=, terminal=False)`:** `terminal=True` puts the shaped field on `h` instead of `g`. | A value function as a terminal cost is the first consumer R133 names; a second verb (`as_terminal_cost`) would be a fourth conversion name. | One keyword. | R133 (adds a keyword) |
| A5 | **The computer moves into the core:** `core/discrete/computer.py` holds the pure `Computer` (a `StepDiagramSystem` with a `StepSchedule`) and `core/discrete/discretization.py` holds `discretize`; `simulation/discrete/` keeps only the clock orchestrator (`HybridSimulator`, runner 2). The lazy `core → simulation` import (R100) goes. | R127 makes the loop a `System` of the step algebra; a `System` cannot live in a tool package. | Two moves, one import exception deleted. | R100, R127 |
| A6 | **`GaussianPolicy(mean, log_std, *, sample_period, seed=None)`: `mean` is a block** (any `u = μ(x)` `System`: `StateFeedback`, `NeuralPolicy`, a `LookupPolicy`), `log_std` an array or a block. | The lesson "exploration noise around the LQR gain" and the RL head are the same object; the mean block already has its own `params` and `plot_control_law`. | None beyond R129; a precision. | R129 (precises) |
| A7 | **The RL planner's `gamma` is derived,** `γ = e^{−ρ dt}` from the problem's `discount_rate` and the training `dt` (the `StepProblem`'s `gamma`); `gamma=` is no longer a constructor argument. | One owner per quantity (RULES 5.6); the one-score `J` and the training return must agree on the discount. | The RL notebooks set `discount_rate=−ln(0.97)/DT` instead of `gamma=0.97`; their numbers re-pin once with the seeded policy. **[ask — student-facing]** | DESIGN §6 |
| A8 | **The MPC block's `params` holds its planner's problem tree under its own keys** (`controller.cost.Q`, `controller.plant.m`, `controller.step_size`, `controller.x0` the initial plan). | Model mismatch becomes explicit: `pe.params["plant"]["m"]` is the real plant, `pe.params["controller"]["plant"]["m"]` the model inside the MPC; tuning a weight is `wrt=["controller.cost.Q"]`. | None; it is what "every number a leaf" means for a block that holds a problem. | R130, R135 (precises) |
| A9 | **`compute_rollout` returns a `Trajectory`** with `t = k·dt` (the `StepRollout` fold), and on a loop with a clock it takes `tf=`. | One run object everywhere (CONSTITUTION §2). | Already a row (V1.1's rest). | — |
| A10 | **Keep `HybridDiagram` as the name now; recommend `SampledDataSystem` at the v0.9 name pass.** | The textbook term; a rename today costs a notebook sweep for no v0.3 gain. | None now. | R127 (confirms) |

Not adjusted, on purpose: propagation within the tick (R125), the operators on step systems
(R126), the three owners of a start (R124), disturbances as source blocks and one `realize`
shape (R128), one program noun with `params=None` (R130), the program convention (R131), the
kernel with a step-system view (R132), the names (R134), the generic planner interface with
the loop mechanics in the MPC (R135), the basic `PlanningSolution` (R138), the one contract
(R140). Each is used below as written.

## 4. The user's view: the main objects and verbs on one page

"The textbook" column says what the object is in a control course; "verbs" the methods a
user calls; "`params`" the leaves the one contract exposes (every leaf a float or an array,
`seed` the one integer).

| Object | In the textbook | Verbs | `params` |
| --- | --- | --- | --- |
| `System` / `DynamicSystem` | ẋ = f(x, u, t; p), y = h(x, u, t; p), poses `tf` | `f`, `h`, `tf`; `compute_trajectory`, `compute_forced`, `linearize`, `jacobian`, `plot_*`, `animate`; `@`, `>>`, `+`, `%`; `compile`, `realize(key)` | the plant's numbers (`m`, `l`, …); `x0` is a field, not a leaf, until the evaluator reads it as the nominal |
| `DiagramSystem` | the block diagram, ẋ = F(X, U, t; P) | the same, plus `connect`, `expose`, `trajectory_of(block)`; `x0` a view of its blocks (write-through) | `{block_id: block.params}` |
| `StepSystem` / `StepDiagramSystem` | x_{k+1} = Φ(x_k, u_k, k; p), the difference equation | `step`, `h`; `compute_rollout`, `jacobian`, `plot_rollout`; `@`, `>>`, `+` (same kind only) | the block's numbers; `sample_period` a field |
| `Computer` | the controller as it runs on a computer: blocks at rates on one base clock | a `StepDiagramSystem` plus `schedule`; built by `block % dt`, `computer(diagram, schedule)` | `{block_id: …}` plus each held output's memory as state |
| `HybridDiagram` | the sampled-data loop: computer + ZOH + plant + sampler, at the base clock | `compute_rollout(tf)` (pure, the ticks), `compute_trajectory(tf, solver=)` (dense), `plot_trajectory`, `plot_diagram`, `animate`, `jacobian` | `{"controller": …, "plant": …}` |
| `discretize(plant, dt)` | the plant one period later, input held: x_{k+1} = Φ(x_k, u_k) | a `StepSystem` with `dt`, `source` | the plant's, live |
| `Trajectory` | the sampled run (t, x, u), with the loop's signals | `of(role)`, `with_cost`, `resample`, `plot`, `save` | — (a record) |
| `Set` / `InputSet` | z ∈ Z(t), written margin(z, t) ≥ 0 | `margin`, `contains`, `sample(key, n)`, `bounding_box`, `&`, `as_cost` | the set's numbers (`lower`, `upper`, `center`, `radius`); `{}` when none |
| `CostFunction` | J = ∫ e^{−ρt} g dt + e^{−ρ tf} h | `g`, `h`, `+`, `*`, `integrate` | `Q`, `R`, `S`, `xbar`, `ubar`; a sum nests by term |
| `Distribution` | a law over a vector | `sample(key, n, params)`, `mean` | the law's numbers (`mean`, `std`, `lower`, `upper`) |
| sources (`Step`, `Sine`, `WhiteNoise`, `TrajectorySource`) | the signal generators | `h`; a random one has `realize(key)` | levels, `psd`, `sample_period`, `seed`; a table as `values` |
| `Field` | a scalar on the model's arguments: V(x), J(x, t), Q(x, u), a barrier h(x) | `value`, `gradient`, `as_set`, `as_input_set`, `as_cost` | `center`, `P` for the quadratic one |
| `PlanningProblem` | the task: system, sets, cost, horizon, discount, start | `discretize(dt)`, `compile(controller, dt)`, `realize(key, n_trials)`, `replace`, `cost_bound` | `plant`, `cost`, `X`, `U`, `X0`, `Xf`, `x_start`, `discount_rate` |
| `StochasticPlanningProblem` | the same with laws on the start and the parameters | the same; `nominal()` | the same tree, the laws' numbers nested under their roles |
| `StepProblem` | the task at `dt`: Φ_c, the sets at t_k, N, γ | `transition`, `margin`, `terminal`, `realize` | the problem's tree |
| `ProblemEvaluator` | the closed loop compiled and scored | `trajectory(params)`, `evaluate(params, key, n_trials)`, `program(wrt)`, `pack`, `unpack` | the whole run: the loop's blocks, the problem's leaves, the free inputs, each stateful block's `x0` |
| `Evaluation` | the report: cost, J, margins, failed, reached, per trial | `objective`, `mean`, `std`, `worst`, `cvar`, rates; `print` | — |
| `MathematicalProgram` | min J(z; p) s.t. h = 0, g ≥ 0, bounds | `J(z, params)`, `h`, `g`, `params_at(z)` | the problem's leaves not in `z` (`x_start`, `plant`, `cost`, `seed`) |
| `Optimizer` | the solver | `solve(z0=, params=, iterations=)`, `as_step_system()` | `step_size`, the method's numbers |
| `Planner` (`LQRPlanner`, `DynamicProgrammingPlanner`, `TrajectoryOptimizationPlanner`, `RRTPlanner`, `ReinforcementLearningPlanner`) | the method | `solve(params=, z0=)`, `compile()`, `shift(z, n)` where it applies; `evaluate` | — (reads the problem's) |
| `PlanningSolution` | the answer and its evidence | `policy`, `trajectory`, `cost_to_go`, `evaluation`, `success`, `solver`; `plot_control_law`, `plot_cost_to_go`, `plot_trajectory` | — |
| `GaussianPolicy` | u = μ(x) + σ ε_k, the stochastic policy | `h`, `log_prob`, `entropy`; `realize(key)` | `mean` (the block's tree), `log_std`, `seed` |
| `ModelPredictiveController` | the receding-horizon law on a computer | a `StepSystem`: `step`, `h`; `@ plant` | the planner's problem tree, `step_size`, `x0` (the first plan) |
| `compare` / `evaluate` | the facades | `evaluate(problem, ctl, dt=, n_trials=, key=)`, `compare(problem, A=, B=, dt=)` | — |

The one rule under the table: **an object's math is `method(args, params=None)`; `params`
is a dict whose leaves are numbers (a `seed` the one integer); a composite's tree nests its
parts by name; a dotted path walks it (`controller.Kp`, `cost.Q`), a colon names a signal
(`plant:u`); and the same call gives the same numbers on NumPy and JAX for the same seed.**

## 5. The pipelines, as user scripts

Every script below is complete in the final API. Each says what prints, what plots, the
error a wrong call raises, and the `params` tree the user sees. The first two are today's
API unchanged (the student syntax never breaks); the rest are v0.3.

### 5.1 Lesson 1: a pendulum, a PID, simulate, plot (unchanged)

```python
import numpy as np
from minilink import Pendulum, PID

plant = Pendulum()                 # ẋ = f(x, u, t; p), the catalog's
plant.x0 = np.array([0.5, 0.0])    # the start: the block owns it

loop = PID(Kp=20.0, Ki=5.0, Kd=4.0) @ plant   # e = r − y inserted; sides named controller, plant
loop.inputs["r"].nominal_value = np.array([np.pi])

traj = loop.compute_trajectory(tf=5.0)        # the adaptive SciPy solver, 10 001 points
loop.plot_trajectory()                        # θ, θ̇ and the torque against t
loop.plot_diagram()
print(loop.params)
```

Prints (the solver line, then the tree):

```text
[simulate] solver=scipy  t ∈ [0, 5]  10001 points  compile_backend=numpy
{'controller': {'Kp': array([[20.]]), 'Ki': array([5.]), 'Kd': array([[4.]]), 'tau': array([0.1]),
                'u_min': array([-inf]), 'u_max': array([inf]), 'e_int_min': ..., 'e_int_max': ...},
 'plant': {'m': 1.0, 'l': 1.0, 'I': 1.0, 'gravity': 9.81, 'd': 0.0}}
```

A wrong call: `PID(20.0) @ 2.0` → `TypeError: '@' means controller at plant and needs a
System on the right; a gain is Gain(2.0)`. `plant.x0 = [1, 2, 3]` → `ValueError: x0 has 3
entries; Pendulum has 2 states`.

### 5.2 Classical analysis: linearize, poles, Bode, margins (unchanged)

```python
from minilink import Pendulum, PID, linearize, pzmap, bode, margins, step_info

plant = Pendulum()
ctl = PID(20.0, 5.0, 4.0)
x_up = np.array([np.pi, 0.0])

G = linearize(plant, x_up)         # the LTISystem (A, B, C, D) at the upright point
L = ctl >> plant                   # the loop gain C(s) G(s), an open-loop diagram
T = ctl @ plant                    # the closed loop

print(pzmap(T, x_up))              # closed-loop poles and zeros, a table
bode(L, x_up, show=True)           # magnitude and phase of L(jω)
print(margins(L, x_up))            # gain margin, phase margin, their crossover frequencies
print(step_info(*T.step_response(x_up)))   # rise time, overshoot, settling, final value
```

Prints: `Poles: -1.2 ± 3.4j, -8.7 ...`, `GM = 12.1 dB at 4.3 rad/s, PM = 51° at 1.9 rad/s`,
the step table. Plots: the pole-zero map, the Bode pair, the step. A wrong call:
`bode(plant)` without a point on a nonlinear plant → it linearizes at the nominal
(`state.nominal_value`), which the printout names; `margins(T, x_up)` on the closed loop →
`ValueError: margins read a loop gain; pass the open loop (ctl >> plant)`.

### 5.3 State space: LQG and a Monte Carlo over noise and parameter draws with one key

```python
import numpy as np
from minilink import (Pendulum, lqg, WhiteNoise, Gaussian, Uniform, QuadraticCost,
                      StochasticPlanningProblem)

plant = Pendulum()
x_up = np.array([np.pi, 0.0])
Q, R = np.diag([10.0, 1.0]), np.array([[0.1]])
W, V = 0.05, 1e-3                                   # process and measurement intensities

ctl = lqg(plant, Q, R, Q_n=W, R_n=V, x_bar=x_up)   # a box: observer + gain, ports r, y → u

problem = StochasticPlanningProblem(
    plant,
    x_start=Gaussian(x_up + [0.3, 0.0], std=[0.1, 0.1]),       # the start is a law (A2)
    params={"plant": {"m": Uniform(0.8, 1.2)}},                 # a law where a number goes
    disturbances={"d": WhiteNoise(1, psd=W, sample_period=0.001),
                  "n": WhiteNoise(2, psd=V, sample_period=0.001)},   # source blocks (R128)
    cost=QuadraticCost.from_system(plant, Q=Q, R=R, xbar=x_up),
    X=plant.state.box, tf=4.0,
)

pe = problem.compile(ctl, dt=0.001)                 # the loop built, lowered at dt, compiled once
report = pe.evaluate(n_trials=200, key=1)           # 200 realizations from one key, one vmap on JAX
print(report)
print(pe.params["controller"]["observer"]["x0"])   # the observer's own start, a leaf (R124)

runs = pe.evaluate(n_trials=8, key=1, record=True)  # the same first 8 draws, trajectories kept
pe.loop.plot_trajectory(runs.trajectories[0], signals=("plant:x", "controller.observer:x"))
```

Prints:

```text
Evaluation: 200 trials, criterion expectation
  J        mean 2.413  std 0.611  worst 4.902  cvar(0.1) 3.96
  cost     mean 2.413  (no price: infeasible_cost=None)
  failed   0.0 %   reached  —   success 100.0 %
```

Then the observer's `x0` (`[π, 0]`, the block's). Plots: the plant's state and the estimate
of the first trial. The `params` tree of the run, `pe.params`:

```text
{'controller': {'observer': {'A':…, 'B':…, 'C':…, 'D':…, 'L':…, 'xbar':…, 'ubar':…, 'x0': [3.1416, 0.]},
                'gain': {'K':…, 'ubar':…}},
 'plant': {'m': 1.0, 'l': 1.0, 'I': 1.0, 'gravity': 9.81, 'd': 0.0},
 'd': {'psd': 0.05, 'sample_period': 0.001, 'seed': 0},
 'n': {'psd': 0.001, 'sample_period': 0.001, 'seed': 0},
 'cost': {'Q':…, 'R':…, 'S':…, 'xbar':…, 'ubar':…},
 'X': {'lower':…, 'upper':…},
 'x_start': [3.4416, 0.], 'discount_rate': 0.0,
 'r': [3.1416, 0.]}                         # the free loop input, held at its nominal
```

`problem.realize(1, 200)` returns `(x0s (200, 2), params)` where `params["plant"]["m"]` has
shape `(200,)`, `params["d"]["seed"]` and `params["n"]["seed"]` are `(200,)` integer arrays,
and every other leaf is shared. The same key gives the same draws on both backends (the
counter-based cipher, `core/distributions.py:230`).

Wrong calls: `disturbances={"d": Gaussian(0, 0.1)}` → `TypeError: a disturbance is a signal,
not a law: WhiteNoise(1, psd=0.01, sample_period=0.001), a Step, or any source block`.
`pe.evaluate()` on a stochastic problem without `n_trials` → `ValueError: a stochastic
problem needs n_trials (and a key); pe.evaluate(n_trials=100, key=0)`. `lqg(...) @ plant`
when the plant declares no `y` → `ValueError: an observer reads the plant's y; Pendulum
declares none`.

### 5.4 The digital loop: `pid % dt @ plant`, two runners, one score, the gradient of J

```python
import jax, numpy as np
from minilink import Pendulum, PID, PlanningProblem, QuadraticCost

plant = Pendulum()
x_up = np.array([np.pi, 0.0])
plant.x0 = x_up + [0.4, 0.0]
pid = PID(20.0, 5.0, 4.0)

digital = pid % 0.01                 # the PID as it runs on a computer at 100 Hz: a step system
loop = digital @ plant               # a HybridDiagram: computer, ZOH, plant, sampler, at dt_base = 0.01
loop.inputs["r"].nominal_value = x_up[:1]
print(loop)

ticks = loop.compute_rollout(tf=3.0)                      # runner 1: the ticks, pure, traced
dense = loop.compute_trajectory(tf=3.0, solver="scipy")   # runner 2: the plant integrated between ticks
loop.plot_trajectory(ticks, signals=("plant:y", "controller:u"))
loop.plot_trajectory(dense, signals=("plant:y", "controller:u"))

problem = PlanningProblem(plant, cost=QuadraticCost.from_system(plant, xbar=x_up), tf=3.0)
pe = problem.compile(loop)                                # dt is the loop's base clock
print(pe.evaluate())                                      # the same ticks as compute_rollout, scored
dJ = jax.grad(lambda p: pe.evaluate(p).objective)(pe.params)
print(dJ["controller"]["pid"]["Kp"], dJ["plant"]["m"])    # the gradient reaches every float leaf
```

Prints:

```text
HybridDiagram 'PID % 0.01 @ Pendulum'  base clock 0.01 s
  controller: Computer(pid @ 100 Hz)    states: e_int, m_filt, held u
  plant:      Discretized(Pendulum, dt=0.01, rk4, substeps=1)
Evaluation: 1 trial   J 1.8427   cost 1.8427   failed no
[[-0.31]]  0.0742
```

Plots: the sampled run (stairs on `controller:u`) and the dense one; they agree at the ticks
to the plant's integration error only. The `J` printed is the score of `compute_rollout`'s
samples, by construction (one kernel). The tree:

```text
{'controller': {'pid': {'Kp':…, 'Ki':…, 'Kd':…, 'tau':…, 'u_min':…, 'u_max':…, …, 'x0': [0., 0.]},
                'held': {'u': [0.]}},          # the ZOH memory, a state of the loop (R104)
 'plant': {'m': 1.0, …},
 'cost': {…}, 'x_start': [3.5416, 0.], 'discount_rate': 0.0, 'r': [3.1416]}
```

Wrong calls: `pid @ discretize(plant, 0.01)` → `TypeError: a continuous controller cannot
close on a step plant. Write pid % 0.01 @ plant (the PID on a computer), or
discretize(pid, 0.01) @ discretize(plant, 0.01)`. `problem.compile(loop, dt=0.02)` →
`ValueError: the loop runs at 0.01 s; drop dt or pass dt=0.01`. `pid % 0.01 @ plant` with a
`Pendulum` whose `d` port carries a `WhiteNoise` of `sample_period=0.003` → a warning: the
noise period does not divide the base clock.

### 5.5 Trajectory optimization and MPC: one program compiled once, rebound

```python
import numpy as np
from minilink import (CartPole, PlanningProblem, QuadraticCost, BallSet,
                      TrajectoryOptimizationPlanner, ModelPredictiveController, Optimizer)

plant = CartPole()
x_goal = np.array([0.0, np.pi, 0.0, 0.0])
problem = PlanningProblem(
    plant, x_start=np.zeros(4), x_goal=x_goal, Xf=BallSet(x_goal, 0.05),
    cost=QuadraticCost.from_system(plant, Q=np.diag([1, 10, 0.1, 0.1]), R=0.01 * np.eye(1), xbar=x_goal),
    U=plant.input_box, tf=2.0,
)

planner = TrajectoryOptimizationPlanner(problem, transcription="direct_collocation", n_steps=40,
                                        method="ipopt", backend="jax")
planner.compile()                                   # the program built once: J(z, params), h, g
sol = planner.solve()                               # params=None: the problem's own values
print(sol.solver)                                   # TrajectoryOptimizationRecord: success, iterations, residuals
sol.plot_trajectory()

program = planner.program                           # the MathematicalProgram with params
print(program)                                      # n_z=200, h: 160 rows, g: 80 rows, params: plant, cost, X, U, Xf, x_start
heavy = planner.solve(params={"plant": {"m_cart": 1.5}, "x_start": [0.2, 0, 0, 0]})   # rebound, no rebuild
z_star = Optimizer(program, method="ipopt").solve(params={"cost": {"Q": np.diag([1, 20, 0.1, 0.1])}}).z
print(program.params_at(z_star)["cost"]["Q"])       # the full tuned tree read back

mpc = ModelPredictiveController(planner, sample_period=0.05)   # a StepSystem; its state is the plan z
loop = mpc @ plant                                  # the left side sets the clock (A1)
traj = loop.compute_rollout(tf=3.0)                 # NumPy face: Ipopt is host code, a Python loop over ticks
loop.plot_trajectory(traj)
loop.animate(traj)                                  # the horizon overlays, from mpc's record
```

Prints the record (`success True, 23 iterations, max defect 3e-9`), the program's summary,
the rebound solve's record, `Q`. Plots the plan, then the closed loop with the receding
horizons drawn. The MPC's tree, `loop.params["controller"]`:

```text
{'plant': {'m_cart':…, 'm_pole':…, …},   # the model inside the MPC (may differ from loop.params['plant'])
 'cost': {'Q':…, 'R':…, 'S':…, 'xbar':…, 'ubar':…}, 'U': {…}, 'Xf': {…},
 'discount_rate': 0.0, 'x0': z0}         # the first plan, the block's start
```

Wrong calls: `planner.solve(params={"plant": {"mass": 2.0}})` → `KeyError: unknown
parameter 'mass' under plant; known: m_cart, m_pole, l, gravity, d`. `program.J(z,
params={"x_start": [0, 0]})` → `ValueError: x_start has 2 entries; CartPole has 4 states`.
`Optimizer(program, method="gd")` on a program with constraint rows → `ValueError: gd solves
unconstrained or box-bounded programs; this one has 160 equality rows (use ipopt or
scipy_slsqp)`. `mpc % 0.02` → `ValueError: this controller runs at 0.05 s; mpc @ plant uses
it, or build the MPC with sample_period=0.02`.

### 5.6 The native fixed-iteration MPC: its closed loop in one scan, tuned by gradient

```python
import jax, numpy as np
from minilink import (Pendulum, PlanningProblem, QuadraticCost, TrajectoryOptimizationPlanner,
                      ModelPredictiveController, Optimizer)

plant = Pendulum()
x_up = np.array([np.pi, 0.0])
cost = QuadraticCost.from_system(plant, Q=np.diag([1.0, 0.1]), R=0.01 * np.eye(1),
                                 S=np.diag([5.0, 1.0]), xbar=x_up)
model_problem = PlanningProblem(plant, cost=cost, U=plant.input_box, tf=0.75)

planner = TrajectoryOptimizationPlanner(model_problem, transcription="single_shooting", n_steps=15,
                                        method="gd", iterations=20, step_size=0.02, backend="jax")
mpc = ModelPredictiveController(planner, sample_period=0.05)    # z_{k+1} = S^K(shift(z_k); x0 = y_k)
loop = mpc @ plant                                              # one step diagram at 0.05 s

task = PlanningProblem(plant, x_start=x_up - [0.8, 0.0], cost=cost, tf=3.0)
pe = task.compile(loop)                                         # one lax.scan over 60 ticks, K = 20 inside each
report = pe.evaluate()
print(report.J)

# Monte Carlo over starts, one vmap
starts = x_up + np.array([[-0.8, 0], [-0.3, 0.5], [0.5, -0.5]])
print(pe.evaluate(params={"x_start": starts}).J)                # a batched leaf: three runs in one call

# Tune the MPC's own weight on θ by the closed-loop cost
program = pe.program(wrt=["controller.cost.Q"])                 # J(z) = pe.evaluate(unpack(z)).objective, ∇J by reverse mode
best = Optimizer(program, method="adam", iterations=200, step_size=0.05).solve()
print(program.params_at(best.z)["controller"]["cost"]["Q"])
```

Prints `J = 3.21`, then `[3.21, 1.07, 1.44]`, then the tuned `Q`. The gradient through the
MPC is exact for the controller that runs (K iterations, the shift, the projection on the
input box), by reverse mode through the scan; memory is one checkpoint per tick
(`jax.checkpoint` on the tick body). The tree is 5.5's with `step_size` and `x0` (the first
plan, zeros) under `controller`. Wrong calls: `method="gd", iterations=None` →
`ValueError: a native method inside a traced block needs a fixed iteration count`;
`pe.program(wrt=["controller.x0"])` → allowed (the first plan is a tunable start, R124);
`wrt=["d.seed"]` → `ValueError: d.seed is an integer leaf (a seed); it cannot be tuned`.

### 5.7 Value iteration and RL reading the same `J`; a `QuadraticField` as terminal cost and `V`

```python
import numpy as np
from minilink import (Pendulum, PlanningProblem, QuadraticCost, DynamicProgrammingPlanner,
                      LQRPlanner, ReinforcementLearningPlanner, QuadraticField, Uniform,
                      StochasticPlanningProblem, compare, evaluate)

plant = Pendulum()
x_up = np.array([np.pi, 0.0])
cost = QuadraticCost.from_system(plant, Q=np.diag([1.0, 0.1]), R=0.1 * np.eye(1), xbar=x_up)
problem = PlanningProblem(plant, x_start=np.zeros(2), cost=cost, X=plant.state.box,
                          U=plant.input_box, tf=np.inf, discount_rate=0.5)
DT = 0.05
sp = problem.discretize(DT, solver="euler", episode_length=10.0)   # the task at dt (A3)
print(sp)

lqr = LQRPlanner(problem).solve()
V = QuadraticField(x_up, lqr.solver.P)             # the LQR value xᵀ P x, a Field
region = V.as_set(upper=20.0)                       # the ellipsoid {x : V(x) ≤ 20}, a Set (samples, bounding box)

vi = DynamicProgrammingPlanner(problem, x_grid=(101, 101), u_grid=(11,), dt=DT).solve()   # reads sp.transition, sp.gamma
rl_problem = StochasticPlanningProblem(plant, x_start=Uniform([-np.pi, -6], [np.pi, 6]), cost=cost,
                                       X=plant.state.box, U=plant.input_box, tf=np.inf, discount_rate=0.5)
rl = ReinforcementLearningPlanner(rl_problem, dt=DT, algorithm="ppo", seed=0).solve(timesteps=400_000)

race = compare(problem, VI=vi, LQR=lqr, PPO=rl, dt=DT, solver="euler")   # one J, the same kernel for all
print(race)
race.plot_control_law(); race.plot_cost_to_go()

# A terminal cost from the LQR value in a finite-horizon problem, for trajectory optimization
finite = problem.replace(tf=2.0, cost=cost + V.as_cost(terminal=True))
print(evaluate(finite, lqr.policy, dt=DT).J)
```

Prints:

```text
StepProblem: dt 0.05, N 200 (episode 10 s, infinite horizon), γ 0.9753, rk4/euler, price 1.0e3
Comparison        J (evaluate)   cost     failed   record
  VI               12.41          12.41    no       ValueIteration: 412 sweeps, converged
  LQR              11.98          11.98    no       Riccati: Hurwitz
  PPO              12.07          12.07    no       PPO: 400k steps, 27 s
```

and VI's own table at the start, `vi.cost_to_go(x_start)`, equals its `J (evaluate)` row to
the interpolation error, because the planner's backup used `sp.transition` (the same `c_k =
g dt` under Euler) and the same `γ`. Plots: the three laws and cost-to-go fields on one
scale. Wrong calls: `DynamicProgrammingPlanner(problem)` with an unbounded `X` → `ValueError:
value iteration needs a finite state box: X=plant.state.box`; `V.as_cost()` on a
`QuadraticField` over `x` passed where an `InputSet` is needed → `TypeError`.

### 5.8 The research frontier: a stochastic hybrid loop, a seeded policy, vmapped over seeds, tuned by gradient

```python
import jax, numpy as np
from minilink import (Pendulum, NeuralPolicy, GaussianPolicy, WhiteNoise, Gaussian, Uniform,
                      StochasticPlanningProblem, QuadraticCost, Optimizer)

plant = Pendulum()
x_up = np.array([np.pi, 0.0])
mean = NeuralPolicy(plant, hidden=(32, 32), seed=0)                # u = μ_θ(x), weights in params["mlp"]
policy = GaussianPolicy(mean, log_std=np.log(0.3), sample_period=0.05, seed=None)   # u = μ + σ ε(seed, ⌊t/Δ⌋)

problem = StochasticPlanningProblem(
    plant, x_start=Gaussian(np.zeros(2), [0.5, 0.5]),
    params={"plant": {"m": Uniform(0.8, 1.2), "l": Uniform(0.9, 1.1)}},
    disturbances={"d": WhiteNoise(1, psd=0.02, sample_period=0.05)},
    cost=QuadraticCost.from_system(plant, xbar=x_up), X=plant.state.box, U=plant.input_box,
    tf=np.inf, discount_rate=0.5, infeasible_cost=True,
)
loop = policy @ plant                                              # the policy's period sets the clock (A1)
pe = problem.compile(loop, substeps=5, episode_length=8.0)        # 160 ticks, 5 RK4 substeps each

report = pe.evaluate(n_trials=256, key=3)                          # seeds for the policy and the load per trial, one vmap
print(report)

# Tune the mean's weights and the exploration scale on the Monte Carlo objective
program = pe.program(wrt=["controller.mean.mlp", "controller.log_std"], n_trials=256, key=3)
best = Optimizer(program, method="adam", iterations=500, step_size=1e-3).solve()
tuned = pe.unpack(best.z, program.metadata["wrt"])
print(pe.evaluate(tuned, n_trials=1000, key=4))                   # scored on fresh draws

# The same policy's log-probabilities along a run: the RL planner's own training signal
run = pe.trajectory(tuned, key=3).of("plant")
logp = jax.vmap(lambda x, u, t: policy.log_prob(x, u, t, tuned["controller"]))(run.x.T, run.u.T, run.t)
```

Prints the two evaluations (the second on new keys, the honest number) with their
feasibility rates. `pe.params["controller"]` is `{"mean": {"mlp": {...}, "u_mid":…, "u_half":…},
"log_std": -1.204, "seed": None}`; `realize` gives the seed a `(256,)` integer leaf and
`log_std` stays shared. Wrong calls: `wrt=["controller.seed"]` → refused as an integer leaf;
`pe.program(wrt=["x_start"])` → `ValueError: x_start is drawn on a stochastic problem; tune
the law's numbers instead (x_start.mean)`; `GaussianPolicy(mean, log_std=…, seed=1)` with no
`sample_period` → `TypeError: a seeded policy holds its draw over sample_period`.

### 5.9 One course notebook migrated: `pendulum_value_iteration_vs_lqr_vs_rl.ipynb`

What changes, cell by cell (the rest is identical):

| Cell today | Cell in v0.3 | Why |
| --- | --- | --- |
| `ppo_problem = StochasticPlanningProblem(plant, cost=cost, tf=np.inf, x0_distribution=Uniform(...), U=...)` | `... x_start=Uniform([-np.pi, -6.0], [np.pi, 6.0]), U=...` | A2 |
| `ReinforcementLearningPlanner(ppo_problem, dt=DT, ..., gamma=0.97, ...)` | `ppo_problem = StochasticPlanningProblem(..., discount_rate=-np.log(0.97) / DT)` and the planner without `gamma=` | A7: one owner of the discount |
| `race = compare(VI=sol_vi, LQR=sol_lqr, PPO=sol_ppo)` | `race = compare(problem, VI=sol_vi, LQR=sol_lqr, PPO=sol_ppo, dt=DT, solver="euler")` | the one score: each entry scored by the same kernel; `print(race)` gains the `J (evaluate)` column |
| `cl = ctl @ plant; traj = cl.compute_trajectory(tf=TF, n_steps=..., solver="euler")` | unchanged | the continuous simulation stays the lesson's picture |
| (new, optional) | `print(evaluate(problem, vi_ctl, dt=DT, solver="euler").J, sol_vi.cost_to_go(X0))` | the lesson: the table's number is the rollout's number |

Numbers: VI's table does not move (Euler, `g·dt`, `γ = e^{−ρ dt}`); the PPO numbers move once
when the seeded policy lands (the collectors on per-episode seeds), re-pinned with their
baseline; LQR unchanged. The cost cell `Q = np.diag([1.0 / DT, 0.1 / DT])` (a per-step weight
written as a rate) keeps working; the notebook may later drop the `/ DT` since the per-step
cost now integrates `g` over `dt`, a teaching choice for the maintainer.

## 6. The final API, object by object

For each object: the signatures, the contract, the `params` tree, what exists today (file and
line at the reviewed commit) and what changes.

### 6.1 `System`, `DynamicSystem`, `DiagramSystem`

```python
class System:                                   # core/system.py:26
    params: dict                                # {} when the block has none (never None)
    x0: np.ndarray                              # the block's own start (R124)
    def h(self, x, u, t=0.0, params=None): ...  # y
    def tf(self, x, u, t=0.0, params=None): ... # poses
    def realize(self, key): ...                 # params with a fresh seed per random block, by id
    def __matmul__(self, plant): ...            # controller @ plant; a step left side sets the clock (A1)
    def __rshift__(self, other): ...            # series
    def __add__(self, other): ...               # parallel
    def __mod__(self, dt_or_schedule): ...      # block % dt -> Computer (a step system at dt)

class DynamicSystem(System):                    # core/system.py:516
    def f(self, x, u, t=0.0, params=None): ...  # ẋ

class DiagramSystem(DynamicSystem):             # core/diagram.py:19
    x0: property                                # the blocks' x0 stacked; the setter writes through (R124)
    n, state_index: structural, cached          # recomputed when the wiring changes
    params: property                            # {block_id: block.params}, live (wiring.py:54, unchanged)
```

What changes: `x0` becomes a view (today `wiring.py:258` restacks on `refresh()` and keeps a
"user-set" copy; `derived_x0` and the `refresh()` call in `Simulator` go). `__mod__`
(`system.py:452`) returns the core's `Computer` instead of the simulation tool's. `__matmul__`
(`system.py:435`) accepts a step left operand on a continuous plant (A1) and refuses a
continuous left operand on a step plant, naming `%` and `discretize`. `params` of a block
with no parameters is `{}` (the five resolutions of R140). Everything else is unchanged.

### 6.2 `StepSystem`, `StepDiagramSystem`, the operators, `discretize`, `%`, `Computer`, `HybridDiagram`

```python
class StepSystem(System):                       # core/discrete/step_system.py:16
    sample_period: float | None                 # the block's own clock, None when it has none
    def step(self, x, u, k=0, params=None): ... # x_{k+1}
    def h(self, x, u, k=0, params=None): ...    # y_k
    def compute_rollout(self, n_steps=None, u=None, *, tf=None, x0=None, backend="numpy") -> Trajectory
    def jacobian(self, of, wrt, x=None, u=None, k=0, params=None)
    # @, >>, + : the same meaning as on the continuous algebra, same kind only (R126)

class StepDiagramSystem(StepSystem):            # core/discrete/step_diagram.py:13
    # add_subsystem refuses a DynamicSystem with states, naming discretize (unchanged)
    # step: every block propagated within the tick (R125); an algebraic loop refused at wiring

def discretize(plant, dt, *, solver="rk4", substeps=1) -> StepSystem   # core/discrete/discretization.py (moved from analysis/discretization.py:21)
    # x_{k+1} = Φ(x_k, u_k), u held; "exact" for a linear model; keeps the source (live params)

class Computer(StepDiagramSystem):              # core/discrete/computer.py (new home; today simulation/discrete/computer.py:34)
    schedule: StepSchedule                      # dt_base and {block_id: divisor d}, plain data
    sample_period = dt_base
    # state: each block's state and each block's held outputs (the ZOH memory); a block
    # fires on k mod d == 0 as a select; signals propagate within the tick (R125)

class HybridDiagram(StepDiagramSystem):         # core/discrete/hybrid_diagram.py:30 (today a plain class, not a System)
    # built by computer @ plant: {"controller": computer, "plant": discretize(plant, dt_base, substeps)}
    sample_period = dt_base
    def compute_rollout(self, tf, *, x0=None, params=None, backend="numpy") -> Trajectory   # runner 1
    def compute_trajectory(self, tf, *, solver="scipy", substeps=None, ...) -> Trajectory   # runner 2
    def plot_trajectory(...), plot_diagram(...), animate(...)
```

The contract of the two runners: `compute_rollout` is the step rollout, every signal recorded
at the ticks (`controller:u`, `plant:y`, each block's ports), pure, traced; `evaluate` scores
exactly these samples. `compute_trajectory` ticks the same pure computer and integrates the
plant between ticks with a continuous solver (adaptive SciPy by default; RK4 substeps by
`solver="rk4", substeps=`), returning the dense plant run with the ticks as a `signals`
entry; it is the realtime runner's offline twin. Both store `loop.traj`. The substep count of
the discretized plant in runner 1 defaults from the plant's smallest time constant against
`dt_base` (one owner, `simulation/time_grid.py`'s `AUTO_DT_SCALE`), and `problem.compile(...,
substeps=)` overrides it.

`block % dt`: a continuous block with states becomes `discretize(block, dt)` inside a
one-block `Computer` (the digital PID: its integrator and filter stepped by RK4 at `dt`, the
difference equation of GRO501); a static block is wrapped as is; a `StepSystem` is wrapped
with its period checked; a `StepDiagramSystem` with a `StepSchedule` is a multi-rate computer.

What changes: `compute_rollout` returns a `Trajectory` (today a `StepRollout`,
`core/compile/evaluators/discrete.py:39`); the discrete evaluators' `rollout` returns native
arrays and traces end to end (today `np.asarray` at `discrete.py:57`); `Computer.tick`
(`computer.py:114`: in-place buffers, NumPy only, one tick of latency) becomes the pure
tick with selects, and the orchestrator calls that tick; `HybridDiagram` becomes a
`StepDiagramSystem` (today it bundles a `Computer` and a plant diagram and delegates to
`HybridSimulator`, `hybrid_diagram.py:103`); `compile_step_diagram` (`compiler.py:253`)
accepts `flatten=True`; the MPC's latch (`mpc/controller.py:567`) goes (R123).

### 6.3 `Trajectory`

```python
@dataclass(frozen=True)
class Trajectory:                               # core/trajectory.py:52, unchanged
    t: (N,); x: (n, N); u: (m, N); signals: dict[str, (p, N)]; roles: dict
    def of(self, role) -> Trajectory            # "plant": the plant's run by static indexing
    def with_cost(self, cost, *, discount_rate=0.0, params=None) -> Trajectory
```

A step rollout is a `Trajectory` with `t = k·dt` (`dt = 1` when the system has no period).
The hybrid loop's `compute_trajectory` returns the dense plant run with `signals["ticks"]`
marking the sample times.

### 6.4 `Set`, `InputSet`, `CostFunction`, `Field`

```python
class Set:                                      # core/sets.py:22
    params: dict                                # the set's numbers; {} when none
    def margin(self, z, t=0.0, params=None): ...      # ≥ 0 feasible
    def contains(self, z, t=0.0, params=None): ...
    def sample(self, key, n=None, params=None): ...   # key an integer seed; the cipher on both backends
    def bounding_box(self): ...
    def as_cost(self, weight=1.0, shaping=None) -> CostFunction   # = Field(margin).as_cost (R133)
    def __and__(self, other) -> IntersectionSet

class InputSet:                                 # core/sets.py:90
    def margin(self, u, x=None, t=0.0, params=None): ...

class CostFunction:                             # core/costs.py:29
    params: dict
    def g(self, x, u, t=0.0, params=None): ...  # the running cost
    def h(self, x, t=0.0, params=None): ...     # the terminal cost
    def integrate(self, traj, *, discount_rate=0.0, params=None)   # the trapezoid on a held run (reporting only)
    # +, * as today; a sum nests its terms by id

class Field:                                    # core/fields.py (today planning/spatial/state_fields.py:26, StateField)
    params: dict
    def value(self, x, u=None, t=0.0, params=None): ...
    def gradient(self, x, u=None, t=0.0, params=None): ...   # jax.grad under JAX, central differences on NumPy
    def as_set(self, lower=None, upper=None) -> Set           # {x : lower ≤ V(x) ≤ upper}
    def as_input_set(self, lower=None, upper=None) -> InputSet
    def as_cost(self, weight=1.0, shaping=None, *, terminal=False) -> CostFunction   # g (or h) = weight · shaping(V)
    __call__ = value

class QuadraticField(Field):                    # new
    def __init__(self, center, P): ...          # params = {"center", "P"}; V = (x − c)ᵀ P (x − c)
    # as_set(upper=level) -> EllipsoidSet with sample and bounding_box
```

What changes: the library sets and `BoxSet` read `params` (today `Set.params = None`,
`sets.py:177` reads frozen fields); `Set.as_cost` and the shaping functions move to
`core/fields.py`; `StateField` is promoted to `Field` in `core/` with `as_constraint` →
`as_set` (`state_fields.py:33`) and `as_input_constraint` → `as_input_set`, renamed with every
caller (R133, R82); `FieldCost` (`state_fields.py:185`) gains the terminal switch (A4).
`CostFunction.integrate` stays as the plotting quadrature (`traj.with_cost`); the score no
longer uses it (§6.9).

### 6.5 `Distribution` and the sources

```python
class Distribution:                             # core/distributions.py:24
    params: dict                                # the law's numbers
    dim: int
    mean: np.ndarray                            # a property of params
    def sample(self, key, n=None, params=None): ...   # (dim,) or (n, dim); the cipher, both backends alike

Gaussian(mean, std) | Gaussian(mean, cov=)      # params {"mean", "std"} or {"mean", "cov"}
Uniform(lower, upper)                           # params {"lower", "upper"}; support a BoxSet
Particles(points)                               # params {"points"}

class WhiteNoise(Source):                       # blocks/sources.py:211, unchanged in shape
    params = {"seed": int | None, "sample_period": float, "psd": array}
    def h(self, x, u, t=0.0, params=None): ...  # w_k = Σ^{1/2} ε(seed, ⌊t/Δ⌋), the counter-based draw
    def realize(self, key): ...                 # a fresh seed under key
```

What changes: `sample(key, n)` (`distributions.py:46`) gains `params=None` and takes an
integer key on both backends (today a NumPy generator or a JAX key: KNOWN_ISSUES); `Gaussian`
holds `mean`, `std` in `params` (today attributes, `distributions.py:59`). The sources are
already on the contract (`sources.py:257`, `sources.py:290`).

### 6.6 `GaussianPolicy`, `NeuralPolicy`

```python
class GaussianPolicy(System):                   # control/stochastic.py (new)
    def __init__(self, mean, log_std, *, sample_period, seed=None): ...
    # ports: the mean block's (x or y, and r when it has one) in, u out
    params = {"mean": mean.params, "log_std": (m,), "seed": seed}
    is_random = True
    def h(self, x, u, t=0.0, params=None): ...  # u = μ(x) + exp(log_std) · ε(seed, ⌊t/Δ⌋); seed=None: μ(x)
    def log_prob(self, x, u, t=0.0, params=None): ...
    def entropy(self, x, t=0.0, params=None): ...
    def realize(self, key): ...                 # a fresh seed

class SquashedGaussianPolicy(GaussianPolicy): ...   # the SAC head; the same three methods
class NeuralPolicy(System):                     # control/neural.py:15, NeuralPolicyController renamed at the v1.0 window (R134)
```

What changes: the RL package's `StochasticPolicy` and its heads (`policy.py:38`, `:119`) are
rebuilt on the block, the collectors draw per-episode seeds through `realize` (the RL numbers
move once), `NeuralPolicyController` is the deterministic face. Prototype 4 shows the draw
held within a period and vmappable over child seeds (§10).

### 6.7 `PlanningProblem`, `StochasticPlanningProblem`, `realize`, `StepProblem`

```python
class PlanningProblem:                          # planning/problems.py:29
    def __init__(self, sys, x_start=None, x_goal=None, cost=None, X=None, U=None, X0=None, Xf=None,
                 tf=None, infeasible_cost=None, discount_rate=0.0, *, disturbances=None): ...
    # disturbances={"d": Step(...)}: source blocks wired onto the plant's ports (R128); a law refused.
    # A plant that declares the port owns it; otherwise a visible Sum adds "d" to the command and
    # "n" to the measurement, today's closed_loop(disturbance=, noise=) rule, so the Pendulum of 5.3 works
    params: property                            # {"plant", "cost", "X", "U", "X0", "Xf", "x_start", "discount_rate", <source ids>}
    def realize(self, key=None, n_trials=1) -> (x0s, params)   # one shape (R128): (1, n) and the tree
    def discretize(self, dt, *, solver="rk4", substeps=1, episode_length=None) -> StepProblem   # A3
    def compile(self, controller, dt=None, *, solver="rk4", substeps=1, backend="auto", episode_length=None) -> ProblemEvaluator
    def replace(self, **fields), cost_bound(...), discount_factor(dt), horizon_kind()

class StochasticPlanningProblem(PlanningProblem):   # planning/problems.py:506
    def __init__(self, sys, x_start: array | Distribution = None, ..., params: dict[laws] = None,
                 disturbances=None, criterion="expectation", ...): ...                                   # A2
    def realize(self, key, n_trials) -> (x0s (n, nx), params)   # drawn leaves and every random block's seed batched
    def nominal(self) -> PlanningProblem

@dataclass(frozen=True)
class StepProblem:                              # planning/step_problem.py (new)
    problem: PlanningProblem; dt: float; N: int; gamma: float; solver: str; substeps: int
    def transition(self, x, u, k, params=None): ...   # (x_{k+1}, c_k): Φ_c on the plant, or on the loop (§6.9)
    def margin(self, x, u, k, params=None): ...       # min(X.margin(x, t_k), U.margin(u, x, t_k))
    def terminal(self, x, params=None): ...           # h(x, t_N) on a finite horizon, 0 otherwise
    def reached(self, x, params=None): ...            # Xf.margin(x, t_N)
    price: float | None
    def realize(self, key, n_trials): ...             # the problem's
```

What changes: `realize` returns one shape (today three: `problems.py:246`, `:638`, and the
evaluator's `realize` at `evaluator.py:324`); `disturbances` takes blocks on both problems
(today `{port: Distribution}` on the stochastic one, `problems.py:631`); the keywords of A2;
`discretize` is new and the three stepping tools read it.

### 6.8 `ProblemEvaluator`, `evaluate`, `Evaluation`

```python
class ProblemEvaluator:                         # planning/evaluator.py:40
    def __init__(self, problem, controller, dt=None, *, solver="rk4", substeps=1, backend="auto", episode_length=None)
    loop: StepDiagramSystem                     # the lowered loop: discretize(ctl @ plant, dt) or the hybrid diagram itself
    step_problem: StepProblem
    params: property                            # the run's complete tree (every block, each stateful block's x0, the problem's leaves, the free inputs)
    def trajectory(self, params=None, key=None) -> Trajectory
    def evaluate(self, params=None, key=0, n_trials=None, *, record=False) -> Evaluation
    def program(self, wrt, bounds=None, key=0, n_trials=None, *, params=None) -> MathematicalProgram
    def pack(self, params, wrt) -> z;  def unpack(self, z, wrt, params=None) -> tree

def evaluate(problem, controller, *, dt=None, solver="rk4", substeps=1, n_trials=None, key=0, record=False, backend="auto", episode_length=None) -> Evaluation

@dataclass(frozen=True)
class Evaluation:                               # planning/evaluation.py:27, unchanged
    cost, J, feasibility_margin, failed, reachability_margin, reached: (n_trials,); x0; trajectories; criterion
    objective, mean, std, worst, cvar(alpha), feasibility_rate, reachability_rate, success_rate
```

`problem.compile` dispatches on the controller's kind (R105): a continuous law or a built
continuous loop is closed and lowered by `discretize(loop, dt, solver, substeps)`; a
`Computer`, a step system with a period, or a built `HybridDiagram` is taken as the step loop
(its base clock is `dt`; a given `dt` must agree); a `TrajectorySource` is `source >> plant`;
a `PlanningSolution` is its policy. `backend="auto"` keeps JAX when the whole score traces;
a host-solver block (an Ipopt MPC) makes the loop NumPy, announced once.

What changes: the run (`evaluator.py:355`) becomes the one scan on `step_problem.transition`;
`require_continuous` (`evaluator.py:789`) goes; `score_run` (`evaluation.py:125`) reads the
scan's sums instead of the trapezoid; the `trials_fn` cache (`evaluator.py:462`) stays as the
one jitted `vmap`.

### 6.9 One score: the concrete answer

The investigation's questions, answered one by one, with the prototype's evidence (§10, P1).

1. **Sets and costs keep their continuous signature and read `t`; nothing discrete is
   written twice.** The `StepProblem` binds `t_k = t₀ + k·dt` when it calls
   `X.margin(x_k, t_k)`, `U.margin(u_k, x_k, t_k)`, `Xf.margin(x_N, t_N)`, `h(x_N, t_N)` and
   `g` at the stage times inside the step. A natively discrete task is written as a
   `StepSystem` plant and the same continuous-signature cost; its per-step cost is
   `g(x_k, u_k, t_k)·dt` (one stage), so with `dt = 1` the cost reads as a per-step cost.
2. **Yes: a continuous loop is evaluated exactly as a `StepSystem` on its `dt`.**
   `problem.compile(ctl, dt)` lowers `ctl @ plant` through `discretize(loop, dt, solver,
   substeps)` and scores the resulting step system with the same scan that scores a native
   step loop or a hybrid loop. The trapezoid on the samples becomes the step's own quadrature.
3. **The per-step cost is integrated with the state, by the same scheme:** the augmented
   state `ċ = e^{−ρ s} g(x, u_k, t_k + s)` over one period, so
   `c_k = ∫₀^{dt} e^{−ρ s} g(x(t_k + s), u_k, t_k + s) ds` by the step's Euler or RK4 stages
   and substeps, and `J = Σ_k γ^k c_k + γ^N h(x_N)`, `γ = e^{−ρ dt}`. Under Euler `c_k =
   g(x_k, u_k, t_k)·dt` bitwise (P1a): value iteration's and RL's numbers do not move. Under
   RK4 the cost is fourth-order exact (P1c). The evaluator's number moves once: on a PID
   loop at `dt = 0.05` by 2 %, at `dt = 0.01` by 0.08 %, toward the converged value (P1b).
4. **The failure rule is `score_run`'s, once:** the run goes to the horizon, no cut;
   `failed = ¬finite ∨ (min_k margin_k < 0)`; `reached = Xf.margin(x_N, t_N) ≥ 0`; the price
   switch adds `price` to `cost` on `failed ∨ ¬reached` and replaces a non-finite cost by
   `cost_bound`. The RL environment's early stop and training zone remain a training choice
   of `RolloutEnvironment`; a learner reports the evaluator's `J`. Value iteration prices a
   transition that leaves its grid (the table's domain, a tool limit, documented), through
   `sp.price`. Shooting's reported `cost` is the evaluator's (`solve(evaluate=True)`).
5. **The hold:** a sampled controller's output is a state of the loop (ZOH by construction,
   R104); a continuous law is inside the ODE and is not held; the loop's free inputs are held
   over `dt`. `solver="scipy"` on `compute_trajectory` is the cross-check that the hold and
   the step error are small.
6. **The object:** `StepProblem`, from `problem.discretize(dt, ...)` (A3). It holds the
   problem, `dt`, `N`, `γ`, the scheme, the price, and `transition(x, u, k, params) ->
   (x_next, c_k)`, `margin`, `terminal`, `reached`, `realize`. Three readers: value
   iteration (`StateSpaceGrid.successor` and `running_cost_table` read `sp.transition`),
   `RolloutEnvironment(sp)` (reward `−c_k`, `gamma = sp.gamma`, one owner, R108), and the
   problem evaluator (the scan). A loop's `StepProblem` is the plant's `transition` lifted to
   the loop's state: the plant's slice and its input signal `plant:u` at the stage times.
7. **Margins between samples** are not checked (today's rule); the step could return the
   least margin over its substeps later without changing a signature (an open point, §11).
8. **Time-varying sets and costs** read `t_k` at the samples and the stage times; a
   schedule (`S(t)`, `K(t)`) is interpolated in `t` by its field or block.

How the plant's cost is read inside a loop: the loop's compiled evaluator already gives the
internal signals at any `(X, U, t)` (`DiagramEvaluator.signals`), so `g_loop(X, U, t) =
g(X[plant], signals(X, U, t)[plant:u], t)` is one function, evaluated at the stage times
with the state. For a hybrid loop the plant block is a `Discretization` whose step runs the
substeps; `u_plant` is the computer's held output of tick `k`, constant over the period, so
`transition` is the plant's `step_with_cost` beside the computer's pure step.

### 6.10 `MathematicalProgram`, `Optimizer`, the kernels

```python
@dataclass(frozen=True)
class MathematicalProgram:                      # optimization/mathematical_program.py:26
    n_z: int
    J: callable                                 # J(z) or J(z, params=None); the program adapts a one-argument J
    h, g: callable | None                       # h(z, params=None) = 0, g(z, params=None) ≥ 0
    lower, upper: array | None
    grad_J, hess_J, jac_h, jac_g: callable | None
    params: dict = {}                           # the frozen defaults (R130); {} for a student's program
    backend: str; metadata: dict
    def params_at(self, z) -> dict              # the full tuned tree (an exported program; R134 line 6)

class Optimizer:                                # optimization/optimizer.py:75
    def __init__(self, program, z0=None, *, method="auto", backend=None, iterations=None, step_size=None,
                 use_hessian=False, options=None, **method_kwargs)
    params: dict                                # {"program": program.params, "step_size": …}: the solver's own leaves
    adapter: OptimizerBackend                   # today's .backend attribute (R134 line 1)
    def solve(self, *, z0=None, params=None, callback=None, record_solve_time=False, verbose=False) -> OptimizationResult
    def as_step_system(self) -> StepSystem      # x_{k+1} = step(x_k; p): the solve as a rollout (R132)

# optimization/kernels.py (new): the native algorithms as pure init / step laws
def gd_init(z0, params) -> state;  def gd_step(program, state, params) -> state       # projected on the box
def adam_init(z0, params) -> state; def adam_step(program, state, params) -> state   # today's Adam (first_order.py:135) made pure of the program
```

`solve(params=)` merges a partial tree onto `program.params` once at the boundary (R131) and
hands the complete tree to every function. A native method with `iterations=K` runs `K`
kernel steps: a Python loop on NumPy, one `lax.scan` on JAX, so a solve inside a traced
block is a pure function of `(z0, params)`. The wrapped methods (`scipy_slsqp`,
`scipy_trust_constr`, `ipopt`, `l-bfgs`) bind the tree into closures per solve. The
performance gate (R130): on JAX, `params` is a pytree argument of the jitted functions, the
same mechanism the evaluators use (`DESIGN §5`); carrying it costs no retrace and no
measurable per-call time, and a frozen export is one `functools.partial` if a benchmark ever
asks for it.

What changes: `J(z, params=None)` and `params` on the noun; `ParametricMathematicalProgram`
(`trajectory_optimization/parametric_program.py:25`) folds in; `Optimizer.__init__`'s
`compile_backend` → `backend`, `.backend` → `.adapter`; the first-order backend
(`first_order.py:22`, a Python loop) becomes the kernel runner; `Adam.init` / `update`
(`first_order.py:155`, `:167`) are already the kernel shape.

### 6.11 The planners, `PlanningSolution`, the planner interface, `ModelPredictiveController`

```python
class Planner:                                  # planning/planner.py:21
    def __init__(self, problem): ...
    def compile(self, backend=None): ...                      # build once (the program, the jitted rollout); idempotent
    def solve(self, *, params=None, z0=None, evaluate=False, n_trials=None, **kwargs) -> PlanningSolution
    # params: a partial tree on problem.params (x_start among them); z0: a warm start where the planner has a decision
    def shift(self, z, n_ticks) -> z            # the warm-start guess one period later (trajopt, MPPI, the native solver); NotImplementedError by name elsewhere
    program: MathematicalProgram | None         # after compile, where the planner has one
    def evaluate(self, *, dt, n_trials=None, key=0) -> Evaluation

@dataclass(frozen=True)
class PlanningSolution:                         # planning/results.py:21, basic (R138)
    problem, policy, trajectory, evaluation, cost_to_go: Field | None, solver: record, success: property
    def plot_control_law(...), plot_cost_to_go(...), plot_trajectory(...), plot_cost(...)

class ModelPredictiveController(StepSystem):    # planning/trajectory_optimization/mpc/controller.py (rebuilt; re-exported by control)
    def __init__(self, planner, *, sample_period, replan_every=1, warm_start=True, parameter_ports=()): ...
    # state: the plan z (n_z); ports: y (the measurement) and each parameter port in, u (and x_ff, z) out
    params = {**planner.problem.params minus x_start, "step_size": …, "x0": z0}   # A8
    def step(self, x, u, k=0, params=None): ...     # z_{k+1} = solve(params | x_start = y_k, z0 = shift(z_k)) on replan ticks, else z_k (a select)
    def h(self, x, u, k=0, params=None): ...        # u_k = the first input of z_k (one period of computational delay, R123)
```

The planner interface (R135) is three verbs the MPC reads and nothing else: `compile()`,
`solve(params=, z0=)`, `shift(z, n_ticks)`; the plan is read off the solution
(`solution.solver.z`, `solution.trajectory`). `TrajectoryOptimizationPlanner` offers all
three (collocation and shooting; `shift` moves the knot schedule); the native fixed-iteration
form is `TrajectoryOptimizationPlanner(..., transcription="single_shooting", method="gd" |
"adam", iterations=K)`, whose `solve` is pure. MPPI (v0.4) offers the same three.
`LQRPlanner`, `DynamicProgrammingPlanner`, `RRTPlanner` and the RL planner offer `solve`
(and `compile` where it builds tables or jits), and an MPC built on one is refused by name.

What changes: `solve_trajectory_from(x0, params=)` (`planner.py:83`) folds into
`solve(params={"x_start": x0})`; `validate_mpc_planner`'s `hasattr` probing
(`mpc/controller.py:905`) goes; the latch (`:567`) goes; `dt_mpc` → `sample_period`; the
dual-rate broadcast becomes a `replan_every` divisor on the one block (today a second leaf,
`:778`); the block's `params` holds the model tree (A8).

## 7. The architecture under it

### 7.1 Layers

```text
┌──────────────────────────────────────────────────────────────────────────────────────┐
│ facades (the student's verbs)   compute_trajectory  compute_rollout  linearize  bode  │
│                                 evaluate  compare  lqr  lqg  discretize  plot_*       │
├──────────────────────────────────────────────────────────────────────────────────────┤
│ tools (verbs on a System or a PlanningProblem)                                        │
│   simulation/   Simulator (continuous)   HybridSimulator (runner 2)   realtime/       │
│   analysis/     linearize, frequency, time response, lyapunov, riccati, structural    │
│   planning/     PlanningProblem ─discretize→ StepProblem ─compile→ ProblemEvaluator   │
│                 planners: LQR · DP · trajopt (collocation, shooting; native K) · RRT  │
│                 MPC block (StepSystem) · PlanningSolution · compare                   │
│   learning/     RL planner on RolloutEnvironment(StepProblem) · tabular learners       │
│   optimization/ MathematicalProgram(params) · Optimizer · kernels (gd, adam) · adapters│
├──────────────────────────────────────────────────────────────────────────────────────┤
│ libraries (System subclasses by diagram role)                                         │
│   blocks/ (sources, routing, filters, MLP, UnitDelay)  dynamics/ (plants)             │
│   control/ (PID, StateFeedback, impedance, robotic, NeuralPolicy, GaussianPolicy)     │
│   estimation/ (StateObserver; luenberger, kalman)                                     │
├──────────────────────────────────────────────────────────────────────────────────────┤
│ core (the model and the nouns)                                                        │
│   System · DynamicSystem · DiagramSystem · the operators @ >> + %                     │
│   discrete/: StepSystem · StepDiagramSystem · discretize · Computer · HybridDiagram   │
│   Trajectory · Set · CostFunction · Distribution · Field · geometry                   │
│   params.py (the tree) · distributions.py (the cipher)                                │
│   compile/: the evaluators (one per kind), stepping.py (the step model, with cost)    │
└──────────────────────────────────────────────────────────────────────────────────────┘
          NumPy face: plain bodies, Python loops         JAX face: jit(vmap(scan)), grad
```

### 7.2 The one contract (R140), as the core's first step

`core/params.py`, one module, read by every tree consumer (today four copies:
`planning/evaluator.py:991–1046`, `planning/problems.py:896–914`,
`core/compile/evaluators/jacobian.py`, `core/assembly.py`):

```python
lookup(tree, path)                  # "controller.Kp"
with_leaf(tree, path, value)        # a copy, the tree stays complete
merge(defaults, partial)            # one merge at the user boundary; an unknown key names the known ones
leaves(tree)                        # (path, value) pairs; float_leaves / integer_leaves split the seeds out
pack(tree, wrt) / unpack(z, wrt, tree)
realize(tree, random_paths, key)    # a child seed per random block, by path, with the cipher
batch_axes(tree, n)                 # the vmap axes: 0 for a leaf with a leading n, None otherwise
```

The contract test walks every public core math object on both backends: its `params` is a
dict; a `params=None` call equals the call with its defaults; `jax.grad` through a float leaf
exists; the same seed gives the same values twice and on both backends; a composite's tree is
its parts' trees by name. The exceptions list (DESIGN §4): the RL collectors until the seeded
policy lands; student-written models (RULES 2.3).

### 7.3 Module layout and the dependency law

```text
minilink/
  core/            system.py diagram.py wiring.py operators.py closed_loop.py assembly.py roles.py topology.py
                   params.py (new) trajectory.py sets.py costs.py distributions.py fields.py (new) geometry.py
    discrete/      step_system.py step_diagram.py discretization.py (moved) computer.py (moved, pure) hybrid_diagram.py
    compile/       compiler.py plan.py evaluators/{base, dynamic, diagram, static, discrete, stepping, jacobian, compiled}.py
  blocks/ dynamics/ control/ (+ stochastic.py: GaussianPolicy) estimation/
  simulation/      simulator.py solvers/ time_grid.py discrete/hybrid_simulator.py (runner 2) realtime/
  analysis/        (discretize re-exported from core)
  planning/        problems.py step_problem.py (new) evaluator.py evaluation.py planner.py results.py comparison.py
                   dynamic_programming/ trajectory_optimization/ (+ mpc/) search/ spatial/
  learning/        reinforcement_learning/ (environment.py on StepProblem; policies from control)
  optimization/    mathematical_program.py optimizer.py kernels.py (new) optimizers/ (adapters) evaluators/
  graphical/ interfaces/ experimental/
```

The dependency law (DESIGN §3) with three edits: `core/discrete` imports nothing from
`simulation` (R100's exception deleted: `Computer` and `discretize` live in the core);
`planning` imports `blocks.sources` for the `disturbances=` shortcut (a new written
exception: the problem wires a source block); `learning` imports `control.stochastic`
(the RL heads are the `GaussianPolicy` block; the existing `learning → control.neural`
exception widens by one module). Core never imports a tool; tools import core; libraries
import core and `dynamics/abstraction`.

### 7.4 The runtime model

| What | Compiled once per | Varies without recompiling | Where host code stays |
| --- | --- | --- | --- |
| A continuous loop's evaluator | loop structure, backend | `params` (every leaf), `x0`, `u_seq` | the adaptive SciPy solver (`compute_trajectory`) |
| A step / hybrid loop's evaluator | structure, `dt_base`, substeps, schedule, backend | `params` | a block whose `step` calls a host solver (Ipopt MPC): the loop is NumPy, announced |
| `ProblemEvaluator` | loop, `dt`, scheme, horizon `N`, backend; one jitted `vmap(scan)` per batch layout (`trials_fn`) | `params`, the key, `n_trials` (a new layout retraces once) | — |
| `MathematicalProgram` | the problem's structure, `n_z`, the rows | `params` at `solve` | SciPy and Ipopt adapters (closures per solve) |
| The native MPC tick | `K`, `N_h`, the transcription | `params` (model, weights, `x_start = y_k`, `step_size`) | none: the tick is pure |
| The RL training loop | the environment, the policy's shapes | the seed, the weights | none (today's `lax.scan` collectors) |

Two runners of the hybrid loop, one set of block functions: runner 1 is the pure step diagram
(semantics, `jit`, `grad`, `vmap`, the evaluator); runner 2 the clock orchestrator over the
same pure tick (realtime, game mode, hardware in the loop, an adaptive plant solver, a
host-only block). Runner 1 is runner 2's test oracle at the ticks (P2 shows the two agree to
1e-12 on an integrator plant).

### 7.5 JAX traceability and efficiency, path by path

| Main math path | Traced form (`jax_shapes.py` in the scratch folder) | Why it traces | Why it is efficient |
| --- | --- | --- | --- |
| A continuous rollout | `lax.scan` over `k` of `step_body` (today's `stepping.py:62`) | no Python control on traced values; `solver`, `substeps`, `hold` static | one scan, no per-step dispatch |
| The one-score run | `score_scan`: carry `(x, J, m)`, body = `transition` + `margin`; `N` static; `finite_horizon` a `where` | the augmented cost is arithmetic on the stages; `min` and `where` are traced | one scan computes the run, the cost and the margins together; no second pass over the samples |
| Monte Carlo | `vmap(score_scan)` over `(x0s, params)` with batch axes on drawn leaves and seeds | `realize` is the cipher (`uint32` arithmetic, `distributions.py:199`), traced; a seed is an integer leaf, not differentiated | one `jit(vmap(scan))` per layout, cached (`trials_fn`); 1000 × 1000 steps in tens of ms today |
| The tuning gradient | `jax.grad` of `objective` over the float leaves | reverse mode through scan; the price is a `where` step with zero gradient (documented) | memory linear in `N`; `jax.checkpoint` on the tick body when an MPC sits inside |
| The multi-rate computer tick | `computer_tick`: per block `fired = (k % d == 0)`, `where(fired, new, held)`; block order static | no branch on `k`; the held outputs are carry | idle blocks are computed and discarded (the base-tick cost); the two-clock common case nests the plant's substeps as a static loop, so nothing is wasted there |
| The hybrid loop | `hybrid_rollout`: `scan` over base ticks of (tick, plant step) | the plant step is `step_body`; `u_k` is the computer's held output | one scan; `vmap` over starts, seeds, parameter families |
| The native MPC tick | `mpc_step`: an inner `lax.scan` over `K` of a projected gradient step; `grad_J` by `jax.grad` of the shooting cost | `K` static; the shift is a `concatenate`; the projection a `clip` | `K` iterations per tick, no host call; the gradient of the closed loop is exact for the controller that runs |
| The native solver | `scan` over `iterations` of `kernel_step` | the kernel is pure arithmetic on a pytree (today's `Adam.update`) | one jitted solve; `vmap` over seeds is a Monte Carlo over solves |
| The seeded policy | `h` = `mean.h` + `exp(log_std) · standard_normal(seed, ⌊t/Δ⌋)` | the draw is the cipher on `(seed, k)`; `floor` on a traced `t` | no key plumbing; `vmap` over a `(n,)` seed leaf |
| Value iteration | today's `lax.while_loop` backup on tables built with `vmap` of `sp.transition` | unchanged | unchanged |

Untested on JAX here (not installed): every line above. The confidence comes from the fact
that each form is already in the code on JAX today (the rollout scan, the `vmap`ped trials,
the cipher, the RL `lax.scan` collectors, the Adam kernel) and the new pieces (the augmented
cost, the select tick, the inner `K` scan) add only arithmetic, `where`, `clip` and
`concatenate` on arrays of static shape.

## 8. The v0.3 coverage table

| Workstream / choice | Where this design satisfies it | Revised? |
| --- | --- | --- |
| §1 every object pure, every number a parameter (R140) | §4's rule, §6 (every object's `params`), §7.2 (`core/params.py`, the contract test) | no |
| §2 the step algebra first-class (R125, R126) | §6.2: propagation within the tick, the operators on step systems, traced step rollouts returning `Trajectory` | no |
| §3 the draws (R128) | §6.5, §6.7: sources as disturbances, one `realize` shape, the cipher on both backends | A2 renames the stochastic problem's keywords |
| §4 the seeded policy (R129) | §6.6 `GaussianPolicy`, P4 | A6: `mean` is a block |
| §5 the hybrid loop, one model two runners (R127) | §6.2 `Computer`, `HybridDiagram`, `compute_rollout` / `compute_trajectory`; §7.4 | A1 (`@` reads the left side's clock), A5 (the computer in the core) |
| §6 one score | §6.9, A3, P1 | A3 adds `StepProblem`; A7 derives `gamma` |
| §7 the parametric program, the stochastic program its seed case (R130, R131) | §6.10, §5.5, §5.8 (`pe.program(wrt, key, n_trials)` with the seed a leaf) | no |
| §8 the native proof of concept (R132) | §6.10 kernels, `as_step_system`; §6.11 the fixed-iteration MPC; §5.6; P3 | no |
| §9 Fields (R133) | §6.4 `Field`, `QuadraticField`, `as_set` / `as_input_set` / `as_cost`; §5.7 | A4 adds `terminal=` |
| §10 hardening and the merge | §9 (the order of work, the notebook sweep, the number ledger) | — |
| R124 three owners of a start | §6.1 (`x0` a view, write-through), §6.7 (the problem's `x_start`), §6.8 (each stateful block's `x0` a leaf); P4 | no |
| R134 names | `backend=`, `adapter`, `sample_period=`, `.`/`:`, `…Policy`, `parameter_ports=`, `params_at` used throughout | no |
| R135 planner interface | §6.11: `compile`, `solve(params=, z0=)`, `shift` | no |
| R138 basic `PlanningSolution` | §6.11 | no |
| R136, R137, R139 | out of the core's API; nothing here contradicts them | — |

## 9. The migration from today's code

Module by module: what changes, which numbers move, the risks, and a proposed order.

### 9.1 What changes, by module

| Module | Change | Numbers |
| --- | --- | --- |
| `core/params.py` (new) | the tree verbs, from the four copies | none (byte-identical baselines) |
| `core/system.py`, `core/wiring.py`, `core/diagram.py` | `x0` a view with write-through; `refresh()` leaves `Simulator`; `params = {}` for a bare block; `__mod__` → the core `Computer`; `__matmul__` reads a step left side | none |
| `core/sets.py`, `core/costs.py`, `core/distributions.py` | sets read `params`; `Set.as_cost`; `sample(key, n, params)` on the cipher; `Gaussian(cov=)` | the start and parameter draws of a stochastic problem change once (one convention on both backends) |
| `core/fields.py` (new), `planning/spatial/state_fields.py`, `shaping.py` | `Field`, `QuadraticField`, the shaping, `as_set` / `as_input_set` / `as_cost(terminal=)`; the spatial fields subclass it | none (RoA, corridor and RRT baselines `cmp`) |
| `core/discrete/` | `Computer` and `discretize` move in; the pure tick with selects; `HybridDiagram` a `StepDiagramSystem`; `compute_rollout` → `Trajectory`; `flatten` on step diagrams | the hybrid demos move once (propagation within the tick: no one-tick lag); the MPC loops move once (the one-period delay, R123) |
| `core/compile/evaluators/stepping.py`, `discrete.py` | `step_with_cost`; native-array step rollouts; `rollout_batch` on step evaluators | none for the rollouts |
| `simulation/discrete/hybrid_simulator.py` | runner 2 over the pure tick; the adaptive plant solver between ticks | with the tick semantic, once |
| `planning/problems.py`, `step_problem.py` (new) | A2's keywords; `disturbances=` blocks on both problems; one `realize`; `discretize` | the frozen-noise defect fixed: a stochastic problem with a disturbance draws fresh noise per step (an intended change) |
| `planning/evaluator.py`, `evaluation.py` | the scan on `StepProblem.transition`; `require_continuous` gone; the sampled face | the evaluator's `J` moves once (the step's quadrature; 2 % at `dt = 0.05` on P1's loop) |
| `planning/dynamic_programming/` | the grid's successor and running cost from `sp.transition`; `cost_to_go` a `GridField` (later) | none under Euler (bitwise, P1a) |
| `learning/reinforcement_learning/` | `RolloutEnvironment(sp)`; the heads on `GaussianPolicy`; per-episode seeds; `gamma` derived | the RL demos re-pin once |
| `optimization/` | `params` on the noun; `Optimizer.solve(params=)`, `iterations=`, `backend=`, `adapter`; `kernels.py`; `as_step_system` | none for the wrapped methods; `gd` / `adam` byte-identical when run as kernels |
| `planning/trajectory_optimization/` | `Planner.compile` / `solve(params=)` / `shift`; collocation reads the tree; `ParametricMathematicalProgram` folds in; single shooting with the native kernel | none for a solve with `x0` only bound |
| `planning/trajectory_optimization/mpc/` | the block rebuilt: a `StepSystem` with the plan as state, the one-period delay, `sample_period=`, `replan_every=`, `parameter_ports=`, no latch, no `hasattr` | the MPC demos move once (R123) |
| `control/stochastic.py` (new), `control/neural.py` | `GaussianPolicy`, `SquashedGaussianPolicy`; `NeuralPolicyController` the deterministic face | — |
| `analysis/discretization.py` | re-exports `core.discrete.discretize` | none |
| `interfaces/gymnasium.py` | `Sys2Gym.from_problem` on `StepProblem` and `realize` | draws once |

### 9.2 The risks

1. **The course notebooks.** GRO860: `pendulum_value_iteration_vs_lqr_vs_rl` and `drone_ppo`
   take A2 and A7 (two keywords each); every other GRO860 notebook is untouched (VI, LQR,
   trajopt, `compare`, `plot_*` keep their names). GRO501: `racecar_toward_mpc` meets the
   MPC's `sample_period=` (R134 already) and the one-period delay (a visible but small change
   in its plots: the maintainer's call on the lesson's text); the other five are untouched.
   The tutorial `06_hybrid` keeps `P(...) % 0.1 @ Integrator()` verbatim and gains
   `compute_rollout`. The course material stays on `main` until the merge (R47).
2. **The number ledger.** Five intended moves, each its own commit with a closeness report
   (R80): the tick semantic (hybrid demos), the MPC delay, the step quadrature (the
   evaluator's `J`, the showcase cells), the draw convention (every stochastic baseline), the
   RL re-pin. P1 quantifies the third.
3. **The sampled face's substep default.** A plant with a small time constant under a slow
   computer needs many substeps per base tick; the default from `AUTO_DT_SCALE` must not
   silently make a loop expensive. A warning names the count when it exceeds, say, 100.
4. **Idle blocks under selects.** A 1 kHz filter beside a 10 Hz MPC computes the MPC's `K`
   iterations on every base tick and discards 99 of 100 results. The fix is the hyperperiod
   layout (sampled-path.md §8.1, draft 3), an optimization of the same semantics, when a
   benchmark asks; the MPC's own `replan_every` select is cheap because its `step` is a
   `where` around one pure solve.
5. **Gradient memory through an MPC in the loop:** `N_ticks × K × N_h` stage evaluations
   on the tape; `jax.checkpoint` per tick bounds it. Documented on the MPC block.
6. **The one contract's audit** may find blocks whose `params` carry a `bool` or a `str`
   (R118 is recent); each is a constructor flag to move.

### 9.3 A proposed order of work

Three lanes and two joins, as core-architecture §10, with the steps this design adds:

1. **Objects and draws:** `core/params.py` (byte-identical) → the contract test and the
   `params = {}` rule → sets reading `params`, `Set.as_cost`, `Field` and `QuadraticField`
   (baselines `cmp`) → `sample(key, n, params)` on the cipher and one `realize` shape (the
   draw numbers move once) → A2's keywords → `GaussianPolicy`, then the RL heads (re-pin).
2. **The loop, first:** native step rollouts and `Trajectory` out of `compute_rollout` →
   the `x0` view (R124) → the pure `Computer` tick in `core/discrete` with the orchestrator
   as its runner (the tick numbers move once, `Computer.tick` the oracle until it is
   deleted) → `HybridDiagram` as a step diagram, `@` reading the left side's clock →
   `step_with_cost` and `StepProblem` → the evaluator on the scan, the sampled face
   (the `J` moves once) → value iteration and RL on `StepProblem`.
3. **The program:** `MathematicalProgram(params=)` with one evaluator body and the
   backend table → collocation on the tree, `Planner.compile` / `solve(params=)` / `shift`
   → `kernels.py`, `Optimizer.solve(params=, iterations=)`, `as_step_system`.
4. **Join 1:** `compare` and the planners' reported scores on the one kernel; the ledger.
5. **Join 2:** the MPC block rebuilt (delay, `sample_period`, model tree), the native
   single-shooting MPC, its closed loop through `problem.compile`, tuned by gradient (P3's
   script as a test).
6. **Last:** S66's wiring refusals, the notebook sweep (A2, A7, the MPC), DESIGN §4–§6 and
   the CONSTITUTION amendments R126–R128 name, the merge.

## 10. What I verified

Everything ran with `python3` on NumPy (SciPy and JAX are not installed on this machine,
so today's `evaluate` could not be imported: `simulation/simulator.py:25` loads SciPy at
module level; its formula was reproduced by hand from `score_run`, `evaluation.py:125`).
Files are in `/tmp/claude-0/-home-user-minilink/36350854-ab7e-51d8-8262-d68717b27e02/scratchpad/fable-arch-C/`.

**P1 `proto_one_score.py` (the per-step cost and the scan).** Output:

```text
(a) Euler c_k == g dt bitwise: True
(c) RK4 on g = t^3, rho = 0:  1.5625000000000005e-06 exact 1.5625000000000003e-06
    same samples as the compiled rollout: True
(b) dt=0.05: one-score J = 282.700163   today's evaluate J = 288.437901   rel diff = 1.99e-02
    same samples as the compiled rollout: True
(b) dt=0.01: one-score J = 282.623521   today's evaluate J = 282.855987   rel diff = 8.22e-04
```

The scan's samples equal the compiled rollout's (`ev.rollout`, `base.py:110`) to 1e-12; the
augmented cost under Euler is the DP/RL rule bitwise; under RK4 it is fourth-order exact;
today's trapezoid converges to it as `dt → 0` (the one intended number move, quantified).

**P2 `proto_hybrid_tick.py` (the pure multi-rate tick, propagation within the tick, two
runners).** A 100 Hz low-pass feeding a 10 Hz P controller on an integrator, at
`dt_base = 0.01`, written as a pure tick with `where` selects and run by the scan helper:

```text
pure tick == engineer's loop (y): True
pure tick == engineer's loop (u): True
held u changes only every 10 ticks: 19 changes over 200 ticks
orchestrator (dense plant) == pure rollout at the ticks: True
```

**P3 `proto_native_mpc_eta5.py` (the fixed-iteration MPC, its closed loop in one scan, the
tuning signal).** Single shooting on the pendulum, `N_h = 10`, `K = 10` projected gradient
steps per tick with the shift warm start and the one-period delay, 40 ticks at 0.05 s, the
cost by P1's transition; gradients by central differences (the structure is `jax.grad`'s):

```text
closed loop: J = 0.1835, final state = [ 3.15401811 -0.02234943], |u| max = 5.00
stabilized to the upright (|theta - pi| < 0.05): True
dJ_closed/dq (central difference) = 0.0296
J over three starts: [0.184, 0.015, 0.118]
```

Two earlier runs are instructive: from `π − 0.8` the pendulum cannot be held (`m g l sin 0.8
= 7 > 5`, the torque limit) and the loop fails honestly; with a step size of 0.03 the plan
grows too slowly to catch the fall. A fixed-iteration MPC's step size is a real tuning
parameter, which is exactly why it is a leaf (`controller.step_size`) a gradient can reach.

**P4 `proto_objects.py` (the tree module, the `x0` view, one `realize` shape, the seeded
policy, the program with `params`, the kernel's step-system view).** Output:

```text
(1) lookup controller.Kp: [[2.0]] | merge keeps the rest: {'m': 2.0, 'l': 0.5} | integer leaves: ['controller.seed']
    pack/unpack round trip: True
    the error that teaches: "unknown parameter 'mass'; known: ['l', 'm']"
(2) write-through: controller.x0 = [0.7]  plant.x0 = [1.0, 2.0]  view = [0.7, 1.0, 2.0]
    the view follows the block: [0.7, 3.0, 4.0]
(3) x0s shape (3, 2) | plant.m draws [0.978, 1.054, 0.946] | load.seed per trial [1741505616, 234087939, 844337249] | plant.l shared: 1.0
    adding a law leaves the other streams unchanged: True True
(4) u held within a period: [ True] | new draw next period: [ True] | seed=None is the mean: [ True]
    log_prob at the mean > log_prob off it: True | entropy 0.215
    vmap over seeds (a loop here): [-0.635..., -1.053..., -1.260...]
(5) solve at a=1: [1.] | rebound a=3: [3.]
    the solve as a step system: d(step)/dz = [0.4] = 1 - 2 eta -> 0.4 (|.| < 1: converges)
```

The draws use the library's own cipher (`random_bits`, `standard_normal`, `child_seed`,
`core/distributions.py:199–284`), so they are the numbers JAX would produce. Run twice and
compared with `cmp`, the two outputs are byte-identical (the determinism the one contract
promises: every number a function of the seed).

**`jax_shapes.py`** holds the traced forms of §7.5 as pure functions in the `lax.scan` shape;
it was syntax-checked only (`py_compile`). Untested on JAX, by necessity: the augmented RK4
cost, the select tick, the inner `K` scan, `vmap` over the batched seed leaf. Each is a
composition of operations already traced in the repo today.

**Facts checked in the code** (the reviewed commit): `System.realize` and `__mod__`
(`core/system.py:190`, `:452`); `StepSystem.step` (`core/discrete/step_system.py:62`);
`StepDiagramSystem.step` (`step_diagram.py:54`); `HybridDiagram` not a `System`
(`hybrid_diagram.py:30`); `Computer.tick`'s committed buffer and NumPy-only
(`simulation/discrete/computer.py:114`, `:295`); the discrete rollout returning a NumPy
`StepRollout` (`core/compile/evaluators/discrete.py:39–60`); `step_body` and `iterate`
(`stepping.py:40`, `:62`); `rollout` and `rollout_batch` (`base.py:110`, `:146`);
`PlanningProblem.realize` and the stochastic `realize` returning different shapes
(`planning/problems.py:246`, `:638`); `problem.compile` (`:358`); the evaluator's `run`,
`program` and `require_continuous` (`planning/evaluator.py:355`, `:195`, `:789`);
`score_run`'s trapezoid (`planning/evaluation.py:125–212`, `core/costs.py:95`); the RL step's
`−g dt` (`learning/reinforcement_learning/environment.py:130`); DP's backup `g dt + α J`
(`planning/dynamic_programming/dp.py:308`); `MathematicalProgram` without `params`
(`optimization/mathematical_program.py:26`); `Optimizer.solve` without `params`
(`optimizer.py:157`); `Adam.init` / `update` (`optimizers/first_order.py:155`, `:167`); the
MPC's `dt_mpc`, latch and `hasattr` validation (`mpc/controller.py:104`, `:567`, `:905`);
`WhiteNoise.h` and `realize` (`blocks/sources.py:257`, `:290`); `Distribution.sample(key, n)`
and `Gaussian(mean, std)` as attributes (`core/distributions.py:46`, `:59`); `StateField`
and its conversions (`planning/spatial/state_fields.py:26–45`); `compile_step_diagram`
refusing `flatten` (`core/compile/compiler.py:253`); the diagram's `refresh` restacking `x0`
(`core/wiring.py:258`); `lqg` returning a box with `observer` and `gain` (`control/design.py:135`);
the notebooks' `x0_distribution=` and `gamma=` cells (`examples/teaching/courses/udes_gro860/pendulum_value_iteration_vs_lqr_vs_rl.ipynb`).

## 11. Open questions I could not settle

1. **`StepProblem` public or internal (A3).** Recommendation: public on the research lane
   (importable from `minilink.planning`, not the prelude), because a researcher comparing
   discretizations wants to print `N`, `γ` and the scheme, and because three tools name it.
2. **The margins between samples.** Checking `X` at the stage times (the step returning its
   least margin) costs the `margin` calls per stage and changes the failure rule for
   violations inside a period. Recommendation: samples only in v0.3, the stage check an
   option later, with a KNOWN_ISSUES entry naming the gap.
3. **The hybrid loop's substep default.** From the plant's time constant and `dt_base`
   (`AUTO_DT_SCALE`) with a cap and a warning, or one substep unless asked. Recommendation:
   the time-constant rule, since a student who writes `pid % 0.01 @ plant` expects the plant
   to be integrated correctly and `problem.compile(..., substeps=)` is the researcher's knob.
4. **The RL `gamma` (A7).** Deriving it from the problem's `discount_rate` is the clean rule
   but touches two course notebooks and the lesson's vocabulary ("γ = 0.97" is what the
   students read). Recommendation: derive it, keep `gamma=` one more release as an explicit
   override that warns and sets the problem's rate, then drop it (a stated exception to
   R82's no-alias rule, for a course-facing keyword).
5. **`Trajectory` from `compute_rollout` with no period.** A native step system has no `dt`;
   `t = k` reads oddly on a plot's time axis. Recommendation: `t = k` with the axis labelled
   "k" when `sample_period` is `None`; a `Computer` and a `Discretization` always have one.
6. **The `SampledDataSystem` name (A10).** Recommendation: at the v0.9 name pass, with
   `HybridDiagram` kept until then.
7. **The idle-block cost under selects** (§9.2, risk 4): whether v0.3 ships the hyperperiod
   layout or waits for a benchmark. Recommendation: wait; the base tick with selects is the
   one semantics, and the two-clock case (one controller, one plant) wastes nothing.
8. **Where `GaussianPolicy` lives:** `control/stochastic.py` (by role: a control law with a
   draw) or `control/neural.py` beside the mean it wraps. Recommendation: `control/stochastic.py`,
   since the mean may be any block (A6).
