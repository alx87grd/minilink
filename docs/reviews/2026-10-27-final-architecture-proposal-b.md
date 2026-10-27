# The final core architecture of minilink: Proposal B, the compiler and performance lens

**Date:** 2026-10-27.
**Status:** an independent proposal, not a ruling. One of three written in parallel for the
maintainer; it revises some of the preliminary choices of 2026-10-27 (R124–R140), each
revision with its reason and its cost.
**Commit reviewed:** `082103cbf4c62f627ffd416f9b66eaa4aae5998e` (`dev-v0.3`).
**Lens:** what compiles once and what varies; how the `params` tree flows through one compiled
function without retracing; where host Python (SciPy, Ipopt, realtime I/O) must stay outside
the trace; what each choice costs at compile time and at run time. The lens is a tie-breaker:
the user's view (the objects, their verbs, the names, the pipelines) comes first.

**How to read it in an hour.** §2 is the aim; §3 the eight revisions; §4 one page of objects;
§5 the eight pipelines as scripts (skim the code, read the `params` trees); §6 the signatures;
§7 the architecture and the traceability argument; §8–§11 the coverage, the migration, the
evidence, the open points. Workboard ids (`S31`, `NS-8`, …) appear only in parentheses, as
pointers into `docs/plans/TODO.md`.

---

## 2. The aim in one screen

The maintainer wants one architecture from the plant to the optimizer, on which every control
approach is a tool, where a student's one-line simulation and a researcher's batched,
differentiated, seeded experiment are the same syntax on the same objects. The core objects are
the textbook's nouns (CONSTITUTION §2): a `System`, a `Trajectory`, a `Set`, a `Distribution`,
a `CostFunction`, a `Field`, a `PlanningProblem`, a `PlanningSolution`, a
`MathematicalProgram`. Each is a pure function of its arguments and of `params`, and the
composed `params` tree is the one thing a compiled function varies.

What v0.3 must finish, in the order the code wants it:

1. **The contract** (R140): every core math object is `f(args, params=None)`, deterministic
   given its tree, the seed an integer leaf.
2. **The step algebra as the one runtime**: every run that steps, continuous plant or digital
   loop, is `x_{k+1} = step(x_k, u_k, k; p)` on an integer tick, composed with the same
   operators (R125, R126), and a continuous plant enters it through `discretize`.
3. **The hybrid loop as a step diagram at the base clock** (R127): `computer @ plant` is a
   `System` of the step algebra; two runners, one scanned and traced, one orchestrated.
4. **One score**: one discrete reading of a problem, so the evaluator, value iteration, RL and
   the sampled loop give a policy one `J`.
5. **The draws** (R128, R129): disturbances are source blocks, a seeded policy is a block, one
   `realize` from one key, the same bits on NumPy and JAX.
6. **One program noun with parameters** (R130, R131), and a native kernel that makes the solve a
   rollout (R132), so a fixed-iteration MPC's closed loop is one scan and is tuned by gradient.
7. **A minimal `Field`** (R133), the last textbook noun.
8. **The names** (R134), the planner interface (R135), a basic `PlanningSolution` (R138).

The compiler reading of that aim: **one scan body, one tree.** Every run is a `lax.scan` over
ticks of one pure step function whose only varying input is `params`; every score is a reduction
over that scan's outputs; every Monte Carlo is a `vmap` over a tree with a trial axis on its drawn
leaves; every tuning is `jax.grad` of that reduction with respect to some float leaves; an MPC is
a block whose `step` is itself a `lax.scan` of `K` solver iterations; and a host solver never
enters a trace: it runs under the orchestrator, the second runner. Everything else (the diagrams,
the operators, the problem, the planners) is how the user writes that scan body and that tree
without seeing either.

---

## 3. Adjustments to the vision

Each revises or settles a preliminary choice; each says what it costs. Everything not listed here
is taken as ruled on 2026-10-27.

### 3.1 `problem.realize` returns one tree, not a pair (revises R128's shape)

**The choice.** `problem.realize(key, n_trials) -> params`: one batched tree, in which `x_start`
is a leaf like any other (R140, point 3 already makes the start a leaf of `problem.params`).
Drawn leaves carry a leading trial axis; shared leaves (the MLP weights, the plant's undrawn
constants) carry none. The evaluator's batched run is then `vmap(score, in_axes=axes)(params)`
with no second argument.

**Reason.** R128's `(x0s, params)` duplicates the start: it is already `params["x_start"]`. One
tree is one `in_axes`, one `take_trial`, one `merge`; the code today keeps a `Batched` wrapper and
an `axes_like` merge for exactly the pair's sake (`planning/evaluator.py:324–352`). With one tree
the batch layout is read structurally from the tree and nothing is special about the start.

**Cost.** `realize`'s return shape changes once (R128 is a week old, no caller outside the
evaluator and `Sys2Gym`). The start of a *trajectory optimization* (a `Set`, R124) is not drawn, so
on a deterministic problem `x_start` is the one point, unbatched.

### 3.2 `block % dt` discretizes a continuous block instead of refusing it (settles an open row)

**The choice.** `%` on a continuous-time block with state (a `PID` with its filter, an observer,
a `LowPassFilter`) returns the computer holding `discretize(block, dt)`: the textbook digital
implementation, the block's own equations integrated one period per tick. `%` on a static block
(a gain, a `StateFeedback`, a `NeuralPolicyController`) samples it: the block's `h` read at
`t_k = k·dt` and held. `%` on a `StepSystem` schedules it as today. So `pid % 0.01 @ plant` is
the digital PID loop of GRO501 with no other verb.

**Reason.** Today `%` refuses a controller with continuous states, so the "digital PID" of the
definition of done (core-architecture §11, item 2) cannot be written. `discretize` already exists
for plants (`analysis/discretization.py:21`) and is the same equation for a controller.

**Cost.** The scheme is `discretize`'s default (`rk4`); a backward-Euler or Tustin digital PID is
written `discretize(pid, dt, solver="euler") % dt` (the `"tustin"` scheme for a linear block is a
one-line addition to `ExactDiscretization`, later). `discretize` moves from `analysis/` to
`core/discrete/` (§7.2): the core needs it for `%` and `@`.

### 3.3 `Computer` is a `StepDiagramSystem` in `core/` (realizes R127; revises the R100 exception)

**The choice.** The computer is a pure step diagram at its base clock: its blocks, their
divisors (structure, fixed at wiring), and for each block with divisor `d > 1` a hold register of
its outputs in the diagram's state. Its `step` fires each block by a select on `k mod d`, reads
the hold register otherwise, and propagates within the tick (R125). The runtime that ticks a
computer on the host (today's `Computer.tick`, `simulation/discrete/computer.py:114`) becomes a
runner in `simulation/` over that pure step: the orchestrator.

**Reason.** R127 needs `computer @ plant` to be a `StepDiagramSystem`; the simplest way is for the
computer side to already be one. The hidden one-tick lag goes with the double buffer (R125). The
`core → simulation` lazy import (`core/discrete/hybrid_diagram.py`, R100) is no longer needed.

**Cost.** A single-rate computer (every divisor 1, the common case) carries no hold register and
pays nothing. A multi-rate computer computes its idle blocks and discards them on the ticks they
skip (sampled-path §8.2, option O3); the hyperperiod unrolling that avoids it is a later
optimization, not a v0.3 item. The hybrid demos' numbers move once, with R125.

### 3.4 One score: the same `PlanningProblem` on a `StepSystem`, with `transition` as the one primitive (settles the open investigation)

**The choice.** No new noun. A `PlanningProblem` whose `sys` is a `StepSystem` *is* the discrete
problem; `problem.discretize(dt, solver=, substeps=)` builds it by replacing the plant with
`discretize(plant, dt, …)` and keeping the cost, the sets, the horizon and the start as they are.
The discrete reading is a rule on that problem, not new objects:

- time binds: a cost or a set reads `t_k = k·Δ`, `Δ = sys.sample_period` (the `dt` of a
  `discretize`d plant; `1.0` on a natively discrete system, so `t = k`);
- the horizon is `N = round(tf / Δ)` and the discount `γ = e^{−ρΔ}`;
- the per-step cost is **integrated with the state by the same scheme**:
  `c_k = ∫₀^{Δ} e^{−ρs} g(x(t_k + s), u_k, t_k + s) ds` with `u_k` held, computed by the same
  `step_body` on the augmented state `[x; c]`; on a natively discrete plant `c_k = g(x_k, u_k, t_k)`
  (the user writes the stage cost);
- the terminal cost is `h(x_N, t_N)` on a finite horizon, never on an infinite one;
- one failure rule, `score_run`'s: the run goes to the horizon, the least margin of `X` and `U`
  over the samples, failed when negative or not finite, the price switch as today;
- the hold is the discretization's: `u_k` constant over `[t_k, t_{k+1})`, the physical truth of
  the digital loop; shooting's reported score is lowered to it; a `TrajectorySource` that
  interpolates between knots is a source block, so its interpolation is inside the loop and the
  plant still sees the held sample at the base tick.

The one primitive every reader calls is `problem.transition(x, u, k, params=None) -> (x_next, c_k)`.
Value iteration's backup is `Q = c_k + γ J(x_next)`, the RL reward `r_k = −c_k`, the evaluator's
cost `Σ γ^k c_k + γ^N h(x_N)`, shooting's objective the same sum. One policy, one `J`.

**Reason.** Under forward Euler with one substep the augmented integral is exactly `g·Δ`
(verified, §10), so value iteration's and RL's numbers do not move; under RK4 it is the integral
to the scheme's order, so the evaluator's trapezoid moves by its own quadrature error (≈ 1e-3
relative on the pendulum, §10). No discrete twins of sets and costs, no new class, and the
continuous problem stays primary (collocation, the continuous LQR, the linear analysis and
`solver="scipy"` read it as they do today).

**Cost.** The evaluator computes `c_k` in a second pass, a `vmap` over `k` of one-period
reintegrations from the recorded `(x_k, u_k)`: about twice the plant work of the rollout, with
no sequential dependency (it parallelizes; the rollout does not). The fused one-pass form (the
augmented state inside the loop's scan) is what `transition` gives DP, RL and shooting, where a
transition is the natural unit. `score_run`'s trapezoid becomes this rule in one commit with a
closeness report (R80).

### 3.5 `pe.gradient(params)` beside `jax.grad` (an addition the seeds force)

**The choice.** The evaluator offers `pe.objective(params)` and `pe.gradient(params)`, the
gradient tree with `None` on the integer leaves.

**Reason.** `jax.grad(lambda p: pe.evaluate(p).objective)(pe.params)`, the line DESIGN §6
promises, raises once the tree holds an integer `seed` leaf (JAX refuses integer inputs to
`grad` unless `allow_int=True`, which returns `float0` tangents the user cannot use). The program
export is unaffected (`wrt` names float leaves only, R131). The one-liner stays valid on a tree
with no seed; `pe.gradient` is the one that always works.

**Cost.** One method, implemented as `jax.grad(..., allow_int=True)` with the `float0` leaves
mapped to `None`; central differences on NumPy.

### 3.6 R130's performance gate is met by construction; the baked export is a fallback, not a plan

**The choice.** A compiled program's functions take the tree as an argument (`J(z, params)`),
jitted once; `Optimizer.solve(params=)` merges the partial tree once, moves it to the device
once, and the solver's callbacks close over that device-resident tree. The `frozen()` export with
parameters baked in is written only if the benchmark of the row asks for it.

**Reason.** Tracing cost does not depend on whether a leaf is a constant or an argument; XLA
constant-folds a baked tree, which buys a few microseconds per call at most. The per-call cost
that matters on a host-driven solve is the host↔device round trip of `z`, present either way.
Inside a traced MPC tick there is no round trip at all.

**Cost.** None now; the benchmark (compile time and per-solve time on the MPC and PID examples)
is still run, as the ruling asks.

### 3.7 The planner interface is three pure verbs plus the host `solve` (makes R135 concrete)

**The choice.** A planner that can run under an MPC exposes `decision_dimension()`,
`solve_from(z0, params) -> (z, success)`, `warm_start(z, params) -> z0` and
`plan_of(z, params) -> Trajectory`, all pure array functions of the problem's tree; the familiar
`solve(params=None, z0=None) -> PlanningSolution` wraps them on the host. A planner whose
`solve_from` calls a host solver (SLSQP, Ipopt) is still a planner; the MPC built on it runs on
the orchestrator and refuses the traced runner by name.

**Reason.** The MPC block's `step` must be a pure function of arrays to sit in a scan; a verb that
returns a `PlanningSolution` cannot. The host wrapper keeps the student's `planner.solve()`.

**Cost.** Two verbs more than mppi.md §6 proposed (`plan_of`, and `solve_from` split from
`solve`); `hasattr` probing goes (T6).

### 3.8 Two small names

- `HybridDiagram` stays, as R127 says; the freeze may rename it `SampledDataSystem`. This proposal
  writes `HybridDiagram`.
- A `StepSystem` gains `sample_period` (the physical length of one tick: the `dt` of a
  discretization, `1.0` by default), the one place a tool reads the time of a tick from. It is
  structural, not a `params` leaf (R134: `sample_period=` is a block's period).

---

## 4. The user's view: the main objects and verbs

| Object | In the textbook | The verbs | `params` |
| --- | --- | --- | --- |
| `System` / `DynamicSystem` | `ẋ = f(x, u, t; p)`, `y = h(x, u, t; p)` | `f`, `h`, `tf`; `compile`, `compute_trajectory`, `linearize`, `plot_*`, `animate`; `@`, `>>`, `+`, `%` | the model's numbers, by textbook name (`m`, `l`, `Kp`) |
| `DiagramSystem` | a block diagram, closed under composition | the same, plus `connect`, `expose`, `trajectory_of(block)` | `{block_id: block.params}`, a live view; `x0` a view of the blocks (R124) |
| `StepSystem` | `x_{k+1} = step(x_k, u_k, k; p)`, `y_k = h(x_k, u_k, k; p)` | `step`, `h`; `compile`, `compute_rollout`, `jacobian`; the same operators | as a `System`; `sample_period` structural |
| `discretize(sys, dt)` | `x_{k+1} = Φ(x_k, u_k)`, the plant one period later, input held | a `StepSystem` keeping its `source` | the source's tree, forwarded |
| `Computer` (`block % dt`) | the digital controller at its base clock, its blocks at integer divisors | a `StepDiagramSystem`; `@ plant` | `{block_id: …}`; hold registers are state, not params |
| `HybridDiagram` (`computer @ plant`) | the sampled-data loop at the base clock | `compute_rollout` (the scan), `compute_trajectory` (the orchestrator), `compile`, `jacobian`, `plot_*` | `{"controller": …, "plant": …}` |
| `Trajectory` | `(t, x, u)` sampled, with the loop's signals | `of(role)`, `with_cost`, `resample`, `plot` | none (a record) |
| `Set` / `InputSet` | `z ∈ Z(t)` as `margin ≥ 0` | `margin`, `contains`, `sample`, `bounding_box`, `as_cost`; `&` | the set's numbers (`lower`, `upper`, `center`, `radius`); `{}` when none |
| `Distribution` | a law over a vector | `sample(key, n, params)`, `mean` | `{"mean", "std"}`, `{"lower", "upper"}`, `{"points"}` |
| `WhiteNoise`, `Step`, `Sine`, `TrajectorySource` | signal sources; noise as a held train `w_k ~ N(0, W/Δ)` | `h(t)`; `realize(key)` | `psd`, `seed`, `sample_period`; a source's table `values` |
| `CostFunction` | `g(x, u, t)`, `h(x, t)` | `g`, `h`, `g_max`, `h_max`; `+`, `*` | `Q`, `R`, `S`, `xbar`, `ubar`; a sum by term id |
| `Field` | a scalar `V(x, u, t)`: a Lyapunov function, a value, a barrier | `value`, `gradient`, `as_set`, `as_input_set`, `as_cost` | the field's numbers (`center`, `P`) |
| `PlanningProblem` | the task: `sys`, `X`, `U`, `X0`, `Xf`, `cost`, `tf`, `ρ` | `compile(controller, dt)`, `discretize(dt)`, `transition`, `stage_cost`, `realize`, `replace` | `{"plant", "cost", "X", "U", "Xf", "x_start", "discount_rate"}`, a view of the parts |
| `StochasticPlanningProblem` | the same with laws on the start and the parameters | the same; `realize(key, n_trials)` | the same, plus the laws' own numbers |
| `ProblemEvaluator` (`problem.compile(ctl, dt)`) | the closed loop, compiled once, and its score | `params`, `trajectory`, `evaluate`, `objective`, `gradient`, `program`, `pack`, `unpack` | the loop's blocks, the problem's parts, the free inputs, `x_start` |
| `Evaluation` | the Monte Carlo record of `J` | `objective`, `mean`, `std`, `worst`, `cvar`, the rates | none (a record) |
| `MathematicalProgram` | `min J(z; p)` s.t. `h = 0`, `g ≥ 0`, box | `J`, `h`, `g` with `params=None`; `params_at(z)` when exported | the problem's tree, optional |
| `Optimizer` | the solver boundary | `solve(z0=, params=)`; `method=` names the algorithm | none |
| `GradientDescent`, `Adam` (kernels) | `z_{k+1} = z_k − η ∇J` | `init`, `step`; `as_step_system` | `learning_rate`, the moments' betas |
| `GaussianPolicy` | `u = μ_θ(x) + σ_θ(x) ε_k` | `h`, `log_prob`, `entropy` | `{"mean": …, "log_std", "seed"}` |
| `Planner` family | a method that produces a policy | `solve`; the pure trio for the MPC | the planner's knobs are constructor arguments |
| `PlanningSolution` | the policy and the evidence | `policy`, `trajectory`, `evaluation`, `cost_to_go`, `success`; three plots | none |
| `ModelPredictiveController` | the receding-horizon law as a sampled block | a `StepSystem`; `% sample_period @ plant` | its planner's cost and sets, by path (`controller.cost.Q`) |

The rule that makes the table one object model: **every row's mathematics is a method of its
arguments and `params=None`; its structure (ports, sizes, schemes, divisors, `K`, `N`) is fixed at
construction and never in `params`; its `params` is a dict of floats, arrays and integer `seed`
leaves, composed by id or role; it is deterministic given that tree.**

---

## 5. The pipelines, as user scripts

Each script is written in the final API. Under it: what prints, what plots, and the `params`
tree the user sees. The pendulum is the plant throughout (`Pendulum`, `params`
`m, l, I, gravity, d`, `dynamics/catalog/pendulum/pendulum.py:32`).

### 5.1 Lesson 1: a pendulum, a PID, simulate, plot

```python
import numpy as np
from minilink import Pendulum, PID, Step

plant = Pendulum(length=1.0, mass=1.0)
plant.x0 = np.array([0.0, 0.0])

pid = PID(Kp=8.0, Ki=1.0, Kd=2.0)
loop = Step(0.5, port="r") >> (pid @ plant)        # "Step >> PID @ Pendulum"

traj = loop.compute_trajectory(tf=5.0)             # Simulator, adaptive SciPy by default
loop.plot_trajectory(signals=("plant:y", "controller:u", "error:e"))
loop.animate()

print(loop)
print(loop.params)
print(loop.x0)
loop.x0 = np.array([0.3, 0.0, 0.0, 0.0])           # writes through to the blocks (R124)
```

Prints: the diagram's one-line description, then

```text
{'Step': {'value': array([0.5])},
 'controller': {'Kp': array([8.]), 'Ki': array([1.]), 'Kd': array([2.]), 'tau': array([0.1]),
                'u_min': array([-inf]), 'u_max': array([inf]), 'e_int_min': …, 'e_int_max': …},
 'error': {}, 'demux': {},
 'plant': {'m': 1.0, 'l': 1.0, 'I': 1.0, 'gravity': 9.81, 'd': 0.0}}
[0. 0. 0. 0.]            # plant θ, θ̇; the PID's integrator and filter states
```

Plots: the angle and the torque against time, the error below; the pendulum animation. Nothing
here changes from today except the `{}` of the parameterless blocks (R140, point 2) and the
write-through `x0`.

### 5.2 Classical analysis: linearize, poles, Bode, margins of a closed loop

```python
from minilink import Pendulum, PID
from minilink.analysis import linearize, poles, margins

plant = Pendulum()
x_bar = np.array([np.pi, 0.0])                    # the upright equilibrium

lin = plant.linearize(x_bar)                      # an LTISystem at (x̄, ū)
print(lin.A(), lin.B())
print(poles(lin.A()))                             # ± √(m g l / I): unstable

pid = PID(Kp=20.0, Ki=5.0, Kd=4.0)
L = pid >> plant                                   # the open loop C·G, from e to y
L.plot_bode(x_bar=L.x0_at(plant=x_bar), of="y", wrt="e")
print(margins(L, x_bar=L.x0_at(plant=x_bar), of="y", wrt="e"))

closed = pid @ plant
closed.plot_pzmap(x_bar=closed.x0_at(plant=x_bar))
closed.plot_step_response(tf=3.0)
```

Prints: `A`, `B`, the two open-loop poles `[+3.13, −3.13]`, and the margin record (gain margin
in dB and its frequency, phase margin in degrees and its crossover). Plots: the Bode pair, the
pole-zero map of the closed loop, the step response. One addition: `x0_at(plant=x_bar)`, a diagram
helper returning the stacked operating point with one block's slice set (the controller's at its
nominal, zero), because `loop.x0 = …` now writes through and the lessons mostly want a point for
one block. Everything here is NumPy and SciPy; nothing is traced.

### 5.3 State space: LQR and an observer (LQG), closed loop, Monte Carlo over noise and parameter draws with one key

```python
from minilink import Pendulum, QuadraticCost, WhiteNoise, Gaussian, Uniform
from minilink.control import lqg
from minilink.planning import StochasticPlanningProblem

plant = Pendulum()
x_bar = np.array([np.pi, 0.0])

ctl = lqg(plant, Q=np.diag([10.0, 1.0]), R=np.eye(1) * 0.1, Q_n=0.05, R_n=1e-3,
          x_bar=x_bar)                                   # a box: observer + gain
loop = ctl @ plant                                        # r, d, n on the edge
traj = loop.compute_trajectory(tf=4.0, x0=loop.x0_at(plant=x_bar + [0.2, 0.0]))

# The stochastic task: a law on the start and on the mass; noise as source blocks on the ports
problem = StochasticPlanningProblem(
    plant,
    cost=QuadraticCost.from_system(plant, Q=np.diag([10.0, 1.0]), R=0.1 * np.eye(1), xbar=x_bar),
    tf=4.0,
    x0=Gaussian(x_bar, std=[0.2, 0.1]),
    params={"m": Uniform(0.9, 1.1)},
    disturbances={"d": WhiteNoise(1, psd=0.05, sample_period=0.01),
                  "n": WhiteNoise(1, psd=1e-3, sample_period=0.01)},
)
pe = problem.compile(ctl, dt=0.01)                        # the loop compiled once, JAX if it traces
print(pe.params)                                          # below
report = pe.evaluate(n_trials=200, key=1)                 # 200 draws from one key, one vmap
print(report)
print(report.mean, report.std, report.feasibility_rate)

trials = problem.realize(key=1, n_trials=200)             # the same draws, one tree
print(trials["plant"]["m"].shape, trials["d"]["seed"][:3], trials["x_start"].shape)
runs = pe.evaluate(trials, record=True).trajectories      # the 200 Trajectory objects (outside the trace)
```

`pe.params` (the `n` noise block is held at its mean by `realize(None)` when the user runs a
nominal trajectory; its `seed` is still a leaf):

```text
{'controller': {'observer': {'A': …, 'B': …, 'C': …, 'L': …, 'xbar': …, 'ubar': …, 'ybar': …, 'x0': array([0., 0.])},
                'gain': {'K': array([[…]]), 'xbar': …, 'ubar': …}},
 'plant': {'m': 1.0, 'l': 1.0, 'I': 1.0, 'gravity': 9.81, 'd': 0.0},
 'd': {'seed': 0, 'sample_period': 0.01, 'psd': array([0.05])},
 'n': {'seed': 0, 'sample_period': 0.01, 'psd': array([0.001])},
 'cost': {'Q': …, 'R': …, 'S': …, 'xbar': …, 'ubar': …},
 'X': {}, 'U': {}, 'Xf': {},
 'x0': {'mean': array([3.14, 0.]), 'std': array([0.2, 0.1])},      # the laws' own numbers (R140, point 1)
 'laws': {'plant.m': {'lower': 0.9, 'upper': 1.1}},
 'x_start': array([3.14, 0.]),
 'discount_rate': 0.0,
 'r': array([3.14, 0.])}                                            # the loop's free input, held
```

Prints: the `Evaluation` table (`n_trials`, mean, std, worst, the three rates). `trials` carries
`m` with shape `(200,)`, `d.seed` as 200 distinct integers (`child_seed(child_seed(1, "trial i"), "d.seed")`),
`x_start` as `(200, 2)`; `l` and the gains carry no trial axis. The same key draws the same bits on
NumPy and JAX (the counter-based cipher, `core/distributions.py:162`). Plots: `pe.plot_trajectories(runs)`
fans the 200 runs; the LQG lesson compares `report.std` with the covariance prediction
(core-architecture §11, item 1).

### 5.4 The digital loop: `pid % dt @ plant`, `compute_rollout` vs `compute_trajectory`, `problem.compile`, `evaluate`, `jax.grad` of `J` with respect to the gains

```python
import jax
from minilink import Pendulum, PID, QuadraticCost, discretize
from minilink.planning import PlanningProblem

plant = Pendulum()
pid = PID(Kp=8.0, Ki=1.0, Kd=2.0)
DT = 0.01

digital = pid % DT                                    # a Computer: discretize(pid, DT) at base clock DT
loop = digital @ plant                                # a HybridDiagram: a StepDiagramSystem at DT
print(loop)                                           # "PID % 0.01 @ Pendulum", n = 4, sample_period = 0.01
loop.inputs["r"].nominal_value = np.array([0.5])

run = loop.compute_rollout(n_steps=500)               # the scan: ticks only, t = k·DT
dense = loop.compute_trajectory(tf=5.0)               # the orchestrator: solve_ivp between ticks
print(np.max(np.abs(run.of("plant").x - dense.resample(run.t).of("plant").x)))   # ~ integration error
loop.plot_trajectory(signals=("plant:y", "controller:u"))   # the held torque is a staircase

problem = PlanningProblem(plant, cost=QuadraticCost.from_system(plant, Q=np.diag([1, 0.1]),
                                                                 R=0.01 * np.eye(1), xbar=[0.5, 0]),
                          tf=5.0, U=plant.input_box)
pe = problem.compile(digital)                         # dt = the computer's base period; the same scan
report = pe.evaluate()
print(report.J, run.of("plant").with_cost(problem.cost).J)   # the same number: evaluate is compute_rollout's semantics

dJ = jax.grad(lambda Kp, Kd: pe.objective({"controller": {"Kp": Kp, "Kd": Kd}}))(
    pe.params["controller"]["Kp"], pe.params["controller"]["Kd"])
print(dJ)                                             # (∂J/∂Kp, ∂J/∂Kd), reverse mode through the scan
grad_tree = pe.gradient(pe.params)                    # the whole tree, None on integer leaves
```

Prints: the diagram line; a gap of order `1e-6` between the scanned ticks and the orchestrated
dense run resampled at the ticks (RK4 at `DT` against the adaptive solver); two equal `J`s; the two
partial derivatives. The `pe.params` tree:

```text
{'controller': {'Kp': array([8.]), 'Ki': …, 'Kd': …, 'tau': …, 'u_min': …, 'u_max': …, 'e_int_min': …, 'e_int_max': …,
                'x0': array([0., 0.])},                          # the digital PID's start, a design leaf (R124)
 'plant': {'m': 1.0, 'l': 1.0, 'I': 1.0, 'gravity': 9.81, 'd': 0.0},
 'cost': {…}, 'X': {}, 'U': {'lower': array([-5.]), 'upper': array([5.])}, 'Xf': {},
 'x_start': array([0., 0.]), 'discount_rate': 0.0,
 'r': array([0.5])}
```

Plots: the plant's angle (smooth) and the torque (a staircase at `DT`) on one time axis. The
`params` of `controller` are the continuous PID's own: `discretize` forwards the tree, so the
dotted path `controller.Kp` is the same whether the PID is continuous or digital.

### 5.5 Trajectory optimization and MPC: a program compiled once and rebound with new `x0`, plant and cost parameters; the MPC in the loop

```python
from minilink import Pendulum, QuadraticCost, SingletonSet
from minilink.planning import PlanningProblem, TrajectoryOptimizationPlanner, ModelPredictiveController
from minilink.optimization import Optimizer

plant = Pendulum()
goal = np.array([np.pi, 0.0])
problem = PlanningProblem(plant, x_start=[0.0, 0.0], Xf=SingletonSet(goal),
                          cost=QuadraticCost.from_system(plant, Q=np.eye(2), R=0.1 * np.eye(1), xbar=goal),
                          U=plant.input_box, tf=3.0)

planner = TrajectoryOptimizationPlanner(problem, transcription="direct_collocation", n_steps=40,
                                        method="ipopt", backend="jax")
solution = planner.solve()                            # the swing-up: a TrajectorySource policy
solution.plot_trajectory()
print(solution.solver)                                # the record: success, iterations, residuals

program = planner.program                             # the MathematicalProgram, compiled once, params=problem.params
print(program.params.keys())                          # dict_keys(['x_start', 'plant', 'cost', 'X', 'U', 'Xf', 'discount_rate'])
heavy = Optimizer(program, method="ipopt").solve(params={"plant": {"m": 1.3}, "x_start": [0.3, 0.0]},
                                                 z0=solution.solver.z)                # rebind, no recompile
cheap = Optimizer(program, method="ipopt").solve(params={"cost": {"R": 0.01 * np.eye(1)}})
print(program.params_at(heavy.z)["plant"]["m"])      # 1.3: the full tree the plan was solved under

mpc = ModelPredictiveController(planner, sample_period=0.05)      # a StepSystem; its plan z is its state
loop = mpc @ plant                                                # mpc % 0.05 @ plant, the one period implied
dense = loop.compute_trajectory(tf=4.0)                           # the orchestrator: Ipopt is host code
loop.plot_trajectory(signals=("plant:y", "controller:u"))
mpc.plot_horizons(dense)                                          # the planner-specific view stays on the MPC
```

Prints: the planner's record, the program's tree keys, `1.3`. The rebinds change `x0`, the plant's
mass and the cost's `R` through `Optimizer.solve(params=…)` on one compiled program (R130); the
decision `z` is warm-started from the first plan. The MPC's closed loop runs on the orchestrator
because Ipopt is a host solver; `loop.compute_rollout()` refuses with the line to write
(`method="gd", iterations=K` for the traced form, §5.6). Plots: the swing-up plan; the closed-loop
run with the held command; the MPC horizons overlaid.

The `loop.params` tree: `{"controller": {"cost": {…}, "X": {}, "U": {…}, "Xf": {"point": goal}, "plant": {…}, "x0": zeros(n_z)}, "plant": {…}}`.
The MPC's `plant` entry is its model (the problem's plant, possibly mismatched), its outer
`plant` the one simulated; the embedded-model rule of DESIGN §4 reads as two branches of one
tree.

### 5.6 The native fixed-iteration MPC: its closed loop in one scan, tuned by gradient on a cost weight

```python
planner = TrajectoryOptimizationPlanner(problem, transcription="single_shooting", n_steps=20,
                                        method="gd", iterations=15, learning_rate=0.05, backend="jax")
mpc = ModelPredictiveController(planner, sample_period=0.05)      # pure: step = 15 gd steps on the shifted plan
loop = mpc @ plant                                                # a StepDiagramSystem, every block pure

run = loop.compute_rollout(n_steps=80)                            # one lax.scan of 80 ticks, each a scan of 15 steps
loop.plot_trajectory()

pe = problem.compile(mpc)                                         # the same scan, scored
print(pe.evaluate().J)
starts = problem.replace(x_start=Uniform([-0.5, -0.5], [0.5, 0.5])).realize(key=3, n_trials=64)
print(pe.evaluate(starts).mean)                                   # 64 closed loops in one vmap

# Tuning the MPC's own cost weight by the closed-loop score
program = pe.program(wrt=["controller.cost.Q"])                   # J(z) = the closed-loop evaluation; ∇ by reverse mode
tuned = Optimizer(program, method="adam", iterations=200).solve()
print(program.params_at(tuned.z)["controller"]["cost"]["Q"])

kernel_view = planner.as_step_system()                            # the solve as a rollout: z_{k+1} = z_k − η ∇J(z_k)
z_star = run.signals["controller:z"][:, -1]                       # the last plan of the run
print(kernel_view.jacobian("step", "x")(z_star, None, 0))          # I − η H at the optimum: the step size's stability
```

Prints: `J` of the nominal loop; the mean over 64 starts; the tuned `Q`; the eigenvalues of
`I − ηH` inside the unit circle. The tree exposed by `pe.params`:

```text
{'controller': {'cost': {'Q': …, 'R': …, 'S': …, 'xbar': …, 'ubar': …},      # the MPC's own cost, tunable
                'plant': {…}, 'X': {}, 'U': {…}, 'Xf': {…},
                'learning_rate': 0.05,                                       # a kernel parameter, a float leaf
                'x0': zeros(20)},                                            # the first plan (warm start), a design leaf
 'plant': {…}, 'cost': {…}, 'X': {}, 'U': {…}, 'Xf': {…}, 'x_start': …, 'discount_rate': 0.0}
```

What is static: `n_steps=20`, `iterations=15`, the scheme, the substeps (constructor arguments,
R131). What varies without retracing: every leaf above. Plots: the closed-loop run; the training
curve of the tuning (`Optimizer(..., verbose=True)`).

### 5.7 Value iteration and RL reading the same `J` as the evaluator; a `QuadraticField` as terminal cost and Lyapunov `V`

```python
from minilink import Pendulum, QuadraticCost, QuadraticField, BoxSet
from minilink.planning import PlanningProblem, DynamicProgrammingPlanner, ReinforcementLearningPlanner, LQRPlanner, compare
from minilink.analysis import lyapunov_certificate

plant = Pendulum()
goal = np.array([np.pi, 0.0])
lqr = LQRPlanner(PlanningProblem(plant, cost=QuadraticCost.from_system(plant, Q=np.eye(2), R=0.1*np.eye(1), xbar=goal))).solve()
V = lqr.cost_to_go                                     # a QuadraticField(goal, P): the Riccati value
print(V.value(goal + [0.1, 0.0]), V.gradient(goal + [0.1, 0.0]))

problem = PlanningProblem(plant, cost=QuadraticCost.from_system(plant, Q=np.eye(2), R=0.1*np.eye(1), xbar=goal)
                                  + V.as_cost(terminal=True),          # h(x_N) = xᵀ P x: the LQR value as terminal cost
                          X=BoxSet([-np.pi, -8], [3*np.pi, 8]), U=plant.input_box, tf=5.0, discount_rate=0.05)

DT = 0.05
disc = problem.discretize(DT)                          # the same PlanningProblem on discretize(plant, DT)
print(disc.n_steps, disc.discount_factor())            # 100, e^{−0.05·0.05}
x_next, c = disc.transition(np.array([0.0, 0.0]), np.array([1.0]), 0)   # the one primitive: (x_{k+1}, c_k)

vi = DynamicProgrammingPlanner(problem, x_grid=(101, 101), u_grid=11, dt=DT).solve()       # Q = c_k + γ J(x_{k+1})
rl = ReinforcementLearningPlanner(problem, dt=DT, algorithm="ppo").solve(timesteps=400_000)  # r_k = −c_k
cmp = compare(problem, VI=vi, PPO=rl, LQR=lqr, dt=DT, n_trials=1)   # every entry scored by evaluate on disc
print(cmp)                                             # one table: the planner's own J and the evaluator's, side by side
cmp.plot_cost_to_go(); cmp.plot_control_law()

region = lyapunov_certificate(lqr.policy @ plant, V=V, x_bar=goal)   # V̇ = ∇V · f < 0 on V.as_set(upper=level)
print(region.level, region.V.as_set(upper=region.level).bounding_box())
region.plot()
```

Prints: `V` and `∇V` at a point; `100` and `0.9975`; a transition; the comparison table whose
column "evaluated J" agrees with each planner's reported `J` to the planner's own approximation
(the grid for VI, the critic for PPO), because all three read `disc.transition`; the certified
level and the ellipsoid's box. Plots: the three cost-to-go maps on one scale, the three laws, the
region of attraction with `V`'s level set. The field's `params` is `{"center": goal, "P": P}`; the
problem's `cost` tree gains a term `QuadraticField` under `SumCost`'s term ids.

### 5.8 The research frontier: a stochastic hybrid loop with a seeded policy, vmapped over seeds, tuned by gradient

```python
import jax
from minilink import Pendulum, WhiteNoise, Gaussian, QuadraticCost
from minilink.control import NeuralPolicyController, GaussianPolicy
from minilink.planning import StochasticPlanningProblem
from minilink.optimization import Optimizer

plant = Pendulum()
mean = NeuralPolicyController(plant, hidden=(32, 32), seed=0)              # u = μ_θ(x), weights in params["mlp"]
policy = GaussianPolicy(mean, log_std=np.log(0.3), sample_period=0.02)     # u = μ_θ(x) + σ ε(seed, ⌊t/Δ⌋)
digital = policy % 0.02                                                     # sampled at its own period

problem = StochasticPlanningProblem(
    plant, cost=QuadraticCost.from_system(plant, Q=np.diag([5, 0.5]), R=0.05*np.eye(1), xbar=[np.pi, 0]),
    tf=4.0, x0=Gaussian([0.0, 0.0], std=[0.5, 0.5]), params={"m": Uniform(0.8, 1.2)},
    disturbances={"d": WhiteNoise(1, psd=0.1, sample_period=0.02)},
    X=plant.state.box, U=plant.input_box, infeasible_cost=True)

pe = problem.compile(digital)                                               # the hybrid loop at 0.02, one scan
trials = problem.realize(key=7, n_trials=256)                               # starts, masses, d.seed, controller.seed
print(trials["controller"]["seed"][:4])                                     # four distinct policy seeds

report = pe.evaluate(trials)                                                # 256 seeded rollouts, one vmap, one jit
print(report.mean, report.cvar(0.1), report.feasibility_rate)

program = pe.program(wrt=["controller.mean.mlp", "controller.log_std"], key=7, n_trials=256)   # the sample-average program
best = Optimizer(program, method="adam", iterations=2000, learning_rate=3e-3).solve(verbose=True)
tuned = program.params_at(best.z)
print(pe.evaluate(tuned, key=99, n_trials=1000).mean)                       # held-out draws

mean_law = mean % 0.02                                                      # the deterministic face: the mean block, sampled
print(problem.compile(mean_law).evaluate(trials, params={"controller": tuned["controller"]["mean"]}).mean)
```

`seed` is an integer leaf and never `None` in a tree (R118): the deterministic face of a seeded
policy is a construction choice (`GaussianPolicy(…, seed=None)` or the mean block itself), not a
parameter value, so the mean law is evaluated by compiling that block with the tuned `mean`
subtree. The tree:

```text
{'controller': {'mean': {'mlp': {'W0': …, 'b0': …, 'W1': …, 'b1': …, 'W2': …, 'b2': …}, 'u_mid': …, 'u_half': …},
                'log_std': array([-1.2]), 'seed': 0, 'sample_period': 0.02},
 'plant': {…}, 'd': {'seed': 0, 'sample_period': 0.02, 'psd': array([0.1])},
 'cost': {…}, 'X': {…}, 'U': {…}, 'Xf': {},
 'x0': {'mean': …, 'std': …}, 'laws': {'plant.m': {'lower': 0.8, 'upper': 1.2}},
 'x_start': array([0., 0.]), 'discount_rate': 0.0}
```

Prints: four seeds; the Monte Carlo summary; the optimizer's one line per update; the held-out
mean. Plots: `pe.plot_trajectories(pe.evaluate(trials[:16], record=True).trajectories)`, sixteen
seeded runs. The gradient is reverse mode through 256 scans of 200 ticks, the noise entering by
reparameterization (`ε` is a pure function of `(seed, k)`, so `∂u/∂log_std = σ ε`); the price
switch is a step with no gradient (as today).

---

## 6. The final API, object by object

Each entry: the signatures, the `params` tree it exposes, what exists today (`file:line`) and
what changes. Equation paths keep RULES 5.1's bare signatures; the type hints below are for
the reader.

### 6.1 `System`, `DynamicSystem`, `StepSystem`

```python
class System:                                   # core/system.py:26
    params: dict                                 # {} when the block has no parameter (R140, point 2)
    x0: ndarray                                  # the block's own start (R124)
    def h(self, x, u, t=0.0, params=None): ...   # y = h(x, u, t; p)
    def tf(self, x, u, t=0.0, params=None): ...  # body poses
    def realize(self, key): ...                  # the params of one realization (seeds under key)
    # composition: __matmul__ (@), __rshift__ (>>), __add__ (+), __mod__ (%)

class DynamicSystem(System):                    # core/system.py:516
    def f(self, x, u, t=0.0, params=None): ...   # ẋ = f(x, u, t; p)

class StepSystem(System):                       # core/discrete/step_system.py:16
    sample_period: float = 1.0                   # the physical length of one tick (structural)
    def step(self, x, u, k=0, params=None): ...  # x_{k+1} = step(x_k, u_k, k; p)
    def h(self, x, u, k=0, params=None): ...     # y_k = h(x_k, u_k, k; p)
```

**Today.** As above, minus `sample_period`; `h`'s default stub builds `np.zeros(p)` (T1).
`StepSystem.rollout` caches a run (`step_system.py:49`): it goes (RULES 2.4); `self.traj` is the
one cache, on every kind.

**Changes.** `sample_period` on `StepSystem` (set by `discretize` and `%`); `params = {}` on a
parameterless block; the cache gone; the `h` stub on `xp`.

### 6.2 `DiagramSystem`, `StepDiagramSystem`, the operators, `discretize`, `%`, `Computer`, `HybridDiagram`

```python
class DiagramSystem(DynamicSystem):             # core/diagram.py
    subsystems: dict[str, System]
    params: dict                                 # {block_id: block.params}, a live view (DESIGN §4)
    x0: ndarray                                  # a view: stacked from the blocks; the setter writes through (R124)
    def x0_at(self, **block_starts) -> ndarray: ...   # the stacked start with named blocks' slices replaced
    def connect(self, src_id, src_port, dst_id, dst_port): ...
    def expose(self, *names): ...
    def trajectory_of(self, block, traj=None) -> Trajectory: ...
    def compile(self, backend="numpy") -> DiagramEvaluator: ...

class StepDiagramSystem(StepSystem):            # core/discrete/step_diagram.py:13
    # the same wiring surface (WiredDiagramMixin); every block a StepSystem or a static System
    def step(self, x, u, k=0, params=None): ...  # propagation within the tick (R125); a cycle without a delay refused at wiring

def discretize(system, dt, *, solver="rk4", substeps=1) -> StepSystem   # core/discrete/discretization.py (moved from analysis/)
    # a DynamicSystem → Discretization(StepSystem): step = step_body(f, …) over one period, h at t_k = k·dt,
    #   sample_period = dt, source kept, params forwarded (the tree is the source's)
    # a static System    → Sampled(System): h read at t_k = k·dt (a static step block, n = 0)
    # solver="exact" on a linear model (the matrix exponential); "tustin" later for linear blocks

class StepSchedule:                             # core/discrete/schedule.py (from simulation/discrete/computer.py:227)
    dt_base: float
    fire: dict[str, int]                         # block id → divisor d; a block fires on k mod d == 0
    @classmethod from_rates(cls, dt_base, rates_hz): ...

class Computer(StepDiagramSystem):              # core/discrete/computer.py (today a runtime, simulation/discrete/computer.py:34)
    schedule: StepSchedule                       # structural; sample_period = schedule.dt_base
    # state = [block states ; hold registers of every block with d > 1]
    # step: for each block in evaluation order, fired = (k % d == 0);
    #       y_i = where(fired, h_i(x_i, u_i, k), held_i); x_i⁺ = where(fired, step_i(x_i, u_i, k), x_i); held_i⁺ = y_i

System.__mod__(self, schedule) -> Computer       # block % dt or block % StepSchedule
    # a StepSystem is scheduled as is; a DynamicSystem with state becomes discretize(self, dt_base);
    # a static block becomes its sampled form; a StepDiagramSystem's blocks keep their ids

Computer.__matmul__(self, plant) -> HybridDiagram
class HybridDiagram(StepDiagramSystem):         # core/discrete/hybrid_diagram.py:30 (today a plain class around a Computer and a plant)
    # blocks: "controller" (the computer) and "plant" = discretize(plant, dt_base, solver=, substeps=);
    # the boundary ports as today (resolve_hybrid_feedback_ports, hybrid_composition.py:154);
    # roles() answers plant / controller / observer as a continuous loop does
    def compute_rollout(self, n_steps=None, *, tf=None, x0=None, u=None, backend="numpy") -> Trajectory   # the scan at the ticks, t = k·dt_base
    def compute_trajectory(self, t0=0.0, tf=10.0, *, solver="scipy", x0=None, backend="numpy") -> Trajectory   # the orchestrator, dense

def hybrid_closed_loop(computer, plant, *, solver="rk4", substeps=None, **ports) -> HybridDiagram   # the explicit form; substeps=None reads the plant's time constant
```

The operators on step systems (R126): `@`, `>>` and `+` accept two step operands and return a
`StepDiagramSystem`, built by `core/assembly.py` and `core/closed_loop.py` with the same port
roles; a continuous and a step operand together are refused with the line to write (`%` or
`discretize`). `Computer @ plant` is the one place a step operand meets a continuous one, and it
discretizes the plant itself.

**Today.** `DiagramSystem` stores stacked `x0`, `n`, `state` copies restacked by `refresh()`
(`core/wiring.py`); `Computer` is a mutable NumPy runtime with a double buffer and a one-tick lag
(`computer.py:114–171`); `HybridDiagram` is a dataclass holding a `Computer` and a plant
(`hybrid_diagram.py:30`) with `compute_trajectory` on `HybridSimulator`; `@`, `>>`, `+` refuse step
systems; `%` refuses a controller with continuous states; `discretize` lives in `analysis/`.

**Changes.** The views and write-through (R124); `Computer` and `StepSchedule` to `core/discrete/`
as pure structure; `HybridDiagram` a `StepDiagramSystem` subclass (R127) with two verbs; the
operators on the step algebra (R126); `%` discretizing (§3.2); `discretize` moved; the
`core → simulation` lazy import gone. `x0_at` is new (one helper, because the lessons set one
block's start).

### 6.3 `Trajectory`, `Set`, `CostFunction`, `Distribution`, the sources, `Field`

```python
@dataclass(frozen=True)
class Trajectory:                               # core/trajectory.py:52, a JAX pytree
    t: ndarray; x: ndarray; u: ndarray           # (N,), (n, N), (m, N)
    signals: dict[str, ndarray]                  # "block:port" (R134, line 3)
    roles: dict                                  # {"plant": {"x": (a, b), "u": "plant:u"}, …}
    def of(self, role) -> Trajectory: ...
    def with_cost(self, cost, *, discount_rate=0.0, params=None) -> Trajectory: ...   # adds the signals; .J the total
    def resample(self, t) -> Trajectory: ...
    # a step rollout is a Trajectory with t = k·sample_period (StepRollout folds in)

class Set:                                      # core/sets.py:22
    params: dict                                 # {} when none; BoxSet: {"lower", "upper"}; BallSet: {"center", "radius"}; SingletonSet: {"point"}
    def margin(self, z, t=0.0, params=None): ...
    def contains(self, z, t=0.0, params=None): ...
    def sample(self, key, n=None, params=None): ...
    def bounding_box(self): ...
    def as_cost(self, weight=1.0, shaping=None) -> CostFunction: ...   # Field.as_cost on the margin seen as a field (R133)
class InputSet: margin(self, u, x=None, t=0.0, params=None) ...        # core/sets.py:90

class CostFunction:                             # core/costs.py:29
    params: dict
    def g(self, x, u, t=0.0, params=None): ...   # the running cost
    def h(self, x, t=0.0, params=None): ...      # the terminal cost
    def g_max(self, X, U, params=None): ...; def h_max(self, X, params=None): ...
    # + → SumCost (terms by id), * → ScaledCost ({"weight", "cost"}); integrate() leaves (the problem owns the quadrature, §6.6)

class Distribution:                             # core/distributions.py:24
    params: dict                                 # Gaussian {"mean", "std"} (or "cov"), Uniform {"lower", "upper"}, Particles {"points"}
    def sample(self, key, n=None, params=None): ...   # key: an int, a NumPy generator or a JAX key; (dim,) or (n, dim)
    mean: property                               # reads params

class WhiteNoise(Source):                       # blocks/sources.py:211 — as today
    def __init__(self, p=1, *, psd=1.0, sample_period=0.01, seed=0, hold="zoh", port="y")
    params = {"seed", "sample_period", "psd"};  h(x, u, t, params): w_k = (W/Δ)^{1/2} ε(seed, ⌊t/Δ⌋)
class TrajectorySource(Source):                 # params = {"values": (N, m)}; the knots t are structural; h interpolates

class Field:                                    # core/fields.py (from planning/spatial/state_fields.py)
    params: dict
    def value(self, x, u=None, t=0.0, params=None): ...
    def gradient(self, x, u=None, t=0.0, params=None): ...   # jax.grad under JAX, central differences on NumPy
    def as_set(self, lower=None, upper=None) -> Set: ...
    def as_input_set(self, lower=None, upper=None) -> InputSet: ...
    def as_cost(self, weight=1.0, shaping=None, *, terminal=False) -> CostFunction: ...   # g = w·shaping(V); terminal=True puts it on h
class QuadraticField(Field):                    # params = {"center", "P"}; value = (x−c)ᵀP(x−c); as_set(upper=ℓ) → EllipsoidSet with sample
```

**Today.** `Trajectory` as above but `plant.u` (becomes `plant:u`) and `StepRollout` apart;
`Set.params = None` on the library sets, `margin` reads `params` (`sets.py:41`); `CostFunction`
as above with `integrate` (`costs.py:95`, the trapezoid); `Distribution.sample(key, n)` with
`mean`/`std` attributes (`distributions.py:46–67`); `WhiteNoise` exactly this; `StateField` in
`planning/spatial/state_fields.py` with `as_constraint`.

**Changes.** `params = {}` and the library sets reading theirs (A5); `sample(key, n, params=None)`
(R140, point 1); `TrajectorySource.values` a leaf; `Field` promoted with `as_set` /
`as_input_set` / `as_cost` (R133) and `terminal=`; `Set.as_cost` on the margin field; the
shaping functions to `core/fields.py`.

### 6.4 `PlanningProblem`, `StochasticPlanningProblem`, `realize`

```python
class PlanningProblem:                          # planning/problems.py:29
    def __init__(self, sys, x_start=None, x_goal=None, cost=None, X=None, U=None, X0=None, Xf=None,
                 tf=None, infeasible_cost=None, discount_rate=0.0, *, disturbances=None)
    # disturbances={"d": WhiteNoise(...)}: source blocks only, wired onto the plant's ports; sys becomes that diagram,
    #   the role "plant" the original system (R128). A bare Distribution is refused naming the fix.
    params: dict           # {"plant", "cost", "X", "U", "Xf", "x_start", "discount_rate"} + a key per source, a view (R140, point 3)
    sys: System            # the plant, or the diagram of plant and sources
    def discretize(self, dt, *, solver="rk4", substeps=1) -> PlanningProblem    # the same parts on discretize(plant, dt, …)
    is_discrete: property  # sys is a StepSystem
    n_steps: property      # round(tf / sys.sample_period) on a discrete problem
    def discount_factor(self, dt=None) -> float    # e^{−ρ dt}; the problem's own Δ when dt is None
    def transition(self, x, u, k, params=None) -> (x_next, c_k)   # §6.6, on a discrete problem
    def stage_cost(self, x, u, k, params=None) -> c_k
    def compile(self, controller, dt=None, *, solver="rk4", substeps=1, backend="auto", episode_length=None) -> ProblemEvaluator
    def realize(self, key, n_trials=1) -> dict     # one tree (§3.1); deterministic: the sources reseeded per trial, nothing else drawn
    def replace(self, **changes) -> PlanningProblem
    def cost_bound(self, tf=None, *, params=None) ...; def infeasibility_price(...)

class StochasticPlanningProblem(PlanningProblem):     # planning/problems.py:506
    def __init__(self, sys, ..., x0: Distribution, params: dict[str, Distribution] = None, criterion="expectation", disturbances=None)
    params: dict           # the parent's, plus "x0": the start law's numbers and "laws": each parameter law's, keyed by its path
    def realize(self, key, n_trials) -> dict       # x_start drawn (n, n_x); each drawn leaf (n, …); every source's and seeded block's seed per trial
```

**Today.** `x0_distribution`, `params_distribution`, `disturbances={port: Distribution}`
(`problems.py:551`); `realize(key, n) -> (x0s, {"params", "disturbances"})` (`problems.py:638`);
`evaluate` holds the disturbance ports at nominal and warns.

**Changes.** The keyword names (`x0=`, `params=`, as core-architecture §0 writes them);
disturbances as blocks on both problem kinds (R128); one tree from `realize` (§3.1);
`discretize`, `transition`, `stage_cost`, `n_steps`, `is_discrete` (§3.4); `cost_bound(params=)`
so the price follows an overridden cost (the exact bound of a quadratic is in-trace; a sampled
bound is computed once at compile and documented as nominal).

### 6.5 `problem.compile`, `ProblemEvaluator`, `evaluate`, `Evaluation`

```python
pe = problem.compile(controller, dt=None, *, solver="rk4", substeps=1, backend="auto", episode_length=None)
# controller: a continuous block or box (closed as controller @ plant, then discretized as a whole at dt);
#             a Computer or HybridDiagram or StepSystem (dt defaults to its sample_period; another dt is refused);
#             a source block (open loop); a PlanningSolution (its policy); a built loop with a "plant" block.
# The run is always a discrete problem on a step loop: problem.discretize(dt, solver, substeps) scored on the loop's scan.

class ProblemEvaluator:                         # planning/evaluator.py:40
    params: dict                                 # the loop's blocks by id (+ "x0" per stateful block), the problem's parts, the free inputs
    def trajectory(self, params=None, key=None) -> Trajectory          # one run, every signal recorded
    def evaluate(self, params=None, key=0, n_trials=None, *, record=False) -> Evaluation
    def objective(self, params=None, key=0, n_trials=None) -> float    # evaluate(...).objective, a scalar for grad
    def gradient(self, params=None, key=0, n_trials=None) -> dict      # the tree; None on integer leaves (§3.5)
    def realize(self, key, n_trials, params=None) -> dict              # the problem's realize merged into this tree
    def program(self, wrt, bounds=None, key=0, n_trials=None, *, params=None) -> MathematicalProgram
    def pack(self, params, wrt) -> ndarray;  def unpack(self, z, wrt, params=None) -> dict
    def merge(self, override) -> dict            # a partial tree merged once onto params (R131)

def evaluate(problem, controller, *, dt=None, key=0, n_trials=None, **compile_kwargs) -> Evaluation   # the facade

@dataclass(frozen=True)
class Evaluation:                               # planning/evaluation.py:27, a pytree record; unchanged
    cost, J, feasibility_margin, failed, reachability_margin, reached, x0; trajectories (outside a trace)
    objective, mean, std, worst, cvar(alpha), feasibility_rate, reachability_rate, success_rate
```

**Today.** As above minus `objective`, `gradient`, `merge` as public names; `run` integrates the
continuous loop with `ev.rollout` (`evaluator.py:355`) and scores by `score_run`'s trapezoid;
`require_continuous` refuses a `StepSystem` controller (`evaluator.py:789`).

**Changes.** Every controller kind through one scan (the sampled face, `require_continuous`
gone); the score by `transition` (§6.6); the three public names; `realize` one tree.

### 6.6 One score: the discrete reading and its kernel

```python
# On a PlanningProblem whose sys is a StepSystem with sample_period Δ (built by discretize(dt), or natively discrete):
#   t_k = k Δ ; N = round(tf / Δ) ; γ = e^{−ρΔ}
#   transition(x, u, k; p) = (x_{k+1}, c_k):
#       Discretization: the augmented step [x; c] by step_body with ċ = e^{−ρ(τ − t_k)} g(x, u_k, τ)  (c(0) = 0)
#       native StepSystem: x_{k+1} = step(x, u, k; p), c_k = g(x, u, t_k; p["cost"])
#   score of a run (x_0..x_N, u_0..u_{N−1}):
#       finite  = all finite;  x, u = where(finite, ·, 0)
#       m       = min_k [ X.margin(x_k, t_k), U.margin(u_k, x_k, t_k) ]      (the ticks; substep margins later)
#       cost    = Σ_k γ^k c_k + [tf finite] γ^N h(x_N, t_N) ;  +inf when not finite
#       failed  = ¬finite ∨ m < 0 ;  reached = Xf.margin(x_N, t_N) ≥ 0
#       J       = cost, or with the price: where(finite, cost, bound) + where(failed ∨ ¬reached, price, 0)
# Readers: ProblemEvaluator (c_k by a vmap over k of transition's cost half, after the loop's scan);
#          DynamicProgrammingPlanner (Q = c_k + γ J(x_{k+1}), both halves per (x, u) node);
#          RolloutEnvironment (r_k = −c_k, the exit a truncation that still reports this J);
#          shooting (the objective is this sum on its own scan); Sys2Gym (the environment's view).
```

**Today.** Three quadratures (`dp.py:270,309`: `g·dt`; `environment.py:162`: `−g·dt`;
`evaluation.py:199–205`: the trapezoid of `costs.py:95`); two failure rules. **Changes.** The rule
above, one commit per reader with a closeness report; `CostFunction.integrate` leaves (the problem
owns the quadrature); `score_run` keeps its name and reads `transition`.

### 6.7 `MathematicalProgram`, `Optimizer`, the kernels

```python
@dataclass(frozen=True)
class MathematicalProgram:                      # optimization/mathematical_program.py:26
    n_z: int
    J: Callable; h: Callable | None = None; g: Callable | None = None
    lower: ndarray | None = None; upper: ndarray | None = None
    params: dict | None = None                   # None: J(z); a tree: J(z, params), params=None reading this default (R130)
    grad_J, hess_J, jac_h, jac_g: Callable | None
    backend: str = "numpy"; metadata: dict
    def params_at(self, z) -> dict               # an exported program: the full tree with z written into its wrt leaves

class Optimizer:                                # optimization/optimizer.py:75
    def __init__(self, program, z0=None, *, method="auto", backend=None, iterations=None, tolerance=None, options=None, **method_kwargs)
    # method: "gd", "adam" (native kernels), "l-bfgs", "scipy_slsqp", "scipy_trust_constr", "ipopt" (wrapped, host);
    # iterations=K runs a native kernel K times as one scan (traced on JAX); tolerance= runs it to a stop (a while_loop, forward-only)
    adapter: OptimizerBackend                    # was .backend (R134)
    def solve(self, *, z0=None, params=None, callback=None, verbose=False) -> OptimizationResult   # params: a partial tree, merged once

class GradientDescent:                          # optimization/kernels.py (new; today a loop in optimizers/first_order.py:56)
    params = {"learning_rate": η}
    def init(self, z0, oracle, params=None) -> state        # state = {"z": Π(z0), "k": 0}
    def step(self, state, oracle, params=None) -> state     # z ← Π(z − η ∇J(z; p)), oracle = the compiled program's gradient with its tree bound
class Adam: params = {"learning_rate", "b1", "b2", "eps"}; state = {"z", "m", "v", "k"}   # the shape of Adam in first_order.py:135

def as_step_system(kernel, program) -> StepSystem           # x = the packed state, step(x, u, k, params): one iteration; u = the program's params leaves (none by default)
```

**Today.** `J(z)` only; `ParametricMathematicalProgram` varies `x0` (`trajectory_optimization/parametric_program.py`);
`Optimizer(program, z0, *, method, compile_backend, …)` with `.backend` the adapter; `gd` and
`adam` a NumPy loop with Adam's update jitted. **Changes.** `params` on the one noun;
`solve(params=)`; `backend=` and `adapter` (R134); `iterations=` the fixed-`K` form of the native
kernels as one scan; the kernels as `init`/`step` objects with `params` (R132) and the step-system
view; `params_at`. The program evaluator binds the tree once per solve and jits `(z, params)`
once (§3.6).

### 6.8 The policies: `GaussianPolicy`, `NeuralPolicyController`, `LookupPolicy`, the classical laws

```python
class GaussianPolicy(System):                   # control/policies.py (new)
    def __init__(self, mean: System, log_std, *, sample_period, seed=None, squash=None)
    # ports: those of mean (x or y in, u out); params = {"mean": mean.params, "log_std": …, "seed": int, "sample_period": Δ}
    def h(self, x, u, t=0.0, params=None): ...   # u = μ(x; p["mean"]) + exp(p["log_std"]) ⊙ ε(p["seed"], ⌊t/Δ⌋)   (seed=None at construction: u = μ)
    def log_prob(self, x, u, t=0.0, params=None): ...
    def entropy(self, x, t=0.0, params=None): ...
    def realize(self, key): ...                  # a fresh seed under key
    is_random = True                             # seed is not None
```

A policy is a `System`; `policy % Δ` samples it at its own period (the sample index is then the
computer's tick, R129); `policy @ plant` closes a continuous loop where the draw is held over
`Δ` by `h` itself. `NeuralPolicyController` (`control/neural.py:15`) is the mean block and keeps
its name until the v1.0 rename (R134, line 4). The RL learners' internal `StochasticPolicy`
(`learning/.../policy.py`) is replaced by this block, the heads' `theta = {"mlp", "head"}`
becoming `{"mean": {"mlp"}, "log_std"}`; the collectors move from their own keys to per-episode
seeds (`realize`), the RL numbers moving once.

### 6.9 The planners, the planner interface, `PlanningSolution`, `ModelPredictiveController`

```python
class Planner:                                  # planning/planner.py:21
    def __init__(self, problem): ...
    def solve(self, *, params=None, z0=None, evaluate=False, n_trials=None, **kwargs) -> PlanningSolution   # host
    # The pure trio an MPC reads (R135), implemented by the trajectory-family planners (trajopt, MPPI, the native solver):
    def decision_dimension(self) -> int
    def solve_from(self, z0, params) -> tuple[ndarray, ndarray]       # (z, success): pure arrays; a host solver inside is allowed and marks the planner host-only
    def warm_start(self, z, params) -> ndarray                        # the shift, the planner's knowledge of its decision
    def plan_of(self, z, params) -> Trajectory                        # unpack
    traces: bool                                 # whether solve_from traces (a native kernel: yes; SciPy, Ipopt: no)

@dataclass(frozen=True)
class PlanningSolution:                         # planning/results.py:21 — as R138
    problem; policy: System; solver: record; trajectory: Trajectory | None; evaluation: Evaluation | None; cost_to_go: Field | None
    success, open_loop, method; plot_trajectory, plot_cost_to_go, plot_control_law, plot_cost

class ModelPredictiveController(StepSystem):    # planning/trajectory_optimization/mpc/controller.py (today the Mixin + two classes)
    def __init__(self, planner, *, sample_period, warm_start=True, parameter_ports=())
    # ports: y (the measurement) [+ parameter_ports, each feeding a leaf by path, nominal when unwired]; outputs u, x_ff, z
    # state x = the plan z of the last tick; sample_period = the planner's period; params = the planner's problem tree + "x0"
    def step(self, x, u, k=0, params=None): ...  # z⁺ = solve_from(warm_start(z), params with x_start = y, …)   — one solve per tick, no latch
    def h(self, x, u, k=0, params=None): ...     # u_k = the first input of z (read from the state: the one-period delay, R123)
    # % reads sample_period; @ plant builds the HybridDiagram; compute_rollout refuses a host-only planner naming the orchestrator
```

**Today.** `Planner.solve_trajectory_from(x0, params=None)` binds `x0` only; the MPC probes the
planner with `hasattr` (`controller.py:905`) and solves through `MPCTickLatch` (`controller.py:567`);
`dt_mpc`, `warm_start=False` as a static `System`. **Changes.** The trio; one `StepSystem` class
with the solve in `step` and the latch gone (R123); `sample_period=` (R134); `parameter_ports=`;
the warm start as the state's start (`x0`, a design leaf: the first plan).

### 6.10 The names, in one place (R134)

`backend=` the array module everywhere (`compile_backend=` goes), `solver=` the time integration,
`method=` the optimization algorithm, `sample_period=` a block's period, `dt=` a tool's grid;
`Optimizer.adapter`; `mount(geometry, on=sys, frame=)`; `.` walks a tree and `:` names a signal;
`…Policy` for learned laws; `Compiled…` for the evaluators at the v0.9 pass; `parameter_ports=`
and `params_at(z)`.

---

## 7. The architecture under it

### 7.1 The layers

```text
 user scripts, notebooks                      the eight pipelines of §5
 ───────────────────────────────────────────────────────────────────────────────────────────
 planners & learners        LQRPlanner · TrajectoryOptimizationPlanner · DynamicProgrammingPlanner · RL · MPPI (v0.4)
 the MPC block              ModelPredictiveController(planner)             planning/
 the task and its score     PlanningProblem · discretize · transition · ProblemEvaluator · Evaluation · compare
 ───────────────────────────────────────────────────────────────────────────────────────────
 the solver boundary        MathematicalProgram(params) · Optimizer · kernels (gd, adam) · wrapped solvers   optimization/
 ───────────────────────────────────────────────────────────────────────────────────────────
 the two runners            runner 1: compile(sys).rollout = one lax.scan; rollout_batch = vmap   core/compile/
                            runner 2: Simulator (solve_ivp) · HybridSimulator (ticks + solve_ivp) · Realtime   simulation/
 ───────────────────────────────────────────────────────────────────────────────────────────
 the step algebra           StepSystem · StepDiagramSystem · Computer · HybridDiagram · discretize · %   core/discrete/
 the continuous algebra     System · DynamicSystem · DiagramSystem · @ >> +                               core/
 the textbook objects       Trajectory · Set · CostFunction · Distribution · Field · the sources         core/, blocks/
 the one contract           f(args, params=None); one tree; float leaves + integer seeds; deterministic
```

The arrow of dependency points down. The step algebra is the one runtime: a continuous object
enters it through `discretize` (a plant), `%` (a controller) or `problem.discretize` (a task);
the continuous algebra stays primary for what is continuous (the ODE solvers, linearization,
frequency analysis, collocation, the continuous LQR).

### 7.2 Module layout and the dependency law

```text
minilink/
  core/            system, diagram, wiring, assembly, closed_loop, operators, roles, topology,
                   trajectory, sets, costs, distributions, fields, params (the tree ops: lookup, with_leaf, merge, flatten, axes),
                   discrete/ (step_system, step_diagram, discretization, schedule, computer, hybrid_diagram, hybrid_composition),
                   compile/ (compiler, plan, evaluators/), facades/, backends, exports
  blocks/ dynamics/ control/ estimation/      → core                          (the library of systems and laws)
  analysis/                                   → core                          (linearize, frequency, lyapunov; discretize re-exported from core)
  simulation/                                 → core, analysis                (Simulator, HybridSimulator, Realtime, solvers)
  optimization/                               → core.backends only            (programs, Optimizer, kernels, adapters)
  planning/                                   → core, analysis, simulation, optimization
  learning/                                   → planning
  graphical/                                  ← loaded lazily by the facades
  interfaces/ experimental/                   → anything
```

**The law.** `core` imports nothing above it, `optimization` never imports `planning`, `planning`
never imports `learning`, `simulation` never imports `optimization` (today it does, for display
constants: S65). The one exception R100 wrote (`core/discrete` → `simulation/discrete`) disappears
because `Computer` and `StepSchedule` become core structure (§3.3). `discretize` moves to
`core/discrete/discretization.py` (it is the step algebra's own conversion, and `%` needs it);
`analysis.discretize` is the same function exported from `minilink`, not an alias. The four copies
of the tree operations (`planning/evaluator.py:991–1044`, `planning/problems.py:896–905`,
`core/compile/evaluators/jacobian.py`, `core/assembly.py`) become `core/params.py`.

### 7.3 The runtime model: what compiles once, what varies, where host code stays

| | Compiled once (structure) | Varies per call (the tree) | Host only |
| --- | --- | --- | --- |
| a leaf `step` / `f` | the equations, the port layout, `solver`, `substeps` | every leaf of `params` | — |
| a diagram | the plan (evaluation order, slices), the schedule's divisors, hold-register layout | the blocks' trees | — |
| `rollout` | `N` (the scan length), the knobs | `x0`, `u_seq`, the tree | — |
| `rollout_batch` / `evaluate` | the batch layout (which leaves carry a trial axis) | the batched tree | the draw of the per-trial seeds (`child_seed`, a few integers) |
| `score` | the problem's parts (cost, sets as objects), `N`, the price switch | the tree, `x_start` | a sampled `cost_bound` for a non-quadratic cost (once, at compile) |
| a program | `n_z`, the transcription, the rows | the tree at `solve(params=)` | the wrapped solver's loop (SciPy, Ipopt); `callback` |
| the native kernel | `K`, the box | `learning_rate`, the program's tree | — |
| the MPC tick | `K`, `n_steps`, the planner's structure | the plan `z` (state), `y`, the tree | a host planner's solve (then runner 2 only) |
| `compute_trajectory` on a hybrid | the computer's compiled `step` | the tree | the adaptive plant integration between ticks, the realtime I/O |

Retracing happens only when structure changes: a new batch layout, a different problem part
object, a leaf whose shape changes. `merge` refuses a leaf of a new shape (R131) so a typo does not
silently retrace. The jit cache is per evaluator, keyed on the static knobs; `params` is a pytree
argument, never a closure constant, so a thousand `evaluate(p)` calls trace once.

**The second runner is a shell, not a second model.** `HybridSimulator` ticks the computer's
compiled `step` and `outputs` (the same functions runner 1 scans), integrates the plant with
`solve_ivp` between ticks, restarting at each sample time, and records the dense plant
trajectory. The step diagram is its oracle at the ticks (`rtol` of the integrator). The realtime
simulator is the same shell with I/O between ticks. No `pure_callback` is offered in the core: a
host solver inside a traced scan would compile but could not be differentiated, and the
orchestrator runs it without pretending.

### 7.4 The JAX traceability and efficiency argument, path by path

**P1. A discretized plant.** `Discretization.step` is `step_body(f, x, u, u, k·dt, dt, solver, substeps)`
(`core/compile/evaluators/stepping.py:40`): `substeps` is a Python loop over static knobs, unrolled
into four `f` evaluations per RK4 substep. `k` arrives as an `int32` traced scalar from the scan's
`xs`; `t_k = k·dt` is a traced float. Traces today (the evaluator's `rollout` is this). Cost: zero
beyond the plant's `f`.

**P2. The computer's tick with selects.** `fired = (k % d == 0)` on a traced integer is a
traced boolean; `where(fired, step_i(...), x_i)` evaluates both branches (the select), so an idle
block costs its own step on every base tick. For a single-rate computer there is no select at all
(`d == 1` is static, the `where` is not emitted). The hold registers are plain state slices. The
gradient flows through the branch taken (`where`'s VJP). Prototype: `computer_step`, §10.

**P3. The hybrid rollout.** One `lax.scan` over `k = 0 … N−1` with the carry `(x_plant, x_computer)`,
the body being P2 then P1; `xs = (ks,)` so the tick index is a scan input, not a Python counter.
`vmap` over trials maps the carry's start and the batched leaves. Reverse mode stores the carry per
tick: `N × (n_plant + n_computer)` floats, `jax.checkpoint` on the tick body when `N` is large. The
orchestrated twin agrees to the last bit at the ticks when both use the fixed-step plant
(verified, §10).

**P4. The score.** After the scan, `c_k` is `vmap(transition_cost)(xs[:-1], us, ks)`: `N`
independent one-period augmented integrations, embarrassingly parallel, no carry. Reductions
(`sum`, `min`) over `k`; `where` on the finiteness flag and the price keeps the whole function
total. Cost: about one plant period per sample, in parallel. Under Euler with one substep this is
exactly `g·Δ` (verified), so DP and RL keep their numbers.

**P5. Monte Carlo.** `jit(vmap(score, in_axes=axes))(params)` where `axes` is the tree of `0` /
`None` derived from `realize`'s output: a drawn leaf batches, a shared leaf is broadcast by XLA
without a copy. Integer seed leaves batch like floats (`uint32` arithmetic in the cipher,
`distributions.py:199`). The 1000 × 1000-step pendulum batch today runs in tens of milliseconds
(DESIGN §5); the family price of a batched `params` leaf (278 ms against 27 ms, S53) is a separate
row and does not change the design.

**P6. The seeded policy and the sources.** `ε(seed, ⌊t/Δ⌋)` is Threefry on `uint32` traced
arrays (`random_bits`, `distributions.py:199`), already traced today inside `WhiteNoise.h`
(`sources.py:257`); the policy's `h` is the same call. `floor(t/Δ)` on a traced float is fine
under `jit`, `vmap` and `grad` (zero gradient through the index, as it should be).

**P7. The program with a tree.** The program evaluator jits `J(z, params)` and `grad_z J(z, params)`
once; `solve(params=p)` merges and binds `p` (a device-resident pytree), and the SciPy or
Ipopt callbacks call `J_jit(z, p)`: one dispatch, no retrace, the same cost as today's `x0`-only
binding. Inside a traced MPC tick the whole thing is inlined.

**P8. The native fixed-iteration solve.** `lax.scan(step, init(z0), None, length=K)`: reverse
mode is well defined (a fixed trip count), memory `K × n_z`. The MPC's closed loop is then a scan
(ticks) whose body contains a scan (iterations) whose body contains a scan (single shooting over
the horizon): three nested scans, all with static lengths. Reverse-mode memory is
`N × K × N_h × n` plant states without checkpointing; `jax.checkpoint` on the tick body brings it
to `N × n_z + K × N_h × n` at the price of recomputing the tick in the backward pass. For the
pendulum (`N = 80, K = 15, N_h = 20, n = 2`) it is 48 000 floats either way. Prototype: the
adjoint through single shooting matches central differences to `7e-11`, and the closed-loop
score is a smooth function of the MPC's cost weight (§10).

**P9. `jax.grad` and integer leaves.** `grad` refuses an integer input; `pe.gradient` uses
`allow_int=True` and maps the `float0` leaves to `None`; `pe.program(wrt)` packs float leaves only
(R131). `jax.grad` over a tree with no seed works as DESIGN §6 states.

**P10. What stays untraced, by design.** `compute_trajectory` on a continuous loop (`solve_ivp`),
the orchestrator's plant integration between ticks, the realtime loop, a host planner's
`solve_from`, the frequency and linear analysis (`numpy.linalg`, SciPy), collocation on Ipopt.
Each is reachable from the same objects and the same tree.

### 7.5 The one contract, as the test will hold it

For every public core math object `o` (the `System` family and the blocks, costs, sets,
distributions, fields, the sources, `Trajectory`'s verbs excepted, `PlanningProblem`,
`MathematicalProgram`, the policies, the kernels), on both backends:

1. `o.params` is a dict whose leaves are floats, numeric arrays or integer `seed`s (`{}` allowed);
2. every method CONSTITUTION §2 names returns the same value with `params=None` and with
   `params=o.params`;
3. `jax.grad` through any float leaf runs, and matches central differences;
4. two calls with one seed return the same values; a composed object's tree is the dict of its
   parts' trees by id or role, reachable by a dotted path through `core.params.lookup`;
5. the exceptions (RL training's collectors until the seeded policy lands; student-written
   models) are listed in DESIGN §4.

---

## 8. The v0.3 coverage table

| Workstream / choice | Where this design satisfies it | Revised? |
| --- | --- | --- |
| §1 Every object pure, every number a parameter; R140 | §4's rule, §6.1–§6.3 trees, §7.5 the test; `core/params.py` one module; `params = {}`; `sample(key, n, params)` | no |
| R124 three owners of a start | §6.2 `x0` views with write-through, `x0_at`; §6.4 `x_start` the problem's; §6.5 `controller.x0`, `observer.x0` leaves | no (adds `x0_at`) |
| §2 The step algebra first-class; R125, R126 | §6.1 `sample_period`; §6.2 operators on step systems, propagation within the tick, `Computer` pure with hold registers; §7.4 P2–P3 | §3.3 (Computer in core) |
| §3 The draws; R128 | §6.3 sources, §6.4 `disturbances=` blocks on both problems, `realize` one tree, per-trial seeds by path; §7.4 P5–P6 | §3.1 (one tree, not a pair) |
| §4 The seeded policy; R129 | §6.8 `GaussianPolicy(mean, log_std, *, sample_period, seed=None)`, `log_prob`, `entropy`; §5.8 | no |
| §5 The hybrid loop; R127 | §6.2 `HybridDiagram(StepDiagramSystem)`, `compute_rollout` / `compute_trajectory`; §7.3 the two runners; §5.4 | §3.2 (`%` discretizes) |
| §6 One score (open investigation) | §3.4 and §6.6: the same `PlanningProblem` on a `StepSystem`, `transition`, the integrated per-step cost, one failure rule, the hold; §5.7 | settled here |
| §7 The parametric program; R130, R131 | §6.7 `MathematicalProgram(params=)`, `solve(params=)`, `params_at`; the convention (complete trees inside, merge at the boundary, sizes structural, `z0` on the solve, no `J(z, key)`); §7.4 P7 | §3.6 (gate met by construction) |
| §8 The native proof of concept; R132 | §6.7 kernels with `init`/`step`, `iterations=K` one scan, `as_step_system`; §6.9 the MPC's `step`; §5.6; §7.4 P8 | no |
| §9 Fields; R133 | §6.3 `Field`, `QuadraticField`, `as_set` / `as_input_set` / `as_cost`, `Set.as_cost`; §5.7 | adds `terminal=` on `as_cost` |
| §10 Hardening and the merge | §7.2 the dependency law; §7.5 the contract test; §9 the order; S66 at wiring time (an algebraic loop in a step diagram refused, §6.2) | no |
| R134 names | §6.10 | no |
| R135 planner interface | §6.9 the pure trio + host `solve`; §3.7 | made concrete |
| R136 learning from data | not in the core; the program export with a data cost stays the route C4 flags (§11) | no |
| R137 no `KinematicModel` | nothing here adds a delegate | no |
| R138 basic `PlanningSolution` | §6.9 | no |
| R139 the road to the freeze | `HybridDiagram` kept, `SampledDataSystem` an option (§3.8); no shim (R82) | no |
| Definition of done (core-architecture §11) | items 1–6 are §5.3, §5.4, §5.5, §5.4, §5.6, §5.7; item 7 the merge gate | — |

---

## 9. The migration from today's code

Module by module: what changes, which numbers move (R80–R82: a move or rename is byte-identical;
a restructure agrees to `rtol 1e-6`; an intended change carries a closeness report and re-pins
its baselines in its own commit), the risks, then the order.

### 9.1 Module by module

| Module | What changes | Numbers | Risk |
| --- | --- | --- | --- |
| `core/params.py` (new) | `lookup`, `with_leaf`, `merge`, `flat_leaves`, `axes_like`, `take_trial` from their four copies | byte-identical | low; the four callers' tests |
| `core/system.py`, `core/diagram.py`, `core/wiring.py` | `params = {}` default; `x0`/`n`/`state` as views with write-through; `x0_at`; `refresh()` leaves `Simulator`; the `h` stub on `xp` | byte-identical | medium: `derived_x0` in `wiring.py` and the bubbled solver hints depend on `refresh()` (the S29 row lists them) |
| `core/sets.py`, `core/costs.py`, `core/distributions.py` | the library sets read `params`; `sample(key, n, params=None)`; `CostFunction.integrate` retired after §6.6 lands | byte-identical until §6.6 | low |
| `core/fields.py` (new) | `StateField` promoted; `QuadraticField`; `as_set` / `as_input_set` / `as_cost(terminal=)`; the shaping functions moved; `Set.as_cost` | byte-identical (RoA, DP, corridor baselines) | low; one rename sweep (`as_constraint` → `as_set`) across the track demos |
| `core/discrete/discretization.py` (moved from `analysis/`) | `sample_period`; the static `Sampled` form; `"tustin"` later | byte-identical | low |
| `core/discrete/schedule.py`, `computer.py` (from `simulation/discrete/computer.py`) | the pure `Computer(StepDiagramSystem)` with divisors and hold registers; propagation within the tick | **intended change**: the hybrid demos move by the one-tick lag (R125) | medium: the oracle is today's `Computer.tick` with the lag removed by hand |
| `core/discrete/hybrid_diagram.py`, `hybrid_composition.py` | `HybridDiagram(StepDiagramSystem)`; `compute_rollout` the scan, `compute_trajectory` the orchestrator; `last_result` and `rollout` caches gone | **intended change**: the plant's default integration between ticks becomes adaptive in `compute_trajectory` (today one RK4 step per tick) | medium: `plant_dt_inner` callers (`sampled_smc_pendulum.py` and the hybrid demos) |
| `core/assembly.py`, `core/closed_loop.py`, `core/operators.py` | `@`, `>>`, `+` on step operands; mixing refused; `%` discretizing | byte-identical for continuous operands | low |
| `core/compile/evaluators/discrete.py` | `rollout` returns native arrays and a `Trajectory` with `t = k·Δ`; `rollout_batch`; signals recorded | byte-identical (`StepRollout` → `Trajectory` is a fold) | low |
| `core/compile/evaluators/stepping.py` | `transition_body`: the augmented `[x; c]` step beside `step_body` | — | low |
| `simulation/discrete/hybrid_simulator.py` | a runner over the compiled computer step; adaptive plant integration restarted at each tick | with the row above | medium |
| `simulation/realtime/` | unchanged in v0.3 (its re-base goes with its architectural review, v1.0) | — | — |
| `blocks/sources.py`, `control/policies.py` (new) | `TrajectorySource.values` a leaf; `GaussianPolicy` | byte-identical; the RL numbers move once when the heads rebuild on it | medium (the RL re-pin) |
| `planning/problems.py` | `disturbances=` blocks on both problems, the keywords `x0=` / `params=`, `realize` one tree, `discretize`, `transition`, `stage_cost`, `n_steps`, `cost_bound(params=)` | **intended change**: live disturbances (today held at nominal); cross-backend draws | medium: the Monte Carlo demos and RN-5's LQG gate re-pin |
| `planning/evaluation.py` | `score_run` reads `transition`; one failure rule | **intended change**: the per-step cost (trapezoid → the integrated `c_k`), ≈ 1e-3 relative on RK4 at `dt = 0.01` | low: a closeness report |
| `planning/evaluator.py` | every controller kind through one scan (`require_continuous` gone); `objective`, `gradient`, `merge`; the price in-trace for a quadratic cost | byte-identical for a continuous law (the loop is discretized as a whole, as today) | medium: the `auto` backend's `traces()` probe grows a computer case |
| `planning/dynamic_programming/dp.py`, `learning/.../environment.py` | `Q = c_k + γ J` and `r_k = −c_k` through `transition` | byte-identical under Euler with one substep (the defaults); RK4 grids move | low |
| `planning/trajectory_optimization/*` | `MathematicalProgram(params=)` from every transcription (collocation reads `p["plant"]`, `p["cost"]`, `p["X"]`…); `ParametricMathematicalProgram` folds in; single shooting with the native kernel and `iterations=K`; the pure trio; `program` on the planner | byte-identical with only `x0` bound; shooting's reported score lowered to §6.6 (an intended change) | medium: the collocation closures (`direct_collocation.py`) are rewritten on the tree |
| `planning/trajectory_optimization/mpc/controller.py` | one `StepSystem` with the solve in `step`, the latch gone, `sample_period=`, `parameter_ports=`; `export_to_computer` replaced by `%` | **intended change**: the one-period delay (R123) | medium: the MPC baselines re-pin |
| `optimization/mathematical_program.py`, `optimizer.py`, `kernels.py` (new), `evaluators/` | `params=` on the noun; `solve(params=)`; `backend=`, `adapter`; the kernels; `iterations=`; one program-evaluator body | byte-identical (`gd` and `adam` as kernels reproduce the loop step for step) | low |
| `learning/reinforcement_learning/` | the heads on `GaussianPolicy`; the collectors on per-episode seeds | **intended change**: the RL numbers, once | medium: timing of the re-pin (with the block, or with C5 in v0.4: the row's open detail) |
| docs | DESIGN §4–§6, CONSTITUTION §2 and invariant 1 amendments (R127, R128, R140), ROADMAP §3, the plan docs' status lines | — | — |

### 9.2 The order of work (three lanes, two joins, as core-architecture §10, with this design's edges)

1. **Lane A, objects.** `core/params.py` → `params = {}` and the sets / distributions reading
   `params` → `x0` views and write-through → `Field` minimal → `TrajectorySource.values`. Each
   byte-identical. The contract test (§7.5) lands at the end of the lane and then gates every
   later step.
2. **Lane B, the loop (the critical path, starts first).** `sample_period` and the discrete
   evaluators tracing end to end with a `Trajectory` result → `discretize` to `core/` → the pure
   `Computer` with hold registers (the R125 change, with the closeness report) → `HybridDiagram`
   as a step diagram with its two verbs (the adaptive default of `compute_trajectory`, with the
   report) → the operators on step systems and `%` discretizing → the evaluator's sampled face
   (`require_continuous` gone). Here the digital PID of §5.4 runs, `jax.grad` and all.
3. **Lane C, the program.** `MathematicalProgram(params=)` with one evaluator body and the
   backend table → collocation on the tree, the planner's `program` and `solve(params=)` → the
   kernels, `iterations=K`, `as_step_system` → single shooting on the kernel.
4. **Join 1, one score.** `transition` and `stage_cost` on the problem; `score_run` on it (the
   closeness report); DP and RL on it (byte-identical under their Euler defaults); shooting's
   score lowered; `compare` showing one column of evaluated `J`s.
5. **Lane A continued, the draws.** `disturbances=` blocks and `realize` one tree with per-trial
   seeds by path (the cross-backend draws and live disturbances, with their report); `GaussianPolicy`;
   the RL heads on it (or deferred to v0.4 with C5).
6. **Join 2, the MPC.** The MPC block as one `StepSystem` with the solve in `step` (R123's
   report); the pure trio on the trajopt planner; the fixed-iteration tick; its closed loop in
   one scan through the sampled face, `vmap`ped and tuned (§5.6).
7. **Last.** S66 at wiring time, the dependency-law check as a test, the notebooks, DESIGN, the
   CONSTITUTION amendments, the merge.

Each intended number change is one commit with its ledger line, in this order: the same-tick
semantic (lane B), the hybrid default (lane B), the one-score `J` (join 1), the live
disturbances and cross-backend draws (lane A), the MPC delay (join 2), the RL re-pin (last).

---

## 10. What I verified

### 10.1 Prototypes run (NumPy; SciPy and JAX are not installed in this container)

The scratch folder is
`/tmp/claude-0/-home-user-minilink/36350854-ab7e-51d8-8262-d68717b27e02/scratchpad/fable-arch-B/`:
`proto_kernels.py` (the kernels, written in scan shape) and `run_protos.py` (the checks). Run with
`PYTHONPATH=/home/user/minilink python3 run_protos.py` (21 s). Every function is a pure function
of a pytree carry and a tree of parameters, with `where` in place of `if` on anything a trace
would see; `scan` and `vmap` are Python stand-ins with `lax.scan`'s and `jax.vmap`'s signatures.
The output:

```text
1. digital PD % 0.01 @ Pendulum, N=300: x_N = [0.186775 0.313886]  J = 0.536737  margin = 1.75  failed = False
   orchestrator twin: max|u - u_orc| = 0.0  max|x_N - x_N,orc| = 0.0
2. Euler 1 substep: c_k - g(x_k,u_k,t_k)·dt = 0.0
   RK4 integrated running cost = 0.456757  trapezoid on the samples = 0.456309  (relative gap 9.82e-04)
3. multi-rate hold register (slow block d=5 reads the ramp u=k, held): [0.0, 0.0, 0.0, 0.0, 0.0, 10.0, 10.0, 10.0, 10.0, 10.0, 20.0, 20.0]
   computer state dim with one d=5 block of p=1: 2 (1 state + 1 hold register)
4. single-shooting gradient, adjoint scan vs central differences: max abs gap = 7.00e-11
   fixed-K MPC (K=15 gd steps/tick, N_h=20, 80 ticks of 0.05 s): x_end = [0.020719 0.027376]  J = 0.93336  |u|max = 0.179  failed = False
   dJ_closed/dQ11 (central differences through the closed loop) = -0.01168   (the slope jax.grad would return through the nested scans)
5. GaussianPolicy: same seed twice equal: True  other seed differs: True  same index held: True  log_prob(u_a) = -1.1992
6. realize(key=1, n_trials=4): plant.m = [1.019253 0.990842 0.870984 1.063019]  d.seed = [ 927393791  481380958 1306373812  298569811]  axes plant.m/plant.l/d.seed = 0 None 0
   trial 0: m = 1.0192534375624764  l (shared) = 1.0  x_start = [ 0.114354 -0.093723]
   adding a random block leaves the other streams unchanged: True
7. student program J(z) = 2.0 ; parametric J(z0) = 0.4885 ; rebound x0 = 1.9678
   unknown key refused: "unknown parameter 'plnt'; known: ['cost', 'plant', 'x0']"
```

What each line shows:

1. **The hybrid loop as one scan** (`hybrid_rollout`): a digital PD (backward-Euler filter
   state) at `dt = 0.01` on `Pendulum().f` through `rk4` from `stepping.py`, the computer's tick
   reading `y_k` and the plant holding `u_k`; scored by the two-pass rule. The stateful
   orchestrator written separately (`Orchestrator.tick`) gives the same `u_k` and `x_N` to the
   last bit: the step diagram is the orchestrator's oracle (R127).
2. **The per-step cost rule** (`transition`): under Euler with one substep `c_k` equals `g·dt`
   exactly, so DP and RL keep their numbers; under RK4 the integrated cost and the trapezoid differ
   by `1e-3` relative at `dt = 0.01`, the closeness the one-score commit will report.
3. **The multi-rate computer** (`computer_step`): a slow static block with `d = 5` reading a ramp,
   its hold register in the state, fires at `k = 0, 5, 10` and holds between; the state has one
   extra entry per held output and none for a `d = 1` block.
4. **The fixed-iteration MPC** (`gd_kernel`, `solve_fixed`, `mpc_loop`): single shooting under
   Euler with its gradient by a reverse scan (the adjoint, the shape `jax.grad` produces through
   `lax.scan`), checked against central differences to `7e-11`; 15 projected-gradient steps per
   tick on the shifted plan with `x0 = y`, the command read from the state (the one-period
   delay), the closed loop as a scan of ticks each holding a scan of iterations; the pendulum
   settles. The closed-loop `J` is a smooth function of the MPC's own `Q₁₁` (its central-difference
   slope is what `jax.grad` would return through the nested scans), which is §5.6's tuning.
5. **The seeded policy** (`gaussian_policy_h` on `standard_normal` from `core/distributions.py`):
   deterministic given the seed, distinct across seeds, held within a sample index; `log_prob`.
6. **`realize` as one tree** with per-leaf trial axes (`0` on the drawn leaves, `None` on the
   shared ones); `child_seed` by path; adding a random block leaves every other stream unchanged
   (the spawned streams of randomness.md §5).
7. **The program with `params=None`**: the student's `J(z)` and the parametric `J(z, params)`
   on one class; the rebind of `x0` without rebuilding; an unknown key refused with the known ones.

The run is deterministic: a second run reproduces the output above exactly.

### 10.2 Facts checked in the code

- `StepSystem.step` / `h` take `(x, u, k, params)` (`core/discrete/step_system.py:62,87`);
  `StepDiagramSystem.step` already propagates within the tick (`step_diagram.py:54–90`), it is
  `Computer.tick`'s double buffer that lags (`simulation/discrete/computer.py:130–163`).
- `Discretization.step` binds `t_k = k·dt` and calls `step_body` (`analysis/discretization.py:130–146`);
  `step_body` unrolls `substeps` in a Python loop (`stepping.py:40–51`); `iterate` is a
  `lax.scan` on JAX (`stepping.py:62–85`).
- The discrete evaluator's `rollout` returns a NumPy `StepRollout` (`evaluators/discrete.py:39–60`)
  while its `scan_steps` is `@compiled`; the fold to a native `Trajectory` is a wrapper change.
- `ProblemEvaluator.realize` returns a `Batched` tree and `x_starts` apart (`planning/evaluator.py:324–352`);
  `run_trials_jax` vmaps with `in_axes=(axes, 0, None, None)` (`evaluator.py:462–487`); the
  one-tree form drops the second axis.
- `score_run` integrates by `CostFunction.integrate`'s trapezoid (`evaluation.py:199–205`,
  `costs.py:95–123`); DP backs up `g·dt + α J` (`dp.py:270,309`); the environment rewards
  `−g·dt` (`environment.py:162`).
- `WhiteNoise.h` draws by `standard_normal(seed, k, p)` with `k = sample_index(t, period)`
  (`sources.py:257–288`); `random_bits` runs Threefry on `uint32` arrays under JAX
  (`distributions.py:199–227`); `child_seed` hashes the stream name with CRC-32 (`distributions.py:275`).
- `MathematicalProgram` is a frozen dataclass with `J(z)` only (`mathematical_program.py:26–96`);
  `Optimizer.__init__` compiles the program evaluator at `z0` and names the adapter `backend`
  (`optimizer.py:118–149`); the first-order solver is a NumPy loop with Adam's update jitted
  (`optimizers/first_order.py:56–132`).
- The MPC block probes its planner (`controller.py:905`) and solves through a latch
  (`controller.py:567–705`); `export_to_computer` refuses a `dt` other than `dt_mpc` (DESIGN §4).
- `PID(Kp, Ki, Kd, tau, …)` is a `DynamicSystem` with two states (`control/siso.py:80,125`); a
  closed loop `PID @ Pendulum` has blocks `controller, plant, demux, error` and `n = 4` (run in
  this container).
- `lqg` returns a box with `observer` and `gain` and `r, y → u` on its edge (`control/design.py:135–183`);
  `StateObserver` holds `A, B, C, L` and the operating point (`estimation/observer.py:33`).

### 10.3 What is untested on JAX, and why I believe it traces

- **The hold-register select under `vmap` and `grad`.** `where` on a traced boolean is the
  standard select; its VJP routes the cotangent to the taken branch. The same construct is in
  `score_run` today (`xp.where(finite, …)`, `evaluation.py:179–180`) and traces.
- **`k % d` on a traced `int32`.** `lax.rem` on integers is supported; the step evaluator already
  passes `k` from `self.indices(...)` as scan input (`discrete.py:63–82`).
- **Three nested static-length scans** (the MPC closed loop). `lax.scan` nests; reverse mode of a
  scan is a scan; `jax.checkpoint` on the tick body is the standard remedy for memory. The RL
  learners already run their update steps in one `lax.scan` (DESIGN §6).
- **`vmap` over a tree with `None` axes and integer leaves.** `in_axes` as a pytree prefix with
  `None` is how `run_trials_jax` works today; integer leaves batch as arrays.
- **`jax.grad(..., allow_int=True)`** returns `float0` tangents for integer leaves; mapping them
  to `None` is a `tree_map`. This is JAX's documented behaviour; I could not run it here.
- **The augmented step for `c_k`.** It is `step_body` on a concatenated state with one more `f`
  component; nothing new is traced.

---

## 11. Open questions I could not settle

1. **The default scheme of `block % dt` for a controller.** `discretize`'s `rk4` is honest but
   not the textbook digital PID (backward Euler or Tustin). Recommendation: keep one default
   (`rk4`) for every `discretize`, and add `solver="tustin"` to the linear `ExactDiscretization`
   so GRO501 writes `discretize(pid, dt, solver="tustin") % dt`; the lesson then says which it
   uses. The alternative (a controller-specific default) is a second rule for one keyword.
2. **The margins between samples.** `transition` can return the least margin over its substeps as
   a third output, at the cost of `substeps` set evaluations per period. Recommendation: not in
   v0.3; the ticks are the contract (as today), and a `margins="substeps"` knob on
   `problem.discretize` is additive.
3. **The failure price under a cost override.** The exact bound of a quadratic cost is cheap
   in-trace, so the price follows the overridden weights; a sampled bound (`box_samples`, 4096
   calls) is computed once at compile at the nominal weights. Recommendation: that split, written
   in DESIGN; a cache keyed on the cost's parameters is not worth its code.
4. **Where the RL heads rebuild.** With `GaussianPolicy` (the core) or with the RL follow-ups
   (v0.4). Recommendation: the block and its `log_prob` / `entropy` in the core with one algorithm
   (PPO) moved onto it as the proof; the rest with C5, so the RL numbers move once per family.
5. **The multi-rate computer's idle work.** The base-tick select computes idle blocks; an MPC at
   20 Hz beside a 1 kHz inner loop pays 50 solves per solve. Recommendation: accept in v0.3 (the
   common case is single-rate), and plan the hyperperiod unrolling as the one optimization of
   v0.4, with the select form as its oracle.
6. **`compute_trajectory` on a hybrid loop: which adaptive solver and tolerances.** R127 leaves
   it to the implementation. Recommendation: `solve_ivp` RK45 restarted at each tick with the
   simulator's default tolerances, `solver="rk4"` with `substeps` the fixed-step choice; Diffrax
   only if a benchmark asks (R139a).
7. **The name of the sampled loop.** `HybridDiagram` or `SampledDataSystem` at the freeze.
   Recommendation: `SampledDataSystem` when the names are drawn (R139c), since the object is a
   `System` now; not before.
8. **`x0_at`.** A small helper that the write-through `x0` makes convenient (set one block's
   start). Recommendation: add it with the views; drop it if the lessons do not use it.
