# The final core architecture of minilink: Proposal D (2026-10-27)

## 1. Header

**Audience:** Prof. Alexandre Girard (Maintainer)
**Status:** an independent proposal, not a ruling. One of several written in parallel for the
maintainer to consolidate; it read the repository, the rulings and the code, and nothing of the
other proposals.
**Commit reviewed:** `f634e47eb04db70d0089b46c4e043f334eb8bdc7` (`dev-v0.3`).
**Prototypes:** six, run on NumPy and on JAX (`jit`, `grad`, `vmap`, `lax.scan`), in
`/tmp/claude-0/-home-user-minilink/36350854-ab7e-51d8-8262-d68717b27e02/scratchpad/arch-D/`
(§10 reports what ran and what it printed).

The document keeps the order asked for, so the proposals compare side by side: the aim (§2),
the adjustments to the vision (§3), the user's view on one page (§4), the pipelines as complete
scripts (§5), the API object by object (§6), the architecture under it (§7), the coverage of
every v0.3 workstream and preliminary choice (§8), the migration (§9), what was verified (§10),
and what stays open (§11).

---

## 2. The aim in one screen

The repository says one thing in many places, and the code is already most of the way there:

**One model, one path, one score.** A student writes `ẋ = f(x, u, t; p)` once, as a `System`,
and never changes framework again: the same object is simulated, linearized, closed in a loop,
sampled by a computer, planned on, learned on and tuned. The spine (CONSTITUTION §1.1, R119) is
one differentiable path from the model and its parameters to a number:

```text
System ─ compile ─▶ rollout ─▶ Trajectory ─▶ score ─▶ Evaluation ─▶ program ─▶ Optimizer
```

Every control approach is a tool on that path. The parameters of every object on it form one
tree, every leaf a number, so `jax.grad`, `jax.vmap` and a family of plants reach anything the
way they reach `f` (R140). The maintainer's direction of 2026-10-26 names what v0.3 must add
to the landed continuous path so the core is complete:

1. **stochastic problems** as draws realized from one key (starts, parameters, the seeds of
   the noise blocks and of the policy), run batched;
2. **the sampled loop** (a computer at its clocks around a continuous plant) with the same
   rights as a continuous loop: compose, analyse, evaluate, differentiate, batch;
3. **one score** for a policy, whatever tool produced it and whatever loop runs it;
4. **one parametric program**, compiled once and rebound per solve (the MPC's tick and every
   other repeated solve);
5. **a native solver as a pure kernel**, so a fixed-iteration MPC's closed loop compiles and
   differentiates end to end;
6. **a `Field`**, the last textbook noun (a value function, a Lyapunov `V`, a barrier);
7. **the seeded stochastic policy** as a plain block.

Under all of it, two invariants: every object pure and every number a parameter; the step
algebra first-class.

**What the maintainer asks of the design** (his words, 2026-10-27): the user's point of view
first, the main objects, their verbs and contracts, the names, and how each
design-analysis-simulation pipeline runs; and a design that works, which here means: the main
math path traces under JAX and is computationally efficient. §10 holds the evidence.

**The reading that drives this proposal.** Once `dt` and the scheme are chosen, a continuous
closed loop *is* a discrete system. That sentence (the maintainer, 2026-10-26) is not one pillar
among seven; it is the architecture. Every pillar lands on one runtime: **a step-and-stage map
`Φ: (x_k, u_k, k; p) ↦ (x_{k+1}, c_k)` run by one scan.** A continuous plant enters it by the
step model (`discretize`); a native `StepSystem` enters it as is; a computer around a plant
enters it as a step diagram at its base clock; an MPC enters it as a step block whose tick is
`K` iterations of a native kernel; a learner's environment is one tick of it; the Monte Carlo is
a `vmap` of it; the tuning program is its gradient. The sections below make that one sentence a
design that runs, and the prototypes show it runs.

---

## 3. Adjustments to the vision

Each with its reason, its cost and the preliminary choice it revises. The rest of R124–R140 is
taken as is (§8).

### D1. The one score is the stage cost integrated with the state, in one scan (answers the open investigation)

- **The rule.** The problem stays continuous. A tool that steps lowers the pair (plant, cost)
  once, with `(dt, solver, substeps)`, into the step-and-stage map
  `Φ(x_k, u_k, k; p) = (x_{k+1}, c_k)`, where the stage cost
  `c_k = ∫_{t_k}^{t_{k+1}} e^{−ρ(t − t_k)} g(x, u_k, t) dt` is integrated **with the state**:
  the augmented state `[x; c]` with `ċ = e^{−ρ(t − t_k)} g` takes the same RK4 or Euler steps
  and the same substeps as `x`. Then `J = Σ_{k<N} γ^k c_k + γ^N h(x_N, t_N)` with
  `γ = e^{−ρ dt}`, the sets read at the samples, `X.margin(x_k, t_k)` and
  `U.margin(u_k, x_k, t_k)`, and the failure rule is `score_run`'s: the run goes to the horizon,
  `failed = least margin < 0`, the price switch unchanged. A native `StepSystem` plant has
  `c_k = g(x_k, u_k, t_k) Δ` (`Δ` its `sample_period`, 1 for a tick system), the same formula
  Euler gives.
- **The answers to the maintainer's three questions.** Sets and costs need no discrete
  versions: they bind `t_k = t₀ + k Δ`, and a natively discrete cost `c(x_k, u_k, k)` is a
  `CostFunction` on a tickless step system, where `t` *is* `k`. A continuous evaluation is done
  exactly as a `StepSystem` on its `dt`: the trapezoid becomes the plain sum `Σ γ^k c_k`, exact
  for the scheme. No new noun: the lowering is internal to the tools (reviewer C), and a
  `PlanningProblem` whose `sys` is a `StepSystem` is already the discrete problem (reviewers A
  and B).
- **Reason.** One `J` for value iteration, the learners, the evaluator, shooting and the
  sampled loop, from one function in one module (`stepping.stage_body`). Under forward Euler the
  stage integral *is* `g(x_k, u_k, t_k) dt`, so value iteration's and the RL reward's numbers do
  not move (prototype 1: equal to 4·10⁻¹⁵). Under RK4 it is the quadrature the state itself
  gets: on the pendulum test it lands 2.6× closer to the fine-grid reference than the trapezoid
  on the same samples (§10, part 1).
- **Cost.** The evaluator's `J` and shooting's move once: 0.04 % under RK4 and 1 % under Euler
  on the test loop (§10, part 6); a closeness report and one re-pin (R80). The RL environment's
  early termination becomes a training choice that still *reports* the one score (the three
  reviewers' view).
- **Revises:** nothing ruled; it fills the `Answer:` line of QUESTIONS' one-score entry.

### D2. `%` samples a block into a `StepSystem`; `computer @ plant` is a `SampledDataSystem`

- **The rule.** `ctl % Δ` returns a step block of the step algebra: the block *sampled* at the
  period `Δ`. A static law (a gain, a neural policy, a lookup table) becomes a static block with
  `sample_period = Δ`; a law with continuous states (a PID with its filter and integrator) becomes
  its discretization at `Δ` (`discretize(ctl, Δ)`, the textbook digital implementation GRO501
  teaches); a block already a `StepSystem` (an MPC, a digital filter) is tagged with `Δ`. Then
  `(ctl % Δ) @ plant` is the one place a step operand meets a continuous one: the plant is
  discretized at the loop's base period with its substeps, and the result is a
  **`SampledDataSystem`**, a `StepDiagramSystem` that keeps the continuous plant as its
  `source`. Multi-rate is a property of the step diagram: each block carries its
  `sample_period`, the diagram's base period is their greatest common divisor, a block with
  divisor `d > 1` fires on `k mod d = 0` as a select, and its held outputs are state of the
  diagram (the zero-order hold is memory, R104 applied to the hold). No `StepSchedule` object,
  no `Computer` class: the schedule is the blocks' periods, and the runtime is a runner (§7).
- **Reason.** One noun for the sampled-data loop, the textbook's (Åström and Wittenmark); the
  whole step algebra applies (`@`, `>>`, `+`, `rollout`, `jacobian`, the sampled-data poles,
  `problem.compile`, `vmap`, `grad`); and the pure base-tick form is byte-identical to a clock
  orchestrator on NumPy (prototype 2), so the orchestrator is a runner, not a second model.
- **Cost.** `Computer`, `StepSchedule`, `HybridDiagram`, `HybridSimulator` and
  `hybrid_closed_loop` leave the public surface (provisional names, no stability promise,
  ROADMAP §2); the hybrid demos and tutorial 06 are rewritten; the hybrid regression gates
  re-baseline once with the same-tick semantic (R125) and the MPC delay (R123), as planned.
- **Revises:** R127's name (`HybridDiagram` "for now" → `SampledDataSystem`, the reviewers'
  choice, taken now so the notebooks migrate once) and R126's wording (the mixed operand is
  refused *unless* the step side carries a `sample_period`, which is exactly `computer @ plant`).

### D3. One verb on the sampled loop, two solvers, like the continuous side

- **The rule.** `loop.compute_trajectory()` on a `SampledDataSystem` runs the pure scan at the
  base clock and records the plant's substeps, so the plot is as dense as today's; `solver="scipy"`
  swaps the plant's integration between ticks for the adaptive solver, restarted when the plant's
  input changes, the same pure computer tick around it. `compute_rollout()` stays the plain step
  verb of every `StepSystem` (ticks, no seconds). `evaluate` scores the default runner, so
  `compute_trajectory` and `J` agree by default and `solver="scipy"` is the cross-check, R15's
  split applied literally.
- **Reason.** The student meets one verb on every loop; the numbers of the plot, the score and
  the gradient are one set.
- **Cost.** None beyond D2's.
- **Revises:** R127's "two verbs, two runners" to "one verb, two solvers".

### D4. The MPC block and its planner interface, in four verbs

- **The rule.** `ModelPredictiveController(planner, *, sample_period, iterations=None)` is a
  `StepSystem` whose state is the last plan `z` and whose `step` solves from the shifted plan
  with the measurement bound as `x_start`; its `u` port reads the state (R123's one-period delay).
  The planner does not know it runs in a loop (R135). Its interface is four verbs:
  `planner.program` (a `MathematicalProgram` with `params`, built once),
  `planner.solve(params=None, z0=None) -> PlanningSolution`, `planner.shift(z) -> z` and
  `planner.plan(z, params=None) -> Trajectory`. With `iterations=K` the block runs the native
  kernel `K` times and traces; with `iterations=None` it runs the host solver to tolerance and
  stays on the NumPy face.
- **Reason.** The four verbs are everything the loop mechanics need, and they are the same for
  collocation, shooting, MPPI (v0.4) and the native kernel.
- **Cost.** The MPC's constructor changes (`dt_mpc` → `sample_period`, already R134; `warm_start`
  goes: the state is always the plan, a cold start is `shift` of the default guess);
  `MPCStatelessController`, `MPCTickLatch`, `Command`, the dual-rate computer and the debug
  figure leave with the latch.
- **Revises:** R135 (fleshed out, not contradicted).

### D5. The program's tree is the problem's tree, word for word

- **The rule.** A transcription's `MathematicalProgram.params` *is* `problem.params`: the keys
  `plant`, `cost`, `X`, `U`, `X0`, `Xf`, `x_start`, `discount_rate`. The MPC rebinds
  `{"x_start": y}`; a tuning program over the closed loop reads `controller.cost.Q`. One word
  for the start everywhere: `x_start`.
- **Reason.** R140's third point (a composed object's tree is its parts' dicts); the student
  learns one tree.
- **Cost.** None. **Revises:** R130's example key `x0` (a wording).

### D6. `GridField` joins the core's minimal `Field`

- **The rule.** `Field`, `QuadraticField` and `GridField` (a table on a `StateSpaceGrid`, its
  `value` by interpolation, a leading time axis optional) land in `core/fields.py`, with the
  shaping functions and the three conversions; the planners hand their cost-to-go out as a
  `Field`. `CallableField`, the approximator and the certificate's rebuild stay later.
- **Reason.** The value-iteration-then-RL pipeline (§5.7) needs the DP table as a terminal cost
  on the RL problem, and that is the one-score pillar's own demonstration.
- **Cost.** One more class in the core (about sixty lines). **Revises:** R133's scope.

### D7. The stochastic problem's start is `x_start=Distribution`

- **The rule.** `x_start` takes a point (deterministic), a `Set` (a set of starts, trajectory
  optimization's `X0`) or a `Distribution` (a stochastic problem): R124's three kinds under one
  keyword. `x0_distribution` goes; `params_distribution` stays; `disturbances` takes source
  blocks only (R128). `problem.params["x_start"]` is the point, or the law's mean.
- **Reason.** One keyword per concept (R134) and R124's own wording.
- **Cost.** A rename across the RL demos and notebooks (one sweep, no alias). **Revises:**
  nothing ruled.

### D8. `StepRollout` folds into `Trajectory`, and a step system knows its period

- **The rule.** `StepSystem(n, *, sample_period=None)`: seconds per tick, `None` for a pure tick
  system; `Discretization` sets it; a rollout returns a `Trajectory` with `t = k Δ` (or `t = k`).
- **Reason.** One record for every run; `t_k` is what the costs and sets bind (D1).
- **Cost.** `StepRollout` and `HybridSimResult` go; the hybrid plots read `Trajectory`.

### D9. The native kernel and its two runs

- **The rule.** A native method is `init(z0, params) -> state` and
  `step(state, grad, k, params) -> state` (R132). `Optimizer.solve(z0=, params=)` runs it to a
  tolerance on the host; `Optimizer.iterate(z0, params=, n_steps=K) -> (z_K, Trajectory)` runs
  exactly `K` steps in one scan and traces in every mode; `as_step_system(optimizer)` is the
  lesson's view (the loop `z⁺ = z − η ∇J` as a `StepDiagramSystem`).
- **Reason.** Prototype 3: the fixed-`K` scan jits, vmaps and differentiates, and the MPC tick
  on it is the shape the maintainer asked for.
- **Cost.** `FirstOrderOptimizer`'s Python loop becomes a call of the kernel.

### D10. What this proposal does not change

The continuous path as landed (DESIGN §4–§6): `System`, `DiagramSystem`, the operators, the
evaluators, the step model, `Trajectory`, sets, costs, distributions, `PlanningProblem`,
`problem.compile`, `evaluate`, `compare`, the program export, `Optimizer`'s method table, the
planners' `PlanningSolution`. It is the spine, and the proposal is the spine made complete.

---

## 4. The user's view: the main objects and verbs

| Object | In the textbook | Verbs | `params` |
| --- | --- | --- | --- |
| `System` / `DynamicSystem` | `ẋ = f(x, u, t; p)`, `y = h(x, u, t; p)` | `f`, `h`, `tf`; `compute_trajectory`, `linearize`, `plot_*`, `animate`; `+`, `>>`, `@`, `%`; `compile`, `realize(key)` | the model's numbers; a stateful block's `x0` is its own field, a leaf of a loop's tree |
| `DiagramSystem` | a block diagram, closed under wiring | the same; `connect`, `expose`, `roles` | `{block_id: block.params}`, a live view |
| `StepSystem` / `StepDiagramSystem` | `x_{k+1} = F(x_k, u_k, k; p)`, a period `Δ` | `step`, `h`; `compute_rollout`, `jacobian`, `poles`; the same operators | as above |
| `ctl % Δ` (a `StepSystem`) | the law run by a computer every `Δ` | a step block | the law's |
| `SampledDataSystem` | the sampled-data loop: computer + ZOH + plant + sampler | `compute_trajectory(solver=)`, `compute_rollout`, the step tools, `plot_diagram`, `animate` | `{controller, plant, …}`, the holds as state |
| `Trajectory` | `(t, x, u)` sampled | `of(role)`, `with_cost`, `plot`, `resample` | — (a record) |
| `Set` / `InputSet` | `z ∈ Z(t)`, `margin ≥ 0` | `margin`, `contains`, `sample(key)`, `bounding_box`, `&`, `as_cost` | the set's numbers (`{}` when none) |
| `Distribution` | a law over a vector | `sample(key, n, params)`, `mean` | `mean`, `std` / `cov`, … |
| `CostFunction` | `g(x, u, t)`, `h(x, t)` | `g`, `h`, `+`, `*` | `Q`, `R`, weights, targets |
| `Field` | `V(x, u, t)` | `value`, `gradient`, `as_set`, `as_input_set`, `as_cost` | `P`, a table, … |
| a source (`Step`, `WhiteNoise`, `TrajectorySource`) | an exogenous signal | `h(t)` | amplitude, `psd`, `sample_period`, `seed`, the table |
| `PlanningProblem` / `StochasticPlanningProblem` | the task: plant, sets, cost, horizon; the laws over starts and parameters, the noise sources on ports | `compile(policy, dt)`, `realize(key, n_trials)`, `replace`, `nominal`, `cost_bound` | `{plant, cost, X, U, X0, Xf, x_start, discount_rate}`, a live view |
| `ProblemEvaluator` (`problem.compile(...)`) | the closed loop run and scored | `trajectory`, `evaluate`, `program`, `pack` / `unpack` | the run's tree: every block, the problem's leaves, the free inputs |
| `Evaluation` | the scores of the runs | `objective`, `mean`, `worst`, `cvar`, the rates, `print` | — |
| `MathematicalProgram` | `min J(z; p)` s.t. `h = 0`, `g ≥ 0`, bounds | `J`, `h`, `g`, `params_at(z)` | the problem's tree (an exported program), `{}` (a hand-written one) |
| `Optimizer` | the solver | `solve(z0, params)`, `iterate(z0, params, n_steps)`, `as_step_system` | the method's `eta`, `b1`, … |
| `GaussianPolicy`, `NeuralPolicy`, `LookupPolicy`, `StateFeedback`, `PID`, … | `u = π(x)`, `u = μ(x) + σ ε` | `h` / `ctl`; `log_prob`, `entropy`; `plot_control_law` | gains, weights, `log_std`, `seed` |
| a planner (`TrajectoryOptimizationPlanner`, `DynamicProgrammingPlanner`, `ReinforcementLearningPlanner`, `RRTPlanner`, `LQRPlanner`) | the method | `solve(...)`, and for the MPC `program`, `solve(params, z0)`, `shift`, `plan` | the method's knobs stay constructor arguments |
| `PlanningSolution` | the policy and the evidence | `policy`, `trajectory`, `evaluation`, `cost_to_go` (a `Field`), the generic plots | — |
| `ModelPredictiveController` | the receding horizon law, a step block | `step`; `% Δ @ plant` | `{cost, plant, …}` of its internal problem, the solver's |
| `compare(problem, A=…, B=…)` | the side-by-side | `print`, the overlays, `difference` | — |

---

## 5. The pipelines, as user scripts

Every script imports through the teaching surface, runs on NumPy unless it says `backend="jax"`,
and reports through the objects' own `print` and plots (RULES 6.1). "What the user sees" names
the `params` tree the script touches. The eight scripts are one story on one plant, a pendulum,
from the first lesson to the frontier. The printed numbers of §5.6 and §5.8 are the prototypes' (§10, parts 3 and 5), those of §5.1 today's; the others are illustrative, written to show the shape of what prints, not measured.

### 5.1 Modelling and simulation (the first lesson)

```python
"""A pendulum: the equations, a simulation, a closed loop."""
import numpy as np
from minilink import DynamicSystem, Pendulum, PID, Step, array_module

# The catalog plant: dx = f(x, u, t; p), its parameters a dict of numbers
plant = Pendulum()
plant.params["m"] = 1.5
plant.x0 = np.array([0.5, 0.0])
traj = plant.compute_trajectory(tf=5.0)         # adaptive SciPy on a smooth plant, 10 001 points
plant.plot_trajectory()                          # θ(t), θ̇(t), u(t)
plant.plot_phase_plane()
plant.animate()

# The student's own model, hard-coded constants and all (RULES 2.3's tolerance)
class MyPendulum(DynamicSystem):
    def __init__(self):
        super().__init__(n=2, input_dim=1, output_dim=2)
        self.params = {"m": 1.0, "l": 1.0, "g": 9.81}

    def f(self, x, u, t=0.0, params=None):
        params = self.params if params is None else params
        xp = array_module(x)
        m, l, g = params["m"], params["l"], params["g"]
        theta, omega = x[0], x[1]
        # (m l²) ω̇ = τ − m g l sin θ
        dx = xp.array([omega, (u[0] - m * g * l * xp.sin(theta)) / (m * l**2)])
        return dx

# A closed loop is a diagram; the loop's parameters are the blocks' by role
loop = PID(Kp=20.0, Ki=5.0, Kd=4.0) @ MyPendulum()
loop.params["controller"]["Kp"] = 30.0
traj = loop.compute_trajectory(tf=5.0)           # the reference r reads its nominal value, 0
loop.plot_trajectory()
loop.plot_diagram()
```

What prints: nothing (the framed simulation panel at `verbose=True`). What plots: the time
signals, the phase plane, the animation, the diagram. The tree the student sees:
`loop.params == {"controller": {"Kp", "Ki", "Kd", "tau", "u_min", "u_max", …}, "plant": {"m", "l", "g"}}`.
Nothing here changes from today.

### 5.2 Classical analysis and the digital implementation

```python
"""Design a PID to specification, then implement it on a computer at 100 Hz."""
import numpy as np
from minilink import Pendulum, PID, Step, bode, margins, step_info, linearize, closed_loop

plant = Pendulum()
G = linearize(plant, x_bar=np.array([np.pi, 0.0]))     # the linear model at the upright point
G.plot_bode()
G.plot_pzmap()

C = PID(Kp=40.0, Ki=10.0, Kd=8.0)
L = C >> G                                              # the loop gain
print(margins(L))                                       # gain and phase margins, crossovers
T = closed_loop(C, plant, reference=Step(final_value=[np.pi + 0.3]))
print(step_info(C @ G))                                 # rise time, overshoot, settling time

# The digital implementation: the same law sampled at Δ = 0.01 s; the plant stays continuous
loop_c = C @ plant                                      # the continuous loop
loop_d = (C % 0.01) @ plant                             # a SampledDataSystem at the base period 0.01 s
loop_d.plot_diagram()                                   # computer | ZOH | plant | sampler
traj_d = loop_d.compute_trajectory(tf=3.0)              # the pure base-tick scan, substeps recorded
traj_c = loop_c.compute_trajectory(tf=3.0)
loop_d.plot_trajectory(traj_d, overlay=traj_c)          # the sampled loop against the continuous one
print(loop_d.poles())                                   # the sampled-data loop's poles, against the unit circle
```

What prints: `margins(L)` (`GM = 12.1 dB at 9.8 rad/s, PM = 48° at 3.1 rad/s`), `step_info`
(the textbook table), the poles of the step diagram at the upright point. What plots: Bode,
the pole-zero map, the two loops overlaid. The tree: `loop_d.params` is
`{"controller": {…PID…}, "plant": {"m", "l", "I", "gravity", "d"}}`; the sampling period is
structural (`loop_d.sample_period == 0.01`), never a parameter.

### 5.3 State-space design with an observer, and a Monte Carlo over draws

```python
"""LQG on the pendulum: the gain, the Kalman filter, the loop by role, and its Monte Carlo."""
import numpy as np
from minilink import (Pendulum, QuadraticCost, StochasticPlanningProblem, Gaussian, Uniform,
                      WhiteNoise, lqr, kalman, closed_loop, evaluate)

plant = Pendulum()
x_bar = np.array([np.pi, 0.0])
K = lqr(plant, Q=np.diag([10.0, 1.0]), R=np.eye(1) * 0.1, x_bar=x_bar)      # a StateFeedback block
obs = kalman(plant, Q_n=np.diag([0.0, 0.5]), R_n=np.eye(2) * 1e-3, x_bar=x_bar)  # a StateObserver block
loop = closed_loop(K, plant, observer=obs)         # y and u into the observer, x̂ into the gain
loop.plot_diagram()

# The task with its draws: a start law, a parameter law, noise sources on the plant's ports
problem = StochasticPlanningProblem(
    plant,
    cost=QuadraticCost.from_system(plant, Q=np.diag([10.0, 1.0]), R=0.1 * np.eye(1), xbar=x_bar),
    tf=5.0,
    x_start=Gaussian(mean=x_bar + [0.3, 0.0], std=[0.2, 0.2]),
    params_distribution={"m": Uniform(0.8, 1.2)},
    disturbances={"d": WhiteNoise(psd=0.5, sample_period=0.01)},         # a block, wired onto the port d
    X=plant.state.box, U=plant.input_box, infeasible_cost=True,
)

report = evaluate(problem, loop, dt=0.01, n_trials=200, key=1)    # 200 draws of (x0, m, the noise seed), one key
print(report)
report.plot_trajectories(signals=("plant:x", "observer:x"))        # the fan of runs, the estimate against the state
print(problem.realize(key=1, n_trials=3))                           # the draws themselves: (x0s, params)
```

What prints: `J over 200 runs: objective 4.12, mean 4.12, std 0.61, worst 6.3; feasible 100%,
reached 100%` and the three realizations (the starts `(3, 2)`, the tree with `plant.m` of
shape `(3,)` and `d.seed` of shape `(3,)`, every other leaf shared). What plots: the diagram
with the observer under its role, the fan of runs. The tree the evaluator holds:

```text
pe.params = {
  "controller": {"K": (1, 2), "xbar": (2,), "ubar": (1,)},
  "observer":   {"L": (2, 2), "A": …, "B": …, "C": …, "x0": (2,)},       # x̂₀ a leaf (R124)
  "plant":      {"m", "l", "I", "gravity", "d"},
  "d":          {"seed": int, "sample_period": 0.01, "psd": (1,)},        # the noise source by its port's name
  "cost":       {"Q", "R", "S", "xbar", "ubar"},
  "X": {}, "U": {},
  "x_start":    (2,),  "discount_rate": 0.0,
}
```

### 5.4 The digital loop and its gradient

```python
"""Tune a digital PID by gradient through the sampled loop; check the gradient; hand it to the optimizer."""
import jax, numpy as np
from minilink import Pendulum, PID, PlanningProblem, QuadraticCost, Optimizer

plant = Pendulum()
x_bar = np.array([np.pi, 0.0])
problem = PlanningProblem(plant, cost=QuadraticCost.from_system(plant, Q=np.diag([1.0, 0.1]), R=0.01 * np.eye(1), xbar=x_bar),
                          tf=3.0, x_start=x_bar - [0.5, 0.0])
pid = PID(Kp=20.0, Ki=5.0, Kd=4.0, tau=0.05)

pe = problem.compile(pid % 0.01, backend="jax")     # the sampled loop; dt is the loop's base period, 0.01 s
print(pe.evaluate())                                  # J = 2.31, cost = 2.31, feasibility margin = inf
dJ = jax.grad(lambda p: pe.evaluate(p).objective)(pe.params)
print(dJ["controller"]["Kp"], dJ["plant"]["m"])       # the gradient reaches the gain and the plant's mass alike

program = pe.program(wrt=["controller.Kp", "controller.Ki", "controller.Kd"])   # z = (Kp, Ki, Kd), ∇J by reverse mode
best = Optimizer(program, method="l-bfgs").solve()
tuned = pe.unpack(best.z, program.metadata["wrt"])
print(pe.evaluate(tuned))
pe.trajectory(tuned).of("plant").plot()
```

What prints: the two evaluations (`J = 2.31` then `J = 0.87`), the two gradient entries. What
plots: the tuned plant run. The tree: `pe.params = {"controller": {"Kp", "Ki", "Kd", "tau", …,
"x0": (2,)}, "plant": {…}, "cost": {…}, "X": {}, "U": {}, "x_start", "discount_rate"}`; the
PID's own `x0` (its integrator and filter memory) is a leaf like any other. On the JAX face the
whole `evaluate` is one `lax.scan` over 300 base ticks (prototype 2: `jax.grad` of this loop's
cost with respect to `Kp` matches a central difference to 10 digits).

### 5.5 Trajectory optimization and MPC, the program rebound without recompiling

```python
"""Swing the pendulum up by collocation, then run the same program receding on a computer at 20 Hz."""
import numpy as np
from minilink import (Pendulum, PlanningProblem, QuadraticCost, SingletonSet,
                      TrajectoryOptimizationPlanner, ModelPredictiveController)

plant = Pendulum()
x_goal = np.array([np.pi, 0.0])
problem = PlanningProblem(plant, cost=QuadraticCost.from_system(plant, Q=np.diag([1.0, 0.1]), R=0.01 * np.eye(1), xbar=x_goal),
                          tf=3.0, x_start=np.zeros(2), Xf=SingletonSet(x_goal), U=plant.input_box)

planner = TrajectoryOptimizationPlanner(problem, n_steps=40, transcription="direct_collocation", backend="jax")
solution = planner.solve()                       # the offline plan
print(solution)                                  # the record: success, iterations, the worst defect
solution.plot_trajectory()
(solution.policy >> plant).animate()             # the open-loop replay

# The program is built once with parameters; a new plant mass or target is a solve, not a build
program = planner.program                        # MathematicalProgram(n_z=120, params=problem.params)
print(program.params["plant"]["m"], program.params["x_start"])
heavier = planner.solve(params={"plant": {"m": 1.3}})                      # a partial tree, merged at the boundary
moved = planner.solve(params={"cost": {"xbar": x_goal + [0.1, 0.0]}}, z0=solution.solver.z)   # warm started
print(heavier.solver.solve_time_s, moved.solver.solve_time_s)              # milliseconds: bind + SLSQP, no retrace

# The same planner receding: a step block, the plant stays continuous
mpc = ModelPredictiveController(planner, sample_period=0.05)   # the host solver to tolerance (SLSQP), the NumPy face
loop = mpc % 0.05 @ plant
loop.plot_diagram()
traj = loop.compute_trajectory(tf=4.0)           # the pure runner; the MPC tick is host code, the plant steps with 5 substeps
loop.plot_trajectory(traj)
loop.animate(traj, overlays=mpc.horizon_overlay())
```

What prints: the solution record (`direct collocation: success, 23 SLSQP iterations, defect
3e-9`), the two parameter leaves, the two solve times. What plots: the plan, the replay, the
sampled loop's run and the animation with the receding horizon. The program's tree is the
problem's (D5): `program.params == problem.params`, so `planner.solve(params={"x_start": y})`
is what the MPC does at every tick. The loop's tree:
`loop.params = {"controller": {"cost": {…}, "plant": {…}, "x0": (120,)}, "plant": {…}}`: the
MPC's internal model and cost are its parameters, distinct from the real plant's (model
mismatch is `loop.params["plant"]["m"] = 1.3`).

### 5.6 The native fixed-iteration MPC, tuned by gradient

```python
"""An MPC whose tick is 15 projected-gradient steps: its closed loop compiles, batches and differentiates."""
import jax, numpy as np
from minilink import (Pendulum, PlanningProblem, QuadraticCost, TrajectoryOptimizationPlanner,
                      ModelPredictiveController, Optimizer)

plant = Pendulum()
x_goal = np.array([np.pi, 0.0])
task = PlanningProblem(plant, cost=QuadraticCost.from_system(plant, Q=np.diag([1.0, 0.1]), R=0.01 * np.eye(1),
                                                              S=np.diag([10.0, 1.0]), xbar=x_goal),
                       tf=1.0, x_start=x_goal - [0.6, 0.0], U=plant.input_box)
planner = TrajectoryOptimizationPlanner(task, n_steps=20, transcription="shooting", backend="jax")
mpc = ModelPredictiveController(planner, sample_period=0.05, iterations=15, method="gd", options={"learning_rate": 0.3})

# The closed loop as a problem: the real plant, the real cost, 3 s
problem = PlanningProblem(plant, cost=task.cost, tf=3.0, x_start=x_goal - [0.6, 0.0], U=plant.input_box)
pe = problem.compile(mpc % 0.05, backend="jax")            # one scan: 60 ticks, each 15 kernel steps on a 20-step shooting program
print(pe.evaluate())                                       # J = 0.665
batch = pe.evaluate(key=0, n_trials=32)                    # the same loop vmapped over 32 starts (a deterministic problem: one row per start given)
dJ = jax.grad(lambda p: pe.evaluate(p).objective)(pe.params)
print(dJ["controller"]["cost"]["S"], dJ["controller"]["solver"]["learning_rate"])   # the gradient through the solver's iterations

program = pe.program(wrt=["controller.cost.S", "controller.cost.Q"])
best = Optimizer(program, method="adam", options={"learning_rate": 0.05, "maxiter": 50}).solve()
print(pe.evaluate(pe.unpack(best.z, program.metadata["wrt"])))
```

What prints: `J = 0.665`, the batch's summary, the two gradient leaves, the tuned evaluation.
The tree: `pe.params["controller"] = {"cost": {"Q", "R", "S", "xbar", "ubar"}, "plant": {…},
"solver": {"learning_rate": 0.3}, "x0": (20,)}`; `iterations=15`, `n_steps=20` and
`sample_period` are structural. Prototype 3 is this loop in full: 60 ticks jitted in 134 ms a
call, 32 starts in 477 ms, `jax.grad` with respect to `S₀₀` and to the step size matching
central differences to 8 digits, the pendulum held at `θ = 3.1428`.

### 5.7 Value iteration and reinforcement learning on one `J`, a field as terminal cost

```python
"""Value iteration gives the cost-to-go as a field; RL on a short horizon uses it as terminal cost; one J judges both."""
import numpy as np
from minilink import (Pendulum, PlanningProblem, StochasticPlanningProblem, Uniform, QuadraticCost,
                      DynamicProgrammingPlanner, ReinforcementLearningPlanner, compare)

plant = Pendulum()
x_goal = np.array([np.pi, 0.0])
cost = QuadraticCost.from_system(plant, Q=np.diag([1.0, 0.1]), R=0.01 * np.eye(1), xbar=x_goal)
problem = PlanningProblem(plant, cost=cost, tf=np.inf, discount_rate=0.1, X=plant.state.box, U=plant.input_box, infeasible_cost=True)

vi = DynamicProgrammingPlanner(problem, x_grid=(101, 101), u_grid=11, dt=0.05)   # Euler successors: c_k = g dt exactly
vi_solution = vi.solve()
print(vi_solution.solver)                        # value iteration: converged in 312 sweeps
V = vi_solution.cost_to_go                       # a GridField on the planner's grid
V.plot()                                         # the cost-to-go surface
vi_solution.plot_control_law()

# A finite-horizon stochastic problem whose terminal cost is the table: h(x_N) = V(x_N)
short = StochasticPlanningProblem(plant, cost=cost + V.as_cost(terminal=True), tf=2.0, discount_rate=0.1,
                                  x_start=Uniform([-np.pi, -1.0], [np.pi, 1.0]), X=plant.state.box, U=plant.input_box, infeasible_cost=True)
rl = ReinforcementLearningPlanner(short, dt=0.05, hidden=(32, 32), algorithm="ppo", n_envs=64)
rl_solution = rl.solve(timesteps=100_000)       # trains on the same stage cost c_k the evaluator scores
rl.plot_learning_curve()

# One judge: the same draws, the same stage cost, the same failure rule, for both laws
cmp = compare(short, VI=vi_solution, PPO=rl_solution, dt=0.05, n_trials=100, key=1, backend="jax")
print(cmp)
cmp.plot_control_law()
print(cmp.difference("VI", "PPO"))               # the paired J_VI − J_PPO over the same 100 draws
```

What prints: the DP record, the comparison table (one row per law: objective, mean, std, worst,
the feasibility and reach rates), the paired difference. What plots: the cost-to-go field, the
two laws overlaid on one colour scale, the learning curve. The RL planner's reward is `−c_k`
from the same `Φ` the evaluator scans (D1), so the learning curve and the comparison speak one
number; `rl_solution.cost_to_go` is the critic as a `Field` when its discount is the problem's.
The tree the comparison scores on: `{"controller": {…the law…}, "plant", "cost": {"QuadraticCost":
{…}, "FieldCost": {"weight": 1.0}}, "X", "U", "x_start", "discount_rate"}` with `x_start` drawn.

### 5.8 A stochastic sampled loop, a seeded policy vmapped over seeds, tuned by gradient

```python
"""A Gaussian policy with a seed in its parameters: one key, 256 seeds in one vmap, the mean score tuned by gradient."""
import jax, numpy as np
from minilink import (Pendulum, StochasticPlanningProblem, Gaussian, WhiteNoise, QuadraticCost,
                      NeuralPolicy, GaussianPolicy, Optimizer)

plant = Pendulum()
x_goal = np.array([np.pi, 0.0])
problem = StochasticPlanningProblem(
    plant, cost=QuadraticCost.from_system(plant, Q=np.diag([1.0, 0.1]), R=0.01 * np.eye(1), xbar=x_goal), tf=5.0,
    x_start=Gaussian(mean=x_goal - [0.3, 0.0], std=[0.05, 0.05]),
    disturbances={"d": WhiteNoise(psd=0.2, sample_period=0.05)},
    U=plant.input_box, infeasible_cost=True,
)
mean = NeuralPolicy(plant, hidden=(16, 16), seed=0)            # u = μ_θ(x); seed draws the weights once, at construction
policy = GaussianPolicy(mean, log_std=np.log(0.3), sample_period=0.05)   # u = μ(x) + σ ε(seed, ⌊t/Δ⌋); seed=None: the mean

pe = problem.compile(policy % 0.05, backend="jax")
report = pe.evaluate(key=7, n_trials=256)                       # realize: x0, the load's seed, the policy's seed, per trial
print(report)                                                   # J over 256 runs: objective 13.4, std 0.53, …
print(policy.log_prob(x_goal, np.zeros(1)))                     # the density of an action, the RL heads' own method

program = pe.program(wrt=["controller.mean.mlp", "controller.log_std"], key=7, n_trials=256)   # a sample average over fixed draws
best = Optimizer(program, method="adam", options={"learning_rate": 0.5, "maxiter": 30}).solve()
tuned = pe.unpack(best.z, program.metadata["wrt"])
print(pe.evaluate(tuned, key=8, n_trials=256))                  # scored on fresh draws: J over 256 runs: objective 0.22, …
pe.trajectory(tuned, key=8).of("plant").plot()
```

What prints: the two evaluations (`13.4` then `0.22` on the prototype's linear mean), the
log-probability. What plots: one tuned run. The tree:

```text
pe.params = {
  "controller": {"mean": {"mlp": {…weights…}}, "log_std": (1,), "seed": int},   # the seed an integer leaf, drawn per trial
  "plant": {…}, "d": {"seed": int, "sample_period", "psd"},
  "cost": {…}, "U": {}, "x_start": (2,), "discount_rate": 0.0,
}
```

`problem.realize(key, 256)` draws `x_start` from its law and gives `controller.seed` and
`d.seed` one child seed each per trial, by name (`child_seed(key, "trial i/controller.seed")`),
so adding a noise block never reshuffles another's stream, and the same key gives the same bits
on NumPy and JAX (prototype 4). The pathwise gradient through the seeded noise is what tunes
the law (prototype 5: 256 seeds in 14 ms a call, the gradient with respect to the gain and to
`log_std` matching central differences to 9 digits, the mean score from 13.4 to 0.22 in 30
steps).

---

## 6. The final API, object by object

Signatures are given in full; "today" cites the file and line of what exists; "changes" says
what moves. The one contract (R140) applies to every object below: its mathematics is a
function of its specific arguments and `params`, `params=None` reads the frozen defaults,
composed objects nest their parts' dicts, leaves are numbers (a seed the one integer leaf),
and every function is deterministic given its parameters.

### 6.1 Systems, step systems, the sampled block and the sampled-data loop

```python
class System:                                           # today core/system.py:26
    def h(self, x, u, t=0.0, params=None): ...          # :154, unchanged
    def tf(self, x, u, t=0.0, params=None): ...         # :394, unchanged
    def realize(self, key) -> dict: ...                 # :190, unchanged: this block's params with a fresh seed under key
    def __add__ / __rshift__ / __matmul__               # :409–:435, unchanged
    def __mod__(self, sample_period) -> StepSystem      # :452: returned a Computer; now the block sampled (6.1, `Sampled`)

class DynamicSystem(System):                            # :516
    def f(self, x, u, t=0.0, params=None): ...          # :565, unchanged
    x0: ndarray                                         # the block's own start (R124); a diagram's x0 is a write-through view

class StepSystem(System):                               # today core/discrete/step_system.py:16
    def __init__(self, n, *, input_dim=None, output_dim=None, expose_state=False,
                 y_dependencies=(), sample_period=None): ...       # new: sample_period, seconds per tick (None: a tick system)
    def step(self, x, u, k=0, params=None): ...         # :62, unchanged
    def h(self, x, u, k=0, params=None): ...            # :87, unchanged
    def compute_rollout(self, n_steps, u=None, *, x0=None, backend="numpy") -> Trajectory   # t = k·Δ, or k (D8)
    def jacobian(self, k=0, ...) ; def poles(self, x_bar=None, u_bar=None) -> ndarray      # new: the eigenvalues of ∂step/∂x

class StepDiagramSystem(WiredDiagramMixin, StepSystem): # today core/discrete/step_diagram.py:13
    # the same wiring verbs as DiagramSystem; `+`, `>>`, `@` return a StepDiagramSystem (R126)
    # multi-rate: a block's sample_period must be a multiple of the diagram's base period (their gcd);
    # a block with divisor d > 1 fires on k mod d == 0 (a select) and its held outputs are state slots of the diagram
    sample_period: float | None                         # the base period, derived from the blocks

def discretize(plant, sample_period, *, solver="rk4", substeps=1) -> Discretization   # today analysis/discretization.py:21, moves to core/discrete/
class Discretization(StepSystem):                       # today :50; step at :130 binds t_k = k·dt; it keeps `source`
```

**The sampled block.** `ctl % Δ` (today `System.__mod__` builds a `Computer`,
`simulation/discrete/computer.py:34`) returns:

```python
def sampled(block, sample_period, *, solver="euler", substeps=1) -> StepSystem | System:
    """The block as a computer runs it: every Δ, between samples its output held.

    A static block (n == 0) comes back tagged with its period (its h at t_k = kΔ);
    a DynamicSystem comes back as discretize(block, Δ, solver, substeps), the textbook digital
    implementation; a StepSystem comes back tagged with Δ (an MPC, a digital filter).
    """
```

**The sampled-data loop.** `(ctl % Δ) @ plant`:

```python
class SampledDataSystem(StepDiagramSystem):             # replaces core/discrete/hybrid_diagram.py:30 (HybridDiagram)
    """A computer at its clocks around a continuous plant, as one step diagram at the base period.

    Blocks: the computer's step blocks under their ids (`controller`, `observer`, …) and the plant
    as `discretize(plant, base_period, substeps=n)` under `plant`; the held commands as state;
    `source` the continuous plant for the orchestrator and the drawing.
    """
    source: DynamicSystem
    sample_period: float                                # the base period
    def compute_trajectory(self, tf, *, x0=None, solver=None, backend="numpy", verbose=True) -> Trajectory
        # solver=None: the pure base-tick scan, the plant's substeps recorded (dense, t in seconds)
        # solver="scipy*": the clock runner with the adaptive solver between input changes (§7.3)
    def plot_diagram(self, ...)                         # the two-side drawing, computer | ZOH | plant | sampler
    def animate(self, traj=None, ...)                   # the plant's animation, the computer's overlays
```

`@` between a step left operand carrying a `sample_period` and a continuous plant builds it
(today `Computer.__matmul__`, `computer.py:173`, and `hybrid_closed_loop`,
`core/discrete/hybrid_composition.py:26`, whose port resolution is kept as is); any other mixed
pair is refused naming `%` or `discretize` (R126). The substep count defaults to the continuous
rule (`time_grid.py`'s `AUTO_DT_SCALE × τ_min` against the base period), one owner.

### 6.2 Trajectories, sets, costs, distributions, sources, fields

```python
@dataclass(frozen=True)
class Trajectory:                                       # today core/trajectory.py:52, unchanged fields
    t: ndarray; x: ndarray; u: ndarray; signals: dict; roles: dict
    # a step rollout is a Trajectory with t = k·Δ (D8); StepRollout (core/discrete/step_rollout.py:18) goes
    def of(self, role) -> Trajectory; def with_cost(self, cost, *, discount_rate=0.0, params=None)
    def plot(self, signals=("x", "u"), *, overlay=None, backend="matplotlib", show=True)   # the one plotting verb every record shares

class Set:                                              # today core/sets.py:22
    params: dict                                        # {} when none (R140 point 2; today None)
    def margin(self, z, t=0.0, params=None); def contains(...); def sample(self, key, n=None, params=None); def bounding_box(self)
    def as_cost(self, weight=1.0, shaping=quadratic_hinge) -> CostFunction      # new: Field.as_cost on the margin as a field (R133)
class InputSet: def margin(self, u, x=None, t=0.0, params=None) ...               # :90, unchanged

class CostFunction:                                     # today core/costs.py:29
    params: dict
    def g(self, x, u, t=0.0, params=None); def h(self, x, t=0.0, params=None)     # :58, :63, unchanged
    def integrate(self, traj, *, discount_rate=0.0, params=None)                  # :95 stays: the trapezoid on a held Trajectory (plots, with_cost)
    # the stage cost of a stepping tool is not here: it is stepping.stage_body (6.4), which reads g and h

class Distribution:                                     # today core/distributions.py:24
    params: dict                                        # the law's numbers: Gaussian {"mean", "std"} or {"mean", "cov"}
    def sample(self, key, n=None, params=None)          # :46 gains params (R140 point 1); key an int, a NumPy generator or a JAX key
    mean: property                                      # unchanged; no log_prob here (R129)

class WhiteNoise(Source):                               # today blocks/sources.py:211, unchanged: h at :257 is ε(seed, ⌊t/Δ⌋), realize at :290
class NoiseSource(Source):                              # new (RN-3): held draws w_k ~ law, params {"seed", "sample_period", law.params}

class Field:                                            # new core/fields.py; today planning/spatial/state_fields.py:26 (StateField)
    params: dict
    def value(self, x, u=None, t=0.0, params=None)      # :30, the signature as is
    def gradient(self, x, u=None, t=0.0, params=None)   # jax.grad under JAX, central differences on NumPy
    def as_set(self, lower=None, upper=None) -> Set; def as_input_set(self, lower=None, upper=None) -> InputSet
    def as_cost(self, weight=1.0, shaping=None, *, terminal=False) -> CostFunction   # g = w·s(V) (running) or h = w·s(V) (terminal)
class QuadraticField(Field):  P, center; value = (x − c)ᵀ P (x − c); as_set(upper=level) -> EllipsoidSet (sample, bounding_box)
class GridField(Field):       grid, values (a leading time axis optional); value by interpolation, 0 outside the grid (the DP numbers hold)
```

### 6.3 The problem and its draws

```python
class PlanningProblem:                                  # today planning/problems.py:29; __init__ at :111
    def __init__(self, sys, x_start=None, x_goal=None, cost=None, X=None, U=None, X0=None, Xf=None,
                 tf=None, infeasible_cost=None, discount_rate=0.0, disturbances=None): ...
        # new: disturbances={"d": block} wires the source onto the plant's port d (R128), for both problems;
        # sys may be a StepSystem (a natively discrete task: tf in seconds when it has a period, else in ticks)
    params: property                                    # :198, the live view {plant, cost, X, U, X0, Xf, x_start, discount_rate}, unchanged
    def realize(self, key=None, n_trials=1) -> tuple[ndarray, dict]      # :246; one shape (R128): (x0s (n, nx), params with a trial axis on drawn leaves and every source's seed)
    def compile(self, policy, dt=None, *, solver="rk4", substeps=1, backend="auto", episode_length=None) -> ProblemEvaluator   # :358
        # dt required for a continuous policy; read from the loop for a sampled one (`ctl % Δ`, a SampledDataSystem) or a StepSystem plant, a different dt refused
    def replace(self, **changes); def nominal(self); def cost_bound(...); def discount_factor(self, dt)   # unchanged

class StochasticPlanningProblem(PlanningProblem):       # today :506; __init__ at :551
    def __init__(self, sys, x_start: Distribution, ..., params_distribution=None, disturbances=None, criterion="expectation"): ...
        # x_start a Distribution (D7; today x0_distribution); disturbances blocks only (today Distributions, :755 refuses the rest)
```

`realize` (today `:638`) draws the starts and the parameter laws with the library's own cipher
from the key, and gives every random block of the loop (the policy's `seed`, each source's
`seed`) one child stream per trial by name: `child_seed(key, f"trial {i}/{path}")`
(`core/distributions.py:275`); the same key gives the same bits on NumPy and JAX (KNOWN_ISSUES'
first entry closes). The result is `(x0s, params)`; the `"disturbances"` key goes.

### 6.4 Compile, the evaluator, the evaluation and the one score

The step model (`core/compile/evaluators/stepping.py`) gains the stage body beside `step_body`
(`:40`), and the one scan beside `iterate` (`:62`):

```python
def stage_body(f, g, x, u, t_k, dt, rho, solver, substeps):
    """(x_{k+1}, c_k): the plant one period later and the stage cost integrated with the state.

    X = [x; c], Ẋ = [f(x, u, t); e^{−ρ(t − t_k)} g(x, u, t)], stepped by `solver` with `substeps`,
    u held; so c_k = g(x_k, u_k, t_k)·dt under Euler, the RK4 quadrature under RK4.
    """
```

The compiled evaluators (`core/compile/evaluators/base.py`, the frozen subset of DESIGN §5)
keep `f`, `outputs`, `signals`, `step` (`:96`), `rollout` (`:110`), `rollout_batch` (`:146`),
`jacobian`; a step evaluator's `rollout` (`discrete.py:39`) returns native arrays and
differentiates end to end (today it returns host NumPy), and gains `rollout_batch`.

```python
class ProblemEvaluator:                                 # today planning/evaluator.py:40; __init__ at :77
    params: property                                    # :126 unchanged in shape; each stateful block gains its "x0" leaf (R124)
    def trajectory(self, params=None, key=None) -> Trajectory          # :146, unchanged
    def evaluate(self, params=None, key=0, n_trials=None, *, record=False) -> Evaluation   # :166, unchanged
    def program(self, wrt, bounds=None, key=0, n_trials=None, *, params=None) -> MathematicalProgram   # :195, unchanged
    def pack(self, params, wrt) -> ndarray; def unpack(self, z, wrt, params=None) -> dict   # :289, :298, unchanged
```

**What changes inside** (`run`, `:355`, and `score`, `:439`): one scan over `k` carries
`(x_k, J_k, margin_k)` and accumulates `γ^k c_k` from `stage_body` and the least margin at the
samples; `score_run` (`planning/evaluation.py:125`) keeps its failure and price rules and
reads `J` from the scan instead of `CostFunction.integrate`'s trapezoid. The three loops the
evaluator serves (R105): a continuous law (`ctl @ plant`, the diagram discretized at `dt`, the
controller's states integrated with the plant's: today's path), a sampled law (`ctl % Δ`, the
`SampledDataSystem` scanned at its base period, the stage cost per base tick), a `StepSystem`
plant (`c_k = g Δ`). `require_continuous` (`:789`) goes. `evaluate(problem, policy, *, dt, ...)`
(`:502`) and `compare(problem, ..., dt=, n_trials=, key=)` (`planning/comparison.py:25`) are
the facades, unchanged.

```python
@dataclass
class Evaluation:                                       # today planning/evaluation.py:27, unchanged
    cost, J, feasibility_margin, failed, reachability_margin, reached, x0, trajectories, criterion
    objective, mean, std, worst, cvar(alpha), feasibility_rate, reachability_rate, success_rate
    def plot_trajectories(self, signals=("x", "u"), ...)  # new: the fan of recorded runs
```

The RL environment (`learning/reinforcement_learning/environment.py:17`, `step` at `:130`) and
the grid's successor (`planning/dynamic_programming/discretizer.py:207`) call `stage_body`; the
reward is `−c_k` and the running-cost table (`dp.py:338`) is the stage-cost table, the Euler
numbers unchanged.

### 6.5 Programs, the optimizer and the kernels

```python
@dataclass(frozen=True)
class MathematicalProgram:                              # today optimization/mathematical_program.py:26
    n_z: int
    J: Callable            # J(z, params=None) -> scalar; a hand-written J(z) is accepted (its arity is read once at construction)
    h: Callable | None; g: Callable | None               # h(z, params=None) -> (n_h,), g(z, params=None) -> (n_g,)
    lower, upper; grad_J, hess_J, jac_h, jac_g; backend; metadata
    params: dict = {}                                   # new (R130): the frozen defaults; an exported program's is the problem's tree (D5)
    def params_at(self, z) -> dict                      # the full tuned tree of an exported program (R134 line 6)

class Optimizer:                                        # today optimization/optimizer.py:75; __init__ at :118
    def __init__(self, program, z0=None, *, method="auto", backend=None, use_hessian=False, options=None, **method_kwargs)
        # compile_backend → backend (R134); the solver adapter is .adapter (today .backend, :148)
    def solve(self, *, z0=None, params=None, callback=None, record_solve_time=False, verbose=False) -> OptimizationResult   # :157 gains params
    def iterate(self, z0=None, *, params=None, n_steps) -> tuple[ndarray, Trajectory]      # new: K steps of a native kernel in one scan, traced
    def as_step_system(self) -> StepDiagramSystem        # new: the update law z⁺ = kernel(z, ∇J) wired to the oracle ∇J(z; p)
```

The native kernels (`optimization/native/`), the shape `Adam` already has
(`optimizers/first_order.py:135`, `init` at `:155`, `update` at `:167`):

```python
class GradientDescent:
    params = {"learning_rate": η}
    def init(self, z0, params=None) -> dict: {"z": z0}
    def step(self, state, grad, k, params=None) -> dict: {"z": z − η grad}
class Adam:  params = {"learning_rate", "b1", "b2", "eps"}; state {"z", "m", "v"}; step as today's update, k the tick
```

`FirstOrderOptimizer.solve` (`:56`, a Python loop) becomes the host run of a kernel to
`gtol`; the projected step `Π(z − η ∇J)` on the box is the kernel's, so `solve` and `iterate`
share one law. `ParametricMathematicalProgram` and `JaxParametricProgramEvaluator` fold into
the one program and the one evaluator body (OPW-1–OPW-5): the evaluator jits `J(z, p)` and its
derivatives in `z` once, the adapters (SciPy, Ipopt) close over `p` at each `solve(params=)`.

### 6.6 The policies

```python
class NeuralPolicy(System):                             # today control/neural.py:15 (NeuralPolicyController; the rename at the v1.0 window, R134)
    def __init__(self, sys, *, features=None, hidden=(64, 64), activation="tanh", normalize=True, squash="clip", seed=0, ...)   # :71
    params = {"mlp": {...}}; ctl(x, u, t, params) the law                                   # :204
class GaussianPolicy(System):                           # new control/stochastic.py (R129)
    def __init__(self, mean: System, log_std, *, sample_period, seed=None): ...
    params = {"mean": mean.params, "log_std": (m,), "seed": int | None}
    def ctl(self, x, u, t=0.0, params=None):   # u = μ(x) + exp(log_std) · ε(seed, ⌊t/Δ⌋); ε by standard_normal (distributions.py:230); seed None: μ
    def log_prob(self, x, u, params=None); def entropy(self, x, params=None)
class SquashedGaussianPolicy(GaussianPolicy)            # the SAC head
class LookupPolicy, StateFeedback, TimeVaryingFeedback, TrajectoryFeedback, PID, …   # unchanged
```

The RL algorithms train `GaussianPolicy`'s tree (`{"mean": …, "log_std": …}`) in place of the
package's `StochasticPolicy` (`learning/reinforcement_learning/policy.py`), the collectors
giving each episode its seed from the planner's key by name.

### 6.7 Planners, the solution, the planner interface and the MPC

```python
class Planner:                                          # today planning/planner.py:21
    def solve(self, **kwargs) -> PlanningSolution        # :69, unchanged
    # the four verbs of a receding-horizon planner (R135, D4), on the trajectory-optimization planner first:
    program: MathematicalProgram                        # built once at construction, params = problem.params
    def solve(self, *, params=None, z0=None, evaluate=False) -> PlanningSolution   # today solve_trajectory_from(x0, params) at trajectory_optimization/planner.py:355
    def shift(self, z) -> ndarray                       # the warm start one period on
    def plan(self, z, params=None) -> Trajectory        # the decision read as a trajectory

@dataclass(frozen=True)
class PlanningSolution:                                 # today planning/results.py:21, unchanged fields
    problem, policy, solver, trajectory, evaluation, cost_to_go: Field | None    # cost_to_go typed as a Field (D6)

class ModelPredictiveController(StepSystem):            # today MPCStatefulController, mpc/controller.py:471
    def __init__(self, planner, *, sample_period, iterations=None, method="gd", options=None, parameter_ports=()): ...
    params = {**planner.program.params, "solver": {...}}    # the internal problem's tree and the kernel's; x0 the default plan
    def step(self, x, u, k=0, params=None):
        # z_{k+1} = solve(shift(z_k); params | {"x_start": y_k}): K kernel steps (iterations=K, traced) or the host solver to tolerance
    def h(self, x, u, k=0, params=None):                # u_k = the first input of z_k (R123: one period late, no feedthrough from y)
```

`parameter_ports=["plant.mu"]` declares optional input ports that feed leaves of the tree per
tick (R134 line 6); an unwired port reads its nominal (RULES 4.5).

---

## 7. The architecture under it

### 7.1 Layers

```text
 ┌──────────────────────────────────────────────────────────────────────────────────────────┐
 │  4  Tools (verbs on a System or a problem; return domain objects, numbers or figures)    │
 │     simulation/  analysis/  planning/  learning/  optimization/  graphical/  interfaces/ │
 │     Simulator (adaptive, continuous)  ·  ProblemEvaluator (the one scan)  ·  planners    │
 │     Optimizer (adapters + native kernels)  ·  the clock runner (realtime, host blocks)   │
 ├──────────────────────────────────────────────────────────────────────────────────────────┤
 │  3  System libraries (blocks you drop into a diagram, shelved by role)                   │
 │     blocks/ (sources, routing, filters, MLP, UnitDelay)  dynamics/ (plants)              │
 │     control/ (laws, policies: PID, StateFeedback, NeuralPolicy, GaussianPolicy, MPC*)    │
 │     estimation/ (observers)                                                              │
 ├──────────────────────────────────────────────────────────────────────────────────────────┤
 │  2  The compiler and the kernel (library-developer code)                                 │
 │     core/compile/: one evaluator per system kind · stepping.py: step_body, stage_body,   │
 │     scan · plan.py: the frozen structure (ids, slices, divisors, holds, substeps)         │
 ├──────────────────────────────────────────────────────────────────────────────────────────┤
 │  1  The two algebras and the textbook nouns (student-facing, NumPy-only)                 │
 │     System, DynamicSystem, DiagramSystem  ·  StepSystem, StepDiagramSystem,              │
 │     Discretization, sampled (%), SampledDataSystem  ·  Trajectory, Set, Distribution,    │
 │     CostFunction, Field, params.py (the tree)  ·  PlanningProblem lives one floor up      │
 └──────────────────────────────────────────────────────────────────────────────────────────┘
   * the MPC block is defined with its planner under planning/trajectory_optimization/mpc/
     and re-exported by minilink.control (the exception DESIGN §3 already writes)
```

### 7.2 The one contract and the one kernel

**The contract** is R140, taken as the first row of the core: `f(x, u, t, params=None)`,
`step(x, u, k, params=None)`, `h`, `g`, `margin`, `value`, `sample(key, n, params=None)`,
`J(z, params=None)`; `params` a dict on every core math object (`{}` for none); composed
objects nest; numeric leaves, the seed the integer leaf; deterministic given the tree. One test
walks every public object on both backends (the row's "done when").

**The kernel** is one function and one scan:

```text
Φ(x_k, u_k, k; p) = (x_{k+1}, c_k)                      stepping.stage_body on a continuous plant,
                                                        the block's own step and c_k = g Δ on a step system
carry_k = (x_k, J_k, m_k)
u_k     = π(x_k, k; p)                                  the law: a diagram's wiring, propagated within the tick
m_{k+1} = min(m_k, X.margin(x_k, t_k), U.margin(u_k, x_k, t_k))
J_{k+1} = J_k + γ^k c_k
(x_N, J_N, m_N) = scan(body, carry_0, k = 0 … N−1);  J = J_N + γ^N h(x_N, t_N)
```

Who runs it: the step evaluator's `rollout` (no cost: `c_k` dropped), `ProblemEvaluator.run`
(the full carry), the RL environment (`step` = one body call, the reward `−c_k`), the grid's
successors (one body call per node and action), shooting (the scan with the cost as the
objective, `x_start` from `params`), the MPC's inner loop (`Optimizer.iterate`'s scan over the
kernel, nested in the outer tick scan), the Monte Carlo (`vmap` of the scan over the trial axis
of `params` and `x0s`).

### 7.3 Module layout and the dependency law

| Package | What moves or appears |
| --- | --- |
| `core/params.py` | new: `lookup`, `with_leaf`, `merge_tree`, `flat_leaves`, `paths_of`, `child_seeds_by_path`; the four copies (`planning/evaluator.py:991–1044`, `planning/problems.py:896–913`, `core/compile/evaluators/jacobian.py`, `core/assembly.py`) call it |
| `core/fields.py` | new: `Field`, `QuadraticField`, `GridField`, `EllipsoidSet`, the shaping functions (from `planning/spatial/shaping.py`) |
| `core/discrete/` | `step_system.py` (+ `sample_period`), `step_diagram.py` (+ holds and firing in the plan), `discretization.py` (from `analysis/`; `analysis.discretize` re-exports it, `ExactDiscretization` stays in `analysis/`), `sampled.py` (new: `sampled`, the `%` operator's body, and `SampledDataSystem`); `hybrid_diagram.py`, `hybrid_composition.py` (its port resolution kept inside `sampled.py`), `step_rollout.py` go |
| `core/compile/evaluators/stepping.py` | + `stage_body`, + `scan_with_score`; `discrete.py`'s `rollout` native, + `rollout_batch` |
| `core/compile/plan.py` | `StepExecutionPlan` gains per-block divisors and hold slots (structural) |
| `simulation/discrete/` | `computer.py`, `hybrid_simulator.py` go; `runner.py` new: the clock runner over the same compiled blocks (realtime, host-only blocks, the adaptive plant solve of `solver="scipy"`) |
| `planning/evaluator.py` | `run`/`score` on the scan; the three loop kinds; `require_continuous` goes |
| `planning/problems.py` | `x_start` three kinds; `disturbances` blocks on both classes; `realize` one shape with per-block seeds by name |
| `planning/evaluation.py` | `score_run` reads the scan's `J`; `Evaluation.plot_trajectories` |
| `planning/trajectory_optimization/` | the transcriptions read `params` (`x_start`, `plant`, `cost`, the sets); `planner.program`, `solve(params, z0)`, `shift`, `plan`; `mpc/controller.py` the one `ModelPredictiveController(StepSystem)`, the latch, the stateless block, the dual-rate export and `Command` gone |
| `planning/dynamic_programming/` | `discretizer.successor` on `stage_body`; `cost_to_go` a `GridField` |
| `learning/reinforcement_learning/` | `environment.step` on `stage_body`; the heads on `GaussianPolicy`; per-episode seeds by name |
| `optimization/` | `MathematicalProgram.params`; `Optimizer.solve(params=)`, `iterate`, `as_step_system`; `native/` (`gd.py`, `adam.py`); `parametric_program.py` and the parametric evaluator fold in |
| `control/stochastic.py` | new: `GaussianPolicy`, `SquashedGaussianPolicy` |

**The dependency law** (DESIGN §3) holds with two edits: `core/discrete/sampled.py` imports
`core/compile/evaluators/stepping.py` (inside `core`, no exception needed; the lazy
`core → simulation.discrete` import of R100 goes with `Computer`); `optimization/` imports
`core/discrete` for `as_step_system` (a tool importing core). `control/stochastic.py` imports
`core.distributions.standard_normal` (a library importing core). Nothing new crosses between
libraries.

### 7.4 The runtime model

| | What |
| --- | --- |
| **Compiled once** | the structure: block ids and state slices, the port graph and its order, the divisors and hold slots, `substeps`, `N`, `K`, `n_steps`, the batch layout (which leaves carry the trial axis). All static under `jit`. |
| **Varies per call** | `params` (one pytree), `x0`, `key` (through the seeds it realizes), `z0`. Values vary without a retrace; a new batch layout (a leaf newly drawn) is one more compiled variant, cached per layout as today (`planning/evaluator.py:462`). |
| **The runners** | the pure scan (JAX: `lax.scan`; NumPy: a Python loop over the same body, `stepping.iterate` `:62`); the clock runner (NumPy, a Python tick loop over the compiled blocks, skipping idle ones, calling `solve_ivp` between input changes when asked, polling I/O in realtime); the `Simulator` (continuous, adaptive, the reference). |
| **Where host code stays** | the SciPy and Ipopt adapters (an MPC with `iterations=None` is host code and runs on the NumPy face or the clock runner), plotting and animation, file I/O, the realtime loop, `print`. A loop that contains a host block does not trace, and `backend="auto"` falls to NumPy with a typed `NotTraceableError` message naming the block (S65's item). |

### 7.5 Traceability and efficiency, path by path

Each row names the main math path, why it traces, and the number measured (§10; CPU, float64).

| Path | Why it traces | Measured |
| --- | --- | --- |
| **The one score** (a continuous loop discretized, `evaluate`) | one `lax.scan` over `k`; `stage_body` is `substeps` static RK4 steps on the augmented state; the margins are `minimum`s in the carry; the failure rule a `where` after the scan; `params` a pytree argument | 1 000 starts × 200 RK4 steps in 44 ms (one `vmap`), 0.94 ms a single jitted run; `grad` w.r.t. the gain and the cost weight = central differences to 10 digits (part 1) |
| **The sampled loop** (`(pid % Δ) @ plant`) | the base-tick body is pure: firing a `where` on `k mod d`, the hold a carry entry, the plant's substeps static; multi-rate costs a discarded compute on idle ticks (the clock runner skips them on NumPy, the same numbers) | 100 starts × 3 000 base ticks in 93 ms; NumPy scan = clock orchestrator byte for byte; JAX = NumPy to 2·10⁻¹⁵; `grad` w.r.t. `Kp` = central difference to 10 digits (part 2) |
| **The native kernel and the MPC tick** | `iterate` is a `lax.scan` over `K` with `jax.grad` of the shooting program inside (itself a scan over the horizon); the tick nests in the loop's scan; the clip on the box has a zero gradient where it binds, as a projected method's does | 60 ticks × 15 iterations × a 20-step horizon: 134 ms a jitted call, 32 starts in 477 ms; `grad` w.r.t. the terminal weight and the step size = central differences to 8 digits (part 3) |
| **The draws** | the cipher is integer array arithmetic on `uint32` (`distributions.py:162`), vmappable over seeds; a trial's seeds are computed on the host once per key (`child_seed`), the signals inside the trace | 1 000 seeds × 4 samples in one `vmap`, NumPy = JAX bit for bit (part 4) |
| **The seeded policy, a Monte Carlo over seeds** | the seed is an integer leaf with a trial axis; `standard_normal(seed, k, m)` inside the body; the gradient is pathwise through the reparameterized noise | 256 trials × 100 steps in 14 ms; `grad` w.r.t. the gain and `log_std` = central differences to 9 digits; 30 gradient steps bring the mean score from 13.4 to 0.22 (part 5) |
| **Memory under `grad`** | a scan's reverse pass stores one carry per tick; for long horizons `jax.checkpoint` per tick (or per controller period in a sampled loop) is a knob on `compile`, not a design change | — |
| **What does not trace, by design** | a host-solver MPC (`iterations=None`), an adaptive plant solve (`solver="scipy"`), a block that calls out (hardware, a file); the clock runner serves them on NumPy with the same numbers for the parts that are pure | — |

The parameter-family price (S53: a `(B,)` operand folds nothing) is unchanged by this design;
its mitigations (hoisting a plant's pre-computation block out of the scan) stay a profiling row.

---

## 8. The v0.3 coverage table

| Workstream / choice | Where this design satisfies or revises it |
| --- | --- |
| §1 Every object pure, every number a parameter | §6 throughout; R140 the first row; `params={}`; `Distribution.sample(key, n, params)`; `core/params.py`; `x0` leaves per stateful block (R124) |
| §2 The step algebra first-class | `StepSystem.sample_period`, the operators on step systems (R126), native step rollouts with `rollout_batch`, `poles` on a step system, `StepRollout` folded (D8) |
| §3 The draws | `realize(key, n_trials) -> (x0s, params)` with per-block seeds by name, disturbances as blocks (R128), one draw on both backends (part 4); `NoiseSource` beside `WhiteNoise` |
| §4 The seeded stochastic policy | `GaussianPolicy` (R129), §5.8, part 5 |
| §5 The hybrid loop: one model, two runners | `sampled`, `SampledDataSystem` (D2), one verb two solvers (D3), the clock runner a runner over the same compiled blocks, byte-identical (part 2); the MPC's delay (R123) in `h` reading the state |
| §6 One score | D1: `stage_body`, the scan, the sets at `t_k`, one failure rule; VI and RL unchanged under Euler; the evaluator's and shooting's `J` move once (part 6) |
| §7 The parametric program; the stochastic program its seed case | `MathematicalProgram.params` the problem's tree (R130, D5), `Optimizer.solve(params=)`, `params_at`; the seed an integer leaf never in `wrt` (R131); the sample-average export kept (§5.8) |
| §8 The native proof of concept | the kernels, `Optimizer.iterate`, `as_step_system` (R132, D9); the MPC with `iterations=K` (§5.6, part 3) |
| §9 Fields | `Field`, `QuadraticField`, `GridField` (D6), `as_set` / `as_input_set` / `as_cost` (R133) |
| §10 Hardening and the merge | unchanged: S66, the material, the notebooks (R47), the merge (R79) |
| R124 three owners of a start | taken; `x_start` the one keyword for the problem's start, three kinds (D7) |
| R125 one same-tick semantic | taken: propagation within the tick in the pure body and the clock runner alike (part 2 shows the two agree) |
| R126 operators on step systems | taken; the mixed pair allowed exactly for a sampled left side (D2) |
| R127 `computer @ plant` a step diagram | taken in substance; the name `SampledDataSystem` now (D2); one verb, two solvers (D3) |
| R128 disturbances are source blocks | taken, on both problem classes; one `realize` shape |
| R129 the seeded policy | taken as is |
| R130 one program noun with `params` | taken; the key `x_start` (D5); the performance gate stays (a baked export if carrying `params` costs speed) |
| R131 the convention | taken as is |
| R132 the kernel with a step-system view | taken; `iterate` named (D9) |
| R133 a minimal `Field` | taken, plus `GridField` (D6) |
| R134 names across the API | taken: `backend=`, `adapter`, `sample_period`, `mount`, the separators, `…Policy`, `parameter_ports`, `params_at` |
| R135 a generic planner interface | the four verbs (D4) |
| R136 learning from data later | untouched |
| R137 no `KinematicModel` | untouched |
| R138 a basic `PlanningSolution` | taken; `cost_to_go` typed `Field` |
| R139 the road to the freeze | untouched; Diffrax would be `solver="diffrax"` on the clock runner's plant solve |
| R140 the one contract | the first row of the core (§7.2) |
| The one-score investigation (QUESTIONS) | answered by D1 |

---

## 9. The migration from today's code

Module by module, what changes, which numbers move (R80–R82: an intended change with a
closeness report and one re-pin; a restructure to `rtol 1e-6`; a move or rename byte-identical),
the risks, and a proposed order. The lanes of core-architecture §10 stand; the steps below say
what each lane's rows become under this design.

| Step | Module(s) | What changes | Numbers | Risk |
| --- | --- | --- | --- | --- |
| **M1 the contract and the tree** | `core/params.py` (new), `core/sets.py`, `core/distributions.py`, `core/costs.py`, `planning/evaluator.py`, `planning/problems.py`, `core/compile/evaluators/jacobian.py`, `core/assembly.py` | the four tree helpers into one module; `params={}` for parameterless sets and costs; `Distribution.params` and `sample(key, n, params)`; the contract test | byte-identical (a move) | low; `None` → `{}` touches every `getattr(set_, "params", None)` probe (`problems.py:208`, `evaluation.py:295`) |
| **M2 the names** | every caller | `compile_backend` → `backend`, `Optimizer.backend` → `adapter`, `dt_mpc` → `sample_period`, `bind` → `mount`, `plant.u` → `plant:u`, `x0_distribution` → `x_start` | byte-identical | low, one sweep per name, no alias |
| **M3 the stage body and the one score** | `stepping.py`, `planning/evaluator.py`, `planning/evaluation.py`, `shooting.py`, `environment.py`, `discretizer.py`, `dp.py` | `stage_body`, the scan with the score carry; `score_run` reads the scan's `J`; shooting's objective on it; the RL reward and the grid's running-cost table call it (Euler: unchanged) | the evaluator's and shooting's `J` move once (0.04 % RK4, 1 % Euler on the test loop); VI, RL unchanged | medium: the hybrid and trajopt regression gates re-pin; the showcase cells that print `J` |
| **M4 the start's owners** | `core/wiring.py` (`state_layout`, `refresh`), `simulation/simulator.py`, `planning/evaluator.py` | a diagram's `x0` a write-through view; `refresh` out of `Simulator`; the `x0` leaves in `pe.params` | byte-identical | medium: `refresh()` also bubbles solver hints; `WhiteNoise.refresh` (`sources.py:295`) and the racecar hint move to properties |
| **M5 step systems first-class** | `core/discrete/step_system.py`, `step_diagram.py`, `core/compile/plan.py`, `evaluators/discrete.py`, `core/operators.py`, `core/closed_loop.py`, `core/trajectory.py` | `sample_period`; the operators on step systems; native `rollout` and `rollout_batch`; `poles`; `StepRollout` → `Trajectory`; propagation within the tick as the one semantic | the step demos byte-identical (the step diagram already propagates within the tick, `discrete.py:188`) | low |
| **M6 the sampled block and the sampled-data loop** | `core/discrete/sampled.py` (new), `discretization.py` (moved), `hybrid_diagram.py`, `hybrid_composition.py`, `simulation/discrete/` | `%` → `sampled`; `@` → `SampledDataSystem`; the holds and the firing in the plan; `Computer`, `StepSchedule`, `HybridDiagram`, `HybridSimulator`, `HybridSimResult` go; the clock runner over the compiled blocks; the hybrid drawing reads the new class | the hybrid demos move once: the same-tick semantic (the `Computer`'s one-tick lag, `computer.py:114` reads the committed buffer) and the MPC delay, each its own commit with a closeness report | high: the largest rewrite; its oracle is part 2's identity (the pure scan against the clock runner, byte for byte on NumPy) |
| **M7 the draws** | `planning/problems.py`, `planning/evaluator.py`, `blocks/sources.py` | `disturbances` blocks wired on the port on both classes; `realize` one shape, per-block seeds by name, the cipher on both backends; `NoiseSource` | the stochastic demos' draws change once (the generator → the cipher); the RL numbers with the seeds (one re-pin) | medium: `Sys2Gym.from_problem` follows |
| **M8 the parametric program** | `optimization/mathematical_program.py`, `optimizer.py`, `evaluators/`, `direct_collocation.py`, `shooting.py`, `trajectory_optimization/planner.py` | `params` on the program; the evaluator body of `(z, p)`; collocation and shooting read `x_start`, `plant`, `cost`, the sets from `p`; `planner.program`, `solve(params, z0)`, `shift`, `plan`; the backend table for both paths | the MPC and trajopt baselines byte-identical with `x_start` alone bound; the performance gate measured first | medium: the `require_online_params` list (`planner.py:891`) and the closures of collocation |
| **M9 the native kernel** | `optimization/native/`, `optimizers/first_order.py`, `optimizer.py` | `GradientDescent`, `Adam` as kernels; `solve` the host run; `iterate` the scan; `as_step_system` | `gd` byte-identical (the same update), `adam` byte-identical when the clip is kept | low |
| **M10 the MPC block** | `mpc/controller.py`, `mpc/viz.py`, `mpc/utilities.py` | one `ModelPredictiveController(StepSystem)`; `iterations=K`; the latch, the stateless block, `Command`, the dual-rate export, the debug figure go; `parameter_ports` | the MPC demos move once (R123's delay), with M6 | medium: the GMC714 name is frozen, its constructor changes once with every caller (R50, R82) |
| **M11 the seeded policy** | `control/stochastic.py` (new), `learning/reinforcement_learning/policy.py`, `collect.py`, `algorithms/` | `GaussianPolicy`; the heads on it; per-episode seeds | the RL demos re-pin once | medium: the SAC head's squash and `log_prob` |
| **M12 fields** | `core/fields.py` (new), `planning/spatial/`, `analysis/lyapunov.py`, `dp.py`, `tabular.py`, `rl planner.py` | `Field`, `QuadraticField`, `GridField`, the shaping; `as_constraint` → `as_set`; `cost_to_go` a field | the RoA, DP and tabular baselines byte-identical | low |
| **M13 hardening and the merge** | as TODO §1.10 | S66, DESIGN, the notebooks, the re-pin at the merge | — | — |

**The order** (the three lanes, two joins): M1 → M2 (the names before any new signature, as
R134 asks) → then in parallel the loop lane M5 → M6, the objects lane M4 → M7 → M11, the program
lane M8 → M9; join 1 is M3 (it needs M5's `sample_period` and M6's loop to score the sampled
face, and it is what M8's shooting and M11's learners read, so it lands after M5 and M6 and
before the re-pins of M7, M8, M11); join 2 is M10 (needs M6, M8, M9); M12 any time after M1;
M13 last. Each intended number change is its own commit: the same-tick semantic (M6), the MPC
delay (M10), the one score (M3), the draws (M7), the RL seeds (M11).

**The risks that deserve a spike before the lane starts.**
1. *The hold slots in the step plan (M6).* The `StepExecutionPlan` grows a slot per slow
   block's output; the compiled evaluator's `step` must read the hold for a consumer of a slow
   block and write it on a fired tick. A two-day spike on `plan.py` with the multi-rate demo as
   its oracle.
2. *The dense trajectory from a scan (D3).* Recording the plant's substeps as the scan's
   per-tick output means `(substeps, n)` per tick; the `Trajectory` then interleaves them on
   `t`. Cheap; a shape question, not a design one.
3. *The performance gate of R130 (M8).* A `params` pytree argument on the compiled program
   must not slow the MPC's bind-and-solve; measure before the collocation closures go.
4. *`jax.checkpoint` on long sampled loops under `grad`.* A knob to add on `compile` when a
   demo needs it; not blocking.

---

## 10. What you verified

**Prototypes run.** `proto_kernel.py` (parts 1–4) and `proto_stochastic.py` (parts 5–6), in
the scratch folder named in the header; `python3 proto_kernel.py run1.json` twice, the two
JSON outputs equal with the timing fields left out (determinism); JAX 0.12 on CPU with
`jax_enable_x64`, NumPy 2.5. Every JAX path was run, not argued: `jax.jit`, `jax.grad`,
`jax.vmap` and `lax.scan` on each part, every gradient against a central difference.

- **Part 1, the one score.** The scan with `stage_body` on the catalog `Pendulum().f`
  (unchanged, it traces as is) under a state feedback, `dt = 0.01`, 200 steps, `ρ = 0.3`.
  Euler: the scan equals the left Riemann sum `Σ e^{−ρt_k} g_k dt` on the same samples to
  4·10⁻¹⁵ (value iteration's and the RL reward's number). RK4: the scan is 0.0055 from the
  fine-grid reference (dt/20), the trapezoid on the same samples 0.0144. JAX = NumPy to
  9·10⁻¹⁶. `grad` w.r.t. `K₀` 0.0982920429 vs 0.0982920428 by central difference; w.r.t.
  `Q₀₀` 3.50251653 vs 3.50251653. One jitted run 0.94 ms; 1 000 starts × 200 RK4 steps in one
  `vmap` 44 ms (482 ms with the compile).
- **Part 2, the sampled loop.** A digital PID (two states, its own Euler step at 20 Hz) around
  the pendulum at a 1 kHz base tick, 3 000 ticks, firing as a `where` on `k mod 50`, the held
  command a carry entry. The NumPy scan and a clock orchestrator (the controller run only on
  its ticks) are byte-identical; the JAX scan agrees to 1.8·10⁻¹⁵. `grad` of the closed-loop
  cost w.r.t. `Kp`: 0.20550836013 vs 0.20550836019. 100 starts × 3 000 ticks in 93 ms.
- **Part 3a, the kernel.** Gradient descent on `½ zᵀHz` with `η = 0.1`: the first step's ratio
  `[0.8, 0.95]`, the spectral radius of `I − ηH` 0.95 (the convergence rate is the stability
  margin of the step system); Adam to 7·10⁻⁶ in 200 steps.
- **Part 3b, the MPC on the kernel.** Single shooting over 20 steps of 0.05 s on the pendulum,
  `K = 15` projected-gradient iterations per tick (`η = 0.3`, `|u| ≤ 10`), the shifted plan as
  the warm start, the applied input read from the state (R123), 60 ticks in one `lax.scan`
  with `jax.grad` of the program inside. From `θ = π − 0.6` the loop holds `θ = 3.14276`
  (goal 3.14159), closed-loop `J = 0.665`, the torque cap hit. One jitted call 134 ms (807 ms
  with the compile); 32 starts in one `vmap` 477 ms. `grad` w.r.t. the terminal weight
  `S₀₀`: −0.0039954776 vs −0.0039954776; w.r.t. the step size `η`: −0.190896612 vs
  −0.190896612.
- **Part 4, the draws.** `child_seed(1, "trial i/load.seed")` streams and
  `standard_normal(seed, k, 1)` from `core/distributions.py` as they are: NumPy and JAX bit for
  bit (`0.6505030924…`), two trials distinct, 1 000 seeds × 4 samples in one `vmap`
  (mean 0.0096, std 1.0018).
- **Part 5, the seeded policy.** `u = −K(x − x̄) + σ ε(seed, k)` with `ε` the cipher's draw,
  256 trials from one key (the starts from a law, the seeds by name), one `vmap`: 14 ms a call.
  Mean `J` 13.385, std 0.533; `grad` w.r.t. `K₀` −2.88085970 vs −2.88085970, w.r.t.
  `log σ` 0.00642928 vs 0.00642928; the same seed replays the same run; 30 gradient steps
  bring the mean score to 0.219 (`K = [11.57, 2.28]`, `σ = 0.23`).
- **Part 6, today's evaluator against the one score.** `problem.compile(StateFeedback, 0.01,
  backend="jax").evaluate()` on the same loop (the real `ProblemEvaluator`, `planning/evaluator.py`)
  against the scan: Euler 5.9524 → 6.0091 (0.95 %), RK4 5.9615 → 5.9592 (0.04 %). These are
  the numbers that move once under D1.

**Facts checked in the code** (the file and line each claim rests on): `Computer.tick` reads
the committed buffer, one tick of latency (`simulation/discrete/computer.py:114–160`); the step
diagram propagates within the tick (`core/compile/evaluators/discrete.py:188–260`); the discrete
`rollout` returns host NumPy (`discrete.py:39–60`); `Discretization.step` binds `t_k = k·dt`
(`analysis/discretization.py:130–146`); the MPC's `step` solves through the latch and drops
`params` (`mpc/controller.py:504–527`); the four tree helpers (`planning/evaluator.py:991–1044`,
`planning/problems.py:896–913`); `score_run`'s trapezoid through `CostFunction.integrate`
(`planning/evaluation.py:200–206`, `core/costs.py:95–124`); the RL reward `−g dt`
(`environment.py:159–162`); value iteration's `g dt` table (`dp.py:390`); `WhiteNoise.h` as a
pure function of `(seed, ⌊t/Δ⌋)` (`blocks/sources.py:257–288`); `Adam.init/update` as the
kernel's shape (`optimizers/first_order.py:155–193`); `StochasticPlanningProblem.realize`'s
shape `(x0s, {"params", "disturbances"})` (`problems.py:638–681`); `require_continuous`
(`evaluator.py:789`); the `derived_x0` copy a diagram keeps (`core/wiring.py:263–283`,
`:711–759`).

**Honest list of what is not shown traced.** (a) The prototypes hand-write the loop bodies; the
library's compiled plan (`StepExecutionPlan` with hold slots and divisors) is the migration's
spike 1, not run here. (b) The MPC prototype uses `jax.grad` of the program inside the tick;
the library's exported program evaluator (`optimization/evaluators/jax_evaluator.py`) must be
callable inside a trace, which means its `J(z, p)` jitted body without host casts — today's
`numpy_evaluator` casts (`decision_vector`), the JAX one does not. (c) The adaptive plant solve
(`solver="scipy"`), a host-solver MPC and the realtime loop do not trace, by design (§7.5).
(d) `jax.checkpoint` was not needed at these sizes (60 ticks × 15 iterations × 20 steps under
`grad`); longer loops will want it. (e) The sampled loop prototype is single-controller
multi-rate (one slow block); a diagram with several slow blocks at different divisors follows
the same `where` per block and was not run.

**Prototype output, verbatim** (`run1.json` and `stoch1.json`):

```json
{
 "part1_one_score": {
  "euler": {
   "J_left_riemann_same_samples": 5.429759914738221,
   "J_scan_numpy": 5.429759914738225,
   "J_trapezoid_same_samples": 5.37285351118469
  },
  "rk4": {
   "J_left_riemann_same_samples": 5.412302438620648,
   "J_reference_fine_grid": 5.369792938762019,
   "J_scan_jax": 5.375313812053613,
   "J_scan_numpy": 5.375313812053614,
   "J_trapezoid_same_samples": 5.355370118871124,
   "gap_scan_to_reference": 0.00552087329159523,
   "gap_trapezoid_to_reference": 0.014422819890894445,
   "grad_K0_central_difference": 0.0982920427716749,
   "grad_K0_jax": 0.09829204293684986,
   "grad_Q00_central_difference": 3.5025165323165903,
   "grad_Q00_jax": 3.5025165321534275,
   "jax_minus_numpy": 8.881784197001252e-16,
   "jit_single_run_ms": {
    "first_call_with_compile": 525.106605000019,
    "second_call": 1.3980710000396357
   },
   "vmap_1000_starts_x_200_rk4_steps_ms": {
    "J_mean": 2464.6059946489668,
    "first_call_with_compile": 459.6043319999126,
    "second_call": 44.90795300000627
   }
  }
 },
 "part2_sampled_loop": {
  "grad_J_dKp_central_difference": 0.20550836018706775,
  "grad_J_dKp_jax": 0.2055083601254134,
  "jax_jit_scan_vs_numpy_max_abs_gap": 1.7763568394002505e-15,
  "numpy_pure_scan_vs_orchestrator_byte_identical": true,
  "theta_final": 0.7697546501362599,
  "vmap_100_starts_x_3000_base_ticks_ms": {
   "J_mean": 3.912076214509814,
   "first_call_with_compile": 450.87753600000724,
   "second_call": 76.35694800001147
  }
 },
 "part3a_kernel": {
  "adam_z_200": [
   -7.2180390245109585e-06,
   -7.218240233298873e-06
  ],
  "gd_ratio_z1_over_z0": [
   0.8,
   0.95
  ],
  "gd_z_50": [
   1.4272476927059596e-05,
   0.07694497527671332
  ],
  "spectral_radius_I_minus_etaH": 0.95
 },
 "part3b_mpc": {
  "J_closed_loop": 0.66472523954373,
  "dJ_dS00_central_difference": -0.003995477614815446,
  "dJ_dS00_jax": -0.003995477585476674,
  "dJ_deta_central_difference": -0.19089661172366765,
  "dJ_deta_jax": -0.1908966115979475,
  "jit_closed_loop_ms": {
   "first_call_with_compile": 806.8723579999642,
   "second_call": 133.96471699991253
  },
  "max_abs_u": 10.0,
  "sizes": {
   "horizon_steps": 20,
   "iterations_per_tick": 15,
   "ticks": 60
  },
  "theta_final": 3.142762021438508,
  "theta_goal": 3.141592653589793,
  "vmap_32_starts_ms": {
   "J_mean": 0.21667769526675215,
   "first_call_with_compile": 1166.4103230000364,
   "second_call": 476.5332709999939
  }
 },
 "part4_draws": {
  "numpy_equals_jax": true,
  "seeds": {
   "trial 0/load.seed": 705854612,
   "trial 0/plant.m": 2016448965,
   "trial 1/load.seed": 953889836,
   "trial 1/plant.m": 2012449392
  },
  "trial0_noise_jax": [
   0.650503092411472,
   -0.3879379643600874,
   -0.5718156006112284,
   -0.2660270754296082
  ],
  "trial0_noise_numpy": [
   0.650503092411472,
   -0.3879379643600874,
   -0.5718156006112284,
   -0.2660270754296082
  ],
  "trial1_noise_numpy": [
   0.041976454338038476,
   -1.4927805989540208,
   -0.5425213652345646,
   0.04705608231434868
  ],
  "vmap_1000_seeds_mean_std": [
   0.009585167590518104,
   1.0018035062341835
  ]
 }
}```

```json
{
 "part5_seeded_policy": {
  "after_30_gradient_steps": {
   "K": [
    [
     11.57358773686213,
     2.2806831979221043
    ]
   ],
   "mean_J": 0.21871400283563136,
   "std": 0.23239204243380962
  },
  "dJ_dK0_central_difference": -2.8808596991591884,
  "dJ_dK0_jax": -2.8808596984301276,
  "dJ_dlogstd_central_difference": 0.006429284038489413,
  "dJ_dlogstd_jax": 0.006429284028469816,
  "log_prob_sample": -0.11351559400918743,
  "mean_J_over_256_seeds": 13.385204178426166,
  "replay_same_seed_identical": true,
  "std_J": 0.532913472853208,
  "vmap_256_trials_ms": {
   "first_call_with_compile": 886.19901200002,
   "second_call": 13.68186800004878
  }
 },
 "part6_today_vs_one_score": {
  "euler": {
   "one_score_scan_J": 6.009085709314871,
   "relative_move": 0.009515437377268302,
   "today_evaluate_J_trapezoid": 5.952445586098751
  },
  "rk4": {
   "one_score_scan_J": 5.959205614244063,
   "relative_move": 0.00038497408531672304,
   "today_evaluate_J_trapezoid": 5.961500637499099
  }
 }
}```

---

## 11. Open questions you could not settle

Each with a recommendation; none blocks a lane from starting.

1. **The name of the sampled-data loop.** `SampledDataSystem` (the textbook's term; this
   proposal) against `HybridDiagram` kept to the freeze (R127). Recommendation: rename now,
   since the hybrid notebooks are rewritten by M6 anyway and R82 forbids an alias later.
2. **The dense plant trajectory of a sampled loop's `compute_trajectory()`.** Record the
   substeps in the scan (this proposal, D3) or reconstruct them after the run (sampled-path
   §8.4's option C). Recommendation: record by default, reconstruct for `evaluate(record=True)`
   and under `grad`; the two give the same numbers.
3. **`as_cost(terminal=True)` or a second verb.** One conversion with a flag (this proposal)
   against `as_terminal_cost()`. Recommendation: the flag, since both return a `CostFunction`
   (R133's "names by what they return").
4. **Where the stage cost of a native `StepSystem` problem is read when the cost reads `t`.**
   `t_k = k Δ` when the system has a period, `t = k` otherwise (this proposal). An alternative
   refuses a time-reading cost on a tickless system. Recommendation: bind `t = k`; it is what
   a grid-world cost means.
5. **The margins between samples.** Checked at the samples (this proposal); the least margin
   over the substeps is a later option on `stage_body` (a `minimum` in its loop) when a demo
   shows a violation hidden inside a period.
6. **The failure price under cost overrides (TODO §1.7's note).** Recommendation: compute the
   problem's price once at compile from the problem's own weights (reviewer A); a tuned weight
   that lowers the price below a feasible run is the user's declared `infeasible_cost` to raise.
7. **`GaussianPolicy` inside the RL algorithms: with the block (M11) or in v0.4 (C5).**
   Recommendation: the block and `log_prob` in v0.3, the algorithms' rebuild on it with C5,
   so the RL numbers move once, with the seeds.
8. **A `Computer` word for the lesson.** The class goes (D2), but "the computer" is how a
   student reads `ctl % Δ`. Recommendation: keep the word in the docstrings and the drawing's
   box label; no class.
