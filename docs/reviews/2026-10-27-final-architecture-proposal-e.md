# The final core architecture of minilink: Proposal E

**Date:** 2026-10-27
**Proposal:** E (one of several independent proposals written in parallel)
**Status:** an independent proposal, not a ruling. Nothing here is decided.
**Audience:** Prof. Alexandre Girard (Maintainer)
**Commit reviewed:** `f634e47eb04db70d0089b46c4e043f334eb8bdc7` on `dev-v0.3`
**Prototype:** `/tmp/claude-0/-home-user-minilink/36350854-ab7e-51d8-8262-d68717b27e02/scratchpad/arch-E/`
(`proto_core.py` and five runnable scripts; their output is in §10). Every JAX path quoted in
this document was run, with JAX 0.12 and SciPy 1.18 installed in this container.

Vocabulary: a *workstream* is one of the ten sections of `docs/plans/core-architecture.md`
(§1–§10); `R124`–`R140` are the maintainer's preliminary choices of 2026-10-27; a workboard id
(`S31`, `V1`) is a pointer into `docs/plans/TODO.md`, never the subject of a sentence.

## 2. The aim in one screen

Minilink is one toolbox on one model. A student writes a plant as `dx = f(x, u, t; p)`, closes
a controller on it with `@`, simulates, draws a Bode plot, places poles, and ten weeks later the
same plant, the same loop and the same syntax carry trajectory optimization, Monte Carlo over
draws, reinforcement learning and MPC, with `jax.grad` reaching every number from a gain to a
mass to a cost weight. The maintainer's direction of 2026-10-26 names the v0.3 target: **one
architecture from the plant to the evaluation and the optimizer**, supporting stochastic problems
and the hybrid sampled loop, built in the natural order of the code.

Reading the repository, that aim decomposes into five commitments the code must keep at once:

1. **One contract for every mathematical object** (R140). Its mathematics is a pure function of
   its specific arguments and a `params` tree; `params=None` reads the frozen default; leaves
   are numbers, arrays and integer seeds; composites nest their parts' trees by name; every
   function is deterministic given its parameters. This is what makes `jit`, `grad`, `vmap`
   and a parameter family reach anything.
2. **The continuous model is primary, the sampled world is its sibling.** `f` is a rate, never a
   next-step map. Discrete-time systems form a closed sibling algebra on an integer tick, and a
   continuous plant joins it through one adapter, `discretize`. The hybrid loop of a computer
   and a plant lives in the sibling algebra at its base clock (R127).
3. **One differentiable path to one score.** A problem (plant, sets, cost, horizon, draws), a
   controller, a sample period: compile once, roll out as one scan, score with one judge,
   export the tuning program over any leaves, solve. Every planner is a tool on that path and
   every policy it returns is a `System` the path can judge.
4. **Randomness is data.** A draw is a pure function of a seed and an index; a seed is a leaf of
   `params`; one integer key realizes a whole experiment, on either backend, to the same bits.
5. **Two audiences, one syntax.** The student never meets a trace, a key or a pytree; the
   researcher gets them for free, because the student's objects already obey points 1–4.

The spine, in the code's own names: `System` → `compile` → `rollout` → `Trajectory` → `score_run`
→ `Evaluation` → `MathematicalProgram` → `Optimizer`. The ten workstreams of the core plan are
the places where today's code still leaks: a start that is not a leaf, a cost integrated three
ways, a `Computer` that mutates, a program that binds only `x0`, a solver that loops on the host,
a scalar function of the state with five APIs. This proposal gives each its final shape and shows
on a prototype that the shape runs, traces and differentiates.

## 3. Adjustments to the vision

Each adjustment names its reason, its cost, and the preliminary choice it revises. Everything
not listed here is taken as the preliminary choices stand.

| # | Adjustment | Reason | Cost | Revises |
| --- | --- | --- | --- | --- |
| A1 | **A discrete problem is a noun, `StepPlanningProblem`, and `problem.discretize(dt, solver=, substeps=)` builds it.** The continuous `PlanningProblem` stays primary; the step problem is the sibling, as `StepSystem` is to `DynamicSystem`. Its cost is a per-step cost `c(x_k, u_k, k; p)` and a terminal `h(x_N, N; p)`; its sets read the tick; it carries `N`, `γ = e^{−ρ dt}`, `dt`, and the same draws. | The lowering (per-step cost, sets at the index, horizon, discount, failure rule) is consumed by six tools (the evaluator, value iteration, the RL environment, shooting, MPPI, the native MPC) and written three times today; a noun gives it one home and one test. A natively discrete task (a grid world, inventory control, a digital filter design) needs an object to state itself in, and has none. The precedent is exact: `discretize(plant, dt)` returns a `StepSystem`. | One class and two small wrappers (`StepCost`, the sets read at ticks) on the research surface; one DESIGN section. The student never writes it: every stepping tool keeps taking the continuous problem plus `dt`. | The open one-score investigation (QUESTIONS); reviewers A and B leaned to "no new noun". |
| A2 | **The per-step cost is the running cost integrated with the state by the same scheme**, `c_k = ∫_{t_k}^{t_{k+1}} e^{−ρ(τ−t_k)} g(x(τ), u_k, τ) dτ` by RK4 (or Euler) with the same substeps; `J = Σ γ^k c_k + γ^N h(x_N)`. The judge computes `c_k` **after** the rollout, as one `vmap` over the recorded samples `(x_k, u_k, k)`, never inside the sequential scan. | Exact to the scheme's order; under Euler it is today's `g·dt` of value iteration and RL, so their numbers do not move; the trapezoid of today's `score_run` becomes a plain sum. Computing `c_k` as a post-pass keeps the sequential rollout free of a second plant evaluation and lets the same judge score any held trajectory (`evaluate_trajectory`, a logged run). Verified: the state part of the augmented step is byte-identical to `step_body` (§10, run 1). | The evaluator's `J` moves once from the trapezoid (0.09 % on the prototype's PID run), re-pinned under R80. | The six points of differentiable-problem §9.4 (point 1: the augmented integral). |
| A3 | **Sets and costs bind `t_k = t₀ + k dt`; no discrete versions of the library objects.** A natively discrete cost is a `StepCost` subclass written directly; a natively discrete set is a `Set` whose `t` is read as the tick, the convention `StepSystem.h(x, u, k)` already uses. | One set of library objects; the sibling algebra already passes `k` in the `t` slot. | None. | Answers the maintainer's question of 2026-10-27. |
| A4 | **`block % dt` returns a `StepSystem` (`Sampled`), not a runtime.** Every port of the block survives, read at `t_k = k dt`; a block with continuous states is `discretize(block, dt)` (the digital PID of GRO501 is the continuous law discretized); a static block stays static. A step diagram whose blocks carry different `sample_period`s runs at their base period (the greatest common divisor), a slower block held between its firings (`k mod d = 0`, a `lax.cond` under a trace). `StepSchedule` and the double-buffered `Computer.tick` go; `Computer` becomes the name of the clock *runner* over the same pure blocks, for realtime and hardware in the loop only. | The pure tick of the multi-rate computer is then an ordinary `StepDiagramSystem`, built by the same assembly code as every diagram; the prototype's step-diagram runner matches a hand-written clock runner byte for byte and traces with `cond` (§10, run 2). Today's `Discretization` keeps only the standard `y` port, so a controller's `u` port is lost: a gap the prototype had to close. | `%` changes its return type (a `Computer` today); `StepSchedule.from_rates` becomes periods on blocks; the hybrid demos are rewritten (they already move for R125 and R123). | Refines R127 ("the computer's pure tick") and R125; retires `StepSchedule`. |
| A5 | **`compute_trajectory()` on the sampled loop runs the step-diagram runner by default, with the plant's substeps recorded**; `solver="scipy"` swaps the plant's integration between samples for `solve_ivp`, restarted where the held input changes. | One truth by default: the plot, `evaluate` and the gradient agree exactly, and the dense trajectory costs no contract change (the runner records the substeps as the scan's per-tick output; §10, run 2 point 3). The adaptive path stays as the cross-check and for stiff plants, the role `evaluate(solver="scipy")` plays on a continuous loop. | The hybrid numbers move once (they move for R123 and R125 anyway). The maintainer's alternative (orchestrator by default) stays one keyword away. | R127's default for `compute_trajectory` ("left to the implementation", adaptive). |
| A6 | **Keys are integers, everywhere.** `Distribution.sample(key, n=None, params=None)` draws with the library's counter-based cipher (`standard_normal(seed, k, p)`), so one key gives the same draws on NumPy and JAX; a JAX PRNG key and a NumPy generator leave the public convention. `problem.realize(key, n_trials)` derives one stream per consumer by name (`child_seed`), so adding a random block never moves another's draw. | Closes the first entry of KNOWN_ISSUES at the root; makes a trial a pure function of `(key, i)`, traceable under `vmap`. Verified on both backends to 3·10⁻¹⁸ (the bits agree; `cos`/`log` round differently; §10, run 4). | The seeded draws change once; the RL collectors keep `jax.random` internally (R140's written exception). | DESIGN §4's three key kinds; RD-R6. |
| A7 | **`GaussianPolicy(mean, *, log_std, sample_period, seed=None)` wraps any mean law that is a `System`** (`StateFeedback`, `NeuralPolicy`), copies its input ports, and adds the seeded noise; `log_prob(a, x, u, t, params)` and `entropy(x, u, t, params)` beside `ctl`. | A policy's mean is already a `System` in the library; wrapping it keeps one law per class and gives the seeded policy to every controller, not only the neural one. | None beyond R129's signature naming `mean` as a block. | Refines R129. |
| A8 | **`Field.as_cost(weight=, shaping=, terminal=False)`**: `terminal=True` makes the field the terminal cost `h`, the LQR value as an MPC terminal cost in one call. | R133 names `as_cost` for the running cost only; the pipeline's first consumer of a field is a terminal cost. | One keyword. | Refines R133. |
| A9 | **The planner interface the MPC needs: three verbs**, `prepare()` (compile once), `plan(x0, params=None, z0=None) -> PlanningSolution` whose record carries `z`, and `shift(z, k=1)` (the warm start). The MPC block is a `StepSystem` with state `z`, the one-period delay (R123), and `iterations=K` for the native fixed-iteration tick. | R135 asked for the verbs. Verified: the native tick is a pure `StepSystem` compiled by today's step-diagram compiler into one scan, `jit`, `vmap` over starts, `grad` through 80 ticks × 15 Adam iterations (§10, run 3). | None. | Fills R135. |
| A10 | **`Optimizer.solve(z0=, params=, iterations=)`**: `params` rebinds the program's tree, `iterations=K` runs a native kernel for a fixed count as one `lax.scan`; the kernels are `init(z0, params) → state`, `step(state, params) → state`; `as_step_system(optimizer)` is the lesson's view. | R130 and R132 as written; `iterations` is the one knob that turns a tolerance solve into a traced one. Verified: scan and loop byte-identical, `vmap` over seeds, the solver's pole at its fixed point (§10, run 5). | None. | Fills R130/R132's "how the solve takes the values". |
| A11 | **The evaluator's tree carries each stateful block's `x0` leaf** and the problem's `x_start` (R124), and the one `core/params.py` module owns lookup, merge, pack and unpack. | As ruled; stated here so the trees in §5 are complete. | None. | None. |

What this proposal does **not** change: the composition grammar (`@`, `>>`, `+`, `%`,
`connect`, `feedback`), the port-name roles, the three import layers, the two lanes, the name
`HybridDiagram` (R127; `SampledDataSystem` stays the candidate for the freeze), the price
switch, the criterion, `PlanningSolution` (R138), and the textbook-style rules.

## 4. The user's view: the main objects and verbs

| Object | In the textbook | Verbs (the student's, then the researcher's) | Its `params` |
| --- | --- | --- | --- |
| `DynamicSystem` | `ẋ = f(x, u, t; p)`, `y = h(x, u, t; p)` | `f`, `h`, `tf`; `compute_trajectory(tf, dt=, solver=)`, `plot_trajectory`, `animate`, `linearize`, `plot_bode`; `compile(backend=)` → `f`, `step`, `rollout`, `rollout_batch`, `jacobian` | its own numbers (`{"m", "l", "g"}`) |
| `System` (static) | `y = h(u, t; p)` | the same, no `f` | its numbers |
| `StepSystem` | `x_{k+1} = F(x_k, u_k, k; p)`, `y_k = h(x_k, u_k, k; p)` | `step`, `h`; `compute_rollout(n_steps)` (a `Trajectory`, `t = k dt`); `compile` → `step`, `rollout`, `jacobian` | its numbers |
| `DiagramSystem` / `StepDiagramSystem` | a block diagram, one algebra each | `@`, `>>`, `+`, `connect`, `expose`, `feedback`; `x0` a view of its blocks (write-through) | `{block_id: block.params}` |
| `block % dt` → `Sampled` | the law sampled at `Δ` (ZOH); a digital filter | a `StepSystem`: every port of the block, `sample_period` | the block's |
| `computer @ plant` → `HybridDiagram` | the sampled-data loop at its base clock | a `StepDiagramSystem`: `compute_rollout`, `compute_trajectory(tf, solver=)` (dense plant), `plot_diagram` (two sides), `run(runner=Computer)` for realtime | `{"controller": …, "plant": …}` |
| `discretize(sys, dt, solver=, substeps=)` | `x_{k+1} = Φ(x_k, u_k)` | the one adapter from the continuous to the step algebra | the source's, live |
| `Trajectory` | `(t, x, u)` sampled | `of("plant")`, `with_cost`, `plot`, `resample`; a pytree | none |
| `Set`, `InputSet` | `z ∈ Z(t)` as `margin ≥ 0` | `margin(z, t, params)`, `contains`, `sample(key, n, params)`, `bounding_box`, `&`, `as_cost(weight, shaping)` | the set's numbers (`{}` for none) |
| `Distribution` | `z ~ p` | `sample(key, n, params)`, `mean`, `support` | `{"mean", "std"}` / `{"lower", "upper"}` |
| source blocks (`Step`, `Sine`, `WhiteNoise`, `TrajectorySource`) | a signal of `t` | `h(x, u, t, params)`; a random one `realize(key)` | levels, `psd`, `sample_period`, `seed`, `values` |
| `CostFunction` | `g(x, u, t; p)`, `h(x, t; p)` | `+`, `*`, `g_max`, `h_max` | `{"Q", "R", "S", "xbar", "ubar"}`; a sum by term id |
| `Field` | `V(x, u, t; p)` | `value`, `gradient`, `as_set(lower, upper)`, `as_input_set`, `as_cost(weight, shaping, terminal)` | `{"center", "P"}` |
| `PlanningProblem` | the task: plant, `X`, `U`, `X0`, `Xf`, cost, `tf`, `ρ`, the price | `compile(controller, dt)`, `discretize(dt)`, `realize(key, n)`, `replace`, `cost_bound` | a view: `{"plant", "cost", "X", "U", "Xf", "x_start", "discount_rate"}` |
| `StochasticPlanningProblem` | the same with laws on the start and the parameters, sources on the ports | the same; `nominal()` | the same, plus the laws' numbers and the sources' seeds |
| `StepPlanningProblem` | the task on a tick: `Φ`, `c_k`, sets at `k`, `N`, `γ` | the stepping tools' input; built by `discretize` or directly | the same shape |
| `ProblemEvaluator` (`pe`) | the closed loop judged: `J(θ)` | `params` (the tree `θ`), `trajectory(params, key)`, `evaluate(params, key, n_trials)` → `Evaluation`, `program(wrt, …)` | `θ`: the loop's blocks (each stateful block with `x0`), the problem's leaves, the free inputs |
| `Evaluation` | the scores, one row per run | `objective`, `mean`, `std`, `worst`, `cvar`, the rates; `print` | none |
| `MathematicalProgram` | `min J(z; p)` s.t. `h = 0`, `g ≥ 0`, box | `J`, `h`, `g` of `(z, params=None)`; `params_at(z)` on an exported program | the problem's tree, the `seed` among them |
| `Optimizer` | the solve | `solve(z0=, params=, iterations=)` → `OptimizationResult`; wrapped (`scipy_slsqp`, `ipopt`, `l-bfgs`) or native (`gd`, `adam`) | `{"eta", …}` of a native kernel |
| `Planner` family | a method | `solve()` → `PlanningSolution`; online: `prepare()`, `plan(x0, params, z0)`, `shift(z)` | — |
| `PlanningSolution` | the policy and its evidence | `policy` (a `System`), `trajectory`, `evaluation`, `cost_to_go` (a `Field`), `solver`; `plot_control_law`, `plot_cost_to_go`, `plot_trajectory` | — |
| `ModelPredictiveController` | the receding horizon | a `StepSystem` with state `z`; `% sample_period @ plant` | its planner's problem tree (`cost`, `plant`, `terminal`), `eta` for a native kernel |
| `GaussianPolicy` | `u = μ(x) + σ ε` | `ctl`, `log_prob`, `entropy`; `realize(key)` | `{"mean": …, "log_std", "sample_period", "seed"}` |

## 5. The pipelines, as user scripts

Every script below is written in the final API of §6. "Prints" is what `print(obj)` shows
through the objects' own `__str__`; "plots" are the native plot verbs; "the tree" is the
`params` the user sees at that point. A pipeline grows from the one before it: nothing is
rewritten when the student becomes a researcher.

### 5.1 Modelling and simulation (the first lesson)

```python
import numpy as np
from minilink import DynamicSystem, Pendulum, Step
from minilink.core.backends import array_module


class Tank(DynamicSystem):
    """ḣ = (q_in − a √(2 g h)) / A: a tank drained by an orifice."""

    def __init__(self):
        super().__init__(n=1, input_dim=1, output_dim=1)
        self.params = {"A": 0.5, "a": 0.01, "g": 9.81}
        self.x0 = np.array([0.2])

    def f(self, x, u, t=0.0, params=None):
        params = self.params if params is None else params
        xp = array_module(x)

        A = params["A"]
        a = params["a"]
        g = params["g"]

        h = x[0]
        q_in = u[0]

        # A ḣ = q_in − a √(2 g h)
        dh = (q_in - a * xp.sqrt(2.0 * g * xp.maximum(h, 0.0))) / A
        dx = xp.array([dh])

        return dx


tank = Tank()
print(tank)                                   # Tank, n=1, ports u(1) → y(1); params A, a, g
traj = tank.compute_trajectory(tf=60.0)       # adaptive solver on the fine grid, u at its nominal
tank.plot_trajectory()                        # h(t) and u(t)

loop = Step(final_value=[0.02], port="u") >> tank     # a source drives the inflow
loop.plot_diagram()
loop.compute_trajectory(tf=60.0, dt=0.1)
loop.plot_trajectory()

plant = Pendulum()
plant.x0 = np.array([0.5, 0.0])
plant.compute_trajectory(tf=10.0)
plant.plot_phase_plane()
plant.animate()
```

Prints: `Tank`'s one-screen summary. Plots: the time signals, the diagram, the phase plane,
the animation. The tree: `tank.params == {"A": 0.5, "a": 0.01, "g": 9.81}`,
`loop.params == {"Step": {"initial_value", "final_value", "step_time"}, "plant": {...}}`.
Nothing of JAX, keys or problems appears; `x0` lives on the block (R124).

### 5.2 Classical analysis, then the digital implementation

```python
import numpy as np
from minilink import PID, Pendulum, bode, margins, step_info, step_response

plant = Pendulum()
pid = PID(Kp=12.0, Ki=4.0, Kd=2.0)            # ports="error": PID @ plant inserts e = r − y
loop = pid @ plant                             # the continuous loop, roles controller / plant

bode(pid >> plant)                             # the loop gain L = C G at the operating point
print(margins(pid >> plant))                   # gain and phase margins, the crossovers
print(step_info(loop))                         # rise time, overshoot, settling, final error
loop.plot_step_response()

# The digital implementation: the same law sampled at 10 ms, the loop at its base clock
digital = pid % 0.01 @ plant                   # a HybridDiagram: a StepDiagramSystem at dt = 0.01
digital.plot_diagram()                         # the computer side and the plant side
digital.compute_trajectory(tf=5.0)             # dense plant trajectory (the substeps recorded)
digital.plot_trajectory(signals=("plant:y", "controller:u"))

print(digital.poles())                         # the sampled-data loop's poles, against the unit circle
loop.compute_trajectory(tf=5.0)
loop.plot_trajectory(compare=digital.traj)     # the continuous and the sampled loop overlaid
```

Prints: `margins` (one line each), `step_info`'s table, the discrete poles. Plots: Bode, the
step response, the two-sided diagram, the overlay. The tree: `digital.params ==
{"controller": {"Kp", "Ki", "Kd", "tau", "u_min", "u_max", "e_int_min", "e_int_max"},
"plant": {"m", "l", "I", "gravity", "d"}}`, the same tree as `loop.params`, because
`pid % 0.01` reads the PID's parameters live. `digital.subsystems["controller"]` is
`discretize(pid, 0.01)` with the `u` port kept (adjustment A4). GRO501's "designed in
continuous time, discretized, validated by simulation" is three lines.

### 5.3 State-space design with an observer, then Monte Carlo over the draws

```python
import numpy as np
from minilink import (CartPole, Gaussian, StochasticPlanningProblem, QuadraticCost, Uniform,
                      WhiteNoise, closed_loop, evaluate, kalman, lqr)

plant = CartPole()
up = np.array([0.0, np.pi, 0.0, 0.0])
K = lqr(plant, Q=np.diag([1, 10, 1, 1]), R=0.1 * np.eye(1), x_bar=up)     # a StateFeedback block
obs = kalman(plant, Q_n=0.01 * np.eye(4), R_n=0.001 * np.eye(2), x_bar=up)  # a StateObserver block
loop = closed_loop(K, plant, observer=obs)     # by role: y → observer → x̂ → K → u → plant
loop.plot_diagram()

problem = StochasticPlanningProblem(
    plant,
    cost=QuadraticCost.from_system(plant, Q=np.diag([1, 10, 1, 1]), R=0.1 * np.eye(1), xbar=up),
    tf=5.0,
    x0_distribution=Gaussian(up + [0, 0.1, 0, 0], std=[0.05, 0.05, 0.05, 0.05]),
    params={"plant": {"m_p": Uniform(0.08, 0.12)}},                 # a law on a parameter
    disturbances={"d": WhiteNoise(1, psd=0.02, sample_period=0.01), # source blocks on the ports
                  "n": WhiteNoise(2, psd=1e-5, sample_period=0.01)},
    X=plant.state.box, U=plant.input_box, infeasible_cost=True,
)
report = evaluate(problem, loop, dt=0.01, n_trials=200, key=1)
print(report)                                  # mean, std, worst, CVaR, feasibility and success rates
report.plot_runs(signals=("plant:x",))         # the 200 runs overlaid, the mean highlighted
problem.compile(loop, dt=0.01).trajectory(key=3).of("plant")   # one trial, replayed exactly
```

Prints: the `Evaluation` table. Plots: the loop with the observer inside, the fan of runs.
The tree, `problem.compile(loop, 0.01).params`:

```text
{"controller": {"K": (1, 4), "xbar": (4,), "ubar": (1,)},
 "observer":   {"L": (4, 2), "A": …, "B": …, "C": …, "xbar": …, "ubar": …, "x0": (4,)},
 "plant":      {"m_c", "m_p", "l", "gravity", "d"},
 "d":          {"psd": (1,), "sample_period": (), "seed": ()},   # the sources the shortcut wired
 "n":          {"psd": (2,), "sample_period": (), "seed": ()},
 "cost":       {"Q", "R", "S", "xbar", "ubar"},
 "X": {}, "U": {}, "x_start": (4,), "discount_rate": ()}
```

`problem.realize(key=1, n_trials=200)` returns the 200 starts and the batched tree: `plant.m_p`
with a leading axis of 200, `d.seed` and `n.seed` with 200 integers, each stream named by its
path (adjustment A6). The observer's start `observer.x0` is a leaf (R124), tunable later.

### 5.4 The digital sampled loop and its gradient

```python
import jax, numpy as np
from minilink import Optimizer, PID, Pendulum, PlanningProblem, QuadraticCost

plant = Pendulum()
cost = QuadraticCost.from_system(plant, Q=np.diag([1.0, 0.1]), R=0.01 * np.eye(1), xbar=[1.0, 0.0])
problem = PlanningProblem(plant, cost=cost, tf=4.0, x_start=[0.0, 0.0], X=plant.state.box)

pid = PID(Kp=8.0, Ki=2.0, Kd=1.0, ports="reference")
pe = problem.compile(pid % 0.02)               # the sampled loop at its own period; no dt needed
print(pe.evaluate())                           # one run, J = Σ γ^k c_k + γ^N h(x_N), the margins
dJ = jax.grad(lambda p: pe.evaluate(p).objective)(pe.params)
print(dJ["controller"]["Kp"], dJ["plant"]["m"])          # every leaf, the plant's mass included

program = pe.program(wrt=["controller.Kp", "controller.Ki", "controller.Kd"])
best = Optimizer(program, method="l-bfgs").solve()
tuned = program.params_at(best.z)              # the full tree with the tuned leaves written in
print(pe.evaluate(tuned))
pe.trajectory(tuned).plot(signals=("plant:y", "controller:u"))
```

Prints: two `Evaluation`s, the gradient leaves, `OptimizationResult`'s line. Plots: the tuned
run. The tree: `pe.params` is `{"controller": {... PID gains ..., "x0": (2,)}, "plant": {...},
"cost": {...}, "X": {}, "x_start": (2,), "discount_rate": (), "r": (1,)}` — the free reference
input `r` held at its nominal is a leaf too. The prototype of §10 (run 1) is this script on
today's step-diagram compiler: `jit` 2 ms per run of 200 ticks, `grad` equal to central
differences to six digits, `vmap` over 1000 starts in 0.2 s.

### 5.5 Trajectory optimization and MPC, the program rebound without recompiling

```python
import numpy as np
from minilink import CartPole, PlanningProblem, QuadraticCost, SingletonSet, TrajectoryOptimizationPlanner
from minilink.control import ModelPredictiveController

plant = CartPole()
up = np.array([0.0, np.pi, 0.0, 0.0])
problem = PlanningProblem(plant, cost=QuadraticCost.from_system(plant, Q=np.eye(4), R=0.1 * np.eye(1), xbar=up),
                          tf=2.0, x_start=np.zeros(4), Xf=SingletonSet(up), U=plant.input_box)
planner = TrajectoryOptimizationPlanner(problem, n_steps=40, transcription="direct_collocation",
                                        backend="jax", method="ipopt")
solution = planner.solve()                     # offline: a fresh program, Ipopt
print(solution)                                # success, cost, the residuals, the solve time
solution.plot_trajectory()
(solution.policy >> plant).animate()           # the plan replayed open loop

# The program with every problem parameter as an argument: compiled once, rebound per solve
planner.prepare()                              # transcribe once: J(z, p), h(z, p), g(z, p)
plan_a = planner.plan(x0=np.zeros(4))
plan_b = planner.plan(x0=np.zeros(4), params={"plant": {"m_p": 0.15}, "cost": {"Q": 2 * np.eye(4)}})
print(plan_a.solver.solve_time_s, plan_b.solver.solve_time_s)     # no retrace between the two

mpc = ModelPredictiveController(planner, sample_period=0.05, parameter_ports=["plant.m_p"])
loop = mpc @ plant                             # a HybridDiagram at 0.05 s; the plant RK4 inside
loop.plot_diagram()
loop.compute_trajectory(tf=5.0)                # the host solver per tick (Ipopt), the NumPy scan
loop.plot_trajectory(signals=("plant:x", "controller:u"))
loop.animate(overlays=mpc.horizons(loop.traj))
```

Prints: the solution's record, two solve times. Plots: the plan, the animation, the closed
loop with its horizons. The tree: `planner.program.params == {"x_start": (4,), "plant": {...},
"cost": {...}, "Xf": {"point": (4,)}, "U": {}}`; `loop.params["controller"]` is the same tree
plus the MPC's own `x0` (the packed `z` at reset). A value on the port `m_p` is forwarded to the
tick's `params`; an unwired port reads the nominal. `method="ipopt"` on the compiled path is
one keyword, because the program evaluator has one body for the frozen and the parametric
program (R130; §6.7).

### 5.6 The native fixed-iteration MPC, tuned by gradient

```python
import jax, numpy as np
from minilink import LQRPlanner, Optimizer, Pendulum, PlanningProblem, QuadraticCost
from minilink.control import ModelPredictiveController
from minilink.planning import ShootingPlanner

plant = Pendulum()
up = np.array([np.pi, 0.0])
V = LQRPlanner(PlanningProblem(plant, cost=QuadraticCost.from_system(plant, Q=np.diag([10, 1]), R=0.1 * np.eye(1), xbar=up), tf=np.inf)).solve().cost_to_go
cost = QuadraticCost.from_system(plant, Q=np.diag([10, 1]), R=0.1 * np.eye(1), xbar=up) + V.as_cost(terminal=True)
horizon = PlanningProblem(plant, cost=cost, tf=0.6, x_start=up + [0.4, 0], U=plant.input_box)

planner = ShootingPlanner(horizon, n_steps=12, backend="jax")         # single shooting, box-bounded z = u_{0:H}
mpc = ModelPredictiveController(planner, sample_period=0.05, iterations=15, method="adam", eta=0.5)
loop = mpc @ plant                             # every block pure: the tick is 15 Adam steps, one scan

task = PlanningProblem(plant, cost=QuadraticCost.from_system(plant, Q=np.diag([10, 1]), R=1.0 * np.eye(1), xbar=up),
                       tf=4.0, x_start=up + [0.4, 0])
pe = task.compile(loop)                        # the loop at its base clock, one scan of 80 ticks
print(pe.evaluate())
starts = up + np.array([[0.4, 0], [-0.4, 0], [0.2, 0.5], [-0.3, -0.5]])
print(jax.vmap(lambda x0: pe.evaluate({"x_start": x0}).objective)(starts))

program = pe.program(wrt=["controller.cost.R", "controller.eta"], key=0)
tuned = Optimizer(program, method="adam", eta=0.05).solve(iterations=20)
print(program.params_at(tuned.z)["controller"]["cost"]["R"])
```

Prints: the closed-loop evaluation, the batch of four scores, the tuned effort weight. The
tree: `pe.params["controller"] == {"plant": {...the MPC's model...}, "cost": {"Q", "R", "S",
"xbar", "ubar", "V": {"center", "P"}}, "eta": (), "x0": (12,)}` beside `"plant"` (the real
plant) and the task's leaves. Two trees for one plant is the model-mismatch study for free:
override `plant.m` and the MPC still believes its own. The prototype of §10 (run 3) is this
script's closed loop on today's compiler: 112 ms per 80-tick run with 15 Adam iterations per
tick, `vmap` over starts, and the gradient on `log R` driving the judge's `J` from 24.62 to
24.19 in seven steps.

### 5.7 Value iteration and reinforcement learning on one `J`, a field as the terminal cost

```python
import numpy as np
from minilink import (DynamicProgrammingPlanner, LQRPlanner, Pendulum, PlanningProblem, QuadraticCost,
                      ReinforcementLearningPlanner, StochasticPlanningProblem, Uniform, compare)

plant = Pendulum()
up = np.array([np.pi, 0.0])
plant.inputs["u"].lower_bound, plant.inputs["u"].upper_bound = [-4.0], [4.0]
quad = QuadraticCost.from_system(plant, Q=np.diag([1, 0.1]), R=0.01 * np.eye(1), xbar=up)
V = LQRPlanner(PlanningProblem(plant, cost=quad, tf=np.inf)).solve().cost_to_go   # a QuadraticField
problem = StochasticPlanningProblem(plant, cost=quad + V.as_cost(terminal=True), tf=6.0,
                                    x0_distribution=Uniform([-np.pi, -1], [np.pi, 1]),
                                    X=plant.state.box, U=plant.input_box, infeasible_cost=True)

vi = DynamicProgrammingPlanner(problem, x_grid=(201, 201), u_grid=(21,), dt=0.05).solve()
rl = ReinforcementLearningPlanner(problem, dt=0.05, algorithm="ppo").solve(timesteps=120_000)
lqr = LQRPlanner(problem.nominal()).solve()

vi.plot_cost_to_go()                           # a GridField
vi.plot_control_law()
rl.plot_control_law()
report = compare(problem, VI=vi, PPO=rl, LQR=lqr, dt=0.05, n_trials=100, key=1)
print(report)                                  # one table: the three policies on the same draws
report.plot_control_law()
```

Prints: `compare`'s table. Plots: the cost-to-go, the three laws on one scale. What makes
the table one `J`: value iteration, the RL environment and `compare` all read
`problem.discretize(0.05)`; the per-step cost `c_k`, the sets at the ticks, `N = 120`,
`γ = e^{−ρ·0.05}` and the failure price are the step problem's, read by three tools and
written once (adjustments A1–A3). `solution.cost_to_go` is a `Field` on every planner
(`GridField` for the tables, `QuadraticField` for LQR, `CallableField` for the critic), so
`vi.cost_to_go.as_cost(terminal=True)` can seed the next problem.

### 5.8 A stochastic sampled loop with a seeded policy, vmapped over seeds, tuned by gradient

```python
import jax, numpy as np
from minilink import (GaussianPolicy, NeuralPolicy, Optimizer, Pendulum, StochasticPlanningProblem,
                      QuadraticCost, Uniform, Gaussian)

plant = Pendulum()
problem = StochasticPlanningProblem(plant, cost=QuadraticCost.from_system(plant, Q=np.eye(2), R=0.1 * np.eye(1)),
                                    tf=3.0, x0_distribution=Gaussian([0.5, 0.0], std=[0.2, 0.1]),
                                    params={"plant": {"m": Uniform(0.8, 1.2)}})

policy = GaussianPolicy(NeuralPolicy(plant, hidden=(16, 16)), log_std=np.log(0.3), sample_period=0.05)
loop = policy % 0.05 @ plant                   # the policy's noise is a signal of its seed and tick
pe = problem.compile(loop)
report = pe.evaluate(key=7, n_trials=256)      # 256 (start, mass, seed) triples, one vmap
print(report)
dJ = jax.grad(lambda p: pe.evaluate(p, key=7, n_trials=256).objective)(pe.params)   # the float leaves

program = pe.program(wrt=["controller.mean.mlp"], key=7, n_trials=256)   # the sample-average program
tuned = Optimizer(program, method="adam", eta=1e-3).solve(iterations=300)
print(pe.evaluate(program.params_at(tuned.z), key=11, n_trials=256))      # scored on a held-out key
loop.subsystems["controller"].params["seed"] = None                          # the mean law, deterministic
loop.compute_trajectory(tf=3.0); loop.plot_trajectory()
```

Prints: two `Evaluation`s, before and after, the second on a held-out key. The tree:
`pe.params["controller"] == {"mean": {"mlp": {...layers...}}, "log_std": (1,),
"sample_period": (), "seed": ()}`; `seed` is the integer leaf `realize` overwrites per trial and
`program` never packs (R131). `log_prob(a, x, u, t, params)` on the block is what the RL
planner's policy-gradient loss reads, so PPO's head and this block are one object. The
prototype of §10 (run 4) is this script with a `StateFeedback` mean: 64 trials in one `vmap`,
the gradient of the mean `J` with respect to the gain, five descent steps from 1.87 to 1.18.

### 5.9 A natively discrete task (the sibling algebra on its own)

```python
import numpy as np
from minilink import StepSystem, StepPlanningProblem, StepCost, BoxSet, DynamicProgrammingPlanner


class Inventory(StepSystem):
    """s_{k+1} = s_k + u_k − d_k: stock, order, demand."""

    def __init__(self):
        super().__init__(n=1, input_dim=1, output_dim=1)
        self.params = {"d": 2.0}

    def step(self, x, u, k=0, params=None):
        params = self.params if params is None else params
        d = params["d"]
        x_next = x + u - d
        return x_next


class HoldingCost(StepCost):
    def c(self, x, u, k=0, params=None):      # c_k = holding + ordering
        return 0.1 * x[0] ** 2 + 0.5 * u[0]

    def h(self, x, k=0, params=None):
        return 0.0


problem = StepPlanningProblem(Inventory(), cost=HoldingCost(), N=52, gamma=0.98,
                              X=BoxSet([0.0], [50.0]), U=BoxSet([0.0], [10.0]), x_start=[10.0])
vi = DynamicProgrammingPlanner(problem, x_grid=(51,), u_grid=(11,)).solve()   # no dt: the problem ticks
vi.plot_cost_to_go()
```

No `dt` anywhere: the tools that step take a step problem as they are, and a continuous
problem reaches them through `discretize`.

## 6. The final API, object by object

Conventions used in every signature: `params=None` reads the object's frozen defaults; an
explicit tree is complete inside the library and may be partial only at a user boundary
(`pe.evaluate(params=…)`, `solve(params=…)`), where one merge happens (R121, R131); every
equation path is bare (no type hints) and backend-native; `key` is an integer; sizes (`n_steps`,
`iterations`, `n_trials`) are structural keywords, never leaves. "Today" cites the line the
signature grows from.

### 6.1 Systems, step systems, diagrams, operators, the sampled loop

```python
class System:                                   # core/system.py:26 (today the same shell)
    def __init__(self, n=0): ...                # x0, params = {}, ports, solver_info, name, id
    def h(self, x, u, t=0.0, params=None): ...  # y = h(x, u, t; p); today core/system.py:154
    def tf(self, x, u, t=0.0, params=None): ... # frames; today :394
    def realize(self, key): ...                 # the params of one realization; today :190
    def compile(self, backend="numpy"): ...     # → an evaluator (§6.4)
    def __add__ / __rshift__ / __matmul__       # +, >>, @ : today :409, :422, :435
    def __mod__(self, sample_period): ...       # block % dt → Sampled (A4); today :452 returns a Computer

class DynamicSystem(System):                    # core/system.py:516
    def f(self, x, u, t=0.0, params=None): ...  # ẋ; today :565

class StepSystem(System):                       # core/discrete/step_system.py:16
    sample_period = None                        # set by `%` or discretize; a bare StepSystem ticks
    def step(self, x, u, k=0, params=None): ... # x_{k+1}; today :62
    def h(self, x, u, k=0, params=None): ...    # y_k; today :87
    def compute_rollout(self, n_steps, u=None, *, x0=None, backend="numpy") -> Trajectory
                                                # t = k·sample_period (StepRollout folds in)
    def poles(self, x_bar=None, u_bar=None, k=0, params=None): ...   # eig(∂step/∂x) (R122)
```

`DiagramSystem` (`core/diagram.py:19`) and `StepDiagramSystem` (`core/discrete/step_diagram.py:13`)
keep their wiring API (`add_subsystem`, `connect`, `connect_new_output_port`, `expose`,
`autowire`, `feedback`). Three changes:

- **`x0`, `n`, `state` are views** of the blocks (R124): the getter stacks the blocks' `x0` on
  each read, the setter writes through by `state_index`; `derived_x0` and `refresh()`'s
  state part (`core/wiring.py:39`, `:258`) go.
- **`@`, `>>`, `+` work on the step algebra** (R126) through `core/assembly.py`, returning a
  `StepDiagramSystem`; a continuous operand beside a step operand is refused with the fix named
  (`%` or `discretize`).
- **A step diagram runs at its base period.** Each block carries a `sample_period` (`%`,
  `discretize`); the diagram's period is the greatest common divisor, each block's divisor
  `d_i = period_i / base`, and a block with `d_i > 1` is wrapped at compile in `Held` (its
  outputs part of the state, a `cond` on `k mod d_i`); `k` seen by the block is its own tick
  `k // d_i`. A bare `StepSystem` with no period ticks on the base. The same-tick semantic is
  propagation within the tick (R125); a cycle without a delay is refused at wiring.

```python
def discretize(sys, dt, *, solver="rk4", substeps=1) -> Sampled   # analysis/discretization.py:21
class Sampled(StepSystem):                      # today's Discretization (:50), every port kept
    source: System; sample_period: float; solver; substeps
    params -> source.params (live; an override replaces it)        # today :119
    def step(self, x, u, k=0, params=None): ... # step_body(f, …, k·dt, dt) ; today :130
    # every input port of the source; every output port read at t_k = k·dt (today only y: :148)

class HybridDiagram(StepDiagramSystem):         # the result of computer @ plant (R127)
    # blocks: the computer's step blocks, and `plant` = discretize(plant, base, substeps=n)
    plant_source: DynamicSystem                 # the continuous plant it was built from
    def compute_rollout(self, n_steps, ...) -> Trajectory    # the ticks: evaluate's semantics
    def compute_trajectory(self, tf, *, solver="rk4", substeps=None, backend="numpy", ...)
        # A5: the step runner with the plant's substeps recorded (dense); solver="scipy" integrates
        # the plant between the held-input changes with solve_ivp, restarted there
    def plot_diagram(self, ...)                 # the two-sided drawing (graphical/diagrams/hybrid_topology.py)
    def run(self, runner, ...)                  # the clock runner: realtime, hardware in the loop
```

`computer @ plant` resolves the standard ports as `ctl @ plant` does
(`core/closed_loop.py:238`, `hybrid_composition.py:154`), names the sides `controller` and
`plant`, picks `substeps` by the continuous rule (the largest divisor of the base period at or
under `AUTO_DT_SCALE × τ_min`, `simulation/time_grid.py`), and nests a multi-block computer
under `controller`. `HybridDiagram` today (`core/discrete/hybrid_diagram.py:30`) is a frozen
dataclass with a `Computer` and a plant diagram; it becomes this subclass. `Computer`
(`simulation/discrete/computer.py:34`) keeps its name as the clock runner: `Computer(loop)`,
`reset(x0)`, `tick(u) -> outputs`, over the same compiled pure blocks, with `StepSchedule`
(`:227`) and the read/write buffers (`:114`) gone; `HybridSimulator`
(`simulation/discrete/hybrid_simulator.py:24`) shrinks to the `solver="scipy"` path.

The params trees: a leaf's own; a diagram's `{block_id: block.params}` (today
`core/wiring.py:54`); `Sampled` and `Held` are transparent (the source's tree, no key of their
own); a `HybridDiagram`'s is `{"controller": …, "plant": …}` with every source the problem
wired as a sibling key.

### 6.2 Trajectories, sets, costs, distributions, sources, fields

```python
@dataclass(frozen=True)
class Trajectory:                               # core/trajectory.py:52, unchanged; a pytree
    t: (N,); x: (n, N); u: (m, N); signals: {name: (dim, N)}; roles: {"plant": {"x": (a, b), "u": "plant:u"}}
    def of(self, role) -> Trajectory            # :121
    def with_cost(self, cost, *, discount_rate=0.0, params=None)  # :111

class Set:                                      # core/sets.py:22
    params: dict = {}                           # every library set reads it (R140.2)
    def margin(self, z, t=0.0, params=None): ... # :41
    def contains(self, z, t=0.0, params=None): ...
    def sample(self, key, n=None, params=None): ...     # key an integer; the cipher (A6); today :61 takes jax keys
    def bounding_box(self): ...
    def as_cost(self, weight=1.0, shaping=None) -> CostFunction   # Field.as_cost on the margin (R133)

class InputSet:  margin(self, u, x=None, t=0.0, params=None)      # :90, unchanged

class CostFunction:                             # core/costs.py:29
    params: dict = {}
    def g(self, x, u, t=0.0, params=None): ...  # :58
    def h(self, x, t=0.0, params=None): ...     # :63
    def g_max(self, X, U, params=None) / h_max(self, X, params=None)        # :67, :84
    __add__ → SumCost (terms by id, :324), __mul__ → ScaledCost (:419)
    # `integrate` (:95, the trapezoid) leaves: the judge integrates (§6.5)

class Distribution:                             # core/distributions.py:24
    params: dict                                # {"mean", "std"} / {"lower", "upper"} / {"points"}
    dim; mean (a property on params); support: Set | None
    def sample(self, key, n=None, params=None): ...   # one integer key; :46 today takes a generator or a jax key

class Field:                                    # core/fields.py (new); today planning/spatial/state_fields.py:26
    params: dict = {}
    def value(self, x, u=None, t=0.0, params=None): ...     # :30
    def gradient(self, x, u=None, t=0.0, params=None): ...  # jax.grad on JAX, central differences on NumPy
    def as_set(self, lower=None, upper=None) -> Set          # today as_constraint, :33
    def as_input_set(self, lower=None, upper=None) -> InputSet
    def as_cost(self, weight=1.0, shaping=None, *, terminal=False) -> CostFunction   # :39, A8
    __call__ = value

class QuadraticField(Field):   params = {"center": (n,), "P": (n, n)}; as_set(upper=level) → EllipsoidSet
class GridField(Field):        params = {"values": table}; the DP and tabular tables (later, R133)
class CallableField(Field):    the critics (later)
```

Sources (`blocks/sources.py`): `Step`, `Sine`, `TrajectorySource` (its table a leaf
`values`, so `pe.program(wrt=["controller.values"])` tunes an open-loop plan), `WhiteNoise(p, *,
psd, sample_period, seed, hold)` (`:232`, unchanged: its `h` is the cipher at `⌊t/Δ⌋`, `:257`),
and `NoiseSource(distribution, *, sample_period, seed)` (its per-sample law any
`Distribution`, `WhiteNoise` its subclass). A random block has `is_random = True` and
`realize(key)` returns its params with a fresh `seed` (`:290`).

The draw convention (A6): `standard_normal(seed, k, p)` (`core/distributions.py:230`) and the
uniform words of `random_bits` (`:199`) are the only generators; `Gaussian.sample(key, n)` is
`mean + L @ ε(key, i)` for `i < n`, `Uniform.sample` the scaled uniform bits, `Particles` an
index from the bits; `BoxSet.sample` the same. `split_keys` (`:145`) and the generator path go.

### 6.3 The problem and its draws

```python
class PlanningProblem:                          # planning/problems.py:29
    def __init__(self, sys, x_start=None, x_goal=None, cost=None, X=None, U=None, X0=None, Xf=None,
                 tf=None, infeasible_cost=None, discount_rate=0.0, *, disturbances=None): ...   # :111; disturbances new (R128)
    params -> {"plant", "cost", "X", "U", "X0", "Xf", "x_start", "discount_rate", <source ids>}  # :198, a live view
    def realize(self, key, n_trials=1) -> (x0s, params)      # :246; one shape (R128)
    def compile(self, controller, dt=None, *, solver="rk4", substeps=1, backend="auto", episode_length=None) -> ProblemEvaluator   # :358
    def discretize(self, dt, *, solver="rk4", substeps=1) -> StepPlanningProblem   # new (A1)
    def cost_bound(self, tf=None, *, discount_rate=None, dt=None)   # :298, unchanged
    def replace(self, **fields)                 # :253
    def nominal(self) -> PlanningProblem

class StochasticPlanningProblem(PlanningProblem):   # :506
    def __init__(self, sys, ..., x0_distribution=None, params=None, disturbances=None, criterion="expectation")
        # `params={"plant": {"m": Uniform(0.9, 1.1)}}` is the laws on the tree (today params_distribution,
        # dotted names, :551); `disturbances={"d": WhiteNoise(...)}` wires each block on its port and is
        # refused a bare Distribution naming the fix (R128; today a law held for the run, :631)
    def realize(self, key, n_trials) -> (x0s, params)   # :638; the batched tree: each drawn leaf and
                                                        # each source's `seed` with a leading trial axis
```

`disturbances=` on either problem builds the diagram `sources >> plant` once at construction
(the sources under their port's name, the plant under `plant`), so `problem.sys` is that
diagram and `traj.of("plant")` still scores the plant; a user-built diagram with a `plant` block
is taken as is. `realize` draws by name: the start on `child_seed(key, "x_start")`, each law on
`child_seed(key, path)`, each random block's seed on `child_seed(key, block_path)`, all at trial
index `i` (`core/distributions.py:275`). A deterministic problem's `realize(key, 1)` returns its
one start and its tree with the blocks' own seeds.

```python
class StepPlanningProblem:                      # planning/step_problem.py (new, A1)
    def __init__(self, sys: StepSystem, cost: StepCost | None, *, N=None, gamma=1.0, X=None, U=None,
                 X0=None, Xf=None, x_start=None, infeasible_cost=None, x0_distribution=None, params=None)
    dt -> sys.sample_period (None for a bare tick)
    params -> the same view as the continuous problem's
    def realize(self, key, n_trials) ...        # the same draws
    def cost_bound(self) ...                    # g_max·Σγ^k + γ^N h_max

class StepCost:                                 # planning/step_problem.py
    params: dict = {}
    def c(self, x, u, k=0, params=None): ...    # the per-step cost
    def h(self, x, k=0, params=None): ...       # the terminal cost at tick N

class IntegratedCost(StepCost):                 # what discretize builds (A2)
    # c = ∫_{t_k}^{t_{k+1}} e^{−ρ(τ−t_k)} g dτ by the scheme on the augmented state [x; c]; h = cost.h(x, t_N)
```

`problem.discretize(dt, solver, substeps)` returns `StepPlanningProblem(discretize(sys, dt,
solver, substeps), IntegratedCost(cost, sys.f, dt, rho, solver, substeps), N=round(tf/dt),
gamma=exp(−ρ dt), X=at_ticks(X, dt), …)` where `at_ticks` binds `t_k = t₀ + k dt` on a
continuous set (A3). Time-varying sets are read at the samples (the margin between samples is a
documented limit; `substeps` recorded by the runner is the finer check, later).

### 6.4 Compile: the evaluators

Unchanged in shape (DESIGN §5, `core/compile/evaluators/base.py:96–146`,
`discrete.py:35–63`): one evaluator class per system kind, one body per method, `@compiled` on
JAX (`compiled.py:8`). Three additions:

```python
class ContinuousEvaluator:   # base.py:88
    step(x, u, t, params=None, *, dt, solver="rk4", substeps=1)                 # :96
    rollout(x0, u_seq, t0, dt, params=None, *, solver, substeps, hold, n_steps) # :110
    rollout_batch(x0s, u_sequences=None, *, t0, dt, n_steps, params=None)        # :146
class DiscreteEvaluator:     # discrete.py:32
    step(x, u, k=0, params=None)                                                 # :35
    rollout(x0, *, n_steps, u=None, params=None) -> Trajectory   # native arrays, differentiable (today a NumPy StepRollout, :39)
    rollout_batch(x0s, u_sequences=None, *, n_steps, params=None)   # new: vmap over starts, seeds, families
    signals(x, u, k=0, params=None)                                 # the buffer, :205 today compute_internal_signals_dict
```

The step diagram's plan (`core/compile/plan.py`) gains the `Held` wrapping at compile and the
algebraic-loop check reads a held output as feedthrough on its firing ticks. The prototype
compiled every loop of §10 through today's `compile_step_diagram` (`compiler.py:231`) with no
change to the plan: `Sampled`, `Held`, a native MPC block and a seeded policy are ordinary
blocks to it.

### 6.5 The evaluator, the evaluation and the one score

```python
class ProblemEvaluator:                         # planning/evaluator.py:40
    def __init__(self, problem, controller, dt=None, *, solver="rk4", substeps=1, backend="auto", episode_length=None)
        # the loop: a continuous controller → discretize(ctl @ plant, dt) whole (today's numbers, :586);
        # a sampled controller (a StepSystem) → ctl @ plant at its base period, dt must be it or None;
        # a built loop with a `plant` block, either kind. The refusal `require_continuous` (:789) goes.
        # the task: problem.discretize(dt, solver, substeps) — the StepPlanningProblem
    params -> {block_id: block.params (+ "x0" for a stateful block), "cost", "X", "U", "Xf",
               "x_start", "discount_rate", <free input>: nominal}            # :126, plus the x0 leaves (R124)
    def trajectory(self, params=None, key=None) -> Trajectory                  # :146
    def evaluate(self, params=None, key=0, n_trials=None, *, record=False) -> Evaluation   # :166
    def program(self, wrt, bounds=None, key=0, n_trials=None, *, params=None) -> MathematicalProgram   # :195
    def pack(self, params, wrt) / unpack(self, z, wrt, params=None)            # :289, :298 → core/params.py
```

The one scan is `run(params, x_start)` (`:355` today): `X_{k+1} = L.step(X_k, u_free, k; p)`
over `k < N` by the step evaluator's `rollout`, the signals along it by one `vmap` of
`signals`, the plant's `(x_k, u_k)` read by its role. The judge:

```python
def score_run(step_problem, traj, *, params=None, price=..., cost_bound=None) -> dict   # planning/evaluation.py:125
    # c_k   = cost.c(x_k, u_k, k; p) for every k, one vmap over the samples (A2)
    # cost  = Σ_{k<N} γ^k c_k + γ^N h(x_N, N)                    (no h on an infinite horizon)
    # feasibility_margin = min_k [X.margin(x_k, k), U.margin(u_k, x_k, k)];  failed = not finite or < 0
    # reachability_margin = Xf.margin(x_N, N);  reached = ≥ 0 (no Xf: reached)
    # J = cost, or with the price on: cost + price·(failed or not reached); cost_bound + price if not finite
def evaluate_trajectory(problem, traj) -> Evaluation   # :223; discretizes the problem at the trajectory's dt
```

`Evaluation` (`:27`) is unchanged: a pytree record with `objective` the criterion over `J`.
Value iteration (`dynamic_programming/dp.py:223`, `Q = g dt + α J_next`), the RL environment
(`learning/reinforcement_learning/environment.py:130`, `reward = −g dt`) and shooting
(`shooting.py:293`) read `step_problem.cost.c`, `.X`, `.U`, `.N`, `.gamma` and `price` from one
object, so a policy has one `J`. The RL environment keeps its early stop as a training choice
(truncation at the zone, failure priced once) and reports `evaluate`'s number.

### 6.6 The one score: the answer to the open investigation

- **Time binds at the tick.** A continuous set or cost is read at `t_k = t₀ + k dt`; no
  discrete library versions. A natively discrete cost is a `StepCost` with `c(x_k, u_k, k)`.
- **A continuous system is evaluated exactly as a `StepSystem` on its `dt`.** Yes: the
  evaluator's loop is a step diagram at `dt` (a continuous law discretized with its plant as one
  ODE; a sampled law at its base period), and the score is a plain sum `Σ γ^k c_k`.
- **The trapezoid becomes the per-step cost integrated by the scheme** (A2). Exact to RK4's
  order for a held input; `g dt` under Euler, so value iteration and RL keep their numbers.
- **One failure rule:** the run goes to the horizon; a negative margin at any sample is a
  failure priced once with the problem's price; RL's early termination is a training choice.
  Value iteration keeps pricing a cell that leaves its *grid* (the table's domain).
- **The hold:** zero-order, the computer's own semantics (`u_k` held over `[t_k, t_{k+1})`).
- **Shooting's reported score** is this `J` (its own trapezoid with a first-order hold goes);
  collocation's objective stays its transcription's quadrature, since its decision is the
  knots, and `solution.evaluation` is always the judge's number.
- **The discount:** `γ = e^{−ρ dt}` and the within-period factor `e^{−ρ(τ−t_k)}` inside `c_k`.
- **The scope:** every stepping tool (the evaluator, value iteration, the RL environment and the
  tabular learners, shooting, the native MPC, MPPI) reads the step problem; collocation, the
  continuous LQR, the linear analysis and `solver="scipy"` read the continuous one.

### 6.7 Programs, the optimizer, the kernels

```python
@dataclass(frozen=True)
class MathematicalProgram:                      # optimization/mathematical_program.py:26
    n_z: int
    J: callable                                 # J(z) or J(z, params) when params is declared (R130)
    h: callable | None; g: callable | None      # the same two forms
    lower, upper: arrays | None
    params: dict | None = None                  # the frozen tree; None: the student's one-argument form
    grad_J, hess_J, jac_h, jac_g: callable | None
    backend: str; metadata: dict
    def params_at(self, z) -> dict              # an exported program: the full tree with z written in (R134.6)

class Optimizer:                                # optimization/optimizer.py:75
    def __init__(self, program, z0=None, *, method="auto", backend=None, use_hessian=False, options=None, **method_kwargs)
        # method: "scipy_slsqp", "scipy_trust_constr", "ipopt", "l-bfgs" (wrapped);
        #         "gd", "adam" (native kernels, projected on the box); "auto" by the program's class
        # `compile_backend` → `backend`; the adapter attribute → `adapter` (R134.1)
    def solve(self, *, z0=None, params=None, iterations=None, callback=None, verbose=False) -> OptimizationResult   # :157
        # params: a partial tree merged once onto program.params (R131); iterations=K: a native kernel for
        # exactly K steps, one lax.scan on JAX (traceable, differentiable); None: to the method's tolerance
```

The program evaluator (`optimization/evaluators/`) jits `J(z, p)` and its derivatives in `z`
with `p` a pytree argument: rebinding is a call, not a trace (verified: one trace for six
rebinds, §10 run 5). `ParametricMathematicalProgram`
(`planning/trajectory_optimization/parametric_program.py:25`) and
`JaxParametricProgramEvaluator.bind` (`parametric_evaluator.py:122`) fold into it; the
transcriptions build `J(z, p)` on `p["plant"]`, `p["cost"]`, `p["x_start"]`, the sets' leaves
(`direct_collocation.py:337` closes over `nominal_plant_params` today, line 353).

```python
class GradientDescent:                          # optimization/kernels.py (new; today first_order.py:22 is a host loop)
    params = {"eta": η}
    def init(self, z0, params=None) -> state    # {"z": z0, "k": 0}
    def step(self, state, params=None, kernel_params=None) -> state     # z ← Π(z − η ∇J(z; p))
class Adam:   params = {"eta", "b1", "b2", "eps"}; state = {"z", "m", "v", "k"}   # today first_order.py:135's shape
def run_kernel(kernel, state, params, K, kernel_params=None) -> state  # a loop on NumPy, lax.scan on JAX
def as_step_system(optimizer) -> StepSystem     # z_{k+1} = step(z_k): the lesson's view (R132), its poles by `poles()`
```

The stochastic program is the convention of R131: `seed` an integer leaf of `params`, never
in `wrt`; `pe.program(wrt, key, n_trials)` exports the sample average realized once; a Monte
Carlo over solves is `jax.vmap` of `solve(iterations=K)` over the seed leaf (verified, §10
run 5); a solver that advances the seed is an `Optimizer` strategy for v0.4.

### 6.8 The policies

```python
class NeuralPolicy(System):                     # today NeuralPolicyController, control/neural.py:15 (rename at v1.0)
    def __init__(self, sys, *, features=None, hidden=(64, 64), activation="tanh", squash="clip", seed=0)
    params = {"mlp": {...}}; input `x`, output `u` = u_mid + u_half·squash(MLP(z(x)))   # :204
    # reads problem.U.bounding_box() when built by a planner (the KNOWN_ISSUES entry)

class GaussianPolicy(System):                   # control/stochastic.py (new, R129 + A7)
    is_random = True
    def __init__(self, mean: System, *, log_std=0.0, sample_period, seed=None)
    params = {"mean": mean.params, "log_std": (m,), "sample_period": (), "seed": int | None}
    # the mean's input ports copied; output u = μ(x; p) + exp(log_std)·ε(seed, ⌊t/Δ⌋); seed=None: u = μ
    def ctl(self, x, u, t=0.0, params=None): ...
    def log_prob(self, a, x, u, t=0.0, params=None): ...   # log π(a | inputs)
    def entropy(self, x, u, t=0.0, params=None): ...
    def realize(self, key): ...                 # a fresh seed under the key
class SquashedGaussianPolicy(GaussianPolicy)    # the SAC head
```

The RL planner's `StochasticPolicy` and heads (`learning/reinforcement_learning/policy.py:38`,
`:119`) are rebuilt on this block (with the block or in v0.4, the row's placement detail); the
collectors draw per-episode seeds through `realize`. `LookupPolicy`, `StateFeedback`,
`TimeVaryingFeedback`, `TrajectoryFeedback`, the PID family and the robotic laws are
unchanged `System`s.

### 6.9 The planners, the solution, the planner interface, the MPC

```python
class Planner:                                  # planning/planner.py:21
    def __init__(self, problem): ...
    def solve(self, **kwargs) -> PlanningSolution                      # :69
    # the online interface (R135, A9), the three verbs an MPC reads and nothing else:
    def prepare(self) -> None                   # compile once (today compile_parametric_program, trajopt planner.py:301)
    def plan(self, x0, params=None, z0=None) -> PlanningSolution       # today solve_trajectory_from, :355; the record carries z
    def shift(self, z, k=1) -> z0               # the warm start moved one period (today mpc_warm_start_guess)

class TrajectoryOptimizationPlanner(Planner):   # trajectory_optimization/planner.py:64
    def __init__(self, problem, *, n_steps, transcription="direct_collocation", backend="auto", method="auto", options=None, ...)
class ShootingPlanner(TrajectoryOptimizationPlanner)   # single shooting on the step problem: z = u_{0:H}, box-bounded (native-friendly)
class DynamicProgrammingPlanner(Planner):  __init__(problem, *, x_grid, u_grid, dt=None, ...)   # dt None on a StepPlanningProblem
class ReinforcementLearningPlanner(Planner): __init__(problem, *, dt=None, algorithm, ...)
class LQRPlanner(Planner):   solve().cost_to_go is a QuadraticField
class RRTPlanner(Planner)

@dataclass(frozen=True)
class PlanningSolution:                         # planning/results.py:21, basic (R138)
    problem; policy: System; trajectory: Trajectory | None; evaluation: Evaluation | None
    cost_to_go: Field | None; solver: record (success, method, z where the planner has one)
    plot_control_law / plot_cost_to_go / plot_trajectory / plot_cost

class ModelPredictiveController(StepSystem):    # trajectory_optimization/mpc/controller.py (today :471 + the latch :567)
    def __init__(self, planner, *, sample_period, iterations=None, method=None, eta=None,
                 parameter_ports=(), warm_start=True, verbose=False)
    # state x = z (the last plan, R104); input y (the measurement) and one port per parameter path;
    # params = planner.program.params without x_start (+ "eta" for a native kernel); x0 = the default guess
    def step(self, x, u, k=0, params=None): ... # z_{k+1} = plan(y_k, params ∪ ports, z0=shift(x)).z : K kernel steps
                                                # when iterations is set (one lax.scan), else the host solve
    def h(self, x, u, k=0, params=None): ...    # u_k = the first input of z_k: one period late (R123)
    def horizons(self, traj) -> overlays         # the plans read back from the recorded z (today mpc/viz.py)
```

With `iterations=None` the block is host code (SciPy, Ipopt): the loop still compiles as a
step diagram on NumPy and runs in the clock runner; with `iterations=K` on a `ShootingPlanner`
the block is pure array code and the loop traces on JAX (verified, §10 run 3). MPPI (v0.4) is
the same block with a derivative-free `plan` whose draws come from its `seed` leaf and the tick.

## 7. The architecture under it

### 7.1 Layers

```text
 ┌──────────────────────────────────────────────────────────────────────────────────────────┐
 │ 5  TOOLS (verbs)   analysis · simulation · planning (DP, trajopt, RRT, MPC) · learning (RL)  │
 │                    optimization (Optimizer: wrapped adapters | native kernels)              │
 ├──────────────────────────────────────────────────────────────────────────────────────────┤
 │ 4  THE PATH        PlanningProblem ──discretize──▶ StepPlanningProblem                      │
 │                    problem.compile(ctl) ─▶ ProblemEvaluator ─▶ rollout ─▶ score_run ─▶      │
 │                    Evaluation ─▶ pe.program(wrt) ─▶ MathematicalProgram(params) ─▶ Optimizer │
 ├──────────────────────────────────────────────────────────────────────────────────────────┤
 │ 3  COMPILE         compile(sys) → evaluator: f · step · rollout · rollout_batch · signals    │
 │                    (one body per method; @compiled on JAX; Held wrapped at compile)         │
 ├──────────────────────────────────────────────────────────────────────────────────────────┤
 │ 2  ALGEBRAS        continuous: System, DynamicSystem, DiagramSystem  (@ >> + connect feedback)│
 │                    step:       StepSystem, StepDiagramSystem, HybridDiagram (the same grammar)│
 │                    the one adapter between them: discretize / %  → Sampled                  │
 ├──────────────────────────────────────────────────────────────────────────────────────────┤
 │ 1  TEXTBOOK NOUNS  Trajectory · Set · InputSet · Distribution · CostFunction · StepCost ·   │
 │                    Field · sources (Step, WhiteNoise, TrajectorySource) · the cipher        │
 ├──────────────────────────────────────────────────────────────────────────────────────────┤
 │ 0  THE CONTRACT    params trees (core/params.py) · array_module (NumPy | JAX) · pure methods │
 └──────────────────────────────────────────────────────────────────────────────────────────┘
```

Layers import downward only. Layer 4 is one package, `planning/`, and it is the only place
that knows both a problem and a loop. Layer 5's planners return layer-2 objects (a policy is a
`System`), so the path judges what the planners produce.

### 7.2 The one contract (R140, restated as the law every layer obeys)

1. An object's mathematics is a method on its specific arguments and `params`:
   `f(x, u, t, params=None)`, `step(x, u, k, params=None)`, `h`, `g`, `c`, `margin`, `value`,
   `sample(key, n, params=None)`, `J(z, params=None)`, `init(z0, params)`, `step(state, params)`.
2. Every core object has `params` (`{}` when it has none); a composite's tree is its parts'
   trees by name, reachable by a dotted path; keys are roles and ids (`.` walks a tree, `:`
   names a signal, R134.3).
3. Leaves are floats, numeric arrays and the integer `seed`; never a `bool`, a `str`, a size.
4. A method is deterministic given its arguments and its tree; a draw is a function of a
   seed and an index; a key is an integer and the cipher is the one generator.
5. `params=None` reads the frozen defaults; inside the library a tree is complete; a user
   boundary merges a partial tree once.
6. A tool converts to arrays at the solver boundary, once, and never probes an object's type
   inside the math (`bounding_box()`, the roles, the kind by `isinstance` at the entry only).

### 7.3 Module layout and the dependency law

```text
minilink/
  core/
    system.py, diagram.py, wiring.py, topology.py, operators.py, closed_loop.py, assembly.py, roles.py
    params.py         NEW   lookup, with_leaf, merge, flatten, pack, unpack, float_leaves (one owner)
    fields.py         NEW   Field, QuadraticField, EllipsoidSet, FieldSet, FieldCost, the shaping functions
    trajectory.py, sets.py, costs.py, distributions.py (the cipher; integer keys), geometry.py
    discrete/
      step_system.py, step_diagram.py
      sampled.py      NEW   Sampled (the % result), Held (the multi-rate select), base_period_of
      hybrid_diagram.py     HybridDiagram(StepDiagramSystem): computer @ plant at its base clock
      (step_rollout.py and hybrid_composition.py fold into the above)
    compile/          compiler.py, plan.py (Held wrapped here), evaluators/ (rollout native, rollout_batch on step)
  blocks/             sources.py (WhiteNoise, NoiseSource, TrajectorySource with `values` a leaf), step.py (UnitDelay) …
  dynamics/           unchanged
  control/            siso.py, state.py, …, neural.py (NeuralPolicy), stochastic.py NEW (GaussianPolicy)
  estimation/         unchanged
  analysis/           discretization.py (discretize → Sampled), the step tools (poles) …
  simulation/         simulator.py (continuous), discrete/computer.py (the clock runner only), realtime/
  planning/
    problems.py (PlanningProblem, StochasticPlanningProblem, disturbances=), step_problem.py NEW
    evaluator.py (one ProblemEvaluator), evaluation.py (score_run on the step problem), environment.py (from learning/, R108)
    planner.py (prepare/plan/shift), results.py, comparison.py
    dynamic_programming/, trajectory_optimization/ (shooting.py: ShootingPlanner; mpc/controller.py: the pure block), search/
  learning/reinforcement_learning/   the planner and the algorithms on GaussianPolicy and RolloutEnvironment
  optimization/       mathematical_program.py (params), optimizer.py (solve(params, iterations)), kernels.py NEW, optimizers/ (adapters)
```

The dependency law of DESIGN §3 holds with two amendments: `core/discrete/` no longer imports
`simulation.discrete` (the `%` result is a core object; the exception of R100 is retired),
and `planning/step_problem.py` imports `analysis.discretization` (a tool on core, allowed:
tools import core, and planning already imports analysis for the LQR planner).

### 7.4 The runtime model

| What | Compiled once | Varies per call | Where host code stays |
| --- | --- | --- | --- |
| a system | `compile(sys, backend)`: one jitted form per method, knobs static | `x, u, t/k, params` (any leaf) | NumPy evaluators (plain bodies) |
| a step loop | the step diagram's plan: the port order, the slices, the `Held` selects | the same, plus the free inputs | — |
| the evaluator | `trials_fn`: `jit(vmap(score))` per batch layout | the tree `θ`, the starts, the seeds, `n_trials` in the batch axis | `solver="scipy"` (one run per trial, `solve_ivp`) |
| a program | the evaluator of `J(z, p)` and its derivatives in `z`, `p` a pytree argument | `z`, `p` (the MPC's `x_start`, a plant leaf, a cost weight, the `seed`) | Ipopt, SciPy (the adapters read `np` arrays) |
| a native solve | `jit(lax.scan(step, …, length=K))` | `z0`, `p`, the kernel's `eta` | `iterations=None`: the Python loop to a tolerance |
| the MPC tick | inside the loop's plan: K kernel steps as a scan | `y_k`, `z_k`, the ports' values | a host planner's `plan` (the NumPy face, the clock runner) |
| realtime | nothing | the keyboard | the whole `Computer` runner |

What retraces: a new batch layout (which leaves carry a trial axis), a new `record` flag, a
new size (`N`, `K`, `n_trials` as an axis length), a replaced problem part (a new cost object).
What never retraces: a new value of any leaf, a new key, a new start.

### 7.5 Why each main math path traces, and what it costs

- **The step rollout.** `iterate` (`core/compile/evaluators/stepping.py:62`) is one
  `lax.scan` whose carry is the stacked state and whose per-step inputs are `(k, u_k)`;
  `step_body` (`:40`) unrolls `substeps` statically. A diagram's step is straight-line port
  arithmetic on gathered buffers (`plan.py`). The only data-dependent control flow in the loop
  is `Held`'s firing, a `lax.cond` on `k mod d` (under `vmap` it becomes a `select`, so a batch
  still compiles). Verified: run 2's jaxpr holds a `cond`; `grad` through it equals central
  differences. Cost: the plant's substeps are `substeps × 4` evaluations of `f` per tick; a slow
  block costs its own tick only every `d` ticks under `jit` (`cond` runs one branch), both
  branches under `vmap`.
- **The score.** A `vmap` of `c(x_k, u_k, k)` over the samples (reverse-mode friendly, no
  carry) then two reductions; the margins are `min` over samples; the price is a `where`.
  Verified: run 1, 1000 loops × 200 ticks scored in 0.19 s on CPU, gradient to six digits.
- **The draws.** `standard_normal(seed, k, p)` is 20 rounds of integer arithmetic on
  `uint32` arrays (`distributions.py:162`), traced with `k` a tracer; `child_seed` with a
  traced key is the same word (the prototype's `child_seed_traced`). `realize` is then a
  `vmap` over the trial index: no Python per trial. Verified: run 4, 64 trials in one `vmap`,
  bits equal across backends.
- **The program and the native solve.** `J(z, p)` is the shooting scan plus the field's
  quadratic; `grad_J` is `jax.grad` inside the kernel's `step`; `run_kernel` is a scan over
  `K`; the MPC tick nests it; the closed loop nests the tick in the diagram's scan; the tuning
  gradient is `grad` of all of it (reverse over reverse over two scans). Verified: run 3,
  112 ms for 80 ticks × 15 iterations, `vmap` over starts, the tuning gradient finite and
  useful. Cost: reverse mode stores the kernel's iterates (`N × K × H` floats, trivial here;
  checkpointing is the knob when `K` grows).
- **The parametric rebind.** `p` is a pytree argument of the jitted `J`: a new value is a call.
  Verified: run 5 (one trace, six rebinds) and run 3 (one trace, three starts).
- **What does not trace, by design.** `solver="scipy"` (SciPy's adaptive integrator), Ipopt
  and SLSQP (host adapters), the clock runner (I/O), the plots. Each sits behind one keyword
  and never inside a traced body.

## 8. The v0.3 coverage table

| Workstream / choice | Where this proposal satisfies or revises it |
| --- | --- |
| §1 every object pure, every number a parameter | §7.2 the contract; §6.2 (`params = {}`, sets and distributions reading `params`, integer keys); §6.3 (`x0` leaves, `x_start`, `disturbances=`); `core/params.py` (§7.3); `TrajectorySource.values` a leaf; `Set.as_cost` (§6.2) |
| §2 the step algebra first-class | §6.1 (`@`, `>>`, `+` on step systems; the base period; `poles`); §6.4 (`rollout` native and differentiable, `rollout_batch`); the `Trajectory` fold (`compute_rollout` → `Trajectory`); the eight hand loops on `step_body` all read the step problem or the step evaluator (§6.5) |
| §3 the draws | §6.2–§6.3 (A6: one cipher, integer keys, `realize(key, n_trials) → (x0s, params)` with per-block seeds by path, `disturbances=` as source blocks, the same bits on both backends); `Sys2Gym.from_problem` reads `realize` |
| §4 the seeded stochastic policy | §6.8 (`GaussianPolicy` on any mean `System`, `log_prob`, `entropy`, the seed leaf, `seed=None` the mean; A7) |
| §5 the hybrid loop: one model, two runners | §6.1 (`%` → `Sampled`, `Held`, `HybridDiagram(StepDiagramSystem)`, `compute_rollout` / `compute_trajectory`, the clock runner `Computer`); A4, A5 |
| §6 one score | §6.5–§6.6 (`StepPlanningProblem`, `IntegratedCost`, `score_run` as a sum, one failure rule, the scope); A1–A3 |
| §7 the parametric program; the stochastic program | §6.7 (`MathematicalProgram(params=)`, `solve(params=)`, `params_at`, the transcriptions on `p`, one evaluator body, the `seed` leaf, the sample-average export, `vmap` over seeds) |
| §8 the native proof of concept | §6.7 (`GradientDescent`, `Adam` as `init`/`step`, `run_kernel`, `iterations=K`, `as_step_system`); §6.9 (`ModelPredictiveController(iterations=K)` on `ShootingPlanner`); §5.6; run 3 |
| §9 Fields | §6.2 (`Field`, `QuadraticField`, `as_set`/`as_input_set`/`as_cost(terminal=)`, `GridField`/`CallableField` later); `cost_to_go: Field` on every solution; §5.7 |
| §10 hardening and the merge | §9 (the order of work, the number changes, the re-pins); the wiring refusals unchanged (S66 as planned) |
| R124 three owners of a start | kept: block `x0`, diagram views with write-through, the problem's `x_start`, the evaluator's `x0` leaves (§6.1, §6.3, §6.5) |
| R125 one same-tick semantic | kept: propagation within the tick, `Held` recomputes its outputs on a firing tick, a delay only as a block (§6.1) |
| R126 the operators on step systems | kept (§6.1) |
| R127 `computer @ plant` a step diagram, two verbs | kept for the class and the two verbs; **revised** on `compute_trajectory`'s default runner (A5) |
| R128 disturbances are source blocks, one realization shape | kept (§6.3); the shortcut builds the diagram at construction |
| R129 the seeded policy | kept; **refined**: the mean is any `System`, `log_prob(a, x, u, t)` (A7) |
| R130 one program noun with `params=None` | kept (§6.7) |
| R131 programs, parameters, randomness | kept (§6.7): complete trees inside, the `seed` leaf, integers never in `wrt`, sizes structural, `z0` on the solve |
| R132 a native solver is a pure kernel with a step-system view | kept (§6.7), `iterations=` the fixed-`K` form (A10) |
| R133 a minimal `Field`, one `as_cost`, names by return | kept; **refined** with `terminal=` (A8) |
| R134 names across the API | kept: `backend=`, `adapter`, `sample_period=`, `.` and `:`, `…Policy`, `parameter_ports=`, `params_at`; `mount` for the geometry verb |
| R135 a generic planner interface | **filled**: `prepare`, `plan`, `shift` (A9) |
| R136–R139 | unchanged (learning from data later; no `KinematicModel`; a basic solution; the road to the freeze) |
| R140 the one contract | §7.2, with the five resolutions kept (`{}`, the distribution's key, the composite tree, RL's exception, the student's tolerance) |
| The one-score investigation (QUESTIONS) | answered in §6.6 |

## 9. The migration from today's code

Module by module: what changes, which numbers move (each an intended change under R80, one
commit with its closeness report and re-pin), the risk, then a proposed order.

| Module (today) | What changes | Numbers | Risk |
| --- | --- | --- | --- |
| `core/params.py` (new) | the four copies (`planning/evaluator.py:991–1036`, `planning/problems.py:896–905`, `core/compile/evaluators/jacobian.py`, `core/assembly.py`) become one module; `pack`/`unpack` move here | none (byte-identical) | low |
| `core/distributions.py`, `core/sets.py` | `sample(key: int, n, params)` on the cipher; `split_keys`, the generator and the JAX-key paths go; `Gaussian(cov=)`; `params` on every law and set (`{}` default) | every seeded draw changes once; `evaluate(key=)` the same on both backends | low; the cipher exists (`:162–:275`) |
| `blocks/sources.py` | `NoiseSource`; `TrajectorySource.values` a leaf; `WhiteNoise` unchanged | none | low |
| `core/system.py`, `core/wiring.py` | `x0`, `n`, `state` as diagram views with write-through; `derived_x0` and the state part of `refresh` go; `%` returns `Sampled` | none for continuous loops | medium: `refresh()` callers (the simulator's hint bubbling) |
| `analysis/discretization.py` → `core/discrete/sampled.py` | `Discretization` becomes `Sampled`: every port kept, `sample_period`; `Held`; `ExactDiscretization` kept as `solver="exact"` | none | low |
| `core/discrete/step_diagram.py`, `core/compile/plan.py` | the base period and the `Held` wrapping at compile; `@`, `>>`, `+` on step systems; the algebraic-loop check on firing ticks | none for single-rate diagrams | medium: the plan builder |
| `core/compile/evaluators/discrete.py` | `rollout` native and differentiable, returns a `Trajectory`; `rollout_batch`; `signals` | none | low |
| `core/discrete/hybrid_diagram.py`, `hybrid_composition.py`, `simulation/discrete/` | `HybridDiagram(StepDiagramSystem)`; `StepSchedule`, the double buffer and `HybridSimulator`'s tick loop go; `Computer` the clock runner; `compute_trajectory` on the step runner with substeps recorded, `solver="scipy"` the adaptive path | **the hybrid demos and the two hybrid regression gates move once** (R125's within-tick propagation, R123's MPC delay, A5's default runner) | medium: the demos' plots read `HybridSimResult` fields |
| `planning/step_problem.py` (new) | `StepPlanningProblem`, `StepCost`, `IntegratedCost`, the sets at ticks; `PlanningProblem.discretize` | none by itself | low |
| `planning/evaluation.py` | `score_run(step_problem, traj)`: the sum of `c_k`, one failure rule; `evaluate_trajectory` discretizes at the trajectory's `dt` | **the evaluator's `J` moves once** (trapezoid → the integrated per-step cost; 0.09 % on the prototype's run); value iteration and RL unchanged under Euler | low |
| `planning/evaluator.py` | one class on the step problem and the step loop; the continuous law discretized with its plant as one ODE (today's numbers); the sampled face (`StepSystem` controllers accepted, `require_continuous` gone); `x0` leaves; `realize` on the cipher | continuous loops: none beyond the score's change; sampled loops: new | medium: the largest file today (1179 lines) shrinks |
| `planning/problems.py` | `disturbances=` on both problems as blocks (a bare law refused); `params=` the laws; `realize` one shape | the stochastic numbers with disturbances change once (held draw → a signal) | low |
| `planning/dynamic_programming/`, `learning/reinforcement_learning/environment.py` | read `problem.discretize(dt)`: `c_k`, the sets at ticks, `γ`, the price; `RolloutEnvironment` moves to `planning/` (R108) | none under Euler (verified: `c = g dt`); the RL re-pin comes with the seeded policy | low |
| `planning/trajectory_optimization/` | the transcriptions build `J(z, p)` on the tree; `ParametricMathematicalProgram` and its evaluator fold into `MathematicalProgram(params=)`; `ShootingPlanner` on the step problem; `prepare`/`plan`/`shift`; shooting's reported score lowered to the judge's | collocation's plans: none (`x0`-only binding byte-identical); shooting's reported score moves once | medium: `planner.py` (1025 lines) |
| `optimization/` | `MathematicalProgram.params`; `Optimizer.solve(params=, iterations=)`; `kernels.py` (`gd`, `adam` as `init`/`step`); `backend=`, `adapter` | the first-order presets byte-identical for `iterations=None` (the same update) | low |
| `mpc/controller.py` | one pure `StepSystem`: state `z`, the one-period delay, `parameter_ports`, `iterations=`; the latch, `MPCStatelessController`, `export_to_computer`, the dual-rate computer go (a dual rate is `mpc % 0.05` beside a `% 0.01` block) | **the MPC demos move once** (R123's delay) | medium: `viz.py` reads the recorded `z` |
| `control/stochastic.py` (new), `control/neural.py` | `GaussianPolicy`; `NeuralPolicy` reads `problem.U` | the RL numbers move once when the heads are rebuilt | low (the heads' rebuild may wait for v0.4) |
| `core/fields.py` (new), `planning/spatial/` | `Field`, `QuadraticField`, `as_set`/`as_input_set`/`as_cost(terminal=)`; the shaping moves; `LQRPlanner.cost_to_go` a field | none (the RoA and DP baselines `cmp`) | low |

**A proposed order of work** (the three lanes of core-architecture §10, with the joins placed
where the prototype shows they are cheap):

1. **The contract pass** (§1): `core/params.py`; `params = {}`; the cipher-only draws; the
   diagram views of `x0`; `TrajectorySource.values`. Byte-identical except the seeded draws.
2. **The loop lane** (§2, §5): `Sampled` with every port; `Held` and the base period; the step
   operators; the native step `rollout`; `HybridDiagram` as a step diagram with its two verbs;
   `Computer` the runner; the hybrid demos re-pinned once (R125 + R123 + A5 together).
3. **Join 1, the one score** (§6): `StepPlanningProblem` and `discretize`; `score_run` as
   the sum; the evaluator on the step problem, continuous and sampled faces; value iteration
   and the RL environment reading it; the evaluator's `J` re-pinned once.
4. **The draws and the policy** (§3, §4): `realize` by path with per-block seeds;
   `disturbances=` as blocks; `GaussianPolicy`; the RL collectors on seeds (or deferred).
5. **The program lane** (§7, §8): `MathematicalProgram(params=)` with one evaluator body;
   the transcriptions on the tree; `kernels.py` and `iterations=`; `ShootingPlanner`; the MPC
   block pure with `prepare`/`plan`/`shift`; the MPC demos re-pinned once.
6. **Join 2** (§8): the native MPC's closed loop under `problem.compile`, tuned by gradient
   (the §5.6 script as the acceptance test).
7. **Fields** (§9), beside steps 1–6 from step 1 on.
8. **Hardening and the merge** (§10): the wiring refusals, DESIGN brought to each landed step,
   the notebooks, the student-facing re-pin at the merge.

The five intended number changes of core-architecture §10 become four commits here: the
same-tick semantic with the MPC delay and the hybrid default (one re-pin), the one-score `J`,
the cross-backend draws with live disturbances, the RL re-pin with the seeded policy.

## 10. What I verified

Everything below ran in this container (`python3 3.13`, NumPy 2.5.3, JAX 0.12.0.1, SciPy
1.18.1, minilink at `f634e47`) from
`/tmp/claude-0/-home-user-minilink/36350854-ab7e-51d8-8262-d68717b27e02/scratchpad/arch-E/`
with `PYTHONPATH=/home/user/minilink`; the full log is `all_runs.log` there. The prototype
reuses the real `Pendulum`, `PID`, `StateFeedback`, `Gain`, `QuadraticCost`, `PlanningProblem`,
`discretize`, `StepDiagramSystem`, `compile_step_diagram`, `step_body`, `score_run` and the
cipher (`standard_normal`, `random_bits`, `child_seed`); it adds the pieces of §6 that do not
exist (`proto_core.py`: the params module, `step_with_cost`, `StepCost`, the sets at ticks,
`StepPlanningProblem`, `score_samples`, `Sampled`, `Held`, `branch`, `GaussianPolicy`,
`realize`, `Program`, `GradientDescent`, `Adam`, `run_kernel`, `QuadraticField`).

**Run 1, one score (`run1_one_score.py`):** a digital PID (`Sampled(PID, 0.02)`) on
`discretize(Pendulum, 0.02)` in a `StepDiagramSystem`, scored by `discretize_problem`.

```text
(1) rk4 state part byte-identical: True  c_k = 0.00998825561305984
(2) euler c_k == g dt: True 0.00993 0.00993
(3) NumPy J = 1.97979898870779  least margin to X: 9.084267051157404
    today's score_run (trapezoid) J = 1.978054465961911  rel diff: 0.0008811615501519382
(4) JAX jit J = 1.9797989887  NumPy J = 1.9797989887  |diff| = 2.22e-16
    first call (trace+compile) 0.65 s, second call 2.14 ms for 200 ticks
    dJ/dKp  grad = 0.109266   central diff = 0.109266
    dJ/dm   grad = 1.336067   central diff = 1.336067
(5) vmap over 4 starts: [1.97979899 1.61692317 3.77691854 2.25212105]
    1000 loops x 200 ticks scored in 193.6 ms (CPU)
```

**Run 2, the pure sampled loop (`run2_sampled_loop.py`):** a two-rate computer (a digital
low-pass at 100 Hz, the PID held every 5 ticks) on the pendulum discretized with 4 substeps,
as one step diagram at the base clock, compiled by today's compiler on both backends.

```text
(1) step-diagram runner == clock runner, byte for byte: True  max |diff| = 0.0
(2) JAX J = 0.3954570438979661  NumPy J = 0.3954570438979661
    dJ/dKp = -0.030301769896364617  dJ/dalpha = 0.008654635886261322  dJ/dm (plant mass) = 0.21918416522687434
    central diff dJ/dalpha = 0.00865463589683202
    jaxpr of one tick holds a cond: True
(3) dense plant samples: (1200, 2)  every 4th row == tick sample: True  final state agrees: True
```

**Run 3, the native fixed-iteration MPC (`run3_native_mpc.py`):** single shooting with `H = 12`
on the discretized pendulum, the LQR value as a `QuadraticField` terminal cost, 15 Adam
iterations per tick warm-started by the shift, the MPC a `StepSystem` with the one-period
delay, the closed loop compiled by today's step-diagram compiler, judged by a second problem.

```text
(5) terminal field P = [[14.645, 4.771], [4.771, 2.082]]
(1) J(z, p) rebound with three starts: traced 1 time(s), evaluated 3 times
(2) 400 Adam steps: J = 2.3806765755423855   L-BFGS-B: J = 2.3806765762255537  max|z diff| = 0.0002443262435738447
(3) closed loop in one scan: J = 19.6184; trace+compile 1.3 s, then 112.2 ms per run of 80 ticks x 15 Adam iterations
    theta - pi over the run (every 10 ticks): [0.4, 0.335, 0.132, 0.035, 0.004, -0.005, -0.007, -0.008, -0.008]
    vmap over 6 starts: [19.6184, 19.6184, 22.7515, 46.392, 24.05, 15.2824]
    tuning step 0: mean J = 24.6188, d J / d log R = 6.4636, R = 0.1000
    ...
    tuning step 7: mean J = 24.1898, d J / d log R = 0.0200, R = 0.1802
(4) the MPC's effort weight tuned by gradient through its own fixed-K solver, K = 15
```

**Run 4, the draws and the seeded policy (`run4_draws_policy.py`):** `realize` by named
streams on both backends; `GaussianPolicy` around a `StateFeedback`; the stochastic sampled
loop `vmap`ped over 64 (start, mass, seed) triples; the gradient of the mean score.

```text
(1) starts (NumPy): [[0.4389, 0.0679], [0.1356, 0.0172], [0.5862, -0.0436], [0.581, 0.0707]]
    starts (JAX)  : [[0.4389, 0.0679], [0.1356, 0.0172], [0.5862, -0.0436], [0.581, 0.0707]]
    max |NumPy - JAX| of the starts: 3.469446951953614e-18 (the cipher bits agree; cos/log round differently)
    same bits on both backends: False True True
    adding a random block 'load' leaves the others unchanged: True True
(2) u(t_k), seed 123, NumPy: [0.00873, -0.14203, -0.33385, -0.56896]
    u(t_k), seed 123, JAX  : [0.00873, -0.14203, -0.33385, -0.56896]  equal: True
    seed=None is the mean: [-0.5]  log_prob: -1.152786524109589
(3) mean J over 64 trials (starts, masses, policy seeds drawn from key 7): 1.8680245384373346
    d mean J / d K = [[0.27275, -0.88046]]   d / d log_std = 0.06584524889756413
    gradient step 0: mean J = 1.52663  K = [[2.8636, 1.4402]]
    ...
    gradient step 4: mean J = 1.18465  K = [[2.5485, 2.2267]]
```

**Run 5, the parametric program and the kernels (`run5_program.py`):** a robust gain-tuning
program whose `seed` is a leaf; one trace for six rebinds; the kernel's scan against its loop;
`vmap` of a whole solve over seeds; the step-system view of the solver and its pole.

```text
(1) program evaluated 6 times with 3 seeds x 2 plant values: traced 1 time(s)
(2) 50 gd steps: scan z = 1.5020845891765127  loop z = 1.5020845891765127  identical: True
(3) z* over 8 seeds (sample averages of 16 draws each): [1.5146, 1.5569, 1.4998, 1.5515, 1.5311, 1.5447, 1.5546, 1.5603]
    mean z* = 1.5391919575419513  std = 0.020616794731535548
    as_step_system rollout: z_50 = 1.5020845891765127  == scan: True
    the solver's pole at its fixed point (|∂step/∂z| < 1 means the descent is stable): 0.9517799630876397
```

**Facts checked in the code** (each cited at its line in §6): `Discretization` keeps only the
standard `y` port, so `discretize(PID(...), dt)` has no `u` output (`analysis/discretization.py:50–:119`;
probed: `list(discretize(pid, 0.01).outputs) == []`); `Computer.tick` reads the committed
buffer and mutates (`simulation/discrete/computer.py:114`); `require_continuous` refuses a
`StepSystem` controller (`planning/evaluator.py:789`); `trial_key` draws differently per
backend (`:1173`); the four parameter-tree helper copies; `ParametricMathematicalProgram` binds
`x0` only (`parametric_program.py:25`); collocation closes over `nominal_plant_params`
(`direct_collocation.py:353`); the first-order optimizer is a host loop (`first_order.py:56`);
the RL reward is `−g dt` (`environment.py:130`) and value iteration's backup `g dt + α J`
(`dp.py:223`); `score_run` is the trapezoid (`evaluation.py:125`); `Pendulum.y` is the full
state (`p = 2`), so a scalar PID needs a `Gain([[1, 0]])` sensor in a hand-built step diagram
(what `@` inserts as a `Demux` on the continuous side).

**Honest list of what is not verified.**
- The real `compile_step_diagram` was used unchanged: `Held` and `Sampled` are ordinary blocks
  to it. The compile-time `Held` wrapping from `sample_period`s and the step operators on
  `core/assembly.py` were not prototyped (they are wiring, not math).
- `jax.grad` with respect to a `Held` block's own `x0` was not run (the diagram `x0` views).
- The adaptive hybrid path (`solver="scipy"` restarted at the held-input changes) was not
  prototyped; its NumPy-only nature is by design.
- `GaussianPolicy` with a `NeuralPolicy` mean was not run (only a `StateFeedback` mean);
  nothing in the block depends on the mean's form.
- The program's constraint rows (`h`, `g` of `(z, p)`) on the compiled path with Ipopt were not
  run; only the box-bounded single-shooting form was, with SciPy L-BFGS-B as the host check.
- The same key gives the same *bits* on both backends; the floats differ by 3·10⁻¹⁸ because
  XLA's `cos`/`log` round differently from NumPy's. A cross-backend test needs a tolerance.
- `vmap` over 1000 starts at 0.19 s and the MPC's 112 ms per run are CPU numbers in this
  container; no GPU figure.

## 11. Open questions I could not settle

1. **`compute_trajectory`'s default runner on the sampled loop** (A5 against R127). I recommend
   the step runner with recorded substeps (one truth, verified cheap); the maintainer's
   alternative stays one keyword away either way. A decision is needed before the hybrid
   demos are re-pinned, since they move once.
2. **Whether `StepPlanningProblem` is a public noun or an internal lowering** (A1). I recommend
   public on the research surface, because a natively discrete task has no other home and six
   tools consume it; if the maintainer prefers internal, the class stays as the evaluator's
   private object and `problem.discretize` is a method on `ProblemEvaluator`, with the grid
   world of §5.9 lost.
3. **The margin between samples.** Checked at the samples only in v0.3; recording the substeps'
   margins from the plant's `Sampled` step is a later refinement. I recommend stating the limit
   in DESIGN and leaving it.
4. **The name of the loop.** `HybridDiagram` (kept by R127) now names a `StepDiagramSystem`
   subclass; `SampledDataSystem` or `SampledLoop` reads truer. I recommend deciding it at the
   v0.9 name pass, with `NeuralPolicyController → NeuralPolicy`.
5. **Where the RL heads' rebuild lands** (with `GaussianPolicy` in v0.3, or with the RL
   follow-ups in v0.4). I recommend v0.4: the block's contract is what the core needs, and the
   re-pin of the RL numbers is cheaper once.
6. **`Optimizer` on a program with integer leaves.** `jax.grad` refuses an integer leaf in the
   differentiated tree (seen in run 4); `pe.program` packs float leaves only, and a `grad` the
   user writes by hand must do the same. I recommend `pe.float_params` (the tree without its
   integer leaves) as the documented argument of a hand-written `jax.grad`, or `jax.grad(...,
   allow_int=True)` noted in the docstring.
7. **The `Held` block's cost under `vmap`** (both branches evaluated). For a slow expensive
   block (an MPC at divisor 5 in a batch of starts) the hyperperiod-unrolled layout of
   sampled-path §8.1 is the optimization; I recommend leaving it to a benchmark in v0.4.
8. **The deterministic problem's `disturbances=` with a `Step` load and the Kalman design
   reading the noise model from the problem.** The shape is set (sources on ports); which design
   verbs read `problem.sys`'s sources for `Q_n`, `R_n` is an estimation question for the
   estimation plan.
