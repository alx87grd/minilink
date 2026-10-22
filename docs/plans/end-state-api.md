# The end state on paper: the architecture and the API after v0.4

**Status:** a sketch for the maintainer, 2026-10-22 — ideas to review, nothing decided.
Every open decision of ROADMAP §6 that shapes a public name or a core contract is answered
here with its recommended option *as the code a user would write*, so the whole can be read
once before any of it is built.
**Lane:** maintainer-owned (every item is an **[ask — core]** or **[ask — public names]**).
**Rung:** the agenda of the two rulings sessions (ROADMAP §5.3, §6 "Blocking code now"), and
the v0.4 asks that fix a name the v0.3 code will already print.
**Owners of the detail:** this document cites, it does not restate. The arguments, the
evidence and the alternatives live in [sampled-path.md](sampled-path.md),
[differentiable-problem.md](differentiable-problem.md), [parametric-program.md](parametric-program.md),
[core-consolidation.md](core-consolidation.md), [future-policies.md](future-policies.md),
[mppi.md](mppi.md), [cost-params.md](cost-params.md), [fields.md](fields.md),
[geometry-module.md](geometry-module.md) and [randomness.md](randomness.md). A ruling taken
here goes to the plan doc it shapes and to a dated record; this file is deleted once the
sessions have ruled and the plan docs carry the answers (one fact, one home).

---

## 1. The end state in one page

One spine, read left to right; every pillar is a special case of it.

```
System ──compile──► rollout ──► Trajectory ──► cost · margin ──► MathematicalProgram ──► Optimizer
  p = (plant, controller, cost, sets, x0, draws)                                  ∂J/∂p
```

Two step paths on the spine, and no third:

| Path | What steps | Integrator | Who uses it |
| --- | --- | --- | --- |
| **continuous** | the closed-loop *diagram*, the controller's state inside `x` | `Simulator` (adaptive RK45 by default) for "what does it really do"; SM-1's compiled fixed step for the compiled problem | simulation, analysis, `compare()`, `tune` of a continuous law |
| **sampled** | the plant by the *step model* `x_{k+1} = Φ(x_k, u_k, k; p)`, the controller run once per period on a computer | SM-1's compiled fixed step, `substeps` per period | RL, DP, the gym bridge, MPC, MPPI, the digital PID, the realtime loop, identification and imitation |

The same compiled step serves both; the continuous path integrates the whole loop with it,
the sampled path integrates the plant with it and calls the controller between samples.

### 1.1 The whole toolbox, in the final API

```python
import numpy as np
from minilink import (Pendulum, PID, StateFeedbackController, WhiteNoise, Step,
                      PlanningProblem, StochasticPlanningProblem, QuadraticCost, BoxSet,
                      Gaussian, lqr, discretize, compare, tune)

# 1. A model, a loop, a simulation, an analysis — unchanged from v0.2
plant = Pendulum()
loop = Step(port="r") >> PID(Kp=20, Ki=10, Kd=2) @ plant
loop.compute_trajectory(tf=10)
loop.plot_trajectory()
loop.plot_bode()
K = lqr(plant, Q=np.eye(2), R=np.eye(1))

# 2. The sampled path: one step model, one sampled loop
plant_k = discretize(plant, dt=0.05)                      # x_{k+1} = Φ(x_k, u_k, k; p), RK4, zoh, 1 substep
plant_k = discretize(plant, dt=0.05, method="rk4", substeps=4, hold="zoh")
digital = PID(Kp=20, Ki=10, Kd=2) % 0.05                  # a computer: the law, sampled (continuous states: future work)
sampled = StateFeedbackController(K) % 0.05 @ plant       # the sampled loop: a tool, not a System
sampled.compute_trajectory(tf=10)                        # the plant continuous between samples, the law held
sampled.compile(backend="jax")                           # the kernel's JAX face: scan over ticks, vmap over trials

# 3. The problem, deterministic and stochastic
problem = PlanningProblem(plant, cost=QuadraticCost.from_system(plant), X=BoxSet.from_system_state(plant), tf=10.0)
noisy = StochasticPlanningProblem(plant, cost=problem.cost, X=problem.X, tf=10.0,
                                  x0_distribution=Gaussian(mean=plant.x0, cov=0.1 * np.eye(2)),
                                  params_distribution={"m": Gaussian(1.0, cov=0.01)},
                                  disturbances={"d": WhiteNoise(psd=1e-2)})

# 4. The compiled problem: one differentiable path from any parameter to any metric
ev = problem.compile(PID(Kp=20, Ki=10, Kd=2), dt=0.05)   # a ProblemEvaluator; the loop as a diagram
ev.params                                                # {"controller": …, "plant": …, "cost": …, "sets": …, "x0": …, "r": …}
traj = ev.trajectory()                                   # a Trajectory, every internal signal recorded
J = ev.cost()                                            # the one scoring contract
m = ev.margin()                                          # ≥ 0 iff the run stayed in X and U
dJ = jax.grad(ev.cost)(ev.params)                        # d J / d every parameter
report = noisy.compile(ctl, dt=0.05).evaluate(key=0, n_trials=200)   # mean, worst, failure_rate; replay any trial
J_bar = noisy.compile(ctl, dt=0.05).value(key=0, n_trials=200)       # the criterion over the draws, one traced scalar
ev_k = problem.compile(StateFeedbackController(K) % 0.05, dt=0.05)   # v0.4: a sampled controller runs the kernel's JAX face

# 5. Tuning, identification, imitation: three verbs on one bridge to the optimizer
program = ev.program(wrt=["controller.Kp", "controller.Kd"], bounds={"controller.Kp": (0, None)})
Optimizer(program, method="l-bfgs").solve()               # the program knows its class; "auto" picks by class
tuned = tune(problem, PID(Kp=20, Ki=10, Kd=2), wrt=["controller.Kp", "controller.Ki", "controller.Kd"], dt=0.05)
fitted = tune(logged_problem, plant, wrt=["plant.m", "plant.b"], dt=0.05)   # identification: a tracking cost on logged data
compare(LQR=LQRPlanner(problem).solve(), tuned=tuned).plot_trajectory()

# 6. The planners, every one a PlanningSolution; MPC and MPPI on the sampled loop
sol_dp = DynamicProgrammingPlanner(problem, dt=0.05).solve()           # Euler on the grid (byte-identical today), method= later
sol_to = TrajectoryOptimizationPlanner(problem, transcription="direct_collocation").solve()
sol_rl = ReinforcementLearningPlanner(noisy, dt=0.05, substeps=1, algorithm="ppo").solve()
mpc = ModelPredictiveController(TrajectoryOptimizationPlanner(problem, compile_backend="jax"), dt_mpc=0.1)
mppi = ModelPredictiveController(MPPIPlanner(noisy, dt=0.05, n_samples=1024), dt_mpc=0.05)
(mpc @ plant).compute_trajectory(tf=10)                               # the sampled loop again
mpc.bind({"plant": {"m": 1.2}, "cost": {"xbar": x_target}})           # v0.4: every problem parameter bound online
```

Everything above is one of the five nouns the constitution names, or a verb on one of them.
No new noun beyond `ProblemEvaluator` (the compiled problem) and the sampled loop's class.

### 1.2 The surface, by band and lane, after v0.4

| Band | Teaching surface (frozen at v1.0) | Provisional (in the wheel, no promise) | Experimental (repo only) |
| --- | --- | --- | --- |
| `core` | `System`, `DynamicSystem`, `StepSystem`, `DiagramSystem`, `Trajectory`, `CostFunction`, `QuadraticCost`, `TimeCost`, `Set` family, `Distribution` family, `Field`, `QuadraticField`, `GridField`, `Shape`, `Scene`, `Track`, `closed_loop`, `feedback`, `discretize`, `realize` | `StepDiagramSystem`, `SampledLoop`, `CallableField` | — |
| `catalog` | the vehicle ladder, the three racecars, the arms and the UR5, pendulums, cart-poles, double pendulum, double integrator, single mass, drone, rocket (≈ 26 plants) | — | `experimental/catalog/<domain>/` (≈ 24 plants) |
| `blocks` | routing, nonlinear, filters, sources (`Step`, `Sine`, `Ramp`, `Chirp`, `WhiteNoise`, `NoiseSource`, `TrajectorySource`), `TransferFunction`, `MLP` | `Delay`, `Switch`, `ZOHHold` | — |
| `control` | `P` / `PI` / `PD` / `PID`, `Lead` / `Lag`, `ImpedanceController` family, `StateFeedbackController`, `OutputReferenceController`, `lqr`, `place`, `lqg`, computed torque, sliding mode, robotic laws, `PurePursuit`, `NeuralPolicyController` | `ModelPredictiveController`, `GaussianPolicy` (the seeded stochastic policy), `CBFSafetyFilter` | — |
| `estimation` | `StateObserver`, `luenberger`, `kalman` | `ExtendedKalmanFilter`, `TrajectoryObserver` | — |
| `analysis` | `linearize`, `transfer_function`, `bode`, `margins`, `pzmap`, `root_locus`, `step_response`, `sensitivity` family, `controllability`, `observability`, `region_of_attraction` | the traceable twins (S73) | `modal` |
| `simulation` | `Simulator`, `RealtimeSimulator`, `game` | the sampled kernel's faces | — |
| `planning` | `PlanningProblem`, `StochasticPlanningProblem`, `PlanningSolution`, `Evaluation`, `compare`, `tune`, `DynamicProgrammingPlanner`, `LQRPlanner`, `TrajectoryOptimizationPlanner`, `ReinforcementLearningPlanner`, `TabularLearningPlanner`, `MonteCarloEvaluator` (facade) | `ProblemEvaluator` and its verbs, `MPPIPlanner`, `LQGPlanner`, `RRTPlanner` / `RRTStarPlanner` | — |
| `optimization` | `MathematicalProgram`, `Optimizer` | `ParametricProgram` (S76) | — |
| `identification` | — (the verb is `tune` with a data cost) | `fitting` helpers (C4) | — |
| `interfaces` | — | `Sys2Gym`, `SB3Controller` (to experimental at S49) | `c_export`, `symbolic`, `mjx` |

About 115 prelude names instead of 162 (core-consolidation.md §6.4); the five nouns plus
`Field`, `Shape` and the sampled loop are the only classes a student meets outside the
catalog and the laws.

---

## 2. Session 1: the shape, the convergence, the step model

Each decision: the code as it stands, the options, the recommended answer as code.

### D1 The shape of the framework (K7, ask 1)

**Today:** the constitution names one toolbox and the two surfaces; no document says which
approaches are pillars, so a "keep or move" verdict has no bar.

**Options:** six pillars (classical and modern, nonlinear, trajectory and MPC, RL,
identification and imitation, estimation); RRT as a seventh; imitation folded into RL;
deployment (C export) as a pillar.

**Ruled 2026-10-22 (yes, amended):** the six pillars are *tooling* pillars standing on two
foundations: **modelling** is the primary foundation (the `System` and its algebra, the
catalog plants, compile and the rollout), **optimization** the secondary one (the
`MathematicalProgram` and the `Optimizer`, the solver boundary every pillar lowers to). RRT
stays a side pillar kept small; deployment stays experimental until a vehicle runs it. The
sentence for CONSTITUTION §1 (the maintainer's to place):

> One model and one differentiable path from it to a cost, solved by one optimizer; every
> approach that controls a vehicle or a robot is a tool on that path: classical and modern
> control, nonlinear control, trajectory optimization and MPC, reinforcement learning,
> identification and imitation, estimation.

### D2 The catalog clean-up (K7, ask 2)

**Today:** 50 catalog plants on the root prelude beside 110 other names, 24 used by nothing;
three import layers (prelude, `minilink.catalog`, the defining module); the Mermaid exporters,
a fourth renderer, `ProblemEnv` / `to_gymnasium` and the `identification/` stub with no caller.

**Ruled 2026-10-22:** a plant is a leaf, so the choice is managed at the import surface and
by support, not by architecture.

- **Two import layers.** The root prelude carries the course set (about 15 plants: what the
  course notebooks and the tutorial import; K1 counts it). `minilink.catalog` is the one full
  index of every *supported* plant, flat. The defining module stays valid as Python does, but
  is not a documented layer; no domain sub-modules.
- **Supported means tested.** Every plant in `minilink.catalog` runs the both-backends
  contract test and has a docs entry; that is the price of entry and of staying.
- **Unsupported plants move to `minilink/experimental/catalog/`**, untested, off the docs
  site, out of the wheel, free to be abandoned later. The maintainer rules the list item by
  item from K1's table; the candidates are core-consolidation.md §6.1's first row. The rule for
  the line: a plant stays supported when it is a vehicle, a robot, or a benchmark a course or
  a tutorial chapter uses, or the maintainer names it as one to keep.
- **Twins merge, dead helpers go**: `SimpleIntegrator`, `TwoIndependentPendulums` and
  parameter-only drone variants are candidates (K1 lists them); the Mermaid exporters,
  `camera.py`, the pygame renderer, `ProblemEnv` / `to_gymnasium` and the `identification/`
  stub are deleted.
- **S64's hygiene rides along**, one plant family per commit.

```python
from minilink import Pendulum                              # the course set
from minilink.catalog import DynamicBicycleCar3D           # every supported plant, one flat index
from minilink.experimental.catalog import Plane3D          # unsupported: no test, no promise
```

Steps: the K8 row of TODO §4. Done when the root prelude lists at most 15 plants,
`minilink.catalog` lists only tested plants, no two supported plants differ by parameters
alone, and the README names the catalog by domain, not by class.

### D3 `experimental/engines/` (K7, ask 3)

**Ruled 2026-10-22 (c, amended):** kept in `minilink/experimental/` as a demo of a contact-engine
leaf, clearly not core. With it, the experimental tier's contract: `minilink/experimental/`
(importable prototypes) and a top-level `experimental/` folder (prototype scripts, moved out of
`examples/`, the student tree) are repo-only, untested by default (their tests leave the CI gate
for the nightly sweep or go), off the docs site, never cited by a course, and the staging area
for what may leave into its own project at v1.0 (with K5). The one exception: `c_export`'s
flagship smoke test stays in the JAX regression job, since the showcase runs it. Steps: TODO K9.

### D4 The exit rule (K7, ask 5)

**Ruled 2026-10-22: no exit rule.** The maintainer prunes by hand at each release from a
K1-style table; the three seeds lose their clock and are plain rows (tabular RL's demo, the
Lyapunov field in A1, PurePursuit's lesson in C2). The proposal, for the record:

~~**Recommended:** the sentence of core-consolidation.md §8 into ROADMAP §2 in session 1, so
K2's removals in the December tag follow a rule that exists:

> At each minor release's close-out, a teaching-surface or provisional name that no notebook,
> course checklist or tutorial chapter uses for two consecutive releases moves to
> `experimental/` or to the example that motivated it, unless a roadmap row names it as a
> pillar's seed; a seed that has not grown for two releases moves too.

The three seeds with a clock: tabular RL (owes its demo before v0.9), the Lyapunov field (A1),
`PurePursuit` (C2's lesson).~~

### D5 The prelude trim's form (K7, ask 6)

**Ruled 2026-10-22: (a) silent.** One ledger line per release in ROADMAP §2 (as every
breaking change so far); shims start with V4 at v1.0. The names stay importable from their
band facade, so a deep import never breaks.

### D6 The step model's contract and name (S75, asks 1 and 6)

**Today:** eight discretizations of the same plant (sampled-path.md §1);
`discretize(system, dt=None, *, integrator="rk4", params=None) -> StepSystem` is hand-written
over `source.f`, and the evaluator keeps a frozen and a `params` twin of every integrator.

**Options:** a new noun (`StepModel`); the existing `StepSystem` returned by `discretize`; a
method on `System` (`plant.sampled(dt)`).

**Ruled 2026-10-22: yes.** The existing verb and noun. `discretize` is already the GRO501 word for the
digital-implementation step, and a `StepSystem` already composes, rolls out and compiles.

```python
plant_k = discretize(plant, dt, *, method="rk4", substeps=1, hold="zoh")   # -> StepSystem
x1 = plant_k.step(x0, u0, k=0)                     # the bound form students use
x1 = plant_k.step(x0, u0, k=0, params={"m": 1.2})  # the same body; params always an argument
plant_k.compile(backend="jax").step_trace_p         # the one scannable step every sampled tool reads
```

`integrator=` becomes `method=` (the word `Simulator` and the MPPI plan use); the keyword
stays accepted for one release with the ledger line, no alias beyond that.

### D7 The four knobs, their defaults, and who exposes `substeps` (S75, ask 2)

**Ruled 2026-10-22: yes.** `method="rk4"`, `substeps=1`, `hold="zoh"` (today's numbers everywhere);
`"exact"` for linear models (the `discretize` follow-up); `hold="foh"` only where shooting
needs it. `substeps` is exposed on every sampled consumer that takes `dt`:

```python
ReinforcementLearningPlanner(problem, dt=0.05, substeps=4)
Sys2Gym(problem, dt=0.05, substeps=4)
MPPIPlanner(problem, dt=0.05, substeps=2)
HybridSimulator(loop, dt_inner=…)   # -> substeps=, the same word
```

### D8 DP's integrator (S75, ask 3)

**Today:** the grid steps `x + f(x, u, t) dt` inline (Euler), twice.

**Ruled 2026-10-22: (a).** The step model with `method="euler"` by default (byte-identical, every
value-iteration lesson's numbers hold); `DynamicProgrammingPlanner(problem, dt=, method="rk4")`
as an option from day one; whether the default flips is V3's question, decided with the course
notebooks pinned on a release.

### D9 One evaluator family; the ODE solver is the judge (S75 ask 4, and D17 revisited)

**Today:** `compare()` replays each law with the adaptive `Simulator`; `MonteCarloEvaluator`
runs a held static law in a hand scan on three backends; the differentiable-problem plan made
the compiled fixed-step run the evaluator and the Monte Carlo class a facade on it.

**Ruled 2026-10-22:** one verb, one record, one judge.

```python
report = evaluate(problem, controller)                              # one run from x_start, the true ODE solver
report = evaluate(noisy_problem, controller, n_trials=200, key=0)   # Monte Carlo over the draws, same solver
report.J, report.mean, report.worst, report.failure_rate, report.trajectories
compare(LQR=report_lqr, PPO=report_ppo).plot()                      # the side-by-side figure: a plot of Evaluations
```

- `evaluate` runs the closed loop as a diagram with the adaptive ODE solver, deterministic or
  Monte Carlo alike, and scores with `evaluate_trajectory`; what is reported is always what
  the plant really does. On NumPy the trials are a loop; on JAX they batch when the solver is
  fixed-step.
- The fixed-step compiled run (`problem.compile(controller)`, the `ProblemEvaluator`) is the
  **engine under `tune`** and the researcher's `jax.grad`, never the reported number: `tune`
  differentiates through it, then reports the tuned controller through `evaluate`.
- `MonteCarloEvaluator` the class folds into `evaluate(..., n_trials=)`; `Planner.evaluate`
  and `solve(evaluate=True)` call the same verb. Its numbers change once (the held static law
  becomes the real loop), in v0.4 with the draws.
- **Continuous first.** In v0.3 `evaluate` and `tune` take continuous controllers; a sampled
  controller (anything built by `%`) is refused with a message. Its face lands in v0.4 with the
  sampled loop (step 7b): the law held per period, the plant between samples still integrated
  by the judge. No sampled replay option before then.
- Infeasibility price: `PlanningProblem.infeasible_cost`, one owner for DP's out-of-bound
  cost, the RL failure price and the evaluator's exit price; its default derived as
  `cost.g_max(X, U) · tf` (or `/ ρ` on a discounted infinite horizon); no "infinity" on the
  cost.

### D10 `params` always an argument; freezing is a binding (S75, ask 6) — **open**

**Today:** every integrator exists frozen and with `params` (36 methods); a frozen evaluator
silently ignores parameters changed after `compile()`.

**Measured 2026-10-22** (`benchmarks/params_binding.py`, the table in sampled-path.md §3.5):
inside a scan or a `vmap` the two forms are equal; called step by step from Python the dict
costs about 3 µs per call (10.1 vs 13.6 µs on the pendulum), all of it dispatch, and the
bound form `jit(partial(step_p, params=sys.params))` equals the frozen call within noise.
Only the `params` form can take a gradient through the rollout (2.2× the value's cost).

**Options:** (A) keep the frozen and parametric bodies as hand-written twins: nothing moves;
two bodies to keep in step per integrator and per knob; the silent-ignore bug stays. (B) one
`_p` body per method, the frozen name a binding under its current stable name: half the grid,
one place for a knob, freezing an explicit act; the first call after `sys.params` changes
recompiles. Recommended: B. The family slowdown (S53) is untouched either way.

**The maintainer's call, pending** (session 1 stopped here on 2026-10-22).

### D10b The compiled problem is the traceable twin of `evaluate` (ruled 2026-10-22)

`evaluate(problem, controller, …)` and `problem.compile(controller).evaluate(…)` share every
piece but the run: one `closed_loop_of(problem, controller)` wiring the loop from the problem,
one `problem.realize(key, n)` for the draws, one `evaluate_trajectory` for the score, one
`feasibility_margin(problem, traj)` for the margin, one `Evaluation` record; the run is the
`Simulator` on one side and the compiled fixed-step scan on the other. The compiled object's
verbs are thin calls into those functions with `run="compiled"`, plus the program bridge,
which has no ODE twin. A pinned test: `Simulator(solver="rk4_fixedsteps")` and the compiled
scan on the same grid agree to round-off. **The loop may be passed built**: a diagram carrying
the role `plant` (that block the problem's system) is used as is; anything else is a controller
and wired; roles are never inferred from the wiring; the cost is scored on the `plant` role
either way; `tune(problem, loop, wrt=["controller/observer.L"])` names leaves by the diagram's
params paths.

---

## 3. Session 2: the sampled loop, the compiled problem, the names

### D11 The sampled loop: a `System` or a tool (S31)

**Today:** `law % dt` builds a `Computer`; `computer @ plant` builds a `HybridDiagram`, which
is not a `System`, with hand-copied facades, `last_result` as a cache, and three copies of
the sampled rollout (`RolloutEnvironment`, `HybridSimulator`, the MPC computer export).

**Options:** (a) a `System` with state `[plant; computer]` and periodic update events, so it
nests in diagrams and `linearize` has to be taught to refuse it; (b) a tool built by `%` and
`@`, not a `System`, on one sampled rollout kernel in two faces.

**Recommended:** (b), named `SampledLoop` (the plan's working name is `HybridLoop`; "sampled"
is the word the rest of the plan uses, and the course notes' vocabulary decides in the
naming session). Two closed algebras stay closed; the loop that joins their clocks is a tool
with `compute_trajectory`, `compile`, `plot_trajectory`, `animate` and nothing else.

```python
loop = StateFeedbackController(K) % 0.05 @ plant     # SampledLoop
loop.plant, loop.computer                            # the two roles
traj = loop.compute_trajectory(tf=10)                # NumPy face: a Trajectory with the held u as a signal
run = loop.compile(backend="jax")                    # JAX face: scan over ticks; vmap over trials; grad through a gain
run.rollout_p(x0, t0, n_ticks, params)               # the one kernel RolloutEnvironment, HybridSimulator and MPC export call
```

### D12 The sampled seam's time: ticks or seconds (S31)

**Today:** `StepSystem.step(x, u, k, params)` takes the integer tick; the computer schedules
in ticks; the plant runs in seconds.

**Recommended:** ticks stay on the step contract, the textbook's `x_{k+1} = Φ(x_k, u_k, k)`,
and the loop owns the conversion `t_k = t0 + k·dt` where it calls the continuous plant. A
`StepSystem` that needs the clock reads `self.dt` (one owner, as `discretize` already rules).
Byte-identical; no signature changes.

### D13 The run-state caches (S31; RULES 2.4)

**Today:** `StepSystem.rollout` and `HybridDiagram.last_result` cache a run beside the
allowed `self.traj`.

**Recommended:** they go. A tool returns its result; `loop.compute_trajectory()` returns the
`Trajectory` and stores it as `loop.traj` like every `System`, nothing else. The facades that
read `last_result` read `traj`.

### D14 Controller memory and the draws on the tick (S31; future-policies.md §5 points 2, 4)

**Today:** MPPI's shifted warm start and a diffusion policy's chunk have no channel; the RL
collectors thread their own keys.

**Recommended:** no new channel. A sampled controller with memory *is* a `StepSystem` with
state, and the kernel already carries the computer's `x` across ticks (`MPCStatefulController`
packs `z` there today). Its draws come from its own `params["seed"]` and the tick, the noise
block's counter-based pattern, so `vmap` over seeds and `realize(key)` work unchanged:

```python
class MPPIController(StepSystem):          # memory = the shifted input sequence, in x
    def step(self, x, u, k=0, params=None):
        eps = standard_normal(params["seed"], k, self.n_samples)   # the noise block's draw
        ...
```

### D15 The compiled problem dispatches on the policy's kind (S31; V1 step 7b)

**Recommended:** `problem.compile(policy)` reads the policy: a continuous law runs the loop as
a diagram; a `Computer` (anything built by `%`) runs the sampled kernel's JAX face. In v0.3 the
second branch refuses with a message naming step 7b; in v0.4 it lands. One `compile`,
`evaluate`, `tune` for both kinds.

### D16 The cost of a trajectory: one formula, and who owns what (V1 step 2)

**Today:** five scoring rules with four quadratures; `h` charged by some and not others;
`discount_rate` an attribute of the cost.

**Ruled 2026-10-22:** one function, `evaluate_trajectory(problem, traj)` (the name; today's
`score_trajectory` and `CostFunction.evaluate_trajectory` fold into it), the one body on `xp`:
trapezoid on the trajectory's own grid, the problem's discount `e^{−ρt}`, cut at the first exit
from `X` and charged `infeasible_cost`, `h(x_N, t_N)` added exactly when `tf` is finite. Same
numbers as today on NumPy; traceable on JAX. An infinite horizon with a nonzero `h` warns once
at problem construction. **Ownership:** the cost owns the integrand, `g(x, u, t)` and
`h(x_f, t_f)`; the problem owns the functional's shape: `tf`, `X`, `U`, `Xf`,
`infeasible_cost` and **`discount_rate`** (moved from `CostFunction`; RL's γ and DP read it
on the problem; one ledger line).

```python
problem = PlanningProblem(plant, cost=QuadraticCost(Q, R, S), X=box, tf=10.0, discount_rate=0.1)
J = evaluate_trajectory(problem, traj)          # the one formula, every tool
```

### D17 The noun: `problem.compile(policy) -> ProblemEvaluator`, an engine (ask 1)

**Ruled with D9 (2026-10-22):** the compiled problem exists as the engine under `tune` and
the researcher's direct `jax.grad`; it is not the user-facing evaluator, which is the verb
`evaluate` on the ODE solver. Its noun and verbs are named in step 0 (D19) and sit on the
planning facade as provisional names.

### D18 `Evaluation`: a NumPy report plus a traced `value` verb (ask 2)

**Recommended:** lower late, convert once. `evaluate()` returns the NumPy record (`J`,
`failed`, `x0`, `keys`, `trajectories` when recorded; `mean`, `worst`, `failure_rate`);
`value()` is the one traced scalar, the problem's criterion over the draws. No pytree record,
no second spelling of the traced number.

### D19 The names of step 0 (ask 5)

**Recommended package**, one line each, with the alternative in parentheses:

| Thing | Recommended | Alternatives | Why |
| --- | --- | --- | --- |
| the facade | `problem.compile(policy, dt=)` | `closed_loop`, `evaluator` | the `System.compile` pair |
| the noun | `ProblemEvaluator` | `CompiledProblem`, `PlanningProblemEvaluator` | the `compile → Evaluator` pair |
| the run | `trajectory()` | `simulate()`, `rollout()` | returns a `Trajectory`; "simulate" is `Simulator`'s |
| the scalar | `cost()` | `J()`, `score()` | the cost is the noun the problem holds |
| the criterion over the draws | `value()` | `expected_cost()` | the criterion may be a CVaR; "value" is the DP word |
| the feasibility number | `margin()` | `feasibility()` | the sets' own verb |
| the system-evaluator method | `rollout_p()` | `simulate_p()` | the RL word for a run of the compiled step |
| the selection keyword | `wrt=` | `decision=`, `free=` | `jacobian(of, wrt)` already says it |
| the facade on the program | `tune()` | none | the course notes' word for gain tuning |
| the parametric program (S76) | `ParametricProgram` | keep `ParametricMathematicalProgram` | RULES 2.6, short |
| the plant's key in every parameter tree | `"plant"` | `"system"` | the role name every tool reads (P10); `ProblemParameters.system` renames on V4's list |

### D20 No scoring modes — ruled with D16

`evaluate_trajectory` is the problem's contract and nothing else; a tool that wants the uncut
integral scores `replace(problem, X=None)`; the transcriptions take the problem's discount and
keep their own sum (byte-identical: no baseline declares a rate).

### D21 The evaluator's numbers change once (ask 7) — confirm

The held-input static law becomes the continuous loop's own RK4 (O(dt²) apart on a smooth
law), re-pinned with a release note in V1's step 7 (v0.4), the window the calendar rule names.

### D22 The verbs dropped (ask 8) — confirm

No `final_state`, no `success_rate`, no `mean_cost`: the trajectory's last column, `1 −
failure_rate` and `report.mean` already say them. `margin` is the exact minimum; the
per-sample margins are the bridge's constraint rows.

### D23 The optimizer's method keys (ask 4; S74)

**Today:** `method="scipy_slsqp" | "scipy_trust_constr" | "ipopt"`.

**Recommended:** new presets named by their algorithm (`l-bfgs`, `bfgs`, `gd`, `adam`,
`nelder-mead`, `differential-evolution`), the three `scipy_*` keys kept as aliases until V3
rules on their rename; `method="auto"` picks by the program's class; the default stays
`scipy_slsqp` until V3.

```python
Optimizer(program).solve()                      # scipy_slsqp, as today
Optimizer(program, method="auto").solve()        # l-bfgs for a bounded smooth program, slsqp if constrained, adam if stochastic
Optimizer(program, method="adam", steps=400).solve()
```

### D24 The lane (ask 9)

**Recommended:** `tune` on the teaching surface by the demo rule (its demo is the autotuning
one); `compile`, `ProblemEvaluator`, `program`, `pack` / `unpack`, `margin`, `value`
provisional on the planning facade until GMC714 has run them.

### D25 S76's two remaining asks, for v0.4 (parametric-program.md §2)

**Recommended:** a hand-written program may declare its own `p` (`ParametricProgram(n_z, J, params=…)`
with `J(z, p)`), since it costs nothing and lets tutorial 08 solve one NLP for several
parameter values; the MPC block takes optional live-parameter ports, unwired ports at their
nominal:

```python
mpc = ModelPredictiveController(planner, dt_mpc=0.1, live=["plant.mu"])   # a port "mu" appears
loop = (estimator + mpc).autowire() @ plant                              # the estimate drives the plan
```

---

## 4. The v0.4 names the v0.3 code will already print

### D26 MPPI's name (mppi.md ask 1)

**Recommended:** `MPPIPlanner` and `MPPIRecord`, by the `LQRPlanner` precedent (an algorithm's
own acronym, as the course notes say it); `PathIntegralPlanner` stays the docstring's long
form.

### D27 The MPC block's planner contract (mppi.md ask 2; T6)

**Recommended:** the three `Planner` verbs (`decision_dimension()`, `prepare_online()`,
`warm_start_guess(z_prev, y, *, dt_mpc, k)`) and `z` on the tick's record; no `hasattr`
probing. `ModelPredictiveController(planner, dt_mpc=)` then wraps trajopt and MPPI alike.

### D28 The stochastic default of MPPI (mppi.md ask 3)

**Recommended:** `n_plant_samples=1` on a stochastic problem with the bias stated in the
docstring; the user raises it knowingly, as with `n_trials`.

### D29 The standard stochastic policy as a seeded block (future-policies.md §5 point 4)

**Recommended:** `GaussianPolicy(mean, log_std, *, dt, seed=None)` in `control/neural.py`, a
plain `System` whose `h` is `u = μ_θ(x) + σ_θ(x) · ε(seed, ⌊t/Δ⌋)`; `seed=None` is the
deterministic face (`NeuralPolicyController` becomes that face); `log_prob(x, u)` and
`entropy(x)` as methods beside `h`, the squashed variant a subclass. The RL heads are rebuilt
on it in C5 (v0.4), the RL numbers changing once.

### D30 Where `RolloutEnvironment` lives (mppi.md ask 5)

**Recommended:** answered by D11: it becomes a caller of the sampled kernel and stays a thin
RL-facing adapter in `reinforcement_learning/`; no move.

---

## 5. What this sketch changes in the other plans, if taken

| Plan | Change |
| --- | --- |
| sampled-path.md | the loop's name (`SampledLoop`), ticks kept, caches gone, memory as the computer's `x`: §8's "still to rule" list closes |
| differentiable-problem.md | §4's working names become the names; ask 13 (S31 first) is met by session order |
| parametric-program.md | `ParametricProgram`, the key `plant`, live ports as `live=` |
| mppi.md | `MPPIPlanner`, `n_plant_samples=1`, the three verbs |
| future-policies.md | points 2 and 4 answered without a new channel; `GaussianPolicy` named |
| core-consolidation.md | asks 1–6 answered; the plant rule's sentence |
| CONSTITUTION §1 | one sentence, the spine and the six pillars (maintainer-owned) |

## 6. How to read this in the sessions

Session 1 rules D1–D10 in one pass; each is a yes, a no with the alternative named, or a
different list. Session 2 rules D11–D24 the same way; D19 is the one table to argue over.
D25–D30 can wait for v0.4 but are written so that nothing v0.3 prints has to be renamed.
