# The planner and the MPC: one interface for every planner

The design home of the planner interface and of `ModelPredictiveController`, as converged in the
review of 2026-10-27. The choices are preliminary (open to readjustment with the final
architecture): [R148](../reviews/2026-10-27-core-vision-decisions.md) and its amendment, on top
of R141–R147 of the same record and R123, R104, R130–R135. How it was reached, the use cases and
the rejected candidates: [the study](../reviews/2026-10-27-mpc-planner-interface-proposal.md).
The rows are TODO §1.7 and §1.8 ([core-architecture.md](core-architecture.md) §7, §8); MPPI
([mppi.md](mppi.md)) is a planner under this contract.

## 1. The pattern

A planner is a **description**, and `planner.compile()` gives its **pure face**, as the library
already does for systems and problems:

| | Description: math, parameters, convenience verbs | Compiled face: pure arrays, traceable |
| --- | --- | --- |
| System | `sys.f`, `sys.params`, `sys.compute_trajectory()` | `ev = sys.compile()`: `ev.step`, `ev.rollout` |
| Problem | `problem.cost`, the sets, `problem.params` | `problem.compile(dt=)`: `transition` (R141) |
| Planner | `planner.params`, `planner.solve()` | `pv = planner.compile()`: `pv.solve`, `pv.shift`, `pv.policy`, … |

Two rules hold throughout:
- **A planner does not know it runs in a loop** (R135). What shifting a solution means is the
  planner's knowledge; deciding to shift it each tick is the MPC's.
- **The MPC never reads inside a solution.** It knows only its size.

## 2. The planner, as a description

```python
planner = TrajectoryOptimizationPlanner(problem, n_knots=40, method="adam", iterations=200)
planner.params        # the tree the plan depends on
sol = planner.solve() # a PlanningSolution, as today
pv  = planner.compile(backend="jax")
```

- **The constructor holds structure only:** the method, the knot count, the grid, `iterations`.
  They set sizes, so they are fixed at build (R131).
- **`planner.params`:** the problem's tree (`x_start`, `t_start`, `plant`, `cost`, the sets,
  `discount_rate`), plus `"method"` (the algorithm's own numbers: a learning rate, MPPI's
  temperature), plus `seed` where the planner draws. Every number a plan depends on is a leaf, so
  it can be rebound, differentiated or swept.
- **`planner.solve(params=None, *, guess=None, evaluate=False, n_trials=50) -> PlanningSolution`:**
  the convenience verb. It compiles (cached), packs the guess, solves, rebuilds the solution and
  evaluates it on request. `planner.solve()` with no argument behaves as today.
- **`planner.compile(backend="auto") -> pv`:** the pure face below.

## 3. The compiled planner `pv`

| Member | Signature | Contract |
| --- | --- | --- |
| `n_z` | `int` | the size of the planner's **solution vector `z`**: everything it needs to continue, in its own layout, opaque to everyone else (a system's `n`); fixed for any parameter values |
| `params` | tree | the compiled defaults |
| `initial` | `initial(params=None) -> z` | the default initial guess in `z` form, before any solve (§4); what `solve` starts from when `z0` is `None` |
| `solve` | `solve(params=None, z0=None) -> (z, status)` | the solve as a function: from the guess `z0` as given, no time logic, no mutation of the planner; it records in `z` the origin it solved from (`params["t_start"]`); `status` is a small numeric record (success, iterations, residual) |
| `shift` | `shift(z, t) -> z` | the same plan rebased to the absolute time `t` (R155): the planner's data moved by `t − origin` on its own knots, its recorded origin set to `t`; the identity for a solution that does not depend on time (the base default) |
| `policy` | `policy(z, x, t, params=None) -> u` | the command a solution gives at state `x` and time `t`: the traceable bridge from a solution to an input. `policy(z, x_start, t_start)` is the first action; `policy(z, x_k, t_k)` reads the plan later. Open-loop plans ignore `x`; feedback plans read it |
| `prediction` | `prediction(z, t, params=None) -> (x, u)` | optional: the planned state and input at `t`, for planners that predict |
| `solution` | `solution(z, params=None) -> PlanningSolution` | host-side: the policy as a block, the nominal trajectory, the cost-to-go as a `Field` (value iteration's stays a callable until `GridField`, v0.4, R153), the record. `solution.policy.h(x, ·, t)` equals `pv.policy(z, x, t)` |
| `pack` | `pack(guess, params=None) -> z` | the other way: from a `Trajectory` or a `PlanningSolution` (a user's guess, a saved solution, another planner's result) |
| `program` | `MathematicalProgram` or `None` | the parametric program, for planners that have one; outside the MPC's contract |

**Purity.** `solve`, `shift`, `policy`, `prediction` and `initial` are pure functions of their
arguments wherever the planner's algorithm is: native fixed-iteration trajectory optimization,
value iteration on JAX, MPPI on its `seed`, LQR, the RL learners in `lax.scan`. Ipopt and RRT
run on the host. `PlanningSolution` stays a rich host object rebuilt from `z` (no DTO,
CONSTITUTION §4.6); the planner keeps no `last_solution` or cache on the pure path.

```python
z, st = pv.solve(p | {"x_start": x1}, z0=pv.shift(pv.pack(sol), 0.05))   # re-solve, warm
u0    = pv.policy(z, x1, p["t_start"])                                   # the first action
zs, _ = jax.vmap(lambda x: pv.solve(p | {"x_start": x}))(starts)         # a family, traced
pv.solution(zs[3]).plot_trajectory()                                     # look at one
```

## 4. What each planner puts in `z`

| Planner | `z` | `initial(params)` | `shift(z, t)` | `policy(z, x, t)` |
| --- | --- | --- | --- | --- |
| Trajectory optimization (collocation, shooting) | the decision vector, the origin `t_start`, Ipopt's multipliers where used | today's default guess (`default_initial_trajectory`: a cubic path for a mechanical plant, else linear, from `x_start` to the goal; zero or nominal inputs), packed | re-read the knots `dt` later, hold the tail | the input of the plan at `t` |
| MPPI | the nominal input sequence `U`, `t_start` | zeros or the nominal input | shift `U`, repeat the last input | the input of `U` at `t` |
| Value iteration | the `J` and `π` tables | `J` from the terminal cost (or zeros) | identity | the lookup `π(x)` |
| LQR | `K`, `x̄`, `ū` | the gain at the operating point | identity | `ū − K(x − x̄)` |
| RL | the weights | the initialized network, from `seed` | identity | the network at `x` |
| RRT | the path resampled to `N` knots, `t_start` | the straight line from `x_start` to the goal | drop the part already passed | the path's input at `t` |
| A discrete-time planner on a `StepPlanningProblem` (later) | as trajectory optimization, on ticks | as trajectory optimization | by `dt` ticks | the input at tick `t` |

## 5. The problem side

- **`t_start`, a new leaf.** The horizon is `[t_start, t_start + tf]`; `t_start = 0` reproduces
  today. **The discount and the horizon count from `t_start`**, while costs, sets and references
  read absolute time: `J = ∫_{t_s}^{t_s+T} e^{−ρ(t − t_s)} g(x, u, t) dt + e^{−ρT} h`. (On absolute
  time a cost would be scaled by `e^{−ρ t_s}`, `4.5e−5` at `ρ = 0.1`, `t = 100 s`, defeating the
  solver's tolerances.)
- **On a native step problem `t_start` is the tick offset**, an integer: integer leaves are
  allowed, never in `wrt` (amends R118). A discretized problem (`discretize(problem, dt)`) is a
  problem on a clock and keeps `(dt, t_start)` in seconds, its transition forming
  `t_k = t_start + k·dt` (R158).
- **A planner pins the start from the `x_start` leaf**; `X0` is derived from it (shooting and
  collocation read `X0` first today, `shooting.py`, `direct_collocation.py`).

## 6. The MPC

```python
mpc  = ModelPredictiveController(planner)                     # the delayed form (R123), the default
mpc  = ModelPredictiveController(planner, delay=False)        # the pure law
mpc  = ModelPredictiveController(planner, inputs={"x_start": "x_hat",
                                                  "plant.m": "m_hat", "plant.mu": "mu_hat"})
mpc  = ModelPredictiveController(planner, lookahead=0.05)     # delay compensation, with a predictor on x_start
loop = (mpc % 0.05) @ plant                                   # % sets the native law's period
```

The MPC uses five members of `pv`: `n_z`, `initial`, `shift`, `solve` and `policy`. The delayed
form is a **native `SampledSystem`** (R158, [sampled-path.md](sampled-path.md) §8.1): its equations
are `update(z, y, t)` in seconds and its output ports' functions (`u` first), its tick faces serve the step algebra, and
`%` sets its period.

### 6.1 Inputs are bindings; time is the argument

- `inputs` maps leaves of the planner's tree to the block's input ports; the default is
  `{"x_start": "y"}`. At each tick the block forms `P_k`: the planner's tree (the loop's
  `controller.*` leaves) with each bound leaf overwritten from its port, `t_start` bound to the
  time `t` its `update` receives, and `seed` drawn from the block's own seed and `⌊t/Δ⌋` (R104,
  R158).
- A bound leaf stays in the tree as the value used when its port is not wired, so the tree's
  shape never depends on the wiring.
- **Time is the argument** (R158): the loop's plan hands the block `t_k = t_start + k·Δ`, the
  one channel every equation path has; no clock port, no clock source. The block's equations
  never read `Δ` (only its base class does, to serve the tick face); the planner's knots and the
  sample period stay independent.
- `x_start` must be a point in the planner's problem; a problem whose start is a set or a law is
  refused, naming why.

### 6.2 The delayed form (the default, R123)

One native `SampledSystem` whose state is `z` alone, `n = pv.n_z` (R155, R158): the plan carries
its own origin, so the MPC keeps no time of its own and reads nothing inside `z`.

```python
class ModelPredictiveController(SampledSystem):          # no source: a native sampled law
    def __init__(self, planner, *, inputs=None, delay=True, lookahead=0.0):
        ...
        self.add_output_port("u", dim=m, function=self.compute_u, dependencies=("y",))
        self.add_output_port("z", dim=pv.n_z, function=self.compute_state)

    def compute_u(self, z, y, t, params=None):           # u_k = π(z, x̂_k, t_k), the stored plan
        return pv.policy(z, y, t)

    def update(self, z, y, t, params=None):              # one solve per tick, from the rebased plan
        P = bind(params, self.inputs, y) | {"t_start": t + self.lookahead}
        z_new, status = pv.solve(P, z0=pv.shift(z, t))
        return where(status.success, z_new, z)
```

- **The plan is one period late; applying it is instantaneous.** `u_k` reads the plan solved at
  `t_{k−1}` at `Δ` into it (an open-loop plan's second knot when the knots equal `Δ`); a feedback
  plan also reads the fresh `x̂_k`.
- **A failed solve keeps `z`** with its old origin; the next tick rebases it `2Δ` on and reads the
  old plan `2Δ` in, with no special case.
- **One solve per tick**, no cache; no algebraic loop (the discretized plant has no feedthrough,
  `z` is state). Traced, `step` is a `lax.scan` body and the fallback a `where`. A fixed-count
  native solve (R149) always reports success, so the fallback is live only on a host solver
  (Ipopt, SLSQP): the traced tick has no branch.
- **Tick 0:** `x0 = (pv.initial(planner.params), t_start)`, the default guess, no solve at
  construction. The first period's command is the guess's first input (zero for a default
  trajectory-optimization guess). `x0` is a leaf of the loop's tree (R124), so a solved first plan
  is one assignment.

### 6.3 The pure form (`delay=False`)

One static block, no memory: `u_k = pv.policy(pv.solve(P_k)[0], x̂_k, t_k)`, the first action of a
solve from the default guess. It is the ideal law, a reference for studies; a cold start each tick
is slow on Ipopt and weak for a fixed-iteration solver. It needs **one signal pass per tick** in
the step evaluators (today they compute the signals in `step` and again in `outputs`,
`core/compile/evaluators/discrete.py`), else it solves twice per tick, with the same result.

### 6.4 Ports, the tree, dual rate, compensation

- **Ports out:** `u`; `z` (the plan, for dual rate and logging); `x_ref` and `u_ref` from
  `pv.prediction` where the planner predicts (an MPC feeding a reference to an inner loop).
- **The tree:** `pe.params["controller"]` is `planner.params`: `controller.cost.Q` to tune,
  `controller.plant.m` beside `plant.m` for a model-mismatch study, `controller.method.lr`.
- **Dual rate:** `(mpc % 0.05) >> (PlanFollower(planner) % 0.01)`, the follower applying
  `pv.policy` to the `z` it receives. The plan solved from `y_j` reaches the follower at the next
  slow tick: one slow period of delay, as on a real computer, not two.
- **Delay compensation:** `lookahead=Δ` binds `t_start = t + Δ`, and `x_start` is bound to a
  predictor block (`inputs={"x_start": "x_pred"}`); the keyword alone shifts only the time.

## 7. The cases it serves

An Ipopt MPC on the host; the native fixed-iteration MPC with its closed loop in one scan, tuned
by gradient; an adaptive MPC fed estimated state, mass and friction; a moving reference bound to a
cost leaf; MPPI with a seed per tick; RRT, value iteration, LQR or an RL planner as the replanner;
a future discrete-time planner; dual rate; a model-mismatch study; Monte Carlo over the loop;
realtime and hardware in the loop on the host runner; a failed solve; terminal ingredients and
constraints moving per tick (bound leaves of cost and set parameters, once sets read `params`,
core §1); per-tick plans logged; a reference for an inner PID. The study's §1 lists them (U1–U25)
with how each lands.

## 8. Out of the contract

- **A shrinking horizon:** `N` is structural; a later option makes `tf` a leaf of planners with a
  fixed knot count (it conflicts with `N = tf/dt` of the compiled step face, R141).
- **RRT traced:** never, its algorithm is not pure.
- **Scenario and robust MPC:** a tree holding laws is the planner's business, which expands them.
- **Binding part of a start** (one block of a diagram plant): syntax later.
- **Event-triggered replanning:** a select in `step`, a later option.

## 9. What goes from today's code

From `planning/trajectory_optimization/mpc/`: `dt_mpc`, `t0`, `warm_start`, `MPCTickLatch`, the
pair `MPCStatelessController` / `MPCStatefulController`, `export_to_computer`, `dt_broadcast`,
`Command`, `replan_divisor`, and the `hasattr` probing of the planner. From the planners:
`last_solution` and the caches on the solve path; `initial_guess=` / `warm_start=` become `guess=`.

## 10. What the core must provide

1. Every planner implements the compiled face of §3: trajectory optimization first (the MPC's
   consumer), then LQR and value iteration, the others as they are wrapped; MPPI with its v0.4
   row (S72).
2. Every policy keeps its tables in `params` (core §1: `TrajectorySource` and `LookupPolicy` keep
   them as attributes today).
3. `t_start` on problems, the discount counted from it; integer leaves.
4. `SampledSystem` as the sampled-data kind, `update` and the port functions its seconds faces, the loop's plan
   handing them `t_k` (R158).
5. One signal pass per tick in the step evaluators (for the pure form).

## 11. Names for the sweep

`pv.policy` (or `action`, `control`); the letter `z` (a planner's `z` is its decision, extended);
`PlanFollower`; `inputs=`; `delay=`; `lookahead=`; `update` (sampled-path.md §8.7).
