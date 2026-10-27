# Native optimizers: the solve as a rollout

Status: written 2026-10-25 from the maintainer's vision (§1), with the state of the code (§2)
and the agent's read (§3–§4). Lane: research. The rows: the proof of concept (NS-1, NS-2's
fixed-`K` form, NS-8, NS-9) is [TODO.md](TODO.md) §1.8 ([core-architecture.md](core-architecture.md)
§8); the rest is TODO §4.

**Preliminary choices, 2026-10-27** (open to readjustment with the final architecture):
- the two-layer split of §3: a pure kernel run by `Optimizer`, a step-system view for the
  lesson; §4.1–§4.2's categories and algorithms added only with the first solver that needs them
  ([R132](../reviews/2026-10-27-design-questions-rulings.md));
- the fixed-count solve: `Optimizer(program, method=, iterations=K)`, exactly `K` steps in one
  `lax.scan`; `iterations=None` runs to a tolerance, `maxiter` its cap; a cap and a fixed count
  stay two things ([R149](../reviews/2026-10-27-core-vision-decisions.md));
- the fixed-iteration MPC is a planner under the contract of [planner-and-mpc.md](planner-and-mpc.md)
  (R148): this doc holds its solver, that doc its tick;
- programs, parameters and randomness: §7's convention (R131), on one program noun,
  `MathematicalProgram(..., params=None)` (R130).

**Where each idea lives** (one fact, one home):

| Idea | Home |
| --- | --- |
| Native solvers as pure laws, program categories, the solve as a rollout, the three ways to differentiate a solve | this doc, §3–§4 |
| The fixed-iteration MPC's solver (goal 1) | this doc, §4.6, NS-8 |
| The planner's compiled face, the MPC block and its tick | [planner-and-mpc.md](planner-and-mpc.md) (R148) |
| MPPI, the planner | [mppi.md](mppi.md) (S72); its tick as a pure planner, this doc §4.6b |
| Convergence analysed as stability (goal 2), with the discrete step analysis it needs | this doc, §4.7, NS-5 (R122) |
| The program's parameters and one backend table | [parametric-program.md](parametric-program.md) (S76, OPW-1–OPW-5) |
| The sampled loop as one scan (`SampledDiagramSystem`, R146; the two run verbs, R147) | [sampled-path.md](sampled-path.md), [core-architecture.md](core-architecture.md) §5 |
| The differentiable evaluation of a sampled loop (`problem.compile` of a sampled controller) | [differentiable-problem.md](differentiable-problem.md), step 7b (R105) |
| A gradient-descent tutorial on the extended `Optimizer` | [TODO.md](TODO.md) §6 (R91) |
| Parametric and stochastic programs in the solvers: seeds, the convention | this doc, §7; randomness in simulation [randomness.md](randomness.md) |

Contracts in code today: [DESIGN.md](../dev/DESIGN.md) §6 (Planning / NLP).

---

## 1. The vision (the maintainer, 2026-10-25)

- **An upgraded `MathematicalProgram` with categories**, and an upgraded `Optimizer` whose
  solvers fall in two families: **wrapped backends** (Ipopt, `scipy.optimize.minimize`, other
  libraries) and **native algorithms** implemented in minilink.
- **Native algorithms are pure array code, like a `System`**: the equations are stateless
  functions of arrays, with optional `params` the way `f(x, u, t; p)` has them.
- **The solve is a rollout.** The sequence of iterates of gradient descent is the analog of
  the rollout of a step system, `z_{k+1} = step(z_k, k; p)`.
- **The ultimate goal: some solvers fully JAX-compatible**, so that the whole solve can be
  jitted, differentiated and vmapped: a solve inside a loss, a batch of starts or of problem
  parameters in one call.

### 1.1 The two goals (the maintainer, 2026-10-25)

1. **First: an MPC whose solver runs a fixed number of iterations**, so the controller's tick
   is fully compilable and even differentiable; the closed loop with the MPC then simulates
   end to end in JAX, very fast.
2. **Long term: analyse the convergence of an optimization process with the control tools**:
   the algorithm as a feedback loop, its convergence as stability.

## 2. What exists today

| Piece | Where | What it does | Traceable end to end? |
| --- | --- | --- | --- |
| `MathematicalProgram` | `optimization/mathematical_program.py` | Frozen record of `J(z)`, `h(z) = 0`, `g(z) ≥ 0`, the box, optional derivatives, the native backend | the callables are, if the user writes them so |
| Program classes | `optimizer.py`, `program_class` | Three, read from the fields: `unconstrained`, `bounded`, `constrained`; `method="auto"` picks the class default (S74, landed 2026-10-24) | n/a |
| `Optimizer` | `optimizer.py` | A stateful orchestrator: owns the program, its compiled evaluator, `z0`, the method; `solve()` returns an `OptimizationResult` | no |
| Wrapped backends | `optimizers/scipy_minimize.py`, `optimizers/ipopt.py` | L-BFGS-B, SLSQP, trust-constr; Ipopt through optional `cyipopt` | no (host callbacks) |
| Native backend | `optimizers/first_order.py` | `gd` and `adam`, projected on the box: a Python `for` loop on NumPy arrays; Adam's update is `jax.jit`-ed, so `adam` needs JAX even on a NumPy program | no |
| Program evaluators | `optimization/evaluators/` | NumPy and JAX; the JAX one jits `J`, `∇J`, `∇²J`, the Jacobians, then **returns NumPy arrays** to the solver | the derivatives are; the solve is not |
| Parameters | `planning/trajectory_optimization/parametric_program.py` | `ParametricMathematicalProgram`: only `x0` varies between solves (it folds into `MathematicalProgram(..., params=)`, R130, S76) | partly (the JAX MPC path) |
| `Adam` | `optimizers/first_order.py` | Optax-style `init` / `update` on a pytree; also the RL learners' optimizer (`sac.py`, `algorithms/base.py`) | **yes**: the RL planner runs its gradient steps in one `lax.scan` (`planner.py`), PPO too |
| The step model | `core/discrete/step_system.py`, `core/compile/evaluators/discrete.py`, `stepping.py` | `step(x, u, k, params)`; `rollout` is a Python loop on NumPy, one `lax.scan` on JAX; no `rollout_batch` for step systems yet | yes (open point: `DiscreteEvaluator.rollout` returns host NumPy, core-architecture.md §2) |
| Other solvers in disguise | `planning/dynamic_programming/dp_jax.py` (value iteration in a `lax.while_loop`), `learning/.../ppo.py` (scan of minibatch steps), MPPI (planned, S72) | each owns its own loop | yes |

Nothing in the library differentiates *through* a solve today: no `custom_vjp`, no implicit
differentiation, no traceable optimizer loop outside RL. The problem evaluator's program
export (`pe.program(wrt=…)`) does give a JAX-traceable `J(z)`, so the inputs a traceable solver
needs already exist.

## 3. The agent's read: the optimizer as a step system

The idea is the control-theory view of optimization algorithms (Lessard, Recht and Packard,
2016). It fits the CONSTITUTION's first article better than any other option.

### 3.1 The textbook picture: an update law in feedback with a gradient oracle

Gradient descent is a discrete integrator with gain `−η`, in feedback with a static
nonlinearity, the gradient `∇J`:

```text
        ┌──────────────┐  z_k   ┌───────────────┐
  ┌────▶│ update law   │───────▶│ oracle  ∇J(z) │──┐
  │     │ z⁺ = z − η u │        │ (static)      │  │
  │     └──────────────┘        └───────────────┘  │
  └──────────────────── u_k = ∇J(z_k) ◀────────────┘
```

- The **update law** is a `StepSystem` with input `u` (a gradient) and state `z`, plus the
  method's memory: the momentum of heavy ball, the moments `m, v` of Adam. Gradient descent and
  heavy ball are *linear* step systems; Adam is a nonlinear one.
- The **oracle** is a static system `y = ∇J(u; p)` built from the program, with the problem's
  parameters `p` as its `params`.
- **The solve is the closed loop's rollout**: a `StepDiagramSystem` of the two, compiled and
  rolled out like any other. Its parameter tree follows the diagram contract, every block
  named: `{"law": {"eta": …}, "oracle": {…problem parameters…}}`.

This already runs today, wired by hand (checked 2026-10-25): a one-state `StepSystem` law
`z⁺ = z − η u` with `η = 0.1`, a `Gain(2)` as the oracle of `J = z²`, connected in a
`StepDiagramSystem` and rolled out, gives `z = 1, 0.8, 0.64, 0.512`: the contraction
`1 − 2η`. Two gaps today: the operators refuse step systems (explicit `connect` only; R126 gives
the step algebra `>>`, `+` and `@`), and a step system has `jacobian` but no `linearize` or poles.

What this buys, beyond reuse:

- **Convergence is stability.** Linearize the loop at the optimum `z*`:
  `z_{k+1} − z* = (I − ηH)(z_k − z*)`. The step size is stable when every eigenvalue of
  `I − ηH` is inside the unit circle, and the rate is the spectral radius: a lesson GRO860 and
  the control courses would get almost for free, once step systems get `linearize` and discrete
  poles.
- **The iterates are a `Trajectory`** (`compute_rollout` on the view, R147, once `StepRollout`
  folds into `Trajectory`): the plot of `J` against `k`, the path of `z` on the contour.
- **`vmap` over the start** is a multi-start; over `η`, a step-size sweep; over `p`, a family
  of problems.

### 3.2 Where the analogy stops

- **Stopping.** A rollout runs a fixed count; a solver stops on a tolerance. A fixed count is a
  `lax.scan`, traceable in every mode; a tolerance is a `lax.while_loop`, which `jit` and
  `vmap` take but reverse-mode `grad` does not. Both stay, as one structural keyword (R149):
  `iterations=K` the process, `iterations=None` the answer.
- **Differentiating the answer is not differentiating the process.** Two different
  gradients, both useful (§4.3).
- **Constraints.** A projection on the box fits the step model as is. Equality and inequality
  rows need an outer loop (augmented Lagrangian: a rollout of unconstrained rollouts) or
  linear algebra inside the step (Newton on the KKT system, interior point). The production
  constrained solver stays Ipopt.
- **The `System` contract costs something.** `x` is a flat vector; Adam's state is a pytree
  `(z, m, v, k)`. It flattens to `x = [z; m; v]` with `k` the tick, fine for a vector `z` and
  awkward for a neural network's weight tree. The RL learners keep their pytree path.

### 3.3 The two layers (R132)

1. **The kernel**: each native method is a pure law, `init(z0, params) -> state` and
   `step(state, params) -> state`, Optax-shaped (as `Adam` already is), on the array module of
   its inputs (one equation path, CONSTITUTION §2 invariant 5). `Optimizer` runs it: a Python
   loop on NumPy, `lax.scan` (`iterations=K`) or `lax.while_loop` (`iterations=None`) on JAX.
2. **The step-system view**: `as_step_system(solver, program)` (R132), reached as
   `optimizer.as_step_system()` (R149), the law as a `StepSystem` update block and the oracle as
   a static block, so a lesson can wire the loop, roll it out and linearize it. A *view* of the
   kernel, not a second implementation.

Every solver a `StepSystem` by construction would force the pytree state into a flat `x`; none
of them one would lose the best lesson in the plan.

## 4. The design, in outline

### 4.1 Program categories: few, textbook, declared by the program's fields

Today's three classes stay. The additions, each with its consumer (R132: a category lands with
the first solver that needs it):

| Category | The form | What it unlocks | First consumer |
| --- | --- | --- | --- |
| unconstrained / bounded / constrained | as today | the method table of S74 | every program |
| **least squares** | `J(z) = ½‖r(z)‖²`, the residual `r` given | Gauss–Newton, Levenberg–Marquardt (native); `scipy.optimize.least_squares` | identification (C4), inverse kinematics (`Manipulator` calls `scipy.optimize.least_squares` by hand today) |
| **stochastic** | the seed a leaf of `params` (R131; no `J(z, key)`); comes with the optimizer seed strategies | SGD and Adam on minibatches | the Monte Carlo evaluator's objective, the ML lessons |
| **parametric** | `J(z, params)`, `h(z, params)`, `g(z, params)` on the one noun (R130) | one compile, many solves; `vmap` over `params`; the sensitivity `dz*/dp` | the MPC (S76) |

A category is read from the program's fields (a `residual` field makes it least squares, as `h`
makes it constrained today), not from a modelling language: CONSTITUTION §1.4 ("not an
optimization modeling language or solver framework") holds. Quadratic and linear programs
(a linear MPC with OSQP) are a later row, if a consumer appears.

### 4.2 The native set: the textbook algorithms, not a solver library

- First order: gradient descent, heavy ball and Nesterov, Adam; projected on the box.
- Second order: Newton (with a line search), Gauss–Newton, Levenberg–Marquardt.
- Constraints: augmented Lagrangian over any of the above.

Each is a few lines of textbook math (RULES §5), on NumPy and JAX; the core needs gradient
descent and Adam (R132). Out of scope: competing with Ipopt on large constrained NLPs, or with
Optax and Optimistix on breadth. `Adam` already says any Optax-style object can replace it.

### 4.3 Three ways to differentiate a solve

| Mode | How | Cost | When |
| --- | --- | --- | --- |
| **Unrolled** | `grad` through the `lax.scan` of `iterations=K` | memory grows with `K` | gradients with respect to the method's own parameters (`η`, a learned step size), a fixed small budget, the fixed-iteration MPC (§4.6) |
| **Implicit** | at the answer, `∇_z J(z*, p) = 0`, so `dz*/dp = −H⁻¹ ∂²J/∂z∂p` (the KKT system once there are constraints); a `custom_vjp` around the solve | one linear solve, memory constant | the sensitivity of the answer to the problem: a solve inside a loss, bilevel tuning (later, R131) |
| **None** | `lax.while_loop` to a tolerance (`iterations=None`) | cheapest | a batch of solves under `jit` and `vmap`, without gradients |

The implicit mode needs only the converged `z*` and the program's derivatives, so it can wrap
any solver whose answer is accurate, a wrapped one included (through `jax.pure_callback`); that
is a later step.

### 4.4 Wrapped backends

Unchanged in contract: an adapter takes a compiled evaluator and `z0` and returns an
`OptimizationResult`. They stay host code; their improvements are OPW-1–OPW-5 (one backend
table for the offline and the compiled paths, [parametric-program.md](parametric-program.md) §6).

### 4.5 The surface, sketched

`iterations=`, `backend=` (R134), `params=` and `z0=` on the solve (R130, R131) and
`as_step_system` are decided; the other names are placeholders.

```python
# Today, unchanged: a hand-written program, solved to a tolerance (maxiter its cap)
result = Optimizer(program, method="adam").solve()

# A native method on JAX, exactly K steps: the whole solve one lax.scan (R149)
opt = Optimizer(program, method="adam", backend="jax", iterations=200)
z_star = opt.solve(params=p, z0=z0).z                       # jit-able, vmap-able, differentiable
dz_dp = jax.jacobian(lambda p: opt.solve(params=p, z0=z0).z)(p)

# The process: the iterates as a Trajectory over k (R132, R149)
traj = opt.as_step_system().compute_rollout(200)
traj.plot("J")

# The teaching face: the loop as a step diagram
loop = gradient_loop(GradientStep(eta=0.1), GradientOracle(program))  # a StepDiagramSystem
loop.linearize(z_star).poles()                       # the convergence rate (needs step linearize)
```

How a result object traces under `jit` (a record of arrays, or `z` alone on the traced path) is
settled with NS-2.

### 4.6 The first target: a fixed-iteration MPC, compiled end to end

**The shape (R148).** The fixed-iteration MPC is no special block: it is a trajectory-optimization
planner whose method is native and whose `iterations=K` is forwarded to its `Optimizer` (R149),
wrapped by `ModelPredictiveController` like any planner. Its compiled face's `pv.solve(params,
z0)` is `K` iterations of the method from the shifted plan; the tick (shift, solve, policy), the
delay, tick 0 and the bindings are [planner-and-mpc.md](planner-and-mpc.md) §6. In the solver's
terms one tick is

```text
z_k = S^K( pv.shift(z_{k−1}, t_k − t_last) ; P_k )      P_k: the planner's tree, x_start and t_start bound
```

with `S` one iteration of the method (projected fast gradient, Adam). `K` is structural, fixed at
construction (R131). Every piece is pure array code, so the MPC's `step` is one jitted function
and the closed loop `(mpc % 0.05) @ plant` is one `lax.scan` (R146, R147).

**Today, for contrast:** `MPCStatefulController` (`planning/trajectory_optimization/mpc/controller.py`)
already keeps the packed plan as its state (R104), but its `step` is a host solve (SciPy SLSQP)
through a tick latch with side effects; R148 drops the stateful and stateless pair and the latch
(planner-and-mpc.md §9).

**Why a fixed `K` is the right contract here**, not a tolerance:
- The tick's cost is known in advance: a real-time budget, the same on every tick.
- `lax.scan` over `K` is traceable in every mode; a tolerance would need `while_loop`, which
  reverse-mode `grad` refuses.
- The gradient of the closed-loop cost is then **exact for the controller that runs**,
  truncation included: tuning sees the truncated solver honestly, not an idealised optimum.
- With the warm start, the iterations accumulate across ticks: the loop keeps converging while
  the plant moves (the real-time-iteration idea of Diehl, and the fast-gradient MPC of Richter,
  Jones and Morari).

**The transcription.** Single shooting first (`shooting.py`, already a `lax.scan` rollout): the
dynamics leave the program, the input box is the program's box, so the program is box-bounded
and projected first-order methods solve it with no constraint rows. State constraints come
second, by penalty or augmented Lagrangian over the same `K` budget (NS-6). Direct collocation,
whose dynamics are equality rows, stays on Ipopt and SciPy.

**What it unlocks.**
- The closed loop with an MPC simulates as fast as any compiled diagram, and `vmap` runs a batch
  of starts or of plant parameters in one call (the Monte Carlo evaluator's trials).
- `jax.grad` of the closed-loop cost with respect to the MPC's cost weights
  (`controller.cost.Q`), its terminal weight, or the step size (`controller.method.lr`): MPC
  tuning by gradient, through the same `problem.compile(controller)` path the PID tuning uses.
- A first differentiable MPC without implicit differentiation; the implicit mode (§4.3) stays
  for a solve run to convergence.

**What it needs first.** The parametric program (S76: the solve binds `x_start` and the plant
without a recompile); NS-1 and NS-2's fixed-`K` form; the planner's compiled face
(planner-and-mpc.md §10); for the closed loop, the sampled loop as one scan and its
differentiable evaluation (step 7b of [differentiable-problem.md](differentiable-problem.md),
R105). Today `HybridSimulator` steps the ticks in a Python loop, and the problem evaluator
refuses a `StepSystem` controller (`require_continuous` in `planning/evaluator.py`). The sampled
loop's first consumers are a digital PID and a held neural policy, so it is built and checked
before NS-8. A host-solver MPC (Ipopt, SLSQP) runs the same block on the NumPy face.

### 4.6b MPPI: the same tick, already pure

The path-integral planner (MPPI, S72, [mppi.md](mppi.md)) fits the same template with a
derivative-free iteration: a planner under planner-and-mpc.md's contract, its `z` the nominal
`U` and `t_start`, its `seed` a leaf of `planner.params` that the MPC draws from the block's seed
and the tick (R104). The prototype (`experimental/mppi/pendulum_mppi.py`, `racecar_mppi.py`)
already writes its tick as one jitted pure function `mppi_tick(U, x0, key)`: sample perturbed
sequences, roll them out in one `vmap`, score, weigh by `exp(−S/λ)`, average. Its receding
horizon is a hand `for` loop with the key split by hand; the MPC block replaces it. Several MPPI
iterations per tick (`n_iterations`, structural like `K`) fit the same shape.

Differentiable too: with the seed fixed, `U_new` is a softmax-weighted average of rollouts, a
smooth function of the cost weights, `λ`, `Σ` and the plant's parameters (the noise enters by
reparameterisation, `V = U + ε`). The clip to the input box has a zero gradient where it binds,
as the projection of a projected gradient method does; the gradient is noisy across seeds, so
tuning averages it over the Monte Carlo evaluator's trials.

**Open: which fixed-iteration tick comes first.** MPPI needs no native gradient solver, only
the block and the sampled scan, so it could reach the closed loop in one scan soonest. The
maintainer's direction of 2026-10-26 places the gradient MPC's proof of concept in v0.3 and MPPI
in v0.4 (TODO §1.8, §4); no ruling has taken the question beyond that placement.

### 4.7 The long-term target: convergence as stability

§3.1's loop, analysed with the control tools: the step size's stability region from the poles
of `I − ηH`, heavy ball and Nesterov as second-order linear filters in feedback with `∇J`, and
later the robust view (the gradient a sector-bounded nonlinearity, the rate certified for a
whole class of `J`: the integral quadratic constraints of Lessard, Recht and Packard). It needs
`linearize` and discrete poles on step systems (today `jacobian` only), and the step operators
to wire the law to the oracle (R126).

**Ruled 2026-10-25 (R122): the slice comes back, later, when solver stability needs it.** The
2026-10-17 ruling ([the record](../reviews/2026-10-17-z-tier-dropped.md)) dropped the z-domain
analysis family (exact `c2d`, the z-plane pole map, discrete Bode and root locus) for GRO501,
and that stands. Goal 2 needs only the linearization of a step system at a fixed point
(`A = ∂step/∂x`, from the `jacobian` that exists) and its eigenvalues against the unit circle,
built with NS-5; the rest of the z tier stays dropped.

## 5. Steps (local ids NS-1–NS-9)

The order and each row's "done when" are TODO §1.8 (the proof of concept) and §4 (the rest).

- **NS-1 The kernel**: `init` / `step` laws for `gd` and `adam`, on the array module of their
  input; the `Optimizer` loop on NumPy unchanged in behaviour (the AGENTS refactor recipe, a
  byte-identical baseline), and `adam` runs on NumPy without JAX.
- **NS-2 The traced solve**: `iterations=K` as a `lax.scan` and `iterations=None` as a
  `lax.while_loop` for the native methods on a JAX program (R149); a batch of starts solves in
  one `vmap` call and matches the loop.
- **NS-3 Implicit differentiation** of the unconstrained and box-bounded answer
  (`custom_vjp`); `dz*/dp` matches finite differences on a parametric quadratic and on a small
  least-squares identification.
- **NS-4 The least-squares category** with Gauss–Newton and Levenberg–Marquardt;
  `Manipulator`'s inverse kinematics can use it in place of its hand SciPy call.
- **NS-5 The teaching face**: `as_step_system`, the loop rolled out and linearized; one
  notebook-sized demo (the step-size stability lesson). Brings `linearize` of a step system at a
  fixed point and its eigenvalues against the unit circle (R122), and nothing more of the z tier.
- **NS-6 Constraints natively**: augmented Lagrangian over NS-1's kernel, then its implicit
  derivative through the KKT system.
- **NS-7 The stochastic category**: the optimizer seed strategies (R131), SGD and Adam advancing
  the seed, the problem evaluator's Monte Carlo objective their first consumer.
- **NS-8 The fixed-iteration planner** (§4.6): single-shooting trajectory optimization with a
  native method and `iterations=K`, its compiled face (planner-and-mpc.md §3) pure; one
  `pv.solve` jits, the MPC on it matches the host SLSQP plan within a tolerance on the pendulum
  and the cart-pole after enough ticks, and `jax.grad` of one tick's input with respect to a
  cost weight matches finite differences.
- **NS-9 The closed loop in one scan** (§4.6): `problem.compile(mpc % Δ)` evaluates the MPC loop
  as one scan, `vmap`s over starts, and its gradient with respect to the MPC's cost weights tunes
  a demo by `Optimizer`.

## 6. The asks (all answered)

1. The two-layer split, a pure kernel run by `Optimizer` with the `StepSystem` loop as a view:
   **yes (R132).**
2. Which categories and native algorithms (§4.1, §4.2): **added with the first solver that
   needs them, minimalist (R132).**
3. The rung: **answered 2026-10-26** by the maintainer's direction
   ([the record](../reviews/2026-10-26-core-consolidation.md) §2): a proof of concept in v0.3,
   the rest in v0.4.
4. The fixed-count keyword: **`iterations=K` on `Optimizer` (R149)**; rejected, an `iterate`
   verb and `solve(iterations=K)`.

Still open: which fixed-iteration tick comes first (§4.6b). Ruled: the discrete step analysis
comes back with NS-5 (R122, §4.7).

## 7. Parametric and stochastic programs

**Preliminary choice, 2026-10-27 (R131, [record](../reviews/2026-10-27-design-questions-rulings.md)),
on one program noun (R130):**
- inside, a program's functions always see a complete tree (R121); at the user boundary
  (`solve(params=...)`) a partial tree is merged once onto the defaults; no `bind` on programs
  (R130);
- the seed is an integer leaf found by the `seed` key, as on blocks, policies (R129) and sources
  (R128); integer leaves are allowed in `params` (R148, amending R118) and never enter `wrt`;
  the raw gradient over a whole tree is `jax.grad(f, allow_int=True)` (R152);
- sizes (the horizon `N`, the iterations `K`, batch sizes) are constructor arguments; only
  values live in `params`;
- the warm start is an argument of the solve (`z0=`), not of the program;
- sensitivities through a solve come later, with implicit differentiation (§4.3);
- `J(z, key)` is dropped: the seed lives in `params`.

The core keeps this convention and the sample-average export (`pe.program(wrt, key=,
n_trials=)`: a deterministic program whose `J` is the mean over draws realized once). How a
solver uses the seed (keep it fixed, `vmap` over it, advance it per iteration) is an `Optimizer`
strategy, taken up with the native solvers beyond the proof of concept (NS-7). The design notes
below are for that work.

### 7.1 The maintainer's leaning (2026-10-25)

> Seeds are params, as on a system: **every program is deterministic given its params**. A
> stochastic solver may decide internally to change the seed, to work directly with the
> sampled, noisy function.

It mirrors the system side: a stochastic simulation is the deterministic simulation of one
realized tree, the seeds inside it (R144); a stochastic program is the deterministic program at
one `params` with its seed inside.

### 7.2 Three kinds of stochastic program

| Kind | The objective | Draws | Solvers |
| --- | --- | --- | --- |
| **Sample average** (common random numbers) | `J(z) = 1/N Σᵢ J(z, ωᵢ)` | fixed at build, from one seed | any, Ipopt and SciPy included: it is deterministic (today's `pe.program`) |
| **Stochastic approximation** | `E[J(z, ω)]`, estimated per iteration | fresh at every iteration `k` | native first order only (SGD, Adam); L-BFGS and SLSQP break on a noisy `J` |
| **Risk and chance** | `CVaR`, `P(x ∈ X) ≥ 1 − α` | either | later; the problem's `criterion=` is the hook (R143) |

With the seed a leaf of `params`:
- a sample-average program is a parametric program whose seed is bound: a new seed is a new
  instance, solved without recompiling; `vmap` over seeds is a Monte Carlo over solves;
- stochastic approximation is the solver advancing the seed itself, `seed_k = child(seed, k)`,
  the same rule as R104's sampled controller, with §3.3's kernel unchanged.

### 7.3 What makes a solver that changes the seed sound

1. **The solve stays deterministic given `params`.** The solver derives iteration `k`'s seed
   from the user's, `seed_k = child(seed, k)`; the same `params` and `z0` give the same iterates,
   on replay and under `jit`; the seed stream is part of the result's record.
2. **One seed per iteration, shared inside it.** Every evaluation within iteration `k` (the
   gradient, a line search, a second sample for a variance estimate) uses `seed_k`: otherwise a
   line search compares two different functions.
3. **The solver touches only the seed leaf.** The rest of `params` is the problem; the seed
   takes no gradient.
4. **The answer is scored at a stated seed**, the user's or a held-out set, never the last
   iteration's noisy sample.

| Solver | Seed handling | Program it sees |
| --- | --- | --- |
| Wrapped (Ipopt, SciPy) and deterministic natives (L-BFGS, Newton) | keeps the seed fixed | the sample average at that seed |
| Stochastic natives (SGD, Adam) | advances it, `child(seed, k)` | a fresh sample per iteration |
| A Monte Carlo over solves | `vmap` over seeds, each solve keeping its own | a family of sample averages |

So the stochastic category is a property of how a *solver* uses the program, not a different
kind of program. Sensitivities `dz*/dp` of a sample-average program are the derivative for a
fixed sample; for stochastic approximation they are not defined the same way.
